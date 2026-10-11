// SPDX-License-Identifier: MIT
// Fly a vehicle's MISSION: read a vehicle description that has a `mission` block (docs/design/GNC.md, docs/design/RECOVERY.md), design the mission on its nominal flight (the gains of the attitude loop, the
// allocation to the surfaces, the guidance's starting solutions, the drag of the way down), then fly it for real: every body (the stack, then the stages that are let go) on three flight computers with
// their sensor models, a GNSS receiver, the navigation filter, the guidance and the voted commands, in the vehicle simulator. Reports what happened to each body and what the mission was meant to do.
//   tfc_mission VEHICLE.json [--ideal] [--seed N] [--gnss-sigma M] [--pad FRAMES] [--max-time S] [--design-only] [--expect-catch] [--expect-orbit] [--expect-landing]
//                            [--csv FILE] [--every S] [--pose FILE] [--pose-body N] [--pose-hz N]
// --ideal: the attitude of every body is forced to the one its mission asks for (the guidance and the plant without the sensors and the loop). --expect-*: exit 3 unless the stated outcome happened
// (a booster caught by the tower, a ship in an orbit with its perigee above 100 km, a stage on its legs or its parachute at a speed the legs and the parachute can take).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "mission_design.hpp"
#include "spec_io.hpp"
#include "viewer_state.hpp"

namespace {

struct Options {
  const char* file = nullptr;
  bool ideal = false;
  bool design_only = false;
  bool expect_catch = false;
  bool expect_orbit = false;
  bool expect_landing = false;
  uint32_t seed = 0U;
  double gnss_sigma = 2.0;
  uint32_t pad_frames = 1500U;
  double max_time = 0.0;
  double every = 10.0;
  const char* csv = nullptr;
  const char* pose = nullptr;
  std::size_t pose_body = 0U;
  uint32_t pose_hz = 20U;
};

void usage() {
  std::fprintf(stderr,
               "usage: tfc_mission VEHICLE.json [--ideal] [--seed N] [--gnss-sigma M] [--pad FRAMES] [--max-time S] [--design-only] [--expect-catch] [--expect-orbit] [--expect-landing]\n"
               "                                [--csv FILE] [--every S] [--pose FILE] [--pose-body N] [--pose-hz N]\n");
}

bool parse(int argc, char** argv, Options& o) {
  if (argc < 2) {
    return false;
  }
  o.file = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--ideal") {
      o.ideal = true;
    } else if (a == "--design-only") {
      o.design_only = true;
    } else if (a == "--expect-catch") {
      o.expect_catch = true;
    } else if (a == "--expect-orbit") {
      o.expect_orbit = true;
    } else if (a == "--expect-landing") {
      o.expect_landing = true;
    } else if (a == "--seed" && more) {
      o.seed = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--gnss-sigma" && more) {
      o.gnss_sigma = std::strtod(argv[++i], nullptr);
    } else if (a == "--pad" && more) {
      o.pad_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--max-time" && more) {
      o.max_time = std::strtod(argv[++i], nullptr);
    } else if (a == "--every" && more) {
      o.every = std::strtod(argv[++i], nullptr);
    } else if (a == "--csv" && more) {
      o.csv = argv[++i];
    } else if (a == "--pose" && more) {
      o.pose = argv[++i];
    } else if (a == "--pose-body" && more) {
      o.pose_body = static_cast<std::size_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--pose-hz" && more) {
      o.pose_hz = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else {
      std::fprintf(stderr, "unknown option or missing value: %s\n", a.c_str());
      return false;
    }
  }
  return o.pose_hz >= 1U && o.pose_hz <= 100U && o.every > 0.0;
}

// The orbit a body is on: its perigee and apogee altitudes (km) from the state, and whether it is bound at all.
struct Orbit {
  bool bound = false;
  double perigee_km = 0.0;
  double apogee_km = 0.0;
  double speed = 0.0;
};

Orbit orbit_of(const sim::State& s, double mu, double radius) {
  Orbit o;
  const double r = sim::norm(s.r);
  const double v2 = sim::dot(s.v, s.v);
  const double energy = (0.5 * v2) - (mu / r);
  o.speed = std::sqrt(v2);
  if (!(energy < 0.0)) {
    return o;
  }
  const double a = -mu / (2.0 * energy);
  const sim::V3 h = sim::cross(s.r, s.v);
  const double p = sim::dot(h, h) / mu;
  const double e = std::sqrt(std::fmax(0.0, 1.0 - (p / a)));
  o.bound = true;
  o.perigee_km = ((a * (1.0 - e)) - radius) / 1000.0;
  o.apogee_km = ((a * (1.0 + e)) - radius) / 1000.0;
  return o;
}

const char* ending(const sim::Vehicle6& v) {
  if (v.caught()) {
    return "caught by the tower";
  }
  if (v.landed()) {
    return "landed";
  }
  if (v.crashed()) {
    return "crashed";
  }
  return "still flying";
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse(argc, argv, opt)) {
    usage();
    return 2;
  }
  sim::VehicleFile vf;
  std::vector<std::string> errors;
  if (!sim::load_vehicle_file(opt.file, vf, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 1;
  }
  if (!vf.mission.present) {
    std::fprintf(stderr, "%s has no mission block (docs/design/GNC.md section 9 says what one holds)\n", opt.file);
    return 1;
  }
  sim::BuiltMission bm;
  if (!sim::build_mission(vf, bm, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 1;
  }
  const sim::MissionSpec& ms = vf.mission;
  std::printf("%s: %s\n", vf.name.c_str(), vf.description.c_str());
  const sim::DesignReport rep = sim::design_mission(vf, bm);
  if (!rep.ok) {
    std::fprintf(stderr, "the design run produced nothing: the mission does not fly on its own nominal flight\n");
    return 1;
  }
  std::printf("design: the nominal flight took %.0f s, %u probes of the plant", rep.end_time_s, static_cast<unsigned>(rep.probes));
  for (std::size_t s = 0; s < sim::kMaxStages; ++s) {
    if (bm.has_stage[s] && bm.stage[s].drag.beta > 0.0) {
      std::printf("; stage %zu's way down measured at a ballistic coefficient of %.0f kg/m^2", s, bm.stage[s].drag.beta);
    }
  }
  std::printf("\n");
  if (opt.design_only) {
    return 0;
  }

  sim::WorldConfig wc;
  wc.params = vf.params;
  wc.scenario = vf.scenario;
  wc.scenario.tower.enabled = ms.arm_height_m > 0.0;
  wc.scenario.tower.height_m = ms.arm_height_m;
  wc.scenario.tower.offset_y_m = ms.site_offset_y_m;
  wc.scenario.tower.offset_z_m = ms.site_offset_z_m;
  wc.scenario.tower.capture_radius_m = ms.capture_radius_m;
  wc.main_tables = &bm.main;
  for (std::size_t s = 0; s < sim::kMaxStages; ++s) {
    wc.stage_tables[s] = bm.has_stage[s] ? &bm.stage[s] : nullptr;
  }
  wc.ideal = opt.ideal;
  wc.pad_frames = opt.ideal ? 0U : opt.pad_frames;
  wc.avionics.gnss.pos_sigma_m = opt.gnss_sigma;
  if (opt.seed != 0U) {
    wc.avionics.gnss.seed = opt.seed;
    wc.avionics.noise_seed = opt.seed * 7U;
  }
  sim::MissionWorld world(wc);

  std::FILE* csv = nullptr;
  if (opt.csv != nullptr) {
    csv = std::fopen(opt.csv, "w");
    if (csv == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", opt.csv);
      return 1;
    }
    std::fprintf(csv, "t_s,body,name,phase,altitude_m,speed_ms,mass_kg,downrange_m,crossrange_m\n");
  }
  std::FILE* pose = nullptr;
  if (opt.pose != nullptr) {
    pose = std::fopen(opt.pose, "w");
    if (pose == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", opt.pose);
      return 1;
    }
  }
  bool pose_spec_written = false;

  const double max_time = opt.max_time > 0.0 ? opt.max_time : ms.max_time_s;
  const uint32_t frames = static_cast<uint32_t>(max_time * 100.0) + wc.pad_frames;
  const uint32_t pose_every = std::max<uint32_t>(1U, 100U / opt.pose_hz);
  double next_report = 0.0;
  double tail_since = -1.0;
  std::vector<double> phase_start;
  std::vector<unsigned> last_phase;
  std::vector<std::string> names;
  for (uint32_t k = 0U; k < frames; ++k) {
    world.step();
    if (world.clamped()) {
      continue;
    }
    const double t = static_cast<double>(world.flight_frame()) * 0.01;
    // note the phases each body passes through
    for (std::size_t i = 0; i < world.bodies(); ++i) {
      const sim::MissionWorld::Body& b = world.body(i);
      if (i >= last_phase.size()) {
        last_phase.push_back(99U);
        names.push_back(b.name);
      }
      unsigned ph = 99U;
      if (b.av != nullptr) {
        ph = b.av->computer(0).mission().phase();
      } else if (b.ideal != nullptr) {
        ph = b.ideal->phase();
      }
      if (ph != last_phase[i] && ph != 99U) {
        const sim::Vehicle6& v = b.veh;
        std::printf("  t=%7.1f s  %-8s phase %u: altitude %7.2f km, speed %7.1f m/s, mass %9.0f kg\n", t, b.name.c_str(), ph, v.altitude() / 1000.0, v.speed(), v.mass());
        last_phase[i] = ph;
      }
    }
    if (t >= next_report) {
      next_report = t + opt.every;
      if (csv != nullptr) {
        for (std::size_t i = 0; i < world.bodies(); ++i) {
          const sim::MissionWorld::Body& b = world.body(i);
          const sim::State& s = b.veh.state();
          std::fprintf(csv, "%.1f,%zu,%s,%u,%.1f,%.2f,%.0f,%.1f,%.1f\n", t, i, b.name.c_str(), last_phase[i], b.veh.altitude(), b.veh.speed(), b.veh.mass(), s.r.y, s.r.z);
        }
      }
    }
    if (pose != nullptr && opt.pose_body < world.bodies() && k % pose_every == 0U) {
      const sim::MissionWorld::Body& b = world.body(opt.pose_body);
      if (!pose_spec_written) {
        sim::VehicleFile bf = vf;
        bf.params.spec = b.veh.spec();
        std::fprintf(pose, "%s\n", sim::viewer_spec_json(bf, b.veh).c_str());
        pose_spec_written = true;
      }
      std::fprintf(pose, "%s\n", sim::viewer_pose_json(b.veh, t, sim::PoseExtra{}).c_str());
    }
    // the mission is over when every body that was let go has ended and the stack has reached the last phase of its plan
    bool all_done = world.bodies() > 0U;
    for (std::size_t i = 0; i < world.bodies(); ++i) {
      const sim::MissionWorld::Body& b = world.body(i);
      const bool ended = b.veh.landed() || b.veh.caught() || b.veh.crashed();
      unsigned ph = 0U;
      unsigned n = 1U;
      if (b.av != nullptr) {
        ph = b.av->computer(0).mission().phase();
        n = b.tables != nullptr ? b.tables->n_phases : 1U;
      } else if (b.ideal != nullptr) {
        ph = b.ideal->phase();
        n = b.tables != nullptr ? b.tables->n_phases : 1U;
      }
      // a body in the last phase of its plan is done if that phase is one that has no end of its own to wait for (a coast in orbit); in a descent it is done when it has touched the ground
      bool settled = ph + 1U >= n;
      if (settled && b.tables != nullptr && n > 0U) {
        const uint8_t last_kind = b.tables->phase[n - 1U].kind;
        settled = last_kind != tfc::gnc::kind::kChute && last_kind != tfc::gnc::kind::kLanding && last_kind != tfc::gnc::kind::kGlide && last_kind != tfc::gnc::kind::kBoostback;
      }
      all_done = all_done && (ended || settled);
    }
    if (all_done) {
      if (tail_since < 0.0) {
        tail_since = t;
      }
      if (t - tail_since >= ms.tail_s) {
        break;
      }
    } else {
      tail_since = -1.0;
    }
  }
  if (csv != nullptr) {
    std::fclose(csv);
  }
  if (pose != nullptr) {
    std::fclose(pose);
  }

  // ---- what happened ----
  std::printf("\n");
  const double mu = bm.main.nav.gravity.mu;
  const double radius = bm.main.nav.gravity.radius;
  bool caught = false;
  bool orbit_ok = false;
  bool landing_ok = false;
  for (std::size_t i = 0; i < world.bodies(); ++i) {
    const sim::MissionWorld::Body& b = world.body(i);
    const sim::State& s = b.veh.state();
    const Orbit orb = orbit_of(s, mu, radius);
    std::printf("%s: %s", b.name.c_str(), ending(b.veh));
    if (b.veh.caught() || b.veh.landed() || b.veh.crashed()) {
      const sim::V3 site{0.0, 0.0, ms.site_offset_z_m};
      const sim::V3 p = s.r - sim::V3{b.veh.ground_radius(), 0.0, 0.0};
      std::printf(", %.1f m from the site, %.0f kg left", std::hypot(p.y - site.y, p.z - site.z), b.veh.mass());
    } else if (orb.bound) {
      std::printf(", in orbit: perigee %.1f km, apogee %.1f km, speed %.1f m/s", orb.perigee_km, orb.apogee_km, orb.speed);
    } else {
      std::printf(", altitude %.1f km, speed %.1f m/s", b.veh.altitude() / 1000.0, b.veh.speed());
    }
    std::printf("\n");
    caught = caught || b.veh.caught();
    orbit_ok = orbit_ok || (!b.veh.caught() && !b.veh.landed() && !b.veh.crashed() && orb.bound && orb.perigee_km > 100.0);
    landing_ok = landing_ok || b.veh.landed() || b.veh.caught();
  }
  int rc = 0;
  if ((opt.expect_catch && !caught) || (opt.expect_orbit && !orbit_ok) || (opt.expect_landing && !landing_ok)) {
    std::printf("EXPECTATION NOT MET%s%s%s\n", opt.expect_catch && !caught ? ": no booster was caught" : "", opt.expect_orbit && !orbit_ok ? ": no body is in an orbit above 100 km" : "",
                opt.expect_landing && !landing_ok ? ": nothing landed" : "");
    rc = 3;
  }
  return rc;
}
