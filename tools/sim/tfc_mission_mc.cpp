// SPDX-License-Identifier: MIT
// A Monte Carlo of a whole mission (docs/design/MONTE_CARLO.md section 7, docs/design/FUTURE_WORK.md 6a item 4): the probability that the stage that returns is caught and the ship is in its orbit, with
// every flight drawing every dispersion of the file at once, and the extremes of the loads each flight met.
//
//   tfc_mission_mc MISSION.json --config DISPERSIONS.json [--flights N] [--seed S] [--threads T] [--pad FRAMES] [--max-time S] [--detail] [--trace N] [--list]
//
// The mission is designed once, on the nominal vehicle (the gains, the guidance's starting solutions and the drag of the way down belong to the computers); each flight then flies the same tables on a vehicle
// whose parameters, atmosphere, wind and sensors are drawn: the generator, the distributions, the targets and the statistics are those of tfc_mc (sim/vehicle/montecarlo.hpp), so a flight's draws depend on the
// seed and the flight's index and on nothing else, whatever the number of threads (`gnss_sigma_m`, the receiver's noise, is a target of the mission's alone). The result for each flight: whether the booster was caught
// (and how far from the site), the ship's orbit, and the envelope (max q, max q sin(alpha), the accelerations) of each body. The report gives the fractions with the 95 % Wilson interval, the distribution of the
// miss, the loads' percentiles, and the draws that move with the miss and with the loads (Pearson: a ranking of suspects, not a cause).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "mission_design.hpp"
#include "montecarlo.hpp"
#include "spec_io.hpp"

namespace {

struct Options {
  const char* file = nullptr;
  const char* config = nullptr;
  uint32_t flights = 0U;
  uint64_t seed = 0U;
  unsigned threads = 1U;
  uint32_t pad_frames = 1500U;
  double max_time = 0.0;
  bool list = false;
  bool trace_landing = false;  // with --trace: the landing every quarter second too
  int trace = -1;          // the landing of this flight, every half second, on standard output
  bool detail = false;     // one line for each flight: its draws and what became of the returning stage
};

struct Outcome {
  bool finite = true;
  bool caught = false;
  bool landed = false;
  bool crashed = false;
  double miss_m = -1.0;          // the returning stage's distance from the site, if it came down
  bool ship_orbit = false;
  double perigee_km = 0.0;
  double apogee_km = 0.0;
  double max_q_pa = 0.0;         // the largest of any body's
  double max_q_sin_alpha_pa = 0.0;
  double booster_max_q_pa = 0.0;
  double end_sink_ms = 0.0;      // the returning stage at the end: how fast it was falling, how fast across, how far from upright
  double end_lateral_ms = 0.0;
  double end_tilt_deg = 0.0;
  double ignition_miss_m = -1.0; // the horizontal distance the landing burn had to cover when it was lit (the glide's footprint error)
  bool passed = false;           // the stage's pins came down through the arms' height: what they were then
  double pass_miss_m = 0.0;
  double pass_sink_ms = 0.0;
  double pass_lateral_ms = 0.0;
  double pass_tilt_deg = 0.0;
  double max_axial_g = 0.0;
  std::vector<double> draws;
};

void orbit_of(const sim::State& s, double mu, double radius, bool& bound, double& perigee_km, double& apogee_km) {
  const double r = sim::norm(s.r);
  const double v2 = sim::dot(s.v, s.v);
  const double energy = (0.5 * v2) - (mu / r);
  bound = false;
  if (!(energy < 0.0)) {
    return;
  }
  const double a = -mu / (2.0 * energy);
  const sim::V3 h = sim::cross(s.r, s.v);
  const double p = sim::dot(h, h) / mu;
  const double e = std::sqrt(std::fmax(0.0, 1.0 - (p / a)));
  bound = true;
  perigee_km = ((a * (1.0 - e)) - radius) / 1000.0;
  apogee_km = ((a * (1.0 + e)) - radius) / 1000.0;
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--config" && more) {
      o.config = argv[++i];
    } else if (a == "--flights" && more) {
      o.flights = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--seed" && more) {
      o.seed = std::strtoull(argv[++i], nullptr, 10);
    } else if (a == "--threads" && more) {
      o.threads = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--pad" && more) {
      o.pad_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--max-time" && more) {
      o.max_time = std::strtod(argv[++i], nullptr);
    } else if (a == "--list") {
      o.list = true;
    } else if (a == "--trace-landing") {
      o.trace_landing = true;
    } else if (a == "--trace" && more) {
      o.trace = std::atoi(argv[++i]);
    } else if (a == "--detail") {
      o.detail = true;
    } else if (a[0] != '-' && o.file == nullptr) {
      o.file = argv[i];
    } else {
      std::fprintf(stderr, "unknown option or missing value: %s\n", a.c_str());
      return 2;
    }
  }
  if (o.list) {
    for (const sim::mc::Target& t : sim::mc::targets()) {
      std::printf("%-28s %-10s %s\n", t.name, t.unit, t.meaning);
    }
    return 0;
  }
  if (o.file == nullptr || o.config == nullptr || o.threads == 0U) {
    std::fprintf(stderr, "usage: tfc_mission_mc MISSION.json --config DISPERSIONS.json [--flights N] [--seed S] [--threads T] [--pad FRAMES] [--max-time S] [--detail] [--trace N] [--list]\n");
    return 2;
  }
  sim::VehicleFile vf;
  std::vector<std::string> errors;
  if (!sim::load_vehicle_file(o.file, vf, errors) || !vf.mission.present) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    std::fprintf(stderr, "%s: not a vehicle file with a mission\n", o.file);
    return 1;
  }
  sim::mc::Config cfg;
  {
    std::ifstream in(o.config);
    std::stringstream ss;
    ss << in.rdbuf();
    if (!in || !sim::mc::read_config(ss.str(), cfg, errors)) {
      for (const std::string& e : errors) {
        std::fprintf(stderr, "%s\n", e.c_str());
      }
      return 1;
    }
  }
  if (o.flights != 0U) {
    cfg.flights = o.flights;
  }
  if (o.seed != 0U) {
    cfg.seed = o.seed;
  }
  for (const sim::mc::Dispersion& d : cfg.dispersions) {
    if (sim::mc::find_target(d.name) == nullptr) {
      std::fprintf(stderr, "unknown dispersion \"%s\" (--list shows the names)\n", d.name.c_str());
      return 1;
    }
  }
  sim::BuiltMission bm;
  if (!sim::build_mission(vf, bm, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 1;
  }
  const sim::MissionSpec& ms = vf.mission;
  const sim::DesignReport rep = sim::design_mission(vf, bm);
  if (!rep.ok) {
    std::fprintf(stderr, "the design run produced nothing\n");
    return 1;
  }
  sim::WorldConfig base;
  base.params = vf.params;
  base.scenario = vf.scenario;
  base.scenario.tower.enabled = ms.arm_height_m > 0.0;
  base.scenario.tower.height_m = ms.arm_height_m;
  base.scenario.tower.offset_y_m = ms.site_offset_y_m;
  base.scenario.tower.offset_z_m = ms.site_offset_z_m;
  base.scenario.tower.capture_radius_m = ms.capture_radius_m;
  base.main_tables = &bm.main;
  for (std::size_t s = 0; s < sim::kMaxStages; ++s) {
    base.stage_tables[s] = bm.has_stage[s] ? &bm.stage[s] : nullptr;
  }
  base.ideal = false;
  base.pad_frames = o.pad_frames;
  base.avionics.sensors = cfg.base.sensors;
  const double max_time = o.max_time > 0.0 ? o.max_time : ms.max_time_s;
  const uint32_t frames = static_cast<uint32_t>(max_time * 100.0) + base.pad_frames;
  const double mu = bm.main.nav.gravity.mu;
  const double radius = bm.main.nav.gravity.radius;

  std::vector<Outcome> out(cfg.flights);
  std::atomic<uint32_t> next{0U};
  const auto work = [&]() {
    for (;;) {
      const uint32_t idx = next.fetch_add(1U);
      if (idx >= cfg.flights) {
        return;
      }
      sim::mc::Rng rng = sim::mc::flight_rng(cfg.seed, idx);
      sim::Loop lp;                       // the targets act on a Loop: the parameters, the scenario and the sensors of one flight
      lp.cfg.params = base.params;
      lp.cfg.scenario = base.scenario;
      lp.sensors = base.avionics.sensors;
      Outcome& oc = out[idx];
      for (const sim::mc::Dispersion& d : cfg.dispersions) {
        const double v = sim::mc::draw(d, rng);
        oc.draws.push_back(v);
        sim::mc::find_target(d.name)->apply(lp, v);
      }
      const double gnss_sigma = lp.gnss_sigma_m > 0.0 ? lp.gnss_sigma_m : base.avionics.gnss.pos_sigma_m;
      sim::WorldConfig wc = base;
      wc.params = lp.cfg.params;
      wc.scenario = lp.cfg.scenario;
      wc.avionics.sensors = lp.sensors;
      wc.avionics.gnss.pos_sigma_m = gnss_sigma;
      wc.avionics.gnss.seed = 1000U + idx;
      wc.avionics.noise_seed = 7000U + (idx * 7U);
      sim::MissionWorld world(wc);
      double tail_since = -1.0;
      unsigned trace_phase = 99U;
      for (uint32_t k = 0U; k < frames; ++k) {
        world.step();
        if (world.clamped()) {
          continue;
        }
        const double t = static_cast<double>(world.flight_frame()) * 0.01;
        if (out[idx].ignition_miss_m < 0.0 && k % 10U == 0U) {
          for (std::size_t i = 1; i < world.bodies(); ++i) {
            const sim::MissionWorld::Body& b = world.body(i);
            if (b.av != nullptr && b.av->computer(0).mission().landing_out().ignite) {
              out[idx].ignition_miss_m = tfc::dm::norm(b.av->computer(0).mission().landing_out().miss);
            }
          }
        }
        if (o.trace == static_cast<int>(idx)) {                          // the phases of the returning stage: where it was, relative to the site, as each began
          for (std::size_t i = 1; i < world.bodies(); ++i) {
            const sim::MissionWorld::Body& b = world.body(i);
            if (b.av == nullptr) {
              continue;
            }
            const unsigned ph = b.av->computer(0).mission().phase();
            if (ph != trace_phase) {
              trace_phase = ph;
              const sim::V3 pp = b.veh.pad_offset(b.veh.state().r);
              std::printf("  t=%.1f phase %u: altitude %.1f km, speed %.0f m/s, %.2f km downrange and %.2f km across from the site, mass %.0f\n", t, ph, b.veh.altitude() / 1000.0, b.veh.speed(), (pp.y - 0.0) / 1000.0, (pp.z - ms.site_offset_z_m) / 1000.0, b.veh.mass());
            }
          }
        }
        if (o.trace == static_cast<int>(idx) && o.trace_landing && k % 25U == 0U) {
          for (std::size_t i = 1; i < world.bodies(); ++i) {
            const sim::MissionWorld::Body& b = world.body(i);
            if (b.av == nullptr || b.veh.altitude() > 4000.0 || b.veh.caught() || b.veh.landed()) {
              continue;
            }
            const tfc::descent::DescentOut& lo = b.av->computer(0).mission().landing_out();
            const sim::V3 up = b.veh.state().r * (1.0 / sim::norm(b.veh.state().r));
            std::printf("  t=%.1f alt=%.1f pins=%.1f down=%.2f m/s lateral=%.2f engines=%d throttle=%.2f height=%.1f miss=%.0f m mass=%.0f\n", t, b.veh.altitude(), b.veh.pin_height(), -sim::dot(b.veh.state().v, up), sim::norm(b.veh.state().v - (up * sim::dot(b.veh.state().v, up))),
                        static_cast<int>(lo.engines), lo.throttle, lo.height, tfc::dm::norm(lo.miss), b.veh.mass());
          }
        }
        bool all_done = world.bodies() > 0U;
        for (std::size_t i = 0; i < world.bodies(); ++i) {
          const sim::MissionWorld::Body& b = world.body(i);
          const bool ended = b.veh.landed() || b.veh.caught() || b.veh.crashed();
          unsigned ph = 0U;
          unsigned n = 1U;
          if (b.av != nullptr) {
            ph = b.av->computer(0).mission().phase();
            n = b.tables != nullptr ? b.tables->n_phases : 1U;
          }
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
      for (std::size_t i = 0; i < world.bodies(); ++i) {
        const sim::MissionWorld::Body& b = world.body(i);
        const sim::State& s = b.veh.state();
        if (!std::isfinite(s.r.x) || !std::isfinite(s.v.x)) {
          oc.finite = false;
        }
        oc.max_q_pa = std::max(oc.max_q_pa, b.env.max_q_pa);
        oc.max_q_sin_alpha_pa = std::max(oc.max_q_sin_alpha_pa, b.env.max_q_sin_alpha_pa);
        oc.max_axial_g = std::max(oc.max_axial_g, b.env.max_axial_g);
        if (b.stage >= 0) {                  // the stage that was let go and comes back
          oc.booster_max_q_pa = std::max(oc.booster_max_q_pa, b.env.max_q_pa);
          oc.caught = oc.caught || b.veh.caught();
          oc.landed = oc.landed || b.veh.landed();
          oc.crashed = oc.crashed || b.veh.crashed();
          if (b.veh.catch_pass().valid) {
            oc.passed = true;
            oc.pass_miss_m = b.veh.catch_pass().miss_m;
            oc.pass_sink_ms = b.veh.catch_pass().sink_ms;
            oc.pass_lateral_ms = b.veh.catch_pass().lateral_ms;
            oc.pass_tilt_deg = b.veh.catch_pass().tilt_deg;
          }
          if (b.veh.caught() || b.veh.landed() || b.veh.crashed()) {
            const sim::V3 site{0.0, 0.0, ms.site_offset_z_m};
            const sim::V3 p = b.veh.pad_offset(s.r);
            oc.miss_m = std::hypot(p.y - site.y, p.z - site.z);
            const sim::V3 up = s.r * (1.0 / sim::norm(s.r));
            const double vz = sim::dot(s.v, up);
            oc.end_sink_ms = -vz;
            oc.end_lateral_ms = sim::norm(s.v - (up * vz));
            oc.end_tilt_deg = std::acos(std::fmax(-1.0, std::fmin(1.0, sim::dot(sim::rotate(s.q, sim::V3{1.0, 0.0, 0.0}), up)))) * 57.29577951308232;
          }
        } else {
          bool bound = false;
          orbit_of(s, mu, radius, bound, oc.perigee_km, oc.apogee_km);
          oc.ship_orbit = bound && oc.perigee_km > 100.0 && !b.veh.crashed() && !b.veh.landed();
        }
      }
    }
  };
  std::vector<std::thread> pool;
  for (unsigned t = 1U; t < o.threads; ++t) {
    pool.emplace_back(work);
  }
  work();
  for (std::thread& t : pool) {
    t.join();
  }

  // ---- the report ----
  const unsigned n = cfg.flights;
  unsigned caught = 0U;
  unsigned orbit = 0U;
  unsigned both = 0U;
  unsigned bad = 0U;
  std::vector<double> miss, miss_all, qmax, qsa, bq, ax, perigee;
  for (const Outcome& c : out) {
    caught += c.caught ? 1U : 0U;
    orbit += c.ship_orbit ? 1U : 0U;
    both += (c.caught && c.ship_orbit) ? 1U : 0U;
    bad += c.finite ? 0U : 1U;
    if (c.caught) {
      miss.push_back(c.miss_m);
    }
    miss_all.push_back(c.miss_m >= 0.0 ? c.miss_m : 1e4);
    qmax.push_back(c.max_q_pa / 1000.0);
    qsa.push_back(c.max_q_sin_alpha_pa / 1000.0);
    bq.push_back(c.booster_max_q_pa / 1000.0);
    ax.push_back(c.max_axial_g);
    perigee.push_back(c.perigee_km);
  }
  if (o.detail) {
    for (uint32_t i = 0U; i < cfg.flights; ++i) {
      const Outcome& c = out[i];
      std::printf("  flight %u:", static_cast<unsigned>(i));
      for (std::size_t d = 0; d < cfg.dispersions.size(); ++d) {
        std::printf(" %s=%.4g", cfg.dispersions[d].name.c_str(), c.draws[d]);
      }
      std::printf("  ->  stage %s", c.caught ? "caught" : (c.landed ? "landed" : (c.crashed ? "crashed" : "not down")));
      if (c.passed) {
        std::printf(" (at the arms: %.1f m from the site, %.1f m/s down, %.1f m/s across, %.1f deg from upright; the arms take 1.5 m, 2.0, 1.5, 3.0)", c.pass_miss_m, c.pass_sink_ms, c.pass_lateral_ms, c.pass_tilt_deg);
      }
      std::printf("; the burn was lit %.0f m from the site", c.ignition_miss_m);
      std::printf("; ship perigee %.0f km apogee %.0f km; stage max q %.0f kPa\n", c.perigee_km, c.apogee_km, c.booster_max_q_pa / 1000.0);
    }
  }
  std::printf("%s: %u flights, seed %llu, %zu dispersions, pad %u frames\n", vf.name.c_str(), n, static_cast<unsigned long long>(cfg.seed), cfg.dispersions.size(), o.pad_frames);
  const auto line = [n](const char* what, unsigned k) {
    const std::array<double, 2> w = sim::mc::wilson(k, n);
    std::printf("  %-34s %3u of %u  (%.1f %%; 95 %% interval %.1f to %.1f %%)\n", what, k, n, 100.0 * k / n, 100.0 * w[0], 100.0 * w[1]);
  };
  line("the returning stage caught", caught);
  line("the ship in orbit, perigee > 100 km", orbit);
  line("both", both);
  if (bad > 0U) {
    std::printf("  %u flights ended with a non-finite state\n", bad);
  }
  const auto dist = [](const char* what, const char* unit, const std::vector<double>& v) {
    if (v.empty()) {
      return;
    }
    std::printf("  %-34s median %.1f, 95th %.1f, 99th %.1f, max %.1f %s\n", what, sim::mc::percentile(v, 50.0), sim::mc::percentile(v, 95.0), sim::mc::percentile(v, 99.0), *std::max_element(v.begin(), v.end()), unit);
  };
  dist("miss of the caught stages", "m", miss);
  dist("maximum q of any body", "kPa", qmax);
  dist("the returning stage's maximum q", "kPa", bq);
  dist("maximum q sin(alpha)", "kPa", qsa);
  dist("peak axial acceleration", "g", ax);
  // the draws that move with the miss and with the loads
  struct Rank {
    double r;
    std::string name;
  };
  const std::vector<std::pair<const char*, const std::vector<double>*>> rank_targets = {{"the miss", &miss_all}, {"the returning stage's maximum q", &bq}};
  for (const auto& target : rank_targets) {
    std::vector<Rank> ranks;
    for (std::size_t d = 0; d < cfg.dispersions.size(); ++d) {
      std::vector<double> x;
      for (const Outcome& c : out) {
        x.push_back(c.draws[d]);
      }
      ranks.push_back({sim::mc::pearson(x, *target.second), cfg.dispersions[d].name});
    }
    std::sort(ranks.begin(), ranks.end(), [](const Rank& a, const Rank& b) { return std::fabs(a.r) > std::fabs(b.r); });
    std::printf("  draws that move with %s:", target.first);
    for (std::size_t i = 0; i < ranks.size() && i < 4U; ++i) {
      std::printf(" %s (%+.2f)", ranks[i].name.c_str(), ranks[i].r);
    }
    std::printf("\n");
  }
  std::printf("  failures:");
  unsigned shown = 0U;
  for (uint32_t i = 0U; i < n && shown < 12U; ++i) {
    if (!out[i].caught || !out[i].ship_orbit) {
      std::printf(" #%u(%s%s)", i, out[i].caught ? "" : "no catch", out[i].ship_orbit ? "" : (out[i].caught ? "no orbit" : ", no orbit"));
      ++shown;
    }
  }
  std::printf("%s\n", shown == 0U ? " none" : "");
  return 0;
}
