// SPDX-License-Identifier: MIT
// The propulsion of a vehicle, analysed (docs/design/PROPULSION.md; sim/vehicle/propulsion.hpp).
//
//   tfc_propulsion VEHICLE.json [--strict] [--fill 1,0.5,0.05] [--detail]
//
// For each stage with engines: the cluster (how many engines, how many gimbal, the thrust and the specific impulse in vacuum and at sea level, the burn and the velocity it can give), the thrust
// and specific impulse against altitude, each kind of engine's nozzle against an ideal one (and the throttle below which it separates at sea level), the gimbal's authority at points of the burn
// (the angular acceleration a degree of gimbal gives, the roll the differential gimbal gives against the roll thrusters, the rate the servo can follow), and what an engine out, and the worst
// pair, do to the balance of the cluster: the torque it leaves, the gimbal that trims it, and whether that is within the travel. A finding is a line that begins "!"; `--strict` exits 3 when there is one.
//
// Everything is computed from the vehicle file in closed form. The plant is the reference and the tests fly the same cluster to check this one.
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "atmosphere.hpp"
#include "propulsion.hpp"
#include "spec_io.hpp"

namespace {

using namespace sim;
using namespace sim::propulsion;

struct Options {
  const char* file = nullptr;
  bool strict = false;
  bool detail = false;
  std::vector<double> fill{1.0, 0.5, 0.05};
};

unsigned g_findings = 0;

void finding(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void finding(const char* fmt, ...) {
  ++g_findings;
  std::printf("  ! ");
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::printf("\n");
}

std::vector<double> parse_list(const char* s) {
  std::vector<double> out;
  const char* p = s;
  while (*p != '\0') {
    char* end = nullptr;
    out.push_back(std::strtod(p, &end));
    if (end == p) {
      break;
    }
    p = *end == ',' ? end + 1 : end;
  }
  return out;
}

bool same_kind(const EngineSpec& a, const EngineSpec& b) {
  return a.thrust_vac == b.thrust_vac && a.exit_area == b.exit_area && a.isp_vac == b.isp_vac && a.chamber_pressure_pa == b.chamber_pressure_pa && a.area_ratio == b.area_ratio && a.gamma == b.gamma;
}

void print_cluster(const VehicleSpec& g, std::size_t s, const Cluster& c) {
  std::printf("  %u main engines (%u gimbal, %u fixed): %.2f MN in vacuum, %.2f MN at sea level, %.0f kg/s; Isp %.1f s in vacuum, %.1f s at sea level\n", c.n, c.n_gimbal, c.n_fixed, c.f_vac / 1e6, c.f_sl / 1e6,
              c.mdot, c.isp_vac, c.isp_sl);
  // the rings: engines at the same radius
  std::vector<std::pair<double, unsigned>> rings;
  for (const std::size_t i : main_engines(g, s)) {
    const double r = std::round(std::hypot(g.engines[i].pos.y, g.engines[i].pos.z) * 100.0) / 100.0;
    auto it = std::find_if(rings.begin(), rings.end(), [r](const std::pair<double, unsigned>& q) { return q.first == r; });
    if (it == rings.end()) {
      rings.emplace_back(r, 1U);
    } else {
      ++it->second;
    }
  }
  std::sort(rings.begin(), rings.end());
  std::printf("  rings:");
  for (const auto& q : rings) {
    std::printf(" %u at r = %.2f m;", q.second, q.first);
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--strict") {
      o.strict = true;
    } else if (a == "--detail") {
      o.detail = true;
    } else if (a == "--fill" && i + 1 < argc) {
      o.fill = parse_list(argv[++i]);
    } else if (a[0] != '-' && o.file == nullptr) {
      o.file = argv[i];
    } else {
      std::fprintf(stderr, "usage: tfc_propulsion VEHICLE.json [--strict] [--fill 1,0.5,0.05] [--detail]\n");
      return 2;
    }
  }
  if (o.file == nullptr || o.fill.empty()) {
    std::fprintf(stderr, "usage: tfc_propulsion VEHICLE.json [--strict] [--fill 1,0.5,0.05] [--detail]\n");
    return 2;
  }
  VehicleFile f;
  std::vector<std::string> errors;
  if (!load_vehicle_file(o.file, f, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 2;
  }
  const VehicleSpec& g0 = f.params.spec;
  std::printf("%s: %zu stage(s), %zu engines (%s)\n", f.name.empty() ? o.file : f.name.c_str(), g0.stages.size(), g0.engines.size(),
              g0.per_engine_gimbal ? "each engine gimbals on its own" : "the engines of a stage share one gimbal angle");
  Vehicle6 stack(f.params, f.scenario);
  // the vehicle as it is after each separation (the stage below gone), so that an upper stage is analysed alone
  std::vector<Vehicle6> states{stack};
  for (std::size_t s = 1; s < g0.stages.size(); ++s) {
    Vehicle6 nxt = states.back();
    Controls c;
    c.events = ev::kSeparate;
    nxt.step(0.001, c);
    (void)nxt.take_detached();   // (the stage that left; the ones that remain are the vehicle)
    states.push_back(nxt);
  }
  for (std::size_t s = 0; s < g0.stages.size(); ++s) {
    const std::vector<std::size_t> mains = main_engines(g0, s);
    if (mains.empty()) {
      continue;
    }
    const Vehicle6& v = states[s];
    std::printf("\n== stage %zu: %s ==\n", s, g0.stages[s].name.c_str());
    const Cluster cl = cluster_of(g0, s, kSeaLevelPa);
    print_cluster(g0, s, cl);

    // the burn: from full to empty with the stages above full
    std::array<double, kMaxStages> full{};
    std::array<double, kMaxStages> empty{};
    for (std::size_t k = s; k < g0.stages.size() && k < kMaxStages; ++k) {
      full[k] = v.stage_capacity(k);
      empty[k] = k == s ? 0.0 : v.stage_capacity(k);
    }
    const double m0 = v.mass_props_with(full).mass;
    const double mf = v.mass_props_with(empty).mass;
    const double prop = v.stage_capacity(s);
    const double dv = cl.isp_vac * 9.80665 * std::log(m0 / mf);
    std::printf("  burn: %.0f t of propellant for %.1f s at full throttle; %.0f t at the start, %.0f t at the end; %.0f m/s from the rocket equation at the vacuum Isp (before gravity and drag)\n", prop / 1e3,
                cl.mdot > 0.0 ? prop / cl.mdot : 0.0, m0 / 1e3, mf / 1e3, dv);
    std::printf("  thrust-to-weight at the start: %.2f at sea level, %.2f in vacuum\n", cl.f_sl / (m0 * 9.80665), cl.f_vac / (m0 * 9.80665));
    if (s == 0 && cl.f_sl / (m0 * 9.80665) < 1.0 && g0.stages[s].guided == false) {
      finding("the first stage cannot lift off: thrust-to-weight %.2f at sea level", cl.f_sl / (m0 * 9.80665));
    }

    // against altitude
    std::printf("  against altitude:\n    alt km   pressure Pa   thrust MN   Isp s   T/W (at the start)\n");
    for (const double km : {0.0, 5.0, 10.0, 20.0, 40.0, 80.0}) {
      const double pa = air_at(km * 1000.0).pressure;
      const Cluster c = cluster_of(g0, s, pa);
      std::printf("    %5.0f   %11.0f   %9.2f   %5.1f   %6.2f\n", km, pa, c.f_sl / 1e6, c.isp_sl, c.f_sl / (m0 * 9.80665));
    }

    // the nozzles, by kind of engine
    std::vector<EngineSpec> kinds;
    for (const std::size_t i : mains) {
      if (std::none_of(kinds.begin(), kinds.end(), [&](const EngineSpec& k) { return same_kind(k, g0.engines[i]); })) {
        kinds.push_back(g0.engines[i]);
      }
    }
    for (const EngineSpec& k : kinds) {
      const NozzleReport nr = check_nozzle(k);
      const unsigned count = static_cast<unsigned>(std::count_if(mains.begin(), mains.end(), [&](std::size_t i) { return same_kind(k, g0.engines[i]); }));
      if (!nr.given) {
        std::printf("  engine kind (x%u: %.2f MN vacuum, exit area %.2f m^2, Isp %.0f s): no chamber pressure, area ratio and gamma in the file, so no nozzle check\n", count, k.thrust_vac / 1e6, k.exit_area, k.isp_vac);
        continue;
      }
      std::printf("  nozzle of the engine kind x%u (%.2f MN vacuum, pc %.0f bar, area ratio %.0f, gamma %.2f): exit Mach %.2f, exit pressure %.2f bar at full throttle, ideal Cf %.3f in vacuum and %.3f at sea level\n",
                  count, k.thrust_vac / 1e6, nr.pc / 1e5, nr.eps, nr.gamma, nr.mach_exit, nr.pe_full / 1e5, nr.cf_vac, nr.cf_sl);
      std::printf("    the throat that gives the file's thrust is %.4f m^2 (%.3f m across); the ideal exit area is %.2f m^2, the file's is %.2f m^2 (%+.0f%%)\n", nr.throat_area,
                  2.0 * std::sqrt(nr.throat_area / 3.14159265358979), nr.exit_area_ideal, nr.exit_area_file, 100.0 * nr.area_mismatch);
      std::printf("    sea-level thrust over vacuum thrust: %.3f for the ideal nozzle, %.3f in the file; sea-level separation (pe/pa < %.2f) below %.0f%% throttle\n", nr.sl_ratio_ideal, nr.sl_ratio_file,
                  kSummerfield, 100.0 * nr.separation_throttle);
      if (nr.separation_throttle > 1.0) {
        std::printf("    note: this nozzle is over-expanded at sea level even at full throttle (pe %.2f bar against 1.01 bar of air): the engine is for the vacuum and must not be lit in the lower atmosphere\n", nr.pe_full / 1e5);
      }
      if (std::fabs(nr.area_mismatch) > 0.10) {
        finding("an engine's exit area (%.2f m^2) is %+.0f%% from the ideal nozzle's for its area ratio, chamber pressure and thrust: one of the four numbers in the file is wrong, or the nozzle is not what it is described as", nr.exit_area_file,
                100.0 * nr.area_mismatch);
      }
      if (std::fabs(nr.sl_ratio_ideal - nr.sl_ratio_file) > 0.03) {
        finding("the file's sea-level thrust loss (%.3f of vacuum) differs from the ideal nozzle's (%.3f) by more than 3%%", nr.sl_ratio_file, nr.sl_ratio_ideal);
      }
      if (k.min_throttle > 0.0 && nr.separation_throttle > k.min_throttle + 1e-9 && s == 0) {
        finding("at sea level the engine separates below %.0f%% throttle but the file lets it run down to %.0f%%: expect side loads in the landing burn and at lift-off throttle-down", 100.0 * nr.separation_throttle,
                100.0 * k.min_throttle);
      }
    }

    // the gimbal's authority
    const EngineSpec* first_gimbal = nullptr;
    for (const std::size_t i : mains) {
      if (g0.engines[i].gimbal && first_gimbal == nullptr) {
        first_gimbal = &g0.engines[i];
      }
    }
    if (first_gimbal == nullptr) {
      std::printf("  no engine of this stage gimbals\n");
      continue;
    }
    const ServoReport sv = servo_report(f.params, s, *first_gimbal);
    std::printf("  gimbal servo: travel %.1f deg, rate %.1f deg/s, lag %.3f s%s; a sine of the full travel is followed up to %.2f Hz, one of 2 degrees up to %.2f Hz; the lag's corner is %.2f Hz\n", sv.limit_deg,
                sv.rate_dps, sv.lag_s, sv.second_order_hz > 0.0 ? " (a second-order servo as well)" : "", sv.rate_full_hz, sv.rate_2deg_hz, sv.lag_corner_hz);
    std::printf("  authority (vacuum thrust, the stages above full):\n    fill   mass t   cg m   It kg m^2   pitch/yaw per deg   at travel   roll by diff. gimbal per deg   at travel   roll thrusters\n");
    for (const double fl : o.fill) {
      const Authority a = authority(v, f.params, s, fl, 0.0);
      std::printf("    %4.2f   %6.0f   %5.1f   %9.3g   %9.3f rad/s2   %6.3f      %11.4f rad/s2             %7.3f      %7.4f rad/s2\n", fl, a.mass / 1e3, a.x_cg, a.i_t, a.pitch_accel_per_deg, a.pitch_accel_max,
                  a.roll_accel_per_deg, a.roll_accel_max, a.thruster_roll_accel);
    }
    const Authority a_end = authority(v, f.params, s, o.fill.back(), 0.0);
    if (a_end.roll_accel_per_deg <= 0.0 && a_end.thruster_roll_accel <= 0.0) {
      if (f.params.ideal_roll_control && !g0.roll.enabled) {
        std::printf("  roll: the plant holds it ideally (ideal_roll_control), so the vehicle file needs no roll actuator\n");
      } else {
        finding("stage %zu has no roll authority at all (no differential gimbal, no roll thrusters)", s);
      }
    }

    // engine out
    std::printf("  engine out (at sea-level thrust, fill %.2f and %.2f):\n", o.fill.front(), o.fill.back());
    for (const double fl : {o.fill.front(), o.fill.back()}) {
      EngineOut worst;
      double worst_frac = -1.0;
      unsigned beyond = 0;
      for (const std::size_t i : mains) {
        const EngineOut r = engine_out(v, f.params, s, fl, kSeaLevelPa, {i});
        if (o.detail) {
          std::printf("    engine %zu at (%.2f, %.2f) m (%s): loses %.1f%% of the thrust, torque %.2f MN m, trim %.2f deg pitch %.2f deg yaw = %.0f%% of the travel\n", i, g0.engines[i].pos.y, g0.engines[i].pos.z,
                      g0.engines[i].gimbal ? "gimbal" : "fixed", 100.0 * r.thrust_loss, r.imbalance_nm / 1e6, r.trim_pitch_deg, r.trim_yaw_deg, 100.0 * r.trim_fraction);
        }
        beyond += r.trim_fraction > 1.0 ? 1U : 0U;
        if (r.trim_fraction > worst_frac) {
          worst_frac = r.trim_fraction;
          worst = r;
        }
      }
      const Authority whole = authority(v, f.params, s, fl, kSeaLevelPa);
      std::printf("    fill %.2f, one engine out: the worst (engine %zu at (%.2f, %.2f) m) loses %.1f%% of the thrust, leaves %.2f MN m (the thrust acts %.0f mm off the axis), and its trim is %.2f deg pitch and %.2f deg yaw: %.0f%% of the %.1f deg travel; %u of %zu engines beyond the travel\n",
                  fl, worst.out[0], g0.engines[worst.out[0]].pos.y, g0.engines[worst.out[0]].pos.z, 100.0 * worst.thrust_loss, worst.imbalance_nm / 1e6, 1000.0 * worst.offset_m, worst.trim_pitch_deg,
                  worst.trim_yaw_deg, 100.0 * worst.trim_fraction, whole.limit_deg, beyond, mains.size());
      if (worst.trim_fraction > 1.0) {
        finding("stage %zu at fill %.2f: an engine out can unbalance the cluster beyond what the gimbal can trim (%.0f%% of the travel)", s, fl, 100.0 * worst.trim_fraction);
      } else if (worst.trim_fraction > 0.5) {
        finding("stage %zu at fill %.2f: the worst engine out uses %.0f%% of the gimbal's travel to trim, leaving little to steer with", s, fl, 100.0 * worst.trim_fraction);
      }
      // the worst pair
      EngineOut wp;
      double wp_frac = -1.0;
      for (std::size_t a = 0; a < mains.size(); ++a) {
        for (std::size_t b = a + 1U; b < mains.size(); ++b) {
          const EngineOut r = engine_out(v, f.params, s, fl, kSeaLevelPa, {mains[a], mains[b]});
          if (r.trim_fraction > wp_frac) {
            wp_frac = r.trim_fraction;
            wp = r;
          }
        }
      }
      std::printf("    fill %.2f, two engines out: the worst pair (engines %zu and %zu) loses %.1f%% of the thrust and its trim is %.0f%% of the travel\n", fl, wp.out[0], wp.out[1], 100.0 * wp.thrust_loss,
                  100.0 * wp.trim_fraction);
    }

    // the throttle
    const double min_t = [&] {
      double lowest = 1.0e30;
      for (const std::size_t i : mains) {
        const EngineSpec& e = g0.engines[i];
        lowest = std::min(lowest, engine_thrust(e, kSeaLevelPa, std::max(e.min_throttle, 0.0)));
      }
      return lowest;
    }();
    const double m_end = authority(v, f.params, s, o.fill.back(), 0.0).mass;
    if (min_t > 0.0) {
      std::printf("  the lowest thrust one engine can hold at sea level is %.0f kN; the stage at fill %.2f weighs %.0f kN (thrust-to-weight %.2f for one engine at its lowest throttle)\n", min_t / 1e3, o.fill.back(),
                  m_end * 9.80665 / 1e3, min_t / (m_end * 9.80665));
    } else {
      std::printf("  the engines have no lowest throttle in the file (min_throttle 0)\n");
    }
  }
  std::printf("\n%u finding(s)\n", g_findings);
  return o.strict && g_findings > 0 ? 3 : 0;
}
