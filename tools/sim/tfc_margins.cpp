// SPDX-License-Identifier: MIT
// Stability margins of the attitude loop along a mission (docs/design/CONTROL_LOOP.md section 7, docs/design/FUTURE_WORK.md 6a item 2).
//
//   tfc_margins MISSION.json [--delay MS] [--min-gm DB] [--min-pm DEG] [--every N] [--strict]
//
// The mission is designed as `tfc_mission` designs it (the nominal flight, probed every few seconds for the aerodynamic stiffness of each axis and the effect of each effector). At every probe the loop
// of each axis is rebuilt from what the flight computers really carry (the gains the table gives at that point of the phase, the allocation to the surfaces, the gimbal's thrust scaling) and what the
// probe measured (the stiffness a and the effectiveness b), with the servo of the effector and the delay of the frame, and analysed by sim/vehicle/margins.hpp: the gain margins up and down, the phase
// margin and the delay margin. The table of each phase gives the smallest of each over its probes and the time they were at. The limits asked for are the usual ones (6 dB, 30 degrees: from memory, to be
// replaced by the programme's own); `--strict` exits 3 when any loop is below them.
// Not in the loop: the slosh, the bending, the servo's rate limit and the controller's slew limit (margins.hpp says what that costs).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "margins.hpp"
#include "mission_design.hpp"
#include "spec_io.hpp"

namespace {

struct Options {
  const char* file = nullptr;
  double delay_ms = 15.0;
  double min_gm = 6.0;
  double min_pm = 30.0;
  unsigned every = 1U;
  bool strict = false;
  bool detail = false;   // the stiffness, the effectiveness, the gains and the crossover at the worst phase margin
};

struct Worst {
  unsigned n = 0U;
  double pm = 1e9, gm_lo = 1e9, gm_hi = 1e9, dm = 1e9;
  double t_pm = 0.0, t_lo = 0.0, t_hi = 0.0, t_dm = 0.0;
  unsigned unstable = 0U;
  double a = 0.0, b = 0.0, wc = 0.0, kp = 0.0, kd = 0.0, ki = 0.0, lag = 0.0;   // at the smallest phase margin
};

const char* kAxis[3] = {"roll", "yaw", "pitch"};

}  // namespace

int main(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--delay" && more) {
      o.delay_ms = std::strtod(argv[++i], nullptr);
    } else if (a == "--min-gm" && more) {
      o.min_gm = std::strtod(argv[++i], nullptr);
    } else if (a == "--min-pm" && more) {
      o.min_pm = std::strtod(argv[++i], nullptr);
    } else if (a == "--every" && more) {
      o.every = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--detail") {
      o.detail = true;
    } else if (a == "--strict") {
      o.strict = true;
    } else if (a[0] != '-' && o.file == nullptr) {
      o.file = argv[i];
    } else {
      std::fprintf(stderr, "usage: tfc_margins MISSION.json [--delay MS] [--min-gm DB] [--min-pm DEG] [--every N] [--strict]\n");
      return 2;
    }
  }
  if (o.file == nullptr || o.every == 0U) {
    std::fprintf(stderr, "usage: tfc_margins MISSION.json [--delay MS] [--min-gm DB] [--min-pm DEG] [--every N] [--strict]\n");
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
  sim::BuiltMission bm;
  if (!sim::build_mission(vf, bm, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 1;
  }
  const sim::DesignReport rep = sim::design_mission(vf, bm);
  if (!rep.ok) {
    std::fprintf(stderr, "the design run produced nothing\n");
    return 1;
  }
  std::printf("%s: %u probes of the nominal flight; delay %.0f ms; limits %.0f dB, %.0f degrees\n", vf.name.c_str(), static_cast<unsigned>(rep.probes), o.delay_ms, o.min_gm, o.min_pm);
  std::printf("%-9s %-5s %-5s %4s | %-18s | %-22s | %-22s | %s\n", "body", "phase", "axis", "n", "phase margin (deg)", "gain margin low (dB)", "gain margin high (dB)", "delay margin (ms)");
  bool below = false;
  const sim::VehicleSpec& spec = vf.params.spec;
  for (int owner = -1; owner < static_cast<int>(sim::kMaxStages); ++owner) {
    const tfc::gnc::Tables* tab = owner < 0 ? &bm.main : (bm.has_stage[static_cast<std::size_t>(owner)] ? &bm.stage[static_cast<std::size_t>(owner)] : nullptr);
    if (tab == nullptr) {
      continue;
    }
    for (unsigned ph = 0; ph < tab->n_phases; ++ph) {
      Worst w[3];
      unsigned count = 0U;
      for (const sim::AxisProbe& p : rep.probe_list) {
        if (p.stage != owner || p.phase != ph || (count++ % o.every) != 0U) {
          continue;
        }
        const tfc::gnc::GainTrack& tr = tab->gains[ph];
        if (tr.n == 0U) {
          continue;
        }
        const uint32_t frame = static_cast<uint32_t>(p.t_in_phase * 100.0 + 0.5);
        const tfc::att::Gains3 g = tr.at(frame);
        tfc::gnc::Mixer mx = tab->mixer[tab->phase[ph].mixer < tfc::gnc::kMaxMixers ? tab->phase[ph].mixer : 0U];
        if (tr.has_alloc) {
          const tfc::gnc::SurfAlloc al = tr.alloc_at(frame);
          mx.from_pitch = al.from_pitch;
          mx.from_yaw = al.from_yaw;
          mx.from_roll = al.from_roll;
        }
        // the gimbal's command is scaled at run time by the thrust it was designed at over the thrust it has
        const double designed = static_cast<double>(tr.thrust_at(frame));
        constexpr double kThrustFloorN = 1.0e4;   // as core/include/tfc/flight.hpp: below this there is no thrust to scale by, and the ratio is held between a tenth and ten
        const double ratio = designed > kThrustFloorN && p.thrust > kThrustFloorN ? std::min(10.0, std::max(0.1, designed / p.thrust)) : 1.0;
        const tfc::att::AxisGains* ag[3] = {&g.roll, &g.yaw, &g.pitch};
        for (int axis = 0; axis < 3; ++axis) {
          const std::size_t ax = static_cast<std::size_t>(axis);
          const double gimbal_gain = axis == 0 ? 0.0 : static_cast<double>(axis == 1 ? mx.gimbal_yaw : mx.gimbal_pitch);
          const double b_gimbal = (axis == 0 ? static_cast<double>(mx.roll) * p.col[2][0] : gimbal_gain * p.col[axis == 1 ? 1U : 0U][ax]) * (axis == 0 ? 1.0 : ratio);
          double b_surf = 0.0;
          double lag_sum = 0.0;
          double lag_wt = 0.0;
          for (std::size_t k = 0; k < sim::kSurfaceChannels; ++k) {
            const double gain = static_cast<double>(axis == 0 ? mx.from_roll[k] : (axis == 1 ? mx.from_yaw[k] : mx.from_pitch[k]));
            const double c = gain * p.col[3U + k][ax];
            b_surf += c;
            for (const sim::SurfaceSpec& sf : spec.surfaces) {
              if (sf.channel == static_cast<int>(k)) {
                lag_sum += std::fabs(c) * std::max(sf.lag_s, 0.02);
                lag_wt += std::fabs(c);
              }
            }
          }
          const double b = b_gimbal + b_surf;
          const double kmax = std::max({std::fabs(static_cast<double>(ag[axis]->kp)), std::fabs(static_cast<double>(ag[axis]->kd))});
          if (!(std::fabs(b) > 1e-9) || !(kmax > 1e-9)) {
            continue;
          }
          if (axis != 0 && std::fabs(b_gimbal) >= std::fabs(b_surf) && p.thrust < kThrustFloorN) {
            continue;   // the gimbal moves nothing without thrust (on the pad at T-zero, or in a coast): there is no loop to have a margin
          }
          double gimbal_lag = 0.0;
          for (const sim::StageSpec& st : spec.stages) {
            gimbal_lag = std::max(gimbal_lag, st.gimbal_lag_s >= 0.0 ? st.gimbal_lag_s : vf.params.gimbal_lag_s);
          }
          if (spec.stages.empty()) {
            gimbal_lag = vf.params.gimbal_lag_s;
          }
          const double lag_s = (std::fabs(b_gimbal) >= std::fabs(b_surf) || lag_wt <= 0.0) ? (axis == 0 ? 0.05 : std::max(gimbal_lag, 0.02)) : lag_sum / lag_wt;
          sim::margins::Loop l;
          l.a = p.a[ax];
          l.b = b;
          l.kp = static_cast<double>(ag[axis]->kp);
          l.kd = static_cast<double>(ag[axis]->kd);
          l.ki = static_cast<double>(ag[axis]->ki);
          l.delay_s = o.delay_ms * 1e-3;
          if (spec.actuator.order == 2 && axis != 0 && std::fabs(b_gimbal) >= std::fabs(b_surf)) {
            l.servo.order = 2;
            l.servo.natural_hz = spec.actuator.natural_hz;
            l.servo.damping = spec.actuator.damping;
          } else {
            l.servo.lag_s = lag_s;
          }
          const sim::margins::Result r = sim::margins::analyse(l);
          Worst& x = w[axis];
          ++x.n;
          if (!r.stable) {
            if (o.detail) {
              std::printf("          unstable: phase %u, %s, t = %.1f s: a = %.4f, b = %.5f, kp = %.4f, kd = %.4f, ki = %.5f, thrust %.0f N (designed at %.0f), servo lag %.3f s\n", ph, kAxis[axis], p.t, l.a, l.b, l.kp, l.kd, l.ki, p.thrust, designed, lag_s);
            }
            ++x.unstable;
            x.pm = -999.0;
            x.t_pm = p.t;
            continue;
          }
          if (r.phase_margin_deg < x.pm) {
            x.pm = r.phase_margin_deg;
            x.t_pm = p.t;
            x.a = l.a;
            x.b = l.b;
            x.wc = r.crossover_rad_s;
            x.kp = l.kp;
            x.kd = l.kd;
            x.ki = l.ki;
            x.lag = l.servo.lag_s;
          }
          if (r.gain_lower_db < x.gm_lo) {
            x.gm_lo = r.gain_lower_db;
            x.t_lo = p.t;
          }
          if (r.gain_upper_db < x.gm_hi) {
            x.gm_hi = r.gain_upper_db;
            x.t_hi = p.t;
          }
          if (r.delay_margin_s * 1e3 < x.dm) {
            x.dm = r.delay_margin_s * 1e3;
            x.t_dm = p.t;
          }
        }
      }
      for (int axis = 0; axis < 3; ++axis) {
        const Worst& x = w[axis];
        if (x.n == 0U) {
          continue;
        }
        const char* flag = "";
        if (x.unstable > 0U) {
          flag = "  UNSTABLE";
        } else if (x.pm < o.min_pm || std::min(x.gm_lo, x.gm_hi) < o.min_gm) {
          flag = "  BELOW";
        }
        below = below || *flag != '\0';
        const std::string name = owner < 0 ? "main" : (static_cast<std::size_t>(owner) < spec.stages.size() ? spec.stages[static_cast<std::size_t>(owner)].name : "stage");
        if (x.unstable > 0U) {
          std::printf("%-9s %-5u %-5s %4u | %u of %u probes UNSTABLE (first at t = %.0f s)%s\n", name.c_str(), ph, kAxis[axis], x.n, x.unstable, x.n, x.t_pm, flag);
        } else {
          std::printf("%-9s %-5u %-5s %4u | %6.1f at %5.0f s    | %6.1f at %5.0f s      | %6.1f at %5.0f s      | %6.0f at %5.0f s%s\n", name.c_str(), ph, kAxis[axis], x.n, x.pm, x.t_pm, x.gm_lo, x.t_lo, x.gm_hi, x.t_hi, x.dm, x.t_dm, flag);
        }
        if (o.detail && x.unstable == 0U) {
          std::printf("          worst phase margin: a = %.3f 1/s^2, b = %.4f rad/s^2 per degree, kp = %.4f, kd = %.4f, ki = %.5f, servo lag %.3f s, crossover %.2f rad/s\n", x.a, x.b, x.kp, x.kd, x.ki, x.lag, x.wc);
        }
      }
    }
  }
  std::printf("%s\n", below ? "SOME LOOPS ARE BELOW THE LIMITS" : "every loop is within the limits");
  return below && o.strict ? 3 : 0;
}
