// SPDX-License-Identifier: MIT
// Sensitivity of the closed loop: for each departure from the nominal (a sensor error, a dispersed vehicle, a late frame) how large can it grow before the flight is lost?
// The whole software chain is flown (sim/vehicle/closed_loop.hpp: the simulated vehicle, three flight functions with their IMU models, the real ACT logic) with one departure
// scaled up until the flight fails, found by bisection. A flight FAILS if, after the first 3 s, the attitude strays more than `--limit` degrees from the pitch program (the lift-off transient is reported separately), the platform saturates, the vehicle is still on the ground at 3 s or is destroyed, ACT enters
// Safe, or a value is not finite. Run it twice, with the platform's sensors (the rig) and with the vehicle's own (a real vehicle): gravity is observable on the platform and not on
// a vehicle under thrust, which changes what a gyro bias does.
//   tfc_sens [--mode platform|vehicle] [--no-accel] [--pad FRAMES] [--frames N] [--limit DEG] [--only NAME] [--list]
// --pad FRAMES puts a pad phase before T-zero (the vehicle clamped, the estimators calibrating their gyros, ACT going Nominal); 1500 is 15 s.
// --no-accel switches the estimator's accelerometer correction off (EstimatorConfig::use_accel), which is what a vehicle under thrust needs.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <future>
#include <string>
#include <vector>

#include "closed_loop.hpp"

namespace {

struct Departure {
  const char* name;
  const char* unit;
  double cap;                                       // the largest value tried
  bool integer;                                     // frames: bisect over whole numbers
  std::function<void(sim::Loop&, double)> apply;    // set the departure to this value
};

struct Verdict {
  bool ok = true;
  sim::Result r;
};

Verdict fly(const sim::Loop& base, const Departure& d, double v, double limit_deg) {
  sim::Loop lp = base;
  d.apply(lp, v);
  Verdict out;
  out.r = sim::run(lp);
  // A flight that never rose is not a flight (found 6 Oct 2026: with 30% less thrust the vehicle stayed on the pad, or before the ground model sank through it, and "flew"
  // with a small attitude error): it must be off the ground by 3 s, and must not have been destroyed.
  out.ok = out.r.finite && out.r.max_deg_settled <= limit_deg && out.r.safe_frames == 0U && out.r.platform_saturated == 0U && !out.r.crashed && out.r.liftoff_frame <= 300U;
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  bool vehicle_true = false;
  uint32_t frames = 8000U;
  double limit_deg = 5.0;
  std::string only;
  bool list = false;
  bool no_accel = false;
  uint32_t pad_frames = 0U;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--mode" && i + 1 < argc) {
      vehicle_true = std::string(argv[++i]) == "vehicle";
    } else if (a == "--frames" && i + 1 < argc) {
      frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--limit" && i + 1 < argc) {
      limit_deg = std::atof(argv[++i]);
    } else if (a == "--only" && i + 1 < argc) {
      only = argv[++i];
    } else if (a == "--pad" && i + 1 < argc) {
      pad_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--no-accel") {
      no_accel = true;
    } else if (a == "--list") {
      list = true;
    } else {
      std::fprintf(stderr, "usage: tfc_sens [--mode platform|vehicle] [--no-accel] [--pad FRAMES] [--frames N] [--limit DEG] [--only NAME] [--list]\n");
      return 2;
    }
  }
  using L = sim::Loop&;
  const std::vector<Departure> deps = {
      {"gyro bias", "dps", 8.0, false, [](L l, double v) { l.sensors.gyro_bias_dps = static_cast<float>(v); }},
      {"gyro scale error", "fraction", 0.5, false, [](L l, double v) { l.sensors.gyro_scale_err = static_cast<float>(v); }},
      {"accelerometer bias", "g", 0.3, false, [](L l, double v) { l.sensors.accel_bias_g = static_cast<float>(v); }},
      {"accelerometer scale error", "fraction", 0.5, false, [](L l, double v) { l.sensors.accel_scale_err = static_cast<float>(v); }},
      {"IMU misalignment", "deg", 20.0, false, [](L l, double v) { l.sensors.misalign_deg = static_cast<float>(v); }},
      {"gyro noise (times 0.17 dps)", "x", 200.0, false, [](L l, double v) { l.sensors.gyro_noise_amp_dps = static_cast<float>(0.17 * v); }},
      {"accelerometer noise (times 3.5 mg)", "x", 200.0, false, [](L l, double v) { l.sensors.accel_noise_amp_g = static_cast<float>(0.0035 * v); }},
      {"sensor latency (extra frames)", "frames", 7.0, true, [](L l, double v) { l.sensors.latency_frames = static_cast<unsigned>(v); }},
      {"stale sample probability", "fraction", 0.95, false, [](L l, double v) { l.sensors.stale_prob = static_cast<float>(v); }},
      {"gimbal actuator lag", "s", 0.5, false, [](L l, double v) { l.cfg.params.gimbal_lag_s = v; }},
      {"thrust misalignment, pitch", "deg", 3.0, false, [](L l, double v) { l.cfg.params.thrust_misalign_pitch_deg = v; }},
      {"thrust misalignment, yaw", "deg", 3.0, false, [](L l, double v) { l.cfg.params.thrust_misalign_yaw_deg = v; }},
      {"thrust low (fraction lost)", "fraction", 0.5, false, [](L l, double v) { l.cfg.params.thrust_scale = 1.0 - v; }},
      {"thrust high (fraction gained)", "fraction", 0.5, false, [](L l, double v) { l.cfg.params.thrust_scale = 1.0 + v; }},
      {"normal-force slope high (fraction gained)", "fraction", 3.0, false, [](L l, double v) { l.cfg.params.cn_scale = 1.0 + v; }},
      {"normal-force slope low (fraction lost)", "fraction", 0.9, false, [](L l, double v) { l.cfg.params.cn_scale = 1.0 - v; }},
      {"axial force high (fraction gained)", "fraction", 2.0, false, [](L l, double v) { l.cfg.params.cd_scale = 1.0 + v; }},
      {"mean wind (times the profile)", "x", 8.0, false, [](L l, double v) { l.cfg.scenario.wind_scale = v; }},
      {"centre of gravity forward", "m", 3.0, false, [](L l, double v) { l.cfg.scenario.dry_cg_shift = v; }},
      {"centre of gravity aft", "m", 3.0, false, [](L l, double v) { l.cfg.scenario.dry_cg_shift = -v; }},
  };
  if (list) {
    for (const Departure& d : deps) {
      std::printf("%s\n", d.name);
    }
    return 0;
  }
  sim::Loop base;
  base.frames = frames;
  base.cfg.vehicle_true = vehicle_true;
  base.estimator.use_accel = !no_accel;
  base.pad_frames = pad_frames;
  const Verdict nominal = fly(base, deps[0], 0.0, limit_deg);
  std::printf("Mode: %s sensors%s%s; %u frames; a flight fails if, after 3 s, it is more than %.1f degrees from the program, or on platform saturation, Safe or a non-finite value, or if the vehicle is still on the ground at 3 s or is destroyed.\n",
              vehicle_true ? "vehicle" : "platform", no_accel ? ", accelerometer correction off" : "", pad_frames > 0U ? ", with a pad phase" : "", static_cast<unsigned>(frames), limit_deg);
  std::printf("Nominal: %s (max error %.2f deg after 3 s and %.2f in the first 3 s, rms %.3f deg)\n\n", nominal.ok ? "flies" : "FAILS", nominal.r.max_deg_settled, nominal.r.max_deg_liftoff, nominal.r.rms_deg);
  std::printf("| Departure | Largest value that still flies | Unit | Max error there after 3 s (deg) | Lift-off transient there (deg) |\n|---|---|---|---|---|\n");

  // Each departure is searched on its own thread: the cap first (if it flies, report ">= cap"), then bisection.
  std::vector<std::future<std::string>> jobs;
  for (const Departure& d : deps) {
    if (!only.empty() && only != d.name) {
      continue;
    }
    jobs.push_back(std::async(std::launch::async, [&base, &d, limit_deg]() {
      char line[256];
      const Verdict top = fly(base, d, d.cap, limit_deg);
      if (top.ok) {
        std::snprintf(line, sizeof line, "| %s | at least %g (the largest tried) | %s | %.2f | %.2f |", d.name, d.cap, d.unit, top.r.max_deg_settled, top.r.max_deg_liftoff);
        return std::string(line);
      }
      double lo = 0.0;
      double hi = d.cap;
      Verdict best = fly(base, d, 0.0, limit_deg);
      if (!best.ok) {
        std::snprintf(line, sizeof line, "| %s | the nominal flight already fails | %s | | |", d.name, d.unit);
        return std::string(line);
      }
      for (int it = 0; it < 8; ++it) {
        double mid = 0.5 * (lo + hi);
        if (d.integer) {
          mid = std::floor(mid);
          if (mid <= lo) {
            break;
          }
        }
        const Verdict v = fly(base, d, mid, limit_deg);
        if (v.ok) {
          lo = mid;
          best = v;
        } else {
          hi = mid;
        }
      }
      std::snprintf(line, sizeof line, "| %s | %g | %s | %.2f | %.2f |", d.name, lo, d.unit, best.r.max_deg_settled, best.r.max_deg_liftoff);
      return std::string(line);
    }));
  }
  for (auto& j : jobs) {
    std::printf("%s\n", j.get().c_str());
  }
  return 0;
}
