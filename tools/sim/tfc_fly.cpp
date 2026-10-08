// SPDX-License-Identifier: MIT
// Fly any vehicle with the real flight software: read a vehicle description (docs/design/VEHICLE_SPEC.md), design the pitch program and the gain schedule on its nominal flight, and close the
// loop over it (the simulated vehicle, three flight functions with their IMU models, the real ACT logic: sim/vehicle/closed_loop.hpp). Reports what happened, and can write the whole flight
// as a CSV for a plot.
//   tfc_fly VEHICLE.json|reference [--frames N] [--pad FRAMES] [--sensors platform|vehicle] [--no-accel] [--csv FILE] [--every N] [--check] [--dump FILE]
// "reference" is the built-in reference vehicle of VEHICLE_SIM.md. --frames: frames of flight after T-zero (default: the design's flight time). --check: only read and validate the file (and print
// what is in it), do not fly. --dump: write the vehicle back out as a normalised file (every field, in the order the format documents) and stop.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "closed_loop.hpp"
#include "spec_io.hpp"

namespace {

void describe(const sim::VehicleFile& v) {
  const sim::VehicleSpec g = v.params.spec.empty() ? sim::legacy_spec(v.params) : v.params.spec;
  sim::Vehicle6 veh(v.params, v.scenario);
  const sim::MassProps mp = veh.mass_props();
  std::printf("%s: %s\n", v.name.empty() ? "(unnamed vehicle)" : v.name.c_str(), v.description.c_str());
  std::printf("  %zu stage(s), %zu engine(s), %zu payload(s), %zu fin set(s)%s; mass at T-zero %.0f kg, centre of gravity %.2f m, transverse inertia %.0f kg m^2\n", g.stages.size(), g.engines.size(), g.payloads.size(),
              g.fins.size(), g.wheels.enabled ? ", reaction wheels" : "", veh.mass(), mp.x_cg, mp.i_t);
  for (std::size_t s = 0; s < g.stages.size(); ++s) {
    double prop = 0.0;
    for (const sim::TankSpec& t : g.stages[s].tanks) {
      prop += t.propellant;
    }
    double thrust_vac = 0.0;
    int n = 0;
    for (const sim::EngineSpec& e : g.engines) {
      if (e.stage == static_cast<int>(s) && e.control == sim::Control::None) {
        thrust_vac += e.thrust_vac;
        ++n;
      }
    }
    std::printf("  stage %zu (%s): dry %.0f kg, propellant %.0f kg, %d main engine(s), %.0f kN vacuum\n", s + 1U, g.stages[s].name.c_str(), g.stages[s].dry_mass, prop, n, thrust_vac / 1000.0);
  }
  // the static stability: where the centre of pressure is against the centre of gravity, in calibres (diameters): positive behind it is stable, negative ahead of it unstable
  for (const double mach : {0.3, 2.0}) {
    double cn = 0.0;
    double xcp = 0.0;
    double ca = 0.0;
    double s_ref = 0.0;
    veh.aero_summary(mach, cn, xcp, ca, s_ref);
    const double d_ref = std::sqrt(4.0 * s_ref / sim::kPi);
    if (std::fabs(cn) < 1e-9) {
      std::printf("  aerodynamics at Mach %.1f: no normal force (a body with no lift: nothing to be stable or unstable about), axial coefficient %.2f\n", mach, ca);
      continue;
    }
    std::printf("  aerodynamics at Mach %.1f: normal-force slope %.2f per radian, axial coefficient %.2f, centre of pressure %.2f m from the aft end, %.2f calibres %s the centre of gravity (%s)\n", mach, cn, ca, xcp,
                std::fabs(xcp - mp.x_cg) / d_ref, xcp < mp.x_cg ? "behind" : "ahead of", xcp < mp.x_cg ? "stable" : "unstable");
  }
  // the rated thrust of the first stage's main engines at sea level (an engine with a rise time starts cold, so the vehicle's thrust at T-zero would read zero)
  double sea_level = 0.0;
  for (const sim::EngineSpec& e : g.engines) {
    if (e.stage == 0 && e.control == sim::Control::None) {
      sea_level += std::fmax(0.0, e.thrust_vac - (101325.0 * e.exit_area));
    }
  }
  std::printf("  first-stage thrust at sea level %.1f kN, thrust-to-weight %.2f\n", sea_level / 1000.0, sea_level / (veh.mass() * sim::kG0));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argv[1][0] == '-') {
    std::fprintf(stderr, "usage: tfc_fly VEHICLE.json|reference [--frames N] [--pad FRAMES] [--sensors platform|vehicle] [--no-accel] [--csv FILE] [--every N] [--check] [--dump FILE]\n");
    return 2;
  }
  uint32_t frames = 0U;
  uint32_t pad = 0U;
  bool vehicle_true = false;
  bool no_accel = false;
  bool check = false;
  uint32_t every = 10U;
  std::string csv;
  std::string dump;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--frames" && i + 1 < argc) {
      frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--pad" && i + 1 < argc) {
      pad = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--sensors" && i + 1 < argc) {
      vehicle_true = std::string(argv[++i]) == "vehicle";
    } else if (a == "--no-accel") {
      no_accel = true;
    } else if (a == "--csv" && i + 1 < argc) {
      csv = argv[++i];
    } else if (a == "--every" && i + 1 < argc) {
      every = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--check") {
      check = true;
    } else if (a == "--dump" && i + 1 < argc) {
      dump = argv[++i];
    } else {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    }
  }
  sim::VehicleFile vf;
  std::vector<std::string> errors;
  if (std::string(argv[1]) == "reference") {
    vf.name = "reference";
    vf.description = "The reference vehicle of docs/design/VEHICLE_SIM.md: five engines, 30 t, a 100 s gravity-turn ascent through max-Q. The other files in vehicles/ are not this one.";
  } else if (!sim::load_vehicle_file(argv[1], vf, errors)) {
    for (const std::string& e : errors) {
      std::fprintf(stderr, "%s\n", e.c_str());
    }
    return 1;
  }
  if (!dump.empty()) {
    std::FILE* f = std::fopen(dump.c_str(), "w");
    if (f == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", dump.c_str());
      return 1;
    }
    std::fputs(sim::write_vehicle(vf).c_str(), f);
    std::fclose(f);
    return 0;
  }
  describe(vf);
  if (check) {
    return 0;
  }
  sim::Loop lp;
  lp.cfg.params = vf.params;
  lp.cfg.design = vf.params;
  lp.cfg.scenario = vf.scenario;
  lp.cfg.plan = vf.plan;
  lp.cfg.vehicle_true = vehicle_true;
  lp.estimator.use_accel = !no_accel;
  lp.pad_frames = pad;
  lp.frames = frames != 0U ? frames : static_cast<uint32_t>(vf.plan.trajectory.t_end / 0.01);
  std::vector<sim::TraceRow> trace;
  lp.trace = &trace;
  lp.trace_every = every;
  const sim::Result r = sim::run(lp);

  double max_q = 0.0;
  double t_max_q = 0.0;
  double max_alt = 0.0;
  double max_mach = 0.0;
  double max_err = 0.0;
  for (const sim::TraceRow& row : trace) {
    if (row.dynamic_pressure > max_q) {
      max_q = row.dynamic_pressure;
      t_max_q = row.t;
    }
    max_alt = std::fmax(max_alt, row.altitude);
    max_mach = std::fmax(max_mach, row.mach);
    if (row.t > 3.0) {
      max_err = std::fmax(max_err, std::fmax(std::fabs(row.err_y_deg), std::fabs(row.err_x_deg)));
    }
  }
  std::printf("\nFlight of %.1f s with %s sensors%s:\n", lp.frames * 0.01, vehicle_true ? "the vehicle's own" : "the platform's", no_accel ? " (accelerometer correction off)" : "");
  if (!trace.empty()) {
    std::printf("  at the end: altitude %.1f km, speed %.0f m/s, mass %.0f kg\n", trace.back().altitude / 1000.0, trace.back().speed, trace.back().mass);
  }
  std::printf("  max-Q %.1f kPa at %.1f s; highest Mach %.2f; highest altitude %.1f km\n", max_q / 1000.0, t_max_q, max_mach, max_alt / 1000.0);
  std::printf("  attitude error against the program: worst %.2f deg after 3 s, rms %.3f deg; platform saturated in %u frames\n", max_err, r.rms_deg, r.platform_saturated);
  std::printf("  lift-off at frame %s; %s; ACT safe frames %u; finite %s\n", r.liftoff_frame == 0xFFFFFFFFU ? "never" : std::to_string(r.liftoff_frame).c_str(), r.crashed ? "THE VEHICLE WAS DESTROYED" : "not destroyed",
              r.safe_frames, r.finite ? "yes" : "NO");
  unsigned last_active = trace.empty() ? 0U : trace.front().stages_active;
  unsigned last_ignited = trace.empty() ? 0U : trace.front().stages_ignited;
  for (const sim::TraceRow& row : trace) {
    for (unsigned s = 0; s < sim::kMaxStages; ++s) {
      if (((row.stages_ignited >> s) & 1U) != 0U && ((last_ignited >> s) & 1U) == 0U) {
        std::printf("  t = %6.1f s: stage %u ignited\n", row.t, s + 1U);
      }
      if (((row.stages_active >> s) & 1U) == 0U && ((last_active >> s) & 1U) != 0U) {
        std::printf("  t = %6.1f s: stage %u separated\n", row.t, s + 1U);
      }
    }
    last_active = row.stages_active;
    last_ignited = row.stages_ignited;
  }
  if (!csv.empty()) {
    std::FILE* f = std::fopen(csv.c_str(), "w");
    if (f == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", csv.c_str());
      return 1;
    }
    std::fprintf(f, "t_s,altitude_m,speed_ms,mach,dynamic_pressure_pa,mass_kg,thrust_n,tilt_pitch_deg,tilt_yaw_deg,err_pitch_deg,err_yaw_deg,cmd_pitch_deg,cmd_yaw_deg,stages_active,stages_ignited,engines_on,act_mode\n");
    for (const sim::TraceRow& row : trace) {
      std::fprintf(f, "%.2f,%.3f,%.3f,%.4f,%.2f,%.2f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%d,%d\n", row.t, row.altitude, row.speed, row.mach, row.dynamic_pressure, row.mass, row.thrust, row.tilt_y_deg,
                   row.tilt_x_deg, row.err_y_deg, row.err_x_deg, row.cmd_pitch_deg, row.cmd_yaw_deg, row.stages_active, row.stages_ignited, row.engines_on, row.act_mode);
    }
    std::fclose(f);
    std::printf("  wrote %zu rows to %s\n", trace.size(), csv.c_str());
  }
  const bool ok = r.finite && !r.crashed && r.liftoff_frame != 0xFFFFFFFFU && r.safe_frames == 0U;
  std::printf("  verdict: %s\n", ok ? "flown (no crash, no Safe, finite)" : "NOT a clean flight (see above)");
  return ok ? 0 : 1;
}
