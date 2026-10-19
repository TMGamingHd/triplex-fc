// SPDX-License-Identifier: MIT
// The analysis of a vehicle's propulsion (docs/design/PROPULSION.md): what the engines give at an altitude, what an ideal nozzle of the chamber pressure and area ratio the file names would give
// (and whether the file's thrust, exit area and specific impulse agree with it), how strongly the gimbal turns the vehicle at a chosen point of the burn (pitch and yaw, and roll by the
// differential gimbal), what an engine out does to the balance of the cluster and whether the gimbal can trim it, and how fast the gimbal's servo can move.
//
// It reads the vehicle as the plant has it (`Vehicle6::spec()` and its mass properties) and computes in closed form. The plant is the reference: the tests fly the same cluster and compare.
//
// The nozzle is the textbook one: a calorically perfect gas of ratio of specific heats gamma expanded isentropically from the chamber pressure to the exit by the area ratio, so that
//     A/A* = (1/M) [ (2/(g+1)) (1 + (g-1)/2 M^2) ]^((g+1)/(2(g-1)))        pe/pc = (1 + (g-1)/2 M^2)^(-g/(g-1))
//     Cf   = sqrt( 2 g^2/(g-1) (2/(g+1))^((g+1)/(g-1)) (1 - (pe/pc)^((g-1)/g)) ) + (pe - pa)/pc A/A*
// and thrust = Cf pc A*. A real engine has losses (the divergence of the flow, the boundary layer, the chemistry that is not frozen): its thrust is a few per cent below the ideal at the same area
// ratio, which is why `check_nozzle` reports the file's thrust against the ideal rather than asking for equality. The chamber pressure falls with the throttle and the exit pressure with it
// (pe is proportional to pc at a fixed area ratio): an engine throttled down at sea level runs over-expanded, and below about 0.35 of the ambient pressure at the exit (the Summerfield criterion,
// from memory, for a bell nozzle) the flow separates from the wall, the thrust becomes unsteady and the side load can exceed what the gimbal and the structure were sized for.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "atmosphere.hpp"
#include "spec.hpp"
#include "vehicle6.hpp"

namespace sim::propulsion {

constexpr double kSeaLevelPa = 101325.0;
constexpr double kSummerfield = 0.35;   // pe/pa below which a bell nozzle separates (a rule of thumb; the engine's own test data replace it)

// ---- the ideal nozzle ----

inline double area_ratio_of_mach(double g, double m) {
  return (1.0 / m) * std::pow((2.0 / (g + 1.0)) * (1.0 + (0.5 * (g - 1.0) * m * m)), (g + 1.0) / (2.0 * (g - 1.0)));
}

// The supersonic exit Mach number for an area ratio (bisection: the function rises with M above 1).
inline double exit_mach(double g, double eps) {
  double lo = 1.0;
  double hi = 40.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    (area_ratio_of_mach(g, mid) < eps ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

inline double exit_pressure_ratio(double g, double m) { return std::pow(1.0 + (0.5 * (g - 1.0) * m * m), -g / (g - 1.0)); }

// The thrust coefficient at the ambient pressure `pa_over_pc` (0 for vacuum).
inline double thrust_coefficient(double g, double eps, double pa_over_pc) {
  const double pe = exit_pressure_ratio(g, exit_mach(g, eps));
  const double vdot = std::sqrt(((2.0 * g * g) / (g - 1.0)) * std::pow(2.0 / (g + 1.0), (g + 1.0) / (g - 1.0)) * (1.0 - std::pow(pe, (g - 1.0) / g)));
  return vdot + ((pe - pa_over_pc) * eps);
}

struct NozzleReport {
  bool given = false;            // the file names the chamber pressure, the area ratio and gamma
  double pc = 0.0, eps = 0.0, gamma = 0.0;
  double mach_exit = 0.0;
  double pe_full = 0.0;          // Pa, the exit pressure at full throttle
  double cf_vac = 0.0;           // the ideal thrust coefficient in vacuum and at sea level at full throttle
  double cf_sl = 0.0;
  double throat_area = 0.0;      // m^2, the throat that gives the file's vacuum thrust at the ideal coefficient and the chamber pressure
  double exit_area_ideal = 0.0;  // that throat times the area ratio
  double exit_area_file = 0.0;
  double area_mismatch = 0.0;    // file / ideal - 1
  double efficiency_if_ideal_area = 0.0;   // the file's vacuum thrust over the ideal thrust at the file's own exit area (thrust coefficient efficiency, with the throat the file's area ratio gives)
  double sl_ratio_ideal = 0.0;   // F_sl / F_vac of the ideal nozzle at the file's throat ...
  double sl_ratio_file = 0.0;    // ... and of the file ((F_vac - pa Ae) / F_vac)
  double separation_throttle = 0.0;   // at sea level, the throttle below which pe/pa < kSummerfield (pe falls with the throttle): above 1, it separates even at full throttle
};

inline NozzleReport check_nozzle(const EngineSpec& e, double summerfield = kSummerfield) {
  NozzleReport r;
  r.exit_area_file = e.exit_area;
  if (!(e.chamber_pressure_pa > 0.0) || !(e.area_ratio > 1.0) || !(e.gamma > 1.0)) {
    return r;
  }
  r.given = true;
  r.pc = e.chamber_pressure_pa;
  r.eps = e.area_ratio;
  r.gamma = e.gamma;
  r.mach_exit = exit_mach(r.gamma, r.eps);
  r.pe_full = exit_pressure_ratio(r.gamma, r.mach_exit) * r.pc;
  r.cf_vac = thrust_coefficient(r.gamma, r.eps, 0.0);
  r.cf_sl = thrust_coefficient(r.gamma, r.eps, kSeaLevelPa / r.pc);
  r.throat_area = e.thrust_vac / (r.cf_vac * r.pc);
  r.exit_area_ideal = r.throat_area * r.eps;
  r.area_mismatch = r.exit_area_ideal > 0.0 ? (e.exit_area / r.exit_area_ideal) - 1.0 : 0.0;
  // the file's own throat, if its exit area and area ratio are taken as true: the thrust the ideal nozzle would give there, against the file's
  const double at_file = e.exit_area / r.eps;
  const double f_ideal_at_file = r.cf_vac * r.pc * at_file;
  r.efficiency_if_ideal_area = f_ideal_at_file > 0.0 ? e.thrust_vac / f_ideal_at_file : 0.0;
  r.sl_ratio_ideal = r.cf_sl / r.cf_vac;   // (the coefficient in vacuum is positive for any area ratio above 1 and gamma above 1)
  r.sl_ratio_file = e.thrust_vac > 0.0 ? (e.thrust_vac - (kSeaLevelPa * e.exit_area)) / e.thrust_vac : 0.0;
  r.separation_throttle = summerfield * kSeaLevelPa / r.pe_full;   // (the exit pressure is positive for the same reason)
  return r;
}

// ---- the cluster ----

inline double engine_rated_thrust(const EngineSpec& e) { return e.thrust_vac * e.thrust_factor; }

// The thrust of one engine at full throttle in the ambient pressure `pa` (the plant's formula: the vacuum thrust less the pressure on the exit area).
inline double engine_thrust(const EngineSpec& e, double pa, double throttle = 1.0) { return std::max(0.0, (throttle * engine_rated_thrust(e)) - (pa * e.exit_area)); }

inline double engine_mdot(const EngineSpec& e, double throttle = 1.0) {
  const double isp = e.isp_throttle_loss > 0.0 ? e.isp_vac * (1.0 - (e.isp_throttle_loss * (1.0 - std::min(throttle, 1.0)))) : e.isp_vac;
  return throttle * engine_rated_thrust(e) / (isp * 9.80665);
}

// The main engines (not thrusters) of a stage.
inline std::vector<std::size_t> main_engines(const VehicleSpec& g, std::size_t stage) {
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < g.engines.size(); ++i) {
    if (static_cast<std::size_t>(g.engines[i].stage) == stage && g.engines[i].control == Control::None) {
      out.push_back(i);
    }
  }
  return out;
}

struct Cluster {
  unsigned n = 0, n_gimbal = 0, n_fixed = 0;
  double f_vac = 0.0, f_sl = 0.0, mdot = 0.0, isp_vac = 0.0, isp_sl = 0.0;
};

inline Cluster cluster_of(const VehicleSpec& g, std::size_t stage, double pa) {
  Cluster c;
  for (const std::size_t i : main_engines(g, stage)) {
    const EngineSpec& e = g.engines[i];
    ++c.n;
    (e.gimbal ? c.n_gimbal : c.n_fixed) += 1U;
    c.f_vac += engine_thrust(e, 0.0);
    c.f_sl += engine_thrust(e, pa);
    c.mdot += engine_mdot(e);
  }
  c.isp_vac = c.mdot > 0.0 ? c.f_vac / (c.mdot * 9.80665) : 0.0;
  c.isp_sl = c.mdot > 0.0 ? c.f_sl / (c.mdot * 9.80665) : 0.0;
  return c;
}

// ---- the gimbal's authority ----

// The travel, rate and lag of one engine's gimbal as the plant resolves them (the engine's own, the stage's, the vehicle's).
struct GimbalLimits {
  double limit_deg = 0.0, rate_dps = 0.0, lag_s = 0.0;
};
inline GimbalLimits gimbal_limits(const Params& p, std::size_t stage, const EngineSpec& e) {
  const StageSpec& st = p.spec.stages[stage];
  const double sl = st.gimbal_limit_deg >= 0.0 ? st.gimbal_limit_deg : p.gimbal_limit_deg;
  const double sr = st.gimbal_rate_dps >= 0.0 ? st.gimbal_rate_dps : p.gimbal_rate_dps;
  const double sg = st.gimbal_lag_s >= 0.0 ? st.gimbal_lag_s : p.gimbal_lag_s;
  return GimbalLimits{e.gimbal_limit_deg >= 0.0 ? e.gimbal_limit_deg : sl, e.gimbal_rate_dps >= 0.0 ? e.gimbal_rate_dps : sr, e.gimbal_lag_s >= 0.0 ? e.gimbal_lag_s : sg};
}

struct Authority {
  double fill = 0.0;                 // the stage's propellant as a fraction of its capacity
  double mass = 0.0, x_cg = 0.0, i_t = 0.0, i_x = 0.0;
  double thrust = 0.0;               // N of the whole cluster at the pressure asked
  double pitch_nm_per_deg = 0.0;     // the torque a degree of the pitch (or yaw) command gives: the sum over the engines that gimbal of thrust times the arm to the centre of gravity
  double pitch_accel_per_deg = 0.0;  // rad/s^2
  double pitch_accel_max = 0.0;      // rad/s^2 with every gimbal at its travel
  double roll_nm_per_deg = 0.0;      // the differential gimbal, per degree of the roll command
  double roll_accel_per_deg = 0.0;
  double roll_accel_max = 0.0;       // with the roll command large enough to take every engine to its travel
  double thruster_roll_nm = 0.0;     // the roll thrusters' torque with all of one sign firing
  double thruster_roll_accel = 0.0;
  double limit_deg = 0.0;            // the smallest travel of the engines that gimbal
};

// The authority of stage `stage` when it holds `fill` of its propellant, the stages above it full, the ones below it gone. `failed` lists engines that are out (their thrust is not counted).
inline Authority authority(const Vehicle6& v, const Params& p, std::size_t stage, double fill, double pa, const std::vector<std::size_t>& failed = {}) {
  const VehicleSpec& g = v.spec();
  Authority a;
  a.fill = fill;
  std::array<double, kMaxStages> prop{};
  for (std::size_t s = 0; s < g.stages.size(); ++s) {
    if (v.stage_on_vehicle(s)) {
      prop[s] = s == stage ? fill * v.stage_capacity(s) : (s > stage ? v.stage_capacity(s) : 0.0);
    }
  }
  const MassProps mp = v.mass_props_with(prop);
  a.mass = mp.mass;
  a.x_cg = mp.x_cg;
  a.i_t = mp.i_t;
  a.i_x = mp.i_x;
  a.limit_deg = 1.0e9;
  const auto is_out = [&failed](std::size_t i) { return std::find(failed.begin(), failed.end(), i) != failed.end(); };
  for (std::size_t i = 0; i < g.engines.size(); ++i) {
    const EngineSpec& e = g.engines[i];
    if (static_cast<std::size_t>(e.stage) != stage) {
      continue;
    }
    if (e.control == Control::None) {
      if (is_out(i)) {
        continue;
      }
      const double t = engine_thrust(e, pa);
      a.thrust += t;
      if (e.gimbal) {
        const double arm = mp.x_cg - e.pos.x;
        const double lim = gimbal_limits(p, stage, e).limit_deg;
        a.pitch_nm_per_deg += t * arm * kDeg2Rad;
        a.pitch_accel_max += t * arm * std::sin(lim * kDeg2Rad);
        a.limit_deg = std::min(a.limit_deg, lim);
        const double r = std::hypot(e.pos.y, e.pos.z);
        if (e.roll_gain > 0.0) {
          a.roll_nm_per_deg += t * r * e.roll_gain * kDeg2Rad;
          a.roll_accel_max += t * r * std::sin(lim * kDeg2Rad);
        }
      }
    } else if (e.control == Control::RollPlus) {
      a.thruster_roll_nm += std::fabs(e.pos.y * e.dir.z - e.pos.z * e.dir.y) * e.thrust_vac;
    }
  }
  if (a.limit_deg > 1.0e8) {
    a.limit_deg = 0.0;
  }
  a.pitch_accel_per_deg = a.pitch_nm_per_deg / a.i_t;
  a.pitch_accel_max /= a.i_t;
  a.roll_accel_per_deg = a.roll_nm_per_deg / a.i_x;
  a.roll_accel_max /= a.i_x;
  a.thruster_roll_accel = a.thruster_roll_nm / a.i_x;
  return a;
}

// ---- an engine out ----

struct EngineOut {
  std::vector<std::size_t> out;     // the engines that are out
  double thrust_loss = 0.0;         // fraction of the cluster's thrust
  double imbalance_nm = 0.0;        // the torque about the centre of gravity of the thrust that is left, from its offset from the axis (axial thrust at a lateral offset)
  double offset_m = 0.0;            // that torque over the thrust left: how far from the axis the thrust acts
  double trim_pitch_deg = 0.0;      // the gimbal deflections that cancel it with what gimbals of the rest
  double trim_yaw_deg = 0.0;
  double trim_fraction = 0.0;       // the larger of them over the travel: more than 1 and the cluster cannot be trimmed (the vehicle tumbles), more than about 0.5 and nothing is left to steer with
  double accel_left = 0.0;          // the angular acceleration authority left after the trim, rad/s^2 (pitch plane)
};

inline EngineOut engine_out(const Vehicle6& v, const Params& p, std::size_t stage, double fill, double pa, const std::vector<std::size_t>& out) {
  EngineOut r;
  r.out = out;
  const Authority whole = authority(v, p, stage, fill, pa);
  const Authority left = authority(v, p, stage, fill, pa, out);
  r.thrust_loss = whole.thrust > 0.0 ? 1.0 - (left.thrust / whole.thrust) : 0.0;
  const VehicleSpec& g = v.spec();
  double my = 0.0;
  double mz = 0.0;
  for (const std::size_t i : main_engines(g, stage)) {
    if (std::find(out.begin(), out.end(), i) != out.end()) {
      continue;
    }
    const double t = engine_thrust(g.engines[i], pa);
    my += g.engines[i].pos.z * t;     // tau = r x F, F along +X: tau_y = z F, tau_z = -y F
    mz -= g.engines[i].pos.y * t;
  }
  r.imbalance_nm = std::hypot(my, mz);
  r.offset_m = left.thrust > 0.0 ? r.imbalance_nm / left.thrust : 0.0;
  if (left.pitch_nm_per_deg > 0.0) {
    r.trim_pitch_deg = -mz / left.pitch_nm_per_deg;
    r.trim_yaw_deg = -my / left.pitch_nm_per_deg;   // (the same authority in the yaw plane: the engines are on a ring, and the torque of a deflection is thrust times the arm along X)
    r.trim_fraction = left.limit_deg > 0.0 ? std::max(std::fabs(r.trim_pitch_deg), std::fabs(r.trim_yaw_deg)) / left.limit_deg : 1.0e9;
  } else {
    r.trim_fraction = r.imbalance_nm > 1.0e-9 * whole.thrust ? 1.0e9 : 0.0;   // (no gimbal to trim with: nothing to do about any imbalance bigger than rounding)
  }
  r.accel_left = left.pitch_accel_max * std::max(0.0, 1.0 - r.trim_fraction);
  return r;
}

// ---- the gimbal's servo ----

struct ServoReport {
  double limit_deg = 0.0, rate_dps = 0.0, lag_s = 0.0;
  double lag_corner_hz = 0.0;       // the first-order lag's corner, 1 / (2 pi lag)
  double rate_full_hz = 0.0;        // the highest frequency of a sine of the full travel before the rate limit bites: rate / (2 pi travel)
  double rate_2deg_hz = 0.0;        // the same for a sine of 2 degrees
  double second_order_hz = 0.0;     // the `actuator` block's natural frequency, if the servo has dynamics
};

inline ServoReport servo_report(const Params& p, std::size_t stage, const EngineSpec& e) {
  ServoReport s;
  const GimbalLimits gl = gimbal_limits(p, stage, e);
  s.limit_deg = gl.limit_deg;
  s.rate_dps = gl.rate_dps;
  s.lag_s = gl.lag_s;
  s.lag_corner_hz = gl.lag_s > 0.0 ? 1.0 / (2.0 * kPi * gl.lag_s) : 0.0;
  s.rate_full_hz = gl.limit_deg > 0.0 ? gl.rate_dps / (2.0 * kPi * gl.limit_deg) : 0.0;
  s.rate_2deg_hz = gl.rate_dps / (2.0 * kPi * 2.0);
  s.second_order_hz = p.spec.actuator.order == 2 ? p.spec.actuator.natural_hz : 0.0;
  return s;
}

}  // namespace sim::propulsion
