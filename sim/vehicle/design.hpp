// SPDX-License-Identifier: MIT
// Trajectory and gain design for the 6-DOF vehicle (docs/design/VEHICLE_SIM.md section 7): the nominal ascent with the attitude forced to follow a
// pitch program, the program found by iterating until the angle of attack is small (a gravity turn), and the controller gains for each instant
// from the local divergence a(t) and control effectiveness b(t).
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "tfc/controller.hpp"
#include "vehicle6.hpp"

namespace sim {

struct NominalPoint {
  double t = 0.0;
  double theta_deg = 0.0;  // the pitch program: angle of the long axis from the pad vertical toward downrange
  double altitude = 0.0;
  double speed = 0.0;
  double mass = 0.0;
  double thrust = 0.0;
  double dynamic_pressure = 0.0;
  double mach = 0.0;
  double alpha_deg = 0.0;
  double gamma_deg = 0.0;  // flight-path angle from the vertical
  double a_div = 0.0;      // aerodynamic divergence, 1/s^2
  double b_ctl = 0.0;      // control effectiveness, (rad/s^2) per rad of gimbal
};

struct DesignConfig {
  double t_end = 100.0;
  double dt = 0.01;
  double vertical_s = 8.0;    // straight up for this long
  double kick_ramp_s = 4.0;   // then ramp to the kick angle over this long
  double kick_deg = 1.0;  // 1 degree gives max-Q of about 31 kPa at 65 s and 32 degrees of pitch at 100 s: inside the platform's range
  double follow_from_s = 16.0;  // then follow the flight-path angle
  double blend_s = 4.0;         // blending into it over this long
};

namespace detail {
// Force the attitude: the long axis along `axis` in the pad's downrange plane (crossrange component zero), rates zero.
inline void force_attitude(Vehicle6& v, double theta_rad) {
  State s = v.state();
  const V3 x_b{std::cos(theta_rad), std::sin(theta_rad), 0.0};  // long axis: up (+X) tilted toward downrange (+Y)
  const V3 y_b{-std::sin(theta_rad), std::cos(theta_rad), 0.0};
  const V3 z_b = cross(x_b, y_b);
  // rotation matrix columns are the body axes in inertial coordinates; build the quaternion
  const double m00 = x_b.x;
  const double m11 = y_b.y;
  const double m22 = z_b.z;
  const double tr = m00 + m11 + m22;
  Q4 q;
  if (tr > 0.0) {
    const double sq = std::sqrt(tr + 1.0) * 2.0;
    q = {0.25 * sq, (y_b.z - z_b.y) / sq, (z_b.x - x_b.z) / sq, (x_b.y - y_b.x) / sq};
  } else {
    q = Q4{};  // not reached for the pitch angles used here (|theta| < 90 degrees)
  }
  s.q = normalized(q);
  s.w = V3{};
  v.set_state(s);
}
}  // namespace detail

// The pitch program at time t, given the flight-path angle `gamma_rad` at that instant: straight up, a smooth kick, then (after a blend) the attitude
// follows the velocity vector, which is a gravity turn (zero angle of attack). It is causal: it needs only the state now, so the nominal run is one pass.
inline double program_deg(const DesignConfig& c, double t, double gamma_rad) {
  if (t < c.vertical_s) {
    return 0.0;
  }
  const double s = std::min(1.0, (t - c.vertical_s) / c.kick_ramp_s);
  const double kick = c.kick_deg * s * s * (3.0 - (2.0 * s));  // smoothstep
  if (t < c.follow_from_s) {
    return kick;
  }
  const double f = std::min(1.0, (t - c.follow_from_s) / c.blend_s);
  const double w = f * f * (3.0 - (2.0 * f));
  return ((1.0 - w) * kick) + (w * gamma_rad * kRad2Deg);
}

inline std::vector<NominalPoint> nominal_trajectory(const Params& p, const DesignConfig& c = DesignConfig{}) {
  std::vector<NominalPoint> out;
  Vehicle6 v(p);
  const int steps = static_cast<int>(c.t_end / c.dt);
  double gam = 0.0;
  for (int k = 0; k <= steps; ++k) {
    const double t = k * c.dt;
    const double theta = program_deg(c, t, gam) * kDeg2Rad;
    detail::force_attitude(v, theta);
    const Loads l = v.current_loads();
    const State& s = v.state();
    const double speed = norm(s.v);
    gam = speed > 1.0 ? std::acos(std::clamp(dot(normalized(s.v), V3{1.0, 0.0, 0.0}), -1.0, 1.0)) : 0.0;
    const MassProps mp = v.mass_props(s.m);
    NominalPoint np;
    np.t = t;
    np.theta_deg = theta * kRad2Deg;
    np.altitude = norm(s.r) - kEarthR;
    np.speed = speed;
    np.mass = s.m;
    np.thrust = l.thrust;
    np.dynamic_pressure = l.dynamic_pressure;
    np.mach = l.mach;
    np.alpha_deg = l.alpha * kRad2Deg;
    np.gamma_deg = gam * kRad2Deg;
    const double area = kPi * 0.25 * p.diameter * p.diameter;
    np.a_div = l.dynamic_pressure * area * p.c_n_alpha * (p.x_cp - mp.x_cg) / mp.i_t;
    np.b_ctl = l.thrust * mp.x_cg / mp.i_t;
    out.push_back(np);
    v.step(c.dt, 0.0, 0.0);
  }
  return out;
}

struct GainPoint {
  double t = 0.0;
  double kp = 0.0;  // gimbal degrees per degree of attitude error
  double kd = 0.0;  // gimbal degrees per degree/second of rate error
  double ki = 0.0;  // per second
};

// Gains for a closed loop of natural frequency wn and damping zeta: s^2 + b kd s + (b kp - a) = 0.
inline std::vector<GainPoint> gain_schedule(const std::vector<NominalPoint>& nominal, double every_s, double wn, double zeta, double ki_over_kp) {
  std::vector<GainPoint> g;
  if (nominal.empty()) {
    return g;
  }
  double next = 0.0;
  for (const NominalPoint& n : nominal) {
    if (n.t + 1e-9 < next) {
      continue;
    }
    next = n.t + every_s;
    GainPoint p;
    p.t = n.t;
    const double b = std::max(n.b_ctl, 1e-3);
    p.kp = ((wn * wn) + std::max(0.0, n.a_div)) / b;
    p.kd = 2.0 * zeta * wn / b;
    p.ki = ki_over_kp * p.kp;
    g.push_back(p);
  }
  return g;
}

// The tables the flight computers carry: the pitch program at sixteen points of the nominal trajectory, and the gains designed every `every_s` seconds
// (up to the sixteen the schedule holds). The firmware's generated tables (firmware/app/src/flight_tables.hpp) and the closed-loop tests both come from here.
struct FlightTables {
  tfc::Guidance guidance;
  tfc::GainSchedule gains;
};

inline FlightTables flight_tables(const Params& p, double every_s = 6.0, double wn = 2.5, double zeta = 0.8, double ki_over_kp = 0.2) {
  FlightTables t;
  const std::vector<NominalPoint> nominal = nominal_trajectory(p);
  for (unsigned k = 0; k < tfc::Guidance::kMaxPoints; ++k) {
    const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints - 1U);
    (void)t.guidance.add(1U, static_cast<uint32_t>(idx), static_cast<float>(nominal[idx].theta_deg));
  }
  for (const GainPoint& g : gain_schedule(nominal, every_s, wn, zeta, ki_over_kp)) {
    if (t.gains.size() < tfc::GainSchedule::kMaxPoints) {
      (void)t.gains.add(static_cast<uint32_t>(g.t / 0.01),
                        tfc::ControllerGains{static_cast<float>(g.kp), static_cast<float>(g.kd), static_cast<float>(g.ki)});
    }
  }
  return t;
}

}  // namespace sim
