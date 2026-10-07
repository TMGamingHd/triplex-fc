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
  // A vehicle that does not fly a gravity turn (a spacecraft that holds an attitude and slews, a vehicle with a chosen pitch history) gives the pitch program as a table instead: the
  // angle of the long axis from the pad vertical, toward downrange, against time, linear between the points and held after the last. Empty: the gravity turn above.
  std::vector<std::array<double, 2>> table;
};

// How the controller's gains are designed from the vehicle's local divergence a(t) and control effectiveness b(t) (docs/design/CONTROL_LOOP.md): the closed loop's natural frequency and
// damping, the integral gain as a fraction of the proportional, and how often a gain is designed. The flight computers carry 16 points, so `every_s` <= 0 spreads 16 over the flight.
struct GainDesign {
  double every_s = 6.0;    // <= 0: adaptive: the 16 points go where the gains change (below), within `tolerance`
  double wn = 2.5;
  double zeta = 0.8;
  double ki_over_kp = 0.2;
  double kp_max = 1.0e30;  // a ceiling on the proportional gain (degrees of command per degree of error)
  // The control effectiveness the gains are designed for is at least this (1/s^2 per radian of command). A vehicle whose effectiveness is tiny at lift-off (fins at low speed, thrusters of a
  // heavy spacecraft) would otherwise ask for gains of thousands that the effectiveness it later has would turn into a loop that rings at the actuator's limit.
  double b_min = 1.0e-3;
  double tolerance = 0.1;  // adaptive spacing: the largest relative error of a gain between two points of the schedule that is accepted (while points remain)
};

// The trajectory the tables are designed on and how the gains are designed from it.
struct Plan {
  DesignConfig trajectory;
  GainDesign gains;
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
  if (!c.table.empty()) {
    if (t <= c.table.front()[0]) {
      return c.table.front()[1];
    }
    for (std::size_t i = 1; i < c.table.size(); ++i) {
      if (t <= c.table[i][0]) {
        const double f = (t - c.table[i - 1][0]) / (c.table[i][0] - c.table[i - 1][0]);
        return c.table[i - 1][1] + (f * (c.table[i][1] - c.table[i - 1][1]));
      }
    }
    return c.table.back()[1];
  }
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
    NominalPoint np;
    np.t = t;
    np.theta_deg = theta * kRad2Deg;
    np.altitude = v.altitude();
    np.speed = speed;
    np.mass = s.m;
    np.thrust = l.thrust;
    np.dynamic_pressure = l.dynamic_pressure;
    np.mach = l.mach;
    np.alpha_deg = l.alpha * kRad2Deg;
    np.gamma_deg = gam * kRad2Deg;
    np.a_div = v.divergence();
    np.b_ctl = v.control_effectiveness();
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
inline std::vector<GainPoint> gain_schedule(const std::vector<NominalPoint>& nominal, double every_s, double wn, double zeta, double ki_over_kp, double kp_max = 1.0e30, double b_min = 1.0e-3) {
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
    const double b = std::max(n.b_ctl, b_min);
    p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);
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

// The gains at every point of the nominal flight, then the `max_points` that approximate them best by straight lines between points: start with the two ends and keep adding the
// sample whose gain is furthest (in relative terms) from the line, until the largest error is under `tolerance` or the points are used up. A gain that changes by a hundred times in
// four seconds gets its points there and not spread evenly over a flight it hardly changes in.
inline std::vector<GainPoint> gain_schedule_adaptive(const std::vector<NominalPoint>& nominal, std::size_t max_points, double wn, double zeta, double ki_over_kp, double kp_max, double b_min, double tolerance) {
  std::vector<GainPoint> all;
  all.reserve(nominal.size());
  for (const NominalPoint& n : nominal) {
    GainPoint p;
    p.t = n.t;
    const double b = std::max(n.b_ctl, b_min);
    p.kp = std::min(((wn * wn) + std::max(0.0, n.a_div)) / b, kp_max);
    p.kd = 2.0 * zeta * wn / b;
    p.ki = ki_over_kp * p.kp;
    all.push_back(p);
  }
  std::vector<GainPoint> out;
  if (all.empty()) {
    return out;
  }
  std::vector<std::size_t> idx{0U, all.size() - 1U};
  while (idx.size() < max_points) {
    double worst = tolerance;
    std::size_t worst_at = 0U;
    for (std::size_t s = 0; s + 1U < idx.size(); ++s) {
      const GainPoint& a = all[idx[s]];
      const GainPoint& z = all[idx[s + 1U]];
      for (std::size_t i = idx[s] + 1U; i < idx[s + 1U]; ++i) {
        const double f = (all[i].t - a.t) / (z.t - a.t);
        const double ekp = std::fabs((a.kp + (f * (z.kp - a.kp))) - all[i].kp) / std::max(all[i].kp, 1e-9);
        const double ekd = std::fabs((a.kd + (f * (z.kd - a.kd))) - all[i].kd) / std::max(all[i].kd, 1e-9);
        const double e = std::max(ekp, ekd);
        if (e > worst) {
          worst = e;
          worst_at = i;
        }
      }
    }
    if (worst_at == 0U) {
      break;
    }
    idx.insert(std::upper_bound(idx.begin(), idx.end(), worst_at), worst_at);
  }
  for (const std::size_t i : idx) {
    out.push_back(all[i]);
  }
  return out;
}

inline FlightTables flight_tables(const Params& p, const Plan& plan) {
  FlightTables t;
  const std::vector<NominalPoint> nominal = nominal_trajectory(p, plan.trajectory);
  for (unsigned k = 0; k < tfc::Guidance::kMaxPoints; ++k) {
    const std::size_t idx = (nominal.size() - 1U) * k / (tfc::Guidance::kMaxPoints - 1U);
    (void)t.guidance.add(1U, static_cast<uint32_t>(idx), static_cast<float>(nominal[idx].theta_deg));
  }
  const std::vector<GainPoint> points = plan.gains.every_s > 0.0
                                            ? gain_schedule(nominal, plan.gains.every_s, plan.gains.wn, plan.gains.zeta, plan.gains.ki_over_kp, plan.gains.kp_max, plan.gains.b_min)
                                            : gain_schedule_adaptive(nominal, tfc::GainSchedule::kMaxPoints, plan.gains.wn, plan.gains.zeta, plan.gains.ki_over_kp, plan.gains.kp_max, plan.gains.b_min,
                                                                     plan.gains.tolerance);
  for (const GainPoint& g : points) {
    if (t.gains.size() < tfc::GainSchedule::kMaxPoints) {
      (void)t.gains.add(static_cast<uint32_t>(g.t / 0.01),
                        tfc::ControllerGains{static_cast<float>(g.kp), static_cast<float>(g.kd), static_cast<float>(g.ki)});
    }
  }
  return t;
}

// The reference design: the 100 s ascent, a gain every 6 s (17 designed, 16 held: VEHICLE_SIM.md section 7).
inline FlightTables flight_tables(const Params& p, double every_s = 6.0, double wn = 2.5, double zeta = 0.8, double ki_over_kp = 0.2) {
  Plan plan;
  plan.gains.every_s = every_s;
  plan.gains.wn = wn;
  plan.gains.zeta = zeta;
  plan.gains.ki_over_kp = ki_over_kp;
  return flight_tables(p, plan);
}

}  // namespace sim
