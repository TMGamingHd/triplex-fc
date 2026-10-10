// SPDX-License-Identifier: MIT
// Guidance for a vehicle that comes back (docs/design/GNC.md section 6): where a burn leaves it falling (the impact point), how to steer it through the air on the way down, and how to land it.
//
//  * ballistic_impact: the point where the coasting vehicle (no air, point-mass gravity) would meet a sphere of a given radius, found in closed form from the conic its position and velocity define, with the
//    time of flight. The boost-back burn thrusts against the horizontal velocity until this point reaches the landing site; the glide is then steered to what the air takes away from it.
//  * descent_predict: the same with the air: a numerical integration (fixed step, point mass, exponential atmosphere, drag from a ballistic coefficient) down to the landing altitude. The glide's steering reads the
//    miss from it.
//  * PoweredDescent: the landing burn. The thrust acceleration is the one that stops the vehicle at the catch point: vertically, the constant deceleration that brings the speed to the arms' sink speed at the arms'
//    height (the "suicide burn": ignite as late as the engines can still stop it), horizontally the zero-effort-miss / zero-effort-velocity law (a quadratic in the time to go) that brings the position and the
//    velocity over the point to nothing together; the tilt of the thrust is limited, and the number of engines is chosen so that the thrust asked for lies between what the engines running can hold at their
//    lowest throttle and their highest.
//
// All arithmetic is + - * / and sqrt on doubles through dmath.hpp, with fixed-length loops. No heap, no exceptions, no RTTI.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/dmath.hpp"
#include "tfc/nav.hpp"

namespace tfc::descent {

using dm::Vec3;

struct Impact {
  bool valid = false;     // the conic reaches the sphere (it can fail to: an orbit that stays above it)
  Vec3 point{};           // where, on the sphere, centred at the planet's centre
  double tof = 0.0;       // s until then
};

// The point where the vehicle at r with velocity v would reach radius `radius` coming down, flying under gravity alone.
inline Impact ballistic_impact(Vec3 r, Vec3 v, double mu, double radius) noexcept {
  Impact out;
  const double rn = dm::norm(r);
  const double v2 = dm::dot(v, v);
  const double energy = (0.5 * v2) - (mu / rn);
  if (!(energy < 0.0) || !(rn > 0.0)) {
    return out;   // a hyperbolic path: no landing to speak of
  }
  const Vec3 h = dm::cross(r, v);
  const double h2 = dm::dot(h, h);
  if (!(h2 > 1.0e-6)) {   // straight up or down: the point under it
    out.valid = dm::dot(r, v) < 0.0 || rn > radius;
    out.point = (r / rn) * radius;
    const double vr = dm::dot(r, v) / rn;
    out.tof = vr < 0.0 ? (rn - radius) / dm::max_(-vr, 1.0) : 0.0;   // (rough: a free fall straight down from the speed it has)
    return out;
  }
  const double p = h2 / mu;
  const double e = dm::sqrt_(dm::max_(1.0 + (2.0 * energy * h2 / (mu * mu)), 0.0));
  const double a = -mu / (2.0 * energy);
  if (!(e > 1.0e-9)) {   // a circle: it never comes down to a lower radius
    return out;
  }
  const double cos_nu_s = ((p / radius) - 1.0) / e;
  if (cos_nu_s < -1.0 || cos_nu_s > 1.0) {
    return out;   // the sphere is above the perigee of this orbit, or below it with the perigee above it: no crossing
  }
  const double nu_s = -dm::acos_(cos_nu_s);   // the descending crossing, true anomaly in (-pi, 0] (equivalently (pi, 2 pi])
  // the vehicle's own true anomaly: from the eccentricity vector
  const Vec3 e_vec = ((r * (v2 - (mu / rn))) - (v * dm::dot(r, v))) / mu;
  const Vec3 e_hat = dm::unit(e_vec, dm::unit(r, Vec3{1.0, 0.0, 0.0}));
  const Vec3 h_hat = h / dm::sqrt_(h2);
  const Vec3 q_hat = dm::cross(h_hat, e_hat);
  const double nu0 = dm::atan2_(dm::dot(r, q_hat), dm::dot(r, e_hat));
  double dnu = nu_s - nu0;
  // the descending crossing is after the vehicle if the vehicle is still on its way down to it or going up: bring the difference into [0, 2 pi)
  dnu = dnu - (dm::kTwoPi * dm::floor_(dnu / dm::kTwoPi));
  // the position: the vehicle's direction turned about the angular momentum by dnu
  double s = 0.0;
  double c = 0.0;
  dm::sincos_(dnu, s, c);
  const Vec3 r_hat = r / rn;
  const Vec3 t_hat = dm::cross(h_hat, r_hat);
  out.point = ((r_hat * c) + (t_hat * s)) * radius;
  // the time of flight: eccentric anomaly E from nu, then Kepler's equation M = E - e sin E
  const auto ecc = [e](double nu) {
    return 2.0 * dm::atan2_(dm::sqrt_(1.0 - e) * dm::sin_(0.5 * nu), dm::sqrt_(1.0 + e) * dm::cos_(0.5 * nu));
  };
  const double e0 = ecc(nu0);
  const double e1 = ecc(nu0 + dnu);
  const double m0 = e0 - (e * dm::sin_(e0));
  const double m1 = e1 - (e * dm::sin_(e1));
  double dm_anom = m1 - m0;
  dm_anom = dm_anom - (dm::kTwoPi * dm::floor_(dm_anom / dm::kTwoPi));
  out.tof = dm_anom / dm::sqrt_(mu / (a * a * a));
  out.valid = true;
  return out;
}

// The air for the descent predictor: density rho0 e^(-h / H) and the ballistic coefficient beta = m / (Cd A) of the vehicle in the attitude it descends in (kg/m^2).
struct DragModel {
  double rho0 = 1.225;
  double scale_height = 7200.0;   // m (a fit to the lower atmosphere, better than the standard 8500 through the stratosphere)
  double beta = 6000.0;
};

// The place a vehicle would meet the sphere of radius `radius` flying down under gravity and drag from (r, v): steps of `dt` seconds of velocity Verlet with the drag deceleration -(rho |v| v) / (2 beta) on the velocity
// through the air (the planet does not turn in this frame). Returns the point reached (the last step is cut at the sphere) and the time; `valid` false if it did not get there in `max_steps`.
inline Impact descent_predict(Vec3 r, Vec3 v, const nav::Gravity& grav, const DragModel& drag, double radius, double dt, unsigned max_steps) noexcept {
  Impact out;
  Vec3 rr = r;
  Vec3 vv = v;
  double t = 0.0;
  for (unsigned i = 0; i < max_steps; ++i) {
    const auto accel = [&](Vec3 pos, Vec3 vel) {
      const double alt = dm::norm(pos) - grav.radius;
      const double rho = drag.rho0 * dm::exp_(-dm::max_(alt, 0.0) / drag.scale_height);
      return grav.at(pos) - (vel * (0.5 * rho * dm::norm(vel) / drag.beta));
    };
    const Vec3 a0 = accel(rr, vv);
    const Vec3 r1 = rr + (vv * dt) + (a0 * (0.5 * dt * dt));
    const Vec3 v_pred = vv + (a0 * dt);
    const Vec3 a1 = accel(r1, v_pred);
    const Vec3 v1 = vv + ((a0 + a1) * (0.5 * dt));
    if (dm::norm(r1) <= radius) {   // crossed the sphere inside this step: interpolate to it
      const double r_a = dm::norm(rr);
      const double r_b = dm::norm(r1);
      const double f = (r_a - radius) / dm::max_(r_a - r_b, 1.0e-12);
      out.point = rr + ((r1 - rr) * f);
      out.tof = t + (f * dt);
      out.valid = true;
      return out;
    }
    rr = r1;
    vv = v1;
    t += dt;
  }
  return out;
}

// ---- the landing burn ----

struct LandingTarget {
  Vec3 point{};            // the catch point (or the touchdown point), navigation frame
  double sink_ms = 0.5;    // the sink speed to arrive with
  double tilt_max_deg = 12.0;      // the most the thrust may lean from the vertical, far from the ground
  double tilt_final_deg = 2.5;     // ... and near the ground
  double final_height_m = 25.0;    // the height over which the limit comes down from the first to the second
  double decel_plan = 15.0;        // m/s^2: the net deceleration (beyond holding up the weight) the burn is planned on: it is lit when the stopping distance at this deceleration reaches the height
};

// The engines the landing burn may use: the numbers of engines it may run (in increasing order), each engine's full thrust and the lowest throttle it holds. A burn starts with as many as it needs to stop in time and
// drops to fewer as the thrust it asks for falls.
constexpr unsigned kEngineOptions = 4U;
struct Engines {
  double thrust_each = 2.2e6;      // N, one engine at full throttle (the running ones add up)
  double min_throttle = 0.4;
  std::array<unsigned, kEngineOptions> counts{1U, 2U, 3U, 3U};
  unsigned options = 3U;           // how many of `counts` are in use
};

struct DescentOut {
  Vec3 direction{0.0, 0.0, 1.0};   // unit vector of the thrust the vehicle wants
  double throttle = 0.0;           // of the engines that are on (0 to 1)
  unsigned engines = 3U;           // how many to run
  unsigned option = 0U;            // which of the options of `Engines` that is
  double accel_cmd = 0.0;          // m/s^2, the thrust acceleration asked for
  double height = 0.0;             // m above the target point
  double time_to_go = 0.0;
  bool ignite = false;             // the burn should be running: the stopping distance has reached the height
  Vec3 miss{};                     // the horizontal position error at the target point, projected from now at the present velocity (zero effort): for telemetry
};

class PoweredDescent {
 public:
  PoweredDescent() noexcept = default;

  // The stopping height: how far above the target point the vehicle must be to be stopped at the arrival speed `sink` by the planned deceleration `decel` (m/s^2, beyond the weight).
  [[nodiscard]] static double stopping_height(double v_down, double decel, double sink) noexcept {
    const double v2 = dm::max_((v_down * v_down) - (sink * sink), 0.0);
    return v2 / (2.0 * dm::max_(decel, 0.1));
  }

  // One cycle. `ignition_margin` is the multiple of the stopping height at which to start (1.15 leaves a reserve for the start-up and the lateral correction).
  [[nodiscard]] DescentOut update(Vec3 r, Vec3 v, double mass, const LandingTarget& tgt, const Engines& eng, const nav::Gravity& grav, double ignition_margin, bool burning) noexcept {
    DescentOut out;
    const Vec3 up = dm::unit(tgt.point, Vec3{1.0, 0.0, 0.0});
    const Vec3 rel = r - tgt.point;
    const double h = dm::dot(rel, up);
    const Vec3 e_h = dm::perp(rel, up);   // horizontal position error
    const double v_z = dm::dot(v, up);
    const Vec3 v_h = dm::perp(v, up);
    const double g_local = dm::norm(grav.at(r));
    const double v_down = dm::max_(-v_z, 0.0);
    out.height = h;
    const double h_stop = stopping_height(v_down, tgt.decel_plan, tgt.sink_ms);
    out.ignite = burning || h <= (h_stop * ignition_margin) + 1.0;
    out.miss = e_h;
    if (!out.ignite) {
      out.direction = dm::unit(v * -1.0, up);   // not yet: the attitude to be burning in
      return out;
    }
    // vertical: the deceleration that brings the speed to the arrival sink at zero height, holding up the weight on top of it
    const double h_rem = dm::max_(h, 0.05);
    const double decel = dm::max_(((v_down * v_down) - (tgt.sink_ms * tgt.sink_ms)) / (2.0 * h_rem), 0.0);
    const double a_up = g_local + decel;
    // horizontal: ZEM / ZEV over the time to go (the time the vertical profile takes: 2 h / (v_down + sink), at least a second and a half)
    const double t_go = dm::clamp_(2.0 * h_rem / (v_down + tgt.sink_ms), 1.5, 60.0);
    out.time_to_go = t_go;
    Vec3 a_h = (e_h * (-6.0 / (t_go * t_go))) - (v_h * (4.0 / t_go));
    // the tilt limit, coming down towards the final value as the ground nears
    const double blend = dm::clamp_(h / dm::max_(tgt.final_height_m, 1.0), 0.0, 1.0);
    const double tilt_max = (tgt.tilt_final_deg + (blend * (tgt.tilt_max_deg - tgt.tilt_final_deg))) * dm::kDegToRad;
    const double a_h_max = a_up * dm::sin_(tilt_max) / dm::cos_(tilt_max);
    const double a_h_mag = dm::norm(a_h);
    if (a_h_mag > a_h_max && a_h_mag > 0.0) {
      a_h = a_h * (a_h_max / a_h_mag);
    }
    const Vec3 a_vec = (up * a_up) + a_h;
    const double a_mag = dm::norm(a_vec);
    out.accel_cmd = a_mag;
    out.direction = a_vec / a_mag;
    // engines: the fewest that can deliver the thrust asked for at their highest throttle, held while the thrust asked for is not well under what the next fewer could give
    const double thrust = mass * a_mag;
    unsigned opt = eng.options > 0U ? eng.options - 1U : 0U;
    for (unsigned k = 0; k < eng.options; ++k) {
      if (thrust <= eng.thrust_each * static_cast<double>(eng.counts[k])) {
        opt = k;
        break;
      }
    }
    if (have_option_ && opt < option_ && thrust > 0.9 * eng.thrust_each * static_cast<double>(eng.counts[opt])) {
      opt = option_;   // do not drop an engine until the thrust asked for is well under what the smaller number can hold
    }
    if (have_option_ && opt > option_ && thrust < eng.thrust_each * static_cast<double>(eng.counts[option_])) {
      opt = option_;   // and do not add one while the running ones can still give it
    }
    option_ = opt;
    have_option_ = true;
    out.option = opt;
    out.engines = eng.counts[opt];
    out.throttle = dm::clamp_(thrust / (eng.thrust_each * static_cast<double>(out.engines)), eng.min_throttle, 1.0);
    return out;
  }

  void reset() noexcept { have_option_ = false; }
  [[nodiscard]] bool running() const noexcept { return have_option_; }
  [[nodiscard]] unsigned option() const noexcept { return option_; }

 private:
  unsigned option_ = 0U;
  bool have_option_ = false;
};

}  // namespace tfc::descent
