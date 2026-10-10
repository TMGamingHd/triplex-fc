// SPDX-License-Identifier: MIT
// Attitude control through any attitude: the error between the attitude the estimator holds and the one the guidance wants is a rotation, taken as a quaternion, so a vehicle can turn through a half circle
// (a booster flipping to fly its engines first, a ship going from a vertical ascent to a belly-first entry) where the two tilt angles of controller.hpp, which describe a vehicle within about 80 degrees of the
// vertical, cannot (docs/design/GNC.md section 5).
//
// The control law is a PD with an integral on each body axis, written for the three torque axes of the vehicle: roll about the body's x axis (the long axis), "yaw" about its y axis and "pitch" about its z axis,
// the same planes the gimbal commands of the rest of the system are named for (a positive pitch command turns the vehicle positively about z, a positive yaw command about y). The output of each axis is a command in
// degrees of the vehicle's own effector for that axis (the gimbal's angle, a surface's, a thruster's duty), through the gains of the current phase, which are designed from the effectiveness of that effector
// (docs/design/CONTROL_LOOP.md): the loop is the same second-order system whatever moves the vehicle.
//
// The attitude is a unit quaternion q that takes body coordinates to navigation coordinates. The error quaternion q_e = conj(q) * q_ref is the rotation, in the body frame, that would bring the vehicle to the reference; the rotation vector is the axis times twice the angle whose cosine is the scalar part (its angle taken from the scalar part and the sign so that the rotation is the short way round).
// All arithmetic is + - * / and sqrt on doubles (dmath.hpp). No heap, no exceptions, no RTTI.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/dmath.hpp"
#include "tfc/estimator.hpp"

namespace tfc::att {

using dm::Quat;
using dm::Vec3;

// The gains of one axis: command degrees per degree of attitude error, per degree/second of rate error, and per degree-second of integrated error.
struct AxisGains {
  float kp = 2.0F;
  float kd = 1.5F;
  float ki = 0.2F;
};
struct Gains3 {
  AxisGains roll;
  AxisGains yaw;
  AxisGains pitch;
};

struct Limits {
  float command_deg = 8.0F;      // the largest command on an axis
  float integrator_deg = 6.0F;   // the largest the integral term contributes
  float slew_deg_per_frame = 0.6F;
};

// The command of the three axes, in degrees of the effector of each.
struct Demand {
  double roll = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
};

// The attitude of the vehicle as a quaternion body -> navigation frame, from the estimator's (sensor frame relative to its level reference): the long axis, which is the sensor frame's z, is the body's x; the body's
// y and z are the sensor frame's x and y; and the level frame's axes are the navigation frame's (Y, Z, X). Written as the turn of the three body axes into the navigation frame.
inline Quat body_to_nav(const std::array<float, 4>& q_level_sensor) noexcept {
  const Quat q = dm::normalized(Quat{static_cast<double>(q_level_sensor[0]), static_cast<double>(q_level_sensor[1]), static_cast<double>(q_level_sensor[2]), static_cast<double>(q_level_sensor[3])});
  const auto to_nav = [&q](Vec3 sensor) {
    const Vec3 level = dm::rotate(q, sensor);
    return Vec3{level.z, level.x, level.y};
  };
  const Vec3 bx = to_nav(Vec3{0.0, 0.0, 1.0});   // body x is sensor z
  const Vec3 by = to_nav(Vec3{1.0, 0.0, 0.0});   // body y is sensor x
  const Vec3 bz = to_nav(Vec3{0.0, 1.0, 0.0});   // body z is sensor y
  return dm::quat_of_columns(bx, by, bz);
}

// The inverse of the conversion above: the quaternion the estimator would hold (sensor frame relative to its level reference) for a vehicle at attitude q_body_to_nav. For tools and tests that put a
// computer in the state of a vehicle already in flight.
inline Quat estimator_quaternion(Quat q_body_to_nav) noexcept {
  const auto level_image = [&q_body_to_nav](Vec3 body) {
    const Vec3 n = dm::rotate(q_body_to_nav, body);
    return Vec3{n.y, n.z, n.x};   // the navigation frame's (X, Y, Z) are the level frame's (z, x, y)
  };
  const Vec3 cx = level_image(Vec3{0.0, 1.0, 0.0});   // the sensor x axis is the body y
  const Vec3 cy = level_image(Vec3{0.0, 0.0, 1.0});   // the sensor y axis is the body z
  const Vec3 cz = level_image(Vec3{1.0, 0.0, 0.0});   // the sensor z axis is the body x
  return dm::quat_of_columns(cx, cy, cz);
}

// The body rates (rad/s, body axes) from the estimator's sensor-frame rates in degrees per second.
inline Vec3 body_rates_rad(const std::array<float, 3>& sensor_dps) noexcept {
  return Vec3{static_cast<double>(sensor_dps[2]), static_cast<double>(sensor_dps[0]), static_cast<double>(sensor_dps[1])} * dm::kDegToRad;
}

// The reference attitude whose body x axis lies along `x_axis` and whose body z axis is as close to `z_hint` as it can be (perpendicular to x): the roll is chosen by the hint.
inline Quat attitude_from_axes(Vec3 x_axis, Vec3 z_hint) noexcept {
  const Vec3 x = dm::unit(x_axis, Vec3{1.0, 0.0, 0.0});
  Vec3 z = dm::unit(dm::perp(z_hint, x), Vec3{0.0, 0.0, 0.0});
  if (dm::norm(z) < 0.5) {   // the hint lies along the axis: any perpendicular will do
    z = dm::unit(dm::cross(x, dm::fabs_(x.z) < 0.9 ? Vec3{0.0, 0.0, 1.0} : Vec3{0.0, 1.0, 0.0}), Vec3{0.0, 0.0, 1.0});
  }
  const Vec3 y = dm::cross(z, x);
  return dm::quat_of_columns(x, y, z);
}

// The rotation vector (rad, body axes) from the attitude q to the reference q_ref, the short way round.
inline Vec3 error_vector(Quat q, Quat q_ref) noexcept {
  const Quat e = dm::conj(q) * q_ref;
  const Vec3 v{e.x, e.y, e.z};
  const double n = dm::norm(v);
  const double w = e.w < 0.0 ? -e.w : e.w;   // the short way round: the sign of the quaternion is free, so take the scalar part positive
  const double sgn = e.w < 0.0 ? -1.0 : 1.0;
  if (n < 1.0e-9) {
    return v * (2.0 * sgn);
  }
  return v * (sgn * 2.0 * dm::atan2_(n, w) / n);
}

// The angular rate (rad/s, body axes) the reference attitude turns at between two frames dt apart: twice the vector part of conj(q_prev) * q_next over dt.
inline Vec3 reference_rate(Quat q_prev, Quat q_next, double dt) noexcept {
  if (!(dt > 0.0)) {
    return Vec3{};
  }
  const Vec3 e = error_vector(q_prev, q_next);
  return e / dt;
}

class Controller3 {
 public:
  Controller3() noexcept = default;

  // One step. `q` and `w` are the estimated attitude and body rate (rad/s), `q_ref` and `w_ref` the reference attitude and rate (body axes). With `valid` false (no trustworthy gyro) the last command is held and the integrators stand still.
  [[nodiscard]] Demand step(Quat q, Vec3 w, Quat q_ref, Vec3 w_ref, const Gains3& g, const Limits& lim, double dt, bool valid) noexcept {
    if (!valid) {
      ++holds_;
      return out_;
    }
    const Vec3 e = error_vector(q, q_ref) * dm::kRadToDeg;      // degrees
    const Vec3 de = (w_ref - w) * dm::kRadToDeg;                // degrees per second
    err_deg_ = e;
    out_.roll = axis(0U, e.x, de.x, g.roll, lim, dt);
    out_.yaw = axis(1U, e.y, de.y, g.yaw, lim, dt);
    out_.pitch = axis(2U, e.z, de.z, g.pitch, lim, dt);
    return out_;
  }

  [[nodiscard]] Vec3 error_deg() const noexcept { return err_deg_; }
  [[nodiscard]] Demand output() const noexcept { return out_; }
  [[nodiscard]] uint32_t holds() const noexcept { return holds_; }
  [[nodiscard]] uint32_t saturated_frames() const noexcept { return saturated_; }

  struct State {
    Demand out;
    std::array<double, 3> integ{};
  };
  [[nodiscard]] State state() const noexcept { return State{out_, integ_}; }
  void set_state(const State& s) noexcept {
    out_ = s.out;
    integ_ = s.integ;
  }
  // Clear the integrators (a new phase has new gains and a new plant: what the old one accumulated is not meant for it).
  void clear_integrators() noexcept { integ_ = {0.0, 0.0, 0.0}; }

  // A 16-bit fingerprint of the command and the integrators, for the digest the replicas exchange.
  [[nodiscard]] uint16_t digest() const noexcept {
    uint32_t h = 2166136261U;
    for (const double v : {out_.roll, out_.yaw, out_.pitch, integ_[0], integ_[1], integ_[2]}) {
      const double q = v / 0.001;
      const int32_t n = static_cast<int32_t>(q >= 0.0 ? q + 0.5 : q - 0.5);
      h = (h ^ static_cast<uint32_t>(n)) * 16777619U;
    }
    return static_cast<uint16_t>((h ^ (h >> 16U)) & 0xFFFFU);
  }

 private:
  double axis(unsigned a, double err, double rate_err, const AxisGains& g, const Limits& lim, double dt) noexcept {
    const double lo = -static_cast<double>(lim.command_deg);
    const double hi = static_cast<double>(lim.command_deg);
    const double unsat = (static_cast<double>(g.kp) * err) + (static_cast<double>(g.kd) * rate_err) + integ_[a];
    const bool pushing_further = (unsat > hi && err > 0.0) || (unsat < lo && err < 0.0);
    if (pushing_further) {
      ++saturated_;   // anti-windup: no integrating while saturated with the error still pushing the same way
    } else {
      const double lim_i = static_cast<double>(lim.integrator_deg);
      integ_[a] = dm::clamp_(integ_[a] + (static_cast<double>(g.ki) * err * dt), -lim_i, lim_i);
    }
    const double prev = a == 0U ? out_.roll : (a == 1U ? out_.yaw : out_.pitch);
    const double target = dm::clamp_(unsat, lo, hi);
    const double slew = static_cast<double>(lim.slew_deg_per_frame);
    return prev + dm::clamp_(target - prev, -slew, slew);
  }

  Demand out_{};
  std::array<double, 3> integ_{};
  Vec3 err_deg_{};
  uint32_t holds_ = 0U;
  uint32_t saturated_ = 0U;
};

}  // namespace tfc::att
