// SPDX-License-Identifier: MIT
// A vehicle model for closed-loop tests on the host (double precision, libm allowed: this is the simulated world, not flight code).
//
// Two decoupled planes, each an unstable rigid-body pitch (or yaw) channel steered by thrust vectoring:
//     angle'' = a * angle + b * scale * sin(gimbal) + disturbance
// `a` is the aerodynamic divergence (1/s^2), `b` the control effectiveness ((rad/s^2) per rad of gimbal), `scale` shrinks `b` for an
// engine-out, and the gimbal is an actuator with a limit and a rate limit. The IMU does not sit on the vehicle: it sits on a platform
// that follows the vehicle's attitude through a servo with a lag, a rate limit and a travel limit (ADR-005: the rig, not the rocket).
// Its measurements are what such a platform's sensors would read: the body rates of a two-axis platform and gravity in the sensor frame.
//   vehicle yaw plane  <-> platform tilt about X (phi);  vehicle pitch plane <-> platform tilt about Y (theta)
//   gravity in the sensor frame v = [-sin(theta), cos(theta) sin(phi), cos(theta) cos(phi)]  (what an accelerometer at rest reads, in g)
//   body rates p = phi', q = theta' cos(phi), r = -theta' sin(phi)
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace vehicle {

constexpr double kDeg = 0.017453292519943295;
constexpr double kRad = 57.29577951308232;

struct Params {
  double a = 0.44;                    // divergence, 1/s^2 (open-loop time constant about 1.5 s)
  double b = 2.0;                     // control effectiveness, (rad/s^2) per rad of gimbal
  double gimbal_limit_deg = 8.0;
  double gimbal_rate_dps = 60.0;      // the actuator's rate limit
  double platform_tau_s = 0.06;       // servo lag
  double platform_rate_limit_dps = 300.0;
  double platform_limit_deg = 45.0;
};

class Vehicle {
 public:
  explicit Vehicle(const Params& p = Params{}) : p_(p) {}

  void set_disturbance(double yaw_plane, double pitch_plane) {  // rad/s^2
    dist_ = {yaw_plane, pitch_plane};
  }
  void set_effectiveness_scale(double s) { scale_ = s; }

  // Advance by dt seconds with the commanded gimbal angles (degrees: pitch plane, yaw plane).
  void step(double pitch_cmd_deg, double yaw_cmd_deg, double dt) {
    const int sub = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
    const double h = dt / sub;
    for (int i = 0; i < sub; ++i) {
      actuate(0, yaw_cmd_deg, h);
      actuate(1, pitch_cmd_deg, h);
      for (int a = 0; a < 2; ++a) {
        const double acc = (p_.a * ang_[a]) + (p_.b * scale_ * std::sin(gimbal_[a] * kDeg)) + dist_[a];
        rate_[a] += acc * h;
        ang_[a] += rate_[a] * h;
        // the platform follows the vehicle through a lagged, rate- and travel-limited servo
        double v = (ang_[a] - plat_[a]) / p_.platform_tau_s;
        v = std::clamp(v, -p_.platform_rate_limit_dps * kDeg, p_.platform_rate_limit_dps * kDeg);
        plat_rate_[a] = v;
        plat_[a] = std::clamp(plat_[a] + (v * h), -p_.platform_limit_deg * kDeg, p_.platform_limit_deg * kDeg);
      }
    }
  }

  // The IMU's true inputs: body rates in dps and gravity in g, in the sensor frame, from the platform.
  void imu_truth(std::array<double, 3>& gyro_dps, std::array<double, 3>& accel_g) const {
    const double phi = plat_[0];
    const double theta = plat_[1];
    const double phid = plat_rate_[0];
    const double thetad = plat_rate_[1];
    gyro_dps = {phid * kRad, thetad * std::cos(phi) * kRad, -thetad * std::sin(phi) * kRad};
    accel_g = {-std::sin(theta), std::cos(theta) * std::sin(phi), std::cos(theta) * std::cos(phi)};
  }

  [[nodiscard]] double yaw_plane_deg() const { return ang_[0] * kRad; }
  [[nodiscard]] double pitch_plane_deg() const { return ang_[1] * kRad; }
  [[nodiscard]] double platform_x_deg() const { return plat_[0] * kRad; }
  [[nodiscard]] double platform_y_deg() const { return plat_[1] * kRad; }
  [[nodiscard]] double gimbal_pitch_deg() const { return gimbal_[1]; }
  [[nodiscard]] double gimbal_yaw_deg() const { return gimbal_[0]; }

 private:
  void actuate(int plane, double cmd_deg, double h) {
    const double c = std::clamp(cmd_deg, -p_.gimbal_limit_deg, p_.gimbal_limit_deg);
    const double d = std::clamp(c - gimbal_[plane], -p_.gimbal_rate_dps * h, p_.gimbal_rate_dps * h);
    gimbal_[plane] += d;
  }

  Params p_;
  double scale_ = 1.0;
  std::array<double, 2> dist_{0.0, 0.0};
  std::array<double, 2> ang_{0.0, 0.0};        // vehicle angle: yaw plane, pitch plane (rad)
  std::array<double, 2> rate_{0.0, 0.0};
  std::array<double, 2> plat_{0.0, 0.0};       // platform angle: phi (about X), theta (about Y)
  std::array<double, 2> plat_rate_{0.0, 0.0};
  std::array<double, 2> gimbal_{0.0, 0.0};     // the actuator's actual angle, degrees
};

}  // namespace vehicle
