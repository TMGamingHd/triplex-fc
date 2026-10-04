// SPDX-License-Identifier: MIT
// The motion platform the IMUs sit on (host model of the rig): it follows the vehicle's two tilt angles through a servo with a lag, a rate limit and a
// travel limit. What its sensors feel is the rotation of a two-axis platform and gravity in the platform frame (the platform does not accelerate).
//   gravity in the sensor frame v = [-sin(ty), cos(ty) sin(tx), cos(ty) cos(tx)] (g);  body rates p = tx', q = ty' cos(tx), r = -ty' sin(tx).
#pragma once
#include <algorithm>
#include <cmath>

#include "math3.hpp"
#include "vehicle6.hpp"

namespace sim {

struct PlatformConfig {
  double tau_s = 0.06;                 // servo lag
  double rate_limit_dps = 300.0;
  double limit_deg = 45.0;             // the D85MG travel is about 60 degrees; hard stops are fitted inside it
};

class Platform {
 public:
  explicit Platform(const PlatformConfig& c = PlatformConfig{}) : c_(c) {}

  // Follow the target tilts for dt seconds. `saturated()` says the target was beyond the platform's travel at the last step.
  void step(const Tilts& target, double dt) {
    saturated_ = std::fabs(target.x_deg) > c_.limit_deg || std::fabs(target.y_deg) > c_.limit_deg;
    follow(tx_, rx_, target.x_deg, dt);
    follow(ty_, ry_, target.y_deg, dt);
  }

  [[nodiscard]] Tilts tilts() const { return Tilts{tx_, ty_}; }
  [[nodiscard]] bool saturated() const { return saturated_; }

  // The IMU inputs in the sensor frame: body rates (dps) and gravity (g).
  void imu_truth(V3& gyro_dps, V3& accel_g) const {
    const double phi = tx_ * kDeg2Rad;
    const double theta = ty_ * kDeg2Rad;
    gyro_dps = V3{rx_, ry_ * std::cos(phi), -ry_ * std::sin(phi)};
    accel_g = V3{-std::sin(theta), std::cos(theta) * std::sin(phi), std::cos(theta) * std::cos(phi)};
  }

 private:
  void follow(double& angle, double& rate, double target, double dt) const {
    const double goal = std::clamp(target, -c_.limit_deg, c_.limit_deg);
    rate = std::clamp((goal - angle) / c_.tau_s, -c_.rate_limit_dps, c_.rate_limit_dps);
    angle = std::clamp(angle + (rate * dt), -c_.limit_deg, c_.limit_deg);
  }

  PlatformConfig c_;
  double tx_ = 0.0;
  double ty_ = 0.0;
  double rx_ = 0.0;  // deg/s
  double ry_ = 0.0;
  bool saturated_ = false;
};

}  // namespace sim
