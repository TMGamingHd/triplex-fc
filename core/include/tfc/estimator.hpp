// SPDX-License-Identifier: MIT
// Attitude estimator: a Mahony complementary filter on the consensus gyro and accelerometer (a quaternion, a gyro-bias
// integrator, gravity as the reference). It is deliberately simple, deterministic and free of library transcendentals: the only
// operations are + - * / and sqrt (correctly rounded by IEEE 754), with its own atan2, so that three replicas, and the host and the
// target, compute the same bits. The build must not contract multiply-add (-ffp-contract=off) and must not use -ffast-math.
//
// Frames and angles. The rig is a platform that tilts about two horizontal axes, so both angles are observable from gravity:
//   tilt_y: rotation about the sensor's Y axis (the vehicle's pitch plane), tilt_x: rotation about X (the vehicle's yaw plane).
// With the sensor's gravity vector v = [-sin(ty), cos(ty) sin(tx), cos(ty) cos(tx)], which is what an accelerometer at rest reads.
// A rotation about the vertical is not a degree of freedom of the rig and is not estimated or controlled.
// The accelerometer is used for the correction only while its magnitude is within a gate of 1 g: thrust and vibration would
// corrupt the gravity reference (docs/DEFERRED.md, analytical redundancy), so the filter then coasts on the gyro.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cmath>
#include <cstdint>

#include "tfc/consensus.hpp"
#include "tfc/protocol.hpp"

namespace tfc {

constexpr float kDegToRad = 0.017453292519943295F;
constexpr float kRadToDeg = 57.29577951308232F;
constexpr float kPiF = 3.14159265358979F;

namespace det {
// atan(x) for |x| <= 1: Abramowitz and Stegun 4.4.49, |error| <= 1e-5 rad. Pure arithmetic, so identical everywhere.
constexpr float atan_unit(float x) noexcept {
  const float x2 = x * x;
  return x * (0.9998660F + x2 * (-0.3302995F + x2 * (0.1801410F + x2 * (-0.0851330F + x2 * 0.0208351F))));
}

// atan2(y, x) in radians from the polynomial above, with the usual quadrant handling. atan2(0, 0) is 0.
constexpr float atan2_approx(float y, float x) noexcept {
  const float ax = x < 0.0F ? -x : x;
  const float ay = y < 0.0F ? -y : y;
  const float mx = ax > ay ? ax : ay;
  const float mn = ax > ay ? ay : ax;
  if (!(mx > 0.0F)) {
    return 0.0F;
  }
  float r = atan_unit(mn / mx);
  if (ay > ax) {
    r = 0.5F * kPiF - r;
  }
  if (x < 0.0F) {
    r = kPiF - r;
  }
  return y < 0.0F ? -r : r;
}
}  // namespace det

struct EstimatorConfig {
  float kp = 2.0F;               // gain of the tilt correction, 1/s (time constant about half a second)
  float ki = 0.05F;              // gain of the bias integrator, 1/s^2
  float bias_limit_dps = 5.0F;   // the estimated gyro bias is clamped to this
  float accel_gate_g = 0.3F;     // the accelerometer corrects only while |a| is within 1 g +- this
  // False: the accelerometer is not used at all (the attitude is the gyro's integral). For a vehicle under thrust, where the specific force is the thrust and not gravity: with a
  // thrust-to-weight near 1 the gate above accepts thrust as gravity and pulls the estimated tilt to zero (docs/SIM_FIDELITY.md 3.2). The rig's platform is never under thrust.
  bool use_accel = true;
};

struct Attitude {
  float tilt_x_deg = 0.0F;  // the vehicle's yaw plane
  float tilt_y_deg = 0.0F;  // the vehicle's pitch plane
  float rate_x_dps = 0.0F;  // rates of those two angles, bias corrected
  float rate_y_dps = 0.0F;
  bool valid = false;       // the last update had a trustworthy gyro
};

class AttitudeEstimator {
 public:
  AttitudeEstimator() noexcept = default;
  explicit AttitudeEstimator(const EstimatorConfig& cfg) noexcept : cfg_(cfg) {}

  // One step of dt seconds from the consensus input. With no trustworthy gyro the last rates are held (and counted);
  // with no trustworthy accelerometer, or one outside the gate, the correction is skipped.
  void update(const ConsensusInput& in, float dt) noexcept {
    ++steps_;
    const bool gyro_good = in.gyro_ok && std::isfinite(in.gyro_dps.v[0]) && std::isfinite(in.gyro_dps.v[1]) && std::isfinite(in.gyro_dps.v[2]);
    if (gyro_good) {
      for (unsigned i = 0; i < 3U; ++i) {
        w_[i] = in.gyro_dps.v[i] * kDegToRad;
      }
    } else {
      ++gyro_holds_;
    }
    std::array<float, 3> corr{0.0F, 0.0F, 0.0F};
    const bool accel_usable = cfg_.use_accel && in.accel_ok;
    if (!aligned_ && accel_usable && align(in.accel_g)) {
      aligned_ = true;  // the first trustworthy gravity reading sets the attitude: no start-up transient for the integrator to learn from
    }
    if (accel_usable && correction(in.accel_g, corr)) {
      for (unsigned i = 0; i < 3U; ++i) {
        bias_[i] += cfg_.ki * corr[i] * dt;
        bias_[i] = clamp(bias_[i], cfg_.bias_limit_dps * kDegToRad);
      }
    } else {
      corr = {0.0F, 0.0F, 0.0F};
      ++accel_skips_;
    }
    std::array<float, 3> w{};
    for (unsigned i = 0; i < 3U; ++i) {
      w[i] = w_[i] + bias_[i] + (cfg_.kp * corr[i]);  // the integrator carries the (negated) bias
    }
    integrate(w, dt);
    rates_valid_ = gyro_good;
  }

  [[nodiscard]] Attitude attitude() const noexcept {
    Attitude a;
    const std::array<float, 3> v = gravity();
    const float ct = std::sqrt((v[1] * v[1]) + (v[2] * v[2]));  // cos(ty), >= 0
    a.tilt_y_deg = det::atan2_approx(-v[0], ct) * kRadToDeg;
    a.tilt_x_deg = det::atan2_approx(v[1], v[2]) * kRadToDeg;
    // Rates of the two angles from the body rates (p, q, r) of a two-axis platform: p = dtx, q = dty cos(tx), r = -dty sin(tx).
    const float p = w_[0] + bias_[0];
    const float q = w_[1] + bias_[1];
    const float r = w_[2] + bias_[2];
    const float norm = ct > 1e-6F ? ct : 1e-6F;
    const float sx = v[1] / norm;
    const float cx = v[2] / norm;
    a.rate_x_dps = p * kRadToDeg;  // bias_ is the integrator, which carries the negated gyro bias: w_ + bias_ is the corrected rate
    a.rate_y_dps = ((q * cx) - (r * sx)) * kRadToDeg;
    a.valid = rates_valid_;
    return a;
  }

  // A 16-bit fingerprint of the quantised state (the angles to 0.01 degree, the bias to 0.01 dps, the number of updates), for the
  // digest cross-check of ADR-006; it agrees between replicas that did the same updates and is version-stable in the sense of ADR-021.
  [[nodiscard]] uint16_t digest() const noexcept {
    const Attitude a = attitude();
    const int32_t qx = quantize_i32(a.tilt_x_deg, 0.01F);
    const int32_t qy = quantize_i32(a.tilt_y_deg, 0.01F);
    const int32_t b0 = quantize_i32(bias_[0] * kRadToDeg, 0.01F);
    const int32_t b1 = quantize_i32(bias_[1] * kRadToDeg, 0.01F);
    const int32_t b2 = quantize_i32(bias_[2] * kRadToDeg, 0.01F);
    uint32_t h = 2166136261U;
    for (const int32_t v : {qx, qy, b0, b1, b2, static_cast<int32_t>(steps_ & 0xFFU)}) {
      h = (h ^ static_cast<uint32_t>(v)) * 16777619U;
    }
    return static_cast<uint16_t>((h ^ (h >> 16)) & 0xFFFFU);
  }

  [[nodiscard]] uint32_t steps() const noexcept { return steps_; }
  [[nodiscard]] uint32_t gyro_holds() const noexcept { return gyro_holds_; }
  [[nodiscard]] uint32_t accel_skips() const noexcept { return accel_skips_; }
  [[nodiscard]] float bias_x_dps() const noexcept { return -bias_[0] * kRadToDeg; }
  [[nodiscard]] float bias_y_dps() const noexcept { return -bias_[1] * kRadToDeg; }
  [[nodiscard]] float bias_z_dps() const noexcept { return -bias_[2] * kRadToDeg; }

 private:
  static constexpr float clamp(float v, float lim) noexcept { return v > lim ? lim : (v < -lim ? -lim : v); }

  // The state is finite and bounded (the gyro is checked for finiteness above, the angles are within +-180 degrees, the bias within its
  // limit), so the conversion cannot overflow.
  static constexpr int32_t quantize_i32(float v, float lsb) noexcept {
    const float q = v / lsb;
    return static_cast<int32_t>(q >= 0.0F ? q + 0.5F : q - 0.5F);
  }

  // The gravity direction the current quaternion predicts, in the sensor frame.
  [[nodiscard]] std::array<float, 3> gravity() const noexcept {
    return {2.0F * ((q_[1] * q_[3]) - (q_[0] * q_[2])), 2.0F * ((q_[0] * q_[1]) + (q_[2] * q_[3])),
            (q_[0] * q_[0]) - (q_[1] * q_[1]) - (q_[2] * q_[2]) + (q_[3] * q_[3])};
  }

  // Set the quaternion from the first gravity reading: the shortest rotation that takes the level reference to the measured direction
  // (no yaw, which the rig does not have). False if the reading is outside the gate or nearly upside down.
  bool align(const Vec3& a) noexcept {
    const float n2 = (a.v[0] * a.v[0]) + (a.v[1] * a.v[1]) + (a.v[2] * a.v[2]);
    const float lo = 1.0F - cfg_.accel_gate_g;
    const float hi = 1.0F + cfg_.accel_gate_g;
    if (!(n2 > lo * lo) || !(n2 < hi * hi)) {
      return false;
    }
    const float inv = 1.0F / std::sqrt(n2);
    const float mx = a.v[0] * inv;
    const float my = a.v[1] * inv;
    const float mz = a.v[2] * inv;
    if (!(mz > -0.9F)) {
      return false;
    }
    const float n = std::sqrt((2.0F * (1.0F + mz)));  // |(1 + mz, -my, mx, 0)| is sqrt(2 (1 + mz))
    q_ = {(1.0F + mz) / n, my / n, -mx / n, 0.0F};
    return true;
  }

  // The tilt error: the cross product of the measured and the predicted gravity direction. False if the reading is outside the gate.
  bool correction(const Vec3& a, std::array<float, 3>& e) const noexcept {
    const float n2 = (a.v[0] * a.v[0]) + (a.v[1] * a.v[1]) + (a.v[2] * a.v[2]);
    const float lo = 1.0F - cfg_.accel_gate_g;
    const float hi = 1.0F + cfg_.accel_gate_g;
    if (!(n2 > lo * lo) || !(n2 < hi * hi)) {  // also rejects NaN
      return false;
    }
    const float inv = 1.0F / std::sqrt(n2);
    const std::array<float, 3> m{a.v[0] * inv, a.v[1] * inv, a.v[2] * inv};
    const std::array<float, 3> v = gravity();
    e[0] = (m[1] * v[2]) - (m[2] * v[1]);
    e[1] = (m[2] * v[0]) - (m[0] * v[2]);
    e[2] = (m[0] * v[1]) - (m[1] * v[0]);
    return true;
  }

  void integrate(const std::array<float, 3>& w, float dt) noexcept {
    const float h = 0.5F * dt;
    const float qw = q_[0];
    const float qx = q_[1];
    const float qy = q_[2];
    const float qz = q_[3];
    q_[0] = qw + (h * (-qx * w[0] - qy * w[1] - qz * w[2]));
    q_[1] = qx + (h * (qw * w[0] + qy * w[2] - qz * w[1]));
    q_[2] = qy + (h * (qw * w[1] - qx * w[2] + qz * w[0]));
    q_[3] = qz + (h * (qw * w[2] + qx * w[1] - qy * w[0]));
    const float n2 = (q_[0] * q_[0]) + (q_[1] * q_[1]) + (q_[2] * q_[2]) + (q_[3] * q_[3]);
    const float inv = 1.0F / std::sqrt(n2);
    for (float& c : q_) {
      c *= inv;
    }
  }

  EstimatorConfig cfg_{};
  std::array<float, 4> q_{1.0F, 0.0F, 0.0F, 0.0F};  // sensor frame relative to the level reference
  std::array<float, 3> w_{};                         // last trusted gyro, rad/s
  std::array<float, 3> bias_{};                      // the integrator, rad/s
  uint32_t steps_ = 0U;
  uint32_t gyro_holds_ = 0U;
  uint32_t accel_skips_ = 0U;
  bool aligned_ = false;
  bool rates_valid_ = false;
};

}  // namespace tfc
