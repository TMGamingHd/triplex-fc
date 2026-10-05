// SPDX-License-Identifier: MIT
// Stationary gyro calibration of one IMU channel (docs/LAUNCH_SEQUENCE.md). On the pad, before lift-off, the platform is at rest, so the gyro's mean over a few seconds is its bias.
// Each flight computer calibrates its OWN IMU and subtracts the bias from its samples before it sends them, so the consensus sees corrected values: IMUs differ from one another by more
// than the consensus tolerance (1 dps) long before any of them is faulty, and the consensus must keep detecting a real fault. (This is the per-channel calibration that STAGED_BUILD
// rule 11 puts in every computer's configuration.)
//   set_pad(true)    start collecting; the bias in use is the running mean once 100 samples are in
//   process(raw)     the corrected sample (raw minus the bias in use); while on the pad it also feeds the mean
//   set_pad(false)   lift-off: if the calibration is ready the bias is frozen at the final mean; if not, the running mean (or the previous bias) stays
//   ready()          enough samples, the platform really at rest (the spread of every axis under `max_std_dps`), a plausible bias (under `max_bias_dps`)
// Only + - * / on floats (Welford's running mean and variance), so replicas agree bit for bit.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/protocol.hpp"

namespace tfc {

struct ImuCalibratorConfig {
  uint16_t samples = 1000U;       // 10 s at 100 Hz: the mean is good to about 0.003 dps with the simulator's noise
  uint16_t apply_after = 100U;    // the running mean is used from this many samples on
  float max_std_dps = 0.6F;
  float max_bias_dps = 5.0F;
};

class ImuCalibrator {
 public:
  ImuCalibrator() noexcept = default;
  explicit ImuCalibrator(const ImuCalibratorConfig& cfg) noexcept : cfg_(cfg) {}

  void set_pad(bool on) noexcept {
    if (on && !pad_) {
      n_ = 0U;
      mean_ = {};
      m2_ = {};
    } else if (!on && pad_ && ready()) {
      bias_ = mean_;  // frozen at the final mean
    }
    pad_ = on;
  }

  [[nodiscard]] Vec3 process(const Vec3& raw) noexcept {
    if (pad_ && finite(raw)) {
      accumulate(raw);
      if (n_ >= cfg_.apply_after) {
        bias_ = mean_;
      }
    }
    Vec3 out;
    for (unsigned i = 0; i < 3U; ++i) {
      out.v[i] = raw.v[i] - bias_.v[i];
    }
    return out;
  }

  [[nodiscard]] bool ready() const noexcept {
    if (n_ < 2U || n_ < cfg_.samples) {
      return false;
    }
    for (unsigned i = 0; i < 3U; ++i) {
      const float var = m2_[i] / static_cast<float>(n_ - 1U);
      if (!(var <= cfg_.max_std_dps * cfg_.max_std_dps) || !(std::fabs(mean_.v[i]) <= cfg_.max_bias_dps)) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] const Vec3& bias() const noexcept { return bias_; }
  [[nodiscard]] uint32_t samples() const noexcept { return n_; }
  [[nodiscard]] bool pad() const noexcept { return pad_; }

 private:
  static bool finite(const Vec3& v) noexcept {
    return v.v[0] > -1.0e6F && v.v[0] < 1.0e6F && v.v[1] > -1.0e6F && v.v[1] < 1.0e6F && v.v[2] > -1.0e6F && v.v[2] < 1.0e6F;
  }

  void accumulate(const Vec3& g) noexcept {
    if (n_ >= 60000U) {
      return;
    }
    ++n_;
    const float n = static_cast<float>(n_);
    for (unsigned i = 0; i < 3U; ++i) {
      const float d = g.v[i] - mean_.v[i];
      mean_.v[i] += d / n;
      m2_[i] += d * (g.v[i] - mean_.v[i]);
    }
  }

  ImuCalibratorConfig cfg_{};
  bool pad_ = false;
  uint32_t n_ = 0U;
  Vec3 mean_{};
  std::array<float, 3> m2_{};
  Vec3 bias_{};
};

}  // namespace tfc
