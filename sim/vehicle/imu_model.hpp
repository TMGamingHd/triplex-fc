// SPDX-License-Identifier: MIT
// The IMU models of the simulator: what a real IMU does that the nominal simulated one does not (noise, bias, scale error, misalignment, latency, stale samples), and the small deterministic
// generator behind them. Used by the closed loop (closed_loop.hpp) and by the release compatibility harness (tools/compat), which must not depend on anything of the flight core beyond the
// protocol's vectors.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/protocol.hpp"

namespace sim {

// A small deterministic generator for the IMU models.
class Lcg {
 public:
  explicit Lcg(uint32_t s) : s_(s) {}
  float uniform() {  // [-1, 1)
    s_ = (s_ * 1664525U) + 1013904223U;
    return (static_cast<float>(s_ >> 8) / 8388608.0F) - 1.0F;
  }

 private:
  uint32_t s_;
};

// What a real IMU does that the nominal simulated one does not. All zero or neutral by default.
struct SensorErrors {
  float gyro_noise_amp_dps = 0.17F;    // uniform noise, +- this (the firmware's BusImu uses the same)
  float accel_noise_amp_g = 0.0035F;
  float gyro_bias_dps = 0.0F;          // each node gets a constant bias of up to +- this on each axis
  float accel_bias_g = 0.0F;
  float gyro_scale_err = 0.0F;         // each node's gain error, up to +- this fraction, per axis
  float accel_scale_err = 0.0F;
  float misalign_deg = 0.0F;           // each node's mounting error: a fixed small rotation of up to this angle about each axis
  unsigned latency_frames = 0U;        // the sample is this many frames old
  float stale_prob = 0.0F;             // chance that a frame brings the previous sample again (the sensor's output rate is not synchronous with the frame)
};

class ImuModel {
 public:
  ImuModel(const SensorErrors& e, uint32_t seed) : e_(e), noise_(seed), draw_(seed ^ 0xA5A5A5A5U) {
    for (unsigned i = 0; i < 3U; ++i) {
      gb_[i] = e.gyro_bias_dps * draw_.uniform();
      ab_[i] = e.accel_bias_g * draw_.uniform();
      gs_[i] = 1.0F + (e.gyro_scale_err * draw_.uniform());
      as_[i] = 1.0F + (e.accel_scale_err * draw_.uniform());
      mis_[i] = e.misalign_deg * 0.0174532925F * draw_.uniform();
    }
  }

  // One frame: the true inputs in, this node's sample out (the noise draws keep the order of the original test: gyro axis, accel axis, ...).
  void sample(const tfc::Vec3& g_true, const tfc::Vec3& a_true, tfc::Vec3& g_out, tfc::Vec3& a_out) {
    tfc::Vec3 g = rotate(g_true);
    tfc::Vec3 a = rotate(a_true);
    for (unsigned i = 0; i < 3U; ++i) {
      g.v[i] = (g.v[i] * gs_[i]) + gb_[i] + (e_.gyro_noise_amp_dps * noise_.uniform());
      a.v[i] = (a.v[i] * as_[i]) + ab_[i] + (e_.accel_noise_amp_g * noise_.uniform());
    }
    history_g_[head_] = g;
    history_a_[head_] = a;
    const unsigned lat = e_.latency_frames < kHistory ? e_.latency_frames : kHistory - 1U;
    const unsigned idx = (head_ + kHistory - lat) % kHistory;
    head_ = (head_ + 1U) % kHistory;
    if (e_.stale_prob > 0.0F && have_prev_ && ((draw_.uniform() + 1.0F) * 0.5F) < e_.stale_prob) {
      g_out = prev_g_;
      a_out = prev_a_;
      return;
    }
    g_out = history_g_[idx];
    a_out = history_a_[idx];
    prev_g_ = g_out;
    prev_a_ = a_out;
    have_prev_ = true;
  }

 private:
  static constexpr unsigned kHistory = 8U;
  // a fixed small rotation (about X, Y, Z in turn) applied to the true vector: the sensor's axes are not exactly the platform's
  [[nodiscard]] tfc::Vec3 rotate(const tfc::Vec3& v) const {
    tfc::Vec3 r = v;
    const float c0 = std::cos(mis_[0]);
    const float s0 = std::sin(mis_[0]);
    r.v = {v.v[0], (c0 * v.v[1]) - (s0 * v.v[2]), (s0 * v.v[1]) + (c0 * v.v[2])};
    const float c1 = std::cos(mis_[1]);
    const float s1 = std::sin(mis_[1]);
    r.v = {(c1 * r.v[0]) + (s1 * r.v[2]), r.v[1], (-s1 * r.v[0]) + (c1 * r.v[2])};
    const float c2 = std::cos(mis_[2]);
    const float s2 = std::sin(mis_[2]);
    r.v = {(c2 * r.v[0]) - (s2 * r.v[1]), (s2 * r.v[0]) + (c2 * r.v[1]), r.v[2]};
    return r;
  }

  SensorErrors e_;
  Lcg noise_;
  Lcg draw_;
  std::array<float, 3> gb_{}, ab_{}, gs_{}, as_{}, mis_{};
  std::array<tfc::Vec3, kHistory> history_g_{};
  std::array<tfc::Vec3, kHistory> history_a_{};
  unsigned head_ = 0U;
  tfc::Vec3 prev_g_{};
  tfc::Vec3 prev_a_{};
  bool have_prev_ = false;
};

}  // namespace sim
