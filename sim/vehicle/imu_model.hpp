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

// What a real IMU does that the nominal simulated one does not. All zero or neutral by default. The first group is the bench model of the first weeks (uniform noise and constant errors); the
// second is the ISM330DHCX's datasheet model (docs/design/IMU_MODEL.md): white noise from a density, a wandering bias, drift with temperature, cross-axis sensitivity and sample jitter.
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
  // The datasheet model. A noise density above zero REPLACES the uniform noise above by Gaussian noise of rms density x sqrt(bandwidth) per sample.
  float gyro_noise_density_dps = 0.0F;     // dps per root hertz
  float accel_noise_density_g = 0.0F;      // g per root hertz
  float noise_bandwidth_hz = 416.0F;       // the noise bandwidth: half the output data rate (833 Hz); the 100 Hz samples of the firmware alias the rest, they do not filter it
  float gyro_bias_instability_dps = 0.0F;  // the standard deviation of a slowly wandering bias (a first-order Gauss-Markov process, one sample a frame of 10 ms)
  float bias_correlation_s = 100.0F;       // and its correlation time
  float temperature_offset_c = 0.0F;       // the sensor's temperature above 25 degrees C
  float gyro_bias_tc_dps_per_c = 0.0F;     // zero-rate level change with temperature, up to this per degree (the sign drawn once per sensor and axis)
  float accel_bias_tc_g_per_c = 0.0F;
  float gyro_sens_tc_per_c = 0.0F;         // sensitivity change with temperature, up to this fraction per degree
  float accel_sens_tc_per_c = 0.0F;
  float gyro_cross_axis = 0.0F;            // cross-axis sensitivity: every off-diagonal term of the sensor's matrix is up to this fraction, drawn once per sensor
  float accel_cross_axis = 0.0F;
  float sample_jitter = 0.0F;              // the sample is taken up to this fraction of a frame early (uniform), by interpolating between this frame's input and the last one's
};

// The ISM330DHCX of DS13012 rev 7 (Table 3), the typical values where it gives them and the limits where it gives only those: a zero-rate level of up to 1 dps and a zero-g offset of up to 10 mg,
// a rate noise of 5 mdps per root hertz and an acceleration noise of 60 micro-g per root hertz, a gyro bias instability of 3 degrees an hour, a sensitivity within 2 %, cross-axis sensitivity of 1 %
// (gyro) and 0.5 % (accelerometer), zero-rate and zero-g drift of 0.005 dps and 0.1 mg per degree, and sensitivity drift of 0.007 % and 0.005 % per degree. Read from the datasheet on 7 Oct 2026.
inline SensorErrors ism330dhcx_typical() {
  SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.gyro_noise_density_dps = 0.005F;
  e.accel_noise_density_g = 0.00006F;
  e.gyro_bias_dps = 1.0F;
  e.accel_bias_g = 0.010F;
  e.gyro_scale_err = 0.02F;
  e.accel_scale_err = 0.02F;
  e.gyro_cross_axis = 0.01F;
  e.accel_cross_axis = 0.005F;
  e.gyro_bias_instability_dps = 3.0F / 3600.0F;
  e.gyro_bias_tc_dps_per_c = 0.005F;
  e.accel_bias_tc_g_per_c = 0.0001F;
  e.gyro_sens_tc_per_c = 0.00007F;
  e.accel_sens_tc_per_c = 0.00005F;
  return e;
}

// The same with the datasheet's worst cases (the 3-sigma limits it gives): a zero-rate level of 3 dps, a zero-g offset of 65 mg, a rate noise of 8 mdps and an acceleration noise of 100 micro-g
// per root hertz, drift of 0.015 dps and 0.5 mg per degree, and 0.015 % and 0.01 % per degree for the sensitivities.
inline SensorErrors ism330dhcx_maximum() {
  SensorErrors e = ism330dhcx_typical();
  e.gyro_noise_density_dps = 0.008F;
  e.accel_noise_density_g = 0.0001F;
  e.gyro_bias_dps = 3.0F;
  e.accel_bias_g = 0.065F;
  e.gyro_bias_tc_dps_per_c = 0.015F;
  e.accel_bias_tc_g_per_c = 0.0005F;
  e.gyro_sens_tc_per_c = 0.00015F;
  e.accel_sens_tc_per_c = 0.0001F;
  return e;
}

class ImuModel {
 public:
  ImuModel(const SensorErrors& e, uint32_t seed) : e_(e), noise_(seed), draw_(seed ^ 0xA5A5A5A5U), dist_(seed ^ 0x5A5A5A5AU), gaussian_(seed ^ 0x3C3C3C3CU) {
    for (unsigned i = 0; i < 3U; ++i) {
      gb_[i] = e.gyro_bias_dps * draw_.uniform();
      ab_[i] = e.accel_bias_g * draw_.uniform();
      gs_[i] = 1.0F + (e.gyro_scale_err * draw_.uniform());
      as_[i] = 1.0F + (e.accel_scale_err * draw_.uniform());
      mis_[i] = e.misalign_deg * 0.0174532925F * draw_.uniform();
    }
    // The datasheet model's draws come from generators of their own, and only when it is used: the bench model's stream, and so every earlier flight, is unchanged.
    if (datasheet_model()) {
      // The bench model's constants come from the first outputs of a linear congruential generator seeded with neighbouring numbers (the firmware's nodes are seeded 0x1234, 0x1235, 0x1236):
      // those outputs differ by a few parts in a thousand, so the three computers' IMUs have almost the same bias, scale error and misalignment, which a real set of parts does not. The
      // datasheet model lets its generators run for a while first and draws every constant from them.
      for (int k = 0; k < 16; ++k) {
        (void)dist_.uniform();
        (void)gaussian_.uniform();
      }
      for (unsigned i = 0; i < 3U; ++i) {
        gb_[i] = e.gyro_bias_dps * dist_.uniform();
        ab_[i] = e.accel_bias_g * dist_.uniform();
        gs_[i] = 1.0F + (e.gyro_scale_err * dist_.uniform());
        as_[i] = 1.0F + (e.accel_scale_err * dist_.uniform());
        mis_[i] = e.misalign_deg * 0.0174532925F * dist_.uniform();
      }
      for (unsigned i = 0; i < 3U; ++i) {
        gbias_state_[i] = e.gyro_bias_instability_dps * gauss();
        gtc_[i] = e.gyro_bias_tc_dps_per_c * dist_.uniform();
        atc_[i] = e.accel_bias_tc_g_per_c * dist_.uniform();
        for (unsigned j = 0; j < 3U; ++j) {
          gx_[i][j] = i == j ? 0.0F : e.gyro_cross_axis * dist_.uniform();
          ax_[i][j] = i == j ? 0.0F : e.accel_cross_axis * dist_.uniform();
        }
      }
    }
  }

  // One frame: the true inputs in, this node's sample out (the noise draws keep the order of the original test: gyro axis, accel axis, ...).
  void sample(const tfc::Vec3& g_true, const tfc::Vec3& a_true, tfc::Vec3& g_out, tfc::Vec3& a_out) {
    tfc::Vec3 g = rotate(g_true);
    tfc::Vec3 a = rotate(a_true);
    if (datasheet_model()) {
      sample_datasheet(g_true, a_true, g, a);
    } else {
      for (unsigned i = 0; i < 3U; ++i) {
        g.v[i] = (g.v[i] * gs_[i]) + gb_[i] + (e_.gyro_noise_amp_dps * noise_.uniform());
        a.v[i] = (a.v[i] * as_[i]) + ab_[i] + (e_.accel_noise_amp_g * noise_.uniform());
      }
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
  [[nodiscard]] bool datasheet_model() const {
    return e_.gyro_noise_density_dps > 0.0F || e_.accel_noise_density_g > 0.0F || e_.gyro_bias_instability_dps > 0.0F || e_.temperature_offset_c != 0.0F || e_.gyro_cross_axis > 0.0F ||
           e_.accel_cross_axis > 0.0F || e_.sample_jitter > 0.0F;
  }

  // A normal deviate (Box-Muller) from the model's own generator.
  float gauss() {
    float u1 = (gaussian_.uniform() + 1.0F) * 0.5F;
    while (u1 <= 1e-12F) {
      u1 = (gaussian_.uniform() + 1.0F) * 0.5F;
    }
    const float u2 = (gaussian_.uniform() + 1.0F) * 0.5F;
    return std::sqrt(-2.0F * std::log(u1)) * std::cos(6.2831853F * u2);
  }

  // The sample of the datasheet model: the input (late by the jitter), turned by the mounting error, through the cross-axis matrix and the sensitivity (which drifts with temperature), plus the bias
  // (constant, wandering and drifting with temperature) and Gaussian noise (or the bench's uniform noise for a channel whose density is not given).
  void sample_datasheet(const tfc::Vec3& g_true, const tfc::Vec3& a_true, tfc::Vec3& g, tfc::Vec3& a) {
    tfc::Vec3 gi = g_true;
    tfc::Vec3 ai = a_true;
    if (e_.sample_jitter > 0.0F) {
      const float u = e_.sample_jitter * ((dist_.uniform() + 1.0F) * 0.5F);
      if (have_input_) {
        for (unsigned i = 0; i < 3U; ++i) {
          gi.v[i] = g_true.v[i] + ((prev_in_g_.v[i] - g_true.v[i]) * u);
          ai.v[i] = a_true.v[i] + ((prev_in_a_.v[i] - a_true.v[i]) * u);
        }
      }
      prev_in_g_ = g_true;
      prev_in_a_ = a_true;
      have_input_ = true;
    }
    gi = rotate(gi);
    ai = rotate(ai);
    const float dt = 0.01F;  // one sample a frame
    const float decay = std::exp(-dt / (e_.bias_correlation_s > 0.0F ? e_.bias_correlation_s : 100.0F));
    const float drive = std::sqrt(1.0F - (decay * decay)) * e_.gyro_bias_instability_dps;
    const float gn = e_.gyro_noise_density_dps * std::sqrt(e_.noise_bandwidth_hz);
    const float an = e_.accel_noise_density_g * std::sqrt(e_.noise_bandwidth_hz);
    for (unsigned i = 0; i < 3U; ++i) {
      float gv = gi.v[i];
      float av = ai.v[i];
      for (unsigned j = 0; j < 3U; ++j) {
        gv += gx_[i][j] * gi.v[j];
        av += ax_[i][j] * ai.v[j];
      }
      gbias_state_[i] = (decay * gbias_state_[i]) + (drive * gauss());
      const float gain_g = gs_[i] * (1.0F + (e_.gyro_sens_tc_per_c * e_.temperature_offset_c));
      const float gain_a = as_[i] * (1.0F + (e_.accel_sens_tc_per_c * e_.temperature_offset_c));
      const float noise_g = e_.gyro_noise_density_dps > 0.0F ? gn * gauss() : e_.gyro_noise_amp_dps * noise_.uniform();
      const float noise_a = e_.accel_noise_density_g > 0.0F ? an * gauss() : e_.accel_noise_amp_g * noise_.uniform();
      g.v[i] = (gv * gain_g) + gb_[i] + gbias_state_[i] + (gtc_[i] * e_.temperature_offset_c) + noise_g;
      a.v[i] = (av * gain_a) + ab_[i] + (atc_[i] * e_.temperature_offset_c) + noise_a;
    }
  }

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
  Lcg dist_;      // the datasheet model's constant draws and its jitter
  Lcg gaussian_;  // and its Gaussian noise
  std::array<float, 3> gb_{}, ab_{}, gs_{}, as_{}, mis_{};
  std::array<float, 3> gbias_state_{}, gtc_{}, atc_{};
  std::array<std::array<float, 3>, 3> gx_{}, ax_{};
  tfc::Vec3 prev_in_g_{};
  tfc::Vec3 prev_in_a_{};
  bool have_input_ = false;
  std::array<tfc::Vec3, kHistory> history_g_{};
  std::array<tfc::Vec3, kHistory> history_a_{};
  unsigned head_ = 0U;
  tfc::Vec3 prev_g_{};
  tfc::Vec3 prev_a_{};
  bool have_prev_ = false;
};

}  // namespace sim
