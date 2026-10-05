// SPDX-License-Identifier: MIT
// The whole software chain closed over the 6-DOF vehicle, without a socket: SimRunner publishes the sensor inputs, three FlightFunctions (each with its own IMU model: noise, bias, scale,
// misalignment, latency, stale samples) compute commands, the real ActLogic votes them, and ACT's frame goes back to the runner. Used by tests/test_runner.cpp and by tools/sim/tfc_sens.cpp
// (sensitivity: how far each departure from the nominal can go before the flight is lost). The flown vehicle (`cfg.params`) can differ from the vehicle the tables were designed on (`cfg.design`).
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "design.hpp"
#include "runner.hpp"
#include "tfc/act.hpp"
#include "tfc/flight.hpp"
#include "tfc/imu_calibration.hpp"
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

struct Result {
  double rms_deg = 0.0;
  double max_deg = 0.0;
  double max_deg_settled = 0.0;     // the same, ignoring the first 3 s: ACT is in Standby (neutral gimbal) for the first second, and the vehicle is already flying
  double max_deg_liftoff = 0.0;     // the largest error in the first 3 s
  double final_altitude = 0.0;
  double final_tilt_deg = 0.0;
  uint32_t nominal_from = 0U;       // the frame ACT reached Nominal
  uint32_t ready_at = 0U;           // the first frame on which all three flight computers were ready for launch (0 if never, or no pad)
  bool calibrated = false;          // every computer's IMU calibration was ready at lift-off
  uint32_t safe_frames = 0U;
  uint32_t held_frames = 0U;        // frames in which the runner did not get ACT's frame
  uint32_t platform_saturated = 0U;
  bool finite = true;
  tfc::ActMode final_mode = tfc::ActMode::Standby;
  uint8_t final_excluded = 0U;
  uint8_t flags = 0U;
};

struct Loop {
  RunnerConfig cfg;
  SensorErrors sensors;
  tfc::EstimatorConfig estimator;           // the flight computers' estimator settings (use_accel false for a vehicle under thrust)
  uint32_t frames = 10000U;                  // frames of flight (after T-zero)
  uint32_t pad_frames = 0U;                  // frames on the pad before T-zero: the vehicle is clamped, the estimators calibrate, ACT goes Nominal; 0: released at the first frame, as before
  uint32_t node_b_dead_from = 0xFFFFFFFFU;   // node B stops sending (all its frames) from this frame
  uint32_t all_dead_from = 0xFFFFFFFFU;      // every flight computer stops sending from this frame (a total loss)
  uint32_t act_lost_from = 0xFFFFFFFFU;      // ACT's frames stop reaching the runner for `act_lost_for` frames from here
  uint32_t act_lost_for = 0U;
  bool sensor_fault_b = false;               // node B's gyro reads 15 dps too much
};

inline Result run(const Loop& lp) {
  RunnerConfig rcfg = lp.cfg;
  rcfg.start_held = lp.pad_frames > 0U;
  SimRunner runner(rcfg);
  const FlightTables tables = flight_tables(lp.cfg.design);
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator), tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator),
                                        tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator)};
  std::array<tfc::ImuCalibrator, 3> cal{};  // each computer calibrates its own IMU on the pad and subtracts the bias before it sends
  std::array<ImuModel, 3> imu{ImuModel(lp.sensors, 0x1234U), ImuModel(lp.sensors, 0x1235U), ImuModel(lp.sensors, 0x1236U)};  // the firmware's seeds: 0x1234 + node
  tfc::ActLogic act;
  tfc::ActRecord none{};
  act.boot(tfc::ResetCause::PowerOn, none);
  Result r;
  SimFrames pending = runner.start(0U);
  double sum_sq = 0.0;
  bool reached_nominal = false;
  const uint32_t total = lp.pad_frames + lp.frames;
  for (uint32_t k = 0; k < total; ++k) {
    const bool pad_now = k < lp.pad_frames;
    const uint32_t fk = pad_now ? 0U : k - lp.pad_frames;  // frames since T-zero
    if (lp.pad_frames > 0U && k == lp.pad_frames) {
      runner.release();
    }
    const tfc::DecodedVec3 dr = tfc::unpack_vec3(pending.f[0], tfc::kGyroLsbDps);
    const tfc::DecodedVec3 da = tfc::unpack_vec3(pending.f[1], tfc::kAccelLsbG);
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<tfc::Frame, 6> sensor{};
    std::array<bool, 3> alive{true, true, true};
    alive[1] = k < lp.node_b_dead_from;
    if (k >= lp.all_dead_from) {
      alive = {false, false, false};
    }
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 g;
      tfc::Vec3 a;
      imu[n].sample(dr.x, da.x, g, a);
      if (lp.pad_frames > 0U) {
        cal[n].set_pad(pad_now);
        g = cal[n].process(g);
      }
      if (lp.sensor_fault_b && n == 1U) {
        g.v[0] += 15.0F;
        g.v[1] += 15.0F;
      }
      sensor[n] = tfc::pack_gyro(n, g, seq);
      sensor[3U + n] = tfc::pack_accel(n, a, seq);
    }
    act.begin_frame();
    for (unsigned n = 0; n < 3U; ++n) {
      if (!alive[n]) {
        continue;
      }
      ff[n].begin_frame(k, alive[1] ? 0x07U : 0x05U);  // (a total loss never gets here)
      if (lp.pad_frames > 0U) {
        ff[n].set_mission(pad_now, fk);
      }
      for (unsigned m = 0; m < 3U; ++m) {
        if (!alive[m]) {
          continue;
        }
        (void)ff[n].on_frame(sensor[m]);
        (void)ff[n].on_frame(sensor[3U + m]);
      }
      (void)act.on_frame(tfc::pack_cmd(static_cast<uint8_t>(n), ff[n].step(), seq));
    }
    act.safe_request(false);
    if (pad_now && r.ready_at == 0U && ff[0].sensors_ok() && ff[1].sensors_ok() && ff[2].sensors_ok() && cal[0].ready() && cal[1].ready() && cal[2].ready()) {
      r.ready_at = k;
    }
    const tfc::ActOutput& out = act.end_frame();
    if (out.mode == tfc::ActMode::Nominal && !reached_nominal) {
      reached_nominal = true;
      r.nominal_from = k;
    }
    r.safe_frames += out.mode == tfc::ActMode::Safe ? 1U : 0U;
    const tfc::DecodedAct wire = tfc::unpack_act_out(tfc::pack_act_out(tfc::to_act_frame(out), seq));
    const bool lost = k >= lp.act_lost_from && k < lp.act_lost_from + lp.act_lost_for;
    r.held_frames += lost ? 1U : 0U;
    pending = runner.end_of_frame(k, lost ? nullptr : &wire.act);
    const Tilts t = runner.vehicle().tilts();
    if (pad_now) {
      continue;  // nothing flies on the pad
    }
    const tfc::Reference ref = tables.guidance.at(fk + 1U);
    const double ep = t.y_deg - static_cast<double>(ref.tilt_y_deg);
    const double ey = t.x_deg - static_cast<double>(ref.tilt_x_deg);
    const double e = std::fmax(std::fabs(ep), std::fabs(ey));
    r.max_deg = std::fmax(r.max_deg, e);
    (fk < 300U ? r.max_deg_liftoff : r.max_deg_settled) = std::fmax(fk < 300U ? r.max_deg_liftoff : r.max_deg_settled, e);
    sum_sq += (ep * ep) + (ey * ey);
    r.platform_saturated += runner.platform().saturated() ? 1U : 0U;
    r.finite = r.finite && std::isfinite(t.x_deg) && std::isfinite(t.y_deg) && std::isfinite(static_cast<double>(out.pitch_deg));
    r.final_mode = out.mode;
    r.final_excluded = out.excluded_nodes;
    if (!r.finite) {
      break;
    }
  }
  r.rms_deg = std::sqrt(sum_sq / (2.0 * lp.frames));
  r.calibrated = lp.pad_frames > 0U && cal[0].ready() && cal[1].ready() && cal[2].ready();
  r.final_altitude = runner.vehicle().altitude();
  r.final_tilt_deg = runner.vehicle().tilts().y_deg;
  const tfc::DecodedSimFlags fl = tfc::unpack_sim_flags(pending.f[4]);
  r.flags = fl.ok ? fl.s.flags : 0xFFU;
  return r;
}

}  // namespace sim
