// SPDX-License-Identifier: MIT
// Attitude controller and guidance: from the estimated attitude and a reference to a thrust-vector (gimbal) command in each of
// the two planes. A PID with the rate taken from the estimator (so it is a PD on the angle with the gyro as the derivative),
// a limit on the gimbal angle, a limit on how fast the command may change per frame, and an integrator that stops while the
// command is saturated and the error would push it further (anti-windup). With the estimator's `valid` flag cleared the last
// command is held and the integrator does not move (a trustworthy rate is the premise of the derivative).
// Plane mapping: the pitch plane is the platform's tilt about Y, the yaw plane its tilt about X (estimator.hpp).
// No heap, no exceptions, no RTTI. Deterministic: only + - * / comparisons.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/estimator.hpp"
#include "tfc/protocol.hpp"

namespace tfc {

struct ControllerConfig {
  float kp = 2.2F;                 // gimbal degrees per degree of attitude error
  float kd = 1.6F;                 // gimbal degrees per degree/second of rate error
  float ki = 0.4F;                 // gimbal degrees per degree of error, per second
  float i_limit_deg = 6.0F;        // the integrator's contribution is limited to this (holding 20 degrees against the divergence needs 4.4)
  float gimbal_limit_deg = 8.0F;   // the largest gimbal angle commanded
  float slew_deg_per_frame = 0.6F; // the largest change of the command in one frame (60 degrees per second at 100 Hz)
};

struct Reference {
  float tilt_x_deg = 0.0F;  // yaw plane
  float tilt_y_deg = 0.0F;  // pitch plane
  float rate_x_dps = 0.0F;
  float rate_y_dps = 0.0F;
};

// A piecewise-linear reference over time (a pitch program): up to sixteen (frame, angle) points, in increasing frame order. Before the
// first point it holds the first value, after the last it holds the last. Both planes have their own table.
struct GuidancePoint {
  uint32_t frame = 0U;
  float deg = 0.0F;
};

class Guidance {
 public:
  static constexpr unsigned kMaxPoints = 16U;

  Guidance() noexcept = default;

  // Add a point to the table of one plane (0 = yaw plane / tilt_x, 1 = pitch plane / tilt_y). False if the table is full, the
  // plane does not exist, or the frame does not increase.
  bool add(unsigned plane, uint32_t frame, float deg) noexcept {
    if (plane >= 2U || n_[plane] >= kMaxPoints) {
      return false;
    }
    if (n_[plane] > 0U && frame <= pt_[plane][n_[plane] - 1U].frame) {
      return false;
    }
    pt_[plane][n_[plane]] = GuidancePoint{frame, deg};
    ++n_[plane];
    return true;
  }

  [[nodiscard]] Reference at(uint32_t frame) const noexcept {
    Reference r;
    r.tilt_x_deg = eval(0U, frame, r.rate_x_dps);
    r.tilt_y_deg = eval(1U, frame, r.rate_y_dps);
    return r;
  }

  // The points of one plane's table, for tools that print or check it (a plane or index out of range gives size 0 or a zero point).
  [[nodiscard]] unsigned size(unsigned plane) const noexcept { return plane < 2U ? n_[plane] : 0U; }
  [[nodiscard]] GuidancePoint point(unsigned plane, unsigned i) const noexcept {
    return (plane < 2U && i < n_[plane]) ? pt_[plane][i] : GuidancePoint{};
  }

 private:
  // The value and slope (deg per second, at 100 frames per second) of one plane's table at `frame`.
  [[nodiscard]] float eval(unsigned plane, uint32_t frame, float& rate_dps) const noexcept {
    rate_dps = 0.0F;
    const unsigned n = n_[plane];
    if (n == 0U) {
      return 0.0F;
    }
    const std::array<GuidancePoint, kMaxPoints>& p = pt_[plane];
    if (frame <= p[0].frame) {
      return p[0].deg;
    }
    for (unsigned i = 1; i < n; ++i) {
      if (frame <= p[i].frame) {
        const float span = static_cast<float>(p[i].frame - p[i - 1U].frame);
        const float f = static_cast<float>(frame - p[i - 1U].frame) / span;
        rate_dps = (p[i].deg - p[i - 1U].deg) / span * 100.0F;
        return p[i - 1U].deg + (f * (p[i].deg - p[i - 1U].deg));
      }
    }
    return p[n - 1U].deg;
  }

  std::array<std::array<GuidancePoint, kMaxPoints>, 2> pt_{};
  std::array<unsigned, 2> n_{0U, 0U};
};

// Controller gains that change through the flight: the plant (thrust, mass, inertia, dynamic pressure) changes by a large factor, so gains are
// designed for each instant of the nominal trajectory and interpolated in time. Up to sixteen (frame, kp, kd, ki) points in increasing frame order;
// before the first point the first values hold, after the last the last.
struct ControllerGains {
  float kp = 2.2F;
  float kd = 1.6F;
  float ki = 0.4F;
};

class GainSchedule {
 public:
  static constexpr unsigned kMaxPoints = 16U;

  bool add(uint32_t frame, const ControllerGains& g) noexcept {
    if (n_ >= kMaxPoints || (n_ > 0U && frame <= frame_[n_ - 1U])) {
      return false;
    }
    frame_[n_] = frame;
    gain_[n_] = g;
    ++n_;
    return true;
  }

  [[nodiscard]] unsigned size() const noexcept { return n_; }

  [[nodiscard]] ControllerGains at(uint32_t frame) const noexcept {
    if (n_ == 0U) {
      return ControllerGains{};
    }
    if (frame <= frame_[0]) {
      return gain_[0];
    }
    for (unsigned i = 1; i < n_; ++i) {
      if (frame <= frame_[i]) {
        const float f = static_cast<float>(frame - frame_[i - 1U]) / static_cast<float>(frame_[i] - frame_[i - 1U]);
        const ControllerGains& a = gain_[i - 1U];
        const ControllerGains& b = gain_[i];
        return ControllerGains{a.kp + (f * (b.kp - a.kp)), a.kd + (f * (b.kd - a.kd)), a.ki + (f * (b.ki - a.ki))};
      }
    }
    return gain_[n_ - 1U];
  }

  // The points, for tools that print or check the table (an index out of range gives a zero point).
  [[nodiscard]] uint32_t frame_at(unsigned i) const noexcept { return i < n_ ? frame_[i] : 0U; }
  [[nodiscard]] ControllerGains gains_at(unsigned i) const noexcept { return i < n_ ? gain_[i] : ControllerGains{}; }

 private:
  std::array<uint32_t, kMaxPoints> frame_{};
  std::array<ControllerGains, kMaxPoints> gain_{};
  unsigned n_ = 0U;
};

class Controller {
 public:
  Controller() noexcept = default;
  explicit Controller(const ControllerConfig& cfg) noexcept : cfg_(cfg) {}

  // One step of dt seconds. Returns the gimbal command (degrees) in the pitch and yaw planes; the digest field is left for the
  // caller to fill from the estimator and `digest()`.
  [[nodiscard]] Command step(const Attitude& att, const Reference& ref, float dt) noexcept {
    if (!att.valid) {
      ++holds_;
    } else {
      const std::array<float, 2> err{ref.tilt_y_deg - att.tilt_y_deg, ref.tilt_x_deg - att.tilt_x_deg};
      const std::array<float, 2> rate_err{ref.rate_y_dps - att.rate_y_dps, ref.rate_x_dps - att.rate_x_dps};
      for (unsigned a = 0; a < 2U; ++a) {
        out_[a] = axis(a, err[a], rate_err[a], dt);
      }
    }
    Command c;
    c.pitch_deg = out_[0];
    c.yaw_deg = out_[1];
    return c;
  }

  // What the controller carries from one step to the next (the gains come from the schedule): exchanged between the replicas for the state resynchronisation (docs/design/RESYNC.md).
  struct State {
    std::array<float, 2> out{0.0F, 0.0F};
    std::array<float, 2> integ{0.0F, 0.0F};
  };
  [[nodiscard]] State state() const noexcept { return State{out_, integ_}; }
  void set_state(const State& s) noexcept {
    out_ = s.out;
    integ_ = s.integ;
  }

  // A 16-bit fingerprint of the quantised outputs and integrators, to be combined with the estimator's digest.
  [[nodiscard]] uint16_t digest() const noexcept {
    uint32_t h = 2166136261U;
    for (const float v : {out_[0], out_[1], integ_[0], integ_[1]}) {
      h = (h ^ static_cast<uint32_t>(static_cast<int32_t>(quantize(v, 0.001F)))) * 16777619U;
    }
    return static_cast<uint16_t>((h ^ (h >> 16)) & 0xFFFFU);
  }

  // Replace the three gains (from a GainSchedule, once per frame).
  void set_gains(const ControllerGains& g) noexcept {
    cfg_.kp = g.kp;
    cfg_.kd = g.kd;
    cfg_.ki = g.ki;
  }

  [[nodiscard]] uint32_t holds() const noexcept { return holds_; }
  [[nodiscard]] uint32_t saturated_frames() const noexcept { return saturated_; }
  [[nodiscard]] float output_pitch_deg() const noexcept { return out_[0]; }
  [[nodiscard]] float output_yaw_deg() const noexcept { return out_[1]; }

 private:
  static constexpr float clamp(float v, float lim) noexcept { return v > lim ? lim : (v < -lim ? -lim : v); }

  float axis(unsigned a, float err, float rate_err, float dt) noexcept {
    const float unsat = (cfg_.kp * err) + (cfg_.kd * rate_err) + integ_[a];
    const float lim = cfg_.gimbal_limit_deg;
    const bool pushing_further = (unsat > lim && err > 0.0F) || (unsat < -lim && err < 0.0F);
    if (pushing_further) {
      ++saturated_;  // anti-windup: do not integrate while saturated and the error would push it further
    } else {
      integ_[a] = clamp(integ_[a] + (cfg_.ki * err * dt), cfg_.i_limit_deg);
    }
    const float target = clamp(unsat, lim);
    const float step = clamp(target - out_[a], cfg_.slew_deg_per_frame);
    return out_[a] + step;
  }

  ControllerConfig cfg_{};
  std::array<float, 2> out_{0.0F, 0.0F};    // the command last issued: pitch plane, yaw plane
  std::array<float, 2> integ_{0.0F, 0.0F};  // the integrators, degrees
  uint32_t holds_ = 0U;
  uint32_t saturated_ = 0U;
};

}  // namespace tfc
