// SPDX-License-Identifier: MIT
// The platform driver's safety logic (TFC-PLAT-001, 002, 004; docs/PICO.md). The PC commands the platform's two tilts at 100 Hz; this decides what the servos are
// told, whatever the PC says:
//   limit     a tilt beyond the travel is clamped to it (and the command reported as saturated, PLAT-004)
//   rate      the output moves toward the target by at most `rate_limit_dps` (PLAT-001), so a step in the command is a ramp
//   timeout   no command for `hold_after_ms`: hold where it is; for `level_after_ms`: go to level at `level_rate_dps` (PLAT-002)
//   validity  a command that is not a number is rejected, counted, and does not restart the timeout
// And the map from a tilt to a servo pulse. It does not know about USB, PWM or time: the caller passes the elapsed milliseconds.
// No heap, no exceptions, no RTTI. Deterministic: only + - * / and comparisons.
#pragma once
#include <cstdint>

namespace tfc {

struct PlatformConfig {
  float limit_deg = 45.0F;           // the travel the platform may be driven to, each plane
  float rate_limit_dps = 300.0F;     // the fastest the output may move
  uint16_t hold_after_ms = 100U;     // no command for this long: hold
  uint16_t level_after_ms = 1000U;   // no command for this long: bring to level
  float level_rate_dps = 30.0F;      // how fast it goes to level
};

struct PlatformOutput {
  float x_deg = 0.0F;       // the tilt the servos are told, about X (the yaw plane)
  float y_deg = 0.0F;       // about Y (the pitch plane)
  bool saturated = false;   // the target is beyond the travel
  bool holding = false;
  bool levelling = false;
};

class PlatformDriver {
 public:
  PlatformDriver() noexcept = default;
  explicit PlatformDriver(const PlatformConfig& cfg) noexcept : cfg_(cfg) {}

  // A command from the PC. False (and nothing changes) if a value is not a number or is wildly out of range; the timeout is not restarted.
  bool command(float x_deg, float y_deg) noexcept {
    if (!finite(x_deg) || !finite(y_deg)) {
      ++rejected_;
      return false;
    }
    target_x_ = x_deg;
    target_y_ = y_deg;
    age_ms_ = 0U;
    have_ = true;
    return true;
  }

  // Advance by `dt_ms` milliseconds: apply the timeout, the limit and the rate limit.
  void tick(uint16_t dt_ms) noexcept {
    age_ms_ = age_ms_ > 0xFFFFU - dt_ms ? 0xFFFFU : static_cast<uint16_t>(age_ms_ + dt_ms);
    const float dt = static_cast<float>(dt_ms) * 0.001F;
    float gx = clamp(target_x_);
    float gy = clamp(target_y_);
    float rate = cfg_.rate_limit_dps;
    out_.saturated = have_ && age_ms_ < cfg_.hold_after_ms && (magnitude(target_x_) > cfg_.limit_deg || magnitude(target_y_) > cfg_.limit_deg);
    out_.holding = false;
    out_.levelling = false;
    if (!have_ || age_ms_ >= cfg_.level_after_ms) {
      gx = 0.0F;
      gy = 0.0F;
      rate = cfg_.level_rate_dps;
      out_.levelling = true;
    } else if (age_ms_ >= cfg_.hold_after_ms) {
      gx = out_.x_deg;
      gy = out_.y_deg;
      out_.holding = true;
    }
    out_.x_deg = approach(out_.x_deg, gx, rate * dt);
    out_.y_deg = approach(out_.y_deg, gy, rate * dt);
  }

  [[nodiscard]] const PlatformOutput& output() const noexcept { return out_; }
  [[nodiscard]] uint16_t command_age_ms() const noexcept { return age_ms_; }
  [[nodiscard]] uint32_t rejected() const noexcept { return rejected_; }

 private:
  static bool finite(float v) noexcept { return v > -1.0e6F && v < 1.0e6F; }  // false for NaN and infinities
  static float magnitude(float v) noexcept { return v < 0.0F ? -v : v; }
  [[nodiscard]] float clamp(float v) const noexcept {
    return v > cfg_.limit_deg ? cfg_.limit_deg : (v < -cfg_.limit_deg ? -cfg_.limit_deg : v);
  }
  static float approach(float cur, float goal, float step) noexcept {
    const float d = goal - cur;
    return d > step ? cur + step : (d < -step ? cur - step : goal);
  }

  PlatformConfig cfg_{};
  PlatformOutput out_{};
  float target_x_ = 0.0F;
  float target_y_ = 0.0F;
  uint16_t age_ms_ = 0U;
  bool have_ = false;
  uint32_t rejected_ = 0U;
};

// A tilt to a servo pulse: neutral plus a scale per degree, with a sign and a trim, clamped to a safe pulse range. The D85MG's pulse range is 850 to 2350 us over 145
// degrees (ServoCity, parts sheet row 29), about 10.3 us per degree; +-45 degrees is then +-466 us, inside it.
struct ServoMap {
  float neutral_us = 1500.0F;
  float us_per_deg = 10.34F;
  float sign = 1.0F;           // -1 reverses the axis
  float trim_deg = 0.0F;       // added to the angle (mounting offset)
  float min_us = 900.0F;       // the pulse is never outside this range
  float max_us = 2100.0F;

  [[nodiscard]] float pulse_us(float angle_deg) const noexcept {
    const float p = neutral_us + (sign * (angle_deg + trim_deg) * us_per_deg);
    return p < min_us ? min_us : (p > max_us ? max_us : p);
  }
};

}  // namespace tfc
