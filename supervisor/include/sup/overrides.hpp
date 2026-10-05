// SPDX-License-Identifier: MIT
// The hardware overrides' read-only sense lines (docs/design/HARDWARE_OVERRIDE.md sections 5 and 7, TFC-HWO-005 and HWO-007): which override switches are engaged, which have been seen to work this session,
// and the operator's acknowledgement that a run may start with one engaged. A sense line is **never** an input to a control decision about the units: the one thing it does is to refuse a launch
// while an override is engaged that the operator has not acknowledged (the guard against mode confusion, G5).
//
// Six lines: bit 0 H1 servo-rail E-stop, 1 H2 FORCE-SAFE, 2 H3 PLATFORM-LEVEL, 3 H4 INJECTOR-DISARM, 4 H5 SUPERVISOR-DISARM, 5 H6/H7 any node power-kill (diode-OR'd). A set bit means engaged.
// `fitted` says which lines exist (the overrides' parts are ordered separately: an unfitted line is ignored). A line must hold its level for the debounce time to count.
// Integer-only, no heap, deterministic.
#pragma once
#include <array>
#include <cstdint>

namespace sup {

constexpr unsigned kOverrideLines = 6U;
constexpr uint8_t kOverrideMask = static_cast<uint8_t>((1U << kOverrideLines) - 1U);

class OverrideSense {
 public:
  OverrideSense(uint8_t fitted, uint64_t debounce_ticks) noexcept : debounce_(debounce_ticks), fitted_(static_cast<uint8_t>(fitted & kOverrideMask)) {}

  // One look at the lines (`raw`: bit set = engaged) at tick `now`. Returns the lines whose debounced state changed.
  uint8_t sample(uint8_t raw, uint64_t now) noexcept {
    uint8_t changed = 0U;
    for (unsigned i = 0; i < kOverrideLines; ++i) {
      const uint8_t bit = static_cast<uint8_t>(1U << i);
      if ((fitted_ & bit) == 0U) {
        continue;
      }
      const bool level = (raw & bit) != 0U;
      if (level != candidate(i)) {
        cand_ = static_cast<uint8_t>(level ? (cand_ | bit) : (cand_ & ~bit));
        since_[i] = now;
      }
      if (candidate(i) != ((engaged_ & bit) != 0U) && now - since_[i] >= debounce_) {
        engaged_ = static_cast<uint8_t>(engaged_ ^ bit);
        changed = static_cast<uint8_t>(changed | bit);
        if ((engaged_ & bit) == 0U) {  // released, so it had been engaged: the switch has been seen to work, and an acknowledgement ends with the engagement
          tested_ = static_cast<uint8_t>(tested_ | bit);
          ack_ = static_cast<uint8_t>(ack_ & ~bit);
        }
      }
    }
    return changed;
  }

  [[nodiscard]] uint8_t engaged() const noexcept { return engaged_; }
  [[nodiscard]] uint8_t tested() const noexcept { return tested_; }
  // The fitted overrides that have not been seen both engaged and released since power-up: the pre-session check has not covered them (HWO-007).
  [[nodiscard]] uint8_t untested() const noexcept { return static_cast<uint8_t>(fitted_ & ~tested_); }
  // Engaged and not acknowledged: what blocks a launch.
  [[nodiscard]] uint8_t unacknowledged() const noexcept { return static_cast<uint8_t>(engaged_ & ~ack_); }
  [[nodiscard]] uint8_t fitted() const noexcept { return fitted_; }

  // The operator accepts the overrides that are engaged now; a line engaged later, or the same one after a release, needs its own acknowledgement.
  void acknowledge() noexcept { ack_ = engaged_; }

 private:
  [[nodiscard]] bool candidate(unsigned i) const noexcept { return ((cand_ >> i) & 1U) != 0U; }

  uint64_t debounce_;
  std::array<uint64_t, kOverrideLines> since_{};
  uint8_t fitted_;
  uint8_t cand_ = 0U;         // the level each line has held most recently
  uint8_t engaged_ = 0U;      // the debounced state
  uint8_t tested_ = 0U;
  uint8_t ack_ = 0U;
};

}  // namespace sup
