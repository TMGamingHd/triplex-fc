// SPDX-License-Identifier: MIT
// Reset log (TFC-FDIR-042): a small record that survives a reset (the firmware keeps it in no-init RAM), so a node can tell
// how it came up, count its resets, and recognise a reset loop. A reset loop is `loop_boots` boots in a row, each caused by
// a fault reset (watchdog, brown-out, hard fault, pin or unknown) and each following a boot that ran for less than
// `short_boot_frames`. The core never touches the memory itself: it works on a record handed to it.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "tfc/crc8.hpp"

namespace tfc {

enum class ResetCause : uint8_t {
  Unknown = 0,
  PowerOn = 1,
  Pin = 2,
  Watchdog = 3,
  Software = 4,  // a deliberate reset: neither counted nor allowed to clear the count
  Brownout = 5,
  Fault = 6
};

constexpr uint32_t kResetMagic = 0x54465243U;  // "TFRC"

// Plain data with no initialisers on purpose: the firmware keeps one in no-init RAM, where a constructor would erase it at start-up.
// Write `ResetRecord{}` for a zeroed one.
struct ResetRecord {
  uint32_t magic;
  uint32_t boots;             // boots since the last power-on (saturating)
  uint32_t short_boots;       // consecutive fault-reset boots that followed a short boot
  uint32_t frames_last_boot;  // how long the current boot has run, in frames (updated while it runs)
  uint8_t last_cause;
  uint8_t crc;
};

struct ResetPolicy {
  uint32_t short_boot_frames = 6000U;  // a boot shorter than 60 s is short
  uint32_t loop_boots = 3U;            // this many short boots in a row is a loop
};

namespace detail {
constexpr uint32_t sat_inc(uint32_t v) noexcept { return v == 0xFFFFFFFFU ? v : v + 1U; }

inline uint8_t record_crc(const ResetRecord& r) noexcept {
  std::array<uint8_t, 17> b{};
  const std::array<uint32_t, 4> w{r.magic, r.boots, r.short_boots, r.frames_last_boot};
  for (std::size_t i = 0; i < w.size(); ++i) {
    for (std::size_t k = 0; k < 4U; ++k) {
      b[(i * 4U) + k] = static_cast<uint8_t>((w[i] >> (8U * k)) & 0xFFU);
    }
  }
  b[16] = r.last_cause;
  return crc8(b.data(), b.size());
}
}  // namespace detail

class ResetLog {
 public:
  explicit ResetLog(ResetRecord& record, ResetPolicy policy = ResetPolicy{}) noexcept : rec_(record), policy_(policy) {}

  // True if the record holds a sealed, undamaged log (after a power-on the RAM holds garbage).
  [[nodiscard]] bool valid() const noexcept {
    return rec_.magic == kResetMagic && rec_.crc == detail::record_crc(rec_);
  }

  // Call once at boot with the cause the hardware reports.
  void boot(ResetCause cause) noexcept {
    if (cause == ResetCause::PowerOn || !valid()) {
      rec_ = ResetRecord{};
      rec_.magic = kResetMagic;
      rec_.boots = 1U;
    } else {
      rec_.boots = detail::sat_inc(rec_.boots);
      if (cause != ResetCause::Software) {
        rec_.short_boots = rec_.frames_last_boot < policy_.short_boot_frames ? detail::sat_inc(rec_.short_boots) : 0U;
      }
    }
    rec_.frames_last_boot = 0U;
    rec_.last_cause = static_cast<uint8_t>(cause);
    seal();
  }

  // Call every so often while running (every 100 frames is plenty): records how long this boot has lasted.
  void running(uint32_t frames_since_boot) noexcept {
    rec_.frames_last_boot = frames_since_boot;
    seal();
  }

  // The node has been resetting again and again: it should stay out of the vote and say so.
  [[nodiscard]] bool loop_detected() const noexcept { return rec_.short_boots >= policy_.loop_boots; }
  [[nodiscard]] uint32_t boots() const noexcept { return rec_.boots; }
  [[nodiscard]] uint32_t short_boots() const noexcept { return rec_.short_boots; }
  [[nodiscard]] ResetCause last_cause() const noexcept { return static_cast<ResetCause>(rec_.last_cause); }

 private:
  void seal() noexcept { rec_.crc = detail::record_crc(rec_); }

  ResetRecord& rec_;
  ResetPolicy policy_;
};

}  // namespace tfc
