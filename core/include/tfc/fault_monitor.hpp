// SPDX-License-Identifier: MIT
// Fault detection / isolation / recovery (FDIR) building blocks.
// All state is fixed-size; update() is O(1) and allocation-free.
#pragma once
#include <cstdint>

namespace tfc {

// Number of set bits. A fixed 32-iteration loop: the bound is evident to the reader and to the analysers.
[[nodiscard]] constexpr unsigned popcount32(uint32_t v) noexcept {
  unsigned n = 0;
  for (unsigned i = 0; i < 32U; ++i) {
    n += (v >> i) & 1U;
  }
  return n;
}

// M-out-of-N persistence filter with latch, probation-based reintegration and a
// permanent-failure limit. A "bad" sample is a per-frame miscompare, timeout or
// CRC/sequence error for one channel.
//
//   * A single glitch never latches (needs M bad samples inside the last N).
//   * Once latched, the channel is excluded from voting until reintegrated.
//   * Reintegration is requested explicitly (operator/ground command) and then
//     needs `clean_needed` consecutive good frames; any bad frame restarts it.
//   * After `max_latches` latch events the channel is permanently failed.
class ChannelMonitor {
 public:
  constexpr ChannelMonitor(uint8_t m, uint8_t n, uint16_t clean_needed,
                           uint8_t max_latches) noexcept
      : m_(m), window_mask_(n >= 32U ? 0xFFFFFFFFU : ((1U << n) - 1U)),
        clean_needed_(clean_needed), max_latches_(max_latches) {}

  // Feed one frame's verdict. Returns true on the frame the channel latches.
  bool update(bool bad) noexcept {
    history_ = (history_ << 1) | (bad ? 1U : 0U);
    if (!latched_) {
      if (bad) {
        ++bad_total_;
      }
      if (popcount32(history_ & window_mask_) >= m_) {
        latched_ = true;
        ++latch_count_;
        clean_run_ = 0;
        probation_ = false;
        if (latch_count_ >= max_latches_) {
          permanent_ = true;
        }
        return true;
      }
      return false;
    }
    if (probation_ && !permanent_) {
      clean_run_ = bad ? 0U : static_cast<uint16_t>(clean_run_ + 1U);
      if (clean_run_ >= clean_needed_) {
        latched_ = false;
        probation_ = false;
        history_ = 0U;
        clean_run_ = 0U;
      }
    }
    return false;
  }

  // Ground/operator command: start counting clean frames toward reintegration.
  void request_reintegration() noexcept {
    if (latched_ && !permanent_) {
      probation_ = true;
      clean_run_ = 0U;
    }
  }

  // Supervisor action (RedundancyManager): the node proved itself on probation, so forget the latch
  // and the bad history. The latch count is kept (it is the strike record).
  void force_unlatch() noexcept {
    latched_ = false;
    probation_ = false;
    permanent_ = false;
    history_ = 0U;
    clean_run_ = 0U;
  }

  constexpr bool latched() const noexcept { return latched_; }
  constexpr bool permanent() const noexcept { return permanent_; }
  constexpr uint8_t latch_count() const noexcept { return latch_count_; }
  constexpr uint32_t bad_total() const noexcept { return bad_total_; }

 private:
  uint8_t m_;
  uint32_t window_mask_;
  uint16_t clean_needed_;
  uint8_t max_latches_;
  uint32_t history_ = 0U;
  uint32_t bad_total_ = 0U;
  uint16_t clean_run_ = 0U;
  uint8_t latch_count_ = 0U;
  bool latched_ = false;
  bool probation_ = false;
  bool permanent_ = false;
};

// Leaky "alpha-count" for intermittent faults (Bondavalli et al.): the score goes up by 1 on every
// bad observation and is multiplied by K (0 < K < 1) on every good one, and a node whose score
// reaches the threshold is diagnosed as intermittently (or permanently) faulty. Unlike an M-of-N
// window it remembers bad frames that are spread out: with K = 0.9 and a threshold of 3 a node that
// is bad one frame in three crosses it within about a dozen frames, while a single glitch, a
// two-frame burst, or one bad frame in ten never does. A three-frame burst crosses it on the third
// frame, exactly when 3-of-5 does.
//
// Fixed-point (Q16) so every replica computes bit-identical scores on any platform, with no float
// rounding in a verdict that all flight computers must agree on.
class AlphaCount {
 public:
  // k is held below 1 (a factor of 1 or more would make the score grow on good frames); a NaN or negative constant is 0.
  constexpr AlphaCount(float k, float threshold) noexcept : k_(to_q16(k) > kMaxK ? kMaxK : to_q16(k)), threshold_(to_q16(threshold)) {}

  // Feed one observation. Returns true while the score is at or above the threshold (never when the
  // threshold is 0, which disables the detector).
  bool update(bool bad) noexcept {
    if (bad) {
      if (score_ <= 0xFFFFFFFFU - kOne) {
        score_ += kOne;
      }
    } else {
      score_ = static_cast<uint32_t>((static_cast<uint64_t>(score_) * k_) >> 16U);
    }
    return threshold_ != 0U && score_ >= threshold_;
  }

  void reset() noexcept { score_ = 0U; }
  constexpr float score() const noexcept { return static_cast<float>(score_) / 65536.0F; }

 private:
  static constexpr uint32_t kOne = 65536U;
  static constexpr uint32_t kMaxK = kOne - 1U;  // the largest decay factor: just under 1.0
  // Q16 conversion that cannot overflow: NaN and anything not above 0 give 0, anything that does not fit saturates.
  static constexpr uint32_t to_q16(float v) noexcept {
    if (!(v > 0.0F)) {
      return 0U;
    }
    if (v >= 65535.0F) {
      return 0xFFFFFFFFU;
    }
    const float x = v * 65536.0F;
    const uint32_t whole = static_cast<uint32_t>(x);
    return (x - static_cast<float>(whole) >= 0.5F) ? whole + 1U : whole;  // round to nearest
  }
  uint32_t k_;
  uint32_t threshold_;
  uint32_t score_ = 0U;
};

// Flags a sensor whose output is bit-identical for `limit` consecutive frames.
// A live MEMS gyro/accelerometer always shows LSB-level noise, so a frozen value
// is a strong stuck-at signature (a real bias-free vehicle at rest still dithers).
class StuckDetector {
 public:
  explicit constexpr StuckDetector(uint16_t limit) noexcept : limit_(limit) {}
  bool update(int32_t raw) noexcept {
    if (have_prev_ && raw == prev_) {
      if (run_ < 0xFFFFU) {
        ++run_;
      }
    } else {
      run_ = 0U;
    }
    prev_ = raw;
    have_prev_ = true;
    return run_ >= limit_;
  }

 private:
  uint16_t limit_;
  int32_t prev_ = 0;
  uint16_t run_ = 0U;
  bool have_prev_ = false;
};

// System redundancy mode derived from the number of healthy flight computers.
enum class Mode : uint8_t { Triplex = 3, Duplex = 2, Simplex = 1, Safe = 0 };

constexpr Mode mode_from_healthy(unsigned healthy) noexcept {
  return healthy >= 3U ? Mode::Triplex
                       : healthy == 2U ? Mode::Duplex
                                       : healthy == 1U ? Mode::Simplex : Mode::Safe;
}

}  // namespace tfc
