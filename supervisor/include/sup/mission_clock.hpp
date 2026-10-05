// SPDX-License-Identifier: MIT
// The supervisor's clock of record (docs/MISSION_CLOCK.md, ADR-029, TFC-SUP-009, TFC-SUP-011): mission elapsed time (MET) since T-zero on the
// supervisor's own oscillator, kept across a reset by a battery-backed real-time clock, and the plausibility check of a flight computer's mission time
// against it. Portable and integer-only (the supervisor's small processor has no double-precision unit), and by TFC-SUP-001 it shares no code with
// `core/`: nothing here includes `tfc/`. The few protocol facts it needs (the frame period, the mission-frame encoding of SYNC) are restated below and
// pinned by a test against `core/`.
//
// The supervisor never decides whether a working node is right (TFC-SUP-010) and no control decision depends on this clock (TFC-SUP-014): the check
// reports, and nothing else.
#pragma once
#include <cstdint>

namespace sup {

// ---------------------------------------------------------------- facts restated from the flight protocol
constexpr uint32_t kFrameUs = 10000U;            // the frame period: 100 Hz
constexpr uint16_t kMissionFirstFlight = 1001U;  // the SYNC mission field at T-zero (1..1000 count down to it, 0 is not launched)
constexpr uint16_t kMissionSaturated = 65535U;   // the field stops here: 654 s after T-zero

// ---------------------------------------------------------------- a 64-bit tick count from a 32-bit hardware counter
// The hardware timer is 32 bits (71 minutes at 1 MHz); the mission is years. `extend` must be called at least once per wrap of the counter, which the
// supervisor's loop does thousands of times in that time.
class TickExtender {
 public:
  [[nodiscard]] uint64_t extend(uint32_t hw) noexcept {
    if (started_ && hw < last_) {
      ++high_;
    }
    started_ = true;
    last_ = hw;
    return (high_ << 32U) | hw;
  }

 private:
  uint64_t high_ = 0U;
  uint32_t last_ = 0U;
  bool started_ = false;
};

// ---------------------------------------------------------------- the battery-backed record of T-zero
// What is written to the real-time clock's backup memory when T-zero happens: the RTC's own seconds count at that moment. After a reset or a power cut
// the supervisor's timer starts again from zero, but the RTC kept counting, so MET is recovered from it, to a second.
constexpr uint32_t kMetMagic = 0x4D455452U;  // "METR"

struct MetRecord {
  uint32_t magic = 0U;
  uint64_t t0_rtc_s = 0U;  // RTC seconds at T-zero
  uint16_t crc = 0U;
};

// CRC-16/CCITT-FALSE over the magic and the T0 seconds, least significant byte first. The supervisor has its own copy of every routine (TFC-SUP-001).
[[nodiscard]] constexpr uint16_t met_crc(uint32_t magic, uint64_t t0_rtc_s) noexcept {
  uint16_t crc = 0xFFFFU;
  for (unsigned i = 0; i < 12U; ++i) {
    const uint8_t byte = i < 4U ? static_cast<uint8_t>(magic >> (8U * i)) : static_cast<uint8_t>(t0_rtc_s >> (8U * (i - 4U)));
    crc = static_cast<uint16_t>(crc ^ static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8U));
    for (unsigned b = 0; b < 8U; ++b) {
      crc = static_cast<uint16_t>((crc & 0x8000U) != 0U ? ((crc << 1U) ^ 0x1021U) : (crc << 1U));
    }
  }
  return crc;
}

[[nodiscard]] constexpr MetRecord make_record(uint64_t t0_rtc_s) noexcept { return MetRecord{kMetMagic, t0_rtc_s, met_crc(kMetMagic, t0_rtc_s)}; }

[[nodiscard]] constexpr bool record_valid(const MetRecord& r) noexcept { return r.magic == kMetMagic && r.crc == met_crc(r.magic, r.t0_rtc_s); }

// ---------------------------------------------------------------- mission elapsed time
enum class MetSource : uint8_t {
  None = 0,       // not launched, or no usable record
  Counted = 1,    // T-zero seen by this run of the supervisor: every tick counted
  Recovered = 2   // after a reset: T-zero's tick reconstructed from the RTC, good to about a second
};

struct ClockConfig {
  uint32_t tick_hz = 1000000U;  // the supervisor's timer rate; a multiple of 100 so that a frame is a whole number of ticks
};

class MissionClock {
 public:
  explicit MissionClock(ClockConfig cfg = {}) noexcept : cfg_(cfg) {}

  [[nodiscard]] bool config_ok() const noexcept { return cfg_.tick_hz >= 100U && (cfg_.tick_hz % 100U) == 0U; }

  // T-zero: the supervisor asserted its T0 line at tick `ticks`, when the RTC read `rtc_s`. Returns the record to store in battery-backed memory.
  // The first T-zero stands: a second call (the line is asserted once, so this is a repeat or a glitch) changes nothing.
  MetRecord latch_t0(uint64_t ticks, uint64_t rtc_s) noexcept {
    if (source_ == MetSource::None) {
      t0_ticks_ = ticks;
      source_ = MetSource::Counted;
      stored_ = make_record(rtc_s);
    }
    return stored_;
  }

  // After a reset: the timer counts from zero again, the RTC did not stop. Returns false (and stays unlaunched) if the record is not valid or its T-zero is in
  // the RTC's future (the RTC was set backwards, or the record is stale from another run).
  bool resume(const MetRecord& rec, uint64_t ticks_now, uint64_t rtc_now_s) noexcept {
    if (!record_valid(rec) || rec.t0_rtc_s > rtc_now_s) {
      return false;
    }
    const uint64_t elapsed_s = rtc_now_s - rec.t0_rtc_s;
    const uint64_t elapsed_ticks = elapsed_s * cfg_.tick_hz;
    if (elapsed_ticks > ticks_now) {
      // The timer has not been running as long as the mission has: T-zero is before this run's tick zero. Represent that with an offset.
      offset_ticks_ = elapsed_ticks - ticks_now;
      t0_ticks_ = 0U;
    } else {
      offset_ticks_ = 0U;
      t0_ticks_ = ticks_now - elapsed_ticks;
    }
    stored_ = rec;
    source_ = MetSource::Recovered;
    return true;
  }

  [[nodiscard]] MetSource source() const noexcept { return source_; }
  [[nodiscard]] bool launched() const noexcept { return source_ != MetSource::None; }
  // The error of the time since T-zero that comes only from how T-zero was known: none if counted, up to one second if recovered from the RTC.
  [[nodiscard]] uint32_t recovery_error_us() const noexcept { return source_ == MetSource::Recovered ? 1000000U : 0U; }

  // MET in ticks, 0 before T-zero.
  [[nodiscard]] uint64_t met_ticks(uint64_t ticks_now) const noexcept {
    if (!launched() || ticks_now + offset_ticks_ < t0_ticks_) {
      return 0U;
    }
    return ticks_now + offset_ticks_ - t0_ticks_;
  }

  // MET in microseconds without overflow over any mission: whole seconds first, then the remainder (ticks * 1e6 alone would overflow 64 bits at about
  // five years at 1 MHz).
  [[nodiscard]] uint64_t met_us(uint64_t ticks_now) const noexcept {
    const uint64_t t = met_ticks(ticks_now);
    const uint64_t s = t / cfg_.tick_hz;
    const uint64_t r = t % cfg_.tick_hz;
    return (s * 1000000U) + ((r * 1000000U) / cfg_.tick_hz);
  }

  [[nodiscard]] uint64_t met_frames(uint64_t ticks_now) const noexcept { return met_ticks(ticks_now) / (static_cast<uint64_t>(cfg_.tick_hz / 100U)); }

 private:
  ClockConfig cfg_;
  MetSource source_ = MetSource::None;
  uint64_t t0_ticks_ = 0U;
  uint64_t offset_ticks_ = 0U;  // set only when a recovered T-zero lies before this run's tick zero
  MetRecord stored_{};
};

// ---------------------------------------------------------------- the plausibility check (TFC-SUP-009)
// Does a flight computer's mission time (the SYNC mission field it reports, or a count of its FRAME pulses since T-zero) agree with MET? It allows a few frames
// plus a drift allowance of `ppm` of the elapsed time (the nodes' crystals are not the supervisor's), and it takes `persist` bad checks in a row to flag.
struct WatchConfig {
  uint32_t base_frames = 3U;
  uint32_t ppm = 200U;
  uint8_t persist = 3U;
};

enum class Verdict : uint8_t {
  NotLaunched = 0,  // T-zero has not happened: nothing to compare
  Ok = 1,
  Behind = 2,       // the node's time is less than MET allows
  Ahead = 3         // the node's time is more than MET allows
};

class MissionWatch {
 public:
  explicit MissionWatch(WatchConfig cfg = {}) noexcept : cfg_(cfg) {}

  // `field` is the mission field of SYNC as reported. Called about once a second. Returns this check's verdict; `flagged()` is the persistent result.
  Verdict check(const MissionClock& clock, uint64_t ticks_now, uint16_t field) noexcept {
    if (!clock.launched()) {
      bad_run_ = 0U;
      return Verdict::NotLaunched;
    }
    const uint64_t met = clock.met_frames(ticks_now);
    const uint64_t expect_wide = static_cast<uint64_t>(kMissionFirstFlight) + met;
    const uint64_t expect = expect_wide > kMissionSaturated ? static_cast<uint64_t>(kMissionSaturated) : expect_wide;
    const uint64_t allow = static_cast<uint64_t>(cfg_.base_frames) + ((met * cfg_.ppm) / 1000000U) + clock_slack(clock);
    const uint64_t got = field;
    Verdict v = Verdict::Ok;
    if (got + allow < expect) {
      v = Verdict::Behind;
    } else if (got > expect + allow) {
      v = Verdict::Ahead;
    }
    bad_run_ = v == Verdict::Ok ? 0U : (bad_run_ < 255U ? static_cast<uint8_t>(bad_run_ + 1U) : bad_run_);
    last_ = v;
    return v;
  }

  [[nodiscard]] bool flagged() const noexcept { return bad_run_ >= cfg_.persist; }
  [[nodiscard]] Verdict last() const noexcept { return last_; }

 private:
  // A T-zero recovered from the RTC is known to a second: allow the frames in it.
  static uint64_t clock_slack(const MissionClock& clock) noexcept { return clock.recovery_error_us() / kFrameUs; }

  WatchConfig cfg_;
  uint8_t bad_run_ = 0U;
  Verdict last_ = Verdict::NotLaunched;
};

}  // namespace sup
