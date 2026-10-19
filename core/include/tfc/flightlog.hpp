// SPDX-License-Identifier: MIT
// Flight log: the last frames a computer ran and what a fatal fault knew, kept in no-init RAM so that they are still there after the reset that follows a fault (docs/design/FUTURE_WORK.md 6b item 12). A flight
// computer that cannot say why it reset has not been debugged; the reset log (resetlog.hpp) says how it came up and how often, this says what it was doing.
//
// A ring of `N` records of one frame each (the frame number, the mode, who voted, who was out, the flags, the voted output and the command, in hundredths of a degree), each with its own CRC-8, so a record that a
// reset or a stray write damaged is skipped and said so, and a ring that was never written (a power-on: the RAM is garbage) is recognised by its magic number and its header's CRC and starts empty. And one
// record of the fault itself: its reason, the program counter and the link register, and the frame it happened in. The core never touches the memory: it works on a store the firmware hands it, kept in a
// `__noinit` variable (plain data with no initialisers, as for the reset log).
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "tfc/crc8.hpp"

namespace tfc {

constexpr uint32_t kFlightLogMagic = 0x544C4F47U;   // "TLOG"
constexpr uint32_t kFaultMagic = 0x54464C54U;       // "TFLT"

constexpr uint8_t kLogSafeRequest = 1U;   // flags of a record
constexpr uint8_t kLogBusAlarm = 2U;
constexpr uint8_t kLogUnresolved = 4U;
constexpr uint8_t kLogHeld = 8U;

struct FlightRecord {
  uint32_t frame;
  uint8_t mode;
  uint8_t valid_mask;
  uint8_t latched_mask;
  uint8_t flags;
  int16_t output;    // the voted value of the first channel, hundredths
  int16_t cmd_pitch; // the command this computer sent, hundredths of a degree
  int16_t cmd_yaw;
  uint8_t crc;
};

// What a fatal fault knew: Zephyr's reason code, the program counter and the link register of the faulting context, and the frame the computer was in.
struct FaultRecord {
  uint32_t magic;
  uint32_t reason;
  uint32_t pc;
  uint32_t lr;
  uint32_t frame;
  uint8_t crc;
};

// Plain data with no initialisers on purpose: the firmware keeps one in no-init RAM. Write `FlightLogStore<N>{}` for a zeroed one.
template <std::size_t N>
struct FlightLogStore {
  uint32_t magic;
  uint32_t head;     // the slot the next record goes in
  uint32_t count;    // how many records are held (at most N)
  uint8_t head_crc;
  FlightRecord rec[N];
  FaultRecord fault;
};

namespace logdetail {

// A value in hundredths, saturated to what 16 bits hold (a NaN is zero: a number that is not one has no hundredths).
[[nodiscard]] inline int16_t hundredths(float v) noexcept {
  if (std::isnan(v)) {
    return 0;
  }
  const float x = v * 100.0F;
  if (x >= 32767.0F) {
    return 32767;
  }
  if (x <= -32768.0F) {
    return -32768;
  }
  return static_cast<int16_t>(x >= 0.0F ? x + 0.5F : x - 0.5F);
}

inline void put32(uint8_t* p, uint32_t v) noexcept {
  for (std::size_t i = 0; i < 4U; ++i) {
    p[i] = static_cast<uint8_t>((v >> (8U * i)) & 0xFFU);
  }
}

[[nodiscard]] inline uint8_t record_crc(const FlightRecord& r) noexcept {
  std::array<uint8_t, 14> b{};
  put32(b.data(), r.frame);
  b[4] = r.mode;
  b[5] = r.valid_mask;
  b[6] = r.latched_mask;
  b[7] = r.flags;
  const std::array<int16_t, 3> v{r.output, r.cmd_pitch, r.cmd_yaw};
  for (std::size_t i = 0; i < v.size(); ++i) {
    b[8U + (2U * i)] = static_cast<uint8_t>(static_cast<uint16_t>(v[i]) & 0xFFU);
    b[9U + (2U * i)] = static_cast<uint8_t>((static_cast<uint16_t>(v[i]) >> 8U) & 0xFFU);
  }
  return crc8(b.data(), b.size());
}

[[nodiscard]] inline uint8_t fault_crc(const FaultRecord& f) noexcept {
  std::array<uint8_t, 20> b{};
  put32(b.data(), f.magic);
  put32(b.data() + 4, f.reason);
  put32(b.data() + 8, f.pc);
  put32(b.data() + 12, f.lr);
  put32(b.data() + 16, f.frame);
  return crc8(b.data(), b.size());
}

[[nodiscard]] inline uint8_t head_crc(uint32_t magic, uint32_t head, uint32_t count) noexcept {
  std::array<uint8_t, 12> b{};
  put32(b.data(), magic);
  put32(b.data() + 4, head);
  put32(b.data() + 8, count);
  return crc8(b.data(), b.size());
}

}  // namespace logdetail

template <std::size_t N>
class FlightLog {
  static_assert(N >= 1U && N <= 4096U, "a ring of a useful size");

 public:
  explicit FlightLog(FlightLogStore<N>& s) noexcept : s_(s) {}

  // At boot: a store whose header is intact keeps what it holds (the previous run's last frames); anything else (a power-on's garbage) starts empty. Returns whether there was a log to keep.
  bool boot() noexcept {
    const bool ok = s_.magic == kFlightLogMagic && s_.count <= N && s_.head < N && s_.head_crc == logdetail::head_crc(s_.magic, s_.head, s_.count);
    if (!ok) {
      s_ = FlightLogStore<N>{};
      s_.magic = kFlightLogMagic;
      seal();
    }
    return ok;
  }

  // One frame. Overwrites the oldest when the ring is full.
  void push(uint32_t frame, uint8_t mode, uint8_t valid_mask, uint8_t latched_mask, uint8_t flags, float output, float cmd_pitch_deg, float cmd_yaw_deg) noexcept {
    FlightRecord r{frame, mode, valid_mask, latched_mask, flags, logdetail::hundredths(output), logdetail::hundredths(cmd_pitch_deg), logdetail::hundredths(cmd_yaw_deg), 0U};
    r.crc = logdetail::record_crc(r);
    if (s_.head >= N) {  // an upset in the index since boot(), which is where the header is checked: never write outside the ring (finding 6 of the review of core/)
      s_.head = 0U;
    }
    s_.rec[s_.head] = r;
    s_.head = (s_.head + 1U) % static_cast<uint32_t>(N);
    s_.count = s_.count < N ? s_.count + 1U : static_cast<uint32_t>(N);
    seal();
  }

  [[nodiscard]] std::size_t size() const noexcept { return s_.count; }

  // The `age`-th newest record (0 the newest). `ok` is false when there is none that old, or when the record was damaged (its CRC does not match): the caller skips it and says so.
  [[nodiscard]] FlightRecord get(std::size_t age, bool& ok) const noexcept {
    ok = false;
    if (age >= s_.count) {
      return FlightRecord{};
    }
    const std::size_t slot = (static_cast<std::size_t>(s_.head) + N - 1U - age) % N;
    const FlightRecord r = s_.rec[slot];
    ok = r.crc == logdetail::record_crc(r);
    return r;
  }

  // What a fatal fault knew, kept until the next one; the frame it happened in is the one the computer had reached.
  void record_fault(uint32_t reason, uint32_t pc, uint32_t lr, uint32_t frame) noexcept {
    FaultRecord f{kFaultMagic, reason, pc, lr, frame, 0U};
    f.crc = logdetail::fault_crc(f);
    s_.fault = f;
  }
  [[nodiscard]] bool has_fault() const noexcept { return s_.fault.magic == kFaultMagic && s_.fault.crc == logdetail::fault_crc(s_.fault); }
  [[nodiscard]] FaultRecord fault() const noexcept { return s_.fault; }
  void clear_fault() noexcept { s_.fault = FaultRecord{}; }

 private:
  void seal() noexcept { s_.head_crc = logdetail::head_crc(s_.magic, s_.head, s_.count); }
  FlightLogStore<N>& s_;
};

}  // namespace tfc
