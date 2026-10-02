// SPDX-License-Identifier: MIT
// Wire protocol for the flight bus (classic CAN, 8-byte payloads).
// Every payload is: 6 data bytes | seq | crc8(bytes 0..6). Lower CAN ID = higher
// priority, so SYNC wins arbitration, then sensors, then commands.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/crc8.hpp"

namespace tfc {

namespace id {
constexpr uint32_t kSync = 0x010;       // frame tick, sent by the sync master
constexpr uint32_t kGyroBase = 0x100;   // + node (0..2)
constexpr uint32_t kAccelBase = 0x110;  // + node
constexpr uint32_t kCmdBase = 0x200;    // + node: per-node control command
constexpr uint32_t kActOut = 0x300;     // voted output + vote status from ACT
constexpr uint32_t kHeartbeat = 0x400;  // + node: health / mode flags
constexpr uint32_t kSim = 0x500;        // simulator <-> flight bus gateway
constexpr uint32_t kGround = 0x510;     // operator / ground command (lowest priority of the control traffic)
}  // namespace id

struct Frame {
  uint32_t id = 0;
  uint8_t len = 0;
  std::array<uint8_t, 8> data{};
};

// Fixed-point scales (chosen to cover the ISM330DHCX ranges used: +-2000 dps, +-16 g).
constexpr float kGyroLsbDps = 0.125F;      // int16 -> +-4095.9 dps
constexpr float kAccelLsbG = 1.0F / 2048;  // int16 -> +-16 g
constexpr float kCmdLsbDeg = 0.001F;       // int16 -> +-32.767 deg

constexpr int16_t quantize(float v, float lsb) noexcept {
  const float q = v / lsb;
  if (std::isnan(q)) {  // NaN -> 0 (callers must not send NaN; validity is handled upstream)
    return 0;
  }
  if (q >= 32767.0F) {
    return 32767;
  }
  if (q <= -32768.0F) {
    return -32768;
  }
  return static_cast<int16_t>(q >= 0.0F ? q + 0.5F : q - 0.5F);
}

struct Vec3 {
  std::array<float, 3> v{};
};

namespace detail {
inline void put16(Frame& f, unsigned off, int16_t x) noexcept {
  const uint16_t u = static_cast<uint16_t>(x);
  f.data[off] = static_cast<uint8_t>(u & 0xFFU);
  f.data[off + 1U] = static_cast<uint8_t>(u >> 8);
}
inline int16_t get16(const Frame& f, unsigned off) noexcept {
  const uint16_t u = static_cast<uint16_t>(f.data[off] | (f.data[off + 1U] << 8));
  return static_cast<int16_t>(u);
}
inline void seal(Frame& f, uint8_t seq) noexcept {
  f.len = 8;
  f.data[6] = seq;
  f.data[7] = crc8(f.data.data(), 7);
}
inline bool check(const Frame& f) noexcept {
  return f.len == 8U && f.data[7] == crc8(f.data.data(), 7);
}
}  // namespace detail

// ---- Vector triple frames (gyro in dps, accel in g) ----
inline Frame pack_vec3(uint32_t can_id, const Vec3& x, float lsb, uint8_t seq) noexcept {
  Frame f;
  f.id = can_id;
  for (unsigned i = 0; i < 3; ++i) {
    detail::put16(f, 2U * i, quantize(x.v[i], lsb));
  }
  detail::seal(f, seq);
  return f;
}

struct DecodedVec3 {
  Vec3 x;
  uint8_t seq = 0;
  bool ok = false;
};

inline DecodedVec3 unpack_vec3(const Frame& f, float lsb) noexcept {
  DecodedVec3 d;
  if (!detail::check(f)) {
    return d;
  }
  for (unsigned i = 0; i < 3; ++i) {
    d.x.v[i] = static_cast<float>(detail::get16(f, 2U * i)) * lsb;
  }
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

inline Frame pack_gyro(uint8_t node, const Vec3& dps, uint8_t seq) noexcept {
  return pack_vec3(id::kGyroBase + node, dps, kGyroLsbDps, seq);
}
inline Frame pack_accel(uint8_t node, const Vec3& g, uint8_t seq) noexcept {
  return pack_vec3(id::kAccelBase + node, g, kAccelLsbG, seq);
}

// ---- Per-node command: pitch/yaw gimbal command + estimator-state digest ----
// `state_digest` is a 16-bit fingerprint of the node's quantized estimator state.
// Healthy replicas fed identical consensus inputs must produce identical digests,
// so a mismatch exposes silent state divergence before it reaches the actuator.
struct Command {
  float pitch_deg = 0.0F;
  float yaw_deg = 0.0F;
  uint16_t state_digest = 0;
};

struct DecodedCommand {
  Command cmd;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_cmd(uint8_t node, const Command& c, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kCmdBase + node;
  detail::put16(f, 0, quantize(c.pitch_deg, kCmdLsbDeg));
  detail::put16(f, 2, quantize(c.yaw_deg, kCmdLsbDeg));
  detail::put16(f, 4, static_cast<int16_t>(c.state_digest));
  detail::seal(f, seq);
  return f;
}

inline DecodedCommand unpack_cmd(const Frame& f) noexcept {
  DecodedCommand d;
  if (!detail::check(f)) {
    return d;
  }
  d.cmd.pitch_deg = static_cast<float>(detail::get16(f, 0)) * kCmdLsbDeg;
  d.cmd.yaw_deg = static_cast<float>(detail::get16(f, 2)) * kCmdLsbDeg;
  d.cmd.state_digest = static_cast<uint16_t>(detail::get16(f, 4));
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- SYNC: sent by the sync master (lowest healthy FC) at the start of every major frame ----
// Payload: 32-bit frame number (little endian) | 2 reserved bytes (0) | seq | crc8. Receivers
// phase-lock their frame timer to its arrival; the frame number lets late joiners (and the
// virtual peers) agree on which frame it is.
struct DecodedSync {
  uint32_t frame_no = 0;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_sync(uint32_t frame_no, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kSync;
  for (unsigned i = 0; i < 4U; ++i) {
    f.data[i] = static_cast<uint8_t>((frame_no >> (8U * i)) & 0xFFU);
  }
  detail::seal(f, seq);
  return f;
}

inline DecodedSync unpack_sync(const Frame& f) noexcept {
  DecodedSync d;
  if (!detail::check(f)) {
    return d;
  }
  for (unsigned i = 0; i < 4U; ++i) {
    d.frame_no |= static_cast<uint32_t>(f.data[i]) << (8U * i);
  }
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- Ground command: an operator action sent to the flight computers over the bus ----
// Payload: opcode | node (0..2, ignored for ClearSafe) | 4 reserved bytes (0) | seq | crc8.
// Every operation is idempotent, so a repeated frame is harmless. There is no authentication:
// this is an educational bench, not a flight uplink.
enum class GroundOp : uint8_t {
  Reintegrate = 1,    // start probation for a latched node (it must then prove itself by shadow vote)
  Disable = 2,        // exclude a node for the rest of the run
  ClearDisabled = 3,  // maintenance: bring a disabled node back to "latched" with its strikes cleared
  ClearSafe = 4       // lift a sticky Safe request
};

struct DecodedGround {
  uint8_t op = 0;
  uint8_t node = 0;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_ground(GroundOp op, uint8_t node, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kGround;
  f.data[0] = static_cast<uint8_t>(op);
  f.data[1] = node;
  detail::seal(f, seq);
  return f;
}

inline DecodedGround unpack_ground(const Frame& f) noexcept {
  DecodedGround d;
  if (f.id != id::kGround || !detail::check(f)) {
    return d;
  }
  d.op = f.data[0];
  d.node = f.data[1];
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// Sequence numbers wrap at 256; returns true if `seq` is the expected frame.
constexpr bool seq_is_next(uint8_t last, uint8_t seq) noexcept {
  return static_cast<uint8_t>(last + 1U) == seq;
}

// Per-stream sequence check for one sender. A frame that arrives damaged (CRC failure) was
// still sent, so it used up a sequence number: note_damaged() advances the expectation, and
// one corrupted frame then costs one bad sample (the CRC failure) instead of two (the failure
// plus a false "gap" on the next good frame). Real gaps are still reported. The first good
// frame after construction is always accepted.
class SeqTracker {
 public:
  // Feed the sequence number of a CRC-good frame. Returns false if it is not the expected next.
  // One exception: after note_missing(), a frame carrying one of the numbers we merely ASSUMED were
  // missing is that slot's own frame arriving late (it missed the vote; under load a stalled sender
  // can flush several at once). It is accepted without moving the expectation, so a late frame costs
  // one bad sample (it was missing) and not two. Each assumed number is accepted once.
  bool accept(uint8_t seq) noexcept {
    const uint8_t behind = static_cast<uint8_t>(last_ - seq);  // 0 = exactly the number assumed last
    if (have_ && pending_ > 0U && behind < pending_) {
      pending_ = behind;  // older assumed slots can no longer arrive (a sender's frames stay in order)
      return true;
    }
    const bool ok = !have_ || seq_is_next(last_, seq);
    last_ = seq;
    have_ = true;
    pending_ = 0U;
    return ok;
  }

  // A frame on this stream arrived but failed its CRC: its sequence number cannot be trusted,
  // so assume it was the next one.
  void note_damaged() noexcept {
    if (have_) {
      last_ = static_cast<uint8_t>(last_ + 1U);
    }
    pending_ = 0U;
  }

  // The slot passed and no frame arrived at all (lost on the bus, sent late, or the sender was silent).
  // The sender's counter still advances with the schedule, so assume the frame carried the next
  // number: one lost frame then costs one bad sample (it is missing), not two (missing + a false
  // "gap" on the next good frame). If those frames turn up late, accept() recognises them. A sender
  // that restarts its counter is reported once, on its first frame back.
  void note_missing() noexcept {
    if (have_) {
      last_ = static_cast<uint8_t>(last_ + 1U);
      if (pending_ < kMaxLate) {
        ++pending_;
      }
    }
  }

 private:
  uint8_t last_ = 0U;
  bool have_ = false;
  static constexpr uint8_t kMaxLate = 32U;  // a longer silence is a dead node, not a late burst
  uint8_t pending_ = 0U;  // assumed-missing numbers ending at last_ that may still arrive late
};

}  // namespace tfc
