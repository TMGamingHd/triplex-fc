// SPDX-License-Identifier: MIT
// Wire protocol for the flight bus (classic CAN, 8-byte payloads).
// Every payload is: 6 data bytes | seq | crc8(bytes 0..6). Lower CAN ID = higher
// priority, so SYNC wins arbitration, then sensors, then commands.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/auth.hpp"
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

// ---- Ground command: an operator action sent to the flight computers over the bus (ADR-019) ----
// Payload: opcode byte | node (0..2, ignored for ClearSafe) | 32-bit tag (little endian) | counter | crc8.
//  * opcode byte: the operation in the low 7 bits, kArmFlag (0x80) set on an ARM frame (see below).
//  * tag: the low 32 bits of SipHash-2-4 over (id, opcode byte, node, counter) with the shared key (auth.hpp).
//  * counter: the ground station's command counter, strictly increasing (modulo 256); a receiver accepts a command
//    only if the counter is newer than the last one it accepted, within a window, so a replay is refused.
// A dangerous operation is a two-step: an ARM frame (same operation and node, arm flag set) followed within the arm window by
// the EXECUTE frame. Frames that fail the tag or the counter check are counted and dropped without a trace on the bus.
enum class GroundOp : uint8_t {
  Reintegrate = 1,    // start probation for a latched node (it must then prove itself by shadow vote)
  Disable = 2,        // exclude a node for the rest of the run (needs an arm if it would leave fewer than 2 healthy nodes)
  ClearDisabled = 3,  // maintenance: bring a disabled node back to "latched" with its strikes cleared (always needs an arm)
  ClearSafe = 4       // lift a sticky Safe request (always needs an arm)
};
constexpr uint8_t kArmFlag = 0x80U;

struct DecodedGround {
  uint8_t op = 0;       // GroundOp value (arm flag removed)
  bool arm = false;     // an ARM frame, not an EXECUTE
  uint8_t node = 0;
  uint8_t counter = 0;
  uint32_t tag = 0;
  bool ok = false;      // CRC good and frame id/length right (authenticity is judged by the manager)
};

// An authenticated ground command frame.
inline Frame pack_ground_auth(GroundOp op, uint8_t node, uint8_t counter, const AuthKey& key, bool arm = false) noexcept {
  Frame f;
  f.id = id::kGround;
  const uint8_t op_byte = static_cast<uint8_t>(static_cast<uint8_t>(op) | (arm ? kArmFlag : 0U));
  f.data[0] = op_byte;
  f.data[1] = node;
  const uint32_t tag = ground_mac(key, id::kGround, op_byte, node, counter);
  for (unsigned i = 0; i < 4U; ++i) {
    f.data[2U + i] = static_cast<uint8_t>((tag >> (8U * i)) & 0xFFU);
  }
  detail::seal(f, counter);
  return f;
}

// An UNauthenticated frame (tag bytes zero): what a node that does not know the key, or a corrupted frame that still
// passes its CRC, looks like. Used by the tests; a manager with authentication on refuses it.
inline Frame pack_ground(GroundOp op, uint8_t node, uint8_t counter) noexcept {
  Frame f;
  f.id = id::kGround;
  f.data[0] = static_cast<uint8_t>(op);
  f.data[1] = node;
  detail::seal(f, counter);
  return f;
}

inline DecodedGround unpack_ground(const Frame& f) noexcept {
  DecodedGround d;
  if (f.id != id::kGround || !detail::check(f)) {
    return d;
  }
  d.op = static_cast<uint8_t>(f.data[0] & ~kArmFlag);
  d.arm = (f.data[0] & kArmFlag) != 0U;
  d.node = f.data[1];
  for (unsigned i = 0; i < 4U; ++i) {
    d.tag |= static_cast<uint32_t>(f.data[2U + i]) << (8U * i);
  }
  d.counter = f.data[6];
  d.ok = true;
  return d;
}

// Does the tag of this frame verify under `key`? (The opcode byte as received, arm flag included, is what is tagged.)
[[nodiscard]] inline bool ground_authentic(const Frame& f, const AuthKey& key) noexcept {
  const DecodedGround d = unpack_ground(f);
  const uint8_t op_byte = static_cast<uint8_t>(d.op | (d.arm ? kArmFlag : 0U));
  return d.ok && d.tag == ground_mac(key, id::kGround, op_byte, d.node, d.counter);
}

// Sequence numbers wrap at 256; returns true if `seq` is the expected frame.
constexpr bool seq_is_next(uint8_t last, uint8_t seq) noexcept {
  return static_cast<uint8_t>(last + 1U) == seq;
}

// How a frame's number compares with the frame the receiver is collecting.
enum class FrameTiming : uint8_t {
  OnTime,  // the number of the current frame
  Late,    // the number of an earlier frame (up to 31 ago) whose slot had nothing: that frame, arriving late (stale data)
  Bad      // anything else: a number from the future (an early stream), too old, a repeat of one already seen, or nonsense
};

// Per-stream check of the frame number in `seq` against the receiver's own frame count (ADR-018). Every node stamps its
// frames with the number of the SYNC frame that opened the cycle (modulo 256), so the number says WHICH frame the data belongs
// to, independent of when the frame happened to arrive. The receiver keeps a 32-cycle history of which numbers it has seen:
//  * the current number is on time; an earlier number not seen before is a late frame (it missed the vote: one lost sample,
//    not a sequence error); a repeat of a number already seen is a duplicate or replay;
//  * a number from the future is a stream that is a whole frame (or more) early: previously invisible, because a counter that
//    simply counted frames stays contiguous however early the frames were sent.
// A node that reboots, or joins late, is in phase at once because it takes the number from SYNC, so a restart is no longer a
// sequence break. Call next_frame() once at the start of each frame, then classify() for every CRC-good frame.
class PhaseTracker {
 public:
  void next_frame() noexcept {
    seen_ <<= 1U;
    arrived_ <<= 1U;
  }

  // A frame on this stream arrived but failed its CRC: its number cannot be trusted, but its slot was not empty.
  void note_damaged() noexcept { arrived_ |= 1U; }

  FrameTiming classify(uint8_t seq, uint32_t frame_no) noexcept {
    const uint8_t behind = static_cast<uint8_t>(static_cast<uint8_t>(frame_no & 0xFFU) - seq);  // 0 = this frame
    const uint32_t earlier_arrivals = arrived_;
    arrived_ |= 1U;
    if (behind >= kHistory) {
      return FrameTiming::Bad;
    }
    const uint32_t bit = 1U << behind;
    if ((seen_ & bit) != 0U) {
      return FrameTiming::Bad;
    }
    // A late frame is the frame of a slot that came up EMPTY. If that frame's own cycle already delivered something (a stream
    // that is permanently one number behind delivers one frame every cycle, just labelled for the cycle before), this is not a late
    // frame: it is a frame with the wrong number.
    if (behind > 0U && (earlier_arrivals & bit) != 0U) {
      return FrameTiming::Bad;
    }
    seen_ |= bit;
    return behind == 0U ? FrameTiming::OnTime : FrameTiming::Late;
  }

 private:
  static constexpr uint8_t kHistory = 32U;
  uint32_t seen_ = 0U;      // bit i set: a frame numbered (current - i) has arrived
  uint32_t arrived_ = 0U;   // bit i set: SOMETHING arrived on this stream in the frame (current - i)
};

}  // namespace tfc
