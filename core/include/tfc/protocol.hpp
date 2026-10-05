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
constexpr uint32_t kHeartbeat = 0x400;  // + node: health / mode flags (protocol v2: see Heartbeat)
constexpr uint32_t kState = 0x410;      // + node: strike counts and the last accepted command counter (FDIR-041)
constexpr uint32_t kResync = 0x420;     // + 4 * node + chunk: the replicas' estimator and controller state, in 4 frames of 3 words, once per resync period (docs/RESYNC.md)
constexpr uint32_t kSim = 0x500;        // simulator <-> flight bus gateway: the range 0x500 to 0x50F (kSimLast)
constexpr uint32_t kSimRates = 0x501;      // simulator: sensor-frame body rates, as a gyro frame
constexpr uint32_t kSimAccel = 0x502;      // simulator: sensor-frame accelerometer input, as an accel frame
constexpr uint32_t kSimState = 0x503;      // simulator telemetry: altitude, speed, mass
constexpr uint32_t kSimTelemetry = 0x504;  // simulator telemetry: dynamic pressure, attitude error
constexpr uint32_t kSimFlags = 0x505;      // simulator: flags, engines on, time
constexpr uint32_t kSimLast = 0x50F;
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
// Mission time (docs/LAUNCH_SEQUENCE.md): the two bytes after the frame number carry the mission frame `m`, a 16-bit count that the sync master starts at the launch command.
//   0                  not launched (this is also what a sender that knows nothing of the launch sequence puts there)
//   1 .. 1000          the countdown: T minus (1001 - m) frames, so 10 s long
//   1001               T-zero; `m - 1001` is the number of frames of flight
//   65535              the largest value; the count stops there (655 s)
namespace mission {
constexpr uint16_t kNotLaunched = 0U;
constexpr uint16_t kCountdownFrames = 1000U;
constexpr uint16_t kMax = 0xFFFFU;
constexpr bool counting(uint16_t m) noexcept { return m != kNotLaunched; }
constexpr bool in_countdown(uint16_t m) noexcept { return m != kNotLaunched && m <= kCountdownFrames; }
constexpr bool in_flight(uint16_t m) noexcept { return m > kCountdownFrames; }
// Frames since T-zero (0 before it).
constexpr uint32_t flight_frames(uint16_t m) noexcept { return m > kCountdownFrames ? static_cast<uint32_t>(m) - kCountdownFrames - 1U : 0U; }
// Frames still to T-zero (0 once it has passed or if there is no countdown).
constexpr uint32_t frames_to_zero(uint16_t m) noexcept { return in_countdown(m) ? static_cast<uint32_t>(kCountdownFrames) + 1U - m : 0U; }
}  // namespace mission

struct DecodedSync {
  uint32_t frame_no = 0;
  uint16_t mission = 0;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_sync(uint32_t frame_no, uint8_t seq, uint16_t mission_frame = mission::kNotLaunched) noexcept {
  Frame f;
  f.id = id::kSync;
  for (unsigned i = 0; i < 4U; ++i) {
    f.data[i] = static_cast<uint8_t>((frame_no >> (8U * i)) & 0xFFU);
  }
  f.data[4] = static_cast<uint8_t>(mission_frame & 0xFFU);
  f.data[5] = static_cast<uint8_t>((mission_frame >> 8U) & 0xFFU);
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
  d.mission = static_cast<uint16_t>(static_cast<uint16_t>(f.data[4]) | (static_cast<uint16_t>(f.data[5]) << 8U));
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
  ClearSafe = 4,      // lift a sticky Safe request (always needs an arm)
  Launch = 5,         // start the countdown (always needs an arm; the node field is ignored; the sync master acts, docs/LAUNCH_SEQUENCE.md)
  Scrub = 6           // back to the pad, before T-zero (no arm; the node field is ignored)
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


// ======================================================================================================================================
// Protocol version 2 (docs/PROTOCOL.md): ACT's output, the heartbeat, the state share, and the simulator's frames.
// Every payload is still 6 data bytes | seq | crc8. Multi-byte fields are little endian.
// ======================================================================================================================================
constexpr uint8_t kProtocolVersion = 2U;

namespace detail {
inline void put_u16(Frame& f, unsigned off, uint16_t x) noexcept {
  f.data[off] = static_cast<uint8_t>(x & 0xFFU);
  f.data[off + 1U] = static_cast<uint8_t>(x >> 8);
}
inline uint16_t get_u16(const Frame& f, unsigned off) noexcept {
  return static_cast<uint16_t>(f.data[off] | (f.data[off + 1U] << 8));
}
// A non-negative quantity in units of `lsb`, rounded and saturated to 16 bits (NaN and negatives are 0).
inline uint16_t quantize_u16(float v, float lsb) noexcept {
  const float q = v / lsb;
  if (!(q > 0.0F)) {
    return 0U;
  }
  if (q >= 65535.0F) {
    return 65535U;
  }
  return static_cast<uint16_t>(q > 0.0F ? q + 0.5F : 0.0F);  // q > 0 here; written so that rounding is half up, like the Python mirror
}
}  // namespace detail

// ---- ACT output (0x300): the voted gimbal command and the vote status ----
// Bytes 0-1: pitch plane, bytes 2-3: yaw plane (0.001 degree, as in a command frame). Bytes 4-5, a 16-bit field:
//   bits 0-2   state: 0 Standby, 1 Nominal, 2 Safe (hold), 3 Safe (ramp), 4 Safe (neutral)
//   bit 3      held: no trustworthy vote this frame, the last output is repeated
//   bits 4-6   vote status of the pitch vote (tfc::VoteStatus)
//   bits 7-9   nodes whose command took part in the vote (bit n = node n)
//   bits 10-12 nodes ACT has excluded
//   bits 13-15 cause of Safe: 0 none, 1 lost votes, 2 flight computers' request, 3 hardware line, 4 reset
struct ActFrame {
  float pitch_deg = 0.0F;
  float yaw_deg = 0.0F;
  uint8_t state = 0U;
  bool held = false;
  uint8_t vote_status = 0U;
  uint8_t voted_nodes = 0U;
  uint8_t excluded_nodes = 0U;
  uint8_t cause = 0U;
};

struct DecodedAct {
  ActFrame act;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_act_out(const ActFrame& a, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kActOut;
  detail::put16(f, 0, quantize(a.pitch_deg, kCmdLsbDeg));
  detail::put16(f, 2, quantize(a.yaw_deg, kCmdLsbDeg));
  const uint16_t w = static_cast<uint16_t>((a.state & 0x7U) | ((a.held ? 1U : 0U) << 3U) | ((a.vote_status & 0x7U) << 4U) |
                                           ((a.voted_nodes & 0x7U) << 7U) | ((a.excluded_nodes & 0x7U) << 10U) | ((a.cause & 0x7U) << 13U));
  detail::put_u16(f, 4, w);
  detail::seal(f, seq);
  return f;
}

inline DecodedAct unpack_act_out(const Frame& f) noexcept {
  DecodedAct d;
  if (f.id != id::kActOut || !detail::check(f)) {
    return d;
  }
  d.act.pitch_deg = static_cast<float>(detail::get16(f, 0)) * kCmdLsbDeg;
  d.act.yaw_deg = static_cast<float>(detail::get16(f, 2)) * kCmdLsbDeg;
  const uint16_t w = detail::get_u16(f, 4);
  d.act.state = static_cast<uint8_t>(w & 0x7U);
  d.act.held = ((w >> 3U) & 1U) != 0U;
  d.act.vote_status = static_cast<uint8_t>((w >> 4U) & 0x7U);
  d.act.voted_nodes = static_cast<uint8_t>((w >> 7U) & 0x7U);
  d.act.excluded_nodes = static_cast<uint8_t>((w >> 10U) & 0x7U);
  d.act.cause = static_cast<uint8_t>((w >> 13U) & 0x7U);
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- Heartbeat (0x400 + node): who is on which release, in what state ----
// Byte 0: protocol version. Byte 1: bits 0-1 mode (tfc::Mode), bit 2 Safe requested, bit 3 bus alarm, bits 4-5 role (0 hot, 1 warm, 2 cold),
// bit 6 quarantined (a reset loop), bit 7 ready for launch (docs/LAUNCH_SEQUENCE.md). Byte 2: this node's view of the three nodes, 2 bits each (A in bits 0-1): 0 healthy, 1 latched,
// 2 probation, 3 disabled. Byte 3: the reset count since power-on (saturating). Bytes 4-5: the first 16 bits of the release's source hash,
// so that a node on the golden release (ADR-021) can be told from one on the current release.
struct Heartbeat {
  uint8_t protocol_version = kProtocolVersion;
  uint8_t mode = 0U;
  bool safe_requested = false;
  bool bus_alarm = false;
  uint8_t role = 0U;
  bool quarantined = false;
  bool ready = false;  // ready for launch: this computer's IMU calibration, sensors and attitude are good
  std::array<uint8_t, 3> node_state{};
  uint8_t reset_count = 0U;
  uint16_t release_hash = 0U;
};

struct DecodedHeartbeat {
  Heartbeat hb;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_heartbeat(uint8_t node, const Heartbeat& h, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kHeartbeat + node;
  f.data[0] = h.protocol_version;
  f.data[1] = static_cast<uint8_t>((h.mode & 0x3U) | ((h.safe_requested ? 1U : 0U) << 2U) | ((h.bus_alarm ? 1U : 0U) << 3U) |
                                   ((h.role & 0x3U) << 4U) | ((h.quarantined ? 1U : 0U) << 6U) | ((h.ready ? 1U : 0U) << 7U));
  f.data[2] = static_cast<uint8_t>((h.node_state[0] & 0x3U) | ((h.node_state[1] & 0x3U) << 2U) | ((h.node_state[2] & 0x3U) << 4U));
  f.data[3] = h.reset_count;
  detail::put_u16(f, 4, h.release_hash);
  detail::seal(f, seq);
  return f;
}

inline DecodedHeartbeat unpack_heartbeat(const Frame& f) noexcept {
  DecodedHeartbeat d;
  if (f.id < id::kHeartbeat || f.id >= id::kHeartbeat + 3U || !detail::check(f)) {
    return d;
  }
  d.hb.protocol_version = f.data[0];
  d.hb.mode = static_cast<uint8_t>(f.data[1] & 0x3U);
  d.hb.safe_requested = ((f.data[1] >> 2U) & 1U) != 0U;
  d.hb.bus_alarm = ((f.data[1] >> 3U) & 1U) != 0U;
  d.hb.role = static_cast<uint8_t>((f.data[1] >> 4U) & 0x3U);
  d.hb.quarantined = ((f.data[1] >> 6U) & 1U) != 0U;
  d.hb.ready = ((f.data[1] >> 7U) & 1U) != 0U;
  d.hb.node_state = {static_cast<uint8_t>(f.data[2] & 0x3U), static_cast<uint8_t>((f.data[2] >> 2U) & 0x3U),
                     static_cast<uint8_t>((f.data[2] >> 4U) & 0x3U)};
  d.hb.reset_count = f.data[3];
  d.hb.release_hash = detail::get_u16(f, 4);
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- State share (0x410 + node): what a restarted node needs from the others (FDIR-041) ----
// Byte 0: strikes of node A (bits 0-3) and B (bits 4-7); byte 1: strikes of node C (bits 0-3); byte 2: the last accepted ground-command counter.
struct StateShare {
  std::array<uint8_t, 3> strikes{};  // 0..15 each
  uint8_t command_counter = 0U;
};

struct DecodedStateShare {
  StateShare share;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_state_share(uint8_t node, const StateShare& s, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kState + node;
  f.data[0] = static_cast<uint8_t>((s.strikes[0] & 0xFU) | ((s.strikes[1] & 0xFU) << 4U));
  f.data[1] = static_cast<uint8_t>(s.strikes[2] & 0xFU);
  f.data[2] = s.command_counter;
  detail::seal(f, seq);
  return f;
}

inline DecodedStateShare unpack_state_share(const Frame& f) noexcept {
  DecodedStateShare d;
  if (f.id < id::kState || f.id >= id::kState + 3U || !detail::check(f)) {
    return d;
  }
  d.share.strikes = {static_cast<uint8_t>(f.data[0] & 0xFU), static_cast<uint8_t>((f.data[0] >> 4U) & 0xFU), static_cast<uint8_t>(f.data[1] & 0xFU)};
  d.share.command_counter = f.data[2];
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- State resynchronisation (0x420 + 4 * node + chunk): one quarter of a node's shared state per frame (docs/RESYNC.md) ----
// Three signed 16-bit words per frame (bytes 0 to 5); the sequence byte is the low byte of the frame number the state belongs to, like the sensor frames, so a late chunk of an earlier cycle is not mixed in.
constexpr unsigned kResyncChunks = 4U;
constexpr uint32_t kResyncIds = 3U * kResyncChunks;

struct DecodedResync {
  uint8_t node = 0U;
  uint8_t chunk = 0U;
  std::array<int16_t, 3> words{};
  uint8_t seq = 0U;
  bool ok = false;
};

inline Frame pack_resync(uint8_t node, uint8_t chunk, const std::array<int16_t, 3>& words, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kResync + (kResyncChunks * node) + chunk;
  for (unsigned i = 0; i < 3U; ++i) {
    detail::put16(f, 2U * i, words[i]);
  }
  detail::seal(f, seq);
  return f;
}

inline DecodedResync unpack_resync(const Frame& f) noexcept {
  DecodedResync d;
  if (f.id < id::kResync || f.id >= id::kResync + kResyncIds || !detail::check(f)) {
    return d;
  }
  const uint32_t k = f.id - id::kResync;
  d.node = static_cast<uint8_t>(k / kResyncChunks);
  d.chunk = static_cast<uint8_t>(k % kResyncChunks);
  for (unsigned i = 0; i < 3U; ++i) {
    d.words[i] = detail::get16(f, 2U * i);
  }
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// ---- Simulator frames (0x501 to 0x505): the world, as the vehicle simulator publishes it (docs/VEHICLE_SIM.md section 6) ----
// 0x501 and 0x502 carry the sensor inputs of the frame in the same scales as the gyro and accel frames; each node's simulated IMU adds its own noise and faults.
inline Frame pack_sim_rates(const Vec3& dps, uint8_t seq) noexcept { return pack_vec3(id::kSimRates, dps, kGyroLsbDps, seq); }
inline Frame pack_sim_accel(const Vec3& g, uint8_t seq) noexcept { return pack_vec3(id::kSimAccel, g, kAccelLsbG, seq); }

// 0x503: altitude (10 m per count), speed (1 m/s per count), mass (1 kg per count); each saturates at 65,535 counts.
struct SimState {
  float altitude_m = 0.0F;
  float speed_ms = 0.0F;
  float mass_kg = 0.0F;
};
struct DecodedSimState {
  SimState s;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_sim_state(const SimState& s, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kSimState;
  detail::put_u16(f, 0, detail::quantize_u16(s.altitude_m, 10.0F));
  detail::put_u16(f, 2, detail::quantize_u16(s.speed_ms, 1.0F));
  detail::put_u16(f, 4, detail::quantize_u16(s.mass_kg, 1.0F));
  detail::seal(f, seq);
  return f;
}

inline DecodedSimState unpack_sim_state(const Frame& f) noexcept {
  DecodedSimState d;
  if (f.id != id::kSimState || !detail::check(f)) {
    return d;
  }
  d.s.altitude_m = static_cast<float>(detail::get_u16(f, 0)) * 10.0F;
  d.s.speed_ms = static_cast<float>(detail::get_u16(f, 2));
  d.s.mass_kg = static_cast<float>(detail::get_u16(f, 4));
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// 0x504: dynamic pressure (10 Pa per count), attitude error in the pitch and yaw planes (0.001 degree per count, signed).
struct SimTelemetry {
  float dynamic_pressure_pa = 0.0F;
  float pitch_error_deg = 0.0F;
  float yaw_error_deg = 0.0F;
};
struct DecodedSimTelemetry {
  SimTelemetry t;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_sim_telemetry(const SimTelemetry& t, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kSimTelemetry;
  detail::put_u16(f, 0, detail::quantize_u16(t.dynamic_pressure_pa, 10.0F));
  detail::put16(f, 2, quantize(t.pitch_error_deg, kCmdLsbDeg));
  detail::put16(f, 4, quantize(t.yaw_error_deg, kCmdLsbDeg));
  detail::seal(f, seq);
  return f;
}

inline DecodedSimTelemetry unpack_sim_telemetry(const Frame& f) noexcept {
  DecodedSimTelemetry d;
  if (f.id != id::kSimTelemetry || !detail::check(f)) {
    return d;
  }
  d.t.dynamic_pressure_pa = static_cast<float>(detail::get_u16(f, 0)) * 10.0F;
  d.t.pitch_error_deg = static_cast<float>(detail::get16(f, 2)) * kCmdLsbDeg;
  d.t.yaw_error_deg = static_cast<float>(detail::get16(f, 4)) * kCmdLsbDeg;
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

// 0x505: flags, engines on, and the simulation time in frames (10 ms each).
namespace simflag {
constexpr uint8_t kSafed = 0x01U;               // a Safe event has happened in this run
constexpr uint8_t kPlatformSaturated = 0x02U;   // the vehicle's tilt is beyond the platform's travel
constexpr uint8_t kEngineOut = 0x04U;           // an engine has failed
constexpr uint8_t kCommandHeld = 0x08U;         // ACT's frame was late: the last command was held
constexpr uint8_t kAborted = 0x10U;             // the operator has ended the run
}  // namespace simflag

struct SimFlags {
  uint8_t flags = 0U;
  uint8_t engines_on = 0U;
  uint32_t time_frames = 0U;
};
struct DecodedSimFlags {
  SimFlags s;
  uint8_t seq = 0;
  bool ok = false;
};

inline Frame pack_sim_flags(const SimFlags& s, uint8_t seq) noexcept {
  Frame f;
  f.id = id::kSimFlags;
  f.data[0] = s.flags;
  f.data[1] = s.engines_on;
  for (unsigned i = 0; i < 4U; ++i) {
    f.data[2U + i] = static_cast<uint8_t>((s.time_frames >> (8U * i)) & 0xFFU);
  }
  detail::seal(f, seq);
  return f;
}

inline DecodedSimFlags unpack_sim_flags(const Frame& f) noexcept {
  DecodedSimFlags d;
  if (f.id != id::kSimFlags || !detail::check(f)) {
    return d;
  }
  d.s.flags = f.data[0];
  d.s.engines_on = f.data[1];
  for (unsigned i = 0; i < 4U; ++i) {
    d.s.time_frames |= static_cast<uint32_t>(f.data[2U + i]) << (8U * i);
  }
  d.seq = f.data[6];
  d.ok = true;
  return d;
}

}  // namespace tfc
