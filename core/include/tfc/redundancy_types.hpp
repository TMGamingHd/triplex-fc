// SPDX-License-Identifier: MIT
// The types shared by the redundancy manager (redundancy.hpp) and the sensor-channel life cycle (sensor_health.hpp): the node and channel counts, the reasons a node
// is judged bad, the node states and command results, and the manager's configuration. Split out of redundancy.hpp so that the two can include it without including
// each other. No heap, no exceptions, no RTTI.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "tfc/auth.hpp"
#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace tfc {

constexpr unsigned kNodes = kChannels;      // flight computers A, B, C
constexpr unsigned kVoteChannels = 8;       // gyro x3, accel x3, command pitch, command yaw
constexpr unsigned kStreams = 3;            // per node: gyro, accel, command
constexpr unsigned kChPitch = 6;            // indices into FrameReport::output / votes
constexpr unsigned kChYaw = 7;
constexpr unsigned kMaxCommandsPerFrame = 4;
constexpr unsigned kSensorBase = 4;         // a ground command's node field 4 to 6 addresses IMU channel 0 to 2 (the sensor split), 0 to 2 a computer

// Why a node's data was judged bad in a frame (bit mask in FrameReport::reason).
namespace reason {
constexpr uint8_t kMissing = 1U;  // a gyro/accel/command frame did not arrive
constexpr uint8_t kCrc = 2U;      // a frame arrived but failed its CRC
constexpr uint8_t kSeq = 4U;      // sequence number out of order
constexpr uint8_t kVote = 8U;     // value disagreed with the vote on some channel
constexpr uint8_t kDigest = 16U;  // estimator-state digest disagreed
constexpr uint8_t kStuck = 32U;   // sensor bytes bit-identical for too many frames
constexpr uint8_t kIntermittent = 64U;  // latched by the leaky count: bad often enough, never 3-of-5 in a row
constexpr uint8_t kResync = 128U;       // the state resynchronisation found this node's state far from the vote (RedundancyManager::report_state_correction)
}  // namespace reason

// Writes the set reason bits as words, e.g. "vote disagreement + digest mismatch", into `out`
// (NUL-terminated, truncated to `cap`). No heap; for logs on the host and on the target.
inline void format_reasons(uint8_t bits, char* out, std::size_t cap) noexcept {
  struct Name {
    uint8_t bit;
    const char* text;
  };
  constexpr std::array<Name, 8> names = {{{reason::kMissing, "frame missing"},
                                          {reason::kCrc, "CRC failure"},
                                          {reason::kSeq, "sequence error"},
                                          {reason::kVote, "vote disagreement"},
                                          {reason::kDigest, "digest mismatch"},
                                          {reason::kStuck, "stuck sensor"},
                                          {reason::kIntermittent, "intermittent fault"},
                                          {reason::kResync, "state far from the vote"}}};
  if (cap == 0U) {
    return;
  }
  std::size_t n = 0;
  auto put = [&](const char* text) {
    for (const char* c = text; *c != '\0' && n + 1U < cap; ++c) {
      out[n++] = *c;
    }
  };
  for (const Name& nm : names) {
    if ((bits & nm.bit) != 0U) {
      if (n != 0U) {
        put(" + ");
      }
      put(nm.text);
    }
  }
  if (n == 0U) {
    put("(none)");
  }
  out[n] = '\0';
}

enum class NodeState : uint8_t {
  Healthy,    // votes
  Latched,    // excluded; serving its minimum dwell, waiting for a reintegration request
  Probation,  // excluded, being compared with the voted output (shadow vote)
  Disabled    // excluded for the rest of the run (strikes used up, or disabled by an operator)
};

// Who may start a probation. Manual: only an operator command (the Space Shuttle / airliner
// practice, ADR-010). AutoTransient: also automatically after the minimum dwell, but only for a
// first latch whose cause looks transient (frame problems or one vote episode); never for a stuck
// sensor or a digest mismatch, never for a repeat offender.
enum class ReintegrationPolicy : uint8_t { Manual, AutoTransient };

enum class CommandResult : uint8_t {
  Accepted,
  AlreadyDone,         // e.g. reintegration already requested / on probation, node already disabled
  RefusedDisabled,     // the node is disabled: only ClearDisabled can help
  RefusedNotLatched,   // reintegration of a node that is healthy
  RefusedNotDisabled,  // ClearDisabled for a node that is not disabled
  RefusedBadNode,
  RefusedBadOp,
  RefusedNotArmed      // a dangerous operation without a matching ARM frame in the arm window (ADR-019)
};

inline const char* state_text(NodeState st) noexcept {
  switch (st) {
    case NodeState::Healthy: return "healthy";
    case NodeState::Latched: return "latched";
    case NodeState::Probation: return "probation";
    case NodeState::Disabled: return "disabled";
    default: break;
  }
  return "?";
}

inline const char* op_text(uint8_t op) noexcept {
  switch (static_cast<GroundOp>(op)) {
    case GroundOp::Reintegrate: return "reintegrate";
    case GroundOp::Disable: return "disable";
    case GroundOp::ClearDisabled: return "clear-disabled";
    case GroundOp::ClearSafe: return "clear-safe";
    case GroundOp::Launch: return "launch";
    case GroundOp::Scrub: return "scrub";
    default: break;
  }
  return "unknown-op";
}

inline const char* result_text(CommandResult r) noexcept {
  switch (r) {
    case CommandResult::Accepted: return "accepted";
    case CommandResult::AlreadyDone: return "already done";
    case CommandResult::RefusedDisabled: return "refused: node is disabled";
    case CommandResult::RefusedNotLatched: return "refused: node is not latched";
    case CommandResult::RefusedNotDisabled: return "refused: node is not disabled";
    case CommandResult::RefusedBadNode: return "refused: no such node";
    case CommandResult::RefusedBadOp: return "refused: no such operation";
    case CommandResult::RefusedNotArmed: return "refused: needs an ARM frame first";
    default: break;
  }
  return "?";
}

namespace cmdflag {
constexpr uint8_t kArm = 1U;       // this event is an ARM frame
constexpr uint8_t kArmed = 2U;     // executed under a matching ARM
constexpr uint8_t kCritical = 4U;  // removed the last voting node: reported loudly
}  // namespace cmdflag

struct CommandEvent {
  uint8_t op = 0U;
  uint8_t node = 0U;
  CommandResult result = CommandResult::Accepted;
  uint8_t flags = 0U;  // cmdflag bits
};

struct RedundancyConfig {
  // Vote tolerances per channel: gyro (dps) x3, accel (g) x3, command (deg) x2.
  std::array<float, kVoteChannels> tol{{1.0F, 1.0F, 1.0F, 0.02F, 0.02F, 0.02F, 0.01F, 0.01F}};
  uint8_t persist_m = 3;            // latch when M of the last N frames are bad
  uint8_t persist_n = 5;
  // A digest disagreement counts (as a bad frame for the node it blames, or as an unresolved disagreement) only once it has lasted this many frames in a row. 1: at once, as before.
  // With the state resynchronisation (docs/RESYNC.md) a lost frame leaves the digests different for up to one resync period, and the resync heals it: set this to a little more than the
  // period, so that a mismatch which the resync did not heal is what counts.
  uint16_t digest_persist_frames = 1;
  uint16_t stuck_limit = 20;         // identical sensor frames before "stuck"
  // Leaky count for intermittent faults, OR'd with the M-of-N window (ADR-013): +1 per bad frame,
  // x alpha_k per good frame, latch at alpha_threshold. Catches a node that is bad one frame in three
  // (or two in five) that the window never sees. alpha_threshold = 0 disables it.
  float alpha_k = 0.9F;
  float alpha_threshold = 3.0F;
  // A node that has never delivered a good sample is not judged for this many frames: peers boot
  // in any order, and "not here yet" is not "failed". 0 = judge from the first frame (absent means
  // invalid). Once a node has been seen, it is always judged.
  uint32_t startup_grace_frames = 0;
  // Duplex arbitration (two valid nodes that disagree): blame a node only if it jumped more than
  // `factor` x tolerance away from the last agreed value (set by the previous frame) while the other
  // stayed within one tolerance of it. Otherwise the disagreement is unresolved: nobody is blamed,
  // the last good output is held (and, being stale, never used to blame anyone later), and a
  // persistent run of them requests Safe. 0 disables arbitration.
  float duplex_arbitration_factor = 2.0F;
  // This many out-of-schedule frames inside one 10 ms frame raises the bus alarm (babbling node).
  uint32_t bus_alarm_per_frame = 3;

  // ---- reintegration and disabling (ADR-010) ----
  ReintegrationPolicy policy = ReintegrationPolicy::Manual;
  uint16_t min_dwell_frames = 200;          // a latched node waits at least this long (2 s) before probation
  // ... except after a first, transient-looking latch (frame problems or one vote episode): then the dwell is this long
  // (0.5 s). Every frame a node is out is a frame the system runs with less redundancy, and the node still has to pass the
  // whole probation; a repeat offender or a physical cause keeps the full dwell (ADR-010 amended).
  uint16_t min_dwell_frames_transient = 50;
  uint16_t probation_frames = 100;          // agreeing frames needed after the first latch (1 s)
  uint16_t probation_frames_repeat = 300;   // ... after a repeat latch (strike 2 or more)
  uint8_t max_strikes = 3;                  // latches before the node is disabled for the run
  uint8_t max_strikes_physical = 2;         // ... when the cause is physical (below)
  // Reason bits that point at failed hardware (a recurring fault counts: it is how loose connectors and
  // wearing-out parts behave), so a repeat disables the node at the second strike.
  uint8_t physical_causes = reason::kStuck | reason::kIntermittent;
  // Strikes older than this many frames are forgotten. 0 = the whole run (a flight is minutes long).
  uint32_t strike_window_frames = 0;
  // AutoTransient only: causes that look transient, and how many failed probations are tolerated.
  uint8_t auto_eligible_causes = reason::kMissing | reason::kCrc | reason::kSeq | reason::kVote;
  uint8_t auto_max_attempts = 3;

  // ---- ground commands (ADR-019) ----
  bool ground_auth = true;               // require a valid SipHash tag and a fresh counter on every ground frame
  AuthKey ground_key = kBenchKey;        // the shared key. The default is the PUBLIC bench key: provision your own
  uint8_t command_window = 32;           // a command is fresh if its counter is 1..window ahead of the last accepted one
  uint8_t arm_window_frames = 250;       // an ARM frame stays valid this long (2.5 s)

  // ---- sensing and computing judged separately (ADR-020 case 1, TS-15) ----
  // False (today's behaviour, and the baseline of the study): a computer is one unit, and a bad IMU latches the whole computer. True: each IMU's gyro and accelerometer
  // pair is a sensor channel with its own life cycle (`sensor_health.hpp`); a computer whose IMU is excluded stays a voter for commands for as long as its commands
  // agree, and only a disagreement of its commands (or its digest, or its command frames) removes it from the command vote. Operator commands address a sensor channel
  // by node number 4 to 6.
  bool sensor_split = false;
};

// One frame's verdict on a unit on probation (a computer, or with the sensor split a sensor channel).
enum class Verdict : uint8_t { Clean, Dirty, Neutral };

}  // namespace tfc
