// SPDX-License-Identifier: MIT
// One flight computer's view of its two peers (and itself): collect a major frame of bus
// traffic, judge every node, vote every channel, and update the FDIR state.
//
//   mgr.begin_frame();                 // start of each 10 ms major frame
//   mgr.on_frame(f);                   // every flight-bus frame received (and this node's own)
//   const FrameReport& r = mgr.end_frame();   // vote + FDIR step; r says who latched and why
//
// Per node it checks that gyro, accel and command arrived, CRC-good and in sequence; votes the
// 8 channels (gyro x3, accel x3, command pitch/yaw) with vote3; cross-checks the estimator
// digest; runs a StuckDetector on the raw sensor bytes; and feeds one ChannelMonitor per node
// (M-of-N persistence). Absent means invalid: a node that sends nothing is just a node whose
// frames are missing.
//
// Node life cycle (ADR-010):  Healthy -> Latched -> Probation -> Healthy, or -> Disabled.
//  * A node that latches is excluded from the vote and counts a strike.
//  * After a minimum dwell, and when asked (operator command, or the opt-in automatic policy for a
//    first, transient-looking latch), it goes on probation: it is still excluded, but every frame
//    its data is compared with the voted output of the healthy nodes (a "shadow vote") and with
//    their digest. Only a run of frames that all agree readmits it, so a node that is still broken
//    can never come back. One node is on probation at a time.
//  * Too many strikes (3; 2 for a physical cause such as a stuck sensor) disable the node for the
//    run; only a maintenance command brings it back to Latched.
// No heap, no exceptions, deterministic.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace tfc {

constexpr unsigned kNodes = kChannels;      // flight computers A, B, C
constexpr unsigned kVoteChannels = 8;       // gyro x3, accel x3, command pitch, command yaw
constexpr unsigned kStreams = 3;            // per node: gyro, accel, command
constexpr unsigned kChPitch = 6;            // indices into FrameReport::output / votes
constexpr unsigned kChYaw = 7;
constexpr unsigned kMaxCommandsPerFrame = 4;

// Why a node's data was judged bad in a frame (bit mask in FrameReport::reason).
namespace reason {
constexpr uint8_t kMissing = 1U;  // a gyro/accel/command frame did not arrive
constexpr uint8_t kCrc = 2U;      // a frame arrived but failed its CRC
constexpr uint8_t kSeq = 4U;      // sequence number out of order
constexpr uint8_t kVote = 8U;     // value disagreed with the vote on some channel
constexpr uint8_t kDigest = 16U;  // estimator-state digest disagreed
constexpr uint8_t kStuck = 32U;   // sensor bytes bit-identical for too many frames
}  // namespace reason

// Writes the set reason bits as words, e.g. "vote disagreement + digest mismatch", into `out`
// (NUL-terminated, truncated to `cap`). No heap; for logs on the host and on the target.
inline void format_reasons(uint8_t bits, char* out, std::size_t cap) noexcept {
  struct Name {
    uint8_t bit;
    const char* text;
  };
  constexpr std::array<Name, 6> names = {{{reason::kMissing, "frame missing"},
                                          {reason::kCrc, "CRC failure"},
                                          {reason::kSeq, "sequence error"},
                                          {reason::kVote, "vote disagreement"},
                                          {reason::kDigest, "digest mismatch"},
                                          {reason::kStuck, "stuck sensor"}}};
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
  RefusedBadOp
};

inline const char* state_text(NodeState st) noexcept {
  switch (st) {
    case NodeState::Healthy: return "healthy";
    case NodeState::Latched: return "latched";
    case NodeState::Probation: return "probation";
    case NodeState::Disabled: return "disabled";
  }
  return "?";
}

inline const char* op_text(uint8_t op) noexcept {
  switch (static_cast<GroundOp>(op)) {
    case GroundOp::Reintegrate: return "reintegrate";
    case GroundOp::Disable: return "disable";
    case GroundOp::ClearDisabled: return "clear-disabled";
    case GroundOp::ClearSafe: return "clear-safe";
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
  }
  return "?";
}

struct CommandEvent {
  uint8_t op = 0U;
  uint8_t node = 0U;
  CommandResult result = CommandResult::Accepted;
};

struct RedundancyConfig {
  // Vote tolerances per channel: gyro (dps) x3, accel (g) x3, command (deg) x2.
  std::array<float, kVoteChannels> tol{{1.0F, 1.0F, 1.0F, 0.02F, 0.02F, 0.02F, 0.01F, 0.01F}};
  uint8_t persist_m = 3;            // latch when M of the last N frames are bad
  uint8_t persist_n = 5;
  uint16_t stuck_limit = 20;         // identical sensor frames before "stuck"
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
  uint16_t probation_frames = 100;          // agreeing frames needed after the first latch (1 s)
  uint16_t probation_frames_repeat = 300;   // ... after a repeat latch (strike 2 or more)
  uint8_t max_strikes = 3;                  // latches before the node is disabled for the run
  uint8_t max_strikes_physical = 2;         // ... when the cause is physical (below)
  uint8_t physical_causes = reason::kStuck;  // reason bits that point at failed hardware
  // Strikes older than this many frames are forgotten. 0 = the whole run (a flight is minutes long).
  uint32_t strike_window_frames = 0;
  // AutoTransient only: causes that look transient, and how many failed probations are tolerated.
  uint8_t auto_eligible_causes = reason::kMissing | reason::kCrc | reason::kSeq | reason::kVote;
  uint8_t auto_max_attempts = 3;
};

struct Counters {
  uint32_t frames = 0;
  uint32_t crc_bad = 0;             // frames discarded for a failed CRC
  uint32_t seq_bad = 0;             // frames whose sequence number was not the expected next
  uint32_t missing = 0;             // node-frames where gyro/accel/command did not all arrive
  uint32_t out_of_schedule = 0;     // frames on IDs that are not part of the schedule
  uint32_t stuck_flags = 0;
  uint32_t digest_flags = 0;
  uint32_t vote_disagreements = 0;  // frames where any channel vote blamed a node
  uint32_t unresolved_frames = 0;   // frames with a disagreement nobody could be blamed for
  uint32_t held_frames = 0;         // frames in which some output channel held its last good value
  uint32_t safe_request_frames = 0; // frames spent requesting Safe
  uint32_t bus_alarm_frames = 0;    // frames with the out-of-schedule flood alarm raised
  uint32_t probations_started = 0;
  uint32_t probation_failures = 0;  // probations ended by a disagreeing or broken frame
  uint32_t reintegrations = 0;      // nodes readmitted to the vote
  uint32_t nodes_disabled = 0;      // disable events (strikes used up, or by command)
  uint32_t commands_accepted = 0;   // ground commands applied
  uint32_t commands_refused = 0;    // ground commands refused (see CommandResult)
  uint32_t commands_bad = 0;        // ground frames dropped: failed CRC or queue full
};

struct FrameReport {
  uint8_t valid_mask = 0U;      // nodes whose data took part in the vote this frame
  uint8_t newly_latched = 0U;   // nodes that latched on this frame (a strike was counted)
  uint8_t newly_seen = 0U;      // nodes whose first good sample arrived on this frame
  uint8_t latched_mask = 0U;    // nodes currently out of the vote (latched, on probation or disabled)
  uint8_t probation_mask = 0U;  // nodes on probation
  uint8_t disabled_mask = 0U;   // nodes disabled for the run
  uint8_t probation_started = 0U;   // nodes that went on probation this frame
  uint8_t probation_failed = 0U;    // nodes thrown back to Latched this frame (they disagreed or broke)
  uint8_t newly_reintegrated = 0U;  // nodes readmitted to the vote this frame
  uint8_t newly_disabled = 0U;      // nodes disabled this frame
  std::array<uint8_t, kNodes> reason{};   // reason:: bits, per node, this frame
  std::array<uint8_t, kNodes> strikes{};  // latches counted against each node
  std::array<CommandEvent, kMaxCommandsPerFrame> commands{};  // ground commands applied this frame
  unsigned command_count = 0U;
  Mode mode = Mode::Safe;
  unsigned healthy = 0U;        // nodes that have been seen and are Healthy (voting)
  std::array<VoteResult, kVoteChannels> votes{};  // raw vote result per channel (value undefined on a miscompare)
  // What a downstream consumer should use: the voted value, or, when the vote could not produce a
  // trustworthy one (duplex disagreement nobody can be blamed for, no majority, no data), the last
  // good value. 0 before there is any history. `held_mask` has bit ch set when output[ch] is held.
  std::array<float, kVoteChannels> output{};
  uint8_t held_mask = 0U;
  bool unresolved = false;      // a disagreement this frame that no node could be blamed for
  bool safe_request = false;    // unresolved disagreements persisted (M of the last N frames): Safe is
                                // requested and STAYS requested until clear_safe_request(); outputs are held
  bool bus_alarm = false;       // out-of-schedule frames this frame >= RedundancyConfig::bus_alarm_per_frame
  uint32_t out_of_schedule_in_frame = 0U;
};

class RedundancyManager {
 public:
  explicit RedundancyManager(const RedundancyConfig& cfg = RedundancyConfig{}) noexcept
      : cfg_(cfg),
        mon_{ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU)},
        stuck_{StuckDetector(cfg.stuck_limit), StuckDetector(cfg.stuck_limit),
               StuckDetector(cfg.stuck_limit)} {}

  void begin_frame() noexcept {
    rx_ = {};
    oos_in_frame_ = 0U;
  }

  // Offer one received frame. Returns false if its ID is not part of the flight-bus schedule
  // (it is then only counted). SYNC, actuator, heartbeat and sim frames are accepted and ignored;
  // ground commands are queued and applied at the end of the frame.
  bool on_frame(const Frame& f) noexcept {
    if (f.id == id::kGround) {
      const DecodedGround d = unpack_ground(f);
      if (!d.ok || npending_ >= kMaxCommandsPerFrame) {
        ++counters_.commands_bad;
      } else {
        pending_[npending_++] = d;
      }
      return true;
    }
    unsigned stream = 0U;
    unsigned node = 0U;
    if (f.id >= id::kGyroBase && f.id < id::kGyroBase + kNodes) {
      stream = 0U;
      node = f.id - id::kGyroBase;
    } else if (f.id >= id::kAccelBase && f.id < id::kAccelBase + kNodes) {
      stream = 1U;
      node = f.id - id::kAccelBase;
    } else if (f.id >= id::kCmdBase && f.id < id::kCmdBase + kNodes) {
      stream = 2U;
      node = f.id - id::kCmdBase;
    } else {
      const bool known = f.id == id::kSync || f.id == id::kActOut ||
                         (f.id >= id::kHeartbeat && f.id < id::kHeartbeat + kNodes) || f.id >= id::kSim;
      if (!known) {
        ++counters_.out_of_schedule;
        ++oos_in_frame_;
      }
      return known;
    }
    NodeRx& r = rx_[node];
    r.arrived[stream] = true;
    bool ok = false;
    uint8_t seq = 0U;
    if (stream == 2U) {
      const DecodedCommand d = unpack_cmd(f);
      ok = d.ok;
      seq = d.seq;
      if (ok) {
        r.x[6] = d.cmd.pitch_deg;
        r.x[7] = d.cmd.yaw_deg;
        r.digest = d.cmd.state_digest;
        r.cmd = true;
      }
    } else {
      const bool is_gyro = stream == 0U;
      const DecodedVec3 d = unpack_vec3(f, is_gyro ? kGyroLsbDps : kAccelLsbG);
      ok = d.ok;
      seq = d.seq;
      if (ok) {
        for (unsigned i = 0; i < 3U; ++i) {
          r.x[(is_gyro ? 0U : 3U) + i] = d.x.v[i];
        }
        std::memcpy(r.raw.data() + (is_gyro ? 0U : 6U), f.data.data(), 6U);
        (is_gyro ? r.gyro : r.accel) = true;
      }
    }
    if (!ok) {
      r.crc_bad = true;
      ++counters_.crc_bad;
      seq_[node][stream].note_damaged();  // it was sent, so it used up a sequence number
      return true;
    }
    if (!seq_[node][stream].accept(seq)) {
      r.seq_bad = true;
      ++counters_.seq_bad;
    }
    return true;
  }

  // Close the frame: apply ground commands, judge nodes, vote, cross-check, update FDIR and the
  // node life cycle. The returned reference is valid until the next end_frame().
  const FrameReport& end_frame() noexcept {
    FrameReport& rep = report_;
    rep = FrameReport{};
    ++counters_.frames;
    apply_pending(rep);

    std::array<bool, kNodes> good{};
    uint8_t valid = 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      const NodeRx& r = rx_[n];
      for (unsigned st = 0; st < kStreams; ++st) {
        if (!r.arrived[st]) {
          seq_[n][st].note_missing();  // slot passed with nothing: the sender's counter still advanced
        }
      }
      const bool present = r.gyro && r.accel && r.cmd;
      const bool in_grace = !seen_[n] && counters_.frames <= cfg_.startup_grace_frames;
      if (!present && !r.crc_bad && !in_grace) {
        ++counters_.missing;
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kMissing);
      }
      if (r.crc_bad) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kCrc);
      }
      if (r.seq_bad) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kSeq);
      }
      good[n] = present && !r.crc_bad && !r.seq_bad;
      if (good[n] && !seen_[n]) {
        seen_[n] = true;
        rep.newly_seen = static_cast<uint8_t>(rep.newly_seen | (1U << n));
      }
      if (good[n] && state_[n] == NodeState::Healthy) {
        valid = static_cast<uint8_t>(valid | (1U << n));
      }
    }
    rep.valid_mask = valid;

    uint8_t disagree = 0U;
    bool unresolved = false;
    for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
      const std::array<float, kNodes> x = {rx_[0].x[ch], rx_[1].x[ch], rx_[2].x[ch]};
      const VoteResult v = vote3(x, valid, cfg_.tol[ch]);
      rep.votes[ch] = v;
      uint8_t blame = v.disagree_mask;
      bool trusted = true;
      float out = v.value;
      switch (v.status) {
        case VoteStatus::Triplex:
        case VoteStatus::Duplex:
        case VoteStatus::Simplex:
          break;
        case VoteStatus::DuplexMiscompare:
          // Two nodes disagree: the vote alone cannot say who is wrong. Use continuity with the last
          // agreed value; if that does not single one out, blame nobody.
          blame = 0U;
          trusted = arbitrate(ch, x, valid, out, blame);
          if (!trusted) {
            unresolved = true;
          }
          break;
        case VoteStatus::NoMajority:
        case VoteStatus::NoData:
          trusted = false;
          break;
      }
      if (trusted && !safe_latched_) {
        last_good_[ch] = out;
        have_last_[ch] = true;
        ref_fresh_[ch] = true;
        rep.output[ch] = out;
      } else {
        ref_fresh_[ch] = false;  // a held reference goes stale: it must not be used to blame anyone
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;  // hold the last good value
        rep.held_mask = static_cast<uint8_t>(rep.held_mask | (1U << ch));
      }
      disagree = static_cast<uint8_t>(disagree | blame);
    }
    const DigestVerdict dv = digest_outliers(valid);
    const uint8_t digest_bad = dv.blame;
    unresolved = unresolved || dv.unresolved;
    if (disagree != 0U) {
      ++counters_.vote_disagreements;
    }
    if (digest_bad != 0U || dv.unresolved) {
      ++counters_.digest_flags;
    }
    rep.unresolved = unresolved;
    unres_hist_ = (unres_hist_ << 1) | (unresolved ? 1U : 0U);
    const uint32_t window = cfg_.persist_n >= 32U ? 0xFFFFFFFFU : ((1U << cfg_.persist_n) - 1U);
    if (popcount32(unres_hist_ & window) >= cfg_.persist_m) {
      safe_latched_ = true;  // sticky: only clear_safe_request() (operator/ground) lifts it
    }
    rep.safe_request = safe_latched_;
    if (safe_latched_) {  // hold the last voted values for the whole time Safe is requested
      for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;
      }
      rep.held_mask = 0xFFU;
    }
    rep.out_of_schedule_in_frame = oos_in_frame_;
    rep.bus_alarm = oos_in_frame_ >= cfg_.bus_alarm_per_frame && cfg_.bus_alarm_per_frame != 0U;
    counters_.unresolved_frames += unresolved ? 1U : 0U;
    counters_.safe_request_frames += rep.safe_request ? 1U : 0U;
    counters_.held_frames += rep.held_mask != 0U ? 1U : 0U;
    counters_.bus_alarm_frames += rep.bus_alarm ? 1U : 0U;

    // ---- per node: stuck detector, FDIR verdict, latch / strikes ----
    std::array<bool, kNodes> stuck_now{};
    for (unsigned n = 0; n < kNodes; ++n) {
      if (good[n]) {
        stuck_now[n] = stuck_[n].update(static_cast<int32_t>(fnv1a(rx_[n].raw.data(), rx_[n].raw.size())));
      }
      if (stuck_now[n]) {
        ++counters_.stuck_flags;
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kStuck);
      }
      if (((disagree >> n) & 1U) != 0U) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kVote);
      }
      if (((digest_bad >> n) & 1U) != 0U) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kDigest);
      }
      if (state_[n] == NodeState::Healthy && mon_[n].update(rep.reason[n] != 0U)) {
        on_latch(n, rep);
      }
    }

    // ---- node life cycle: probation verdicts first, then dwell and new probations ----
    for (unsigned n = 0; n < kNodes; ++n) {
      if (state_[n] == NodeState::Probation) {
        judge_probation(n, good[n], stuck_now[n], valid, rep);
      }
    }
    bool one_on_probation = false;
    for (unsigned n = 0; n < kNodes; ++n) {
      one_on_probation = one_on_probation || state_[n] == NodeState::Probation;
    }
    for (unsigned n = 0; n < kNodes; ++n) {
      if (state_[n] != NodeState::Latched) {
        continue;
      }
      if (((rep.newly_latched >> n) & 1U) == 0U && dwell_[n] < 0xFFFFU) {
        ++dwell_[n];
      }
      if (!one_on_probation && dwell_[n] >= cfg_.min_dwell_frames && wants_probation(n)) {
        state_[n] = NodeState::Probation;
        req_[n] = false;
        probation_clean_[n] = 0U;
        one_on_probation = true;
        rep.probation_started = static_cast<uint8_t>(rep.probation_started | (1U << n));
        ++counters_.probations_started;
      }
    }

    for (unsigned n = 0; n < kNodes; ++n) {
      rep.strikes[n] = strikes_[n];
      if (state_[n] != NodeState::Healthy) {
        rep.latched_mask = static_cast<uint8_t>(rep.latched_mask | (1U << n));
      }
      if (state_[n] == NodeState::Probation) {
        rep.probation_mask = static_cast<uint8_t>(rep.probation_mask | (1U << n));
      }
      if (state_[n] == NodeState::Disabled) {
        rep.disabled_mask = static_cast<uint8_t>(rep.disabled_mask | (1U << n));
      }
      if (state_[n] == NodeState::Healthy && seen_[n]) {
        ++rep.healthy;
      }
    }
    rep.mode = rep.safe_request ? Mode::Safe : mode_from_healthy(rep.healthy);
    return rep;
  }

  // ---- operator / ground commands (also reachable as CAN ground frames) ----
  // Start probation for a latched node (after its minimum dwell, one node at a time).
  CommandResult request_reintegration(unsigned node) noexcept { return command(GroundOp::Reintegrate, node); }

  // Apply one command now; results are also reported in FrameReport::commands when they arrive as frames.
  CommandResult command(GroundOp op, unsigned node) noexcept {
    CommandResult r = CommandResult::Accepted;
    switch (op) {
      case GroundOp::Reintegrate:
        if (node >= kNodes) {
          r = CommandResult::RefusedBadNode;
        } else if (state_[node] == NodeState::Disabled) {
          r = CommandResult::RefusedDisabled;
        } else if (state_[node] == NodeState::Healthy) {
          r = CommandResult::RefusedNotLatched;
        } else if (state_[node] == NodeState::Probation || req_[node]) {
          r = CommandResult::AlreadyDone;
        } else {
          req_[node] = true;
        }
        break;
      case GroundOp::Disable:
        if (node >= kNodes) {
          r = CommandResult::RefusedBadNode;
        } else if (state_[node] == NodeState::Disabled) {
          r = CommandResult::AlreadyDone;
        } else {
          state_[node] = NodeState::Disabled;
          req_[node] = false;
          ++counters_.nodes_disabled;
          disabled_by_command_ = static_cast<uint8_t>(disabled_by_command_ | (1U << node));
        }
        break;
      case GroundOp::ClearDisabled:
        if (node >= kNodes) {
          r = CommandResult::RefusedBadNode;
        } else if (state_[node] != NodeState::Disabled) {
          r = CommandResult::RefusedNotDisabled;
        } else {
          state_[node] = NodeState::Latched;  // back to square one: dwell, then a fresh request
          strikes_[node] = 0U;
          dwell_[node] = 0U;
          attempts_[node] = 0U;
          req_[node] = false;
        }
        break;
      case GroundOp::ClearSafe:
        if (!safe_latched_) {
          r = CommandResult::AlreadyDone;
        }
        clear_safe_request();
        break;
      default:
        r = CommandResult::RefusedBadOp;
        break;
    }
    if (r == CommandResult::Accepted) {
      ++counters_.commands_accepted;
    } else if (r != CommandResult::AlreadyDone) {
      ++counters_.commands_refused;
    }
    return r;
  }

  // Lift a Safe request (the disagreement history is forgotten too).
  void clear_safe_request() noexcept {
    safe_latched_ = false;
    unres_hist_ = 0U;
  }

  bool safe_requested() const noexcept { return safe_latched_; }
  bool seen(unsigned node) const noexcept { return node < kNodes && seen_[node]; }
  NodeState state(unsigned node) const noexcept { return node < kNodes ? state_[node] : NodeState::Disabled; }
  unsigned strikes(unsigned node) const noexcept { return node < kNodes ? strikes_[node] : 0U; }
  // Out of the vote for any reason (latched, on probation, or disabled).
  bool latched(unsigned node) const noexcept { return node < kNodes && state_[node] != NodeState::Healthy; }
  bool permanent(unsigned node) const noexcept { return node < kNodes && state_[node] == NodeState::Disabled; }
  const Counters& counters() const noexcept { return counters_; }
  const FrameReport& last_report() const noexcept { return report_; }

 private:
  struct NodeRx {
    bool gyro = false;
    bool accel = false;
    bool cmd = false;
    bool crc_bad = false;
    bool seq_bad = false;
    std::array<bool, kStreams> arrived{};  // a frame (good or damaged) came in on this stream
    std::array<float, kVoteChannels> x{};
    uint16_t digest = 0U;
    std::array<uint8_t, 12> raw{};  // gyro + accel payload bytes, for the stuck detector
  };

  static uint32_t fnv1a(const uint8_t* d, std::size_t n) noexcept {
    uint32_t h = 2166136261U;
    for (std::size_t i = 0; i < n; ++i) {
      h = (h ^ d[i]) * 16777619U;
    }
    return h;
  }

  void apply_pending(FrameReport& rep) noexcept {
    for (unsigned i = 0; i < npending_; ++i) {
      CommandEvent ev;
      ev.op = pending_[i].op;
      ev.node = pending_[i].node;
      ev.result = command(static_cast<GroundOp>(pending_[i].op), pending_[i].node);
      rep.commands[rep.command_count++] = ev;
    }
    npending_ = 0U;
    // Disables made by command (frame or direct call) since the last frame are reported now.
    rep.newly_disabled = static_cast<uint8_t>(rep.newly_disabled | disabled_by_command_);
    disabled_by_command_ = 0U;
  }

  // The node's monitor just latched: count the strike and decide Latched or Disabled.
  void on_latch(unsigned n, FrameReport& rep) noexcept {
    rep.newly_latched = static_cast<uint8_t>(rep.newly_latched | (1U << n));
    if (cfg_.strike_window_frames != 0U && strikes_[n] != 0U &&
        counters_.frames - last_latch_frame_[n] > cfg_.strike_window_frames) {
      strikes_[n] = 0U;  // old strikes are forgotten
    }
    if (strikes_[n] < 0xFFU) {
      ++strikes_[n];
    }
    last_latch_frame_[n] = counters_.frames;
    latch_cause_[n] = rep.reason[n];
    const unsigned limit = (rep.reason[n] & cfg_.physical_causes) != 0U ? cfg_.max_strikes_physical : cfg_.max_strikes;
    dwell_[n] = 0U;
    req_[n] = false;
    attempts_[n] = 0U;
    if (limit != 0U && strikes_[n] >= limit) {
      state_[n] = NodeState::Disabled;
      rep.newly_disabled = static_cast<uint8_t>(rep.newly_disabled | (1U << n));
      ++counters_.nodes_disabled;
    } else {
      state_[n] = NodeState::Latched;
    }
  }

  bool wants_probation(unsigned n) const noexcept {
    if (req_[n]) {
      return true;
    }
    return cfg_.policy == ReintegrationPolicy::AutoTransient && strikes_[n] == 1U &&
           (latch_cause_[n] & ~cfg_.auto_eligible_causes) == 0U && attempts_[n] < cfg_.auto_max_attempts;
  }

  // One frame of a node on probation: shadow-compare it with the voted output of the healthy nodes.
  void judge_probation(unsigned n, bool good, bool stuck, uint8_t valid, FrameReport& rep) noexcept {
    enum class Verdict : uint8_t { Clean, Dirty, Neutral };
    Verdict v = Verdict::Clean;
    if (!good || stuck) {
      v = Verdict::Dirty;  // a missing, damaged, out-of-sequence or frozen frame
    } else if (valid == 0U || rep.held_mask != 0U) {
      v = Verdict::Neutral;  // no trustworthy reference this frame (nobody healthy, Safe, unresolved)
    } else {
      for (unsigned ch = 0; ch < kVoteChannels && v == Verdict::Clean; ++ch) {
        if (std::fabs(rx_[n].x[ch] - rep.output[ch]) > cfg_.tol[ch]) {
          v = Verdict::Dirty;
          rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kVote);  // disagrees with the shadow vote
        }
      }
      if (v == Verdict::Clean) {
        bool have_ref = false;
        bool ref_agrees = true;
        uint16_t ref = 0U;
        for (unsigned i = 0; i < kNodes; ++i) {
          if (((valid >> i) & 1U) == 0U) {
            continue;
          }
          if (!have_ref) {
            ref = rx_[i].digest;
            have_ref = true;
          } else if (rx_[i].digest != ref) {
            ref_agrees = false;
          }
        }
        if (!ref_agrees) {
          v = Verdict::Neutral;
        } else if (have_ref && rx_[n].digest != ref) {
          v = Verdict::Dirty;
          rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kDigest);
        }
      }
    }
    if (v == Verdict::Dirty) {
      state_[n] = NodeState::Latched;  // thrown back: dwell starts again, a new request is needed
      dwell_[n] = 0U;
      req_[n] = false;
      if (attempts_[n] < 0xFFU) {
        ++attempts_[n];
      }
      rep.probation_failed = static_cast<uint8_t>(rep.probation_failed | (1U << n));
      ++counters_.probation_failures;
    } else if (v == Verdict::Clean) {
      const unsigned needed = strikes_[n] >= 2U ? cfg_.probation_frames_repeat : cfg_.probation_frames;
      if (++probation_clean_[n] >= needed) {
        state_[n] = NodeState::Healthy;
        mon_[n].force_unlatch();
        stuck_[n] = StuckDetector(cfg_.stuck_limit);
        rep.newly_reintegrated = static_cast<uint8_t>(rep.newly_reintegrated | (1U << n));
        ++counters_.reintegrations;
      }
    }
  }

  struct DigestVerdict {
    uint8_t blame = 0U;       // nodes in the minority (only when three can be compared)
    bool unresolved = false;  // two comparable nodes disagree: nobody can be blamed
  };

  // Cross-check the estimator-state digests of the voting nodes. With three, the odd one out is
  // blamed. With two, a mismatch cannot be attributed.
  DigestVerdict digest_outliers(uint8_t valid) const noexcept {
    DigestVerdict d;
    const unsigned n = count_channels(valid);
    if (n == 3U) {
      const uint16_t a = rx_[0].digest;
      const uint16_t b = rx_[1].digest;
      const uint16_t c = rx_[2].digest;
      if (a == b && b == c) {
        return d;
      }
      if (a == b) {
        d.blame = 0x4U;
      } else if (a == c) {
        d.blame = 0x2U;
      } else {
        d.blame = b == c ? 0x1U : 0x7U;
      }
    } else if (n == 2U) {
      uint16_t first = 0U;
      bool have = false;
      for (unsigned i = 0; i < kNodes; ++i) {
        if (((valid >> i) & 1U) == 0U) {
          continue;
        }
        if (!have) {
          first = rx_[i].digest;
          have = true;
        } else if (rx_[i].digest != first) {
          d.unresolved = true;
        }
      }
    }
    return d;
  }

  // Two valid nodes disagree on channel `ch`. Blame the one that jumped away from the last agreed
  // value (more than factor x tolerance) if the other stayed within one tolerance of it. Returns
  // true and sets `out` (the consistent node's value) and `blame` when resolved.
  bool arbitrate(unsigned ch, const std::array<float, kNodes>& x, uint8_t valid, float& out,
                 uint8_t& blame) const noexcept {
    // Only a fresh reference (the previous frame was trusted) can single a node out. After a hold the
    // vehicle may have moved arbitrarily far from the stale value, and a healthy node would look like
    // the outlier.
    if (!have_last_[ch] || !ref_fresh_[ch] || cfg_.duplex_arbitration_factor <= 0.0F) {
      return false;
    }
    unsigned idx[2] = {0U, 0U};
    unsigned cnt = 0U;
    for (unsigned i = 0; i < kNodes && cnt < 2U; ++i) {
      if (((valid >> i) & 1U) != 0U) {
        idx[cnt++] = i;
      }
    }
    const float tol = cfg_.tol[ch];
    const float far = cfg_.duplex_arbitration_factor * tol;
    const float dev0 = std::fabs(x[idx[0]] - last_good_[ch]);
    const float dev1 = std::fabs(x[idx[1]] - last_good_[ch]);
    if (dev0 > far && dev1 <= tol) {
      blame = static_cast<uint8_t>(1U << idx[0]);
      out = x[idx[1]];
      return true;
    }
    if (dev1 > far && dev0 <= tol) {
      blame = static_cast<uint8_t>(1U << idx[1]);
      out = x[idx[0]];
      return true;
    }
    return false;
  }

  RedundancyConfig cfg_;
  std::array<ChannelMonitor, kNodes> mon_;
  std::array<StuckDetector, kNodes> stuck_;
  std::array<bool, kNodes> seen_{};
  std::array<NodeState, kNodes> state_{};
  std::array<uint8_t, kNodes> strikes_{};
  std::array<uint8_t, kNodes> latch_cause_{};
  std::array<uint8_t, kNodes> attempts_{};          // probations that failed since the last latch
  std::array<uint32_t, kNodes> last_latch_frame_{};
  std::array<uint16_t, kNodes> dwell_{};
  std::array<uint16_t, kNodes> probation_clean_{};
  std::array<bool, kNodes> req_{};                  // an operator asked for this node's reintegration
  std::array<DecodedGround, kMaxCommandsPerFrame> pending_{};
  unsigned npending_ = 0U;
  uint8_t disabled_by_command_ = 0U;
  std::array<std::array<SeqTracker, kStreams>, kNodes> seq_{};
  std::array<NodeRx, kNodes> rx_{};
  std::array<float, kVoteChannels> last_good_{};   // last trustworthy voted value per channel
  std::array<bool, kVoteChannels> have_last_{};
  std::array<bool, kVoteChannels> ref_fresh_{};    // last_good_ was set by the previous frame's trusted vote
  uint32_t unres_hist_ = 0U;                       // 1 bit per frame: an unresolved disagreement
  bool safe_latched_ = false;
  uint32_t oos_in_frame_ = 0U;
  Counters counters_{};
  FrameReport report_{};
};

}  // namespace tfc
