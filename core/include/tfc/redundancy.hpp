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
//  * Total loss (ADR-014): if no node at all is Healthy there is no voted output to judge a probation
//    against, so the nodes on probation (at least two) judge each other: each frame their data is voted among
//    themselves, and a node that agrees with the cohort is clean, one that is the odd one out is thrown back.
//    The one-at-a-time rule is lifted in that state only. Without this a bus-wide outage plus one more failure
//    would end the flight with no way back short of a reset.
//
// Frames carry the number of the SYNC frame that opened their cycle (ADR-018), checked against the receiver's own frame count
// per stream, so a stream that is a whole frame early, late, replayed or off by any number of frames is seen. Ground commands
// are authenticated (SipHash tag), refused if stale (counter window), and the dangerous ones need an ARM frame before the
// EXECUTE frame; a plain Disable that would leave fewer than two healthy nodes is one of them (ADR-019).
//
// The manager protects itself (ADR-015): its configuration is validated and sanitised at construction, its
// critical state (node states, the Safe flag, the configuration) is stored redundantly and scrubbed every frame,
// and an upset is reported and answered on the safe side. No heap, no exceptions, deterministic.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "tfc/fault_monitor.hpp"
#include "tfc/integrity.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy_types.hpp"
#include "tfc/sensor_health.hpp"
#include "tfc/voter.hpp"

namespace tfc {

// The shared types (node and channel counts, reasons, node states, command results, the configuration) are in redundancy_types.hpp.

// What validate_config() found wrong (bit mask). Each bad field is replaced by its default.
namespace cfgerr {
constexpr uint32_t kTolerance = 1U << 0;     // a vote tolerance that is not a finite positive number
constexpr uint32_t kPersistence = 1U << 1;   // M-of-N outside 1 <= M <= N <= 32
constexpr uint32_t kStuckLimit = 1U << 2;    // fewer than 2 identical frames can never mean "stuck"
constexpr uint32_t kAlpha = 1U << 3;         // leaky-count constants out of range
constexpr uint32_t kArbitration = 1U << 4;   // duplex arbitration factor neither 0 (off) nor >= 1
constexpr uint32_t kLifeCycle = 1U << 5;     // probation / strike limits that contradict each other
constexpr uint32_t kGroundAuth = 1U << 6;    // authentication on with an all-zero key, or a zero counter / arm window
}  // namespace cfgerr

namespace detail {
[[nodiscard]] inline bool finite_positive(float v) noexcept { return v > 0.0F && v <= std::numeric_limits<float>::max(); }
[[nodiscard]] inline bool in_range(float v, float lo, float hi) noexcept { return v >= lo && v <= hi; }  // false for NaN
}  // namespace detail

// A zero, negative or NaN tolerance does not fail loudly: it makes every comparison false, so the voter stops
// seeing disagreement at all. Configuration is data from outside the proven code, so it is checked.
[[nodiscard]] inline uint32_t validate_config(const RedundancyConfig& c) noexcept {
  uint32_t e = 0U;
  for (unsigned i = 0; i < kVoteChannels; ++i) {
    if (!detail::finite_positive(c.tol[i])) {
      e |= cfgerr::kTolerance;
    }
  }
  if (c.persist_m < 1U || c.persist_m > c.persist_n || c.persist_n > 32U || c.digest_persist_frames < 1U) {
    e |= cfgerr::kPersistence;
  }
  if (c.stuck_limit < 2U) {
    e |= cfgerr::kStuckLimit;
  }
  if (!detail::in_range(c.alpha_k, 0.0F, 0.999F) || !detail::in_range(c.alpha_threshold, 0.0F, 1000.0F)) {
    e |= cfgerr::kAlpha;
  }
  const float f = c.duplex_arbitration_factor;  // 0 switches arbitration off; otherwise at least 1 tolerance
  if (!(detail::in_range(f, 0.0F, 1000.0F) && (f >= 1.0F || f <= 0.0F))) {
    e |= cfgerr::kArbitration;
  }
  if (c.probation_frames < 1U || c.probation_frames_repeat < c.probation_frames ||
      (c.max_strikes != 0U && c.max_strikes_physical > c.max_strikes) || c.min_dwell_frames_transient > c.min_dwell_frames) {
    e |= cfgerr::kLifeCycle;
  }
  bool key_zero = true;
  for (const uint8_t b : c.ground_key) {
    key_zero = key_zero && b == 0U;
  }
  if ((c.ground_auth && key_zero) || c.command_window < 1U || c.command_window > 127U || c.arm_window_frames < 1U) {
    e |= cfgerr::kGroundAuth;
  }
  return e;
}

// The configuration the manager actually runs with: every invalid field replaced by its default. `errors`
// receives validate_config()'s verdict on the original.
[[nodiscard]] inline RedundancyConfig sanitize_config(const RedundancyConfig& in, uint32_t& errors) noexcept {
  const RedundancyConfig def{};
  RedundancyConfig c = in;
  errors = validate_config(in);
  for (unsigned i = 0; i < kVoteChannels; ++i) {
    if (!detail::finite_positive(c.tol[i])) {
      c.tol[i] = def.tol[i];
    }
  }
  if ((errors & cfgerr::kPersistence) != 0U) {
    c.persist_m = def.persist_m;
    c.persist_n = def.persist_n;
    c.digest_persist_frames = def.digest_persist_frames;
  }
  if ((errors & cfgerr::kStuckLimit) != 0U) {
    c.stuck_limit = def.stuck_limit;
  }
  if ((errors & cfgerr::kAlpha) != 0U) {
    c.alpha_k = def.alpha_k;
    c.alpha_threshold = def.alpha_threshold;
  }
  if ((errors & cfgerr::kArbitration) != 0U) {
    c.duplex_arbitration_factor = def.duplex_arbitration_factor;
  }
  if ((errors & cfgerr::kLifeCycle) != 0U) {
    if (c.probation_frames < 1U) {
      c.probation_frames = def.probation_frames;
    }
    if (c.probation_frames_repeat < c.probation_frames) {
      c.probation_frames_repeat = static_cast<uint16_t>(def.probation_frames_repeat < c.probation_frames ? c.probation_frames
                                                                                                  : def.probation_frames_repeat);
    }
    if (c.max_strikes != 0U && c.max_strikes_physical > c.max_strikes) {
      c.max_strikes_physical = c.max_strikes;
    }
    if (c.min_dwell_frames_transient > c.min_dwell_frames) {
      c.min_dwell_frames_transient = c.min_dwell_frames;
    }
  }
  if ((errors & cfgerr::kGroundAuth) != 0U) {  // the public bench key is better than a key of zeros, and the error is reported
    bool key_zero = true;
    for (const uint8_t b : c.ground_key) {
      key_zero = key_zero && b == 0U;
    }
    if (key_zero) {
      c.ground_key = def.ground_key;
    }
    if (c.command_window < 1U || c.command_window > 127U) {
      c.command_window = def.command_window;
    }
    if (c.arm_window_frames < 1U) {
      c.arm_window_frames = def.arm_window_frames;
    }
  }
  return c;
}

// A checksum over every field's value (not over the bytes of the struct: padding is indeterminate). It guards
// the configuration copy held in RAM against upsets.
[[nodiscard]] inline uint32_t config_digest(const RedundancyConfig& c) noexcept {
  uint32_t h = 2166136261U;
  auto mix = [&h](uint32_t v) {
    for (unsigned i = 0; i < 4U; ++i) {
      h = (h ^ ((v >> (8U * i)) & 0xFFU)) * 16777619U;
    }
  };
  auto bits = [](float f) {
    uint32_t u = 0U;
    std::memcpy(&u, &f, sizeof u);
    return u;
  };
  for (unsigned i = 0; i < kVoteChannels; ++i) {
    mix(bits(c.tol[i]));
  }
  mix(c.persist_m);
  mix(c.persist_n);
  mix(c.digest_persist_frames);
  mix(c.stuck_limit);
  mix(bits(c.alpha_k));
  mix(bits(c.alpha_threshold));
  mix(c.startup_grace_frames);
  mix(bits(c.duplex_arbitration_factor));
  mix(c.bus_alarm_per_frame);
  mix(static_cast<uint32_t>(c.policy));
  mix(c.min_dwell_frames);
  mix(c.probation_frames);
  mix(c.probation_frames_repeat);
  mix(c.max_strikes);
  mix(c.max_strikes_physical);
  mix(c.physical_causes);
  mix(c.strike_window_frames);
  mix(c.auto_eligible_causes);
  mix(c.auto_max_attempts);
  mix(c.min_dwell_frames_transient);
  mix(c.ground_auth ? 1U : 0U);
  mix(c.command_window);
  mix(c.arm_window_frames);
  for (const uint8_t b : c.ground_key) {
    mix(b);
  }
  return h;
}

struct Counters {
  uint32_t frames = 0;
  uint32_t crc_bad = 0;             // frames discarded for a failed CRC
  uint32_t seq_bad = 0;             // frames whose sequence number was not the expected next
  uint32_t missing = 0;             // node-frames where gyro/accel/command did not all arrive
  uint32_t out_of_schedule = 0;     // frames on IDs that are not part of the schedule
  uint32_t stuck_flags = 0;
  uint32_t digest_flags = 0;
  uint32_t state_corrections = 0;   // computers whose state the resynchronisation found far from the vote (reported by the firmware)
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
  uint32_t integrity_faults = 0;    // upsets found in the manager's own state (ADR-015)
  uint32_t invariant_violations = 0;  // internal invariants that did not hold (counted, recovered from)
  uint32_t commands_unauthentic = 0;  // ground frames whose tag did not verify (dropped silently)
  uint32_t commands_replayed = 0;     // ground frames with a stale or repeated counter (dropped silently)
  uint32_t arms_expired = 0;          // ARM frames that were never followed by their EXECUTE in time
  uint32_t critical_commands = 0;     // commands that removed the last voting node
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
  uint8_t integrity_mask = 0U;      // upsets found and repaired this frame: 1 node state, 2 Safe flag, 4 configuration, 8 invariant, 16 command state
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
  // ---- sensing (ADR-020 case 1). Without the split these mirror the computers' masks and count, so a consumer has one code path. ----
  std::array<uint8_t, kNodes> sensor_reason{};   // reason:: bits per IMU channel this frame
  uint8_t sensor_valid_mask = 0U;                // IMUs whose gyro and accelerometer took part in the sensor votes
  uint8_t sensor_latched_mask = 0U;              // IMUs out of the consensus for any reason
  uint8_t sensor_probation_mask = 0U;
  uint8_t sensor_disabled_mask = 0U;
  uint8_t sensor_newly_latched = 0U;
  uint8_t sensor_newly_disabled = 0U;
  uint8_t sensor_probation_started = 0U;
  uint8_t sensor_probation_failed = 0U;
  uint8_t sensor_newly_reintegrated = 0U;
  unsigned sensor_healthy = 0U;                  // IMU channels that have been seen and are Healthy
  Mode sensor_mode = Mode::Safe;                 // from the number of healthy IMU channels (never Safe on a request: that is the computers' mode)
};

class RedundancyManager {
 public:
  explicit RedundancyManager(const RedundancyConfig& cfg = RedundancyConfig{}) noexcept
      : cfg_(sanitize_config(cfg, config_errors_)),
        cfg_backup_(cfg_),
        cfg_digest_(config_digest(cfg_)),
        cfg_digest_backup_(cfg_digest_),
        mon_{ChannelMonitor(cfg_.persist_m, cfg_.persist_n, 0xFFFFU, 0xFFU),
             ChannelMonitor(cfg_.persist_m, cfg_.persist_n, 0xFFFFU, 0xFFU),
             ChannelMonitor(cfg_.persist_m, cfg_.persist_n, 0xFFFFU, 0xFFU)},
        stuck_{StuckDetector(cfg_.stuck_limit), StuckDetector(cfg_.stuck_limit), StuckDetector(cfg_.stuck_limit)},
        alpha_{AlphaCount(cfg_.alpha_k, cfg_.alpha_threshold), AlphaCount(cfg_.alpha_k, cfg_.alpha_threshold),
               AlphaCount(cfg_.alpha_k, cfg_.alpha_threshold)},
        sensors_(cfg_) {
    for (GuardedByte& g : st_) {
      g.set(static_cast<uint8_t>(NodeState::Healthy));
    }
    safe_.set(0U);
  }

  // Start collecting a frame. The frame number is the manager's own count, which is SYNC's number when the manager runs
  // every frame from the start (FC-A, the sync master); a node that learns the number from SYNC passes it.
  void begin_frame() noexcept {
    rx_ = {};
    oos_in_frame_ = 0U;
    for (auto& per_node : phase_) {
      for (PhaseTracker& p : per_node) {
        p.next_frame();
      }
    }
  }
  void begin_frame(uint32_t frame_no) noexcept {
    frame_no_ = frame_no;
    begin_frame();
  }
  [[nodiscard]] uint32_t frame_number() const noexcept { return frame_no_; }

  // Offer one received frame. Returns false if its ID is not part of the flight-bus schedule
  // (it is then only counted). SYNC, actuator, heartbeat and sim frames are accepted and ignored;
  // ground commands are queued and applied at the end of the frame.
  bool on_frame(const Frame& f) noexcept {
    if (f.id == id::kGround) {
      queue_ground(f);
      return true;
    }
    unsigned stream = 0U;
    unsigned node = 0U;
    if (!classify(f.id, stream, node)) {
      const bool known = f.id == id::kSync || f.id == id::kActOut || (f.id >= id::kSim && f.id <= id::kSimLast) ||
                         (f.id >= id::kHeartbeat && f.id < id::kHeartbeat + kNodes) || (f.id >= id::kState && f.id < id::kState + kNodes) ||
                         (f.id >= id::kResync && f.id < id::kResync + kResyncIds);
      if (!known) {
        ++counters_.out_of_schedule;
        ++oos_in_frame_;
      }
      return known;
    }
    store_sensor_frame(f, stream, node);
    return true;
  }

  // Close the frame: scrub the manager's own state, apply ground commands, judge nodes, vote,
  // cross-check, update FDIR and the node life cycle. The returned reference is valid until the next end_frame().
  const FrameReport& end_frame() noexcept {
    FrameReport& rep = report_;
    rep = FrameReport{};
    ++counters_.frames;
    scrub(rep);
    tick_arm();
    apply_pending(rep);

    std::array<bool, kNodes> good{};
    const uint8_t valid = judge_arrivals(rep, good);
    const VoteSummary vs = vote_channels(rep, valid);
    DigestVerdict dv = digest_outliers(valid);
    digest_run_ = (dv.blame != 0U || dv.unresolved) ? (digest_run_ < 0xFFFFU ? static_cast<uint16_t>(digest_run_ + 1U) : digest_run_) : uint16_t{0};
    if (digest_run_ < cfg_.digest_persist_frames) {
      dv = DigestVerdict{};  // not yet: a mismatch that the resynchronisation may still heal
    }
    tally_votes(vs, dv);
    update_safe(rep, vs.unresolved || dv.unresolved);
    report_bus(rep);

    std::array<bool, kNodes> stuck_now{};
    judge_nodes(rep, good, vs.disagree, dv.blame, stuck_now);
    if (cfg_.sensor_split) {
      judge_sensors(rep, vs);
    }
    advance_life_cycle(rep, good, stuck_now, valid);
    summarize(rep);
    ++frame_no_;
    resync_large_ = 0U;
    return rep;
  }

  // The state resynchronisation found these computers' states far from the vote (bit n = node n; docs/RESYNC.md): called before end_frame() of the frame the resync was in. It is a
  // bad frame for each, through the same detectors as any other reason, so one large correction is not a latch and a computer that keeps needing them is one.
  void report_state_correction(uint8_t large_mask) noexcept {
    resync_large_ = static_cast<uint8_t>(resync_large_ | (large_mask & 0x07U));
    counters_.state_corrections += popcount32(large_mask & 0x07U);
  }

  // ---- operator / ground commands (also reachable as CAN ground frames) ----
  // Start probation for a latched node (after its minimum dwell, one node at a time).
  CommandResult request_reintegration(unsigned node) noexcept { return command(GroundOp::Reintegrate, node); }

  // Apply one command now; results are also reported in FrameReport::commands when they arrive as frames.
  CommandResult command(GroundOp op, unsigned node) noexcept {
    CommandResult r = CommandResult::Accepted;
    const bool imu = cfg_.sensor_split && node >= kSensorBase && node < kSensorBase + kNodes;  // an IMU channel, not a computer
    switch (op) {
      case GroundOp::Reintegrate:
        r = imu ? sensors_.reintegrate(node - kSensorBase) : cmd_reintegrate(node);
        break;
      case GroundOp::Disable:
        r = imu ? sensors_.disable(node - kSensorBase) : cmd_disable(node);
        break;
      case GroundOp::ClearDisabled:
        r = imu ? sensors_.clear_disabled(node - kSensorBase) : cmd_clear_disabled(node);
        break;
      case GroundOp::ClearSafe:
        if (!safe_requested()) {
          r = CommandResult::AlreadyDone;
        }
        clear_safe_request();
        break;
      case GroundOp::Launch:  // the manager does not own the mission clock: the firmware acts on the reported event (the sync master, after the go/no-go)
      case GroundOp::Scrub:
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
    safe_.set(0U);
    unres_hist_ = 0U;
  }

  // A Safe flag that fails its integrity check reads as "requested": the fail-safe side.
  [[nodiscard]] bool safe_requested() const noexcept { return !safe_.intact() || safe_.get() != 0U; }
  [[nodiscard]] bool seen(unsigned node) const noexcept { return node < kNodes && seen_[node]; }
  [[nodiscard]] NodeState state(unsigned node) const noexcept { return node < kNodes ? state_of(node) : NodeState::Disabled; }
  [[nodiscard]] unsigned strikes(unsigned node) const noexcept { return node < kNodes ? strikes_[node] : 0U; }
  // Out of the vote for any reason (latched, on probation, or disabled).
  [[nodiscard]] bool latched(unsigned node) const noexcept { return node < kNodes && state_of(node) != NodeState::Healthy; }
  [[nodiscard]] bool permanent(unsigned node) const noexcept { return node < kNodes && state_of(node) == NodeState::Disabled; }
  // An IMU channel's state and strikes (with the split off, the computer's: an IMU is not judged apart from its computer).
  [[nodiscard]] NodeState sensor_state(unsigned k) const noexcept { return cfg_.sensor_split ? sensors_.state(k) : state(k); }
  [[nodiscard]] unsigned sensor_strikes(unsigned k) const noexcept { return cfg_.sensor_split ? sensors_.strikes(k) : strikes(k); }
  [[nodiscard]] const SensorHealth::Counters& sensor_counters() const noexcept { return sensors_.counters(); }
  [[nodiscard]] const Counters& counters() const noexcept { return counters_; }
  [[nodiscard]] const FrameReport& last_report() const noexcept { return report_; }
  // The configuration in force (sanitised), and what was wrong with the one that was passed in (cfgerr bits).
  [[nodiscard]] const RedundancyConfig& config() const noexcept { return cfg_; }
  [[nodiscard]] uint32_t config_errors() const noexcept { return config_errors_; }

 private:
  friend struct ManagerTestAccess;

  struct NodeRx {
    bool gyro = false;
    bool accel = false;
    bool cmd = false;
    bool crc_bad = false;
    bool seq_bad = false;
    bool sensor_crc_bad = false;  // the same, for the gyro and accelerometer frames only ...
    bool sensor_seq_bad = false;
    bool cmd_crc_bad = false;     // ... and for the command frame only (the sensor split judges them separately)
    bool cmd_seq_bad = false;
    std::array<bool, kStreams> arrived{};  // a frame (good or damaged) came in on this stream
    std::array<float, kVoteChannels> x{};
    uint16_t digest = 0U;
    std::array<uint8_t, 12> raw{};  // gyro + accel payload bytes, for the stuck detector
  };

  struct VoteSummary {
    uint8_t disagree = 0U;    // nodes blamed by any channel's vote (with the split: by a command channel's vote)
    uint8_t sensor_disagree = 0U;  // with the split: IMUs blamed by a gyro or accelerometer vote
    bool unresolved = false;  // a disagreement nobody could be blamed for
  };

  struct DigestVerdict {
    uint8_t blame = 0U;       // nodes in the minority (only when three can be compared)
    bool unresolved = false;  // two comparable nodes disagree: nobody can be blamed
  };

  // Reference for probations when no node is Healthy: the vote among the probationers themselves.
  struct Cohort {
    bool usable = false;
    uint8_t members = 0U;
    std::array<VoteResult, kVoteChannels> votes{};
    DigestVerdict digest{};
  };

  static void add_reason(FrameReport& rep, unsigned n, uint8_t bit) noexcept {
    rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | bit);
  }

  static uint32_t fnv1a(const uint8_t* d, std::size_t n) noexcept {
    uint32_t h = 2166136261U;
    for (std::size_t i = 0; i < n; ++i) {
      h = (h ^ d[i]) * 16777619U;
    }
    return h;
  }

  // ---- guarded node state ----
  [[nodiscard]] bool state_valid(unsigned n) const noexcept { return st_[n].intact() && st_[n].get() <= static_cast<uint8_t>(NodeState::Disabled); }
  // After scrub() a state is always valid; a corrupted one read between scrubs is taken as Latched (excluded).
  [[nodiscard]] NodeState state_of(unsigned n) const noexcept {
    return state_valid(n) ? static_cast<NodeState>(st_[n].get()) : NodeState::Latched;
  }
  void set_state(unsigned n, NodeState s) noexcept { st_[n].set(static_cast<uint8_t>(s)); }
  [[nodiscard]] unsigned count_in_state(NodeState s) const noexcept {
    unsigned c = 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      c += state_of(n) == s ? 1U : 0U;
    }
    return c;
  }

  // ---- receive path ----
  // A ground frame is dropped unless it is intact, authentic (tag) and fresh (counter); only then is it queued. A frame that fails
  // is counted and leaves no trace on the bus, so a flood of garbage on this id cannot be used to learn anything or fill the queue.
  void queue_ground(const Frame& f) noexcept {
    const DecodedGround d = unpack_ground(f);
    if (!d.ok) {
      ++counters_.commands_bad;
      return;
    }
    if (cfg_.ground_auth) {
      if (!ground_authentic(f, cfg_.ground_key)) {
        ++counters_.commands_unauthentic;
        return;
      }
      if (!cmd_have_.intact() || !cmd_ctr_.intact()) {
        cmd_damage_seen_ = true;  // a damaged record counts as "no history"; the next scrub() reports it (this frame overwrites the bytes)
      } else if (cmd_have_.get() == 1U) {
        const uint8_t ahead = static_cast<uint8_t>(d.counter - cmd_ctr_.get());
        if (ahead == 0U || ahead > cfg_.command_window) {
          ++counters_.commands_replayed;  // a repeat, or older than the last command: a replay
          return;
        }
      }
      cmd_ctr_.set(d.counter);
      cmd_have_.set(1U);
    }
    if (npending_ >= kMaxCommandsPerFrame) {
      ++counters_.commands_bad;
    } else {
      pending_[npending_++] = d;
    }
  }

  static bool classify(uint32_t can_id, unsigned& stream, unsigned& node) noexcept {
    if (can_id >= id::kGyroBase && can_id < id::kGyroBase + kNodes) {
      stream = 0U;
      node = can_id - id::kGyroBase;
    } else if (can_id >= id::kAccelBase && can_id < id::kAccelBase + kNodes) {
      stream = 1U;
      node = can_id - id::kAccelBase;
    } else if (can_id >= id::kCmdBase && can_id < id::kCmdBase + kNodes) {
      stream = 2U;
      node = can_id - id::kCmdBase;
    } else {
      return false;
    }
    return true;
  }

  void store_sensor_frame(const Frame& f, unsigned stream, unsigned node) noexcept {
    NodeRx& r = rx_[node];
    r.arrived[stream] = true;
    bool ok = false;
    uint8_t seq = 0U;
    if (stream == 2U) {
      const DecodedCommand d = unpack_cmd(f);
      ok = d.ok;
      seq = d.seq;
      if (ok) {
        r.x[kChPitch] = d.cmd.pitch_deg;
        r.x[kChYaw] = d.cmd.yaw_deg;
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
      (stream == 2U ? r.cmd_crc_bad : r.sensor_crc_bad) = true;
      r.crc_bad = true;  // a damaged frame has no trustworthy number: it did not arrive as a good frame, but its slot was not empty
      ++counters_.crc_bad;
      phase_[node][stream].note_damaged();
      return;
    }
    if (phase_[node][stream].classify(seq, frame_no_) == FrameTiming::Bad) {
      (stream == 2U ? r.cmd_seq_bad : r.sensor_seq_bad) = true;
      r.seq_bad = true;
      ++counters_.seq_bad;
    }
  }

  // ---- self protection (ADR-015) ----
  // Runs first every frame. A node state or the Safe flag that fails its integrity check is repaired on the
  // safe side (node excluded, Safe requested); a configuration that fails its checksum is restored from its
  // second copy and Safe is requested, because the frame just before may have been judged with a wrong limit.
  void scrub(FrameReport& rep) noexcept {
    uint8_t mask = 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      if (!state_valid(n)) {
        set_state(n, NodeState::Latched);
        dwell_[n] = 0U;
        dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << n));
        req_[n] = false;
        probation_clean_[n] = 0U;
        mask = static_cast<uint8_t>(mask | 1U);
      }
    }
    if (cfg_.sensor_split && sensors_.scrub()) {
      mask = static_cast<uint8_t>(mask | 1U);  // an IMU channel's state failed its check: excluded, like a computer's
    }
    if (!safe_.intact() || safe_.get() > 1U) {
      safe_.set(1U);
      mask = static_cast<uint8_t>(mask | 2U);
    }
    if (!repair_config()) {
      safe_.set(1U);
      mask = static_cast<uint8_t>(mask | 4U);
    }
    if (cmd_damage_seen_) {  // found (and already overwritten with the new command's counter) on the receive path: only report it
      cmd_damage_seen_ = false;
      mask = static_cast<uint8_t>(mask | 16U);
    }
    if (!cmd_ctr_.intact() || !cmd_have_.intact() || !arm_code_.intact() || !arm_left_.intact()) {
      cmd_have_.set(0U);  // the counter history is gone: the next authentic command is accepted and re-establishes it
      cmd_ctr_.set(0U);
      clear_arm();        // and nothing stays armed on a damaged record
      mask = static_cast<uint8_t>(mask | 16U);
    }
    if (!ensure(npending_ <= kMaxCommandsPerFrame, counters_.invariant_violations)) {
      npending_ = 0U;  // a corrupted queue length must not be used as a loop bound
      mask = static_cast<uint8_t>(mask | 8U);
    }
    if (mask != 0U) {
      rep.integrity_mask = mask;
      ++counters_.integrity_faults;
    }
  }

  // True if both configuration copies were intact. Otherwise restores what it can and returns false.
  bool repair_config() noexcept {
    const bool ok_a = config_digest(cfg_) == cfg_digest_;
    const bool ok_b = config_digest(cfg_backup_) == cfg_digest_backup_ && cfg_digest_backup_ == cfg_digest_;
    if (ok_a && ok_b) {
      return true;
    }
    if (ok_a) {
      cfg_backup_ = cfg_;
      cfg_digest_backup_ = cfg_digest_;
    } else if (config_digest(cfg_backup_) == cfg_digest_backup_) {
      cfg_ = cfg_backup_;
      cfg_digest_ = cfg_digest_backup_;
    } else {  // both copies are gone: the defaults are the last resort
      uint32_t ignored = 0U;
      cfg_ = sanitize_config(RedundancyConfig{}, ignored);
      cfg_backup_ = cfg_;
      cfg_digest_ = config_digest(cfg_);
      cfg_digest_backup_ = cfg_digest_;
    }
    return false;
  }

  void apply_pending(FrameReport& rep) noexcept {
    for (unsigned i = 0; i < npending_ && i < kMaxCommandsPerFrame; ++i) {
      CommandEvent ev;
      ev.op = pending_[i].op;
      ev.node = pending_[i].node;
      ev.result = apply_ground(pending_[i], ev.flags);
      rep.commands[rep.command_count++] = ev;
    }
    npending_ = 0U;
    // Disables made by command (frame or direct call) since the last frame are reported now.
    rep.newly_disabled = static_cast<uint8_t>(rep.newly_disabled | disabled_by_command_);
    disabled_by_command_ = 0U;
  }

  // ---- ground commands through the frame path: ARM, interlock, EXECUTE (ADR-019) ----
  void clear_arm() noexcept {
    arm_code_.set(0U);
    arm_left_.set(0U);
  }

  static uint8_t arm_code_for(GroundOp op, unsigned node) noexcept {
    return static_cast<uint8_t>((static_cast<unsigned>(op) << 3U) | (node & 7U));  // node 0 to 2: a computer; 4 to 6: an IMU channel (the sensor split)
  }

  [[nodiscard]] bool armed_for(GroundOp op, unsigned node) const noexcept {
    return arm_left_.get() > 0U && arm_code_.get() == arm_code_for(op, node);
  }

  void tick_arm() noexcept {
    if (arm_left_.get() == 0U) {
      return;
    }
    arm_left_.set(static_cast<uint8_t>(arm_left_.get() - 1U));
    if (arm_left_.get() == 0U) {
      clear_arm();
      ++counters_.arms_expired;
    }
  }

  // Does executing `op` on `node` need an ARM first, and would it remove the last voting node?
  struct Needs {
    bool arm = false;
    bool critical = false;
  };
  Needs needs_arm(GroundOp op, unsigned node) const noexcept {
    Needs n;
    if (op == GroundOp::ClearDisabled || op == GroundOp::ClearSafe || op == GroundOp::Launch) {
      n.arm = true;  // both undo a protective action
    } else if (op == GroundOp::Disable) {
      const bool sensor = node >= kSensorBase;
      if (sensor ? sensors_.healthy(node - kSensorBase) : state_of(node) == NodeState::Healthy) {
        const unsigned healthy = sensor ? sensors_.count_healthy() : count_in_state(NodeState::Healthy);
        n.arm = healthy <= 2U;       // Triplex -> Duplex is plain; Duplex -> Simplex and Simplex -> nothing are not
        n.critical = healthy <= 1U;  // the last voter
      }
    }
    return n;
  }

  CommandResult apply_ground(const DecodedGround& d, uint8_t& flags) noexcept {
    if (d.op < static_cast<uint8_t>(GroundOp::Reintegrate) || d.op > static_cast<uint8_t>(GroundOp::Scrub)) {
      ++counters_.commands_refused;
      return CommandResult::RefusedBadOp;
    }
    const GroundOp op = static_cast<GroundOp>(d.op);
    const unsigned node = (op == GroundOp::ClearSafe || op == GroundOp::Launch || op == GroundOp::Scrub) ? 0U : d.node;
    const bool imu_op = cfg_.sensor_split && node >= kSensorBase && node < kSensorBase + kNodes;  // (the other operations were given node 0 above)
    if (node >= kNodes && !imu_op) {
      ++counters_.commands_refused;
      return CommandResult::RefusedBadNode;
    }
    if (d.arm) {
      arm_code_.set(arm_code_for(op, node));
      arm_left_.set(cfg_.arm_window_frames);
      flags = static_cast<uint8_t>(flags | cmdflag::kArm);
      ++counters_.commands_accepted;
      return CommandResult::Accepted;
    }
    const Needs need = needs_arm(op, node);
    if (need.arm) {
      if (!armed_for(op, node)) {
        ++counters_.commands_refused;
        return CommandResult::RefusedNotArmed;
      }
      clear_arm();  // an ARM covers exactly one EXECUTE
      flags = static_cast<uint8_t>(flags | cmdflag::kArmed);
    }
    const CommandResult r = command(op, node);
    if (need.critical && r == CommandResult::Accepted) {
      flags = static_cast<uint8_t>(flags | cmdflag::kCritical);
      ++counters_.critical_commands;
    }
    return r;
  }

  // ---- commands ----
  CommandResult cmd_reintegrate(unsigned node) noexcept {
    if (node >= kNodes) {
      return CommandResult::RefusedBadNode;
    }
    const NodeState st = state_of(node);
    if (st == NodeState::Disabled) {
      return CommandResult::RefusedDisabled;
    }
    if (st == NodeState::Healthy) {
      return CommandResult::RefusedNotLatched;
    }
    if (st == NodeState::Probation || req_[node]) {
      return CommandResult::AlreadyDone;
    }
    req_[node] = true;
    return CommandResult::Accepted;
  }

  CommandResult cmd_disable(unsigned node) noexcept {
    if (node >= kNodes) {
      return CommandResult::RefusedBadNode;
    }
    if (state_of(node) == NodeState::Disabled) {
      return CommandResult::AlreadyDone;
    }
    set_state(node, NodeState::Disabled);
    req_[node] = false;
    ++counters_.nodes_disabled;
    disabled_by_command_ = static_cast<uint8_t>(disabled_by_command_ | (1U << node));
    return CommandResult::Accepted;
  }

  CommandResult cmd_clear_disabled(unsigned node) noexcept {
    if (node >= kNodes) {
      return CommandResult::RefusedBadNode;
    }
    if (state_of(node) != NodeState::Disabled) {
      return CommandResult::RefusedNotDisabled;
    }
    set_state(node, NodeState::Latched);  // back to square one: dwell, then a fresh request
    strikes_[node] = 0U;
    dwell_[node] = 0U;
    dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << node));  // the frame of the command is not part of the dwell
    attempts_[node] = 0U;
    req_[node] = false;
    return CommandResult::Accepted;
  }

  // ---- one frame, step by step ----
  // Who delivered what; returns the mask of nodes whose data takes part in the vote.
  uint8_t judge_arrivals(FrameReport& rep, std::array<bool, kNodes>& good) noexcept {
    uint8_t valid = 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      const NodeRx& r = rx_[n];
      const bool split = cfg_.sensor_split;  // with the split, a computer is judged on its command frame only; its IMU has its own verdict
      const bool present = split ? r.cmd : (r.gyro && r.accel && r.cmd);
      const bool crc_bad = split ? r.cmd_crc_bad : r.crc_bad;
      const bool seq_bad = split ? r.cmd_seq_bad : r.seq_bad;
      const bool in_grace = !seen_[n] && counters_.frames <= cfg_.startup_grace_frames;
      if (!present && !crc_bad && !in_grace) {
        ++counters_.missing;
        add_reason(rep, n, reason::kMissing);
      }
      if (crc_bad) {
        add_reason(rep, n, reason::kCrc);
      }
      if (seq_bad) {
        add_reason(rep, n, reason::kSeq);
      }
      good[n] = present && !crc_bad && !seq_bad;
      if (good[n] && !seen_[n]) {
        seen_[n] = true;
        rep.newly_seen = static_cast<uint8_t>(rep.newly_seen | (1U << n));
      }
      if (good[n] && state_of(n) == NodeState::Healthy) {
        valid = static_cast<uint8_t>(valid | (1U << n));
      }
    }
    rep.valid_mask = valid;
    if (cfg_.sensor_split) {
      judge_sensor_arrivals(rep);
    }
    return valid;
  }

  // The sensor side of the arrivals (the split on): which IMUs delivered both frames, clean and in sequence, and why not.
  void judge_sensor_arrivals(FrameReport& rep) noexcept {
    uint8_t svalid = 0U;
    for (unsigned k = 0; k < kNodes; ++k) {
      const NodeRx& r = rx_[k];
      const bool present = r.gyro && r.accel;
      const bool in_grace = !sensors_.seen(k) && counters_.frames <= cfg_.startup_grace_frames;
      uint8_t why = 0U;
      if (!present && !r.sensor_crc_bad && !in_grace) {
        why = static_cast<uint8_t>(why | reason::kMissing);
      }
      if (r.sensor_crc_bad) {
        why = static_cast<uint8_t>(why | reason::kCrc);
      }
      if (r.sensor_seq_bad) {
        why = static_cast<uint8_t>(why | reason::kSeq);
      }
      sensor_arrival_[k] = why;
      sensor_good_[k] = present && !r.sensor_crc_bad && !r.sensor_seq_bad;
      if (sensor_good_[k]) {
        (void)sensors_.note_good(k);
      }
      if (sensor_good_[k] && sensors_.healthy(k)) {
        svalid = static_cast<uint8_t>(svalid | (1U << k));
      }
    }
    sensor_valid_ = svalid;
    rep.sensor_valid_mask = svalid;
  }

  // The sensor channels' verdicts for the frame (the split on), from the votes of the six sensor values.
  void judge_sensors(FrameReport& rep, const VoteSummary& vs) noexcept {
    SensorHealth::Input in;
    in.good = sensor_good_;
    in.blame = vs.sensor_disagree;
    for (unsigned k = 0; k < kNodes; ++k) {
      in.x[k] = &rx_[k].x;
      in.stuck_hash[k] = fnv1a(rx_[k].raw.data(), rx_[k].raw.size());
    }
    in.output = &rep.output;
    in.tol = &cfg_.tol;
    in.output_trusted = (rep.held_mask & 0x3FU) == 0U && !rep.safe_request;
    in.healthy_valid = sensor_valid_;
    sensor_report_ = SensorHealth::Report{};
    sensors_.update(in, cfg_, sensor_arrival_, sensor_report_);
    for (unsigned k = 0; k < kNodes; ++k) {
      counters_.stuck_flags += (sensor_report_.reason[k] & reason::kStuck) != 0U ? 1U : 0U;
    }
  }

  // Vote the 8 channels; fills rep.votes / output / held_mask.
  VoteSummary vote_channels(FrameReport& rep, uint8_t valid) noexcept {
    VoteSummary s;
    const bool safe_now = safe_requested();  // the flag as of the end of the previous frame
    for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
      const std::array<float, kNodes> x = {rx_[0].x[ch], rx_[1].x[ch], rx_[2].x[ch]};
      const bool on_sensors = cfg_.sensor_split && ch < kSensorValues;  // the six sensor values are voted among the healthy IMUs, the commands among the healthy computers
      const uint8_t mask = on_sensors ? sensor_valid_ : valid;
      const VoteResult v = vote3(x, mask, cfg_.tol[ch]);
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
          trusted = arbitrate(ch, x, mask, out, blame);
          if (!trusted) {
            s.unresolved = true;
          }
          break;
        case VoteStatus::NoMajority:
        case VoteStatus::NoData:
        default:
          trusted = false;
          break;
      }
      if (trusted && !safe_now) {
        prev_good_[ch] = last_good_[ch];
        have_prev_[ch] = have_last_[ch] && ref_fresh_[ch];  // the value being replaced was itself fresh: a motion can be inferred
        last_good_[ch] = out;
        have_last_[ch] = true;
        ref_fresh_[ch] = true;
        rep.output[ch] = out;
      } else {
        ref_fresh_[ch] = false;  // a held reference goes stale: it must not be used to blame anyone
        have_prev_[ch] = false;
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;  // hold the last good value
        rep.held_mask = static_cast<uint8_t>(rep.held_mask | (1U << ch));
      }
      uint8_t& blamed = on_sensors ? s.sensor_disagree : s.disagree;
      blamed = static_cast<uint8_t>(blamed | blame);
    }
    return s;
  }

  void tally_votes(const VoteSummary& vs, const DigestVerdict& dv) noexcept {
    if (vs.disagree != 0U) {
      ++counters_.vote_disagreements;
    }
    if (dv.blame != 0U || dv.unresolved) {
      ++counters_.digest_flags;
    }
  }

  // Persistent unresolved disagreement requests Safe (sticky), and Safe holds every output.
  void update_safe(FrameReport& rep, bool unresolved) noexcept {
    rep.unresolved = unresolved;
    unres_hist_ = (unres_hist_ << 1) | (unresolved ? 1U : 0U);
    const uint32_t window = cfg_.persist_n >= 32U ? 0xFFFFFFFFU : ((1U << cfg_.persist_n) - 1U);
    if (popcount32(unres_hist_ & window) >= cfg_.persist_m) {
      safe_.set(1U);  // sticky: only clear_safe_request() (operator/ground) lifts it
    }
    rep.safe_request = safe_requested();
    if (rep.safe_request) {  // hold the last voted values for the whole time Safe is requested
      for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;
      }
      rep.held_mask = 0xFFU;
    }
    counters_.unresolved_frames += unresolved ? 1U : 0U;
    counters_.safe_request_frames += rep.safe_request ? 1U : 0U;
    counters_.held_frames += rep.held_mask != 0U ? 1U : 0U;
  }

  // Babbling-idiot alarm: too many out-of-schedule frames inside one 10 ms frame.
  void report_bus(FrameReport& rep) noexcept {
    rep.out_of_schedule_in_frame = oos_in_frame_;
    rep.bus_alarm = oos_in_frame_ >= cfg_.bus_alarm_per_frame && cfg_.bus_alarm_per_frame != 0U;
    counters_.bus_alarm_frames += rep.bus_alarm ? 1U : 0U;
  }

  // Per node: stuck detector, then the reasons of this frame, then the latch decision.
  void judge_nodes(FrameReport& rep, const std::array<bool, kNodes>& good, uint8_t disagree, uint8_t digest_bad,
                   std::array<bool, kNodes>& stuck_now) noexcept {
    for (unsigned n = 0; n < kNodes; ++n) {
      if (good[n] && !cfg_.sensor_split) {  // (with the split a frozen sensor is the IMU's verdict)
        stuck_now[n] = stuck_[n].update(static_cast<int32_t>(fnv1a(rx_[n].raw.data(), rx_[n].raw.size())));
      }
      if (stuck_now[n]) {
        ++counters_.stuck_flags;
        add_reason(rep, n, reason::kStuck);
      }
      if (((disagree >> n) & 1U) != 0U) {
        add_reason(rep, n, reason::kVote);
      }
      if (((digest_bad >> n) & 1U) != 0U) {
        add_reason(rep, n, reason::kDigest);
      }
      if (((resync_large_ >> n) & 1U) != 0U) {
        add_reason(rep, n, reason::kResync);
      }
      if (state_of(n) == NodeState::Healthy) {
        decide_latch(n, rep);
      }
    }
  }

  // Feed this frame's verdict to the node's two detectors: the M-of-N window and the leaky count.
  void decide_latch(unsigned n, FrameReport& rep) noexcept {
    const bool bad = rep.reason[n] != 0U;
    const bool by_window = mon_[n].update(bad);
    const bool by_alpha = alpha_[n].update(bad);
    if (!by_window && !by_alpha) {
      return;
    }
    if (!by_window) {
      add_reason(rep, n, reason::kIntermittent);
    }
    on_latch(n, rep);
  }

  // Probation verdicts first, then dwell and new probations.
  void advance_life_cycle(FrameReport& rep, const std::array<bool, kNodes>& good, const std::array<bool, kNodes>& stuck_now,
                          uint8_t valid) noexcept {
    const Cohort cohort = cohort_reference(good, stuck_now);
    for (unsigned n = 0; n < kNodes; ++n) {
      if (state_of(n) == NodeState::Probation) {
        judge_probation(n, good[n], stuck_now[n], valid, cohort, rep);
      }
    }
    start_probations(rep);
    dwell_hold_ = 0U;
  }

  // Count the dwell of latched nodes and put the ones that are due (and wanted) on probation. One at a time,
  // unless no node is Healthy: then the probationers are each other's only reference (ADR-014).
  void start_probations(FrameReport& rep) noexcept {
    bool one_on_probation = count_in_state(NodeState::Probation) != 0U;
    const bool no_healthy = count_in_state(NodeState::Healthy) == 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      if (state_of(n) != NodeState::Latched) {
        continue;
      }
      if (((dwell_hold_ >> n) & 1U) == 0U && dwell_[n] < 0xFFFFU) {
        ++dwell_[n];  // the frame in which a node latched, failed probation or was cleared is not part of its dwell
      }
      if ((!one_on_probation || no_healthy) && dwell_[n] >= dwell_needed(n) && wants_probation(n)) {
        set_state(n, NodeState::Probation);
        req_[n] = false;
        probation_clean_[n] = 0U;
        one_on_probation = true;
        rep.probation_started = static_cast<uint8_t>(rep.probation_started | (1U << n));
        ++counters_.probations_started;
      }
    }
  }

  void summarize(FrameReport& rep) noexcept {
    for (unsigned n = 0; n < kNodes; ++n) {
      rep.strikes[n] = strikes_[n];
      const NodeState st = state_of(n);
      if (st != NodeState::Healthy) {
        rep.latched_mask = static_cast<uint8_t>(rep.latched_mask | (1U << n));
      }
      if (st == NodeState::Probation) {
        rep.probation_mask = static_cast<uint8_t>(rep.probation_mask | (1U << n));
      }
      if (st == NodeState::Disabled) {
        rep.disabled_mask = static_cast<uint8_t>(rep.disabled_mask | (1U << n));
      }
      if (st == NodeState::Healthy && seen_[n]) {
        ++rep.healthy;
      }
    }
    rep.mode = rep.safe_request ? Mode::Safe : mode_from_healthy(rep.healthy);
    if (cfg_.sensor_split) {
      const SensorHealth::Report& sr = sensor_report_;
      rep.sensor_reason = sr.reason;
      rep.sensor_latched_mask = sr.latched_mask;
      rep.sensor_probation_mask = sr.probation_mask;
      rep.sensor_disabled_mask = sr.disabled_mask;
      rep.sensor_newly_latched = sr.newly_latched;
      rep.sensor_newly_disabled = sr.newly_disabled;
      rep.sensor_probation_started = sr.probation_started;
      rep.sensor_probation_failed = sr.probation_failed;
      rep.sensor_newly_reintegrated = sr.newly_reintegrated;
      rep.sensor_healthy = sr.healthy;
    } else {
      rep.sensor_valid_mask = rep.valid_mask;
      rep.sensor_latched_mask = rep.latched_mask;
      rep.sensor_probation_mask = rep.probation_mask;
      rep.sensor_disabled_mask = rep.disabled_mask;
      rep.sensor_healthy = rep.healthy;
    }
    rep.sensor_mode = mode_from_healthy(rep.sensor_healthy);
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
    dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << n));
    req_[n] = false;
    attempts_[n] = 0U;
    if (limit != 0U && strikes_[n] >= limit) {
      set_state(n, NodeState::Disabled);
      rep.newly_disabled = static_cast<uint8_t>(rep.newly_disabled | (1U << n));
      ++counters_.nodes_disabled;
    } else {
      set_state(n, NodeState::Latched);
    }
  }

  // How long a latched node waits before it may go on probation: the short dwell after a first, transient-looking latch.
  [[nodiscard]] uint16_t dwell_needed(unsigned n) const noexcept {
    const bool transient = strikes_[n] == 1U && (latch_cause_[n] & ~cfg_.auto_eligible_causes) == 0U;
    return transient ? cfg_.min_dwell_frames_transient : cfg_.min_dwell_frames;
  }

  bool wants_probation(unsigned n) const noexcept {
    if (req_[n]) {
      return true;
    }
    return cfg_.policy == ReintegrationPolicy::AutoTransient && strikes_[n] == 1U &&
           (latch_cause_[n] & ~cfg_.auto_eligible_causes) == 0U && attempts_[n] < cfg_.auto_max_attempts;
  }

  // The vote among nodes on probation, used only when no node is Healthy (so there is no voted output).
  Cohort cohort_reference(const std::array<bool, kNodes>& good, const std::array<bool, kNodes>& stuck_now) const noexcept {
    Cohort c;
    for (unsigned n = 0; n < kNodes; ++n) {
      if (state_of(n) == NodeState::Probation && good[n] && !stuck_now[n]) {
        c.members = static_cast<uint8_t>(c.members | (1U << n));
      }
    }
    if (count_in_state(NodeState::Healthy) != 0U || count_channels(c.members) < 2U) {
      return c;
    }
    c.usable = true;
    for (unsigned ch = first_channel(); ch < kVoteChannels; ++ch) {
      const std::array<float, kNodes> x = {rx_[0].x[ch], rx_[1].x[ch], rx_[2].x[ch]};
      c.votes[ch] = vote3(x, c.members, cfg_.tol[ch]);
    }
    c.digest = digest_outliers(c.members);
    return c;
  }

  // Judge one probationer against its cohort: the odd one out is dirty; if nobody can be singled out (two
  // members that disagree, three that all differ) the frame is neutral.
  Verdict cohort_verdict(unsigned n, const Cohort& c, FrameReport& rep) const noexcept {
    bool neutral = false;
    for (unsigned ch = first_channel(); ch < kVoteChannels; ++ch) {
      const VoteResult& v = c.votes[ch];
      if (v.status == VoteStatus::Triplex) {
        if (((v.disagree_mask >> n) & 1U) != 0U) {
          add_reason(rep, n, reason::kVote);
          return Verdict::Dirty;
        }
      } else if (v.status != VoteStatus::Duplex) {
        neutral = true;
      }
    }
    if (c.digest.unresolved) {
      return Verdict::Neutral;
    }
    if (((c.digest.blame >> n) & 1U) != 0U) {
      add_reason(rep, n, reason::kDigest);
      return Verdict::Dirty;
    }
    return neutral ? Verdict::Neutral : Verdict::Clean;
  }

  // Shadow vote: compare a probationer with the voted output and the digest of the healthy nodes.
  Verdict shadow_verdict(unsigned n, uint8_t valid, FrameReport& rep) const noexcept {
    for (unsigned ch = first_channel(); ch < kVoteChannels; ++ch) {
      if (std::fabs(rx_[n].x[ch] - rep.output[ch]) > cfg_.tol[ch]) {
        add_reason(rep, n, reason::kVote);  // disagrees with the shadow vote
        return Verdict::Dirty;
      }
    }
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
      return Verdict::Neutral;
    }
    if (have_ref && rx_[n].digest != ref) {
      add_reason(rep, n, reason::kDigest);
      return Verdict::Dirty;
    }
    return Verdict::Clean;
  }

  // One frame of a node on probation.
  void judge_probation(unsigned n, bool good, bool stuck, uint8_t valid, const Cohort& cohort, FrameReport& rep) noexcept {
    Verdict v = Verdict::Clean;
    if (!good || stuck) {
      v = Verdict::Dirty;  // a missing, damaged, out-of-sequence or frozen frame
    } else if (cohort.usable) {
      v = cohort_verdict(n, cohort, rep);
    } else if (valid == 0U || (rep.held_mask & (cfg_.sensor_split ? 0xC0U : 0xFFU)) != 0U) {
      v = Verdict::Neutral;  // no trustworthy reference this frame (nobody healthy, Safe, unresolved)
    } else {
      v = shadow_verdict(n, valid, rep);
    }
    if (v == Verdict::Dirty) {
      set_state(n, NodeState::Latched);  // thrown back: dwell starts again, a new request is needed
      dwell_[n] = 0U;
      dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << n));
      req_[n] = false;
      if (attempts_[n] < 0xFFU) {
        ++attempts_[n];
      }
      rep.probation_failed = static_cast<uint8_t>(rep.probation_failed | (1U << n));
      ++counters_.probation_failures;
    } else if (v == Verdict::Clean) {
      const unsigned needed = strikes_[n] >= 2U ? cfg_.probation_frames_repeat : cfg_.probation_frames;
      if (++probation_clean_[n] >= needed) {
        set_state(n, NodeState::Healthy);
        mon_[n].force_unlatch();
        alpha_[n].reset();
        stuck_[n] = StuckDetector(cfg_.stuck_limit);
        rep.newly_reintegrated = static_cast<uint8_t>(rep.newly_reintegrated | (1U << n));
        ++counters_.reintegrations;
      }
    }
  }

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
      } else if (b == c) {
        d.blame = 0x1U;
      } else {
        d.unresolved = true;  // three different digests: no majority, nobody can be singled out (E17)
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

  // The first channel a computer's probation is judged on: all eight, or with the sensor split only the two command channels (its IMU has its own probation).
  [[nodiscard]] unsigned first_channel() const noexcept { return cfg_.sensor_split ? kChPitch : 0U; }

  // Where the signal should be now. The step is added only as far as the signal really moves: below one tolerance per frame it is
  // indistinguishable from the noise of the two samples it is made of (adding it only widens and shifts the decision band: E19),
  // from two tolerances up it is used in full (the command channels, where the standstill reference blamed the wrong node: E16).
  [[nodiscard]] float reference(unsigned ch, float tol) const noexcept {
    if (!have_prev_[ch]) {
      return last_good_[ch];
    }
    const float step = last_good_[ch] - prev_good_[ch];
    const float moving = std::fabs(step) / tol;
    const float weight = moving <= 1.0F ? 0.0F : (moving >= 2.0F ? 1.0F : moving - 1.0F);
    return last_good_[ch] + weight * step;
  }

  // Two valid nodes disagree on channel `ch`. Judge them against where the signal should be now: the last agreed
  // value carried forward by the last agreed step (the motion), or the last agreed value itself if there is no step
  // to infer. Blame the node that is farther than factor x tolerance from it, if the other stayed within one
  // tolerance. Returns true and sets `out` (the consistent node's value) and `blame` when resolved.
  //
  // Why the motion matters (E16): against a standstill reference a node that stopped following a moving signal (a
  // frozen command, a stale frame) looks like the stable one and the healthy node looks like the one that "jumped".
  // When the signal moves more than 2 tolerances per frame (the command channels, tolerance 0.01 deg) that blamed the
  // healthy node every time and left the frozen one as the only voter.
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
    const float ref = reference(ch, tol);
    const float far = cfg_.duplex_arbitration_factor * tol;
    const float dev0 = std::fabs(x[idx[0]] - ref);
    const float dev1 = std::fabs(x[idx[1]] - ref);
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

  uint32_t config_errors_ = 0U;  // must precede cfg_: filled in while it is initialised
  RedundancyConfig cfg_;
  RedundancyConfig cfg_backup_;  // second copy: the first is checked against a checksum every frame
  uint32_t cfg_digest_;
  std::array<bool, kNodes> sensor_good_{};        // per IMU: both frames arrived clean this frame (the split on)
  std::array<uint8_t, kNodes> sensor_arrival_{};  // ... and the arrival reasons if not
  uint8_t sensor_valid_ = 0U;                     // the IMUs that took part in the sensor votes
  uint32_t cfg_digest_backup_;
  std::array<ChannelMonitor, kNodes> mon_;
  std::array<StuckDetector, kNodes> stuck_;
  std::array<AlphaCount, kNodes> alpha_;
  SensorHealth sensors_;                     // the sensor channels' life cycle, used when cfg_.sensor_split (ADR-020 case 1)
  SensorHealth::Report sensor_report_{};      // its report for the frame in progress
  std::array<bool, kNodes> seen_{};
  std::array<GuardedByte, kNodes> st_{};            // NodeState, stored with its complement
  GuardedByte safe_{};                              // Safe requested (0/1), stored with its complement
  std::array<uint8_t, kNodes> strikes_{};
  std::array<uint8_t, kNodes> latch_cause_{};
  std::array<uint8_t, kNodes> attempts_{};          // probations that failed since the last latch
  std::array<uint32_t, kNodes> last_latch_frame_{};
  std::array<uint16_t, kNodes> dwell_{};
  uint8_t dwell_hold_ = 0U;                         // nodes whose dwell does not advance in this frame
  std::array<uint16_t, kNodes> probation_clean_{};
  std::array<bool, kNodes> req_{};                  // an operator asked for this node's reintegration
  std::array<DecodedGround, kMaxCommandsPerFrame> pending_{};
  unsigned npending_ = 0U;
  uint8_t disabled_by_command_ = 0U;
  std::array<std::array<PhaseTracker, kStreams>, kNodes> phase_{};
  uint32_t frame_no_ = 0U;                          // the frame being collected (SYNC's number for the sync master)
  GuardedByte cmd_ctr_{};                           // counter of the last accepted ground command
  GuardedByte cmd_have_{};                          // 1 once a command has been accepted
  GuardedByte arm_code_{};                          // 0 = nothing armed, else (op << 2 | node)
  GuardedByte arm_left_{};                          // frames the ARM stays valid
  bool cmd_damage_seen_ = false;                    // queue_ground() found the counter record damaged; scrub() reports it
  std::array<NodeRx, kNodes> rx_{};
  std::array<float, kVoteChannels> last_good_{};   // last trustworthy voted value per channel
  std::array<float, kVoteChannels> prev_good_{};   // the trustworthy value before that (gives the motion)
  std::array<bool, kVoteChannels> have_prev_{};    // prev_good_ and last_good_ are consecutive trusted frames
  std::array<bool, kVoteChannels> have_last_{};
  std::array<bool, kVoteChannels> ref_fresh_{};    // last_good_ was set by the previous frame's trusted vote
  uint16_t digest_run_ = 0U;                       // consecutive frames with a digest disagreement
  uint8_t resync_large_ = 0U;                      // computers reported far from the resync vote, for this frame
  uint32_t unres_hist_ = 0U;                       // 1 bit per frame: an unresolved disagreement
  uint32_t oos_in_frame_ = 0U;
  Counters counters_{};
  FrameReport report_{};
};

static_assert(kNodes == 3U && kVoteChannels == 8U && kStreams == 3U, "the vote tables below are sized for 3 nodes x 8 channels");
static_assert(sizeof(RedundancyConfig::tol) / sizeof(float) == kVoteChannels, "one tolerance per vote channel");
static_assert(static_cast<uint8_t>(NodeState::Disabled) == 3U, "NodeState values are range-checked against Disabled");
static_assert(kMaxCommandsPerFrame >= 1U && kMaxCommandsPerFrame <= 255U, "command queue bound");

}  // namespace tfc
