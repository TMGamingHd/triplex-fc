// SPDX-License-Identifier: MIT
// Sensor-channel health, separate from compute-channel health (ADR-020 case 1, TFC-ARCH-001, TS-15).
//
// With the split on, a gyro/accelerometer pair (one per IMU: channel k is hosted by computer k) is judged and isolated on its own life cycle, and the computer that hosts
// it stays a voter for commands for as long as its commands agree. The unit of isolation for sensing is the IMU; for computing it is the computer. A bad IMU is excluded from
// the sensor consensus (every computer sees the same samples, so all exclude the same channel in the same frame and the replicas stay identical), and the computer's command
// is judged on its own merits by the command vote and the state digest.
//
// The life cycle is the node's (ADR-010, ADR-013, ADR-014) applied to a channel: 3-of-5 window plus the leaky count, latch with a strike, a dwell, a probation that compares the
// channel with the voted sensor outputs (or, when no channel is healthy, with the other probationers), readmission after a run of agreeing frames, and disabling after the strike
// limit. The manager (`redundancy.hpp`) owns the votes and calls this once per frame. No heap, no exceptions, no RTTI. Every loop is bounded.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/fault_monitor.hpp"
#include "tfc/integrity.hpp"
#include "tfc/redundancy_types.hpp"
#include "tfc/voter.hpp"

namespace tfc {

constexpr unsigned kSensorChannels = 3U;   // one per IMU, hosted by the computer of the same number
constexpr unsigned kSensorValues = 6U;     // gyro x, y, z, accel x, y, z

class SensorHealth {
 public:
  friend struct ManagerTestAccess;  // fault injection in the host tests only
  explicit SensorHealth(const RedundancyConfig& cfg) noexcept
      : mon_{ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU), ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, 0xFFFFU, 0xFFU)},
        alpha_{AlphaCount(cfg.alpha_k, cfg.alpha_threshold), AlphaCount(cfg.alpha_k, cfg.alpha_threshold), AlphaCount(cfg.alpha_k, cfg.alpha_threshold)},
        stuck_{StuckDetector(cfg.stuck_limit), StuckDetector(cfg.stuck_limit), StuckDetector(cfg.stuck_limit)} {
    for (GuardedByte& g : st_) {
      g.set(static_cast<uint8_t>(NodeState::Healthy));
    }
  }

  // What the manager hands over each frame, after the votes of the six sensor values.
  struct Input {
    std::array<bool, kSensorChannels> good{};         // both frames arrived, undamaged and in sequence
    uint8_t blame = 0U;                               // channels that the gyro and accelerometer votes blamed
    std::array<uint32_t, kSensorChannels> stuck_hash{};  // a hash of the channel's raw payload bytes this frame, for the stuck detector
    // Pointers to the manager's own arrays (the first six values of each channel's eight, the voted outputs, the tolerances): nothing is copied.
    std::array<const std::array<float, kVoteChannels>*, kSensorChannels> x{};
    const std::array<float, kVoteChannels>* output = nullptr;
    const std::array<float, kVoteChannels>* tol = nullptr;
    bool output_trusted = false;                      // none of the six was held and Safe is not requested: the voted output can be used as a reference
    uint8_t healthy_valid = 0U;                       // the channels that took part in the vote (healthy and good)
  };

  struct Report {
    std::array<uint8_t, kSensorChannels> reason{};
    uint8_t newly_latched = 0U;
    uint8_t newly_disabled = 0U;
    uint8_t probation_started = 0U;
    uint8_t probation_failed = 0U;
    uint8_t newly_reintegrated = 0U;
    uint8_t latched_mask = 0U;     // out of the consensus for any reason
    uint8_t probation_mask = 0U;
    uint8_t disabled_mask = 0U;
    unsigned healthy = 0U;         // channels that have been seen and are Healthy
    std::array<uint8_t, kSensorChannels> strikes{};
  };

  struct Counters {
    uint32_t latches = 0U;
    uint32_t reintegrations = 0U;
    uint32_t probation_failures = 0U;
    uint32_t disabled = 0U;
  };

  [[nodiscard]] NodeState state(unsigned k) const noexcept { return k < kSensorChannels ? state_of(k) : NodeState::Disabled; }
  [[nodiscard]] bool healthy(unsigned k) const noexcept { return k < kSensorChannels && state_of(k) == NodeState::Healthy; }
  [[nodiscard]] bool seen(unsigned k) const noexcept { return k < kSensorChannels && seen_[k]; }
  [[nodiscard]] unsigned count_healthy() const noexcept { return count_in_state(NodeState::Healthy); }
  [[nodiscard]] unsigned strikes(unsigned k) const noexcept { return k < kSensorChannels ? strikes_[k] : 0U; }
  [[nodiscard]] const Counters& counters() const noexcept { return counters_; }

  // Called first in a frame by the manager: a state that failed its integrity check is taken as Latched (excluded), and the fact is returned.
  bool scrub() noexcept {
    bool repaired = false;
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      if (!state_valid(k)) {
        set_state(k, NodeState::Latched);
        dwell_[k] = 0U;
        dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << k));
        req_[k] = false;
        clean_[k] = 0U;
        repaired = true;
      }
    }
    return repaired;
  }

  // A good frame makes the channel "seen" (before that a channel that never delivered is not judged: peers boot in any order). Returns whether the channel was seen before.
  bool note_good(unsigned k) noexcept {
    const bool was = seen_[k];
    seen_[k] = true;
    return was;
  }

  // ---- operator commands (routed by the manager) ----
  CommandResult reintegrate(unsigned k) noexcept {
    if (k >= kSensorChannels) {
      return CommandResult::RefusedBadNode;
    }
    const NodeState st = state_of(k);
    if (st == NodeState::Disabled) {
      return CommandResult::RefusedDisabled;
    }
    if (st == NodeState::Healthy) {
      return CommandResult::RefusedNotLatched;
    }
    if (st == NodeState::Probation || req_[k]) {
      return CommandResult::AlreadyDone;
    }
    req_[k] = true;
    return CommandResult::Accepted;
  }

  CommandResult disable(unsigned k) noexcept {
    if (k >= kSensorChannels) {
      return CommandResult::RefusedBadNode;
    }
    if (state_of(k) == NodeState::Disabled) {
      return CommandResult::AlreadyDone;
    }
    set_state(k, NodeState::Disabled);
    req_[k] = false;
    ++counters_.disabled;
    disabled_by_command_ = static_cast<uint8_t>(disabled_by_command_ | (1U << k));
    return CommandResult::Accepted;
  }

  CommandResult clear_disabled(unsigned k) noexcept {
    if (k >= kSensorChannels) {
      return CommandResult::RefusedBadNode;
    }
    if (state_of(k) != NodeState::Disabled) {
      return CommandResult::RefusedNotDisabled;
    }
    set_state(k, NodeState::Latched);
    strikes_[k] = 0U;
    dwell_[k] = 0U;
    dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << k));
    attempts_[k] = 0U;
    req_[k] = false;
    return CommandResult::Accepted;
  }

  // One frame: judge each channel, advance the probations, count the dwell. Fills `out` (the caller zeroes it first).
  void update(const Input& in, const RedundancyConfig& cfg, std::array<uint8_t, kSensorChannels>& arrival_reason, Report& out) noexcept {
    std::array<bool, kSensorChannels> stuck_now{};
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      uint8_t r = arrival_reason[k];
      if (in.good[k]) {
        stuck_now[k] = stuck_[k].update(static_cast<int32_t>(in.stuck_hash[k]));
      }
      if (stuck_now[k]) {
        r = static_cast<uint8_t>(r | reason::kStuck);
      }
      if (((in.blame >> k) & 1U) != 0U) {
        r = static_cast<uint8_t>(r | reason::kVote);
      }
      out.reason[k] = r;
      if (state_of(k) == NodeState::Healthy) {
        decide_latch(k, cfg, out);
      }
    }
    advance(in, cfg, stuck_now, out);
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      out.strikes[k] = strikes_[k];
      const NodeState st = state_of(k);
      if (st != NodeState::Healthy) {
        out.latched_mask = static_cast<uint8_t>(out.latched_mask | (1U << k));
      }
      if (st == NodeState::Probation) {
        out.probation_mask = static_cast<uint8_t>(out.probation_mask | (1U << k));
      }
      if (st == NodeState::Disabled) {
        out.disabled_mask = static_cast<uint8_t>(out.disabled_mask | (1U << k));
      }
      if (st == NodeState::Healthy && seen_[k]) {
        ++out.healthy;
      }
    }
    out.newly_disabled = static_cast<uint8_t>(out.newly_disabled | disabled_by_command_);
    disabled_by_command_ = 0U;
  }

 private:
  [[nodiscard]] bool state_valid(unsigned k) const noexcept { return st_[k].intact() && st_[k].get() <= static_cast<uint8_t>(NodeState::Disabled); }
  [[nodiscard]] NodeState state_of(unsigned k) const noexcept { return state_valid(k) ? static_cast<NodeState>(st_[k].get()) : NodeState::Latched; }
  void set_state(unsigned k, NodeState s) noexcept { st_[k].set(static_cast<uint8_t>(s)); }
  [[nodiscard]] unsigned count_in_state(NodeState s) const noexcept {
    unsigned c = 0U;
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      c += state_of(k) == s ? 1U : 0U;
    }
    return c;
  }

  void decide_latch(unsigned k, const RedundancyConfig& cfg, Report& out) noexcept {
    const bool bad = out.reason[k] != 0U;
    const bool by_window = mon_[k].update(bad);
    const bool by_alpha = alpha_[k].update(bad);
    if (!by_window && !by_alpha) {
      return;
    }
    if (!by_window) {
      out.reason[k] = static_cast<uint8_t>(out.reason[k] | reason::kIntermittent);
    }
    on_latch(k, cfg, out);
  }

  void on_latch(unsigned k, const RedundancyConfig& cfg, Report& out) noexcept {
    out.newly_latched = static_cast<uint8_t>(out.newly_latched | (1U << k));
    ++counters_.latches;
    if (strikes_[k] < 0xFFU) {
      ++strikes_[k];
    }
    cause_[k] = out.reason[k];
    const unsigned limit = (out.reason[k] & cfg.physical_causes) != 0U ? cfg.max_strikes_physical : cfg.max_strikes;
    dwell_[k] = 0U;
    dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << k));
    req_[k] = false;
    attempts_[k] = 0U;
    if (limit != 0U && strikes_[k] >= limit) {
      set_state(k, NodeState::Disabled);
      out.newly_disabled = static_cast<uint8_t>(out.newly_disabled | (1U << k));
      ++counters_.disabled;
    } else {
      set_state(k, NodeState::Latched);
    }
  }

  [[nodiscard]] uint16_t dwell_needed(unsigned k, const RedundancyConfig& cfg) const noexcept {
    const bool transient = strikes_[k] == 1U && (cause_[k] & ~cfg.auto_eligible_causes) == 0U;
    return transient ? cfg.min_dwell_frames_transient : cfg.min_dwell_frames;
  }

  [[nodiscard]] bool wants_probation(unsigned k, const RedundancyConfig& cfg) const noexcept {
    if (req_[k]) {
      return true;
    }
    return cfg.policy == ReintegrationPolicy::AutoTransient && strikes_[k] == 1U && (cause_[k] & ~cfg.auto_eligible_causes) == 0U &&
           attempts_[k] < cfg.auto_max_attempts;
  }

  // The vote among the channels on probation, used only when no channel is Healthy (ADR-014 for sensing).
  struct Cohort {
    bool usable = false;
    uint8_t members = 0U;
    std::array<VoteResult, kSensorValues> votes{};
  };

  [[nodiscard]] Cohort cohort_reference(const Input& in, const std::array<bool, kSensorChannels>& stuck_now) const noexcept {
    Cohort c;
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      if (state_of(k) == NodeState::Probation && in.good[k] && !stuck_now[k]) {
        c.members = static_cast<uint8_t>(c.members | (1U << k));
      }
    }
    if (count_in_state(NodeState::Healthy) != 0U || count_channels(c.members) < 2U) {
      return c;
    }
    c.usable = true;
    for (unsigned v = 0; v < kSensorValues; ++v) {
      const std::array<float, kChannels> x = {(*in.x[0])[v], (*in.x[1])[v], (*in.x[2])[v]};
      c.votes[v] = vote3(x, c.members, (*in.tol)[v]);
    }
    return c;
  }

  [[nodiscard]] Verdict cohort_verdict(unsigned k, const Cohort& c) const noexcept {
    bool neutral = false;
    for (unsigned v = 0; v < kSensorValues; ++v) {
      if (c.votes[v].status == VoteStatus::Triplex) {
        if (((c.votes[v].disagree_mask >> k) & 1U) != 0U) {
          return Verdict::Dirty;
        }
      } else if (c.votes[v].status != VoteStatus::Duplex) {
        neutral = true;
      }
    }
    return neutral ? Verdict::Neutral : Verdict::Clean;
  }

  [[nodiscard]] Verdict shadow_verdict(unsigned k, const Input& in) const noexcept {
    for (unsigned v = 0; v < kSensorValues; ++v) {
      if (std::fabs((*in.x[k])[v] - (*in.output)[v]) > (*in.tol)[v]) {
        return Verdict::Dirty;
      }
    }
    return Verdict::Clean;
  }

  void judge_probation(unsigned k, const Input& in, const RedundancyConfig& cfg, bool stuck, const Cohort& cohort, Report& out) noexcept {
    Verdict v = Verdict::Clean;
    if (!in.good[k] || stuck) {
      v = Verdict::Dirty;
    } else if (cohort.usable) {
      v = cohort_verdict(k, cohort);
    } else if (in.healthy_valid == 0U || !in.output_trusted) {
      v = Verdict::Neutral;
    } else {
      v = shadow_verdict(k, in);
    }
    if (v == Verdict::Dirty) {
      set_state(k, NodeState::Latched);
      dwell_[k] = 0U;
      dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << k));
      req_[k] = false;
      if (attempts_[k] < 0xFFU) {
        ++attempts_[k];
      }
      out.probation_failed = static_cast<uint8_t>(out.probation_failed | (1U << k));
      ++counters_.probation_failures;
    } else if (v == Verdict::Clean) {
      const unsigned needed = strikes_[k] >= 2U ? cfg.probation_frames_repeat : cfg.probation_frames;
      if (++clean_[k] >= needed) {
        set_state(k, NodeState::Healthy);
        mon_[k].force_unlatch();
        alpha_[k].reset();
        stuck_[k] = StuckDetector(cfg.stuck_limit);
        out.newly_reintegrated = static_cast<uint8_t>(out.newly_reintegrated | (1U << k));
        ++counters_.reintegrations;
      }
    }
  }

  void advance(const Input& in, const RedundancyConfig& cfg, const std::array<bool, kSensorChannels>& stuck_now, Report& out) noexcept {
    const Cohort cohort = cohort_reference(in, stuck_now);
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      if (state_of(k) == NodeState::Probation) {
        judge_probation(k, in, cfg, stuck_now[k], cohort, out);
      }
    }
    bool one_on_probation = count_in_state(NodeState::Probation) != 0U;
    const bool no_healthy = count_in_state(NodeState::Healthy) == 0U;
    for (unsigned k = 0; k < kSensorChannels; ++k) {
      if (state_of(k) != NodeState::Latched) {
        continue;
      }
      if (((dwell_hold_ >> k) & 1U) == 0U && dwell_[k] < 0xFFFFU) {
        ++dwell_[k];
      }
      if ((!one_on_probation || no_healthy) && dwell_[k] >= dwell_needed(k, cfg) && wants_probation(k, cfg)) {
        set_state(k, NodeState::Probation);
        req_[k] = false;
        clean_[k] = 0U;
        one_on_probation = true;
        out.probation_started = static_cast<uint8_t>(out.probation_started | (1U << k));
      }
    }
    dwell_hold_ = 0U;
  }

  std::array<GuardedByte, kSensorChannels> st_{};
  std::array<bool, kSensorChannels> seen_{};
  std::array<ChannelMonitor, kSensorChannels> mon_;
  std::array<AlphaCount, kSensorChannels> alpha_;
  std::array<StuckDetector, kSensorChannels> stuck_;
  std::array<uint8_t, kSensorChannels> strikes_{};
  std::array<uint16_t, kSensorChannels> dwell_{};
  std::array<bool, kSensorChannels> req_{};
  std::array<uint16_t, kSensorChannels> clean_{};
  std::array<uint8_t, kSensorChannels> attempts_{};
  std::array<uint8_t, kSensorChannels> cause_{};
  uint8_t dwell_hold_ = 0U;
  uint8_t disabled_by_command_ = 0U;
  Counters counters_{};
};

}  // namespace tfc
