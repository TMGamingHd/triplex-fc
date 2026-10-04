// SPDX-License-Identifier: MIT
// Actuator node logic (ADR-023, ADR-024): the vote on the three flight computers' commands, the latch of the output with a bound on how fast it may
// move, and the Safe sequence. ACT has no servo behind it: its output is the voted gimbal command on the bus, and its Safe action is the value of that
// command (docs/SAFE_MODE.md, docs/VEHICLE_SIM.md section 2). It does not trust the flight computers' own judgements: it votes what it receives and
// flags on its own.
//
// Modes.
//   Standby  after a power-on: the output is neutral until trustworthy votes have been seen for `standby_frames` in a row (nothing has failed;
//            this is the pad), then Nominal. A Safe request in Standby goes to Safe.
//   Nominal  the output follows the voted command, no faster than `normal_slew` degrees a frame. A frame without a trustworthy vote holds the last
//            output; `lost_votes` such frames in a row, a Safe request from the flight computers, or the SAFE line enters Safe.
//   Safe     freeze at once (no step), hold for `hold_frames` (the transient may clear and the operator may act), then ramp to neutral at
//            `ramp_deg_per_frame`, then hold neutral. Nothing leaves Safe by itself: clear_safe() accepts only when the votes have been trustworthy,
//            from at least two nodes, for `exit_frames` in a row since Safe was entered, and no Safe request or SAFE line is active (the caller passes the operator's
//            authenticated, ARMed clear-safe only; authentication belongs to the ground-command path, not here).
// After a reset that is not a power-on ACT starts in Safe, holding the output it had stored before the reset (the caller keeps `ActRecord` in memory
// that survives a reset), so the command does not step across the reset.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/crc8.hpp"
#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/resetlog.hpp"
#include "tfc/voter.hpp"

namespace tfc {

enum class ActMode : uint8_t { Standby = 0, Nominal = 1, Safe = 2 };
enum class SafePhase : uint8_t { None = 0, Hold = 1, Ramp = 2, Neutral = 3 };
enum class SafeCause : uint8_t { None = 0, LostVotes = 1, FcRequest = 2, HardwareLine = 3, Reset = 4 };

struct ActConfig {
  float tol_deg = 0.05F;             // two commands agree if they differ by no more than this
  float normal_slew = 1.0F;          // largest change of the output per frame in Standby and Nominal, degrees
  uint16_t lost_votes = 3U;          // frames without a trustworthy vote before Safe
  uint16_t hold_frames = 50U;        // Safe: hold for this long (0.5 s) before ramping
  float ramp_deg_per_frame = 0.02F;  // Safe: ramp to neutral at this rate (2 degrees per second: 4 s from the limit)
  uint16_t standby_frames = 100U;    // Standby: trustworthy votes needed before Nominal
  uint16_t exit_frames = 100U;       // Safe: trustworthy votes needed before clear_safe() is accepted
  uint8_t persist_m = 3U;            // a node is excluded after persist_m bad frames of the last persist_n
  uint8_t persist_n = 5U;
  bool resume_from_stored = true;    // after a reset: false starts the Safe hold from neutral instead of the stored output
};

struct ActOutput {
  float pitch_deg = 0.0F;
  float yaw_deg = 0.0F;
  ActMode mode = ActMode::Standby;
  SafePhase phase = SafePhase::None;
  SafeCause cause = SafeCause::None;
  bool held = false;           // no trustworthy vote this frame: the last output is repeated
  uint8_t voted_nodes = 0U;    // bit n: node n's command took part in this frame's vote
  uint8_t excluded_nodes = 0U; // bit n: ACT has excluded node n
  uint8_t vote_status = 0U;    // tfc::VoteStatus of the pitch vote
};

// ACT's output as the frame it broadcasts (protocol.hpp, ActFrame): the voted command and the status field that tells everyone what ACT is doing.
[[nodiscard]] constexpr ActFrame to_act_frame(const ActOutput& o) noexcept {
  ActFrame a;
  a.pitch_deg = o.pitch_deg;
  a.yaw_deg = o.yaw_deg;
  a.state = static_cast<uint8_t>(o.mode);
  a.held = o.held;
  a.vote_status = o.vote_status;
  a.voted_nodes = o.voted_nodes;
  a.excluded_nodes = o.excluded_nodes;
  a.cause = static_cast<uint8_t>(o.cause);
  return a;
}

// What ACT keeps across a reset (in no-init RAM): the last output. Plain data with no initialisers, like ResetRecord.
constexpr uint32_t kActMagic = 0x54464143U;  // "TFAC"

struct ActRecord {
  uint32_t magic;
  int16_t pitch;  // 0.001 degree
  int16_t yaw;
  uint8_t mode;
  uint8_t crc;
};

namespace detail {
inline uint8_t act_record_crc(const ActRecord& r) noexcept {
  const std::array<uint8_t, 9> b{static_cast<uint8_t>(r.magic & 0xFFU),
                                 static_cast<uint8_t>((r.magic >> 8U) & 0xFFU),
                                 static_cast<uint8_t>((r.magic >> 16U) & 0xFFU),
                                 static_cast<uint8_t>((r.magic >> 24U) & 0xFFU),
                                 static_cast<uint8_t>(static_cast<uint16_t>(r.pitch) & 0xFFU),
                                 static_cast<uint8_t>((static_cast<uint16_t>(r.pitch) >> 8U) & 0xFFU),
                                 static_cast<uint8_t>(static_cast<uint16_t>(r.yaw) & 0xFFU),
                                 static_cast<uint8_t>((static_cast<uint16_t>(r.yaw) >> 8U) & 0xFFU),
                                 r.mode};
  return crc8(b.data(), b.size());
}
}  // namespace detail

[[nodiscard]] inline bool act_record_valid(const ActRecord& r) noexcept {
  return r.magic == kActMagic && r.crc == detail::act_record_crc(r) && r.mode <= static_cast<uint8_t>(ActMode::Safe);
}

class ActLogic {
 public:
  ActLogic() noexcept : ActLogic(ActConfig{}) {}
  explicit ActLogic(const ActConfig& cfg) noexcept
      : cfg_(cfg),
        mon_{ChannelMonitor(cfg.persist_m, cfg.persist_n, 100U, 3U), ChannelMonitor(cfg.persist_m, cfg.persist_n, 100U, 3U),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, 100U, 3U)} {}

  // Call once at boot with the reset cause and the record found in no-init RAM.
  void boot(ResetCause cause, const ActRecord& stored) noexcept {
    out_ = ActOutput{};
    if (cause == ResetCause::PowerOn || !act_record_valid(stored)) {
      out_.mode = ActMode::Standby;
      return;
    }
    if (cfg_.resume_from_stored) {
      out_.pitch_deg = static_cast<float>(stored.pitch) * kCmdLsbDeg;
      out_.yaw_deg = static_cast<float>(stored.yaw) * kCmdLsbDeg;
    }
    enter_safe(SafeCause::Reset);
  }

  void begin_frame() noexcept {
    have_ = 0U;
    pitch_in_ = {};
    yaw_in_ = {};
  }

  // Offer a frame; true if it was a good command frame of node 0..2 and was stored.
  bool on_frame(const Frame& f) noexcept {
    const uint32_t base = f.id & ~0x3U;
    const unsigned node = f.id & 0x3U;
    if (base != id::kCmdBase || node >= kChannels) {
      return false;
    }
    const DecodedCommand d = unpack_cmd(f);
    if (!d.ok) {
      return false;
    }
    pitch_in_[node] = d.cmd.pitch_deg;
    yaw_in_[node] = d.cmd.yaw_deg;
    have_ = static_cast<uint8_t>(have_ | (1U << node));
    return true;
  }

  // The flight computers' sticky Safe request (from their heartbeat) and the hardware SAFE line from the supervisor or the operator.
  void safe_request(bool on) noexcept { fc_request_ = on; }
  void hardware_safe(bool on) noexcept { hw_safe_ = on; }

  // The operator's authenticated, ARMed clear-safe. True if accepted. Refused unless the votes have been good for `exit_frames`, from at least two
  // nodes, and no request is active.
  bool clear_safe() noexcept {
    if (out_.mode != ActMode::Safe) {
      return true;  // nothing to clear
    }
    if (good_run_ < cfg_.exit_frames || good_nodes_ < 2U || fc_request_ || hw_safe_) {
      ++refused_clears_;
      return false;
    }
    out_.mode = ActMode::Nominal;
    out_.phase = SafePhase::None;
    out_.cause = SafeCause::None;
    lost_ = 0U;
    ++clears_;
    return true;
  }

  // The operator readmits excluded nodes (they must then prove themselves by agreeing).
  void clear_exclusions() noexcept {
    for (ChannelMonitor& m : mon_) {
      m.force_unlatch();
    }
  }

  // The end of the frame: vote, judge the nodes, advance the mode, and return the output for this frame.
  const ActOutput& end_frame() noexcept {
    float vote_pitch = 0.0F;
    float vote_yaw = 0.0F;
    const bool trusted = vote(vote_pitch, vote_yaw);
    out_.held = !trusted;
    switch (out_.mode) {
      case ActMode::Standby: standby(trusted); break;
      case ActMode::Nominal: nominal(trusted, vote_pitch, vote_yaw); break;
      case ActMode::Safe:
      default:  // a corrupted mode value is treated as Safe: the safe side
        safe();
        break;
    }
    return out_;
  }

  // The output to keep in memory that survives a reset (call every frame, or at least every few).
  [[nodiscard]] ActRecord record() const noexcept {
    ActRecord r{};
    r.magic = kActMagic;
    r.pitch = quantize(out_.pitch_deg, kCmdLsbDeg);
    r.yaw = quantize(out_.yaw_deg, kCmdLsbDeg);
    r.mode = static_cast<uint8_t>(out_.mode);
    r.crc = detail::act_record_crc(r);
    return r;
  }

  [[nodiscard]] const ActOutput& output() const noexcept { return out_; }
  [[nodiscard]] uint32_t safe_entries() const noexcept { return safe_entries_; }
  [[nodiscard]] uint32_t clears() const noexcept { return clears_; }
  [[nodiscard]] uint32_t refused_clears() const noexcept { return refused_clears_; }
  [[nodiscard]] uint32_t held_frames() const noexcept { return held_frames_; }
  [[nodiscard]] uint32_t good_run() const noexcept { return good_run_; }

 private:
  static constexpr float clamp(float v, float lim) noexcept { return v > lim ? lim : (v < -lim ? -lim : v); }

  // Move `cur` toward `target` by at most `step`.
  static constexpr float approach(float cur, float target, float step) noexcept { return cur + clamp(target - cur, step); }

  // Vote the commands of the nodes that are present and not excluded. True if the result is a trustworthy command (agreeing, or alone).
  bool vote(float& pitch, float& yaw) noexcept {
    uint8_t mask = 0U;
    for (unsigned n = 0; n < kChannels; ++n) {
      if (((have_ >> n) & 1U) != 0U && !mon_[n].latched()) {
        mask = static_cast<uint8_t>(mask | (1U << n));
      }
    }
    const VoteResult vp = vote3(pitch_in_, mask, cfg_.tol_deg);
    const VoteResult vy = vote3(yaw_in_, mask, cfg_.tol_deg);
    out_.voted_nodes = mask;
    out_.vote_status = static_cast<uint8_t>(vp.status);
    const bool ok = ok_status(vp.status) && ok_status(vy.status);
    if (ok) {
      pitch = vp.value;
      yaw = vy.value;
    }
    judge_nodes(vp, vy, vp.status == VoteStatus::Triplex && vy.status == VoteStatus::Triplex);
    good_nodes_ = static_cast<uint8_t>(count_channels(mask));
    good_run_ = ok ? good_run_ + 1U : 0U;
    return ok;
  }

  static constexpr bool ok_status(VoteStatus s) noexcept {
    return s == VoteStatus::Triplex || s == VoteStatus::Duplex || s == VoteStatus::Simplex;
  }

  // Missing frames count against a node; so does a command that is the odd one out of a Triplex vote. Two nodes that cannot be told apart blame nobody.
  void judge_nodes(const VoteResult& vp, const VoteResult& vy, bool triplex) noexcept {
    for (unsigned n = 0; n < kChannels; ++n) {
      if (mon_[n].latched()) {
        continue;
      }
      const bool missing = ((have_ >> n) & 1U) == 0U;
      const bool outlier = triplex && ((((vp.disagree_mask | vy.disagree_mask) >> n) & 1U) != 0U);
      (void)mon_[n].update(missing || outlier);
    }
    uint8_t ex = 0U;
    for (unsigned n = 0; n < kChannels; ++n) {
      ex = static_cast<uint8_t>(ex | (mon_[n].latched() ? (1U << n) : 0U));
    }
    out_.excluded_nodes = ex;
  }

  void standby(bool trusted) noexcept {
    if (fc_request_ || hw_safe_) {
      enter_safe(hw_safe_ ? SafeCause::HardwareLine : SafeCause::FcRequest);
      return;
    }
    out_.pitch_deg = approach(out_.pitch_deg, 0.0F, cfg_.normal_slew);  // neutral until Nominal
    out_.yaw_deg = approach(out_.yaw_deg, 0.0F, cfg_.normal_slew);
    if (trusted && good_run_ >= cfg_.standby_frames) {
      out_.mode = ActMode::Nominal;
      lost_ = 0U;
    }
  }

  void nominal(bool trusted, float pitch, float yaw) noexcept {
    if (fc_request_ || hw_safe_) {
      enter_safe(hw_safe_ ? SafeCause::HardwareLine : SafeCause::FcRequest);
      return;
    }
    if (trusted) {
      lost_ = 0U;
      out_.pitch_deg = approach(out_.pitch_deg, pitch, cfg_.normal_slew);
      out_.yaw_deg = approach(out_.yaw_deg, yaw, cfg_.normal_slew);
      return;
    }
    ++held_frames_;
    ++lost_;
    if (lost_ >= cfg_.lost_votes) {
      enter_safe(SafeCause::LostVotes);
    }
  }

  void enter_safe(SafeCause cause) noexcept {
    out_.mode = ActMode::Safe;
    out_.phase = SafePhase::Hold;
    out_.cause = cause;
    hold_count_ = 0U;
    good_run_ = 0U;  // the exit conditions are counted from here: at least `exit_frames` of good votes *in* Safe
    ++safe_entries_;
  }

  // Freeze, hold, ramp to neutral, hold neutral.
  void safe() noexcept {
    switch (out_.phase) {
      case SafePhase::Hold:
        ++hold_count_;
        if (hold_count_ >= cfg_.hold_frames) {
          out_.phase = SafePhase::Ramp;
        }
        break;
      case SafePhase::Ramp:
        out_.pitch_deg = approach(out_.pitch_deg, 0.0F, cfg_.ramp_deg_per_frame);
        out_.yaw_deg = approach(out_.yaw_deg, 0.0F, cfg_.ramp_deg_per_frame);
        if (std::fabs(out_.pitch_deg) <= 1e-6F && std::fabs(out_.yaw_deg) <= 1e-6F) {
          out_.pitch_deg = 0.0F;
          out_.yaw_deg = 0.0F;
          out_.phase = SafePhase::Neutral;
        }
        break;
      case SafePhase::None:
      case SafePhase::Neutral:
      default:
        break;  // Neutral (None is never the phase of Safe): hold
    }
  }

  ActConfig cfg_{};
  ActOutput out_{};
  std::array<float, kChannels> pitch_in_{};
  std::array<float, kChannels> yaw_in_{};
  uint8_t have_ = 0U;
  std::array<ChannelMonitor, kChannels> mon_;
  bool fc_request_ = false;
  bool hw_safe_ = false;
  uint16_t lost_ = 0U;
  uint16_t hold_count_ = 0U;
  uint32_t good_run_ = 0U;
  uint8_t good_nodes_ = 0U;
  uint32_t safe_entries_ = 0U;
  uint32_t clears_ = 0U;
  uint32_t refused_clears_ = 0U;
  uint32_t held_frames_ = 0U;
};

// What ACT concludes from the flight computers' heartbeats: do they ask for Safe? A heartbeat is fresh for `fresh_frames` frames. Of the fresh heartbeats of the
// nodes ACT has not excluded, a majority must ask (so a single faulty computer cannot safe the vehicle by itself); if only one is fresh, that one decides (a lone
// survivor's request is all there is). Two computers that disagree and cannot blame each other both ask, which is a majority of two.
class HeartbeatMonitor {
 public:
  explicit HeartbeatMonitor(uint8_t fresh_frames = 3U) noexcept : fresh_(fresh_frames) {}

  // Call once per frame before the heartbeats of that frame are offered: ages every record.
  void begin_frame() noexcept {
    for (uint8_t& a : age_) {
      a = a < 255U ? static_cast<uint8_t>(a + 1U) : a;
    }
  }

  // Offer a frame; true if it was a good heartbeat of node 0..2.
  bool on_frame(const Frame& f) noexcept {
    const DecodedHeartbeat d = unpack_heartbeat(f);
    if (!d.ok) {
      return false;
    }
    const unsigned node = f.id - id::kHeartbeat;
    asks_[node] = d.hb.safe_requested;
    age_[node] = 0U;
    seen_ = static_cast<uint8_t>(seen_ | (1U << node));
    return true;
  }

  // `excluded`: bit n set if ACT has excluded node n.
  [[nodiscard]] bool safe_requested(uint8_t excluded) const noexcept {
    unsigned fresh = 0U;
    unsigned asking = 0U;
    for (unsigned n = 0; n < kChannels; ++n) {
      const bool usable = ((seen_ >> n) & 1U) != 0U && age_[n] < fresh_ && ((excluded >> n) & 1U) == 0U;
      if (usable) {
        ++fresh;
        asking += asks_[n] ? 1U : 0U;
      }
    }
    return fresh > 0U && (asking * 2U > fresh || (fresh == 1U && asking == 1U));
  }

 private:
  uint8_t fresh_;
  std::array<uint8_t, kChannels> age_{};
  std::array<bool, kChannels> asks_{};
  uint8_t seen_ = 0U;
};

}  // namespace tfc
