// SPDX-License-Identifier: MIT
// Frame-number bookkeeping for SYNC following and sync-master takeover (ADR-018's firmware requirements, ARCHITECTURE section 3).
// The firmware does the timing (wait for SYNC, lock the frame timer to its arrival); this class decides what the node does in the frame that
// starts now:
//   - a node that hears SYNC takes the frame number from it and follows (a master that hears someone else's SYNC yields: two masters cannot both be right);
//   - a node that misses SYNC keeps counting, so the number stays continuous through the gap;
//   - a follower that has missed SYNC for `takeover_frames` in a row, and has locked to a SYNC at least once, becomes the master and sends SYNC itself;
//   - a node that may claim an empty bus (node A) and restarts after a reset listens for `boot_listen_frames` before it does, so that a returning
//     node A follows the master that took over instead of colliding with it; B and C never claim a bus on which they have not heard a SYNC.
// Who takes over first when several nodes would is settled by time, not by this class: the wait for SYNC lasts `sync_window_us(node)`, longer for a higher
// node, so the lowest healthy node's SYNC reaches the others before their own windows end and they follow it.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

#include "tfc/protocol.hpp"

namespace tfc {

struct SyncPolicy {
  uint8_t takeover_frames = 2U;     // SYNC missing for this many frames in a row: the node takes over
  uint8_t boot_listen_frames = 5U;  // after a restart a node listens this long before it may become the master
};

// How long node `node` waits for SYNC at the start of a frame before it gives up on it, microseconds after the nominal start. The stagger between
// nodes must be longer than the time a SYNC takes to be heard by another node: about 0.1 ms on the wire, plus the receive latency (interrupt-driven on the
// board; up to 1 ms on the host, whose CAN driver polls its socket, so the host builds use a longer stagger).
constexpr int64_t sync_window_us(unsigned node, int64_t stagger_us = 500) noexcept { return 1000 + (stagger_us * static_cast<int64_t>(node)); }

enum class SyncStart : uint8_t {
  Master = 0,      // node A powering up: the master from the first frame
  Listen = 1,      // node A after a reset: follow a master if there is one, claim the bus if there is none after `boot_listen_frames`
  FollowOnly = 2,  // nodes B and C: follow; take over only after having locked to a SYNC and then lost it
  Observer = 3     // the actuator node: follows and counts like a follower but is never the master
};

struct SyncTick {
  uint32_t frame = 0U;      // the number of the frame that starts now
  bool master = false;      // this node sends SYNC for it
  bool locked = false;      // it has heard SYNC in this frame (not just counted on)
  bool synced = false;      // the frame number means something: this node is the master or has heard a SYNC at some time. A node that is not
                            // synced has no frame number to stamp its frames with and stays silent
  bool took_over = false;   // this frame the node became the master after missing SYNC
  bool yielded = false;     // this frame a master heard another's SYNC and became a follower
  uint8_t missed = 0U;      // SYNC frames missed in a row (0 when SYNC was heard)
  uint16_t mission = 0U;    // the mission frame of the frame that starts now (protocol.hpp, `mission::`); the master sends it in SYNC
  bool mission_disagrees = false;  // in flight, SYNC's mission frame differs from this node's own count: kept, and reported
};

class SyncClock {
 public:
  explicit SyncClock(SyncStart start, SyncPolicy policy = SyncPolicy{}) noexcept
      : policy_(policy), master_(start == SyncStart::Master), may_claim_(start == SyncStart::Master || start == SyncStart::Listen), observer_(start == SyncStart::Observer) {}

  // Call once per frame at the end of the wait for SYNC: `heard` is whether a SYNC arrived in the window, `number` its frame number and `heard_mission` its mission frame.
  // Mission time (docs/design/LAUNCH_SEQUENCE.md): on the pad and in the countdown a follower takes the mission frame from SYNC (so a scrub reaches it); once in flight it counts for itself and
  // only verifies against SYNC, so a damaged or false SYNC cannot move the schedules of a flying computer; a node that is not in flight (a late joiner, a restart) adopts what it hears.
  [[nodiscard]] SyncTick cycle(bool heard, uint32_t number, uint16_t heard_mission = mission::kNotLaunched) noexcept {
    SyncTick t;
    if (heard) {
      if (!mission::in_flight(mission_)) {
        mission_ = heard_mission;
      } else if (mission_ != heard_mission) {
        t.mission_disagrees = true;
      }
      t.yielded = master_;
      master_ = false;
      ever_locked_ = true;
      missed_ = 0U;
      next_ = number;
    } else if (!master_) {
      missed_ = missed_ < 255U ? static_cast<uint8_t>(missed_ + 1U) : missed_;
      const uint8_t needed = ever_locked_ ? policy_.takeover_frames : policy_.boot_listen_frames;
      if (!observer_ && (ever_locked_ || may_claim_) && missed_ >= needed) {
        master_ = true;
        t.took_over = true;
      }
    }
    t.frame = next_;
    t.master = master_;
    t.locked = heard;
    t.synced = master_ || ever_locked_;
    t.missed = missed_;
    t.mission = mission_;
    if (mission_ != mission::kNotLaunched && mission_ != mission::kMax) {
      ++mission_;
    }
    ++next_;
    return t;
  }

  // The launch command (the sync master only): the countdown starts with the next frame. False if this node is not the master or the sequence has already begun.
  bool launch() noexcept {
    if (!master_ || mission_ != mission::kNotLaunched) {
      return false;
    }
    mission_ = 1U;
    return true;
  }

  // A scrub (the sync master only, and only before T-zero): back to the pad.
  bool scrub() noexcept {
    if (!master_ || !mission::in_countdown(mission_)) {
      return false;
    }
    mission_ = mission::kNotLaunched;
    return true;
  }

  // The supervisor's T0 line (docs/design/LAUNCH_SEQUENCE.md section 2): call once per frame with the level of the line. Only the sync master acts. During the last `kT0Window` frames of the
  // countdown a rising edge (its second high sample in a row, after the line has been seen low; a line that stays high gives one edge, not one per frame) is T-zero: the next frame is the T-zero frame. An edge earlier than that is refused, and a line
  // that is high from the start (stuck, or a supervisor that fired early) is never an edge, so a fault on the line cannot shorten the countdown to less than its last second; the
  // countdown itself is the fallback when no edge comes.
  enum class T0 : uint8_t { None = 0, Latched, TooEarly };
  static constexpr uint16_t kT0Window = 100U;
  [[nodiscard]] T0 t0_line(bool level) noexcept {
    t0_high_run_ = level ? (t0_high_run_ < 3U ? static_cast<uint8_t>(t0_high_run_ + 1U) : t0_high_run_) : 0U;
    const bool rising = t0_high_run_ == 2U && t0_low_seen_;  // exactly the second high sample: one edge, however long the line stays high
    t0_low_seen_ = t0_low_seen_ || !level;
    if (!master_ || !mission::in_countdown(mission_) || !rising) {
      return T0::None;
    }
    if (mission::frames_to_zero(mission_) > kT0Window) {
      return T0::TooEarly;
    }
    mission_ = static_cast<uint16_t>(mission::kCountdownFrames + 1U);
    return T0::Latched;
  }

  // The mission frame the next frame will carry.
  [[nodiscard]] uint16_t mission_frame() const noexcept { return mission_; }

  [[nodiscard]] bool master() const noexcept { return master_; }
  [[nodiscard]] uint32_t next_frame() const noexcept { return next_; }

 private:
  SyncPolicy policy_;
  bool master_;
  bool may_claim_;
  bool observer_;
  bool ever_locked_ = false;
  uint8_t missed_ = 0U;
  uint32_t next_ = 0U;
  uint16_t mission_ = 0U;
  uint8_t t0_high_run_ = 0U;  // consecutive high samples of the T0 line (saturates at 3)
  bool t0_low_seen_ = false;  // the line has been low since this node started
};

}  // namespace tfc
