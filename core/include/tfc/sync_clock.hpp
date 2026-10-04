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
  FollowOnly = 2   // nodes B and C: follow; take over only after having locked to a SYNC and then lost it
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
};

class SyncClock {
 public:
  explicit SyncClock(SyncStart start, SyncPolicy policy = SyncPolicy{}) noexcept
      : policy_(policy), master_(start == SyncStart::Master), may_claim_(start != SyncStart::FollowOnly) {}

  // Call once per frame at the end of the wait for SYNC: `heard` is whether a SYNC arrived in the window and `number` its frame number.
  [[nodiscard]] SyncTick cycle(bool heard, uint32_t number) noexcept {
    SyncTick t;
    if (heard) {
      t.yielded = master_;
      master_ = false;
      ever_locked_ = true;
      missed_ = 0U;
      next_ = number;
    } else if (!master_) {
      missed_ = missed_ < 255U ? static_cast<uint8_t>(missed_ + 1U) : missed_;
      const uint8_t needed = ever_locked_ ? policy_.takeover_frames : policy_.boot_listen_frames;
      if ((ever_locked_ || may_claim_) && missed_ >= needed) {
        master_ = true;
        t.took_over = true;
      }
    }
    t.frame = next_;
    t.master = master_;
    t.locked = heard;
    t.synced = master_ || ever_locked_;
    t.missed = missed_;
    ++next_;
    return t;
  }

  [[nodiscard]] bool master() const noexcept { return master_; }
  [[nodiscard]] uint32_t next_frame() const noexcept { return next_; }

 private:
  SyncPolicy policy_;
  bool master_;
  bool may_claim_;
  bool ever_locked_ = false;
  uint8_t missed_ = 0U;
  uint32_t next_ = 0U;
};

}  // namespace tfc
