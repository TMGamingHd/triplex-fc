// SPDX-License-Identifier: MIT
// The actuator node's ground-command path (ADR-019 applied to ACT, docs/design/ACT_LOGIC.md): the same frame, tag, counter window and ARM/EXECUTE rules as the
// flight computers' manager, kept in a small class of its own because ACT has no manager. ACT hears every ground frame, authenticates it, tracks the
// counter of every authentic one (the operator's counter is shared by all nodes, so a command meant for B moves it too), and acts on two operations:
//   clear-safe   always needs an ARM: lifts ACT's Safe (ActLogic::clear_safe(), which refuses unless the votes have been good); the node field is ignored,
//                because the same operator command also lifts the flight computers' sticky request
//   reintegrate  plain: ACT readmits the nodes it has excluded (ActLogic::clear_exclusions()); the node field is ignored, they must prove themselves again
// Anything else is not ACT's. The class only decides; the caller applies the result.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

#include "tfc/auth.hpp"
#include "tfc/protocol.hpp"

namespace tfc {

enum class ActGroundResult : uint8_t {
  None = 0,        // not a ground frame
  Bad,             // a ground frame that did not decode
  Unauthentic,     // the tag did not verify
  Replayed,        // a stale or repeated counter
  NotForAct,       // authentic and fresh, but an operation that is not ACT's
  NeedsArm,        // clear-safe without a matching ARM first
  Armed,           // an ARM for clear-safe was accepted
  ClearSafe,       // apply ActLogic::clear_safe()
  ClearExclusions  // apply ActLogic::clear_exclusions()
};

struct ActGroundConfig {
  AuthKey key = kBenchKey;
  uint8_t command_window = 32U;       // a counter this far ahead of the last accepted one or less is fresh
  uint8_t arm_window_frames = 250U;   // an ARM stays valid this long (2.5 s)
};

class ActGround {
 public:
  ActGround() noexcept = default;
  explicit ActGround(const ActGroundConfig& cfg) noexcept : cfg_(cfg) {}

  // Offer any frame; only ground frames (id 0x510) count.
  [[nodiscard]] ActGroundResult on_frame(const Frame& f) noexcept {
    if (f.id != id::kGround) {
      return ActGroundResult::None;
    }
    const DecodedGround d = unpack_ground(f);
    if (!d.ok) {
      return ActGroundResult::Bad;
    }
    if (!ground_authentic(f, cfg_.key)) {
      return ActGroundResult::Unauthentic;
    }
    if (have_) {
      const uint8_t ahead = static_cast<uint8_t>(d.counter - last_);
      if (ahead == 0U || ahead > cfg_.command_window) {
        return ActGroundResult::Replayed;
      }
    }
    last_ = d.counter;
    have_ = true;
    const GroundOp op = static_cast<GroundOp>(d.op);
    if (op == GroundOp::Reintegrate && !d.arm) {
      return ActGroundResult::ClearExclusions;
    }
    if (op != GroundOp::ClearSafe) {
      return ActGroundResult::NotForAct;
    }
    if (d.arm) {
      arm_left_ = cfg_.arm_window_frames;
      return ActGroundResult::Armed;
    }
    if (arm_left_ == 0U) {
      return ActGroundResult::NeedsArm;
    }
    arm_left_ = 0U;  // one ARM covers one EXECUTE
    return ActGroundResult::ClearSafe;
  }

  // Once per frame: an ARM that has not been followed by its EXECUTE expires.
  void tick() noexcept {
    if (arm_left_ > 0U) {
      --arm_left_;
    }
  }

  [[nodiscard]] bool armed() const noexcept { return arm_left_ > 0U; }

 private:
  ActGroundConfig cfg_{};
  uint8_t last_ = 0U;
  bool have_ = false;
  uint8_t arm_left_ = 0U;
};

}  // namespace tfc
