// SPDX-License-Identifier: MIT
// ACT's ground-command path (core/include/tfc/act_ground.hpp): authentication, the counter window, ARM then EXECUTE, and what ACT acts on.
#include <cstdint>

#include "tfc/act_ground.hpp"
#include "tfc/auth.hpp"
#include "tfc/protocol.hpp"
#include "tfc_test.hpp"

namespace {

using tfc::ActGroundResult;
using tfc::GroundOp;

tfc::Frame cmd(GroundOp op, uint8_t node, uint8_t counter, bool arm = false) { return tfc::pack_ground_auth(op, node, counter, tfc::kBenchKey, arm); }

}  // namespace

TFC_TEST(act_ground_clear_safe_needs_an_arm_and_one_arm_covers_one_execute) {
  tfc::ActGround g;
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 1U)) == ActGroundResult::NeedsArm);
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 2U, true)) == ActGroundResult::Armed && g.armed());
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 3U)) == ActGroundResult::ClearSafe && !g.armed());
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 4U)) == ActGroundResult::NeedsArm);  // the ARM was used up
}

TFC_TEST(act_ground_an_arm_expires_after_its_window) {
  tfc::ActGroundConfig c;
  c.arm_window_frames = 3U;
  tfc::ActGround g(c);
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 1U, true)) == ActGroundResult::Armed);
  g.tick();
  g.tick();
  CHECK(g.armed());
  g.tick();
  CHECK(!g.armed());
  g.tick();  // nothing to expire: stays at zero
  CHECK(g.on_frame(cmd(GroundOp::ClearSafe, 0U, 2U)) == ActGroundResult::NeedsArm);
}

TFC_TEST(act_ground_reintegrate_is_plain_and_readmits_whatever_the_node_field_says) {
  tfc::ActGround g;
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 1U, 1U)) == ActGroundResult::ClearExclusions);
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 2U, 2U, true)) == ActGroundResult::NotForAct);  // an ARM for it is not ACT's
}

TFC_TEST(act_ground_operations_that_are_not_acts_are_ignored_but_move_the_counter) {
  tfc::ActGround g;
  CHECK(g.on_frame(cmd(GroundOp::Disable, 1U, 10U)) == ActGroundResult::NotForAct);
  CHECK(g.on_frame(cmd(GroundOp::ClearDisabled, 1U, 11U, true)) == ActGroundResult::NotForAct);
  CHECK(g.on_frame(cmd(GroundOp::Disable, 1U, 10U)) == ActGroundResult::Replayed);  // the counter moved with the earlier ones
}

TFC_TEST(act_ground_drops_forged_replayed_and_damaged_frames) {
  tfc::ActGround g;
  CHECK(g.on_frame(tfc::pack_ground(GroundOp::ClearSafe, 0U, 1U)) == ActGroundResult::Unauthentic);  // no tag
  tfc::AuthKey other = tfc::kBenchKey;
  other[0] = static_cast<uint8_t>(other[0] ^ 0x55U);
  CHECK(g.on_frame(tfc::pack_ground_auth(GroundOp::ClearSafe, 0U, 1U, other)) == ActGroundResult::Unauthentic);  // another key
  tfc::Frame damaged = cmd(GroundOp::ClearSafe, 0U, 1U);
  damaged.data[7] = static_cast<uint8_t>(damaged.data[7] ^ 1U);
  CHECK(g.on_frame(damaged) == ActGroundResult::Bad);
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 5U)) == ActGroundResult::ClearExclusions);
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 5U)) == ActGroundResult::Replayed);   // a repeat
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 4U)) == ActGroundResult::Replayed);   // older
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 5U + 33U)) == ActGroundResult::Replayed);  // too far ahead
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 5U + 32U)) == ActGroundResult::ClearExclusions);  // the edge of the window
}

TFC_TEST(act_ground_the_counter_wraps_and_other_frames_are_not_ground) {
  tfc::ActGround g;
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 250U)) == ActGroundResult::ClearExclusions);
  CHECK(g.on_frame(cmd(GroundOp::Reintegrate, 0U, 4U)) == ActGroundResult::ClearExclusions);  // 250 -> 4 is ten ahead, modulo 256
  CHECK(g.on_frame(tfc::pack_sync(1U, 1U)) == ActGroundResult::None);
  CHECK(g.on_frame(tfc::pack_cmd(0U, tfc::Command{}, 1U)) == ActGroundResult::None);
}
