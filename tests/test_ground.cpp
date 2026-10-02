// SPDX-License-Identifier: MIT
// Ground commands (ADR-019): authentication, replay protection, ARM/EXECUTE, the interlock tiers; the shorter dwell after a first
// transient latch (ADR-010 amended); the arbitration reference that follows the motion only where it matters (ADR-017 amended).
// Campaign findings E10, E6/D and E19, each pinned by a test written before the change.
#include <array>
#include <cmath>
#include <cstring>

#include "ground.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

using namespace tfc;
using tfct::gcmd;

namespace {

float smooth(int k) { return 5.0F + 3.0F * std::sin(0.2F * static_cast<float>(k)); }

struct Node {
  bool present = true;
  float bias = 0.0F;
  uint16_t digest_xor = 0U;
};
using Nodes = std::array<Node, 3>;
const Nodes kNone{};

// One frame with the three nodes' traffic and the given ground frames, closed.
const FrameReport& step(RedundancyManager& m, int k, const Nodes& f, std::initializer_list<Frame> ground = {}) {
  m.begin_frame();
  const uint8_t seq = static_cast<uint8_t>(k);
  for (unsigned n = 0; n < 3; ++n) {
    if (!f[n].present) continue;
    const uint8_t id = static_cast<uint8_t>(n);
    (void)m.on_frame(pack_gyro(id, Vec3{{smooth(k) + f[n].bias, -2.0F, 1.0F}}, seq));
    (void)m.on_frame(pack_accel(id, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
    (void)m.on_frame(pack_cmd(id, Command{0.5F, -0.25F, static_cast<uint16_t>(0x1234U ^ f[n].digest_xor)}, seq));
  }
  for (const Frame& g : ground) (void)m.on_frame(g);
  return m.end_frame();
}

void run_quiet(RedundancyManager& m, int from, int to) {
  for (int k = from; k < to; ++k) (void)step(m, k, kNone);
}

}  // namespace

// ============================== authentication and replay (E10 A + B) ==============================
TFC_TEST(ground_a_frame_without_a_valid_tag_is_dropped_and_counted_without_a_trace) {
  RedundancyManager m;
  run_quiet(m, 0, 5);
  const FrameReport& r = step(m, 5, kNone, {pack_ground(GroundOp::Disable, 1, 1)});  // no tag at all
  CHECK(r.command_count == 0U && m.counters().commands_unauthentic == 1U);
  CHECK(m.state(1) == NodeState::Healthy);
  for (unsigned byte = 2; byte < 6U; ++byte) {  // any single-bit damage to the tag, CRC re-sealed to hide it
    Frame f = gcmd(GroundOp::Disable, 1, 2);
    f.data[byte] = static_cast<uint8_t>(f.data[byte] ^ 0x01U);
    f.data[7] = crc8(f.data.data(), 7);
    (void)step(m, 4 + static_cast<int>(byte), kNone, {f});  // frames 6..9, consecutive
  }
  CHECK(m.counters().commands_unauthentic == 5U && m.state(1) == NodeState::Healthy);
}

TFC_TEST(ground_a_frame_tagged_with_another_key_is_refused) {
  RedundancyManager m;
  const AuthKey other = {{9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9}};
  run_quiet(m, 0, 3);
  (void)step(m, 3, kNone, {pack_ground_auth(GroundOp::Disable, 1, 1, other)});
  CHECK(m.counters().commands_unauthentic == 1U && m.state(1) == NodeState::Healthy);
  RedundancyConfig cfg;
  cfg.ground_key = other;  // the same frame IS authentic for a manager that holds that key
  RedundancyManager m2(cfg);
  run_quiet(m2, 0, 3);
  CHECK(step(m2, 3, kNone, {pack_ground_auth(GroundOp::Disable, 1, 1, other)}).command_count == 1U);
  CHECK(m2.state(1) == NodeState::Disabled);
}

TFC_TEST(ground_a_replayed_or_stale_command_is_refused_and_a_fresh_one_is_accepted) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  const Frame first = gcmd(GroundOp::Reintegrate, 1, 5);
  CHECK(step(m, 3, kNone, {first}).command_count == 1U);                          // accepted (and refused by state: node 1 is healthy)
  CHECK(step(m, 4, kNone, {first}).command_count == 0U);                          // the very same frame again: a replay
  CHECK(step(m, 5, kNone, {gcmd(GroundOp::Reintegrate, 1, 4)}).command_count == 0U);  // older than the last accepted
  CHECK(m.counters().commands_replayed == 2U);
  CHECK(step(m, 6, kNone, {gcmd(GroundOp::Reintegrate, 1, 6)}).command_count == 1U);  // the next one is fine
  CHECK(step(m, 7, kNone, {gcmd(GroundOp::Reintegrate, 1, 16)}).command_count == 1U);  // lost commands in between are tolerated
  CHECK(step(m, 8, kNone, {gcmd(GroundOp::Reintegrate, 1, 16 + 33)}).command_count == 0U);  // too far ahead: refused
  CHECK(step(m, 9, kNone, {gcmd(GroundOp::Reintegrate, 1, 16 + 32)}).command_count == 1U);   // exactly the window is allowed
  CHECK(m.counters().commands_replayed == 3U);
}

TFC_TEST(ground_the_counter_wraps_at_256) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  uint8_t c = 240;
  for (int i = 0; i < 40; ++i, c = static_cast<uint8_t>(c + 1U)) {
    const FrameReport& r = step(m, 3 + i, kNone, {gcmd(GroundOp::Reintegrate, 0, c)});
    CHECK(r.command_count == 1U);  // across 255 -> 0 every fresh counter is accepted
  }
  CHECK(m.counters().commands_replayed == 0U);
}

TFC_TEST(ground_garbage_on_the_command_id_cannot_fill_the_queue_or_move_the_counter) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  m.begin_frame();
  for (uint8_t i = 0; i < 50U; ++i) (void)m.on_frame(pack_ground(GroundOp::Disable, static_cast<uint8_t>(i % 3U), i));
  (void)m.on_frame(gcmd(GroundOp::Reintegrate, 1, 1));  // a genuine command in the middle of the flood still gets in
  const FrameReport& r = m.end_frame();
  CHECK(r.command_count == 1U && m.counters().commands_unauthentic == 50U && m.counters().commands_bad == 0U);
}

TFC_TEST(ground_authentication_can_be_switched_off_for_legacy_benches) {
  RedundancyConfig cfg;
  cfg.ground_auth = false;
  RedundancyManager m(cfg);
  run_quiet(m, 0, 3);
  CHECK(step(m, 3, kNone, {pack_ground(GroundOp::Disable, 2, 0)}).command_count == 1U);
  CHECK(step(m, 4, kNone, {pack_ground(GroundOp::Disable, 2, 0)}).command_count == 1U);  // no counter check either
  CHECK(m.counters().commands_unauthentic == 0U && m.counters().commands_replayed == 0U);
}

// ============================== ARM / EXECUTE (E10 C) ==============================
TFC_TEST(arm_clear_safe_and_clear_disabled_always_need_an_arm) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 1);
  uint8_t c = 0;
  FrameReport r = step(m, 3, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c)});
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed && m.state(1) == NodeState::Disabled);
  r = step(m, 4, kNone, {gcmd(GroundOp::ClearSafe, 0, ++c)});
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed);
  r = step(m, 5, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c, true), gcmd(GroundOp::ClearDisabled, 1, ++c)});  // ARM then EXECUTE
  CHECK(r.command_count == 2U && r.commands[0].result == CommandResult::Accepted && (r.commands[0].flags & cmdflag::kArm) != 0U);
  CHECK(r.commands[1].result == CommandResult::Accepted && (r.commands[1].flags & cmdflag::kArmed) != 0U);
  CHECK(m.state(1) == NodeState::Latched && m.counters().commands_refused == 2U);
}

TFC_TEST(arm_covers_exactly_one_execute_of_exactly_that_operation_and_node) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 1);
  (void)m.command(GroundOp::Disable, 2);
  uint8_t c = 0;
  (void)step(m, 3, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c, true)});
  FrameReport r = step(m, 4, kNone, {gcmd(GroundOp::ClearDisabled, 2, ++c)});  // armed for node 1, not node 2
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed);
  r = step(m, 5, kNone, {gcmd(GroundOp::ClearSafe, 0, ++c)});                    // armed for clear-disabled, not clear-safe
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed);
  r = step(m, 6, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c)});                // the matching one still works...
  CHECK(r.commands[0].result == CommandResult::Accepted);
  (void)m.command(GroundOp::Disable, 1);
  r = step(m, 7, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c)});                // ...once: the arm was consumed
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed);
}

TFC_TEST(arm_a_new_arm_replaces_the_old_one) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 1);
  (void)m.command(GroundOp::Disable, 2);
  uint8_t c = 0;
  (void)step(m, 3, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c, true), gcmd(GroundOp::ClearDisabled, 2, ++c, true)});
  CHECK(step(m, 4, kNone, {gcmd(GroundOp::ClearDisabled, 1, ++c)}).commands[0].result == CommandResult::RefusedNotArmed);
  CHECK(step(m, 5, kNone, {gcmd(GroundOp::ClearDisabled, 2, ++c)}).commands[0].result == CommandResult::Accepted);
}

TFC_TEST(arm_expires_after_the_arm_window_and_is_counted) {
  for (int wait : {249, 250}) {  // EXECUTE at arm + 249 is still inside the 250-frame window; at +250 it is not
    RedundancyManager m;
    run_quiet(m, 0, 3);
    (void)m.command(GroundOp::Disable, 1);
    (void)step(m, 3, kNone, {gcmd(GroundOp::ClearDisabled, 1, 1, true)});
    run_quiet(m, 4, 3 + wait);
    const FrameReport& r = step(m, 3 + wait, kNone, {gcmd(GroundOp::ClearDisabled, 1, 2)});
    CHECK((r.commands[0].result == CommandResult::Accepted) == (wait == 249));
    CHECK(m.counters().arms_expired == (wait == 249 ? 0U : 1U));
  }
}

TFC_TEST(arm_the_window_is_configurable_and_validated) {
  RedundancyConfig cfg;
  cfg.arm_window_frames = 3;
  RedundancyManager m(cfg);
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 1);
  (void)step(m, 3, kNone, {gcmd(GroundOp::ClearDisabled, 1, 1, true)});
  run_quiet(m, 4, 6);
  CHECK(step(m, 6, kNone, {gcmd(GroundOp::ClearDisabled, 1, 2)}).commands[0].result == CommandResult::RefusedNotArmed);  // 3 frames: gone
  RedundancyConfig bad;
  bad.arm_window_frames = 0;
  CHECK((validate_config(bad) & cfgerr::kGroundAuth) != 0U);
  CHECK(RedundancyManager(bad).config().arm_window_frames == RedundancyConfig{}.arm_window_frames);
}

// ============================== the interlock tiers (E6 / interlock decision) ==============================
TFC_TEST(interlock_disabling_one_of_three_is_a_plain_command) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  const FrameReport& r = step(m, 3, kNone, {gcmd(GroundOp::Disable, 2, 1)});
  CHECK(r.commands[0].result == CommandResult::Accepted && r.commands[0].flags == 0U);
  CHECK(m.state(2) == NodeState::Disabled);
}

TFC_TEST(interlock_going_from_duplex_to_simplex_needs_an_arm) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 2);  // Duplex
  uint8_t c = 0;
  FrameReport r = step(m, 3, kNone, {gcmd(GroundOp::Disable, 1, ++c)});
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed && m.state(1) == NodeState::Healthy);
  r = step(m, 4, kNone, {gcmd(GroundOp::Disable, 1, ++c, true), gcmd(GroundOp::Disable, 1, ++c)});
  CHECK(r.commands[1].result == CommandResult::Accepted && (r.commands[1].flags & cmdflag::kArmed) != 0U);
  CHECK((r.commands[1].flags & cmdflag::kCritical) == 0U && m.state(1) == NodeState::Disabled);  // Simplex, not yet the last voter
  CHECK(m.counters().critical_commands == 0U);
}

TFC_TEST(interlock_removing_the_last_voter_needs_an_arm_and_is_reported_loudly) {
  RedundancyManager m;
  run_quiet(m, 0, 3);
  (void)m.command(GroundOp::Disable, 1);
  (void)m.command(GroundOp::Disable, 2);  // Simplex: only node 0 votes
  uint8_t c = 0;
  FrameReport r = step(m, 3, kNone, {gcmd(GroundOp::Disable, 0, ++c)});
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed && m.state(0) == NodeState::Healthy);
  r = step(m, 4, kNone, {gcmd(GroundOp::Disable, 0, ++c, true), gcmd(GroundOp::Disable, 0, ++c)});
  CHECK(r.commands[1].result == CommandResult::Accepted);
  CHECK((r.commands[1].flags & cmdflag::kCritical) != 0U && (r.commands[1].flags & cmdflag::kArmed) != 0U);
  CHECK(m.counters().critical_commands == 1U && m.state(0) == NodeState::Disabled && r.mode == Mode::Safe);
}

TFC_TEST(interlock_disabling_a_node_that_is_not_voting_is_plain_even_in_simplex) {
  RedundancyManager m;
  for (int k = 0; k < 40; ++k) {
    Nodes f{};
    f[1].present = false;  // B latches
    (void)step(m, k, f);
  }
  (void)m.command(GroundOp::Disable, 2);  // Simplex on A; B is latched
  Nodes b_silent{};
  b_silent[1].present = false;
  const FrameReport& r = step(m, 40, b_silent, {gcmd(GroundOp::Disable, 1, 1)});
  CHECK(r.commands[0].result == CommandResult::Accepted && r.commands[0].flags == 0U && m.state(1) == NodeState::Disabled);
}

TFC_TEST(interlock_reintegrate_never_needs_an_arm) {
  RedundancyManager m;
  for (int k = 0; k < 30; ++k) {
    Nodes f{};
    f[1].present = !(k >= 10 && k < 15);
    (void)step(m, k, f);
  }
  CHECK(step(m, 30, kNone, {gcmd(GroundOp::Reintegrate, 1, 1)}).commands[0].result == CommandResult::Accepted);  // verified by the shadow vote instead
}

// ============================== the shorter dwell after a first transient-looking latch (E6 / D) ==============================
namespace {
int first_probation_start(const Nodes& base, unsigned node, int latch_from, int latch_to, int total, RedundancyConfig cfg = RedundancyConfig{},
                          int request_after = 15) {
  RedundancyManager m(cfg);
  int started = -1;
  for (int k = 0; k < total; ++k) {
    Nodes f = base;
    if (k >= latch_from && k < latch_to) {
      f[node].present = false;
    }
    if ((step(m, k, f).probation_started & (1U << node)) != 0U && started < 0) started = k;
    if (k == latch_from + request_after) (void)m.request_reintegration(node);
  }
  return started;
}
}  // namespace

TFC_TEST(transient_dwell_a_first_frame_problem_waits_50_frames_not_200) {
  CHECK(first_probation_start(kNone, 1, 10, 15, 400) == 12 + 50);  // latched at 12
}

TFC_TEST(transient_dwell_a_digest_mismatch_or_a_second_strike_keeps_the_full_dwell) {
  {  // a digest mismatch is state divergence: not transient-looking
    RedundancyManager m;
    int started = -1;
    for (int k = 0; k < 400; ++k) {
      Nodes f{};
      f[1].digest_xor = (k >= 10 && k < 20) ? 1U : 0U;
      if ((step(m, k, f).probation_started & 2U) != 0U && started < 0) started = k;
      if (k == 30) (void)m.request_reintegration(1);
    }
    CHECK(started == 12 + 200);
  }
  {  // the second latch of the same node waits the full dwell
    RedundancyManager m;
    int first = -1;
    int second = -1;
    for (int k = 0; k < 900; ++k) {
      Nodes f{};
      f[1].present = !((k >= 10 && k < 15) || (k >= 400 && k < 405));
      const uint8_t started = step(m, k, f).probation_started;
      if ((started & 2U) != 0U) (first < 0 ? first : second) = k;
      if (k == 20 || k == 420) (void)m.request_reintegration(1);
    }
    CHECK(first == 62 && second == 402 + 200);
  }
}

TFC_TEST(transient_dwell_is_configurable_and_never_longer_than_the_dwell) {
  RedundancyConfig cfg;
  cfg.min_dwell_frames_transient = 10;
  CHECK(first_probation_start(kNone, 1, 10, 15, 200, cfg, 5) == 12 + 10);
  RedundancyConfig bad;
  bad.min_dwell_frames_transient = 300;  // longer than the normal dwell: contradictory
  CHECK((validate_config(bad) & cfgerr::kLifeCycle) != 0U);
  CHECK(RedundancyManager(bad).config().min_dwell_frames_transient == bad.min_dwell_frames);
}

// ============================== configuration of the command path ==============================
TFC_TEST(ground_config_an_all_zero_key_with_authentication_on_is_replaced_and_reported) {
  RedundancyConfig cfg;
  cfg.ground_key = AuthKey{};
  CHECK((validate_config(cfg) & cfgerr::kGroundAuth) != 0U);
  RedundancyManager m(cfg);
  CHECK((m.config_errors() & cfgerr::kGroundAuth) != 0U && m.config().ground_key == kBenchKey);
  cfg.ground_auth = false;  // with authentication off a zero key is irrelevant
  CHECK((validate_config(cfg) & cfgerr::kGroundAuth) == 0U);
  RedundancyConfig w;
  w.command_window = 0;
  CHECK((validate_config(w) & cfgerr::kGroundAuth) != 0U);
  w.command_window = 128;
  CHECK((validate_config(w) & cfgerr::kGroundAuth) != 0U);
  w.command_window = 127;
  CHECK(validate_config(w) == 0U);
  w.command_window = 0;
  CHECK(RedundancyManager(w).config().command_window == RedundancyConfig{}.command_window);  // replaced by the default
  w.command_window = 200;
  CHECK(RedundancyManager(w).config().command_window == RedundancyConfig{}.command_window);
}

// ============================== arbitration reference (E19) ==============================
namespace {
struct DuplexResult {
  int b_latched = -1;
  bool safe = false;
};

// A and B alone (C absent); the gyro x signal moves `slope` dps per frame; B adds `bias` from frame 30. Noise-free, so the decision
// is exactly the threshold logic.
DuplexResult duplex_bias(float slope, float bias) {
  RedundancyManager m;
  DuplexResult out;
  for (int k = 0; k < 80; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (uint8_t n = 0; n < 2U; ++n) {
      const float b = (n == 1U && k >= 30) ? bias : 0.0F;
      (void)m.on_frame(pack_gyro(n, Vec3{{2.0F + slope * static_cast<float>(k) + b, -2.0F + 0.2F * std::sin(0.7F * static_cast<float>(k)), 1.0F}}, seq));  // y varies: constant bytes would trip the stuck detector
      (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      (void)m.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, seq));
    }
    const FrameReport& r = m.end_frame();
    if ((r.newly_latched & 2U) != 0U && out.b_latched < 0) out.b_latched = k;
    out.safe = out.safe || r.safe_request;
  }
  return out;
}
}  // namespace

TFC_TEST(reference_a_signal_moving_less_than_a_tolerance_per_frame_is_judged_against_where_it_was) {
  // 0.5 dps per frame is half a tolerance: below the noise-free step threshold the prediction is not used, so the decision band
  // is the classic one: a bias of 1.7 is 0.5 + 1.7 = 2.2 from the last agreed value, beyond 2 tolerances: blamed.
  const DuplexResult a = duplex_bias(0.5F, 1.7F);
  CHECK(a.b_latched == 32 && !a.safe);
  const DuplexResult b = duplex_bias(0.5F, -1.7F);  // the other direction: 0.5 - 1.7 = -1.2: within 2 tolerances, unresolved
  CHECK(b.b_latched < 0 && b.safe);
  const DuplexResult still = duplex_bias(0.0F, 2.2F);  // (the 0.125 dps quantisation rounds 2.05 down to exactly 2.0)
  CHECK(still.b_latched == 32);
  const DuplexResult below = duplex_bias(0.0F, 1.95F);
  CHECK(below.b_latched < 0 && below.safe);
}

TFC_TEST(reference_a_signal_moving_more_than_two_tolerances_per_frame_is_judged_against_the_motion) {
  // 3 tolerances per frame: with the standstill reference the healthy node would be 3.0 from the last value (blamed). The full
  // prediction makes a bias of 2.5 plainly B's.
  const DuplexResult a = duplex_bias(3.0F, 2.5F);
  CHECK(a.b_latched == 32 && !a.safe);
  const DuplexResult b = duplex_bias(3.0F, -2.5F);
  CHECK(b.b_latched == 32 && !b.safe);
}

// ============================== the frame number (ADR-018) ==============================
TFC_TEST(frame_number_the_manager_counts_its_own_frames_and_can_be_told_the_number) {
  RedundancyManager m;
  for (int k = 0; k < 10; ++k) {
    CHECK(m.frame_number() == static_cast<uint32_t>(k));
    (void)step(m, k, kNone);
  }
  CHECK(m.frame_number() == 10U);
  m.begin_frame(300U);  // SYNC says it is frame 300 (a manager that joined late, or follows a master)
  CHECK(m.frame_number() == 300U);
  const uint8_t seq = static_cast<uint8_t>(300U & 0xFFU);
  for (uint8_t n = 0; n < 3U; ++n) {
    (void)m.on_frame(pack_gyro(n, Vec3{{smooth(300), -2.0F, 1.0F}}, seq));
    (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
    (void)m.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, seq));
  }
  (void)m.end_frame();
  CHECK(m.counters().seq_bad == 0U && m.frame_number() == 301U);
}

TFC_TEST(frame_number_a_node_one_number_early_or_late_is_seen_in_every_stream) {
  for (int shift : {1, -1, 2, -2, 7}) {
    RedundancyManager m;
    int latched = -1;
    for (int k = 0; k < 40; ++k) {
      m.begin_frame();
      for (uint8_t n = 0; n < 3U; ++n) {
        const uint8_t seq = static_cast<uint8_t>((n == 1U && k >= 10) ? k + shift : k);  // node B stamps the wrong number from frame 10
        (void)m.on_frame(pack_gyro(n, Vec3{{smooth(k), -2.0F, 1.0F}}, seq));
        (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
        (void)m.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, seq));
      }
      if ((m.end_frame().newly_latched & 2U) != 0U && latched < 0) latched = k;
    }
    CHECK(latched == 12);  // three bad frames: the wrong number is as good as no frame
  }
}

TFC_TEST(frame_number_a_damaged_frame_fills_its_slot_so_a_clean_copy_a_cycle_later_is_a_repeat) {
  RedundancyManager m;
  for (int k = 0; k < 10; ++k) (void)step(m, k, kNone);
  m.begin_frame();  // cycle 10: node B's gyro frame arrives damaged
  Frame bad = pack_gyro(1, Vec3{{smooth(10), -2.0F, 1.0F}}, 10);
  bad.data[0] = static_cast<uint8_t>(bad.data[0] ^ 1U);
  for (uint8_t n = 0; n < 3U; ++n) {
    if (n == 1U) {
      (void)m.on_frame(bad);
    } else {
      (void)m.on_frame(pack_gyro(n, Vec3{{smooth(10), -2.0F, 1.0F}}, 10));
    }
    (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, 10));
    (void)m.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, 10));
  }
  (void)m.end_frame();
  CHECK(m.counters().crc_bad == 1U && m.counters().seq_bad == 0U);
  m.begin_frame();  // cycle 11: a clean copy of the frame numbered 10 turns up, besides the on-time frames
  (void)m.on_frame(pack_gyro(1, Vec3{{smooth(10), -2.0F, 1.0F}}, 10));
  for (uint8_t n = 0; n < 3U; ++n) {
    (void)m.on_frame(pack_gyro(n, Vec3{{smooth(11), -2.0F, 1.0F}}, 11));
    (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, 11));
    (void)m.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, 11));
  }
  (void)m.end_frame();
  CHECK(m.counters().seq_bad == 1U);  // the damaged frame had filled the slot of cycle 10: this is not a late frame
}
