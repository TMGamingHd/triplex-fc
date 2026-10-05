// SPDX-License-Identifier: MIT
// The state share (TFC-FDIR-041, docs/design/PROTOCOL.md): each computer broadcasts its view of the strike counts and the last accepted command counter; a computer that restarts rebuilds them from what
// the others say (the value both agree on, else the more conservative), never lowers anything, and cannot be made to disable a healthy computer by one faulty sender.
#include <array>
#include <cmath>
#include <cstdint>

#include "ground.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

namespace tfc {
struct ManagerTestAccess {  // reaches into the manager to set a strike count or damage a record (test code only; the other test files define their own members the same way)
  static void set_strikes(RedundancyManager& m, unsigned node, uint8_t v) { m.strikes_[node] = v; }
  static void damage_command_history(RedundancyManager& m) { m.cmd_have_.n_ = static_cast<uint8_t>(m.cmd_have_.n_ ^ 0x01U); }
  static void damage_command_counter(RedundancyManager& m) { m.cmd_ctr_.n_ = static_cast<uint8_t>(m.cmd_ctr_.n_ ^ 0x01U); }
};
}
namespace {

using namespace tfc;
using tfct::gcmd;

Frame share(unsigned from, uint8_t sa, uint8_t sb, uint8_t sc, uint8_t counter, uint8_t seq = 1U) {
  StateShare s;
  s.strikes = {sa, sb, sc};
  s.command_counter = counter;
  return pack_state_share(static_cast<uint8_t>(from), s, seq);
}

// One quiet frame with all three computers sending, plus any extra frames.
const FrameReport& step(RedundancyManager& m, uint32_t k, std::initializer_list<Frame> extra = {}, float bias_b = 0.0F) {
  m.begin_frame(k);
  const uint8_t seq = static_cast<uint8_t>(k);
  const float t = static_cast<float>(k);
  for (uint8_t n = 0; n < 3U; ++n) {
    Vec3 g;
    g.v = {3.0F * std::sin(0.2F * t) + (n == 1U ? bias_b : 0.0F), 2.0F, 0.5F};
    Vec3 a;
    a.v = {0.0F, 0.0F, 1.0F};
    m.on_frame(pack_gyro(n, g, seq));
    m.on_frame(pack_accel(n, a, seq));
    m.on_frame(pack_cmd(n, Command{0.2F, 0.1F, 0x2222U}, seq));
  }
  for (const Frame& f : extra) {
    m.on_frame(f);
  }
  return m.end_frame();
}

}  // namespace

TFC_TEST(share_a_manager_reports_its_strike_counts_saturated_at_four_bits_and_the_last_command_counter) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 10U; ++k) {
    (void)step(m, k);
  }
  StateShare s = m.state_share();
  CHECK(s.strikes[0] == 0U && s.strikes[1] == 0U && s.strikes[2] == 0U && s.command_counter == 0U);
  for (; k < 30U; ++k) {  // B's gyro is far off: it is latched, one strike
    (void)step(m, k, {}, 50.0F);
  }
  s = m.state_share();
  CHECK(s.strikes[1] == 1U && s.strikes[0] == 0U && s.strikes[2] == 0U);
  (void)step(m, k++, {gcmd(GroundOp::Noop, 0U, 7U)});
  CHECK(m.state_share().command_counter == 7U);
  ManagerTestAccess::set_strikes(m, 0U, 15U);  // four bits carry 0 to 15: more saturates, it does not wrap to a small number
  ManagerTestAccess::set_strikes(m, 1U, 16U);
  ManagerTestAccess::set_strikes(m, 2U, 200U);
  s = m.state_share();
  CHECK(s.strikes[0] == 15U && s.strikes[1] == 15U && s.strikes[2] == 15U);
  ManagerTestAccess::set_strikes(m, 0U, 14U);
  CHECK(m.state_share().strikes[0] == 14U);
  ManagerTestAccess::damage_command_history(m);  // a record that fails its check says "none", not a number it cannot vouch for
  CHECK(m.state_share().command_counter == 0U);
}

TFC_TEST(share_the_frames_of_the_others_are_collected_without_being_counted_as_out_of_schedule_and_a_damaged_one_is_ignored) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 2U, 3U, 40U), share(2U, 0U, 2U, 3U, 40U)});
  CHECK(m.counters().out_of_schedule == 0U);
  Frame bad = share(2U, 9U, 9U, 9U, 99U);
  bad.data[2] = static_cast<uint8_t>(bad.data[2] ^ 0x10U);  // damaged: its CRC no longer matches
  (void)step(m, k++, {bad});
  const RedundancyManager::Restored r = m.restore_from_peers(0U);
  CHECK(r.strikes_raised == 0x06U && m.strikes(1U) == 2U && m.strikes(2U) == 3U);  // the damaged frame was not used: the two earlier ones still agree
  CHECK(r.disabled == 0x04U && m.state(2U) == NodeState::Disabled);
  CHECK(r.counter && m.state_share().command_counter == 40U);
}

TFC_TEST(share_two_that_agree_set_the_value_and_where_they_differ_the_higher_strike_count_is_taken) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 1U, 2U, 0U), share(2U, 0U, 1U, 1U, 0U)});
  const RedundancyManager::Restored r = m.restore_from_peers(0U);
  CHECK(m.strikes(1U) == 1U && m.strikes(2U) == 2U);  // B: both say 1; C: they differ (2 and 1): the higher
  CHECK(r.strikes_raised == 0x06U && r.disabled == 0U && !r.counter);
  CHECK(m.counters().state_restores == 1U);
}

TFC_TEST(share_one_computer_alone_can_raise_a_count_but_never_disable_a_computer_but_two_that_agree_do) {
  RedundancyManager m;  // max_strikes is 3
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 15U, 0U, 0U), share(2U, 0U, 0U, 0U, 0U)});  // B says "C has 15 strikes", C says "none": a liar or a stale view
  RedundancyManager::Restored r = m.restore_from_peers(0U);
  CHECK(m.strikes(1U) == 2U && m.state(1U) == NodeState::Healthy && r.disabled == 0U);  // raised to one below the limit, not disabled
  RedundancyManager one;  // only one of the others has spoken
  for (k = 0U; k < 5U; ++k) {
    (void)step(one, k);
  }
  (void)step(one, k++, {share(1U, 0U, 0U, 15U, 0U)});
  r = one.restore_from_peers(0U);
  CHECK(one.strikes(2U) == 2U && one.state(2U) == NodeState::Healthy);
  RedundancyManager both;  // both say node C is out of strikes
  for (k = 0U; k < 5U; ++k) {
    (void)step(both, k);
  }
  (void)step(both, k++, {share(1U, 0U, 0U, 3U, 0U), share(2U, 0U, 0U, 3U, 0U)});
  r = both.restore_from_peers(0U);
  CHECK(both.state(2U) == NodeState::Disabled && r.disabled == 0x04U && both.strikes(2U) == 3U);
  CHECK(both.counters().nodes_disabled == 1U);
}

TFC_TEST(share_nothing_is_ever_lowered_and_the_restore_is_idempotent_and_does_not_use_the_computers_own_share) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(0U, 0U, 0U, 0U, 99U), share(1U, 2U, 2U, 0U, 50U), share(2U, 2U, 2U, 0U, 50U)});  // A's own share (self = A) says nothing and 99
  (void)m.restore_from_peers(0U);
  CHECK(m.strikes(0U) == 2U && m.strikes(1U) == 2U && m.state_share().command_counter == 50U);
  const RedundancyManager::Restored again = m.restore_from_peers(0U);
  CHECK(again.strikes_raised == 0U && !again.counter && m.counters().state_restores == 1U);
  (void)step(m, k++, {share(1U, 0U, 0U, 0U, 20U), share(2U, 0U, 0U, 0U, 20U)});  // the others now report less (a stale view, or a reset on their side)
  (void)m.restore_from_peers(0U);
  CHECK(m.strikes(0U) == 2U && m.strikes(1U) == 2U && m.state_share().command_counter == 50U);
}

TFC_TEST(share_with_nobody_heard_or_nothing_to_say_the_manager_is_left_as_it_was) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  RedundancyManager::Restored r = m.restore_from_peers(0U);
  CHECK(r.strikes_raised == 0U && r.disabled == 0U && !r.counter && m.counters().state_restores == 0U);
  (void)step(m, k++, {share(1U, 0U, 0U, 0U, 0U), share(2U, 0U, 0U, 0U, 0U)});  // zero strikes, no counter: nothing to restore
  r = m.restore_from_peers(0U);
  CHECK(r.strikes_raised == 0U && !r.counter && m.counters().state_restores == 0U);
  CHECK(m.restore_from_peers(7U).strikes_raised == 0U);  // an unknown own number: every share counts, but there is nothing in them
}

TFC_TEST(share_a_restarted_computer_that_took_the_counter_from_the_others_refuses_the_replay_of_an_old_command) {
  RedundancyManager m;  // freshly restarted: no command history, so it would accept any counter
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 0U, 0U, 200U), share(2U, 0U, 0U, 0U, 200U)});
  CHECK(m.restore_from_peers(0U).counter);
  (void)step(m, k++, {gcmd(GroundOp::Noop, 0U, 150U)});  // a recorded command from long ago
  CHECK(m.counters().commands_replayed == 1U && m.counters().commands_accepted == 0U);
  const FrameReport& rep = step(m, k++, {gcmd(GroundOp::Noop, 0U, 201U)});
  CHECK(rep.command_count == 1U && rep.commands[0].result == CommandResult::Accepted);
  RedundancyManager without;  // for contrast: without the restore the old command is accepted
  for (uint32_t j = 0U; j < 5U; ++j) {
    (void)step(without, j);
  }
  CHECK(step(without, 5U, {gcmd(GroundOp::Noop, 0U, 150U)}).command_count == 1U);
}

TFC_TEST(share_the_counter_the_others_differ_on_is_the_one_that_is_ahead_also_across_the_wrap_and_an_own_newer_counter_is_kept) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 0U, 0U, 250U), share(2U, 0U, 0U, 0U, 3U)});  // 3 is nine ahead of 250
  CHECK(m.restore_from_peers(0U).counter && m.state_share().command_counter == 3U);
  RedundancyManager n;
  for (k = 0U; k < 5U; ++k) {
    (void)step(n, k);
  }
  (void)step(n, k++, {share(1U, 0U, 0U, 0U, 30U), share(2U, 0U, 0U, 0U, 20U)});  // the other way round
  CHECK(n.restore_from_peers(0U).counter && n.state_share().command_counter == 30U);
  (void)step(n, k++, {gcmd(GroundOp::Noop, 0U, 40U)});  // this computer's own history is now 40
  (void)step(n, k++, {share(1U, 0U, 0U, 0U, 35U), share(2U, 0U, 0U, 0U, 35U)});
  CHECK(!n.restore_from_peers(0U).counter && n.state_share().command_counter == 40U);
}

TFC_TEST(share_the_command_counter_of_one_peer_is_taken_when_only_one_has_spoken_and_a_peer_without_history_is_ignored) {
  RedundancyManager m;
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(2U, 0U, 0U, 0U, 77U)});
  CHECK(m.restore_from_peers(0U).counter && m.state_share().command_counter == 77U);
  RedundancyManager n;
  for (k = 0U; k < 5U; ++k) {
    (void)step(n, k);
  }
  (void)step(n, k++, {share(1U, 0U, 0U, 0U, 0U), share(2U, 0U, 0U, 0U, 77U)});  // B has no history (0), C has 77
  CHECK(n.restore_from_peers(0U).counter && n.state_share().command_counter == 77U);
  RedundancyManager o;
  for (k = 0U; k < 5U; ++k) {
    (void)step(o, k);
  }
  (void)step(o, k++, {share(1U, 0U, 0U, 0U, 200U), share(2U, 0U, 0U, 0U, 0U)});  // 200 and "none": a "none" is not a counter that is behind or ahead
  CHECK(o.restore_from_peers(0U).counter && o.state_share().command_counter == 200U);
}

TFC_TEST(share_with_the_strike_limit_switched_off_the_others_can_raise_a_count_but_nothing_is_ever_disabled) {
  RedundancyConfig cfg;
  cfg.max_strikes = 0U;  // strikes never disable
  RedundancyManager m(cfg);
  uint32_t k = 0U;
  for (; k < 5U; ++k) {
    (void)step(m, k);
  }
  (void)step(m, k++, {share(1U, 0U, 9U, 15U, 0U), share(2U, 0U, 4U, 15U, 0U)});
  const RedundancyManager::Restored r = m.restore_from_peers(0U);
  CHECK(m.strikes(1U) == 9U && m.strikes(2U) == 15U);  // the higher where they differ, no cap: there is no limit to hold below
  CHECK(r.disabled == 0U && m.state(1U) == NodeState::Healthy && m.state(2U) == NodeState::Healthy);
}

TFC_TEST(share_a_command_record_that_fails_its_check_says_none_and_is_replaced_by_what_the_others_say) {
  for (const bool counter_damaged : {false, true}) {
    RedundancyManager m;
    uint32_t k = 0U;
    for (; k < 5U; ++k) {
      (void)step(m, k);
    }
    (void)step(m, k++, {gcmd(GroundOp::Noop, 0U, 9U)});
    CHECK(m.state_share().command_counter == 9U);
    if (counter_damaged) {
      ManagerTestAccess::damage_command_counter(m);
    } else {
      ManagerTestAccess::damage_command_history(m);
    }
    CHECK(m.state_share().command_counter == 0U);  // it cannot vouch for the number
    (void)step(m, k++, {share(1U, 0U, 0U, 0U, 5U), share(2U, 0U, 0U, 0U, 5U)});  // (the scrub of this frame has already reset the damaged record)
    (void)m.restore_from_peers(0U);
    CHECK(m.state_share().command_counter == 5U);
  }
}
