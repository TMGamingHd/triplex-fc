// SPDX-License-Identifier: MIT
// The actuator node's logic: the vote, the node exclusion, the slew bound, the Safe sequence and its exit conditions, and the reset behaviour.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/act.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// Feeds ACT frames: `p` and `y` are each node's command, `present` which nodes send this frame.
struct Bench {
  ActLogic act;
  uint32_t frame = 0U;
  explicit Bench(const ActConfig& c = ActConfig{}) : act(c) {}

  const ActOutput& step(const std::array<float, 3>& p, const std::array<float, 3>& y, uint8_t present = 0x07U) {
    act.begin_frame();
    for (uint8_t n = 0; n < 3U; ++n) {
      if (((present >> n) & 1U) != 0U) {
        (void)act.on_frame(pack_cmd(n, Command{p[n], y[n], 0U}, static_cast<uint8_t>(frame)));
      }
    }
    ++frame;
    return act.end_frame();
  }
  // All three nodes agree on one command.
  const ActOutput& same(float p, float y, uint8_t present = 0x07U) { return step({p, p, p}, {y, y, y}, present); }
  void run_same(float p, float y, unsigned n, uint8_t present = 0x07U) {
    for (unsigned i = 0; i < n; ++i) {
      (void)same(p, y, present);
    }
  }
  // Boots from power-on and goes through Standby into Nominal.
  void nominal(float p = 0.0F, float y = 0.0F) {
    ActRecord none{};
    act.boot(ResetCause::PowerOn, none);
    run_same(p, y, 120U);
    CHECK(act.output().mode == ActMode::Nominal);
  }
};

}  // namespace

TFC_TEST(act_stays_neutral_in_standby_until_votes_have_been_good_for_a_while_then_follows) {
  Bench b;
  ActRecord none{};
  b.act.boot(ResetCause::PowerOn, none);
  CHECK(b.act.output().mode == ActMode::Standby);
  for (int i = 0; i < 99; ++i) {
    const ActOutput& o = b.same(3.0F, -2.0F);
    CHECK(o.mode == ActMode::Standby && o.pitch_deg == 0.0F && o.yaw_deg == 0.0F);  // the pad: nothing moves
  }
  const ActOutput& o = b.same(3.0F, -2.0F);  // the 100th good vote
  CHECK(o.mode == ActMode::Nominal);
  const ActOutput& first = b.same(3.0F, -2.0F);
  CHECK(near(first.pitch_deg, 1.0F, 1e-6F) && near(first.yaw_deg, -1.0F, 1e-6F));  // slewing toward the command, 1 degree a frame
}

TFC_TEST(act_votes_out_a_wrong_node_and_excludes_it_after_persistent_disagreement) {
  Bench b;
  b.nominal(2.0F, 1.0F);
  b.run_same(2.0F, 1.0F, 20U);
  for (int i = 0; i < 2; ++i) {  // two bad frames: not yet excluded
    const ActOutput& o = b.step({2.0F, 7.0F, 2.0F}, {1.0F, 1.0F, 1.0F});
    CHECK(near(o.pitch_deg, 2.0F, 1e-6F) && o.excluded_nodes == 0U && o.vote_status == static_cast<uint8_t>(VoteStatus::Triplex));
  }
  const ActOutput& o = b.step({2.0F, 7.0F, 2.0F}, {1.0F, 1.0F, 1.0F});  // the third in five
  CHECK(o.excluded_nodes == 0x02U && near(o.pitch_deg, 2.0F, 1e-6F));
  const ActOutput& after = b.step({2.0F, 7.0F, 2.0F}, {1.0F, 1.0F, 1.0F});  // node 1 no longer takes part
  CHECK(after.voted_nodes == 0x05U && after.vote_status == static_cast<uint8_t>(VoteStatus::Duplex));
  CHECK(!after.held && after.mode == ActMode::Nominal);
  b.act.clear_exclusions();
  const ActOutput& back = b.step({2.0F, 2.0F, 2.0F}, {1.0F, 1.0F, 1.0F});
  CHECK(back.excluded_nodes == 0U && back.voted_nodes == 0x07U);
}

TFC_TEST(act_counts_missing_frames_against_a_node) {
  Bench b;
  b.nominal();
  const ActOutput* o = nullptr;
  for (int i = 0; i < 3; ++i) {
    o = &b.same(0.0F, 0.0F, 0x05U);  // node 1 is silent
  }
  CHECK(o->excluded_nodes == 0x02U);
  CHECK(o->mode == ActMode::Nominal && !o->held);  // the other two carry on
}

TFC_TEST(act_duplex_agreement_uses_the_mean_and_a_miscompare_holds_the_output) {
  Bench b;
  b.nominal();
  b.run_same(4.0F, 0.0F, 10U);
  const ActOutput& agree = b.step({4.0F, 4.02F, 0.0F}, {0.0F, 0.0F, 0.0F}, 0x03U);
  CHECK(!agree.held && agree.vote_status == static_cast<uint8_t>(VoteStatus::Duplex));
  const ActOutput& miss = b.step({4.0F, 6.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, 0x03U);  // the two cannot be told apart
  CHECK(miss.held && miss.mode == ActMode::Nominal);
  CHECK(near(miss.pitch_deg, 4.0F, 0.05F));  // unchanged: the last output
  CHECK(miss.excluded_nodes == 0U);          // nobody blamed
}

TFC_TEST(act_holds_through_one_or_two_lost_votes_and_enters_safe_on_the_third) {
  Bench b;
  b.nominal();
  b.run_same(3.0F, 3.0F, 10U);
  const float held_pitch = b.act.output().pitch_deg;
  const ActOutput& a = b.step({3.0F, 9.0F, 20.0F}, {3.0F, 9.0F, 20.0F});  // three different values: no majority
  CHECK(a.held && a.mode == ActMode::Nominal && a.pitch_deg == held_pitch);
  const ActOutput& c = b.step({3.0F, 9.0F, 20.0F}, {3.0F, 9.0F, 20.0F});
  CHECK(c.held && c.mode == ActMode::Nominal);
  const ActOutput& s = b.step({3.0F, 9.0F, 20.0F}, {3.0F, 9.0F, 20.0F});
  CHECK(s.mode == ActMode::Safe && s.phase == SafePhase::Hold && s.cause == SafeCause::LostVotes);
  CHECK(s.pitch_deg == held_pitch);  // frozen: no step
  CHECK(b.act.safe_entries() == 1U && b.act.held_frames() == 3U);  // the two held frames and the one that entered Safe
  Bench r;
  r.nominal();
  r.same(5.0F, 5.0F);
  (void)r.same(5.0F, 5.0F, 0x00U);  // everything silent
  (void)r.same(5.0F, 5.0F, 0x00U);
  CHECK(r.same(5.0F, 5.0F, 0x00U).mode == ActMode::Safe);
}

TFC_TEST(act_recovers_from_a_single_lost_vote) {
  Bench b;
  b.nominal(1.0F, 1.0F);
  (void)b.step({1.0F, 9.0F, 20.0F}, {1.0F, 9.0F, 20.0F});
  const ActOutput& o = b.same(1.0F, 1.0F);
  CHECK(!o.held && o.mode == ActMode::Nominal);
  (void)b.step({1.0F, 9.0F, 20.0F}, {1.0F, 9.0F, 20.0F});
  (void)b.step({1.0F, 9.0F, 20.0F}, {1.0F, 9.0F, 20.0F});
  CHECK(b.same(1.0F, 1.0F).mode == ActMode::Nominal);  // the count was reset by the good frame: no Safe
}

TFC_TEST(act_output_never_moves_faster_than_the_slew_bound_in_nominal) {
  Bench b;
  b.nominal(0.0F, 0.0F);
  float last = b.act.output().pitch_deg;
  for (int i = 0; i < 12; ++i) {
    const ActOutput& o = b.same(8.0F, 0.0F);  // a jump of 8 degrees in the command
    CHECK(o.pitch_deg - last <= 1.0F + 1e-6F);
    last = o.pitch_deg;
  }
  CHECK(near(last, 8.0F, 1e-5F));
}

TFC_TEST(act_safe_freezes_holds_ramps_to_neutral_without_a_step_and_stays) {
  Bench b;
  b.nominal(6.0F, -4.0F);
  b.run_same(6.0F, -4.0F, 20U);
  CHECK(near(b.act.output().pitch_deg, 6.0F, 1e-5F));
  b.act.safe_request(true);  // the flight computers ask for Safe
  float p = b.act.output().pitch_deg;
  float y = b.act.output().yaw_deg;
  const ActOutput& entry = b.same(6.0F, -4.0F);
  CHECK(entry.mode == ActMode::Safe && entry.cause == SafeCause::FcRequest && entry.phase == SafePhase::Hold);
  CHECK(entry.pitch_deg == p && entry.yaw_deg == y);  // frozen at once
  for (int i = 1; i < 50; ++i) {
    const ActOutput& o = b.same(0.0F, 0.0F);  // the flight computers' commands no longer matter
    CHECK(o.phase == SafePhase::Hold && o.pitch_deg == p && o.yaw_deg == y);
  }
  float worst = 0.0F;
  unsigned ramp_frames = 0U;
  for (int i = 0; i < 400; ++i) {
    const ActOutput& o = b.same(0.0F, 0.0F);
    worst = std::fmax(worst, std::fmax(std::fabs(o.pitch_deg - p), std::fabs(o.yaw_deg - y)));
    p = o.pitch_deg;
    y = o.yaw_deg;
    if (o.phase == SafePhase::Ramp) {
      ++ramp_frames;
    }
  }
  CHECK(worst <= 0.02F + 1e-6F);               // never a step bigger than the ramp rate
  CHECK(ramp_frames >= 290U && ramp_frames <= 310U);  // 6 degrees at 0.02 a frame
  CHECK(b.act.output().phase == SafePhase::Neutral && p == 0.0F && y == 0.0F);
  for (int i = 0; i < 1000; ++i) {
    CHECK(b.same(5.0F, 5.0F).mode == ActMode::Safe);  // nothing leaves Safe by itself, however good the votes
  }
}

TFC_TEST(act_hardware_safe_line_enters_safe_even_in_standby_and_takes_priority) {
  Bench b;
  b.nominal(2.0F, 2.0F);
  b.act.hardware_safe(true);
  b.act.safe_request(true);
  const ActOutput& o = b.same(2.0F, 2.0F);
  CHECK(o.mode == ActMode::Safe && o.cause == SafeCause::HardwareLine);
  Bench s;
  ActRecord none{};
  s.act.boot(ResetCause::PowerOn, none);
  s.act.safe_request(true);
  CHECK(s.same(0.0F, 0.0F).cause == SafeCause::FcRequest);  // a Safe request on the pad
  Bench h;
  h.act.boot(ResetCause::PowerOn, none);
  h.act.hardware_safe(true);
  CHECK(h.same(0.0F, 0.0F).cause == SafeCause::HardwareLine);
}

TFC_TEST(act_clear_safe_needs_the_exit_conditions) {
  Bench b;
  b.nominal(3.0F, 0.0F);
  b.act.safe_request(true);
  (void)b.same(3.0F, 0.0F);
  CHECK(b.act.output().mode == ActMode::Safe);
  CHECK(!b.act.clear_safe());  // a request is still active
  b.act.safe_request(false);
  CHECK(!b.act.clear_safe());  // and the votes have not been good for 100 frames
  b.run_same(3.0F, 0.0F, 100U);
  b.act.hardware_safe(true);
  CHECK(!b.act.clear_safe());  // the line is asserted
  b.act.hardware_safe(false);
  b.run_same(3.0F, 0.0F, 1U, 0x01U);  // only one node voting, and that breaks the run of two
  CHECK(!b.act.clear_safe());
  b.run_same(3.0F, 0.0F, 100U);
  CHECK(b.act.clear_safe());
  CHECK(b.act.output().mode == ActMode::Nominal && b.act.output().cause == SafeCause::None);
  CHECK(b.act.clears() == 1U && b.act.refused_clears() == 4U);
  CHECK(b.act.clear_safe());  // nothing to clear now: accepted, harmless
  // after the clear the output follows the votes again, within the slew bound
  const float before = b.act.output().pitch_deg;
  const ActOutput& o = b.same(3.0F, 0.0F);
  CHECK(std::fabs(o.pitch_deg - before) <= 1.0F + 1e-6F);
}

TFC_TEST(act_clear_safe_is_refused_with_fewer_than_two_nodes_voting) {
  Bench b;
  b.nominal(1.0F, 1.0F);
  b.act.safe_request(true);
  (void)b.same(1.0F, 1.0F);
  b.act.safe_request(false);
  b.run_same(1.0F, 1.0F, 150U, 0x01U);  // one node only: the votes are trustworthy but not from two nodes
  CHECK(b.act.good_run() >= 100U);
  CHECK(!b.act.clear_safe());
}

TFC_TEST(act_starts_in_standby_after_a_power_on_or_a_damaged_record) {
  ActLogic a;
  ActRecord garbage{};
  garbage.magic = 0x12345678U;
  a.boot(ResetCause::Watchdog, garbage);  // not a power-on, but the record is no good: do not trust it
  CHECK(a.output().mode == ActMode::Standby);
  Bench b;
  b.nominal(4.0F, 4.0F);
  b.run_same(4.0F, 4.0F, 10U);
  const ActRecord rec = b.act.record();
  ActLogic p;
  p.boot(ResetCause::PowerOn, rec);  // a valid record, but power came on: the RAM is not to be believed
  CHECK(p.output().mode == ActMode::Standby && p.output().pitch_deg == 0.0F);
}

TFC_TEST(act_starts_in_safe_after_a_reset_and_resumes_from_the_stored_output) {
  Bench b;
  b.nominal(5.0F, -3.0F);
  b.run_same(5.0F, -3.0F, 20U);
  const ActRecord rec = b.act.record();
  CHECK(act_record_valid(rec));
  ActLogic a;
  a.boot(ResetCause::Watchdog, rec);
  CHECK(a.output().mode == ActMode::Safe && a.output().cause == SafeCause::Reset && a.output().phase == SafePhase::Hold);
  CHECK(near(a.output().pitch_deg, 5.0F, 1e-3F) && near(a.output().yaw_deg, -3.0F, 1e-3F));  // no step across the reset
  a.begin_frame();
  const ActOutput& o = a.end_frame();  // frames with no commands at all: still holding what it had
  CHECK(near(o.pitch_deg, 5.0F, 1e-3F) && o.mode == ActMode::Safe);
  ActConfig neutral;
  neutral.resume_from_stored = false;
  ActLogic n(neutral);
  n.boot(ResetCause::Brownout, rec);
  CHECK(n.output().mode == ActMode::Safe && n.output().pitch_deg == 0.0F && n.output().yaw_deg == 0.0F);
}

TFC_TEST(the_stored_record_detects_damage) {
  Bench b;
  b.nominal(5.0F, 5.0F);
  const ActRecord good = b.act.record();
  CHECK(act_record_valid(good));
  ActRecord r = good;
  r.pitch = static_cast<int16_t>(r.pitch ^ 1);
  CHECK(!act_record_valid(r));
  r = good;
  r.yaw = static_cast<int16_t>(r.yaw ^ 0x100);
  CHECK(!act_record_valid(r));
  r = good;
  r.magic ^= 1U;
  CHECK(!act_record_valid(r));
  r = good;
  r.mode = 9U;  // not a mode, even with a consistent checksum
  r.crc = detail::act_record_crc(r);
  CHECK(!act_record_valid(r));
}

TFC_TEST(act_ignores_what_is_not_a_good_command_frame) {
  ActLogic a;
  a.begin_frame();
  Frame bad = pack_cmd(0, Command{1.0F, 1.0F, 0U}, 1U);
  bad.data[7] ^= 0x33U;
  CHECK(!a.on_frame(bad));
  CHECK(!a.on_frame(pack_gyro(0, Vec3{{1.0F, 0.0F, 0.0F}}, 1U)));
  Frame far = pack_cmd(0, Command{1.0F, 1.0F, 0U}, 1U);
  far.id = id::kCmdBase + 3U;
  CHECK(!a.on_frame(far));
  CHECK(a.on_frame(pack_cmd(2, Command{1.0F, 1.0F, 0U}, 1U)));
}

TFC_TEST(act_output_moves_by_a_bounded_step_whatever_the_flight_computers_send) {
  // A deterministic fuzz: random commands, drops and wild nodes, with a Safe request now and then. The invariants of the Safe-mode requirements:
  // the output is finite, never steps by more than the larger of the slew and the ramp bound, and never leaves Safe without a clear.
  uint32_t s = 12345U;
  auto rnd = [&s]() {
    s = s * 1664525U + 1013904223U;
    return static_cast<float>(s >> 8) / 8388608.0F - 1.0F;
  };
  Bench b;
  b.nominal(0.0F, 0.0F);
  float last_p = b.act.output().pitch_deg;
  float last_y = b.act.output().yaw_deg;
  bool was_safe = false;
  for (int i = 0; i < 20000; ++i) {
    const float base = 8.0F * rnd();
    std::array<float, 3> p{base, base, base};
    std::array<float, 3> y{-base, -base, -base};
    if (rnd() > 0.9F) {
      p[1] = 100.0F * rnd();
    }
    if (rnd() > 0.97F) {
      y[2] = 100.0F * rnd();
    }
    const uint8_t present = rnd() > 0.95F ? 0x05U : 0x07U;
    b.act.safe_request(i % 3000 == 2999);
    const ActOutput& o = b.step(p, y, present);
    CHECK(std::isfinite(o.pitch_deg) && std::isfinite(o.yaw_deg));
    CHECK(std::fabs(o.pitch_deg - last_p) <= 1.0F + 1e-5F && std::fabs(o.yaw_deg - last_y) <= 1.0F + 1e-5F);
    if (was_safe) {
      CHECK(o.mode == ActMode::Safe);  // nothing leaves Safe without clear_safe()
    }
    was_safe = o.mode == ActMode::Safe;
    last_p = o.pitch_deg;
    last_y = o.yaw_deg;
  }
  CHECK(b.act.safe_entries() >= 1U);
}

TFC_TEST(act_safe_ramp_waits_until_both_planes_are_neutral) {
  Bench b;
  b.nominal(1.0F, 3.0F);
  b.run_same(1.0F, 3.0F, 20U);
  b.act.safe_request(true);
  (void)b.same(1.0F, 3.0F);
  b.run_same(0.0F, 0.0F, 49U);  // the hold
  for (int i = 0; i < 70; ++i) {  // the pitch plane reaches neutral first (50 frames), the yaw plane later (150)
    (void)b.same(0.0F, 0.0F);
  }
  CHECK(b.act.output().pitch_deg == 0.0F && b.act.output().yaw_deg > 1.0F && b.act.output().phase == SafePhase::Ramp);
  b.run_same(0.0F, 0.0F, 100U);
  CHECK(b.act.output().yaw_deg == 0.0F && b.act.output().phase == SafePhase::Neutral);
}

TFC_TEST(act_clear_safe_is_refused_while_a_request_is_active_even_with_good_votes) {
  Bench b;
  b.nominal(2.0F, 2.0F);
  b.act.safe_request(true);
  b.run_same(2.0F, 2.0F, 150U);  // good votes for much longer than the exit run, but the request is still on
  CHECK(b.act.output().mode == ActMode::Safe && b.act.good_run() >= 100U);
  CHECK(!b.act.clear_safe());
}

TFC_TEST(act_a_disagreement_in_one_plane_only_holds_and_blames_nobody) {
  Bench b;
  b.nominal(1.0F, 1.0F);
  b.run_same(1.0F, 1.0F, 10U);
  for (int i = 0; i < 2; ++i) {
    const ActOutput& o = b.step({1.0F, 1.0F, 1.0F}, {1.0F, 5.0F, 9.0F});  // pitch agrees, yaw has no majority
    CHECK(o.held && o.excluded_nodes == 0U);
  }
}

TFC_TEST(act_in_standby_does_not_count_untrustworthy_votes_towards_nominal) {
  Bench b;
  ActRecord none{};
  b.act.boot(ResetCause::PowerOn, none);
  for (int i = 0; i < 90; ++i) {
    (void)b.same(0.0F, 0.0F);
  }
  (void)b.step({0.0F, 5.0F, 9.0F}, {0.0F, 5.0F, 9.0F});  // a bad vote resets the run
  for (int i = 0; i < 90; ++i) {
    CHECK(b.same(0.0F, 0.0F).mode == ActMode::Standby);
  }
  b.run_same(0.0F, 0.0F, 20U);
  CHECK(b.act.output().mode == ActMode::Nominal);
}

TFC_TEST(act_output_becomes_the_act_frame_field_for_field_and_survives_the_wire) {
  ActOutput o;
  o.pitch_deg = -3.25F;
  o.yaw_deg = 1.5F;
  o.mode = ActMode::Safe;
  o.phase = SafePhase::Hold;
  o.cause = SafeCause::FcRequest;
  o.held = true;
  o.voted_nodes = 0x5U;
  o.excluded_nodes = 0x2U;
  o.vote_status = 3U;
  const ActFrame a = to_act_frame(o);
  CHECK(a.pitch_deg == -3.25F && a.yaw_deg == 1.5F && a.state == 2U && a.held && a.vote_status == 3U && a.voted_nodes == 5U && a.excluded_nodes == 2U &&
        a.cause == 2U);
  const DecodedAct d = unpack_act_out(pack_act_out(a, 9U));
  CHECK(d.ok && d.seq == 9U && d.act.state == 2U && d.act.cause == 2U && d.act.voted_nodes == 5U && d.act.excluded_nodes == 2U && d.act.held);
  CHECK(std::fabs(d.act.pitch_deg - (-3.25F)) < 0.001F);
  const ActFrame idle = to_act_frame(ActOutput{});  // the defaults: Standby, neutral, nothing voted
  CHECK(idle.state == 0U && !idle.held && idle.cause == 0U && idle.pitch_deg == 0.0F);
}
