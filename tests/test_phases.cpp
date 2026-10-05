// SPDX-License-Identifier: MIT
// Mission phases and roles (ADR-023, docs/MISSION_PHASES.md): the phase table, the `phase` command, the interlock tiers that follow the phase, the WARM role (a computer that is
// shadow-voted but does not vote, promoted by `reintegrate`), the `noop` command, and the alert below the phase minimum (never an automatic abort).
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

#include "ground.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;
using tfct::gcmd;

struct Rig {
  RedundancyManager mgr;
  std::array<float, 3> cmd_offset{};
  std::array<bool, 3> silent{};
  uint32_t frame = 0U;
  uint8_t counter = 0U;

  static RedundancyConfig with_phases(bool on) {
    RedundancyConfig c;
    c.phases = on;
    return c;
  }
  explicit Rig(bool phases_on = true) : mgr(with_phases(phases_on)) {}

  FrameReport step(std::initializer_list<Frame> extra = {}) {
    mgr.begin_frame(frame);
    const float t = static_cast<float>(frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      if (silent[n]) {
        continue;
      }
      const uint8_t seq = static_cast<uint8_t>(frame);
      Vec3 g;
      g.v = {3.0F * std::sin(0.2F * t), 2.0F * std::cos(0.15F * t), 0.5F * std::sin(0.1F * t)};
      Vec3 a;
      a.v = {0.1F * std::sin(0.13F * t), 0.1F * std::cos(0.11F * t), 1.0F};
      mgr.on_frame(pack_gyro(n, g, seq));
      mgr.on_frame(pack_accel(n, a, seq));
      mgr.on_frame(pack_cmd(n, Command{(0.3F * std::sin(0.2F * t)) + cmd_offset[n], 0.05F * std::cos(0.15F * t), 0x1111U}, seq));
    }
    for (const Frame& f : extra) {
      mgr.on_frame(f);
    }
    const FrameReport rep = mgr.end_frame();
    ++frame;
    return rep;
  }
  FrameReport run(unsigned n) {
    FrameReport r;
    for (unsigned i = 0; i < n; ++i) {
      r = step();
    }
    return r;
  }
  // One command in one frame; returns its result.
  CommandResult send(GroundOp op, unsigned node, bool arm = false) {
    const Frame f = gcmd(op, static_cast<uint8_t>(node), ++counter, arm);
    const FrameReport rep = step({f});
    return rep.command_count == 0U ? CommandResult::RefusedBadOp : rep.commands[0].result;
  }
  // The same, with its ARM first.
  CommandResult send_armed(GroundOp op, unsigned node) {
    (void)send(op, node, true);
    return send(op, node, false);
  }
};

}  // namespace

TFC_TEST(phase_the_table_has_the_nominal_and_the_minimum_of_each_phase_and_the_minimum_never_exceeds_the_nominal) {
  using namespace tfc::phases;
  CHECK(kRules[kOff].nominal == 0U && kRules[kOff].minimum == 0U);
  CHECK(kRules[kPowerUp].nominal == 3U && kRules[kPowerUp].minimum == 1U);
  CHECK(kRules[kPreLaunch].nominal == 3U && kRules[kPreLaunch].minimum == 3U);
  CHECK(kRules[kAscent].nominal == 3U && kRules[kAscent].minimum == 2U);
  CHECK(kRules[kCoast].nominal == 2U && kRules[kCoast].minimum == 2U);
  CHECK(kRules[kPreBurn].nominal == 3U && kRules[kPreBurn].minimum == 3U);
  CHECK(kRules[kBurn].nominal == 3U && kRules[kBurn].minimum == 2U);
  CHECK(kRules[kSafed].nominal == 3U && kRules[kSafed].minimum == 1U);
  for (const Rule& r : kRules) {
    CHECK(r.minimum <= r.nominal);
  }
}

TFC_TEST(phase_without_phases_the_manager_knows_none_and_the_command_is_refused) {
  Rig r(false);
  (void)r.run(10U);
  CHECK(r.mgr.mission_phase() == phases::kOff);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::RefusedBadOp);
  const FrameReport rep = r.run(2U);
  CHECK(rep.phase == 0U && !rep.below_minimum);
}

TFC_TEST(phase_the_manager_starts_in_power_up_and_the_command_changes_it_one_phase_at_a_time_or_any) {
  Rig r;
  const FrameReport rep = r.run(10U);
  CHECK(rep.phase == phases::kPowerUp && r.mgr.mission_phase() == phases::kPowerUp);
  CHECK(r.send(GroundOp::Phase, phases::kPreLaunch) == CommandResult::Accepted);
  CHECK(r.mgr.mission_phase() == phases::kPreLaunch && r.step().phase == phases::kPreLaunch);
  CHECK(r.send(GroundOp::Phase, phases::kPreLaunch) == CommandResult::AlreadyDone);
  CHECK(r.send(GroundOp::Phase, phases::kCount) == CommandResult::RefusedBadNode);  // there is no phase 8
  CHECK(r.send(GroundOp::Phase, 200U) == CommandResult::RefusedBadNode);
  CHECK(r.mgr.mission_phase() == phases::kPreLaunch);
  CHECK(r.mgr.counters().phase_changes == 1U);
  for (uint8_t p = phases::kOff; p < phases::kCount; ++p) {  // every phase can be reached with three computers
    if (p != r.mgr.mission_phase()) {
      CHECK(r.send(GroundOp::Phase, p) == CommandResult::Accepted && r.mgr.mission_phase() == p);
    }
  }
}

TFC_TEST(phase_the_direct_command_also_refuses_a_phase_that_does_not_exist) {
  Rig r;
  (void)r.run(5U);
  CHECK(r.mgr.command(GroundOp::Phase, phases::kCount) == CommandResult::RefusedBadNode);  // (the frame path refuses it earlier; this is the API)
  CHECK(r.mgr.command(GroundOp::Phase, 255U) == CommandResult::RefusedBadNode);
  CHECK(r.mgr.mission_phase() == phases::kPowerUp && r.mgr.counters().phase_changes == 0U);
}

TFC_TEST(phase_a_change_that_cannot_meet_the_minimum_is_refused_and_the_phase_is_held) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send_armed(GroundOp::Disable, 2U) == CommandResult::Accepted);  // power-up wants one voter at the least and three nominally: an ARM below three
  const FrameReport rep = r.run(2U);
  CHECK(rep.healthy == 2U);
  CHECK(r.send(GroundOp::Phase, phases::kPreLaunch) == CommandResult::RefusedPhase);  // 3 needed
  CHECK(r.mgr.mission_phase() == phases::kPowerUp);
  CHECK(r.send(GroundOp::Phase, phases::kPreBurn) == CommandResult::RefusedPhase);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::Accepted);  // 2 are enough
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Phase, phases::kBurn) == CommandResult::Accepted);
  CHECK(r.mgr.counters().commands_refused == 2U);
}

TFC_TEST(phase_the_edge_of_the_minimum_is_exact_one_more_computer_is_enough) {
  Rig r;
  (void)r.run(10U);
  r.cmd_offset[1] = 0.3F;  // B is off by far: latched
  (void)r.run(20U);
  CHECK(r.mgr.state(1) != NodeState::Healthy && r.mgr.counters().commands_accepted == 0U);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::Accepted);  // minimum 2, two healthy
  r.silent[0] = true;
  (void)r.run(20U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::RefusedPhase);  // minimum 2, one healthy
  CHECK(r.send(GroundOp::Phase, phases::kSafed) == CommandResult::Accepted);     // minimum 1
}

TFC_TEST(phase_the_disable_tiers_follow_the_phase_plain_above_the_nominal_arm_below_it_refused_below_the_minimum) {
  Rig r;  // ascent: nominal 3, minimum 2
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Disable, 0U) == CommandResult::RefusedNotArmed);  // three voters to two: below the nominal 3
  CHECK(r.send_armed(GroundOp::Disable, 0U) == CommandResult::Accepted);
  CHECK(r.send_armed(GroundOp::Disable, 1U) == CommandResult::RefusedPhase);  // two to one: below the minimum, even with an ARM
  CHECK(r.mgr.state(1) == NodeState::Healthy);

  Rig c;  // coast: nominal 2, minimum 2: one computer can go plain, the next not at all
  (void)c.run(10U);
  CHECK(c.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(c.send(GroundOp::Disable, 0U) == CommandResult::Accepted);
  CHECK(c.send_armed(GroundOp::Disable, 1U) == CommandResult::RefusedPhase);

  Rig p;  // pre-launch: three or nothing
  (void)p.run(10U);
  CHECK(p.send(GroundOp::Phase, phases::kPreLaunch) == CommandResult::Accepted);
  CHECK(p.send_armed(GroundOp::Disable, 2U) == CommandResult::RefusedPhase);

  Rig o;  // off: nothing is needed, and the last voter can be removed as before, with an ARM and as a critical command
  (void)o.run(10U);
  CHECK(o.send(GroundOp::Phase, phases::kOff) == CommandResult::Accepted);
  CHECK(o.send(GroundOp::Disable, 0U) == CommandResult::Accepted);
  CHECK(o.send(GroundOp::Disable, 1U) == CommandResult::Accepted);
  CHECK(o.send_armed(GroundOp::Disable, 2U) == CommandResult::Accepted);
}

TFC_TEST(phase_a_command_that_does_not_remove_a_voter_is_not_held_to_the_tiers) {
  Rig r;  // pre-launch wants three, but disabling a computer that is already latched takes no voter away
  (void)r.run(10U);
  r.cmd_offset[2] = 0.3F;
  (void)r.run(20U);
  CHECK(r.mgr.state(2) != NodeState::Healthy);
  CHECK(r.send(GroundOp::Disable, 2U) == CommandResult::Accepted && r.mgr.state(2) == NodeState::Disabled);
}

TFC_TEST(phase_below_the_minimum_is_an_alert_and_nothing_else_there_is_no_automatic_abort) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::Accepted);
  CHECK(!r.run(5U).below_minimum);
  r.silent[0] = true;
  FrameReport two;
  for (unsigned i = 0; i < 20U; ++i) {
    two = r.step();
  }
  CHECK(two.healthy == 2U && !two.below_minimum && two.phase == phases::kAscent);  // exactly the minimum is not below it
  r.silent[1] = true;
  FrameReport rep;
  for (unsigned i = 0; i < 20U; ++i) {
    rep = r.step();
  }
  CHECK(rep.healthy == 1U && rep.below_minimum && rep.phase == phases::kAscent);
  CHECK(rep.mode == Mode::Simplex && !rep.safe_request);  // Simplex in ascent alerts only
  CHECK(r.mgr.counters().below_minimum_frames > 0U);
  const uint32_t before = r.mgr.counters().below_minimum_frames;
  (void)r.step();
  CHECK(r.mgr.counters().below_minimum_frames == before + 1U);
}

TFC_TEST(phase_a_computer_rested_as_warm_does_not_vote_is_shadow_voted_and_stays_ready_until_it_is_promoted) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::Accepted);
  FrameReport rep = r.run(3U);
  CHECK(r.mgr.state(2) == NodeState::Probation && r.mgr.warm(2U) && rep.warm_mask == 0x04U && rep.healthy == 2U);
  CHECK((rep.probation_mask & 0x04U) != 0U && (rep.valid_mask & 0x04U) == 0U);
  rep = r.run(400U);  // far longer than the probation: it has proved itself, and it stays resting
  CHECK(r.mgr.state(2) == NodeState::Probation && rep.warm_mask == 0x04U && rep.healthy == 2U);
  CHECK(r.mgr.counters().reintegrations == 0U);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::AlreadyDone);
  CHECK(r.send(GroundOp::Reintegrate, 2U) == CommandResult::Accepted);  // the promotion
  rep = r.run(3U);
  CHECK(r.mgr.state(2) == NodeState::Healthy && rep.warm_mask == 0U && rep.healthy == 3U && r.mgr.counters().reintegrations == 1U);
  CHECK(!r.mgr.warm(2U));
}

TFC_TEST(phase_a_promotion_is_made_by_the_probation_criteria_so_a_computer_that_was_not_ready_is_not_promoted) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Warm, 1U) == CommandResult::Accepted);
  (void)r.run(20U);  // too early: it has not yet been clean for the probation length
  CHECK(r.send(GroundOp::Reintegrate, 1U) == CommandResult::Accepted);
  (void)r.run(2U);
  CHECK(r.mgr.state(1) == NodeState::Probation && !r.mgr.warm(1U));  // no longer resting, still proving itself
  (void)r.run(150U);
  CHECK(r.mgr.state(1) == NodeState::Healthy);
}

TFC_TEST(phase_a_computer_that_is_rested_again_proves_itself_again_before_it_can_be_promoted) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::Accepted);
  (void)r.run(150U);
  CHECK(r.send(GroundOp::Reintegrate, 2U) == CommandResult::Accepted);
  (void)r.run(3U);
  CHECK(r.mgr.state(2) == NodeState::Healthy);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::Accepted);  // rested a second time
  CHECK(r.send(GroundOp::Reintegrate, 2U) == CommandResult::Accepted);
  (void)r.run(5U);
  CHECK(r.mgr.state(2) == NodeState::Probation);  // it counts its clean frames from the start again
  (void)r.run(150U);
  CHECK(r.mgr.state(2) == NodeState::Healthy);
}

TFC_TEST(phase_the_phase_switch_is_part_of_the_guarded_configuration) {
  RedundancyConfig cfg;
  cfg.phases = true;
  RedundancyConfig plain;
  CHECK(config_digest(cfg) != config_digest(plain));  // the phase switch is part of the configuration that is guarded
}

TFC_TEST(phase_a_warm_computer_that_fails_the_shadow_vote_is_a_faulty_one_and_is_latched) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Warm, 0U) == CommandResult::Accepted);
  (void)r.run(150U);
  r.cmd_offset[0] = 0.3F;
  (void)r.run(5U);
  CHECK(r.mgr.state(0) == NodeState::Latched && !r.mgr.warm(0U) && r.mgr.counters().probation_failures >= 1U);
  const FrameReport rep = r.run(1U);
  CHECK(rep.warm_mask == 0U && (rep.latched_mask & 1U) != 0U && r.mgr.counters().integrity_faults == 0U);  // the bit was cleared by the failure, not by the scrub
}

TFC_TEST(phase_warm_and_disable_on_a_resting_computer_and_warm_on_one_that_cannot_rest) {
  Rig r;
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::Accepted);
  CHECK(r.send(GroundOp::Disable, 2U) == CommandResult::Accepted);  // it is not a voter: no tier applies
  CHECK(r.mgr.state(2) == NodeState::Disabled && !r.mgr.warm(2U));
  (void)r.run(2U);
  CHECK(r.mgr.counters().integrity_faults == 0U);  // cleared by the command, not by the scrub
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::RefusedNotHealthy);
  CHECK(r.send(GroundOp::Warm, 3U) == CommandResult::RefusedBadNode);
  CHECK(r.send(GroundOp::Warm, 200U) == CommandResult::RefusedBadNode);
  r.cmd_offset[1] = 0.3F;
  (void)r.run(20U);
  CHECK(r.mgr.state(1) != NodeState::Healthy);
  CHECK(r.send(GroundOp::Warm, 1U) == CommandResult::RefusedNotHealthy);
}

TFC_TEST(phase_the_rest_follows_the_tiers_also_without_phases_and_never_takes_the_last_voter) {
  Rig r(false);
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Warm, 0U) == CommandResult::Accepted);  // three to two: plain, as for disable
  CHECK(r.send(GroundOp::Warm, 1U) == CommandResult::RefusedNotArmed);  // two to one needs an ARM
  CHECK(r.send_armed(GroundOp::Warm, 1U) == CommandResult::Accepted);
  CHECK(r.send_armed(GroundOp::Warm, 2U) == CommandResult::RefusedPhase);  // the last voter cannot rest
  CHECK(r.mgr.state(2) == NodeState::Healthy);
}

TFC_TEST(phase_noop_changes_nothing_needs_no_arm_and_is_answered_like_any_command) {
  Rig r;
  (void)r.run(10U);
  const uint32_t accepted = r.mgr.counters().commands_accepted;
  const Frame f = gcmd(GroundOp::Noop, 2U, ++r.counter);  // the node field is ignored
  const FrameReport rep = r.step({f});
  CHECK(rep.command_count == 1U && rep.commands[0].result == CommandResult::Accepted && rep.commands[0].op == static_cast<uint8_t>(GroundOp::Noop));
  CHECK((rep.commands[0].flags & (cmdflag::kArm | cmdflag::kArmed | cmdflag::kCritical)) == 0U);
  CHECK(r.mgr.counters().commands_accepted == accepted + 1U && r.mgr.counters().commands_refused == 0U);
  CHECK(rep.healthy == 3U && rep.phase == phases::kPowerUp && rep.latched_mask == 0U);
  const Frame again = gcmd(GroundOp::Noop, 200U, r.counter);  // a stale counter is replay, as for any command: dropped
  (void)r.step({again});
  CHECK(r.mgr.counters().commands_replayed == 1U);
  CHECK(r.send(GroundOp::Noop, 7U) == CommandResult::Accepted);  // any node field
}

TFC_TEST(phase_the_new_operations_are_named_and_unknown_ones_are_refused) {
  CHECK(std::string_view(op_text(static_cast<uint8_t>(GroundOp::Phase))) == "phase");
  CHECK(std::string_view(op_text(static_cast<uint8_t>(GroundOp::Noop))) == "noop");
  CHECK(std::string_view(op_text(static_cast<uint8_t>(GroundOp::Warm))) == "warm");
  CHECK(std::string_view(op_text(10U)) == "unknown-op");
  CHECK(std::string_view(result_text(CommandResult::RefusedPhase)) != "?" && std::string_view(result_text(CommandResult::RefusedNotHealthy)) != "?");
  Rig r;
  (void)r.run(5U);
  const Frame f = tfct::gcmd_raw(10U, 0U, ++r.counter);
  const FrameReport rep = r.step({f});
  CHECK(rep.command_count == 1U && rep.commands[0].result == CommandResult::RefusedBadOp);
}

TFC_TEST(phase_the_arm_code_of_the_new_operations_is_their_own) {
  Rig r;  // an ARM for warm on node 1 does not cover disable on node 1, nor warm on node 2
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kAscent) == CommandResult::Accepted);
  (void)r.send(GroundOp::Warm, 1U, true);
  CHECK(r.send(GroundOp::Disable, 1U) == CommandResult::RefusedNotArmed);
  (void)r.send(GroundOp::Warm, 1U, true);
  CHECK(r.send(GroundOp::Warm, 2U) == CommandResult::RefusedNotArmed);
  (void)r.send(GroundOp::Warm, 1U, true);
  CHECK(r.send(GroundOp::Warm, 1U) == CommandResult::Accepted);
}

TFC_TEST(phase_the_direct_command_refuses_a_computer_that_does_not_exist_and_warm_is_false_for_one) {
  Rig r;
  (void)r.run(5U);
  CHECK(r.mgr.command(GroundOp::Warm, 3U) == CommandResult::RefusedBadNode);
  CHECK(r.mgr.command(GroundOp::Warm, 200U) == CommandResult::RefusedBadNode);
  CHECK(!r.mgr.warm(3U) && !r.mgr.warm(200U));
}

TFC_TEST(phase_with_the_sensor_split_on_the_phase_number_is_not_taken_for_an_imu_and_an_imu_cannot_be_rested) {
  RedundancyConfig cfg;
  cfg.phases = true;
  cfg.sensor_split = true;
  Rig r;
  r.mgr = RedundancyManager(cfg);
  (void)r.run(10U);
  CHECK(r.send(GroundOp::Phase, phases::kCoast) == CommandResult::Accepted);  // 4: also the number of IMU A
  CHECK(r.mgr.mission_phase() == phases::kCoast);
  CHECK(r.send(GroundOp::Phase, phases::kBurn) == CommandResult::Accepted);   // 6
  CHECK(r.send(GroundOp::Warm, 4U) == CommandResult::RefusedBadNode);         // an IMU channel has no role
  CHECK(r.send_armed(GroundOp::Warm, 2U) == CommandResult::Accepted);         // (burn wants three: an ARM below that)
}
