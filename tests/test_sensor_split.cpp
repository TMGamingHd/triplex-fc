// SPDX-License-Identifier: MIT
// Sensor-channel health, separate from compute-channel health (core/include/tfc/sensor_health.hpp, ADR-020 case 1, TFC-ARCH-001): with `sensor_split` on, a bad IMU is excluded
// from the sensor consensus and its computer stays a voter for commands; a bad computer is excluded from the command vote and its IMU stays in the consensus. With the split off
// the manager is exactly what it was (the whole of the other manager tests), and the report's sensor fields mirror the computers'.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/auth.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

namespace tfc {
struct ManagerTestAccess {
  static void set_sensor_state_consistent(RedundancyManager& m, unsigned k, uint8_t raw);
  static void set_sensor_state_primary(RedundancyManager& m, unsigned k, uint8_t raw);
};
}

namespace {

using namespace tfc;

struct Fault {
  float gyro_bias = 0.0F;      // dps added to the IMU's gyro x
  bool freeze_sensor = false;  // the IMU repeats its first values
  bool drop_sensor = false;    // no gyro and accel frames
  bool drop_cmd = false;       // no command frame
  float cmd_offset = 0.0F;     // degrees added to the command
};

// Three healthy computers on a signal that changes every frame, with faults that can be set per computer.
struct Rig {
  RedundancyManager mgr;
  std::array<Fault, 3> f{};
  uint32_t frame = 0U;
  uint8_t counter = 0U;

  explicit Rig(bool split, uint32_t grace = 0U) : mgr(make(split, grace)) {}

  static RedundancyConfig make(bool split, uint32_t grace) {
    RedundancyConfig c;
    c.sensor_split = split;
    c.startup_grace_frames = grace;
    return c;
  }

  FrameReport step(const Frame* ground = nullptr, uint8_t only_nodes = 0x07U) {
    mgr.begin_frame(frame);
    const float t = static_cast<float>(frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      if (((only_nodes >> n) & 1U) == 0U) {
        continue;
      }
      const uint8_t seq = static_cast<uint8_t>(frame);
      const float tf = f[n].freeze_sensor ? 40.0F : t;
      Vec3 g;
      g.v = {(3.0F * std::sin(0.2F * tf)) + f[n].gyro_bias, 2.0F * std::cos(0.15F * tf), 0.5F * std::sin(0.1F * tf)};
      Vec3 a;
      a.v = {0.1F * std::sin(0.13F * tf), 0.1F * std::cos(0.11F * tf), 1.0F};
      if (!f[n].drop_sensor) {
        mgr.on_frame(pack_gyro(n, g, seq));
        mgr.on_frame(pack_accel(n, a, seq));
      }
      if (!f[n].drop_cmd) {
        mgr.on_frame(pack_cmd(n, Command{(0.1F * std::sin(0.2F * t)) + f[n].cmd_offset, 0.05F * std::cos(0.15F * t), 0x4321U}, seq));
      }
    }
    if (ground != nullptr) {
      mgr.on_frame(*ground);
    }
    const FrameReport rep = mgr.end_frame();
    ++frame;
    return rep;
  }

  FrameReport run(unsigned frames) {
    FrameReport r;
    for (unsigned i = 0; i < frames; ++i) {
      r = step();
    }
    return r;
  }

  Frame cmd(GroundOp op, unsigned node, bool arm = false) { return pack_ground_auth(op, static_cast<uint8_t>(node), ++counter, kBenchKey, arm); }
};

TFC_TEST(split_off_a_bad_imu_latches_the_whole_computer_and_the_sensor_fields_mirror_it) {
  Rig r(false);
  (void)r.run(20U);
  r.f[1].gyro_bias = 3.0F;
  const FrameReport rep = r.run(10U);
  CHECK(rep.latched_mask == 0x02U && rep.healthy == 2U && rep.mode == Mode::Duplex);
  CHECK(rep.sensor_latched_mask == 0x02U && rep.sensor_healthy == 2U && rep.sensor_mode == Mode::Duplex && rep.sensor_valid_mask == rep.valid_mask);
  CHECK(rep.sensor_probation_mask == rep.probation_mask && rep.sensor_disabled_mask == rep.disabled_mask);
}

TFC_TEST(split_on_a_bad_imu_is_excluded_and_its_computer_stays_a_voter) {
  Rig r(true);
  (void)r.run(20U);
  r.f[1].gyro_bias = 3.0F;
  bool latched_seen = false;
  FrameReport rep;
  FrameReport at_latch;
  for (unsigned i = 0; i < 10U; ++i) {
    rep = r.step();
    if ((rep.sensor_newly_latched & 0x02U) != 0U) {
      latched_seen = true;
      at_latch = rep;
    }
  }
  CHECK(latched_seen);
  CHECK(rep.latched_mask == 0U && rep.healthy == 3U && rep.mode == Mode::Triplex && rep.newly_latched == 0U);  // the computer is still a voter, in Triplex
  CHECK(rep.sensor_latched_mask == 0x02U && rep.sensor_healthy == 2U && rep.sensor_mode == Mode::Duplex && rep.sensor_valid_mask == 0x05U);
  CHECK((at_latch.sensor_reason[1] & reason::kVote) != 0U && at_latch.sensor_reason[0] == 0U && at_latch.sensor_reason[2] == 0U && at_latch.reason[1] == 0U);  // the verdict of the frame it latched in
  CHECK(std::fabs(rep.output[0] - (3.0F * std::sin(0.2F * 29.0F))) < 0.5F);  // the gyro vote follows the two good IMUs
  CHECK(!rep.safe_request && rep.held_mask == 0U);
  CHECK(r.mgr.state(1) == NodeState::Healthy && r.mgr.strikes(1) == 0U);
}

TFC_TEST(split_on_a_bad_computer_is_excluded_from_the_commands_and_its_imu_stays_in_the_consensus) {
  Rig r(true);
  (void)r.run(20U);
  r.f[2].cmd_offset = 0.05F;
  FrameReport rep;
  FrameReport at_latch;
  for (unsigned i = 0; i < 10U; ++i) {
    rep = r.step();
    if ((rep.newly_latched & 0x04U) != 0U) {
      at_latch = rep;
    }
  }
  CHECK(rep.latched_mask == 0x04U && rep.healthy == 2U && rep.mode == Mode::Duplex);
  CHECK(rep.sensor_latched_mask == 0U && rep.sensor_healthy == 3U && rep.sensor_mode == Mode::Triplex && rep.sensor_valid_mask == 0x07U);
  CHECK((at_latch.reason[2] & reason::kVote) != 0U && at_latch.sensor_reason[2] == 0U && at_latch.sensor_newly_latched == 0U);
}

TFC_TEST(split_on_an_imu_of_one_computer_and_another_computer_failing_leave_two_voters_and_two_imus) {
  Rig r(true);
  (void)r.run(20U);
  r.f[0].gyro_bias = 3.0F;      // IMU A
  r.f[1].cmd_offset = 0.05F;    // computer B (case 2 of ADR-020, without the ring: A keeps voting on its commands, B's IMU keeps sensing)
  const FrameReport rep = r.run(12U);
  CHECK(rep.latched_mask == 0x02U && rep.healthy == 2U && rep.mode == Mode::Duplex);
  CHECK(rep.sensor_latched_mask == 0x01U && rep.sensor_healthy == 2U && rep.sensor_mode == Mode::Duplex);
  CHECK(!rep.safe_request && std::fabs(rep.output[kChPitch] - (0.1F * std::sin(0.2F * 31.0F))) < 0.02F);
}

TFC_TEST(split_on_missing_sensor_frames_are_the_imus_fault_and_missing_command_frames_the_computers) {
  Rig a(true);
  (void)a.run(20U);
  a.f[1].drop_sensor = true;
  FrameReport rep = a.run(8U);
  CHECK(rep.sensor_latched_mask == 0x02U && (rep.sensor_reason[1] & reason::kMissing) != 0U && rep.latched_mask == 0U && rep.reason[1] == 0U);
  Rig b(true);
  (void)b.run(20U);
  b.f[1].drop_cmd = true;
  rep = b.run(8U);
  CHECK(rep.latched_mask == 0x02U && (rep.reason[1] & reason::kMissing) != 0U && rep.sensor_latched_mask == 0U && rep.sensor_reason[1] == 0U);
}

TFC_TEST(split_on_a_frozen_imu_is_found_by_its_own_stuck_detector) {
  Rig r(true);
  (void)r.run(20U);
  r.f[2].freeze_sensor = true;
  bool stuck_seen = false;
  FrameReport rep;
  for (unsigned i = 0; i < 40U; ++i) {
    rep = r.step();
    stuck_seen = stuck_seen || (rep.sensor_reason[2] & reason::kStuck) != 0U;
  }
  CHECK(stuck_seen && rep.sensor_latched_mask == 0x04U && rep.latched_mask == 0U);
  CHECK(r.mgr.counters().stuck_flags > 0U);
  Rig off(false);  // the same fault without the split latches the computer
  (void)off.run(20U);
  off.f[2].freeze_sensor = true;
  CHECK(off.run(40U).latched_mask == 0x04U);
}

TFC_TEST(split_on_a_channel_is_reintegrated_by_a_command_through_probation_and_a_bad_one_is_refused) {
  Rig r(true);
  (void)r.run(20U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::RefusedNotLatched);  // IMU B is healthy
  r.f[1].gyro_bias = 3.0F;
  (void)r.run(10U);
  r.f[1].gyro_bias = 3.0F;
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::AlreadyDone);
  bool started = false;
  bool failed = false;
  for (unsigned i = 0; i < 120U; ++i) {  // the fault is still there: the probation starts after the dwell and is thrown back
    const FrameReport rep = r.step();
    started = started || (rep.sensor_probation_started & 0x02U) != 0U;
    failed = failed || (rep.sensor_probation_failed & 0x02U) != 0U;
  }
  CHECK(started && failed);
  r.f[1].gyro_bias = 0.0F;  // now it is healthy again
  (void)r.run(60U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  bool back = false;
  FrameReport rep;
  for (unsigned i = 0; i < 260U; ++i) {
    rep = r.step();
    back = back || (rep.sensor_newly_reintegrated & 0x02U) != 0U;
  }
  CHECK(back && rep.sensor_healthy == 3U && rep.sensor_latched_mask == 0U && rep.sensor_mode == Mode::Triplex);
  CHECK(r.mgr.sensor_counters().reintegrations == 1U && r.mgr.sensor_counters().probation_failures >= 1U && r.mgr.sensor_counters().latches == 1U);
}

TFC_TEST(split_on_repeated_latches_disable_an_imu_and_only_clear_disabled_brings_it_back) {
  Rig r(true);
  (void)r.run(20U);
  for (unsigned cycle = 0; cycle < 3U; ++cycle) {
    r.f[1].gyro_bias = 3.0F;
    (void)r.run(10U);
    r.f[1].gyro_bias = 0.0F;
    if (cycle < 2U) {
      CHECK(r.mgr.sensor_state(1) == NodeState::Latched && r.mgr.sensor_strikes(1) == cycle + 1U);
      (void)r.run(220U);  // the full dwell, for a repeat
      CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
      if (cycle == 1U) {
        (void)r.run(150U);
        CHECK(r.mgr.sensor_state(1) == NodeState::Probation);  // a repeat latch needs the long probation, not the 100 frames of the first
        (void)r.run(170U);
      } else {
        (void)r.run(320U);  // the probation (300 frames after a repeat)
      }
      CHECK(r.mgr.sensor_state(1) == NodeState::Healthy);
    }
  }
  const FrameReport rep = r.run(2U);
  CHECK(r.mgr.sensor_state(1) == NodeState::Disabled && r.mgr.sensor_strikes(1) == 3U && (rep.sensor_disabled_mask & 0x02U) != 0U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::RefusedDisabled);
  CHECK(r.mgr.command(GroundOp::Disable, 5U) == CommandResult::AlreadyDone);
  CHECK(r.mgr.command(GroundOp::ClearDisabled, 4U) == CommandResult::RefusedNotDisabled);
  CHECK(r.mgr.command(GroundOp::ClearDisabled, 5U) == CommandResult::Accepted && r.mgr.sensor_state(1) == NodeState::Latched && r.mgr.sensor_strikes(1) == 0U);
  CHECK(r.mgr.sensor_counters().disabled == 1U && r.mgr.sensor_counters().latches == 3U);
  CHECK(r.mgr.state(1) == NodeState::Healthy);  // the computer was never touched
}

TFC_TEST(split_on_commands_address_imus_by_node_4_to_6_and_the_arm_is_per_unit) {
  Rig r(true);
  (void)r.run(20U);
  Frame f = r.cmd(GroundOp::Disable, 5U);  // Triplex to Duplex for the IMUs is plain
  FrameReport rep = r.step(&f);
  CHECK(rep.command_count == 1U && rep.commands[0].result == CommandResult::Accepted && rep.commands[0].node == 5U && (rep.sensor_disabled_mask & 0x02U) != 0U);
  CHECK((rep.sensor_newly_disabled & 0x02U) != 0U && r.mgr.state(1) == NodeState::Healthy);
  f = r.cmd(GroundOp::Disable, 6U);  // Duplex to Simplex needs an ARM
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::RefusedNotArmed);
  f = r.cmd(GroundOp::Disable, 2U, true);  // an ARM for COMPUTER C does not cover IMU C (same low two bits)
  rep = r.step(&f);
  CHECK((rep.commands[0].flags & cmdflag::kArm) != 0U);
  f = r.cmd(GroundOp::Disable, 6U);
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::RefusedNotArmed);
  f = r.cmd(GroundOp::Disable, 6U, true);
  rep = r.step(&f);
  f = r.cmd(GroundOp::Disable, 6U);
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::Accepted && (rep.commands[0].flags & cmdflag::kArmed) != 0U && (rep.commands[0].flags & cmdflag::kCritical) == 0U);
  CHECK(rep.sensor_healthy == 1U && rep.sensor_mode == Mode::Simplex && rep.healthy == 3U);
  f = r.cmd(GroundOp::Disable, 4U, true);  // the last IMU: armed, and flagged as critical
  rep = r.step(&f);
  f = r.cmd(GroundOp::Disable, 4U);
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::Accepted && (rep.commands[0].flags & cmdflag::kCritical) != 0U && rep.sensor_healthy == 0U);
  f = r.cmd(GroundOp::Reintegrate, 7U);  // there is no IMU 3
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::RefusedBadNode);
  f = r.cmd(GroundOp::Disable, 3U);  // and 3 is neither a computer nor an IMU
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::RefusedBadNode);
  Rig off(false);  // without the split the same node numbers are refused outright
  (void)off.run(5U);
  f = off.cmd(GroundOp::Disable, 5U);
  rep = off.step(&f);
  CHECK(rep.commands[0].result == CommandResult::RefusedBadNode);
  CHECK(off.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::RefusedBadNode);
}

TFC_TEST(split_on_a_computer_on_probation_is_judged_on_its_commands_alone) {
  for (const bool split : {true, false}) {
    Rig r(split);
    (void)r.run(20U);
    r.f[1].cmd_offset = 0.05F;
    r.f[1].gyro_bias = 3.0F;
    (void)r.run(12U);
    r.f[1].cmd_offset = 0.0F;  // the computer is right again; its IMU is still bad
    (void)r.run(60U);
    CHECK(r.mgr.command(GroundOp::Reintegrate, 1U) == CommandResult::Accepted);
    (void)r.run(160U);
    if (split) {
      CHECK(r.mgr.state(1) == NodeState::Healthy && r.mgr.sensor_state(1) == NodeState::Latched);  // back as a voter, IMU still out
      CHECK(r.mgr.counters().reintegrations == 1U);
    } else {
      CHECK(r.mgr.state(1) != NodeState::Healthy);  // as a unit it fails every probation: the bad IMU disagrees with the vote
    }
  }
}

TFC_TEST(split_on_all_three_imus_lost_recover_by_judging_each_other) {
  Rig r(true);
  (void)r.run(20U);
  for (auto& f : r.f) {
    f.drop_sensor = true;
  }
  FrameReport rep = r.run(8U);
  CHECK(rep.sensor_latched_mask == 0x07U && rep.sensor_healthy == 0U && rep.sensor_mode == Mode::Safe);
  CHECK(rep.latched_mask == 0U && rep.healthy == 3U && rep.mode == Mode::Triplex);  // the computers are fine and still vote
  for (auto& f : r.f) {
    f.drop_sensor = false;
  }
  (void)r.run(60U);
  for (unsigned k = 4U; k <= 6U; ++k) {
    CHECK(r.mgr.command(GroundOp::Reintegrate, k) == CommandResult::Accepted);
  }
  bool all_back = false;
  for (unsigned i = 0; i < 260U; ++i) {  // nobody to compare with: the three on probation judge each other, and all three are clean
    rep = r.step();
    all_back = all_back || rep.sensor_healthy == 3U;
  }
  CHECK(all_back && rep.sensor_latched_mask == 0U);
  Rig d(true);  // two that disagree are neutral: neither readmitted nor thrown back
  (void)d.run(20U);
  for (auto& f : d.f) {
    f.drop_sensor = true;
  }
  (void)d.run(8U);
  d.f[0].drop_sensor = false;
  d.f[1].drop_sensor = false;
  d.f[1].gyro_bias = 3.0F;
  (void)d.run(60U);
  CHECK(d.mgr.command(GroundOp::Reintegrate, 4U) == CommandResult::Accepted && d.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  (void)d.run(200U);
  CHECK(d.mgr.sensor_state(0) == NodeState::Probation && d.mgr.sensor_state(1) == NodeState::Probation);
  d.f[1].gyro_bias = 0.0F;
  (void)d.run(160U);
  CHECK(d.mgr.sensor_state(0) == NodeState::Healthy && d.mgr.sensor_state(1) == NodeState::Healthy);
  Rig e(true);  // the odd one out of three is thrown back
  (void)e.run(20U);
  for (auto& f : e.f) {
    f.drop_sensor = true;
  }
  (void)e.run(8U);
  for (auto& f : e.f) {
    f.drop_sensor = false;
  }
  e.f[2].gyro_bias = 3.0F;
  (void)e.run(60U);
  for (unsigned k = 4U; k <= 6U; ++k) {
    (void)e.mgr.command(GroundOp::Reintegrate, k);
  }
  (void)e.run(160U);
  CHECK(e.mgr.sensor_state(0) == NodeState::Healthy && e.mgr.sensor_state(1) == NodeState::Healthy && e.mgr.sensor_state(2) != NodeState::Healthy);
}

TFC_TEST(split_on_two_imus_that_disagree_without_continuity_request_safe_and_hold_the_sensor_outputs) {
  Rig r(true);
  (void)r.run(20U);
  r.f[1].drop_sensor = true;  // IMU B gone: two IMUs left
  (void)r.run(8U);
  CHECK(r.mgr.sensor_state(1) == NodeState::Latched);
  r.f[0].gyro_bias = 0.9F;  // a slow drift on A that nothing can attribute (both IMUs stay within reach of the last agreed value)
  FrameReport rep;
  bool safe = false;
  for (unsigned i = 0; i < 40U; ++i) {
    r.f[0].gyro_bias += 0.05F;
    rep = r.step();
    safe = safe || rep.safe_request;
  }
  CHECK(safe && rep.safe_request && rep.held_mask == 0xFFU);  // the existing duplex rule, applied to the sensors
  CHECK(rep.latched_mask == 0U);                                // and no computer is touched
}

TFC_TEST(split_on_a_peer_that_has_not_booted_is_not_judged_until_it_has_been_seen) {
  Rig r(true, 30U);
  for (unsigned i = 0; i < 25U; ++i) {
    (void)r.step(nullptr, 0x03U);  // C is not there yet: neither its IMU nor its computer is judged
  }
  FrameReport rep = r.step(nullptr, 0x03U);
  CHECK(rep.sensor_latched_mask == 0U && rep.latched_mask == 0U && rep.sensor_reason[2] == 0U);
  (void)r.run(5U);  // C arrives
  for (unsigned i = 0; i < 12U; ++i) {
    rep = r.step(nullptr, 0x03U);  // and goes quiet again: now it is judged, for both
  }
  CHECK(rep.sensor_latched_mask == 0x04U && rep.latched_mask == 0x04U);
}

// One frame with a chosen damage, for the frame-damage test.
FrameReport damaged_frame(Rig& r, bool damage_gyro_b, bool wrong_cycle_cmd_c, bool wrong_cycle_gyro_b = false) {
  r.mgr.begin_frame(r.frame);
  const uint8_t seq = static_cast<uint8_t>(r.frame);
  const float t = static_cast<float>(r.frame);
  for (uint8_t n = 0; n < 3U; ++n) {
    Vec3 g;
    g.v = {3.0F * std::sin(0.2F * t), 2.0F * std::cos(0.15F * t), 0.5F * std::sin(0.1F * t)};
    Vec3 a;
    a.v = {0.1F * std::sin(0.13F * t), 0.1F * std::cos(0.11F * t), 1.0F};
    Frame gf = pack_gyro(n, g, (n == 1U && wrong_cycle_gyro_b) ? static_cast<uint8_t>(seq + 9U) : seq);
    if (n == 1U && damage_gyro_b) {
      gf.data[0] = static_cast<uint8_t>(gf.data[0] ^ 0x55U);
    }
    r.mgr.on_frame(gf);
    r.mgr.on_frame(pack_accel(n, a, seq));
    r.mgr.on_frame(pack_cmd(n, Command{0.1F * std::sin(0.2F * t), 0.05F * std::cos(0.15F * t), 0x4321U}, (n == 2U && wrong_cycle_cmd_c) ? static_cast<uint8_t>(seq + 7U) : seq));
  }
  const FrameReport rep = r.mgr.end_frame();
  ++r.frame;
  return rep;
}

TFC_TEST(split_on_the_two_sides_have_their_own_frame_damage) {
  Rig a(true);
  (void)a.run(20U);
  FrameReport rep;
  for (unsigned i = 0; i < 6U; ++i) {
    rep = damaged_frame(a, true, false);  // a corrupted gyro frame of B is IMU B's CRC fault
  }
  CHECK(rep.sensor_latched_mask == 0x02U && (rep.sensor_reason[1] & reason::kCrc) != 0U && rep.latched_mask == 0U && rep.reason[1] == 0U);
  Rig c(true);
  (void)c.run(20U);
  for (unsigned i = 0; i < 6U; ++i) {
    rep = damaged_frame(c, false, false, true);  // a gyro frame of B in the wrong cycle is IMU B's sequence fault
  }
  CHECK(rep.sensor_latched_mask == 0x02U && (rep.sensor_reason[1] & reason::kSeq) != 0U && rep.latched_mask == 0U);
  Rig b(true);
  (void)b.run(20U);
  for (unsigned i = 0; i < 6U; ++i) {
    rep = damaged_frame(b, false, true);  // a command frame of C in the wrong cycle is computer C's sequence fault
  }
  CHECK(rep.latched_mask == 0x04U && (rep.reason[2] & reason::kSeq) != 0U && rep.sensor_latched_mask == 0U && rep.sensor_reason[2] == 0U);
}

TFC_TEST(split_on_a_sensor_channel_that_is_bad_one_frame_in_three_is_found_by_the_leaky_count) {
  Rig r(true);
  (void)r.run(20U);
  FrameReport at_latch;
  bool latched = false;
  for (unsigned i = 0; i < 60U; ++i) {
    r.f[2].drop_sensor = (i % 3U) == 0U;  // one frame in three: never three bad in five
    const FrameReport rep = r.step();
    if ((rep.sensor_newly_latched & 0x04U) != 0U) {
      at_latch = rep;
      latched = true;
      break;
    }
  }
  CHECK(latched && (at_latch.sensor_reason[2] & reason::kIntermittent) != 0U && at_latch.latched_mask == 0U);
  CHECK(r.mgr.sensor_strikes(2) == 1U);
  Rig clean(true);  // and a single glitch is never a latch
  (void)clean.run(20U);
  clean.f[1].drop_sensor = true;
  (void)clean.step();
  clean.f[1].drop_sensor = false;
  CHECK(clean.run(40U).sensor_latched_mask == 0U);
}

TFC_TEST(split_on_a_probation_is_neutral_without_a_reference_and_dirty_when_the_channel_stops_delivering) {
  // one channel on probation, the others latched and silent: no cohort, no healthy channel: neutral for ever
  Rig a(true);
  (void)a.run(20U);
  for (auto& f : a.f) {
    f.drop_sensor = true;
  }
  (void)a.run(8U);
  a.f[0].drop_sensor = false;
  (void)a.run(60U);
  CHECK(a.mgr.command(GroundOp::Reintegrate, 4U) == CommandResult::Accepted);
  (void)a.run(300U);
  CHECK(a.mgr.sensor_state(0) == NodeState::Probation);  // not readmitted (nothing to compare with), not thrown back
  // a channel on probation that stops delivering is thrown back
  Rig b(true);
  (void)b.run(20U);
  b.f[1].gyro_bias = 3.0F;
  (void)b.run(10U);
  b.f[1].gyro_bias = 0.0F;
  (void)b.run(60U);
  CHECK(b.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  (void)b.run(5U);
  CHECK(b.mgr.sensor_state(1) == NodeState::Probation);
  b.f[1].drop_sensor = true;
  (void)b.run(2U);
  CHECK(b.mgr.sensor_state(1) == NodeState::Latched);
  // while Safe is requested the voted outputs are held and cannot be a reference: a probation neither advances nor fails
  Rig c(true);
  (void)c.run(20U);
  c.f[1].drop_sensor = true;
  (void)c.run(8U);
  c.f[1].drop_sensor = false;
  c.f[0].gyro_bias = 0.9F;
  for (unsigned i = 0; i < 40U; ++i) {
    c.f[0].gyro_bias += 0.05F;
    (void)c.step();
  }
  CHECK(c.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  c.f[1].gyro_bias = 4.0F;  // B delivers again, and is wrong
  FrameReport rep;
  for (unsigned i = 0; i < 120U; ++i) {
    rep = c.step();
  }
  CHECK(rep.safe_request && c.mgr.sensor_state(1) == NodeState::Probation);  // Safe holds everything: no verdict either way
  c.mgr.clear_safe_request();
  c.f[0].gyro_bias = 0.0F;
  for (unsigned i = 0; i < 30U; ++i) {
    rep = c.step();
  }
  CHECK(!rep.safe_request && c.mgr.sensor_state(1) == NodeState::Latched);  // the reference is back: B is wrong, B is thrown back
}

TFC_TEST(split_on_a_physical_cause_disables_an_imu_at_its_second_latch_and_a_vote_cause_only_at_its_third) {
  Rig r(true);
  (void)r.run(20U);
  for (unsigned cycle = 0; cycle < 2U; ++cycle) {  // bad one frame in three: the leaky count latches it, and that cause is physical
    for (unsigned i = 0; i < 60U && r.mgr.sensor_state(2) == NodeState::Healthy; ++i) {
      r.f[2].drop_sensor = (i % 3U) == 0U;
      (void)r.step();
    }
    r.f[2].drop_sensor = false;
    if (cycle == 0U) {
      CHECK(r.mgr.sensor_state(2) == NodeState::Latched && r.mgr.sensor_strikes(2) == 1U);
      (void)r.run(260U);
      CHECK(r.mgr.command(GroundOp::Reintegrate, 6U) == CommandResult::Accepted);
      (void)r.run(160U);
      CHECK(r.mgr.sensor_state(2) == NodeState::Healthy);
    }
  }
  CHECK(r.mgr.sensor_state(2) == NodeState::Disabled && r.mgr.sensor_strikes(2) == 2U);  // the second latch of a physical cause
}

TFC_TEST(split_on_the_sensor_object_refuses_an_impossible_channel) {
  SensorHealth sh{RedundancyConfig{}};
  CHECK(sh.reintegrate(3U) == CommandResult::RefusedBadNode && sh.disable(3U) == CommandResult::RefusedBadNode && sh.clear_disabled(7U) == CommandResult::RefusedBadNode);
  CHECK(sh.state(3U) == NodeState::Disabled && !sh.healthy(3U) && !sh.seen(3U) && sh.strikes(3U) == 0U && sh.count_healthy() == 3U);
}

// A configuration in which a channel recovers in a few frames and is never disabled, to drive the counters to their limits.
RedundancyConfig quick_config(ReintegrationPolicy policy = ReintegrationPolicy::Manual) {
  RedundancyConfig c;
  c.sensor_split = true;
  c.min_dwell_frames = 1U;
  c.min_dwell_frames_transient = 1U;
  c.probation_frames = 1U;
  c.probation_frames_repeat = 1U;
  c.max_strikes = 0U;
  c.max_strikes_physical = 0U;
  c.policy = policy;
  c.auto_max_attempts = 255U;
  return c;
}

TFC_TEST(split_on_the_strike_attempt_and_dwell_counters_saturate_and_a_limit_of_zero_never_disables) {
  Rig r(true);
  r.mgr = RedundancyManager(quick_config());
  (void)r.run(20U);
  for (unsigned i = 0; i < 262U; ++i) {  // 262 latches: the strike counter stops at 255
    r.f[1].gyro_bias = 3.0F;
    (void)r.run(6U);
    r.f[1].gyro_bias = 0.0F;
    (void)r.run(3U);
    (void)r.mgr.command(GroundOp::Reintegrate, 5U);
    (void)r.run(4U);
  }
  CHECK(r.mgr.sensor_strikes(1) == 255U && r.mgr.sensor_state(1) != NodeState::Disabled);
  // a channel that is wrong on every probation: the attempt counter stops at 255 as well
  r.f[1].gyro_bias = 3.0F;
  (void)r.run(6U);
  for (unsigned i = 0; i < 262U; ++i) {
    (void)r.mgr.command(GroundOp::Reintegrate, 5U);
    (void)r.run(3U);
  }
  CHECK(r.mgr.sensor_state(1) != NodeState::Healthy);
  // a latched channel nobody asks to bring back: its dwell counter stops at its limit
  (void)r.run(70000U);
  CHECK(r.mgr.sensor_state(1) == NodeState::Latched);
}

TFC_TEST(split_on_the_automatic_policy_brings_back_a_transient_imu_fault_a_limited_number_of_times) {
  RedundancyConfig c;
  c.sensor_split = true;
  c.policy = ReintegrationPolicy::AutoTransient;
  c.auto_max_attempts = 2U;
  Rig r(true);
  r.mgr = RedundancyManager(c);
  (void)r.run(20U);
  r.f[2].gyro_bias = 3.0F;
  (void)r.run(10U);
  r.f[2].gyro_bias = 0.0F;
  bool back = false;
  for (unsigned i = 0; i < 260U; ++i) {  // a first, transient-looking latch: the short dwell, then probation without a command
    back = back || (r.step().sensor_newly_reintegrated & 0x04U) != 0U;
  }
  CHECK(back && r.mgr.sensor_state(2) == NodeState::Healthy);
  // a fault that is still there on every probation: after two failed attempts the automatic policy gives up
  Rig q(true);
  q.mgr = RedundancyManager(c);
  (void)q.run(20U);
  q.f[2].gyro_bias = 3.0F;
  unsigned failures = 0U;
  for (unsigned i = 0; i < 600U; ++i) {
    failures += (q.step().sensor_probation_failed & 0x04U) != 0U ? 1U : 0U;
  }
  CHECK(failures == 2U && q.mgr.sensor_state(2) == NodeState::Latched);
  // a cause that is not transient-looking (a fault bad one frame in three, found by the leaky count) is never brought back automatically
  Rig p(true);
  p.mgr = RedundancyManager(c);
  (void)p.run(20U);
  for (unsigned i = 0; i < 60U; ++i) {
    p.f[0].drop_sensor = (i % 3U) == 0U;
    (void)p.step();
  }
  p.f[0].drop_sensor = false;
  (void)p.run(400U);
  CHECK(p.mgr.sensor_state(0) == NodeState::Latched);
}

TFC_TEST(split_on_one_probation_at_a_time_unless_no_channel_is_healthy_and_a_cohort_member_must_deliver) {
  Rig r(true);
  (void)r.run(20U);
  r.f[0].gyro_bias = 3.0F;
  (void)r.run(8U);
  r.f[0].gyro_bias = 0.0F;
  r.f[1].gyro_bias = 3.0F;  // a second channel fails while the first is still out (Duplex becomes Simplex for the sensors)
  (void)r.run(8U);
  r.f[1].gyro_bias = 0.0F;
  (void)r.run(80U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 4U) == CommandResult::Accepted && r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  (void)r.run(20U);
  const unsigned on_probation = (r.mgr.sensor_state(0) == NodeState::Probation ? 1U : 0U) + (r.mgr.sensor_state(1) == NodeState::Probation ? 1U : 0U);
  CHECK(on_probation <= 1U);  // never two at once while a channel is healthy
  CHECK(r.mgr.sensor_state(0) == NodeState::Probation || r.mgr.sensor_state(1) == NodeState::Probation);
  (void)r.run(400U);
  CHECK(r.mgr.sensor_state(0) == NodeState::Healthy && r.mgr.sensor_state(1) == NodeState::Healthy);
  // a probation during which the second probationer is silent or frozen: the cohort has too few members, or throws the frozen one back
  Rig q(true);
  (void)q.run(20U);
  for (auto& f : q.f) {
    f.drop_sensor = true;
  }
  (void)q.run(8U);
  for (auto& f : q.f) {
    f.drop_sensor = false;
  }
  q.f[2].freeze_sensor = true;
  (void)q.run(60U);
  for (unsigned k = 4U; k <= 6U; ++k) {
    (void)q.mgr.command(GroundOp::Reintegrate, k);
  }
  (void)q.run(300U);
  CHECK(q.mgr.sensor_state(2) != NodeState::Healthy);  // the frozen one was found, whichever way: stuck on probation, or odd one out
  CHECK(q.mgr.sensor_state(0) == NodeState::Healthy && q.mgr.sensor_state(1) == NodeState::Healthy);
  CHECK(q.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::RefusedNotLatched);
}

TFC_TEST(split_on_a_channel_with_only_one_of_its_two_frames_is_not_good_and_an_early_reintegrate_on_probation_is_already_done) {
  Rig r(true);
  (void)r.run(20U);
  r.f[1].gyro_bias = 3.0F;
  (void)r.run(10U);
  r.f[1].gyro_bias = 0.0F;
  (void)r.run(60U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::Accepted);
  (void)r.run(3U);
  CHECK(r.mgr.sensor_state(1) == NodeState::Probation);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 5U) == CommandResult::AlreadyDone);  // on probation: nothing to ask
  // a channel whose accelerometer frame alone is missing does not count as delivered
  Rig q(true);
  (void)q.run(20U);
  for (unsigned i = 0; i < 8U; ++i) {
    q.mgr.begin_frame(q.frame);
    const uint8_t seq = static_cast<uint8_t>(q.frame);
    const float t = static_cast<float>(q.frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 g;
      g.v = {3.0F * std::sin(0.2F * t), 2.0F * std::cos(0.15F * t), 0.5F * std::sin(0.1F * t)};
      Vec3 a;
      a.v = {0.1F * std::sin(0.13F * t), 0.1F * std::cos(0.11F * t), 1.0F};
      q.mgr.on_frame(pack_gyro(n, g, seq));
      if (n != 2U) {
        q.mgr.on_frame(pack_accel(n, a, seq));
      }
      q.mgr.on_frame(pack_cmd(n, Command{0.1F * std::sin(0.2F * t), 0.05F * std::cos(0.15F * t), 0x4321U}, seq));
    }
    const FrameReport rep = q.mgr.end_frame();
    ++q.frame;
    if ((rep.sensor_newly_latched & 0x04U) != 0U) {
      CHECK((rep.sensor_reason[2] & reason::kMissing) != 0U);  // the missing accelerometer frame is the reason, not only the zeros that the vote then saw
    }
  }
  CHECK(q.mgr.sensor_state(2) == NodeState::Latched && q.mgr.state(2) == NodeState::Healthy);
}

TFC_TEST(split_on_a_computer_probation_goes_on_while_the_sensor_outputs_are_held) {
  Rig r(true);
  (void)r.run(20U);
  r.f[2].cmd_offset = 0.05F;
  (void)r.run(10U);
  r.f[2].cmd_offset = 0.0F;
  for (auto& f : r.f) {
    f.drop_sensor = true;  // every IMU gone: the six sensor outputs are held, the commands are not
  }
  (void)r.run(60U);
  CHECK(r.mgr.command(GroundOp::Reintegrate, 2U) == CommandResult::Accepted);
  (void)r.run(160U);
  const FrameReport rep = r.step();
  CHECK(rep.held_mask == 0x3FU && !rep.safe_request);
  CHECK(r.mgr.state(2) == NodeState::Healthy && rep.sensor_healthy == 0U);  // the computer was readmitted on its commands alone
}

}  // namespace
