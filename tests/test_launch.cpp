// SPDX-License-Identifier: MIT
// The launch commands and the go/no-go (docs/design/LAUNCH_SEQUENCE.md): `launch` needs an ARM, `scrub` does not, both pass the same tag, counter and window checks as every ground command and are reported to the
// firmware as events; the heartbeat carries a ready bit; the launch gate lists what is not ready.
#include <array>
#include <cstdint>
#include <string>

#include "tfc/act_ground.hpp"
#include "tfc/launch_gate.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;

// A manager with all three nodes present and healthy.
struct Bench {
  RedundancyManager mgr;
  uint32_t frame = 0U;
  uint8_t counter = 0U;
  Bench() : mgr(RedundancyConfig{}) {}

  FrameReport run_frame(const Frame* ground = nullptr) {
    mgr.begin_frame(frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 g;
      Vec3 a;
      a.v = {0.0F, 0.0F, 1.0F};
      mgr.on_frame(pack_gyro(n, g, static_cast<uint8_t>(frame)));
      mgr.on_frame(pack_accel(n, a, static_cast<uint8_t>(frame)));
      mgr.on_frame(pack_cmd(n, Command{}, static_cast<uint8_t>(frame)));
    }
    if (ground != nullptr) {
      mgr.on_frame(*ground);
    }
    const FrameReport& r = mgr.end_frame();
    ++frame;
    return r;
  }
  Frame cmd(GroundOp op, bool arm = false) { return pack_ground_auth(op, 0U, ++counter, kBenchKey, arm); }
};

}  // namespace

TFC_TEST(launch_needs_an_arm_scrub_does_not_and_both_are_reported_as_events) {
  Bench b;
  for (int i = 0; i < 5; ++i) {
    (void)b.run_frame();
  }
  Frame f = b.cmd(GroundOp::Launch);
  FrameReport r = b.run_frame(&f);
  CHECK(r.command_count == 1U && r.commands[0].op == static_cast<uint8_t>(GroundOp::Launch) && r.commands[0].result == CommandResult::RefusedNotArmed);
  f = b.cmd(GroundOp::Launch, true);  // ARM
  r = b.run_frame(&f);
  CHECK(r.command_count == 1U && r.commands[0].result == CommandResult::Accepted && (r.commands[0].flags & cmdflag::kArm) != 0U);
  f = b.cmd(GroundOp::Launch);  // EXECUTE
  r = b.run_frame(&f);
  CHECK(r.command_count == 1U && r.commands[0].op == static_cast<uint8_t>(GroundOp::Launch) && r.commands[0].result == CommandResult::Accepted &&
        (r.commands[0].flags & cmdflag::kArmed) != 0U);
  f = b.cmd(GroundOp::Launch);  // a second EXECUTE with no ARM: refused (one ARM covers one EXECUTE)
  r = b.run_frame(&f);
  CHECK(r.commands[0].result == CommandResult::RefusedNotArmed);
  f = b.cmd(GroundOp::Scrub);  // plain
  r = b.run_frame(&f);
  CHECK(r.command_count == 1U && r.commands[0].op == static_cast<uint8_t>(GroundOp::Scrub) && r.commands[0].result == CommandResult::Accepted);
  CHECK(std::string(op_text(static_cast<uint8_t>(GroundOp::Launch))) == "launch" && std::string(op_text(static_cast<uint8_t>(GroundOp::Scrub))) == "scrub");
}

TFC_TEST(launch_and_scrub_pass_the_same_authentication_and_replay_checks) {
  Bench b;
  for (int i = 0; i < 3; ++i) {
    (void)b.run_frame();
  }
  Frame forged = pack_ground(GroundOp::Scrub, 0U, 1U);  // no tag
  FrameReport r = b.run_frame(&forged);
  CHECK(r.command_count == 0U && b.mgr.counters().commands_unauthentic == 1U);
  Frame good = b.cmd(GroundOp::Scrub);
  r = b.run_frame(&good);
  CHECK(r.command_count == 1U);
  r = b.run_frame(&good);  // replayed
  CHECK(r.command_count == 0U && b.mgr.counters().commands_replayed == 1U);
  Frame bad_op = pack_ground_auth(static_cast<GroundOp>(7), 0U, ++b.counter, kBenchKey);  // an operation that does not exist yet
  r = b.run_frame(&bad_op);
  CHECK(r.command_count == 1U && r.commands[0].result == CommandResult::RefusedBadOp);
}

TFC_TEST(launch_acts_ground_path_leaves_launch_and_scrub_to_the_flight_computers) {
  ActGround g;
  CHECK(g.on_frame(pack_ground_auth(GroundOp::Launch, 0U, 1U, kBenchKey, true)) == ActGroundResult::NotForAct);
  CHECK(g.on_frame(pack_ground_auth(GroundOp::Scrub, 0U, 2U, kBenchKey)) == ActGroundResult::NotForAct);
}

TFC_TEST(launch_the_heartbeat_carries_the_ready_bit_and_the_other_bits_are_untouched) {
  Heartbeat h;
  h.mode = 3U;
  h.safe_requested = true;
  h.role = 1U;
  h.quarantined = true;
  h.ready = true;
  const Frame f = pack_heartbeat(1U, h, 5U);
  CHECK((f.data[1] & 0x80U) != 0U);
  const DecodedHeartbeat d = unpack_heartbeat(f);
  CHECK(d.ok && d.hb.ready && d.hb.quarantined && d.hb.safe_requested && d.hb.mode == 3U && d.hb.role == 1U);
  Heartbeat g;  // pinned with the Python mirror
  g.mode = 3U;
  g.ready = true;
  g.reset_count = 5U;
  g.release_hash = 0xBEEFU;
  const Frame gf = pack_heartbeat(1U, g, 4U);
  const std::array<uint8_t, 8> want{0x02U, 0x83U, 0x00U, 0x05U, 0xEFU, 0xBEU, 0x04U, 0x8CU};
  CHECK(gf.data == want);
  h.ready = false;
  const DecodedHeartbeat e = unpack_heartbeat(pack_heartbeat(1U, h, 5U));
  CHECK(e.ok && !e.hb.ready && e.hb.quarantined);
}

TFC_TEST(launch_gate_lists_everything_that_is_not_ready_and_says_go_only_when_all_hold) {
  LaunchFacts f;
  f.healthy_nodes = 0x07U;
  f.ready_nodes = 0x07U;
  f.act_nominal = true;
  CHECK(launch_check(f) == 0U && std::string(nogo_text(0U)) == "go");
  LaunchFacts x = f;
  x.healthy_nodes = 0x05U;
  CHECK(launch_check(x) == nogo::kNotTriplex);
  x = f;
  x.safe_requested = true;
  CHECK(launch_check(x) == nogo::kSafeRequested);
  x = f;
  x.ready_nodes = 0x03U;
  CHECK(launch_check(x) == nogo::kNodeNotReady);
  x = f;
  x.act_nominal = false;
  CHECK(launch_check(x) == nogo::kActNotNominal);
  x = LaunchFacts{};
  CHECK(launch_check(x) == (nogo::kNotTriplex | nogo::kNodeNotReady | nogo::kActNotNominal));
  CHECK(std::string(nogo_text(nogo::kNotTriplex | nogo::kActNotNominal)) == "not three healthy flight computers");
  CHECK(std::string(nogo_text(nogo::kSafeRequested)).find("Safe") != std::string::npos);
  CHECK(std::string(nogo_text(nogo::kNodeNotReady)).find("not ready") != std::string::npos);
  CHECK(std::string(nogo_text(nogo::kActNotNominal)).find("ACT") != std::string::npos);
  f.healthy_nodes = 0xF7U;  // bits beyond the three nodes are ignored
  CHECK(launch_check(f) == 0U);
}
