// SPDX-License-Identifier: MIT
// Computers of different releases (ADR-021, TFC-ARCH-004, ARCH-007, ARCH-008): the manager learns each computer's release from its heartbeat; when two share one and the third does not, the lone
// computer's commands are held to the version tolerance, its digest is not compared with the pair's, and a disagreement beyond the version tolerance isolates nobody: the outputs are held and Safe
// is requested. Without release information, or with release_aware off, the manager is exactly what it was.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;

struct Rig {
  RedundancyManager mgr;
  std::array<float, 3> cmd_offset{};
  std::array<uint16_t, 3> digest{{0x1111U, 0x1111U, 0x1111U}};
  uint32_t frame = 0U;
  uint8_t counter = 0U;

  explicit Rig(const RedundancyConfig& cfg = RedundancyConfig{}) : mgr(cfg) {}

  FrameReport step(const Frame* extra = nullptr) {
    mgr.begin_frame(frame);
    const float t = static_cast<float>(frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      const uint8_t seq = static_cast<uint8_t>(frame);
      Vec3 g;
      g.v = {3.0F * std::sin(0.2F * t), 2.0F * std::cos(0.15F * t), 0.5F * std::sin(0.1F * t)};
      Vec3 a;
      a.v = {0.1F * std::sin(0.13F * t), 0.1F * std::cos(0.11F * t), 1.0F};
      mgr.on_frame(pack_gyro(n, g, seq));
      mgr.on_frame(pack_accel(n, a, seq));
      mgr.on_frame(pack_cmd(n, Command{(0.3F * std::sin(0.2F * t)) + cmd_offset[n], 0.05F * std::cos(0.15F * t), digest[n]}, seq));
    }
    if (extra != nullptr) {
      mgr.on_frame(*extra);
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

  Frame cmd(GroundOp op, unsigned node, bool arm = false) { return pack_ground_auth(op, static_cast<uint8_t>(node), ++counter, kBenchKey, arm); }
};

void two_releases(Rig& r, unsigned lone) {  // the computer `lone` runs release 2, the other two release 1
  for (unsigned n = 0; n < 3U; ++n) {
    r.mgr.set_release(n, n == lone ? 2U : 1U);
  }
}

TFC_TEST(release_the_heartbeat_names_the_release_and_zero_means_not_reported) {
  Rig r;
  CHECK(r.mgr.release_of(0) == 0U && r.mgr.release_of(5) == 0U);
  Heartbeat hb;
  hb.release_hash = 0xBEEFU;
  Frame f = pack_heartbeat(1U, hb, 0U);
  (void)r.step(&f);
  CHECK(r.mgr.release_of(1) == 0xBEEFU && r.mgr.release_of(0) == 0U);
  CHECK(r.mgr.counters().out_of_schedule == 0U);  // a heartbeat is in the schedule
  Frame bad = f;
  bad.data[3] = static_cast<uint8_t>(bad.data[3] ^ 0x40U);  // damaged: not decoded
  hb.release_hash = 0x1234U;
  (void)r.step(&bad);
  CHECK(r.mgr.release_of(1) == 0xBEEFU);
  r.mgr.set_release(2U, 7U);
  r.mgr.set_release(3U, 9U);  // no such computer
  CHECK(r.mgr.release_of(2) == 7U && r.mgr.release_of(3) == 0U);
}

TFC_TEST(release_without_information_or_with_one_release_the_manager_is_what_it_was) {
  Rig a;  // nothing reported: a computer whose commands are off is the odd one out and is latched, as always
  (void)a.run(20U);
  a.cmd_offset[2] = 0.05F;
  FrameReport rep = a.run(10U);
  CHECK(rep.latched_mask == 0x04U && !rep.release_split && a.mgr.counters().release_split_frames == 0U);
  Rig b;  // all three on one release: the same
  for (unsigned n = 0; n < 3U; ++n) {
    b.mgr.set_release(n, 5U);
  }
  (void)b.run(20U);
  b.cmd_offset[1] = 0.05F;
  rep = b.run(10U);
  CHECK(rep.latched_mask == 0x02U && !rep.safe_request);
  Rig c;  // two reported, one not: not enough to say
  c.mgr.set_release(0U, 1U);
  c.mgr.set_release(1U, 1U);
  (void)c.run(20U);
  c.cmd_offset[2] = 0.05F;
  CHECK(c.run(10U).latched_mask == 0x04U);
  Rig d;  // three different releases: no pair
  d.mgr.set_release(0U, 1U);
  d.mgr.set_release(1U, 2U);
  d.mgr.set_release(2U, 3U);
  (void)d.run(20U);
  d.cmd_offset[2] = 0.05F;
  CHECK(d.run(10U).latched_mask == 0x04U);
  RedundancyConfig off;
  off.release_aware = false;
  Rig e(off);  // release_aware off, however different
  two_releases(e, 2U);
  (void)e.run(20U);
  e.cmd_offset[2] = 0.05F;
  CHECK(e.run(10U).latched_mask == 0x04U);
}

TFC_TEST(release_a_difference_beyond_the_version_tolerance_isolates_nobody_and_requests_safe) {
  for (unsigned lone = 0; lone < 3U; ++lone) {
    Rig r;
    two_releases(r, lone);
    (void)r.run(20U);
    r.cmd_offset[lone] = 0.05F;  // 5 tolerances: beyond 1.5
    bool split = false;
    bool safe = false;
    FrameReport rep;
    for (unsigned i = 0; i < 12U; ++i) {
      rep = r.step();
      split = split || rep.release_split;
      safe = safe || rep.safe_request;
    }
    CHECK(split && safe && rep.safe_request && rep.held_mask == 0xFFU);
    CHECK(rep.latched_mask == 0U && rep.healthy == 3U && rep.newly_latched == 0U);  // nobody isolated
    CHECK(r.mgr.counters().release_split_frames >= 10U && r.mgr.counters().vote_disagreements == 0U);  // and nobody blamed
    CHECK(rep.reason[lone] == 0U);
  }
}

TFC_TEST(release_the_version_tolerance_is_a_factor_of_the_vote_tolerance_and_its_edge_is_exact) {
  Rig a;  // 0.012 degree off: beyond the vote tolerance (0.01), inside 1.5 times it (0.015): a different release, not a fault, not a conflict
  two_releases(a, 2U);
  (void)a.run(20U);
  a.cmd_offset[2] = 0.012F;
  bool split = false;
  FrameReport rep;
  for (unsigned i = 0; i < 30U; ++i) {
    rep = a.step();
    split = split || rep.release_split;
  }
  CHECK(!split && !rep.safe_request && rep.latched_mask == 0U && a.mgr.counters().vote_disagreements == 0U);
  Rig b;  // 0.018: beyond
  two_releases(b, 2U);
  (void)b.run(20U);
  b.cmd_offset[2] = 0.018F;
  split = false;
  for (unsigned i = 0; i < 30U; ++i) {
    split = split || b.step().release_split;
  }
  CHECK(split);
  RedundancyConfig wide;
  wide.version_tol_factor = 3.0F;  // 0.018 is inside 3 x 0.01
  Rig c(wide);
  two_releases(c, 2U);
  (void)c.run(20U);
  c.cmd_offset[2] = 0.018F;
  split = false;
  for (unsigned i = 0; i < 30U; ++i) {
    split = split || c.step().release_split;
  }
  CHECK(!split);
  c.cmd_offset[2] = 0.035F;
  split = false;
  for (unsigned i = 0; i < 30U; ++i) {
    split = split || c.step().release_split;
  }
  CHECK(split);
}

TFC_TEST(release_a_computer_of_the_pair_that_is_wrong_is_still_blamed_in_the_ordinary_way) {
  Rig r;
  two_releases(r, 2U);
  (void)r.run(20U);
  r.cmd_offset[0] = 0.05F;  // A, of the pair, is off: the lone computer is not the odd one out, A is
  const FrameReport rep = r.run(10U);
  CHECK(rep.latched_mask == 0x01U && !rep.release_split);
  Rig q;  // the lone computer is off while ANOTHER is also off the other way: the vote has no lone odd one out
  two_releases(q, 2U);
  (void)q.run(20U);
  q.cmd_offset[2] = 0.05F;
  q.cmd_offset[0] = -0.05F;
  (void)q.run(10U);
  CHECK(q.mgr.counters().release_split_frames == 0U);  // three different values: no majority, not a release conflict
}

TFC_TEST(release_a_fault_common_to_the_pair_no_longer_isolates_the_healthy_computer_of_the_other_release) {
  Rig a;  // F63 without release information: the healthy C is the odd one out and is latched
  (void)a.run(20U);
  a.cmd_offset[0] = 0.05F;
  a.cmd_offset[1] = 0.05F;
  CHECK(a.run(12U).latched_mask == 0x04U);
  Rig b;  // with it: nobody is isolated, the outputs are held, Safe is requested
  two_releases(b, 2U);
  (void)b.run(20U);
  b.cmd_offset[0] = 0.05F;
  b.cmd_offset[1] = 0.05F;
  const FrameReport rep = b.run(12U);
  CHECK(rep.latched_mask == 0U && rep.safe_request && rep.held_mask == 0xFFU);
}

TFC_TEST(release_the_operator_resolves_it_by_naming_the_side_to_trust) {
  Rig r;
  two_releases(r, 2U);
  (void)r.run(20U);
  r.cmd_offset[2] = 0.05F;
  (void)r.run(12U);
  CHECK(r.step().safe_request);
  // trust the pair: disable the other release's computer (Triplex to Duplex is plain), then clear Safe under an ARM
  Frame f = r.cmd(GroundOp::Disable, 2U);
  FrameReport rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::Accepted && rep.disabled_mask == 0x04U && rep.mode == Mode::Safe);
  f = r.cmd(GroundOp::ClearSafe, 0U, true);
  rep = r.step(&f);
  f = r.cmd(GroundOp::ClearSafe, 0U);
  rep = r.step(&f);
  CHECK(rep.commands[0].result == CommandResult::Accepted);
  rep = r.run(30U);
  CHECK(!rep.safe_request && rep.mode == Mode::Duplex && rep.healthy == 2U && !rep.release_split);  // two voting: no family to compare, and C is out
  CHECK(r.mgr.counters().release_split_frames >= 10U);
}

TFC_TEST(release_the_digest_of_the_lone_computer_is_not_compared_with_the_pairs) {
  Rig a;  // without families a different digest blames the odd one
  (void)a.run(20U);
  a.digest[2] = 0x2222U;
  CHECK(a.run(10U).latched_mask == 0x04U);
  Rig b;  // with families: the digests of different releases differ by design
  two_releases(b, 2U);
  (void)b.run(20U);
  b.digest[2] = 0x2222U;
  FrameReport rep = b.run(30U);
  CHECK(rep.latched_mask == 0U && b.mgr.counters().digest_flags == 0U && !rep.safe_request);
  Rig c;  // but the pair's own digests are still compared with each other
  two_releases(c, 2U);
  (void)c.run(20U);
  c.digest[0] = 0x3333U;
  rep = c.run(10U);
  CHECK(b.mgr.counters().digest_flags == 0U && c.mgr.counters().digest_flags > 0U && rep.safe_request);  // two comparable computers that differ cannot be told apart: unresolved
}

TFC_TEST(release_the_family_needs_all_three_voting_so_with_one_out_the_odd_one_is_blamed_by_continuity_and_a_set_release_outside_the_nodes_is_ignored) {
  Rig d;  // A disabled: B and C are left, of different releases. The release rule needs all three voting, so C, which steps away from its own earlier values, is latched as always
  two_releases(d, 2U);
  (void)d.run(20U);
  Frame f = d.cmd(GroundOp::Disable, 0U);
  (void)d.step(&f);
  d.cmd_offset[2] = 0.05F;
  const uint32_t before = d.mgr.counters().vote_disagreements;
  (void)d.step();  // the first frames after the step: continuity singles out C, which is blamed (the vote's disagreement is counted), although it is the lone release
  CHECK(d.mgr.counters().vote_disagreements > before);
  CHECK(!d.run(10U).release_split);
  RedundancyManager m;  // a node number outside the three is not a place to write
  m.set_release(0U, 7U);
  m.set_release(kNodes, 9U);
  m.set_release(200U, 9U);
  CHECK(m.release_of(0U) == 7U && m.release_of(1U) == 0U && m.release_of(2U) == 0U && m.release_of(kNodes) == 0U);
  m.begin_frame(0U);
  m.set_release(kNodes, 0xFFFFU);  // the next member of the manager is the "split now" flag
  CHECK(!m.end_frame().release_split);
}

TFC_TEST(release_the_split_is_judged_only_while_all_three_vote_and_with_the_sensor_split_too) {
  Rig r;
  two_releases(r, 2U);
  (void)r.run(20U);
  Frame f = r.cmd(GroundOp::Disable, 0U);
  (void)r.step(&f);  // A disabled: two computers left, of different releases: Duplex rules, no family
  r.cmd_offset[2] = 0.05F;
  const FrameReport rep = r.run(10U);
  CHECK(!rep.release_split && r.mgr.counters().release_split_frames == 0U);
  RedundancyConfig split;
  split.sensor_split = true;
  Rig s(split);
  two_releases(s, 1U);
  (void)s.run(20U);
  s.cmd_offset[1] = 0.05F;
  const FrameReport rs = s.run(12U);
  CHECK(rs.release_split && rs.safe_request && rs.latched_mask == 0U && rs.sensor_latched_mask == 0U);
}

TFC_TEST(release_the_version_tolerance_factor_is_validated_and_part_of_the_configuration_digest) {
  RedundancyConfig c;
  c.version_tol_factor = 0.5F;
  CHECK((validate_config(c) & cfgerr::kRelease) != 0U);
  uint32_t errors = 0U;
  CHECK(sanitize_config(c, errors).version_tol_factor == 1.5F && errors == cfgerr::kRelease);
  c.version_tol_factor = std::nanf("");
  CHECK((validate_config(c) & cfgerr::kRelease) != 0U);
  c.version_tol_factor = 1.0F;  // exactly one is allowed: the same limit for both releases
  CHECK(validate_config(c) == 0U);
  c.version_tol_factor = 2.0e6F;
  CHECK((validate_config(c) & cfgerr::kRelease) != 0U);
  RedundancyConfig other;
  other.version_tol_factor = 2.0F;
  RedundancyConfig third;
  third.release_aware = false;
  CHECK(config_digest(other) != config_digest(RedundancyConfig{}) && config_digest(third) != config_digest(RedundancyConfig{}));
}

}  // namespace
