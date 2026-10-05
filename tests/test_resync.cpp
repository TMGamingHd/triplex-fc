// SPDX-License-Identifier: MIT
// State resynchronisation (core/include/tfc/resync.hpp, docs/RESYNC.md; TS-16 option C): the shared form of a computer's state, its frames, the vote, the adoption in the flight function,
// the manager's persistence rule for digest mismatches and its handling of state corrections, and the closed loop on a lossy bus.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "closed_loop.hpp"
#include "tfc/flight.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/resync.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;
using namespace tfc::resync;

SharedState state_of(int16_t base) {
  SharedState s;
  for (unsigned i = 0; i < kWords; ++i) {
    s.w[i] = static_cast<int16_t>(base + static_cast<int16_t>(i));
  }
  s.w[kMeta] = 0x0305;  // steps low byte 5, aligned, rates valid
  return s;
}

// A collector holding the given nodes' states, all of them whole, for frame `frame`.
Collector collect(uint32_t frame, const std::array<SharedState, 3>& st, uint8_t nodes) {
  Collector c;
  c.begin(frame);
  for (uint8_t n = 0; n < 3U; ++n) {
    if (((nodes >> n) & 1U) != 0U) {
      for (const Frame& f : pack_state(n, st[n], static_cast<uint8_t>(frame))) {
        (void)c.on_frame(f);
      }
    }
  }
  return c;
}

TFC_TEST(resync_a_state_survives_the_shared_form_to_the_resolution_of_the_digest) {
  AttitudeEstimator est(EstimatorConfig{});
  Controller ctl;
  ConsensusInput in;
  in.gyro_ok = true;
  in.accel_ok = true;
  in.gyro_dps.v = {0.4F, -0.7F, 0.2F};
  in.accel_g.v = {0.05F, -0.03F, 0.99F};
  for (int i = 0; i < 300; ++i) {
    est.update(in, 0.01F);
    (void)ctl.step(est.attitude(), Reference{1.0F, -2.0F, 0.1F, 0.1F}, 0.01F);
  }
  const AttitudeEstimator::State e0 = est.state();
  const Controller::State c0 = ctl.state();
  const SharedState s = make_shared(e0, c0);
  AttitudeEstimator::State e1;
  Controller::State c1;
  CHECK(make_state(s, 0x12345600U | (e0.steps & 0xFFU), e1, c1));
  for (unsigned i = 0; i < 4U; ++i) {
    CHECK(std::fabs(e1.q[i] - e0.q[i]) < 1.0e-4F);
  }
  for (unsigned i = 0; i < 3U; ++i) {
    CHECK(std::fabs(e1.bias[i] - e0.bias[i]) < 1.0e-5F);
  }
  for (unsigned i = 0; i < 2U; ++i) {
    CHECK(std::fabs(c1.integ[i] - c0.integ[i]) < 0.001F && std::fabs(c1.out[i] - c0.out[i]) < 0.001F);
  }
  CHECK(e1.steps == (0x12345600U | (e0.steps & 0xFFU)) && e1.aligned == e0.aligned && e1.rates_valid == e0.rates_valid);  // only the low byte of the count is shared
  SharedState scaled = s;  // a quaternion that quantisation or a vote left a few percent long: the adopted one is a unit quaternion again
  scaled.w[kQuatFirst] = 32767;
  scaled.w[kQuatFirst + 1U] = 20000;  // length 1.17
  scaled.w[kQuatFirst + 2U] = 0;
  scaled.w[kQuatFirst + 3U] = 0;
  AttitudeEstimator::State e2;
  Controller::State c2;
  CHECK(make_state(scaled, 0U, e2, c2));
  CHECK(std::fabs((e2.q[0] * e2.q[0]) + (e2.q[1] * e2.q[1]) + (e2.q[2] * e2.q[2]) + (e2.q[3] * e2.q[3]) - 1.0F) < 1.0e-6F);
  AttitudeEstimator back(EstimatorConfig{});
  back.set_state(e1);
  CHECK(std::fabs(back.attitude().tilt_x_deg - est.attitude().tilt_x_deg) < 0.01F && std::fabs(back.attitude().tilt_y_deg - est.attitude().tilt_y_deg) < 0.01F);
}

TFC_TEST(resync_the_shared_form_is_defined_for_awkward_values) {
  AttitudeEstimator::State e;
  Controller::State c;
  e.q = {-1.0F, 0.0F, 0.0F, 0.0F};  // the same attitude as +1
  const SharedState neg = make_shared(e, c);
  e.q = {1.0F, 0.0F, 0.0F, 0.0F};
  const SharedState pos = make_shared(e, c);
  CHECK(neg.w == pos.w && pos.w[kQuatFirst] == 32767);
  e.bias = {1.0F, -1.0F, std::nanf("")};  // far beyond the range, and not a number
  c.out = {1.0e9F, -1.0e9F};
  const SharedState wild = make_shared(e, c);
  CHECK(wild.w[kBiasFirst] == 32767 && wild.w[kBiasFirst + 1U] == -32767 && wild.w[kBiasFirst + 2U] == 0);
  CHECK(wild.w[kCtrlFirst + 2U] == 32767 && wild.w[kCtrlFirst + 3U] == -32767);
  CHECK(to_word(0.0004F, 0.001F) == 0 && to_word(0.0006F, 0.001F) == 1 && to_word(-0.0006F, 0.001F) == -1);  // rounds to nearest, both signs
  CHECK(to_word(32766.4F, 1.0F) == 32766 && to_word(32767.0F, 1.0F) == 32767 && to_word(-32766.4F, 1.0F) == -32766 && to_word(-32767.0F, 1.0F) == -32767);
  SharedState zero;  // a quaternion of length 0 is not a state
  AttitudeEstimator::State e2;
  Controller::State c2;
  CHECK(!make_state(zero, 0U, e2, c2));
  SharedState big = pos;
  big.w[kQuatFirst + 1U] = 32767;  // length sqrt(2) is as far as it is allowed to be... and 2 is not
  big.w[kQuatFirst + 2U] = 32767;
  CHECK(!make_state(big, 0U, e2, c2));
  SharedState half = pos;
  half.w[kQuatFirst] = 16000;  // length 0.49: too short
  CHECK(!make_state(half, 0U, e2, c2));
}

TFC_TEST(resync_frames_round_trip_and_reject_what_is_not_theirs) {
  const SharedState s = state_of(100);
  const std::array<Frame, kResyncChunks> fr = pack_state(2U, s, 0xA7U);
  for (std::size_t k = 0; k < kResyncChunks; ++k) {
    CHECK(fr[k].id == id::kResync + 8U + k && fr[k].len == 8U);
    const DecodedResync d = unpack_resync(fr[k]);
    const std::size_t i = 3U * k;
    CHECK(d.ok && d.node == 2U && d.chunk == k && d.seq == 0xA7U && d.words[0] == s.w[i] && d.words[1] == s.w[i + 1U] && d.words[2] == s.w[i + 2U]);
  }
  Frame bad = fr[1];
  bad.data[2] = static_cast<uint8_t>(bad.data[2] ^ 1U);
  CHECK(!unpack_resync(bad).ok);                                // CRC
  Frame low = fr[0];
  low.id = id::kResync - 1U;
  Frame high = fr[0];
  high.id = id::kResync + kResyncIds;
  CHECK(!unpack_resync(low).ok && !unpack_resync(high).ok);     // outside the range
  Frame last = pack_resync(2U, 3U, {1, 2, 3}, 0U);
  CHECK(unpack_resync(last).ok && last.id == id::kResync + kResyncIds - 1U);
  Frame golden = pack_resync(1U, 2U, {0x0102, -2, 0x7FFF}, 0x55U);  // the wire format, byte for byte
  CHECK(golden.id == 0x426U && golden.data[0] == 0x02 && golden.data[1] == 0x01 && golden.data[2] == 0xFE && golden.data[3] == 0xFF && golden.data[4] == 0xFF && golden.data[5] == 0x7F && golden.data[6] == 0x55);
}

TFC_TEST(resync_the_collector_keeps_only_whole_states_of_this_cycle) {
  const std::array<SharedState, 3> st{state_of(0), state_of(1000), state_of(-1000)};
  Collector c;
  c.begin(0x1F7U);
  const std::array<Frame, kResyncChunks> a = pack_state(0U, st[0], 0xF7U);
  const std::array<Frame, kResyncChunks> b = pack_state(1U, st[1], 0xF7U);
  const std::array<Frame, kResyncChunks> stale = pack_state(2U, st[2], 0xF6U);  // the previous cycle's
  for (unsigned k = 0; k < kResyncChunks; ++k) {
    CHECK(c.on_frame(a[k]) && !c.on_frame(stale[k]));
  }
  CHECK(c.on_frame(b[0]) && c.on_frame(b[1]) && c.on_frame(b[3]));  // chunk 2 of node B is missing
  CHECK(c.complete() == 0x01U);
  CHECK(c.on_frame(b[2]) && c.on_frame(b[2]));                      // arrives (twice: harmless)
  CHECK(c.complete() == 0x03U && c.share(1U).w == st[1].w && c.share(0U).w == st[0].w);
  CHECK(!c.on_frame(pack_gyro(0U, Vec3{}, 0xF7U)));                 // not a resync frame
  Frame crc = b[0];
  crc.data[0] = static_cast<uint8_t>(crc.data[0] ^ 0x10U);
  CHECK(!c.on_frame(crc));
  CHECK(c.share(3U).w == SharedState{}.w);                          // no such node
  c.begin(0x1F8U);                                                  // a new cycle forgets
  CHECK(c.complete() == 0U && c.share(0U).w == SharedState{}.w);
}

TFC_TEST(resync_the_schedule_is_the_last_frame_of_each_period) {
  CHECK(due(99U, 100U) && due(199U, 100U) && !due(0U, 100U) && !due(98U, 100U) && !due(100U, 100U));
  CHECK(!due(5U, 0U) && due(0U, 1U) && due(7U, 1U));  // a period of 1 is every frame; 0 is never
}

TFC_TEST(resync_three_states_vote_the_mid_value_of_each_word_and_report_who_was_off) {
  std::array<SharedState, 3> st{state_of(100), state_of(100), state_of(100)};
  Collector c = collect(7U, st, 0x07U);
  Outcome o = vote(c, 0x07U, Config{});
  CHECK(o.adopted && o.why == Why::Adopted && o.voters == 0x07U && o.changed == 0U && o.large == 0U && o.state.w == st[0].w);
  st[1].w[kQuatFirst + 1U] = static_cast<int16_t>(st[1].w[kQuatFirst + 1U] + 20);  // B a little off in one word
  st[2].w[kBiasFirst] = static_cast<int16_t>(st[2].w[kBiasFirst] - 7);           // C a little off in another
  c = collect(7U, st, 0x07U);
  o = vote(c, 0x07U, Config{});
  CHECK(o.adopted && o.state.w == state_of(100).w && o.changed == 0x06U && o.large == 0U);  // the mid-value is the common value; B and C changed, none far
  st[1].w[kQuatFirst + 1U] = static_cast<int16_t>(100 + 1 + 151);  // 151 lsb from the mid-value: just beyond the large limit
  c = collect(7U, st, 0x07U);
  o = vote(c, 0x07U, Config{});
  CHECK(o.large == 0x02U);
  st[1].w[kQuatFirst + 1U] = static_cast<int16_t>(100 + 1 + 150);  // exactly at the limit: not large
  c = collect(7U, st, 0x07U);
  CHECK(vote(c, 0x07U, Config{}).large == 0U);
  // the three classes have their own limits: bias 500, controller 500
  st = {state_of(100), state_of(100), state_of(100)};
  st[0].w[kBiasFirst + 1U] = static_cast<int16_t>(100 + 5 + 501);
  st[0].w[kCtrlFirst + 3U] = static_cast<int16_t>(100 + 10 + 500);
  c = collect(7U, st, 0x07U);
  o = vote(c, 0x07U, Config{});
  CHECK(o.large == 0x01U && o.state.w[kBiasFirst + 1U] == 105 && o.state.w[kCtrlFirst + 3U] == 110);
  st[0].w[kBiasFirst + 1U] = static_cast<int16_t>(100 + 5 + 500);
  c = collect(7U, st, 0x07U);
  CHECK(vote(c, 0x07U, Config{}).large == 0U);  // the bias at its limit, the controller word at its limit
  st[0].w[kCtrlFirst + 3U] = static_cast<int16_t>(100 + 10 + 501);
  c = collect(7U, st, 0x07U);
  CHECK(vote(c, 0x07U, Config{}).large == 0x01U);
  // a difference in the meta word alone is a change but never a "large" one (it is a count)
  st = {state_of(100), state_of(100), state_of(100)};
  st[2].w[kMeta] = 0x0304;
  c = collect(7U, st, 0x07U);
  o = vote(c, 0x07U, Config{});
  CHECK(o.adopted && o.changed == 0x04U && o.large == 0U && o.state.w[kMeta] == 0x0305);
  st[2].w[kMeta] = 0x0000;  // far from the others as a number (773), and still not a large correction
  c = collect(7U, st, 0x07U);
  o = vote(c, 0x07U, Config{});
  CHECK(o.adopted && o.changed == 0x04U && o.large == 0U);
  st[2].w[kMeta] = 0x0000;  // wrapped: 255, 0, 0 style: the mid-value is one of the real words
  st[0].w[kMeta] = 0x03FF;
  st[1].w[kMeta] = 0x0300;
  c = collect(7U, st, 0x07U);
  CHECK(vote(c, 0x07U, Config{}).state.w[kMeta] == 0x0300);
}

TFC_TEST(resync_two_states_are_averaged_only_if_they_agree_and_an_incomplete_quorum_adopts_nothing) {
  std::array<SharedState, 3> st{state_of(100), state_of(110), state_of(5000)};
  Collector c = collect(9U, st, 0x07U);
  Outcome o = vote(c, 0x03U, Config{});  // C is not healthy (latched, say): its share is sent and ignored
  CHECK(o.adopted && o.voters == 0x03U && o.state.w[0] == 105 && o.state.w[5] == 110);  // the mean of 100 and 110, of 105 and 115
  CHECK(o.state.w[kMeta] == 0x0305 && o.changed == 0x03U);
  st[1].w[kQuatFirst] = static_cast<int16_t>(100 + 331);  // 331 apart: beyond the pair limit
  c = collect(9U, st, 0x07U);
  o = vote(c, 0x03U, Config{});
  CHECK(!o.adopted && o.why == Why::Disagree);
  st[1].w[kQuatFirst] = static_cast<int16_t>(100 + 330);  // exactly at it
  c = collect(9U, st, 0x07U);
  CHECK(vote(c, 0x03U, Config{}).adopted);
  st[1].w[kBiasFirst] = static_cast<int16_t>(100 + 4 + 1201);
  c = collect(9U, st, 0x07U);
  CHECK(!vote(c, 0x03U, Config{}).adopted);
  st[1].w[kBiasFirst] = static_cast<int16_t>(100 + 4 + 1200);
  st[1].w[kCtrlFirst] = static_cast<int16_t>(100 + 7 + 1001);
  c = collect(9U, st, 0x07U);
  CHECK(!vote(c, 0x03U, Config{}).adopted);
  st[1].w[kCtrlFirst] = static_cast<int16_t>(100 + 7 + 1000);
  st[1].w[kMeta] = 0x0306;  // two computers that disagree on the count do not average it
  c = collect(9U, st, 0x07U);
  o = vote(c, 0x03U, Config{});
  CHECK(!o.adopted && o.why == Why::Disagree);
  st[1].w[kMeta] = 0x0305;
  CHECK(vote(collect(9U, st, 0x07U), 0x03U, Config{}).adopted);
  // fewer than two, or a share that did not arrive whole: nothing
  CHECK(vote(collect(9U, st, 0x01U), 0x01U, Config{}).why == Why::TooFew);
  CHECK(vote(collect(9U, st, 0x07U), 0x00U, Config{}).why == Why::TooFew);
  o = vote(collect(9U, st, 0x03U), 0x07U, Config{});  // C is healthy but its share did not arrive: a vote of A and B is not the vote the others take
  CHECK(!o.adopted && o.why == Why::Incomplete && o.voters == 0x03U);
  CHECK(vote(collect(9U, st, 0x07U), 0x0FU, Config{}).adopted);  // bits above the three nodes mean nothing
}

TFC_TEST(resync_median_of_three_words_is_the_middle_one_in_every_order) {
  const auto m3 = [](int a, int b, int c) { return resync::median3(static_cast<int16_t>(a), static_cast<int16_t>(b), static_cast<int16_t>(c)); };
  const int16_t v[3] = {-5, 3, 9};
  const int perm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  for (const auto& p : perm) {
    CHECK(m3(v[p[0]], v[p[1]], v[p[2]]) == 3);
  }
  CHECK(m3(4, 4, 9) == 4 && m3(9, 4, 4) == 4 && m3(4, 9, 4) == 4 && m3(-32767, 32767, 0) == 0);
}

// Two replicas of the flight function, given a different set of frames in one frame, diverge; after the resync they hold the same bits and stay together.
TFC_TEST(resync_replicas_that_diverged_after_a_lost_frame_are_one_again_and_stay_so) {
  GainSchedule g;
  (void)g.add(0U, ControllerGains{1.0F, 0.6F, 0.1F});
  Guidance gd;
  (void)gd.add(1U, 0U, 0.0F);
  (void)gd.add(1U, 400U, 5.0F);
  std::array<FlightFunction, 3> ff{FlightFunction(g, gd), FlightFunction(g, gd), FlightFunction(g, gd)};
  uint32_t s = 12345U;
  auto noise = [&s]() {
    s = (s * 1664525U) + 1013904223U;
    return (static_cast<float>(s >> 8) / 8388608.0F) - 1.0F;
  };
  bool diverged_before_resync = false;
  bool one_after_resync = true;
  for (uint32_t k = 0; k < 400U; ++k) {
    std::array<Frame, 6> fr{};
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 gy;
      Vec3 a;
      gy.v = {0.1F * noise(), 0.5F + (0.1F * noise()) + (n == 0U ? 0.8F : 0.0F), 0.1F * noise()};  // node A's gyro reads 0.8 dps more than the others
      a.v = {0.01F * noise(), 0.01F * noise(), 1.0F + (0.002F * noise())};
      fr[n] = pack_gyro(n, gy, static_cast<uint8_t>(k));
      fr[3U + n] = pack_accel(n, a, static_cast<uint8_t>(k));
    }
    std::array<Command, 3> cmd{};
    for (unsigned n = 0; n < 3U; ++n) {
      ff[n].begin_frame(k, 0x07U);
      for (unsigned i = 0; i < 6U; ++i) {
        if (!(k >= 100U && k < 130U && n == 1U && i == 0U)) {  // node B misses node A's gyro frame in frames 100 to 129 (one frame leaves less than the shared form's resolution)
          (void)ff[n].on_frame(fr[i]);
        }
      }
      cmd[n] = ff[n].step();
    }
    if (due(k, 100U)) {  // frame 199 and 299 and 399
      std::array<Collector, 3> col{};
      std::array<SharedState, 3> shares{};
      for (unsigned n = 0; n < 3U; ++n) {
        shares[n] = ff[n].shared_state();
      }
      for (unsigned n = 0; n < 3U; ++n) {
        col[n].begin(k);
        for (unsigned m = 0; m < 3U; ++m) {
          for (const Frame& f : pack_state(m, shares[m], static_cast<uint8_t>(k))) {
            (void)col[n].on_frame(f);
          }
        }
      }
      if (k == 199U) {
        diverged_before_resync = shares[1].w != shares[0].w && shares[0].w == shares[2].w;  // B alone is off
      }
      for (unsigned n = 0; n < 3U; ++n) {
        const Outcome o = vote(col[n], 0x07U, Config{});
        CHECK(o.adopted && ff[n].adopt_state(o.state));
        CHECK(o.large == 0U);  // a lost frame is a small difference
      }
      one_after_resync = one_after_resync && ff[0].shared_state().w == ff[1].shared_state().w && ff[1].shared_state().w == ff[2].shared_state().w;
    }
    if (k == 200U) {
      CHECK(cmd[0].state_digest == cmd[1].state_digest && cmd[1].state_digest == cmd[2].state_digest);  // the frame after the first resync: bit-identical again
    }
    if (k == 299U) {
      CHECK(cmd[0].state_digest == cmd[1].state_digest && cmd[1].state_digest == cmd[2].state_digest);  // and it stayed so: identical inputs, identical state
    }
  }
  CHECK(one_after_resync);  // after every resync the three shared states are the same words
  CHECK(diverged_before_resync);  // the lost frame did leave the states different, and nothing else healed them before the resync
}

TFC_TEST(resync_adopting_an_unusable_state_changes_nothing) {
  FlightFunction ff;
  const SharedState before = ff.shared_state();
  SharedState bad;  // all zero: no quaternion
  CHECK(!ff.adopt_state(bad));
  CHECK(ff.shared_state().w == before.w);
  SharedState good = before;
  good.w[kBiasFirst] = 1234;
  CHECK(ff.adopt_state(good) && ff.shared_state().w[kBiasFirst] == 1234);
}

// ---- the manager ----

struct Rig {
  RedundancyManager mgr;
  uint32_t frame = 0U;
  explicit Rig(const RedundancyConfig& cfg) : mgr(cfg) {}

  // One frame: three healthy nodes; the digest of each as given.
  const FrameReport& run(const std::array<uint16_t, 3>& digest, const std::array<Frame, 0>* = nullptr) {
    mgr.begin_frame(frame);
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 g;
      Vec3 a;
      g.v[0] = 0.125F * static_cast<float>(frame % 13U);  // the values change every frame: the stuck-sensor detector would otherwise latch a healthy node
      a.v = {0.0F, 0.0F, 1.0F};
      mgr.on_frame(pack_gyro(n, g, static_cast<uint8_t>(frame)));
      mgr.on_frame(pack_accel(n, a, static_cast<uint8_t>(frame)));
      mgr.on_frame(pack_cmd(n, Command{0.5F, -0.25F, digest[n]}, static_cast<uint8_t>(frame)));
    }
    const FrameReport& r = mgr.end_frame();
    ++frame;
    return r;
  }
};

TFC_TEST(resync_a_digest_mismatch_counts_only_after_its_persistence) {
  RedundancyConfig cfg;
  cfg.digest_persist_frames = 5U;
  Rig a(cfg);
  for (int i = 0; i < 10; ++i) {
    (void)a.run({1U, 1U, 1U});
  }
  for (unsigned i = 0; i < 4U; ++i) {
    const FrameReport& r = a.run({1U, 2U, 1U});  // B's digest differs
    CHECK(r.reason[1] == 0U && r.latched_mask == 0U);  // up to 4 frames: not yet counted
  }
  const FrameReport& fifth = a.run({1U, 2U, 1U});
  CHECK((fifth.reason[1] & reason::kDigest) != 0U && fifth.reason[0] == 0U);  // the fifth in a row counts, against B
  CHECK(a.mgr.counters().digest_flags == 1U);
  // a mismatch that healed in time starts from zero the next time
  Rig b(cfg);
  for (int i = 0; i < 10; ++i) {
    (void)b.run({1U, 1U, 1U});
  }
  for (int rep = 0; rep < 6; ++rep) {
    for (unsigned i = 0; i < 4U; ++i) {
      CHECK(b.run({1U, 2U, 1U}).reason[1] == 0U);
    }
    CHECK(b.run({1U, 1U, 1U}).reason[1] == 0U);  // healed
  }
  CHECK(b.mgr.counters().digest_flags == 0U && b.mgr.counters().nodes_disabled == 0U);
  // unresolved (three different digests) is held back by the same rule, so the Safe request needs the persistence plus its own M of N
  Rig c(cfg);
  for (int i = 0; i < 10; ++i) {
    (void)c.run({1U, 1U, 1U});
  }
  for (unsigned i = 0; i < 4U; ++i) {
    CHECK(!c.run({1U, 2U, 3U}).unresolved);
  }
  CHECK(c.run({1U, 2U, 3U}).unresolved);
  // the default is as before: at once
  Rig d{RedundancyConfig{}};
  for (int i = 0; i < 3; ++i) {
    (void)d.run({1U, 1U, 1U});
  }
  CHECK((d.run({1U, 2U, 1U}).reason[1] & reason::kDigest) != 0U);
}

TFC_TEST(resync_the_persistence_is_validated_and_part_of_the_configuration_digest) {
  RedundancyConfig cfg;
  cfg.digest_persist_frames = 0U;
  CHECK((validate_config(cfg) & cfgerr::kPersistence) != 0U);
  uint32_t errors = 0U;
  const RedundancyConfig fixed = sanitize_config(cfg, errors);
  CHECK(fixed.digest_persist_frames == 1U && errors == cfgerr::kPersistence);
  RedundancyConfig other;
  other.digest_persist_frames = 150U;
  CHECK(config_digest(other) != config_digest(RedundancyConfig{}) && validate_config(other) == 0U);
}

TFC_TEST(resync_a_large_correction_is_a_bad_frame_and_a_repeat_offender_latches) {
  Rig r{RedundancyConfig{}};
  for (int i = 0; i < 10; ++i) {
    (void)r.run({1U, 1U, 1U});
  }
  r.mgr.begin_frame(r.frame);  // a correction reported for B in a frame
  r.mgr.report_state_correction(0x02U);
  for (uint8_t n = 0; n < 3U; ++n) {
    Vec3 g;
    Vec3 a;
    g.v[0] = 0.125F * static_cast<float>(r.frame % 13U);
    a.v = {0.0F, 0.0F, 1.0F};
    r.mgr.on_frame(pack_gyro(n, g, static_cast<uint8_t>(r.frame)));
    r.mgr.on_frame(pack_accel(n, a, static_cast<uint8_t>(r.frame)));
    r.mgr.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 1U}, static_cast<uint8_t>(r.frame)));
  }
  const FrameReport& first = r.mgr.end_frame();
  ++r.frame;
  CHECK((first.reason[1] & reason::kResync) != 0U && first.reason[0] == 0U && first.latched_mask == 0U);  // one is a bad frame, not a latch
  CHECK(r.mgr.counters().state_corrections == 1U);
  CHECK(r.run({1U, 1U, 1U}).reason[1] == 0U);  // and the report is for that frame only
  // three corrections in five frames latch it, as three bad frames of any reason do
  uint8_t latched = 0U;
  for (int i = 0; i < 5; ++i) {
    r.mgr.begin_frame(r.frame);
    if (i % 2 == 0) {
      r.mgr.report_state_correction(0x02U);
    }
    for (uint8_t n = 0; n < 3U; ++n) {
      Vec3 g;
      Vec3 a;
      g.v[0] = 0.125F * static_cast<float>(r.frame % 13U);
      a.v = {0.0F, 0.0F, 1.0F};
      r.mgr.on_frame(pack_gyro(n, g, static_cast<uint8_t>(r.frame)));
      r.mgr.on_frame(pack_accel(n, a, static_cast<uint8_t>(r.frame)));
      r.mgr.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 1U}, static_cast<uint8_t>(r.frame)));
    }
    latched = static_cast<uint8_t>(latched | r.mgr.end_frame().newly_latched);
    ++r.frame;
  }
  CHECK((latched & 0x02U) != 0U);
  char text[64];
  format_reasons(reason::kResync, text, sizeof text);
  CHECK(std::string(text) == "state far from the vote");
  Rig q{RedundancyConfig{}};  // bits above the three nodes are ignored
  q.mgr.report_state_correction(0xF8U);
  CHECK(q.mgr.counters().state_corrections == 0U);
}

TFC_TEST(resync_its_frames_are_in_the_schedule_and_do_not_raise_the_babbling_alarm) {
  Rig r{RedundancyConfig{}};
  for (int i = 0; i < 5; ++i) {
    (void)r.run({1U, 1U, 1U});
  }
  r.mgr.begin_frame(r.frame);
  for (uint8_t n = 0; n < 3U; ++n) {
    Vec3 g;
    Vec3 a;
    g.v[0] = 0.125F * static_cast<float>(r.frame % 13U);
    a.v = {0.0F, 0.0F, 1.0F};
    r.mgr.on_frame(pack_gyro(n, g, static_cast<uint8_t>(r.frame)));
    r.mgr.on_frame(pack_accel(n, a, static_cast<uint8_t>(r.frame)));
    r.mgr.on_frame(pack_cmd(n, Command{0.5F, -0.25F, 1U}, static_cast<uint8_t>(r.frame)));
    for (const Frame& f : pack_state(n, state_of(10), static_cast<uint8_t>(r.frame))) {
      r.mgr.on_frame(f);  // twelve frames in the frame
    }
  }
  const FrameReport& rep = r.mgr.end_frame();
  CHECK(!rep.bus_alarm && rep.out_of_schedule_in_frame == 0U && r.mgr.counters().out_of_schedule == 0U);
  Frame outside = pack_state(0U, state_of(10), 0U)[0];
  outside.id = id::kResync + kResyncIds;  // one past the range is out of schedule
  r.mgr.begin_frame(r.frame);
  r.mgr.on_frame(outside);
  CHECK(r.mgr.counters().out_of_schedule == 1U);
}

// ---- the closed loop ----

TFC_TEST(resync_the_closed_loop_flies_a_one_percent_lossy_bus_that_it_loses_without_it) {
  sim::Loop base;
  base.frames = 6000U;
  base.estimator.use_accel = false;
  base.frame_loss_prob = 0.01F;
  const sim::Result without = sim::run(base);
  CHECK(without.safe_frames > 0U && without.max_deg_settled > 100.0);  // TS-16: the flight is lost
  sim::Loop with = base;
  with.resync_period = 100U;
  const sim::Result r = sim::run(with);
  CHECK(r.safe_frames == 0U && r.max_deg_settled < 1.0 && r.finite);  // healed: the flight flies as the nominal one does
  CHECK(r.resyncs_adopted > 100U && r.corrections_changed > 0U && r.corrections_large == 0U);
  CHECK(r.frames_over_tol < without.frames_over_tol / 10U);
  sim::Loop clean = base;
  clean.frame_loss_prob = 0.0F;
  clean.resync_period = 100U;
  const sim::Result c = sim::run(clean);
  CHECK(c.resyncs_adopted == 180U && c.resyncs_skipped == 0U && c.corrections_changed == 0U && c.digest_mismatch_frames == 0U);  // 3 computers x 60 resyncs, nothing to correct
  const sim::Result again = sim::run(with);
  CHECK(again.resyncs_adopted == r.resyncs_adopted && again.corrections_changed == r.corrections_changed);  // repeatable
}

TFC_TEST(resync_a_corrupted_state_is_healed_and_reported_as_a_large_correction) {
  sim::Loop lp;
  lp.frames = 3000U;
  lp.estimator.use_accel = false;
  lp.resync_period = 100U;
  lp.corrupt_b_at = 1050U;  // an attitude error of about 2 degrees in node B
  const sim::Result r = sim::run(lp);
  CHECK(r.corrections_large == 1U && r.safe_frames == 0U && r.max_deg_settled < 1.0);
  CHECK(r.digest_mismatch_frames >= 49U && r.digest_mismatch_frames <= 51U);  // from the corruption to the resync that healed it, and no longer
  sim::Loop none = lp;
  none.corrupt_b_at = 0xFFFFFFFFU;
  CHECK(sim::run(none).corrections_large == 0U);
  sim::Loop dead = lp;
  dead.node_b_dead_from = 500U;  // B is gone: two computers still resynchronise (their states agree)
  const sim::Result d = sim::run(dead);
  CHECK(d.resyncs_adopted > 40U && d.resyncs_skipped == 0U);
}

}  // namespace
