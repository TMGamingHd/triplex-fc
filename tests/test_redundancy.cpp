// SPDX-License-Identifier: MIT
// RedundancyManager: one flight computer's per-frame consensus + FDIR step, driven with frames
// built by the real pack_* functions. Detection-time numbers here match the SIL scenarios and
// docs/REQUIREMENTS.md; the virtual-peers end-to-end tests check the same through the wire log.
#include <array>
#include <cmath>
#include <cstring>

#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

struct NodeFault {
  bool present = true;
  float gyro_bias = 0.0F;      // dps added to gyro axis 0
  uint16_t digest_xor = 0;     // XORed into the command digest
  bool corrupt_gyro = false;   // flip one payload bit of the gyro frame
};

float truth(int k) { return 5.0F + 0.125F * static_cast<float>(k % 16); }  // moves every frame (sawtooth)
// Smooth motion for tests that need continuity between frames (max slope 0.6 dps/frame, under the 1.0 tolerance).
float smooth(int k) { return 5.0F + 3.0F * std::sin(0.2F * static_cast<float>(k)); }

// Feeds one major frame from the three nodes and returns the manager's report.
const FrameReport& frame(RedundancyManager& m, int k, const std::array<NodeFault, 3>& f,
                         float (*tr)(int) = truth) {
  m.begin_frame();
  const uint8_t seq = static_cast<uint8_t>(k);
  for (unsigned n = 0; n < 3; ++n) {
    if (!f[n].present) {
      continue;
    }
    Frame g = pack_gyro(static_cast<uint8_t>(n), Vec3{{tr(k) + f[n].gyro_bias, -2.0F, 1.0F}}, seq);
    if (f[n].corrupt_gyro) {
      g.data[0] = static_cast<uint8_t>(g.data[0] ^ 1U);
    }
    m.on_frame(g);
    m.on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F + 0.001F * static_cast<float>(k % 7)}}, seq));
    m.on_frame(pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, static_cast<uint16_t>(0x1234U ^ f[n].digest_xor)}, seq));
  }
  return m.end_frame();
}

// Runs `frames` frames; node `bad` gets `fault` from frame `from` on. Returns the frame on which
// `bad` latched, or -1. `out_reason` receives its reason bits on that frame.
int run(unsigned bad, const NodeFault& fault, int from, int frames, RedundancyManager& m, uint8_t* out_reason = nullptr) {
  int latched_at = -1;
  for (int k = 0; k < frames; ++k) {
    std::array<NodeFault, 3> f{};
    if (k >= from) {
      f[bad] = fault;
    }
    const FrameReport& r = frame(m, k, f);
    if (((r.newly_latched >> bad) & 1U) != 0U && latched_at < 0) {
      latched_at = k;
      if (out_reason != nullptr) {
        *out_reason = r.reason[bad];
      }
    }
  }
  return latched_at;
}

}  // namespace

TFC_TEST(manager_healthy_triplex_has_no_flags) {
  RedundancyManager m;
  const std::array<NodeFault, 3> none{};
  for (int k = 0; k < 60; ++k) {
    const FrameReport& r = frame(m, k, none);
    CHECK(r.mode == Mode::Triplex);
    CHECK(r.newly_latched == 0U);
    CHECK(r.valid_mask == kAllChannels);
    CHECK(r.votes[0].status == VoteStatus::Triplex);
  }
  const Counters& c = m.counters();
  CHECK(c.frames == 60U);
  CHECK(c.crc_bad == 0U && c.seq_bad == 0U && c.missing == 0U && c.vote_disagreements == 0U);
}

TFC_TEST(manager_bias_latches_in_exactly_m_frames_and_vote_ignores_it) {
  for (unsigned bad = 0; bad < 3; ++bad) {
    RedundancyManager m;
    uint8_t why = 0;
    NodeFault f;
    f.gyro_bias = 3.0F;
    CHECK(run(bad, f, 10, 60, m, &why) == 12);  // 3-of-5: fault frame + 2
    CHECK(why == reason::kVote);
    CHECK(m.latched(bad));
    CHECK(m.last_report().mode == Mode::Duplex);
    CHECK(m.last_report().healthy == 2U);
  }
}

TFC_TEST(manager_voted_output_never_follows_the_bad_node) {
  RedundancyManager m;
  for (int k = 0; k < 40; ++k) {
    std::array<NodeFault, 3> f{};
    if (k >= 10) {
      f[1].gyro_bias = 3.0F;
    }
    const FrameReport& r = frame(m, k, f);
    CHECK(std::fabs(r.votes[0].value - truth(k)) < 0.2F);  // median of 3 picks a healthy value
  }
}

TFC_TEST(manager_missing_node_latches_in_3_frames) {
  RedundancyManager m;
  uint8_t why = 0;
  NodeFault f;
  f.present = false;
  CHECK(run(2, f, 5, 40, m, &why) == 7);
  CHECK(why == reason::kMissing);
  CHECK(m.counters().missing == 35U);  // keeps counting after the latch
}

TFC_TEST(manager_one_corrupt_frame_costs_one_sample_and_never_latches) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[0].corrupt_gyro = (k == 20);
    const FrameReport& r = frame(m, k, f);
    CHECK(r.newly_latched == 0U);
    if (k == 20) {
      CHECK(r.reason[0] == reason::kCrc);  // and nothing else: no false sequence gap
    }
    if (k == 21) {
      CHECK(r.reason[0] == 0U);
    }
  }
  CHECK(m.counters().crc_bad == 1U);
  CHECK(m.counters().seq_bad == 0U);
}

TFC_TEST(manager_two_separate_corrupt_frames_in_the_window_do_not_latch) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].corrupt_gyro = (k == 20 || k == 22);
    CHECK(frame(m, k, f).newly_latched == 0U);
  }
}

TFC_TEST(manager_digest_mismatch_alone_latches_and_is_labelled) {
  RedundancyManager m;
  uint8_t why = 0;
  NodeFault f;
  f.digest_xor = 1U;
  CHECK(run(0, f, 10, 40, m, &why) == 12);
  CHECK(why == reason::kDigest);
}

TFC_TEST(manager_stuck_sensor_is_flagged_by_the_stuck_detector) {
  // Constant sensor bytes for 30 frames from node 1; the others keep moving.
  RedundancyManager m;
  uint32_t flags_at_29 = 0;
  for (int k = 0; k < 30; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (unsigned n = 0; n < 3; ++n) {
      const int kk = n == 1U ? 0 : k;  // node 1 repeats frame 0's values
      m.on_frame(pack_gyro(static_cast<uint8_t>(n), Vec3{{truth(kk), -2.0F, 1.0F}}, seq));
      m.on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      m.on_frame(pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, 0x1234U}, seq));
    }
    m.end_frame();
    flags_at_29 = m.counters().stuck_flags;
  }
  CHECK(flags_at_29 > 0U);
}

TFC_TEST(manager_absent_everything_ends_in_safe) {
  RedundancyManager m;
  for (int k = 0; k < 3; ++k) {
    m.begin_frame();
    m.end_frame();
  }
  CHECK(m.last_report().mode == Mode::Safe);
  CHECK(m.last_report().healthy == 0U);
  CHECK(m.last_report().latched_mask == kAllChannels);
}

TFC_TEST(manager_duplex_second_step_fault_is_attributed_by_continuity) {
  // B latches at 12 (duplex A,C). At 30 C steps by 3 dps: C is the node that jumped away from the
  // last agreed value while A did not, so C is blamed and the system continues in Simplex on A
  // (before: both survivors were blamed and it fell to Safe).
  RedundancyManager m;
  int latched_c = -1;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    if (k >= 10) {
      f[1].gyro_bias = 3.0F;
    }
    if (k >= 30) {
      f[2].gyro_bias = 3.0F;
    }
    const FrameReport& r = frame(m, k, f, smooth);
    if (((r.newly_latched >> 2) & 1U) != 0U) {
      latched_c = k;
    }
    CHECK(!m.latched(0));  // the healthy node is never blamed
  }
  CHECK(latched_c == 32);
  CHECK(m.last_report().mode == Mode::Simplex);
}

TFC_TEST(manager_duplex_spike_is_blamed_on_the_node_that_jumped_not_on_both) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);  // C dies: latched at 7, duplex A,B from here on
    f[1].gyro_bias = (k == 30) ? 20.0F : 0.0F;
    const FrameReport& r = frame(m, k, f, smooth);
    if (k == 30) {
      CHECK((r.reason[1] & reason::kVote) != 0U);
      CHECK(r.reason[0] == 0U);
    }
  }
  CHECK(!m.latched(0) && !m.latched(1));
}

TFC_TEST(manager_duplex_three_spikes_in_five_frames_latch_only_the_culprit) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);
    f[1].gyro_bias = (k == 30 || k == 32 || k == 34) ? 20.0F : 0.0F;
    frame(m, k, f, smooth);
  }
  CHECK(m.latched(1));
  CHECK(!m.latched(0));
  CHECK(m.last_report().mode == Mode::Simplex);  // not Safe
}

TFC_TEST(manager_one_lost_frame_costs_one_sample_not_two) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].present = (k != 20);
    const FrameReport& r = frame(m, k, f);
    CHECK(r.newly_latched == 0U);
    if (k == 20) {
      CHECK(r.reason[1] == reason::kMissing);
    }
    if (k == 21) {
      CHECK(r.reason[1] == 0U);  // the next good frame is not a "sequence gap"
    }
  }
  CHECK(m.counters().missing == 1U);
  CHECK(m.counters().seq_bad == 0U);
}

TFC_TEST(manager_two_isolated_lost_frames_in_the_window_do_not_latch) {
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].present = !(k == 20 || k == 23);
    CHECK(frame(m, k, f).newly_latched == 0U);
  }
}

TFC_TEST(manager_a_node_that_returns_with_a_continuing_counter_is_not_penalised) {
  // Silent for 30 frames, then back with the counter still running (a hung node that recovers).
  RedundancyManager m;
  for (int k = 0; k < 80; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].present = !(k >= 20 && k < 50);
    frame(m, k, f);
  }
  CHECK(m.counters().seq_bad == 0U);
}

TFC_TEST(manager_reintegration_needs_explicit_request_and_100_clean_frames) {
  RedundancyManager m;
  for (int k = 0; k < 130; ++k) {
    std::array<NodeFault, 3> f{};
    if (k >= 10 && k < 15) {
      f[1].gyro_bias = 3.0F;  // transient fault, gone by frame 15
    }
    frame(m, k, f);
    if (k == 20) {
      CHECK(m.latched(1));  // clean again, but stays out without a request
      m.request_reintegration(1);
    }
    if (k == 110) {
      CHECK(m.latched(1));  // 90 clean frames since the request: not yet
    }
  }
  CHECK(!m.latched(1));
  CHECK(m.last_report().mode == Mode::Triplex);
}

TFC_TEST(manager_ignores_non_data_ids_and_counts_unknown_ones) {
  RedundancyManager m;
  m.begin_frame();
  CHECK(m.on_frame(pack_sync(7, 0)));
  Frame act;
  act.id = id::kActOut;
  CHECK(m.on_frame(act));
  Frame sim;
  sim.id = id::kSim;
  CHECK(m.on_frame(sim));
  Frame babble;
  babble.id = 0x023;
  CHECK(!m.on_frame(babble));
  Frame beyond;
  beyond.id = id::kGyroBase + 3U;  // no node D
  CHECK(!m.on_frame(beyond));
  CHECK(m.counters().out_of_schedule == 2U);
}

TFC_TEST(format_reasons_names_every_bit_and_truncates_safely) {
  std::array<char, 96> buf{};
  format_reasons(reason::kVote | reason::kDigest, buf.data(), buf.size());
  CHECK(std::strcmp(buf.data(), "vote disagreement + digest mismatch") == 0);
  format_reasons(0U, buf.data(), buf.size());
  CHECK(std::strcmp(buf.data(), "(none)") == 0);
  format_reasons(reason::kMissing, buf.data(), buf.size());
  CHECK(std::strcmp(buf.data(), "frame missing") == 0);
  std::array<char, 6> tiny{};
  format_reasons(reason::kCrc | reason::kSeq, tiny.data(), tiny.size());
  CHECK(std::strcmp(tiny.data(), "CRC f") == 0);  // truncated, still NUL-terminated
  format_reasons(reason::kCrc, tiny.data(), 0);   // zero capacity must not write
}

TFC_TEST(manager_startup_grace_lets_late_peers_join_without_latching) {
  RedundancyConfig cfg;
  cfg.startup_grace_frames = 200;
  RedundancyManager m(cfg);
  for (int k = 0; k < 300; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].present = (k >= 100);  // B boots at frame 100
    f[2].present = (k >= 150);  // C boots at frame 150
    const FrameReport& r = frame(m, k, f);
    CHECK(r.newly_latched == 0U);
    if (k == 99) {
      CHECK(r.mode == Mode::Simplex && r.healthy == 1U);  // only A has been seen
    }
    if (k == 100) {
      CHECK(r.newly_seen == 0x2U);
    }
    if (k == 120) {
      CHECK(r.mode == Mode::Duplex);
    }
    if (k == 150) {
      CHECK(r.newly_seen == 0x4U);
    }
  }
  CHECK(m.last_report().mode == Mode::Triplex);
  CHECK(m.counters().missing == 0U);
}

TFC_TEST(manager_startup_grace_expires_and_a_node_never_seen_then_latches) {
  RedundancyConfig cfg;
  cfg.startup_grace_frames = 50;
  RedundancyManager m(cfg);
  NodeFault gone;
  gone.present = false;
  CHECK(run(2, gone, 0, 100, m) == 52);  // first judged frame is 50; third bad frame latches it
  CHECK(!m.seen(2));
}

TFC_TEST(manager_a_seen_node_is_judged_even_inside_the_grace_period) {
  RedundancyConfig cfg;
  cfg.startup_grace_frames = 1000;
  RedundancyManager m(cfg);
  NodeFault gone;
  gone.present = false;
  CHECK(run(1, gone, 20, 60, m) == 22);  // seen at frame 0, so no grace once it goes silent
}

namespace {
float flat(int) { return 5.0F; }  // vehicle at rest; accel still varies per frame so nothing looks stuck
}  // namespace

TFC_TEST(manager_output_is_the_voted_value_and_not_held_in_normal_operation) {
  RedundancyManager m;
  for (int k = 0; k < 40; ++k) {
    const std::array<NodeFault, 3> none{};
    const FrameReport& r = frame(m, k, none, smooth);
    CHECK(r.held_mask == 0U);
    CHECK(std::fabs(r.output[0] - smooth(k)) < 0.2F);
    CHECK(!r.unresolved && !r.safe_request && !r.bus_alarm);
  }
}

TFC_TEST(manager_holds_the_last_good_output_when_nothing_can_be_voted) {
  RedundancyManager m;
  const std::array<NodeFault, 3> none{};
  float last = 0.0F;
  for (int k = 0; k < 30; ++k) {
    last = frame(m, k, none, smooth).output[0];
  }
  for (int k = 30; k < 40; ++k) {  // every node goes silent
    m.begin_frame();
    const FrameReport& r = m.end_frame();
    CHECK(r.output[0] == last);               // held, exactly
    CHECK((r.held_mask & 1U) != 0U);
  }
  // Before any history there is nothing to hold: 0, flagged as held.
  RedundancyManager fresh;
  fresh.begin_frame();
  const FrameReport& r0 = fresh.end_frame();
  CHECK(r0.output[0] == 0.0F && r0.held_mask == 0xFFU);
}

TFC_TEST(manager_slow_drift_in_duplex_requests_safe_and_blames_nobody_afterwards) {
  // C dies (duplex A,B). B then drifts 0.1 dps/frame. Right at the miscompare both nodes are within
  // reach of the last agreed value, so nobody can be blamed: hold the output and request Safe. The
  // request is sticky and the output frozen. B keeps drifting far beyond 2x tolerance, but the
  // held reference is stale by then, so arbitration must NOT use it: nobody is ever blamed (before
  // this rule a healthy node could be blamed once the vehicle had moved away from the stale value).
  RedundancyManager m;
  int first_safe = -1;
  int blamed = -1;
  float held_value = 0.0F;
  for (int k = 0; k < 120; ++k) {
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);
    f[1].gyro_bias = k >= 30 ? 0.1F * static_cast<float>(k - 30) : 0.0F;
    const FrameReport& r = frame(m, k, f, flat);
    if (r.safe_request && first_safe < 0) {
      first_safe = k;
      CHECK(r.mode == Mode::Safe);
      CHECK(!m.latched(0) && !m.latched(1));  // nobody blamed yet
      CHECK(r.held_mask == 0xFFU);
      held_value = r.output[0];
    }
    if (first_safe >= 0) {
      CHECK(r.safe_request);                  // sticky
      CHECK(r.output[0] == held_value);       // frozen for the whole time
    }
    if ((r.newly_latched & 0x3U) != 0U && blamed < 0) {  // A or B (C was killed on purpose)
      blamed = k;
    }
  }
  CHECK(first_safe >= 38 && first_safe <= 46);
  CHECK(blamed < 0);                           // nobody is blamed on a stale reference
  CHECK(!m.latched(0) && !m.latched(1));
  CHECK(m.last_report().mode == Mode::Safe);   // and Safe is not lifted by luck
  CHECK(m.safe_requested());

  m.clear_safe_request();                      // operator/ground command, after the fault is gone
  CHECK(!m.safe_requested());
  const std::array<NodeFault, 3> none{};
  const FrameReport& r = frame(m, 120, none, flat);
  CHECK(!r.safe_request);
  CHECK(r.mode == Mode::Duplex);               // A and B agree again
  CHECK(r.held_mask == 0U);
}

TFC_TEST(manager_duplex_digest_mismatch_is_unresolved_and_blames_nobody) {
  RedundancyManager m;
  bool saw_safe = false;
  for (int k = 0; k < 40; ++k) {
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);
    f[1].digest_xor = (k >= 20) ? 1U : 0U;
    const FrameReport& r = frame(m, k, f, smooth);
    saw_safe = saw_safe || r.safe_request;
  }
  CHECK(saw_safe);
  CHECK(!m.latched(0) && !m.latched(1));
  CHECK(m.counters().digest_flags > 0U);
}

TFC_TEST(manager_duplex_arbitration_can_be_disabled) {
  RedundancyConfig cfg;
  cfg.duplex_arbitration_factor = 0.0F;
  RedundancyManager m(cfg);
  for (int k = 0; k < 40; ++k) {
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);
    f[1].gyro_bias = (k == 30) ? 20.0F : 0.0F;
    const FrameReport& r = frame(m, k, f, smooth);
    if (k == 30) {
      CHECK(r.unresolved);
      CHECK(r.reason[0] == 0U && r.reason[1] == 0U);
    }
  }
}

TFC_TEST(manager_bus_alarm_follows_the_out_of_schedule_rate) {
  RedundancyManager m;
  const std::array<NodeFault, 3> none{};
  Frame babble;
  babble.id = 0x023;
  // 2 stray frames in a major frame: below the default limit of 3.
  m.begin_frame();
  m.on_frame(babble);
  m.on_frame(babble);
  CHECK(!m.end_frame().bus_alarm);
  // 5 stray frames: alarm, and the counter says how many.
  m.begin_frame();
  for (int i = 0; i < 5; ++i) {
    m.on_frame(babble);
  }
  const FrameReport& r = m.end_frame();
  CHECK(r.bus_alarm && r.out_of_schedule_in_frame == 5U);
  // A quiet frame clears it.
  CHECK(!frame(m, 2, none).bus_alarm);
  CHECK(m.counters().bus_alarm_frames == 1U);
  CHECK(m.counters().out_of_schedule == 7U);
}

TFC_TEST(manager_a_frame_that_misses_the_vote_costs_one_sample_not_two) {
  // Jitter: node 1's gyro for frame 20 leaves after the vote and is drained with frame 21. Frame 20
  // counts one missing sample; the late frame (sequence 20) and the on-time one (21) are not errors.
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (unsigned n = 0; n < 3; ++n) {
      const bool is_b = n == 1U;
      if (is_b && k == 21) {
        m.on_frame(pack_gyro(1, Vec3{{truth(20), -2.0F, 1.0F}}, 20));  // frame 20's gyro, a frame late
      }
      if (!(is_b && k == 20)) {
        m.on_frame(pack_gyro(static_cast<uint8_t>(n), Vec3{{truth(k), -2.0F, 1.0F}}, seq));
      }
      m.on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F + 0.001F * static_cast<float>(k % 7)}}, seq));
      m.on_frame(pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, 0x1234U}, seq));
    }
    const FrameReport& r = m.end_frame();
    CHECK(r.newly_latched == 0U);
    if (k == 20) {
      CHECK(r.reason[1] == reason::kMissing);
    }
    if (k == 21) {
      CHECK(r.reason[1] == 0U);  // no false sequence error on the late frame or the one after it
    }
  }
  CHECK(m.counters().missing == 1U);
  CHECK(m.counters().seq_bad == 0U);
}

TFC_TEST(manager_a_stalled_peer_flushing_two_late_frames_costs_one_sample_each) {
  // Node 1 stalls for two frames (20, 21) and flushes both, in order, with frame 22's own.
  RedundancyManager m;
  for (int k = 0; k < 60; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (unsigned n = 0; n < 3; ++n) {
      const bool is_b = n == 1U;
      const bool stalled = is_b && (k == 20 || k == 21);
      if (is_b && k == 22) {
        for (int late = 20; late <= 21; ++late) {  // the backlog, oldest first
          m.on_frame(pack_gyro(1, Vec3{{truth(late), -2.0F, 1.0F}}, static_cast<uint8_t>(late)));
        }
      }
      if (!stalled) {
        m.on_frame(pack_gyro(static_cast<uint8_t>(n), Vec3{{truth(k), -2.0F, 1.0F}}, seq));
      }
      m.on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F + 0.001F * static_cast<float>(k % 7)}}, seq));
      m.on_frame(pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, 0x1234U}, seq));
    }
    CHECK(m.end_frame().newly_latched == 0U);  // two missing samples are below 3-of-5
  }
  CHECK(m.counters().missing == 2U);
  CHECK(m.counters().seq_bad == 0U);
}
