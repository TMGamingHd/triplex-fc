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
  int freeze_k = -1;           // >= 0: gyro and accel repeat frame freeze_k's values (a stuck sensor)
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
    const int kk = f[n].freeze_k >= 0 ? f[n].freeze_k : k;
    Frame g = pack_gyro(static_cast<uint8_t>(n), Vec3{{tr(kk) + f[n].gyro_bias, -2.0F, 1.0F}}, seq);
    if (f[n].corrupt_gyro) {
      g.data[0] = static_cast<uint8_t>(g.data[0] ^ 1U);
    }
    m.on_frame(g);
    m.on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F + 0.001F * static_cast<float>(kk % 7)}}, seq));
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

// ======================= node life cycle: reintegration, probation, strikes (ADR-010) =======================
namespace {

float flat_truth(int) { return 5.0F; }

// Drives frames [from, to). `make(k, faults)` fills in this frame's per-node faults; `on(k, report)` sees each report.
template <class Make, class On>
void drive(RedundancyManager& m, int from, int to, float (*tr)(int), Make make, On on) {
  for (int k = from; k < to; ++k) {
    std::array<NodeFault, 3> f{};
    make(k, f);
    on(k, frame(m, k, f, tr));
  }
}
auto no_report = [](int, const FrameReport&) {};

// Records the frames on which `bit` of a mask selected by `pick` is set.
struct Events {
  int latched = -1;
  int probation_started = -1;
  int probation_failed = -1;
  int reintegrated = -1;
  int disabled = -1;
  uint8_t failure_reason = 0;
  int max_on_probation = 0;
  void see(int k, const FrameReport& r, unsigned node) {
    const uint8_t bit = static_cast<uint8_t>(1U << node);
    if ((r.newly_latched & bit) != 0U && latched < 0) latched = k;
    if ((r.probation_started & bit) != 0U && probation_started < 0) probation_started = k;
    if ((r.probation_failed & bit) != 0U && probation_failed < 0) {
      probation_failed = k;
      failure_reason = r.reason[node];
    }
    if ((r.newly_reintegrated & bit) != 0U && reintegrated < 0) reintegrated = k;
    if ((r.newly_disabled & bit) != 0U && disabled < 0) disabled = k;
    int n = 0;
    for (unsigned i = 0; i < 3; ++i) n += ((r.probation_mask >> i) & 1U) != 0U ? 1 : 0;
    if (n > max_on_probation) max_on_probation = n;
  }
};

}  // namespace

TFC_TEST(life_cycle_transient_fault_is_reintegrated_after_dwell_and_probation) {
  RedundancyManager m;
  Events ev;
  drive(m, 0, 400, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = (k >= 10 && k < 15) ? 3.0F : 0.0F; },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20) {
            CHECK(m.request_reintegration(1) == CommandResult::Accepted);
            CHECK(m.request_reintegration(1) == CommandResult::AlreadyDone);  // idempotent
          }
          if (k == 100) {
            CHECK(m.state(1) == NodeState::Latched && r.mode == Mode::Duplex);  // still serving its dwell
          }
          if (k == 250) {
            CHECK(m.state(1) == NodeState::Probation && r.mode == Mode::Duplex);  // excluded while on probation
            CHECK(r.probation_mask == 0x2U && r.valid_mask == 0x5U);
          }
        });
  CHECK(ev.latched == 12);
  CHECK(ev.probation_started == 212);  // latched at 12 + 200-frame dwell; the request at 20 was queued
  CHECK(ev.reintegrated == 312);       // 100 agreeing frames
  CHECK(ev.probation_failed < 0);
  CHECK(m.state(1) == NodeState::Healthy);
  CHECK(m.strikes(1) == 1U);           // the strike stays on record
  CHECK(m.last_report().mode == Mode::Triplex);
  CHECK(m.counters().reintegrations == 1U && m.counters().probations_started == 1U);
}

TFC_TEST(life_cycle_manual_policy_never_readmits_without_a_request) {
  RedundancyManager m;
  drive(m, 0, 700, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = (k >= 10 && k < 15) ? 3.0F : 0.0F; },
        no_report);
  CHECK(m.state(1) == NodeState::Latched);  // perfectly clean for 600 frames, still out
  CHECK(m.counters().probations_started == 0U);
}

TFC_TEST(life_cycle_a_still_broken_node_is_refused_by_the_shadow_vote) {
  // The hole the audit found: a node that is STILL biased used to pass "100 clean frames" because a
  // latched node was never compared with the vote, and then bounced back in (and in Duplex would have
  // pulled the system to Safe). Now probation compares it with the healthy nodes' voted output.
  RedundancyManager m;
  Events ev;
  drive(m, 0, 700, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = k >= 10 ? 3.0F : 0.0F; },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20 || k == 300 || k == 600) {
            m.request_reintegration(1);  // the operator keeps trying
          }
        });
  CHECK(ev.latched == 12);
  CHECK(ev.probation_started == 212);
  CHECK(ev.probation_failed == 213);   // refused on the very first frame it is compared
  CHECK(ev.failure_reason == reason::kVote);  // and the report says why (an operator needs that)
  CHECK(ev.reintegrated < 0);
  CHECK(m.state(1) != NodeState::Healthy);
  CHECK(m.counters().probation_failures >= 2U);
  CHECK(m.strikes(1) == 1U);           // failing probation is not a new strike: it was never readmitted
  CHECK(m.last_report().mode == Mode::Duplex);
}

TFC_TEST(life_cycle_one_disagreeing_frame_during_probation_sends_the_node_back) {
  RedundancyManager m;
  Events ev;
  drive(m, 0, 500, smooth,
        [](int k, std::array<NodeFault, 3>& f) {
          f[1].gyro_bias = ((k >= 10 && k < 15) || k == 260) ? 3.0F : 0.0F;  // one relapse at 260
        },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20) m.request_reintegration(1);
        });
  CHECK(ev.probation_started == 212);
  CHECK(ev.probation_failed == 260);
  CHECK(m.state(1) == NodeState::Latched);  // back to the dwell, and a new request is needed
  CHECK(ev.reintegrated < 0);
}

TFC_TEST(life_cycle_probation_pauses_when_there_is_no_reference_and_resumes) {
  // Two frames in which nobody healthy delivers: no reference, so the frame neither counts nor fails.
  RedundancyManager m;
  Events ev;
  drive(m, 0, 400, smooth,
        [](int k, std::array<NodeFault, 3>& f) {
          f[1].gyro_bias = (k >= 10 && k < 15) ? 3.0F : 0.0F;
          if (k == 250 || k == 251) {
            f[0].present = false;
            f[2].present = false;
          }
        },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20) m.request_reintegration(1);
        });
  CHECK(ev.probation_failed < 0);
  CHECK(ev.reintegrated == 314);  // 312 plus the two frames that did not count
  CHECK(m.last_report().mode == Mode::Triplex);
}

TFC_TEST(life_cycle_auto_policy_readmits_a_first_transient_latch_by_itself) {
  RedundancyConfig cfg;
  cfg.policy = ReintegrationPolicy::AutoTransient;
  RedundancyManager m(cfg);
  Events ev;
  drive(m, 0, 400, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].present = !(k >= 10 && k < 20); },
        [&](int k, const FrameReport& r) { ev.see(k, r, 1); });  // never any command
  CHECK(ev.latched == 12);
  CHECK(ev.probation_started == 212);
  CHECK(ev.reintegrated == 312);
}

TFC_TEST(life_cycle_auto_policy_never_readmits_a_digest_mismatch_or_a_repeat_offender) {
  RedundancyConfig cfg;
  cfg.policy = ReintegrationPolicy::AutoTransient;
  {  // digest mismatch = state divergence: not "transient-looking", needs a human
    RedundancyManager m(cfg);
    drive(m, 0, 700, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].digest_xor = (k >= 10 && k < 20) ? 1U : 0U; },
          no_report);
    CHECK(m.state(1) == NodeState::Latched && m.counters().probations_started == 0U);
  }
  {  // second strike: manual only, and a longer probation (300 frames)
    RedundancyManager m(cfg);
    Events ev;
    drive(m, 0, 1100, smooth,
          [](int k, std::array<NodeFault, 3>& f) {
            f[1].present = !((k >= 10 && k < 20) || (k >= 400 && k < 410));
          },
          [&](int k, const FrameReport& r) {
            ev.see(k, r, 1);
            if (k == 700) CHECK(m.state(1) == NodeState::Latched && m.strikes(1) == 2U);  // not auto
            if (k == 710) m.request_reintegration(1);
          });
    CHECK(ev.latched == 12);
    CHECK(m.counters().reintegrations == 2U);  // first automatic, second after the command
    // second latch ~402 (strike 2); request at 710 (dwell long served) -> probation at 710, 300 clean frames
    CHECK(m.last_report().mode == Mode::Triplex);
  }
}

TFC_TEST(life_cycle_third_strike_disables_the_node_until_a_maintenance_command) {
  RedundancyManager m;
  Events ev;
  CommandResult refused = CommandResult::Accepted;
  drive(m, 0, 1300, smooth,
        [](int k, std::array<NodeFault, 3>& f) {
          const bool bad = (k >= 10 && k < 15) || (k >= 400 && k < 405) || (k >= 1000 && k < 1005);
          f[1].gyro_bias = bad ? 3.0F : 0.0F;
        },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20 || k == 410) m.request_reintegration(1);
          if (k == 1100) {
            CHECK(m.state(1) == NodeState::Disabled && r.disabled_mask == 0x2U);
            refused = m.request_reintegration(1);
          }
        });
  CHECK(ev.disabled == 1002);                  // third latch = third strike
  CHECK(m.strikes(1) == 3U);
  CHECK(m.permanent(1) && m.latched(1));
  CHECK(refused == CommandResult::RefusedDisabled);
  CHECK(m.counters().nodes_disabled == 1U && m.counters().commands_refused == 1U);
  CHECK(m.last_report().mode == Mode::Duplex);

  CHECK(m.command(GroundOp::ClearDisabled, 1) == CommandResult::Accepted);  // maintenance
  CHECK(m.state(1) == NodeState::Latched && m.strikes(1) == 0U);
  CHECK(m.command(GroundOp::ClearDisabled, 1) == CommandResult::RefusedNotDisabled);
}

TFC_TEST(life_cycle_the_strike_limit_is_configurable_and_two_disables_on_the_second_latch) {
  RedundancyConfig cfg;
  cfg.max_strikes = 2;
  RedundancyManager m(cfg);
  Events ev;
  drive(m, 0, 700, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = ((k >= 10 && k < 15) || (k >= 400 && k < 405)) ? 3.0F : 0.0F; },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20) m.request_reintegration(1);
        });
  CHECK(ev.disabled == 402);
  CHECK(m.state(1) == NodeState::Disabled);
}

TFC_TEST(life_cycle_a_physical_cause_disables_at_the_second_strike) {
  // A stuck sensor is failed hardware. With the vehicle at rest only the stuck detector can see it.
  RedundancyManager m;
  Events ev;
  drive(m, 0, 700, flat_truth,
        [](int k, std::array<NodeFault, 3>& f) {
          if ((k >= 10 && k < 45) || (k >= 400 && k < 445)) f[1].freeze_k = 9;
        },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 60) m.request_reintegration(1);
        });
  CHECK(ev.latched > 20 && ev.latched <= 36);   // 20 identical frames, then 3-of-5
  CHECK(ev.reintegrated > 0 && ev.reintegrated < 400);  // it recovered once (strike 1)
  CHECK(ev.disabled > 400 && ev.disabled < 440);        // frozen again: second strike, physical cause
  CHECK(m.strikes(1) == 2U && m.state(1) == NodeState::Disabled);
}

TFC_TEST(life_cycle_old_strikes_are_forgotten_when_a_window_is_configured) {
  RedundancyConfig cfg;
  cfg.strike_window_frames = 300;
  RedundancyManager m(cfg);
  drive(m, 0, 520, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = ((k >= 10 && k < 15) || (k >= 400 && k < 405)) ? 3.0F : 0.0F; },
        no_report);
  CHECK(m.strikes(1) == 1U);  // the first latch (frame 12) is older than 300 frames at the second (402)
}

TFC_TEST(life_cycle_only_one_node_is_on_probation_at_a_time) {
  RedundancyManager m;
  Events e1;
  Events e2;
  drive(m, 0, 700, smooth,
        [](int k, std::array<NodeFault, 3>& f) {
          f[1].present = !(k >= 10 && k < 30);
          f[2].present = !(k >= 10 && k < 30);
        },
        [&](int k, const FrameReport& r) {
          e1.see(k, r, 1);
          e2.see(k, r, 2);
          if (k == 40) {
            m.request_reintegration(1);
            m.request_reintegration(2);
          }
          if (k == 100) CHECK(r.mode == Mode::Simplex);  // only node 0 is voting
        });
  CHECK(e1.max_on_probation == 1 && e2.max_on_probation == 1);
  CHECK(e1.probation_started == 212 && e1.reintegrated == 312);
  CHECK(e2.probation_started == 312 && e2.reintegrated == 412);  // starts when the first is readmitted
  CHECK(m.last_report().mode == Mode::Triplex);
}

TFC_TEST(life_cycle_ground_command_frames_drive_the_same_state_machine) {
  RedundancyManager m;
  uint8_t seq = 0;
  drive(m, 0, 400, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = (k >= 10 && k < 15) ? 3.0F : 0.0F; },
        [&](int k, const FrameReport& r) {
          if (k == 18) {  // a command that arrives as a frame is applied when that frame closes (k == 19)
            m.on_frame(pack_ground(GroundOp::Reintegrate, 1, seq++));
          }
          if (k == 19) {
            CHECK(r.command_count == 1U);
            CHECK(r.commands[0].op == static_cast<uint8_t>(GroundOp::Reintegrate) && r.commands[0].node == 1U);
            CHECK(r.commands[0].result == CommandResult::Accepted);
          }
        });
  CHECK(m.state(1) == NodeState::Healthy);  // same outcome as the direct call
  CHECK(m.counters().commands_accepted == 1U);
}

TFC_TEST(life_cycle_ground_frames_are_validated_and_refusals_are_reported) {
  RedundancyManager m;
  m.begin_frame();
  Frame bad = pack_ground(GroundOp::Disable, 1, 0);
  bad.data[7] = static_cast<uint8_t>(bad.data[7] ^ 1U);  // corrupted
  m.on_frame(bad);
  m.on_frame(pack_ground(GroundOp::Reintegrate, 1, 1));    // node 1 is healthy: nothing to reintegrate
  m.on_frame(pack_ground(GroundOp::Reintegrate, 9, 2));    // no such node
  Frame unknown = pack_ground(GroundOp::Disable, 1, 3);
  unknown.data[0] = 99;                                    // no such operation (CRC re-sealed)
  unknown.data[7] = crc8(unknown.data.data(), 7);
  m.on_frame(unknown);
  const FrameReport& r = m.end_frame();
  CHECK(r.command_count == 3U);
  CHECK(r.commands[0].result == CommandResult::RefusedNotLatched);
  CHECK(r.commands[1].result == CommandResult::RefusedBadNode);
  CHECK(r.commands[2].result == CommandResult::RefusedBadOp);
  CHECK(m.counters().commands_bad == 1U && m.counters().commands_refused == 3U);
  CHECK(m.state(1) == NodeState::Healthy);
}

TFC_TEST(life_cycle_operator_can_disable_a_node_and_bring_it_back_through_the_normal_path) {
  RedundancyManager m;
  Events ev;
  drive(m, 0, 700, smooth, [](int, std::array<NodeFault, 3>&) {},
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 2);
          if (k == 20) m.on_frame(pack_ground(GroundOp::Disable, 2, 0));
          if (k == 21) {
            CHECK((r.newly_disabled & 0x4U) != 0U && r.disabled_mask == 0x4U);
            CHECK(r.mode == Mode::Duplex && r.valid_mask == 0x3U);  // healthy and talking, but not voting
          }
          if (k == 50) m.on_frame(pack_ground(GroundOp::ClearDisabled, 2, 1));
          if (k == 52) m.request_reintegration(2);
        });
  CHECK(ev.probation_started == 251);  // the clear is applied in frame 51; the dwell counts the 200 frames after it
  CHECK(ev.reintegrated == 351);       // (it was 250/350: the frame of the clear itself used to count, E1)
  CHECK(m.state(2) == NodeState::Healthy && m.strikes(2) == 0U);
}

TFC_TEST(life_cycle_clear_safe_works_as_a_ground_command) {
  RedundancyManager m;
  for (int k = 0; k < 40; ++k) {  // C dead; B's digest diverges: unattributable, Safe requested
    std::array<NodeFault, 3> f{};
    f[2].present = (k < 5);
    f[1].digest_xor = (k >= 20) ? 1U : 0U;
    frame(m, k, f, smooth);
  }
  CHECK(m.safe_requested());
  m.on_frame(pack_ground(GroundOp::ClearSafe, 0, 0));
  const std::array<NodeFault, 3> none{};
  m.begin_frame();
  const FrameReport& r = m.end_frame();  // the command is applied at the end of this (empty) frame
  CHECK(r.commands[0].result == CommandResult::Accepted);
  (void)none;
}

TFC_TEST(life_cycle_probation_failure_names_a_digest_mismatch) {
  RedundancyManager m;
  Events ev;
  drive(m, 0, 300, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].digest_xor = (k >= 10) ? 1U : 0U; },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (k == 20) m.request_reintegration(1);
        });
  CHECK(ev.probation_failed > 0);
  CHECK(ev.failure_reason == reason::kDigest);  // values agree, the estimator fingerprint does not
}

// ======================= intermittent faults: the leaky count (ADR-013) =======================
TFC_TEST(intermittent_one_bad_frame_in_three_latches_with_its_own_reason) {
  RedundancyManager m;
  Events ev;
  uint8_t why = 0;
  drive(m, 0, 200, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].corrupt_gyro = (k >= 10 && (k - 10) % 3 == 0); },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (((r.newly_latched >> 1) & 1U) != 0U) why = r.reason[1];
        });
  CHECK(ev.latched == 22);                         // 3-of-5 never fires on this; the leaky count does
  CHECK((why & reason::kIntermittent) != 0U);
  CHECK(m.state(1) == NodeState::Latched);
}

TFC_TEST(intermittent_a_hard_burst_is_still_latched_by_the_window_on_the_same_frame_as_before) {
  RedundancyManager m;
  Events ev;
  uint8_t why = 0;
  drive(m, 0, 60, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].gyro_bias = (k >= 10) ? 3.0F : 0.0F; },
        [&](int k, const FrameReport& r) {
          ev.see(k, r, 1);
          if (((r.newly_latched >> 1) & 1U) != 0U) why = r.reason[1];
        });
  CHECK(ev.latched == 12);
  CHECK(why == reason::kVote);  // the window got there first: not labelled intermittent
}

TFC_TEST(intermittent_sparse_glitches_never_latch_a_healthy_node_even_over_a_long_run) {
  RedundancyManager m;
  for (int k = 0; k < 5000; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].corrupt_gyro = (k % 23 == 0);   // a glitch every 23 frames for 50 s
    CHECK(frame(m, k, f, smooth).newly_latched == 0U);
  }
  CHECK(m.state(1) == NodeState::Healthy);
}

TFC_TEST(intermittent_cause_is_physical_so_a_repeat_disables_and_auto_never_readmits) {
  {
    RedundancyConfig cfg;
    cfg.policy = ReintegrationPolicy::AutoTransient;
    RedundancyManager m(cfg);
    drive(m, 0, 700, smooth,
          [](int k, std::array<NodeFault, 3>& f) { f[1].corrupt_gyro = (k >= 10 && k < 60 && (k - 10) % 3 == 0); }, no_report);
    CHECK(m.state(1) == NodeState::Latched);  // clean for 600 frames, but an intermittent latch is not auto-readmitted
    CHECK(m.counters().probations_started == 0U);
  }
  {
    RedundancyManager m;
    Events ev;
    drive(m, 0, 1000, smooth,
          [](int k, std::array<NodeFault, 3>& f) {
            f[1].corrupt_gyro = ((k >= 10 && k < 60) || k >= 600) && (k % 3 == 0);
          },
          [&](int k, const FrameReport& r) {
            ev.see(k, r, 1);
            if (k == 70) m.request_reintegration(1);
          });
    CHECK(ev.reintegrated > 0 && ev.reintegrated < 600);
    CHECK(ev.disabled > 600);                       // latched again as intermittent: second strike, physical class
    CHECK(m.strikes(1) == 2U && m.state(1) == NodeState::Disabled);
  }
}

TFC_TEST(intermittent_detector_can_be_switched_off) {
  RedundancyConfig cfg;
  cfg.alpha_threshold = 0.0F;
  RedundancyManager m(cfg);
  drive(m, 0, 300, smooth, [](int k, std::array<NodeFault, 3>& f) { f[1].corrupt_gyro = (k >= 10 && (k - 10) % 3 == 0); },
        no_report);
  CHECK(m.state(1) == NodeState::Healthy);  // the old behaviour: 3-of-5 alone is blind to it
}

TFC_TEST(intermittent_score_is_cleared_when_a_node_is_readmitted) {
  RedundancyManager m;
  drive(m, 0, 500, smooth,
        [](int k, std::array<NodeFault, 3>& f) { f[1].corrupt_gyro = (k >= 10 && k < 50 && (k - 10) % 3 == 0); },
        [&](int k, const FrameReport&) { if (k == 60) m.request_reintegration(1); });
  CHECK(m.state(1) == NodeState::Healthy);
  // Straight after readmission one glitch must not re-latch it on leftover score.
  for (int k = 500; k < 540; ++k) {
    std::array<NodeFault, 3> f{};
    f[1].corrupt_gyro = (k == 505);
    CHECK(frame(m, k, f, smooth).newly_latched == 0U);
  }
}
