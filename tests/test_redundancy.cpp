// SPDX-License-Identifier: MIT
// RedundancyManager: one flight computer's per-frame consensus + FDIR step, driven with frames
// built by the real pack_* functions. Detection-time numbers here match the SIL scenarios and
// docs/REQUIREMENTS.md; the virtual-peers end-to-end tests check the same through the wire log.
#include <array>
#include <cmath>

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

float truth(int k) { return 5.0F + 0.125F * static_cast<float>(k % 16); }  // moves every frame

// Feeds one major frame from the three nodes and returns the manager's report.
const FrameReport& frame(RedundancyManager& m, int k, const std::array<NodeFault, 3>& f) {
  m.begin_frame();
  const uint8_t seq = static_cast<uint8_t>(k);
  for (unsigned n = 0; n < 3; ++n) {
    if (!f[n].present) {
      continue;
    }
    Frame g = pack_gyro(static_cast<uint8_t>(n), Vec3{{truth(k) + f[n].gyro_bias, -2.0F, 1.0F}}, seq);
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

TFC_TEST(manager_second_unattributable_fault_leaves_no_healthy_node) {
  // After B is latched (duplex A,C), a bias on C cannot be attributed: both survivors latch.
  RedundancyManager m;
  for (int k = 0; k < 40; ++k) {
    std::array<NodeFault, 3> f{};
    if (k >= 10) {
      f[1].gyro_bias = 3.0F;
    }
    if (k >= 30) {
      f[2].gyro_bias = 3.0F;
    }
    frame(m, k, f);
  }
  CHECK(m.last_report().mode == Mode::Safe);
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
