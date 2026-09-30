// SPDX-License-Identifier: MIT
// End-to-end software-in-the-loop scenarios for the redundancy logic: three noisy
// replicas, a fault injected mid-run, and hard assertions on (a) the voted output
// never leaving tolerance and (b) how many frames isolation takes. These are the
// same numbers the hardware campaign later re-measures on the bench.
#include <array>
#include <cmath>

#include "tfc/fault_monitor.hpp"
#include "tfc/voter.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

struct Lcg {  // deterministic noise, identical on every platform
  uint32_t s = 12345U;
  float next() {  // uniform in [-1, 1)
    s = s * 1664525U + 1013904223U;
    return static_cast<float>(s >> 8) / 8388608.0F - 1.0F;
  }
};

enum class Fault { None, StuckAt, BiasStep, Dropout, Wild };

struct Outcome {
  int latch_frame = -1;     // frame index on which the bad channel was latched
  float worst_error = 0.0F;  // max |voted - truth| over the whole run
  bool ever_no_majority = false;
};

// One replica set of a sensor. `bad` is the faulty channel. Each channel keeps a
// ChannelMonitor fed by vote disagreement (or missing data).
Outcome run(Fault fault, unsigned bad, int fault_frame, int frames = 400) {
  constexpr float kTol = 0.5F;
  Lcg rng;
  std::array<ChannelMonitor, 3> mon{ChannelMonitor(3, 5, 100, 3), ChannelMonitor(3, 5, 100, 3),
                                    ChannelMonitor(3, 5, 100, 3)};
  Outcome o;
  float stuck_value = 0.0F;
  for (int k = 0; k < frames; ++k) {
    const float truth = 10.0F * std::sin(0.05F * static_cast<float>(k));
    std::array<float, 3> x{};
    uint8_t valid = 0;
    for (unsigned i = 0; i < 3; ++i) {
      x[i] = truth + 0.05F * rng.next();
      if (!mon[i].latched()) valid = static_cast<uint8_t>(valid | (1U << i));
    }
    if (k == fault_frame - 1) stuck_value = x[bad];
    if (k >= fault_frame) {
      switch (fault) {
        case Fault::StuckAt: x[bad] = stuck_value; break;
        case Fault::BiasStep: x[bad] += 3.0F; break;
        case Fault::Dropout: valid = static_cast<uint8_t>(valid & ~(1U << bad)); break;
        case Fault::Wild: x[bad] = (k % 2 == 0) ? 1e6F : -1e6F; break;
        case Fault::None: break;
      }
    }
    const VoteResult r = vote3(x, valid, kTol);
    if (r.status == VoteStatus::NoMajority || r.status == VoteStatus::NoData) o.ever_no_majority = true;
    o.worst_error = std::fmax(o.worst_error, std::fabs(r.value - truth));
    for (unsigned i = 0; i < 3; ++i) {
      const bool missing = ((valid >> i) & 1U) == 0U && !mon[i].latched();
      const bool bad_now = ((r.disagree_mask >> i) & 1U) != 0U || missing;
      if (mon[i].update(bad_now) && i == bad && o.latch_frame < 0) o.latch_frame = k;
    }
  }
  return o;
}

}  // namespace

TFC_TEST(no_fault_no_latch_and_small_error) {
  const Outcome o = run(Fault::None, 0, 1000);
  CHECK(o.latch_frame < 0);
  CHECK(o.worst_error < 0.1F);
}

TFC_TEST(stuck_sensor_is_isolated_and_never_reaches_output) {
  // A frozen sensor only diverges once the truth moves away, so isolation takes longer
  // than for a step fault; assert an upper bound rather than an exact frame.
  for (unsigned bad = 0; bad < 3; ++bad) {
    const Outcome o = run(Fault::StuckAt, bad, 100);
    CHECK(o.latch_frame > 100);
    CHECK(o.latch_frame <= 100 + 40);
    CHECK(!o.ever_no_majority);
    CHECK(o.worst_error < 0.1F);  // output stays within sensor noise of truth
  }
}

TFC_TEST(bias_step_isolated_in_exactly_m_frames) {
  for (unsigned bad = 0; bad < 3; ++bad) {
    const Outcome o = run(Fault::BiasStep, bad, 100);
    CHECK(o.latch_frame == 100 + 2);  // 3-of-5: latches on the 3rd bad frame
    CHECK(o.worst_error < 0.1F);
  }
}

TFC_TEST(dropout_fail_silent_node_is_isolated_and_output_unaffected) {
  for (unsigned bad = 0; bad < 3; ++bad) {
    const Outcome o = run(Fault::Dropout, bad, 100);
    CHECK(o.latch_frame >= 100 && o.latch_frame <= 100 + 2);
    CHECK(o.worst_error < 0.1F);
  }
}

TFC_TEST(wild_values_isolated_without_output_excursion) {
  const Outcome o = run(Fault::Wild, 1, 50);
  CHECK(o.latch_frame == 50 + 2);
  CHECK(o.worst_error < 0.1F);
}

TFC_TEST(two_simultaneous_faults_fall_back_to_safe_handling) {
  // Second independent fault: with one channel already latched the vote is duplex; a
  // miscompare there must be surfaced (not silently averaged).
  const float ok = vote3({1.0F, 1.0F, 50.0F}, 0b011, 0.5F).status == VoteStatus::Duplex ? 1.0F : 0.0F;
  CHECK(ok == 1.0F);
  CHECK(vote3({1.0F, 9.0F, 50.0F}, 0b011, 0.5F).status == VoteStatus::DuplexMiscompare);
}
