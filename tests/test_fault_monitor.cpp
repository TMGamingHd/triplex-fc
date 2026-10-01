// SPDX-License-Identifier: MIT
#include "tfc/fault_monitor.hpp"
#include "tfc_test.hpp"

using namespace tfc;

TFC_TEST(single_glitch_does_not_latch) {
  ChannelMonitor m(3, 5, 100, 3);  // 3-of-5
  for (int i = 0; i < 20; ++i) CHECK(!m.update(false));
  CHECK(!m.update(true));
  for (int i = 0; i < 20; ++i) CHECK(!m.update(false));
  CHECK(!m.latched());
}

TFC_TEST(two_glitches_in_window_do_not_latch) {
  ChannelMonitor m(3, 5, 100, 3);
  m.update(true);
  m.update(false);
  m.update(true);
  m.update(false);
  m.update(false);
  CHECK(!m.latched());
}

TFC_TEST(persistent_fault_latches_on_mth_bad_frame) {
  ChannelMonitor m(3, 5, 100, 3);
  CHECK(!m.update(true));
  CHECK(!m.update(true));
  CHECK(m.update(true));  // third bad frame latches
  CHECK(m.latched());
  CHECK(!m.update(true));  // latch event reported only once
}

TFC_TEST(old_faults_age_out_of_window) {
  ChannelMonitor m(3, 5, 100, 3);
  m.update(true);
  m.update(true);
  for (int i = 0; i < 5; ++i) m.update(false);  // both bad samples leave the window
  m.update(true);
  m.update(true);
  CHECK(!m.latched());
}

TFC_TEST(reintegration_needs_request_and_clean_run) {
  ChannelMonitor m(3, 5, 10, 3);
  for (int i = 0; i < 3; ++i) m.update(true);
  CHECK(m.latched());
  for (int i = 0; i < 50; ++i) m.update(false);  // no request -> stays out
  CHECK(m.latched());
  m.request_reintegration();
  for (int i = 0; i < 9; ++i) m.update(false);
  CHECK(m.latched());
  m.update(true);  // a bad frame restarts probation
  for (int i = 0; i < 9; ++i) m.update(false);
  CHECK(m.latched());
  m.update(false);
  CHECK(!m.latched());
}

TFC_TEST(repeat_offender_becomes_permanent) {
  ChannelMonitor m(2, 4, 3, 2);  // permanent after 2 latches
  for (int round = 0; round < 2; ++round) {
    m.update(true);
    m.update(true);
    CHECK(m.latched());
    m.request_reintegration();
    for (int i = 0; i < 3; ++i) m.update(false);
  }
  CHECK(m.permanent());
  CHECK(m.latched());
  m.request_reintegration();
  for (int i = 0; i < 100; ++i) m.update(false);
  CHECK(m.latched());  // no way back
}

TFC_TEST(stuck_detector_flags_frozen_sensor_only) {
  StuckDetector s(5);
  bool flagged = false;
  for (int i = 0; i < 200; ++i) flagged = flagged || s.update(i % 3);  // dithering
  CHECK(!flagged);
  StuckDetector t(5);
  int first = -1;
  for (int i = 0; i < 20; ++i) {
    if (t.update(1234) && first < 0) first = i;
  }
  CHECK(first == 5);  // 1 baseline sample + 5 repeats
}

TFC_TEST(mode_ladder) {
  CHECK(mode_from_healthy(3) == Mode::Triplex);
  CHECK(mode_from_healthy(2) == Mode::Duplex);
  CHECK(mode_from_healthy(1) == Mode::Simplex);
  CHECK(mode_from_healthy(0) == Mode::Safe);
}

// ---------------------- AlphaCount (leaky score for intermittent faults) ----------------------
namespace {

// First observation index (0-based) at which the score reaches the threshold, or -1. `bad(k)` says
// whether observation k is bad. Same parameters as the manager's default: K = 0.9, threshold 3.
template <class Bad>
int alpha_first_trip(Bad bad, int n = 400) {
  AlphaCount a(0.9F, 3.0F);
  for (int k = 0; k < n; ++k) {
    if (a.update(bad(k))) {
      return k;
    }
  }
  return -1;
}

}  // namespace

TFC_TEST(alpha_count_ignores_sparse_and_short_trouble) {
  CHECK(alpha_first_trip([](int k) { return k == 10; }) == -1);                          // one glitch
  CHECK(alpha_first_trip([](int k) { return k == 10 || k == 11; }) == -1);              // two-frame burst
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 20 == 0; }) == -1);   // 1 in 20
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 10 == 0; }) == -1);   // 1 in 10
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 5 == 0; }) == -1);    // 1 in 5
}

TFC_TEST(alpha_count_trips_on_a_three_frame_burst_like_three_of_five) {
  CHECK(alpha_first_trip([](int k) { return k >= 10 && k < 13; }) == 12);  // the third bad frame
}

TFC_TEST(alpha_count_catches_the_intermittent_patterns_three_of_five_cannot_see) {
  // The numbers the design was tuned on (docs/DECISIONS.md ADR-013): 3-of-5 never fires on any of these.
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 3 == 0; }) == 22);   // 1 bad in 3 (33%)
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 5 < 2; }) == 16);    // 2 bad in 5 (40%)
  CHECK(alpha_first_trip([](int k) { return k >= 10 && (k - 10) % 10 < 2; }) == 31);   // 2 bad in 10, paired
}

TFC_TEST(alpha_count_is_deterministic_fixed_point_and_resettable) {
  AlphaCount a(0.9F, 3.0F);
  AlphaCount b(0.9F, 3.0F);
  for (int k = 0; k < 200; ++k) {
    const bool bad = (k * 7 + 3) % 5 == 0;
    CHECK(a.update(bad) == b.update(bad));
    CHECK(a.score() == b.score());  // bit-identical, not merely close
  }
  a.update(true);
  a.update(true);
  CHECK(a.score() > 1.0F);
  a.reset();
  CHECK(a.score() == 0.0F);
}

TFC_TEST(alpha_count_threshold_zero_disables_it_and_a_long_run_cannot_overflow) {
  AlphaCount off(0.9F, 0.0F);
  for (int k = 0; k < 100; ++k) {
    CHECK(!off.update(true));
  }
  AlphaCount a(0.9F, 3.0F);
  for (int k = 0; k < 200000; ++k) {  // saturates instead of wrapping
    a.update(true);
  }
  CHECK(a.update(true));
  CHECK(a.score() > 3.0F);
  for (int k = 0; k < 200; ++k) {  // and recovers
    a.update(false);
  }
  CHECK(!a.update(false));
}
