// SPDX-License-Identifier: MIT
#include <cmath>

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
  // The numbers the design was tuned on (docs/decisions/DECISIONS.md ADR-013): 3-of-5 never fires on any of these.
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

// ---- hardening found by static analysis and the coding-standard check (docs/verification/FAULT_CAMPAIGN.md, E18) ----
TFC_TEST(popcount_matches_the_reference_for_every_bit_pattern_class) {
  CHECK(popcount32(0U) == 0U && popcount32(1U) == 1U && popcount32(0x80000000U) == 1U && popcount32(0xFFFFFFFFU) == 32U);
  CHECK(popcount32(0x55555555U) == 16U && popcount32(0xAAAAAAAAU) == 16U && popcount32(0x0000FFFFU) == 16U);
  uint32_t x = 12345U;
  for (int i = 0; i < 20000; ++i) {  // an LCG sweep against the compiler's own population count
    x = x * 1664525U + 1013904223U;
    CHECK(popcount32(x) == static_cast<unsigned>(__builtin_popcount(x)));
  }
  for (unsigned b = 0; b < 32U; ++b) CHECK(popcount32(1U << b) == 1U);
}

TFC_TEST(alpha_count_survives_nonsense_constants_without_overflow_or_undefined_behaviour) {
  const float nan = std::nanf("");
  const float inf = HUGE_VALF;
  {  // a NaN or negative threshold is "off", like 0
    AlphaCount a(0.9F, nan);
    AlphaCount b(0.9F, -5.0F);
    for (int i = 0; i < 100; ++i) CHECK(!a.update(true) && !b.update(true));
  }
  {  // an infinite or enormous threshold saturates and can never be reached
    AlphaCount a(0.9F, inf);
    AlphaCount b(0.9F, 1e30F);
    for (int i = 0; i < 200000; ++i) CHECK(!a.update(true) && !b.update(true));
    CHECK(a.score() > 1000.0F);  // it kept counting, it did not wrap
  }
  {  // a decay factor of 1 or more would make the score grow on good frames; it is held just below 1
    AlphaCount a(5.0F, 3.0F);
    AlphaCount b(inf, 3.0F);
    for (int i = 0; i < 10; ++i) {
      (void)a.update(true);
      (void)b.update(true);
    }
    for (int i = 0; i < 100000; ++i) {
      (void)a.update(false);
      (void)b.update(false);
    }
    CHECK(a.score() <= 11.0F && b.score() <= 11.0F);  // never above the ten bad frames it was given
  }
  {  // NaN or negative decay: the score is wiped by the first good frame
    AlphaCount a(nan, 3.0F);
    CHECK(!a.update(true) && !a.update(true) && !a.update(false));
    CHECK(a.score() == 0.0F);
  }
}

TFC_TEST(a_permanent_channel_ignores_reintegration_requests_and_its_probation_never_runs) {
  ChannelMonitor m(3, 5, 2, 1);  // the first latch is permanent
  for (int i = 0; i < 3; ++i) (void)m.update(true);
  CHECK(m.latched() && m.permanent());
  m.request_reintegration();
  for (int i = 0; i < 50; ++i) CHECK(!m.update(false));
  CHECK(m.latched() && m.permanent());
  ChannelMonitor h(3, 5, 2, 5);  // a healthy channel ignores a request too
  h.request_reintegration();
  CHECK(!h.latched());
}

TFC_TEST(the_stuck_detector_run_length_saturates_instead_of_wrapping) {
  StuckDetector d(20);
  bool flagged = false;
  for (int i = 0; i < 70000; ++i) flagged = d.update(42);
  CHECK(flagged);  // still flagged after 70,000 identical frames: the 16-bit run counter stopped at its maximum
  CHECK(!d.update(43));
}
