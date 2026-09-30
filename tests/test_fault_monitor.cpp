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
