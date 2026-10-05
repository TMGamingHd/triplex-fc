// SPDX-License-Identifier: MIT
// Findings of the fault campaign (docs/FAULT_CAMPAIGN.md), each pinned by a regression test:
//   E1  the dwell after clear-disabled counted the frame of the command (readmission one frame early)
//   E12 total loss: with every node latched there was no reference to judge a probation against, so no
//       node could ever be readmitted without a reset (now: cohort probation)
//   E13 configuration was never validated (a zero or NaN tolerance silently disabled detection)
//   E14 every CAN id >= 0x500 was treated as "known", so a babbler on a high id was invisible
//   E15 no protection against single-event upsets in the manager's own critical state
#include <array>
#include <cmath>
#include <cstring>

#include "tfc/redundancy.hpp"
#include "ground.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace tfc {
// Reaches into the manager to simulate a single-event upset. Test code only; never linked into firmware.
struct ManagerTestAccess {
  static void set_state_primary(RedundancyManager& m, unsigned node, uint8_t raw) { m.st_[node].v_ = raw; }
  static void set_state_complement(RedundancyManager& m, unsigned node, uint8_t raw) { m.st_[node].n_ = raw; }
  static void set_state_consistent(RedundancyManager& m, unsigned node, uint8_t raw) { m.st_[node].v_ = raw; m.st_[node].n_ = static_cast<uint8_t>(~raw); }
  static void set_safe_consistent(RedundancyManager& m, uint8_t raw) { m.safe_.v_ = raw; m.safe_.n_ = static_cast<uint8_t>(~raw); }
  static void set_safe_complement(RedundancyManager& m, uint8_t raw) { m.safe_.n_ = raw; }
  static void set_safe_primary(RedundancyManager& m, uint8_t raw) { m.safe_.v_ = raw; }
  static void flip_config_tolerance(RedundancyManager& m) { m.cfg_.tol[0] *= 2.0F; }
  static void set_pending_count(RedundancyManager& m, unsigned n) { m.npending_ = n; }
  static void flip_backup_tolerance(RedundancyManager& m) { m.cfg_backup_.tol[0] *= 2.0F; }
  static void flip_arm_complement(RedundancyManager& m) { m.arm_code_.n_ = static_cast<uint8_t>(m.arm_code_.n_ ^ 0x04U); }
  static void flip_arm_timer(RedundancyManager& m) { m.arm_left_.v_ = static_cast<uint8_t>(m.arm_left_.v_ ^ 0x01U); }
  static void flip_cmd_counter(RedundancyManager& m) { m.cmd_ctr_.v_ = static_cast<uint8_t>(m.cmd_ctr_.v_ ^ 0x80U); }
  static void flip_cmd_have(RedundancyManager& m) { m.cmd_have_.n_ = static_cast<uint8_t>(m.cmd_have_.n_ ^ 0x01U); }
  static void set_failures(uint32_t& counter, uint32_t v) { counter = v; }
  static void set_sensor_state_consistent(RedundancyManager& m, unsigned k, uint8_t raw) { m.sensors_.st_[k].v_ = raw; m.sensors_.st_[k].n_ = static_cast<uint8_t>(~raw); }
  static void set_sensor_state_primary(RedundancyManager& m, unsigned k, uint8_t raw) { m.sensors_.st_[k].v_ = raw; }
};
}  // namespace tfc

namespace {

struct Node {
  bool present = true;
  float bias = 0.0F;
  uint16_t digest_xor = 0U;
};
using Nodes = std::array<Node, 3>;
const Nodes kNone{};  // a named object: passing a temporary would make the returned report look dangling to the compiler

float smooth(int k) { return 5.0F + 3.0F * std::sin(0.2F * static_cast<float>(k)); }

const FrameReport& step(RedundancyManager& m, int k, const Nodes& f) {
  m.begin_frame();
  const uint8_t seq = static_cast<uint8_t>(k);
  for (unsigned n = 0; n < 3; ++n) {
    if (!f[n].present) {
      continue;
    }
    const uint8_t id = static_cast<uint8_t>(n);
    m.on_frame(pack_gyro(id, Vec3{{smooth(k) + f[n].bias, -2.0F, 1.0F}}, seq));
    m.on_frame(pack_accel(id, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
    m.on_frame(pack_cmd(id, Command{0.5F, -0.25F, static_cast<uint16_t>(0x1234U ^ f[n].digest_xor)}, seq));
  }
  return m.end_frame();
}

struct Seen {
  std::array<int, 3> probation_started{{-1, -1, -1}};
  std::array<int, 3> probation_failed{{-1, -1, -1}};
  std::array<int, 3> reintegrated{{-1, -1, -1}};
  int max_on_probation = 0;
  void see(int k, const FrameReport& r) {
    int on = 0;
    for (unsigned n = 0; n < 3; ++n) {
      const uint8_t bit = static_cast<uint8_t>(1U << n);
      if ((r.probation_started & bit) != 0U && probation_started[n] < 0) probation_started[n] = k;
      if ((r.probation_failed & bit) != 0U && probation_failed[n] < 0) probation_failed[n] = k;
      if ((r.newly_reintegrated & bit) != 0U && reintegrated[n] < 0) reintegrated[n] = k;
      on += ((r.probation_mask >> n) & 1U) != 0U ? 1 : 0;
    }
    if (on > max_on_probation) max_on_probation = on;
  }
};

// All three nodes silent for frames [10, 15): every node latches at frame 12 (a bus-wide outage, or the last
// healthy node failing while the others were already out).
void all_silent(int k, Nodes& f) {
  if (k >= 10 && k < 15) {
    for (Node& n : f) n.present = false;
  }
}

}  // namespace

// ============================== E12: total loss ==============================
TFC_TEST(total_loss_every_node_latched_can_be_recovered_by_cohort_probation) {
  RedundancyManager m;
  Seen s;
  for (int k = 0; k < 600; ++k) {
    Nodes f{};
    all_silent(k, f);
    const FrameReport& r = step(m, k, f);
    s.see(k, r);
    if (k == 20) {
      CHECK(r.latched_mask == 0x7U && r.mode == Mode::Safe);  // no healthy node left
      for (unsigned n = 0; n < 3; ++n) CHECK(m.request_reintegration(n) == CommandResult::Accepted);
    }
    if (k == 100) {
      CHECK(r.probation_mask == 0x7U);  // all three at once: with no healthy reference there is no other way
      CHECK(r.mode == Mode::Safe);      // and nobody is voting yet
    }
  }
  for (unsigned n = 0; n < 3; ++n) {
    CHECK(s.probation_started[n] == 62);   // latched at 12 + the 50-frame dwell after a first transient-looking latch
    CHECK(s.reintegrated[n] == 162);       // 100 frames in which the cohort agreed
    CHECK(m.state(n) == NodeState::Healthy);
    CHECK(m.strikes(n) == 1U);
  }
  CHECK(s.max_on_probation == 3);
  CHECK(m.last_report().mode == Mode::Triplex);
  CHECK(m.counters().reintegrations == 3U);
}

TFC_TEST(total_loss_cohort_throws_out_a_member_that_disagrees_and_readmits_the_others) {
  RedundancyManager m;
  Seen s;
  for (int k = 0; k < 600; ++k) {
    Nodes f{};
    all_silent(k, f);
    f[1].bias = k >= 15 ? 3.0F : 0.0F;  // B comes back, but still wrong
    s.see(k, step(m, k, f));
    if (k == 20) {
      for (unsigned n = 0; n < 3; ++n) m.request_reintegration(n);
    }
    if (k == 63) {
      CHECK(m.state(1) == NodeState::Latched);  // thrown back the first frame it was compared
    }
  }
  CHECK(s.probation_failed[1] == 63);  // refused on the first frame it is compared (probation started at 62)
  CHECK(s.reintegrated[1] < 0 && m.state(1) != NodeState::Healthy);
  CHECK(s.reintegrated[0] == 162 && s.reintegrated[2] == 162);
  CHECK(m.state(0) == NodeState::Healthy && m.state(2) == NodeState::Healthy);
  CHECK(m.last_report().mode == Mode::Duplex);
}

TFC_TEST(total_loss_two_candidates_that_agree_are_readmitted_without_the_third) {
  RedundancyManager m;
  Seen s;
  for (int k = 0; k < 500; ++k) {
    Nodes f{};
    all_silent(k, f);
    s.see(k, step(m, k, f));
    if (k == 20) {
      m.request_reintegration(0);
      m.request_reintegration(2);
    }
  }
  CHECK(s.reintegrated[0] == 162 && s.reintegrated[2] == 162);
  CHECK(m.state(1) == NodeState::Latched);  // nobody asked for B
  CHECK(m.last_report().mode == Mode::Duplex);
}

TFC_TEST(total_loss_two_candidates_that_disagree_cannot_be_judged_and_nobody_is_readmitted) {
  RedundancyManager m;
  Seen s;
  for (int k = 0; k < 520; ++k) {
    Nodes f{};
    all_silent(k, f);
    f[1].bias = (k >= 15 && k < 400) ? 3.0F : 0.0F;
    s.see(k, step(m, k, f));
    if (k == 20) {
      m.request_reintegration(0);
      m.request_reintegration(1);
    }
    if (k == 350) {
      CHECK(m.state(0) == NodeState::Probation && m.state(1) == NodeState::Probation);  // neither blamed, neither admitted
    }
  }
  CHECK(s.probation_failed[0] < 0 && s.probation_failed[1] < 0);
  CHECK(s.reintegrated[0] == 499 && s.reintegrated[1] == 499);  // 100 agreeing frames from 400 on
}

TFC_TEST(total_loss_a_lone_candidate_has_no_reference_and_waits) {
  RedundancyManager m;
  for (int k = 0; k < 600; ++k) {
    Nodes f{};
    all_silent(k, f);
    step(m, k, f);
    if (k == 20) m.request_reintegration(0);
  }
  CHECK(m.state(0) == NodeState::Probation);  // one node cannot be judged against anyone
  CHECK(m.counters().reintegrations == 0U && m.counters().probation_failures == 0U);
}

TFC_TEST(with_a_healthy_node_probation_is_still_one_at_a_time) {
  RedundancyManager m;
  Seen s;
  for (int k = 0; k < 700; ++k) {
    Nodes f{};
    f[0].present = !(k >= 10 && k < 20);
    f[1].present = !(k >= 10 && k < 20);
    s.see(k, step(m, k, f));
    if (k == 30) {
      m.request_reintegration(0);
      m.request_reintegration(1);
    }
  }
  CHECK(s.max_on_probation == 1);  // C is healthy, so the shadow vote is the reference and the old rule holds
  CHECK(s.reintegrated[0] == 162 && s.reintegrated[1] == 262);
}

// ============================== E13: configuration validation ==============================
TFC_TEST(config_default_is_valid) {
  CHECK(validate_config(RedundancyConfig{}) == 0U);
  RedundancyManager m;
  CHECK(m.config_errors() == 0U);
}

TFC_TEST(config_every_invalid_field_is_replaced_by_its_default_and_reported) {
  const RedundancyConfig def{};
  const float nan = std::nanf("");
  const float inf = HUGE_VALF;
  struct Case {
    const char* what;
    uint32_t bit;
    RedundancyConfig cfg;
  };
  std::array<Case, 23> cases{};
  unsigned i = 0;
  auto add = [&](const char* what, uint32_t bit, auto mutate) {
    RedundancyConfig c;
    mutate(c);
    cases[i++] = Case{what, bit, c};
  };
  add("tol zero", cfgerr::kTolerance, [](RedundancyConfig& c) { c.tol[0] = 0.0F; });
  add("tol negative", cfgerr::kTolerance, [](RedundancyConfig& c) { c.tol[3] = -0.02F; });
  add("tol nan", cfgerr::kTolerance, [&](RedundancyConfig& c) { c.tol[6] = nan; });
  add("tol inf", cfgerr::kTolerance, [&](RedundancyConfig& c) { c.tol[7] = inf; });
  add("persist_m zero", cfgerr::kPersistence, [](RedundancyConfig& c) { c.persist_m = 0; });
  add("persist_m > n", cfgerr::kPersistence, [](RedundancyConfig& c) { c.persist_m = 6; });
  add("persist_n zero", cfgerr::kPersistence, [](RedundancyConfig& c) { c.persist_n = 0; });
  add("persist_n 33", cfgerr::kPersistence, [](RedundancyConfig& c) { c.persist_n = 33; });
  add("stuck_limit 0", cfgerr::kStuckLimit, [](RedundancyConfig& c) { c.stuck_limit = 0; });
  add("stuck_limit 1", cfgerr::kStuckLimit, [](RedundancyConfig& c) { c.stuck_limit = 1; });
  add("alpha_k negative", cfgerr::kAlpha, [](RedundancyConfig& c) { c.alpha_k = -0.1F; });
  add("alpha_k 1", cfgerr::kAlpha, [](RedundancyConfig& c) { c.alpha_k = 1.0F; });
  add("alpha_k nan", cfgerr::kAlpha, [&](RedundancyConfig& c) { c.alpha_k = nan; });
  add("alpha_threshold negative", cfgerr::kAlpha, [](RedundancyConfig& c) { c.alpha_threshold = -1.0F; });
  add("alpha_threshold nan", cfgerr::kAlpha, [&](RedundancyConfig& c) { c.alpha_threshold = nan; });
  add("alpha_threshold inf", cfgerr::kAlpha, [&](RedundancyConfig& c) { c.alpha_threshold = inf; });
  add("arbitration negative", cfgerr::kArbitration, [](RedundancyConfig& c) { c.duplex_arbitration_factor = -1.0F; });
  add("arbitration nan", cfgerr::kArbitration, [&](RedundancyConfig& c) { c.duplex_arbitration_factor = nan; });
  add("arbitration below 1", cfgerr::kArbitration, [](RedundancyConfig& c) { c.duplex_arbitration_factor = 0.5F; });
  add("probation 0", cfgerr::kLifeCycle, [](RedundancyConfig& c) { c.probation_frames = 0; });
  add("probation repeat 0", cfgerr::kLifeCycle, [](RedundancyConfig& c) { c.probation_frames_repeat = 0; });
  add("probation repeat shorter", cfgerr::kLifeCycle, [](RedundancyConfig& c) { c.probation_frames_repeat = 50; });
  add("physical strikes above normal", cfgerr::kLifeCycle, [](RedundancyConfig& c) { c.max_strikes_physical = 5; });
  CHECK(i == cases.size());
  for (const Case& c : cases) {
    CHECK((validate_config(c.cfg) & c.bit) != 0U);
    RedundancyManager m(c.cfg);
    CHECK((m.config_errors() & c.bit) != 0U);
    CHECK(validate_config(m.config()) == 0U);  // what the manager actually runs with is always valid
    if (c.bit == cfgerr::kTolerance) {
      for (unsigned ch = 0; ch < kVoteChannels; ++ch) CHECK(m.config().tol[ch] > 0.0F && std::isfinite(m.config().tol[ch]));
    }
    if (c.bit == cfgerr::kPersistence) {
      CHECK(m.config().persist_m == def.persist_m && m.config().persist_n == def.persist_n);
    }
    if (c.bit == cfgerr::kStuckLimit) CHECK(m.config().stuck_limit == def.stuck_limit);
    if (c.bit == cfgerr::kAlpha) CHECK(m.config().alpha_k == def.alpha_k && m.config().alpha_threshold == def.alpha_threshold);
    if (c.bit == cfgerr::kArbitration) CHECK(m.config().duplex_arbitration_factor == def.duplex_arbitration_factor);
    // And the sanitised manager still does its job: a biased node is isolated in three frames.
    int latched = -1;
    for (int k = 0; k < 60; ++k) {
      Nodes f{};
      f[2].bias = k >= 20 ? 3.0F : 0.0F;
      if ((step(m, k, f).newly_latched & 0x4U) != 0U && latched < 0) latched = k;
    }
    CHECK(latched == 22);
    CHECK(!m.safe_requested());
  }
}

TFC_TEST(config_legitimate_special_values_are_accepted) {
  RedundancyConfig c;
  c.alpha_threshold = 0.0F;                // leaky count off
  c.duplex_arbitration_factor = 0.0F;      // arbitration off
  c.bus_alarm_per_frame = 0U;              // alarm off
  c.persist_n = 32;
  c.persist_m = 32;
  c.max_strikes = 0;                       // never disable
  c.strike_window_frames = 0;
  c.startup_grace_frames = 100;
  CHECK(validate_config(c) == 0U);
}

// ============================== E14: known CAN ids ==============================
TFC_TEST(unknown_ids_above_the_simulator_range_count_as_out_of_schedule) {
  RedundancyManager m;
  m.begin_frame();
  Frame f;
  f.len = 8;
  // the simulator's range is 0x500 to 0x50F and the ground command is 0x510 (protocol v2): everything above, and the gaps below, is out of schedule
  for (uint32_t idv : {0x511U, 0x520U, 0x5FFU, 0x600U, 0x7FFU, 0x403U, 0x301U, 0x011U, 0x020U, 0x413U}) {
    f.id = idv;
    CHECK(!m.on_frame(f));
  }
  for (uint32_t idv : {id::kSync, id::kActOut, id::kHeartbeat, id::kHeartbeat + 2U, id::kSim, id::kSimRates, id::kSimLast, id::kState, id::kState + 2U}) {
    f.id = idv;
    CHECK(m.on_frame(f));
  }
  CHECK(m.counters().out_of_schedule == 10U);
  CHECK(m.end_frame().bus_alarm);  // ten of them in one frame
}

// ============================== E15: single-event upsets in the manager's own state ==============================
TFC_TEST(seu_a_flipped_node_state_is_detected_and_fails_safe_to_excluded) {
  for (unsigned node = 0; node < 3; ++node) {
    for (uint8_t raw : {uint8_t{0}, uint8_t{1}, uint8_t{2}, uint8_t{3}, uint8_t{4}, uint8_t{0x80}, uint8_t{0xFF}}) {
      RedundancyManager m;
      for (int k = 0; k < 20; ++k) step(m, k, kNone);
      const bool same = raw == 0U;  // writing the value it already holds changes nothing
      ManagerTestAccess::set_state_primary(m, node, raw);
      const FrameReport& r = step(m, 20, kNone);
      if (same) {
        // The complement no longer matches the primary only if the raw value differs; here it still does.
        CHECK(m.counters().integrity_faults == 0U);
        continue;
      }
      CHECK(m.counters().integrity_faults == 1U);
      CHECK((r.integrity_mask & 1U) != 0U);
      CHECK(m.state(node) == NodeState::Latched);  // excluded, recoverable by command: never silently trusted
      CHECK(((r.latched_mask >> node) & 1U) != 0U);
      for (unsigned other = 0; other < 3; ++other) {
        if (other != node) CHECK(m.state(other) == NodeState::Healthy);
      }
      step(m, 21, kNone);
      CHECK(m.counters().integrity_faults == 1U);  // repaired, so it is counted once
    }
  }
}

TFC_TEST(seu_a_flipped_complement_is_detected_too) {
  RedundancyManager m;
  for (int k = 0; k < 20; ++k) step(m, k, kNone);
  ManagerTestAccess::set_state_complement(m, 1, 0x55);
  step(m, 20, kNone);
  CHECK(m.counters().integrity_faults == 1U && m.state(1) == NodeState::Latched);
}

TFC_TEST(seu_a_flipped_safe_flag_fails_to_safe) {
  {  // spurious "not safe" -> intact? primary says 0, complement says 0: a 1->0 flip of the primary only
    RedundancyManager m;
    for (int k = 0; k < 40; ++k) {  // C dead, B's digest diverges: unattributable, Safe requested
      Nodes f{};
      f[2].present = k < 5;
      f[1].digest_xor = k >= 20 ? 1U : 0U;
      step(m, k, f);
    }
    CHECK(m.safe_requested());
    ManagerTestAccess::set_safe_primary(m, 0U);  // the flag is knocked down
    Nodes still_bad{};
    still_bad[2].present = false;
    still_bad[1].digest_xor = 1U;
    const FrameReport& r = step(m, 40, still_bad);
    CHECK(m.counters().integrity_faults == 1U && (r.integrity_mask & 2U) != 0U);
    CHECK(m.safe_requested() && r.safe_request && r.mode == Mode::Safe);  // fail-safe direction: Safe stays
  }
  {  // spurious "safe" in a healthy system: detected, Safe requested (not silently believed either way)
    RedundancyManager m;
    for (int k = 0; k < 20; ++k) step(m, k, kNone);
    ManagerTestAccess::set_safe_primary(m, 1U);
    const FrameReport& r = step(m, 20, kNone);
    CHECK(m.counters().integrity_faults == 1U);
    CHECK(r.safe_request);  // an upset in the flag that guards Safe is itself treated as a reason to be safe
  }
}

TFC_TEST(seu_a_changed_configuration_is_detected_and_requests_safe) {
  RedundancyManager m;
  for (int k = 0; k < 20; ++k) step(m, k, kNone);
  CHECK(m.counters().integrity_faults == 0U && !m.safe_requested());
  ManagerTestAccess::flip_config_tolerance(m);
  const FrameReport& r = step(m, 20, kNone);
  CHECK(m.counters().integrity_faults == 1U && (r.integrity_mask & 4U) != 0U);
  CHECK(r.safe_request);
  CHECK(m.config().tol[0] == RedundancyConfig{}.tol[0]);  // repaired from the second copy
  step(m, 21, kNone);
  CHECK(m.counters().integrity_faults == 1U);              // counted once
}

TFC_TEST(seu_a_healthy_run_never_trips_the_integrity_checks) {
  RedundancyManager m;
  for (int k = 0; k < 2000; ++k) {
    Nodes f{};
    f[1].bias = (k >= 100 && k < 110) ? 3.0F : 0.0F;
    step(m, k, f);
    if (k == 400) m.request_reintegration(1);
  }
  CHECK(m.counters().integrity_faults == 0U);
}

TFC_TEST(seu_a_corrupted_command_queue_length_is_never_used_as_a_loop_bound) {
  RedundancyManager m;
  for (int k = 0; k < 20; ++k) step(m, k, kNone);
  ManagerTestAccess::set_pending_count(m, 200U);
  const FrameReport& r = step(m, 20, kNone);
  CHECK(r.command_count == 0U && (r.integrity_mask & 8U) != 0U);
  CHECK(m.counters().invariant_violations == 1U && m.counters().integrity_faults == 1U);
}

// ============================== exact boundaries (found by mutation testing the suite) ==============================
TFC_TEST(boundary_bus_alarm_is_raised_at_exactly_the_configured_count) {
  for (unsigned n = 0; n <= 4U; ++n) {
    RedundancyManager m;  // default: 3 out-of-schedule frames in one 10 ms frame
    m.begin_frame();
    Frame f;
    f.len = 8;
    f.id = 0x020U;
    for (unsigned i = 0; i < n; ++i) (void)m.on_frame(f);
    CHECK(m.end_frame().bus_alarm == (n >= 3U));
  }
}

TFC_TEST(boundary_a_duplicate_of_a_frame_already_seen_is_not_a_late_frame) {
  PhaseTracker t;
  t.next_frame();
  CHECK(t.classify(10, 10) == FrameTiming::OnTime);
  t.next_frame();                                       // cycle 11: nothing on time
  CHECK(t.classify(10, 11) == FrameTiming::Bad);        // frame 10 again: a repeat, not the late 11
  PhaseTracker u;
  u.next_frame();
  CHECK(u.classify(10, 10) == FrameTiming::OnTime);
  u.next_frame();
  CHECK(u.classify(11, 11) == FrameTiming::OnTime);
  CHECK(u.classify(11, 11) == FrameTiming::Bad);        // the same number twice
}

TFC_TEST(boundary_the_dwell_restarts_the_frame_after_a_failed_probation) {
  RedundancyManager m;
  std::array<int, 8> starts{};
  unsigned n_starts = 0;
  int failed_at = -1;
  for (int k = 0; k < 400; ++k) {
    Nodes f{};
    f[1].present = !(k >= 10 && k < 15);  // B latches at 12; A and C stay healthy and are the reference
    f[1].bias = k == 63 ? 3.0F : 0.0F;     // B's first compared frame is wrong: probation (started at 62) fails at 63
    const FrameReport& r = step(m, k, f);
    if ((r.probation_started & 2U) != 0U && n_starts < starts.size()) starts[n_starts++] = k;
    if ((r.probation_failed & 2U) != 0U && failed_at < 0) failed_at = k;
    if (k == 20 || k == 64) (void)m.request_reintegration(1);
  }
  CHECK(failed_at == 63);
  CHECK(n_starts == 2U && starts[0] == 62 && starts[1] == 113);  // 50 whole frames after the failure, like after a latch
}

// ============================== E16: Duplex arbitration with a moving signal ==============================
// A pair of nodes (C is absent from the start) whose command moves `slew` degrees per frame; from frame `from` node
// B's command output is frozen at its last value (a software hang that keeps the frame alive).
namespace {
struct DuplexRun {
  int a_latched = -1;
  int b_latched = -1;
  bool safe = false;
  float worst_error = 0.0F;  // worst |output - truth| over the frames in which the output was not held
};

DuplexRun frozen_command_in_duplex(float slew, int from, int frames) {
  RedundancyManager m;
  DuplexRun out;
  for (int k = 0; k < frames; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (uint8_t n = 0; n < 2U; ++n) {
      const int kk = (n == 1U && k >= from) ? from - 1 : k;  // B repeats the command of frame from-1
      m.on_frame(pack_gyro(n, Vec3{{smooth(k), -2.0F, 1.0F}}, seq));  // live sensors: constant bytes would trip the stuck detector
      m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      m.on_frame(pack_cmd(n, Command{slew * static_cast<float>(kk), -0.25F, 0x1234U}, seq));
    }
    const FrameReport& r = m.end_frame();
    if (((r.newly_latched & 1U) != 0U) && out.a_latched < 0) out.a_latched = k;
    if (((r.newly_latched & 2U) != 0U) && out.b_latched < 0) out.b_latched = k;
    out.safe = out.safe || r.safe_request;
    if (k > from && (r.held_mask & (1U << kChPitch)) == 0U) {
      const float err = std::fabs(r.output[kChPitch] - slew * static_cast<float>(k));
      if (err > out.worst_error) out.worst_error = err;
    }
  }
  return out;
}
}  // namespace

TFC_TEST(duplex_a_frozen_command_is_blamed_on_the_frozen_node_when_the_signal_moves_fast) {
  // 0.05 deg per frame is 5 tolerances: the healthy node "jumps" away from the last agreed value every frame, while
  // the frozen node sits exactly on it. Continuity with a standstill blames the HEALTHY node (and trusts the frozen
  // one); continuity with the motion blames the node that stopped following it.
  for (float slew : {0.03F, 0.05F, 0.1F, 0.25F}) {  // the command is 16-bit: +-32.767 deg, 0.25 deg x 120 frames fits
    const DuplexRun r = frozen_command_in_duplex(slew, 60, 120);
    CHECK(r.a_latched < 0);                 // the healthy node is never isolated
    CHECK(r.b_latched >= 60 && r.b_latched <= 66);  // the frozen one is, within a few frames
    CHECK(r.worst_error <= 0.01F + 1e-4F);  // and no output ever follows the frozen command
  }
}

TFC_TEST(duplex_a_frozen_command_at_a_slow_slew_cannot_be_attributed_and_goes_safe_without_blaming_anyone) {
  // 0.015 deg per frame is between one and two tolerances: neither node can be singled out by continuity.
  const DuplexRun r = frozen_command_in_duplex(0.015F, 60, 120);
  CHECK(r.a_latched < 0 && r.b_latched < 0);
  CHECK(r.safe);
  CHECK(r.worst_error <= 0.01F + 1e-4F);
}

TFC_TEST(duplex_a_genuine_step_command_is_not_blamed_on_either_node) {
  // Both nodes follow a commanded step together: they agree, there is no miscompare and nothing to arbitrate.
  RedundancyManager m;
  for (int k = 0; k < 100; ++k) {
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (uint8_t n = 0; n < 2U; ++n) {
      m.on_frame(pack_gyro(n, Vec3{{smooth(k), -2.0F, 1.0F}}, seq));
      m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      m.on_frame(pack_cmd(n, Command{k < 50 ? 0.0F : 5.0F, -0.25F, 0x1234U}, seq));
    }
    const FrameReport& r = m.end_frame();
    CHECK(r.newly_latched == 0U || k < 3);  // only the absent C latches, at frame 2
    CHECK(!r.safe_request);
  }
}

// ============================== structural coverage: the paths the scenarios above do not reach ==============================
TFC_TEST(coverage_a_damaged_backup_configuration_is_rebuilt_from_the_intact_one) {
  RedundancyManager m;
  for (int k = 0; k < 20; ++k) (void)step(m, k, kNone);
  ManagerTestAccess::flip_backup_tolerance(m);
  const FrameReport& r = step(m, 20, kNone);
  CHECK((r.integrity_mask & 4U) != 0U && m.counters().integrity_faults == 1U);
  (void)step(m, 21, kNone);
  CHECK(m.counters().integrity_faults == 1U);  // repaired
  ManagerTestAccess::flip_config_tolerance(m);
  ManagerTestAccess::flip_backup_tolerance(m);  // both copies gone: the defaults are the last resort
  (void)step(m, 22, kNone);
  CHECK(m.counters().integrity_faults == 2U && m.config().tol[0] == RedundancyConfig{}.tol[0]);
  (void)step(m, 23, kNone);
  CHECK(m.counters().integrity_faults == 2U);
}

TFC_TEST(coverage_a_failure_counter_saturates_instead_of_wrapping) {
  uint32_t n = 0U;
  CHECK(!ensure(false, n) && n == 1U);
  CHECK(ensure(true, n) && n == 1U);
  ManagerTestAccess::set_failures(n, 0xFFFFFFFFU);
  CHECK(!ensure(false, n) && n == 0xFFFFFFFFU);
}

TFC_TEST(coverage_strikes_older_than_the_window_are_forgotten) {
  for (uint32_t window : {0U, 300U}) {
    RedundancyConfig cfg;
    cfg.strike_window_frames = window;
    RedundancyManager m(cfg);
    for (int k = 0; k < 1000; ++k) {
      Nodes f{};
      f[1].present = !((k >= 10 && k < 15) || (k >= 700 && k < 705));
      (void)step(m, k, f);
      if (k == 20) (void)m.request_reintegration(1);
      if (k == 400) CHECK(m.state(1) == NodeState::Healthy);
    }
    CHECK(m.strikes(1) == (window == 0U ? 2U : 1U));  // the whole run counts, or only the last 300 frames
  }
}

TFC_TEST(coverage_max_strikes_zero_never_disables) {
  RedundancyConfig cfg;
  cfg.max_strikes = 0U;
  cfg.max_strikes_physical = 0U;
  cfg.min_dwell_frames = 5U;
  cfg.probation_frames = 5U;
  cfg.probation_frames_repeat = 5U;
  RedundancyManager m(cfg);
  for (int k = 0; k < 1500; ++k) {
    Nodes f{};
    f[1].present = !(k % 100 >= 10 && k % 100 < 15);
    (void)step(m, k, f);
    if (k % 100 == 50) (void)m.request_reintegration(1);
  }
  CHECK(m.strikes(1) >= 10U && m.state(1) != NodeState::Disabled);
  CHECK(m.counters().nodes_disabled == 0U);
}

TFC_TEST(coverage_dwell_saturates_at_its_16_bit_limit) {
  RedundancyConfig cfg;
  cfg.min_dwell_frames = 0xFFFFU;
  cfg.min_dwell_frames_transient = 0xFFFFU;
  RedundancyManager m(cfg);
  int started = -1;
  for (int k = 0; k < 66000; ++k) {
    Nodes f{};
    f[1].present = !(k >= 10 && k < 15);
    if ((step(m, k, f).probation_started & 2U) != 0U && started < 0) started = k;
    if (k == 20) (void)m.request_reintegration(1);
  }
  CHECK(started == 12 + 65535);
}

TFC_TEST(coverage_the_failed_probation_counter_saturates) {
  RedundancyConfig cfg;
  cfg.min_dwell_frames = 1U;
  RedundancyManager m(cfg);
  for (int k = 0; k < 2000; ++k) {  // B is always wrong when compared: probation fails every time it starts
    Nodes f{};
    f[1].bias = k >= 10 ? 3.0F : 0.0F;
    (void)step(m, k, f);
    if (k >= 20) (void)m.request_reintegration(1);
  }
  CHECK(m.counters().probation_failures > 500U);  // many failures; attempts_ saturates at 255 without wrapping to 0
  CHECK(m.state(1) != NodeState::Healthy);
}

TFC_TEST(coverage_a_full_command_queue_drops_and_counts_the_excess) {
  RedundancyManager m;
  m.begin_frame();
  for (uint8_t i = 0; i < kMaxCommandsPerFrame + 2U; ++i) (void)m.on_frame(tfct::gcmd(GroundOp::Reintegrate, 1U, static_cast<uint8_t>(i + 1U)));
  const FrameReport& r = m.end_frame();
  CHECK(r.command_count == kMaxCommandsPerFrame);
  CHECK(m.counters().commands_bad == 2U);
}

TFC_TEST(coverage_queries_about_a_node_that_does_not_exist_are_answered_safely) {
  RedundancyManager m;
  CHECK(!m.seen(3) && !m.seen(99));
  CHECK(m.state(3) == NodeState::Disabled);  // out of range reads as the most restrictive state
  CHECK(m.strikes(3) == 0U);
  CHECK(!m.latched(3) && !m.permanent(3));
  CHECK(m.command(GroundOp::Reintegrate, 3) == CommandResult::RefusedBadNode);
  CHECK(m.command(GroundOp::Disable, 3) == CommandResult::RefusedBadNode);
  CHECK(m.command(GroundOp::ClearDisabled, 3) == CommandResult::RefusedBadNode);
  CHECK(m.command(static_cast<GroundOp>(77), 0) == CommandResult::RefusedBadOp);  // NOLINT(clang-analyzer-optin.core.EnumCastOutOfRange): a corrupt frame can carry any byte
  CHECK(std::strcmp(state_text(static_cast<NodeState>(9)), "?") == 0);
  CHECK(std::strcmp(op_text(77), "unknown-op") == 0);
  CHECK(std::strcmp(result_text(static_cast<CommandResult>(99)), "?") == 0);
}

TFC_TEST(coverage_a_corrupted_node_state_read_between_scrubs_reads_as_excluded) {
  RedundancyManager m;
  for (int k = 0; k < 10; ++k) (void)step(m, k, kNone);
  ManagerTestAccess::set_state_primary(m, 2, 7U);
  CHECK(m.state(2) == NodeState::Latched && m.latched(2));
}

TFC_TEST(coverage_the_bus_alarm_can_be_switched_off) {
  RedundancyConfig cfg;
  cfg.bus_alarm_per_frame = 0U;
  RedundancyManager m(cfg);
  m.begin_frame();
  Frame f;
  f.len = 8;
  f.id = 0x020U;
  for (int i = 0; i < 50; ++i) (void)m.on_frame(f);
  CHECK(!m.end_frame().bus_alarm && m.counters().out_of_schedule == 50U);
}

TFC_TEST(coverage_a_32_frame_persistence_window_still_requests_safe) {
  RedundancyConfig cfg;
  cfg.persist_m = 3;
  cfg.persist_n = 32;
  RedundancyManager m(cfg);
  for (int k = 0; k < 60; ++k) {  // C absent: Duplex; B's digest disagrees from 20 on: unattributable
    Nodes f{};
    f[2].present = k < 5;
    f[1].digest_xor = k >= 20 ? 1U : 0U;
    (void)step(m, k, f);
  }
  CHECK(m.safe_requested());
}

TFC_TEST(coverage_digest_odd_one_out_is_blamed_and_three_different_digests_blame_everyone) {
  for (unsigned odd = 0; odd < 3; ++odd) {  // one node's digest differs: it alone is blamed
    RedundancyManager m;
    int latched = -1;
    for (int k = 0; k < 40; ++k) {
      Nodes f{};
      f[odd].digest_xor = k >= 20 ? 1U : 0U;
      if ((step(m, k, f).newly_latched & (1U << odd)) != 0U && latched < 0) latched = k;
    }
    CHECK(latched == 22 && m.state(odd) == NodeState::Latched);
    for (unsigned other = 0; other < 3; ++other) {
      if (other != odd) CHECK(m.state(other) == NodeState::Healthy);
    }
  }
  // E17: all three differ. Nobody can be singled out, so nobody is isolated (it used to latch all three, a self-inflicted
  // total loss): the disagreement is unresolved, the output is held and Safe is requested for an operator to resolve.
  RedundancyManager m;
  const FrameReport* last = nullptr;
  for (int k = 0; k < 60; ++k) {
    Nodes f{};
    f[1].digest_xor = k >= 20 ? 1U : 0U;
    f[2].digest_xor = k >= 20 ? 2U : 0U;
    last = &step(m, k, f);
  }
  CHECK(last != nullptr && last->safe_request && last->mode == Mode::Safe && last->unresolved);
  CHECK(m.strikes(0) == 0U && m.strikes(1) == 0U && m.strikes(2) == 0U);
  CHECK(!m.latched(0) && !m.latched(1) && !m.latched(2));
}

TFC_TEST(coverage_total_loss_cohort_names_a_digest_outlier_and_cannot_judge_three_different_digests) {
  {
    RedundancyManager m;  // B comes back with a diverged digest while A and C agree: B is the odd one out
    Seen s;
    for (int k = 0; k < 400; ++k) {
      Nodes f{};
      all_silent(k, f);
      f[1].digest_xor = k >= 15 ? 1U : 0U;
      s.see(k, step(m, k, f));
      if (k == 20) {
        for (unsigned n = 0; n < 3; ++n) (void)m.request_reintegration(n);
      }
    }
    CHECK(s.probation_failed[1] == 63 && s.reintegrated[0] == 162 && s.reintegrated[2] == 162);
  }
  {
    RedundancyManager m;  // three different digests: no reference at all, nobody is judged
    Seen s;
    for (int k = 0; k < 400; ++k) {
      Nodes f{};
      all_silent(k, f);
      f[1].digest_xor = k >= 15 ? 1U : 0U;
      f[2].digest_xor = k >= 15 ? 2U : 0U;
      s.see(k, step(m, k, f));
      if (k == 20) {
        for (unsigned n = 0; n < 3; ++n) (void)m.request_reintegration(n);
      }
    }
    CHECK(s.probation_failed[0] < 0 && s.probation_failed[1] < 0 && s.probation_failed[2] < 0);
    CHECK(s.reintegrated[0] < 0 && m.state(0) == NodeState::Probation);
  }
}

TFC_TEST(coverage_sanitising_a_short_repeat_probation_keeps_it_at_least_as_long_as_the_first) {
  RedundancyConfig c;
  c.probation_frames = 400;  // longer than the default repeat (300)
  c.probation_frames_repeat = 50;
  CHECK((validate_config(c) & cfgerr::kLifeCycle) != 0U);
  RedundancyManager m(c);
  CHECK(m.config().probation_frames_repeat == 400U && validate_config(m.config()) == 0U);
  RedundancyConfig d;
  d.max_strikes = 2;
  d.max_strikes_physical = 5;
  RedundancyManager m2(d);
  CHECK(m2.config().max_strikes_physical == 2U);
}

TFC_TEST(coverage_every_state_operation_and_result_has_its_text) {
  CHECK(std::strcmp(state_text(NodeState::Healthy), "healthy") == 0 && std::strcmp(state_text(NodeState::Latched), "latched") == 0);
  CHECK(std::strcmp(state_text(NodeState::Probation), "probation") == 0 && std::strcmp(state_text(NodeState::Disabled), "disabled") == 0);
  CHECK(std::strcmp(op_text(static_cast<uint8_t>(GroundOp::Reintegrate)), "reintegrate") == 0);
  CHECK(std::strcmp(op_text(static_cast<uint8_t>(GroundOp::Disable)), "disable") == 0);
  CHECK(std::strcmp(op_text(static_cast<uint8_t>(GroundOp::ClearDisabled)), "clear-disabled") == 0);
  CHECK(std::strcmp(op_text(static_cast<uint8_t>(GroundOp::ClearSafe)), "clear-safe") == 0);
  const std::array<CommandResult, 8> all = {CommandResult::Accepted, CommandResult::AlreadyDone, CommandResult::RefusedDisabled,
                                           CommandResult::RefusedNotLatched, CommandResult::RefusedNotDisabled,
                                           CommandResult::RefusedBadNode, CommandResult::RefusedBadOp, CommandResult::RefusedNotArmed};
  for (std::size_t i = 0; i < all.size(); ++i) {
    for (std::size_t j = i + 1U; j < all.size(); ++j) CHECK(std::strcmp(result_text(all[i]), result_text(all[j])) != 0);  // all distinct
    CHECK(std::strcmp(result_text(all[i]), "?") != 0);
  }
  char buf[80];
  format_reasons(static_cast<uint8_t>(reason::kVote | reason::kDigest), buf, sizeof buf);
  CHECK(std::strcmp(buf, "vote disagreement + digest mismatch") == 0);
  format_reasons(0U, buf, sizeof buf);
  CHECK(std::strcmp(buf, "(none)") == 0);
  format_reasons(0xFFU, buf, 12U);  // truncation never overruns the buffer
  CHECK(std::strlen(buf) == 11U);
}

TFC_TEST(coverage_a_state_or_flag_that_is_self_consistent_but_out_of_range_is_still_caught) {
  RedundancyManager m;
  for (int k = 0; k < 10; ++k) (void)step(m, k, kNone);
  ManagerTestAccess::set_state_consistent(m, 1, 7U);  // value and complement agree, but 7 is not a node state
  CHECK(m.state(1) == NodeState::Latched);
  (void)step(m, 10, kNone);
  CHECK(m.counters().integrity_faults == 1U && m.state(1) == NodeState::Latched);
  ManagerTestAccess::set_safe_consistent(m, 5U);       // likewise for the Safe flag (must be 0 or 1)
  (void)step(m, 11, kNone);
  CHECK(m.counters().integrity_faults == 2U && m.safe_requested());
  m.clear_safe_request();
  ManagerTestAccess::set_safe_complement(m, 0x12U);    // a damaged flag reads as "requested" even before the next scrub
  CHECK(m.safe_requested());
}

TFC_TEST(coverage_a_sanely_configured_manager_reports_nodes_it_has_seen) {
  RedundancyManager m;
  CHECK(!m.seen(0));
  for (int k = 0; k < 5; ++k) (void)step(m, k, kNone);
  CHECK(m.seen(0) && m.seen(1) && m.seen(2) && !m.permanent(0));
  (void)m.command(GroundOp::Disable, 1);
  CHECK(m.permanent(1) && !m.permanent(0));
}

TFC_TEST(coverage_a_duplex_that_disagrees_from_the_first_frame_has_no_history_to_hold) {
  RedundancyManager m;
  const FrameReport* last = nullptr;
  for (int k = 0; k < 20; ++k) {  // C absent; A and B disagree on the command from frame 0: no last agreed value exists
    m.begin_frame();
    const uint8_t seq = static_cast<uint8_t>(k);
    for (uint8_t n = 0; n < 2U; ++n) {
      (void)m.on_frame(pack_gyro(n, Vec3{{smooth(k), -2.0F, 1.0F}}, seq));
      (void)m.on_frame(pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      (void)m.on_frame(pack_cmd(n, Command{n == 0U ? 0.5F : 2.5F, -0.25F, 0x1234U}, seq));
    }
    last = &m.end_frame();
  }
  CHECK(last != nullptr && last->safe_request && last->output[kChPitch] == 0.0F);  // nothing was ever agreed: the hold is zero
}

TFC_TEST(coverage_the_strike_counter_saturates_at_255) {
  RedundancyConfig cfg;
  cfg.max_strikes = 0U;
  cfg.max_strikes_physical = 0U;
  cfg.min_dwell_frames = 1U;
  cfg.probation_frames = 1U;
  cfg.probation_frames_repeat = 1U;
  RedundancyManager m(cfg);
  for (int k = 0; k < 12000; ++k) {
    Nodes f{};
    f[1].present = !(k % 40 >= 5 && k % 40 < 10);
    (void)step(m, k, f);
    if (k % 40 == 20) (void)m.request_reintegration(1);
  }
  CHECK(m.strikes(1) == 255U);  // 300 latches happened; the count stops at 255 instead of wrapping to 44
}

// ============================== SEU in the command path (ADR-015 + ADR-019) ==============================
TFC_TEST(seu_an_upset_in_an_armed_state_disarms_it_and_is_reported) {
  for (int which = 0; which < 2; ++which) {
    RedundancyManager m;
    for (int k = 0; k < 5; ++k) (void)step(m, k, kNone);
    (void)m.command(GroundOp::Disable, 1);
    m.begin_frame();
    (void)m.on_frame(tfct::gcmd(GroundOp::ClearDisabled, 1, 1, true));  // ARM
    (void)m.end_frame();
    if (which == 0) ManagerTestAccess::flip_arm_complement(m); else ManagerTestAccess::flip_arm_timer(m);
    m.begin_frame();
    (void)m.on_frame(tfct::gcmd(GroundOp::ClearDisabled, 1, 2));         // EXECUTE: the damaged arm is gone, so this is refused
    const FrameReport& r = m.end_frame();
    CHECK((r.integrity_mask & 16U) != 0U && m.counters().integrity_faults == 1U);
    CHECK(r.commands[0].result == CommandResult::RefusedNotArmed && m.state(1) == NodeState::Disabled);
  }
}

TFC_TEST(seu_an_upset_in_the_command_counter_forgets_the_history_and_is_reported) {
  for (int which = 0; which < 2; ++which) {
    RedundancyManager m;
    for (int k = 0; k < 5; ++k) (void)step(m, k, kNone);
    (void)step(m, 5, kNone);
    m.begin_frame();
    (void)m.on_frame(tfct::gcmd(GroundOp::Reintegrate, 0, 50));
    (void)m.end_frame();
    if (which == 0) ManagerTestAccess::flip_cmd_counter(m); else ManagerTestAccess::flip_cmd_have(m);
    m.begin_frame();
    (void)m.on_frame(tfct::gcmd(GroundOp::Reintegrate, 0, 10));  // older than 50: a replay if the history were intact
    const FrameReport& r = m.end_frame();
    CHECK((r.integrity_mask & 16U) != 0U && r.command_count == 1U);  // the history was damaged, so the next authentic command re-establishes it
    m.begin_frame();
    (void)m.on_frame(tfct::gcmd(GroundOp::Reintegrate, 0, 10));
    CHECK(m.end_frame().command_count == 0U && m.counters().commands_replayed == 1U);  // and replay protection works again from there
  }
}

TFC_TEST(seu_an_upset_in_a_sensor_channel_state_excludes_the_channel_and_is_reported) {
  RedundancyConfig cfg;
  cfg.sensor_split = true;
  RedundancyManager m(cfg);
  for (int k = 0; k < 10; ++k) (void)step(m, k, kNone);
  ManagerTestAccess::set_sensor_state_primary(m, 2U, static_cast<uint8_t>(NodeState::Healthy) ^ 1U);  // one bit flipped: its complement now disagrees
  CHECK(m.sensor_state(2) == NodeState::Latched);  // read before the scrub: a damaged state reads as excluded
  FrameReport r = step(m, 10, kNone);
  CHECK((r.integrity_mask & 1U) != 0U && m.sensor_state(2) == NodeState::Latched && r.sensor_latched_mask == 0x04U && r.latched_mask == 0U);
  ManagerTestAccess::set_sensor_state_consistent(m, 0U, 9U);  // a value that is no state at all
  r = step(m, 11, kNone);
  CHECK((r.integrity_mask & 1U) != 0U && m.sensor_state(0) == NodeState::Latched);
  CHECK(m.sensor_state(3U) == NodeState::Disabled && m.sensor_strikes(3U) == 0U);  // out of range
  RedundancyManager off;  // with the split off the sensor states are not looked at
  for (int k = 0; k < 5; ++k) (void)step(off, k, kNone);
  ManagerTestAccess::set_sensor_state_primary(off, 0U, 3U);
  CHECK((step(off, 5, kNone).integrity_mask & 1U) == 0U);
}
