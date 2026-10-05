// SPDX-License-Identifier: MIT
// The supervisor's decision logic (supervisor/include/sup/supervisor.hpp, commands.hpp; ADR-022, SUP-Lite): what it watches, what it does, and what it never does, against a small
// simulation of the four units it watches (three flight computers and ACT) that pulse FRAME and KICK, hang, boot, and obey NRST and the power relay.
#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include "sup/rtc.hpp"
#include "sup/supervisor.hpp"
#include "tfc_test.hpp"

namespace {

using namespace sup;

constexpr uint64_t kMs = 1000U;  // ticks per millisecond at 1 MHz
constexpr uint64_t kSecond = 1000U * kMs;

// One watched unit: frames every 10 ms (the period can be off by some ppm and the phase by some microseconds), a KICK 9 ms into the frame, a boot time after a reset or a power-cycle.
struct Sim {
  bool hung = false;        // stops sending until reset or power-cycled
  bool broken = false;      // does not come back after a reset or a power-cycle
  bool no_kick = false;     // sends FRAME pulses but never completes a frame (no KICK)
  bool restart_fixes = true;
  int64_t ppm = 0;
  int64_t phase_us = 0;
  uint64_t boot_us = 500U * kMs;
  // internal
  bool running = true;
  uint64_t boot_until = 0U;
  int64_t next_frame_milli = 0;  // thousandths of a tick
  uint64_t next_kick = 0U;
  uint32_t frames = 0U;
  uint32_t kicks = 0U;
  uint64_t frame_tick = 0U;
  uint64_t kick_tick = 0U;
  bool was_off = false;
};

struct World {
  Supervisor sup;
  std::array<Sim, kUnits> u{};
  Inputs in{};
  Outputs out{};
  Events ev{};
  Response resp = Response::Done;
  uint64_t now = 0U;
  std::array<uint32_t, kUnits> resets_seen{};
  std::array<uint32_t, kUnits> cycles_seen{};
  std::array<uint64_t, kUnits> reset_at{};   // the last tick at which a reset pulse started (from the events)
  std::array<uint64_t, kUnits> cycle_at{};
  uint64_t first_dead_at = 0U;
  bool saw_total_loss = false;
  bool saw_phase_bad = false;
  std::array<bool, kUnits> saw_period_bad{};
  std::array<bool, kUnits> saw_mission_bad{};
  MetRecord record{};
  bool saw_t0 = false;
  uint64_t t0_tick = 0U;
  std::array<uint64_t, kUnits> release_tick{};  // when each unit's NRST last went high (cold-start sequencing)
  std::array<bool, kUnits> prev_nrst{};

  explicit World(SupConfig cfg = {}) : sup(cfg) {
    for (unsigned i = 0; i < kUnits; ++i) {
      u[i].next_frame_milli = static_cast<int64_t>(1000 * 1000);  // every unit's frame starts on the same instant; the phase tests offset it
      u[i].next_kick = 1000U + 9000U;
    }
  }

  void sim_unit(unsigned i) {
    Sim& s = u[i];
    const bool off = out.nrst_low[i] || out.power_cut[i];
    if (off) {
      s.was_off = true;
      s.running = false;
      if (s.restart_fixes) {
        s.hung = false;
      }
      return;
    }
    if (s.was_off) {  // just released: it boots
      s.was_off = false;
      s.boot_until = now + s.boot_us;
      s.next_frame_milli = static_cast<int64_t>(s.boot_until) * 1000;
      s.next_kick = s.boot_until + 9000U;
      if (s.broken) {
        s.hung = true;
      }
    }
    if (s.hung || now < s.boot_until) {
      return;
    }
    s.running = true;
    if (!s.no_kick && s.next_kick != 0U && now >= s.next_kick && s.kick_tick < s.next_kick) {  // the kick of the previous frame first: it falls before the next frame's start
      ++s.kicks;
      s.kick_tick = s.next_kick;
    }
    if (s.next_frame_milli <= static_cast<int64_t>(now) * 1000) {
      ++s.frames;
      s.frame_tick = static_cast<uint64_t>(s.next_frame_milli / 1000 + s.phase_us);
      s.next_frame_milli += 10000000 + (s.ppm * 10);
      s.next_kick = s.frame_tick + 9000U;
    }
  }

  // Advance one millisecond, with an optional command line.
  void ms(const char* line = nullptr) {
    now += kMs;
    for (unsigned i = 0; i < kUnits; ++i) {
      sim_unit(i);
      in.unit[i].frames = u[i].frames;
      in.unit[i].frame_tick = u[i].frame_tick;
      in.unit[i].kicks = u[i].kicks;
      in.unit[i].kick_tick = u[i].kick_tick;
    }
    in.ticks = now;
    in.rtc_s = 5000U + (now / 1000000U);
    Command c;
    const Command* cp = nullptr;
    if (line != nullptr) {
      c = parse_command(line, std::strlen(line));
      cp = &c;
    }
    out = sup.step(in, cp, ev, resp);
    for (unsigned i = 0; i < kUnits; ++i) {
      if (ev.reset[i]) {
        ++resets_seen[i];
        reset_at[i] = now;
      }
      if (ev.cycle[i]) {
        ++cycles_seen[i];
        cycle_at[i] = now;
      }
      if (ev.dead[i] && first_dead_at == 0U) {
        first_dead_at = now;
      }
      saw_period_bad[i] = saw_period_bad[i] || ev.period_bad[i];
      saw_mission_bad[i] = saw_mission_bad[i] || ev.mission_bad[i];
      if (prev_nrst[i] && !out.nrst_low[i]) {
        release_tick[i] = now;
      }
      prev_nrst[i] = out.nrst_low[i];
    }
    saw_total_loss = saw_total_loss || ev.total_loss;
    saw_phase_bad = saw_phase_bad || ev.phase_bad;
    if (ev.t0) {
      saw_t0 = true;
      t0_tick = now;
      record = ev.record;
    }
  }

  void run_ms(uint64_t n) {
    for (uint64_t i = 0; i < n; ++i) {
      ms();
    }
  }
};

Command cmd(const char* line) { return parse_command(line, std::strlen(line)); }

TFC_TEST(sup_parse_every_command_unit_and_error) {
  CHECK(cmd("reset A").kind == Kind::Reset && cmd("reset A").unit == Unit::A && cmd("reset A").parse == Parse::Ok);
  CHECK(cmd("cycle b").kind == Kind::Cycle && cmd("cycle b").unit == Unit::B);
  CHECK(cmd("HOLD C").kind == Kind::Hold && cmd("HOLD C").unit == Unit::C);
  CHECK(cmd("release Act").kind == Kind::Release && cmd("release Act").unit == Unit::Act);
  CHECK(cmd("safe-now").kind == Kind::SafeNow && cmd("safe-clear").kind == Kind::SafeClear && cmd("launch").kind == Kind::Launch && cmd("scrub").kind == Kind::Scrub);
  CHECK(cmd("t0").kind == Kind::T0 && cmd("status").kind == Kind::Status && cmd("status").parse == Parse::Ok);
  CHECK(cmd("  reset \t B \r\n").kind == Kind::Reset && cmd("  reset \t B \r\n").unit == Unit::B);  // blanks, tabs and a line end around the words
  CHECK(cmd("").parse == Parse::Empty && cmd("   ").parse == Parse::Empty && cmd("").kind == Kind::Unknown);
  CHECK(cmd("explode A").parse == Parse::UnknownWord && cmd("resetx A").parse == Parse::UnknownWord && cmd("rese A").parse == Parse::UnknownWord);
  CHECK(cmd("reset").parse == Parse::MissingUnit && cmd("reset").kind == Kind::Unknown);
  CHECK(cmd("reset D").parse == Parse::BadUnit && cmd("reset AB").parse == Parse::BadUnit && cmd("reset AC").parse == Parse::BadUnit && cmd("reset ac").parse == Parse::BadUnit);
  CHECK(cmd("reset A B").parse == Parse::ExtraWords && cmd("status now").parse == Parse::ExtraWords && cmd("status now").kind == Kind::Unknown);
  const std::string longline(41U, 'a');
  CHECK(parse_command(longline.c_str(), longline.size()).parse == Parse::TooLong);
  const std::string fits = "reset  " + std::string(33U, ' ') + "A";  // exactly 41 characters
  CHECK(fits.size() == 41U && parse_command(fits.c_str(), fits.size()).parse == Parse::TooLong);
  const std::string ok40 = "reset " + std::string(33U, ' ') + "A";    // 40 characters
  CHECK(ok40.size() == 40U && parse_command(ok40.c_str(), ok40.size()).parse == Parse::Ok);
}

TFC_TEST(sup_a_power_up_that_finds_units_running_adopts_them_and_touches_nothing) {
  World w;
  w.run_ms(1500U);
  CHECK(w.sup.warm() && !w.ev.cold_start);
  for (unsigned i = 0; i < kUnits; ++i) {
    CHECK(w.sup.state(static_cast<Unit>(i)) == UnitState::Running && w.resets_seen[i] == 0U && !w.out.nrst_low[i] && !w.out.power_cut[i]);
  }
  CHECK(!w.out.safe_line && !w.out.t0_line && !w.saw_total_loss && !w.saw_phase_bad);
}

TFC_TEST(sup_a_cold_power_up_holds_every_unit_and_releases_them_in_order) {
  World w;
  for (auto& s : w.u) {
    s.running = false;
    s.hung = true;  // nothing is running when the supervisor comes up
    s.boot_us = 300U * kMs;
  }
  w.run_ms(90U);
  CHECK(!w.out.nrst_low[0] && !w.out.nrst_low[3]);  // while it listens it holds nothing
  w.run_ms(20U);
  CHECK(w.out.nrst_low[0] && w.out.nrst_low[1] && w.out.nrst_low[2] && w.out.nrst_low[3] && !w.sup.warm());
  uint64_t seen_cold = 0U;
  for (unsigned i = 0; i < 4000U; ++i) {
    w.ms();
    seen_cold += w.ev.cold_start ? 1U : 0U;
    for (auto& s : w.u) {
      if (!w.out.nrst_low[0]) {
        s.hung = false;  // once released they run
      }
    }
  }
  CHECK(seen_cold == 0U);  // (the event was in the step that decided: before this loop)
  // order ACT, A, B, C, 500 ms apart
  CHECK(w.release_tick[3] != 0U && w.release_tick[3] >= 100U * kMs + 500U * kMs && w.release_tick[0] > w.release_tick[3] && w.release_tick[1] > w.release_tick[0] && w.release_tick[2] > w.release_tick[1]);
  CHECK(w.release_tick[0] - w.release_tick[3] == 500U * kMs && w.release_tick[1] - w.release_tick[0] == 500U * kMs && w.release_tick[2] - w.release_tick[1] == 500U * kMs);
  CHECK(!w.out.nrst_low[0] && !w.out.nrst_low[1] && !w.out.nrst_low[2] && !w.out.nrst_low[3]);
}

TFC_TEST(sup_the_cold_start_event_is_raised_once_in_the_step_that_decides) {
  World w;
  for (auto& s : w.u) {
    s.hung = true;
  }
  bool cold = false;
  bool warm = false;
  for (unsigned i = 0; i < 300U; ++i) {
    w.ms();
    cold = cold || w.ev.cold_start;
    warm = warm || w.ev.warm_start;
  }
  CHECK(cold && !warm);
  World x;
  bool warm2 = false;
  for (unsigned i = 0; i < 300U; ++i) {
    x.ms();
    warm2 = warm2 || x.ev.warm_start;
  }
  CHECK(warm2 && !x.ev.cold_start);
}

TFC_TEST(sup_a_unit_that_stops_kicking_is_reset_after_three_frames_and_comes_back) {
  World w;
  w.run_ms(1500U);
  w.u[1].hung = true;
  const uint64_t t_hang = w.now;
  w.run_ms(200U);
  CHECK(w.resets_seen[1] == 1U && w.reset_at[1] > t_hang + 30U * kMs && w.reset_at[1] < t_hang + 60U * kMs);  // a kick is 10 ms apart: 30 ms of silence and the next step
  CHECK(w.sup.resets(Unit::B) == 1U && w.resets_seen[0] == 0U && w.resets_seen[2] == 0U && w.resets_seen[3] == 0U);
  w.run_ms(200U);
  w.run_ms(3000U);
  CHECK(w.sup.state(Unit::B) == UnitState::Running && w.resets_seen[1] == 1U);  // it came back, and was not judged during its boot
}

TFC_TEST(sup_the_reset_pulse_lasts_100_ms_and_the_grace_after_it_is_two_seconds) {
  World w;
  w.u[2].boot_us = 1800U * kMs;  // a slow boot, still inside the grace
  w.run_ms(1500U);
  w.u[2].hung = true;
  uint64_t low_from = 0U;
  uint64_t low_to = 0U;
  for (unsigned i = 0; i < 400U; ++i) {
    w.ms();
    if (w.out.nrst_low[2] && low_from == 0U) {
      low_from = w.now;
    }
    if (!w.out.nrst_low[2] && low_from != 0U && low_to == 0U) {
      low_to = w.now;
    }
  }
  CHECK(low_to - low_from == 100U * kMs);
  w.run_ms(4000U);
  CHECK(w.resets_seen[2] == 1U && w.sup.state(Unit::C) == UnitState::Running);  // 1.8 s of boot is not a failure
  World v;
  v.u[2].boot_us = 2600U * kMs;  // a boot longer than the grace: judged stopped, reset again
  v.run_ms(1500U);
  v.u[2].hung = true;
  v.run_ms(3500U);
  CHECK(v.resets_seen[2] >= 2U);
}

TFC_TEST(sup_three_resets_in_a_minute_become_a_power_cycle_and_two_cycles_make_it_dead) {
  World w;
  w.run_ms(1500U);
  w.u[0].restart_fixes = false;  // every restart brings it back hung
  w.u[0].hung = true;
  for (unsigned i = 0; i < 60000U && w.first_dead_at == 0U; ++i) {
    w.ms();
  }
  CHECK(w.resets_seen[0] == 6U && w.cycles_seen[0] == 2U && w.first_dead_at != 0U);  // 3 resets, a cycle, 3 resets, a cycle, and the next stop is the end
  CHECK(w.sup.dead(Unit::A) && w.sup.state(Unit::A) == UnitState::Held && w.out.nrst_low[0] && !w.out.power_cut[0]);
  const uint32_t r = w.resets_seen[0];
  w.run_ms(20000U);
  CHECK(w.resets_seen[0] == r && w.cycles_seen[0] == 2U && w.out.nrst_low[0]);  // held, and left alone
  CHECK(w.resets_seen[1] == 0U && w.sup.state(Unit::B) == UnitState::Running);  // the others were never touched
}

TFC_TEST(sup_the_power_cycle_opens_the_relay_for_half_a_second) {
  World w;
  w.run_ms(1500U);
  w.u[3].restart_fixes = false;
  w.u[3].hung = true;
  uint64_t cut_from = 0U;
  uint64_t cut_to = 0U;
  for (unsigned i = 0; i < 40000U && cut_to == 0U; ++i) {
    w.ms();
    if (w.out.power_cut[3] && cut_from == 0U) {
      cut_from = w.now;
    }
    if (!w.out.power_cut[3] && cut_from != 0U && cut_to == 0U) {
      cut_to = w.now;
    }
  }
  CHECK(cut_from != 0U && cut_to - cut_from == 500U * kMs && w.resets_seen[3] == 3U && w.cycles_seen[3] == 1U);
}

TFC_TEST(sup_old_resets_are_forgotten_so_a_rare_hang_is_never_escalated) {
  World w;
  w.run_ms(1500U);
  for (unsigned k = 0; k < 4U; ++k) {  // a hang every 70 s: each is one reset, never a cycle
    w.u[1].hung = true;
    w.run_ms(70000U);
  }
  CHECK(w.resets_seen[1] == 4U && w.cycles_seen[1] == 0U && !w.sup.dead(Unit::B));
}

TFC_TEST(sup_actions_on_different_units_are_staggered_and_a_total_loss_is_reported) {
  World w;
  w.run_ms(1500U);
  for (unsigned i = 0; i < 3U; ++i) {
    w.u[i].hung = true;  // all three flight computers at once
  }
  w.run_ms(1200U);
  CHECK(w.saw_total_loss);
  CHECK(w.resets_seen[0] == 1U && w.resets_seen[1] == 1U && w.resets_seen[2] == 1U);
  CHECK(w.reset_at[1] - w.reset_at[0] >= 200U * kMs && w.reset_at[2] - w.reset_at[1] >= 200U * kMs);  // one at a time
  CHECK(w.reset_at[0] < w.reset_at[1] && w.reset_at[1] < w.reset_at[2]);
  w.run_ms(4000U);
  CHECK(w.sup.state(Unit::A) == UnitState::Running && w.sup.state(Unit::B) == UnitState::Running && w.sup.state(Unit::C) == UnitState::Running);
}

TFC_TEST(sup_no_total_loss_while_one_flight_computer_still_kicks) {
  World w;
  w.run_ms(1500U);
  w.u[0].hung = true;
  w.u[1].hung = true;
  w.run_ms(1500U);
  CHECK(!w.saw_total_loss && w.resets_seen[0] == 1U && w.resets_seen[1] == 1U);
  CHECK(w.resets_seen[2] == 0U && w.sup.state(Unit::C) == UnitState::Running);  // the one that works is never touched
}

TFC_TEST(sup_the_operator_can_reset_cycle_hold_and_release_and_is_refused_when_it_makes_no_sense) {
  World w;
  w.run_ms(1500U);
  w.ms("reset B");
  CHECK(w.resp == Response::Done && w.resets_seen[1] == 1U && w.out.nrst_low[1]);
  w.run_ms(3000U);
  w.ms("cycle C");
  CHECK(w.resp == Response::Done && w.cycles_seen[2] == 1U && w.out.power_cut[2]);
  w.run_ms(3000U);
  w.ms("hold A");
  CHECK(w.resp == Response::Done && w.out.nrst_low[0] && w.sup.state(Unit::A) == UnitState::Held);
  w.run_ms(5000U);
  CHECK(w.resets_seen[0] == 0U && w.out.nrst_low[0]);  // a held unit stays held, and is not "reset"
  w.ms("reset A");
  CHECK(w.resp == Response::Refused);
  w.ms("cycle A");
  CHECK(w.resp == Response::Refused);
  w.ms("release B");
  CHECK(w.resp == Response::Refused);  // B is running: nothing to release
  w.ms("release A");
  CHECK(w.resp == Response::Done && !w.out.nrst_low[0] && w.sup.state(Unit::A) == UnitState::Grace);
  w.run_ms(4000U);
  CHECK(w.sup.state(Unit::A) == UnitState::Running && w.resets_seen[0] == 0U);
  w.ms("frobnicate");
  CHECK(w.resp == Response::Unknown);
  w.ms("reset");
  CHECK(w.resp == Response::Unknown);
  w.ms("status");
  CHECK(w.resp == Response::Done);
}

TFC_TEST(sup_release_brings_back_a_dead_unit_with_a_clean_record) {
  World w;
  w.run_ms(1500U);
  w.u[0].restart_fixes = false;
  w.u[0].hung = true;
  for (unsigned i = 0; i < 60000U && w.first_dead_at == 0U; ++i) {
    w.ms();
  }
  CHECK(w.sup.dead(Unit::A));
  w.u[0].restart_fixes = true;
  w.u[0].broken = false;
  w.ms("release A");
  CHECK(w.resp == Response::Done && !w.sup.dead(Unit::A));
  w.u[0].hung = false;
  w.run_ms(4000U);
  CHECK(w.sup.state(Unit::A) == UnitState::Running);
  const uint32_t resets_before = w.resets_seen[0];
  w.u[0].hung = true;  // a new hang: one reset again, not an immediate cycle or death (the record was cleared)
  w.run_ms(1500U);
  CHECK(w.resets_seen[0] == resets_before + 1U && w.cycles_seen[0] == 2U && !w.sup.dead(Unit::A));
}

TFC_TEST(sup_the_safe_line_follows_the_operator) {
  World w;
  w.run_ms(1500U);
  CHECK(!w.out.safe_line);
  w.ms("safe-now");
  CHECK(w.out.safe_line && w.resp == Response::Done);
  w.run_ms(100U);
  CHECK(w.out.safe_line);  // it stays until cleared
  w.ms("safe-clear");
  CHECK(!w.out.safe_line);
}

TFC_TEST(sup_a_frame_period_off_by_more_than_200_ppm_is_reported_and_nothing_is_done_about_it) {
  World w;
  w.u[0].ppm = 300;
  w.u[1].ppm = -300;
  w.u[2].ppm = 150;
  w.u[3].ppm = 0;
  w.run_ms(14000U);
  CHECK(w.saw_period_bad[0] && w.saw_period_bad[1] && !w.saw_period_bad[2] && !w.saw_period_bad[3]);
  CHECK(w.sup.period_ppm(Unit::A) > 280 && w.sup.period_ppm(Unit::A) < 320 && w.sup.period_ppm(Unit::B) < -280 && w.sup.period_ppm(Unit::B) > -320);
  CHECK(w.sup.period_ppm(Unit::C) > 130 && w.sup.period_ppm(Unit::C) < 170 && w.sup.period_ppm(Unit::Act) == 0);
  for (unsigned i = 0; i < kUnits; ++i) {
    CHECK(w.resets_seen[i] == 0U && w.sup.state(static_cast<Unit>(i)) == UnitState::Running);  // report only
  }
  char text[256];
  const std::size_t n = w.sup.status_text(text, sizeof text);
  CHECK(n > 0U && std::string(text).find("A running resets=0 cycles=0 period=+") == 0U && std::string(text).find("B running resets=0 cycles=0 period=-") != std::string::npos);
}

TFC_TEST(sup_two_flight_computers_whose_frames_start_more_than_200_us_apart_are_reported) {
  World w;
  w.u[1].phase_us = 150;
  w.run_ms(2000U);
  CHECK(!w.saw_phase_bad);
  World x;
  x.u[1].phase_us = 260;
  x.run_ms(2000U);
  CHECK(x.saw_phase_bad && x.resets_seen[1] == 0U);
  World y;  // the same offset the other way round the frame: 9.8 ms later is 200 us earlier
  y.u[1].phase_us = 9800;
  y.run_ms(2000U);
  CHECK(!y.saw_phase_bad);
  World wrap;  // A's frame starts at 100 us into a 10 ms frame and B's at 9 900 us: 9 800 us apart on the counter, 200 us apart on the frame
  wrap.u[0].phase_us = -900;
  wrap.u[1].phase_us = 8900;
  wrap.u[2].phase_us = -900;  // (C with A)
  wrap.run_ms(2000U);
  CHECK(!wrap.saw_phase_bad);
  World z;
  z.u[2].phase_us = -400;
  z.run_ms(2000U);
  CHECK(z.saw_phase_bad);
}

TFC_TEST(sup_launch_counts_down_ten_seconds_asserts_t0_and_latches_the_mission_clock) {
  World w;
  w.run_ms(1500U);
  w.ms("launch");
  CHECK(w.resp == Response::Done && w.ev.countdown_started && w.sup.counting_down() && !w.sup.launched());
  const uint64_t t_launch = w.now;
  w.ms("launch");
  CHECK(w.resp == Response::Refused);  // already counting
  w.run_ms(9000U);
  CHECK(!w.saw_t0 && !w.out.t0_line);
  w.run_ms(1200U);
  CHECK(w.saw_t0 && w.t0_tick - t_launch >= 10U * kSecond && w.t0_tick - t_launch <= 10U * kSecond + 2U * kMs);
  CHECK(w.sup.launched() && !w.sup.counting_down() && sup::record_valid(w.record) && w.record.t0_rtc_s >= 5000U + 11U);
  // the T0 line is a 50 ms pulse
  World v;
  v.run_ms(1500U);
  v.ms("t0");
  CHECK(v.resp == Response::Done && v.out.t0_line && v.saw_t0 && v.sup.launched());
  v.run_ms(40U);
  CHECK(v.out.t0_line);
  v.run_ms(20U);
  CHECK(!v.out.t0_line);
  v.ms("t0");
  CHECK(v.resp == Response::Refused);
  v.ms("launch");
  CHECK(v.resp == Response::Refused);  // after T-zero
  v.ms("scrub");
  CHECK(v.resp == Response::Refused);
  CHECK(v.sup.clock().launched() && v.sup.clock().source() == MetSource::Counted);
}

TFC_TEST(sup_a_scrub_cancels_the_countdown_and_a_later_launch_works) {
  World w;
  w.run_ms(1500U);
  w.ms("scrub");
  CHECK(w.resp == Response::Refused && !w.ev.scrubbed);  // nothing to scrub
  w.ms("launch");
  w.run_ms(5000U);
  w.ms("scrub");
  CHECK(w.resp == Response::Done && w.ev.scrubbed && !w.sup.counting_down());
  w.run_ms(8000U);
  CHECK(!w.saw_t0 && !w.sup.launched());
  w.ms("launch");
  CHECK(w.resp == Response::Done);
  w.run_ms(10100U);
  CHECK(w.saw_t0 && w.sup.launched());
}

TFC_TEST(sup_after_t_zero_a_flight_computer_whose_frame_count_disagrees_with_the_clock_is_flagged) {
  World w;
  w.run_ms(1500U);
  w.ms("t0");
  w.run_ms(20000U);
  CHECK(!w.saw_mission_bad[0] && !w.saw_mission_bad[1] && !w.saw_mission_bad[2]);  // healthy computers agree with the clock
  w.u[1].ppm = -5000;  // B's frame clock is 0.5 % slow: after a minute it is 300 frames behind the mission clock
  w.run_ms(70000U);
  CHECK(w.saw_mission_bad[1] && !w.saw_mission_bad[0] && !w.saw_mission_bad[2]);
  CHECK(w.resets_seen[1] == 0U);  // flagged, never reset
  World x;  // a computer that is 150 ppm slow is inside the allowance
  x.u[2].ppm = 150;
  x.run_ms(1500U);
  x.ms("t0");
  x.run_ms(120000U);
  CHECK(!x.saw_mission_bad[2]);
}

TFC_TEST(sup_after_its_own_reset_the_supervisor_resumes_the_mission_clock_and_refuses_a_second_launch) {
  World w;
  w.run_ms(1500U);
  w.ms("t0");
  w.run_ms(3000U);
  const MetRecord rec = w.record;
  Supervisor fresh;  // the supervisor reset: new object, ticks from zero, the RTC kept counting
  CHECK(fresh.restore(rec, 500U * kMs, rec.t0_rtc_s + 40U) && fresh.launched() && fresh.clock().source() == MetSource::Recovered);
  Inputs in{};
  in.ticks = 600U * kMs;
  Events ev;
  Response r = Response::Done;
  const Command c = cmd("launch");
  (void)fresh.step(in, &c, ev, r);
  CHECK(r == Response::Refused && !ev.countdown_started);
  Supervisor other;
  CHECK(!other.restore(MetRecord{}, 0U, 0U) && !other.launched());  // no record: not launched
}

TFC_TEST(sup_status_text_is_bounded_and_says_what_each_unit_is_doing) {
  World w;
  w.run_ms(1500U);
  w.ms("hold C");
  char big[512];
  const std::size_t n = w.sup.status_text(big, sizeof big);
  const std::string s(big);
  CHECK(n == s.size() && s.find("A running") == 0U && s.find("C held") != std::string::npos && s.find("ACT running") != std::string::npos && s.find("not launched") != std::string::npos);
  char tiny[10];
  const std::size_t m = w.sup.status_text(tiny, sizeof tiny);
  CHECK(m == 9U && tiny[9] == '\0');
  char one[1];
  CHECK(w.sup.status_text(one, 0U) == 0U && w.sup.status_text(one, 1U) == 0U && one[0] == '\0');
  w.ms("safe-now");
  w.ms("launch");
  CHECK(w.sup.status_text(big, sizeof big) > 0U && std::string(big).find("counting down SAFE-line-on") != std::string::npos);
  w.u[0].restart_fixes = false;
  w.u[0].hung = true;
  for (unsigned i = 0; i < 60000U && w.first_dead_at == 0U; ++i) {
    w.ms();
  }
  (void)w.sup.status_text(big, sizeof big);
  CHECK(std::string(big).find("A DEAD resets=") == 0U);
}

TFC_TEST(sup_a_unit_with_only_frames_or_only_kicks_is_adopted_but_judged_by_its_kicks) {
  World w;  // B sends FRAME pulses but never KICKs (it never completes a frame)
  w.u[1].no_kick = true;
  w.run_ms(1500U);
  CHECK(w.sup.warm() && w.resets_seen[1] >= 1U && w.resets_seen[0] == 0U);
  World k;  // C sends KICKs but no FRAME pulses: it is alive (it kicks), its period and phase are simply not judged
  for (unsigned i = 0; i < 1500U; ++i) {
    k.ms();
    k.in.unit[2].frames = 0U;
    k.u[2].frames = 0U;
  }
  CHECK(k.resets_seen[2] == 0U && !k.saw_phase_bad);
}

TFC_TEST(sup_a_command_while_it_is_still_listening_is_refused_and_a_unit_that_was_reset_after_t_zero_is_judged_from_its_new_count) {
  World w;
  w.ms("reset A");
  CHECK(w.resp == Response::Refused);
  w.ms("cycle B");
  CHECK(w.resp == Response::Refused);
  w.run_ms(1500U);
  w.ms("t0");
  w.run_ms(5000U);
  w.ms("reset B");  // after T-zero B restarts and its frame count starts again from zero
  w.u[1].frames = 0U;
  w.run_ms(60000U);
  CHECK(w.saw_mission_bad[1] && !w.saw_mission_bad[0]);  // a count behind the clock by the whole time it was down
}

TFC_TEST(sup_the_mission_field_saturates_like_sync_and_a_healthy_unit_is_not_flagged_at_the_limit) {
  Supervisor s;
  Inputs in{};
  Events ev;
  Response r = Response::Done;
  in.ticks = 1U * kMs;
  for (unsigned u = 0; u < kUnits; ++u) {
    in.unit[u].frames = 1U;
    in.unit[u].frame_tick = in.ticks;
    in.unit[u].kicks = 1U;
    in.unit[u].kick_tick = in.ticks;
  }
  (void)s.step(in, nullptr, ev, r);
  for (unsigned i = 0; i < 20U; ++i) {  // warm start
    in.ticks += (10U * kMs);
    for (unsigned u = 0; u < kUnits; ++u) {
      ++in.unit[u].frames;
      in.unit[u].frame_tick = in.ticks;
      ++in.unit[u].kicks;
      in.unit[u].kick_tick = in.ticks;
    }
    (void)s.step(in, nullptr, ev, r);
  }
  const Command t0 = cmd("t0");
  (void)s.step(in, &t0, ev, r);
  CHECK(s.launched());
  // 700 seconds later, in steps of 10 s, with every unit's counters keeping exact pace: the field saturates at 65 535 in the node and in the check alike
  bool bad = false;
  for (unsigned i = 0; i < 70U; ++i) {
    in.ticks += 10U * kSecond;
    for (unsigned u = 0; u < kUnits; ++u) {
      in.unit[u].frames += 1000U;
      in.unit[u].frame_tick = in.ticks;
      in.unit[u].kicks += 1000U;
      in.unit[u].kick_tick = in.ticks;
    }
    (void)s.step(in, nullptr, ev, r);
    bad = bad || ev.mission_bad[0] || ev.mission_bad[1] || ev.mission_bad[2];
  }
  CHECK(!bad);
}

TFC_TEST(sup_rtc_converts_the_seven_registers_to_seconds_since_2000) {
  using Regs = std::array<uint8_t, 7>;
  const RtcReading a = ds3231_seconds(Regs{0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00}, 0x00U);
  CHECK(a.valid && a.seconds == 0U);
  const RtcReading b = ds3231_seconds(Regs{0x56, 0x34, 0x12, 0x01, 0x05, 0x10, 0x26}, 0x08U);  // 2026-10-05 12:34:56 (other status bits do not matter)
  CHECK(b.valid && b.seconds == 844518896U);
  const RtcReading leap = ds3231_seconds(Regs{0x59, 0x59, 0x23, 0x01, 0x29, 0x02, 0x24}, 0x00U);  // 2024-02-29 23:59:59
  CHECK(leap.valid && leap.seconds == 762566399U);
  const RtcReading end99 = ds3231_seconds(Regs{0x59, 0x59, 0x23, 0x01, 0x31, 0x12, 0x99}, 0x00U);
  CHECK(end99.valid && end99.seconds == 3155759999ULL);
  const RtcReading c21 = ds3231_seconds(Regs{0x00, 0x00, 0x00, 0x01, 0x01, 0x83, 0x00}, 0x00U);  // the century bit: 2100-03-01 (2100 is not a leap year)
  CHECK(c21.valid && c21.seconds == 3160857600ULL);
  const Regs good{0x30, 0x15, 0x10, 0x03, 0x15, 0x06, 0x25};
  CHECK(ds3231_seconds(good, 0x00U).valid);
  CHECK(!ds3231_seconds(good, 0x80U).valid);                                        // the oscillator-stop flag: the time cannot be trusted
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x50, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // 12-hour mode is not ours (bit 6)
  CHECK(!ds3231_seconds(Regs{0x60, 0x15, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // 60 seconds
  CHECK(!ds3231_seconds(Regs{0x30, 0x60, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // 60 minutes
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x24, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // hour 24
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x00, 0x06, 0x25}, 0x00U).valid);  // day 0
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x32, 0x06, 0x25}, 0x00U).valid);  // day 32
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x31, 0x06, 0x25}, 0x00U).valid);  // 31 June
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x29, 0x02, 0x23}, 0x00U).valid);  // 29 February 2023
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x15, 0x00, 0x25}, 0x00U).valid);  // month 0
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x15, 0x13, 0x25}, 0x00U).valid);  // month 13
  CHECK(!ds3231_seconds(Regs{0x3A, 0x15, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // not BCD
  CHECK(!ds3231_seconds(Regs{0x30, 0x1A, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x1A, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x1A, 0x06, 0x25}, 0x00U).valid);
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x15, 0x0A, 0x25}, 0x00U).valid);
  CHECK(!ds3231_seconds(Regs{0x30, 0x15, 0x10, 0x03, 0x15, 0x06, 0xA5}, 0x00U).valid);
  CHECK(!ds3231_seconds(Regs{0xB0, 0x15, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // a stray bit 7 in the seconds
  CHECK(!ds3231_seconds(Regs{0x30, 0x95, 0x10, 0x03, 0x15, 0x06, 0x25}, 0x00U).valid);  // and in the minutes
}

}  // namespace

// ============================== the override sense lines (HWO-005, HWO-007, G5) ==============================
TFC_TEST(overrides_a_line_counts_only_after_it_has_held_its_level_for_the_debounce_time) {
  OverrideSense s(0x3FU, 50U * kMs);
  CHECK(s.sample(0x08U, 1000U * kMs) == 0U && s.engaged() == 0U);  // first seen: not yet
  CHECK(s.sample(0x08U, 1049U * kMs) == 0U && s.engaged() == 0U);  // 49 ms
  CHECK(s.sample(0x08U, 1050U * kMs) == 0x08U && s.engaged() == 0x08U);  // 50 ms: counts, and the change is reported once
  CHECK(s.sample(0x08U, 1100U * kMs) == 0U && s.engaged() == 0x08U);
  CHECK(s.sample(0x00U, 1200U * kMs) == 0U && s.engaged() == 0x08U);  // released: the same wait
  CHECK(s.sample(0x00U, 1249U * kMs) == 0U && s.engaged() == 0x08U);
  CHECK(s.sample(0x00U, 1250U * kMs) == 0x08U && s.engaged() == 0U);
}

TFC_TEST(overrides_a_glitch_shorter_than_the_debounce_is_ignored_whichever_way_it_goes_and_each_line_has_its_own_clock) {
  OverrideSense s(0x3FU, 50U * kMs);
  (void)s.sample(0x01U, 100U * kMs);
  (void)s.sample(0x00U, 130U * kMs);  // a 30 ms blip
  (void)s.sample(0x00U, 300U * kMs);
  CHECK(s.engaged() == 0U && s.tested() == 0U);
  (void)s.sample(0x01U, 400U * kMs);
  (void)s.sample(0x01U, 450U * kMs);
  CHECK(s.engaged() == 0x01U);
  (void)s.sample(0x00U, 500U * kMs);
  (void)s.sample(0x01U, 520U * kMs);  // a release blip while engaged
  (void)s.sample(0x01U, 700U * kMs);
  CHECK(s.engaged() == 0x01U && s.tested() == 0U);
  (void)s.sample(0x03U, 800U * kMs);  // line 1 starts later than line 0 is steady
  (void)s.sample(0x03U, 840U * kMs);
  CHECK(s.engaged() == 0x01U);
  (void)s.sample(0x03U, 850U * kMs);
  CHECK(s.engaged() == 0x03U);
}

TFC_TEST(overrides_a_line_that_is_not_fitted_or_not_a_line_is_ignored) {
  OverrideSense s(0x0AU, 10U * kMs);  // only lines 1 and 3 exist
  (void)s.sample(0xFFU, 100U * kMs);
  CHECK(s.sample(0xFFU, 200U * kMs) == 0x0AU && s.engaged() == 0x0AU);
  CHECK(s.fitted() == 0x0AU && s.untested() == 0x0AU);
  OverrideSense wide(0xFFU, 10U * kMs);  // only six lines exist
  CHECK(wide.fitted() == kOverrideMask);
  OverrideSense none(0U, 10U * kMs);
  (void)none.sample(0xFFU, 100U * kMs);
  (void)none.sample(0xFFU, 200U * kMs);
  CHECK(none.engaged() == 0U && none.untested() == 0U);
}

TFC_TEST(overrides_an_override_is_tested_when_it_has_been_seen_engaged_and_then_released_in_this_session) {
  OverrideSense s(0x07U, 10U * kMs);
  (void)s.sample(0x01U, 0U);
  (void)s.sample(0x01U, 20U * kMs);
  CHECK(s.tested() == 0U && s.untested() == 0x07U);  // engaged only: it has not been seen to release
  (void)s.sample(0x00U, 100U * kMs);
  (void)s.sample(0x00U, 120U * kMs);
  CHECK(s.tested() == 0x01U && s.untested() == 0x06U);
  (void)s.sample(0x00U, 200U * kMs);  // a line that was never engaged is not tested by being released
  CHECK(s.tested() == 0x01U);
  (void)s.sample(0x01U, 300U * kMs);
  (void)s.sample(0x01U, 320U * kMs);  // engaging again does not undo the test
  CHECK(s.tested() == 0x01U);
}

TFC_TEST(overrides_the_acknowledgement_covers_what_is_engaged_now_and_ends_when_that_line_is_released) {
  OverrideSense s(0x3FU, 10U * kMs);
  (void)s.sample(0x08U, 0U);
  (void)s.sample(0x08U, 20U * kMs);
  CHECK(s.unacknowledged() == 0x08U);
  s.acknowledge();
  CHECK(s.unacknowledged() == 0U);
  (void)s.sample(0x28U, 100U * kMs);  // a second one engaged later is not covered
  (void)s.sample(0x28U, 120U * kMs);
  CHECK(s.engaged() == 0x28U && s.unacknowledged() == 0x20U);
  (void)s.sample(0x20U, 200U * kMs);  // the first is released, and engaged again: it needs its own acknowledgement
  (void)s.sample(0x20U, 220U * kMs);
  (void)s.sample(0x28U, 300U * kMs);
  (void)s.sample(0x28U, 320U * kMs);
  CHECK(s.unacknowledged() == 0x28U);
}

TFC_TEST(overrides_a_launch_is_refused_while_an_override_is_engaged_that_nobody_acknowledged_and_goes_ahead_after_override_ok) {
  SupConfig cfg;
  cfg.override_fitted = 0x3FU;
  World w(cfg);
  w.run_ms(300U);
  w.in.overrides = 0x08U;  // H4 INJECTOR-DISARM left open
  w.run_ms(60U);
  CHECK(w.ev.overrides_changed == 0U);  // (reported once, at the step it counted)
  w.ms("launch");
  CHECK(w.resp == Response::Refused && w.ev.launch_blocked == 0x08U && !w.sup.counting_down());
  w.ms("t0");
  CHECK(w.resp == Response::Refused && w.ev.launch_blocked == 0x08U && !w.sup.launched());
  w.ms("override-ok");
  CHECK(w.resp == Response::Done);
  w.ms("launch");
  CHECK(w.resp == Response::Done && w.ev.countdown_started && w.ev.launch_blocked == 0U && w.sup.counting_down());
  CHECK(w.ev.launch_untested == 0x3FU);  // and none has been tested this session
}

TFC_TEST(overrides_the_change_is_reported_at_the_step_it_counts_and_a_release_ends_the_block) {
  SupConfig cfg;
  cfg.override_fitted = 0x3FU;
  World w(cfg);
  w.run_ms(300U);
  w.in.overrides = 0x02U;
  uint8_t seen = 0U;
  for (unsigned i = 0; i < 80U; ++i) {
    w.ms();
    seen = static_cast<uint8_t>(seen | w.ev.overrides_changed);
  }
  CHECK(seen == 0x02U);
  w.in.overrides = 0x00U;
  w.run_ms(80U);
  w.ms("launch");
  CHECK(w.resp == Response::Done && w.ev.launch_blocked == 0U);
  CHECK(w.ev.launch_untested == 0x3DU);  // H2 was engaged and released: tested; the others not
}

TFC_TEST(overrides_without_fitted_lines_the_sense_inputs_change_nothing_and_status_says_nothing_about_them) {
  World w;  // override_fitted = 0
  w.run_ms(300U);
  w.in.overrides = 0xFFU;
  w.run_ms(100U);
  w.ms("launch");
  CHECK(w.resp == Response::Done && w.ev.launch_blocked == 0U && w.ev.launch_untested == 0U && w.ev.overrides_changed == 0U);
  char text[400];
  (void)w.sup.status_text(text, sizeof text);
  CHECK(std::string(text).find("overrides") == std::string::npos);
}

TFC_TEST(overrides_the_status_shows_what_is_engaged_and_what_is_untested_when_lines_are_fitted) {
  SupConfig cfg;
  cfg.override_fitted = 0x3FU;
  World w(cfg);
  w.run_ms(300U);
  w.in.overrides = 0x04U;
  w.run_ms(80U);
  char text[400];
  (void)w.sup.status_text(text, sizeof text);
  CHECK(std::string(text).find("overrides engaged=4 untested=63\n") != std::string::npos);
}

TFC_TEST(overrides_a_sense_line_is_never_an_input_to_a_decision_about_the_units) {  // TFC-HWO-005: two runs, one with every override engaged, give the same outputs, step by step
  SupConfig cfg;
  cfg.override_fitted = 0x3FU;
  World a(cfg);
  World b(cfg);
  b.in.overrides = 0x3FU;
  for (unsigned i = 0; i < 4000U; ++i) {
    if (i == 1000U) {
      a.u[1].hung = true;
      b.u[1].hung = true;
    }
    a.ms();
    b.ms();
    CHECK(a.out.nrst_low == b.out.nrst_low && a.out.power_cut == b.out.power_cut && a.out.safe_line == b.out.safe_line && a.out.t0_line == b.out.t0_line);
  }
  CHECK(a.resets_seen[1] >= 1U && a.resets_seen == b.resets_seen && a.cycles_seen == b.cycles_seen);
}

TFC_TEST(overrides_the_command_is_parsed_and_takes_no_argument) {
  CHECK(cmd("override-ok").kind == Kind::OverrideOk && cmd("OVERRIDE-OK").parse == Parse::Ok);
  CHECK(cmd("override-ok A").parse == Parse::ExtraWords);
}

// ============================== `time`: the counter the PC correlates with UTC (TFC-SUP-011, SUP-012) ==============================
TFC_TEST(time_the_command_is_parsed_and_answers_with_the_counter_the_rtc_and_no_mission_time_before_t_zero) {
  CHECK(cmd("time").kind == Kind::Time && cmd("TIME").parse == Parse::Ok && cmd("time A").parse == Parse::ExtraWords);
  World w;
  w.run_ms(1234U);
  w.ms("time");
  CHECK(w.resp == Response::Done);
  char text[96];
  const std::size_t n = w.sup.time_text(w.in, text, sizeof text);
  CHECK(n == std::strlen(text));
  const std::string s(text);
  CHECK(s == "time ticks=" + std::to_string(w.in.ticks) + " rtc=" + std::to_string(w.in.rtc_s) + " met_us=-\n");
}

TFC_TEST(time_after_t_zero_the_mission_elapsed_time_runs_on_the_supervisors_own_ticks) {
  World w;
  w.run_ms(300U);
  w.ms("t0");
  const uint64_t t_zero = w.in.ticks;
  w.run_ms(2500U);
  char text[96];
  (void)w.sup.time_text(w.in, text, sizeof text);
  const std::string s(text);
  const std::size_t at = s.find("met_us=");
  CHECK(at != std::string::npos);
  const uint64_t met = std::stoull(s.substr(at + 7U));
  CHECK(met == w.in.ticks - t_zero);  // 1 MHz ticks: microseconds
  CHECK(met >= 2500U * kMs);
}

TFC_TEST(time_the_answer_is_bounded_and_always_terminated) {
  World w;
  w.run_ms(10U);
  char big[96];
  const std::size_t full = w.sup.time_text(w.in, big, sizeof big);
  CHECK(full > 10U);
  for (std::size_t cap = 0U; cap <= full + 1U; ++cap) {
    char small[100];
    std::memset(small, 'x', sizeof small);
    const std::size_t n = w.sup.time_text(w.in, small, cap);
    CHECK(cap == 0U ? (n == 0U && small[0] == 'x') : (n < cap && small[n] == '\0'));
  }
}
