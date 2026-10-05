// SPDX-License-Identifier: MIT
// The supervisor's decision logic (docs/SUPERVISOR.md, ADR-022; the SUP-Lite build): what it watches, what it does, and what it never does. Pure logic over counters and ticks: the
// hardware layer hands it, once per loop, the cumulative counts and the latest rising-edge tick of each unit's FRAME and KICK lines and the RTC's seconds, and it returns the levels
// of its output lines and a list of what happened. Integer-only and portable; by TFC-SUP-001 it shares no code with `core/`.
//
//   * KICK missing for 3 frames: pulse NRST for 100 ms; 3 resets inside 60 s: open the power relay for 500 ms; once 2 power-cycles have been made inside 5 minutes, the next stop holds the unit
//     in reset and calls it DEAD (an operator `release` brings it back). Actions on different units are at least `stagger_ms` apart, so a total loss is recovered one unit at a time.
//   * A unit is not judged for `boot_grace_ms` after a reset or a power-cycle (it has to boot and run a frame).
//   * FRAME period off by more than 200 ppm against the supervisor's clock, or two flight computers' frame starts more than 200 us apart, are reported and nothing else: the supervisor never
//     isolates a working unit (TFC-SUP-010).
//   * Power-up: it listens for `warm_probe_ms`; if any unit is already running it adopts it and touches nothing (a supervisor that resets in flight leaves every node running,
//     TFC-SUP-007). Only with nothing running it holds every unit in reset and releases them in order ACT, A, B, C, `release_spacing_ms` apart (boot sequencing).
//   * Operator commands (`commands.hpp`) are executed without any flight computer, and answered.
//   * T-zero: `launch` starts the supervisor's own countdown, `t0` is the bench shortcut; at T-zero it asserts the T0 line and latches the mission clock (`mission_clock.hpp`), returning the record to
//     store in non-volatile memory; from then on it checks each flight computer's frame count against its mission clock once a second (TFC-SUP-009). Nothing here controls on that clock.
//   * No unit is ever reset because of its data: only because it stopped (TFC-SUP-010). The last-agreeing rule of TFC-RESP-004 is satisfied by construction: the Lite supervisor cannot see a
//     vote, so it can only act on a unit that has stopped kicking.
// No heap, no exceptions, no RTTI. Every loop is bounded.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "sup/commands.hpp"
#include "sup/mission_clock.hpp"

namespace sup {

struct SupConfig {
  uint32_t tick_hz = 1000000U;           // the supervisor's timer; a multiple of 100 so that a frame is a whole number of ticks
  uint32_t kick_missing_frames = 3U;     // a unit whose KICK has not come for this many frames is judged stopped
  uint32_t reset_pulse_ms = 100U;
  uint32_t cycle_open_ms = 500U;
  uint32_t boot_grace_ms = 2000U;        // after a reset or a power-cycle a unit is not judged for this long
  uint32_t resets_before_cycle = 3U;
  uint32_t reset_window_s = 60U;
  uint32_t cycles_before_dead = 2U;
  uint32_t cycle_window_s = 300U;
  uint32_t stagger_ms = 200U;            // between actions on different units
  uint32_t warm_probe_ms = 100U;         // power-up: this long listening before deciding between a warm and a cold start
  uint32_t release_spacing_ms = 500U;    // cold start: units are let out of reset this far apart
  uint32_t period_ppm_limit = 200U;
  uint32_t period_window_frames = 1000U; // the period is averaged over 10 s
  uint32_t phase_limit_us = 200U;
  uint32_t countdown_s = 10U;
  uint32_t t0_pulse_ms = 50U;
  uint32_t mission_check_frames = 100U;  // the mission-time plausibility check runs this often
};

// What the hardware layer reports for one unit: cumulative counts of rising edges and the tick of the latest one.
struct UnitInput {
  uint32_t frames = 0U;
  uint64_t frame_tick = 0U;
  uint32_t kicks = 0U;
  uint64_t kick_tick = 0U;
};

struct Inputs {
  uint64_t ticks = 0U;   // the supervisor's own counter now
  uint64_t rtc_s = 0U;   // the battery-backed RTC's seconds now (read about once a second)
  std::array<UnitInput, kUnits> unit{};
};

enum class UnitState : uint8_t {
  Running = 0,   // judged
  Resetting,     // NRST held low for a pulse
  Cutting,       // power relay open for a moment
  Grace,         // booting: not judged yet
  Held,          // NRST held low until released (operator hold, cold start, or DEAD)
  Probing        // power-up: listening before deciding warm or cold
};

enum class Response : uint8_t {
  Done = 0,
  Refused,       // the command is understood but not allowed now (a countdown already running, a scrub after T-zero, ...)
  Unknown        // not a command
};

struct Outputs {
  std::array<bool, kUnits> nrst_low{};   // the unit's reset line pulled low
  std::array<bool, kUnits> power_cut{};  // the unit's power relay energised (power removed)
  bool safe_line = false;                // SAFE to ACT
  bool t0_line = false;                  // T0 to every flight computer
};

struct Events {
  std::array<bool, kUnits> reset{};        // a reset pulse started this step
  std::array<bool, kUnits> cycle{};        // a power-cycle started
  std::array<bool, kUnits> dead{};         // the unit was held and declared DEAD
  std::array<bool, kUnits> period_bad{};   // the 10 s average of the FRAME period is out of limit (reported once per window)
  std::array<bool, kUnits> mission_bad{};  // the unit's frame count disagrees with the mission clock
  bool warm_start = false;
  bool cold_start = false;
  bool phase_bad = false;                  // two flight computers' frame starts are further apart than the limit
  bool total_loss = false;                 // no flight computer is producing KICKs
  bool countdown_started = false;
  bool scrubbed = false;
  bool t0 = false;                         // T-zero happened this step: store `record`
  MetRecord record{};
};

class Supervisor {
 public:
  explicit Supervisor(SupConfig cfg = {}) noexcept : clock_(ClockConfig{cfg.tick_hz}), cfg_(cfg) {}

  [[nodiscard]] bool config_ok() const noexcept { return clock_.config_ok(); }

  // After a supervisor reset: the record of T-zero read back from non-volatile memory, and the clocks as they are now.
  bool restore(const MetRecord& rec, uint64_t ticks_now, uint64_t rtc_now_s) noexcept {
    const bool ok = clock_.resume(rec, ticks_now, rtc_now_s);
    launched_ = ok;
    return ok;
  }

  // One loop of the supervisor: execute `cmd` (if any), then judge every unit. `response` is the answer to the command.
  Outputs step(const Inputs& in, const Command* cmd, Events& ev, Response& response) noexcept {
    ev = Events{};
    response = Response::Done;
    if (!started_) {
      begin(in);
    }
    if (cmd != nullptr) {
      response = execute(*cmd, in, ev);
    }
    if (mode_ == Mode::Probe) {
      probe(in, ev);
    }
    for (unsigned u = 0; u < kUnits; ++u) {
      advance_unit(u, in, ev);
    }
    countdown(in, ev);
    t0_pulse(in);
    if (mode_ != Mode::Probe) {
      judge_frames(in, ev);
      judge_phase(in, ev);
      judge_mission(in, ev);
    }
    return outputs(in);
  }

  // The answer to `status`: one line per unit ("A running resets=0 cycles=0 period=+12ppm"), then the supervisor's own state. Truncated to `cap` (NUL-terminated); returns the length.
  std::size_t status_text(char* out, std::size_t cap) const noexcept {
    if (cap == 0U) {
      return 0U;
    }
    std::size_t n = 0U;
    constexpr const char* names[kUnits] = {"A", "B", "C", "ACT"};
    constexpr const char* states[] = {"running", "resetting", "cutting", "grace", "held", "probing"};
    for (unsigned u = 0; u < kUnits; ++u) {
      put(out, cap, n, names[u]);
      put(out, cap, n, " ");
      put(out, cap, n, dead_[u] ? "DEAD" : states[static_cast<unsigned>(state_[u])]);
      put(out, cap, n, " resets=");
      put_uint(out, cap, n, resets_[u]);
      put(out, cap, n, " cycles=");
      put_uint(out, cap, n, cycles_[u]);
      put(out, cap, n, " period=");
      put(out, cap, n, period_ppm_[u] < 0 ? "-" : "+");
      put_uint(out, cap, n, static_cast<uint64_t>(period_ppm_[u] < 0 ? -static_cast<int64_t>(period_ppm_[u]) : static_cast<int64_t>(period_ppm_[u])));
      put(out, cap, n, "ppm\n");
    }
    put(out, cap, n, launched_ ? "launched" : (counting_ ? "counting down" : "not launched"));
    put(out, cap, n, safe_ ? " SAFE-line-on\n" : "\n");
    out[n] = '\0';
    return n;
  }

  [[nodiscard]] UnitState state(Unit u) const noexcept { return state_[idx(u)]; }
  [[nodiscard]] bool dead(Unit u) const noexcept { return dead_[idx(u)]; }
  [[nodiscard]] uint32_t resets(Unit u) const noexcept { return resets_[idx(u)]; }
  [[nodiscard]] uint32_t cycles(Unit u) const noexcept { return cycles_[idx(u)]; }
  [[nodiscard]] int32_t period_ppm(Unit u) const noexcept { return period_ppm_[idx(u)]; }
  [[nodiscard]] bool launched() const noexcept { return launched_; }
  [[nodiscard]] bool counting_down() const noexcept { return counting_; }
  [[nodiscard]] const MissionClock& clock() const noexcept { return clock_; }
  [[nodiscard]] bool warm() const noexcept { return mode_ == Mode::Warm; }

 private:
  enum class Mode : uint8_t { Probe, Warm, Cold };

  static void put(char* out, std::size_t cap, std::size_t& n, const char* text) noexcept {
    for (std::size_t i = 0; text[i] != '\0' && n + 1U < cap; ++i) {
      out[n++] = text[i];
    }
  }

  static void put_uint(char* out, std::size_t cap, std::size_t& n, uint64_t v) noexcept {
    std::array<char, 20> digits{};  // all twenty, most significant last; the leading zeros are skipped when writing
    for (std::size_t i = 0; i < digits.size(); ++i) {
      digits[i] = static_cast<char>('0' + (v % 10U));
      v /= 10U;
    }
    bool started = false;
    for (std::size_t i = digits.size(); i > 0U && n + 1U < cap; --i) {
      started = started || digits[i - 1U] != '0' || i == 1U;
      if (started) {
        out[n++] = digits[i - 1U];
      }
    }
  }

  [[nodiscard]] static constexpr unsigned idx(Unit u) noexcept { return static_cast<unsigned>(u); }
  [[nodiscard]] uint64_t ms(uint32_t m) const noexcept { return static_cast<uint64_t>(m) * (cfg_.tick_hz / 1000U); }
  [[nodiscard]] uint64_t seconds(uint32_t s) const noexcept { return static_cast<uint64_t>(s) * cfg_.tick_hz; }
  [[nodiscard]] uint64_t frame_ticks() const noexcept { return cfg_.tick_hz / 100U; }

  void begin(const Inputs& in) noexcept {
    started_ = true;
    start_tick_ = in.ticks;
    mode_ = Mode::Probe;
    for (unsigned u = 0; u < kUnits; ++u) {
      state_[u] = UnitState::Probing;
      seen_kicks_[u] = in.unit[u].kicks;
      seen_frames_[u] = in.unit[u].frames;
    }
  }

  // Power-up: any activity in the probe window means a warm start (adopt, touch nothing); none means a cold start (hold everything, release in order).
  void probe(const Inputs& in, Events& ev) noexcept {
    bool active = false;
    for (unsigned u = 0; u < kUnits; ++u) {
      active = active || in.unit[u].kicks != seen_kicks_[u] || in.unit[u].frames != seen_frames_[u];
    }
    if (active) {
      mode_ = Mode::Warm;
      ev.warm_start = true;
      for (unsigned u = 0; u < kUnits; ++u) {
        state_[u] = UnitState::Running;
        last_alive_[u] = in.ticks;
        grace_end_[u] = in.ticks;
        kicks_[u] = in.unit[u].kicks;
      }
      return;
    }
    if (in.ticks - start_tick_ >= ms(cfg_.warm_probe_ms)) {
      mode_ = Mode::Cold;
      ev.cold_start = true;
      constexpr std::array<Unit, kUnits> order{Unit::Act, Unit::A, Unit::B, Unit::C};
      for (unsigned k = 0; k < kUnits; ++k) {
        const unsigned u = idx(order[k]);
        state_[u] = UnitState::Held;
        auto_release_[u] = in.ticks + (static_cast<uint64_t>(k + 1U) * ms(cfg_.release_spacing_ms));  // every unit is held for at least one spacing
      }
    }
  }

  Response execute(const Command& c, const Inputs& in, Events& ev) noexcept {
    const unsigned u = idx(c.unit);
    switch (c.kind) {
      case Kind::Reset:
        if (state_[u] == UnitState::Probing || state_[u] == UnitState::Held) {
          return Response::Refused;  // a held unit is released, not reset
        }
        start_reset(u, in, ev, false);
        return Response::Done;
      case Kind::Cycle:
        if (state_[u] == UnitState::Probing || state_[u] == UnitState::Held) {
          return Response::Refused;
        }
        start_cycle(u, in, ev, false);
        return Response::Done;
      case Kind::Hold:
        state_[u] = UnitState::Held;
        auto_release_[u] = 0U;
        return Response::Done;
      case Kind::Release:
        if (state_[u] != UnitState::Held) {
          return Response::Refused;
        }
        dead_[u] = false;
        reset_hist_[u] = {};
        cycle_hist_[u] = {};
        enter_grace(u, in);
        return Response::Done;
      case Kind::SafeNow:
        safe_ = true;
        return Response::Done;
      case Kind::SafeClear:
        safe_ = false;
        return Response::Done;
      case Kind::Launch:
        if (launched_ || counting_) {
          return Response::Refused;
        }
        counting_ = true;
        count_end_ = in.ticks + seconds(cfg_.countdown_s);
        ev.countdown_started = true;
        return Response::Done;
      case Kind::Scrub:
        if (!counting_) {
          return Response::Refused;  // nothing to scrub: not counting, or already past T-zero
        }
        counting_ = false;
        ev.scrubbed = true;
        return Response::Done;
      case Kind::T0:
        if (launched_) {
          return Response::Refused;
        }
        counting_ = false;
        fire_t0(in, ev);
        return Response::Done;
      case Kind::Status:
        return Response::Done;
      case Kind::Unknown:
      default:
        return Response::Unknown;
    }
  }

  void enter_grace(unsigned u, const Inputs& in) noexcept {
    state_[u] = UnitState::Grace;
    grace_end_[u] = in.ticks + ms(cfg_.boot_grace_ms);
    last_alive_[u] = grace_end_[u];
    kicks_[u] = in.unit[u].kicks;
    window_frames0_[u] = 0U;
    window_open_[u] = false;
  }

  [[nodiscard]] static unsigned in_window(const std::array<uint64_t, 8>& hist, uint64_t now, uint64_t window) noexcept {
    unsigned n = 0U;
    for (const uint64_t t : hist) {
      n += (t != 0U && now + 1U - t < window) ? 1U : 0U;  // (entries are stored as tick + 1 so that 0 can mean an empty slot)
    }
    return n;
  }

  static void record(std::array<uint64_t, 8>& hist, uint64_t now) noexcept {
    unsigned oldest = 0U;
    for (unsigned i = 0; i < hist.size(); ++i) {
      if (hist[i] < hist[oldest]) {
        oldest = i;
      }
    }
    hist[oldest] = now + 1U;
  }

  void start_reset(unsigned u, const Inputs& in, Events& ev, bool automatic) noexcept {
    state_[u] = UnitState::Resetting;
    action_end_[u] = in.ticks + ms(cfg_.reset_pulse_ms);
    last_action_ = in.ticks;
    have_action_ = true;
    ++resets_[u];
    ev.reset[u] = true;
    if (automatic) {
      record(reset_hist_[u], in.ticks);
    }
  }

  void start_cycle(unsigned u, const Inputs& in, Events& ev, bool automatic) noexcept {
    state_[u] = UnitState::Cutting;
    action_end_[u] = in.ticks + ms(cfg_.cycle_open_ms);
    last_action_ = in.ticks;
    have_action_ = true;
    ++cycles_[u];
    ev.cycle[u] = true;
    if (automatic) {
      record(cycle_hist_[u], in.ticks);
    }
  }

  void advance_unit(unsigned u, const Inputs& in, Events& ev) noexcept {
    switch (state_[u]) {
      case UnitState::Probing:
        return;
      case UnitState::Held:
        if (auto_release_[u] != 0U && in.ticks >= auto_release_[u]) {
          auto_release_[u] = 0U;
          enter_grace(u, in);
        }
        return;
      case UnitState::Resetting:
      case UnitState::Cutting:
        if (in.ticks >= action_end_[u]) {
          enter_grace(u, in);
        }
        return;
      case UnitState::Grace:
        if (in.ticks >= grace_end_[u]) {
          state_[u] = UnitState::Running;
        }
        return;
      case UnitState::Running:
      default:
        break;
    }
    if (in.unit[u].kicks != kicks_[u]) {
      kicks_[u] = in.unit[u].kicks;
      last_alive_[u] = in.unit[u].kick_tick;
    }
    const bool stopped = in.ticks > last_alive_[u] && in.ticks - last_alive_[u] > static_cast<uint64_t>(cfg_.kick_missing_frames) * frame_ticks();
    if (!stopped) {
      return;
    }
    if (have_action_ && in.ticks - last_action_ < ms(cfg_.stagger_ms)) {
      return;  // another unit was just acted on: this one waits its turn
    }
    const uint64_t reset_window = seconds(cfg_.reset_window_s);
    const uint64_t cycle_window = seconds(cfg_.cycle_window_s);
    if (in_window(cycle_hist_[u], in.ticks, cycle_window) >= cfg_.cycles_before_dead) {
      state_[u] = UnitState::Held;  // out of ways to bring it back: hold it and say so
      auto_release_[u] = 0U;
      dead_[u] = true;
      last_action_ = in.ticks;
      have_action_ = true;
      ev.dead[u] = true;
    } else if (in_window(reset_hist_[u], in.ticks, reset_window) >= cfg_.resets_before_cycle) {
      start_cycle(u, in, ev, true);
      reset_hist_[u] = {};  // the cycle answers those resets
    } else {
      start_reset(u, in, ev, true);
    }
  }

  void countdown(const Inputs& in, Events& ev) noexcept {
    if (counting_ && in.ticks >= count_end_) {
      counting_ = false;
      fire_t0(in, ev);
    }
  }

  void fire_t0(const Inputs& in, Events& ev) noexcept {
    ev.record = clock_.latch_t0(in.ticks, in.rtc_s);
    ev.t0 = true;
    launched_ = true;
    t0_until_ = in.ticks + ms(cfg_.t0_pulse_ms);
    for (unsigned u = 0; u < kUnits; ++u) {
      t0_frames_[u] = in.unit[u].frames;
    }
    mission_next_ = in.unit[0].frames + cfg_.mission_check_frames;
  }

  void t0_pulse(const Inputs& in) noexcept { t0_line_ = t0_until_ != 0U && in.ticks < t0_until_; }

  // The FRAME period of each running unit, averaged over a window, against the supervisor's clock.
  void judge_frames(const Inputs& in, Events& ev) noexcept {
    for (unsigned u = 0; u < kUnits; ++u) {
      if (state_[u] != UnitState::Running || in.unit[u].frames == 0U) {
        window_open_[u] = false;
        continue;
      }
      if (!window_open_[u]) {
        window_open_[u] = true;
        window_frames0_[u] = in.unit[u].frames;
        window_tick0_[u] = in.unit[u].frame_tick;
        continue;
      }
      const uint32_t frames = in.unit[u].frames - window_frames0_[u];
      if (frames < cfg_.period_window_frames) {
        continue;
      }
      const int64_t expected = static_cast<int64_t>(frames) * static_cast<int64_t>(frame_ticks());
      const int64_t measured = static_cast<int64_t>(in.unit[u].frame_tick - window_tick0_[u]);
      const int64_t ppm = ((measured - expected) * 1000000) / expected;
      period_ppm_[u] = static_cast<int32_t>(ppm);
      const int64_t mag = ppm < 0 ? -ppm : ppm;
      ev.period_bad[u] = mag > static_cast<int64_t>(cfg_.period_ppm_limit);
      window_frames0_[u] = in.unit[u].frames;
      window_tick0_[u] = in.unit[u].frame_tick;
    }
  }

  // Two flight computers' frame starts: the same instant, within a limit (modulo the frame, since the latest edges may belong to different frames).
  void judge_phase(const Inputs& in, Events& ev) noexcept {
    bool have_ref = false;
    uint64_t ref = 0U;
    const uint64_t period = frame_ticks();
    const uint64_t limit = static_cast<uint64_t>(cfg_.phase_limit_us) * (cfg_.tick_hz / 1000000U);
    for (unsigned u = 0; u < 3U; ++u) {
      if (state_[u] != UnitState::Running || in.unit[u].frames == 0U) {
        continue;
      }
      const uint64_t phase = in.unit[u].frame_tick % period;
      if (!have_ref) {
        have_ref = true;
        ref = phase;
        continue;
      }
      const uint64_t d = phase >= ref ? phase - ref : ref - phase;
      const uint64_t folded = d > period / 2U ? period - d : d;
      ev.phase_bad = ev.phase_bad || folded > limit;
    }
    // A total loss: at least one flight computer is judged and has stopped, and none is producing KICKs.
    bool producing = false;
    bool stopped = false;
    for (unsigned u = 0; u < 3U; ++u) {
      const bool running = state_[u] == UnitState::Running;
      producing = producing || (running && !stopped_now(u, in));
      stopped = stopped || (running && stopped_now(u, in));
    }
    ev.total_loss = stopped && !producing;
  }

  [[nodiscard]] bool stopped_now(unsigned u, const Inputs& in) const noexcept {
    return in.ticks > last_alive_[u] && in.ticks - last_alive_[u] > static_cast<uint64_t>(cfg_.kick_missing_frames) * frame_ticks();
  }

  // After T-zero: each flight computer's frame count since T-zero, in the mission-field encoding, against the mission clock (TFC-SUP-009).
  void judge_mission(const Inputs& in, Events& ev) noexcept {
    if (!launched_ || in.unit[0].frames < mission_next_) {
      return;
    }
    mission_next_ = in.unit[0].frames + cfg_.mission_check_frames;
    for (unsigned u = 0; u < 3U; ++u) {
      if (state_[u] != UnitState::Running || in.unit[u].frames == 0U) {
        continue;
      }
      const uint64_t since = in.unit[u].frames >= t0_frames_[u] ? in.unit[u].frames - t0_frames_[u] : 0U;
      const uint64_t field = kMissionFirstFlight + since;
      const uint16_t reported = field > kMissionSaturated ? kMissionSaturated : static_cast<uint16_t>(field);
      (void)watch_[u].check(clock_, in.ticks, reported);
      ev.mission_bad[u] = watch_[u].flagged();
    }
  }

  [[nodiscard]] Outputs outputs(const Inputs&) const noexcept {
    Outputs o;
    for (unsigned u = 0; u < kUnits; ++u) {
      o.nrst_low[u] = state_[u] == UnitState::Resetting || state_[u] == UnitState::Held;
      o.power_cut[u] = state_[u] == UnitState::Cutting;
    }
    o.safe_line = safe_;
    o.t0_line = t0_line_;
    return o;
  }

  // (ordered by size, largest first, so that the object has no padding to speak of)
  uint64_t start_tick_ = 0U;
  uint64_t last_action_ = 0U;
  uint64_t count_end_ = 0U;
  uint64_t t0_until_ = 0U;
  std::array<uint64_t, kUnits> last_alive_{};
  std::array<uint64_t, kUnits> grace_end_{};
  std::array<uint64_t, kUnits> action_end_{};
  std::array<uint64_t, kUnits> auto_release_{};
  std::array<uint64_t, kUnits> window_tick0_{};
  MissionClock clock_;
  std::array<std::array<uint64_t, 8>, kUnits> reset_hist_{};
  std::array<std::array<uint64_t, 8>, kUnits> cycle_hist_{};
  uint32_t mission_next_ = 0U;
  std::array<uint32_t, kUnits> kicks_{};
  std::array<uint32_t, kUnits> seen_kicks_{};
  std::array<uint32_t, kUnits> seen_frames_{};
  std::array<uint32_t, kUnits> resets_{};
  std::array<uint32_t, kUnits> cycles_{};
  std::array<uint32_t, kUnits> window_frames0_{};
  std::array<uint32_t, kUnits> t0_frames_{};
  std::array<int32_t, kUnits> period_ppm_{};
  std::array<MissionWatch, 3> watch_{};
  SupConfig cfg_;
  std::array<UnitState, kUnits> state_{};
  std::array<bool, kUnits> dead_{};
  std::array<bool, kUnits> window_open_{};
  Mode mode_ = Mode::Probe;
  bool started_ = false;
  bool have_action_ = false;
  bool safe_ = false;
  bool counting_ = false;
  bool launched_ = false;
  bool t0_line_ = false;
};

}  // namespace sup
