// SPDX-License-Identifier: MIT
// The simulator's runner (sim/vehicle/runner.hpp) with the real flight computers and the real actuator logic on the other end of the same frames the bus carries:
// the runner publishes 0x501/0x502, each node adds its own noise and runs the flight function, ACT votes the three commands, and ACT's frame goes back to the runner.
// This is the whole P1 software chain, closed over the 6-DOF vehicle, without a socket. The live version (tools/sim/tfc_simd.cpp on vcan0) puts the same frames on a bus.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "tfc/act.hpp"
#include "tfc/flight.hpp"
#include "tfc/protocol.hpp"
#include "tfc_test.hpp"

#include "design.hpp"
#include "runner.hpp"

namespace {

class Lcg {
 public:
  explicit Lcg(uint32_t s) : s_(s) {}
  float uniform() {
    s_ = (s_ * 1664525U) + 1013904223U;
    return (static_cast<float>(s_ >> 8) / 8388608.0F) - 1.0F;
  }

 private:
  uint32_t s_;
};

struct Result {
  double rms_deg = 0.0;
  double max_deg = 0.0;
  double final_altitude = 0.0;
  double final_tilt_deg = 0.0;
  uint32_t nominal_from = 0U;       // the frame ACT reached Nominal
  uint32_t safe_frames = 0U;
  uint32_t held_frames = 0U;        // frames in which the runner did not get ACT's frame
  uint32_t platform_saturated = 0U;
  bool finite = true;
  tfc::ActMode final_mode = tfc::ActMode::Standby;
  uint8_t final_excluded = 0U;
  uint8_t flags = 0U;
};

struct Loop {
  sim::RunnerConfig cfg;
  uint32_t frames = 10000U;
  uint32_t node_b_dead_from = 0xFFFFFFFFU;   // node B stops sending (all its frames) from this frame
  uint32_t act_lost_from = 0xFFFFFFFFU;      // ACT's frames stop reaching the runner for `act_lost_for` frames from here
  uint32_t act_lost_for = 0U;
  bool sensor_fault_b = false;               // node B's gyro reads 15 dps too much
};

Result run(const Loop& lp) {
  using namespace sim;
  SimRunner runner(lp.cfg);
  const FlightTables tables = flight_tables(lp.cfg.params);
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(tables.gains, tables.guidance), tfc::FlightFunction(tables.gains, tables.guidance),
                                        tfc::FlightFunction(tables.gains, tables.guidance)};
  std::array<Lcg, 3> noise{Lcg(0x1234U), Lcg(0x1235U), Lcg(0x1236U)};  // the firmware's seeds: 0x1234 + node
  tfc::ActLogic act;
  tfc::ActRecord none{};
  act.boot(tfc::ResetCause::PowerOn, none);
  Result r;
  SimFrames pending = runner.start(0U);
  double sum_sq = 0.0;
  bool reached_nominal = false;
  for (uint32_t k = 0; k < lp.frames; ++k) {
    // each node latches the simulator's inputs for this frame and adds its own noise
    tfc::Vec3 rates;
    tfc::Vec3 accel;
    const tfc::DecodedVec3 dr = tfc::unpack_vec3(pending.f[0], tfc::kGyroLsbDps);
    const tfc::DecodedVec3 da = tfc::unpack_vec3(pending.f[1], tfc::kAccelLsbG);
    CHECK(dr.ok && da.ok && pending.f[0].id == tfc::id::kSimRates && pending.f[1].id == tfc::id::kSimAccel);
    rates = dr.x;
    accel = da.x;
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<tfc::Frame, 6> sensor{};
    std::array<bool, 3> alive{true, true, true};
    alive[1] = k < lp.node_b_dead_from;
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 g;
      tfc::Vec3 a;
      for (unsigned i = 0; i < 3U; ++i) {
        g.v[i] = rates.v[i] + (0.17F * noise[n].uniform());
        a.v[i] = accel.v[i] + (0.0035F * noise[n].uniform());
      }
      if (lp.sensor_fault_b && n == 1U) {
        g.v[0] += 15.0F;
        g.v[1] += 15.0F;
      }
      sensor[n] = tfc::pack_gyro(n, g, seq);
      sensor[3U + n] = tfc::pack_accel(n, a, seq);
    }
    std::array<tfc::Command, 3> cmd{};
    act.begin_frame();
    for (unsigned n = 0; n < 3U; ++n) {
      if (!alive[n]) {
        continue;
      }
      ff[n].begin_frame(k, alive[1] ? 0x07U : 0x05U);
      for (unsigned m = 0; m < 3U; ++m) {
        if (!alive[m]) {
          continue;
        }
        (void)ff[n].on_frame(sensor[m]);
        (void)ff[n].on_frame(sensor[3U + m]);
      }
      cmd[n] = ff[n].step();
      (void)act.on_frame(tfc::pack_cmd(static_cast<uint8_t>(n), cmd[n], seq));
    }
    act.safe_request(false);
    const tfc::ActOutput& out = act.end_frame();
    if (out.mode == tfc::ActMode::Nominal && !reached_nominal) {
      reached_nominal = true;
      r.nominal_from = k;
    }
    r.safe_frames += out.mode == tfc::ActMode::Safe ? 1U : 0U;
    // ACT's frame goes over the wire and back to the runner (or is lost)
    const tfc::DecodedAct wire = tfc::unpack_act_out(tfc::pack_act_out(tfc::to_act_frame(out), seq));
    CHECK(wire.ok);
    const bool lost = k >= lp.act_lost_from && k < lp.act_lost_from + lp.act_lost_for;
    r.held_frames += lost ? 1U : 0U;
    pending = runner.end_of_frame(k, lost ? nullptr : &wire.act);
    const sim::Tilts t = runner.vehicle().tilts();
    const tfc::Reference ref = tables.guidance.at(k + 1U);
    const double ep = t.y_deg - static_cast<double>(ref.tilt_y_deg);
    const double ey = t.x_deg - static_cast<double>(ref.tilt_x_deg);
    r.max_deg = std::fmax(r.max_deg, std::fmax(std::fabs(ep), std::fabs(ey)));
    sum_sq += (ep * ep) + (ey * ey);
    r.platform_saturated += runner.platform().saturated() ? 1U : 0U;
    r.finite = r.finite && std::isfinite(t.x_deg) && std::isfinite(t.y_deg) && std::isfinite(static_cast<double>(out.pitch_deg));
    r.final_mode = out.mode;
    r.final_excluded = out.excluded_nodes;
  }
  r.rms_deg = std::sqrt(sum_sq / (2.0 * lp.frames));
  r.final_altitude = runner.vehicle().altitude();
  r.final_tilt_deg = runner.vehicle().tilts().y_deg;
  const tfc::DecodedSimFlags fl = tfc::unpack_sim_flags(pending.f[4]);  // frame 10000 is a multiple of ten: the flags are in the batch
  r.flags = fl.ok ? fl.s.flags : 0xFFU;
  return r;
}

void report(const char* name, const Result& r) {
  if (std::getenv("TFC_LOOP_VERBOSE") != nullptr) {
    std::printf("    [%s] rms %.3f deg max %.3f  end tilt %.1f alt %.0f m  ACT nominal from frame %u, safe frames %u, held %u  excluded 0x%x  flags 0x%02x\n", name, r.rms_deg,
                r.max_deg, r.final_tilt_deg, r.final_altitude, static_cast<unsigned>(r.nominal_from), static_cast<unsigned>(r.safe_frames),
                static_cast<unsigned>(r.held_frames), static_cast<unsigned>(r.final_excluded), static_cast<unsigned>(r.flags));
  }
}

}  // namespace

TFC_TEST(runner_frames_start_a_run_in_progress_and_publish_one_frame_ahead) {
  sim::SimRunner runner;
  sim::SimFrames first = runner.start(5U);
  CHECK(first.n == 2U);  // frame 5 is not a status frame
  CHECK(first.f[0].id == tfc::id::kSimRates && first.f[1].id == tfc::id::kSimAccel && first.f[0].data[6] == 5U);
  CHECK(runner.frame() == 5U && std::fabs(runner.vehicle().state().m - 30000.0) > 1.0);  // five frames of burn have passed
  const tfc::DecodedVec3 a = tfc::unpack_vec3(first.f[1], tfc::kAccelLsbG);
  CHECK(a.ok && std::fabs(a.x.v[2] - 1.0F) < 0.01F);  // at rest on the platform: +1 g on Z
  tfc::ActFrame act;
  sim::SimFrames next = runner.end_of_frame(5U, &act);
  CHECK(runner.frame() == 6U && next.f[0].data[6] == 6U && !runner.command_held());
  sim::SimFrames ten = runner.end_of_frame(8U, &act);  // -> frame 9, then 10
  ten = runner.end_of_frame(9U, nullptr);              // ACT's frame did not come
  CHECK(ten.n == 5U && runner.command_held());         // frame 10 carries the status frames
  const tfc::DecodedSimFlags fl = tfc::unpack_sim_flags(ten.f[4]);
  CHECK(fl.ok && (fl.s.flags & tfc::simflag::kCommandHeld) != 0U && fl.s.engines_on == 5U && fl.s.time_frames == 10U);
  const tfc::DecodedSimState st = tfc::unpack_sim_state(ten.f[2]);
  CHECK(st.ok && st.s.mass_kg > 29000.0F && st.s.mass_kg < 30000.0F);
  const tfc::DecodedSimTelemetry tm = tfc::unpack_sim_telemetry(ten.f[3]);
  CHECK(tm.ok && std::fabs(tm.t.pitch_error_deg) < 1.0F);
  sim::SimRunner other;
  (void)other.start(9U);
  other.abort_run();
  const sim::SimFrames at10 = other.end_of_frame(9U, &act);  // frame 10 is a status frame
  CHECK(at10.n == 5U && (tfc::unpack_sim_flags(at10.f[4]).s.flags & tfc::simflag::kAborted) != 0U);
}

TFC_TEST(runner_closed_loop_nominal_ascent_through_the_bus_frames_three_computers_and_act) {
  Loop lp;
  const Result r = run(lp);
  report("nominal", r);
  CHECK(r.finite);
  CHECK(r.nominal_from >= 99U && r.nominal_from <= 105U);  // ACT's standby run of 100 trustworthy votes
  CHECK(r.safe_frames == 0U && r.held_frames == 0U && r.platform_saturated == 0U);
  CHECK(r.max_deg < 1.0 && r.rms_deg < 0.25);
  CHECK(r.final_altitude > 25000.0 && r.final_tilt_deg > 25.0 && r.final_tilt_deg < 40.0);
  CHECK(r.final_mode == tfc::ActMode::Nominal && r.final_excluded == 0U);
}

TFC_TEST(runner_closed_loop_flies_through_a_gust_and_an_engine_out_at_max_q) {
  Loop gust;
  sim::Gust g;
  g.t0 = 60.0;
  g.duration = 3.0;
  g.peak = sim::V3{0.0, 0.0, 15.0};
  gust.cfg.scenario.gusts.push_back(g);
  const Result rg = run(gust);
  report("gust", rg);
  CHECK(rg.finite && rg.max_deg < 2.5 && rg.safe_frames == 0U && rg.platform_saturated == 0U);
  Loop eo;
  eo.cfg.scenario.engine_out_time = 62.0;
  eo.cfg.scenario.engine_out_index = 1;
  const Result re = run(eo);
  report("engine out", re);
  CHECK(re.finite && re.max_deg < 3.0 && re.safe_frames == 0U && re.platform_saturated == 0U);
  CHECK((re.flags & tfc::simflag::kEngineOut) != 0U);
}

TFC_TEST(runner_closed_loop_masks_a_failed_sensor_and_flies_on_with_a_computer_lost) {
  Loop clean;
  const Result rc = run(clean);
  Loop sf;
  sf.sensor_fault_b = true;
  const Result rs = run(sf);
  report("sensor fault on B", rs);
  CHECK(rs.finite && rs.safe_frames == 0U && rs.max_deg < rc.max_deg + 0.5);  // the consensus masks it
  Loop dead;
  dead.node_b_dead_from = 3000U;  // B goes silent at 30 s, before max-Q
  const Result rd = run(dead);
  report("B dead from 30 s", rd);
  CHECK(rd.finite && rd.safe_frames == 0U && rd.max_deg < 1.5);
  CHECK((rd.final_excluded & 0x2U) != 0U);  // ACT excluded B
  CHECK(rd.final_mode == tfc::ActMode::Nominal);
}

TFC_TEST(runner_closed_loop_holds_the_last_command_through_a_short_loss_of_acts_frame) {
  Loop lp;
  lp.act_lost_from = 3000U;
  lp.act_lost_for = 20U;  // 0.2 s without ACT's frame
  const Result r = run(lp);
  report("ACT frame lost for 0.2 s", r);
  CHECK(r.finite && r.held_frames == 20U && r.safe_frames == 0U && r.max_deg < 1.5);
}
