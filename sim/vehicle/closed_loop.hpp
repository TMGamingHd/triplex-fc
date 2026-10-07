// SPDX-License-Identifier: MIT
// The whole software chain closed over the 6-DOF vehicle, without a socket: SimRunner publishes the sensor inputs, three FlightFunctions (each with its own IMU model: noise, bias, scale,
// misalignment, latency, stale samples) compute commands, the real ActLogic votes them, and ACT's frame goes back to the runner. Used by tests/test_runner.cpp and by tools/sim/tfc_sens.cpp
// (sensitivity: how far each departure from the nominal can go before the flight is lost). The flown vehicle (`cfg.params`) can differ from the vehicle the tables were designed on (`cfg.design`).
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "design.hpp"
#include "imu_model.hpp"
#include "runner.hpp"
#include "tfc/act.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/flight.hpp"
#include "tfc/imu_calibration.hpp"
#include "tfc/protocol.hpp"

namespace sim {

struct Result {
  double rms_deg = 0.0;
  double max_deg = 0.0;
  double max_deg_settled = 0.0;     // the same, ignoring the first 3 s: ACT is in Standby (neutral gimbal) for the first second, and the vehicle is already flying
  double max_deg_liftoff = 0.0;     // the largest error in the first 3 s
  double final_altitude = 0.0;
  double final_tilt_deg = 0.0;
  uint32_t nominal_from = 0U;       // the frame ACT reached Nominal
  uint32_t ready_at = 0U;           // the first frame on which all three flight computers were ready for launch (0 if never, or no pad)
  bool calibrated = false;          // every computer's IMU calibration was ready at lift-off
  uint32_t safe_frames = 0U;
  uint32_t held_frames = 0U;        // frames in which the runner did not get ACT's frame
  uint32_t platform_saturated = 0U;
  bool finite = true;
  tfc::ActMode final_mode = tfc::ActMode::Standby;
  uint8_t final_excluded = 0U;
  uint8_t flags = 0U;
  uint32_t digest_mismatch_frames = 0U;   // frames in which the three computers' state digests were not all equal
  uint32_t longest_mismatch_run = 0U;     // the longest run of such frames
  uint32_t lost_frames = 0U;              // sensor frames a receiver did not get
  double max_command_spread_deg = 0.0;    // the largest difference between two computers' commands in a frame
  uint32_t first_safe_frame = 0U;         // the first frame (counted from the start, pad included) ACT was in Safe (0: never)
  tfc::SafeCause safe_cause = tfc::SafeCause::None;  // why, the first time
  double spread_at_safe_deg = 0.0;        // the command spread of the frame ACT went to Safe
  uint32_t frames_over_tol = 0U;          // frames in which the commands differed by more than ACT's agreement tolerance
  uint32_t resyncs_adopted = 0U;          // state resynchronisations a computer adopted (counted per computer)
  uint32_t resyncs_skipped = 0U;          // ... that a computer could not do (too few complete states, or no agreement)
  uint32_t corrections_changed = 0U;      // adoptions that changed a computer's own state in any bit
  uint32_t corrections_large = 0U;        // adoptions that found a computer's own state far from the vote
  uint32_t frames_over_manager_tol = 0U;  // frames in which two computers' commands differed by more than the fault manager's command tolerance
  uint32_t runs_of_three_over_manager_tol = 0U;  // ... and how many times that lasted three frames in a row (what its 3-of-5 detector needs)
  double max_spread_ab_deg = 0.0;         // the largest command difference between computers A and B in a frame
  double max_spread_c_deg = 0.0;          // ... between C and either of them (what a version tolerance for a diverse computer C has to absorb)
  uint32_t liftoff_frame = 0xFFFFFFFFU;   // the first frame after T-zero on which the vehicle was off the ground (0xFFFFFFFF: it never left it)
  bool crashed = false;                   // the vehicle reached the ground faster than the crash speed
  double min_altitude = 0.0;              // the lowest altitude reached after T-zero (never below 0 with the ground model: the vehicle cannot sink through the pad)
};

// One row of the record of a flight (see Loop::trace): what the vehicle and the loop were doing at a frame.
struct TraceRow {
  double t = 0.0;            // s since T-zero
  double altitude = 0.0;     // m
  double speed = 0.0;        // m/s
  double mach = 0.0;
  double dynamic_pressure = 0.0;  // Pa
  double mass = 0.0;         // kg
  double thrust = 0.0;       // N
  double tilt_y_deg = 0.0;   // the pitch plane tilt of the long axis from the pad vertical
  double tilt_x_deg = 0.0;   // and the yaw plane
  double err_y_deg = 0.0;    // against the pitch program
  double err_x_deg = 0.0;
  double cmd_pitch_deg = 0.0;  // ACT's voted command
  double cmd_yaw_deg = 0.0;
  unsigned stages_active = 0U;   // a bit per stage still on the vehicle
  unsigned stages_ignited = 0U;  // and per stage that has ignited
  int engines_on = 0;
  int act_mode = 0;
};

struct Loop {
  RunnerConfig cfg;
  SensorErrors sensors;
  tfc::EstimatorConfig estimator;           // the flight computers' estimator settings (use_accel false for a vehicle under thrust)
  uint32_t frames = 10000U;                  // frames of flight (after T-zero)
  uint32_t pad_frames = 0U;                  // frames on the pad before T-zero: the vehicle is clamped, the estimators calibrate, ACT goes Nominal; 0: released at the first frame, as before
  uint32_t node_b_dead_from = 0xFFFFFFFFU;   // node B stops sending (all its frames) from this frame
  uint32_t all_dead_from = 0xFFFFFFFFU;      // every flight computer stops sending from this frame (a total loss)
  uint32_t act_lost_from = 0xFFFFFFFFU;      // ACT's frames stop reaching the runner for `act_lost_for` frames from here
  uint32_t act_lost_for = 0U;
  bool sensor_fault_b = false;               // node B's gyro reads 15 dps too much
  float frame_loss_prob = 0.0F;              // the chance that a receiver does not get a given sensor frame of another computer in a frame (a late or lost frame): TS-16
  uint32_t loss_seed = 77U;
  uint32_t resync_period = 0U;               // the replicas exchange their state in the last frame of every period and adopt the vote (TS-16 option C); 0: never
  tfc::resync::Config resync;
  uint8_t resync_nodes = 0x07U;              // the computers that take part in the resync (send their state, vote, adopt); the others never adopt: the diverse computer of ADR-021 is left out with 0x03
  uint32_t corrupt_b_at = 0xFFFFFFFFU;       // at this frame (after its step) node B's attitude state is corrupted by about 2 degrees: a real estimator fault
  std::vector<TraceRow>* trace = nullptr;    // if set, a row is appended every `trace_every` frames after T-zero (a record of the flight for tools and plots)
  uint32_t trace_every = 10U;
};

inline Result run(const Loop& lp) {
  RunnerConfig rcfg = lp.cfg;
  rcfg.start_held = lp.pad_frames > 0U;
  SimRunner runner(rcfg);
  const FlightTables tables = flight_tables(lp.cfg.design, lp.cfg.plan);
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator), tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator),
                                        tfc::FlightFunction(tables.gains, tables.guidance, lp.estimator)};
  std::array<tfc::ImuCalibrator, 3> cal{};  // each computer calibrates its own IMU on the pad and subtracts the bias before it sends
  std::array<ImuModel, 3> imu{ImuModel(lp.sensors, 0x1234U), ImuModel(lp.sensors, 0x1235U), ImuModel(lp.sensors, 0x1236U)};  // the firmware's seeds: 0x1234 + node
  tfc::ActLogic act;
  tfc::ActRecord none{};
  act.boot(tfc::ResetCause::PowerOn, none);
  Result r;
  Lcg loss(lp.loss_seed);
  uint32_t mismatch_run = 0U;
  uint32_t over_run = 0U;
  SimFrames pending = runner.start(0U);
  double sum_sq = 0.0;
  bool reached_nominal = false;
  const uint32_t total = lp.pad_frames + lp.frames;
  for (uint32_t k = 0; k < total; ++k) {
    const bool pad_now = k < lp.pad_frames;
    const uint32_t fk = pad_now ? 0U : k - lp.pad_frames;  // frames since T-zero
    if (lp.pad_frames > 0U && k == lp.pad_frames) {
      runner.release();
    }
    const tfc::DecodedVec3 dr = tfc::unpack_vec3(pending.f[0], tfc::kGyroLsbDps);
    const tfc::DecodedVec3 da = tfc::unpack_vec3(pending.f[1], tfc::kAccelLsbG);
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<tfc::Frame, 6> sensor{};
    std::array<bool, 3> alive{true, true, true};
    alive[1] = k < lp.node_b_dead_from;
    if (k >= lp.all_dead_from) {
      alive = {false, false, false};
    }
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 g;
      tfc::Vec3 a;
      imu[n].sample(dr.x, da.x, g, a);
      if (lp.pad_frames > 0U) {
        cal[n].set_pad(pad_now);
        g = cal[n].process(g);
      }
      if (lp.sensor_fault_b && n == 1U) {
        g.v[0] += 15.0F;
        g.v[1] += 15.0F;
      }
      sensor[n] = tfc::pack_gyro(n, g, seq);
      sensor[3U + n] = tfc::pack_accel(n, a, seq);
    }
    act.begin_frame();
    std::array<tfc::Command, 3> cmds{};
    for (unsigned n = 0; n < 3U; ++n) {
      if (!alive[n]) {
        continue;
      }
      ff[n].begin_frame(k, alive[1] ? 0x07U : 0x05U);  // (a total loss never gets here)
      if (lp.pad_frames > 0U) {
        ff[n].set_mission(pad_now, fk);
      }
      for (unsigned m = 0; m < 3U; ++m) {
        if (!alive[m]) {
          continue;
        }
        for (const unsigned slot : {m, 3U + m}) {
          if (m != n && lp.frame_loss_prob > 0.0F && ((loss.uniform() + 1.0F) * 0.5F) < lp.frame_loss_prob) {
            ++r.lost_frames;  // this receiver did not get this frame: the others did
            continue;
          }
          (void)ff[n].on_frame(sensor[slot]);
        }
      }
      cmds[n] = ff[n].step();
      (void)act.on_frame(tfc::pack_cmd(static_cast<uint8_t>(n), cmds[n], seq));
    }
    double frame_spread = 0.0;
    if (alive[0] && alive[1] && alive[2]) {
      const bool same = cmds[0].state_digest == cmds[1].state_digest && cmds[1].state_digest == cmds[2].state_digest;
      mismatch_run = same ? 0U : mismatch_run + 1U;
      r.digest_mismatch_frames += same ? 0U : 1U;
      r.longest_mismatch_run = mismatch_run > r.longest_mismatch_run ? mismatch_run : r.longest_mismatch_run;
      for (unsigned a = 0; a < 3U; ++a) {
        for (unsigned b = a + 1U; b < 3U; ++b) {
          frame_spread = std::fmax(frame_spread, std::fmax(std::fabs(static_cast<double>(cmds[a].pitch_deg - cmds[b].pitch_deg)),
                                                           std::fabs(static_cast<double>(cmds[a].yaw_deg - cmds[b].yaw_deg))));
        }
      }
      r.max_command_spread_deg = std::fmax(r.max_command_spread_deg, frame_spread);
      const auto d2 = [&cmds](unsigned a, unsigned b) {
        return std::fmax(std::fabs(static_cast<double>(cmds[a].pitch_deg - cmds[b].pitch_deg)), std::fabs(static_cast<double>(cmds[a].yaw_deg - cmds[b].yaw_deg)));
      };
      r.max_spread_ab_deg = std::fmax(r.max_spread_ab_deg, d2(0U, 1U));
      r.max_spread_c_deg = std::fmax(r.max_spread_c_deg, std::fmax(d2(0U, 2U), d2(1U, 2U)));
      r.frames_over_tol += frame_spread > static_cast<double>(tfc::ActConfig{}.tol_deg) ? 1U : 0U;
      const bool over_mgr = frame_spread > static_cast<double>(tfc::RedundancyConfig{}.tol[tfc::kChPitch]);
      over_run = over_mgr ? over_run + 1U : 0U;
      r.frames_over_manager_tol += over_mgr ? 1U : 0U;
      r.runs_of_three_over_manager_tol += over_run == 3U ? 1U : 0U;
    }
    if (lp.corrupt_b_at == k) {
      tfc::resync::SharedState bad = ff[1].shared_state();
      bad.w[tfc::resync::kQuatFirst + 1U] = static_cast<int16_t>(bad.w[tfc::resync::kQuatFirst + 1U] + 600);
      (void)ff[1].adopt_state(bad);
    }
    if (lp.resync_period != 0U && tfc::resync::due(k, lp.resync_period)) {
      uint8_t healthy = 0U;
      std::array<std::array<tfc::Frame, tfc::kResyncChunks>, 3> tx{};
      for (uint8_t n = 0; n < 3U; ++n) {
        if (alive[n] && ((lp.resync_nodes >> n) & 1U) != 0U) {
          healthy = static_cast<uint8_t>(healthy | (1U << n));
          tx[n] = tfc::resync::pack_state(n, ff[n].shared_state(), seq);
        }
      }
      std::array<tfc::resync::Outcome, 3> votes{};
      for (unsigned n = 0; n < 3U; ++n) {
        if (!alive[n] || ((healthy >> n) & 1U) == 0U) {
          continue;
        }
        tfc::resync::Collector col;
        col.begin(k);
        for (unsigned m = 0; m < 3U; ++m) {
          for (unsigned c = 0; c < tfc::kResyncChunks && ((healthy >> m) & 1U) != 0U; ++c) {
            if (m != n && lp.frame_loss_prob > 0.0F && ((loss.uniform() + 1.0F) * 0.5F) < lp.frame_loss_prob) {
              ++r.lost_frames;
              continue;
            }
            (void)col.on_frame(tx[m][c]);
          }
        }
        votes[n] = tfc::resync::vote(col, healthy, lp.resync);
      }
      for (unsigned n = 0; n < 3U; ++n) {  // adopt after every vote was taken: the shares are those of the end of this frame
        if (!alive[n] || ((healthy >> n) & 1U) == 0U) {
          continue;
        }
        if (votes[n].adopted && ff[n].adopt_state(votes[n].state)) {
          ++r.resyncs_adopted;
          r.corrections_changed += ((votes[n].changed >> n) & 1U) != 0U ? 1U : 0U;
          r.corrections_large += ((votes[n].large >> n) & 1U) != 0U ? 1U : 0U;
        } else {
          ++r.resyncs_skipped;
        }
      }
    }
    act.safe_request(false);
    if (pad_now && r.ready_at == 0U && ff[0].sensors_ok() && ff[1].sensors_ok() && ff[2].sensors_ok() && cal[0].ready() && cal[1].ready() && cal[2].ready()) {
      r.ready_at = k;
    }
    const tfc::ActOutput& out = act.end_frame();
    if (out.mode == tfc::ActMode::Nominal && !reached_nominal) {
      reached_nominal = true;
      r.nominal_from = k;
    }
    r.safe_frames += out.mode == tfc::ActMode::Safe ? 1U : 0U;
    if (out.mode == tfc::ActMode::Safe && r.first_safe_frame == 0U) {
      r.first_safe_frame = k + 1U;
      r.safe_cause = out.cause;
      r.spread_at_safe_deg = frame_spread;
    }
    const tfc::DecodedAct wire = tfc::unpack_act_out(tfc::pack_act_out(tfc::to_act_frame(out), seq));
    const bool lost = k >= lp.act_lost_from && k < lp.act_lost_from + lp.act_lost_for;
    r.held_frames += lost ? 1U : 0U;
    pending = runner.end_of_frame(k, lost ? nullptr : &wire.act);
    const Tilts t = runner.vehicle().tilts();
    if (pad_now) {
      continue;  // nothing flies on the pad
    }
    if (r.liftoff_frame == 0xFFFFFFFFU && !runner.vehicle().on_ground() && runner.vehicle().altitude() > 0.0) {
      r.liftoff_frame = fk;
    }
    if (lp.trace != nullptr && lp.trace_every != 0U && fk % lp.trace_every == 0U) {
      const Vehicle6& veh = runner.vehicle();
      const Loads ld = veh.current_loads();
      TraceRow row;
      row.t = static_cast<double>(fk) * 0.01;
      row.altitude = veh.altitude();
      row.speed = veh.speed();
      row.mach = ld.mach;
      row.dynamic_pressure = ld.dynamic_pressure;
      row.mass = veh.mass();
      row.thrust = ld.thrust;
      row.tilt_y_deg = t.y_deg;
      row.tilt_x_deg = t.x_deg;
      const tfc::Reference rf = tables.guidance.at(fk + 1U);
      row.err_y_deg = t.y_deg - static_cast<double>(rf.tilt_y_deg);
      row.err_x_deg = t.x_deg - static_cast<double>(rf.tilt_x_deg);
      row.cmd_pitch_deg = static_cast<double>(out.pitch_deg);
      row.cmd_yaw_deg = static_cast<double>(out.yaw_deg);
      for (std::size_t sg = 0; sg < veh.spec().stages.size(); ++sg) {
        row.stages_active |= veh.stage_active(sg) ? (1U << sg) : 0U;
        row.stages_ignited |= veh.stage_ignited(sg) ? (1U << sg) : 0U;
      }
      row.engines_on = veh.engines_on();
      row.act_mode = static_cast<int>(out.mode);
      lp.trace->push_back(row);
    }
    r.crashed = r.crashed || runner.vehicle().crashed();
    r.min_altitude = std::fmin(r.min_altitude, runner.vehicle().altitude());
    const tfc::Reference ref = tables.guidance.at(fk + 1U);
    const double ep = t.y_deg - static_cast<double>(ref.tilt_y_deg);
    const double ey = t.x_deg - static_cast<double>(ref.tilt_x_deg);
    const double e = std::fmax(std::fabs(ep), std::fabs(ey));
    r.max_deg = std::fmax(r.max_deg, e);
    (fk < 300U ? r.max_deg_liftoff : r.max_deg_settled) = std::fmax(fk < 300U ? r.max_deg_liftoff : r.max_deg_settled, e);
    sum_sq += (ep * ep) + (ey * ey);
    r.platform_saturated += runner.platform().saturated() ? 1U : 0U;
    r.finite = r.finite && std::isfinite(t.x_deg) && std::isfinite(t.y_deg) && std::isfinite(static_cast<double>(out.pitch_deg));
    r.final_mode = out.mode;
    r.final_excluded = out.excluded_nodes;
    if (!r.finite) {
      break;
    }
  }
  r.rms_deg = std::sqrt(sum_sq / (2.0 * lp.frames));
  r.calibrated = lp.pad_frames > 0U && cal[0].ready() && cal[1].ready() && cal[2].ready();
  r.final_altitude = runner.vehicle().altitude();
  r.final_tilt_deg = runner.vehicle().tilts().y_deg;
  const tfc::DecodedSimFlags fl = tfc::unpack_sim_flags(pending.f[4]);
  r.flags = fl.ok ? fl.s.flags : 0xFFU;
  return r;
}

}  // namespace sim
