// SPDX-License-Identifier: MIT
// The pad phase (docs/design/LAUNCH_SEQUENCE.md): the per-IMU stationary gyro calibration, the flight function's mission state (schedules by flight time, not by the SYNC frame number), the runner's
// clamp and release, and the closed loop with a pad: what it changes at lift-off and for a biased gyro.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/flight.hpp"
#include "tfc/imu_calibration.hpp"
#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "runner.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

tfc::Vec3 vec(float x, float y, float z) {
  tfc::Vec3 v;
  v.v = {x, y, z};
  return v;
}

}  // namespace

TFC_TEST(calibrator_finds_the_bias_of_a_platform_at_rest_and_subtracts_it) {
  tfc::ImuCalibrator c;
  sim::Lcg noise(9U);
  c.set_pad(true);
  CHECK(c.pad() && !c.ready());
  tfc::Vec3 out;
  for (int k = 0; k < 1000; ++k) {
    const tfc::Vec3 raw = vec(1.0F + (0.17F * noise.uniform()), -2.0F + (0.17F * noise.uniform()), 0.5F + (0.17F * noise.uniform()));
    out = c.process(raw);
    if (k < 99) {
      CHECK(near_abs(static_cast<double>(out.v[0]), static_cast<double>(raw.v[0]), 1e-6));  // before 100 samples the raw value is passed on
    }
  }
  CHECK(c.ready() && c.samples() == 1000U);
  CHECK(near_abs(static_cast<double>(c.bias().v[0]), 1.0, 0.02) && near_abs(static_cast<double>(c.bias().v[1]), -2.0, 0.02) && near_abs(static_cast<double>(c.bias().v[2]), 0.5, 0.02));
  CHECK(std::fabs(out.v[0]) < 0.3F && std::fabs(out.v[1]) < 0.3F);  // the corrected sample is the noise only
  c.set_pad(false);  // lift-off: frozen
  const tfc::Vec3 b = c.bias();
  for (int k = 0; k < 100; ++k) {
    (void)c.process(vec(50.0F, 50.0F, 50.0F));  // a real rotation in flight must not move the bias
  }
  CHECK(c.bias().v[0] == b.v[0] && c.bias().v[1] == b.v[1] && c.samples() == 1000U);
  CHECK(near_abs(static_cast<double>(c.process(vec(11.0F, 0.0F, 0.0F)).v[0]), 11.0 - static_cast<double>(b.v[0]), 1e-5));
}

TFC_TEST(calibrator_is_not_ready_when_the_platform_moves_the_bias_is_implausible_or_samples_are_missing) {
  tfc::ImuCalibrator moving;
  moving.set_pad(true);
  for (int k = 0; k < 2000; ++k) {
    (void)moving.process(vec(10.0F * std::sin(0.1F * static_cast<float>(k)), 0.0F, 0.0F));  // someone is moving the platform
  }
  CHECK(!moving.ready());
  tfc::ImuCalibrator big;
  big.set_pad(true);
  for (int k = 0; k < 2000; ++k) {
    (void)big.process(vec(7.0F, 0.0F, 0.0F));  // steady, but a 7 dps bias is not credible
  }
  CHECK(!big.ready());
  tfc::ImuCalibrator few;
  few.set_pad(true);
  for (int k = 0; k < 999; ++k) {
    (void)few.process(vec(0.1F, 0.0F, 0.0F));
  }
  CHECK(!few.ready());
  (void)few.process(vec(0.1F, 0.0F, 0.0F));
  CHECK(few.ready());
  tfc::ImuCalibrator nan;
  nan.set_pad(true);
  const float inf = std::numeric_limits<float>::infinity();
  for (int k = 0; k < 1500; ++k) {
    (void)nan.process(vec(inf, std::nanf(""), 0.0F));  // not numbers: not counted
  }
  CHECK(nan.samples() == 0U && !nan.ready());
  tfc::ImuCalibrator one;
  tfc::ImuCalibratorConfig tiny;
  tiny.samples = 0U;
  tfc::ImuCalibrator zero(tiny);
  zero.set_pad(true);
  (void)zero.process(vec(0.0F, 0.0F, 0.0F));
  CHECK(!zero.ready());  // a single sample has no spread
  (void)one.process(vec(1.0F, 1.0F, 1.0F));
  CHECK(near_abs(static_cast<double>(one.process(vec(1.0F, 1.0F, 1.0F)).v[0]), 1.0, 1e-6) && one.samples() == 0U);  // never on the pad: nothing is learnt
}

TFC_TEST(calibrator_going_back_to_the_pad_starts_again_and_a_missed_calibration_keeps_the_running_mean) {
  tfc::ImuCalibrator c;
  c.set_pad(true);
  for (int k = 0; k < 1000; ++k) {
    (void)c.process(vec(1.0F, 0.0F, 0.0F));
  }
  c.set_pad(false);
  c.set_pad(false);  // idempotent
  c.set_pad(true);   // a scrub: collect again, with the old bias in use meanwhile
  CHECK(c.samples() == 0U && near_abs(static_cast<double>(c.bias().v[0]), 1.0, 1e-5));
  for (int k = 0; k < 300; ++k) {
    (void)c.process(vec(3.0F, 0.0F, 0.0F));
  }
  c.set_pad(false);  // lift-off with too few samples: the running mean (3.0) is what was in use
  CHECK(!c.ready() && near_abs(static_cast<double>(c.bias().v[0]), 3.0, 1e-4));
  tfc::ImuCalibrator many;
  many.set_pad(true);
  for (int k = 0; k < 61000; ++k) {
    (void)many.process(vec(0.2F, 0.0F, 0.0F));
  }
  CHECK(many.samples() == 60000U && many.ready());  // the count stops
}

TFC_TEST(calibrator_with_fewer_required_samples_than_the_apply_threshold_is_applied_at_lift_off) {
  tfc::ImuCalibratorConfig cfg;
  cfg.samples = 50U;
  cfg.apply_after = 100U;
  tfc::ImuCalibrator c(cfg);
  c.set_pad(true);
  for (int k = 0; k < 60; ++k) {
    (void)c.process(vec(1.0F, 0.0F, 0.0F));
  }
  CHECK(c.ready() && near_abs(static_cast<double>(c.bias().v[0]), 0.0, 1e-6));  // ready, but the running mean is not in use yet
  c.set_pad(false);                                                              // lift-off applies it
  CHECK(near_abs(static_cast<double>(c.bias().v[0]), 1.0, 1e-5));
}

TFC_TEST(flight_function_schedules_follow_the_flight_frame_not_the_sync_frame_number) {
  const sim::FlightTables t = sim::flight_tables(sim::Params{});
  tfc::FlightFunction plain(t.gains, t.guidance);
  tfc::FlightFunction mission(t.gains, t.guidance);
  tfc::Vec3 gyro;
  tfc::Vec3 accel;
  accel.v = {0.0F, 0.0F, 1.0F};
  for (uint32_t k = 0; k < 3000U; ++k) {
    const uint32_t sync = k + 7000U;  // the SYNC numbering started long before T-zero
    plain.begin_frame(k, 0x07U);
    mission.begin_frame(sync, 0x07U);
    mission.set_mission(false, k);
    for (uint8_t n = 0; n < 3U; ++n) {
      (void)plain.on_frame(tfc::pack_gyro(n, gyro, static_cast<uint8_t>(k)));
      (void)plain.on_frame(tfc::pack_accel(n, accel, static_cast<uint8_t>(k)));
      (void)mission.on_frame(tfc::pack_gyro(n, gyro, static_cast<uint8_t>(sync)));
      (void)mission.on_frame(tfc::pack_accel(n, accel, static_cast<uint8_t>(sync)));
    }
    const tfc::Command a = plain.step();
    const tfc::Command b = mission.step();
    CHECK(a.pitch_deg == b.pitch_deg && a.yaw_deg == b.yaw_deg && a.state_digest == b.state_digest);
  }
  // on the pad the schedules stay at their first point whatever the frame number
  tfc::FlightFunction pad(t.gains, t.guidance);
  CHECK(!pad.sensors_ok());
  pad.begin_frame(5000U, 0x07U);
  pad.set_mission(true, 0U);
  for (uint8_t n = 0; n < 3U; ++n) {
    (void)pad.on_frame(tfc::pack_gyro(n, gyro, static_cast<uint8_t>(5000U)));
    (void)pad.on_frame(tfc::pack_accel(n, accel, static_cast<uint8_t>(5000U)));
  }
  CHECK(!pad.sensors_ok());  // frames heard, but the estimator has not yet processed a trustworthy gyro: the attitude is not valid
  const tfc::Command c = pad.step();
  CHECK(near_abs(static_cast<double>(c.pitch_deg), 0.0, 0.05) && pad.sensors_ok());
  // on the pad the schedule index is 0 even if the caller passes a flight frame
  tfc::FlightFunction strict(t.gains, t.guidance);
  strict.begin_frame(0U, 0x07U);
  strict.set_mission(true, 3000U);  // at flight frame 3000 the program asks for about 6 degrees
  for (uint8_t n = 0; n < 3U; ++n) {
    (void)strict.on_frame(tfc::pack_gyro(n, gyro, 0U));
    (void)strict.on_frame(tfc::pack_accel(n, accel, 0U));
  }
  CHECK(near_abs(static_cast<double>(strict.step().pitch_deg), 0.0, 0.05));
}

TFC_TEST(runner_the_vehicle_stands_on_the_pad_at_rest_until_released_and_the_schedules_start_at_release) {
  for (const bool vehicle_true : {false, true}) {
    sim::RunnerConfig cfg;
    cfg.start_held = true;
    cfg.vehicle_true = vehicle_true;
    sim::SimRunner runner(cfg);
    sim::SimFrames f = runner.start(0U);
    CHECK(runner.clamped());
    tfc::ActFrame wild;
    wild.pitch_deg = 8.0F;  // whatever ACT says, nothing moves on the pad
    for (uint32_t k = 0; k < 500U; ++k) {
      f = runner.end_of_frame(k, &wild);
      const tfc::DecodedVec3 g = tfc::unpack_vec3(f.f[0], tfc::kGyroLsbDps);
      const tfc::DecodedVec3 a = tfc::unpack_vec3(f.f[1], tfc::kAccelLsbG);
      CHECK(g.ok && a.ok && g.x.v[0] == 0.0F && g.x.v[1] == 0.0F && g.x.v[2] == 0.0F);
      CHECK(near_abs(static_cast<double>(a.x.v[2]), 1.0, 0.001) && near_abs(static_cast<double>(a.x.v[0]), 0.0, 0.001));
    }
    CHECK(runner.vehicle().altitude() < 1e-6 && runner.flight_frame() == 0U && runner.vehicle().time() == 0.0);
    runner.release();
    CHECK(!runner.clamped());
    const tfc::ActFrame neutral;
    for (uint32_t k = 500; k < 1000U; ++k) {
      f = runner.end_of_frame(k, &neutral);
    }
    CHECK(runner.vehicle().altitude() > 10.0 && runner.flight_frame() == 500U);  // it flies, and flight time counts from the release
    const tfc::DecodedSimFlags fl = tfc::unpack_sim_flags(f.f[4]);
    CHECK(fl.ok && fl.s.time_frames == 500U);
  }
}

TFC_TEST(closed_loop_with_a_pad_the_computers_become_ready_calibrate_and_fly_the_same_nominal_flight) {
  sim::Loop lp;
  lp.pad_frames = 1500U;
  lp.frames = 8000U;
  const sim::Result r = sim::run(lp);
  CHECK(r.finite && r.calibrated && r.safe_frames == 0U);
  CHECK(r.ready_at >= 999U && r.ready_at <= 1100U);  // ten seconds of rest data, plus a few frames for ACT and the consensus
  CHECK(r.nominal_from < 120U);                       // ACT is Nominal on the pad, before the release
  CHECK(r.max_deg < 0.6 && r.rms_deg < 0.15);
  sim::Loop dirty;  // someone is moving the platform on the pad: it is not ready
  dirty.pad_frames = 1500U;
  dirty.frames = 100U;
  dirty.sensors.gyro_noise_amp_dps = 3.0F;  // far beyond the rest spread
  CHECK(sim::run(dirty).ready_at == 0U);
}

TFC_TEST(closed_loop_a_pad_removes_the_lift_off_transient_of_a_misaligned_engine_and_the_bias_of_a_real_gyro) {
  sim::Loop no_pad;
  no_pad.frames = 4000U;
  no_pad.cfg.params.thrust_misalign_pitch_deg = 1.0;
  const sim::Result a = sim::run(no_pad);
  sim::Loop pad = no_pad;
  pad.pad_frames = 1500U;
  const sim::Result b = sim::run(pad);
  CHECK(a.max_deg_liftoff > 4.0 && b.max_deg_liftoff < 2.0);  // ACT is already Nominal at the release, so the gimbal answers at once
  sim::Loop vehicle;  // a vehicle: gravity is not seen under thrust, so a gyro bias must be calibrated out on the pad
  vehicle.cfg.vehicle_true = true;
  vehicle.estimator.use_accel = false;
  vehicle.sensors.gyro_bias_dps = 2.0F;
  vehicle.frames = 6000U;
  const sim::Result c = sim::run(vehicle);
  vehicle.pad_frames = 1500U;
  const sim::Result d = sim::run(vehicle);
  CHECK(c.max_deg_settled > 5.0 && d.max_deg_settled < 2.0);
}

TFC_TEST(runner_a_simulator_started_after_t_zero_joins_the_flight_in_progress) {
  sim::SimRunner runner;
  (void)runner.start_in_flight(4000U, 500U);  // SYNC says frame 4000, 500 frames after T-zero
  CHECK(!runner.clamped() && runner.flight_frame() == 500U && runner.frame() == 4000U);
  CHECK(runner.vehicle().altitude() > 10.0 && runner.vehicle().time() > 4.9);  // it has flown five seconds with the gimbal neutral
  const tfc::ActFrame neutral;
  const sim::SimFrames f = runner.end_of_frame(4000U, &neutral);
  CHECK(runner.flight_frame() == 501U && f.n >= 2U);
  sim::SimRunner early;  // a system frame number smaller than the flight age is possible (the numbering restarted): no underflow
  (void)early.start_in_flight(10U, 500U);
  CHECK(early.flight_frame() == 500U);
}
