// SPDX-License-Identifier: MIT
// The dispersions of the vehicle model (gimbal lag, thrust misalignment, thrust and aerodynamic scale) and the IMU error model (bias, scale, misalignment, latency, stale samples): each does
// what it says, and with everything at its default neither changes anything. These are what tools/sim/tfc_sens.cpp sweeps.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "vehicle6.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

}  // namespace

TFC_TEST(realism_a_gimbal_lag_gives_a_first_order_response) {
  sim::Params p;
  p.gimbal_lag_s = 0.1;
  p.gimbal_rate_dps = 1.0e6;  // out of the way: only the lag acts
  sim::Vehicle6 v(p);
  v.step(0.1, 4.0, 0.0);  // one time constant
  CHECK(near_abs(v.gimbal_pitch_deg(), 4.0 * (1.0 - std::exp(-1.0)), 0.08));
  v.step(0.5, 4.0, 0.0);
  CHECK(near_abs(v.gimbal_pitch_deg(), 4.0, 0.05));
  sim::Vehicle6 nominal;  // no lag: the rate limit alone (60 degrees per second): 4 degrees in 67 ms
  nominal.step(0.1, 4.0, 0.0);
  CHECK(near_abs(nominal.gimbal_pitch_deg(), 4.0, 1e-9));
}

TFC_TEST(realism_a_thrust_misalignment_is_a_constant_moment_in_the_right_direction) {
  sim::Params p;
  p.thrust_misalign_pitch_deg = 1.0;
  sim::Vehicle6 v(p);
  const sim::Loads l = v.current_loads();
  const double expected = l.thrust * v.mass_props(v.mass()).x_cg * std::sin(1.0 * sim::kDeg2Rad);
  CHECK(l.m_thrust.z > 0.0 && near_abs(l.m_thrust.z, expected, 0.02 * expected));  // positive pitch raises the tilt, as a positive gimbal command does
  p.thrust_misalign_pitch_deg = 0.0;
  p.thrust_misalign_yaw_deg = 1.0;
  sim::Vehicle6 w(p);
  CHECK(w.current_loads().m_thrust.y > 0.0 && std::fabs(w.current_loads().m_thrust.z) < 1e-6);
  sim::Vehicle6 n;
  CHECK(near_abs(n.current_loads().m_thrust.z, 0.0, 1e-6) && near_abs(n.current_loads().m_thrust.y, 0.0, 1e-6));  // none by default
}

TFC_TEST(realism_thrust_and_aerodynamic_scales_scale_the_forces) {
  sim::Params p;
  p.thrust_scale = 0.9;
  sim::Vehicle6 weak(p);
  sim::Vehicle6 nominal;
  const sim::Loads lw = weak.current_loads();
  const sim::Loads ln = nominal.current_loads();
  const double ambient = 101325.0 * 0.12 * 5.0;  // the pressure term of all five engines at sea level
  CHECK(near_abs(lw.thrust, (0.9 * 5.0 * 92000.0) - ambient, 1.0) && near_abs(ln.thrust, (5.0 * 92000.0) - ambient, 1.0));
  CHECK(near_abs(lw.mdot, 0.9 * ln.mdot, 1e-9));  // the same Isp: the mass flow scales with the thrust
  sim::Params q;
  q.cd_scale = 2.0;
  q.cn_scale = 3.0;
  sim::State s;
  s.r = sim::V3{sim::kEarthR + 10000.0, 0.0, 0.0};
  s.v = sim::V3{250.0, 5.0, 0.0};
  sim::Vehicle6 a;
  sim::Vehicle6 b(q);
  a.set_state(s);
  b.set_state(s);
  CHECK(near_abs(b.current_loads().f_aero.x, 2.0 * a.current_loads().f_aero.x, 1e-6 * std::fabs(a.current_loads().f_aero.x)));
  CHECK(near_abs(b.current_loads().f_aero.y, 3.0 * a.current_loads().f_aero.y, 1e-6 * std::fabs(a.current_loads().f_aero.y)));
}

TFC_TEST(realism_the_imu_model_is_the_old_noise_by_default_and_each_error_does_what_it_says) {
  sim::SensorErrors none;
  sim::ImuModel m(none, 7U);
  sim::Lcg ref(7U);
  tfc::Vec3 g_true;
  tfc::Vec3 a_true;
  g_true.v = {10.0F, -20.0F, 30.0F};
  a_true.v = {0.1F, 0.2F, 0.9F};
  for (int k = 0; k < 50; ++k) {  // by default: the true value plus the uniform noise, draws in the original order
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(g_true, a_true, g, a);
    for (unsigned i = 0; i < 3U; ++i) {
      const float ge = g_true.v[i] + (0.17F * ref.uniform());
      const float ae = a_true.v[i] + (0.0035F * ref.uniform());
      CHECK(g.v[i] == ge && a.v[i] == ae);
    }
  }
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.gyro_bias_dps = 1.0F;
  e.gyro_scale_err = 0.1F;
  sim::ImuModel b1(e, 1U);
  sim::ImuModel b2(e, 2U);
  tfc::Vec3 g1;
  tfc::Vec3 g2;
  tfc::Vec3 a1;
  tfc::Vec3 a2;
  tfc::Vec3 zero;
  b1.sample(zero, a_true, g1, a1);
  b2.sample(zero, a_true, g2, a2);
  CHECK(std::fabs(g1.v[0]) <= 1.0F && std::fabs(g1.v[1]) <= 1.0F && std::fabs(g1.v[0] - g2.v[0]) > 1e-4F);  // a bias within the bound, different per node
  tfc::Vec3 g3;
  tfc::Vec3 g4;
  b1.sample(zero, a_true, g3, a1);
  CHECK(g3.v[0] == g1.v[0]);  // and constant in time
  b1.sample(g_true, a_true, g4, a1);
  CHECK(std::fabs((g4.v[0] - g1.v[0]) - (10.0F * (g4.v[0] - g1.v[0]) / 10.0F)) < 1e-5F && std::fabs(g4.v[0] - g1.v[0]) > 8.9F && std::fabs(g4.v[0] - g1.v[0]) < 11.1F);  // gain within 10%
}

TFC_TEST(realism_latency_delays_the_sample_by_whole_frames_and_stale_samples_repeat) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.latency_frames = 3U;
  sim::ImuModel m(e, 1U);
  tfc::Vec3 a;
  for (int k = 0; k < 20; ++k) {
    tfc::Vec3 g_true;
    g_true.v = {static_cast<float>(k), 0.0F, 0.0F};
    tfc::Vec3 g;
    m.sample(g_true, a, g, a);
    CHECK(k < 3 ? g.v[0] == 0.0F : near_abs(static_cast<double>(g.v[0]), static_cast<double>(k - 3), 1e-6));  // the ramp comes out three frames late
  }
  sim::SensorErrors s;
  s.gyro_noise_amp_dps = 0.0F;
  s.accel_noise_amp_g = 0.0F;
  s.stale_prob = 0.5F;
  sim::ImuModel st(s, 5U);
  unsigned repeats = 0U;
  float last = -1.0F;
  for (int k = 1; k <= 2000; ++k) {
    tfc::Vec3 g_true;
    g_true.v = {static_cast<float>(k), 0.0F, 0.0F};
    tfc::Vec3 g;
    st.sample(g_true, a, g, a);
    repeats += g.v[0] == last ? 1U : 0U;
    last = g.v[0];
  }
  CHECK(repeats > 800U && repeats < 1200U);  // about half of the frames repeat the previous sample
}

TFC_TEST(realism_a_misaligned_imu_sees_a_rotated_gravity_vector) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.misalign_deg = 5.0F;
  sim::ImuModel m(e, 3U);
  tfc::Vec3 g_true;
  tfc::Vec3 a_true;
  a_true.v = {0.0F, 0.0F, 1.0F};
  tfc::Vec3 g;
  tfc::Vec3 a;
  m.sample(g_true, a_true, g, a);
  const double ax = static_cast<double>(a.v[0]);
  const double ay = static_cast<double>(a.v[1]);
  const double az = static_cast<double>(a.v[2]);
  const double mag = std::sqrt((ax * ax) + (ay * ay) + (az * az));
  const double tilt = std::acos(az / mag) * sim::kRad2Deg;
  CHECK(near_abs(mag, 1.0, 1e-5) && tilt > 0.1 && tilt < 10.0);  // a rotation: the length is kept, the direction moves by a few degrees
}

TFC_TEST(realism_the_closed_loop_with_default_dispersions_is_the_nominal_flight_and_a_dispersed_one_is_not) {
  sim::Loop nominal;
  nominal.frames = 3000U;
  const sim::Result a = sim::run(nominal);
  sim::Loop again = nominal;
  const sim::Result b = sim::run(again);
  CHECK(a.max_deg == b.max_deg && a.rms_deg == b.rms_deg && a.safe_frames == 0U);  // deterministic
  sim::Loop worse = nominal;
  worse.cfg.params.thrust_misalign_pitch_deg = 2.0;
  const sim::Result c = sim::run(worse);
  CHECK(c.max_deg_liftoff > 5.0 * a.max_deg_liftoff);  // the pad hold is missing: ACT is in Standby for the first second and the misaligned engines turn the vehicle
}

TFC_TEST(ts16_a_clean_bus_keeps_the_three_state_digests_equal_and_a_lossy_one_measurably_does_not) {
  sim::Loop clean;
  clean.frames = 3000U;
  const sim::Result a = sim::run(clean);
  CHECK(a.lost_frames == 0U && a.digest_mismatch_frames == 0U && a.max_command_spread_deg == 0.0);  // identical inputs: identical states
  sim::Loop lossy = clean;
  lossy.frame_loss_prob = 0.01F;
  const sim::Result b = sim::run(lossy);
  CHECK(b.lost_frames > 100U && b.lost_frames < 500U);  // 6 frames x 2 receivers x 3000 frames x 1% (the own frames are never lost) = 240 expected
  CHECK(b.digest_mismatch_frames > 0U && b.longest_mismatch_run >= 1U);  // TS-16: the replicated estimators drift apart
  const sim::Result c = sim::run(lossy);
  CHECK(b.digest_mismatch_frames == c.digest_mismatch_frames && b.max_command_spread_deg == c.max_command_spread_deg);  // and it is repeatable
}

TFC_TEST(edge_a_flight_past_burnout_stays_finite_and_ends_with_the_engines_off) {
  sim::Loop lp;
  lp.frames = 30000U;  // 300 s: the 24 t of propellant burn out at about 158 s; the tables stop at 100 s and hold their last values
  const sim::Result r = sim::run(lp);
  CHECK(r.finite);
  sim::SimRunner runner;  // and the vehicle itself, flown with the gimbal neutral to the end of the propellant
  (void)runner.start(0U);
  tfc::ActFrame neutral;
  for (uint32_t k = 0; k < 30000U; ++k) {
    (void)runner.end_of_frame(k, &neutral);
  }
  CHECK(!runner.vehicle().burning() && runner.vehicle().mass() >= runner.vehicle().params().m_dry - 1e-6 && std::isfinite(runner.vehicle().altitude()));
  CHECK(runner.vehicle().current_loads().thrust == 0.0);
}

TFC_TEST(edge_a_total_loss_of_the_flight_computers_sends_acts_vehicle_into_safe_and_the_run_is_marked) {
  sim::Loop lp;
  lp.frames = 8000U;
  lp.all_dead_from = 3000U;  // at 30 s no flight computer sends anything
  const sim::Result r = sim::run(lp);
  CHECK(r.finite);
  CHECK(r.safe_frames > 4000U && r.final_mode == tfc::ActMode::Safe);   // lost votes: Safe within a few frames, and it stays
  CHECK((r.flags & tfc::simflag::kSafed) != 0U);                        // the simulator marks the run
  CHECK(r.platform_saturated > 0U || r.max_deg > 10.0);                  // an unstable vehicle with no control leaves the platform's range: the rig would hit its stops
}

TFC_TEST(edge_the_runner_survives_the_frame_number_going_backwards_and_a_gap) {
  sim::SimRunner runner;
  (void)runner.start(500U);
  CHECK(runner.frame() == 500U);
  (void)runner.start(10U);  // a new run: the world is rebuilt at frame 10
  CHECK(runner.frame() == 10U && runner.vehicle().time() < 0.2);
  tfc::ActFrame a;
  for (uint32_t k = 10; k < 1010U; ++k) {  // stepped without ACT's frame for a second: held, and counted
    (void)runner.end_of_frame(k, k < 500U ? &a : nullptr);
  }
  CHECK(runner.frame() == 1010U && runner.command_held());
}
