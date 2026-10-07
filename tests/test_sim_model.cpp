// SPDX-License-Identifier: MIT
// The vehicle simulator's ground, and the behaviours that the deliberate-bug sweep of 6 Oct 2026 (tools/mutation/run_sim.py) found no test pinning: the sign of an
// engine-out in the other plane, the platform's stops and its cross-axis rates, the IMU model's draws and its stale samples, the runner's "safed" flag, the quaternion
// norm, the atmosphere's constants and the roll momentum. See docs/design/VEHICLE_SIM.md and docs/design/SIM_FIDELITY.md.
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/protocol.hpp"
#include "tfc_test.hpp"

#include "atmosphere.hpp"
#include "closed_loop.hpp"
#include "imu_model.hpp"
#include "platform.hpp"
#include "runner.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// The surface gravity the model uses (inverse square on its spherical Earth).
double surface_gravity() { return sim::kEarthMu / (sim::kEarthR * sim::kEarthR); }

bool same_state(const sim::State& a, const sim::State& b) {
  return a.r.x == b.r.x && a.r.y == b.r.y && a.r.z == b.r.z && a.v.x == b.v.x && a.v.y == b.v.y && a.v.z == b.v.z && a.q.w == b.q.w && a.q.x == b.q.x && a.q.y == b.q.y &&
         a.q.z == b.q.z && a.w.x == b.w.x && a.w.y == b.w.y && a.w.z == b.w.z && a.m == b.m;
}

}  // namespace

// ---- the ground ----

TFC_TEST(ground_a_vehicle_with_less_than_one_g_of_thrust_stands_on_the_pad_and_rises_when_it_is_light_enough) {
  sim::Params p;
  p.thrust_scale = 0.70;  // 261 kN against a 294 kN weight: it cannot lift off (found 6 Oct 2026: the model had no ground and the vehicle sank through the pad)
  sim::Vehicle6 v(p);
  const double mdot = v.current_loads().mdot;
  for (int k = 0; k < 500; ++k) {
    v.step(0.01, 0.0, 0.0);  // 5 s
  }
  CHECK(v.on_ground() && !v.crashed());
  CHECK(near_abs(v.altitude(), 0.0, 1e-6) && v.speed() == 0.0);
  CHECK(near_abs(v.mass(), 30000.0 - (5.0 * mdot), 1e-6));  // it stands there burning propellant
  // It rises when thrust / weight reaches 1: the thrust at the surface is 5 (0.7 * 92000 - 101325 * 0.12) N, the weight m g.
  const double thrust = 5.0 * ((0.70 * 92000.0) - (101325.0 * 0.12));
  const double mass_to_rise = thrust / surface_gravity();
  const double expected_t = (30000.0 - mass_to_rise) / mdot;
  double t_rise = -1.0;
  for (int k = 500; k < 6000 && t_rise < 0.0; ++k) {
    v.step(0.01, 0.0, 0.0);
    if (!v.on_ground()) {
      t_rise = v.time();
    }
  }
  CHECK(t_rise > 0.0 && near_abs(t_rise, expected_t, 0.1));
  for (int k = 0; k < 2000; ++k) {
    v.step(0.01, 0.0, 0.0);  // 20 s more: it has hardly any thrust to spare when it first rises, so it climbs slowly
  }
  CHECK(!v.on_ground() && v.altitude() > 10.0 && !v.crashed());  // and it climbs from there
}

TFC_TEST(ground_without_the_ground_a_weak_vehicle_sinks_through_the_pad) {
  sim::Params p;
  p.thrust_scale = 0.50;
  p.ground_contact = false;  // the bare equations
  sim::Vehicle6 v(p);
  for (int k = 0; k < 1000; ++k) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.altitude() < -100.0 && !v.on_ground());
}

TFC_TEST(ground_a_vehicle_that_lifts_off_at_once_is_not_touched_by_it) {
  sim::Params with;
  sim::Params without;
  without.ground_contact = false;
  sim::Vehicle6 a(with);
  sim::Vehicle6 b(without);
  for (int k = 0; k < 1500; ++k) {  // 15 s, with a command, so that the attitude moves too
    const double c = 0.5 * std::sin(0.01 * k);
    a.step(0.01, c, -c);
    b.step(0.01, c, -c);
  }
  CHECK(same_state(a.state(), b.state()));  // exactly, not approximately: the ground adds no arithmetic to a flight
  CHECK(!a.on_ground() && a.altitude() > 100.0);
}

TFC_TEST(ground_a_vehicle_that_comes_down_gently_lands_and_one_that_comes_down_fast_is_destroyed) {
  sim::Params p;
  p.thrust_scale = 0.0;  // no engines: a falling body
  sim::State s;
  s.r = sim::V3{sim::kEarthR + 0.5, 0.0, 0.0};
  s.v = sim::V3{-1.0, 0.0, 0.0};  // it arrives at about 3.3 m/s, under the 5 m/s crash speed
  s.m = 12000.0;
  sim::Vehicle6 soft(p);
  soft.set_state(s);
  for (int k = 0; k < 600; ++k) {
    soft.step(0.01, 0.0, 0.0);
  }
  CHECK(soft.on_ground() && !soft.crashed());
  CHECK(near_abs(soft.altitude(), 0.0, 1e-9) && soft.speed() == 0.0);  // it stays where it landed
  s.v = sim::V3{-1.0, 0.0, 0.0};
  s.w = sim::V3{0.0, 0.2, 0.1};  // turning as it lands: the ground stops that too
  sim::Vehicle6 spinning(p);
  spinning.set_state(s);
  for (int k = 0; k < 600; ++k) {
    spinning.step(0.01, 0.0, 0.0);
  }
  CHECK(spinning.on_ground() && spinning.state().w.x == 0.0 && spinning.state().w.y == 0.0 && spinning.state().w.z == 0.0);
  s.w = sim::V3{};
  s.r = sim::V3{sim::kEarthR + 5.0, 0.0, 0.0};
  s.v = sim::V3{-80.0, 0.0, 0.0};
  sim::Vehicle6 hard(p);
  hard.set_state(s);
  for (int k = 0; k < 600; ++k) {
    hard.step(0.01, 0.0, 0.0);
  }
  CHECK(hard.crashed() && hard.on_ground());
  const sim::State frozen = hard.state();
  hard.step(1.0, 0.0, 0.0);
  CHECK(same_state(frozen, hard.state()));  // a destroyed vehicle stays as it was
  sim::Params limit = p;
  limit.crash_speed_ms = 100.0;  // the limit is a parameter
  sim::Vehicle6 sturdy(limit);
  sturdy.set_state(s);
  for (int k = 0; k < 600; ++k) {
    sturdy.step(0.01, 0.0, 0.0);
  }
  CHECK(!sturdy.crashed() && sturdy.on_ground());
}

TFC_TEST(ground_the_closed_loop_reports_when_the_vehicle_left_the_pad_and_whether_it_did) {
  sim::Loop nominal;
  nominal.frames = 600U;
  const sim::Result r = sim::run(nominal);
  CHECK(r.liftoff_frame <= 1U && !r.crashed && near_abs(r.min_altitude, 0.0, 1e-6));
  sim::Loop weak;
  weak.frames = 1000U;
  weak.cfg.params.thrust_scale = 0.70;  // it needs about 31 s to become light enough
  const sim::Result w = sim::run(weak);
  CHECK(w.liftoff_frame == 0xFFFFFFFFU && !w.crashed && near_abs(w.min_altitude, 0.0, 1e-6));
}

TFC_TEST(ground_an_unguided_vehicle_tumbles_comes_back_and_is_destroyed) {
  sim::SimRunner runner;  // the aerodynamically unstable vehicle with a neutral gimbal and no control: it turns over within 40 s, falls back and hits the pad
  (void)runner.start(0U);
  tfc::ActFrame neutral;
  uint32_t k = 0U;
  for (; k < 6000U && !runner.vehicle().crashed(); ++k) {
    (void)runner.end_of_frame(k, &neutral);
  }
  CHECK(runner.vehicle().crashed() && runner.vehicle().on_ground());
  CHECK(runner.vehicle().time() > 30.0 && runner.vehicle().time() < 60.0);
  CHECK(runner.vehicle().burning());  // it had propellant left: it did not burn out first
}

// ---- engines in the other plane ----

TFC_TEST(engine_out_in_the_yaw_plane_mirrors_the_pitch_plane_and_has_the_right_sign) {
  double tilt[5][2] = {};  // [engine][0: yaw plane tilt x, 1: pitch plane tilt y] after 7 s with the engine out from 5 s
  for (int e = 1; e <= 4; ++e) {
    sim::Params p;
    sim::Scenario sc;
    sc.wind_scale = 0.0;  // the default crosswind would add a yaw tilt of its own
    sc.engine_out_time = 5.0;
    sc.engine_out_index = e;
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 700; ++k) {
      v.step(0.01, 0.0, 0.0);
    }
    tilt[e][0] = v.tilts().x_deg;
    tilt[e][1] = v.tilts().y_deg;
  }
  // engines 1, 2 sit at +y, -y: losing one pushes the nose toward +y (pitch tilt positive), -y; engines 3, 4 at +z, -z: toward +z (tilt x is positive toward -z, so negative), -z
  CHECK(tilt[1][1] > 5.0 && tilt[2][1] < -5.0 && tilt[3][0] < -5.0 && tilt[4][0] > 5.0);
  CHECK(close(tilt[3][0], -tilt[1][1], 1e-6) && close(tilt[4][0], -tilt[2][1], 1e-6));  // the planes are the same by symmetry
  CHECK(near_abs(tilt[1][0], 0.0, 1e-9) && near_abs(tilt[3][1], 0.0, 1e-9));              // and an engine in one plane does not tilt the other
}

// ---- the aerodynamic force and the flow direction ----

TFC_TEST(aerodynamics_there_is_no_force_while_the_vehicle_flies_tail_first_a_stated_limit_of_the_model) {
  sim::Params p;
  p.ground_contact = false;
  sim::State s;
  s.r = sim::V3{sim::kEarthR + 5000.0, 0.0, 0.0};
  s.v = sim::V3{100.0, 0.0, 0.0};  // along +X inertial, the body's +X: nose first
  s.m = 20000.0;
  sim::Vehicle6 fwd(p);
  fwd.set_state(s);
  CHECK(fwd.current_loads().f_aero.x < -1000.0);  // drag
  s.v = sim::V3{-100.0, 0.0, 0.0};                // the same speed, tail first
  sim::Vehicle6 back(p);
  back.set_state(s);
  const sim::Loads l = back.current_loads();
  CHECK(l.dynamic_pressure > 1000.0 && l.f_aero.x == 0.0 && l.f_aero.y == 0.0 && l.f_aero.z == 0.0);  // documented: VEHICLE_SIM section 3
}

// ---- state handling ----

TFC_TEST(the_attitude_quaternion_is_renormalised_every_step) {
  sim::Params p;
  p.ground_contact = false;
  sim::Vehicle6 v(p);
  sim::State s = v.state();
  s.q = sim::Q4{1.002, 0.0, 0.0, 0.0};  // not a unit quaternion
  v.set_state(s);
  v.step(0.01, 0.0, 0.0);
  const sim::Q4 q = v.state().q;
  CHECK(near_abs(std::sqrt((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z)), 1.0, 1e-12));
}

TFC_TEST(without_a_roll_controller_the_roll_momentum_is_conserved_while_the_inertia_falls) {
  sim::Params p;
  p.ideal_roll_control = false;
  sim::Vehicle6 v(p);
  sim::State s = v.state();
  s.w.x = 0.5;  // rad/s about the long axis
  v.set_state(s);
  const double l0 = v.mass_props(v.mass()).i_x * v.state().w.x;
  for (int k = 0; k < 3000; ++k) {
    v.step(0.01, 0.0, 0.0);  // 30 s of burn: the roll inertia falls with the propellant
  }
  const double l1 = v.mass_props(v.mass()).i_x * v.state().w.x;
  CHECK(v.mass_props(v.mass()).i_x < 0.9 * v.mass_props(30000.0).i_x);
  CHECK(close(l0, l1, 2e-3));  // I w' = M - w x (I w) - I' w with no roll torque: I w is constant
}

// ---- the platform ----

TFC_TEST(platform_at_its_stop_holds_the_angle_and_reports_no_rotation) {
  sim::Platform plat;
  sim::Tilts target;
  target.y_deg = 60.0;  // beyond the 45 degree travel
  target.x_deg = -50.0;
  for (int k = 0; k < 500; ++k) {
    plat.step(target, 0.01);
  }
  CHECK(plat.saturated());
  CHECK(near_abs(plat.tilts().y_deg, 45.0, 1e-9) && near_abs(plat.tilts().x_deg, -45.0, 1e-9));
  sim::V3 gyro;
  sim::V3 acc;
  plat.imu_truth(gyro, acc);
  CHECK(near_abs(gyro.x, 0.0, 1e-6) && near_abs(gyro.y, 0.0, 1e-6) && near_abs(gyro.z, 0.0, 1e-6));  // it is at the stop: it does not turn, so the gyros read nothing
}

TFC_TEST(platform_body_rates_and_gravity_are_those_of_the_attitude_it_has) {
  // The platform's attitude is a rotation about X by tx then about Y by ty (body to inertial q = Ry(ty) Rx(tx)). Its gyros must read the rotation between two instants,
  // in the body frame, and its accelerometers gravity in the body frame, with both angles non-zero (a mistake in a cross term shows only then).
  sim::Platform plat;
  const auto attitude = [](const sim::Tilts& t) {
    return sim::from_axis_angle(sim::V3{0.0, 1.0, 0.0}, t.y_deg * sim::kDeg2Rad) * sim::from_axis_angle(sim::V3{1.0, 0.0, 0.0}, t.x_deg * sim::kDeg2Rad);
  };
  sim::Tilts target;
  target.x_deg = 20.0;
  target.y_deg = 30.0;
  for (int k = 0; k < 8; ++k) {
    plat.step(target, 0.01);  // part-way to the target (the servo's lag is 60 ms): both angles non-zero and still changing fast
  }
  CHECK(plat.tilts().x_deg > 3.0 && plat.tilts().x_deg < 19.0 && plat.tilts().y_deg > 3.0 && plat.tilts().y_deg < 29.0);
  const sim::Q4 q0 = attitude(plat.tilts());
  plat.step(target, 1e-4);  // one more, short step: the rotation it makes is the rate the platform reports
  const sim::Q4 q1 = attitude(plat.tilts());
  sim::V3 gyro;
  sim::V3 acc;
  plat.imu_truth(gyro, acc);
  const sim::V3 g_body = sim::rotate_inv(q1, sim::V3{0.0, 0.0, 1.0});  // the inertial "up" seen from the body
  CHECK(near_abs(acc.x, g_body.x, 1e-12) && near_abs(acc.y, g_body.y, 1e-12) && near_abs(acc.z, g_body.z, 1e-12));
  const sim::Q4 d = sim::conj(q0) * q1;                        // the rotation in the body frame over 0.1 ms
  const sim::V3 w = sim::V3{d.x, d.y, d.z} * (2.0 / 1e-4);     // small angle: the rotation vector is twice the vector part
  CHECK(near_abs(gyro.x * sim::kDeg2Rad, w.x, 1e-3) && near_abs(gyro.y * sim::kDeg2Rad, w.y, 1e-3) && near_abs(gyro.z * sim::kDeg2Rad, w.z, 1e-3));
  CHECK(std::fabs(gyro.x) > 5.0 && std::fabs(gyro.y) > 5.0 && std::fabs(gyro.z) > 1.0);  // all three axes are exercised
}

// ---- the IMU error model ----

TFC_TEST(imu_model_applies_the_bias_and_the_scale_error_it_draws_and_keeps_them_constant) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.gyro_bias_dps = 2.0F;
  e.accel_bias_g = 0.05F;
  e.gyro_scale_err = 0.1F;
  e.accel_scale_err = 0.2F;
  const uint32_t seed = 0x1234U;
  sim::ImuModel m(e, seed);
  // the draws, in the order the constructor takes them: for each axis, the gyro bias, the accelerometer bias, the gyro scale, the accelerometer scale, the misalignment
  sim::Lcg draw(seed ^ 0xA5A5A5A5U);
  std::array<float, 3> gb{};
  std::array<float, 3> ab{};
  std::array<float, 3> gs{};
  std::array<float, 3> as{};
  for (unsigned i = 0; i < 3U; ++i) {
    gb[i] = e.gyro_bias_dps * draw.uniform();
    ab[i] = e.accel_bias_g * draw.uniform();
    gs[i] = 1.0F + (e.gyro_scale_err * draw.uniform());
    as[i] = 1.0F + (e.accel_scale_err * draw.uniform());
    (void)draw.uniform();  // the misalignment
  }
  tfc::Vec3 g;
  tfc::Vec3 a;
  tfc::Vec3 gt;
  tfc::Vec3 at;
  gt.v = {10.0F, -20.0F, 30.0F};
  at.v = {0.1F, -0.2F, 1.0F};
  for (int rep = 0; rep < 3; ++rep) {  // the same every frame: they are constants of the sensor
    m.sample(gt, at, g, a);
    for (unsigned i = 0; i < 3U; ++i) {
      CHECK(near_abs(static_cast<double>(g.v[i]), static_cast<double>((gt.v[i] * gs[i]) + gb[i]), 1e-4));
      CHECK(near_abs(static_cast<double>(a.v[i]), static_cast<double>((at.v[i] * as[i]) + ab[i]), 1e-5));
    }
  }
  CHECK(std::fabs(static_cast<double>(as[0] - 1.0F)) > 1e-3);  // the accelerometer scale error is not the identity
}

TFC_TEST(imu_model_stale_samples_happen_with_the_probability_given) {
  const auto stale_fraction = [](float prob) {
    sim::SensorErrors e;
    e.gyro_noise_amp_dps = 0.0F;
    e.accel_noise_amp_g = 0.0F;
    e.stale_prob = prob;
    sim::ImuModel m(e, 77U);
    tfc::Vec3 g;
    tfc::Vec3 a;
    tfc::Vec3 pg;
    tfc::Vec3 in;
    unsigned stale = 0U;
    const unsigned n = 6000U;
    for (unsigned k = 0; k < n; ++k) {
      in.v = {static_cast<float>(k), 0.0F, 0.0F};  // a different input every frame, so that a repeat is a stale sample
      m.sample(in, in, g, a);
      if (k > 0U && g.v[0] == pg.v[0]) {
        ++stale;
      }
      pg = g;
    }
    return static_cast<double>(stale) / (n - 1U);
  };
  CHECK(stale_fraction(0.0F) == 0.0);
  CHECK(near_abs(stale_fraction(0.3F), 0.3, 0.03));
  CHECK(near_abs(stale_fraction(0.8F), 0.8, 0.03));
  CHECK(stale_fraction(1.0F) > 0.99);
}

// ---- the runner's flags ----

TFC_TEST(runner_publishes_the_safed_flag_once_acts_state_is_safe_and_not_before) {
  sim::SimRunner runner;
  (void)runner.start(0U);
  tfc::ActFrame nominal;
  nominal.state = 1U;  // Nominal
  sim::SimFrames out = runner.end_of_frame(9U, &nominal);  // the frames for frame 10, which carries the status frames
  CHECK(out.n == 5U);
  const tfc::DecodedSimFlags before = tfc::unpack_sim_flags(out.f[4]);
  CHECK(before.ok && (before.s.flags & tfc::simflag::kSafed) == 0U);
  tfc::ActFrame safe;
  safe.state = 2U;  // Safe
  (void)runner.end_of_frame(10U, &safe);
  for (uint32_t k = 11U; k < 19U; ++k) {
    (void)runner.end_of_frame(k, &nominal);  // the flag latches: "a Safe event has happened in this run"
  }
  out = runner.end_of_frame(19U, &nominal);
  const tfc::DecodedSimFlags after = tfc::unpack_sim_flags(out.f[4]);
  CHECK(after.ok && (after.s.flags & tfc::simflag::kSafed) != 0U);
}

// ---- the atmosphere ----

TFC_TEST(atmosphere_geopotential_altitude_uses_the_standard_radius_and_the_tail_falls_by_e_every_6_5_km) {
  const double r = 6356766.0;  // the radius of the 1976 standard's geopotential altitude
  const double z = 10000.0;
  const sim::Air a = sim::air_at(z);
  const sim::Air b = sim::air_at_geopotential(r * z / (r + z));
  CHECK(close(a.pressure, b.pressure, 1e-12) && close(a.density, b.density, 1e-12) && close(a.temperature, b.temperature, 1e-12));
  const auto geometric = [r](double h) { return r * h / (r - h); };
  const double h_top = 84852.0;
  const sim::Air top = sim::air_at(geometric(h_top));
  const sim::Air up = sim::air_at(geometric(h_top + 6500.0));
  CHECK(close(up.pressure / top.pressure, std::exp(-1.0), 1e-9) && close(up.density / top.density, std::exp(-1.0), 1e-9));
}
