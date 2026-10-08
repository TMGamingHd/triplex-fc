// SPDX-License-Identifier: MIT
// The 6-DOF vehicle simulator (sim/vehicle): its maths, atmosphere, mass properties and dynamics against conservation laws and known values, the
// platform model, the trajectory and gain design, and finally the flight computers' loop (consensus, estimator, scheduled controller) flying the
// vehicle through max-Q, a gust and an engine-out. See docs/design/VEHICLE_SIM.md section 9.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "tfc/consensus.hpp"
#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
#include "tfc/flight.hpp"
#include "tfc/voter.hpp"
#include "tfc_test.hpp"

#include "atmosphere.hpp"
#include "design.hpp"
#include "flight_tables.hpp"
#include "math3.hpp"
#include "platform.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

}  // namespace

// ---- maths ----

TFC_TEST(quaternion_rotation_inverse_and_composition) {
  using namespace sim;
  const Q4 q = from_axis_angle(V3{0.0, 0.0, 1.0}, 0.5 * kPi);  // 90 degrees about +Z takes X to +Y
  const V3 r = rotate(q, V3{1.0, 0.0, 0.0});
  CHECK(near_abs(r.x, 0.0, 1e-12) && near_abs(r.y, 1.0, 1e-12) && near_abs(r.z, 0.0, 1e-12));
  const V3 back = rotate_inv(q, r);
  CHECK(near_abs(back.x, 1.0, 1e-12) && near_abs(back.y, 0.0, 1e-12));
  const Q4 two = q * q;  // 180 degrees
  const V3 r2 = rotate(two, V3{1.0, 0.0, 0.0});
  CHECK(near_abs(r2.x, -1.0, 1e-12));
  CHECK(norm(normalized(V3{3.0, 4.0, 0.0})) > 0.999999 && norm(normalized(V3{})) == 0.0);
  CHECK(near_abs(norm(cross(V3{1, 0, 0}, V3{0, 1, 0})), 1.0, 1e-12));
  const Q4 z{0.0, 0.0, 0.0, 0.0};
  CHECK(normalized(z).w == 1.0);  // a zero quaternion falls back to the identity
  const Q4 d = q_dot(Q4{}, V3{1.0, 0.0, 0.0});
  CHECK(near_abs(d.x, 0.5, 1e-12));
}

// ---- atmosphere ----

TFC_TEST(atmosphere_matches_the_standard_at_the_layer_bases) {
  using namespace sim;
  const Air sl = air_at_geopotential(0.0);
  CHECK(close(sl.pressure, 101325.0, 1e-6) && close(sl.density, 1.225, 1e-3) && close(sl.sound, 340.294, 1e-4));
  const std::array<std::array<double, 3>, 6> bases{{{11000.0, 22632.06, 216.65}, {20000.0, 5474.889, 216.65}, {32000.0, 868.0187, 228.65},
                                                    {47000.0, 110.9063, 270.65}, {51000.0, 66.93887, 270.65}, {71000.0, 3.956420, 214.65}}};
  for (const auto& b : bases) {
    const Air a = air_at_geopotential(b[0]);
    CHECK(close(a.pressure, b[1], 2e-5));
    CHECK(close(a.temperature, b[2], 1e-9));
  }
  CHECK(close(air_at_geopotential(11000.0).density, 0.36391, 1e-3));
  CHECK(close(air_at_geopotential(20000.0).density, 0.088035, 1e-3));
  CHECK(close(air_at_geopotential(11000.0).sound, 295.07, 1e-4));
}

TFC_TEST(atmosphere_is_continuous_between_layers_decreasing_and_has_an_exponential_top) {
  using namespace sim;
  for (const double base : {11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0}) {
    const Air below = air_at_geopotential(base - 0.01);
    const Air above = air_at_geopotential(base + 0.01);
    CHECK(close(below.pressure, above.pressure, 1e-5));
    CHECK(close(below.temperature, above.temperature, 1e-4));
  }
  double last = 1e9;
  for (int i = 0; i <= 180; ++i) {
    const double rho = sim::air_at(static_cast<double>(i) * 500.0).density;
    CHECK(rho > 0.0 && rho < last);
    last = rho;
  }
  CHECK(air_at(-5.0).density == air_at(0.0).density);              // below the ground: the sea-level values
  CHECK(close(air_at(0.0).density, 1.225, 1e-3));                  // geometric = geopotential at sea level
  CHECK(air_at(300000.0).density < 1e-12);                         // the exponential tail: nothing left
  CHECK(air_at(90000.0).density < air_at(84000.0).density);
}

// ---- mass properties ----

TFC_TEST(mass_properties_move_as_the_propellant_burns) {
  sim::Vehicle6 v;
  const sim::MassProps full = v.mass_props(30000.0);
  const sim::MassProps half = v.mass_props(18000.0);
  const sim::MassProps dry = v.mass_props(6000.0);
  CHECK(full.x_cg > 7.0 && full.x_cg < 9.0);        // about 7.8 m from the aft end at lift-off
  CHECK(dry.x_cg > 9.9 && dry.x_cg < 10.1);         // the dry mass alone
  // the propellant column drops as it burns, so the CG first moves aft, and then forward again as only the dry mass is left
  CHECK(half.x_cg < full.x_cg && dry.x_cg > full.x_cg);
  CHECK(full.i_t > 3.0e5 && full.i_t < 8.0e5);
  CHECK(dry.i_t < full.i_t);
  CHECK(full.i_x > 0.0 && full.i_x < full.i_t);
  CHECK(v.mass_props(1000.0).mass == 1000.0);  // below dry: the propellant clamps at zero
}

// ---- dynamics: conservation laws and known values ----

TFC_TEST(a_circular_orbit_conserves_radius_energy_and_angular_momentum) {
  using namespace sim;
  Params p;
  Vehicle6 v(p);
  State s;
  const double r0 = kEarthR + 300000.0;
  s.r = V3{r0, 0.0, 0.0};
  s.v = V3{0.0, std::sqrt(kEarthMu / r0), 0.0};
  s.m = p.m_dry;  // no propellant: coasting
  v.set_state(s);
  const double e0 = (0.5 * dot(s.v, s.v)) - (kEarthMu / norm(s.r));
  const double h0 = norm(cross(s.r, s.v));
  for (int i = 0; i < 12; ++i) {
    v.step(10.0, 0.0, 0.0);  // 120 s
  }
  const State& e = v.state();
  CHECK(close(norm(e.r), r0, 1e-7));
  CHECK(close((0.5 * dot(e.v, e.v)) - (kEarthMu / norm(e.r)), e0, 1e-9));
  CHECK(close(norm(cross(e.r, e.v)), h0, 1e-9));
  CHECK(!v.burning() && v.engines_on() == 5);
}

TFC_TEST(a_vacuum_burn_obeys_the_rocket_equation) {
  using namespace sim;
  Params p;
  p.gravity_scale = 0.0;
  Vehicle6 v(p);
  State s;
  s.r = V3{kEarthR + 600000.0, 0.0, 0.0};  // far above the air
  v.set_state(s);
  const double m0 = v.mass();
  for (int i = 0; i < 3000; ++i) {
    v.step(0.01, 0.0, 0.0);  // 30 s
  }
  const double dv_expected = p.isp_vac * kG0 * std::log(m0 / v.mass());
  CHECK(close(v.speed(), dv_expected, 2e-4));
  CHECK(close(m0 - v.mass(), 5.0 * p.thrust_vac_each / (p.isp_vac * kG0) * 30.0, 1e-4));
}

TFC_TEST(torque_free_rotation_conserves_angular_momentum_and_energy) {
  using namespace sim;
  Params p;
  p.gravity_scale = 0.0;
  p.ideal_roll_control = false;  // spin about the long axis is the point of this test
  Vehicle6 v(p);
  State s;
  s.r = V3{kEarthR + 600000.0, 0.0, 0.0};
  s.m = p.m_dry;
  s.w = V3{3.0, 0.4, -0.2};
  v.set_state(s);
  const MassProps mp = v.mass_props(s.m);
  auto angular_momentum = [&](const State& st) {
    return rotate(st.q, V3{mp.i_x * st.w.x, mp.i_t * st.w.y, mp.i_t * st.w.z});
  };
  const V3 l0 = angular_momentum(s);
  const double ke0 = 0.5 * ((mp.i_x * s.w.x * s.w.x) + (mp.i_t * s.w.y * s.w.y) + (mp.i_t * s.w.z * s.w.z));
  for (int i = 0; i < 100; ++i) {
    v.step(0.1, 0.0, 0.0);  // 10 s
  }
  const State& e = v.state();
  const V3 l1 = angular_momentum(e);
  CHECK(close(norm(l1), norm(l0), 1e-8));
  CHECK(norm(l1 - l0) < 1e-6 * norm(l0));
  const double ke1 = 0.5 * ((mp.i_x * e.w.x * e.w.x) + (mp.i_t * e.w.y * e.w.y) + (mp.i_t * e.w.z * e.w.z));
  CHECK(close(ke1, ke0, 1e-8));
  CHECK(near_abs(std::sqrt((e.q.w * e.q.w) + (e.q.x * e.q.x) + (e.q.y * e.q.y) + (e.q.z * e.q.z)), 1.0, 1e-12));
}

TFC_TEST(thrust_rises_with_altitude_and_stops_when_the_propellant_is_gone) {
  using namespace sim;
  Vehicle6 v;
  const double t_sl = v.current_loads().thrust;
  State s = v.state();
  s.r = V3{kEarthR + 30000.0, 0.0, 0.0};
  v.set_state(s);
  const double t_30 = v.current_loads().thrust;
  CHECK(t_sl > 3.9e5 && t_sl < 4.1e5);            // 5 x (92 kN - 101 kPa x 0.12 m^2)
  CHECK(t_30 > t_sl && t_30 < 5.0 * 92000.0);
  s.m = v.params().m_dry;
  v.set_state(s);
  CHECK(v.current_loads().thrust == 0.0 && v.current_loads().mdot == 0.0);
  v.step(1.0, 0.0, 0.0);
  CHECK(v.mass() == v.params().m_dry);
}

TFC_TEST(the_aerodynamic_moment_destabilises_when_the_centre_of_pressure_is_ahead_of_the_cg) {
  using namespace sim;
  Params p;  // x_cp 12.5 m is ahead of the CG
  Vehicle6 v(p);
  State s;
  s.r = V3{kEarthR + 10000.0, 0.0, 0.0};
  s.v = V3{250.0, 5.0, 0.0};  // flying up with a small sideways velocity relative to the air: the nose points at x, the velocity is tilted toward +Y
  v.set_state(s);
  const Loads l = v.current_loads();
  CHECK(l.alpha > 0.0 && l.mach > 0.5 && l.dynamic_pressure > 1000.0);
  CHECK(l.f_aero.x < 0.0);                    // drag
  CHECK(l.f_aero.y < 0.0);                    // the normal force opposes the sideways relative velocity
  CHECK(l.m_aero.z < 0.0);                    // ...and, acting ahead of the CG, turns the nose away from the velocity: unstable
  Params q = p;
  q.x_cp = 5.0;  // behind the CG: stable
  Vehicle6 w(q);
  w.set_state(s);
  CHECK(w.current_loads().m_aero.z > 0.0);
}

TFC_TEST(the_gimbal_signs_are_the_ones_the_controller_assumes) {
  using namespace sim;
  Vehicle6 a;
  for (int i = 0; i < 100; ++i) {
    a.step(0.01, 2.0, 0.0);  // a positive pitch command
  }
  CHECK(a.tilts().y_deg > 7.0 && near_abs(a.tilts().x_deg, 0.0, 1e-3));
  Vehicle6 b;
  for (int i = 0; i < 100; ++i) {
    b.step(0.01, 0.0, 2.0);  // a positive yaw command
  }
  CHECK(b.tilts().x_deg > 7.0 && near_abs(b.tilts().y_deg, 0.0, 1e-3));
  CHECK(a.gimbal_pitch_deg() == 2.0 && b.gimbal_yaw_deg() == 2.0);
}

TFC_TEST(the_gimbal_actuator_is_limited_in_angle_and_in_rate) {
  sim::Vehicle6 v;
  v.step(0.1, 50.0, -50.0);  // asks for far more than it may have, in 0.1 s
  CHECK(near_abs(v.gimbal_pitch_deg(), 6.0, 1e-9));   // 60 degrees per second for 0.1 s
  CHECK(near_abs(v.gimbal_yaw_deg(), -6.0, 1e-9));
  v.step(1.0, 50.0, -50.0);
  CHECK(near_abs(v.gimbal_pitch_deg(), 8.0, 1e-9) && near_abs(v.gimbal_yaw_deg(), -8.0, 1e-9));
}

TFC_TEST(an_engine_out_costs_a_fifth_of_the_thrust_and_leaves_a_moment) {
  using namespace sim;
  Scenario sc;
  sc.engine_out_time = 1.0;
  sc.engine_out_index = 1;  // an outer engine, on +Y
  Vehicle6 v(Params{}, sc);
  const Loads before = v.current_loads();
  CHECK(near_abs(before.m_thrust.z, 0.0, 1e-6) && near_abs(before.m_thrust.y, 0.0, 1e-6));  // symmetric: no moment
  v.step(1.5, 0.0, 0.0);
  const Loads after = v.current_loads();
  CHECK(v.engines_on() == 4);
  CHECK(after.thrust < 0.82 * before.thrust && after.thrust > 0.78 * before.thrust);
  CHECK(std::fabs(after.m_thrust.z) > 1000.0);  // the missing engine's moment about the CG
  Scenario bad;
  bad.engine_out_time = 0.0;
  bad.engine_out_index = 99;  // no such engine: ignored
  Vehicle6 w(Params{}, bad);
  w.step(0.1, 0.0, 0.0);
  CHECK(w.engines_on() == 5);
}

TFC_TEST(an_ideal_roll_controller_keeps_the_roll_rate_zero_even_with_an_engine_out_and_a_gimbal_deflection) {
  using namespace sim;
  Scenario sc;
  sc.engine_out_time = 0.0;
  sc.engine_out_index = 1;
  Params ideal;
  ideal.ground_contact = false;  // the bare equations: with the ground, this vehicle (an engine out at lift-off, a deflected gimbal) falls back onto the pad after 3 s and is held there
  Vehicle6 held(ideal, sc);
  for (int i = 0; i < 300; ++i) {
    held.step(0.01, 3.0, 3.0);
  }
  CHECK(held.state().w.x == 0.0);
  Params free_roll = ideal;
  free_roll.ideal_roll_control = false;
  Vehicle6 drifting(free_roll, sc);
  for (int i = 0; i < 300; ++i) {
    drifting.step(0.01, 3.0, 3.0);
  }
  CHECK(std::fabs(drifting.state().w.x) > 1e-3);  // without it the roll torque turns the vehicle about its axis
}

TFC_TEST(wind_is_a_mean_profile_plus_one_minus_cosine_gusts) {
  using namespace sim;
  Scenario sc;
  Gust g;
  g.t0 = 10.0;
  g.duration = 4.0;
  g.peak = V3{0.0, 0.0, 12.0};
  sc.gusts.push_back(g);
  sc.wind_scale = 2.0;
  Vehicle6 v(Params{}, sc);
  CHECK(near_abs(norm(v.wind_at(0.0, 0.0)), 0.0, 1e-12));
  CHECK(near_abs(v.wind_at(12000.0, 0.0).z, 56.0, 1e-9));        // 28 m/s jet-stream peak, doubled
  CHECK(near_abs(v.wind_at(12000.0, 0.0).x, 0.0, 1e-12));
  CHECK(near_abs(v.wind_at(0.0, 12.0).z, 12.0, 1e-9));           // mid-gust: the full peak
  CHECK(near_abs(v.wind_at(0.0, 10.0).z, 0.0, 1e-9));            // the gust starts from zero
  CHECK(near_abs(v.wind_at(0.0, 15.0).z, 0.0, 1e-9));            // and is over
  CHECK(v.wind_at(20000.0, 0.0).z < v.wind_at(12000.0, 0.0).z);  // the profile falls above the jet stream
  CHECK(near_abs(v.wind_at(100000.0, 0.0).z, 0.0, 1e-12));       // nothing above 60 km
}

TFC_TEST(a_dry_cg_shift_moves_the_centre_of_gravity) {
  sim::Scenario sc;
  sc.dry_cg_shift = 1.0;
  sim::Vehicle6 shifted(sim::Params{}, sc);
  sim::Vehicle6 nominal;
  CHECK(shifted.mass_props(30000.0).x_cg > nominal.mass_props(30000.0).x_cg + 0.15);
}

// ---- the platform and the mapping from the vehicle ----

TFC_TEST(the_tilts_of_a_known_attitude_are_recovered_and_the_platform_shows_them) {
  using namespace sim;
  for (const std::array<double, 2> a : {std::array<double, 2>{20.0, 0.0}, std::array<double, 2>{0.0, 10.0}, std::array<double, 2>{-15.0, 12.0},
                                         std::array<double, 2>{30.0, -25.0}}) {
    const double theta = a[0] * kDeg2Rad;  // pitch plane, toward downrange
    const double psi = a[1] * kDeg2Rad;    // yaw plane
    State s;
    s.q = from_axis_angle(V3{0, 0, 1}, theta) * from_axis_angle(V3{0, 1, 0}, psi);
    Vehicle6 v;
    v.set_state(s);
    const Tilts t = v.tilts();
    CHECK(near_abs(t.y_deg, a[0], 1e-9) && near_abs(t.x_deg, a[1], 1e-9));
    // a platform at those tilts feels gravity along the vehicle's long axis in the pad frame
    Platform plat;
    for (int i = 0; i < 400; ++i) {
      plat.step(t, 0.01);
    }
    V3 g;
    V3 acc;
    plat.imu_truth(g, acc);
    CHECK(near_abs(norm(acc), 1.0, 1e-9));
    CHECK(near_abs(g.x, 0.0, 1e-3) && near_abs(g.y, 0.0, 1e-3) && near_abs(g.z, 0.0, 1e-3));  // settled: no rotation
    const double ty = plat.tilts().y_deg * kDeg2Rad;
    const double tx = plat.tilts().x_deg * kDeg2Rad;
    CHECK(near_abs(acc.x, -std::sin(ty), 1e-12) && near_abs(acc.y, std::cos(ty) * std::sin(tx), 1e-12) &&
          near_abs(acc.z, std::cos(ty) * std::cos(tx), 1e-12));
  }
}

TFC_TEST(the_platform_lags_the_vehicle_limits_its_rate_and_its_travel_and_says_when_it_saturates) {
  using namespace sim;
  Platform plat;
  plat.step(Tilts{0.0, 10.0}, 0.01);
  CHECK(plat.tilts().y_deg > 0.0 && plat.tilts().y_deg < 10.0);  // it has not got there
  CHECK(!plat.saturated());
  Platform fast;
  fast.step(Tilts{0.0, 40.0}, 0.01);
  CHECK(near_abs(fast.tilts().y_deg, 3.0, 1e-9));  // 300 degrees per second for 10 ms
  Platform far;
  for (int i = 0; i < 300; ++i) {
    far.step(Tilts{-60.0, 50.0}, 0.01);
  }
  CHECK(far.saturated());
  CHECK(near_abs(far.tilts().x_deg, -45.0, 1e-9) && near_abs(far.tilts().y_deg, 45.0, 1e-9));
  for (int i = 0; i < 300; ++i) {
    far.step(Tilts{0.0, 0.0}, 0.01);
  }
  CHECK(!far.saturated() && near_abs(far.tilts().y_deg, 0.0, 1e-6));
}

TFC_TEST(vehicle_true_sensors_read_specific_force_and_body_rates_in_the_sensor_frame) {
  using namespace sim;
  Vehicle6 v;  // on the pad, burning: specific force is thrust over mass along the long axis
  V3 gyro;
  V3 acc;
  v.vehicle_true_imu(gyro, acc);
  const Loads l = v.current_loads();
  CHECK(near_abs(acc.z, l.thrust / v.mass() / kG0, 1e-9));   // sensor Z is the long axis
  CHECK(acc.z > 1.3 && acc.z < 1.4);                         // 1.36 g at lift-off
  CHECK(near_abs(acc.x, 0.0, 1e-9) && near_abs(acc.y, 0.0, 1e-9));
  State s = v.state();
  s.w = V3{0.1, 0.2, 0.3};  // body X, Y, Z
  v.set_state(s);
  v.vehicle_true_imu(gyro, acc);
  CHECK(near_abs(gyro.z, 0.1 * kRad2Deg, 1e-9) && near_abs(gyro.x, 0.2 * kRad2Deg, 1e-9) && near_abs(gyro.y, 0.3 * kRad2Deg, 1e-9));
}

// A snapshot of a controlled flight of a dispersed vehicle (gimbal lag, thrust misalignment and scale, a normal-force scale, a gust, an engine-out and a
// shifted centre of gravity, held upright by a plain PD loop): the state at 20, 40 and 60 s as the model computed it on 6 Oct 2026, before any change to the
// model was made. It is the guard of every later change (a general vehicle description, a ground, environment models): none of them may move the reference
// vehicle. The tolerances are far below any physical change and above the rounding differences between compilers and machines.
TFC_TEST(a_controlled_flight_of_the_reference_vehicle_is_unchanged_by_changes_to_the_model) {
  using namespace sim;
  Params p;
  p.gimbal_lag_s = 0.05;
  p.thrust_misalign_pitch_deg = 0.3;
  p.thrust_scale = 0.97;
  p.cn_scale = 1.1;
  Scenario sc;
  sc.engine_out_time = 30.0;
  sc.engine_out_index = 2;
  sc.dry_cg_shift = -0.2;
  sc.gusts.push_back(Gust{15.0, 3.0, V3{0.0, 0.0, 12.0}});
  // r.x r.y r.z  v.x v.y v.z  q.w q.x q.y q.z  w.y w.z  m
  static const double kGolden[3][13] = {
      {0x1.855557939fd53p+22, 0x1.2370bdd3c27c2p+3, 0x1.4e9710a555ac1p+1, 0x1.2c1fd65efa485p+6, 0x1.ca7e51772003ap-1, 0x1.6bc989c01b1adp-1, 0x1.ffffc7eb18a89p-1, 0x1.1596bee5924a6p-20, -0x1.1450a187bc938p-11, 0x1.caefbdecc7ecp-10, -0x1.818af77bbeed9p-13, -0x1.8d60cabfafa0ap-16, 0x1.a6e221dcf86p+14},  // t = 20 s
      {0x1.857a6217920bdp+22, 0x1.a850973c6e5d2p+3, 0x1.811112e5a7f25p+5, 0x1.30497a5ff728ap+7, -0x1.3114e279e4f92p+1, 0x1.25ea628ee7d7cp+2, 0x1.fffe6aae255a5p-1, 0x1.1720cd0a86a05p-17, -0x1.5f45530514145p-9, -0x1.0e063a3fb51e9p-8, -0x1.2382278f30714p-14, 0x1.072b7814032cp-12, 0x1.7d9a376bf10c5p+14},  // t = 40 s
      {0x1.85b54c03bea36p+22, -0x1.44f2687487b31p+6, 0x1.942603e8b21fp+7, 0x1.cc294142594d7p+7, -0x1.9f88337a7f62cp+2, 0x1.5fbaf50b9a25dp+3, 0x1.ffff102eb3209p-1, 0x1.8af5a0c810b6bp-19, -0x1.7c72276f0c4ap-9, -0x1.3d8ad0f605a3dp-9, -0x1.8ad2528dacb75p-16, 0x1.40fb8830bc27dp-14, 0x1.58e8b8e9840f5p+14},  // t = 60 s
  };
  Vehicle6 v(p, sc);
  double max_tilt = 0.0;
  unsigned snapshot = 0U;
  for (int k = 1; k <= 6000; ++k) {
    const Tilts tl = v.tilts();
    const V3& w = v.state().w;
    v.step(0.01, (-1.4 * tl.y_deg) - (0.6 * w.z * kRad2Deg), (-1.4 * tl.x_deg) - (0.6 * w.y * kRad2Deg));
    max_tilt = std::fmax(max_tilt, std::fmax(std::fabs(v.tilts().x_deg), std::fabs(v.tilts().y_deg)));
    if (k % 2000 == 0) {
      const State& s = v.state();
      const double* g = kGolden[snapshot++];
      CHECK(near_abs(s.r.x, g[0], 1e-3) && near_abs(s.r.y, g[1], 1e-3) && near_abs(s.r.z, g[2], 1e-3));
      CHECK(near_abs(s.v.x, g[3], 1e-6) && near_abs(s.v.y, g[4], 1e-6) && near_abs(s.v.z, g[5], 1e-6));
      CHECK(near_abs(s.q.w, g[6], 1e-9) && near_abs(s.q.x, g[7], 1e-9) && near_abs(s.q.y, g[8], 1e-9) && near_abs(s.q.z, g[9], 1e-9));
      CHECK(near_abs(s.w.y, g[10], 1e-9) && near_abs(s.w.z, g[11], 1e-9));
      CHECK(near_abs(s.m, g[12], 1e-6));
    }
  }
  CHECK(snapshot == 3U);
  CHECK(max_tilt < 1.0);  // the loop holds the vehicle upright: the snapshot is of a flight, not of a tumble
}

// ---- the design: the nominal ascent and the gain schedule ----

// The figures that docs/design/VEHICLE_SIM.md sections 3, 7 and 11 quote for the reference vehicle. They are pinned here so that the page and the code
// cannot drift apart again (until 6 Oct 2026 the page said 500 kN, a thrust-to-weight of 1.7 and 280 s; the code gives 399.2 kN, 1.357 and 269 s).
TFC_TEST(reference_vehicle_figures_are_the_documented_ones) {
  using namespace sim;
  const Params p;
  const Vehicle6 v(p);
  const Loads l = v.current_loads();
  const double weight = v.mass() * kG0;
  CHECK(close(l.thrust, 5.0 * (92000.0 - (101325.0 * 0.12)), 1e-9));  // 399.2 kN at sea level
  CHECK(near_abs(l.thrust, 399205.0, 1.0));
  CHECK(close(5.0 * p.thrust_vac_each, 460000.0, 1e-12));
  CHECK(close(l.thrust / weight, 1.3569, 1e-3));                       // liftoff thrust-to-weight, sea level
  CHECK(close(5.0 * p.thrust_vac_each / weight, 1.5636, 1e-3));        // and in vacuum
  CHECK(close(l.mdot, 151.31, 1e-4));                                  // 30.26 kg/s per engine
  CHECK(close(l.thrust / (l.mdot * kG0), 269.03, 1e-4));               // the sea-level Isp is derived, not an input
  CHECK(close(p.m_prop0 / l.mdot, 158.61, 1e-4));                      // the burn time
  CHECK(close(v.mass_props(v.mass()).x_cg, 7.792, 1e-3));              // the gimbal arm L_g at liftoff
  CHECK(close(v.mass_props(p.m_dry).x_cg, 10.0, 1e-12));               // and at burnout
  const std::vector<NominalPoint> n = nominal_trajectory(p);
  CHECK(close(n.front().b_ctl, 8.163, 1e-3));                          // control effectiveness at liftoff
  CHECK(close(n.back().b_ctl, 11.089, 1e-3));                          // and at 100 s
  double a_max = 0.0;
  double t_a = 0.0;
  for (const NominalPoint& np : n) {
    if (np.a_div > a_max) {
      a_max = np.a_div;
      t_a = np.t;
    }
  }
  CHECK(close(a_max, 4.418, 1e-3) && near_abs(t_a, 65.0, 0.5));        // the divergence peaks at max-Q
  // the gain schedule: 17 points are designed and the schedule holds 16, so the one at 96 s is dropped and the 90 s gains are held to the end
  const std::vector<GainPoint> designed = gain_schedule(n, 6.0, 2.5, 0.8, 0.2);
  CHECK(designed.size() == 17U && near_abs(designed.back().t, 96.0, 1e-6));
  const FlightTables tables = flight_tables(p);
  CHECK(tables.gains.size() == tfc::GainSchedule::kMaxPoints);
  CHECK(tables.gains.frame_at(tfc::GainSchedule::kMaxPoints - 1U) == 9000U);
  CHECK(close(tables.gains.gains_at(tfc::GainSchedule::kMaxPoints - 1U).kp, designed[15].kp, 1e-5));
  CHECK(close(designed[15].kp, 0.7534, 1e-3) && close(designed[16].kp, 0.6893, 1e-3));
}

TFC_TEST(the_nominal_ascent_is_a_believable_gravity_turn) {
  using namespace sim;
  const std::vector<NominalPoint> n = nominal_trajectory(Params{});
  CHECK(n.size() == 10001U);
  double qmax = 0.0;
  double tq = 0.0;
  for (const NominalPoint& p : n) {
    if (p.dynamic_pressure > qmax) {
      qmax = p.dynamic_pressure;
      tq = p.t;
    }
  }
  CHECK(qmax > 25000.0 && qmax < 40000.0);   // max-Q of about 31 kPa
  CHECK(tq > 55.0 && tq < 75.0);
  CHECK(n[6500].mach > 1.0 && n[6500].mach < 2.0);  // at about Mach 1.4
  CHECK(n[10000].theta_deg > 25.0 && n[10000].theta_deg < 40.0);  // pitched over, within the platform's range
  CHECK(n[10000].altitude > 25000.0 && n[10000].speed > 800.0);
  for (std::size_t i = 2000; i < n.size(); ++i) {
    CHECK(n[i].alpha_deg < 4.0);  // flying along the velocity vector
    CHECK(n[i].theta_deg < 45.0);
  }
  CHECK(n[0].theta_deg == 0.0 && n[500].theta_deg == 0.0);  // vertical for the first eight seconds
  for (std::size_t i = 1; i < n.size(); ++i) {
    CHECK(n[i].mass < n[i - 1].mass + 1e-9);
  }
  const std::vector<GainPoint> g = gain_schedule(n, 10.0, 2.5, 0.8, 0.2);
  CHECK(g.size() == 11U);
  for (const GainPoint& p : g) {
    CHECK(p.kp > 0.3 && p.kp < 2.0 && p.kd > 0.2 && p.kd < 1.0 && p.ki > 0.0);
  }
  CHECK(gain_schedule(std::vector<NominalPoint>{}, 10.0, 2.5, 0.8, 0.2).empty());
}

// ---- the schedule and guidance in the flight code ----

TFC_TEST(the_gain_schedule_interpolates_and_holds_at_the_ends) {
  tfc::GainSchedule gs;
  CHECK(gs.at(10U).kp == tfc::ControllerGains{}.kp);  // empty: the defaults
  CHECK(gs.add(100U, tfc::ControllerGains{1.0F, 0.5F, 0.1F}));
  CHECK(gs.add(300U, tfc::ControllerGains{2.0F, 1.5F, 0.3F}));
  CHECK(!gs.add(300U, tfc::ControllerGains{}));  // frames must increase
  CHECK(gs.at(0U).kp == 1.0F && gs.at(100U).kd == 0.5F);  // before and at the first point
  const tfc::ControllerGains mid = gs.at(200U);
  CHECK(near_abs(static_cast<double>(mid.kp), 1.5, 1e-6) && near_abs(static_cast<double>(mid.kd), 1.0, 1e-6) &&
        near_abs(static_cast<double>(mid.ki), 0.2, 1e-6));
  CHECK(gs.at(300U).kp == 2.0F && gs.at(9999U).kd == 1.5F);  // at and after the last
  CHECK(gs.size() == 2U);
  for (unsigned i = 2; i < tfc::GainSchedule::kMaxPoints; ++i) {
    CHECK(gs.add(300U + i, tfc::ControllerGains{}));
  }
  CHECK(!gs.add(1000U, tfc::ControllerGains{}));  // full
  tfc::Controller c;
  c.set_gains(tfc::ControllerGains{0.5F, 0.4F, 0.0F});
  tfc::Attitude att;
  att.valid = true;
  att.tilt_y_deg = -1.0F;
  const tfc::Command cmd = c.step(att, tfc::Reference{}, 0.01F);
  CHECK(near_abs(static_cast<double>(cmd.pitch_deg), 0.5, 1e-6));  // kp 0.5 times an error of 1 degree
}

// ---- the whole loop, flying the 6-DOF vehicle ----

namespace {

struct Flight {
  sim::Params params;
  sim::Scenario scenario;
  bool sensor_fault = false;   // one IMU reads 15 dps too much on two axes
  bool vehicle_true = false;   // vehicle-true sensors instead of the platform's
  double t_end = 100.0;
};

struct Outcome {
  double max_err_deg = 0.0;       // vehicle tilt against the program, either plane
  double rms_err_deg = 0.0;
  double max_pitch_err = 0.0;
  double max_yaw_err = 0.0;
  double final_theta_deg = 0.0;
  double final_altitude = 0.0;
  unsigned saturated_frames = 0U;  // the controller's command pinned against its limit
  unsigned platform_saturated = 0U;
  unsigned votes_lost = 0U;
  bool replicas_identical = true;
  bool finite = true;
};

class Lcg {
 public:
  explicit Lcg(uint32_t s) : s_(s) {}
  float uniform() {
    s_ = s_ * 1664525U + 1013904223U;
    return static_cast<float>(s_ >> 8) / 8388608.0F - 1.0F;
  }

 private:
  uint32_t s_;
};

uint32_t bits(float f) {
  uint32_t u = 0U;
  std::memcpy(&u, &f, sizeof u);
  return u;
}

Outcome fly(const Flight& f) {
  using namespace sim;
  const FlightTables tables = flight_tables(f.params);  // the same tables the firmware carries (tools/vehicle/gen_tables.cpp)
  const tfc::Guidance& guide = tables.guidance;
  const tfc::GainSchedule& schedule = tables.gains;
  Vehicle6 veh(f.params, f.scenario);
  Platform plat;
  std::array<Lcg, 3> noise{Lcg(11U), Lcg(22U), Lcg(33U)};
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(schedule, guide), tfc::FlightFunction(schedule, guide), tfc::FlightFunction(schedule, guide)};
  float act_p = 0.0F;
  float act_y = 0.0F;
  Outcome o;
  double sum_sq = 0.0;
  const uint32_t frames = static_cast<uint32_t>(f.t_end / 0.01);
  for (uint32_t k = 0; k < frames; ++k) {
    // the sensor inputs: the platform's (the rig) or the vehicle's own
    V3 g;
    V3 a;
    if (f.vehicle_true) {
      veh.vehicle_true_imu(g, a);
    } else {
      plat.imu_truth(g, a);
    }
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<tfc::Frame, 6> fr{};
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 gy;
      tfc::Vec3 ac;
      const std::array<double, 3> gg{g.x, g.y, g.z};
      const std::array<double, 3> aa{a.x, a.y, a.z};
      for (unsigned i = 0; i < 3U; ++i) {
        gy.v[i] = static_cast<float>(gg[i]) + (0.1F * noise[n].uniform());
        ac.v[i] = static_cast<float>(aa[i]) + (0.002F * noise[n].uniform());
      }
      if (f.sensor_fault && n == 1U) {
        gy.v[0] += 15.0F;
        gy.v[1] += 15.0F;
      }
      fr[n] = tfc::pack_gyro(n, gy, seq);
      fr[3U + n] = tfc::pack_accel(n, ac, seq);
    }
    const tfc::Reference ref = guide.at(k);
    std::array<tfc::Command, 3> cmd{};
    for (unsigned r = 0; r < 3U; ++r) {
      ff[r].begin_frame(k, 0x07U);
      for (const tfc::Frame& fm : fr) {
        (void)ff[r].on_frame(fm);
      }
      cmd[r] = ff[r].step();
    }
    for (unsigned r = 1; r < 3U; ++r) {
      o.replicas_identical = o.replicas_identical && bits(cmd[r].pitch_deg) == bits(cmd[0].pitch_deg) && bits(cmd[r].yaw_deg) == bits(cmd[0].yaw_deg) &&
                             cmd[r].state_digest == cmd[0].state_digest;
    }
    std::array<float, 3> pc{};
    std::array<float, 3> yc{};
    for (unsigned r = 0; r < 3U; ++r) {
      const tfc::DecodedCommand d = tfc::unpack_cmd(tfc::pack_cmd(static_cast<uint8_t>(r), cmd[r], seq));
      pc[r] = d.cmd.pitch_deg;
      yc[r] = d.cmd.yaw_deg;
    }
    const tfc::VoteResult vp = tfc::vote3(pc, 0x07U, 0.05F);
    const tfc::VoteResult vy = tfc::vote3(yc, 0x07U, 0.05F);
    if (vp.status == tfc::VoteStatus::Triplex && vy.status == tfc::VoteStatus::Triplex) {
      act_p = vp.value;
      act_y = vy.value;
    } else {
      ++o.votes_lost;
    }
    veh.step(0.01, static_cast<double>(act_p), static_cast<double>(act_y));
    const Tilts t = veh.tilts();
    plat.step(t, 0.01);
    if (plat.saturated()) {
      ++o.platform_saturated;
    }
    const double ep = t.y_deg - static_cast<double>(ref.tilt_y_deg);
    const double ey = t.x_deg - static_cast<double>(ref.tilt_x_deg);
    if (std::getenv("TFC_LOOP_TRACE") != nullptr && k % 250U == 0U) {
      std::printf("      t=%5.1f tilt y %7.3f (ref %7.3f) x %7.3f  cmd p %6.2f y %6.2f  gimbal p %6.2f  est y %7.3f\n", k * 0.01, t.y_deg, static_cast<double>(ref.tilt_y_deg),
                  t.x_deg, static_cast<double>(act_p), static_cast<double>(act_y), veh.gimbal_pitch_deg(), static_cast<double>(ff[0].estimator().attitude().tilt_y_deg));
    }
    o.max_pitch_err = std::fmax(o.max_pitch_err, std::fabs(ep));
    o.max_yaw_err = std::fmax(o.max_yaw_err, std::fabs(ey));
    o.max_err_deg = std::fmax(o.max_err_deg, std::fmax(std::fabs(ep), std::fabs(ey)));
    sum_sq += (ep * ep) + (ey * ey);
    o.finite = o.finite && std::isfinite(t.x_deg) && std::isfinite(t.y_deg) && std::isfinite(static_cast<double>(act_p));
    if (std::fabs(static_cast<double>(act_p)) > 7.9 || std::fabs(static_cast<double>(act_y)) > 7.9) {
      ++o.saturated_frames;
    }
  }
  o.rms_err_deg = std::sqrt(sum_sq / (2.0 * frames));
  o.final_theta_deg = veh.tilts().y_deg;
  o.final_altitude = veh.altitude();
  return o;
}

void report(const char* name, const Outcome& o) {
  if (std::getenv("TFC_LOOP_VERBOSE") != nullptr) {
    std::printf("    [%s] rms %.3f deg  max pitch %.3f yaw %.3f  end theta %.2f deg alt %.0f m  cmd-saturated frames %u  platform-saturated %u  votes lost %u\n",
                name, o.rms_err_deg, o.max_pitch_err, o.max_yaw_err, o.final_theta_deg, o.final_altitude, o.saturated_frames, o.platform_saturated, o.votes_lost);
  }
}

}  // namespace

TFC_TEST(flight_nominal_ascent_is_held_on_the_pitch_program_through_max_q) {
  Flight f;
  const Outcome o = fly(f);
  report("nominal", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.votes_lost == 0U);
  CHECK(o.rms_err_deg < 0.5);
  CHECK(o.max_err_deg < 2.0);
  CHECK(o.platform_saturated == 0U);
  CHECK(o.saturated_frames < 100U);
  CHECK(o.final_theta_deg > 25.0 && o.final_theta_deg < 40.0);  // it flew the program
}

TFC_TEST(flight_survives_a_gust_at_max_q) {
  Flight f;
  sim::Gust g;
  g.t0 = 60.0;
  g.duration = 4.0;
  g.peak = sim::V3{0.0, 0.0, 15.0};  // 15 m/s crosswind
  f.scenario.gusts.push_back(g);
  const Outcome o = fly(f);
  report("gust at max-Q", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.max_err_deg < 4.0);
  CHECK(o.rms_err_deg < 1.0);
  CHECK(o.platform_saturated == 0U);
}

TFC_TEST(flight_survives_a_strong_wind_profile) {
  Flight f;
  f.scenario.wind_scale = 2.0;  // the jet stream at 56 m/s
  f.scenario.wind_dir = sim::V3{0.0, 0.3, 1.0};
  const Outcome o = fly(f);
  report("2x wind", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.max_err_deg < 4.0);
  CHECK(o.rms_err_deg < 1.0);
}

TFC_TEST(flight_survives_an_engine_out_before_and_at_max_q) {
  for (const double t_out : {30.0, 62.0}) {
    Flight f;
    f.scenario.engine_out_time = t_out;
    f.scenario.engine_out_index = 1;
    const Outcome o = fly(f);
    report(t_out < 40.0 ? "engine-out at 30 s" : "engine-out at 62 s", o);
    CHECK(o.replicas_identical && o.finite);
    CHECK(o.max_err_deg < 5.0);
    CHECK(o.rms_err_deg < 1.5);
    CHECK(o.platform_saturated == 0U);
  }
}

TFC_TEST(flight_with_the_centre_of_gravity_off_nominal_still_holds) {
  Flight f;
  f.scenario.dry_cg_shift = -0.8;  // dry mass 0.8 m further aft: less stability margin, different gimbal arm
  const Outcome o = fly(f);
  report("cg offset", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.max_err_deg < 3.0);
}

TFC_TEST(flight_masks_a_failed_imu_with_the_consensus) {
  Flight clean;
  const Outcome c = fly(clean);
  Flight f;
  f.sensor_fault = true;
  const Outcome o = fly(f);
  report("one IMU +15 dps", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.rms_err_deg < c.rms_err_deg + 0.05);
  CHECK(o.max_err_deg < 2.0);
}

TFC_TEST(flight_on_vehicle_true_sensors_coasts_on_the_gyro_because_gravity_is_not_observable_under_thrust) {
  Flight f;
  f.vehicle_true = true;
  const Outcome o = fly(f);
  report("vehicle-true sensors", o);
  CHECK(o.replicas_identical && o.finite);
  CHECK(o.max_err_deg < 3.0);
}

// ---- independent verification: a separately written planar model, step-size convergence, and plane symmetry ----

namespace {

// The pitch-plane flight written again from scratch in two dimensions (x vertical, y downrange, theta of the long axis from +x toward +y), with
// no quaternions, no cross products and no gimbal vector: it shares only the atmosphere and the mass properties with the 3-D model.
struct Planar {
  sim::Params p;
  double engine_out_t = -1.0;
  double t = 0.0;
  double x = sim::kEarthR;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double th = 0.0;
  double om = 0.0;
  double m = 0.0;
  double dp = 0.0;  // gimbal angle, rad
  bool out1 = false;

  explicit Planar(const sim::Params& pp) : p(pp), m(pp.m_dry + pp.m_prop0) {}

  struct S {
    double x, y, vx, vy, th, om, m;
  };
  [[nodiscard]] S s() const { return S{x, y, vx, vy, th, om, m}; }

  static double cd(double mach) {
    const double d = (mach - 1.1) / 0.35;
    return 0.30 + (0.45 * std::exp(-d * d));
  }

  [[nodiscard]] S rate(const S& a, double gimbal) const {
    const double r = std::hypot(a.x, a.y);
    const sim::Air air = sim::air_at(r - sim::kEarthR);
    const sim::MassProps mp = sim::Vehicle6(p).mass_props(a.m);
    const double ex_x = std::cos(a.th);
    const double ex_y = std::sin(a.th);
    const double vbx = (a.vx * ex_x) + (a.vy * ex_y);        // velocity along the long axis
    const double vby = (-a.vx * ex_y) + (a.vy * ex_x);       // and across it (body Y)
    const double vabs = std::hypot(vbx, vby);
    const double q = 0.5 * air.density * vabs * vabs;
    const double area = sim::kPi * 0.25 * p.diameter * p.diameter;
    double fbx = 0.0;
    double fby = 0.0;
    double mz = 0.0;
    if (vabs > 1.0 && vbx > 0.0) {
      const double alpha = std::atan2(std::fabs(vby), vbx);
      fbx += -q * area * cd(vabs / air.sound);
      const double fn = std::fabs(vby) > 1e-9 ? -q * area * p.c_n_alpha * alpha * (vby / std::fabs(vby)) : 0.0;
      fby += fn;
      mz += (p.x_cp - mp.x_cg) * fn;
    }
    double mdot = 0.0;
    if (a.m > p.m_dry + 1e-6) {
      for (int i = 0; i < p.engines; ++i) {
        if (i == 1 && out1) {
          continue;
        }
        const double ti = std::fmax(0.0, p.thrust_vac_each - (air.pressure * p.exit_area_each));
        const double fx = ti * std::cos(gimbal);
        const double fy = -ti * std::sin(gimbal);
        const double ey = i == 1 ? p.engine_offset : (i == 2 ? -p.engine_offset : 0.0);
        fbx += fx;
        fby += fy;
        mz += (-mp.x_cg * fy) - (ey * fx);
        mdot += p.thrust_vac_each / (p.isp_vac * sim::kG0);
      }
    }
    const double g = sim::kEarthMu / (r * r * r);
    const double fx_in = (fbx * ex_x) - (fby * ex_y);
    const double fy_in = (fbx * ex_y) + (fby * ex_x);
    const sim::MassProps mp2 = sim::Vehicle6(p).mass_props(a.m + 1.0);
    const double didt = (mp2.i_t - mp.i_t) * -mdot;
    return S{a.vx, a.vy, (fx_in / a.m) - (g * a.x), (fy_in / a.m) - (g * a.y), a.om, (mz - (didt * a.om)) / mp.i_t, -mdot};
  }

  void step(double h, double cmd_deg) {
    const double target = std::fmax(-p.gimbal_limit_deg, std::fmin(p.gimbal_limit_deg, cmd_deg)) * sim::kDeg2Rad;
    const double lim = p.gimbal_rate_dps * sim::kDeg2Rad * h;
    dp += std::fmax(-lim, std::fmin(lim, target - dp));
    const S a = s();
    auto add = [](const S& u, const S& d, double f) {
      return S{u.x + (d.x * f), u.y + (d.y * f), u.vx + (d.vx * f), u.vy + (d.vy * f), u.th + (d.th * f), u.om + (d.om * f), u.m + (d.m * f)};
    };
    const S k1 = rate(a, dp);
    const S k2 = rate(add(a, k1, 0.5 * h), dp);
    const S k3 = rate(add(a, k2, 0.5 * h), dp);
    const S k4 = rate(add(a, k3, h), dp);
    auto mix = [&](double a1, double b1, double c1, double d1) { return (a1 + (2.0 * b1) + (2.0 * c1) + d1) / 6.0; };
    const S k{mix(k1.x, k2.x, k3.x, k4.x),     mix(k1.y, k2.y, k3.y, k4.y),     mix(k1.vx, k2.vx, k3.vx, k4.vx), mix(k1.vy, k2.vy, k3.vy, k4.vy),
              mix(k1.th, k2.th, k3.th, k4.th), mix(k1.om, k2.om, k3.om, k4.om), mix(k1.m, k2.m, k3.m, k4.m)};
    const S n = add(a, k, h);
    x = n.x;
    y = n.y;
    vx = n.vx;
    vy = n.vy;
    th = n.th;
    om = n.om;
    m = std::fmax(n.m, p.m_dry);
    t += h;
  }
};

// The vehicle is unstable, so the verification flights are closed with a fixed-gain PD law on the attitude and rate (degrees in, degrees out) that tracks a
// slow sinusoid plus a step: the vehicle really pitches and the aerodynamics act, and the flight stays bounded so that two models can be compared along it.
double program_cmd(double t, double tilt_deg, double rate_dps) {
  const double ref = (2.0 * std::sin(2.0 * sim::kPi * t / 10.0)) + (t > 12.0 ? 1.0 : 0.0);
  return (1.0 * (ref - tilt_deg)) - (0.5 * rate_dps);
}

}  // namespace

TFC_TEST(verification_the_3d_model_matches_an_independently_written_planar_model_through_a_pitching_ascent_with_an_engine_out) {
  using namespace sim;
  Scenario sc;
  sc.engine_out_time = 20.0;
  sc.engine_out_index = 1;
  sc.wind_scale = 0.0;
  Vehicle6 v(Params{}, sc);
  Planar pl(Params{});
  const double h = 0.002;
  double worst_pos = 0.0;
  double worst_att = 0.0;
  double worst_mass = 0.0;
  double max_tilt = 0.0;
  for (int k = 0; k < 5000; ++k) {  // 50 s in 10 ms frames
    const double t0 = k * 0.01;
    const double cmd = program_cmd(t0, v.tilts().y_deg, v.state().w.z * kRad2Deg);
    const double cmd_pl = program_cmd(t0, pl.th * kRad2Deg, pl.om * kRad2Deg);
    v.step(0.01, cmd, 0.0);
    for (int i = 0; i < 5; ++i) {
      if (pl.t >= 20.0) {
        pl.out1 = true;
      }
      pl.step(h, cmd_pl);
    }
    worst_pos = std::fmax(worst_pos, std::hypot(v.state().r.x - pl.x, v.state().r.y - pl.y));
    worst_att = std::fmax(worst_att, std::fabs(v.tilts().y_deg - (pl.th * kRad2Deg)));
    worst_mass = std::fmax(worst_mass, std::fabs(v.mass() - pl.m));
    max_tilt = std::fmax(max_tilt, std::fabs(v.tilts().y_deg));
    CHECK(near_abs(v.state().r.z, 0.0, 1e-9));  // it never leaves the plane
  }
  std::printf("  planar cross-check over 50 s: worst position difference %.3g m, attitude %.3g deg, mass %.3g kg (final tilt %.1f deg, altitude %.0f m)\n",
              worst_pos, worst_att, worst_mass, v.tilts().y_deg, v.altitude());
  CHECK(worst_pos < 1e-3);
  CHECK(worst_att < 1e-4);
  CHECK(worst_mass < 1e-6);
  CHECK(max_tilt > 2.0 && v.altitude() > 3000.0 && v.engines_on() == 4);  // a real flight, not a trivial one
}

TFC_TEST(verification_the_yaw_plane_is_the_mirror_of_the_pitch_plane) {
  using namespace sim;
  Scenario sc;
  sc.wind_scale = 0.0;
  Vehicle6 a(Params{}, sc);
  Vehicle6 b(Params{}, sc);
  for (int k = 0; k < 4000; ++k) {
    const double ca = program_cmd(k * 0.01, a.tilts().y_deg, a.state().w.z * kRad2Deg);
    const double cb = program_cmd(k * 0.01, b.tilts().x_deg, b.state().w.y * kRad2Deg);  // the yaw plane turns about body Y
    a.step(0.01, ca, 0.0);
    b.step(0.01, 0.0, cb);
  }
  CHECK(near_abs(a.tilts().y_deg, b.tilts().x_deg, 1e-6));
  CHECK(near_abs(a.state().r.y, -b.state().r.z, 1e-6));  // downrange in one is -crossrange in the other
  CHECK(near_abs(a.state().r.x, b.state().r.x, 1e-6));
  CHECK(near_abs(a.mass(), b.mass(), 1e-9));
  CHECK(near_abs(a.state().w.z, b.state().w.y, 1e-9));
  CHECK(std::fabs(a.state().r.y) > 10.0);  // and the flight did move sideways
}

TFC_TEST(verification_the_answer_converges_as_the_integration_step_is_halved) {
  using namespace sim;
  auto fly_with = [](double substep) {
    Params p;
    p.max_substep = substep;
    Scenario sc;
    sc.wind_scale = 0.0;
    Vehicle6 v(p, sc);
    for (int k = 0; k < 4000; ++k) {
      v.step(0.01, program_cmd(k * 0.01, v.tilts().y_deg, v.state().w.z * kRad2Deg), 0.0);
    }
    return v;
  };
  const Vehicle6 ref = fly_with(0.0005);
  double prev = 0.0;
  for (const double h : {0.004, 0.002, 0.001}) {
    const Vehicle6 v = fly_with(h);
    const double e = norm(v.state().r - ref.state().r);
    const double ea = std::fabs(v.tilts().y_deg - ref.tilts().y_deg);
    std::printf("  substep %.1f ms: position error %.3g m, attitude error %.3g deg (against 0.5 ms)\n", h * 1000.0, e, ea);
    if (prev > 0.0) {
      CHECK(e < prev / 1.8);  // converging, though not at RK4's order: the gimbal is sampled and held over each substep, which is first-order
    }
    prev = e;
    if (h <= 0.002) {
      CHECK(e < 0.01 && ea < 1e-3);  // the 2 ms the simulator uses is converged to a centimetre after 40 s
    }
  }
}

TFC_TEST(verification_the_unstable_pitch_mode_grows_at_the_rate_the_aerodynamics_predict) {
  using namespace sim;
  // A coasting, unpowered vehicle in fast air with its axis a small angle off the velocity: theta'' = a (theta - gamma) with a = q A CNalpha (xcp - xcg) / I,
  // so the angle of attack grows as cosh(sqrt(a) t) while the sideways drift of the velocity is still small.
  Params p;
  p.gravity_scale = 0.0;
  p.m_prop0 = 0.0;
  Scenario sc;
  sc.wind_scale = 0.0;
  Vehicle6 v(p, sc);
  State s;
  s.m = p.m_dry;
  s.r = V3{kEarthR + 8000.0, 0.0, 0.0};
  const double a0 = 0.5 * kDeg2Rad;  // the axis points half a degree toward +Y of the velocity
  s.v = V3{300.0, 0.0, 0.0};
  s.q = normalized(Q4{std::cos(0.5 * a0), 0.0, 0.0, std::sin(0.5 * a0)});
  v.set_state(s);
  const Loads l0 = v.current_loads();
  const MassProps mp = v.mass_props(p.m_dry);
  const double area = kPi * 0.25 * p.diameter * p.diameter;
  const double a_div = l0.dynamic_pressure * area * p.c_n_alpha * (p.x_cp - mp.x_cg) / mp.i_t;
  const double rate = std::sqrt(a_div);
  CHECK(rate > 0.5 && rate < 3.0);
  const double t_end = 0.5;
  v.step(t_end, 0.0, 0.0);
  const double predicted = a0 * std::cosh(rate * t_end);
  const double axis = std::atan2(2.0 * ((v.state().q.w * v.state().q.z) + (v.state().q.x * v.state().q.y)),
                                 1.0 - (2.0 * ((v.state().q.y * v.state().q.y) + (v.state().q.z * v.state().q.z))));
  const double vel = std::atan2(v.state().v.y, v.state().v.x);
  const double alpha = axis - vel;
  std::printf("  divergence: rate %.3f /s, angle of attack after %.1f s: %.4f deg (linear prediction %.4f deg)\n", rate, t_end, alpha * kRad2Deg,
              predicted * kRad2Deg);
  CHECK(close(alpha, predicted, 0.05));  // the 4% or so that the sideways acceleration of the velocity takes off the growth
}

TFC_TEST(the_committed_firmware_tables_are_the_ones_the_design_gives) {
  // firmware/app/src/flight_tables.hpp is generated by tools/vehicle/gen_tables.cpp. Compared here with a tolerance, not as text: the last digit of a table
  // value may differ between C libraries; a real change of the design moves the numbers by far more.
  tfc::GainSchedule committed_gains;
  tfc::Guidance committed_guidance;
  fc::tables::load(committed_gains, committed_guidance);
  const sim::FlightTables now = sim::flight_tables(sim::Params{});
  CHECK(committed_guidance.size(1U) == now.guidance.size(1U) && committed_guidance.size(1U) == 16U);
  CHECK(committed_gains.size() == now.gains.size() && committed_gains.size() > 10U);
  for (unsigned i = 0; i < now.guidance.size(1U); ++i) {
    CHECK(committed_guidance.point(1U, i).frame == now.guidance.point(1U, i).frame);
    CHECK(near_abs(static_cast<double>(committed_guidance.point(1U, i).deg), static_cast<double>(now.guidance.point(1U, i).deg), 1e-3));
  }
  for (unsigned i = 0; i < now.gains.size(); ++i) {
    CHECK(committed_gains.frame_at(i) == now.gains.frame_at(i));
    CHECK(near_abs(static_cast<double>(committed_gains.gains_at(i).kp), static_cast<double>(now.gains.gains_at(i).kp), 1e-4));
    CHECK(near_abs(static_cast<double>(committed_gains.gains_at(i).kd), static_cast<double>(now.gains.gains_at(i).kd), 1e-4));
    CHECK(near_abs(static_cast<double>(committed_gains.gains_at(i).ki), static_cast<double>(now.gains.gains_at(i).ki), 1e-4));
  }
}
