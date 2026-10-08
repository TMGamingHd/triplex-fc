// SPDX-License-Identifier: MIT
// The general vehicle description (sim/vehicle/spec.hpp, docs/design/VEHICLE_SPEC.md): stages, tanks, engines at any position, payloads, staging, throttle and thrust transients.
// Two kinds of check. First, the rewrite that made the model general must not have changed the reference vehicle: it is held to the frozen single-vehicle implementation it replaced
// (tests/oracle/vehicle6_legacy.hpp) over whole flights. Second, each new capability is checked against an answer that does not come from the code: the multi-stage rocket equation,
// the moment of a force at a position (r x F), the centre of gravity of tanks by hand, a first-order rise.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "tfc_test.hpp"

#include "design.hpp"
#include "oracle/vehicle6_legacy.hpp"
#include "spec.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A vehicle of one stage, built by hand: a 1000 kg structure, `prop` kg in one tank, and the engines given.
sim::Params one_stage(double prop, const std::vector<sim::EngineSpec>& engines) {
  sim::Params p;
  p.ground_contact = false;
  sim::StageSpec st;
  st.name = "only";
  st.dry_mass = 1000.0;
  st.x_start = 0.0;
  st.length = 6.0;
  st.radius = 0.5;
  st.inertia_factor = 0.6;
  st.x_cg_dry = 3.0;
  sim::TankSpec tk;
  tk.propellant = prop;
  tk.x_bottom = 0.5;
  tk.radius = 0.5;
  tk.density = 1000.0;
  st.tanks.push_back(tk);
  p.spec.stages.push_back(st);
  p.spec.engines = engines;
  return p;
}

sim::EngineSpec engine_at(double x, double y, double z, double thrust) {
  sim::EngineSpec e;
  e.pos = sim::V3{x, y, z};
  e.thrust_vac = thrust;
  e.exit_area = 0.0;  // no ambient-pressure loss: the thrust is exactly what it says
  e.isp_vac = 300.0;
  return e;
}

// In space and without gravity, so that only the engines act: 300 km up, at rest.
sim::Scenario in_space() {
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 300000.0, 0.0, 0.0};
  return sc;
}

}  // namespace

// ---- the reference vehicle against the frozen implementation ----

TFC_TEST(general_the_reference_vehicle_flies_as_the_frozen_single_vehicle_model_did) {
  // Four scenarios of increasing difficulty, each flown by a PD loop that reads the OLD model, both models given the same commands for 60 s; they must agree to the rounding of the
  // sums (the new model keeps the propellant per stage, the old one derived it from the total mass: the same number to 1e-12 kg, which is the whole difference).
  for (int which = 0; which <= 3; ++which) {
    sim::Params np;
    sim::Scenario ns;
    sim::legacy::Params op;
    sim::legacy::Scenario os;
    if (which >= 1) {
      np.gimbal_lag_s = op.gimbal_lag_s = 0.05;
      np.thrust_misalign_pitch_deg = op.thrust_misalign_pitch_deg = 0.3;
      np.thrust_scale = op.thrust_scale = 0.97;
      np.cn_scale = op.cn_scale = 1.1;
    }
    if (which >= 2) {
      ns.engine_out_time = os.engine_out_time = 30.0;
      ns.engine_out_index = os.engine_out_index = 2;
      ns.dry_cg_shift = os.dry_cg_shift = -0.2;
      ns.gusts.push_back(sim::Gust{15.0, 3.0, sim::V3{0.0, 0.0, 12.0}});
      os.gusts.push_back(sim::legacy::Gust{15.0, 3.0, sim::V3{0.0, 0.0, 12.0}});
    }
    if (which == 3) {
      np.ideal_roll_control = op.ideal_roll_control = false;
      np.cd_scale = op.cd_scale = 1.3;
      ns.wind_scale = os.wind_scale = 2.0;
    }
    sim::Vehicle6 a(np, ns);
    sim::legacy::Vehicle6 b(op, os);
    for (int k = 1; k <= 6000; ++k) {
      const sim::legacy::Tilts tl = b.tilts();
      const sim::V3& w = b.state().w;
      const double cp = (-1.4 * tl.y_deg) - (0.6 * w.z * sim::kRad2Deg);
      const double cy = (-1.4 * tl.x_deg) - (0.6 * w.y * sim::kRad2Deg);
      a.step(0.01, cp, cy);
      b.step(0.01, cp, cy);
      if (k % 1000 == 0) {
        const sim::State& sa = a.state();
        const sim::legacy::State& sb = b.state();
        CHECK(sim::norm(sa.r - sb.r) < 1e-5 && sim::norm(sa.v - sb.v) < 1e-5);
        CHECK(near_abs(sa.q.w, sb.q.w, 1e-7) && near_abs(sa.q.x, sb.q.x, 1e-7) && near_abs(sa.q.y, sb.q.y, 1e-7) && near_abs(sa.q.z, sb.q.z, 1e-7));
        CHECK(sim::norm(sa.w - sb.w) < 1e-7 && near_abs(sa.m, sb.m, 1e-8));
        CHECK(near_abs(a.gimbal_pitch_deg(), b.gimbal_pitch_deg(), 1e-9) && a.engines_on() == b.engines_on());
      }
    }
    CHECK(a.altitude() > 3000.0 && near_abs(a.altitude(), b.altitude(), 1e-5));
  }
}

TFC_TEST(general_the_reference_vehicle_described_as_a_spec_flies_as_the_reference_vehicle_does) {
  sim::Params simple;
  sim::Params general;
  general.spec = sim::legacy_spec(general);  // the same vehicle, written out as stages, tanks and engines
  CHECK(sim::validate(general.spec).empty());
  sim::Vehicle6 a(simple);
  sim::Vehicle6 b(general);
  for (int k = 0; k < 3000; ++k) {
    const double c = 1.0 * std::sin(0.02 * k);
    a.step(0.01, c, -c);
    b.step(0.01, c, -c);
  }
  CHECK(a.state().r.x == b.state().r.x && a.state().v.y == b.state().v.y && a.state().q.z == b.state().q.z && a.mass() == b.mass());  // bit for bit: it is the same description
  CHECK(b.spec().stages.size() == 1U && b.spec().engines.size() == 5U);
}

// ---- engines at positions ----

TFC_TEST(general_an_engine_off_the_axis_makes_the_moment_r_cross_f) {
  const sim::Params p = one_stage(1000.0, {engine_at(0.0, 1.0, 0.5, 10000.0)});
  const sim::Vehicle6 v(p, in_space());
  const sim::Loads l = v.current_loads();
  CHECK(near_abs(l.f_thrust.x, 10000.0, 1e-9));
  const double xcg = v.mass_props().x_cg;
  // r = (0 - x_cg, 1, 0.5), F = (T, 0, 0): r x F = (0, 0.5 T, -1.0 T)
  CHECK(near_abs(l.m_thrust.x, 0.0, 1e-9) && near_abs(l.m_thrust.y, 5000.0, 1e-9) && near_abs(l.m_thrust.z, -10000.0, 1e-9));
  CHECK(xcg > 1.0 && xcg < 4.0);
}

TFC_TEST(general_an_engine_ahead_of_the_centre_of_gravity_pushes_the_other_way_and_a_cant_tilts_the_thrust) {
  sim::EngineSpec fwd = engine_at(5.5, 0.0, 0.0, 10000.0);  // forward of the CG: the moment of an axial force on the axis is zero, but its arm along x does not enter
  fwd.cant_pitch_deg = 3.0;
  const sim::Params p = one_stage(1000.0, {fwd});
  const sim::Vehicle6 v(p, in_space());
  const sim::Loads l = v.current_loads();
  const double s3 = std::sin(3.0 * sim::kDeg2Rad);
  const double c3 = std::cos(3.0 * sim::kDeg2Rad);
  CHECK(near_abs(l.f_thrust.x, 10000.0 * c3, 1e-6) && near_abs(l.f_thrust.y, -10000.0 * s3, 1e-6) && near_abs(l.f_thrust.z, 0.0, 1e-9));
  const double arm = 5.5 - v.mass_props().x_cg;  // r = (arm, 0, 0), F = (Fx, Fy, 0): r x F = (0, 0, arm Fy)
  CHECK(arm > 0.0 && near_abs(l.m_thrust.z, arm * (-10000.0 * s3), 1e-6));
}

TFC_TEST(general_a_ring_of_many_engines_has_no_moment_and_losing_one_has_the_moment_of_that_engine) {
  std::vector<sim::EngineSpec> ring;
  for (int i = 0; i < 33; ++i) {  // 32 around a circle and one in the middle
    const double a = (i < 32) ? 2.0 * sim::kPi * i / 32.0 : 0.0;
    const double rad = (i < 32) ? 1.6 : 0.0;
    ring.push_back(engine_at(0.0, rad * std::cos(a), rad * std::sin(a), 5000.0));
  }
  sim::Params p = one_stage(1000.0, ring);
  const sim::Vehicle6 whole(p, in_space());
  const sim::Loads l = whole.current_loads();
  CHECK(near_abs(l.thrust, 33.0 * 5000.0, 1e-6));
  CHECK(sim::norm(l.m_thrust) < 1e-6 && near_abs(l.f_thrust.y, 0.0, 1e-6));
  sim::Scenario sc = in_space();
  sc.engine_failures.push_back(sim::EngineFailure{0.0, 4});  // engine 4 is at angle 2 pi 4 / 32 = 45 degrees
  sim::Vehicle6 broken(p, sc);
  broken.step(0.01, 0.0, 0.0);
  const sim::Loads lb = broken.current_loads();
  CHECK(near_abs(lb.thrust, 32.0 * 5000.0, 1e-6) && broken.engines_on() == 32);
  const double a4 = 2.0 * sim::kPi * 4.0 / 32.0;  // engine 4 sits at (y, z) = 1.6 (cos a4, sin a4); its moment was (0, z T, -y T), so the rest has the opposite one
  CHECK(near_abs(lb.m_thrust.y, -1.6 * std::sin(a4) * 5000.0, 1e-3) && near_abs(lb.m_thrust.z, 1.6 * std::cos(a4) * 5000.0, 1e-3));
  CHECK(near_abs(lb.m_thrust.x, 0.0, 1e-6));
}

// ---- tanks and masses ----

TFC_TEST(general_the_centre_of_gravity_of_tanks_and_a_payload_is_the_mass_weighted_mean) {
  sim::Params p = one_stage(0.0, {engine_at(0.0, 0.0, 0.0, 10000.0)});
  sim::StageSpec& st = p.spec.stages[0];
  st.tanks.clear();
  sim::TankSpec a;
  a.propellant = 600.0;
  a.x_bottom = 1.0;
  a.radius = 0.5;
  a.density = 1000.0;
  sim::TankSpec b;
  b.propellant = 300.0;
  b.x_bottom = 4.0;
  b.radius = 0.4;
  b.density = 500.0;
  st.tanks = {a, b};
  p.spec.payloads.push_back(sim::PayloadSpec{"sat", 200.0, 6.5, -1.0, {}});
  const sim::Vehicle6 v(p, in_space());
  // full tanks: columns of height m / (rho pi r^2); the centroid of a column is its middle
  const double ha = 600.0 / (1000.0 * sim::kPi * 0.25);
  const double hb = 300.0 / (500.0 * sim::kPi * 0.16);
  const double xa = 1.0 + 0.5 * ha;
  const double xb = 4.0 + 0.5 * hb;
  const double m = 1000.0 + 600.0 + 300.0 + 200.0;
  const double x = (1000.0 * 3.0 + 600.0 * xa + 300.0 * xb + 200.0 * 6.5) / m;
  const sim::MassProps mp = v.mass_props();
  CHECK(close(mp.mass, m, 1e-12) && close(mp.x_cg, x, 1e-12) && close(v.mass(), m, 1e-12));
  // the transverse inertia by the parallel-axis theorem, by hand
  const double i_dry = (1000.0 * 6.0 * 6.0 / 12.0 * 0.6) + (1000.0 * (3.0 - x) * (3.0 - x));
  const double i_a = (600.0 * (3.0 * 0.25 + ha * ha) / 12.0) + (600.0 * (xa - x) * (xa - x));
  const double i_b = (300.0 * (3.0 * 0.16 + hb * hb) / 12.0) + (300.0 * (xb - x) * (xb - x));
  const double i_p = 200.0 * (6.5 - x) * (6.5 - x);
  CHECK(close(mp.i_t, i_dry + i_a + i_b + i_p, 1e-12));
  CHECK(close(mp.i_x, (1000.0 * 0.25) + (0.5 * 600.0 * 0.25) + (0.5 * 300.0 * 0.16), 1e-12));
}

TFC_TEST(general_tanks_drain_together_or_one_after_the_other) {
  const auto tank_masses_hold = [](bool sequential) {
    sim::Params p = one_stage(0.0, {engine_at(0.0, 0.0, 0.0, 10000.0)});
    sim::StageSpec& st = p.spec.stages[0];
    st.tanks.clear();
    sim::TankSpec a;
    a.propellant = 600.0;
    a.x_bottom = 1.0;
    a.radius = 0.5;
    a.density = 1000.0;
    sim::TankSpec b = a;
    b.propellant = 300.0;
    b.x_bottom = 3.5;
    st.tanks = {a, b};
    st.sequential_drain = sequential;
    sim::Vehicle6 v(p, in_space());
    sim::State s = v.state();
    s.prop[0] = 450.0;  // half the propellant is gone
    s.prop_set = true;
    v.set_state(s);
    const double m = v.mass_props().mass;
    const double ha_par = 300.0 / (1000.0 * sim::kPi * 0.25);  // together: 2/3 of 450 in the big tank, 1/3 in the small one
    const double hb_par = 150.0 / (1000.0 * sim::kPi * 0.25);
    const double ha_seq = 150.0 / (1000.0 * sim::kPi * 0.25);  // one after the other: the first tank listed has given up what it can: 450 - 300 = 150 kg left in it, the second is full
    const double hb_seq = 300.0 / (1000.0 * sim::kPi * 0.25);
    const double xa = sequential ? 1.0 + 0.5 * ha_seq : 1.0 + 0.5 * ha_par;
    const double xb = sequential ? 3.5 + 0.5 * hb_seq : 3.5 + 0.5 * hb_par;
    const double ma = sequential ? 150.0 : 300.0;
    const double mb = sequential ? 300.0 : 150.0;
    const double x = (1000.0 * 3.0 + ma * xa + mb * xb) / (1000.0 + ma + mb);
    return close(m, 1000.0 + 450.0, 1e-12) && close(v.mass_props().x_cg, x, 1e-12);
  };
  CHECK(tank_masses_hold(false));
  CHECK(tank_masses_hold(true));
}

// ---- stages: the rocket equation, separation, ignition ----

TFC_TEST(general_a_two_stage_vehicle_in_vacuum_gains_the_speed_of_the_rocket_equation_stage_by_stage) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  sim::StageSpec s1;
  s1.name = "first";
  s1.dry_mass = 2000.0;
  s1.length = 6.0;
  s1.radius = 0.9;
  s1.x_cg_dry = 3.0;
  s1.tanks.push_back(sim::TankSpec{8000.0, 0.5, 0.9, 900.0, {}});
  s1.separate_on_burnout = true;
  s1.separate_delay_s = 1.0;
  sim::StageSpec s2;
  s2.name = "second";
  s2.dry_mass = 500.0;
  s2.x_start = 6.0;
  s2.length = 4.0;
  s2.radius = 0.9;
  s2.x_cg_dry = 8.0;
  s2.tanks.push_back(sim::TankSpec{2000.0, 6.5, 0.9, 900.0, {}});
  s2.ignite_after_sep_of = 0;
  s2.ignite_delay_s = 2.0;
  p.spec.stages = {s1, s2};
  sim::EngineSpec e1 = engine_at(0.0, 0.0, 0.0, 200000.0);
  e1.isp_vac = 300.0;
  sim::EngineSpec e2 = engine_at(6.0, 0.0, 0.0, 40000.0);
  e2.stage = 1;
  e2.isp_vac = 350.0;
  p.spec.engines = {e1, e2};
  p.spec.payloads.push_back(sim::PayloadSpec{"payload", 300.0, 10.0, -1.0, {}});
  CHECK(sim::validate(p.spec).empty());
  sim::Vehicle6 v(p, in_space());
  CHECK(close(v.mass(), 12800.0, 1e-12));
  double t_burnout1 = -1.0;
  double t_separation = -1.0;
  double mass_after_sep = -1.0;
  bool stage1_gone_when_second_lit = false;
  for (int k = 0; k < 40000 && v.time() < 330.0; ++k) {
    v.step(0.01, 0.0, 0.0);
    if (t_burnout1 < 0.0 && v.propellant(0) <= 1e-6) {
      t_burnout1 = v.time();
    }
    if (mass_after_sep < 0.0 && !v.stage_active(0)) {
      mass_after_sep = v.mass();
      t_separation = v.time();
    }
    if (v.stage_ignited(1) && !v.stage_active(0)) {
      stage1_gone_when_second_lit = true;
    }
  }
  const double g0 = sim::kG0;
  const double dv1 = 300.0 * g0 * std::log(12800.0 / 4800.0);
  const double dv2 = 350.0 * g0 * std::log(2800.0 / 800.0);
  CHECK(close(t_burnout1, 8000.0 / (200000.0 / (300.0 * g0)), 2e-3));  // 117.7 s
  CHECK(close(mass_after_sep, 2800.0, 1e-9));                          // the first stage is gone: 500 + 2000 + 300
  CHECK(near_abs(t_separation - t_burnout1, 1.0, 0.03));               // one second after its burnout: separate_delay_s
  CHECK(stage1_gone_when_second_lit && v.stage_ignited(1) && !v.stage_active(0) && v.stage_active(1));
  CHECK(close(v.mass(), 800.0, 1e-6));                                 // second stage burnt out: its structure and the payload
  CHECK(close(v.speed(), dv1 + dv2, 5e-4));                            // the rocket equation, twice
  CHECK(v.engines_on() == 1 && v.engine_count() == 1);                 // only the second stage's engine is left
}

TFC_TEST(general_a_stage_ignites_at_its_time_and_a_payload_leaves_at_its_time) {
  sim::Params p = one_stage(500.0, {engine_at(0.0, 0.0, 0.0, 20000.0)});
  p.gravity_scale = 0.0;
  p.spec.stages[0].ignite_time_s = 2.5;
  p.spec.payloads.push_back(sim::PayloadSpec{"fairing", 150.0, 5.0, 4.0, {}});
  sim::Vehicle6 v(p, in_space());
  CHECK(close(v.mass(), 1000.0 + 500.0 + 150.0, 1e-12) && v.current_loads().thrust == 0.0 && !v.stage_ignited(0));
  while (v.time() < 2.0) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.current_loads().thrust == 0.0 && v.speed() == 0.0);  // not lit yet: nothing happens
  while (v.time() < 3.0) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.stage_ignited(0) && v.current_loads().thrust > 19999.0 && v.speed() > 1.0);
  const double before = v.mass();
  while (v.time() < 4.05) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.mass() < before - 140.0 && v.mass() > before - 170.0);  // 150 kg jettisoned, a little propellant burnt
}

// ---- engines: throttle, rise, tail, cut-off, failures ----

TFC_TEST(general_a_throttle_table_scales_thrust_and_mass_flow_linearly_between_its_points) {
  sim::Params p = one_stage(500.0, {engine_at(0.0, 0.0, 0.0, 20000.0)});
  p.spec.stages[0].throttle = {{0.0, 1.0}, {10.0, 1.0}, {20.0, 0.6}, {30.0, 0.6}};
  sim::Vehicle6 v(p, in_space());
  const double mdot_full = v.current_loads().mdot;
  CHECK(close(v.current_loads().thrust, 20000.0, 1e-12));
  while (v.time() < 15.0 - 1e-9) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(close(v.current_loads().thrust, 20000.0 * 0.8, 1e-3) && close(v.current_loads().mdot, mdot_full * 0.8, 1e-3));  // half way down the ramp (the throttle of the last substep's start: 2 ms old)
  while (v.time() < 25.0 - 1e-9) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(close(v.current_loads().thrust, 20000.0 * 0.6, 1e-9));
}

TFC_TEST(general_thrust_rises_and_dies_away_with_the_time_constants_given) {
  sim::EngineSpec e = engine_at(0.0, 0.0, 0.0, 20000.0);
  e.rise_s = 0.5;
  e.tail_s = 0.25;
  e.cutoff_time_s = 5.0;
  const sim::Params p = one_stage(500.0, {e});
  sim::Vehicle6 v(p, in_space());
  CHECK(v.current_loads().thrust == 0.0);  // it starts cold
  while (v.time() < 0.5 - 1e-9) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(near_abs(v.current_loads().thrust / 20000.0, 1.0 - std::exp(-1.0), 0.01));  // one time constant: 63 %
  while (v.time() < 4.99) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.current_loads().thrust / 20000.0 > 0.999);
  while (v.time() < 5.25 - 1e-9) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(near_abs(v.current_loads().thrust / 20000.0, std::exp(-1.0), 0.01));  // a tail-off time constant after the cut-off at 5 s
}

TFC_TEST(general_engines_can_fail_at_different_times_and_each_is_counted) {
  const std::vector<sim::EngineSpec> four(4U, engine_at(0.0, 0.0, 0.0, 10000.0));
  const sim::Params p = one_stage(2000.0, four);
  sim::Scenario sc = in_space();
  sc.engine_failures = {sim::EngineFailure{1.0, 0}, sim::EngineFailure{2.0, 3}, sim::EngineFailure{2.0, 99}};  // the last names an engine that does not exist: ignored
  sim::Vehicle6 v(p, sc);
  CHECK(v.engines_on() == 4);
  while (v.time() < 1.5) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.engines_on() == 3 && close(v.current_loads().thrust, 30000.0, 1e-12));
  while (v.time() < 2.5) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.engines_on() == 2 && close(v.current_loads().thrust, 20000.0, 1e-12));
}

TFC_TEST(general_a_vehicle_can_start_in_orbit_and_stay_there) {
  sim::Params p = one_stage(100.0, {engine_at(0.0, 0.0, 0.0, 1000.0)});
  p.spec.stages[0].ignite_time_s = 1.0e9;  // never lit
  sim::Scenario sc;
  sc.has_initial = true;
  const double r0 = sim::kEarthR + 400000.0;
  sc.initial.r = sim::V3{r0, 0.0, 0.0};
  sc.initial.v = sim::V3{0.0, std::sqrt(sim::kEarthMu / r0), 0.0};  // a circular orbit
  sim::Vehicle6 v(p, sc);
  const double e0 = (0.5 * sim::dot(v.state().v, v.state().v)) - (sim::kEarthMu / sim::norm(v.state().r));
  for (int k = 0; k < 30000; ++k) {  // 300 s of the 92 minutes
    v.step(0.01, 0.0, 0.0);
  }
  const double e1 = (0.5 * sim::dot(v.state().v, v.state().v)) - (sim::kEarthMu / sim::norm(v.state().r));
  CHECK(near_abs(sim::norm(v.state().r), r0, 5.0) && close(e0, e1, 1e-9));
  CHECK(!v.on_ground() && v.altitude() > 399000.0);
}

// ---- the description is checked ----

TFC_TEST(general_a_bad_description_is_refused_with_a_message_that_names_the_field) {
  const auto message_has = [](const sim::VehicleSpec& s, const char* text) {
    for (const std::string& m : sim::validate(s)) {
      if (m.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  const sim::Params good = one_stage(500.0, {engine_at(0.0, 0.0, 0.0, 20000.0)});
  CHECK(sim::validate(good.spec).empty());
  sim::VehicleSpec s = good.spec;
  s.stages[0].dry_mass = 0.0;
  CHECK(message_has(s, "stages[0].dry_mass"));
  s = good.spec;
  s.engines[0].stage = 3;
  CHECK(message_has(s, "engines[0].stage"));
  s = good.spec;
  s.engines[0].isp_vac = -1.0;
  CHECK(message_has(s, "engines[0]"));
  s = good.spec;
  s.stages[0].tanks[0].propellant = 1.0e6;  // a column far longer than the stage
  CHECK(message_has(s, "more propellant than fits"));
  s = good.spec;
  s.stages[0].tanks.clear();
  CHECK(message_has(s, "has engines and no propellant"));
  s = good.spec;
  s.stages[0].ignite_after_sep_of = 0;
  CHECK(message_has(s, "an earlier stage"));
  s = good.spec;
  s.stages[0].throttle = {{5.0, 1.0}, {2.0, 0.5}};
  CHECK(message_has(s, "throttle"));
  s = good.spec;
  s.payloads.push_back(sim::PayloadSpec{"bad", -1.0, 0.0, -1.0, {}});
  CHECK(message_has(s, "payloads[0].mass"));
  s = good.spec;
  for (unsigned i = 0; i <= sim::kMaxEngines; ++i) {
    s.engines.push_back(engine_at(0.0, 0.0, 0.0, 1000.0));
  }
  CHECK(message_has(s, "at most"));
}

// ---- effectors: fins, thrusters, wheels ----

namespace {

// A bare stage: structure only, no engines, no tanks (a spacecraft body, or a rocket after burnout). 1000 kg, 2 m long: its transverse inertia is 1000 * 4 / 12 * 0.6 = 200 kg m^2.
sim::Params bare_body() {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = 1000.0;
  st.length = 2.0;
  st.radius = 0.5;
  st.x_cg_dry = 1.0;
  p.spec.stages.push_back(st);
  return p;
}

}  // namespace

TFC_TEST(effectors_fins_make_a_force_and_a_moment_from_dynamic_pressure_area_slope_and_deflection) {
  sim::Params p = bare_body();
  sim::FinSpec fin;
  fin.stage = 0;
  fin.x_hinge = 0.2;  // aft of the centre of gravity (x = 1)
  fin.area_each = 0.5;
  fin.lift_slope = 3.0;
  fin.limit_deg = 15.0;
  fin.rate_dps = 1.0e12;  // out of the way: the deflection is what is asked, at once
  p.spec.fins.push_back(fin);
  sim::Scenario sc;
  sc.wind_scale = 0.0;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 5000.0, 0.0, 0.0};
  sc.initial.v = sim::V3{200.0, 0.0, 0.0};  // flying straight ahead: no body angle of attack, so the only aerodynamic force is the fins'
  // Each check on a fresh vehicle after a microsecond: the fins have moved and the vehicle has not (it would otherwise turn, and its own aerodynamic force would join the fins').
  const auto after = [&](double cmd_pitch, double cmd_yaw) {
    sim::Vehicle6 v(p, sc);
    v.step(1e-6, cmd_pitch, cmd_yaw);
    return std::array<sim::Loads, 2>{v.current_loads(), sim::Loads{}};
  };
  const sim::Loads l = after(5.0, 0.0)[0];
  const double q = l.dynamic_pressure;
  const double fy = -q * 2.0 * 0.5 * 3.0 * (5.0 * sim::kDeg2Rad);
  CHECK(q > 10000.0);
  CHECK(near_abs(l.f_aero.y, fy, 1e-6 * std::fabs(fy)) && near_abs(l.f_aero.z, 0.0, 1e-6));
  const double x_cg = sim::Vehicle6(p, sc).mass_props().x_cg;
  CHECK(near_abs(l.m_aero.z, (0.2 - x_cg) * fy, 1e-4 * std::fabs(fy)) && l.m_aero.z > 0.0);  // (the body's own moment at an angle of attack of 2e-8 rad adds 0.02 N m)  // an aft fin, a positive command: a positive moment, as a positive gimbal gives
  const sim::Loads ly = after(0.0, 5.0)[0];
  CHECK(near_abs(ly.f_aero.z, q * 2.0 * 0.5 * 3.0 * (5.0 * sim::kDeg2Rad), 1e-6 * std::fabs(fy)) && ly.m_aero.y > 0.0);  // the yaw plane: along +Z, a positive moment about +Y
  const sim::Loads lm = after(40.0, 0.0)[0];  // far more than the fin can move
  CHECK(near_abs(lm.f_aero.y, -q * 2.0 * 0.5 * 3.0 * (15.0 * sim::kDeg2Rad), 1e-6 * std::fabs(fy) * 3.0));  // held at its 15 degree limit
}

TFC_TEST(effectors_a_canard_needs_a_negative_gain_and_the_control_effectiveness_says_so) {
  sim::Params aft = bare_body();
  sim::FinSpec f;
  f.x_hinge = 0.2;
  f.area_each = 0.5;
  aft.spec.fins.push_back(f);
  sim::Params canard = bare_body();
  f.x_hinge = 1.8;  // forward of the centre of gravity
  canard.spec.fins.push_back(f);
  sim::Params canard_fixed = canard;
  canard_fixed.spec.fins[0].gain = -1.0;
  sim::Scenario sc;
  sc.wind_scale = 0.0;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 5000.0, 0.0, 0.0};
  sc.initial.v = sim::V3{200.0, 0.0, 0.0};
  const double b_aft = sim::Vehicle6(aft, sc).control_effectiveness();
  const double b_canard = sim::Vehicle6(canard, sc).control_effectiveness();
  const double b_fixed = sim::Vehicle6(canard_fixed, sc).control_effectiveness();
  CHECK(b_aft > 0.0 && b_canard < 0.0 && b_fixed > 0.0);  // a canard with a positive gain works against the command
  CHECK(close(b_aft, b_fixed, 1e-12) || (b_fixed > 0.0 && std::fabs(b_fixed - b_aft) < b_aft));  // the same arm (0.8 m) either side: the same size
  const sim::Loads la = sim::Vehicle6(aft, sc).current_loads();
  CHECK(la.dynamic_pressure > 1.0);
}

TFC_TEST(effectors_thrusters_fire_in_proportion_to_the_command_make_the_torque_of_their_position_and_burn_propellant) {
  sim::Params p = bare_body();
  p.spec.stages[0].tanks.push_back(sim::TankSpec{20.0, 0.5, 0.4, 1000.0, {}});  // 20 kg of hydrazine
  sim::EngineSpec up = engine_at(0.0, 0.0, 0.0, 100.0);  // a thruster at the aft end firing toward -Y: a positive pitch command turns the nose toward +Y
  up.gimbal = false;
  up.dir = sim::V3{0.0, -1.0, 0.0};
  up.control = sim::Control::PitchPlus;
  up.full_cmd_deg = 2.0;
  up.isp_vac = 200.0;
  sim::EngineSpec down = up;
  down.dir = sim::V3{0.0, 1.0, 0.0};
  down.control = sim::Control::PitchMinus;
  p.spec.engines = {up, down};
  CHECK(sim::validate(p.spec).empty());
  sim::Vehicle6 v(p, in_space());
  CHECK(v.current_loads().thrust == 0.0);  // nothing is commanded: nothing fires
  v.step(0.01, 1.0, 0.0);                  // half of full: half duty
  sim::Loads l = v.current_loads();
  const double x_cg = v.mass_props().x_cg;
  CHECK(near_abs(l.f_thrust.y, -50.0, 1e-9) && near_abs(l.f_thrust.x, 0.0, 1e-9));
  CHECK(near_abs(l.m_thrust.z, (0.0 - x_cg) * -50.0, 1e-9) && l.m_thrust.z > 0.0);  // r x F with r = (0 - x_cg, 0, 0)
  CHECK(close(l.mdot, 0.5 * 100.0 / (200.0 * sim::kG0), 1e-12));
  v.step(0.01, 5.0, 0.0);  // more than full: full duty, not more
  CHECK(near_abs(v.current_loads().f_thrust.y, -100.0, 1e-9));
  v.step(0.01, -2.0, 0.0);  // the other direction: the other thruster
  l = v.current_loads();
  CHECK(near_abs(l.f_thrust.y, 100.0, 1e-9) && l.m_thrust.z < 0.0);
  // the yaw plane: yaw plus is a torque about +Y: an aft thruster firing along +Z and a forward one along -Z make a yaw-plus couple
  sim::EngineSpec yaw_aft = engine_at(0.0, 0.0, 0.0, 100.0);
  yaw_aft.gimbal = false;
  yaw_aft.dir = sim::V3{0.0, 0.0, 1.0};
  yaw_aft.control = sim::Control::YawPlus;
  yaw_aft.full_cmd_deg = 2.0;
  sim::EngineSpec yaw_fwd = yaw_aft;
  yaw_fwd.pos = sim::V3{2.0, 0.0, 0.0};
  yaw_fwd.dir = sim::V3{0.0, 0.0, -1.0};
  sim::Params py = p;
  py.spec.engines = {yaw_aft, yaw_fwd};
  sim::Vehicle6 vy(py, in_space());
  vy.step(0.01, 0.0, 1.0);  // half of the 2 degrees at which the thrusters are fully on
  const sim::Loads ly = vy.current_loads();
  CHECK(ly.m_thrust.y > 0.0 && near_abs(ly.f_thrust.z, 0.0, 1e-9) && near_abs(ly.m_thrust.z, 0.0, 1e-9));  // a pure couple about +Y: no net force
  CHECK(near_abs(ly.m_thrust.y, 0.5 * 100.0 * ((vy.mass_props().x_cg - 0.0) + (2.0 - vy.mass_props().x_cg)), 1e-6));  // half duty: half the torque of the two thrusters
  // the effectiveness: the torque of full thrust per radian of command = T (x_cg - 0) / (full_cmd rad) / I
  const double i_t = v.mass_props().i_t;
  CHECK(close(v.control_effectiveness(), 100.0 * v.mass_props().x_cg / (2.0 * sim::kDeg2Rad) / v.mass_props().i_t, 1e-9));
  CHECK(i_t > 0.0 && x_cg > 0.0);
  const double before = v.propellant(0);
  for (int k = 0; k < 200; ++k) {
    v.step(0.01, 2.0, 0.0);  // 2 s of full duty
  }
  const double burnt = before - v.propellant(0);
  CHECK(close(burnt, 2.0 * 100.0 / (200.0 * sim::kG0), 2e-2));  // 100 N / (200 s g0) = 0.051 kg/s: 0.102 kg in 2 s (the rise of the valve costs a little)
}

TFC_TEST(effectors_a_thruster_wired_the_wrong_way_makes_the_control_effectiveness_negative) {
  sim::Params p = bare_body();
  p.spec.stages[0].tanks.push_back(sim::TankSpec{20.0, 0.5, 0.4, 1000.0, {}});
  sim::EngineSpec th = engine_at(0.0, 0.0, 0.0, 100.0);
  th.gimbal = false;
  th.dir = sim::V3{0.0, 1.0, 0.0};  // an aft thruster pushing toward +Y turns the nose toward -Y: the opposite of what pitch-plus asks
  th.control = sim::Control::PitchPlus;
  th.full_cmd_deg = 2.0;
  p.spec.engines = {th};
  const sim::Vehicle6 v(p, in_space());
  CHECK(v.control_effectiveness() < 0.0);
  p.spec.engines[0].dir = sim::V3{0.0, -1.0, 0.0};
  CHECK(sim::Vehicle6(p, in_space()).control_effectiveness() > 0.0);
}

TFC_TEST(effectors_fin_gain_scales_the_deflection_and_a_negative_gain_reverses_it) {
  const auto force_with_gain = [](double gain) {
    sim::Params p = bare_body();
    sim::FinSpec fin;
    fin.x_hinge = 0.2;
    fin.area_each = 0.5;
    fin.gain = gain;
    fin.limit_deg = 15.0;
    fin.rate_dps = 1.0e12;
    p.spec.fins.push_back(fin);
    sim::Scenario sc;
    sc.wind_scale = 0.0;
    sc.has_initial = true;
    sc.initial.r = sim::V3{sim::kEarthR + 5000.0, 0.0, 0.0};
    sc.initial.v = sim::V3{200.0, 0.0, 0.0};
    sim::Vehicle6 v(p, sc);
    v.step(1e-6, 4.0, 0.0);
    return v.current_loads().f_aero.y;
  };
  const double full = force_with_gain(1.0);
  CHECK(full < 0.0);
  CHECK(close(force_with_gain(0.5), 0.5 * full, 1e-4));
  CHECK(close(force_with_gain(-1.0), -full, 1e-4));
}

TFC_TEST(effectors_a_wheels_momentum_rides_along_and_precesses_the_body) {
  sim::Params p = bare_body();
  p.ideal_roll_control = false;
  p.spec.wheels.enabled = true;
  p.spec.wheels.stage = 0;
  p.spec.wheels.torque_max = 1.0;
  p.spec.wheels.momentum_max = 50.0;
  p.spec.wheels.full_cmd_deg = 1.0;
  sim::Vehicle6 v(p, in_space());
  sim::State s = v.state();
  s.w = sim::V3{0.0, 0.1, 0.0};      // turning about Y at 0.1 rad/s
  s.wheel_h = sim::V3{0.0, 0.0, 3.0};  // with 3 N m s stored in the Z wheel
  v.set_state(s);
  const double i_x = v.mass_props().i_x;
  for (int k = 0; k < 100; ++k) {
    v.step(0.01, 0.0, 0.0);  // no command: only the gyroscopic coupling acts
  }
  // I w' = -w x (I w + H): the X component is -(w_y H_z - w_z H_y) = -0.3 N m, so w_x falls at 0.3 / I_x per second
  CHECK(near_abs(v.state().w.x, -0.3 / i_x * 1.0, 0.03 * 0.3 / i_x));
}

TFC_TEST(general_parallel_stages_gimbal_separately_each_with_its_own_limit) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.gimbal_rate_dps = 1.0e6;
  for (int i = 0; i < 2; ++i) {
    sim::StageSpec st;
    st.name = i == 0 ? "core" : "booster";
    st.dry_mass = 1000.0;
    st.length = 4.0;
    st.radius = 0.5;
    st.x_cg_dry = 2.0;
    st.tanks.push_back(sim::TankSpec{1000.0, 0.5, 0.5, 1000.0, {}});
    st.gimbal_limit_deg = i == 0 ? 2.0 : 6.0;  // the booster's nozzle can swing further than the core's
    p.spec.stages.push_back(st);
    for (int k = 0; k < 2; ++k) {  // two engines on each stage: they share one direction (found once per stage), which must be the stage's own
      p.spec.engines.push_back(engine_at(0.0, 0.0, 0.0, 10000.0));
      p.spec.engines.back().stage = i;
    }
  }
  sim::Vehicle6 v(p, in_space());
  v.step(0.1, 5.0, 0.0);  // ask for 5 degrees: the core stops at 2, the booster follows to 5
  const sim::Loads l = v.current_loads();
  CHECK(near_abs(l.f_thrust.y, -20000.0 * (std::sin(2.0 * sim::kDeg2Rad) + std::sin(5.0 * sim::kDeg2Rad)), 1e-6));
  CHECK(v.engines_on() == 4);
}

TFC_TEST(general_an_engine_that_starts_later_lights_later) {
  sim::EngineSpec early = engine_at(0.0, 0.0, 0.0, 10000.0);
  sim::EngineSpec late = engine_at(0.0, 0.0, 0.0, 30000.0);
  late.start_offset_s = 2.0;
  const sim::Params p = one_stage(2000.0, {early, late});
  sim::Vehicle6 v(p, in_space());
  while (v.time() < 1.0) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(close(v.current_loads().thrust, 10000.0, 1e-9));  // only the first
  while (v.time() < 3.0) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(close(v.current_loads().thrust, 40000.0, 1e-9));  // both
}

TFC_TEST(design_the_adaptive_schedule_follows_the_derivative_gain_and_honours_the_proportional_ceiling) {
  // a vehicle whose divergence tracks its effectiveness so that kp is constant while kd = 2 zeta wn / b is not: the points must go where kd changes
  std::vector<sim::NominalPoint> nominal;
  for (int k = 0; k <= 6000; ++k) {
    sim::NominalPoint n;
    n.t = k * 0.01;
    n.b_ctl = 1.0 + (99.0 * (1.0 - std::exp(-n.t / 1.5)));
    n.a_div = (5.0 * n.b_ctl) - 4.0;  // (wn^2 + a) / b = 5 whatever b is, for wn = 2 (and a is never negative, which the design would floor at 0)
    nominal.push_back(n);
  }
  const std::vector<sim::GainPoint> g = sim::gain_schedule_adaptive(nominal, 16U, 2.0, 0.8, 0.2, 1.0e30, 1.0e-3, 0.05);
  CHECK(g.size() > 4U && near_abs(g[0].kp, 5.0, 1e-9) && near_abs(g[3].kp, 5.0, 1e-6));  // kp is flat: only kd asks for points
  // the ceiling: no gain above it
  const std::vector<sim::GainPoint> capped = sim::gain_schedule_adaptive(nominal, 16U, 2.0, 0.8, 0.2, 2.0, 1.0e-3, 0.05);  // kp 5 is above it
  for (const sim::GainPoint& pt : capped) {
    CHECK(pt.kp <= 2.0 + 1e-12);
  }
  CHECK(near_abs(capped[0].ki, 0.2 * 2.0, 1e-12));
}

TFC_TEST(effectors_a_reaction_wheel_turns_the_vehicle_by_the_momentum_it_takes_and_stops_when_full) {
  sim::Params p = bare_body();
  p.spec.wheels.enabled = true;
  p.spec.wheels.stage = 0;
  p.spec.wheels.torque_max = 2.0;     // N m
  p.spec.wheels.momentum_max = 3.0;   // N m s
  p.spec.wheels.full_cmd_deg = 1.0;
  sim::Vehicle6 v(p, in_space());
  const double i_t = v.mass_props().i_t;
  CHECK(near_abs(i_t, 200.0, 1e-9));
  for (int k = 0; k < 100; ++k) {
    v.step(0.01, 1.0, 0.0);  // full torque for 1 s
  }
  CHECK(near_abs(v.state().w.z, 2.0 * 1.0 / i_t, 1e-6) && near_abs(v.state().wheel_h.z, -2.0, 1e-6));  // the body gains what the wheel loses: 2 N m s each way
  for (int k = 0; k < 400; ++k) {
    v.step(0.01, 1.0, 0.0);  // it keeps asking, and the wheel fills at 3 N m s after 1.5 s in all
  }
  CHECK(near_abs(v.state().wheel_h.z, -3.0, 1e-6));
  CHECK(near_abs(v.state().w.z, 3.0 / i_t, 1e-4));  // the body's angular momentum is the wheel's, opposite: I w = 3
  const double w_full = v.state().w.z;
  v.step(0.5, 1.0, 0.0);
  CHECK(near_abs(v.state().w.z, w_full, 1e-9));     // a full wheel gives no more torque in that direction...
  v.step(0.5, -1.0, 0.0);
  CHECK(v.state().w.z < w_full - 1e-4);             // ...but it can give it back the other way
  CHECK(close(v.control_effectiveness(), 2.0 / sim::kDeg2Rad / i_t, 1e-12));
}

TFC_TEST(effectors_bad_effector_descriptions_are_refused) {
  const auto has = [](const sim::VehicleSpec& s, const char* text) {
    for (const std::string& m : sim::validate(s)) {
      if (m.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  sim::Params p = bare_body();
  p.spec.stages[0].tanks.push_back(sim::TankSpec{20.0, 0.5, 0.4, 1000.0, {}});
  sim::EngineSpec th = engine_at(0.0, 0.0, 0.0, 100.0);
  th.gimbal = false;
  th.dir = sim::V3{0.0, -1.0, 0.0};
  th.control = sim::Control::PitchPlus;
  p.spec.engines = {th};
  CHECK(sim::validate(p.spec).empty());
  sim::VehicleSpec s = p.spec;
  s.engines[0].gimbal = true;
  CHECK(has(s, "cannot gimbal"));
  s = p.spec;
  s.engines[0].dir = sim::V3{};
  CHECK(has(s, ".dir must not be the zero vector"));
  s = p.spec;
  s.engines[0].full_cmd_deg = 0.0;
  CHECK(has(s, "full_cmd_deg"));
  s = p.spec;
  sim::FinSpec f;
  f.area_each = 0.0;
  s.fins.push_back(f);
  CHECK(has(s, "fins[0]"));
  s = p.spec;
  s.wheels.enabled = true;
  CHECK(has(s, "wheels:"));
}

// ---- the design of the tables for a general vehicle ----


TFC_TEST(design_the_gain_schedule_places_its_sixteen_points_where_the_gains_change) {
  // A vehicle whose control effectiveness grows a hundredfold in the first 4 s and then hardly changes (a fin-steered rocket): 60 s, a sample every 0.01 s.
  std::vector<sim::NominalPoint> nominal;
  for (int k = 0; k <= 6000; ++k) {
    sim::NominalPoint n;
    n.t = k * 0.01;
    n.b_ctl = 1.0 + (99.0 * (1.0 - std::exp(-n.t / 1.5)));
    n.a_div = 0.0;
    nominal.push_back(n);
  }
  const std::vector<sim::GainPoint> g = sim::gain_schedule_adaptive(nominal, 16U, 2.0, 0.8, 0.2, 1.0e30, 1.0e-3, 0.05);
  CHECK(g.size() == 16U && g.front().t == 0.0 && near_abs(g.back().t, 60.0, 1e-9));
  for (std::size_t i = 1; i < g.size(); ++i) {
    CHECK(g[i].t > g[i - 1U].t);  // strictly increasing: the table's rule
  }
  unsigned early = 0U;
  for (const sim::GainPoint& p : g) {
    early += p.t <= 6.0 ? 1U : 0U;
  }
  CHECK(early >= 8U);  // most of the points where the gain moves
  // and the straight lines between them are within a few percent of the true gain everywhere
  double worst = 0.0;
  for (const sim::NominalPoint& n : nominal) {
    std::size_t s = 0;
    while (s + 2U < g.size() && g[s + 1U].t < n.t) {
      ++s;
    }
    const double f = (n.t - g[s].t) / (g[s + 1U].t - g[s].t);
    const double kp_true = (2.0 * 2.0) / n.b_ctl;
    worst = std::fmax(worst, std::fabs((g[s].kp + (f * (g[s + 1U].kp - g[s].kp))) - kp_true) / kp_true);
  }
  CHECK(worst < 0.12);
  // the uniform schedule of the reference design, for comparison, would spend 3 of 16 points in the first 12 s
  const std::vector<sim::GainPoint> u = sim::gain_schedule(nominal, 4.0, 2.0, 0.8, 0.2);
  CHECK(u.size() == 16U && u[3].t == 12.0);
}

TFC_TEST(design_the_effectiveness_floor_limits_the_gains_a_vehicle_with_no_authority_would_ask_for) {
  std::vector<sim::NominalPoint> nominal;
  for (int k = 0; k <= 1000; ++k) {
    sim::NominalPoint n;
    n.t = k * 0.01;
    n.b_ctl = 0.0;  // no control effectiveness at all
    n.a_div = 1.0;
    nominal.push_back(n);
  }
  const std::vector<sim::GainPoint> loose = sim::gain_schedule(nominal, 5.0, 2.0, 0.8, 0.2);
  const std::vector<sim::GainPoint> floored = sim::gain_schedule(nominal, 5.0, 2.0, 0.8, 0.2, 1.0e30, 2.0);
  CHECK(loose[0].kp > 4000.0 && loose[0].kd > 1000.0);              // designed for b = 1e-3: kp = 5 / 1e-3, kd = 3.2 / 1e-3
  CHECK(near_abs(floored[0].kp, 5.0 / 2.0, 1e-12) && near_abs(floored[0].kd, 3.2 / 2.0, 1e-12));
  CHECK(near_abs(floored[0].ki, 0.2 * floored[0].kp, 1e-12));
}

TFC_TEST(design_a_pitch_program_can_be_a_table_and_the_tables_carry_it) {
  sim::DesignConfig c;
  c.table = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 30.0}, {40.0, 30.0}, {50.0, 0.0}};
  CHECK(near_abs(sim::program_deg(c, -1.0, 0.0), 0.0, 1e-12) && near_abs(sim::program_deg(c, 15.0, 0.0), 15.0, 1e-12));  // linear between points
  CHECK(near_abs(sim::program_deg(c, 30.0, 0.5), 30.0, 1e-12) && near_abs(sim::program_deg(c, 45.0, 0.0), 15.0, 1e-12));  // the flight path angle plays no part
  CHECK(near_abs(sim::program_deg(c, 99.0, 0.0), 0.0, 1e-12));                                                           // held after the last point
  sim::Params p;
  p.spec = sim::legacy_spec(p);
  sim::Plan plan;
  plan.trajectory = c;
  plan.trajectory.t_end = 50.0;
  plan.gains.every_s = 0.0;
  const sim::FlightTables t = sim::flight_tables(p, plan);
  CHECK(t.gains.size() >= 2U && t.gains.size() <= tfc::GainSchedule::kMaxPoints);
  CHECK(t.guidance.size(1U) == tfc::Guidance::kMaxPoints);
  CHECK(near_abs(static_cast<double>(t.guidance.at(2500U).tilt_y_deg), 30.0, 0.5));  // 25 s into the table: 30 degrees
  CHECK(near_abs(static_cast<double>(t.guidance.at(0U).tilt_y_deg), 0.0, 1e-6));
}

// ---- the paths of the general model that the first coverage run of it found no test on ----

TFC_TEST(general_the_mass_properties_of_a_stack_for_a_given_mass_scale_the_propellant_of_its_stages) {
  sim::Params p;
  p.ground_contact = false;
  for (int i = 0; i < 2; ++i) {
    sim::StageSpec st;
    st.dry_mass = 500.0;
    st.x_start = 4.0 * i;
    st.length = 4.0;
    st.radius = 0.5;
    st.x_cg_dry = (4.0 * i) + 2.0;
    st.tanks.push_back(sim::TankSpec{1000.0, (4.0 * i) + 0.5, 0.5, 1000.0, {}});
    p.spec.stages.push_back(st);
  }
  p.spec.engines.push_back(engine_at(0.0, 0.0, 0.0, 10000.0));
  const sim::Vehicle6 v(p, in_space());
  const sim::MassProps full = v.mass_props();
  CHECK(close(full.mass, 3000.0, 1e-12));
  const sim::MassProps half = v.mass_props(2000.0);  // half the propellant (1000 of 2000 kg), shared between the stages in the proportion they have
  CHECK(close(half.mass, 2000.0, 1e-12) && half.x_cg < full.x_cg + 1.0 && half.i_t < full.i_t);
  // the same as the mass properties of a state that has that propellant
  sim::State s = v.state();
  s.prop[0] = 500.0;
  s.prop[1] = 500.0;
  s.prop_set = true;
  sim::Vehicle6 w(p, in_space());
  w.set_state(s);
  CHECK(close(half.x_cg, w.mass_props().x_cg, 1e-12) && close(half.i_t, w.mass_props().i_t, 1e-12));
}

TFC_TEST(general_two_fixed_engines_share_a_direction_and_a_table_vehicle_has_a_divergence) {
  sim::EngineSpec a = engine_at(0.0, 0.5, 0.0, 10000.0);
  a.gimbal = false;
  sim::EngineSpec b = engine_at(0.0, -0.5, 0.0, 10000.0);
  b.gimbal = false;
  const sim::Params p = one_stage(1000.0, {a, b});
  const sim::Vehicle6 v(p, in_space());
  const sim::Loads l = v.current_loads();
  CHECK(close(l.f_thrust.x, 20000.0, 1e-12) && near_abs(l.f_thrust.y, 0.0, 1e-9) && near_abs(sim::norm(l.m_thrust), 0.0, 1e-6));  // two fixed engines straight back: no gimbal moves them
  // the reference model's own summary, and a table vehicle's divergence q S C_N-alpha (x_cp - x_cg) / I
  const sim::Vehicle6 ref;
  double cn = 0.0;
  double xcp = 0.0;
  double ca = 0.0;
  double s_ref = 0.0;
  ref.aero_summary(1.1, cn, xcp, ca, s_ref);
  CHECK(close(cn, 2.5, 1e-12) && close(xcp, 12.5, 1e-12) && close(ca, 0.75, 1e-12) && close(s_ref, sim::kPi * 0.25 * 1.8 * 1.8, 1e-12));
  sim::Params tp;
  tp.ground_contact = false;
  tp.gravity_scale = 0.0;
  tp.spec = sim::legacy_spec(tp);
  tp.spec.aero.table = {{0.0, 0.30, 4.0, 10.0}, {3.0, 0.50, 4.0, 10.0}};
  sim::Scenario sc;
  sc.wind_scale = 0.0;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 5000.0, 0.0, 0.0};
  sc.initial.v = sim::V3{300.0, 0.0, 0.0};
  const sim::Vehicle6 tv(tp, sc);
  const sim::Loads tl = tv.current_loads();
  const sim::MassProps mp = tv.mass_props();
  CHECK(close(tv.divergence(), tl.dynamic_pressure * sim::kPi * 0.25 * 1.8 * 1.8 * 4.0 * (10.0 - mp.x_cg) / mp.i_t, 1e-12));
}

TFC_TEST(effectors_a_wheel_saturates_in_yaw_too_and_a_vehicle_with_no_stage_left_still_answers) {
  sim::Params p = bare_body();
  p.ideal_roll_control = false;
  p.spec.wheels.enabled = true;
  p.spec.wheels.stage = 0;
  p.spec.wheels.torque_max = 4.0;
  p.spec.wheels.momentum_max = 2.0;
  p.spec.wheels.full_cmd_deg = 1.0;
  sim::Vehicle6 v(p, in_space());
  for (int k = 0; k < 300; ++k) {
    v.step(0.01, 0.0, -1.0);  // a negative yaw command: the wheel fills the other way
  }
  CHECK(near_abs(v.state().wheel_h.y, 2.0, 1e-9));
  const double w = v.state().w.y;
  v.step(0.5, 0.0, -1.0);
  CHECK(near_abs(v.state().w.y, w, 1e-9));  // full: no more torque in that direction
  // the design of nothing is nothing, and a vehicle whose only stage separated still reports a gimbal
  CHECK(sim::gain_schedule_adaptive({}, 16U, 2.0, 0.8, 0.2, 1.0e30, 1.0e-3, 0.1).empty());
  sim::Params q = bare_body();
  q.spec.stages[0].separate_time_s = 0.5;
  sim::Vehicle6 gone(q, in_space());
  while (gone.time() < 1.0) {
    gone.step(0.01, 1.0, 1.0);
  }
  CHECK(!gone.stage_active(0) && gone.gimbal_pitch_deg() >= 0.0 && gone.engine_count() == 0);
}
