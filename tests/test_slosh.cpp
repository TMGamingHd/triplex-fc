// SPDX-License-Identifier: MIT
// Propellant slosh (docs/design/DYNAMICS.md): the first sloshing mode of a liquid in a cylinder as a mass on a spring and a damper, taken out of the rigid body and put back as a force.
// Checked against figures worked by hand from the formulas, the conservation of what the rigid body and the sloshing mass add up to, the frequency and the damping of a free oscillation against the
// reduced mass of two bodies on a spring, the conservation of the lateral momentum, and the force and the torque on the vehicle of a mass that has moved. With slosh off (the default) nothing changes.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "slosh.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

constexpr double kPiLocal = 3.14159265358979323846;

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A cylinder of liquid of radius r and depth h (density 1000 kg/m^3) standing on x_bottom, sloshing.
sim::TankSpec liquid(double r, double h, double x_bottom) {
  sim::TankSpec t;
  t.radius = r;
  t.density = 1000.0;
  t.propellant = 1000.0 * kPiLocal * r * r * h;
  t.x_bottom = x_bottom;
  t.slosh.enabled = true;
  return t;
}

// A heavy vehicle, far stiffer in roll and pitch than any liquid could turn, so that a sloshing mass acts on a body that only translates: dry mass `dry`, one tank of `liquid_kg` that sloshes,
// an engine pushing along the axis with `g_axial` times the vehicle's weight (so that the sloshing mass has a spring), burning no propellant to speak of, in vacuum and out of gravity.
sim::Params heavy(double dry, double liquid_kg, double g_axial, double damping) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  sim::StageSpec st;
  st.name = "heavy";
  st.dry_mass = dry;
  st.length = 4.0;
  st.radius = 0.6;
  st.x_cg_dry = 2.0;
  st.inertia_factor = 1.0e3;
  const double r = 0.5;
  sim::TankSpec t;
  t.radius = r;
  t.density = 1000.0;
  t.propellant = liquid_kg;
  t.x_bottom = 0.5;
  t.slosh.enabled = true;
  t.slosh.damping = damping;
  st.tanks.push_back(t);
  p.spec.stages.push_back(st);
  sim::EngineSpec e;
  e.thrust_vac = g_axial * (dry + liquid_kg) * 9.80665;
  e.isp_vac = 1.0e9;
  e.exit_area = 0.0;
  p.spec.engines.push_back(e);
  return p;
}

sim::Scenario at_rest_in_space() {
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  return sc;
}

}  // namespace

TFC_TEST(slosh_the_model_gives_the_mass_the_frequency_and_the_place_the_formulas_say) {
  // R = 1 m, depth 1 m (a = 1.84): worked by hand
  {
    const sim::TankSpec t = liquid(1.0, 1.0, 0.0);
    const sim::SloshModel m = sim::slosh_model(t, t.propellant);
    CHECK(close(m.m1 / t.propellant, 0.43217961, 1e-6));  // (R / 2.2 h) tanh(1.84 h / R)
    CHECK(close(m.w2g, 1.74946306, 1e-6));               // 1.84 tanh(a) / R: omega^2 = 1.7495 g, 0.659 Hz in 1 g
    CHECK(near_abs(m.x_s, 0.41467918, 1e-6));            // the bob: the pivot 0.98628 (0.01372 under the surface) less the pendulum length 0.57160
    CHECK(near_abs(m.h, 1.0, 1e-12));
  }
  // a deep thin tank (R = 0.5, depth 3): tanh is 1, the mass is R / 2.2 h of the liquid
  {
    const sim::TankSpec t = liquid(0.5, 3.0, 0.0);
    const sim::SloshModel m = sim::slosh_model(t, t.propellant);
    CHECK(close(m.m1 / t.propellant, 0.07575758, 1e-6) && close(m.w2g, 3.68, 1e-6) && near_abs(m.x_s, 2.72826087, 1e-6));
  }
  // a shallow wide one (R = 1.5, depth 0.3): the pendulum is longer than the liquid is deep, and the mass is kept inside the liquid, on its floor
  {
    const sim::TankSpec t = liquid(1.5, 0.3, 2.0);
    const sim::SloshModel m = sim::slosh_model(t, t.propellant);
    CHECK(close(m.m1 / t.propellant, 0.80054794, 1e-6) && close(m.w2g, 0.43208241, 1e-6) && m.x_s == 2.0);
    // and a liquid bottom further up shifts the place by the same
    const sim::TankSpec u = liquid(1.0, 1.0, 5.0);
    CHECK(near_abs(sim::slosh_model(u, u.propellant).x_s, 5.41467918, 1e-6));
  }
  // the dispersions, and the limit on the mass
  {
    sim::TankSpec t = liquid(1.0, 1.0, 0.0);
    t.slosh.mass_scale = 1.5;
    t.slosh.frequency_scale = 2.0;
    const sim::SloshModel m = sim::slosh_model(t, t.propellant);
    CHECK(close(m.m1 / t.propellant, 1.5 * 0.43217961, 1e-6) && close(m.w2g, 4.0 * 1.74946306, 1e-6));
    sim::TankSpec s = liquid(1.5, 0.3, 0.0);
    s.slosh.mass_scale = 2.0;  // 1.6 of the liquid: not possible
    CHECK(close(sim::slosh_model(s, s.propellant).m1 / s.propellant, 0.95, 1e-12));
  }
  // nothing to slosh: no liquid, or a tank that does not
  {
    sim::TankSpec t = liquid(1.0, 1.0, 0.0);
    CHECK(sim::slosh_model(t, 0.0).m1 == 0.0);
    t.slosh.enabled = false;
    CHECK(sim::slosh_model(t, t.propellant).m1 == 0.0);
  }
}

TFC_TEST(slosh_the_rigid_body_and_the_sloshing_mass_add_up_to_the_liquid_they_replace) {
  sim::Params p = heavy(1000.0, 2000.0, 1.0, 0.02);
  sim::Params off = p;
  off.spec.stages[0].tanks[0].slosh.enabled = false;
  const sim::Vehicle6 v(p);
  const sim::Vehicle6 w(off);
  const sim::MassProps whole = v.mass_props();  // the designer's view: all of it rigid
  const sim::MassProps plain = w.mass_props();
  CHECK(whole.mass == plain.mass && whole.x_cg == plain.x_cg && whole.i_t == plain.i_t && whole.i_x == plain.i_x && whole.slosh_mass == 0.0);
  // (the designer's view for a mass that is asked for: the same, whatever the mass)
  const sim::MassProps asked = v.mass_props(2500.0);
  const sim::MassProps asked_plain = w.mass_props(2500.0);
  CHECK(asked.x_cg == asked_plain.x_cg && asked.i_t == asked_plain.i_t && asked.slosh_mass == 0.0 && asked.mass == 2500.0);
  const sim::MassProps rigid = v.rigid_mass_props();
  const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], 2000.0);
  CHECK(close(rigid.slosh_mass, m.m1, 1e-12) && rigid.slosh_mass > 100.0);
  CHECK(close(rigid.mass + rigid.slosh_mass, whole.mass, 1e-12));
  const double x_all = ((rigid.mass * rigid.x_cg) + (m.m1 * m.x_s)) / whole.mass;
  CHECK(near_abs(x_all, whole.x_cg, 1e-9));  // the centre of mass is where it was
  const double i_all = rigid.i_t + (rigid.mass * (rigid.x_cg - x_all) * (rigid.x_cg - x_all)) + (m.m1 * (m.x_s - x_all) * (m.x_s - x_all));
  CHECK(close(i_all, whole.i_t, 1e-9));  // and the inertia about it
  CHECK(rigid.x_cg != whole.x_cg);       // while the rigid part's own centre has moved
  CHECK(rigid.i_x == whole.i_x);
  // the tanks of a stage that drains in turn, and ones that drain together: the liquid in each is what the stage's propellant says
  for (int mode = 0; mode < 2; ++mode) {
    sim::Params q;
    q.ground_contact = false;
    q.gravity_scale = 0.0;
    q.spec.planet.atmosphere = sim::AtmosphereKind::None;
    sim::StageSpec st;
    st.name = "two tanks";
    st.dry_mass = 500.0;
    st.length = 8.0;
    st.radius = 0.6;
    st.sequential_drain = mode == 0;
    st.tanks.push_back(liquid(0.5, 1.2, 0.5));
    st.tanks.push_back(liquid(0.5, 1.8, 4.0));  // one and a half times the first
    q.spec.stages.push_back(st);
    sim::Scenario sc = at_rest_in_space();
    const double cap = st.tanks[0].propellant + st.tanks[1].propellant;
    sim::Vehicle6 probe(q);
    sc.initial.prop = probe.state().prop;
    sc.initial.prop[0] = 0.75 * cap;
    sc.initial.m = 500.0 + (0.75 * cap);
    sc.initial.prop_set = true;
    const sim::Vehicle6 u(q, sc);
    // sequential: the tank listed last holds its full share first (1.5 c), so the first holds what is left of the three quarters (0.75 (c + 1.5 c) - 1.5 c = 0.375 c); together: three quarters of each
    const double first = mode == 0 ? (0.75 * cap) - st.tanks[1].propellant : 0.75 * st.tanks[0].propellant;
    const double second = mode == 0 ? st.tanks[1].propellant : 0.75 * st.tanks[1].propellant;
    const double expected = sim::slosh_model(st.tanks[0], first).m1 + sim::slosh_model(st.tanks[1], second).m1;
    CHECK(close(u.rigid_mass_props().slosh_mass, expected, 1e-12) && expected > 100.0);
    // the forces use the same division of the liquid: each tank's sloshing mass is its own, and they add up to what the mass properties took out
    const sim::Loads l = u.current_loads();
    CHECK(close(l.pod[0].m1, sim::slosh_model(st.tanks[0], first).m1, 1e-12) && close(l.pod[1].m1, sim::slosh_model(st.tanks[1], second).m1, 1e-12));
    CHECK(close(l.pod[0].m1 + l.pod[1].m1, u.rigid_mass_props().slosh_mass, 1e-12));
  }
}

TFC_TEST(slosh_a_free_oscillation_has_the_frequency_of_two_bodies_on_a_spring_and_the_damping_given) {
  const double dry = 100000.0;
  const double liquid_kg = 1000.0;
  for (const double zeta : {0.0, 0.05}) {
    sim::Params p = heavy(dry, liquid_kg, 1.0, zeta);
    sim::Scenario sc = at_rest_in_space();
    sc.initial.m = dry + liquid_kg;
    sc.initial.slosh[0] = {0.05, 0.0, 0.0, 0.0};  // displaced 5 cm along Y, at rest
    sim::Vehicle6 v(p, sc);
    const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], liquid_kg);
    const double g_axial = 1.0 * 9.80665;  // the engine's thrust over the vehicle's mass
    const double w0 = std::sqrt(g_axial * m.w2g);
    const double mu = m.m1 / (dry + liquid_kg - m.m1);                   // m1 / M_r
    const double wc = w0 * std::sqrt(1.0 + mu);                          // two bodies on one spring: the reduced mass
    const double zc = zeta * (w0 / wc) * (1.0 + mu);                     // the damping ratio of the relative motion
    const double wd = wc * std::sqrt(1.0 - (zc * zc));
    std::vector<double> crossings;
    std::vector<double> descending;
    std::vector<double> peaks;
    double last = v.state().slosh[0][0];
    double last_t = 0.0;
    for (int k = 1; k <= 20000; ++k) {  // 20 s
      v.step(0.001, 0.0, 0.0);
      const double x = v.state().slosh[0][0];
      if ((last > 0.0) != (x > 0.0)) {
        const double tc = last_t + (0.001 * last / (last - x));
        crossings.push_back(tc);
        if (x < 0.0) {
          descending.push_back(tc);
          peaks.push_back(0.0);
        }
      }
      if (!peaks.empty()) {
        peaks.back() = std::fmax(peaks.back(), std::fabs(x));  // the largest |x| in the cycle that began at the last descending crossing
      }
      last = x;
      last_t = 0.001 * k;
    }
    CHECK(crossings.size() >= 10U);
    const double half_period = (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1U);
    CHECK(close(2.0 * half_period, 2.0 * kPiLocal / wd, 2e-3));
    if (zeta > 0.0) {
      // the amplitude falls by exp(-zc wc t) from the first cycle to the last whole one
      CHECK(peaks.size() >= 4U);
      const std::size_t j = peaks.size() - 2U;
      CHECK(close(std::log(peaks.front() / peaks[j]) / (descending[j] - descending.front()), zc * wc, 0.05));
    }
  }
}

// A vehicle free to turn, with no thrust: the sloshing mass is a body on a spring that pushes the vehicle at an arm from its centre, so that the relative motion is exactly harmonic at
// omega^2 = k (1/m1 + 1/M_r + a^2/I) (the vehicle's mass, and its inertia at the arm a). It is compared at 20, 60 and 100 s: a small amplitude, so that the second-order terms the model leaves out
// (the vehicle's own turning against the motion along the tank) are below a part in ten million at this amplitude.
TFC_TEST(slosh_a_vehicle_that_can_turn_is_a_free_oscillation_at_the_frequency_that_includes_its_inertia) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.max_substep = 0.1;
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  sim::StageSpec st;
  st.name = "free";
  st.dry_mass = 1000.0;
  st.length = 6.0;
  st.radius = 0.6;
  st.x_cg_dry = 0.5;  // the structure's centre aft, the liquid forward: the sloshing mass pushes at a long arm
  st.inertia_factor = 0.2;
  sim::TankSpec t = liquid(0.5, 2.0, 3.0);
  t.slosh.damping = 0.0;
  st.tanks.push_back(t);
  p.spec.stages.push_back(st);
  sim::Scenario sc = at_rest_in_space();
  sc.initial.m = 1000.0 + t.propellant;
  const double x0 = 0.001;
  sc.initial.slosh[0] = {x0, 0.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  const sim::SloshModel m = sim::slosh_model(t, t.propellant);
  const sim::MassProps rigid = v.rigid_mass_props();
  const double arm = m.x_s - rigid.x_cg;
  const double k = m.m1 * sim::kSloshMinG * m.w2g;
  const double wc = std::sqrt(k * ((1.0 / m.m1) + (1.0 / rigid.mass) + (arm * arm / rigid.i_t)));
  const double w_stiff = std::sqrt(k * ((1.0 / m.m1) + (1.0 / rigid.mass)));  // what it would be if the vehicle could not turn
  CHECK(std::fabs(wc - w_stiff) > 0.04 * wc);  // the turning matters here: this is what the test is about
  double t_now = 0.0;
  for (const double stop : {20.0, 60.0, 100.0}) {
    while (t_now < stop - 1e-9) {
      v.step(0.1, 0.0, 0.0);
      t_now += 0.1;
    }
    CHECK(near_abs(v.state().slosh[0][0], x0 * std::cos(wc * t_now), 1e-5 * x0));
  }
}

// With the vehicle turning about two axes at once, the point the sloshing mass hangs at has an acceleration along Z from the turning, a_z = a omega_x omega_z I_x / I_t at a distance a
// along the axis (the angular acceleration of the body's own gyroscopic coupling, and the centripetal term): the rate of the mass along Z a millisecond later is that.
TFC_TEST(slosh_the_point_the_mass_hangs_at_accelerates_with_the_turning_of_the_vehicle) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.ideal_roll_control = false;
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  sim::StageSpec st;
  st.name = "turning";
  st.dry_mass = 1000.0;
  st.length = 4.0;
  st.radius = 0.6;
  st.x_cg_dry = 2.0;
  st.tanks.push_back(liquid(0.5, 2.0, 0.5));
  p.spec.stages.push_back(st);
  sim::Scenario sc = at_rest_in_space();
  sc.initial.m = 1000.0 + st.tanks[0].propellant;
  sc.initial.w = sim::V3{1.0, 0.0, 1.0};
  sim::Vehicle6 v(p, sc);
  const sim::SloshModel m = sim::slosh_model(st.tanks[0], st.tanks[0].propellant);
  const sim::MassProps rigid = v.rigid_mass_props();
  const double arm = m.x_s - rigid.x_cg;
  v.step(0.001, 0.0, 0.0);
  const double expected = -arm * 1.0 * 1.0 * rigid.i_x / rigid.i_t * 0.001;
  CHECK(close(v.state().slosh[0][3], expected, 0.01) && std::fabs(expected) > 1e-6);
  CHECK(std::fabs(v.state().slosh[0][2]) < 1e-5 * std::fabs(expected) * 1e3);  // and next to nothing along Y (what the turning builds up in the millisecond)
}

// Along the axis the sloshing mass rides on the tank, pressed on it by the liquid below: a vehicle whose liquid sloshes accelerates under its thrust exactly as one whose liquid does not.
TFC_TEST(slosh_the_axial_acceleration_is_the_thrust_over_the_whole_mass_as_it_is_without_slosh) {
  for (const bool sloshing : {true, false}) {
    sim::Params p = heavy(1000.0, 2000.0, 1.0, 0.02);
    p.spec.stages[0].tanks[0].slosh.enabled = sloshing;
    sim::Scenario sc = at_rest_in_space();
    sc.initial.m = 3000.0;
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 2000; ++k) {  // 2 s
      v.step(0.001, 0.0, 0.0);
    }
    // 1 g for 2 s, less the (negligible) propellant it burnt
    CHECK(close(v.state().v.x, 9.80665 * 2.0, 1e-4));
    CHECK(near_abs(v.state().v.y, 0.0, 1e-12) && near_abs(v.state().v.z, 0.0, 1e-12));
    CHECK(sloshing || v.rigid_mass_props().slosh_mass == 0.0);
  }
}

TFC_TEST(slosh_the_total_lateral_momentum_is_conserved_and_the_vehicle_recoils) {
  sim::Params p = heavy(50000.0, 600.0, 0.0, 0.0);  // no thrust: the spring is the floored one
  p.spec.engines[0].thrust_vac = 1.0;
  sim::Scenario sc = at_rest_in_space();
  sc.initial.m = 50600.0;
  sc.initial.slosh[0] = {0.1, 0.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], 600.0);
  const double w = std::sqrt(sim::kSloshMinG * m.w2g);
  double widest = 0.0;
  for (int k = 0; k < 4000; ++k) {  // 4 s of a 14.6 s period: just past the quarter
    v.step(0.001, 0.0, 0.0);
    const double mr = v.state().m - m.m1;
    // the rigid body at its centre, the sloshing mass at its own velocity: the vehicle's, its own along the tank and the turning of the point it hangs at
    const double momentum = (mr * v.state().v.y) + (m.m1 * (v.state().v.y + v.state().slosh[0][2] + (v.state().w.z * (m.x_s - v.rigid_mass_props().x_cg))));
    widest = std::fmax(widest, std::fabs(momentum));
  }
  CHECK(widest < 1e-6 * m.m1 * w * 0.1);  // against the scale of what moves: m1 w x0
  CHECK(v.state().v.y > 1e-5);            // the spring pulled the sloshing mass back toward -Y and pushed the vehicle toward +Y
  CHECK(v.state().slosh[0][0] < 0.1 && v.state().slosh[0][2] < 0.0);
}

TFC_TEST(slosh_a_displaced_mass_pushes_the_vehicle_with_the_springs_force_and_turns_it_by_that_force_times_its_arm) {
  for (int plane = 0; plane < 2; ++plane) {
    sim::Params p = heavy(1000.0, 2000.0, 1.0, 0.0);
    p.spec.stages[0].inertia_factor = 0.6;  // a vehicle that can turn
    sim::Scenario sc = at_rest_in_space();
    sc.initial.m = 3000.0;
    sc.initial.slosh[0][static_cast<std::size_t>(plane)] = 0.02;
    sim::Vehicle6 v(p, sc);
    const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], 2000.0);
    const sim::MassProps rigid = v.rigid_mass_props();
    const double w2 = 9.80665 * m.w2g;
    const double force = m.m1 * w2 * 0.02;  // the spring's, on the vehicle, toward the displacement
    const double arm = m.x_s - rigid.x_cg;
    CHECK(sim::slosh_model(p.spec.stages[0].tanks[0], 2000.0).m1 > 100.0 && std::fabs(arm) > 0.1);
    const double t = 0.004;
    for (int k = 0; k < 4; ++k) {
      v.step(0.001, 0.0, 0.0);
    }
    const double mr = 3000.0 - rigid.slosh_mass;
    // the lateral velocity: F t / M_r (the vehicle is also pushed axially, which is not what is looked at here)
    const double v_lat = plane == 0 ? v.state().v.y : v.state().v.z;
    CHECK(close(v_lat, force * t / mr, 0.01));
    // and the turn: M = r x F: about Z for a force along Y (+arm F), about Y for a force along Z (-arm F)
    const double w_turn = plane == 0 ? v.state().w.z : v.state().w.y;
    const double expected = (plane == 0 ? 1.0 : -1.0) * arm * force * t / rigid.i_t;
    CHECK(close(w_turn, expected, 0.01));
    CHECK(std::fabs(plane == 0 ? v.state().v.z : v.state().v.y) < 1e-9);  // the other lateral axis is untouched
  }
}

TFC_TEST(slosh_the_force_is_the_spring_plus_the_damper_on_each_plane_and_the_accelerometer_feels_it) {
  sim::Params p = heavy(1000.0, 2000.0, 1.0, 0.1);
  const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], 2000.0);
  const double w2 = 9.80665 * m.w2g;
  const double c = 2.0 * 0.1 * m.m1 * std::sqrt(w2);
  const struct {
    std::array<double, 4> x;
    double fy;
    double fz;
  } cases[] = {{{0.03, 0.0, 0.0, 0.0}, m.m1 * w2 * 0.03, 0.0},
               {{0.0, 0.04, 0.0, 0.0}, 0.0, m.m1 * w2 * 0.04},
               {{0.0, 0.0, 0.1, 0.0}, c * 0.1, 0.0},
               {{0.0, 0.0, 0.0, 0.2}, 0.0, c * 0.2},
               {{0.03, -0.04, -0.1, 0.2}, (m.m1 * w2 * 0.03) - (c * 0.1), (-m.m1 * w2 * 0.04) + (c * 0.2)}};
  for (const auto& cs : cases) {
    sim::Scenario sc = at_rest_in_space();
    sc.initial.m = 3000.0;
    sc.initial.slosh[0] = cs.x;
    const sim::Vehicle6 v(p, sc);
    const sim::Loads l = v.current_loads();
    CHECK(near_abs(l.f_slosh.y, cs.fy, 1e-9 * (1.0 + std::fabs(cs.fy))) && near_abs(l.f_slosh.z, cs.fz, 1e-9 * (1.0 + std::fabs(cs.fz))) && l.f_slosh.x == 0.0);
    // the accelerometers of a vehicle's own IMU: sideways the force on the rigid body over its mass, along the axis the thrust over the whole mass; sensor X = body Y, Z = body X
    sim::V3 gyro;
    sim::V3 accel;
    v.vehicle_true_imu(gyro, accel);
    const double mr = 3000.0 - v.rigid_mass_props().slosh_mass;
    CHECK(close(accel.z, (l.f_thrust.x + l.f_aero.x) / 3000.0 / 9.80665, 1e-12) && near_abs(accel.x, cs.fy / mr / 9.80665, 1e-12) && near_abs(accel.y, cs.fz / mr / 9.80665, 1e-12));  // (along the axis over the whole mass: the sloshing mass rides on the tank)
  }
}

TFC_TEST(slosh_a_stage_that_leaves_takes_its_sloshing_with_it_and_a_vehicle_without_slosh_is_not_touched) {
  sim::Params p = heavy(1000.0, 1000.0, 1.0, 0.02);
  p.spec.stages[0].separate_time_s = 0.5;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 300.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  p.spec.stages.push_back(up);
  sim::Scenario sc = at_rest_in_space();
  sc.initial.slosh[0] = {0.03, 0.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  CHECK(v.current_loads().f_slosh.y != 0.0);
  for (int k = 0; k < 600; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(!v.stage_active(0U));
  CHECK(v.current_loads().f_slosh.y == 0.0 && v.current_loads().f_slosh.z == 0.0 && v.rigid_mass_props().slosh_mass == 0.0);
  // a vehicle that does not slosh has no sloshing state at all
  sim::Params q = heavy(1000.0, 1000.0, 1.0, 0.02);
  q.spec.stages[0].tanks[0].slosh.enabled = false;
  sim::Vehicle6 w(q, at_rest_in_space());
  for (int k = 0; k < 100; ++k) {
    w.step(0.001, 0.0, 0.0);
  }
  CHECK(w.state().slosh[0][0] == 0.0 && w.state().slosh[0][3] == 0.0 && w.current_loads().f_slosh.y == 0.0);
}

TFC_TEST(slosh_the_spring_needs_an_axial_acceleration_and_has_a_floor) {
  // no thrust: the spring is the floored one, omega^2 = 0.05 w2g
  sim::Params p = heavy(10000.0, 500.0, 1.0, 0.0);
  p.spec.engines[0].thrust_vac = 1.0;
  sim::Scenario sc = at_rest_in_space();
  sc.initial.m = 10500.0;
  sc.initial.slosh[0] = {0.01, 0.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  const sim::SloshModel m = sim::slosh_model(p.spec.stages[0].tanks[0], 500.0);
  const sim::Loads l = v.current_loads();
  CHECK(close(l.pod[0].k, m.m1 * sim::kSloshMinG * m.w2g, 1e-6) && close(l.pod[0].m1, m.m1, 1e-12) && close(l.pod[0].x_s, m.x_s, 1e-12));
  // with 2 g it is 2 g's worth, and the damper is 2 zeta m1 omega
  sim::Params q = heavy(10000.0, 500.0, 2.0, 0.1);
  sim::Vehicle6 u(q, sc);
  const sim::Loads lu = u.current_loads();
  CHECK(close(lu.pod[0].k, m.m1 * 2.0 * 9.80665 * m.w2g, 1e-4) && close(lu.pod[0].c, 2.0 * 0.1 * m.m1 * std::sqrt(2.0 * 9.80665 * m.w2g), 1e-4));
}

TFC_TEST(slosh_files_read_a_tank_that_sloshes_write_it_back_and_refuse_what_cannot_be) {
  const std::string text = R"({
    "stages": [{"dry_mass_kg": 500, "length_m": 6, "radius_m": 0.5,
                "tanks": [{"propellant_kg": 600, "x_bottom_m": 1, "radius_m": 0.4, "density_kg_m3": 800, "slosh": {"damping": 0.05, "mass_scale": 1.2, "frequency_scale": 0.9}},
                          {"propellant_kg": 300, "x_bottom_m": 3.5, "radius_m": 0.4, "density_kg_m3": 800}]}]
  })";
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const std::vector<sim::TankSpec>& tanks = v.params.spec.stages[0].tanks;
  CHECK(tanks[0].slosh.enabled && tanks[0].slosh.damping == 0.05 && tanks[0].slosh.mass_scale == 1.2 && tanks[0].slosh.frequency_scale == 0.9 && !tanks[1].slosh.enabled);
  const std::string once = sim::write_vehicle(v);
  sim::VehicleFile w;
  CHECK(sim::read_vehicle(once, w, errors) && errors.empty() && sim::write_vehicle(w) == once);
  CHECK(w.params.spec.stages[0].tanks[0].slosh.mass_scale == 1.2 && !w.params.spec.stages[0].tanks[1].slosh.enabled);
  // an object with nothing in it turns it on with the defaults
  sim::VehicleFile d;
  CHECK(sim::read_vehicle(R"({"stages": [{"dry_mass_kg": 5, "length_m": 2, "radius_m": 0.2, "tanks": [{"propellant_kg": 10, "radius_m": 0.2, "slosh": {}}]}]})", d, errors) &&
        d.params.spec.stages[0].tanks[0].slosh.enabled && d.params.spec.stages[0].tanks[0].slosh.damping == 0.02);
  const auto refused = [](const std::string& tank, const std::string& what) {
    sim::VehicleFile f;
    std::vector<std::string> errs;
    const std::string t = R"({"stages": [{"dry_mass_kg": 5, "length_m": 2, "radius_m": 0.2, "tanks": [)" + tank + "]}]}";
    if (sim::read_vehicle(t, f, errs) && errs.empty()) {
      for (const std::string& p : sim::validate(f.params.spec)) {
        if (p.find(what) != std::string::npos) {
          return true;
        }
      }
      return false;
    }
    for (const std::string& e : errs) {
      if (e.find(what) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  CHECK(refused(R"({"propellant_kg": 10, "radius_m": 0.2, "slosh": {"damping": -1}})", "damping must not be negative"));
  CHECK(refused(R"({"propellant_kg": 10, "radius_m": 0.2, "slosh": {"mass_scale": 0}})", "mass_scale and frequency_scale must be positive"));
  CHECK(refused(R"({"propellant_kg": 10, "radius_m": 0.2, "slosh": {"frequency_scale": -2}})", "mass_scale and frequency_scale must be positive"));
  CHECK(refused(R"({"propellant_kg": 0, "radius_m": 0.2, "slosh": {}})", "nothing to slosh"));
  CHECK(refused(R"({"propellant_kg": 10, "radius_m": 0.2, "slosh": {"dampng": 0.1}})", "damping"));  // a typo is refused with a suggestion
  // more tanks than the vehicle can track (10, in two stages of 5)
  sim::VehicleSpec big;
  for (int st = 0; st < 2; ++st) {
    sim::StageSpec stage;
    stage.dry_mass = 10.0;
    stage.length = 2.0;
    for (int k = 0; k < 5; ++k) {
      sim::TankSpec t;
      t.propellant = 1.0;
      t.radius = 0.1;
      t.slosh.enabled = true;
      stage.tanks.push_back(t);
    }
    big.stages.push_back(stage);
  }
  bool named = false;
  for (const std::string& p : sim::validate(big)) {
    named = named || p.find("at most 8 tanks can slosh") != std::string::npos;
  }
  CHECK(named);
  big.stages[1].tanks.resize(3U);  // 8: allowed
  CHECK(sim::validate(big).empty());
}
