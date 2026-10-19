// SPDX-License-Identifier: MIT
// The cluster of engines as a plant (docs/design/PROPULSION.md): each engine has a gimbal of its own (VehicleSpec::per_engine_gimbal), the pitch and yaw command goes to every engine, the roll command
// is added along each engine's tangent (the differential gimbal), a weak engine unbalances the cluster and the gimbal trims it, an engine's gimbal can stick or run hard over, and the specific
// impulse falls with the throttle. Every number below is a hand calculation of the geometry (a force of thrust at an angle, at an arm from the centre of gravity), not the plant's own formula.
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "propulsion.hpp"
#include "spec.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

constexpr double kThrust = 1.0e6;   // N per engine
constexpr double kDeg = 3.14159265358979323846 / 180.0;

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }

// A stage the computers drive, in vacuum and without gravity, with `n` engines on a circle of radius `r` at the aft end, all in group 0. The gimbal is quick (the servo is not the subject).
sim::Params cluster(unsigned n, double r, bool per_engine) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.ideal_roll_control = false;   // (the roll is free: the plant holds none unless it is asked to)
  p.gimbal_limit_deg = 8.0;
  p.gimbal_rate_dps = 10000.0;
  p.spec.per_engine_gimbal = per_engine;
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = 20000.0;
  st.length = 30.0;
  st.radius = 2.0;
  st.x_cg_dry = 12.0;
  st.guided = true;
  sim::TankSpec tk;
  tk.propellant = 80000.0;
  tk.x_bottom = 1.0;
  tk.radius = 1.8;
  tk.density = 1000.0;
  st.tanks.push_back(tk);
  p.spec.stages.push_back(st);
  for (unsigned i = 0; i < n; ++i) {
    const double a = 2.0 * 3.14159265358979323846 * i / n;
    sim::EngineSpec e;
    e.pos = sim::V3{0.0, r * std::cos(a), r * std::sin(a)};
    e.thrust_vac = kThrust;
    e.exit_area = 0.0;
    e.isp_vac = 330.0;
    e.group = 0;
    p.spec.engines.push_back(e);
  }
  return p;
}

std::string repo_root() {
#ifdef TFC_SOURCE_DIR
  return std::string(TFC_SOURCE_DIR) + "/";
#else
  const std::string f = __FILE__;
  const std::size_t pos = f.rfind("tests/");
  return pos == std::string::npos ? std::string() : f.substr(0, pos);
#endif
}

sim::Scenario in_space() {
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 300000.0, 0.0, 0.0};
  return sc;
}

sim::Controls burn(double pitch = 0.0, double yaw = 0.0, double roll = 0.0) {
  sim::Controls c;
  c.group_mask = 1U;
  c.throttle = 1.0;
  c.pitch_deg = pitch;
  c.yaw_deg = yaw;
  c.roll_deg = roll;
  return c;
}

// The torque and force of the plant after a short burn with the controls.
struct Out {
  sim::V3 m;
  sim::V3 f;
  double xcg;
};
Out fire(const sim::Params& p, const sim::Controls& c, double seconds = 0.2) {
  sim::Vehicle6 v(p, in_space());
  for (int k = 0; k < static_cast<int>(seconds / 0.01); ++k) {
    v.step(0.01, c);
  }
  const sim::Loads l = v.current_loads();
  return Out{l.m_thrust, l.f_thrust, v.mass_props().x_cg};
}

}  // namespace

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_with_the_same_angle_for_every_engine_the_cluster_is_the_stage_gimbal_of_before) {
  const sim::Params shared = cluster(4, 1.5, false);
  const sim::Params each = cluster(4, 1.5, true);
  for (const double pitch : {0.0, 3.0, -5.0}) {
    const Out a = fire(shared, burn(pitch, 1.0));
    const Out b = fire(each, burn(pitch, 1.0));
    CHECK(near_abs(a.m.x, b.m.x, 1.0e-6) && near_abs(a.m.y, b.m.y, 1.0e-6) && near_abs(a.m.z, b.m.z, 1.0e-6));
    CHECK(near_abs(a.f.x, b.f.x, 1.0e-6) && near_abs(a.f.y, b.f.y, 1.0e-6) && near_abs(a.f.z, b.f.z, 1.0e-6));
  }
}

TFC_TEST(propulsion_a_gimbal_angle_turns_the_cluster_by_thrust_times_arm_and_costs_the_cosine) {
  const unsigned n = 4;
  const sim::Params p = cluster(n, 1.5, true);
  for (const double d : {1.0, 4.0, 8.0}) {
    const Out o = fire(p, burn(d));
    const double arm = o.xcg;   // the engines are at the aft end, x = 0
    // each engine pushes (cos d, -sin d, 0) times T at (0, y, z): torque about Z is r_x F_y - r_y F_x, the ring's y terms cancel
    const double expect_mz = n * kThrust * std::sin(d * kDeg) * arm;
    CHECK(near_abs(o.m.z, expect_mz, 1.0e-6 * expect_mz + 1.0e-3));
    CHECK(near_abs(o.f.x, n * kThrust * std::cos(d * kDeg), 1.0e-6 * n * kThrust));   // the cosine loss
    CHECK(near_abs(o.f.y, -static_cast<double>(n) * kThrust * std::sin(d * kDeg), 1.0e-6 * n * kThrust));
    CHECK(near_abs(o.m.x, 0.0, 1.0e-3) && near_abs(o.m.y, 0.0, 1.0e-3));
  }
}

TFC_TEST(propulsion_an_engine_that_does_not_gimbal_or_has_a_shorter_travel_gives_less_torque) {
  sim::Params p = cluster(4, 1.5, true);
  p.spec.engines[0].gimbal = false;
  p.spec.engines[1].gimbal_limit_deg = 2.0;   // a travel of 2 degrees, asked for 6
  const Out o = fire(p, burn(6.0));
  // the torque about Z of each engine: the lateral force at the arm from the centre of gravity, and the axial force at its y offset (engines on the ring at 0, 90, 180 and 270 degrees)
  const auto torque_z = [&o, &p](const std::array<double, 4>& angle_deg) {
    double mz = 0.0;
    for (std::size_t i = 0; i < 4U; ++i) {
      const double d = angle_deg[i] * kDeg;
      mz += (o.xcg * kThrust * std::sin(d)) - (p.spec.engines[i].pos.y * kThrust * std::cos(d));
    }
    return mz;
  };
  CHECK(near_abs(o.m.z, torque_z({0.0, 2.0, 6.0, 6.0}), 1.0e-5 * std::fabs(o.m.z)));   // one fixed, one at the end of its 2 degrees, two at 6
  // the same cluster with the stage's common angle cannot tell the engines apart: the fixed one stays, the others all move 6 degrees
  sim::Params q = p;
  q.spec.per_engine_gimbal = false;
  const Out s = fire(q, burn(6.0));
  CHECK(near_abs(s.m.z, torque_z({0.0, 6.0, 6.0, 6.0}), 1.0e-5 * std::fabs(s.m.z)));
  CHECK(o.m.z < s.m.z);
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_differential_gimbal_rolls_the_vehicle_by_the_tangent_deflections_and_not_the_other_axes) {
  const unsigned n = 6;
  const double r = 1.5;
  sim::Params p = cluster(n, r, true);
  for (sim::EngineSpec& e : p.spec.engines) {
    e.roll_gain = 1.0;
  }
  const double roll = 2.0;   // degrees
  const Out o = fire(p, burn(0.0, 0.0, roll));
  // every engine leans 2 degrees along its tangent: the force is T (cos, tangent sin(2)) and the torque about X is the arm r times it
  const double expect_mx = n * kThrust * r * std::sin(roll * kDeg);
  CHECK(near_abs(o.m.x, expect_mx, 5.0e-3 * expect_mx));   // (the pitch and yaw decomposition of a tangent deflection is not exactly a rotation: a few parts in a thousand)
  CHECK(near_abs(o.m.y, 0.0, 1.0e-3 * expect_mx) && near_abs(o.m.z, 0.0, 1.0e-3 * expect_mx));
  CHECK(close(o.f.x, n * kThrust * std::cos(roll * kDeg), 1.0e-3));
  // a negative roll command is the opposite torque, and with no roll gain the same command does nothing
  const Out neg = fire(p, burn(0.0, 0.0, -roll));
  CHECK(near_abs(neg.m.x, -o.m.x, 1.0e-6 * expect_mx));
  const sim::Params none = cluster(n, r, true);
  CHECK(near_abs(fire(none, burn(0.0, 0.0, roll)).m.x, 0.0, 1.0e-3));
  // and the roll is added to the pitch command: a pitch of 3 degrees and a roll of 2 gives both torques
  const Out both = fire(p, burn(3.0, 0.0, roll));
  CHECK(near_abs(both.m.z, n * kThrust * std::sin(3.0 * kDeg) * both.xcg, 2.0e-2 * both.m.z) && near_abs(both.m.x, expect_mx, 1.0e-2 * expect_mx));
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_a_weak_engine_unbalances_the_cluster_and_the_gimbal_can_trim_it_out) {
  const unsigned n = 4;
  sim::Params p = cluster(n, 1.5, true);
  p.spec.engines[0].thrust_factor = 0.9;   // the engine at +Y (angle 0) gives 90 % of its thrust
  const Out weak = fire(p, burn());
  // the missing 0.1 T at y = 1.5 is a torque about Z (the thrust is along X): tau = r x F, tau_z = -y F_x, so the lost thrust at +Y adds +y dT
  const double dt = 0.1 * kThrust;
  CHECK(near_abs(weak.m.z, 1.5 * dt, 1.0e-6 * 1.5 * dt));
  CHECK(near_abs(weak.f.x, (n - 0.1) * kThrust, 1.0e-6 * n * kThrust));
  // the pitch gimbal that cancels it: d such that sum T sin(d) arm = -tau, in the small-angle sense the plant's own sine
  const double arm = weak.xcg;
  const double trim_rad = std::asin(-weak.m.z / (((n - 0.1) * kThrust) * arm));
  const Out trimmed = fire(p, burn(trim_rad / kDeg));
  CHECK(std::fabs(trimmed.m.z) < 0.01 * std::fabs(weak.m.z));
  CHECK(trim_rad / kDeg < 0.0);   // the nose is pushed toward the weak side's opposite: a negative pitch deflection for a deficit at +Y
  // an engine with a thrust factor of zero is an engine out: its thrust is gone
  sim::Params out = cluster(n, 1.5, true);
  out.spec.engines[2].thrust_factor = 0.0;
  CHECK(close(fire(out, burn()).f.x, 3.0 * kThrust, 1.0e-9));
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_a_gimbal_that_sticks_or_runs_hard_over_acts_from_the_time_of_its_fault) {
  const auto angle_after = [](sim::GimbalFault f, double fault_pitch, double command) {
    sim::Params p = cluster(3, 1.0, true);
    p.gimbal_rate_dps = 20.0;
    p.spec.engines[0].fault = f;
    p.spec.engines[0].fault_time_s = 1.0;
    p.spec.engines[0].fault_pitch_deg = fault_pitch;
    sim::Vehicle6 v(p, in_space());
    for (int k = 0; k < 100; ++k) {
      v.step(0.01, burn(3.0));   // one second asking for 3 degrees
    }
    const double before = v.engine_gimbal_pitch_deg(0);
    for (int k = 0; k < 100; ++k) {
      v.step(0.01, burn(command));   // and then another, asking for something else
    }
    return std::array<double, 3>{before, v.engine_gimbal_pitch_deg(0), v.engine_gimbal_pitch_deg(1)};
  };
  const auto healthy = angle_after(sim::GimbalFault::None, 0.0, -4.0);
  CHECK(near_abs(healthy[0], 3.0, 1.0e-6) && near_abs(healthy[1], -4.0, 1.0e-6));            // it follows (a rate of 20 deg/s: 7 degrees take 0.35 s)
  const auto stuck = angle_after(sim::GimbalFault::Stuck, 0.0, -4.0);
  CHECK(near_abs(stuck[1], stuck[0], 1.0e-9) && near_abs(stuck[2], -4.0, 1.0e-6));          // stuck at 3 while the others went to -4
  const auto hard = angle_after(sim::GimbalFault::Hardover, 8.0, -4.0);
  CHECK(near_abs(hard[1], 8.0, 1.0e-6) && near_abs(hard[2], -4.0, 1.0e-6));                  // and driven to 8 whatever the command
  // a hard-over engine does what the flight computers cannot undo: the torque it adds is that of a lateral force at its arm
  sim::Params p = cluster(3, 1.0, true);
  p.spec.engines[0].fault = sim::GimbalFault::Hardover;
  p.spec.engines[0].fault_pitch_deg = 8.0;
  const Out o = fire(p, burn(0.0), 1.0);
  CHECK(o.m.z > 0.9 * kThrust * std::sin(8.0 * kDeg) * o.xcg);
}

TFC_TEST(propulsion_the_specific_impulse_falls_with_the_throttle_as_the_engine_says) {
  const auto mdot_at = [](double throttle, double loss) {
    sim::Params p = cluster(1, 0.0, false);
    p.spec.engines[0].isp_throttle_loss = loss;
    sim::Vehicle6 v(p, in_space());
    sim::Controls c = burn();
    c.throttle = throttle;
    v.step(0.01, c);
    return v.current_loads().mdot;
  };
  const double g0 = 9.80665;
  CHECK(close(mdot_at(1.0, 0.0), kThrust / (330.0 * g0), 1.0e-9));
  CHECK(close(mdot_at(1.0, 0.05), kThrust / (330.0 * g0), 1.0e-9));   // at full throttle nothing is lost
  CHECK(close(mdot_at(0.5, 0.0), 0.5 * kThrust / (330.0 * g0), 1.0e-9));
  CHECK(close(mdot_at(0.5, 0.06), 0.5 * kThrust / (330.0 * (1.0 - (0.06 * 0.5)) * g0), 1.0e-9));   // 3 % less specific impulse at half throttle: 3 % more propellant for the thrust
}

TFC_TEST(propulsion_the_design_probe_sees_the_differential_gimbal_as_roll_authority) {
  sim::Params with = cluster(6, 1.5, true);
  for (sim::EngineSpec& e : with.spec.engines) {
    e.roll_gain = 1.0;
  }
  const sim::Params without = cluster(6, 1.5, true);
  sim::Controls probe;
  probe.roll_deg = 1.0;
  sim::Vehicle6 a(with, in_space());
  sim::Vehicle6 b(without, in_space());
  for (int k = 0; k < 5; ++k) {
    a.step(0.01, burn());
    b.step(0.01, burn());
  }
  const sim::V3 wa = a.angular_accel_with(burn(0.0, 0.0, 1.0));
  const sim::V3 wb = b.angular_accel_with(burn(0.0, 0.0, 1.0));
  const sim::MassProps mp = a.mass_props();
  CHECK(wa.x > 0.0 && near_abs(wb.x, 0.0, 1.0e-12));
  CHECK(close(wa.x, 6.0 * kThrust * 1.5 * std::sin(kDeg) / mp.i_x, 1.0e-2));   // rad/s^2 per degree of roll command
}

TFC_TEST(propulsion_the_new_fields_are_read_written_and_read_back_as_the_same_vehicle) {
  sim::Params p = cluster(3, 1.0, true);
  p.spec.engines[0].thrust_factor = 0.95;
  p.spec.engines[0].isp_throttle_loss = 0.03;
  p.spec.engines[0].gimbal_limit_deg = 5.0;
  p.spec.engines[0].gimbal_rate_dps = 12.0;
  p.spec.engines[0].gimbal_lag_s = 0.05;
  p.spec.engines[0].roll_gain = 0.5;
  p.spec.engines[0].fault = sim::GimbalFault::Hardover;
  p.spec.engines[0].fault_time_s = 30.0;
  p.spec.engines[0].fault_pitch_deg = 4.0;
  p.spec.engines[0].fault_yaw_deg = -2.0;
  p.spec.engines[0].chamber_pressure_pa = 3.0e7;
  p.spec.engines[0].area_ratio = 34.0;
  p.spec.engines[0].gamma = 1.2;
  sim::VehicleFile f;
  f.params = p;
  const std::string text = sim::write_vehicle(f);
  sim::VehicleFile g;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, g, errors) && errors.empty());
  const sim::EngineSpec& e = g.params.spec.engines[0];
  CHECK(g.params.spec.per_engine_gimbal && e.thrust_factor == 0.95 && e.isp_throttle_loss == 0.03 && e.gimbal_limit_deg == 5.0 && e.gimbal_rate_dps == 12.0 && e.gimbal_lag_s == 0.05);
  CHECK(e.roll_gain == 0.5 && e.fault == sim::GimbalFault::Hardover && e.fault_time_s == 30.0 && e.fault_pitch_deg == 4.0 && e.fault_yaw_deg == -2.0);
  CHECK(e.chamber_pressure_pa == 3.0e7 && e.area_ratio == 34.0 && e.gamma == 1.2);
  CHECK(g.params.spec.engines[1].thrust_factor == 1.0 && g.params.spec.engines[1].fault == sim::GimbalFault::None);
  // a wrong fault name and a roll gain on an engine that does not gimbal are refused with their place
  std::string bad = text;
  const std::size_t at = bad.find("hardover");
  bad.replace(at, 8, "sideways");
  sim::VehicleFile h;
  errors.clear();
  CHECK(!sim::read_vehicle(bad, h, errors) && !errors.empty());
  sim::Params q = cluster(2, 1.0, true);
  q.spec.engines[0].gimbal = false;
  q.spec.engines[0].roll_gain = 1.0;
  std::vector<std::string> problems = sim::validate(q.spec);
  CHECK(!problems.empty());
}

// ---- the analysis (sim/vehicle/propulsion.hpp) against the textbook and against the plant ----

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_analysis_the_ideal_nozzle_agrees_with_the_isentropic_tables) {
  namespace pr = sim::propulsion;
  // Mach 2 in a gamma 1.4 gas: A/A* = 1.6875 and p/p0 = 0.1278 (the tables), and the area ratio gives the Mach number back
  CHECK(near_abs(pr::area_ratio_of_mach(1.4, 2.0), 1.6875, 1.0e-4));
  CHECK(near_abs(pr::exit_pressure_ratio(1.4, 2.0), 0.12780, 1.0e-4));
  CHECK(near_abs(pr::exit_mach(1.4, 1.6875), 2.0, 1.0e-6));
  // the largest thrust coefficient a gamma 1.4 gas can give, expanded to nothing: gamma sqrt(2/(g-1) (2/(g+1))^((g+1)/(g-1))) = 1.8116
  CHECK(near_abs(pr::thrust_coefficient(1.4, 1.0e4, 0.0), 1.8116, 0.01));
  // a nozzle adapted to the ambient pressure has the thrust coefficient of its own exit pressure, and more area in a lower ambient pressure gives more thrust
  const double pe = pr::exit_pressure_ratio(1.2, pr::exit_mach(1.2, 34.0));
  CHECK(close(pr::thrust_coefficient(1.2, 34.0, pe), pr::thrust_coefficient(1.2, 34.0, 0.0) - (pe * 34.0), 1.0e-9));
  CHECK(pr::thrust_coefficient(1.2, 34.0, 0.0) > pr::thrust_coefficient(1.2, 34.0, 0.003));
  // the report: a file with no nozzle is not checked; one with it gets a throat that gives its thrust, and the separation throttle scales with the ambient pressure over the exit pressure
  sim::EngineSpec e;
  e.thrust_vac = 2.584e6;
  e.exit_area = 1.33;
  CHECK(!pr::check_nozzle(e).given);
  e.chamber_pressure_pa = 3.5e7;
  e.area_ratio = 34.0;
  e.gamma = 1.2;
  const pr::NozzleReport r = pr::check_nozzle(e);
  CHECK(r.given && near_abs(r.throat_area * r.eps, r.exit_area_ideal, 1.0e-12));
  CHECK(close(r.throat_area * r.cf_vac * r.pc, e.thrust_vac, 1.0e-12));
  CHECK(near_abs(r.exit_area_ideal, 1.34, 0.01) && near_abs(r.area_mismatch, -0.01, 0.01));                      // the sea-level Raptor's published nozzle is an ideal one
  CHECK(close(r.separation_throttle, pr::kSummerfield * pr::kSeaLevelPa / r.pe_full, 1.0e-12) && near_abs(r.separation_throttle, 0.39, 0.02));
  e.exit_area = 4.5;   // an exit area that does not belong to the nozzle: the report says by how much
  CHECK(pr::check_nozzle(e).area_mismatch > 0.25);
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_analysis_the_authority_and_the_engine_out_trim_are_what_the_plant_does) {
  namespace pr = sim::propulsion;
  const unsigned n = 6;
  sim::Params p = cluster(n, 1.5, true);
  for (sim::EngineSpec& e : p.spec.engines) {
    e.roll_gain = 1.0;
  }
  const sim::Vehicle6 v(p, in_space());
  const pr::Authority a = pr::authority(v, p, 0U, 1.0, 0.0);
  // the torque per degree of pitch from the plant, at a small angle
  const Out o = fire(p, burn(0.1), 0.01);
  CHECK(near_abs(a.pitch_nm_per_deg, o.m.z / 0.1, 1.0e-3 * a.pitch_nm_per_deg));
  const Out r = fire(p, burn(0.0, 0.0, 0.1), 0.01);
  CHECK(near_abs(a.roll_nm_per_deg, r.m.x / 0.1, 5.0e-3 * a.roll_nm_per_deg));
  CHECK(near_abs(a.limit_deg, 8.0, 1.0e-12) && close(a.thrust, n * kThrust, 1.0e-12));
  CHECK(close(a.pitch_accel_per_deg, a.pitch_nm_per_deg / a.i_t, 1.0e-12));
  // an engine out: the thrust that is left acts off the axis by (the engine's radius / the engines left), the torque is the lost thrust times the radius, and the gimbal that cancels it, run in the plant, does
  const pr::EngineOut eo = pr::engine_out(v, p, 0U, 1.0, 0.0, {0U});
  CHECK(close(eo.thrust_loss, 1.0 / n, 1.0e-9) && close(eo.imbalance_nm, 1.5 * kThrust, 1.0e-9));
  CHECK(near_abs(eo.offset_m, 1.5 / (n - 1.0), 1.0e-9));
  sim::Params q = p;
  q.spec.engines[0].thrust_factor = 0.0;
  const Out before = fire(q, burn());
  const Out after = fire(q, burn(eo.trim_pitch_deg, eo.trim_yaw_deg));
  CHECK(std::fabs(before.m.z) > 1.0e5);
  CHECK(std::fabs(after.m.z) < 0.02 * std::fabs(before.m.z) && std::fabs(after.m.y) < 0.02 * std::fabs(before.m.z));
  CHECK(eo.trim_fraction > 0.0 && eo.trim_fraction < 1.0 && near_abs(eo.accel_left, pr::authority(v, p, 0U, 1.0, 0.0, {0U}).pitch_accel_max * (1.0 - eo.trim_fraction), 1.0e-12));
  // a cluster with no gimbal cannot trim at all
  sim::Params fixed = cluster(n, 1.5, true);
  for (sim::EngineSpec& e : fixed.spec.engines) {
    e.gimbal = false;
  }
  const sim::Vehicle6 vf(fixed, in_space());
  CHECK(pr::engine_out(vf, fixed, 0U, 1.0, 0.0, {0U}).trim_fraction > 1.0e6);
  CHECK(pr::engine_out(vf, fixed, 0U, 1.0, 0.0, {}).trim_fraction == 0.0);
}

TFC_TEST(propulsion_analysis_the_cluster_the_servo_and_the_thrusters_are_summed_from_the_file) {
  namespace pr = sim::propulsion;
  sim::Params p = cluster(4, 1.0, true);
  p.spec.engines[0].gimbal = false;
  p.spec.engines[1].exit_area = 0.5;
  p.spec.engines[1].isp_throttle_loss = 0.1;
  p.gimbal_rate_dps = 12.0;
  p.gimbal_lag_s = 0.1;
  // a roll thruster of the stage: counted in the roll torque of the thrusters and not in the cluster
  sim::EngineSpec th;
  th.thrust_vac = 20000.0;
  th.exit_area = 0.0;
  th.gimbal = false;
  th.control = sim::Control::RollPlus;
  th.dir = sim::V3{0.0, 0.0, 1.0};
  th.pos = sim::V3{0.0, 2.0, 0.0};
  p.spec.engines.push_back(th);
  const pr::Cluster c = pr::cluster_of(p.spec, 0U, pr::kSeaLevelPa);
  CHECK(c.n == 4U && c.n_gimbal == 3U && c.n_fixed == 1U);
  CHECK(close(c.f_vac, 4.0 * kThrust, 1.0e-12) && close(c.f_sl, (4.0 * kThrust) - (pr::kSeaLevelPa * 0.5), 1.0e-12));
  CHECK(close(c.mdot, 4.0 * kThrust / (330.0 * 9.80665), 1.0e-12));   // (full throttle: the loss is zero there)
  CHECK(close(pr::engine_mdot(p.spec.engines[1], 0.5), 0.5 * kThrust / (330.0 * 0.95 * 9.80665), 1.0e-12));
  const sim::Vehicle6 v(p, in_space());
  const pr::Authority a = pr::authority(v, p, 0U, 0.5, 0.0);
  CHECK(close(a.thruster_roll_nm, 2.0 * 20000.0, 1.0e-12));
  const pr::ServoReport s = pr::servo_report(p, 0U, p.spec.engines[1]);
  CHECK(near_abs(s.rate_full_hz, 12.0 / (2.0 * 3.14159265358979 * 8.0), 1.0e-9) && near_abs(s.lag_corner_hz, 1.0 / (2.0 * 3.14159265358979 * 0.1), 1.0e-9));
  p.spec.engines[1].gimbal_limit_deg = 4.0;   // an engine of its own travel and rate and lag
  p.spec.engines[1].gimbal_rate_dps = 30.0;
  p.spec.engines[1].gimbal_lag_s = 0.0;
  const pr::GimbalLimits gl = pr::gimbal_limits(p, 0U, p.spec.engines[1]);
  CHECK(gl.limit_deg == 4.0 && gl.rate_dps == 30.0 && gl.lag_s == 0.0);
  CHECK(pr::servo_report(p, 0U, p.spec.engines[1]).lag_corner_hz == 0.0);
  p.spec.actuator.order = 2;
  p.spec.actuator.natural_hz = 7.0;
  CHECK(pr::servo_report(p, 0U, p.spec.engines[2]).second_order_hz == 7.0);
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_the_shipped_starship_class_vehicle_is_what_its_public_numbers_say_and_its_nozzles_are_consistent) {
  namespace pr = sim::propulsion;
  sim::VehicleFile f;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/missions/starship_return.json", f, errors) && errors.empty());
  const sim::VehicleSpec& g = f.params.spec;
  CHECK(g.per_engine_gimbal);
  const pr::Cluster booster = pr::cluster_of(g, 0U, pr::kSeaLevelPa);
  CHECK(booster.n == 33U && booster.n_gimbal == 13U && booster.n_fixed == 20U);
  CHECK(near_abs(booster.f_sl, 80.8e6, 0.1e6));     // the 80.8 MN at sea level of the public pages
  CHECK(booster.isp_vac > 345.0 && booster.isp_vac < 355.0 && booster.isp_sl < booster.isp_vac);
  const pr::Cluster ship = pr::cluster_of(g, 1U, pr::kSeaLevelPa);
  CHECK(ship.n == 6U && ship.n_gimbal == 6U);
  // every kind of engine has a nozzle in the file and it is the ideal one for its area ratio and thrust, within a tenth: the file is self-consistent
  for (const sim::EngineSpec& e : g.engines) {
    if (e.control != sim::Control::None) {
      continue;
    }
    const pr::NozzleReport r = pr::check_nozzle(e);
    CHECK(r.given && std::fabs(r.area_mismatch) < 0.05 && std::fabs(r.sl_ratio_ideal - r.sl_ratio_file) < 0.03);
  }
  // the booster's engine out is trimmed with a small part of the travel at the start of the burn, and the cluster has roll authority from the differential gimbal as well as from its thrusters
  const sim::Vehicle6 v(f.params, f.scenario);
  double worst = 0.0;
  for (const std::size_t i : pr::main_engines(g, 0U)) {
    worst = std::fmax(worst, pr::engine_out(v, f.params, 0U, 1.0, pr::kSeaLevelPa, {i}).trim_fraction);
  }
  CHECK(worst > 0.0 && worst < 0.1);
  const pr::Authority a = pr::authority(v, f.params, 0U, 1.0, 0.0);
  CHECK(a.roll_accel_per_deg > 0.0 && a.thruster_roll_accel > 0.0 && a.roll_accel_per_deg > a.thruster_roll_accel);
}

// Verifies: TFC-SIM-021
TFC_TEST(propulsion_analysis_handles_the_edges_an_empty_stage_a_stage_above_one_gone_a_dead_cluster_and_a_gimbal_with_no_travel) {
  namespace pr = sim::propulsion;
  // the nozzle report with part of the data, or an engine with no thrust or no exit area: it says what it can and divides by nothing
  sim::EngineSpec e;
  e.thrust_vac = 2.0e6;
  e.exit_area = 1.0;
  e.chamber_pressure_pa = 3.0e7;
  e.area_ratio = 1.0;   // not above 1: no nozzle
  e.gamma = 1.2;
  CHECK(!pr::check_nozzle(e).given);
  e.area_ratio = 30.0;
  e.gamma = 1.0;
  CHECK(!pr::check_nozzle(e).given);
  e.gamma = 1.2;
  e.exit_area = 0.0;
  const pr::NozzleReport no_area = pr::check_nozzle(e);
  CHECK(no_area.given && no_area.efficiency_if_ideal_area == 0.0 && near_abs(no_area.area_mismatch, -1.0, 1.0e-12));
  e.thrust_vac = 0.0;
  e.exit_area = 1.0;
  const pr::NozzleReport no_thrust = pr::check_nozzle(e);
  CHECK(no_thrust.given && no_thrust.area_mismatch == 0.0 && no_thrust.sl_ratio_file == 0.0);
  // a stage that has no engines: no thrust, no flow, no specific impulse, nothing to lose
  sim::Params p = cluster(4, 1.5, true);
  const sim::Vehicle6 v(p, in_space());
  const pr::Cluster none = pr::cluster_of(p.spec, 3U, pr::kSeaLevelPa);
  CHECK(none.n == 0U && none.mdot == 0.0 && none.isp_vac == 0.0 && none.isp_sl == 0.0);
  CHECK(pr::engine_out(v, p, 3U, 1.0, 0.0, {}).thrust_loss == 0.0);
  // every engine out: nothing left to trim with, and no thrust to act off the axis
  const pr::EngineOut dead = pr::engine_out(v, p, 0U, 1.0, 0.0, {0U, 1U, 2U, 3U});
  CHECK(dead.thrust_loss == 1.0 && dead.offset_m == 0.0 && dead.trim_fraction == 0.0);
  // a gimbal with no travel gives torque per degree but can trim nothing
  sim::Params locked = cluster(4, 1.5, true);
  for (sim::EngineSpec& en : locked.spec.engines) {
    en.gimbal_limit_deg = 0.0;
  }
  const sim::Vehicle6 vl(locked, in_space());
  CHECK(pr::engine_out(vl, locked, 0U, 1.0, 0.0, {0U}).trim_fraction > 1.0e8);
  CHECK(pr::servo_report(locked, 0U, locked.spec.engines[0]).rate_full_hz == 0.0);
  // the stage above one that is still on the vehicle: the lower one is counted gone in the mass
  sim::Params two = cluster(4, 1.5, true);
  sim::StageSpec upper = two.spec.stages[0];
  upper.name = "upper";
  upper.x_start = 30.0;
  upper.x_cg_dry = 42.0;
  two.spec.stages.push_back(upper);
  for (unsigned i = 0; i < 2U; ++i) {
    sim::EngineSpec en = two.spec.engines[i];
    en.stage = 1;
    en.pos.x = 30.0;
    two.spec.engines.push_back(en);
  }
  const sim::Vehicle6 v2(two, in_space());
  const pr::Authority a0 = pr::authority(v2, two, 0U, 0.5, 0.0);
  const pr::Authority a1 = pr::authority(v2, two, 1U, 0.5, 0.0);
  CHECK(a0.mass > a1.mass && a1.mass > 0.0);   // the stage below, with the other above it full, against the upper one counted with the lower one gone
}

TFC_TEST(propulsion_a_vehicle_file_with_a_bad_engine_is_refused_with_its_place) {
  const auto bad = [](const std::function<void(sim::EngineSpec&)>& change) {
    sim::Params p = cluster(3, 1.0, true);
    change(p.spec.engines[0]);
    return sim::validate(p.spec);
  };
  CHECK(bad([](sim::EngineSpec&) {}).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.thrust_factor = -0.1; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.thrust_factor = 2.5; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.isp_throttle_loss = 1.0; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.roll_gain = -1.0; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.fault_time_s = -1.0; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.chamber_pressure_pa = -1.0; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.area_ratio = -1.0; }).empty());
  CHECK(!bad([](sim::EngineSpec& e) { e.gamma = 0.9; }).empty());
  CHECK(bad([](sim::EngineSpec& e) { e.gamma = 1.2; e.roll_gain = 0.5; e.isp_throttle_loss = 0.1; e.thrust_factor = 0.9; }).empty());
  // the fault names: every one is read and written back, and an unknown one is refused
  for (const sim::GimbalFault f : {sim::GimbalFault::None, sim::GimbalFault::Stuck, sim::GimbalFault::Hardover}) {
    sim::Params p = cluster(2, 1.0, true);
    p.spec.engines[1].fault = f;
    p.spec.engines[1].fault_time_s = 5.0;
    sim::VehicleFile vf;
    vf.params = p;
    sim::VehicleFile back;
    std::vector<std::string> errors;
    CHECK(sim::read_vehicle(sim::write_vehicle(vf), back, errors) && errors.empty() && back.params.spec.engines[1].fault == f);
  }
}
