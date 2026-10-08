// SPDX-License-Identifier: MIT
// One bending mode (docs/design/DYNAMICS.md): a modal coordinate in each lateral plane, driven by the lateral thrust, that bends the thrust's direction at the engines and shows in the gyros and
// accelerometers of a vehicle's own IMU. Checked against the closed form of a damped oscillator, the static deflection under a lateral thrust (with the thrust's own stiffness), the sign and the
// size of what the sensors read, and the way the mode ends with its stage. With it off (the default) nothing changes.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

constexpr double kPiLocal = 3.14159265358979323846;

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }

// A heavy stiff vehicle (so that the body hardly turns or moves in the seconds looked at) with a bending mode, in vacuum and out of gravity, and an engine of `thrust` newtons.
sim::Params flexible(double thrust) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  p.gimbal_limit_deg = 45.0;
  p.gimbal_rate_dps = 10000.0;
  p.spec.planet.atmosphere = sim::AtmosphereKind::None;
  sim::StageSpec st;
  st.name = "flexible";
  st.dry_mass = 1.0e6;
  st.length = 4.0;
  st.radius = 0.6;
  st.x_cg_dry = 2.0;
  st.inertia_factor = 1.0e5;
  p.spec.stages.push_back(st);
  if (thrust > 0.0) {
    sim::EngineSpec e;
    e.thrust_vac = thrust;
    e.isp_vac = 1.0e9;
    e.exit_area = 0.0;
    p.spec.engines.push_back(e);
    p.spec.stages[0].tanks.push_back(sim::TankSpec{1000.0, 0.5, 0.5, 1000.0, {}});
  }
  p.spec.flex.enabled = true;
  p.spec.flex.frequency_hz = 3.0;
  p.spec.flex.damping = 0.05;
  p.spec.flex.generalized_mass = 1000.0;
  p.spec.flex.phi_engine = 1.0;
  p.spec.flex.slope_engine = 0.2;
  p.spec.flex.phi_imu = 0.5;
  p.spec.flex.slope_imu = -0.15;
  return p;
}

sim::Scenario in_space() {
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  return sc;
}

}  // namespace

TFC_TEST(flex_a_free_mode_rings_down_as_a_damped_oscillator_does_in_each_plane) {
  sim::Params p = flexible(0.0);
  const double wn = 2.0 * kPiLocal * 3.0;
  const double zeta = 0.05;
  const double wd = wn * std::sqrt(1.0 - (zeta * zeta));
  sim::Scenario sc = in_space();
  sc.initial.flex = {0.01, -0.02, 0.0, 0.0};  // 1 cm in Y, -2 cm in Z, at rest
  sim::Vehicle6 v(p, sc);
  for (int k = 1; k <= 2000; ++k) {  // 2 s
    v.step(0.001, 0.0, 0.0);
    if (k % 250 == 0) {
      const double t = 0.001 * k;
      const double f = std::exp(-zeta * wn * t) * (std::cos(wd * t) + ((zeta * wn / wd) * std::sin(wd * t)));  // x(t) / x0
      CHECK(near_abs(v.state().flex[0], 0.01 * f, 1e-8) && near_abs(v.state().flex[1], -0.02 * f, 1e-8));
    }
  }
  // no frequency in the vehicle that is not flexible: nothing moves
  sim::Params q = flexible(0.0);
  q.spec.flex.enabled = false;
  sim::Vehicle6 w(q, sc);
  for (int k = 0; k < 500; ++k) {
    w.step(0.001, 0.0, 0.0);
  }
  CHECK(w.state().flex[0] == 0.01 && w.state().flex[1] == -0.02 && w.state().flex[2] == 0.0);
}

TFC_TEST(flex_a_lateral_thrust_deflects_the_mode_by_its_stiffness_less_the_stiffness_of_the_thrust_following_the_slope) {
  const double thrust = 100000.0;
  for (int plane = 0; plane < 2; ++plane) {
    sim::Params p = flexible(thrust);
    sim::Vehicle6 v(p, in_space());
    const double delta = 1.0;  // degrees of gimbal in the plane
    for (int k = 0; k < 12000; ++k) {  // 12 s: the 5 % damping has taken the transient away
      v.step(0.001, plane == 0 ? delta : 0.0, plane == 1 ? delta : 0.0);
    }
    const double wn = 2.0 * kPiLocal * 3.0;
    const double mg = 1000.0;
    // the lateral thrust of a gimbal angle: -T sin in Y for a pitch command, +T sin in Z for a yaw command (the sign of the model's thrust direction)
    const double f_lat = thrust * std::sin(delta * kPiLocal / 180.0) * (plane == 0 ? -1.0 : 1.0);
    const double expected = 1.0 * f_lat / ((mg * wn * wn) - (1.0 * 0.2 * thrust));  // phi_e F / (M w^2 - phi_e sigma_e T)
    const double got = plane == 0 ? v.state().flex[0] : v.state().flex[1];
    CHECK(close(got, expected, 0.02) && std::fabs(expected) > 1e-3);
    CHECK(std::fabs(plane == 0 ? v.state().flex[1] : v.state().flex[0]) < 1e-9 * std::fabs(expected) + 1e-12);  // the other plane is not excited
  }
}

TFC_TEST(flex_the_thrust_follows_the_slope_of_the_structure_at_the_engines) {
  const double thrust = 100000.0;
  sim::Params p = flexible(thrust);
  sim::Scenario sc = in_space();
  sc.initial.flex = {0.01, -0.03, 0.0, 0.0};
  const sim::Vehicle6 v(p, sc);
  const sim::Loads l = v.current_loads();
  // the lateral force T sigma eta in each plane, on top of the straight thrust T along the axis
  CHECK(near_abs(l.f_thrust.y, thrust * 0.2 * 0.01, 1e-6) && near_abs(l.f_thrust.z, thrust * 0.2 * -0.03, 1e-6) && near_abs(l.f_thrust.x, thrust, 1e-6));
  // its moment about the centre of gravity: the engine is at the aft end, 2 m behind it
  const sim::MassProps mp = v.mass_props();
  CHECK(near_abs(l.m_thrust.z, (0.0 - mp.x_cg) * l.f_thrust.y, 1e-6) && near_abs(l.m_thrust.y, -(0.0 - mp.x_cg) * l.f_thrust.z, 1e-6));
  // the generalised force on the mode is phi_engine times the lateral thrust
  CHECK(near_abs(l.flex_q.y, l.f_thrust.y, 1e-9) && near_abs(l.flex_q.z, l.f_thrust.z, 1e-9));
  // the slope is the mode's, whichever way the engines gimbal
  sim::Params q = flexible(thrust);
  q.spec.flex.phi_engine = 2.5;
  const sim::Vehicle6 w(q, sc);
  CHECK(near_abs(w.current_loads().flex_q.y, 2.5 * w.current_loads().f_thrust.y, 1e-9));
}

TFC_TEST(flex_the_gyros_and_accelerometers_on_the_vehicle_read_the_bending) {
  sim::Params p = flexible(0.0);
  sim::Scenario sc = in_space();
  sc.initial.flex = {0.01, -0.02, 0.3, -0.4};  // displaced and moving
  sc.initial.w = sim::V3{0.0, 0.01, 0.02};     // the body turns too
  const sim::Vehicle6 v(p, sc);
  sim::V3 gyro;
  sim::V3 accel;
  v.vehicle_true_imu(gyro, accel);
  const double r2d = 180.0 / kPiLocal;
  // body rates plus the slope's: (0, -sigma eta'_z, sigma eta'_y); the sensor frame is (body Y, body Z, body X)
  CHECK(near_abs(gyro.x, (0.01 + (-(-0.15) * -0.4)) * r2d, 1e-9));  // body Y: 0.01 - sigma * eta'_z, with sigma = -0.15 and eta'_z = -0.4
  CHECK(near_abs(gyro.y, (0.02 + (-0.15 * 0.3)) * r2d, 1e-9));      // body Z: 0.02 + sigma * eta'_y
  CHECK(near_abs(gyro.z, 0.0, 1e-12));                               // no roll
  // the accelerometers: the structure's acceleration there, phi_imu eta'' (no thrust: eta'' = -2 zeta w eta' - w^2 eta)
  const double wn = 2.0 * kPiLocal * 3.0;
  const double ddy = (-2.0 * 0.05 * wn * 0.3) - (wn * wn * 0.01);
  const double ddz = (-2.0 * 0.05 * wn * -0.4) - (wn * wn * -0.02);
  CHECK(near_abs(accel.x, 0.5 * ddy / 9.80665, 1e-9) && near_abs(accel.y, 0.5 * ddz / 9.80665, 1e-9) && near_abs(accel.z, 0.0, 1e-12));
  // with the engine running the mode is driven too, and the accelerometer reads that (the force on the mode over its mass, in each plane)
  {
    sim::Params t = flexible(100000.0);
    sim::Scenario sw = in_space();
    sw.initial.m = 1.0e6 + 1000.0;
    sw.initial.flex = {0.01, -0.02, 0.3, -0.4};
    const sim::Vehicle6 u(t, sw);
    sim::V3 gu;
    sim::V3 au;
    u.vehicle_true_imu(gu, au);
    const sim::Loads l = u.current_loads();
    const double eddy = (l.flex_q.y - (2.0 * 0.05 * wn * 1000.0 * 0.3) - (wn * wn * 1000.0 * 0.01)) / 1000.0;
    const double eddz = (l.flex_q.z - (2.0 * 0.05 * wn * 1000.0 * -0.4) - (wn * wn * 1000.0 * -0.02)) / 1000.0;
    CHECK(std::fabs(l.flex_q.y) > 100.0 && std::fabs(l.flex_q.z) > 100.0);
    CHECK(near_abs(au.x, (l.f_thrust.y / sw.initial.m + (0.5 * eddy)) / 9.80665, 1e-12) && near_abs(au.y, (l.f_thrust.z / sw.initial.m + (0.5 * eddz)) / 9.80665, 1e-12));
  }
  // a vehicle that does not bend reads the rigid body only
  sim::Params q = flexible(0.0);
  q.spec.flex.enabled = false;
  const sim::Vehicle6 w(q, sc);
  sim::V3 g2;
  sim::V3 a2;
  w.vehicle_true_imu(g2, a2);
  CHECK(near_abs(g2.x, 0.01 * r2d, 1e-12) && near_abs(g2.y, 0.02 * r2d, 1e-12) && a2.x == 0.0 && a2.y == 0.0);
}

TFC_TEST(flex_the_mode_ends_with_the_stage_it_belongs_to) {
  sim::Params p = flexible(0.0);
  p.spec.stages[0].separate_time_s = 0.5;
  sim::StageSpec up;
  up.name = "upper";
  up.dry_mass = 1000.0;
  up.x_start = 4.0;
  up.length = 2.0;
  up.radius = 0.4;
  up.tanks.push_back(sim::TankSpec{100.0, 4.1, 0.3, 1000.0, {}});
  p.spec.stages.push_back(up);
  sim::EngineSpec side;  // an engine of the upper stage, canted so that it pushes sideways
  side.stage = 1;
  side.pos = sim::V3{4.0, 0.0, 0.0};
  side.thrust_vac = 5000.0;
  side.isp_vac = 300.0;
  side.exit_area = 0.0;
  side.gimbal = false;
  side.cant_pitch_deg = 5.0;
  p.spec.engines.push_back(side);
  p.spec.stages[1].ignite_time_s = 0.0;
  sim::Scenario sc = in_space();
  sc.initial.flex = {0.01, 0.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 400; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(v.state().flex[0] != 0.0 && v.stage_active(0U));
  for (int k = 0; k < 300; ++k) {
    v.step(0.001, 0.0, 0.0);
  }
  CHECK(!v.stage_active(0U) && v.state().flex[0] == 0.0 && v.state().flex[2] == 0.0);
  sim::V3 gyro;
  sim::V3 accel;
  v.vehicle_true_imu(gyro, accel);
  // (the upper stage's engine turns and pushes the rest of the vehicle: what the IMU reads is that, with nothing of the mode added)
  const double mass_now = v.mass();
  CHECK(near_abs(gyro.x, v.state().w.y * 180.0 / kPiLocal, 1e-12) && near_abs(gyro.y, v.state().w.z * 180.0 / kPiLocal, 1e-12));
  CHECK(near_abs(accel.x, v.current_loads().f_thrust.y / mass_now / 9.80665, 1e-12));
  CHECK(v.current_loads().f_thrust.y != 0.0 && v.current_loads().flex_q.y == 0.0);  // (the upper stage's engine does push sideways, and the mode no longer feels it)
}

TFC_TEST(flex_files_read_the_mode_write_it_back_and_refuse_what_cannot_be) {
  const std::string text = R"({
    "flex": {"stage": 0, "frequency_hz": 2.5, "damping": 0.004, "generalized_mass_kg": 8000, "phi_engine": 1.2, "slope_engine_per_m": 0.03, "phi_imu": -0.4, "slope_imu_per_m": -0.02},
    "stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}]
  })";
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const sim::FlexSpec& f = v.params.spec.flex;
  CHECK(f.enabled && f.stage == 0 && f.frequency_hz == 2.5 && f.damping == 0.004 && f.generalized_mass == 8000.0 && f.phi_engine == 1.2 && f.slope_engine == 0.03 && f.phi_imu == -0.4 && f.slope_imu == -0.02);
  const std::string once = sim::write_vehicle(v);
  sim::VehicleFile w;
  CHECK(sim::read_vehicle(once, w, errors) && errors.empty() && sim::write_vehicle(w) == once);
  CHECK(w.params.spec.flex.slope_imu == -0.02 && w.params.spec.flex.generalized_mass == 8000.0);
  sim::VehicleFile plain;
  CHECK(sim::read_vehicle(R"({"stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}]})", plain, errors));
  CHECK(sim::write_vehicle(plain).find("\"flex\"") == std::string::npos);
  const auto refused = [](const std::string& fragment, const std::string& what) {
    sim::VehicleFile f2;
    std::vector<std::string> errs;
    const std::string t = R"({"stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}], )" + fragment + "}";
    if (sim::read_vehicle(t, f2, errs) && errs.empty()) {
      for (const std::string& p : sim::validate(f2.params.spec)) {
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
  CHECK(refused(R"("flex": {"stage": 3, "frequency_hz": 2, "generalized_mass_kg": 10})", "flex.stage must name a stage"));
  CHECK(refused(R"("flex": {"frequency_hz": 0, "generalized_mass_kg": 10})", "frequency_hz and generalized_mass_kg must be positive"));
  CHECK(refused(R"("flex": {"frequency_hz": 2, "generalized_mass_kg": 0})", "frequency_hz and generalized_mass_kg must be positive"));
  CHECK(refused(R"("flex": {"frequency_hz": 2, "generalized_mass_kg": 10, "damping": -0.1})", "damping not negative"));
  CHECK(refused(R"("flex": {"frequency_hz": 2, "generalized_mass_kg": 10, "slope_enine_per_m": 0.1})", "slope_engine_per_m"));  // a typo is refused with a suggestion
  sim::VehicleSpec g;
  sim::StageSpec st;
  st.dry_mass = 1.0;
  st.length = 1.0;
  g.stages.push_back(st);
  g.flex.enabled = true;
  g.flex.frequency_hz = 2.0;
  g.flex.generalized_mass = 10.0;
  g.flex.phi_imu = std::nan("");
  bool named = false;
  for (const std::string& p : sim::validate(g)) {
    named = named || p.find("mode shape's values must be numbers") != std::string::npos;
  }
  CHECK(named);
}
