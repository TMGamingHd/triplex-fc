// SPDX-License-Identifier: MIT
// The state the 3D viewer is given (sim/vehicle/viewer_state.hpp, docs/design/VIEWER.md): every number in a pose is the simulator's own, the line is valid JSON with no non-finite value, it describes
// the vehicle that the spec describes (as many engines, as many tanks), and the geometry in it is consistent (the position is a planet radius plus the altitude, the attitude is a unit quaternion, the
// velocity through the air agrees with the speed when there is no wind).
#include <cmath>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "json.hpp"
#include "spec_io.hpp"
#include "viewer_state.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

std::string repo_root() {
#ifdef TFC_SOURCE_DIR
  return std::string(TFC_SOURCE_DIR) + "/";
#else
  const std::string f = __FILE__;
  const std::size_t pos = f.rfind("tests/");
  return pos == std::string::npos ? std::string() : f.substr(0, pos);
#endif
}

double vec_norm(const sim::Json* j) {
  double s = 0.0;
  for (const sim::Json& c : j->items) {
    s += c.number * c.number;
  }
  return std::sqrt(s);
}

// a member of a JSON object, or a null value when there is none (never a null pointer: a test that goes on after a failed check must not dereference one)
const sim::Json& get(const sim::Json& j, const char* key) {
  static const sim::Json kNull{};
  const sim::Json* f = j.find(key);
  return f != nullptr ? *f : kNull;
}

double rotate_inv_x(const sim::Vehicle6& v) { return sim::rotate_inv(v.state().q, v.state().v).x; }   // the speed along the long axis (no wind, a planet that does not turn)

sim::VehicleFile load(const char* name) {
  sim::VehicleFile vf;
  std::vector<std::string> errors;
  const bool ok = sim::load_vehicle_file(repo_root() + "vehicles/" + name, vf, errors);
  CHECK(ok);
  return vf;
}

}  // namespace

TFC_TEST(viewer_spec_is_json_and_matches_the_vehicle) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  sim::Vehicle6 veh(vf.params, vf.scenario);
  const std::string text = sim::viewer_spec_json(vf, veh);
  CHECK(text.find('\n') == std::string::npos);        // one datagram, one line of a recording
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(text, j, err));
  CHECK(get(j, "k").text == "spec");
  const sim::Json& v = get(j, "vehicle");
  CHECK(v.type == sim::Json::Type::Object);
  CHECK(get(v, "engines").items.size() == veh.spec().engines.size());   // the rings are expanded: an engine's index is the index in a pose
  CHECK(get(v, "stages").items.size() == veh.spec().stages.size());
  const sim::Json& air = get(j, "air");
  const sim::Json& rho = get(air, "density");
  const sim::Json& t = get(air, "temperature_k");
  CHECK(rho.items.size() == t.items.size() && rho.items.size() > 100U);
  CHECK(rho.items.size() > 50U && t.items.size() > 0U);
  if (rho.items.size() > 50U && t.items.size() > 0U) {
    CHECK(near_abs(t.items[0].number, 288.15, 1e-3));
    CHECK(near_abs(rho.items[0].number, 1.225, 1e-4));
    CHECK(rho.items[50].number < rho.items[10].number);    // the air thins with altitude
  }
  CHECK(near_abs(j.find("radius_m")->number, veh.planet().radius, 1.0));
}

TFC_TEST(viewer_pose_is_consistent_with_the_vehicle) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  vf.scenario.wind_scale = 0.0;                        // no wind: the velocity through the air is the velocity
  sim::Vehicle6 veh(vf.params, vf.scenario);
  for (int i = 0; i < 3000; ++i) {                     // 30 s of vertical flight, no steering
    veh.step(0.01, 0.0, 0.0);
  }
  for (int i = 0; i < 100; ++i) {                      // steer a little, so that the gimbals are not at zero and a swap of pitch and yaw shows
    veh.step(0.01, 0.8, -0.3);
  }
  sim::PoseExtra x;
  x.frame = 3100U;
  x.ref_pitch_deg = 1.25;
  x.ref_yaw_deg = -0.75;
  x.cmd_pitch_deg = 0.5;
  x.cmd_yaw_deg = -0.25;
  x.act_mode = 1;
  const std::string text = sim::viewer_pose_json(veh, 31.0, x);
  CHECK(text.find('\n') == std::string::npos);
  CHECK(text.find("nan") == std::string::npos && text.find("inf") == std::string::npos);
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(text, j, err));
  CHECK(j.find("k")->text == "pose");
  CHECK(near_abs(vec_norm(j.find("r")), veh.planet().radius + veh.altitude(), 0.01));
  CHECK(near_abs(vec_norm(j.find("q")), 1.0, 1e-6));
  CHECK(near_abs(j.find("q")->items[0].number, veh.state().q.w, 1e-6) && near_abs(j.find("q")->items[1].number, veh.state().q.x, 1e-6));   // (w, x, y, z), in that order
  CHECK(near_abs(j.find("q")->items[3].number, veh.state().q.z, 1e-6));
  CHECK(near_abs(j.find("r")->items[0].number, veh.state().r.x, 0.01) && near_abs(j.find("v")->items[1].number, veh.state().v.y, 0.001));
  CHECK(near_abs(j.find("w")->items[2].number, veh.state().w.z, 1e-5));
  CHECK(near_abs(j.find("mach")->number, veh.current_loads().mach, 1e-4) && near_abs(j.find("qd")->number, veh.current_loads().dynamic_pressure, 0.1));
  CHECK(near_abs(j.find("alpha")->number, veh.current_loads().alpha * sim::kRad2Deg, 1e-3));
  CHECK(near_abs(j.find("m")->number, veh.mass(), 0.1) && near_abs(j.find("thr")->number, veh.current_loads().thrust, 1.0) && near_abs(j.find("mdot")->number, veh.current_loads().mdot, 1e-3));
  CHECK(near_abs(j.find("fth")->items[0].number, veh.current_loads().f_thrust.x, 1.0) && near_abs(j.find("fae")->items[1].number, veh.current_loads().f_aero.y, 1.0));
  CHECK(near_abs(j.find("mth")->items[2].number, veh.current_loads().m_thrust.z, 1.0) && near_abs(j.find("mae")->items[1].number, veh.current_loads().m_aero.y, 1.0));
  CHECK(near_abs(j.find("cg")->number, veh.mass_props().x_cg, 1e-3));
  CHECK(near_abs(j.find("rng")->number, veh.range(), 0.1));
  CHECK(near_abs(j.find("tilt")->items[0].number, veh.tilts().y_deg, 1e-3) && near_abs(j.find("tilt")->items[1].number, veh.tilts().x_deg, 1e-3));   // the pitch plane first
  CHECK(near_abs(j.find("ref")->items[0].number, 1.25, 1e-9) && near_abs(j.find("ref")->items[1].number, -0.75, 1e-9));
  CHECK(near_abs(j.find("cmd")->items[0].number, 0.5, 1e-9) && near_abs(j.find("cmd")->items[1].number, -0.25, 1e-9) && j.find("act")->number == 1.0);
  CHECK(near_abs(j.find("t")->number, 31.0, 1e-9) && near_abs(j.find("tt")->number, veh.time(), 1e-3) && j.find("fr")->number == 3100.0);
  CHECK(near_abs(j.find("alt")->number, veh.altitude(), 0.01));
  CHECK(near_abs(j.find("spd")->number, veh.speed(), 0.001));
  CHECK(near_abs(vec_norm(j.find("vair")), veh.speed(), 0.01));
  CHECK(near_abs(j.find("vair")->items[0].number, rotate_inv_x(veh), 0.05));
  CHECK(j.find("eng")->items.size() == veh.spec().engines.size());
  CHECK(j.find("gim")->items.size() == 2U * veh.spec().stages.size());
  CHECK(std::fabs(veh.stage_gimbal_pitch_deg(0U)) > 0.01 && near_abs(j.find("gim")->items[0].number, veh.stage_gimbal_pitch_deg(0U), 1e-3) && near_abs(j.find("gim")->items[1].number, veh.stage_gimbal_yaw_deg(0U), 1e-3));
  CHECK(near_abs(j.find("gim")->items[2].number, veh.stage_gimbal_pitch_deg(1U), 1e-3));                 // stage by stage, the pitch then the yaw of each
  for (std::size_t e = 0; e < veh.spec().engines.size(); ++e) {
    CHECK(near_abs(j.find("eng")->items[e].number, veh.engine_fraction(e), 1e-3));
  }
  // the tanks add up to the propellant of their stage, and the stages to the mass the vehicle has beyond its dry structure and payloads
  const sim::Json* tank = j.find("tank");
  CHECK(tank->items.size() == 4U);
  CHECK(near_abs(tank->items[0].number + tank->items[1].number, j.find("prop")->items[0].number, 0.5));
  CHECK(near_abs(tank->items[2].number + tank->items[3].number, j.find("prop")->items[1].number, 0.5));
  // the air is the model's own at that altitude
  const sim::Air a = veh.atmosphere(veh.altitude());
  CHECK(near_abs(j.find("air")->items[1].number, a.pressure, 0.01));
  CHECK(near_abs(j.find("air")->items[2].number, a.density, 1e-6));
  // the engines run at the throttle of the stage, the thrust is the sum the model reports, and the centre of pressure is on the vehicle
  CHECK(j.find("thr")->number > 1.0e6);
  CHECK(j.find("cp")->number > 0.0 && j.find("cp")->number < 80.0 && j.find("cg")->number > 0.0);
  double cna = 0.0;
  double xcp = 0.0;
  double ca = 0.0;
  double sref = 0.0;
  veh.aero_summary(veh.current_loads().mach, cna, xcp, ca, sref);
  CHECK(near_abs(j.find("cp")->number, xcp, 1e-3) && std::fabs(xcp - j.find("cg")->number) > 1.0);   // the centre of pressure is the model's own, and it is not the centre of mass
  CHECK(near_abs(j.find("cna")->number, cna, 1e-3) && near_abs(j.find("ca")->number, ca, 1e-3) && near_abs(j.find("dref")->number, std::sqrt(4.0 * sref / sim::kPi), 1e-3));
  CHECK(j.find("stg")->number == 3.0 && j.find("ign")->number == 1.0);
}

TFC_TEST(viewer_pose_follows_a_stage_that_leaves_in_a_closed_loop_flight) {
  // The vehicle is unstable without the flight software, so this is flown by it (closed_loop.hpp), and the poses are what the hook delivers.
  sim::VehicleFile vf = load("two_stage_launcher.json");
  sim::Loop lp;
  lp.cfg.params = vf.params;
  lp.cfg.design = vf.params;
  lp.cfg.scenario = vf.scenario;
  lp.cfg.plan = vf.plan;
  lp.cfg.vehicle_true = true;
  lp.estimator.use_accel = false;
  lp.pad_frames = 100U;
  lp.frames = 19000U;
  lp.pose_every = 100U;
  std::vector<std::string> poses;
  std::vector<double> times;
  lp.on_pose = [&](const sim::Vehicle6& v, double t, const sim::PoseExtra& x) {
    poses.push_back(sim::viewer_pose_json(v, t, x));
    times.push_back(t);
  };
  const sim::Result r = sim::run(lp);
  CHECK(r.finite && !r.crashed);
  CHECK(poses.size() > 150U);
  CHECK(times.front() < 0.0 && times.back() > 150.0);                        // the pad frames come first, with a negative time
  sim::Json first;
  sim::Json last;
  std::string err;
  CHECK(sim::parse_json(poses.front(), first, err) && sim::parse_json(poses.back(), last, err));
  CHECK(static_cast<unsigned>(first.find("stg")->number) == 3U);              // both stages on the vehicle on the pad
  CHECK(first.find("clamp")->number == 1.0);
  CHECK((static_cast<unsigned>(last.find("stg")->number) & 1U) == 0U);       // stage 1 has gone
  CHECK(near_abs(last.find("tank")->items[0].number, 0.0, 1e-9));            // and its tanks read empty
  for (std::size_t e = 0; e < 9U; ++e) {
    CHECK(last.find("eng")->items[e].number <= 0.0);                          // none of its engines runs
  }
  CHECK(last.find("eng")->items[9].number > 0.0);                             // the second stage's engine does
  CHECK(last.find("ref")->items.size() == 2U && last.find("act")->number >= 0.0);   // the flight computers' program and mode came with it
}

TFC_TEST(viewer_pose_marks_a_failed_engine) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  vf.scenario.engine_failures.push_back(sim::EngineFailure{1.0, 3});
  sim::Vehicle6 veh(vf.params, vf.scenario);
  for (int i = 0; i < 500; ++i) {
    veh.step(0.01, 0.0, 0.0);
  }
  sim::PoseExtra x;
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(sim::viewer_pose_json(veh, 5.0, x), j, err));
  CHECK(j.find("eng")->items[3].number < 0.0);                              // -1: failed
  CHECK(j.find("eng")->items[2].number > 0.9);
}

TFC_TEST(viewer_pose_reference_vehicle_and_every_example) {
  const char* files[] = {"reference.json", "sounding_rocket.json", "spacecraft.json", "launcher_dynamics.json"};
  for (const char* f : files) {
    sim::VehicleFile vf = load(f);
    sim::Vehicle6 veh(vf.params, vf.scenario);
    for (int i = 0; i < 200; ++i) {
      veh.step(0.01, 0.0, 0.0);
    }
    sim::PoseExtra x;
    sim::Json js;
    sim::Json jp;
    std::string err;
    CHECK(sim::parse_json(sim::viewer_spec_json(vf, veh), js, err));
    CHECK(sim::parse_json(sim::viewer_pose_json(veh, 2.0, x), jp, err));
    CHECK(jp.find("eng")->items.size() == js.find("vehicle")->find("engines")->items.size());
  }
}

TFC_TEST(viewer_pose_takes_the_wind_out_of_the_air_velocity) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  vf.scenario.wind_scale = 3.0;
  vf.scenario.wind_dir = sim::V3{0.0, 0.0, 1.0};
  sim::Vehicle6 veh(vf.params, vf.scenario);
  for (int i = 0; i < 4000; ++i) {
    veh.step(0.01, 0.0, 0.0);
  }
  sim::PoseExtra x;
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(sim::viewer_pose_json(veh, 40.0, x), j, err));
  const sim::V3 wind = veh.wind_at(veh.altitude(), veh.time());
  CHECK(sim::norm(wind) > 5.0);
  CHECK(near_abs(vec_norm(j.find("vair")), sim::norm(veh.state().v - wind), 0.05));         // the air speed is the speed relative to the air, not to the ground
  CHECK(std::fabs(vec_norm(j.find("vair")) - veh.speed()) > 1.0);
  CHECK(near_abs(j.find("wind")->items[2].number, wind.z, 1e-3));
}

TFC_TEST(viewer_spec_samples_the_air_and_the_wind_the_model_uses) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  vf.scenario.wind_scale = 2.0;
  sim::Vehicle6 veh(vf.params, vf.scenario);
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(sim::viewer_spec_json(vf, veh), j, err));
  const sim::Json* air = j.find("air");
  CHECK(near_abs(air->find("step_m")->number, 1000.0, 1e-9));
  for (std::size_t k : {0U, 5U, 11U, 30U, 80U}) {
    const sim::Air a = veh.atmosphere(static_cast<double>(k) * 1000.0);
    CHECK(near_abs(air->find("temperature_k")->items[k].number, a.temperature, 1e-3));
    CHECK(near_abs(air->find("pressure_pa")->items[k].number, a.pressure, 1e-3 + (1e-6 * a.pressure)));
    CHECK(near_abs(air->find("sound_ms")->items[k].number, a.sound, 1e-3));
    CHECK(near_abs(air->find("wind_ms")->items[k].number, veh.mean_wind_ms(static_cast<double>(k) * 1000.0), 1e-3));
  }
  vf.scenario.wind_scale = 1.0;
  const sim::Vehicle6 veh1(vf.params, vf.scenario);
  CHECK(near_abs(veh.mean_wind_ms(10000.0), 2.0 * veh1.mean_wind_ms(10000.0), 1e-9));            // the scenario's scale applies
  CHECK(veh.mean_wind_ms(10000.0) > 5.0);
  CHECK(near_abs(j.find("pole")->items[0].number, veh.pole().x, 1e-5));
  CHECK(near_abs(j.find("rotation_rad_s")->number, veh.planet().rotation_rate, 1e-9));
}

TFC_TEST(viewer_pose_tanks_follow_the_drain_rule_of_the_stage) {
  sim::VehicleFile vf = load("two_stage_launcher.json");
  sim::Vehicle6 veh(vf.params, vf.scenario);
  for (int i = 0; i < 2000; ++i) {
    veh.step(0.01, 0.0, 0.0);
  }
  sim::PoseExtra x;
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(sim::viewer_pose_json(veh, 20.0, x), j, err));
  const sim::Json* tank = j.find("tank");
  CHECK(tank->items.size() == 4U);
  // the kerosene and the oxygen of the first stage drain together, in proportion to their size (130 t and 280 t when full)
  CHECK(tank->items[0].number < 130000.0 && tank->items[1].number < 280000.0);
  CHECK(near_abs(tank->items[0].number / 130000.0, tank->items[1].number / 280000.0, 1e-3));
  CHECK(near_abs(tank->items[2].number, 30000.0, 1e-6) && near_abs(tank->items[3].number, 80000.0, 1e-6));   // the second stage has not burnt
  CHECK(near_abs(tank->items[0].number, veh.tank_liquid_kg(0U, 0U), 0.1) && near_abs(tank->items[3].number, veh.tank_liquid_kg(1U, 1U), 0.1));
}

TFC_TEST(viewer_json_writer_nulls_what_json_cannot_hold_and_escapes_text) {
  sim::viewer_detail::Out o;
  o.open('[');
  o.num(std::nan(""), 2);                              // JSON has no NaN: null, so that the whole line is still valid
  o.num(1.5, 1);
  o.text(std::string("a\"b\\c\n"));                  // a quote and a backslash are escaped, a control character is dropped
  o.close(']');
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(o.str(), j, err));
  CHECK(j.type == sim::Json::Type::Array && j.items.size() == 3U);
  CHECK(j.items[0].type == sim::Json::Type::Null && near_abs(j.items[1].number, 1.5, 1e-12));
  CHECK(j.items[2].text == "a\"b\\c");
}

TFC_TEST(viewer_pose_on_a_turning_planet_has_the_planets_motion_taken_out_of_the_air_velocity) {
  sim::VehicleFile vf = load("sounding_rocket.json");
  vf.params.spec.planet.rotation_rate = 7.2921159e-5;          // a planet that turns: the air turns with it, so on the pad the air velocity is nearly zero though the inertial speed is not
  vf.scenario.wind_scale = 0.0;
  sim::Vehicle6 veh(vf.params, vf.scenario);
  CHECK(veh.speed() > 100.0);                                  // the pad's own speed through inertial space
  sim::PoseExtra x;
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json(sim::viewer_pose_json(veh, 0.0, x), j, err));
  CHECK(vec_norm(j.find("vair")) < 1.0);
  sim::Json js;
  CHECK(sim::parse_json(sim::viewer_spec_json(vf, veh), js, err));
  CHECK(near_abs(js.find("rotation_rad_s")->number, 7.2921159e-5, 1e-12));
}
