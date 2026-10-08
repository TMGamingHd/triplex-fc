// SPDX-License-Identifier: MIT
// Vehicle description files (sim/vehicle/json.hpp, spec_io.hpp, vehicles/*.json; docs/design/VEHICLE_SPEC.md): the JSON reader and its errors, the vehicle reader and what it refuses, the writer
// and the round trip, and every committed example: it loads, it validates, and the real flight software flies it.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "json.hpp"
#include "spec_io.hpp"

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

bool mentions(const std::vector<std::string>& errors, const char* text) {
  for (const std::string& e : errors) {
    if (e.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

// The smallest vehicle the reader accepts: a stage, a tank and an engine.
const char* kMinimal = R"({
  "name": "minimal",
  "stages": [{"dry_mass_kg": 1000, "length_m": 6, "radius_m": 0.5, "x_cg_dry_m": 3,
              "tanks": [{"propellant_kg": 4000, "x_bottom_m": 0.5, "radius_m": 0.5, "density_kg_m3": 1000}]}],
  "engines": [{"stage": 0, "thrust_vac_n": 100000, "exit_area_m2": 0.05, "isp_vac_s": 300}]
})";

}  // namespace

// ---- the JSON reader ----

TFC_TEST(json_reads_values_comments_and_escapes) {
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json("// a comment\n{\"a\": 1.5e2, \"b\": [true, false, null], // another\n \"s\": \"x\\ty\\u0041\\\"\", \"o\": {}}", j, err));
  CHECK(j.type == sim::Json::Type::Object && j.fields.size() == 4U);
  CHECK(near_abs(j.find("a")->number, 150.0, 1e-12));
  const sim::Json* b = j.find("b");
  CHECK(b != nullptr && b->items.size() == 3U && b->items[0].boolean && !b->items[1].boolean && b->items[2].type == sim::Json::Type::Null);
  CHECK(j.find("s")->text == "x\tyA\"");
  CHECK(j.find("nope") == nullptr && j.find("o")->fields.empty());
  CHECK(sim::parse_json("  -0.25  ", j, err) && near_abs(j.number, -0.25, 1e-12));
}

TFC_TEST(json_errors_name_the_line_and_the_column) {
  const auto fails_with = [](const char* text, const char* what) {
    sim::Json j;
    std::string err;
    return !sim::parse_json(text, j, err) && err.find(what) != std::string::npos;
  };
  CHECK(fails_with("{\"a\": 1,}", "expected a quoted name"));      // a trailing comma
  CHECK(fails_with("{\"a\": 1 \"b\": 2}", "expected ',' or '}'"));
  CHECK(fails_with("[1, 2", "unexpected end of the file"));
  CHECK(fails_with("{\"a\": 1, \"a\": 2}", "appears twice"));
  CHECK(fails_with("{\"a\": 01}", "unexpected text") || fails_with("{\"a\": 01}", "expected"));
  CHECK(fails_with("{\"a\": 1.}", "digit must follow the decimal point"));
  CHECK(fails_with("{\"a\": \"abc}", "not closed"));
  CHECK(fails_with("{\"a\": nul}", "expected true, false or null"));
  CHECK(fails_with("{\"a\": 1e999}", "too large"));
  CHECK(fails_with("[1] 2", "after the end"));
  std::string deep(200, '[');
  CHECK(fails_with(deep.c_str(), "nested too deeply"));
  sim::Json j;
  std::string err;
  CHECK(!sim::parse_json("{\n  \"a\": 1,\n  \"b\": @\n}", j, err) && err.find("line 3, column 8") != std::string::npos);
}

// ---- the vehicle reader ----

TFC_TEST(vehicle_reader_accepts_a_minimal_description_and_fills_in_the_defaults) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(kMinimal, v, errors) && errors.empty());
  CHECK(v.name == "minimal" && v.params.spec.stages.size() == 1U && v.params.spec.engines.size() == 1U);
  CHECK(v.params.spec.stages[0].inertia_factor == 0.6 && v.params.spec.engines[0].gimbal && v.params.gimbal_limit_deg == 8.0);
  CHECK(v.plan.trajectory.t_end == 100.0 && v.plan.gains.every_s == 6.0);
  sim::Vehicle6 veh(v.params, v.scenario);
  CHECK(near_abs(veh.mass(), 5000.0, 1e-9) && veh.current_loads().thrust > 90000.0);
}

TFC_TEST(vehicle_reader_refuses_unknown_fields_suggests_the_nearest_and_reports_every_problem_at_once) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  const std::string bad = R"({
    "stages": [{"dry_mass": 1000, "length_m": "six", "radius_m": 0.5,
                "tanks": [{"propellant_kg": 4000, "x_bottom_m": 0.5, "radius_m": 0.5, "density_kg_m3": 1000}]}],
    "engines": [{"stage": 0, "thrust_vac_n": 100000, "isp_vac_s": 300, "gimbel": true}],
    "zzz": 1
  })";
  CHECK(!sim::read_vehicle(bad, v, errors));
  CHECK(mentions(errors, "stages[0].dry_mass: unknown field (did you mean \"dry_mass_kg\"?)"));
  CHECK(mentions(errors, "stages[0].length_m: expected a number"));
  CHECK(mentions(errors, "engines[0].gimbel: unknown field (did you mean \"gimbal\"?)"));
  CHECK(mentions(errors, "zzz: unknown field"));
  CHECK(mentions(errors, "(line 2, column"));  // every message says where
  CHECK(errors.size() >= 4U);
}

TFC_TEST(vehicle_reader_runs_the_vehicle_checks_and_the_cross_references) {
  const auto errors_for = [](const std::string& text) {
    sim::VehicleFile v;
    std::vector<std::string> errors;
    (void)sim::read_vehicle(text, v, errors);
    return errors;
  };
  CHECK(mentions(errors_for(R"({"stages": []})"), "at least one stage"));
  CHECK(mentions(errors_for(R"({"name": 3})"), "name: expected a string"));
  CHECK(mentions(errors_for(R"([1, 2])"), "expected an object"));
  std::string t = kMinimal;
  t.replace(t.find("\"stage\": 0"), 10, "\"stage\": 4");
  CHECK(mentions(errors_for(t), "engines[0].stage"));
  t = kMinimal;
  t.replace(t.find("4000"), 4, "4e9");
  CHECK(mentions(errors_for(t), "more propellant than fits"));
  t = kMinimal;
  t.replace(t.find("\"isp_vac_s\": 300"), 16, "\"isp_vac_s\": -1");
  CHECK(mentions(errors_for(t), "engines[0]"));
  t = kMinimal;
  t.replace(t.find("\"name\": \"minimal\","), 18, "\"design\": {\"t_end_s\": 0},");
  CHECK(mentions(errors_for(t), "t_end_s"));
}

TFC_TEST(vehicle_reader_expands_a_ring_of_engines_and_a_count) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  const std::string text = R"({
    "stages": [{"dry_mass_kg": 1000, "length_m": 6, "radius_m": 0.5, "x_cg_dry_m": 3,
                "tanks": [{"propellant_kg": 4000, "x_bottom_m": 0.5, "radius_m": 0.5, "density_kg_m3": 1000}]}],
    "engines": [{"stage": 0, "count": 1, "thrust_vac_n": 1000, "isp_vac_s": 300},
                {"stage": 0, "count": 8, "position_m": [0, 0.1, 0], "ring_radius_m": 1.2, "ring_start_deg": 0, "thrust_vac_n": 1000, "isp_vac_s": 300},
                {"stage": 0, "count": 3, "position_m": [0.5, 0, 0], "thrust_vac_n": 1000, "isp_vac_s": 300}]
  })";
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const std::vector<sim::EngineSpec>& e = v.params.spec.engines;
  CHECK(e.size() == 12U);
  CHECK(near_abs(e[1].pos.y, 0.1 + 1.2, 1e-12) && near_abs(e[1].pos.z, 0.0, 1e-12));           // the first of the ring: at the start angle
  CHECK(near_abs(e[3].pos.y, 0.1, 1e-12) && near_abs(e[3].pos.z, 1.2, 1e-12));                  // a quarter of the way round: 90 degrees (index 2 of 8 is 90: e[1 + 2])
  CHECK(near_abs(e[5].pos.y, 0.1 - 1.2, 1e-12) && near_abs(std::hypot(e[7].pos.y - 0.1, e[7].pos.z), 1.2, 1e-12));
  CHECK(near_abs(e[9].pos.x, 0.5, 1e-12) && near_abs(e[11].pos.x, 0.5, 1e-12));                // three together at the position
  sim::VehicleFile w;
  errors.clear();
  std::string many = text;
  many.replace(many.find("\"count\": 8"), 10, "\"count\": 99");
  CHECK(!sim::read_vehicle(many, w, errors) && mentions(errors, "count"));
}

TFC_TEST(vehicle_reader_starts_a_vehicle_in_a_circular_orbit_and_with_a_given_state) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  std::string t = kMinimal;
  t.insert(t.rfind('}'), R"(, "scenario": {"start": {"altitude_m": 400000, "circular_orbit": true}})");
  CHECK(sim::read_vehicle(t, v, errors) && errors.empty());
  CHECK(v.scenario.has_initial && near_abs(v.scenario.initial.r.x, sim::kEarthR + 400000.0, 1e-6));
  CHECK(near_abs(v.scenario.initial.v.y, std::sqrt(sim::kEarthMu / (sim::kEarthR + 400000.0)), 1e-9));
  t = kMinimal;
  t.insert(t.rfind('}'), R"(, "scenario": {"start": {"position_m": [7000000, 0, 0], "velocity_ms": [10, 20, 30], "rates_dps": [0, 1, 0]}})");
  errors.clear();
  CHECK(sim::read_vehicle(t, v, errors) && errors.empty());
  CHECK(v.scenario.initial.v.z == 30.0 && near_abs(v.scenario.initial.w.y, 1.0 * sim::kDeg2Rad, 1e-12));
}

// ---- writing, and the round trip ----

TFC_TEST(vehicle_writer_round_trips_every_field_and_a_second_write_is_the_same_text) {
  sim::VehicleFile a;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/two_stage_launcher.json", a, errors) && errors.empty());
  const std::string text = sim::write_vehicle(a);
  sim::VehicleFile b;
  CHECK(sim::read_vehicle(text, b, errors) && errors.empty());
  CHECK(sim::write_vehicle(b) == text);  // writing what was read gives what was written: nothing is lost or invented
  CHECK(b.params.spec.engines.size() == 10U && b.params.spec.stages.size() == 2U && b.params.spec.payloads.size() == 2U);
  CHECK(b.params.spec.stages[0].throttle.size() == 5U && b.params.spec.stages[1].ignite_after_sep_of == 0);
  sim::Vehicle6 va(a.params, a.scenario);
  sim::Vehicle6 vb(b.params, b.scenario);
  for (int k = 0; k < 1500; ++k) {
    va.step(0.01, 0.0, 0.0);
    vb.step(0.01, 0.0, 0.0);
  }
  CHECK(va.state().r.x == vb.state().r.x && va.state().v.y == vb.state().v.y && va.mass() == vb.mass());  // and the vehicle is the same, bit for bit
}

TFC_TEST(the_committed_reference_file_is_what_the_writer_writes_for_the_reference_vehicle) {
  sim::VehicleFile ref;
  ref.name = "reference";
  ref.description = "The reference vehicle of docs/design/VEHICLE_SIM.md: five engines, 30 t, a 100 s gravity-turn ascent through max-Q. The other files in vehicles/ are not this one.";
  const std::string expected = sim::write_vehicle(ref);
  std::vector<std::string> errors;
  sim::VehicleFile loaded;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/reference.json", loaded, errors) && errors.empty());
  CHECK(sim::write_vehicle(loaded) == expected);
  // and it IS the reference vehicle: the same flight, bit for bit, as the simple parameters give
  sim::Vehicle6 a(sim::Params{});
  sim::Vehicle6 b(loaded.params, loaded.scenario);
  for (int k = 0; k < 2000; ++k) {
    const double c = 1.0 * std::sin(0.03 * k);
    a.step(0.01, c, 0.5 * c);
    b.step(0.01, c, 0.5 * c);
  }
  CHECK(a.state().r.x == b.state().r.x && a.state().q.z == b.state().q.z && a.mass() == b.mass());
}

// ---- the examples ----

namespace {

struct Flown {
  bool loaded = false;
  sim::Result r;
  std::vector<sim::TraceRow> trace;
};

// Fly an example with the real flight software for `frames` frames.
Flown fly_example(const std::string& file, uint32_t frames, bool vehicle_sensors) {
  Flown f;
  sim::VehicleFile v;
  std::vector<std::string> errors;
  f.loaded = sim::load_vehicle_file(repo_root() + "vehicles/" + file, v, errors) && errors.empty();
  if (!f.loaded) {
    return f;
  }
  sim::Loop lp;
  lp.cfg.params = v.params;
  lp.cfg.design = v.params;
  lp.cfg.scenario = v.scenario;
  lp.cfg.plan = v.plan;
  lp.cfg.vehicle_true = vehicle_sensors;
  lp.estimator.use_accel = !vehicle_sensors;
  lp.frames = frames;
  lp.trace = &f.trace;
  lp.trace_every = 100U;
  f.r = sim::run(lp);
  return f;
}

}  // namespace

TFC_TEST(example_the_reference_file_flies_the_reference_ascent) {
  const Flown f = fly_example("reference.json", 10000U, false);
  CHECK(f.loaded && f.r.finite && !f.r.crashed && f.r.safe_frames == 0U && f.r.liftoff_frame <= 1U);
  CHECK(f.r.max_deg_settled < 1.0 && f.r.rms_deg < 0.2);
  CHECK(!f.trace.empty() && f.trace.back().altitude > 30000.0 && f.trace.back().speed > 900.0);
}

TFC_TEST(example_the_two_stage_launcher_flies_through_throttling_staging_and_the_fairing_with_the_vehicles_own_sensors) {
  const Flown f = fly_example("two_stage_launcher.json", 22000U, true);  // 220 s: past the separation at 166 s, the ignition at 170 s and the fairing at 210 s
  CHECK(f.loaded && f.r.finite && !f.r.crashed && f.r.safe_frames == 0U && f.r.liftoff_frame < 100U);
  CHECK(f.r.max_deg_settled < 4.0);
  bool saw_first_stage_alone = false;
  bool saw_second_stage_alone = false;
  double mass_before_fairing = 0.0;
  double mass_after_fairing = 0.0;
  for (const sim::TraceRow& row : f.trace) {
    saw_first_stage_alone = saw_first_stage_alone || (row.stages_active == 3U && row.stages_ignited == 1U);  // both on the vehicle, only the first lit
    saw_second_stage_alone = saw_second_stage_alone || (row.stages_active == 2U && row.stages_ignited == 3U);  // the first gone, the second lit
    if (near_abs(row.t, 209.0, 0.01)) {
      mass_before_fairing = row.mass;
    }
    if (near_abs(row.t, 212.0, 0.01)) {
      mass_after_fairing = row.mass;
    }
  }
  CHECK(saw_first_stage_alone && saw_second_stage_alone);
  CHECK(mass_before_fairing - mass_after_fairing > 2650.0 && mass_before_fairing - mass_after_fairing < 2800.0);  // the 1900 kg fairing and 3 s of the second stage's 272 kg/s: 2717 kg
  CHECK(f.trace.back().altitude > 150000.0 && f.trace.back().speed > 2000.0);
}

TFC_TEST(example_the_launcher_with_dynamics_flies_through_staging_with_a_servo_that_rings_slosh_a_bending_mode_and_a_separation_that_twists) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/launcher_dynamics.json", v, errors) && errors.empty());
  const sim::VehicleSpec& g = v.params.spec;
  CHECK(g.actuator.order == 2 && g.actuator.backlash_deg > 0.0 && g.jet_damping && g.roll.enabled && g.flex.enabled && g.stages[0].tanks[0].slosh.enabled && g.stages[0].tipoff_roll_dps != 0.0);
  const Flown f = fly_example("launcher_dynamics.json", 22000U, true);  // 220 s: past the separation at 166 s
  CHECK(f.loaded && f.r.finite && !f.r.crashed && f.r.safe_frames == 0U && f.r.liftoff_frame < 100U);
  CHECK(f.r.max_deg_settled < 4.0);
  CHECK(f.trace.back().altitude > 150000.0 && f.trace.back().speed > 2000.0);
}

// The bending mode and the flight software: the flight computers have no notch, and the gyros of the vehicle read the slope of the structure on top of its rate. With the slope at the IMU of
// the example's (assumed) 0.04 per metre the loop does not notice the mode (0.08 degree); with 0.5 per metre of the other sign it rings (2.2 degrees in 30 s, and 24 over the 400 s); with 1 per metre of the first sign it is lost. Which sign is the bad one
// is the model's output, not a rule: it depends on where the engines and the IMUs are on the mode shape. The numbers are the example's assumptions, not any vehicle's.
TFC_TEST(example_the_bending_mode_can_break_the_loop_the_flight_software_has_no_filter_for_it) {
  sim::VehicleFile base;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/launcher_dynamics.json", base, errors) && errors.empty());
  const sim::FlightTables tables = sim::flight_tables(base.params, base.plan);  // designed once, on the example: the vehicles flown below differ from it in one number
  const auto fly = [&](double slope_imu) {
    Flown f;
    f.loaded = true;
    sim::Params flown = base.params;
    flown.spec.flex.slope_imu = slope_imu;
    sim::Loop lp;
    lp.cfg.params = flown;
    lp.cfg.design = base.params;
    lp.cfg.scenario = base.scenario;
    lp.cfg.plan = base.plan;
    lp.cfg.tables = &tables;
    lp.cfg.vehicle_true = true;
    lp.estimator.use_accel = false;
    lp.frames = 3000U;  // 30 s: the loop that is going to ring or be lost has done so
    f.r = sim::run(lp);
    return f;
  };
  const Flown quiet = fly(-0.04);
  const Flown rings = fly(0.5);
  const Flown lost = fly(-1.0);
  CHECK(quiet.loaded && quiet.r.finite && quiet.r.max_deg_settled < 1.0);
  CHECK(rings.r.max_deg_settled > 1.5);
  CHECK(lost.r.max_deg_settled > 20.0 || !lost.r.finite || lost.r.safe_frames > 0U);
}

TFC_TEST(example_the_sounding_rocket_steers_with_fins_and_no_gimbal) {
  const Flown f = fly_example("sounding_rocket.json", 6000U, false);
  CHECK(f.loaded && f.r.finite && !f.r.crashed && f.r.safe_frames == 0U && f.r.liftoff_frame < 50U);
  CHECK(f.r.max_deg_settled < 2.5);
  double max_q = 0.0;
  for (const sim::TraceRow& row : f.trace) {
    max_q = std::fmax(max_q, row.dynamic_pressure);
  }
  CHECK(max_q > 150000.0 && f.trace.back().altitude > 30000.0);  // a sounding rocket: max-Q in the hundreds of kilopascals, tens of kilometres up
}

TFC_TEST(example_the_spacecraft_slews_in_orbit_with_wheels_and_thrusters) {
  const Flown f = fly_example("spacecraft.json", 30000U, false);
  CHECK(f.loaded && f.r.finite && !f.r.crashed && f.r.safe_frames == 0U);
  CHECK(f.r.max_deg_settled < 1.0);
  CHECK(near_abs(f.trace.back().speed, std::sqrt(sim::kEarthMu / (sim::kEarthR + 500000.0)), 1.0));  // a circular orbit stays circular
  double max_tilt = 0.0;
  for (const sim::TraceRow& row : f.trace) {
    max_tilt = std::fmax(max_tilt, row.tilt_y_deg);
  }
  CHECK(max_tilt > 29.0 && max_tilt < 31.0);  // the slew reached its 30 degrees
  CHECK(f.trace.back().mass < 800.0);         // and the thrusters used some propellant
}

TFC_TEST(vehicle_reader_starts_a_ring_at_the_angle_given) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  const std::string text = R"({
    "stages": [{"dry_mass_kg": 1000, "length_m": 6, "radius_m": 0.5, "x_cg_dry_m": 3,
                "tanks": [{"propellant_kg": 4000, "x_bottom_m": 0.5, "radius_m": 0.5, "density_kg_m3": 1000}]}],
    "engines": [{"stage": 0, "count": 4, "ring_radius_m": 1.0, "ring_start_deg": 90, "thrust_vac_n": 1000, "isp_vac_s": 300}]
  })";
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const std::vector<sim::EngineSpec>& e = v.params.spec.engines;
  CHECK(e.size() == 4U && near_abs(e[0].pos.y, 0.0, 1e-12) && near_abs(e[0].pos.z, 1.0, 1e-12));   // starting at 90 degrees from +Y toward +Z
  CHECK(near_abs(e[1].pos.y, -1.0, 1e-12) && near_abs(e[1].pos.z, 0.0, 1e-12));                     // then a quarter turn on
}
