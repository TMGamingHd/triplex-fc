// SPDX-License-Identifier: MIT
// What the vehicle description files must refuse and what they must carry unchanged: every kind of malformed JSON, every kind of malformed vehicle, and a vehicle that uses every feature at once
// (it must read, write and read again as the same text). The refusals are the point of a strict reader: a typo or a bad number must never fly a different vehicle from the one the file meant.
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "tfc_test.hpp"

#include "json.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

bool has(const std::vector<std::string>& messages, const std::string& text) {
  for (const std::string& m : messages) {
    if (m.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

// A vehicle that reads: one stage with a tank, one engine.
const char* kBase = R"({
  "stages": [{"dry_mass_kg": 1000, "length_m": 6, "radius_m": 0.5, "x_cg_dry_m": 3,
              "tanks": [{"propellant_kg": 4000, "x_bottom_m": 0.5, "radius_m": 0.5, "density_kg_m3": 1000}],
              "throttle": [[0, 1], [10, 0.5]]}],
  "engines": [{"stage": 0, "thrust_vac_n": 100000, "isp_vac_s": 300, "gimbal": true, "position_m": [0, 0, 0]}]
})";

// The text with the first occurrence of `from` replaced by `to`; read it; do the errors mention `what`?
bool refused(const std::string& from, const std::string& to, const char* what) {
  std::string t = kBase;
  const std::size_t at = t.find(from);
  if (at == std::string::npos) {
    return false;
  }
  t.replace(at, from.size(), to);
  sim::VehicleFile v;
  std::vector<std::string> e;
  return !sim::read_vehicle(t, v, e) && has(e, what);
}

}  // namespace

TFC_TEST(refusals_malformed_json_is_refused_with_a_reason_and_valid_json_with_every_escape_is_read) {
  const std::vector<std::pair<std::string, const char*>> bad = {
      {"-", "a number needs a digit"},
      {"1e", "a digit must follow the exponent"},
      {"1e+", "a digit must follow the exponent"},
      {"\"a\nb\"", "cannot run over a line"},
      {"\"abc", "not closed"},
      {"\"abc\\", "not closed"},
      {"\"\\u12\"", "four hexadecimal digits"},
      {"\"\\q\"", "unknown escape"},
      {"{\"a\" 1}", "expected ':'"},
      {"{\"a\":", "unexpected end of the file"},
      {"{", "expected a quoted name"},
      {"[1, ", "unexpected end of the file"},
      {"tru", "expected true, false or null"},
      {"[1 2]", "expected ',' or ']'"},
      {"@", "unexpected character"},
      {"{\"a\": 1} {", "after the end"},
      {"[1,]", "unexpected character ']'"},
      {"{\"a\": 1.}", "digit must follow the decimal point"},
  };
  for (const auto& b : bad) {
    sim::Json j;
    std::string err;
    CHECK(!sim::parse_json(b.first, j, err) && err.find(b.second) != std::string::npos);
  }
  sim::Json j;
  std::string err;
  CHECK(sim::parse_json("\"\\\\ \\/ \\b \\f \\n \\r \\t \\\" \\u0041 \\u00e9\"", j, err));
  CHECK(j.text == "\\ / \b \f \n \r \t \" A ?");  // the escapes, and a character beyond ASCII becomes a placeholder
  CHECK(sim::parse_json("[1.5e+3, -0, 0.5E-2, 1E2]", j, err) && j.items.size() == 4U && near_abs(j.items[0].number, 1500.0, 1e-9) && near_abs(j.items[2].number, 0.005, 1e-12));
  CHECK(sim::parse_json("[]", j, err) && j.items.empty() && sim::parse_json("{}", j, err) && j.fields.empty());
}

TFC_TEST(refusals_every_wrong_type_in_a_vehicle_file_is_named) {
  CHECK(refused("\"stage\": 0", "\"stage\": 1.5", "expected a whole number"));
  CHECK(refused("\"gimbal\": true", "\"gimbal\": \"yes\"", "expected true or false"));
  CHECK(refused("\"position_m\": [0, 0, 0]", "\"position_m\": [0, 0]", "expected three numbers"));
  CHECK(refused("\"throttle\": [[0, 1], [10, 0.5]]", "\"throttle\": \"x\"", "expected a list of [time, value] pairs"));
  CHECK(refused("\"throttle\": [[0, 1], [10, 0.5]]", "\"throttle\": [[1]]", "expected a pair of numbers"));
  CHECK(refused("\"tanks\": [", "\"tanks\": 5, \"xtanks\": [", "expected a list"));
  CHECK(refused("\"thrust_vac_n\": 100000", "\"thrust_vac_n\": \"big\"", "expected a number"));
  CHECK(refused("\"engines\": [{", "\"engines\": [7, {", "expected an object"));
  CHECK(refused("\"gimbal\": true", "\"gimbal\": true, \"control\": \"up\"", "expected \"none\""));
  CHECK(refused("\"engines\": [{\"stage\": 0,", "\"engines\": [{\"stage\": 0, \"count\": 0,", "count: expected 1 to"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"scenario\": {\"gusts\": [1]},\n  \"stages\"", "expected an object"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"scenario\": {\"engine_failures\": [2]},\n  \"stages\"", "expected an object"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"aero\": {\"table\": 3},\n  \"stages\"", "expected a list of [mach, ca, cn_alpha, x_cp_m]"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"aero\": {\"table\": [[1, 2]]},\n  \"stages\"", "expected four numbers"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"design\": {\"program\": 4},\n  \"stages\"", "expected a list of [time, value] pairs"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"scenario\": {\"start\": 1},\n  \"stages\"", "expected an object"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"wheels\": {\"stage\": 0},\n  \"stages\"", "wheels:"));
  CHECK(refused("{\n  \"stages\"", "{\n  \"stages_typo\": 1,\n  \"stages\"", "unknown field"));
  // a file that is not JSON at all, and one that is not there
  sim::VehicleFile v;
  std::vector<std::string> e;
  CHECK(!sim::read_vehicle("{ not json", v, e) && has(e, "line 1"));
  e.clear();
  CHECK(!sim::load_vehicle_file("/nonexistent/vehicle.json", v, e) && has(e, "cannot open the file"));
}

TFC_TEST(refusals_every_check_of_the_description_names_its_field) {
  const auto base = [] {
    sim::VehicleSpec s;
    sim::StageSpec st;
    st.dry_mass = 1000.0;
    st.length = 6.0;
    st.radius = 0.5;
    st.tanks.push_back(sim::TankSpec{4000.0, 0.5, 0.5, 1000.0});
    s.stages.push_back(st);
    s.engines.push_back(sim::EngineSpec{});
    return s;
  };
  CHECK(sim::validate(base()).empty());
  const auto check = [&](const std::function<void(sim::VehicleSpec&)>& mutate, const char* what) {
    sim::VehicleSpec s = base();
    mutate(s);
    return has(sim::validate(s), what);
  };
  CHECK(check([](sim::VehicleSpec& s) { s.stages.assign(sim::kMaxStages + 1U, s.stages[0]); }, "stages: at most"));
  CHECK(check([](sim::VehicleSpec& s) { s.payloads.assign(sim::kMaxPayloads + 1U, sim::PayloadSpec{}); }, "payloads: at most"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].length = 0.0; }, "stages[0].length must be positive"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].radius = -1.0; }, "stages[0].radius must be positive"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].inertia_factor = 0.0; }, "inertia_factor must be positive"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].tanks.assign(sim::kMaxTanks + 1U, s.stages[0].tanks[0]); }, "tanks: at most"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].tanks[0].propellant = -1.0; }, "propellant must not be negative"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].tanks[0].density = 0.0; }, "radius and .density must be positive"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].ignite_after_sep_of = 5; }, "an earlier stage"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages.push_back(s.stages[0]); s.stages[1].ignite_after_sep_of = 0; s.stages[1].ignite_delay_s = -1.0; }, "delays must not be negative"));
  CHECK(check([](sim::VehicleSpec& s) { s.engines[0].rise_s = -1.0; }, "must not be negative"));
  CHECK(check([](sim::VehicleSpec& s) { s.engines[0].stage = -1; }, "engines[0].stage must name a stage"));
  CHECK(check([](sim::VehicleSpec& s) { s.fins.assign(sim::kMaxFins + 1U, sim::FinSpec{}); }, "fins: at most"));
  CHECK(check([](sim::VehicleSpec& s) { s.fins.push_back(sim::FinSpec{}); s.fins[0].stage = 9; s.fins[0].area_each = 0.1; }, "fins[0].stage must name a stage"));
  CHECK(check([](sim::VehicleSpec& s) { s.wheels.enabled = true; s.wheels.stage = 3; s.wheels.torque_max = 1.0; s.wheels.momentum_max = 1.0; }, "wheels.stage must name a stage"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].sections.push_back(sim::SectionSpec{sim::SectionKind::Transition, 0.0, 1.0, 0.2, 0.2, sim::NoseShape::Cone}); }, "is a transition"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].sections.push_back(sim::SectionSpec{sim::SectionKind::Tube, 0.0, -1.0, 0.2, 0.2, sim::NoseShape::Cone}); }, "length_m must be positive"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].stabilizers.push_back(sim::FinPlanform{}); }, "stabilizers[0]"));
  CHECK(check([](sim::VehicleSpec& s) { sim::PayloadSpec p; p.sections.push_back(sim::SectionSpec{sim::SectionKind::Nose, 0.0, 1.0, 0.1, 0.2, sim::NoseShape::Cone}); s.payloads.push_back(p); }, "is a nose"));
  CHECK(check([](sim::VehicleSpec& s) { s.aero.power_on_base = 2.0; }, "power_on_base lies between"));
  CHECK(check([](sim::VehicleSpec& s) { s.stages[0].throttle = {{1.0, 2.0}}; }, "throttle"));
}

TFC_TEST(refusals_a_vehicle_that_uses_every_feature_reads_writes_and_reads_again_unchanged) {
  const std::string text = R"({
    "name": "kitchen sink", "description": "every field once, \"quoted\" and with a backslash \\",
    "vehicle": {"gimbal_limit_deg": 6, "gimbal_rate_dps": 40, "gimbal_lag_s": 0.02, "ideal_roll_control": false, "ground_contact": true, "crash_speed_ms": 4,
                "max_substep_s": 0.001, "thrust_scale": 1.01, "cd_scale": 1.1, "cn_scale": 0.9, "thrust_misalign_pitch_deg": 0.1, "thrust_misalign_yaw_deg": -0.1},
    "aero": {"diameter_m": 0.5, "c_n_alpha": 3, "x_cp_m": 2, "reference_diameter_m": 0.5, "crossflow_cd": 1.1, "crossflow_eta": 0.6, "rear_axial": 1.0, "power_on_base": 0.5,
             "roughness": 1.2, "full_angle": false},
    "stages": [
      {"name": "first", "dry_mass_kg": 500, "x_start_m": 0, "length_m": 5, "radius_m": 0.25, "inertia_factor": 0.5, "x_cg_dry_m": 2,
       "tanks": [{"propellant_kg": 200, "x_bottom_m": 0.5, "radius_m": 0.2, "density_kg_m3": 800}, {"propellant_kg": 100, "x_bottom_m": 2.5, "radius_m": 0.2, "density_kg_m3": 1100}],
       "sections": [{"kind": "tube", "x_start_m": 0, "length_m": 3, "d_aft_m": 0.5, "d_fore_m": 0.5}, {"kind": "transition", "x_start_m": 3, "length_m": 1, "d_aft_m": 0.5, "d_fore_m": 0.3}],
       "stabilizers": [{"count": 3, "x_le_root_m": 0.8, "root_chord_m": 0.3, "tip_chord_m": 0.1, "span_m": 0.2, "sweep_m": 0.1, "thickness_m": 0.004}],
       "sequential_drain": true, "ignite_time_s": 0.5, "ignite_after_sep_of": -1, "ignite_delay_s": 0, "separate_time_s": 40, "separate_on_burnout": true, "separate_delay_s": 1,
       "gimbal_limit_deg": 4, "gimbal_rate_dps": 30, "gimbal_lag_s": 0.01, "throttle": [[0, 1], [20, 0.6]]},
      {"name": "second", "dry_mass_kg": 100, "x_start_m": 5, "length_m": 2, "radius_m": 0.25, "x_cg_dry_m": 6,
       "tanks": [{"propellant_kg": 50, "x_bottom_m": 5.2, "radius_m": 0.2, "density_kg_m3": 1000}],
       "sections": [{"kind": "tube", "x_start_m": 5, "length_m": 1, "d_aft_m": 0.3, "d_fore_m": 0.3}, {"kind": "nose", "x_start_m": 6, "length_m": 1, "d_aft_m": 0.3, "d_fore_m": 0, "shape": "parabola"}],
       "ignite_after_sep_of": 0, "ignite_delay_s": 3}
    ],
    "engines": [
      {"stage": 0, "count": 2, "position_m": [0, 0.1, 0], "ring_radius_m": 0.2, "ring_start_deg": 45, "thrust_vac_n": 20000, "exit_area_m2": 0.02, "isp_vac_s": 280, "cant_pitch_deg": 0.5,
       "cant_yaw_deg": -0.5, "start_offset_s": 0.1, "cutoff_time_s": 39, "rise_s": 0.2, "tail_s": 0.3},
      {"stage": 1, "position_m": [5, 0, 0], "thrust_vac_n": 5000, "isp_vac_s": 320, "gimbal": false},
      {"stage": 1, "position_m": [5, 0, 0], "direction": [0, 1, 0], "gimbal": false, "control": "pitch-", "full_cmd_deg": 3, "thrust_vac_n": 10, "isp_vac_s": 60},
      {"stage": 1, "position_m": [5, 0, 0], "direction": [0, 0, 1], "gimbal": false, "control": "yaw-", "full_cmd_deg": 3, "thrust_vac_n": 10, "isp_vac_s": 60}
    ],
    "payloads": [{"name": "fairing", "mass_kg": 20, "x_m": 6.5, "jettison_time_s": 30, "sections": [{"kind": "nose", "x_start_m": 7, "length_m": 1, "d_aft_m": 0.3, "d_fore_m": 0, "shape": "ellipse"}]}],
    "fins": [{"name": "control", "stage": 0, "x_hinge_m": 0.5, "area_each_m2": 0.02, "lift_slope": 2.5, "gain": -1, "limit_deg": 12, "rate_dps": 80, "lag_s": 0.03}],
    "wheels": {"stage": 1, "torque_max_nm": 0.1, "momentum_max_nms": 2, "full_cmd_deg": 2},
    "scenario": {"wind_scale": 0.5, "wind_direction": [0, 1, 0], "dry_cg_shift_m": 0.1, "gusts": [{"t0_s": 10, "duration_s": 2, "peak_ms": [0, 0, 5]}],
                 "engine_failures": [{"time_s": 12, "engine": 1}], "start": {"position_m": [6800000, 0, 0], "velocity_ms": [0, 7500, 0], "rates_dps": [0, 0.5, 0]}},
    "design": {"t_end_s": 80, "dt_s": 0.01, "vertical_s": 5, "kick_ramp_s": 2, "kick_deg": 0.5, "follow_from_s": 10, "blend_s": 2, "program": [[0, 0], [30, 20]],
               "gains": {"every_s": 0, "wn": 2, "zeta": 0.7, "ki_over_kp": 0.1, "kp_max": 20, "b_min": 0.5, "tolerance": 0.2}}
  })";
  sim::VehicleFile a;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, a, errors) && errors.empty());
  CHECK(a.name == "kitchen sink" && a.description == "every field once, \"quoted\" and with a backslash \\");
  CHECK(a.params.spec.engines.size() == 5U && a.params.spec.engines[4].control == sim::Control::YawMinus && a.params.spec.engines[3].control == sim::Control::PitchMinus);
  CHECK(a.params.spec.fins.size() == 1U && a.params.spec.fins[0].gain == -1.0 && a.params.spec.wheels.enabled && a.params.spec.wheels.stage == 1);
  CHECK(a.params.spec.stages[0].sequential_drain && a.params.spec.stages[0].throttle.size() == 2U && a.params.spec.payloads[0].sections.size() == 1U);
  CHECK(a.scenario.gusts.size() == 1U && a.scenario.engine_failures.size() == 1U && a.scenario.has_initial && near_abs(a.scenario.initial.w.y, 0.5 * sim::kDeg2Rad, 1e-12));
  CHECK(a.plan.trajectory.table.size() == 2U && a.plan.gains.every_s == 0.0 && a.plan.gains.b_min == 0.5 && !a.params.spec.aero.full_angle);
  const std::string once = sim::write_vehicle(a);
  sim::VehicleFile b;
  CHECK(sim::read_vehicle(once, b, errors) && errors.empty());
  CHECK(sim::write_vehicle(b) == once);  // nothing is lost or invented by a round trip
  CHECK(b.params.spec.engines.size() == 5U && near_abs(b.params.spec.engines[0].pos.y, 0.1 + (0.2 * std::cos(45.0 * sim::kDeg2Rad)), 1e-12));
  // and the single failure of the scenario's own fields is written as a failure, and read back as one
  sim::VehicleFile c = a;
  c.scenario.engine_out_time = 25.0;
  c.scenario.engine_out_index = 2;
  sim::VehicleFile d;
  CHECK(sim::read_vehicle(sim::write_vehicle(c), d, errors) && d.scenario.engine_failures.size() == 2U && d.scenario.engine_failures[0].time == 25.0 && d.scenario.engine_failures[0].index == 2);
  // a vehicle with a table by Mach instead of a shape
  sim::VehicleFile t;
  CHECK(sim::read_vehicle(R"({"aero": {"table": [[0, 0.3, 2, 1], [2, 0.5, 3, 1.5]]}, "stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}]})", t, errors) && errors.empty());
  const std::string tt = sim::write_vehicle(t);
  sim::VehicleFile u;
  CHECK(sim::read_vehicle(tt, u, errors) && sim::write_vehicle(u) == tt && u.params.spec.aero.table.size() == 2U);
}
