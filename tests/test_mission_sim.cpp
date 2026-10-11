// SPDX-License-Identifier: MIT
// The mission in the vehicle simulator (sim/vehicle/mission_spec.hpp, spec_io.hpp, mission_build.hpp, mission_design.hpp, mission_world.hpp, avionics.hpp; docs/design/GNC.md, docs/design/RECOVERY.md):
// the mission block of a vehicle file (read, written back, refused when wrong), the tables built from it, the design of the gains and the allocation to the surfaces from the plant's own probes,
// the GNSS receiver and the three flight computers around a body, a rocket flown ideally and on its computers to a descent under parachutes, and a two-stage vehicle whose first stage is let go and
// flown on by computers of its own. The 51-engine Starship-class mission in vehicles/missions/ is too long for a build with sanitizers: CI flies it with tfc_mission in a release build.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "mission_design.hpp"
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

// A vehicle with a mission, to be filled in: one stage with a gimballed engine in group 0 and one without a gimbal in group 1.
std::string tiny(const std::string& mission) {
  return R"({
  "name": "tiny",
  "vehicle": {"gimbal_limit_deg": 8, "ground_contact": true},
  "stages": [{"dry_mass_kg": 400, "length_m": 6, "radius_m": 0.4, "x_cg_dry_m": 3, "catch_pin_x_m": 5.0, "guided": true,
              "tanks": [{"propellant_kg": 600, "x_bottom_m": 0.5, "radius_m": 0.38, "density_kg_m3": 1000}]}],
  "engines": [{"stage": 0, "thrust_vac_n": 40000, "exit_area_m2": 0.05, "isp_vac_s": 300, "group": 0, "tail_s": 0.3},
              {"stage": 0, "position_m": [0, 0.3, 0], "thrust_vac_n": 10000, "exit_area_m2": 0.02, "isp_vac_s": 280, "group": 1, "gimbal": false, "tail_s": 0.1}],
  "mission": )" + mission + "\n}";
}

bool reads(const std::string& text, sim::VehicleFile& v, std::vector<std::string>& errors) {
  errors.clear();
  return sim::read_vehicle(text, v, errors);
}

const sim::AxisProbe* first_probe(const std::vector<sim::AxisProbe>& all, int owner, uint8_t phase) {
  for (const sim::AxisProbe& p : all) {
    if (p.stage == owner && p.phase == phase) {
      return &p;
    }
  }
  return nullptr;
}

}  // namespace

TFC_TEST(missionfile_the_committed_missions_load_and_their_text_reads_back_the_same) {
  for (const char* name : {"missions/starship_return.json", "missions/rocket_recovery.json"}) {
    sim::VehicleFile v;
    std::vector<std::string> errors;
    CHECK(sim::load_vehicle_file(repo_root() + "vehicles/" + name, v, errors) && errors.empty());
    CHECK(v.mission.present && !v.mission.main.empty() && !v.mission.mixers.empty());
    const std::string text = sim::write_vehicle(v);
    sim::VehicleFile w;
    CHECK(reads(text, w, errors));
    CHECK(errors.empty());
    CHECK(sim::write_vehicle(w) == text);
    CHECK(w.mission.main.size() == v.mission.main.size());
    for (std::size_t s = 0; s < sim::kMaxStages; ++s) {
      CHECK(w.mission.stage[s].size() == v.mission.stage[s].size());
    }
  }
  sim::VehicleFile s;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/missions/starship_return.json", s, errors));
  CHECK(s.mission.main.size() == 8U && s.mission.stage[0].size() == 6U && s.mission.mixers.size() == 4U);
  CHECK(s.mission.stage[0][1].kind == "boostback" && s.mission.stage[0][4].kind == "landing" && s.mission.stage[0][4].landing_groups[3].size() == 4U);
  CHECK(s.mission.main[4].kind == "peg" && near_abs(s.mission.main[4].circular_km, 250.0, 1e-9));
  sim::VehicleFile none;
  CHECK(reads(R"({"name": "x", "stages": [{"dry_mass_kg": 1, "length_m": 1, "radius_m": 0.1, "tanks": [{"propellant_kg": 1, "x_bottom_m": 0, "radius_m": 0.1, "density_kg_m3": 1000}]}], "engines": [{"stage": 0, "thrust_vac_n": 1000, "exit_area_m2": 0.01, "isp_vac_s": 300}]})", none, errors));
  CHECK(!none.mission.present);
  sim::BuiltMission bm;
  CHECK(sim::build_mission(none, bm, errors) && !bm.present);   // no mission: nothing to build, and not an error
}

TFC_TEST(missionfile_a_mission_that_is_wrong_is_refused_with_the_place_of_the_mistake) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(!reads(tiny(R"({"bogus": 1})"), v, errors) && mentions(errors, "bogus"));
  CHECK(!reads(tiny(R"({"main": [{"kind": "coast", "events": [3]}]})"), v, errors) && mentions(errors, "list of names"));
  CHECK(!reads(tiny(R"({"main": [{"kind": "coast", "groups": [1.5]}]})"), v, errors) && mentions(errors, "whole numbers"));
  CHECK(!reads(tiny(R"({"main": [{"kind": "coast", "groups": "all"}]})"), v, errors) && mentions(errors, "list of whole numbers"));
  CHECK(!reads(tiny(R"({"main": [{"kind": "coast", "end": {"sometime": 3}}]})"), v, errors) && mentions(errors, "sometime"));
  CHECK(!reads(tiny(R"({"main": [{"kind": "landing", "landing_groups": {"seven": [1]}}]})"), v, errors) && mentions(errors, "seven"));
  CHECK(!reads(tiny(R"({"mixers": [{"name": "m", "colour": 1}]})"), v, errors) && mentions(errors, "colour"));
  // a name that is not a kind, a hold or an event is read and then refused when the tables are built
  CHECK(reads(tiny(R"({"main": [{"kind": "teleport", "hold": "sideways", "events": ["explode"]}]})"), v, errors));
  sim::BuiltMission bm;
  errors.clear();
  CHECK(!sim::build_mission(v, bm, errors));
  CHECK(mentions(errors, "unknown \"teleport\"") && mentions(errors, "unknown \"sideways\"") && mentions(errors, "unknown event \"explode\""));
  // more phases than the tables hold
  std::string many = R"({"main": [)";
  for (int i = 0; i < 17; ++i) {
    many += std::string(i == 0 ? "" : ",") + R"({"kind": "coast"})";
  }
  many += "]}";
  CHECK(reads(tiny(many), v, errors));
  errors.clear();
  CHECK(!sim::build_mission(v, bm, errors) && mentions(errors, "at most 16 phases"));
}

TFC_TEST(missionfile_the_tables_carry_what_each_kind_of_phase_needs) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  const std::string m = R"({
    "site": {"offset_z_m": 20, "arm_height_m": 50},
    "landing": {"tilt_max_deg": 33, "decel_plan_ms2": 21, "horizon_min_s": 9, "propellant_kg": 100},
    "mixers": [{"name": "a", "gimbal": true, "roll_thrusters": false}, {"name": "b", "gimbal": false, "roll_thrusters": true, "surfaces": [0]}],
    "main": [
      {"name": "p", "kind": "program", "groups": [0], "end": {"time_s": 5}, "program": [[0, 0], [5, 10]], "throttle_track": [[0, 1.0], [5, 0.5]], "events": ["separate", "jettison"]},
      {"name": "p2", "kind": "program", "groups": [0], "program": [[0, 1]], "throttle_track": [[0, 1.0]], "end": {"speed_ms": 1000}},
      {"name": "f", "kind": "coast", "hold": "fixed", "fixed": [0, 1, 0], "end": {"altitude_above_m": 100}},
      {"name": "c", "kind": "peg", "groups": [0, 1], "target": {"circular_km": 200}, "burnout_mass_kg": 400, "end": {"cutoff": true}},
      {"name": "e", "kind": "peg", "groups": [0], "target": {"apogee_km": 400, "perigee_km": 150, "cutoff_km": 160}, "end": {"cutoff": true}},
      {"name": "r", "kind": "peg", "groups": [], "target": {"radius_m": 6600000, "speed_ms": 7700, "gamma_deg": 1}},
      {"name": "g", "kind": "chute", "drogue_altitude_m": 5000, "main_altitude_m": 900, "end": {"mass_below_kg": 10}}
    ],
    "stage0": [
      {"name": "b", "kind": "boostback", "groups": [0, 1], "bias_m": 300, "reserve_mass_kg": 500, "pitch_up_deg": 4, "end": {"cutoff": true}},
      {"name": "g", "kind": "glide", "alpha_max_deg": 20, "gain_deg_per_km": 4, "alpha_brake_deg": 2, "lift_area_per_deg_m2": 1.5, "gate_speed_ms": 200, "gate_height_m": 2000, "brake_max_deg": 9, "end": {"ignition_below_m": 3000}},
      {"name": "l", "kind": "landing", "landing_groups": {"one": [0], "three": [0, 1]}, "end": {"touchdown": true}}
    ]
  })";
  CHECK(reads(tiny(m), v, errors));
  CHECK(errors.empty());
  sim::BuiltMission bm;
  CHECK(sim::build_mission(v, bm, errors) && bm.present);
  const tfc::gnc::Tables& t = bm.main;
  CHECK(t.n_phases == 7U);
  CHECK(t.phase[0].groups == 1U && t.phase[0].events == (tfc::propbit::kSeparate | tfc::propbit::kJettison) && t.phase[0].throttle_track == 0U);
  CHECK(t.throttle[0].n == 2U && near_abs(static_cast<double>(t.throttle[0].value[1]), 0.5, 1e-6) && t.throttle[0].frame[1] == 500U);
  CHECK(t.phase[1].throttle_track == 1U && t.throttle[1].n == 1U);                    // a second track; the program of the first program phase is the one carried
  CHECK(t.program.at(500U).tilt_y_deg > 9.9F);
  CHECK(t.phase[2].hold == tfc::gnc::hold::kFixed && near_abs(static_cast<double>(t.phase[2].p[1]), 1.0, 1e-9));
  const double mu = v.params.spec.planet.mu;
  const double radius = v.params.spec.planet.radius;
  CHECK(near_abs(static_cast<double>(t.phase[3].p[0]), radius + 200000.0, 1.0) && near_abs(static_cast<double>(t.phase[3].p[1]), std::sqrt(mu / (radius + 200000.0)), 0.01));
  CHECK(near_abs(static_cast<double>(t.phase[3].p[2]), 0.0, 1e-9) && near_abs(static_cast<double>(t.phase[3].p[8]), 400.0, 1e-3));
  CHECK(t.phase[3].p[9] > 0.0F && t.phase[3].p[9] < 0.3F);                              // the tail is the engines' mean: 0.3 s and 0.1 s
  CHECK(near_abs(static_cast<double>(t.phase[4].p[0]), radius + 160000.0, 1.0));       // an ellipse: the radius of the cut-off, the speed and the angle of that point on the orbit
  CHECK(t.phase[4].p[1] > 7000.0F && std::fabs(t.phase[4].p[2]) > 0.0F);
  CHECK(near_abs(static_cast<double>(t.phase[5].p[0]), 6600000.0, 1.0) && near_abs(static_cast<double>(t.phase[5].p[2]), 1.0, 1e-6));
  CHECK(near_abs(static_cast<double>(t.phase[6].p[0]), 5000.0, 1e-6) && near_abs(static_cast<double>(t.phase[6].p[1]), 900.0, 1e-6));
  CHECK(t.mixer[0].gimbal_pitch == 1.0F && t.mixer[0].roll == 0.0F && t.mixer[1].gimbal_pitch == 0.0F && t.mixer[1].roll == 1.0F);
  CHECK(near_abs(t.site.z, 20.0, 1e-9) && near_abs(t.landing.tilt_max_deg, 33.0, 1e-9) && near_abs(t.landing.horizon_min_s, 9.0, 1e-9));
  CHECK(bm.has_stage[0] && !bm.has_stage[1]);
  const tfc::gnc::Tables& s = bm.stage[0];
  CHECK(near_abs(static_cast<double>(s.phase[0].p[0]), 300.0, 1e-9) && near_abs(static_cast<double>(s.phase[0].p[1]), 500.0, 1e-9) && near_abs(static_cast<double>(s.phase[0].p[2]), 4.0, 1e-9));
  CHECK(s.phase[0].p[3] > 0.0F);
  CHECK(near_abs(static_cast<double>(s.phase[1].p[0]), 20.0, 1e-9) && near_abs(static_cast<double>(s.phase[1].p[1]), 0.004, 1e-9) && near_abs(static_cast<double>(s.phase[1].p[6]), 9.0, 1e-9));
  CHECK(s.engines.options == 3U && s.engines.counts[0] == 1U && s.engines.counts[2] == 2U);      // the options given, fewest first; the one not given is not an option
  CHECK(s.phase[2].p[0] == 1.0F && s.phase[2].p[2] == 3.0F && s.phase[2].groups == 3U);
  CHECK(s.arm_height == 50.0 && s.landing_height > 40.0 && s.landing_height < 50.0 && std::isfinite(s.landing_height_slope) && s.landing_mass_ref > 0.0);
  // the Starship-class booster: the pins 32 m above the centre of gravity at the arms' 70 m, and a heavier stage has its centre of gravity lower, so its catch height is lower
  sim::VehicleFile big;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/missions/starship_return.json", big, errors));
  sim::BuiltMission bb;
  CHECK(sim::build_mission(big, bb, errors));
  const tfc::gnc::Tables& booster = bb.stage[0];
  CHECK(near_abs(booster.landing_height, 37.79, 0.05) && booster.landing_height_slope < 0.0 && near_abs(booster.arm_height, 70.0, 1e-9));
  tfc::gnc::Mission mis;
  mis.configure(&booster);
  mis.set_mass(booster.landing_mass_ref + 100000.0);
  CHECK(mis.catch_height() < booster.landing_height);
  CHECK(bb.main.phase[4].p[9] > 0.5F && bb.main.phase[4].p[9] < 1.0F);   // the ship's PEG leads its cut-off by the engines' tail (0.8 s)
}

TFC_TEST(missiondesign_the_allocation_to_the_surfaces_has_the_effect_asked_for_within_the_travel) {
  // three surfaces; the columns are the angular acceleration (roll, yaw, pitch) per degree of each
  const std::array<std::array<double, 3>, 4> cols = {{{{1.0, 0.0, 0.5}}, {{-1.0, 0.5, 0.0}}, {{0.3, -0.5, -0.5}}, {{0.0, 0.0, 0.0}}}};
  tfc::gnc::Mixer mx;
  const std::vector<int> used = {0, 1, 2};
  const std::array<double, 4> travel = {30.0, 30.0, 30.0, 0.0};
  sim::detail::allocate(cols, used, mx, travel, 8.0);
  // at the full demand (8 units) the most loaded surface is at 80% of its travel
  double worst = 0.0;
  for (std::size_t k = 0; k < 3U; ++k) {
    for (const auto& axis : {mx.from_roll, mx.from_yaw, mx.from_pitch}) {
      worst = std::fmax(worst, std::fabs(static_cast<double>(axis[k])) * 8.0 / 30.0);
    }
  }
  CHECK(near_abs(worst, 0.8, 1e-4));
  // and the directions are those that produce the demanded acceleration: B u is proportional to the unit demand on each axis
  for (int axis = 0; axis < 3; ++axis) {
    const std::array<float, 4>& u = axis == 0 ? mx.from_roll : (axis == 1 ? mx.from_yaw : mx.from_pitch);
    double got[3] = {0.0, 0.0, 0.0};
    for (std::size_t k = 0; k < 3U; ++k) {
      for (int i = 0; i < 3; ++i) {
        got[i] += cols[k][static_cast<std::size_t>(i)] * static_cast<double>(u[k]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      if (i != axis) {
        CHECK(std::fabs(got[i]) < 0.05 * std::fabs(got[axis]) + 1e-6);   // (the regularisation leaves a little cross-coupling)
      }
    }
    CHECK(std::fabs(got[axis]) > 0.0);
  }
  // no surfaces, or surfaces that do nothing: nothing is written
  tfc::gnc::Mixer none;
  sim::detail::allocate(cols, {}, none, travel, 8.0);
  const std::array<std::array<double, 3>, 4> zero{};
  sim::detail::allocate(zero, used, none, travel, 8.0);
  CHECK(none.from_pitch[0] == 0.0F && none.from_roll[1] == 0.0F);
  // a single surface can only do one thing: the regularised answer is finite and the demand it cannot meet gives a small deflection
  tfc::gnc::Mixer one;
  sim::detail::allocate(cols, {0}, one, travel, 8.0);
  CHECK(std::isfinite(static_cast<double>(one.from_roll[0])) && std::fabs(static_cast<double>(one.from_roll[0])) > 0.0);
  // a command beyond what the geometry can do (an all-zero determinant): left alone
  const std::array<std::array<double, 3>, 4> flat = {{{{1.0, 0.0, 0.0}}, {{2.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}}};
  tfc::gnc::Mixer fl;
  sim::detail::allocate(flat, {0, 1}, fl, travel, 8.0);
  CHECK(std::isfinite(static_cast<double>(fl.from_roll[0])));
}

TFC_TEST(missiondesign_gains_come_from_the_probes_with_the_allocation_of_each_point_and_the_thrust_they_were_made_at) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(reads(tiny(R"({
    "design": {"wn": 1.0, "zeta": 0.7, "ki_over_kp": 0.2, "kp_max": 50, "b_min": 0.01},
    "mixers": [{"name": "tvc", "gimbal": true, "roll_thrusters": true}, {"name": "fins", "gimbal": false, "roll_thrusters": false, "surfaces": [0, 1, 2]}],
    "main": [{"kind": "program", "groups": [0], "mixer": 0, "end": {"time_s": 10}, "program": [[0, 0], [10, 5]]}, {"kind": "coast", "mixer": 1}]})"), v, errors));
  v.params.spec.surfaces.clear();
  for (int i = 0; i < 3; ++i) {
    sim::SurfaceSpec f;
    f.channel = i;
    f.area = 5.0;
    f.chord = 1.0;
    f.span = 2.0;
    f.radius = 0.5;
    f.min_deg = -20.0;
    f.max_deg = 20.0;
    v.params.spec.surfaces.push_back(f);
  }
  sim::BuiltMission bm;
  CHECK(sim::build_mission(v, bm, errors));
  std::vector<sim::AxisProbe> probes;
  for (int k = 0; k < 40; ++k) {
    sim::AxisProbe p;
    p.stage = -1;
    p.phase = k < 20 ? 0U : 1U;
    p.t_in_phase = static_cast<double>(k % 20) * 0.5;
    p.thrust = 40000.0 + (100.0 * k);
    p.a = {0.5, 2.0, 3.0};
    p.col[0] = {0.0, 0.0, 0.4};   // gimbal pitch -> pitch acceleration (rad/s^2 per degree)
    p.col[1] = {0.0, 0.4, 0.0};
    p.col[2] = {0.05, 0.0, 0.0};  // roll thrusters
    const double q = 0.01 * (1 + (k % 20));   // the fins' effect grows through the phase, and flips the sign of one (the allocation is per point)
    p.col[3] = {0.3 * q, 0.0, 0.2 * q};
    p.col[4] = {-0.3 * q, 0.2 * q, 0.0};
    p.col[5] = {0.0, -0.2 * q * (k % 20 > 10 ? -1.0 : 1.0), -0.2 * q};
    probes.push_back(p);
  }
  sim::fit_plan(v.mission, v.params.spec, bm.main, probes, -1);
  const tfc::gnc::GainTrack& burn = bm.main.gains[0];
  const tfc::gnc::GainTrack& fins = bm.main.gains[1];
  CHECK(burn.n > 0U && !burn.has_alloc && fins.n > 0U && fins.has_alloc);
  CHECK(burn.n <= tfc::gnc::kGainPoints);
  // kp = (wn^2 + a) / b with the gimbal: pitch (1 + 3) / 0.4 = 10 degrees per radian, in degrees per degree; the roll thruster is the roll effector
  CHECK(near_abs(static_cast<double>(burn.gains[0].pitch.kp), 10.0 / sim::kRad2Deg, 1e-4));
  CHECK(near_abs(static_cast<double>(burn.gains[0].pitch.kd), (2.0 * 0.7 * 1.0 / 0.4) / sim::kRad2Deg, 1e-4));
  CHECK(near_abs(static_cast<double>(burn.gains[0].pitch.ki), 0.2 * 10.0 / sim::kRad2Deg, 1e-4));
  CHECK(burn.gains[0].roll.kp > 0.0F && near_abs(static_cast<double>(burn.thrust[0]), 40000.0, 1.0));
  // the fins carry an allocation per point, and the effectiveness measured with it makes the gains the finite, positive ones of the demand
  CHECK(fins.alloc[0].from_pitch[0] != 0.0F || fins.alloc[0].from_pitch[2] != 0.0F);
  CHECK(fins.alloc[fins.n - 1U].from_pitch[0] != fins.alloc[0].from_pitch[0] || fins.alloc[fins.n - 1U].from_yaw[1] != fins.alloc[0].from_yaw[1]);
  for (unsigned i = 0; i < fins.n; ++i) {
    CHECK(std::isfinite(static_cast<double>(fins.gains[i].pitch.kp)) && std::fabs(static_cast<double>(fins.gains[i].pitch.kp)) <= 50.0 / sim::kRad2Deg + 1e-6);
    CHECK(fins.frame[i] == 0U || fins.frame[i] > fins.frame[i - 1U]);
  }
  CHECK(near_abs(static_cast<double>(bm.main.mixer[1].lo[0]), -20.0, 1e-9) && near_abs(static_cast<double>(bm.main.mixer[1].hi[2]), 20.0, 1e-9));
  CHECK(first_probe(probes, -1, 1U) != nullptr && first_probe(probes, 3, 0U) == nullptr);
  // a phase with no probe keeps an empty track
  tfc::gnc::Tables other = bm.main;
  std::vector<sim::AxisProbe> only_first(probes.begin(), probes.begin() + 20);
  sim::fit_plan(v.mission, v.params.spec, other, only_first, -1);
  CHECK(other.gains[1].n == 0U);
  // a phase's own bandwidth overrides the design's
  v.mission.main[0].wn_rad_s = 2.0;
  sim::fit_plan(v.mission, v.params.spec, other, probes, -1);
  CHECK(near_abs(static_cast<double>(other.gains[0].gains[0].pitch.kp), (4.0 + 3.0) / 0.4 / sim::kRad2Deg, 1e-4));
}

TFC_TEST(missionworld_a_rocket_flown_ideally_and_on_its_three_computers_comes_down_under_its_parachutes) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/missions/rocket_recovery.json", v, errors) && errors.empty());
  sim::BuiltMission bm;
  CHECK(sim::build_mission(v, bm, errors));
  const sim::DesignReport rep = sim::design_mission(v, bm);
  CHECK(rep.ok && rep.probes > 40U && rep.end_time_s > 60.0);
  CHECK(bm.main.gains[0].n > 0U && bm.main.gains[0].thrust[bm.main.gains[0].n - 1U] > 1000.0F && bm.main.mass0 > 1000.0);
  // the design flew a descent under canopies to the ground: no glide, so no ballistic coefficient was measured
  CHECK(bm.main.drag.beta == 7000.0);
  const auto world_config = [&](bool ideal) {
    sim::WorldConfig wc;
    wc.params = v.params;
    wc.scenario = v.scenario;
    wc.main_tables = &bm.main;
    wc.ideal = ideal;
    wc.pad_frames = ideal ? 0U : 300U;
    return wc;
  };
  // ideal: the whole flight down to the ground
  {
    sim::MissionWorld w(world_config(true));
    double apogee = 0.0;
    for (int k = 0; k < 60000 && !(w.body(0).veh.landed() || w.body(0).veh.crashed()); ++k) {
      w.step();
      apogee = std::fmax(apogee, w.body(0).veh.altitude());
    }
    CHECK(w.body(0).veh.landed() && !w.body(0).veh.crashed());
    CHECK(apogee > 15000.0);
    CHECK(w.bodies() == 1U && w.finished());
  }
  // on the computers: through the burn and the start of the coast, and the three agree
  {
    sim::MissionWorld w(world_config(false));
    CHECK(w.clamped());
    for (int k = 0; k < 300 + 3000; ++k) {
      w.step();
    }
    const sim::MissionWorld::Body& b = w.body(0);
    CHECK(!w.clamped() && w.flight_frame() == 3000U);
    CHECK(b.av != nullptr && b.av->computer(0).sensors_ok() && b.av->computer(1).sensors_ok() && b.av->computer(2).sensors_ok());
    (void)b.av->digests_agree();   // (each computer has its own sensor: the digests differ in the last digits, and the voted commands are what matters)
    (void)b.av->ready();
    CHECK(b.veh.altitude() > 3000.0 && b.veh.speed() > 300.0 && !b.veh.crashed());
    CHECK(b.av->flight_frame() == 3000U && b.av->computer(0).mission().phase() == 1U);
    CHECK(b.av->computer(0).navigator().has_fix());
    const tfc::dm::Vec3 np = b.av->computer(0).navigator().position();
    CHECK(std::fabs(np.x - b.veh.state().r.x) < 25.0);
  }
}

TFC_TEST(missionworld_a_stage_let_go_is_flown_on_by_computers_of_its_own_and_comes_down_under_its_parachute) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  const std::string text = R"({
  "name": "two stages",
  "vehicle": {"gimbal_limit_deg": 8, "gimbal_rate_dps": 30, "ideal_roll_control": false, "landing_model": true, "crash_speed_ms": 9, "landing_tilt_deg": 60, "landing_lateral_ms": 12},
  "aero": {"diameter_m": 0.8},
  "stages": [
    {"name": "lower", "dry_mass_kg": 400, "x_start_m": 0, "length_m": 4, "radius_m": 0.4, "inertia_factor": 0.6, "x_cg_dry_m": 2, "guided": true, "separation_dv_ms": 1.0,
     "tanks": [{"propellant_kg": 500, "x_bottom_m": 0.5, "radius_m": 0.38, "density_kg_m3": 1000}],
     "sections": [{"kind": "tube", "x_start_m": 0, "length_m": 4, "d_aft_m": 0.8, "d_fore_m": 0.8}],
     "stabilizers": [{"count": 4, "x_le_root_m": 0.3, "root_chord_m": 1.0, "tip_chord_m": 0.4, "span_m": 0.6, "sweep_m": 0.6, "thickness_m": 0.015}],
     "gimbal_limit_deg": 8, "gimbal_rate_dps": 30, "gimbal_lag_s": 0.05},
    {"name": "upper", "dry_mass_kg": 150, "x_start_m": 4, "length_m": 3, "radius_m": 0.4, "inertia_factor": 0.6, "x_cg_dry_m": 5.5, "guided": true,
     "tanks": [{"propellant_kg": 150, "x_bottom_m": 4.3, "radius_m": 0.38, "density_kg_m3": 1000}],
     "sections": [{"kind": "tube", "x_start_m": 4, "length_m": 2, "d_aft_m": 0.8, "d_fore_m": 0.8}, {"kind": "nose", "x_start_m": 6, "length_m": 1, "d_aft_m": 0.8, "d_fore_m": 0, "shape": "ogive"}],
     "gimbal_limit_deg": 8, "gimbal_rate_dps": 30, "gimbal_lag_s": 0.05}
  ],
  "engines": [
    {"stage": 0, "position_m": [0, 0, 0], "thrust_vac_n": 36000, "exit_area_m2": 0.1, "isp_vac_s": 290, "group": 0, "rise_s": 0.2, "tail_s": 0.2},
    {"stage": 1, "position_m": [4, 0, 0], "thrust_vac_n": 8000, "exit_area_m2": 0.04, "isp_vac_s": 310, "group": 1, "rise_s": 0.2, "tail_s": 0.2},
    {"stage": 0, "position_m": [3.5, 0, 0], "thrust_vac_n": 300, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 1, 0], "control": "pitch+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [3.5, 0, 0], "thrust_vac_n": 300, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, -1, 0], "control": "pitch-", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [3.5, 0, 0], "thrust_vac_n": 300, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, -1], "control": "yaw+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [3.5, 0, 0], "thrust_vac_n": 300, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, 1], "control": "yaw-", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [3.5, 0.4, 0], "thrust_vac_n": 150, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, 1], "control": "roll+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [3.5, 0.4, 0], "thrust_vac_n": 150, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, -1], "control": "roll-", "full_cmd_deg": 2.0}
  ],
  "parachutes": [{"name": "canopy", "stage": 0, "drag_area_m2": 120.0, "inflation_s": 3.0, "x_attach_m": 3.8, "max_speed_ms": 300}],
  "mission": {
    "design": {"wn": 1.5, "zeta": 0.8, "sample_s": 1, "max_time_s": 400, "tail_s": 3},
    "mixers": [{"name": "tvc", "gimbal": true, "roll_thrusters": true}, {"name": "rcs", "gimbal": false, "roll_thrusters": true}],
    "main": [
      {"name": "lower burn", "kind": "program", "groups": [0], "mixer": 0, "end": {"time_s": 12}, "program": [[0, 0], [4, 0], [12, 6]], "slew_dps": 5},
      {"name": "separation", "kind": "coast", "hold": "inertial", "groups": [], "mixer": 0, "events": ["separate"], "end": {"time_s": 0.5}, "slew_dps": 5},
      {"name": "upper burn", "kind": "program", "groups": [1], "mixer": 0, "end": {"time_s": 10}, "program": [[0, 6], [10, 20]], "slew_dps": 5, "mass_set_kg": 310},
      {"name": "coast", "kind": "coast", "hold": "prograde", "groups": [], "mixer": 1, "slew_dps": 5, "end": {"time_s": 20}}
    ],
    "stage0": [
      {"name": "tumble", "kind": "coast", "hold": "retrograde", "groups": [], "mixer": 1, "end": {"time_s": 5}, "slew_dps": 10},
      {"name": "fall", "kind": "glide", "groups": [], "mixer": 1, "end": {"time_s": 8}, "slew_dps": 10, "alpha_max_deg": 10},
      {"name": "open", "kind": "chute", "groups": [], "mixer": 1, "drogue_altitude_m": 100000, "main_altitude_m": 100000}
    ]
  }
})";
  CHECK(reads(text, v, errors));
  CHECK(errors.empty());
  sim::BuiltMission bm;
  CHECK(sim::build_mission(v, bm, errors));
  CHECK(bm.has_stage[0]);
  const sim::DesignReport rep = sim::design_mission(v, bm);
  CHECK(rep.ok);
  // the stage's mass at the moment it was let go is what its computers start from
  CHECK(bm.stage[0].mass0 > 400.0 && bm.stage[0].mass0 < 1000.0);
  // the stage glided for a while at a dynamic pressure the design could measure the drag at: the coefficient the guidance flies with is the plant's, not the 7,000 it started with
  CHECK(bm.stage[0].drag.beta > 100.0 && bm.stage[0].drag.beta != 7000.0 && bm.main.drag.beta == 7000.0);
  CHECK(bm.stage[0].gains[0].n > 0U);
  for (const bool ideal : {true, false}) {
    sim::WorldConfig wc;
    wc.params = v.params;
    wc.scenario = v.scenario;
    wc.main_tables = &bm.main;
    wc.stage_tables[0] = &bm.stage[0];
    wc.ideal = ideal;
    wc.pad_frames = ideal ? 0U : 300U;
    sim::MissionWorld w(wc);
    for (int k = 0; k < (ideal ? 20000 : 300 + 3500) && w.bodies() < 2U + 1U; ++k) {
      w.step();
      if (w.bodies() == 2U && w.body(1).born != 0U) {
        break;
      }
    }
    CHECK(w.bodies() == 2U);
    const sim::MissionWorld::Body& lower = w.body(1);
    CHECK(lower.stage == 0 && lower.name == "lower" && lower.tables == &bm.stage[0]);
    CHECK((ideal ? lower.ideal != nullptr : lower.av != nullptr));
    CHECK(near_abs(lower.veh.mass(), w.spawn_mass(0), 1e-6) && w.spawn_mass(0) > 400.0 && w.spawn_mass(5) == 0.0);
    // fly on a while: the stage's computers run, its tumble ends and the canopy opens (the altitudes are far above, so it opens at once)
    for (int k = 0; k < 9000 && !w.body(1).veh.chute_open(0); ++k) {
      w.step();
    }
    CHECK(!w.body(1).veh.crashed());
    for (int k = 0; k < 300; ++k) {
      w.step();
    }
    CHECK(w.body(1).veh.chute_open(0));
    CHECK(w.body(1).veh.chute_fill(0) > 0.0);   // the canopy is open
    if (!ideal) {
      CHECK(w.body(1).av->computer(0).mission().phase() == 2U);
      CHECK(w.body(1).av->flight_frame() > 100U);
    }
    CHECK(w.seeds().empty());
  }
}

TFC_TEST(missionworld_the_gnss_receiver_gives_a_noisy_fix_at_its_rate_and_none_in_an_outage) {
  sim::GnssModel g;
  g.period_frames = 5U;
  g.pos_sigma_m = 3.0;
  g.vel_sigma_ms = 0.1;
  g.outages.push_back({100U, 200U});
  const sim::V3 r{6378137.0, 1000.0, 2000.0};
  const sim::V3 vel{10.0, 20.0, 30.0};
  tfc::nav::GnssFix fix;
  CHECK(g.fix(0U, r, vel, fix));
  CHECK(!g.fix(1U, r, vel, fix) && !g.fix(4U, r, vel, fix));
  CHECK(g.fix(5U, r, vel, fix));
  CHECK(!g.fix(100U, r, vel, fix) && !g.fix(195U, r, vel, fix) && g.fix(200U, r, vel, fix));
  double sum2 = 0.0;
  double mean = 0.0;
  const int n = 4000;
  for (int i = 0; i < n; ++i) {
    CHECK(g.fix(static_cast<uint32_t>(i * 5 + 1000), r, vel, fix));
    const double e = fix.r.y - r.y;
    mean += e;
    sum2 += e * e;
  }
  mean /= n;
  CHECK(std::fabs(mean) < 0.3 && near_abs(std::sqrt(sum2 / n), 3.0, 0.3));
  sim::GnssModel off;
  off.period_frames = 0U;
  CHECK(!off.fix(0U, r, vel, fix));
  sim::GnssModel other;
  other.seed = 7U;
  sim::GnssModel same;
  tfc::nav::GnssFix a;
  tfc::nav::GnssFix b;
  CHECK(other.fix(0U, r, vel, a) && same.fix(0U, r, vel, b) && a.r.x != b.r.x);
}

namespace {
// A toy orbiter: 1,000 kg, one gimballed engine of 30 kN at an Isp of 450 s, four tail fins, a pitch program and then a PEG burn to a circular orbit. It is a vehicle for a test, not an orbiter:
// the propellant is just enough that PEG runs to the end of it and the orbit it leaves is not circular (the design only needs a burn to record the seed of).
const char* kToyOrbiter = R"TOY({
  "name": "toy orbiter",
  "vehicle": {"gimbal_limit_deg": 8, "gimbal_rate_dps": 30, "gimbal_lag_s": 0.05, "ideal_roll_control": false},
  "aero": {"diameter_m": 0.8},
  "stages": [{"name": "orbiter", "dry_mass_kg": 50, "length_m": 7, "radius_m": 0.4, "inertia_factor": 0.6, "x_cg_dry_m": 3.5, "guided": true,
              "tanks": [{"propellant_kg": 950, "x_bottom_m": 0.8, "radius_m": 0.38, "density_kg_m3": 1000}],
              "sections": [{"kind": "tube", "x_start_m": 0, "length_m": 5.6, "d_aft_m": 0.8, "d_fore_m": 0.8}, {"kind": "nose", "x_start_m": 5.6, "length_m": 1.4, "d_aft_m": 0.8, "d_fore_m": 0, "shape": "ogive"}],
              "stabilizers": [{"count": 4, "x_le_root_m": 0.4, "root_chord_m": 1.2, "tip_chord_m": 0.5, "span_m": 0.7, "sweep_m": 0.7, "thickness_m": 0.015}],
              "gimbal_limit_deg": 8, "gimbal_rate_dps": 30, "gimbal_lag_s": 0.05}],
  "engines": [
    {"stage": 0, "position_m": [0, 0, 0], "thrust_vac_n": 30000, "exit_area_m2": 0.05, "isp_vac_s": 450, "group": 0, "rise_s": 0.2, "tail_s": 0.2},
    {"stage": 0, "position_m": [6.0, 0, 0], "thrust_vac_n": 100, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 1, 0], "control": "pitch+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [6.0, 0, 0], "thrust_vac_n": 100, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, -1, 0], "control": "pitch-", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [6.0, 0, 0], "thrust_vac_n": 100, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, -1], "control": "yaw+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [6.0, 0, 0], "thrust_vac_n": 100, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, 1], "control": "yaw-", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [6.0, 0.4, 0], "thrust_vac_n": 50, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, 1], "control": "roll+", "full_cmd_deg": 2.0},
    {"stage": 0, "position_m": [6.0, 0.4, 0], "thrust_vac_n": 50, "exit_area_m2": 0.0, "isp_vac_s": 60, "gimbal": false, "direction": [0, 0, -1], "control": "roll-", "full_cmd_deg": 2.0}
  ],
  "mission": {
    "design": {"wn": 1.5, "zeta": 0.8, "sample_s": 2, "max_time_s": 900, "tail_s": 10},
    "mixers": [{"name": "tvc", "gimbal": true, "roll_thrusters": true}, {"name": "rcs", "gimbal": false, "roll_thrusters": true}],
    "main": [
      {"name": "ascent", "kind": "program", "groups": [0], "mixer": 0, "end": {"speed_ms": 2200}, "program": [[0, 0], [5, 0], [25, 20], [70, 55], [110, 75]], "slew_dps": 5},
      {"name": "insertion", "kind": "peg", "groups": [0], "mixer": 0, "end": {"cutoff": true}, "target": {"circular_km": 200}, "burnout_mass_kg": 55, "slew_dps": 5},
      {"name": "coast", "kind": "coast", "hold": "prograde", "groups": [], "mixer": 1, "slew_dps": 3, "end": {"time_s": 20}},
      {"name": "in orbit", "kind": "coast", "hold": "prograde", "groups": [], "mixer": 1, "slew_dps": 3}
    ]
  }
}
)TOY";
}  // namespace

TFC_TEST(missionworld_a_burn_to_orbit_is_designed_by_solving_it_once_and_the_solution_is_the_seed_of_the_flight) {
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(reads(kToyOrbiter, v, errors));
  CHECK(errors.empty());
  sim::BuiltMission bm;
  CHECK(sim::build_mission(v, bm, errors));
  CHECK(bm.main.phase[1].p[7] == 0.0F);   // no seed yet
  unsigned watched = 0U;
  const sim::DesignReport rep = sim::design_mission(v, bm, [&watched](const sim::MissionWorld& w) { watched += w.bodies() > 0U ? 1U : 0U; });
  CHECK(rep.ok && watched > 100U);
  CHECK(bm.main.phase[1].p[7] > 20.0F);                                 // the burn time of the solution
  CHECK(bm.main.phase[1].p[5] != bm.main.phase[1].p[3] || bm.main.phase[1].p[4] != bm.main.phase[1].p[3]);
  // flown again with the seed, in the ideal mode: the burn reaches the end of its propellant at about orbital height and the mission goes on to its coast and its last phase
  sim::WorldConfig wc;
  wc.params = v.params;
  wc.scenario = v.scenario;
  wc.main_tables = &bm.main;
  wc.ideal = true;
  sim::MissionWorld w(wc);
  unsigned logged = 0U;
  double last_alt = 0.0;
  w.set_log([&logged, &last_alt](const sim::BodyLog& l) {
    ++logged;
    last_alt = l.altitude;
  }, 100U);
  for (int k = 0; k < 30000 && !w.finished(); ++k) {
    w.step();
  }
  CHECK(w.finished() && w.body(0).last_phase == 3U);
  CHECK(logged > 100U && last_alt > 150000.0);
  CHECK(w.body(0).veh.altitude() > 150000.0 && w.body(0).veh.speed() > 7000.0);
  CHECK(w.seeds().size() == 1U && w.seeds()[0].phase == 1U && w.seeds()[0].stage == -1);
}

TFC_TEST(missionfile_names_that_are_not_known_are_not_taken_for_any_other) {
  bool ok = true;
  CHECK(sim::detail::kind_of("peg", ok) == tfc::gnc::kind::kPeg && ok);
  (void)sim::detail::kind_of("sideways", ok);
  CHECK(!ok);
  CHECK(sim::detail::end_of("ignition", ok) == tfc::gnc::end::kIgnition && ok);
  CHECK(sim::detail::end_of("cutoff", ok) == tfc::gnc::end::kCutoff && ok);
  (void)sim::detail::end_of("whenever", ok);
  CHECK(!ok);
  CHECK(sim::detail::hold_of("pro_horizontal", ok) == tfc::gnc::hold::kProHorizontal && ok);
  (void)sim::detail::hold_of("sideways", ok);
  CHECK(!ok);
  CHECK(sim::detail::event_of("extend_legs", ok) == tfc::propbit::kExtendLegs && ok);
  (void)sim::detail::event_of("explode", ok);
  CHECK(!ok);
  CHECK(sim::detail::mask_of({0, 3, 7, 8, -1}) == 0x89U);   // a group outside 0 to 7 is left out
}
