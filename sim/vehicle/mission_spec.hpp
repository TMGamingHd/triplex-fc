// SPDX-License-Identifier: MIT
// What a vehicle file says about the mission its computers fly (docs/design/GNC.md section 9, docs/design/VEHICLE_SPEC.md): the phases of each set of computers, the mixers that turn the attitude demand into effector
// commands, the place of the catch tower, and the settings of the design that fits the gains. This is the description; `mission_build.hpp` turns it into the constant tables the flight computers carry.
#pragma once
#include <array>
#include <string>
#include <vector>

#include "spec.hpp"

namespace sim {

struct PhaseSpec {
  std::string name;
  std::string kind = "hold";        // hold, program, peg, coast, boostback, glide, belly, landing, chute, done
  std::string hold = "inertial";    // for a coast: inertial, prograde, retrograde, radial, fixed, retro_horizontal, pro_horizontal
  std::string end_kind = "never";   // never, time_s, speed_ms, apoapsis_after_s, altitude_below_m, altitude_above_m, cutoff, mass_below_kg, aligned_deg, touchdown
  double end_value = 0.0;
  std::vector<int> groups;          // engine groups commanded on
  std::vector<std::string> events;  // separate, deploy_surfaces, extend_legs, chute0, chute1, jettison
  int mixer = 0;
  double throttle = 1.0;
  std::vector<std::array<double, 2>> throttle_track;   // [seconds in the phase, fraction]
  double slew_dps = 20.0;
  double wn_rad_s = 0.0;            // the bandwidth of the attitude loop in this phase (0: the design's)
  std::vector<std::array<double, 2>> program;   // a "program" phase: the pitch plane tilt (degrees from the vertical, toward downrange) against the seconds in the phase
  double mass_set_kg = 0.0;         // the mass estimate takes this value at the start of the phase (0: unchanged)
  V3 fixed{1.0, 0.0, 0.0};          // a coast with the "fixed" hold: the direction
  // peg
  double target_radius_m = 0.0;     // 0: derived from the orbit below
  double target_speed_ms = 0.0;
  double target_gamma_deg = 0.0;
  double circular_km = -1.0;        // a circular orbit of this altitude (>= 0)
  double apogee_km = -1.0;          // or an ellipse: the apogee and perigee altitudes and the altitude the burn ends at
  double perigee_km = 0.0;
  double cutoff_km = 0.0;
  double burnout_mass_kg = 0.0;     // the mass at which the propellant of the burn is gone (the burn is limited to it)
  // boost-back
  double bias_m = 0.0;
  double reserve_mass_kg = 0.0;
  double pitch_up_deg = 0.0;        // the thrust of the burn is lifted this far above the horizontal
  // glide
  double alpha_max_deg = 12.0;
  double gain_deg_per_km = 3.0;
  double lift_area_per_deg_m2 = 3.5;   // the sideways force per unit dynamic pressure and degree of angle of attack, for the steering by lift
  double gate_speed_ms = 330.0;     // the speed to come to by the height below, falling (the landing burn is lit there)
  double gate_height_m = 3600.0;
  double brake_max_deg = 25.0;      // the most the tilt adds when the fall is too fast by its own speed or more
  double alpha_brake_deg = 0.0;     // a steady angle of attack toward the site while gliding: extra drag, and the lift that goes with it
  // belly (an entry belly first, steered by the angle of attack: alpha_max_deg and gain_deg_per_km above are its largest angle and its gain)
  // program (a small roll through the climb, as launchers fly: out to roll_deg and back, starting at roll_start_s and taking roll_duration_s; 0: none)
  double roll_deg = 0.0;
  double roll_start_s = 0.0;
  double roll_duration_s = 0.0;
  double alpha_deg = 70.0;          // the nominal angle between the body axis and the velocity, nose raised
  double alpha_min_deg = 50.0;      // the smallest the steering lets it be (the most lift)
  double bank_gain_deg_per_km = 0.0;  // the bank that steers across the ground track: degrees of roll about the velocity per kilometre of predicted miss across it
  double bank_max_deg = 0.0;        // the most it banks (0: it does not)
  double beta_kg_m2 = 0.0;          // the ballistic coefficient the prediction uses in this phase (0: the tables'): a belly entry's drag is 30 times a tail-first fall's
  // landing: the groups that run for each of the numbers of engines the burn may use, fewest first ("one", "two", "three", "many": the first three engines, then all thirteen of the inner ring)
  std::array<std::vector<int>, 4> landing_groups;
  // chute
  double drogue_altitude_m = 0.0;
  double main_altitude_m = 0.0;
};

struct MixerSpec {
  std::string name;
  bool gimbal = true;               // the gimbal pitch and yaw commands carry the pitch and yaw demands (they also drive the thrusters that fire in proportion to them)
  bool roll_thrusters = true;       // the roll demand goes to the roll thrusters
  std::vector<int> surfaces;        // surface channels used: the demands are shared among them by a minimum-effort allocation, designed from the vehicle
  std::array<double, 4> trim_deg{};  // the deflection of each channel at zero demand (a flap held at 60 degrees in a belly flop steers about that, not about 0): the demands are deflections about it
};

// What a body's structure and skin are allowed to meet: the limits a mission is checked against (the envelope of mission_world.hpp is what it met). Zero is no limit. The numbers are the vehicle's and are not
// made up here: a file that has none is not checked, and a file that has them says where they came from.
struct LoadLimits {
  double max_q_pa = 0.0;
  double max_q_sin_alpha_pa = 0.0;
  double max_axial_g = 0.0;
  double max_lateral_g = 0.0;
  double max_stagnation_w_m2 = 0.0;
  double max_wall_k = 0.0;
  [[nodiscard]] bool any() const { return max_q_pa > 0.0 || max_q_sin_alpha_pa > 0.0 || max_axial_g > 0.0 || max_lateral_g > 0.0 || max_stagnation_w_m2 > 0.0 || max_wall_k > 0.0; }
};

struct MissionSpec {
  bool present = false;
  // the catch tower and the landing site, relative to the launch point
  double site_offset_y_m = 0.0;
  double site_offset_z_m = 0.0;
  double arm_height_m = 70.0;
  double capture_radius_m = 1.5;
  double ignition_margin = 1.15;
  // the landing burn
  double sink_ms = 0.5;
  double aim_below_m = 1.5;         // the burn aims this far below the arms' height
  double approach_s = 8.0;
  double horizon_min_s = 8.0;       // the horizontal correction takes at least this long: a position loop faster than the attitude loop that tilts the vehicle for it would oscillate
  double tilt_max_deg = 12.0;
  double tilt_final_deg = 2.5;
  double final_height_m = 25.0;
  double engine_thrust_n = 2.45e6;  // one landing engine at full throttle
  double engine_min_throttle = 0.4;
  double decel_plan_ms2 = 15.0;     // the net deceleration (beyond holding up the weight) the burn is planned on: it is lit when the stopping distance at this deceleration reaches the height
  double main_landing_propellant_kg = 0.0;  // the same for the main body when it is the one that lands (a ship coming home): its top stage's catch height is built from it (0: the main body does not land)
  double landing_propellant_kg = 25000.0;   // what is left in the stage at the catch, for where its centre of gravity is then
  // the air for the glide's predictor
  double ballistic_coefficient = 7000.0;   // kg/m^2, vehicle in the attitude it descends in
  double scale_height_m = 7200.0;
  // the attitude loop
  double command_limit_deg = 8.0;
  double integrator_limit_deg = 6.0;
  double slew_deg_per_frame = 0.6;
  // the design of the gains
  double wn = 0.6;
  double zeta = 0.8;
  double ki_over_kp = 0.1;
  double kp_max = 80.0;
  double b_min = 1.0e-6;
  double sample_s = 1.0;
  double max_time_s = 3000.0;       // the design run stops here at the latest
  double tail_s = 60.0;             // and this long after every body has reached its last phase
  LoadLimits structure_main;                                    // the limits of the main body ("structure": {"main": {...}})
  std::array<LoadLimits, kMaxStages> structure_stage{};          // and of each stage let go, by the stage's index ("stages": [{...}, ...])
  double min_separation_gap_m = 0.0;                            // the smallest gap a stage may come to the vehicle it leaves
  std::string structure_source;                                  // where the numbers came from (said in the file, repeated in the report)
  std::vector<MixerSpec> mixers;
  std::vector<PhaseSpec> main;                                  // the computers that fly the whole vehicle
  std::array<std::vector<PhaseSpec>, kMaxStages> stage;         // the computers of a stage that has been let go, by the stage's index
};

}  // namespace sim
