// SPDX-License-Identifier: MIT
// A 6-DOF launch-vehicle model for the simulator (host only, double precision). See docs/design/VEHICLE_SIM.md and, for the description of a vehicle, docs/design/VEHICLE_SPEC.md.
//
// Frames. Inertial: non-rotating, centred on a spherical Earth, with the launch point on +X (local vertical), downrange +Y, crossrange +Z (right-handed).
// Body: X along the long axis (nose forward), Y and Z lateral; at lift-off the body axes coincide with the inertial ones (the vehicle stands on the pad).
// Sensor/platform frame: (X, Y, Z) = (body Y, body Z, body X): Z is the long axis, so an accelerometer at rest reads +1 g on Z, as on the rig.
//
// Gimbal convention (chosen so that a positive command gives a positive angular acceleration of the platform tilt it is named for):
//   pitch plane: positive command tilts the nose toward downrange (+Y inertial at lift-off); yaw plane: positive command tilts the nose toward -Z
//   (the platform's tilt about X is positive toward -crossrange).
//
// The vehicle is a stack of stages with tanks and engines (spec.hpp). `Params` describes the simple reference vehicle by its own fields; with a non-empty `Params::spec` it describes any
// vehicle. Either way the same equations run: a `Params` without a spec is turned into the equivalent description (`legacy_spec`), so the reference vehicle is one instance of the
// general model, and tests/test_vehicle_general.cpp holds the model to the frozen single-vehicle implementation it replaced (tests/oracle/vehicle6_legacy.hpp).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "aero.hpp"
#include "atmosphere.hpp"
#include "math3.hpp"
#include "newtonian.hpp"
#include "slosh.hpp"
#include "spec.hpp"
#include "surfaces.hpp"

namespace sim {

struct Params {
  // The simple reference vehicle: one stage of `engines` identical engines. Ignored when `spec` is not empty (the global fields below it still apply).
  // masses
  double m_dry = 6000.0;        // kg
  double m_prop0 = 24000.0;     // kg
  // geometry: x is measured from the aft end, forward positive
  double length = 20.0;         // m
  double diameter = 1.8;        // m
  double x_dry_cg = 10.0;       // m
  double x_tank_bottom = 2.0;   // m
  double prop_density = 900.0;  // kg/m^3
  double x_cp = 12.5;           // centre of pressure, m (ahead of the CG: the vehicle is aerodynamically unstable)
  // propulsion: `engines` identical engines, the first on the axis, the others at `engine_offset` from it
  int engines = 5;
  double thrust_vac_each = 92000.0;  // N
  double exit_area_each = 0.12;      // m^2
  double isp_vac = 310.0;            // s
  double engine_offset = 0.5;        // m
  // thrust-vector control (vehicle-wide; a stage of a general vehicle can override them)
  double gimbal_limit_deg = 8.0;
  double gimbal_rate_dps = 60.0;
  // aerodynamics
  double c_n_alpha = 2.5;  // normal-force slope per radian, on the cross-section area
  // An ideal roll controller: the roll rate about the long axis is held at zero and roll torques are ignored. The vehicle has some roll control (the
  // flight computers' two-plane gimbal does not provide it), which is not modelled; without it an engine-out's roll torque, acting on an inertia 40 times
  // smaller than the pitch inertia, would turn the vehicle's gimbal planes away from the pad's and end the flight for a reason that is not the subject.
  bool ideal_roll_control = true;
  // The ground. The pad (and the Earth's surface below it) holds the vehicle up, so that a vehicle whose thrust-to-weight is under 1 stands on the pad (burning
  // propellant, until it is light enough to rise) instead of sinking through it, and a vehicle that comes back down lands or is destroyed. A vehicle that flies from the first step
  // is not touched by it. `false` is for tests that want the bare equations.
  bool ground_contact = true;
  double crash_speed_ms = 5.0;  // a vehicle that reaches the ground faster than this is destroyed: the run stops there (`crashed()`)
  // The landing model (docs/design/RECOVERY.md): once the vehicle has left the pad, the ground is met by the lowest point of the vehicle (its base, or the foot of its legs, or the ends of a body lying
  // down), not by its centre of gravity; a tower with arms can catch it by its pins; a touchdown is judged on its speed and attitude. Off: the pad model above, exactly as it was.
  bool landing_model = false;
  double landing_tilt_deg = 12.0;   // a touchdown more than this far from upright is a crash
  double landing_lateral_ms = 3.0;  // and more than this sideways speed
  // tests only: 0 switches gravity off, so that the rocket equation can be checked exactly
  double gravity_scale = 1.0;
  // the largest integration substep, s (tests lower it to check that the answer has converged)
  double max_substep = 0.002;
  // Dispersions: what real vehicles have that the nominal design does not. All neutral by default, so the nominal flight is unchanged.
  double gimbal_lag_s = 0.0;           // the gimbal actuator's first-order lag (a real TVC servo has a bandwidth of a few to tens of Hz), applied before its rate limit
  double thrust_misalign_pitch_deg = 0.0;  // a constant offset of the thrust direction from the commanded gimbal, pitch plane (an engine mounted a little off)
  double thrust_misalign_yaw_deg = 0.0;    // and in the yaw plane
  double thrust_scale = 1.0;           // every engine's thrust and mass flow times this (a dispersion of the engines' performance)
  double cd_scale = 1.0;               // the axial force coefficient times this
  double cn_scale = 1.0;               // the normal-force slope times this (aerodynamic uncertainty)
  // Any other vehicle: stages, tanks, engines, payloads (spec.hpp, docs/design/VEHICLE_SPEC.md). Empty: the reference vehicle of the fields above.
  VehicleSpec spec;
};

struct Gust {  // a 1-cosine gust of peak velocity `peak` (m/s, inertial frame) lasting `duration` seconds from `t0`
  double t0 = 0.0;
  double duration = 2.0;
  V3 peak{};
};

struct State {
  V3 r{kEarthR, 0.0, 0.0};
  V3 v{};
  Q4 q{};
  V3 w{};
  double m = 30000.0;                         // the total mass (the stages and payloads still on the vehicle, with their propellant)
  std::array<double, kMaxStages> prop{};      // the propellant left in each stage (a stage that has separated keeps what it had)
  bool prop_set = false;                      // false: derive `prop` from `m` (a vehicle of one stage); true: `prop` is given
  V3 wheel_h{};                               // the angular momentum of the reaction wheels about body Y and Z (x unused), N m s
  double roll = 0.0;                          // the angle turned through about the long axis since T-zero, rad (the integral of the roll rate: what a roll controller holds)
  std::array<std::array<double, 4>, kMaxSlosh> slosh{};  // each sloshing tank's mass: its displacement in body Y and Z (m) and the speed of each (m/s) relative to the tank
  std::array<double, 4> flex{};                           // the bending mode: eta in body Y and Z (m) and their rates (m/s)
};

struct EngineFailure {
  double time = 0.0;  // s since T-zero
  int index = 0;      // which engine (0 is the first in the vehicle's list)
};

struct Site {  // where the vehicle is launched from: the pole's direction (for rotation and the oblateness) comes from it
  double latitude_deg = 28.5;
  double azimuth_deg = 90.0;  // of the launch direction (downrange), from north through east
};

struct Turbulence {  // a random wind on top of the mean wind and the gusts: a first-order Gauss-Markov process per axis with a length scale (Dryden's shape), the same every run for a seed
  double sigma_ms = 0.0;           // the standard deviation of each component; 0: none
  double scale_length_m = 533.0;   // 1750 ft, the length scale of Dryden's model above 2000 ft
  uint32_t seed = 1U;
};

// The catch tower: two arms at `height` above the ground, at the place `offset` from the launch point (y downrange, z crossrange of the launch frame), that close on a stage's grapple pins.
// A catch is made when the pins come down through the arms' height inside the capture radius, slowly enough, upright enough. (Starship's tower arms catch the booster at about 70 m.)
struct Tower {
  bool enabled = false;
  double height_m = 70.0;
  double offset_y_m = 0.0;
  double offset_z_m = 0.0;
  double capture_radius_m = 1.5;
  double max_sink_ms = 2.0;       // the vertical speed the arms can take up
  double max_lateral_ms = 1.5;
  double max_tilt_deg = 3.0;
};

struct Scenario {
  Site site;
  Turbulence turbulence;
  std::vector<std::array<double, 2>> wind_profile;  // (altitude m, mean wind speed m/s), linear between points: replaces the built-in profile when given
  std::vector<Gust> gusts;
  V3 wind_dir{0.0, 0.0, 1.0};   // direction of the mean wind (inertial), crossrange by default
  double wind_scale = 1.0;      // multiplies the mean wind profile
  double engine_out_time = -1.0;  // seconds; negative: never
  int engine_out_index = 1;       // which engine fails (0 is on the axis)
  std::vector<EngineFailure> engine_failures;  // more failures, any number
  double dry_cg_shift = 0.0;      // m, a mass offset: moves every stage's dry centre of gravity forward (+) or aft (-)
  bool has_initial = false;       // start from `initial` (a state in flight, an orbit) instead of standing on the pad
  State initial;
  Tower tower;                    // the catch tower at the launch site
  // Dispersion of the engines' starts: the chance that an engine does not light when it is commanded on (Flight 12's boostback relit 20 of 28 engines), drawn from the scenario's own generator
  double start_failure_prob = 0.0;
  uint32_t start_seed = 7U;
  double ground_radius_override = 0.0;   // a spawned body is told where the ground is (the sphere the launch pad stands on, below the planet's reference radius by the height of the launch vehicle's centre of gravity); 0: derived
};

struct MassProps {
  double mass = 0.0;
  double x_cg = 0.0;  // from the aft end
  double i_t = 0.0;   // transverse inertia about the CG
  double i_x = 0.0;   // roll inertia
  double slosh_mass = 0.0;  // the part of the liquid that sloshes, which the rest (mass, x_cg, inertias) leaves out: mass + slosh_mass is the vehicle's
};

// Everything the vehicle is told each step. The flight computers' voted gimbal command (degrees, the two planes) and, for a stage they drive (`StageSpec::guided`), the propulsion command (which engine
// groups run, at what throttle), the surface command (a deflection in degrees for each channel) and the events (level bits: the model acts once, on the rising edge of a bit).
namespace ev {
constexpr uint32_t kSeparate = 1U << 0;        // let go the lowest stage that is still on the vehicle
constexpr uint32_t kDeploySurfaces = 1U << 1;  // free the grid fins and flaps from their stowed position
constexpr uint32_t kExtendLegs = 1U << 2;
constexpr uint32_t kChute0 = 1U << 3;          // release parachute 0 (the drogue)
constexpr uint32_t kChute1 = 1U << 4;          // and parachute 1 (the main)
constexpr uint32_t kJettison = 1U << 5;        // jettison the first payload that is on the vehicle (a fairing, a satellite)
}  // namespace ev

struct Controls {
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double throttle = 0.0;                              // fraction of rated thrust of the engines that run, guided stages (engines run only when told to: the default is none)
  uint32_t group_mask = 0U;                           // guided stages: bit g set: the engines of group g are commanded on
  std::array<double, kSurfaceChannels> surface_deg{};  // the surface command
  double roll_deg = 0.0;                              // the roll command: what the roll thrusters fire in proportion to
  uint32_t events = 0U;
};

struct Tilts {  // the vehicle's long axis relative to the pad vertical, as the platform shows it
  double x_deg = 0.0;  // about the platform's X axis (the yaw plane)
  double y_deg = 0.0;  // about Y (the pitch plane)
};

struct Loads {
  V3 f_thrust{};  // body frame
  V3 f_aero{};
  V3 m_thrust{};  // about the CG, body frame
  V3 m_aero{};
  double mdot = 0.0;  // kg/s, positive while burning
  std::array<double, kMaxStages> mdot_stage{};  // and the part of it each stage burns
  double dynamic_pressure = 0.0;
  double mach = 0.0;
  double alpha = 0.0;  // total angle of attack, rad
  double thrust = 0.0;  // N, total
  V3 f_slosh{};         // the force of the sloshing masses' springs on the vehicle, body frame
  V3 m_slosh{};         // and its moment about the CG
  struct SloshNow {     // each sloshing tank now: its mass, where it acts, its spring and its damper
    double m1 = 0.0;
    double x_s = 0.0;
    double k = 0.0;
    double c = 0.0;
  };
  std::array<SloshNow, kMaxSlosh> pod{};
  V3 flex_q{};          // the generalised force on the bending mode in body Y and Z (N)
};

// The description of the reference vehicle of `Params`' own fields, in the general form: one stage, one tank, `engines` engines.
inline VehicleSpec legacy_spec(const Params& p) {
  VehicleSpec g;
  g.aero = p.spec.aero;      // (a Params without stages keeps the world and the aerodynamics settings its spec holds)
  g.planet = p.spec.planet;
  StageSpec st;
  st.name = "stage 1";
  st.dry_mass = p.m_dry;
  st.x_start = 0.0;
  st.length = p.length;
  st.radius = 0.5 * p.diameter;
  st.inertia_factor = 0.6;
  st.x_cg_dry = p.x_dry_cg;
  TankSpec tk;
  tk.propellant = p.m_prop0;
  tk.x_bottom = p.x_tank_bottom;
  tk.radius = 0.5 * p.diameter;
  tk.density = p.prop_density;
  st.tanks.push_back(tk);
  g.stages.push_back(st);
  for (int i = 0; i < p.engines; ++i) {
    EngineSpec e;
    e.stage = 0;
    const std::size_t k = static_cast<std::size_t>(i);
    e.pos = V3{0.0, k == 1 ? p.engine_offset : (k == 2 ? -p.engine_offset : 0.0), k == 3 ? p.engine_offset : (k == 4 ? -p.engine_offset : 0.0)};
    e.thrust_vac = p.thrust_vac_each;
    e.exit_area = p.exit_area_each;
    e.isp_vac = p.isp_vac;
    g.engines.push_back(e);
  }
  return g;
}

// A stage (or the part of the vehicle left of it) that has been let go, for the simulation that keeps flying it: its description re-based so that its own aft end is x = 0, its state at the instant of
// separation (the position and velocity are those of the parent vehicle's centre of gravity; `Vehicle6::spawn` puts them at the stage's own), and where it came from.
struct Detached {
  int stage = 0;              // its index in the parent vehicle
  double t = 0.0;             // the parent's time at the separation
  VehicleSpec spec;           // the stage alone
  State state;                // attitude, rates, propellant; r and v are the parent's centre of gravity before the separation
  double x_off = 0.0;         // the stage's aft end in the parent's coordinates
  double x_cg_parent = 0.0;   // the parent's whole-vehicle centre of gravity just before (parent coordinates)
  double recoil_ms = 0.0;     // the speed the push of the separation gives it, backward along the axis
  double ground_radius = 0.0; // the sphere the ground is on
  bool launched = false;
};

class Vehicle6 {
 public:
  explicit Vehicle6(const Params& p = Params{}, const Scenario& sc = Scenario{}) : p_(p), sc_(sc), g_(p.spec.empty() ? legacy_spec(p) : p.spec) {
    // A description that does not make sense would index out of its arrays or fly a vehicle nobody described: refuse it loudly (the file reader has checked it already; this is for a
    // vehicle built in code). validate() says what is wrong.
    const std::vector<std::string> problems = p.spec.empty() ? std::vector<std::string>{} : validate(g_);  // (the simple parameters of the reference vehicle keep their old latitude: a coasting body with no propellant is one)
    if (!problems.empty()) {
      for (const std::string& m : problems) {
        std::fprintf(stderr, "Vehicle6: invalid vehicle description: %s\n", m.c_str());
      }
      std::abort();
    }
    pl_ = g_.planet;
    servo_dynamics_ = g_.actuator.order == 2 || g_.actuator.backlash_deg > 0.0;
    for (std::size_t st = 0; st < g_.stages.size(); ++st) {
      for (std::size_t k = 0; k < g_.stages[st].tanks.size(); ++k) {
        if (g_.stages[st].tanks[k].slosh.enabled && pods_.size() < kMaxSlosh) {
          pods_.push_back(Pod{st, k});
        }
      }
    }
    ideal_roll_ = p_.ideal_roll_control && !g_.roll.enabled;
    s_.r = V3{pl_.radius, 0.0, 0.0};
    {
      const double lat = sc_.site.latitude_deg * kDeg2Rad;
      const double azi = sc_.site.azimuth_deg * kDeg2Rad;
      pole_ = V3{std::sin(lat), std::cos(lat) * std::cos(azi), std::cos(lat) * std::sin(azi)};  // the pole in the vehicle's frame (X up at the launch point, Y downrange, Z crossrange)
      omega_ = pole_ * pl_.rotation_rate;
      rotating_ = pl_.rotation_rate != 0.0;
      rng_ = 0x9E3779B97F4A7C15ULL ^ (static_cast<uint64_t>(sc_.turbulence.seed) * 0xBF58476D1CE4E5B9ULL);
    }
    for (std::size_t i = 0; i < kMaxStages; ++i) {
      t_ign_[i] = -1.0;
      t_burnout_[i] = -1.0;
      t_sep_[i] = -1.0;
      active_[i] = i < g_.stages.size();
    }
    for (std::size_t i = 0; i < kMaxPayloads; ++i) {
      payload_active_[i] = i < g_.payloads.size();
    }
    for (std::size_t s = 0; s < g_.stages.size() && s < kMaxStages; ++s) {
      for (const TankSpec& t : g_.stages[s].tanks) {
        cap_[s] += t.propellant;
      }
      s_.prop[s] = cap_[s];
    }
    s_.m = total_mass(s_.prop);
    refresh_fixed();
    if (sc_.engine_out_time >= 0.0) {
      failures_.push_back(EngineFailure{sc_.engine_out_time, sc_.engine_out_index});
    }
    failures_.insert(failures_.end(), sc_.engine_failures.begin(), sc_.engine_failures.end());
    if (rotating_) {
      s_.v = cross(omega_, s_.r);  // on the pad the vehicle moves with the planet, and turns with it
      s_.w = omega_;
    }
    if (sc_.has_initial) {
      State init = sc_.initial;  // a start in flight gives the position, velocity and attitude; the vehicle brings its own mass unless the state carries its propellant
      if (!init.prop_set) {
        init.prop = s_.prop;
        init.m = s_.m;
        init.prop_set = true;
      }
      set_state(init);
    }
    for (std::size_t i = 0; i < g_.surfaces.size() && i < kMaxSurfaces; ++i) {
      surf_deployed_[i] = g_.surfaces[i].deployed;
      surf_pos_[i] = g_.surfaces[i].stow_deg;
    }
    chute_open_.fill(-1.0);
    start_rng_ = 0x2545F4914F6CDD1DULL ^ (static_cast<uint64_t>(sc_.start_seed) * 0x9E3779B97F4A7C15ULL);
    ground_radius_ = sc_.ground_radius_override > 0.0 ? sc_.ground_radius_override : (sc_.has_initial ? pl_.radius : pl_.radius - ground_offset());
    launched_ = sc_.has_initial && altitude() > 1.0;
    process_events();
    update_engines(0.0, 0.0, 0.0);
  }

  // Advance by dt seconds with the commanded gimbal angles (degrees). Substeps of at most `max_substep` (2 ms).
  void step(double dt, double cmd_pitch_deg, double cmd_yaw_deg) {
    Controls c = ctrl_;
    c.pitch_deg = cmd_pitch_deg;
    c.yaw_deg = cmd_yaw_deg;
    step(dt, c);
  }

  // Advance by dt seconds with the whole of the commands (the gimbal, the propulsion and the surfaces of the stages the flight computers drive, the events).
  void step(double dt, const Controls& c) {
    const double cmd_pitch_deg = c.pitch_deg;
    const double cmd_yaw_deg = c.yaw_deg;
    apply_events(c.events);
    ctrl_ = c;
    if (!launched_ && altitude() > 1.0) {
      launched_ = true;
    }
    const int n = std::max(1, static_cast<int>(std::ceil((dt / p_.max_substep) - 1e-9)));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) {
      process_events();
      drive_surfaces(h);
      if (g_.flex.enabled && !active_[static_cast<std::size_t>(g_.flex.stage)]) {
        s_.flex = {};  // the stage the mode belongs to has gone: so has the mode
      }
      step_turbulence(h);
      update_engines(h, cmd_pitch_deg, cmd_yaw_deg);
      for (std::size_t f = 0; f < g_.fins.size(); ++f) {
        const FinSpec& fin = g_.fins[f];
        fin_p_[f] = slew(fin_p_[f], fin.gain * cmd_pitch_deg, h, fin.limit_deg, fin.rate_dps, fin.lag_s);
        fin_y_[f] = slew(fin_y_[f], fin.gain * cmd_yaw_deg, h, fin.limit_deg, fin.rate_dps, fin.lag_s);
      }
      wheel_m_ = wheel_torque(cmd_pitch_deg, cmd_yaw_deg);
      for (std::size_t s = 0; s < g_.stages.size(); ++s) {
        if (active_[s]) {
          const StageSpec& st = g_.stages[s];
          const double limit = st.gimbal_limit_deg >= 0.0 ? st.gimbal_limit_deg : p_.gimbal_limit_deg;
          const double rate = st.gimbal_rate_dps >= 0.0 ? st.gimbal_rate_dps : p_.gimbal_rate_dps;
          const double lag = st.gimbal_lag_s >= 0.0 ? st.gimbal_lag_s : p_.gimbal_lag_s;
          if (servo_dynamics_) {
            drive_servo(servo_p_[s], servo_pv_[s], gimbal_p_[s], cmd_pitch_deg, h, limit, rate, lag);
            drive_servo(servo_y_[s], servo_yv_[s], gimbal_y_[s], cmd_yaw_deg, h, limit, rate, lag);
          } else {
            gimbal_p_[s] = slew(gimbal_p_[s], cmd_pitch_deg, h, limit, rate, lag);
            gimbal_y_[s] = slew(gimbal_y_[s], cmd_yaw_deg, h, limit, rate, lag);
          }
        }
      }
      if (p_.ground_contact && held_by_ground(h)) {
        t_ += h;
        continue;
      }
      rk4(h);
      take_up_wheel_momentum(h);
      t_ += h;
    }
  }

  [[nodiscard]] double time() const { return t_; }
  [[nodiscard]] const State& state() const { return s_; }
  [[nodiscard]] double altitude() const { return norm(s_.r) - pl_.radius; }
  [[nodiscard]] double speed() const { return norm(s_.v); }
  // The distance along the planet's surface from the launch point (the vehicle frame's +X axis at T-zero): the arc of the angle between the position and that axis. It is inertial: a rotating
  // planet's own motion is not taken out of it. (atan2 of the sideways part, not acos of the cosine: acos has no precision left for a small angle, and the first kilometres are small angles.)
  [[nodiscard]] double range() const { return pl_.radius * std::atan2(std::hypot(s_.r.y, s_.r.z), s_.r.x); }
  [[nodiscard]] double mass() const { return s_.m; }
  [[nodiscard]] double gimbal_pitch_deg() const { return gimbal_p_[lead_stage()]; }  // of the stage that is flying (the lowest one on the vehicle)
  [[nodiscard]] double gimbal_yaw_deg() const { return gimbal_y_[lead_stage()]; }
  [[nodiscard]] const Params& params() const { return p_; }
  [[nodiscard]] const VehicleSpec& spec() const { return g_; }  // the description the model runs on (the reference vehicle's too)
  // True while some stage still on the vehicle has propellant: the engines can thrust.
  [[nodiscard]] bool burning() const {
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (active_[s] && s_.prop[s] > kEmptyProp) {
        return true;
      }
    }
    return false;
  }
  [[nodiscard]] bool on_ground() const { return grounded_; }  // standing on the pad (or landed): held there by the ground
  [[nodiscard]] bool crashed() const { return crashed_; }     // reached the ground faster than `crash_speed_ms`: the state is frozen where it hit
  // The engines of the stages still on the vehicle that have not failed, and all the engines of those stages.
  [[nodiscard]] int engines_on() const {
    int n = 0;
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      n += (active_[static_cast<std::size_t>(g_.engines[e].stage)] && !failed_[e]) ? 1 : 0;
    }
    return n;
  }
  [[nodiscard]] int engine_count() const {
    int n = 0;
    for (const EngineSpec& e : g_.engines) {
      n += active_[static_cast<std::size_t>(e.stage)] ? 1 : 0;
    }
    return n;
  }
  [[nodiscard]] bool stage_active(std::size_t s) const { return s < g_.stages.size() && active_[s]; }
  [[nodiscard]] bool stage_ignited(std::size_t s) const { return s < g_.stages.size() && t_ign_[s] >= 0.0; }
  [[nodiscard]] double propellant(std::size_t s) const { return s < kMaxStages ? s_.prop[s] : 0.0; }
  // Read-only views of what the model holds, for the 3D viewer (viewer_state.hpp, docs/design/VIEWER.md): they change nothing and nothing in the flight depends on them.
  [[nodiscard]] double engine_fraction(std::size_t e) const { return e < g_.engines.size() ? frac_[e] : 0.0; }   // the thrust fraction now, after the rise and the tail (0..1, times the stage's throttle)
  [[nodiscard]] bool engine_failed(std::size_t e) const { return e < g_.engines.size() && failed_[e]; }
  [[nodiscard]] double stage_gimbal_pitch_deg(std::size_t s) const { return s < kMaxStages ? gimbal_p_[s] : 0.0; }
  [[nodiscard]] double stage_gimbal_yaw_deg(std::size_t s) const { return s < kMaxStages ? gimbal_y_[s] : 0.0; }
  [[nodiscard]] double fin_pitch_deg(std::size_t f) const { return f < kMaxFins ? fin_p_[f] : 0.0; }
  [[nodiscard]] double fin_yaw_deg(std::size_t f) const { return f < kMaxFins ? fin_y_[f] : 0.0; }
  [[nodiscard]] double surface_deg(std::size_t i) const { return i < kMaxSurfaces ? surf_pos_[i] : 0.0; }   // the deflection of an articulated surface now
  [[nodiscard]] bool surface_deployed(std::size_t i) const { return i < kMaxSurfaces && surf_deployed_[i]; }
  [[nodiscard]] bool chute_open(std::size_t i) const { return i < kMaxParachutes && chute_open_[i] >= 0.0; }
  [[nodiscard]] double chute_fill(std::size_t i) const {   // how full the canopy is, 0 to 1
    return (i < g_.parachutes.size() && chute_open_[i] >= 0.0 && g_.parachutes[i].drag_area > 0.0) ? parachute_drag_area(chute_of(i), t_) / g_.parachutes[i].drag_area : 0.0;
  }
  [[nodiscard]] bool legs_out() const { return legs_out_; }
  [[nodiscard]] bool landed() const { return landed_; }   // touched down upright and slowly: held on the ground
  [[nodiscard]] bool caught() const { return caught_; }   // held in the tower's arms
  [[nodiscard]] bool catch_missed() const { return catch_missed_; }   // the pins came down through the arms' height outside what the arms can take
  [[nodiscard]] bool launched() const { return launched_; }
  [[nodiscard]] bool engine_on_command(std::size_t e) const { return e < g_.engines.size() && eng_on_[e]; }
  [[nodiscard]] int engine_starts(std::size_t e) const { return e < g_.engines.size() ? starts_[e] : 0; }
  [[nodiscard]] double ground_radius() const { return ground_radius_; }
  [[nodiscard]] const Controls& controls() const { return ctrl_; }
  // The height of the lowest point of the vehicle above the ground, m (the base, or the foot of the legs, or an end of a body lying down); the pins' height above the ground for a catch.
  [[nodiscard]] double base_height() const { return lowest_point_height(); }
  [[nodiscard]] double pin_height() const { return pin_height_now(); }
  // The stage let go since the last call, ready to be flown on its own (see `spawn`).
  [[nodiscard]] std::vector<Detached> take_detached() {
    std::vector<Detached> out;
    out.swap(detached_);
    return out;
  }
  // The heating of the skin now: the stagnation-point flux on the nose (Sutton-Graves), the flux over the belly, and the temperature the skin comes to when it radiates the belly flux away.
  struct Heating {
    double stagnation_w_m2 = 0.0;
    double belly_w_m2 = 0.0;
    double wall_k = 0.0;
  };
  [[nodiscard]] Heating heating() const {
    Heating h;
    const double alt = altitude();
    const Air air = atmosphere(alt);
    const double speed = norm(rotating_ ? s_.v - cross(omega_, s_.r) : s_.v);
    const double rn = g_.aero.nose_radius_m > 0.0 ? g_.aero.nose_radius_m : 0.5;
    h.stagnation_w_m2 = stagnation_heat_flux(air.density, speed, rn);
    h.belly_w_m2 = h.stagnation_w_m2 * g_.aero.belly_heat_factor;
    h.wall_k = radiative_equilibrium_k(h.belly_w_m2, g_.aero.emissivity);
    return h;
  }
  [[nodiscard]] bool payload_active(std::size_t i) const { return i < kMaxPayloads && payload_active_[i]; }
  [[nodiscard]] std::size_t slosh_count() const { return pods_.size(); }
  [[nodiscard]] double tank_liquid_kg(std::size_t stage, std::size_t tank) const {   // the propellant in one tank now
    if (stage >= g_.stages.size() || tank >= g_.stages[stage].tanks.size() || !active_[stage]) {
      return 0.0;
    }
    return tank_liquid(stage, tank, prop_of(s_, stage));
  }

  // The mass properties of a vehicle of total mass m with the propellant of its stages in the proportion they have now: exact for a vehicle of one stage (the reference vehicle),
  // an interpolation for a stack.
  [[nodiscard]] MassProps mass_props(double m) const {
    std::array<double, kMaxStages> prop = s_.prop;
    double have = 0.0;
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      have += active_[s] ? prop[s] : 0.0;
    }
    const double fixed = fixed_;
    if (g_.stages.size() == 1U) {
      prop[0] = std::clamp(m - fixed, 0.0, capacity(0U));
    } else if (have > 0.0) {
      const double scale = std::max(m - fixed, 0.0) / have;
      for (std::size_t s = 0; s < g_.stages.size(); ++s) {
        prop[s] *= scale;
      }
    }
    MassProps mp = mass_props_of(prop, false);  // (the designer's view: the whole liquid as part of the rigid body)
    mp.mass = m;  // as asked: the mass given, whatever the parts add up to
    return mp;
  }
  [[nodiscard]] MassProps mass_props() const { return mass_props_state(s_, -1, false); }  // the whole liquid in the rigid body: what the gains are designed from
  [[nodiscard]] MassProps rigid_mass_props() const { return mass_props_state(s_, -1, true); }  // with the sloshing mass taken out (`slosh_mass` says how much): what the dynamics use

  // The forces and moments on a state at time t (the gimbal angles are the vehicle's current ones).
  [[nodiscard]] Loads loads(const State& s, double t) const {
    Loads l;
    const double alt = norm(s.r) - pl_.radius;
    const Air air = atmosphere(alt);
    const MassProps mp = mass_props_state(s, -1);
    const V3 vrel = rotate_inv(s.q, rotating_ ? s.v - wind_at(alt, t) - cross(omega_, s.r) : s.v - wind_at(alt, t));  // the air moves with a rotating planet
    const double v_abs = norm(vrel);
    l.mach = v_abs / air.sound;
    l.dynamic_pressure = 0.5 * air.density * v_abs * v_abs;
    const double area = kPi * 0.25 * p_.diameter * p_.diameter;
    if (shaped_) {
      if (v_abs > 1.0) {
        if (g_.aero.full_regime && geo_.ready()) {
          shape_forces_full(l, air, vrel, v_abs, mp, s.w);
        } else {
          shape_forces(l, air, vrel, v_abs, mp);
        }
      }
    } else if (v_abs > 1.0 && vrel.x > 0.0) {
      const double lat = std::hypot(vrel.y, vrel.z);
      l.alpha = std::atan2(lat, vrel.x);
      l.f_aero.x = -l.dynamic_pressure * area * p_.cd_scale * axial_coefficient(l.mach);
      if (lat > 1e-9) {
        const double fn = -l.dynamic_pressure * area * p_.c_n_alpha * p_.cn_scale * l.alpha;
        l.f_aero.y = fn * vrel.y / lat;
        l.f_aero.z = fn * vrel.z / lat;
      }
      l.m_aero = cross(V3{p_.x_cp - mp.x_cg, 0.0, 0.0}, V3{0.0, l.f_aero.y, l.f_aero.z});
    }
    // The thrust of every engine that is running: along its own direction (the commanded gimbal of its stage if it gimbals, its cant, the misalignment dispersion), at its own position.
    // Engines without a cant of their own share one direction per stage (gimbaled) and one for the fixed ones, found once.
    const bool flex_on = g_.flex.enabled && active_[static_cast<std::size_t>(g_.flex.stage)];
    std::array<V3, kMaxStages> dir_gimbal{};
    std::array<bool, kMaxStages> have_gimbal{};
    V3 dir_fixed{};
    bool have_fixed = false;
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      const double frac = frac_[e];
      const EngineSpec& en = g_.engines[e];
      const std::size_t st = static_cast<std::size_t>(en.stage);
      if (!(frac > 0.0) || !(s_.prop[st] > kEmptyProp)) {  // an engine without propellant in its stage does not run (the vehicle's own state decides, not the intermediate states of a step)
        continue;
      }
      const double ti = std::max(0.0, (frac * p_.thrust_scale * en.thrust_vac) - (air.pressure * en.exit_area));
      V3 dir;
      const bool own_dir = en.dir.x != 1.0 || en.dir.y != 0.0 || en.dir.z != 0.0;  // a thruster pointing its own way
      const bool shared = !own_dir && en.cant_pitch_deg == 0.0 && en.cant_yaw_deg == 0.0;
      if (own_dir) {
        dir = normalized(en.dir);
      } else if (shared && en.gimbal && have_gimbal[st]) {
        dir = dir_gimbal[st];
      } else if (shared && !en.gimbal && have_fixed) {
        dir = dir_fixed;
      } else {
        const double dp = (((en.gimbal ? gimbal_p_[st] : 0.0) + en.cant_pitch_deg) + p_.thrust_misalign_pitch_deg) * kDeg2Rad;
        const double dy = (((en.gimbal ? gimbal_y_[st] : 0.0) + en.cant_yaw_deg) + p_.thrust_misalign_yaw_deg) * kDeg2Rad;
        dir = V3{std::cos(dp) * std::cos(dy), -std::sin(dp) * std::cos(dy), std::sin(dy)};
        if (shared && en.gimbal) {
          dir_gimbal[st] = dir;
          have_gimbal[st] = true;
        } else if (shared) {
          dir_fixed = dir;
          have_fixed = true;
        }
      }
      V3 f = dir * ti;
      if (flex_on) {  // the thrust follows the structure: along the tangent of the bent axis at the engines, a lateral force T sigma eta on top
        f = f + V3{0.0, ti * g_.flex.slope_engine * s.flex[0], ti * g_.flex.slope_engine * s.flex[1]};
      }
      l.f_thrust = l.f_thrust + f;
      l.m_thrust = l.m_thrust + cross(V3{en.pos.x - mp.x_cg, en.pos.y, en.pos.z}, f);
      const double md = frac * p_.thrust_scale * en.thrust_vac / (en.isp_vac * kG0);
      l.mdot += md;
      l.mdot_stage[st] += md;
      l.thrust += ti;
      if (g_.jet_damping) {  // the exhaust leaves with the velocity of the nozzle, which a turning vehicle gives it: it carries away angular momentum, -md r x (w x r)
        const V3 r{en.pos.x - mp.x_cg, en.pos.y, en.pos.z};
        l.m_thrust = l.m_thrust - (cross(r, cross(s.w, r)) * md);
      }
    }
    if (flex_on) {
      l.flex_q = V3{0.0, g_.flex.phi_engine * l.f_thrust.y, g_.flex.phi_engine * l.f_thrust.z};
    }
    // The fins: a force at the hinge of each set, normal to the plane of the two fins that make it (pitch: along -Y for a positive deflection; yaw: along +Z).
    for (std::size_t f = 0; f < g_.fins.size(); ++f) {
      const FinSpec& fin = g_.fins[f];
      if (!active_[static_cast<std::size_t>(fin.stage)]) {
        continue;
      }
      const double qs = l.dynamic_pressure * 2.0 * fin.area_each * fin.lift_slope;
      const double fy = -qs * fin_p_[f] * kDeg2Rad;
      const double fz = qs * fin_y_[f] * kDeg2Rad;
      l.f_aero.y += fy;
      l.f_aero.z += fz;
      l.m_aero = l.m_aero + cross(V3{fin.x_hinge - mp.x_cg, 0.0, 0.0}, V3{0.0, fy, fz});
    }
    if (!g_.surfaces.empty()) {
      add_surface_loads(l, s, air, vrel, mp);
    }
    if (!g_.parachutes.empty()) {
      add_chute_loads(l, s, air, vrel, mp, t);
    }
    if (g_.wheels.enabled) {
      l.m_thrust = l.m_thrust + wheel_m_;  // the wheels' torque on the vehicle (an internal torque: the wheel takes the opposite momentum)
    }
    if (g_.roll.enabled && active_[static_cast<std::size_t>(g_.roll.stage)]) {  // the vehicle's own roll controller: a torque about the long axis against the roll and its rate
      l.m_thrust.x += std::clamp((-g_.roll.kp * s.roll) - (g_.roll.kd * s.w.x), -g_.roll.torque_max, g_.roll.torque_max);
    }
    if (!pods_.empty()) {
      // The sloshing masses: a spring and a damper between each and the tank, along body Y and Z. The spring's stiffness is the mass times omega^2, with omega^2 the model's `w2g` times the
      // acceleration along the axis (floored: a liquid with no axial acceleration is not settled and has no spring to speak of). The force on the vehicle is the spring's, at the mass.
      const double g_axial = std::max((l.f_thrust.x + l.f_aero.x) / s.m, kSloshMinG);
      for (std::size_t i = 0; i < pods_.size(); ++i) {
        const Pod& pd = pods_[i];
        if (!active_[pd.stage]) {
          continue;
        }
        const TankSpec& tk = g_.stages[pd.stage].tanks[pd.tank];
        const SloshModel sm = slosh_model(tk, tank_liquid(pd.stage, pd.tank, prop_of(s, pd.stage)));
        if (!(sm.m1 > 0.0)) {
          continue;
        }
        const double w2 = g_axial * sm.w2g;
        Loads::SloshNow& now = l.pod[i];
        now.m1 = sm.m1;
        now.x_s = sm.x_s;
        now.k = sm.m1 * w2;
        now.c = 2.0 * tk.slosh.damping * sm.m1 * std::sqrt(w2);
        const std::array<double, 4>& x = s.slosh[i];
        const V3 f{0.0, (now.k * x[0]) + (now.c * x[2]), (now.k * x[1]) + (now.c * x[3])};
        l.f_slosh = l.f_slosh + f;
        l.m_slosh = l.m_slosh + cross(V3{sm.x_s - mp.x_cg, 0.0, 0.0}, f);
      }
    }
    return l;
  }

  // The platform tilts that show the vehicle's long axis (about X: the yaw plane, positive toward -crossrange; about Y: the pitch plane,
  // positive toward downrange), from the pad vertical.
  [[nodiscard]] Tilts tilts() const {
    const V3 u = rotate(s_.q, V3{1.0, 0.0, 0.0});
    Tilts t;
    t.y_deg = std::atan2(u.y, u.x) * kRad2Deg;
    t.x_deg = -std::asin(std::clamp(u.z, -1.0, 1.0)) * kRad2Deg;
    return t;
  }

  // What IMUs fixed to the vehicle would feel, in the sensor frame (X, Y, Z) = (body Y, body Z, body X): body rates in dps and specific force in g.
  void vehicle_true_imu(V3& gyro_dps, V3& accel_g) const {
    const Loads l = loads(s_, t_);
    V3 f = (l.f_thrust + l.f_aero) / s_.m / kG0;
    if (!pods_.empty()) {  // lateral: over the rigid mass, with the springs' force; axial: over the whole mass (the sloshing mass rides on the tank)
      const double m_rigid = s_.m - mass_props_state(s_, -1).slosh_mass;
      f = V3{(l.f_thrust.x + l.f_aero.x) / s_.m, (l.f_thrust.y + l.f_aero.y + l.f_slosh.y) / m_rigid, (l.f_thrust.z + l.f_aero.z + l.f_slosh.z) / m_rigid} / kG0;
    }
    V3 w = s_.w;
    V3 a = f;
    if (p_.landing_model && (grounded_ || landed_ || caught_)) {   // held by the ground or the arms: the reaction makes the specific force the local gravity along the vertical, whatever the engines do
      a = rotate_inv(s_.q, s_.r / norm(s_.r)) * (norm(gravity(s_.r)) / kG0);
      f = a;
    }
    if (g_.flex.enabled && active_[static_cast<std::size_t>(g_.flex.stage)]) {  // the structure bends where the IMUs are: its slope turns at eta' and its displacement accelerates at eta''
      const FlexSpec& fx = g_.flex;
      const double wf = 2.0 * kPi * fx.frequency_hz;
      const double ddy = (l.flex_q.y - (2.0 * fx.damping * wf * fx.generalized_mass * s_.flex[2]) - (wf * wf * fx.generalized_mass * s_.flex[0])) / fx.generalized_mass;
      const double ddz = (l.flex_q.z - (2.0 * fx.damping * wf * fx.generalized_mass * s_.flex[3]) - (wf * wf * fx.generalized_mass * s_.flex[1])) / fx.generalized_mass;
      w = w + V3{0.0, -fx.slope_imu * s_.flex[3], fx.slope_imu * s_.flex[2]};
      a = a + (V3{0.0, fx.phi_imu * ddy, fx.phi_imu * ddz} / kG0);
    }
    gyro_dps = V3{w.y, w.z, w.x} * kRad2Deg;
    accel_g = V3{a.y, a.z, a.x};
  }

  [[nodiscard]] Loads current_loads() const { return loads(s_, t_); }

  // For the design of the attitude loop: the angular acceleration (rad/s^2, body axes) the vehicle would have, in the state it is in now, with every actuator at once at the angle `c` commands (the gimbal of the
  // stage that flies, the surfaces that are deployed, the roll thrusters): the answer to "what does one more degree of this command do". A copy is made and nothing in this vehicle changes.
  [[nodiscard]] V3 angular_accel_with(const Controls& c) const {
    Vehicle6 tmp = *this;
    tmp.ctrl_.roll_deg = c.roll_deg;
    tmp.ctrl_.pitch_deg = c.pitch_deg;
    tmp.ctrl_.yaw_deg = c.yaw_deg;
    for (std::size_t st = 0; st < tmp.g_.stages.size(); ++st) {
      tmp.gimbal_p_[st] = c.pitch_deg;
      tmp.gimbal_y_[st] = c.yaw_deg;
    }
    for (std::size_t i = 0; i < tmp.g_.surfaces.size(); ++i) {
      const SurfaceSpec& sf = tmp.g_.surfaces[i];
      if (tmp.surf_deployed_[i] && sf.channel >= 0 && sf.channel < static_cast<int>(kSurfaceChannels)) {
        tmp.surf_pos_[i] = std::clamp(c.surface_deg[static_cast<std::size_t>(sf.channel)], sf.min_deg, sf.max_deg);
      }
    }
    tmp.update_engines(0.0, c.pitch_deg, c.yaw_deg);   // (the thrusters that fire in proportion to a command take it; the main engines keep their thrust)
    return tmp.deriv(tmp.s_, tmp.t_).w;
  }

  // The same with the attitude turned by `angle_rad` about the body axis `axis` (0 roll, 1 yaw, 2 pitch: x, y, z) and the velocity unchanged: the change in angular acceleration per radian is the aerodynamic stiffness
  // of that axis (positive: it makes the vehicle diverge).
  [[nodiscard]] V3 angular_accel_turned(const Controls& c, int axis, double angle_rad) const {
    Vehicle6 tmp = *this;
    const V3 ax = axis == 0 ? V3{1.0, 0.0, 0.0} : (axis == 1 ? V3{0.0, 1.0, 0.0} : V3{0.0, 0.0, 1.0});
    tmp.s_.q = normalized(tmp.s_.q * from_axis_angle(ax, angle_rad));
    return tmp.angular_accel_with(c);
  }

  // The centre of gravity of stage `s` alone with `prop_kg` of propellant, from its own aft end (for the design of the landing: where the pins are against where the centre of gravity is).
  [[nodiscard]] double stage_cg_from_aft(std::size_t s, double prop_kg) const {
    Vehicle6 tmp = *this;
    for (std::size_t k = 0; k < tmp.g_.stages.size(); ++k) {
      tmp.active_[k] = k == s;
    }
    tmp.payload_active_.fill(false);
    tmp.s_.prop = std::array<double, kMaxStages>{};
    tmp.s_.prop[s] = std::clamp(prop_kg, 0.0, tmp.capacity(s));
    tmp.refresh_fixed();
    return tmp.mass_props_state(tmp.s_, -1).x_cg - tmp.g_.stages[s].x_start;
  }

  // Put the vehicle at an attitude and rate (for the design run that forces the attitude to the guidance's, and for tests).
  void set_attitude(const Q4& q, const V3& w) {
    s_.q = normalized(q);
    s_.w = w;
  }

  // How strongly a small gimbal angle turns the vehicle: the sum over the engines that gimbal of thrust times the arm from the centre of gravity, divided by the transverse inertia
  // (rad/s^2 per rad), at the state it is in now. It is what the gains of the flight computers are designed from (design.hpp).
  [[nodiscard]] double control_effectiveness() const {
    const MassProps mp = mass_props();
    const Air air = atmosphere(altitude());
    double num = 0.0;
    double thrust = 0.0;
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      const EngineSpec& en = g_.engines[e];
      const bool own_dir = en.dir.x != 1.0 || en.dir.y != 0.0 || en.dir.z != 0.0;
      if (en.control == Control::None && en.gimbal && frac_[e] > 0.0) {
        const double ti = std::max(0.0, (frac_[e] * p_.thrust_scale * en.thrust_vac) - (air.pressure * en.exit_area));
        num += ti * (mp.x_cg - en.pos.x);
        thrust += ti;
      } else if (en.control == Control::PitchPlus && own_dir && engine_available(e)) {
        // a thruster: the torque about Z of its full thrust, per radian of command (it is on in proportion, fully at full_cmd_deg)
        const double ti = std::max(0.0, (p_.thrust_scale * en.thrust_vac) - (air.pressure * en.exit_area));
        const V3 d = normalized(en.dir);
        const double mz = ((en.pos.x - mp.x_cg) * d.y * ti) - (en.pos.y * d.x * ti);  // signed: a thruster wired the wrong way makes the vehicle's control effectiveness negative, and the validity check says so
        num += mz / (en.full_cmd_deg * kDeg2Rad);
        thrust += ti;
      }
    }
    double b = (thrust > 0.0 ? num / mp.i_t : 0.0);
    const double q = current_loads().dynamic_pressure;
    for (std::size_t f = 0; f < g_.fins.size(); ++f) {
      const FinSpec& fin = g_.fins[f];
      if (active_[static_cast<std::size_t>(fin.stage)]) {
        b += q * 2.0 * fin.area_each * fin.lift_slope * fin.gain * (mp.x_cg - fin.x_hinge) / mp.i_t;
      }
    }
    if (g_.wheels.enabled && active_[static_cast<std::size_t>(g_.wheels.stage)]) {
      b += g_.wheels.torque_max / (g_.wheels.full_cmd_deg * kDeg2Rad) / mp.i_t;
    }
    return b;
  }
  // The aerodynamic divergence a = q S C_Nalpha (x_cp - x_cg) / I (1/s^2) at the state it is in now: how fast the unstable pitch mode grows.
  [[nodiscard]] double divergence() const {
    const Loads l = current_loads();
    const MassProps mp = mass_props();
    if (shaped_) {
      double cn_alpha = 0.0;
      double moment_slope = 0.0;
      double ca = 0.0;
      double s_ref = kPi * 0.25 * p_.diameter * p_.diameter;
      if (geo_.ready()) {
        s_ref = geo_.reference_area();
        geo_.slopes(l.mach, mp.x_cg, cn_alpha, moment_slope);
      } else {
        table_lookup(l.mach, ca, cn_alpha, moment_slope, mp.x_cg);
      }
      return l.dynamic_pressure * s_ref * moment_slope / mp.i_t;
    }
    const double area = kPi * 0.25 * p_.diameter * p_.diameter;
    return l.dynamic_pressure * area * p_.c_n_alpha * (p_.x_cp - mp.x_cg) / mp.i_t;
  }

  // The aerodynamics of the vehicle as it is now, at Mach `mach`: the slope of the normal force (per radian, on `s_ref`), where its centre of pressure is (m from the aft end) and the
  // axial coefficient flying nose first at sea-level density and 1e7 Reynolds. For the reference model's fixed numbers, those. For tools that describe a vehicle and for the tests.
  void aero_summary(double mach, double& cn_alpha, double& x_cp, double& ca, double& s_ref) const {
    double moment_slope = 0.0;
    if (geo_.ready()) {
      s_ref = geo_.reference_area();
      geo_.slopes(mach, 0.0, cn_alpha, moment_slope);
      ca = geo_.axial(mach, 1.0e7, power_fraction(), g_.aero) * p_.cd_scale;
      x_cp = cn_alpha != 0.0 ? moment_slope / cn_alpha : 0.0;
    } else if (shaped_) {
      s_ref = kPi * 0.25 * p_.diameter * p_.diameter;
      table_lookup(mach, ca, cn_alpha, moment_slope, 0.0);
      x_cp = cn_alpha != 0.0 ? moment_slope / cn_alpha : 0.0;
    } else {
      s_ref = kPi * 0.25 * p_.diameter * p_.diameter;
      cn_alpha = p_.c_n_alpha;
      x_cp = p_.x_cp;
      ca = axial_coefficient(mach) * p_.cd_scale;
    }
  }

  // The mean wind (the scenario's profile, or the built-in one), the gusts and the turbulence, inertial frame, m/s.
  [[nodiscard]] V3 wind_at(double altitude, double t) const {
    V3 w = normalized(sc_.wind_dir) * (sc_.wind_scale * mean_wind_speed(altitude));
    for (const Gust& g : sc_.gusts) {
      if (t >= g.t0 && t <= g.t0 + g.duration && g.duration > 0.0) {
        w = w + (g.peak * (0.5 * (1.0 - std::cos(2.0 * kPi * (t - g.t0) / g.duration))));
      }
    }
    if (sc_.turbulence.sigma_ms > 0.0) {
      w = w + turb_;
    }
    return w;
  }

  // The speed of the mean wind (the profile times the scenario's scale) at an altitude, m/s: for the viewer's picture of the atmosphere.
  [[nodiscard]] double mean_wind_ms(double altitude) const { return sc_.wind_scale * mean_wind_speed(altitude); }

  // The air at an altitude: the planet's atmosphere (the 1976 standard, an exponential one, or none) with the scenario's dispersions of density and temperature.
  [[nodiscard]] Air atmosphere(double alt) const {
    Air a;
    switch (pl_.atmosphere) {
      case AtmosphereKind::Us1976:
        a = air_at(alt);
        break;
      case AtmosphereKind::Exponential: {
        a.density = pl_.surface_density * std::exp(-std::max(alt, 0.0) / pl_.scale_height);
        a.temperature = pl_.temperature;
        a.pressure = a.density * pl_.gas_constant * a.temperature;
        a.sound = std::sqrt(pl_.gamma * pl_.gas_constant * a.temperature);
        break;
      }
      case AtmosphereKind::None:
        a.density = 0.0;
        a.pressure = 0.0;
        a.temperature = 0.0;
        a.sound = 300.0;  // (there is no air to have a speed of sound: a placeholder so that a Mach number can be formed)
        break;
    }
    if (pl_.temperature_offset != 0.0 && a.temperature > 0.0) {
      const double t_new = a.temperature + pl_.temperature_offset;
      a.density *= a.temperature / t_new;  // the same pressure in warmer or colder air
      a.sound *= std::sqrt(t_new / a.temperature);
      a.temperature = t_new;
    }
    if (pl_.density_scale != 1.0) {
      a.density *= pl_.density_scale;
      a.pressure *= pl_.density_scale;
    }
    return a;
  }

  // The gravitational acceleration at position r: the inverse-square field of the planet (times `gravity_scale`) and, if it has one, the oblateness term J2 about its pole.
  [[nodiscard]] V3 gravity(const V3& r) const {
    const double rn = norm(r);
    V3 g = r * (-p_.gravity_scale * pl_.mu / (rn * rn * rn));
    if (pl_.j2 != 0.0) {
      const V3 rh = r / rn;
      const double z = dot(rh, pole_);
      const double k = 1.5 * pl_.j2 * pl_.mu * pl_.radius * pl_.radius / (rn * rn * rn * rn);
      g = g + (((rh * ((5.0 * z * z) - 1.0)) - (pole_ * (2.0 * z))) * (k * p_.gravity_scale));
    }
    return g;
  }
  [[nodiscard]] const PlanetSpec& planet() const { return pl_; }
  [[nodiscard]] const V3& turbulence() const { return turb_; }  // the random wind now, m/s (inertial frame)
  [[nodiscard]] const V3& pole() const { return pole_; }

  // Replace the whole state (for tests and for starting an orbit or a coast). With `prop_set` false the propellant is derived from the mass (a vehicle of one stage);
  // a stack takes its propellant from `prop` and its mass from the stages that are still on it.
  void set_state(const State& s) {
    const std::array<double, kMaxStages> keep = s_.prop;
    s_ = s;
    if (!s.prop_set) {
      if (g_.stages.size() == 1U) {
        s_.prop = std::array<double, kMaxStages>{};
        s_.prop[0] = std::clamp(s.m - fixed_, 0.0, capacity(0U));  // the mass is kept as given
      } else {
        s_.prop = keep;
        s_.m = total_mass(s_.prop);
      }
    }
    s_.prop_set = true;
    if (g_.stages.size() == 1U && s.prop_set && s.prop[0] != std::clamp(s.m - fixed_, 0.0, capacity(0U))) {
      s_.m = fixed_ + std::clamp(s.prop[0], 0.0, capacity(0U));  // the caller gave a propellant that is not what the mass says: the propellant is what they meant
    }
    sync_propellant();
  }

  // Fly a stage that has been let go: a vehicle of that stage alone, at its own centre of gravity, with the attitude, rates and velocity the separation left it (the parent's, and the push of the springs
  // backward), and its engines commanded off until the computers that fly it say otherwise. Every stage of the description it takes is driven by its controls (the schedule it had on the parent is gone).
  [[nodiscard]] static Vehicle6 spawn(const Detached& d, const Params& parent, const Scenario& sc) {
    Params cp = parent;
    cp.spec = d.spec;
    Scenario cs = sc;
    cs.has_initial = false;
    cs.ground_radius_override = d.ground_radius;
    cs.engine_out_time = -1.0;
    cs.engine_failures.clear();
    Vehicle6 v(cp, cs);
    State st = d.state;
    v.set_state(st);
    const double x_cg_child = v.mass_props().x_cg;
    const V3 dx{d.x_off + x_cg_child - d.x_cg_parent, 0.0, 0.0};
    st.r = d.state.r + rotate(d.state.q, dx);
    st.v = d.state.v + rotate(d.state.q, cross(d.state.w, dx)) + rotate(d.state.q, V3{-d.recoil_ms, 0.0, 0.0});
    v.set_state(st);
    v.t_ = d.t;
    v.launched_ = d.launched;
    v.grounded_ = false;
    v.ctrl_ = Controls{};
    v.ctrl_.group_mask = 0U;
    v.ctrl_.throttle = 0.0;
    return v;
  }

 private:
  static constexpr double kEmptyProp = 1e-6;  // kg of propellant under which a stage is out of it

  [[nodiscard]] double mean_wind_speed(double altitude) const {
    if (!sc_.wind_profile.empty()) {
      const std::vector<std::array<double, 2>>& pr = sc_.wind_profile;
      if (altitude <= pr.front()[0]) {
        return pr.front()[1];
      }
      for (std::size_t i = 1; i < pr.size(); ++i) {
        if (altitude <= pr[i][0]) {
          const double f = (altitude - pr[i - 1][0]) / (pr[i][0] - pr[i - 1][0]);
          return pr[i - 1][1] + (f * (pr[i][1] - pr[i - 1][1]));
        }
      }
      return pr.back()[1];
    }
    struct Pt {
      double h;
      double v;
    };
    static constexpr std::array<Pt, 9> kProfile{{{0.0, 0.0}, {1000.0, 5.0}, {3000.0, 10.0}, {8000.0, 18.0}, {12000.0, 28.0},
                                                 {16000.0, 20.0}, {25000.0, 10.0}, {40000.0, 5.0}, {60000.0, 0.0}}};
    if (altitude <= kProfile.front().h) {
      return kProfile.front().v;
    }
    for (std::size_t i = 1; i < kProfile.size(); ++i) {
      if (altitude <= kProfile[i].h) {
        const double f = (altitude - kProfile[i - 1].h) / (kProfile[i].h - kProfile[i - 1].h);
        return kProfile[i - 1].v + (f * (kProfile[i].v - kProfile[i - 1].v));
      }
    }
    return 0.0;
  }

  static double axial_coefficient(double mach) {
    const double d = (mach - 1.1) / 0.35;
    return 0.30 + (0.45 * std::exp(-d * d));  // a transonic rise
  }

  [[nodiscard]] double slew(double cur, double cmd, double h, double limit, double rate, double lag) const {
    const double target = std::clamp(cmd, -limit, limit);
    const double want = lag > 0.0 ? (target - cur) * (h / (lag + h)) : (target - cur);  // a first-order lag, then the rate limit
    const double d = std::clamp(want, -rate * h, rate * h);
    return cur + d;
  }

  // One axis of a gimbal servo with dynamics (a second-order servo and/or backlash; the plain first-order lag is `slew`): moves the servo (`pos`, `vel`) toward the command, then the engine
  // (`out`) follows it through the play. The second-order servo is theta'' = wn^2 (target - theta) - 2 zeta wn theta', advanced by RK4 (exact to the step's order for the linear part), then the
  // rate and the travel are limited (at a stop the velocity is lost).
  void drive_servo(double& pos, double& vel, double& out, double cmd, double h, double limit, double rate, double lag) const {
    const ActuatorSpec& ac = g_.actuator;
    if (ac.order == 2) {
      const double target = std::clamp(cmd, -limit, limit);
      const double wn = 2.0 * kPi * ac.natural_hz;
      const double zw = 2.0 * ac.damping * wn;
      const auto acc = [&](double x, double v) { return (wn * wn * (target - x)) - (zw * v); };
      const double k1x = vel;
      const double k1v = acc(pos, vel);
      const double k2x = vel + (0.5 * h * k1v);
      const double k2v = acc(pos + (0.5 * h * k1x), vel + (0.5 * h * k1v));
      const double k3x = vel + (0.5 * h * k2v);
      const double k3v = acc(pos + (0.5 * h * k2x), vel + (0.5 * h * k2v));
      const double k4x = vel + (h * k3v);
      const double k4v = acc(pos + (h * k3x), vel + (h * k3v));
      pos += std::clamp(h * (k1x + (2.0 * k2x) + (2.0 * k3x) + k4x) / 6.0, -rate * h, rate * h);  // (it cannot move faster than the rate limit in a step either)
      vel += h * (k1v + (2.0 * k2v) + (2.0 * k3v) + k4v) / 6.0;
      vel = std::clamp(vel, -rate, rate);
      if (pos > limit) {
        pos = limit;
        vel = std::min(vel, 0.0);
      } else if (pos < -limit) {
        pos = -limit;
        vel = std::max(vel, 0.0);
      }
    } else {
      pos = slew(pos, cmd, h, limit, rate, lag);
    }
    const double half = 0.5 * ac.backlash_deg;
    if (pos - out > half) {
      out = pos - half;
    } else if (out - pos > half) {
      out = pos + half;
    }
  }

  [[nodiscard]] std::size_t lead_stage() const {  // the lowest stage still on the vehicle
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (active_[s]) {
        return s;
      }
    }
    return 0U;
  }

  [[nodiscard]] double capacity(std::size_t s) const { return cap_[s]; }  // what the tanks of stage s hold when full

  // The propellant stage `st` holds in state `s` (a vehicle of one stage has it from its total mass, as mass_props_state does).
  [[nodiscard]] double prop_of(const State& s, std::size_t st) const {
    return g_.stages.size() == 1U ? std::clamp(s.m - fixed_, 0.0, capacity(0U)) : s.prop[st];
  }

  // The liquid in tank `k` of stage `st` when the stage holds `total` kilograms (the same division as mass_props_of's: in proportion to the tanks' size, or the first tank listed last to empty).
  [[nodiscard]] double tank_liquid(std::size_t st, std::size_t k, double total) const {
    const StageSpec& sp = g_.stages[st];
    const double have = std::clamp(total, 0.0, capacity(st));
    if (!sp.sequential_drain) {
      return capacity(st) > 0.0 ? have * (sp.tanks[k].propellant / capacity(st)) : 0.0;
    }
    double left = have;
    double cap_after = capacity(st);
    double tm = 0.0;
    for (std::size_t j = 0; j <= k; ++j) {
      cap_after -= sp.tanks[j].propellant;
      tm = std::clamp(left - cap_after, 0.0, sp.tanks[j].propellant);
      left -= tm;
    }
    return tm;
  }

  // The mass of the stages and payloads still on the vehicle, with the given propellant in the stages that are on it.
  [[nodiscard]] double total_mass(const std::array<double, kMaxStages>& prop) const {
    double m = 0.0;
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (active_[s]) {
        m += g_.stages[s].dry_mass + std::clamp(prop[s], 0.0, capacity(s));
      }
    }
    for (std::size_t i = 0; i < g_.payloads.size(); ++i) {
      m += payload_active_[i] ? g_.payloads[i].mass : 0.0;
    }
    return m;
  }

  // The mass properties of a state, with one more kilogram of propellant in stage `bump` if it is >= 0 (for the rate of change of the inertia). A vehicle of one stage takes its
  // propellant from the total mass (m - structure, as the reference vehicle always did, so that its flights are the same to the last bit); a stack takes it from `prop`.
  [[nodiscard]] MassProps mass_props_state(const State& s, int bump, bool slosh_free = true) const {
    std::array<double, kMaxStages> prop = s.prop;
    if (g_.stages.size() == 1U) {
      prop[0] = std::clamp(s.m + (bump >= 0 ? 1.0 : 0.0) - fixed_, 0.0, capacity(0U));
    } else if (bump >= 0) {
      prop[static_cast<std::size_t>(bump)] += 1.0;
    }
    return mass_props_of(prop, slosh_free);
  }

  // After a step, a vehicle of one stage has the propellant its total mass says (it keeps the mass as the integrated quantity); a stack keeps both.
  void sync_propellant() {
    if (g_.stages.size() == 1U) {
      s_.prop[0] = std::clamp(s_.m - fixed_, 0.0, capacity(0U));
    }
  }

  // The mass that is not propellant (the structure of the stages and the payloads still on the vehicle): it changes only at a separation or a jettison.
  void refresh_fixed() {
    fixed_ = total_mass(std::array<double, kMaxStages>{});
    rebuild_aero();
  }

  // The mass, centre of gravity and inertias of the vehicle with the given propellant in each stage: the sum over its parts (each stage's structure, each tank's propellant column,
  // each payload) with the parallel-axis terms, in the order stage by stage, so that the reference vehicle's sums are those of the single-vehicle model it replaced.
  // With `slosh_free`, the liquid that sloshes is taken out of the rigid body (what the dynamics use); without it all of it is part of the body (what the gain design uses).
  [[nodiscard]] MassProps mass_props_of(const std::array<double, kMaxStages>& prop, bool slosh_free) const {
    struct Part {
      double m, x, it, ix;
    };
    Part parts[(kMaxStages * (1U + kMaxTanks)) + kMaxPayloads];  // filled up to n below; not zeroed (it is large and this is the hottest function)
    std::size_t n = 0;
    double slosh_total = 0.0;
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (!active_[s]) {
        continue;
      }
      const StageSpec& st = g_.stages[s];
      const double r = st.radius;
      const double xd = (st.x_cg_dry >= 0.0 ? st.x_cg_dry : st.x_start + (0.5 * st.length)) + sc_.dry_cg_shift;
      parts[n++] = Part{st.dry_mass, xd, st.dry_mass * st.length * st.length / 12.0 * st.inertia_factor, st.dry_mass * r * r};
      const double total = std::clamp(prop[s], 0.0, capacity(s));
      double left = total;
      double cap_after = capacity(s);
      for (std::size_t k = 0; k < st.tanks.size(); ++k) {
        const TankSpec& tk = st.tanks[k];
        cap_after -= tk.propellant;
        double tm = 0.0;
        if (st.sequential_drain) {
          tm = std::clamp(left - cap_after, 0.0, tk.propellant);  // the tanks after this one fill first from the end: this one holds what they cannot
          left -= tm;
        } else {
          tm = capacity(s) > 0.0 ? total * (tk.propellant / capacity(s)) : 0.0;
        }
        const double tr = tk.radius;
        const double area = kPi * tr * tr;
        const double hp = tm / (tk.density * area);
        const double xc = tk.x_bottom + (0.5 * hp);
        const double it = tm * ((3.0 * tr * tr) + (hp * hp)) / 12.0;
        const SloshModel sm = slosh_free && tk.slosh.enabled ? slosh_model(tk, tm) : SloshModel{};
        if (sm.m1 > 0.0) {  // the rest of the liquid: its mass, its centre and its inertia with the sloshing mass taken out of the column (parallel axes)
          const double mr = tm - sm.m1;
          const double xr = ((tm * xc) - (sm.m1 * sm.x_s)) / mr;
          slosh_total += sm.m1;
          parts[n++] = Part{mr, xr, it + (tm * (xc - xr) * (xc - xr)) - (sm.m1 * (sm.x_s - xr) * (sm.x_s - xr)), 0.5 * tm * tr * tr};
        } else {
          parts[n++] = Part{tm, xc, it, 0.5 * tm * tr * tr};
        }
      }
    }
    for (std::size_t i = 0; i < g_.payloads.size(); ++i) {
      if (payload_active_[i]) {
        parts[n++] = Part{g_.payloads[i].mass, g_.payloads[i].x, 0.0, 0.0};
      }
    }
    MassProps mp;
    double mx = 0.0;
    double m = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      m += parts[i].m;
      mx += parts[i].m * parts[i].x;
    }
    mp.mass = m;
    mp.slosh_mass = slosh_total;
    mp.x_cg = m > 0.0 ? mx / m : 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      mp.i_t = mp.i_t + parts[i].it + (parts[i].m * (parts[i].x - mp.x_cg) * (parts[i].x - mp.x_cg));
      mp.i_x += parts[i].ix;
    }
    return mp;
  }

  // The time-line of the vehicle: engine failures, the ignition of each stage, the burnout that follows, the separation of a spent stage, the jettison of a payload.
  void process_events() {
    for (const EngineFailure& f : failures_) {
      if (f.index >= 0 && f.index < static_cast<int>(failed_.size()) && t_ >= f.time) {
        failed_[static_cast<std::size_t>(f.index)] = true;
      }
    }
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (!active_[s]) {
        continue;
      }
      const StageSpec& st = g_.stages[s];
      if (t_ign_[s] < 0.0) {
        bool go = false;
        if (st.ignite_after_sep_of >= 0) {
          const double ts = t_sep_[static_cast<std::size_t>(st.ignite_after_sep_of)];
          go = ts >= 0.0 && t_ >= ts + st.ignite_delay_s;
        } else {
          go = t_ >= st.ignite_time_s;
        }
        if (go) {
          t_ign_[s] = t_;
        }
      }
      if (t_ign_[s] >= 0.0 && t_burnout_[s] < 0.0 && s_.prop[s] <= kEmptyProp) {
        t_burnout_[s] = t_;
      }
      const bool by_time = st.separate_time_s >= 0.0 && t_ >= st.separate_time_s;
      const bool by_burnout = st.separate_on_burnout && t_burnout_[s] >= 0.0 && t_ >= t_burnout_[s] + st.separate_delay_s;
      if (by_time || by_burnout) {
        separate_stage(s);
      }
    }
    for (std::size_t i = 0; i < g_.payloads.size(); ++i) {
      if (payload_active_[i] && g_.payloads[i].jettison_time_s >= 0.0 && t_ >= g_.payloads[i].jettison_time_s) {
        payload_active_[i] = false;
        refresh_fixed();
        s_.m = total_mass(s_.prop);
      }
    }
  }

  // ---- events, surfaces, parachutes ----

  // The rising edges of the event bits: each acts once.
  void apply_events(uint32_t events) {
    const uint32_t rising = events & ~ev_prev_;
    ev_prev_ = events;
    if ((rising & ev::kSeparate) != 0U) {
      std::size_t active = 0;
      for (std::size_t k = 0; k < g_.stages.size(); ++k) {
        active += active_[k] ? 1U : 0U;
      }
      if (active > 1U) {
        separate_stage(lead_stage());
      }
    }
    if ((rising & ev::kDeploySurfaces) != 0U) {
      surf_deployed_.fill(true);
    }
    if ((rising & ev::kExtendLegs) != 0U) {
      legs_out_ = true;
    }
    const std::array<uint32_t, 2> chute_bit{ev::kChute0, ev::kChute1};
    for (std::size_t k = 0; k < chute_bit.size(); ++k) {
      if ((rising & chute_bit[k]) != 0U && k < g_.parachutes.size() && active_[static_cast<std::size_t>(g_.parachutes[k].stage)] && chute_open_[k] < 0.0) {
        chute_open_[k] = t_;
      }
    }
    if ((rising & ev::kJettison) != 0U) {
      for (std::size_t i = 0; i < g_.payloads.size(); ++i) {
        if (payload_active_[i]) {
          payload_active_[i] = false;
          refresh_fixed();
          s_.m = total_mass(s_.prop);
          break;
        }
      }
    }
  }

  [[nodiscard]] Parachute chute_of(std::size_t i) const { return Parachute{g_.parachutes[i].drag_area, g_.parachutes[i].inflation_s, chute_open_[i]}; }

  // Each surface moves toward its commanded angle (or its stowed angle until it is deployed) at its rate limit, through its lag.
  void drive_surfaces(double h) {
    for (std::size_t i = 0; i < g_.surfaces.size(); ++i) {
      const SurfaceSpec& sf = g_.surfaces[i];
      if (!active_[static_cast<std::size_t>(sf.stage)]) {
        continue;
      }
      double target = sf.stow_deg;
      if (surf_deployed_[i] && sf.channel >= 0 && sf.channel < static_cast<int>(kSurfaceChannels)) {
        target = std::clamp(ctrl_.surface_deg[static_cast<std::size_t>(sf.channel)], sf.min_deg, sf.max_deg);
      }
      const double want = sf.lag_s > 0.0 ? (target - surf_pos_[i]) * (h / (sf.lag_s + h)) : (target - surf_pos_[i]);
      surf_pos_[i] += std::clamp(want, -sf.rate_dps * h, sf.rate_dps * h);
    }
  }

  void add_surface_loads(Loads& l, const State& s, const Air& air, const V3& vrel, const MassProps& mp) const {
    for (std::size_t i = 0; i < g_.surfaces.size(); ++i) {
      const SurfaceSpec& sf = g_.surfaces[i];
      if (!active_[static_cast<std::size_t>(sf.stage)] || (!surf_deployed_[i] && sf.kind == SurfaceKind::GridFin)) {
        continue;   // (a grid fin folded against the skin does nothing)
      }
      const V3 pt = surface_point(sf, surf_pos_[i]);
      const V3 rc{pt.x - mp.x_cg, pt.y, pt.z};
      const V3 v_air = vrel + cross(s.w, rc);   // the velocity of the surface's own point through the air
      const SurfaceFrame fr = surface_frame(sf, surf_pos_[i]);
      const SurfaceLoad sl = surface_load(sf, surf_pos_[i], v_air, air.density, air.sound, stream_fraction(vrel, fr.radial));
      l.f_aero = l.f_aero + sl.force;
      l.m_aero = l.m_aero + cross(rc, sl.force);
    }
  }

  void add_chute_loads(Loads& l, const State& s, const Air& air, const V3& vrel, const MassProps& mp, double t) const {
    for (std::size_t i = 0; i < g_.parachutes.size(); ++i) {
      if (chute_open_[i] < 0.0 || !active_[static_cast<std::size_t>(g_.parachutes[i].stage)]) {
        continue;
      }
      const V3 rc{g_.parachutes[i].x_attach - mp.x_cg, 0.0, 0.0};
      const V3 f = parachute_force(chute_of(i), t, vrel + cross(s.w, rc), air.density);
      l.f_aero = l.f_aero + f;
      l.m_aero = l.m_aero + cross(rc, f);
    }
  }

  // ---- engines the flight computers drive ----

  double start_uniform() {
    start_rng_ ^= start_rng_ << 13U;
    start_rng_ ^= start_rng_ >> 7U;
    start_rng_ ^= start_rng_ << 17U;
    return (static_cast<double>(start_rng_ >> 11U) + 0.5) / 9007199254740992.0;
  }

  // One engine of a stage that the computers drive: it starts when its group is commanded on (if it has starts left, and the start takes: the scenario can make a start fail), runs at the throttle
  // commanded (raised to the engine's lowest, `min_throttle`) and stops when its group is commanded off or the propellant is gone.
  void update_guided_engine(std::size_t e, double h) {
    const EngineSpec& en = g_.engines[e];
    const std::size_t st = static_cast<std::size_t>(en.stage);
    const bool bit = ((ctrl_.group_mask >> static_cast<unsigned>(en.group)) & 1U) != 0U;
    const bool fuel = s_.prop[st] > kEmptyProp;
    const bool want = bit && fuel && !failed_[e] && ctrl_.throttle > 0.0;
    if (want && !eng_on_[e] && !start_failed_[e]) {
      if (en.max_starts > 0 && starts_[e] >= en.max_starts) {
        start_failed_[e] = true;
      } else {
        ++starts_[e];
        if (sc_.start_failure_prob > 0.0 && start_uniform() < sc_.start_failure_prob) {
          start_failed_[e] = true;
        } else {
          eng_on_[e] = true;
          eng_t_on_[e] = t_;
          if (t_ign_[st] < 0.0) {
            t_ign_[st] = t_;
          }
        }
      }
    } else if (!want) {
      eng_on_[e] = false;
      if (!bit) {
        start_failed_[e] = false;   // commanded off: the next time it is commanded on is a new attempt
      }
    }
    const bool running = eng_on_[e] && t_ >= eng_t_on_[e] + en.start_offset_s && fuel;
    const double cmd = running ? std::clamp(std::max(ctrl_.throttle, en.min_throttle), 0.0, 1.0) : 0.0;
    const double tau = cmd > frac_[e] ? en.rise_s : en.tail_s;
    frac_[e] = (tau > 0.0 && !failed_[e]) ? frac_[e] + ((cmd - frac_[e]) * (1.0 - std::exp(-h / tau))) : cmd;
  }

  // ---- the ground, the legs and the tower ----

  // The height of the centre of gravity above the lowest point of the vehicle standing upright (the pad's height of the centre of gravity above the ground).
  [[nodiscard]] double ground_offset() const {
    double aft = 1.0e30;
    for (std::size_t k = 0; k < g_.stages.size(); ++k) {
      if (active_[k]) {
        aft = std::min(aft, g_.stages[k].x_start);
      }
    }
    return aft < 1.0e29 ? mass_props_state(s_, -1).x_cg - aft : 0.0;
  }

  // The aft end of the lowest stage on the vehicle, the foot of its legs if they are out, and the forward end of the highest.
  void ends(double& foot_x, double& fore_x, double& radius) const {
    foot_x = 1.0e30;
    fore_x = -1.0e30;
    radius = 0.0;
    for (std::size_t k = 0; k < g_.stages.size(); ++k) {
      if (!active_[k]) {
        continue;
      }
      const StageSpec& st = g_.stages[k];
      const double foot = st.x_start + (legs_out_ ? st.leg_x : 0.0);
      foot_x = std::min(foot_x, foot);
      fore_x = std::max(fore_x, st.x_start + st.length);
      radius = std::max(radius, st.radius);
    }
  }

  [[nodiscard]] double lowest_point_height() const {
    double foot = 0.0;
    double fore = 0.0;
    double rad = 0.0;
    ends(foot, fore, rad);
    if (foot > 1.0e29) {
      return 1.0e30;
    }
    const double x_cg = mass_props_state(s_, -1).x_cg;
    const V3 up = s_.r / norm(s_.r);
    const V3 u = rotate(s_.q, V3{1.0, 0.0, 0.0});
    const double uz = dot(u, up);
    const double rim = rad * std::sqrt(std::max(1.0 - (uz * uz), 0.0));   // a disc tilted from the horizontal reaches this much lower at its edge
    const double h_foot = norm(s_.r + (u * (foot - x_cg))) - ground_radius_ - rim;
    const double h_fore = norm(s_.r + (u * (fore - x_cg))) - ground_radius_ - rim;
    return std::min(h_foot, h_fore);
  }

  [[nodiscard]] bool pin_x(double& x) const {
    for (std::size_t k = 0; k < g_.stages.size(); ++k) {
      if (active_[k] && g_.stages[k].catch_pin_x > 0.0) {
        x = g_.stages[k].x_start + g_.stages[k].catch_pin_x;
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] V3 pin_position() const {
    double x = 0.0;
    (void)pin_x(x);
    return s_.r + rotate(s_.q, V3{x - mass_props_state(s_, -1).x_cg, 0.0, 0.0});
  }

  [[nodiscard]] double pin_height_now() const {
    double x = 0.0;
    return pin_x(x) ? norm(pin_position()) - ground_radius_ : 1.0e30;
  }

  // The pad's axes now: up, downrange, crossrange (a rotating planet carries them round its pole).
  void pad_frame(V3& up, V3& down, V3& cross_r) const {
    const Q4 turn = rotating_ ? from_axis_angle(pole_, pl_.rotation_rate * t_) : Q4{};
    up = rotate(turn, V3{1.0, 0.0, 0.0});
    down = rotate(turn, V3{0.0, 1.0, 0.0});
    cross_r = rotate(turn, V3{0.0, 0.0, 1.0});
  }

  // Held to the ground, a vehicle goes round with the planet.
  void co_rotate(double h) {
    if (rotating_) {
      const Q4 turn = from_axis_angle(pole_, pl_.rotation_rate * h);
      s_.r = rotate(turn, s_.r);
      s_.q = turn * s_.q;
      s_.v = cross(omega_, s_.r);
      s_.w = rotate_inv(s_.q, omega_);
    }
  }

  // The vehicle that has left the pad meets the ground or the tower. Returns true while it is held (the equations of motion are not integrated).
  bool resolve_contact(double h) {
    if (crashed_ || landed_ || caught_) {
      s_.v = V3{};
      s_.w = V3{};
      co_rotate(h);
      return true;
    }
    V3 up_pad;
    V3 down_pad;
    V3 cross_pad;
    pad_frame(up_pad, down_pad, cross_pad);
    const V3 v_ground = rotating_ ? cross(omega_, s_.r) : V3{};
    if (sc_.tower.enabled) {
      double xp = 0.0;
      if (pin_x(xp)) {
        const double hp = pin_height_now();
        if (prev_pin_h_ >= sc_.tower.height_m && hp < sc_.tower.height_m) {   // the pins come down through the arms
          const V3 p_rel = pin_position() - (up_pad * ground_radius_);
          const double miss = std::hypot(dot(p_rel, down_pad) - sc_.tower.offset_y_m, dot(p_rel, cross_pad) - sc_.tower.offset_z_m);
          const V3 v_rel = s_.v - v_ground;
          const double sink = -dot(v_rel, up_pad);
          const double lat = norm(v_rel + (up_pad * sink));
          const double tilt = std::acos(std::clamp(dot(rotate(s_.q, V3{1.0, 0.0, 0.0}), up_pad), -1.0, 1.0)) * kRad2Deg;
          if (miss <= sc_.tower.capture_radius_m && sink >= 0.0 && sink <= sc_.tower.max_sink_ms && lat <= sc_.tower.max_lateral_ms && tilt <= sc_.tower.max_tilt_deg) {
            caught_ = true;
            s_.r = s_.r + (up_pad * (sc_.tower.height_m - hp));
            s_.v = v_ground;
            s_.w = V3{};
            prev_pin_h_ = sc_.tower.height_m;
            return true;
          }
          catch_missed_ = true;
        }
        prev_pin_h_ = hp;
      }
    }
    const double hb = lowest_point_height();
    if (hb > 0.0) {
      return false;
    }
    const V3 up = s_.r / norm(s_.r);
    const V3 v_rel = s_.v - v_ground;
    const double v_up = dot(v_rel, up);
    if (v_up > 0.0) {
      return false;   // moving away from the ground (a hop)
    }
    const double lateral = norm(v_rel - (up * v_up));
    const double tilt = std::acos(std::clamp(dot(rotate(s_.q, V3{1.0, 0.0, 0.0}), up), -1.0, 1.0)) * kRad2Deg;
    const bool ok = -v_up <= p_.crash_speed_ms && lateral <= p_.landing_lateral_ms && tilt <= p_.landing_tilt_deg;
    landed_ = ok;
    crashed_ = !ok;
    s_.r = s_.r - (up * hb);   // the lowest point rests on the ground
    s_.v = v_ground;
    s_.w = V3{};
    return true;
  }

  // Let stage `s` go. The vehicle that stays is lighter and (with the landing model, which keeps the position of the centre of gravity honest) its centre of gravity moves to where the remaining parts'
  // centre of gravity is: position and velocity shift by the offset along the axis, and by the turning of the body. A stage let go is recorded (`take_detached`) with everything needed to fly it on.
  void separate_stage(std::size_t s) {
    const StageSpec& st = g_.stages[s];
    const MassProps before = mass_props_state(s_, -1);
    Detached d;
    d.stage = static_cast<int>(s);
    d.t = t_;
    d.state = s_;
    d.x_cg_parent = before.x_cg;
    d.ground_radius = ground_radius_;
    d.launched = launched_;
    d.x_off = st.x_start;
    active_[s] = false;
    t_sep_[s] = t_;
    refresh_fixed();
    s_.m = total_mass(s_.prop);
    const MassProps after = mass_props_state(s_, -1);
    if (p_.landing_model && after.mass > 0.0) {
      const V3 off_body{after.x_cg - before.x_cg, 0.0, 0.0};   // the centre of gravity of what is left, from that of the whole
      s_.r = s_.r + rotate(s_.q, off_body);
      s_.v = s_.v + rotate(s_.q, cross(s_.w, off_body));
    }
    const double m_child = st.dry_mass + std::clamp(d.state.prop[s], 0.0, capacity(s));
    if (st.separation_dv_ms != 0.0) {  // the springs push the rest of the vehicle forward (and the stage back, by the same momentum)
      s_.v = s_.v + rotate(s_.q, V3{st.separation_dv_ms, 0.0, 0.0});
      d.recoil_ms = m_child > 0.0 ? st.separation_dv_ms * s_.m / m_child : 0.0;
    }
    if (st.tipoff_pitch_dps != 0.0 || st.tipoff_yaw_dps != 0.0 || st.tipoff_roll_dps != 0.0) {  // and the release leaves it turning (roll about X, yaw about Y, pitch about Z)
      s_.w = s_.w + (V3{st.tipoff_roll_dps, st.tipoff_yaw_dps, st.tipoff_pitch_dps} * kDeg2Rad);
      if (ideal_roll_) {
        s_.w.x = 0.0;  // a roll that is held ideally has no roll rate to hand on
      }
    }
    d.spec = stage_alone(s, d.x_off);
    const double prop_s = d.state.prop[s];   // (the copy was taken before the stage left)
    d.state.prop = std::array<double, kMaxStages>{};
    d.state.prop[0] = prop_s;
    d.state.prop_set = true;
    d.state.m = m_child;
    detached_.push_back(d);
  }

  // The description of stage `s` on its own: its parts and the engines, surfaces and parachutes of the stage, with every x measured from its own aft end.
  [[nodiscard]] VehicleSpec stage_alone(std::size_t s, double x_off) const {
    VehicleSpec v;
    v.planet = g_.planet;
    v.aero = g_.aero;
    v.actuator = g_.actuator;
    v.jet_damping = g_.jet_damping;
    StageSpec st = g_.stages[s];
    st.x_start = 0.0;
    if (st.x_cg_dry >= 0.0) {
      st.x_cg_dry -= x_off;
    }
    for (TankSpec& t : st.tanks) {
      t.x_bottom -= x_off;
    }
    for (SectionSpec& sec : st.sections) {
      sec.x_start -= x_off;
    }
    for (FinPlanform& f : st.stabilizers) {
      f.x_le_root -= x_off;
    }
    st.guided = true;   // flown on its own, its engines are whatever its own computers command (none, if it has none)
    st.ignite_after_sep_of = -1;
    st.ignite_time_s = 0.0;
    st.separate_time_s = -1.0;
    st.separate_on_burnout = false;
    st.separation_dv_ms = 0.0;
    st.tipoff_pitch_dps = 0.0;
    st.tipoff_yaw_dps = 0.0;
    st.tipoff_roll_dps = 0.0;
    v.stages.push_back(st);
    for (const EngineSpec& e : g_.engines) {
      if (e.stage == static_cast<int>(s)) {
        EngineSpec k = e;
        k.stage = 0;
        k.pos.x -= x_off;
        v.engines.push_back(k);
      }
    }
    for (const SurfaceSpec& sf : g_.surfaces) {
      if (sf.stage == static_cast<int>(s)) {
        SurfaceSpec k = sf;
        k.stage = 0;
        k.x_hinge -= x_off;
        v.surfaces.push_back(k);
      }
    }
    for (const ParachuteSpec& pc : g_.parachutes) {
      if (pc.stage == static_cast<int>(s)) {
        ParachuteSpec k = pc;
        k.stage = 0;
        k.x_attach -= x_off;
        v.parachutes.push_back(k);
      }
    }
    return v;
  }

  // The aerodynamic force and moment from the shape (or a table by Mach): the normal force of the slender parts, the cross-flow force of the body in the flow across it, the axial force
  // flying nose first or tail first, each at any angle of attack when `full_angle` (otherwise as the reference model: linear in the angle, none past 90 degrees).
  void shape_forces(Loads& l, const Air& air, const V3& vrel, double v_abs, const MassProps& mp) const {
    const AeroSpec& a = g_.aero;
    const double lat = std::hypot(vrel.y, vrel.z);
    const double alpha = std::atan2(lat, vrel.x);
    l.alpha = alpha;
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    double s_ref = 0.0;
    double cn_alpha = 0.0;
    double moment_slope = 0.0;
    double axial_front = 0.0;
    double plan_ratio = 0.0;
    double x_cf = mp.x_cg;
    if (geo_.ready()) {
      s_ref = geo_.reference_area();
      geo_.slopes(l.mach, mp.x_cg, cn_alpha, moment_slope);
      const double re = air.density * v_abs * geo_.length() / aero::viscosity(air.temperature);
      axial_front = geo_.axial(l.mach, re, power_fraction(), a);
      plan_ratio = geo_.plan_area() / s_ref;
      x_cf = geo_.plan_centroid();
    } else {
      s_ref = kPi * 0.25 * p_.diameter * p_.diameter;
      table_lookup(l.mach, axial_front, cn_alpha, moment_slope, mp.x_cg);
    }
    const double q_s = l.dynamic_pressure * s_ref;
    const bool forward = vrel.x > 0.0;
    const double lin = a.full_angle ? sa * ca : (forward ? alpha : 0.0);
    const double crossflow = a.full_angle ? a.crossflow_eta * a.crossflow_cd * plan_ratio * sa * sa : 0.0;
    const double n_total = (p_.cn_scale * cn_alpha * lin) + crossflow;
    const double arm_sum = (p_.cn_scale * moment_slope * lin) + (crossflow * (x_cf - mp.x_cg));
    if (a.full_angle) {
      l.f_aero.x = -q_s * p_.cd_scale * (ca >= 0.0 ? axial_front : a.rear_axial) * ca;
    } else if (forward) {
      l.f_aero.x = -q_s * p_.cd_scale * axial_front;
    }
    if (lat > 1e-9) {
      const V3 n{0.0, vrel.y / lat, vrel.z / lat};  // the lateral direction of the relative wind: the normal force opposes it
      l.f_aero.y = -q_s * n_total * n.y;
      l.f_aero.z = -q_s * n_total * n.z;
      l.m_aero = cross(V3{-q_s * arm_sum, 0.0, 0.0}, n);
    }
  }

  // The aerodynamic force and moment over the whole speed range and at any angle of attack (aero.full_regime): the slender-body build-up and the cross-flow of the body at the Mach number of the flow across
  // it, blended from Mach 3 to 6 into the Newtonian impact theory of the body in the stream (newtonian.hpp), and, added to either, what the body's own turning adds (the damping of its parts and of the
  // strips of its body in the local flow). The angle of attack runs the full circle; flying tail first the base and the engines face the stream.
  void shape_forces_full(Loads& l, const Air& air, const V3& vrel, double v_abs, const MassProps& mp, const V3& w) const {
    const AeroSpec& a = g_.aero;
    const double lat = std::hypot(vrel.y, vrel.z);
    const double alpha = std::atan2(lat, vrel.x);
    l.alpha = alpha;
    const double sa = std::sin(alpha);
    const double ca = std::cos(alpha);
    const double s_ref = geo_.reference_area();
    double cn_alpha = 0.0;
    double moment_slope = 0.0;
    geo_.slopes(l.mach, mp.x_cg, cn_alpha, moment_slope);
    const double re = air.density * v_abs * geo_.length() / aero::viscosity(air.temperature);
    const double axial_front = geo_.axial(l.mach, re, power_fraction(), a);
    const double plan_ratio = geo_.plan_area() / s_ref;
    const double x_cf = geo_.plan_centroid();
    const double q_s = l.dynamic_pressure * s_ref;
    const double lin = sa * ca;
    const double cdc = aero::cross_cd(l.mach * std::fabs(sa));
    const double crossflow = a.crossflow_eta * cdc * plan_ratio * sa * sa;
    double n_total = (p_.cn_scale * cn_alpha * lin) + crossflow;
    double arm_sum = (p_.cn_scale * moment_slope * lin) + (crossflow * (x_cf - mp.x_cg));
    double x_coef = p_.cd_scale * (ca >= 0.0 ? axial_front : a.rear_axial) * ca;
    const double wn = aero::smoothstep(a.newtonian_from_mach, a.newtonian_to_mach, l.mach);
    if (wn > 0.0 && newton_.ready()) {
      double cx = 0.0;
      double cn = 0.0;
      double a0 = 0.0;
      newton_.at(alpha, cx, cn, a0);
      const double cx_n = p_.cd_scale * (cx + (geo_.skin(l.mach, re, a) * ca));
      n_total = ((1.0 - wn) * n_total) + (wn * p_.cn_scale * cn);
      arm_sum = ((1.0 - wn) * arm_sum) + (wn * p_.cn_scale * (a0 - (mp.x_cg * cn)));
      x_coef = ((1.0 - wn) * x_coef) + (wn * cx_n);
    }
    l.f_aero.x = -q_s * x_coef;
    if (lat > 1e-9) {
      const V3 n{0.0, vrel.y / lat, vrel.z / lat};
      l.f_aero.y = -q_s * n_total * n.y;
      l.f_aero.z = -q_s * n_total * n.z;
      l.m_aero = cross(V3{-q_s * arm_sum, 0.0, 0.0}, n);
    }
    if (w.y != 0.0 || w.z != 0.0) {
      V3 df;
      V3 dm;
      geo_.rotation_increment(l.mach, air.sound, air.density, vrel, w, mp.x_cg, a, p_.cn_scale, df, dm);
      l.f_aero = l.f_aero + df;
      l.m_aero = l.m_aero + dm;
    }
  }

  // The fraction of a stage's main engines' rated thrust that is running now.
  [[nodiscard]] double stage_power(std::size_t st) const {
    double running = 0.0;
    double rated = 0.0;
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      const EngineSpec& en = g_.engines[e];
      if (en.control == Control::None && static_cast<std::size_t>(en.stage) == st) {
        running += frac_[e] * en.thrust_vac;
        rated += en.thrust_vac;
      }
    }
    return rated > 0.0 ? running / rated : 0.0;
  }

  // The fraction of the main engines' rated thrust that is running (the plume fills the wake and takes drag off the base).
  [[nodiscard]] double power_fraction() const {
    double running = 0.0;
    double rated = 0.0;
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      const EngineSpec& en = g_.engines[e];
      if (en.control == Control::None && active_[static_cast<std::size_t>(en.stage)]) {
        running += frac_[e] * en.thrust_vac;
        rated += en.thrust_vac;
      }
    }
    return rated > 0.0 ? running / rated : 0.0;
  }

  // A table by Mach (aero.table): linear between its points, held beyond them.
  void table_lookup(double mach, double& ca, double& cn_alpha, double& moment_slope, double x_cg) const {
    const std::vector<AeroTablePoint>& t = g_.aero.table;
    AeroTablePoint p = t.front();
    if (mach >= t.back().mach) {
      p = t.back();
    } else if (mach > t.front().mach) {
      for (std::size_t i = 1; i < t.size(); ++i) {
        if (mach <= t[i].mach) {
          const double f = (mach - t[i - 1].mach) / (t[i].mach - t[i - 1].mach);
          p.ca = t[i - 1].ca + (f * (t[i].ca - t[i - 1].ca));
          p.cn_alpha = t[i - 1].cn_alpha + (f * (t[i].cn_alpha - t[i - 1].cn_alpha));
          p.x_cp = t[i - 1].x_cp + (f * (t[i].x_cp - t[i - 1].x_cp));
          break;
        }
      }
    }
    ca = p.ca;
    cn_alpha = p.cn_alpha;
    moment_slope = p.cn_alpha * (p.x_cp - x_cg);
  }

  // The aerodynamic geometry of the stages that are on the vehicle: rebuilt whenever one leaves.
  void rebuild_aero() {
    std::vector<SectionSpec> sections;
    std::vector<FinPlanform> fins;
    for (std::size_t s = 0; s < g_.stages.size(); ++s) {
      if (active_[s]) {
        sections.insert(sections.end(), g_.stages[s].sections.begin(), g_.stages[s].sections.end());
        fins.insert(fins.end(), g_.stages[s].stabilizers.begin(), g_.stages[s].stabilizers.end());
      }
    }
    for (std::size_t i = 0; i < g_.payloads.size(); ++i) {
      if (payload_active_[i]) {
        sections.insert(sections.end(), g_.payloads[i].sections.begin(), g_.payloads[i].sections.end());
      }
    }
    geo_.build(sections, fins, g_.aero.reference_diameter_m);
    shaped_ = geo_.ready() || !g_.aero.table.empty();
    if (g_.aero.full_regime && geo_.ready()) {
      newton_.build(sections, fins, geo_.reference_area());
    }
  }

  // Could engine e fire now (its stage is on the vehicle, lit and has propellant, and the engine has not failed)?
  [[nodiscard]] bool engine_available(std::size_t e) const {
    const EngineSpec& en = g_.engines[e];
    const std::size_t st = static_cast<std::size_t>(en.stage);
    return active_[st] && t_ign_[st] >= 0.0 && t_ >= t_ign_[st] + en.start_offset_s && s_.prop[st] > kEmptyProp && !failed_[e];
  }

  [[nodiscard]] double throttle_of(const StageSpec& st, double since_ignition) const {
    if (st.throttle.empty()) {
      return 1.0;
    }
    if (since_ignition <= st.throttle.front()[0]) {
      return st.throttle.front()[1];
    }
    for (std::size_t i = 1; i < st.throttle.size(); ++i) {
      if (since_ignition <= st.throttle[i][0]) {
        const double f = (since_ignition - st.throttle[i - 1][0]) / (st.throttle[i][0] - st.throttle[i - 1][0]);
        return st.throttle[i - 1][1] + (f * (st.throttle[i][1] - st.throttle[i - 1][1]));
      }
    }
    return st.throttle.back()[1];
  }

  // One standard normal draw (Box-Muller from a xorshift generator: the same numbers on every machine for a seed).
  double gauss() {
    const auto uniform = [this]() {
      rng_ ^= rng_ << 13U;
      rng_ ^= rng_ >> 7U;
      rng_ ^= rng_ << 17U;
      return (static_cast<double>(rng_ >> 11U) + 0.5) / 9007199254740992.0;  // (0, 1)
    };
    const double u1 = uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
  }

  // Advance the turbulence over h seconds: each component keeps `exp(-V h / L)` of itself and takes the rest from a fresh draw, so that its variance stays sigma^2 and its correlation
  // falls by 1/e over a distance of L at the airspeed V (exact for a first-order Gauss-Markov process, at any step).
  void step_turbulence(double h) {
    const Turbulence& tb = sc_.turbulence;
    if (!(tb.sigma_ms > 0.0)) {
      return;
    }
    const V3 v_air = rotating_ ? s_.v - cross(omega_, s_.r) : s_.v;
    const double speed = std::max(norm(v_air), 20.0);
    const double a = std::exp(-speed * h / tb.scale_length_m);
    const double k = tb.sigma_ms * std::sqrt(1.0 - (a * a));
    turb_ = V3{(a * turb_.x) + (k * gauss()), (a * turb_.y) + (k * gauss()), (a * turb_.z) + (k * gauss())};
  }

  // The thrust fraction of each engine for the next substep of h seconds: 1 (or the throttle) while it is lit, with the rise and tail-off time constants, 0 once its stage has gone.
  void update_engines(double h, double cmd_pitch_deg, double cmd_yaw_deg) {
    for (std::size_t e = 0; e < g_.engines.size(); ++e) {
      const EngineSpec& en = g_.engines[e];
      const std::size_t st = static_cast<std::size_t>(en.stage);
      if (!active_[st]) {
        frac_[e] = 0.0;
        continue;
      }
      const bool guided = g_.stages[st].guided;
      if (guided && en.control == Control::None) {
        update_guided_engine(e, h);
        continue;
      }
      // (a thruster of a stage the computers drive needs only propellant in the stage: it works before the main engines have ever been lit)
      const bool lit = guided ? (s_.prop[st] > kEmptyProp && !failed_[e])
                              : (t_ign_[st] >= 0.0 && t_ >= t_ign_[st] + en.start_offset_s && s_.prop[st] > kEmptyProp && !failed_[e] && (en.cutoff_time_s < 0.0 || t_ < en.cutoff_time_s));
      double duty = 1.0;  // a thruster fires in proportion to the command that asks for it
      switch (en.control) {
        case Control::None:
          break;
        case Control::PitchPlus:
          duty = std::clamp(cmd_pitch_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
        case Control::PitchMinus:
          duty = std::clamp(-cmd_pitch_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
        case Control::YawPlus:
          duty = std::clamp(cmd_yaw_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
        case Control::YawMinus:
          duty = std::clamp(-cmd_yaw_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
        case Control::RollPlus:
          duty = std::clamp(ctrl_.roll_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
        case Control::RollMinus:
          duty = std::clamp(-ctrl_.roll_deg / en.full_cmd_deg, 0.0, 1.0);
          break;
      }
      if (guided && duty > 0.0 && (en.control == Control::PitchPlus || en.control == Control::PitchMinus || en.control == Control::YawPlus || en.control == Control::YawMinus) && stage_power(st) > 0.05) {
        duty = 0.0;   // the gimbal steers while the main engines burn: the pitch and yaw thrusters are inhibited then (the roll ones are not: the gimbal has no roll)
      }
      const double cmd = lit ? (guided ? duty : throttle_of(g_.stages[st], t_ - t_ign_[st]) * duty) : 0.0;
      const double tau = cmd > frac_[e] ? en.rise_s : en.tail_s;
      frac_[e] = (tau > 0.0 && failed_[e] == false) ? frac_[e] + ((cmd - frac_[e]) * (1.0 - std::exp(-h / tau))) : cmd;
    }
  }

  // The torque the reaction wheels put on the vehicle for the commands (body frame: yaw about +Y, pitch about +Z), none in the direction a wheel is full of momentum.
  [[nodiscard]] V3 wheel_torque(double cmd_pitch_deg, double cmd_yaw_deg) const {
    const WheelSpec& w = g_.wheels;
    if (!w.enabled || !active_[static_cast<std::size_t>(w.stage)]) {
      return V3{};
    }
    double m_pitch = std::clamp(cmd_pitch_deg / w.full_cmd_deg, -1.0, 1.0) * w.torque_max;
    double m_yaw = std::clamp(cmd_yaw_deg / w.full_cmd_deg, -1.0, 1.0) * w.torque_max;
    // a wheel takes the opposite momentum of the torque on the vehicle: it cannot take more than its limit
    if ((s_.wheel_h.z <= -w.momentum_max && m_pitch > 0.0) || (s_.wheel_h.z >= w.momentum_max && m_pitch < 0.0)) {
      m_pitch = 0.0;
    }
    if ((s_.wheel_h.y <= -w.momentum_max && m_yaw > 0.0) || (s_.wheel_h.y >= w.momentum_max && m_yaw < 0.0)) {
      m_yaw = 0.0;
    }
    return V3{0.0, m_yaw, m_pitch};
  }
  void take_up_wheel_momentum(double h) {
    if (g_.wheels.enabled) {
      s_.wheel_h.y = std::clamp(s_.wheel_h.y - (wheel_m_.y * h), -g_.wheels.momentum_max, g_.wheels.momentum_max);
      s_.wheel_h.z = std::clamp(s_.wheel_h.z - (wheel_m_.z * h), -g_.wheels.momentum_max, g_.wheels.momentum_max);
    }
  }

  // The ground, for one substep of h seconds. A vehicle that is at the surface and not moving away from it, with no net force lifting it, stays where it is (position and attitude held,
  // no velocity or rotation) while its engines burn propellant; it is released the moment the net force along the local vertical is upward. One that arrives moving down is landed,
  // or destroyed above `crash_speed_ms`. Returns true while the ground holds the vehicle (the equations of motion are not integrated). Nothing here changes the state of a vehicle
  // that is flying, or that is at the surface with a net upward force and no downward speed (a normal lift-off), so the nominal flight is arithmetic for arithmetic what it was.
  bool held_by_ground(double h) {
    if (p_.landing_model && launched_) {
      return resolve_contact(h);   // after lift-off the ground is met by the lowest point of the vehicle, and the tower's arms can take it
    }
    if (crashed_) {
      return true;
    }
    const double rn = norm(s_.r);
    if (!grounded_ && rn > pl_.radius + 1e-6) {
      return false;  // in the air: the common case, no extra work
    }
    const V3 up = s_.r / rn;
    // the speed away from the surface, measured from the pad: on a rotating planet the pad's own velocity is perpendicular to the vertical, but its dot product is rounding noise (1e-10 m/s), and
    // a vehicle that has "moved away" by that much would be let go for good, and fly off along the tangent
    const double vr = dot(rotating_ ? s_.v - cross(omega_, s_.r) : s_.v, up);
    if (vr > 0.0) {
      grounded_ = false;  // moving away from the surface
      return false;
    }
    const Loads l = loads(s_, t_);
    // the net acceleration along the vertical that would lift it: the thrust and the air, the field (not the whole of it: the planet's own motion carries the pad), and on a rotating planet the
    // centripetal acceleration that the pad itself has (a pad on the equator is carried round a circle: the pad needs only g - omega^2 R toward the centre, so a lift-off needs a little less than the weight)
    double a_up = dot(rotate(s_.q, l.f_thrust + l.f_aero) / s_.m, up) + dot(gravity(s_.r), up);
    if (rotating_) {
      a_up -= dot(cross(omega_, cross(omega_, s_.r)), up);
    }
    if (vr == 0.0 && a_up > 0.0) {
      grounded_ = false;  // lifting off
      return false;
    }
    if (-vr > p_.crash_speed_ms) {
      crashed_ = true;
    }
    grounded_ = true;
    s_.r = up * pl_.radius;
    s_.v = V3{};
    s_.w = V3{};
    if (rotating_) {  // held to the pad, it goes round with the planet: its position, its attitude and its velocity turn about the pole
      const Q4 turn = from_axis_angle(pole_, pl_.rotation_rate * h);
      s_.r = rotate(turn, s_.r);
      s_.q = turn * s_.q;
      s_.v = cross(omega_, s_.r);
      s_.w = rotate_inv(s_.q, omega_);  // and it turns with it: the gyros of a vehicle on the pad read the planet's rotation
    }
    if (!crashed_) {
      if (g_.stages.size() == 1U) {
        s_.m = std::max(s_.m - (l.mdot * h), fixed_);
        sync_propellant();
      } else {
        for (std::size_t s = 0; s < g_.stages.size(); ++s) {
          const double burn = std::min(l.mdot_stage[s] * h, std::max(s_.prop[s], 0.0));
          s_.prop[s] -= burn;
          s_.m -= burn;
        }
      }
    }
    return true;
  }

  struct Deriv {
    V3 r{};
    V3 v{};
    Q4 q{0.0, 0.0, 0.0, 0.0};
    V3 w{};
    double m = 0.0;
    std::array<double, kMaxStages> prop{};
    double roll = 0.0;
    std::array<std::array<double, 4>, kMaxSlosh> slosh{};
    std::array<double, 4> flex{};
  };

  [[nodiscard]] Deriv deriv(const State& s, double t) const {
    Deriv d;
    const Loads l = loads(s, t);
    const MassProps mp = mass_props_state(s, -1);
    const V3 f_body = pods_.empty() ? l.f_thrust + l.f_aero : l.f_thrust + l.f_aero + l.f_slosh;
    // The sloshing mass is not in the rigid body, so a lateral force accelerates only the rigid part; along the axis the sloshing mass rides on the tank and presses on it with its
    // weight (it is held up by the liquid below), so the whole vehicle accelerates: the axial acceleration is the force over the whole mass.
    const double m_rigid = s.m - mp.slosh_mass;
    const V3 a_body = pods_.empty() ? V3{} : V3{f_body.x / s.m, f_body.y / m_rigid, f_body.z / m_rigid};
    const double rn = norm(s.r);
    d.r = s.v;
    d.v = (pods_.empty() ? rotate(s.q, f_body) / s.m : rotate(s.q, a_body)) + (pl_.j2 != 0.0 ? gravity(s.r) : (s.r * (-p_.gravity_scale * pl_.mu / (rn * rn * rn))));
    d.q = q_dot(s.q, s.w);
    // dI/dt = sum over the stages that burn of (dI/d propellant of that stage) * (its mass flow)
    double dit = 0.0;
    double dix = 0.0;
    for (std::size_t st = 0; st < g_.stages.size(); ++st) {
      if (l.mdot_stage[st] > 0.0) {
        const MassProps mp2 = mass_props_state(s, static_cast<int>(st));
        dit += (mp2.i_t - mp.i_t) * -l.mdot_stage[st];
        dix += (mp2.i_x - mp.i_x) * -l.mdot_stage[st];
      }
    }
    const V3 iw{mp.i_x * s.w.x, mp.i_t * s.w.y, mp.i_t * s.w.z};
    const V3 gyro = g_.wheels.enabled ? cross(s.w, iw + V3{0.0, s.wheel_h.y, s.wheel_h.z}) : cross(s.w, iw);  // the wheels' momentum rides along
    const V3 m_total = pods_.empty() ? l.m_thrust + l.m_aero : l.m_thrust + l.m_aero + l.m_slosh;
    d.w = V3{(m_total.x - gyro.x - (dix * s.w.x)) / mp.i_x, (m_total.y - gyro.y - (dit * s.w.y)) / mp.i_t,
             (m_total.z - gyro.z - (dit * s.w.z)) / mp.i_t};
    if (ideal_roll_) {
      d.w.x = 0.0;
    }
    d.roll = s.w.x;
    if (g_.flex.enabled && active_[static_cast<std::size_t>(g_.flex.stage)]) {  // the bending mode: eta'' = (Q - 2 zeta omega M eta' - omega^2 M eta) / M, in each plane
      const double wf = 2.0 * kPi * g_.flex.frequency_hz;
      const double mg = g_.flex.generalized_mass;
      d.flex = {s.flex[2], s.flex[3], (l.flex_q.y - (2.0 * g_.flex.damping * wf * mg * s.flex[2]) - (wf * wf * mg * s.flex[0])) / mg,
                (l.flex_q.z - (2.0 * g_.flex.damping * wf * mg * s.flex[3]) - (wf * wf * mg * s.flex[1])) / mg};
    }
    if (!pods_.empty()) {
      // Each sloshing mass moves against its spring and the acceleration of the point it hangs at (the vehicle's specific force there, with the turning of the vehicle): x'' = -F / m1 - a_point.
      const V3 a_spec = a_body;  // (its lateral part is over the rigid mass, as above)
      for (std::size_t i = 0; i < pods_.size(); ++i) {
        const Loads::SloshNow& now = l.pod[i];
        if (!(now.m1 > 0.0)) {
          continue;
        }
        const V3 r{now.x_s - mp.x_cg, 0.0, 0.0};
        const V3 a_pt = a_spec + cross(d.w, r) + cross(s.w, cross(s.w, r));
        const std::array<double, 4>& x = s.slosh[i];
        d.slosh[i] = {x[2], x[3], (-((now.k * x[0]) + (now.c * x[2])) / now.m1) - a_pt.y, (-((now.k * x[1]) + (now.c * x[3])) / now.m1) - a_pt.z};
      }
    }
    d.m = -l.mdot;
    for (std::size_t st = 0; st < kMaxStages; ++st) {
      d.prop[st] = -l.mdot_stage[st];
    }
    return d;
  }

  [[nodiscard]] static State advanced(const State& s, const Deriv& d, double h) {
    State o;
    o.r = s.r + (d.r * h);
    o.v = s.v + (d.v * h);
    o.q = s.q + (d.q * h);
    o.w = s.w + (d.w * h);
    o.m = s.m + (d.m * h);
    for (std::size_t i = 0; i < kMaxStages; ++i) {
      o.prop[i] = s.prop[i] + (d.prop[i] * h);
    }
    o.prop_set = true;
    o.wheel_h = s.wheel_h;
    o.roll = s.roll + (d.roll * h);
    for (std::size_t j = 0; j < 4U; ++j) {
      o.flex[j] = s.flex[j] + (d.flex[j] * h);
    }
    for (std::size_t i = 0; i < kMaxSlosh; ++i) {
      for (std::size_t j = 0; j < 4U; ++j) {
        o.slosh[i][j] = s.slosh[i][j] + (d.slosh[i][j] * h);
      }
    }
    return o;
  }

  void rk4(double h) {
    const Deriv k1 = deriv(s_, t_);
    const Deriv k2 = deriv(advanced(s_, k1, 0.5 * h), t_ + (0.5 * h));
    const Deriv k3 = deriv(advanced(s_, k2, 0.5 * h), t_ + (0.5 * h));
    const Deriv k4 = deriv(advanced(s_, k3, h), t_ + h);
    Deriv k;
    k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k4.r) / 6.0;
    k.v = (k1.v + (k2.v * 2.0) + (k3.v * 2.0) + k4.v) / 6.0;
    k.q = (k1.q + (k2.q * 2.0) + (k3.q * 2.0) + k4.q) * (1.0 / 6.0);
    k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k4.w) / 6.0;
    k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k4.m) / 6.0;
    k.roll = (k1.roll + (2.0 * k2.roll) + (2.0 * k3.roll) + k4.roll) / 6.0;
    for (std::size_t j = 0; j < 4U; ++j) {
      k.flex[j] = (k1.flex[j] + (2.0 * k2.flex[j]) + (2.0 * k3.flex[j]) + k4.flex[j]) / 6.0;
    }
    for (std::size_t i = 0; i < pods_.size(); ++i) {
      for (std::size_t j = 0; j < 4U; ++j) {
        k.slosh[i][j] = (k1.slosh[i][j] + (2.0 * k2.slosh[i][j]) + (2.0 * k3.slosh[i][j]) + k4.slosh[i][j]) / 6.0;
      }
    }
    for (std::size_t i = 0; i < kMaxStages; ++i) {
      k.prop[i] = (k1.prop[i] + (2.0 * k2.prop[i]) + (2.0 * k3.prop[i]) + k4.prop[i]) / 6.0;
    }
    s_ = advanced(s_, k, h);
    s_.q = normalized(s_.q);
    if (ideal_roll_) {
      s_.w.x = 0.0;
    }
    if (g_.stages.size() == 1U) {
      s_.m = std::max(s_.m, fixed_);  // the propellant cannot go below empty
      sync_propellant();
    } else {
      for (std::size_t i = 0; i < kMaxStages; ++i) {
        if (s_.prop[i] < 0.0) {
          s_.m -= s_.prop[i];
          s_.prop[i] = 0.0;
        }
      }
    }
  }

  Params p_;
  Scenario sc_;
  VehicleSpec g_;
  State s_{};
  double t_ = 0.0;
  PlanetSpec pl_;
  V3 pole_{1.0, 0.0, 0.0};
  V3 omega_{};
  bool rotating_ = false;
  V3 turb_{};
  uint64_t rng_ = 1U;
  AeroGeometry geo_;
  bool shaped_ = false;  // the aerodynamics follow the shape (or a table), not the reference model's fixed numbers
  double fixed_ = 0.0;  // the mass that is not propellant
  std::array<double, kMaxStages> cap_{};
  std::array<double, kMaxStages> gimbal_p_{};  // the gimbal angle of each stage's engines (after any backlash)
  std::array<double, kMaxStages> gimbal_y_{};
  std::array<double, kMaxStages> servo_p_{};   // the servo's own position and velocity when it has dynamics (a second-order servo or backlash)
  std::array<double, kMaxStages> servo_y_{};
  std::array<double, kMaxStages> servo_pv_{};
  std::array<double, kMaxStages> servo_yv_{};
  struct Pod {  // a tank that sloshes: which one
    std::size_t stage;
    std::size_t tank;
  };
  std::vector<Pod> pods_;
  bool servo_dynamics_ = false;
  bool ideal_roll_ = false;
  std::array<bool, kMaxStages> active_{};
  std::array<double, kMaxStages> t_ign_{};
  std::array<double, kMaxStages> t_burnout_{};
  std::array<double, kMaxStages> t_sep_{};
  std::array<bool, kMaxPayloads> payload_active_{};
  std::array<double, kMaxFins> fin_p_{};
  std::array<double, kMaxFins> fin_y_{};
  V3 wheel_m_{};
  std::array<double, kMaxEngines> frac_{};
  std::array<bool, kMaxEngines> failed_{};
  std::vector<EngineFailure> failures_;
  bool grounded_ = false;
  bool crashed_ = false;
  // the propulsion, surfaces, parachutes and landing
  Controls ctrl_{};
  uint32_t ev_prev_ = 0U;
  std::array<bool, kMaxEngines> eng_on_{};       // commanded on (a stage the computers drive)
  std::array<double, kMaxEngines> eng_t_on_{};
  std::array<int, kMaxEngines> starts_{};
  std::array<bool, kMaxEngines> start_failed_{};  // this start did not take: no new try until the group is commanded off
  uint64_t start_rng_ = 1U;
  std::array<double, kMaxSurfaces> surf_pos_{};
  std::array<bool, kMaxSurfaces> surf_deployed_{};
  std::array<double, kMaxParachutes> chute_open_{};   // the time it was released (< 0: not yet)
  bool legs_out_ = false;
  bool launched_ = false;
  bool landed_ = false;
  bool caught_ = false;
  bool catch_missed_ = false;
  double ground_radius_ = 0.0;
  double prev_pin_h_ = 1.0e30;
  NewtonTable newton_;
  std::vector<Detached> detached_;
};

}  // namespace sim
