// SPDX-License-Identifier: MIT
// The description of a vehicle: what a rocket or a spacecraft is made of, as data (host only). See docs/design/VEHICLE_SPEC.md.
//
// A vehicle is a stack along its long axis, measured from the aft end of the first stage and positive forward (x), with y and z lateral. It is made of stages (each with a dry
// structure, tanks, engines, and rules for when it ignites and separates), payload masses (a fairing, a satellite) and, in the later parts of the description, the effectors that
// steer it, its aerodynamics and the planet it flies from. The reference vehicle of VEHICLE_SIM.md is one instance of this description (`Params` with an empty `VehicleSpec` builds
// it from its own simple fields); a vehicle with any number of stages, tanks and engines is another, read from a file (spec_io.hpp).
#pragma once
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "math3.hpp"

namespace sim {

constexpr unsigned kMaxStages = 6U;
constexpr unsigned kMaxEngines = 64U;   // a cluster of 33 engines is a vehicle that exists
constexpr unsigned kMaxTanks = 8U;      // per stage
constexpr unsigned kMaxPayloads = 4U;
constexpr unsigned kMaxFins = 4U;       // sets of fins (a set is a cruciform of four)
constexpr unsigned kMaxSlosh = 8U;      // tanks that slosh, on the whole vehicle
constexpr unsigned kMaxSurfaces = 12U;  // articulated aerodynamic surfaces (flaps, grid fins) on the whole vehicle
constexpr unsigned kMaxParachutes = 4U;
constexpr unsigned kEngineGroups = 8U;  // the propulsion command selects engines by group (a byte of the command)
constexpr unsigned kSurfaceChannels = 4U;  // the surface command carries this many deflections

// The first sloshing mode of a tank (slosh.hpp, docs/design/DYNAMICS.md): off by default (the liquid is part of the rigid body, as always).
struct SloshSpec {
  bool enabled = false;
  double damping = 0.02;          // the damping ratio of the sloshing (a baffled tank has a few percent: an assumption)
  double mass_scale = 1.0;        // a dispersion: the sloshing mass times this
  double frequency_scale = 1.0;   // and its frequency
};

// A propellant tank: a vertical cylinder of radius `radius` whose bottom is at `x_bottom`; the propellant sits at the bottom of it (the vehicle accelerates forward, so it settles aft).
struct TankSpec {
  double propellant = 0.0;  // kg when full
  double x_bottom = 0.0;    // m
  double radius = 0.9;      // m
  double density = 900.0;   // kg/m^3 (kerosene 800, liquid oxygen 1140, liquid hydrogen 71, a solid propellant 1750)
  SloshSpec slosh;
};

// The outer shape of the vehicle, for the aerodynamics that follow it (aero.hpp, docs/design/AERODYNAMICS.md). A stage lists the pieces of its outer body and its fixed fins; when the stage separates they go.
enum class NoseShape : int { Cone = 0, TangentOgive = 1, Parabola = 2, Ellipse = 3 };
enum class SectionKind : int { Nose = 0, Tube = 1, Transition = 2 };

// One piece of the outer body, along [x_start, x_start + length]. A nose has its base (the larger end) aft and its tip forward; a tube has one diameter; a transition has a diameter at its aft
// end and another at its forward end: a flare if the aft one is larger (it widens toward the tail: a skirt, a shoulder, which adds stability), a boat-tail if the aft one is smaller.
struct SectionSpec {
  SectionKind kind = SectionKind::Tube;
  double x_start = 0.0;
  double length = 0.0;
  double d_aft = 0.0;   // m
  double d_fore = 0.0;  // m (a pointed nose: 0)
  NoseShape nose = NoseShape::Cone;
};

// A set of `count` identical fins around the body, fixed (the control fins of FinSpec are listed here too, as fixed fins of the same planform: the control model adds only the force of the deflection).
struct FinPlanform {
  int count = 4;
  double x_le_root = 0.0;   // the forward end of the root chord, on the body
  double root_chord = 0.0;
  double tip_chord = 0.0;
  double span = 0.0;        // exposed, from the body surface to the tip
  double sweep = 0.0;       // the axial distance from the leading edge of the root to the leading edge of the tip, aft positive
  double thickness = 0.0;   // m
};

struct AeroTablePoint {
  double mach = 0.0;
  double ca = 0.0;        // axial force coefficient
  double cn_alpha = 0.0;  // normal-force slope per radian
  double x_cp = 0.0;      // m, centre of pressure
};

struct AeroSpec {
  double reference_diameter_m = 0.0;  // 0: the largest body diameter of the sections
  double crossflow_cd = 1.2;          // drag coefficient of a circular cylinder in cross-flow
  double crossflow_eta = 0.7;         // the end-effect factor on it (a finite cylinder's is lower than an infinite one's)
  double rear_axial = 1.1;            // the axial coefficient while the vehicle flies tail first (a blunt base facing the flow)
  double power_on_base = 0.7;         // how much of the base drag a running engine takes away (the plume fills the wake)
  double wetted_roughness = 1.0;      // a factor on the skin friction (1: a smooth painted skin)
  bool full_angle = true;             // the force at every angle of attack (false: linear in the angle, none past 90 degrees: the reference model)
  // The full speed range (docs/design/AERODYNAMICS.md sections 7 and 8): Mach-dependent cross-flow drag, the forces of the parts at their own places in the local flow (so a turning vehicle
  // feels the damping of its fins and body), and, from Mach 3 to 6, a blend into modified Newtonian impact theory (newtonian.hpp) for the body at any angle of attack. Off: the build-up exactly as it was.
  bool full_regime = false;
  double newtonian_from_mach = 3.0;   // the blend starts here and is complete at `newtonian_to_mach`
  double newtonian_to_mach = 6.0;
  double nose_radius_m = 0.0;         // the radius of the blunt nose or heat shield, for the stagnation heating (0: a 0.5 m default is used for the diagnostic)
  double belly_heat_factor = 0.3;     // the heat flux over the lower surface as a fraction of the stagnation flux (a flat-plate estimate at the angle of attack of an entry)
  double emissivity = 0.85;           // of the heat shield, for the radiative-equilibrium temperature
  std::vector<AeroTablePoint> table;  // a table by Mach instead of the build-up (used when `sections` of every stage are empty)
};

struct StageSpec {
  std::string name;
  // The structure: dry mass, spread along [x_start, x_start + length], its radius (for the roll inertia) and the factor that turns a uniform rod's transverse inertia (m L^2 / 12) into this
  // structure's (engines and payload interface aft, a light skin forward: the reference vehicle uses 0.6). x_cg_dry < 0: the middle of the span.
  double dry_mass = 0.0;
  double x_start = 0.0;
  double length = 0.0;
  double radius = 0.9;
  double inertia_factor = 0.6;
  double x_cg_dry = -1.0;
  std::vector<TankSpec> tanks;
  bool sequential_drain = false;  // false: every tank empties together (in proportion to its size); true: the first tank listed empties first, then the next
  // When the stage ignites: at an absolute time since T-zero, or `ignite_delay_s` after another stage separated (ignite_after_sep_of >= 0).
  double ignite_time_s = 0.0;
  int ignite_after_sep_of = -1;
  double ignite_delay_s = 0.0;
  // When the stage is let go: at an absolute time (>= 0), and/or `separate_delay_s` after its own burnout. Both unset: it stays with the vehicle (the last stage).
  double separate_time_s = -1.0;
  bool separate_on_burnout = false;
  double separate_delay_s = 0.0;
  // The gimbal of this stage's gimballed engines; a negative number takes the vehicle-wide value (Params).
  double gimbal_limit_deg = -1.0;
  double gimbal_rate_dps = -1.0;
  double gimbal_lag_s = -1.0;
  // The outer shape of this stage, aft to forward (a nose, tubes, transitions between diameters), and its fixed fins. Empty for every stage: the reference vehicle's aerodynamics.
  std::vector<SectionSpec> sections;
  std::vector<FinPlanform> stabilizers;
  // Throttle: the fraction of rated thrust against the time since this stage ignited, linear between the points, held after the last. Empty: full thrust.
  std::vector<std::array<double, 2>> throttle;
  // What the separation of this stage does to the vehicle that stays: a push forward (springs or a pressure, m/s added along the long axis) and the angular rates the release leaves it
  // with (its tip-off, deg/s about the body axes: pitch about Z, yaw about Y, roll about X). All zero: a clean separation, as before.
  double separation_dv_ms = 0.0;
  double tipoff_pitch_dps = 0.0;
  double tipoff_yaw_dps = 0.0;
  double tipoff_roll_dps = 0.0;
  // The flight computers drive this stage's engines: they start and stop with the propulsion command's group mask and run at its throttle (the schedule fields above, and `throttle`, are not used),
  // and the stage is let go by the command's separation event as well as by its schedule. False: the stage follows its schedule, as always.
  bool guided = false;
  // Where the stage's grapple pins are, for a catch (m from the stage's own aft end), and how it comes back (docs/design/RECOVERY.md).
  double catch_pin_x = 0.0;
  double leg_x = 0.0;             // the foot of the landing legs when deployed, m from the aft end (0: the aft end itself); legs add nothing to the shape in flight
};

// An engine that fires when the pitch or yaw command asks for that direction, in proportion to it: a reaction-control thruster. PitchPlus fires for a positive pitch command (one that
// should turn the nose toward downrange), PitchMinus for a negative one, and likewise for the yaw plane. Its position and direction decide the torque and the parasitic force it makes.
enum class Control : int { None = 0, PitchPlus = 1, PitchMinus = 2, YawPlus = 3, YawMinus = 4 };

struct EngineSpec {
  int stage = 0;
  V3 pos{};                   // x from the vehicle's aft end (the nozzle exit plane is where the thrust acts), y and z lateral
  double thrust_vac = 92000.0;  // N at full throttle, in vacuum
  double exit_area = 0.12;      // m^2: the ambient pressure takes p_a * exit_area off the thrust
  double isp_vac = 310.0;       // s
  bool gimbal = true;           // moves with the thrust-vector command
  double cant_pitch_deg = 0.0;  // a fixed angle of the thrust direction (the engine is mounted tilted)
  double cant_yaw_deg = 0.0;
  double start_offset_s = 0.0;  // ignites this long after its stage does
  double cutoff_time_s = -1.0;  // shuts down at this time since T-zero (<0: it burns until the propellant is gone or the stage separates)
  double rise_s = 0.0;          // thrust builds up with this time constant after ignition (0: at once)
  double tail_s = 0.0;          // and dies away with this one after shutdown
  // A thruster rather than a main engine: its own direction (a unit vector in the body frame; the default is aft-to-forward along the axis, which is what a gimbaled engine uses)
  // and the command that fires it. A thruster with its own direction does not gimbal and has no cant.
  V3 dir{1.0, 0.0, 0.0};
  Control control = Control::None;
  double full_cmd_deg = 1.0;    // the command (degrees of the flight computers' output) at which the thruster is fully on
  // For a stage the flight computers drive (`StageSpec::guided`): the group the propulsion command's mask addresses it by (0 to 7), the lowest thrust it can hold while it runs (a fraction of
  // full thrust; the command is raised to it, or the engine shut down), and how many times it can be started (0: without limit).
  int group = 0;
  double min_throttle = 0.0;
  int max_starts = 0;
};

// A set of four fins in a cross, two in the pitch plane and two in the yaw plane, that steer by aerodynamic force: deflection = gain x the command, limited, rate-limited, lagged. The force of a
// deflection is q x the area of the two fins of the plane x the lift slope x the angle, at the hinge. A set forward of the centre of gravity (a canard) takes a negative gain.
struct FinSpec {
  std::string name;
  int stage = 0;
  double x_hinge = 0.0;
  double area_each = 0.0;       // m^2 per fin
  double lift_slope = 3.0;      // per radian (a low aspect ratio fin: 2 pi AR / (2 + sqrt(4 + AR^2)) is about 3)
  double gain = 1.0;            // degrees of deflection per degree of command
  double limit_deg = 15.0;
  double rate_dps = 100.0;
  double lag_s = 0.0;
};

// An articulated aerodynamic surface: a flap on a hinge along the skin (a Starship-class ship's four flaps), or a grid fin on a shaft along the radius (the booster's), or a canard. It is driven by
// one channel of the surface command, with a travel, a rate limit and a lag; before it is deployed it lies stowed against the skin. Its force is the plate model of surfaces.hpp at its own place.
enum class SurfaceKind : int { Flap = 0, GridFin = 1 };

struct SurfaceSpec {
  std::string name;
  int stage = 0;
  SurfaceKind kind = SurfaceKind::Flap;
  double x_hinge = 0.0;       // m, axial place of the hinge line
  double azimuth_deg = 0.0;   // where it sits around the axis: 0 is the body's +Y direction, 90 the +Z
  double radius = 0.0;        // m, from the axis to the hinge line
  double area = 0.0;          // m^2: the planform of a flap, the frame of a grid fin
  double chord = 1.0;         // m, from the hinge to the free edge (a grid fin: its depth along the cells)
  double span = 1.0;          // m, along the hinge line
  double sweep_deg = 0.0;
  double chord_dir = -1.0;    // a flap: +1 if its free edge points forward from the hinge, -1 aft
  double stow_deg = 0.0;      // the deflection while stowed
  double min_deg = 0.0;       // travel
  double max_deg = 90.0;
  double rate_dps = 30.0;
  double lag_s = 0.0;
  int channel = -1;           // the surface command channel (0 to 3) that moves it; < 0: it stays at its stowed angle
  bool deployed = true;       // false: stowed until the "deploy surfaces" event
};

// A parachute (a drogue or a main canopy): released by an event, inflates over `inflation_s`, and pulls on the vehicle at `x_attach` along the velocity through the air.
struct ParachuteSpec {
  std::string name;
  int stage = 0;              // the stage it belongs to: it is lost with the stage
  double drag_area = 0.0;     // Cd A when full, m^2
  double inflation_s = 3.0;
  double x_attach = 0.0;      // m
  double max_speed_ms = 0.0;  // the canopy tears above this speed at release (0: no limit); the vehicle file's own check for a bad sequence
};

// Two reaction wheels, one about the body's Y axis (yaw) and one about Z (pitch): the command asks for a torque in proportion, up to `torque_max`; the wheel takes the opposite angular
// momentum and cannot take more than `momentum_max` (then it can only be unloaded, which this model does not do: a saturated wheel gives no torque in that direction).
struct WheelSpec {
  bool enabled = false;
  int stage = 0;
  double torque_max = 0.0;      // N m
  double momentum_max = 0.0;    // N m s
  double full_cmd_deg = 1.0;    // the command at which the torque is the maximum
};

// The servo that moves the gimballed engines (docs/design/DYNAMICS.md). Order 1 is the first-order lag the vehicle always had (the stage's or the vehicle's `gimbal_lag_s`), then the rate limit;
// order 2 is a second-order servo, theta'' = wn^2 (command - theta) - 2 zeta wn theta', which overshoots and rings as a real one does, then the rate and travel limits. Backlash is the play
// between the servo and the engine (either order): the engine does not move until the servo has taken up half of it on that side.
struct ActuatorSpec {
  int order = 1;
  double natural_hz = 10.0;    // order 2: the undamped natural frequency
  double damping = 0.7;        // order 2: the damping ratio
  double backlash_deg = 0.0;   // total play
};

// A roll controller of the vehicle's own, not the flight computers' (which steer two tilt planes): reaction-control jets or the like give a torque about the long axis,
// -kp x (the roll angle turned through since T-zero) - kd x (the roll rate), limited to torque_max, while stage `stage` is on the vehicle. Without it the vehicle either holds its roll
// ideally (the reference model's `ideal_roll_control`) or turns freely about its long axis.
struct RollSpec {
  bool enabled = false;
  int stage = 0;
  double torque_max = 0.0;  // N m
  double kp = 0.0;          // N m per radian
  double kd = 0.0;          // N m s per radian
};

// One bending mode of the structure (docs/design/DYNAMICS.md), in each of the two lateral planes: a modal coordinate eta with the frequency, the damping and the generalised mass given, and the
// mode shape reduced to what couples it to the rest: its displacement `phi` and its slope `sigma` (per metre) where the engines are and where the IMUs are. The lateral thrust drives it
// (generalised force phi_engine x the lateral thrust), the thrust follows the slope of the structure at the engines (the lateral force T sigma_engine eta), and a gyro on the vehicle reads the slope's
// rate on top of the body's (sigma_imu eta') and an accelerometer the structure's acceleration there (phi_imu eta''). The mode exists while `stage` is on the vehicle.
struct FlexSpec {
  bool enabled = false;
  int stage = 0;
  double frequency_hz = 0.0;
  double damping = 0.005;          // the damping ratio (a structure's: half a percent is typical; an assumption)
  double generalized_mass = 0.0;   // kg
  double phi_engine = 1.0;         // the mode shape's displacement at the engines
  double slope_engine = 0.0;       // and its slope there, per metre
  double phi_imu = 0.0;            // the displacement at the IMUs
  double slope_imu = 0.0;          // and the slope, per metre
};

struct PayloadSpec {
  std::string name;
  double mass = 0.0;
  double x = 0.0;
  double jettison_time_s = -1.0;  // it leaves the vehicle at this time (<0: never)
  std::vector<SectionSpec> sections;  // its outer shape (a fairing), which goes with it
};

// The world the vehicle flies in (docs/design/ENVIRONMENT.md). The default is the reference model's: a spherical, non-rotating Earth with the 1976 standard atmosphere, which is what every earlier
// flight and every documented number used. "earth", "moon" and "mars" are presets with their rotation, flattening (J2) and atmosphere.
enum class AtmosphereKind : int { Us1976 = 0, Exponential = 1, None = 2 };

struct PlanetSpec {
  std::string name = "reference";
  double radius = 6378137.0;          // m
  double mu = 3.986004418e14;         // m^3/s^2
  double rotation_rate = 0.0;         // rad/s about the pole (the pole's direction is set by the launch site)
  double j2 = 0.0;                    // the oblateness term of the gravity field
  AtmosphereKind atmosphere = AtmosphereKind::Us1976;
  double surface_density = 1.225;     // exponential atmosphere: kg/m^3 at the surface
  double scale_height = 8500.0;       // m
  double temperature = 288.15;        // K, constant
  double gas_constant = 287.053;      // J/(kg K)
  double gamma = 1.4;
  double density_scale = 1.0;         // a dispersion of any atmosphere: its density and pressure times this
  double temperature_offset = 0.0;    // and its temperature plus this (a hot or a cold day), at the same pressure
};

// The presets, from the standard values of each body (written from memory of the usual references; they are not checked against a source here).
inline bool planet_preset(const std::string& name, PlanetSpec& p) {
  PlanetSpec q;
  if (name == "reference") {
    p = q;
    return true;
  }
  if (name == "earth") {
    q.name = "earth";
    q.rotation_rate = 7.2921159e-5;
    q.j2 = 1.08263e-3;
  } else if (name == "moon") {
    q.name = "moon";
    q.radius = 1737400.0;
    q.mu = 4.9028e12;
    q.rotation_rate = 2.6617e-6;
    q.j2 = 2.034e-4;
    q.atmosphere = AtmosphereKind::None;
  } else if (name == "mars") {
    q.name = "mars";
    q.radius = 3396200.0;
    q.mu = 4.282837e13;
    q.rotation_rate = 7.0882e-5;
    q.j2 = 1.96045e-3;
    q.atmosphere = AtmosphereKind::Exponential;
    q.surface_density = 0.020;
    q.scale_height = 11100.0;
    q.temperature = 210.0;
    q.gas_constant = 188.9;
    q.gamma = 1.29;
  } else {
    return false;
  }
  p = q;
  return true;
}

struct VehicleSpec {
  std::vector<StageSpec> stages;
  std::vector<EngineSpec> engines;
  std::vector<PayloadSpec> payloads;
  std::vector<FinSpec> fins;
  std::vector<SurfaceSpec> surfaces;
  std::vector<ParachuteSpec> parachutes;
  WheelSpec wheels;
  AeroSpec aero;
  PlanetSpec planet;
  ActuatorSpec actuator;
  RollSpec roll;
  FlexSpec flex;
  bool jet_damping = false;  // the moment of the exhaust leaving a turning vehicle: -mdot x r x (omega x r) for each engine (a damping of the pitch and yaw rates)
  [[nodiscard]] bool empty() const { return stages.empty(); }
};

inline void check_sections(const std::vector<SectionSpec>& sections, const std::string& at, std::vector<std::string>& bad) {
  for (std::size_t k = 0; k < sections.size(); ++k) {
    const SectionSpec& sec = sections[k];
    const std::string sk = at + ".sections[" + std::to_string(k) + "]";
    if (!(sec.length > 0.0) || !(sec.d_aft >= 0.0) || !(sec.d_fore >= 0.0)) {
      bad.push_back(sk + ": length_m must be positive and the diameters not negative");
    } else if (sec.kind == SectionKind::Nose && !(sec.d_aft > sec.d_fore)) {
      bad.push_back(sk + " is a nose: its base (d_aft_m) must be larger than its tip (d_fore_m, 0 for a point)");
    } else if (sec.kind == SectionKind::Tube && (!(sec.d_aft > 0.0) || sec.d_aft != sec.d_fore)) {
      bad.push_back(sk + " is a tube: d_aft_m and d_fore_m must be the same positive diameter");
    } else if (sec.kind == SectionKind::Transition && (!(sec.d_aft > 0.0) || !(sec.d_fore > 0.0) || sec.d_aft == sec.d_fore)) {
      bad.push_back(sk + " is a transition: d_aft_m and d_fore_m must be positive and different");
    }
  }
}

// Problems with a description, each in words and with the name of the field; empty if there are none. Cheap: loaders call it, and tests.
inline std::vector<std::string> validate(const VehicleSpec& v) {
  std::vector<std::string> bad;
  const auto fail = [&bad](const std::string& what) { bad.push_back(what); };
  if (v.stages.empty()) {
    return bad;  // the simple reference vehicle: nothing to check here
  }
  if (v.stages.size() > kMaxStages) {
    fail("stages: at most " + std::to_string(kMaxStages) + " are supported");
  }
  if (v.engines.size() > kMaxEngines) {
    fail("engines: at most " + std::to_string(kMaxEngines) + " are supported");
  }
  if (v.payloads.size() > kMaxPayloads) {
    fail("payloads: at most " + std::to_string(kMaxPayloads) + " are supported");
  }
  unsigned sloshing = 0U;
  for (std::size_t s = 0; s < v.stages.size(); ++s) {
    const StageSpec& st = v.stages[s];
    const std::string at = "stages[" + std::to_string(s) + "]";
    if (!(st.dry_mass > 0.0)) {
      fail(at + ".dry_mass must be positive");
    }
    if (!(st.length > 0.0)) {
      fail(at + ".length must be positive");
    }
    if (!(st.radius > 0.0)) {
      fail(at + ".radius must be positive");
    }
    if (!(st.inertia_factor > 0.0)) {
      fail(at + ".inertia_factor must be positive");
    }
    if (st.tanks.size() > kMaxTanks) {
      fail(at + ".tanks: at most " + std::to_string(kMaxTanks) + " are supported");
    }
    for (std::size_t k = 0; k < st.tanks.size(); ++k) {
      const TankSpec& t = st.tanks[k];
      const std::string tk = at + ".tanks[" + std::to_string(k) + "]";
      if (!(t.propellant >= 0.0)) {
        fail(tk + ".propellant must not be negative");
      }
      if (!(t.radius > 0.0) || !(t.density > 0.0)) {
        fail(tk + ".radius and .density must be positive");
      }
      if (t.slosh.enabled) {
        if (!(t.slosh.damping >= 0.0) || !(t.slosh.mass_scale > 0.0) || !(t.slosh.frequency_scale > 0.0)) {
          fail(tk + ".slosh: damping must not be negative and mass_scale and frequency_scale must be positive");
        }
        if (!(t.propellant > 0.0)) {
          fail(tk + ".slosh: a tank with no propellant has nothing to slosh");
        }
        ++sloshing;
      }
      if (t.propellant > 0.0 && t.radius > 0.0 && t.density > 0.0) {
        const double height = t.propellant / (t.density * kPi * t.radius * t.radius);
        if (t.x_bottom + height > st.x_start + st.length + 1e-9) {
          fail(tk + " holds more propellant than fits in the stage (its column would end " + std::to_string(t.x_bottom + height - st.x_start - st.length) + " m beyond the stage)");
        }
      }
    }
    check_sections(st.sections, at, bad);
    for (std::size_t k = 0; k < st.stabilizers.size(); ++k) {
      const FinPlanform& f = st.stabilizers[k];
      if (f.count < 1 || f.count > 12 || !(f.root_chord > 0.0) || !(f.span > 0.0) || f.tip_chord < 0.0 || f.thickness < 0.0 || f.sweep < 0.0) {
        fail(at + ".stabilizers[" + std::to_string(k) + "]: count 1 to 12, root_chord_m and span_m positive, tip_chord_m, sweep_m and thickness_m not negative");
      }
    }
    if (st.ignite_after_sep_of >= static_cast<int>(s)) {
      fail(at + ".ignite_after_sep_of must name an earlier stage");
    }
    if (st.ignite_after_sep_of >= static_cast<int>(v.stages.size())) {
      fail(at + ".ignite_after_sep_of names a stage that does not exist");
    }
    if (st.ignite_delay_s < 0.0 || st.separate_delay_s < 0.0) {
      fail(at + ": delays must not be negative");
    }
    if (!std::isfinite(st.separation_dv_ms) || !std::isfinite(st.tipoff_pitch_dps) || !std::isfinite(st.tipoff_yaw_dps) || !std::isfinite(st.tipoff_roll_dps)) {
      fail(at + ": the separation push and tip-off rates must be numbers");
    }
    double last = -1.0;
    for (const std::array<double, 2>& p : st.throttle) {
      if (p[0] < last || p[1] < 0.0 || p[1] > 1.0) {
        fail(at + ".throttle must have increasing times and fractions between 0 and 1");
        break;
      }
      last = p[0];
    }
  }
  if (sloshing > kMaxSlosh) {
    fail("tanks: at most " + std::to_string(kMaxSlosh) + " tanks can slosh");
  }
  for (std::size_t e = 0; e < v.engines.size(); ++e) {
    const EngineSpec& en = v.engines[e];
    const std::string at = "engines[" + std::to_string(e) + "]";
    if (en.stage < 0 || en.stage >= static_cast<int>(v.stages.size())) {
      fail(at + ".stage must name a stage");
    }
    if (!(en.thrust_vac > 0.0) || !(en.isp_vac > 0.0) || en.exit_area < 0.0) {
      fail(at + ": thrust_vac and isp_vac must be positive and exit_area not negative");
    }
    if (en.rise_s < 0.0 || en.tail_s < 0.0 || en.start_offset_s < 0.0) {
      fail(at + ": rise_s, tail_s and start_offset_s must not be negative");
    }
    const bool own_dir = en.dir.x != 1.0 || en.dir.y != 0.0 || en.dir.z != 0.0;
    if (!(norm(en.dir) > 0.0)) {
      fail(at + ".dir must not be the zero vector");
    } else if (own_dir && (en.gimbal || en.cant_pitch_deg != 0.0 || en.cant_yaw_deg != 0.0)) {
      fail(at + " has its own direction: it cannot gimbal or have a cant (set gimbal to false)");
    }
    if (en.control != Control::None && !(en.full_cmd_deg > 0.0)) {
      fail(at + ".full_cmd_deg must be positive");
    }
    if (en.control != Control::None && en.gimbal) {
      fail(at + " is a thruster (it has a control): it cannot gimbal");
    }
    if (en.group < 0 || en.group >= static_cast<int>(kEngineGroups) || !(en.min_throttle >= 0.0 && en.min_throttle <= 1.0) || en.max_starts < 0) {
      fail(at + ": group is 0 to 7, min_throttle between 0 and 1, max_starts not negative");
    }
  }
  for (std::size_t s = 0; s < v.stages.size(); ++s) {
    bool has_engine = false;
    for (const EngineSpec& en : v.engines) {
      has_engine = has_engine || en.stage == static_cast<int>(s);
    }
    double prop = 0.0;
    for (const TankSpec& t : v.stages[s].tanks) {
      prop += t.propellant;
    }
    if (has_engine && !(prop > 0.0)) {
      fail("stages[" + std::to_string(s) + "] has engines and no propellant");
    }
  }
  for (std::size_t p = 0; p < v.payloads.size(); ++p) {
    if (!(v.payloads[p].mass >= 0.0)) {
      fail("payloads[" + std::to_string(p) + "].mass must not be negative");
    }
    check_sections(v.payloads[p].sections, "payloads[" + std::to_string(p) + "]", bad);
  }
  {
    const AeroSpec& a = v.aero;
    if (a.reference_diameter_m < 0.0 || a.crossflow_cd < 0.0 || a.crossflow_eta < 0.0 || a.rear_axial < 0.0 || a.wetted_roughness < 0.0 || a.power_on_base < 0.0 || a.power_on_base > 1.0) {
      fail("aero: the diameters, coefficients and factors must not be negative, and power_on_base lies between 0 and 1");
    }
    for (std::size_t k = 1; k < a.table.size(); ++k) {
      if (!(a.table[k].mach > a.table[k - 1U].mach)) {
        fail("aero.table: the Mach numbers must increase");
        break;
      }
    }
    bool any_section = false;
    for (const StageSpec& st : v.stages) {
      any_section = any_section || !st.sections.empty();
    }
    for (const PayloadSpec& pl : v.payloads) {
      any_section = any_section || !pl.sections.empty();
    }
    if (any_section && !a.table.empty()) {
      fail("aero: give the shape (stage sections) or a table by Mach, not both");
    }
  }
  {
    const PlanetSpec& pl = v.planet;
    if (!(pl.radius > 0.0) || !(pl.mu > 0.0) || pl.rotation_rate < 0.0 || !(pl.scale_height > 0.0) || !(pl.gas_constant > 0.0) || !(pl.gamma > 1.0) || !(pl.density_scale > 0.0) || !(pl.surface_density >= 0.0)) {
      fail("planet: radius_m, mu, scale_height_m, gas_constant and density_scale must be positive, gamma above 1, rotation_rate_rad_s not negative");
    }
    if (pl.atmosphere != AtmosphereKind::None && !(pl.temperature + pl.temperature_offset > 50.0)) {
      fail("planet: the atmosphere's temperature (with the offset) must be above 50 K");
    }
  }
  if (v.fins.size() > kMaxFins) {
    fail("fins: at most " + std::to_string(kMaxFins) + " sets are supported");
  }
  for (std::size_t f = 0; f < v.fins.size(); ++f) {
    const FinSpec& fin = v.fins[f];
    const std::string at = "fins[" + std::to_string(f) + "]";
    if (fin.stage < 0 || fin.stage >= static_cast<int>(v.stages.size())) {
      fail(at + ".stage must name a stage");
    }
    if (!(fin.area_each > 0.0) || !(fin.lift_slope > 0.0) || !(fin.limit_deg > 0.0) || !(fin.rate_dps > 0.0) || fin.lag_s < 0.0 || fin.gain == 0.0) {
      fail(at + ": area_each, lift_slope, limit_deg and rate_dps must be positive, lag_s not negative and gain not zero");
    }
  }
  if (v.surfaces.size() > kMaxSurfaces) {
    fail("surfaces: at most " + std::to_string(kMaxSurfaces) + " are supported");
  }
  for (std::size_t i = 0; i < v.surfaces.size(); ++i) {
    const SurfaceSpec& sf = v.surfaces[i];
    const std::string at = "surfaces[" + std::to_string(i) + "]";
    if (sf.stage < 0 || sf.stage >= static_cast<int>(v.stages.size())) {
      fail(at + ".stage must name a stage");
    }
    if (!(sf.area > 0.0) || !(sf.chord > 0.0) || !(sf.span > 0.0) || sf.radius < 0.0) {
      fail(at + ": area_m2, chord_m and span_m must be positive and radius_m not negative");
    }
    if (!(sf.max_deg >= sf.min_deg) || !(sf.rate_dps > 0.0) || sf.lag_s < 0.0 || !(std::fabs(sf.chord_dir) == 1.0)) {
      fail(at + ": max_deg must not be below min_deg, rate_dps must be positive, lag_s not negative and chord_dir 1 or -1");
    }
    if (sf.channel >= static_cast<int>(kSurfaceChannels)) {
      fail(at + ".channel must be below " + std::to_string(kSurfaceChannels));
    }
  }
  if (v.parachutes.size() > kMaxParachutes) {
    fail("parachutes: at most " + std::to_string(kMaxParachutes) + " are supported");
  }
  for (std::size_t i = 0; i < v.parachutes.size(); ++i) {
    const ParachuteSpec& pc = v.parachutes[i];
    const std::string at = "parachutes[" + std::to_string(i) + "]";
    if (pc.stage < 0 || pc.stage >= static_cast<int>(v.stages.size())) {
      fail(at + ".stage must name a stage");
    }
    if (!(pc.drag_area > 0.0) || !(pc.inflation_s >= 0.0) || pc.max_speed_ms < 0.0) {
      fail(at + ": drag_area_m2 must be positive, inflation_s and max_speed_ms not negative");
    }
  }
  {
    const AeroSpec& a = v.aero;
    if (a.full_regime && !(a.newtonian_to_mach > a.newtonian_from_mach && a.newtonian_from_mach > 1.0)) {
      fail("aero: newtonian_to_mach must be above newtonian_from_mach, which is above Mach 1");
    }
    if (a.nose_radius_m < 0.0 || a.belly_heat_factor < 0.0 || !(a.emissivity > 0.0 && a.emissivity <= 1.0)) {
      fail("aero: nose_radius_m and belly_heat_factor must not be negative and emissivity lies between 0 and 1");
    }
  }
  {
    const ActuatorSpec& ac = v.actuator;
    if (ac.order != 1 && ac.order != 2) {
      fail("actuator.order must be 1 or 2");
    }
    if (ac.order == 2 && (!(ac.natural_hz > 0.0) || !(ac.damping > 0.0))) {
      fail("actuator: a second-order servo needs natural_hz and damping above zero");
    }
    if (!(ac.backlash_deg >= 0.0)) {
      fail("actuator.backlash_deg must not be negative");
    }
  }
  if (v.flex.enabled) {
    if (v.flex.stage < 0 || v.flex.stage >= static_cast<int>(v.stages.size())) {
      fail("flex.stage must name a stage");
    }
    if (!(v.flex.frequency_hz > 0.0) || !(v.flex.generalized_mass > 0.0) || !(v.flex.damping >= 0.0)) {
      fail("flex: frequency_hz and generalized_mass_kg must be positive and damping not negative");
    }
    if (!std::isfinite(v.flex.phi_engine) || !std::isfinite(v.flex.slope_engine) || !std::isfinite(v.flex.phi_imu) || !std::isfinite(v.flex.slope_imu)) {
      fail("flex: the mode shape's values must be numbers");
    }
  }
  if (v.roll.enabled) {
    if (v.roll.stage < 0 || v.roll.stage >= static_cast<int>(v.stages.size())) {
      fail("roll_control.stage must name a stage");
    }
    if (!(v.roll.torque_max > 0.0) || !(v.roll.kp >= 0.0) || !(v.roll.kd >= 0.0)) {
      fail("roll_control: torque_max_nm must be positive and the gains not negative");
    }
  }
  if (v.wheels.enabled) {
    if (v.wheels.stage < 0 || v.wheels.stage >= static_cast<int>(v.stages.size())) {
      fail("wheels.stage must name a stage");
    }
    if (!(v.wheels.torque_max > 0.0) || !(v.wheels.momentum_max > 0.0) || !(v.wheels.full_cmd_deg > 0.0)) {
      fail("wheels: torque_max, momentum_max and full_cmd_deg must be positive");
    }
  }
  return bad;
}

}  // namespace sim
