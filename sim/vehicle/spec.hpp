// SPDX-License-Identifier: MIT
// The description of a vehicle: what a rocket or a spacecraft is made of, as data (host only). See docs/design/VEHICLE_SPEC.md.
//
// A vehicle is a stack along its long axis, measured from the aft end of the first stage and positive forward (x), with y and z lateral. It is made of stages (each with a dry
// structure, tanks, engines, and rules for when it ignites and separates), payload masses (a fairing, a satellite) and, in the later parts of the description, the effectors that
// steer it, its aerodynamics and the planet it flies from. The reference vehicle of VEHICLE_SIM.md is one instance of this description (`Params` with an empty `VehicleSpec` builds
// it from its own simple fields); a vehicle with any number of stages, tanks and engines is another, read from a file (spec_io.hpp).
#pragma once
#include <array>
#include <string>
#include <vector>

#include "math3.hpp"

namespace sim {

constexpr unsigned kMaxStages = 6U;
constexpr unsigned kMaxEngines = 64U;   // a cluster of 33 engines is a vehicle that exists
constexpr unsigned kMaxTanks = 8U;      // per stage
constexpr unsigned kMaxPayloads = 4U;
constexpr unsigned kMaxFins = 4U;       // sets of fins (a set is a cruciform of four)

// A propellant tank: a vertical cylinder of radius `radius` whose bottom is at `x_bottom`; the propellant sits at the bottom of it (the vehicle accelerates forward, so it settles aft).
struct TankSpec {
  double propellant = 0.0;  // kg when full
  double x_bottom = 0.0;    // m
  double radius = 0.9;      // m
  double density = 900.0;   // kg/m^3 (kerosene 800, liquid oxygen 1140, liquid hydrogen 71, a solid propellant 1750)
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

// Two reaction wheels, one about the body's Y axis (yaw) and one about Z (pitch): the command asks for a torque in proportion, up to `torque_max`; the wheel takes the opposite angular
// momentum and cannot take more than `momentum_max` (then it can only be unloaded, which this model does not do: a saturated wheel gives no torque in that direction).
struct WheelSpec {
  bool enabled = false;
  int stage = 0;
  double torque_max = 0.0;      // N m
  double momentum_max = 0.0;    // N m s
  double full_cmd_deg = 1.0;    // the command at which the torque is the maximum
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
  WheelSpec wheels;
  AeroSpec aero;
  PlanetSpec planet;
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
    double last = -1.0;
    for (const std::array<double, 2>& p : st.throttle) {
      if (p[0] < last || p[1] < 0.0 || p[1] > 1.0) {
        fail(at + ".throttle must have increasing times and fractions between 0 and 1");
        break;
      }
      last = p[0];
    }
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
