// SPDX-License-Identifier: MIT
// The world of a whole mission: the vehicle on the pad and in flight, the stages it lets go (each flown on as a vehicle of its own, with computers of its own if the mission gives it any), and the loop that steps
// them all one frame at a time (docs/design/GNC.md section 8). It is the host-only counterpart of the rig: the same vehicle model, the same flight function in three replicas, the same bus voting, with no sockets.
//
// Two ways to fly it:
//   * closed loop: the computers' sensors, the GNSS receiver, three flight functions and ACT's vote per body (Avionics), exactly what the firmware computes;
//   * ideal: the attitude of each body is forced to the one its mission asks for (the design run of the gains, and a check of the guidance without the sensors and the loop), the engines and events as the mission
//     commands them. A probe is called every few frames with the aerodynamic stiffness and the control effectiveness of each axis, which is how the gains are designed.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "avionics.hpp"
#include "mission_spec.hpp"
#include "vehicle6.hpp"
#include "tfc/mission.hpp"
#include "tfc/peg.hpp"

namespace sim {

struct BodyLog {
  double t = 0.0;
  int body = 0;
  uint8_t phase = 0U;
  double altitude = 0.0;
  double speed = 0.0;
  double mass = 0.0;
};

// The extremes of what a body flew through, sampled at 10 Hz: what a trajectory design review asks for first. The dynamic pressure and, above all, its product with the angle of attack (q alpha, which sets the
// bending moment the structure carries on the way up, taken as q sin(alpha) so that it is zero flying tail-first as well as nose-first), the accelerations along the axis and across it, the Mach number, and the heating of the nose and the skin it comes to. The times are the body's own flight times.
struct Envelope {
  double max_q_pa = 0.0;
  double t_max_q = 0.0;
  double max_q_sin_alpha_pa = 0.0;  // q sin(alpha): q alpha for the small angles of an ascent, and zero at 0 and at 180 degrees (a stage coming down on its engines is not loaded across)
  double t_max_q_sin_alpha = 0.0;
  double max_normal_force_n = 0.0;  // the aerodynamic force across the axis
  double max_alpha_rad = 0.0;       // only where the air has something to say: above 1 kPa
  double max_axial_g = 0.0;         // the thrust and the aerodynamic force along the axis, over the weight at the sea
  double max_lateral_g = 0.0;       // and across it
  double max_mach = 0.0;
  double max_stagnation_w_m2 = 0.0;
  double t_max_heat = 0.0;
  double max_wall_k = 0.0;          // the skin's radiative equilibrium under the belly flux
  double max_aero_moment_nm = 0.0;  // the aerodynamic moment about the centre of mass
  uint32_t samples = 0U;

  void note(double t, const Loads& l, double mass, const Vehicle6::Heating& h) {
    ++samples;
    if (l.dynamic_pressure > max_q_pa) {
      max_q_pa = l.dynamic_pressure;
      t_max_q = t;
    }
    const double qa = l.dynamic_pressure * std::sin(l.alpha);
    if (qa > max_q_sin_alpha_pa) {
      max_q_sin_alpha_pa = qa;
      t_max_q_sin_alpha = t;
    }
    max_normal_force_n = std::fmax(max_normal_force_n, std::hypot(l.f_aero.y, l.f_aero.z));
    if (l.dynamic_pressure > 1000.0) {
      max_alpha_rad = std::fmax(max_alpha_rad, l.alpha);
    }
    const double m = std::fmax(mass, 1.0);
    max_axial_g = std::fmax(max_axial_g, std::fabs(l.f_thrust.x + l.f_aero.x) / (m * kG0));
    max_lateral_g = std::fmax(max_lateral_g, std::hypot(l.f_thrust.y + l.f_aero.y, l.f_thrust.z + l.f_aero.z) / (m * kG0));
    max_mach = std::fmax(max_mach, l.mach);
    if (h.stagnation_w_m2 > max_stagnation_w_m2) {
      max_stagnation_w_m2 = h.stagnation_w_m2;
      t_max_heat = t;
    }
    max_wall_k = std::fmax(max_wall_k, h.wall_k);
    max_aero_moment_nm = std::fmax(max_aero_moment_nm, norm(l.m_aero));
  }
};

// What an envelope has over the limits of a body (an empty list: within them, or no limits given).
inline std::vector<std::string> envelope_violations(const LoadLimits& lim, const Envelope& e) {
  std::vector<std::string> v;
  const auto over = [&v](double limit, double got, const char* what, double scale, const char* unit) {
    if (limit > 0.0 && got > limit) {
      char buf[160];
      std::snprintf(buf, sizeof buf, "%s %.1f %s is over the limit of %.1f %s", what, got / scale, unit, limit / scale, unit);
      v.emplace_back(buf);
    }
  };
  over(lim.max_q_pa, e.max_q_pa, "max q", 1000.0, "kPa");
  over(lim.max_q_sin_alpha_pa, e.max_q_sin_alpha_pa, "max q sin(alpha)", 1000.0, "kPa");
  over(lim.max_axial_g, e.max_axial_g, "axial acceleration", 1.0, "g");
  over(lim.max_lateral_g, e.max_lateral_g, "lateral acceleration", 1.0, "g");
  over(lim.max_stagnation_w_m2, e.max_stagnation_w_m2, "nose heat flux", 1000.0, "kW/m2");
  over(lim.max_wall_k, e.max_wall_k, "skin temperature", 1.0, "K");
  return v;
}

// The smallest distance between two line segments (the axes of two bodies), by the closest points of the two lines clamped to the segments (Ericson, Real-Time Collision Detection, 5.1.9, from memory).
inline double segment_distance(const V3& p1, const V3& q1, const V3& p2, const V3& q2) {
  const V3 d1 = q1 - p1;
  const V3 d2 = q2 - p2;
  const V3 r = p1 - p2;
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);
  double s = 0.0;
  double t = 0.0;
  const auto clamp01 = [](double x) { return std::fmin(1.0, std::fmax(0.0, x)); };
  if (a <= 1e-12 && e <= 1e-12) {
    return norm(r);
  }
  if (a <= 1e-12) {
    t = clamp01(f / e);
  } else {
    const double c = dot(d1, r);
    if (e <= 1e-12) {
      s = clamp01(-c / a);
    } else {
      const double b = dot(d1, d2);
      const double denom = (a * e) - (b * b);
      s = denom > 1e-12 ? clamp01(((b * f) - (c * e)) / denom) : 0.0;
      t = ((b * s) + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-c / a);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((b - c) / a);
      }
    }
  }
  return norm((p1 + (d1 * s)) - (p2 + (d2 * t)));
}

// How close a stage that has been let go came to the vehicle it left, in the first seconds after: the gap between their surfaces (the distance between the two axes' segments less the two radii, which is a
// cylinder's idea of a body: the plume of the one on the other, and the nozzles and the flaps, are not in it), with the time and the speed at which they closed.
struct Clearance {
  bool cleared = false;             // the surfaces have come apart: the gap has been above zero (before that the two are one stack, with the nozzles of the upper stage inside the interstage of the lower)
  double t_clear_s = 0.0;           // seconds after the separation at which they cleared
  double min_gap_m = 1e9;           // the closest the gap came after that
  double t_s = 0.0;                 // seconds after the separation at the closest
  double closing_ms = 0.0;          // the rate at which the gap was shrinking at the closest
  double gap_at_end_m = 0.0;        // at the end of the window (30 s)
  double last_gap = 0.0;
  bool seen = false;
};

// What a design run measures at a point of the flight: the aerodynamic stiffness of the three axes and the effect of one degree of each effector on the angular acceleration (rad/s^2, body axes x y z = roll, yaw,
// pitch): the gimbal in the pitch plane, the gimbal in the yaw plane, the roll thrusters, and each of the four surface channels.
struct AxisProbe {
  double t = 0.0;               // s since T-zero
  int body = 0;
  int stage = -1;               // -1: the main body, else the stage whose computers these are
  uint8_t phase = 0U;
  double t_in_phase = 0.0;
  std::array<double, 3> a{};    // 1/s^2 per radian of attitude: roll, yaw, pitch
  double thrust = 0.0;          // N, the thrust the vehicle had at the probe
  std::array<std::array<double, 3>, 7> col{};   // [0] gimbal pitch, [1] gimbal yaw, [2] roll thrusters, [3..6] surface channels 0..3
};

struct WorldConfig {
  Params params;
  Scenario scenario;
  const tfc::gnc::Tables* main_tables = nullptr;
  std::array<const tfc::gnc::Tables*, kMaxStages> stage_tables{};   // by the stage's index in the vehicle: the mission of the computers that fly that stage once it is let go (null: it is not flown)
  AvionicsConfig avionics;                                           // the template for every set of computers (its `tables` is replaced)
  bool ideal = false;
  bool main_external = false;                                        // the main body is flown from outside (the rig): the caller supplies its controls
  bool children_only = false;                                        // there is no main body here: the stages the caller lets go are handed in (`adopt`) and flown on by computers of their own (the rig's simulator,
                                                                     // whose main body is the bus's and whose returning stages are not on it)
  uint32_t pad_frames = 0U;                                          // frames the main body stands clamped on the pad before T-zero
  uint32_t probe_every = 100U;                                       // frames between the probes of a design run
};

class MissionWorld {
 public:
  static constexpr uint32_t kEnvelopeEvery = 10U;   // frames between the samples of a body's envelope
  struct Body {
    std::string name;
    int stage = -1;                       // the stage index it was let go from; -1 the main body
    Vehicle6 veh;
    Controls ctl;
    std::unique_ptr<Avionics> av;
    std::unique_ptr<tfc::gnc::Mission> ideal;
    const tfc::gnc::Tables* tables = nullptr;
    uint32_t born = 0U;                   // the world frame it began on
    bool control_off = false;
    uint8_t last_phase = 0U;
    uint32_t seeded = 0U;                 // the phases whose PEG solution has been recorded
    Envelope env;                         // the extremes of its loads so far
    Clearance clearance;                  // for a stage that was let go: how close it came to the main body
    double drag_num = 0.0;                // sums for the ballistic coefficient the glide measured (see drag_beta)
    double drag_den = 0.0;
    Body(const Params& p, const Scenario& s) : veh(p, s) {}
    Body(Vehicle6&& v) : veh(std::move(v)) {}
  };

  explicit MissionWorld(const WorldConfig& cfg) : cfg_(cfg) {
    if (!cfg.children_only) {
      bodies_.push_back(std::make_unique<Body>(cfg.params, cfg.scenario));
      Body& m = *bodies_[0];
      m.name = "main";
      m.tables = cfg.main_tables;
      attach(m);
      clamped_ = cfg.pad_frames > 0U;
    }
  }

  void set_probe(std::function<void(const AxisProbe&)> f) { probe_ = std::move(f); }
  void set_log(std::function<void(const BodyLog&)> f, uint32_t every) {
    log_ = std::move(f);
    log_every_ = every;
  }

  // One frame of the whole world.
  void step() {
    const bool pad_now = clamped_ && frame_ < cfg_.pad_frames;
    if (clamped_ && frame_ >= cfg_.pad_frames) {
      clamped_ = false;   // T-zero: the clamps open
      t0_ = frame_;
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      step_body(*bodies_[i], i, i == 0U && pad_now);
    }
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      for (const Detached& d : bodies_[i]->veh.take_detached()) {
        spawn(bodies_[i]->veh.time(), d);
      }
    }
    for (std::size_t i = 1; i < bodies_.size(); ++i) {
      note_clearance(*bodies_[0], *bodies_[i]);
    }
    ++frame_;
  }

  // A stage let go by a vehicle that is not in this world (`children_only`): flown on from now, `parent_time` being the time that vehicle has reached.
  void adopt(const Detached& d, double parent_time) { spawn(parent_time, d); }

  [[nodiscard]] uint32_t frame() const { return frame_; }
  [[nodiscard]] uint32_t flight_frame() const { return clamped_ ? 0U : frame_ - t0_; }
  [[nodiscard]] bool clamped() const { return clamped_; }
  [[nodiscard]] std::size_t bodies() const { return bodies_.size(); }
  [[nodiscard]] const Body& body(std::size_t i) const { return *bodies_[i]; }
  [[nodiscard]] Body& body(std::size_t i) { return *bodies_[i]; }
  // The caller's controls for the main body (when it is flown from outside).
  void set_main_controls(const Controls& c) { bodies_[0]->ctl = c; }
  [[nodiscard]] const WorldConfig& config() const { return cfg_; }

  // What a design run learns: the burn solution of each PEG phase when it began, by body (-1: main, else the stage) and phase; and the mass each stage had when it was let go.
  struct Seed {
    int stage = -1;
    uint8_t phase = 0U;
    std::array<double, tfc::peg::kUnknowns> sol{};
  };
  [[nodiscard]] const std::vector<Seed>& seeds() const { return seeds_; }
  [[nodiscard]] double spawn_mass(std::size_t stage) const { return stage < kMaxStages ? spawn_mass_[stage] : 0.0; }
  // The ballistic coefficient (m / (Cd A), kg/m^2) the stage's glide measured, 0 if it did not glide: the guidance's model of the way down is made to match this.
  [[nodiscard]] double drag_beta(std::size_t stage) const {
    for (const std::unique_ptr<Body>& b : bodies_) {
      if (b->stage == static_cast<int>(stage) && b->drag_den > 0.0) {
        return b->drag_num / b->drag_den;
      }
    }
    return 0.0;
  }
  // Has every body reached the last phase of its mission (or ended on the ground)?
  [[nodiscard]] bool finished() const {
    for (const std::unique_ptr<Body>& b : bodies_) {
      const bool ended = b->veh.landed() || b->veh.caught() || b->veh.crashed();
      const bool last = b->tables == nullptr || (b->ideal != nullptr ? b->ideal->phase() + 1U >= b->tables->n_phases : false);
      if (!ended && !last) {
        return false;
      }
    }
    return true;
  }

 private:
  // The two ends of a body's axis in the inertial frame (the aft end of what is on it, and the forward end) and the largest radius.
  static void axis_of_body(const Vehicle6& v, V3& tail, V3& nose, double& radius) {
    const VehicleSpec& g = v.spec();
    double x0 = 1e30;
    double x1 = -1e30;
    radius = 0.0;
    for (std::size_t s = 0; s < g.stages.size(); ++s) {
      if (!v.stage_active(s)) {
        continue;
      }
      x0 = std::fmin(x0, g.stages[s].x_start);
      x1 = std::fmax(x1, g.stages[s].x_start + g.stages[s].length);
      radius = std::fmax(radius, g.stages[s].radius);
    }
    if (!(x1 > x0)) {
      x0 = 0.0;
      x1 = 0.0;
    }
    const double x_cg = v.mass_props(v.mass()).x_cg;
    const V3 axis = rotate(v.state().q, V3{1.0, 0.0, 0.0});
    tail = v.state().r + (axis * (x0 - x_cg));
    nose = v.state().r + (axis * (x1 - x_cg));
  }

  // At every tenth frame for the first 30 s after a stage is let go: the gap to the main body.
  void note_clearance(const Body& main, Body& child) {
    constexpr double kWindowS = 30.0;
    const uint32_t age = frame_ - child.born;
    if (age % kEnvelopeEvery != 0U || static_cast<double>(age) * 0.01 > kWindowS) {
      return;
    }
    V3 t1;
    V3 n1;
    V3 t2;
    V3 n2;
    double r1 = 0.0;
    double r2 = 0.0;
    axis_of_body(main.veh, t1, n1, r1);
    axis_of_body(child.veh, t2, n2, r2);
    const double gap = segment_distance(t1, n1, t2, n2) - r1 - r2;
    Clearance& c = child.clearance;
    const double t = static_cast<double>(age) * 0.01;
    if (!c.cleared && gap > 0.0) {
      c.cleared = true;
      c.t_clear_s = t;
      c.min_gap_m = gap;
      c.t_s = t;
    } else if (c.cleared && gap < c.min_gap_m) {
      c.closing_ms = (c.last_gap - gap) / (kEnvelopeEvery * 0.01);
      c.min_gap_m = gap;
      c.t_s = t;
    }
    c.last_gap = gap;
    c.gap_at_end_m = gap;
    c.seen = true;
  }

  void attach(Body& b) {
    if (cfg_.ideal) {
      if (b.tables != nullptr) {
        b.ideal = std::make_unique<tfc::gnc::Mission>();
        b.ideal->configure(b.tables);
        b.ideal->set_design_mode(true);
      }
    } else if (b.tables != nullptr && !(cfg_.main_external && b.stage < 0)) {
      AvionicsConfig ac = cfg_.avionics;
      ac.tables = b.tables;
      b.av = std::make_unique<Avionics>(ac);
    }
  }

  static tfc::dm::Vec3 dm_of(const V3& v) { return tfc::dm::Vec3{v.x, v.y, v.z}; }

  [[nodiscard]] tfc::gnc::Inputs truth_inputs(const Body& b, uint32_t flight_frame) const {
    tfc::gnc::Inputs in;
    in.frame = flight_frame;
    const State& s = b.veh.state();
    in.r = dm_of(s.r);
    in.v = dm_of(s.v);
    const Loads l = b.veh.current_loads();
    in.specific_force = norm(l.f_thrust + l.f_aero) / std::max(s.m, 1.0);
    in.q = tfc::dm::Quat{s.q.w, s.q.x, s.q.y, s.q.z};
    in.w = dm_of(s.w);
    in.epoch_s = b.veh.time();
    const std::size_t n = std::min<std::size_t>(b.veh.spec().engines.size(), tfc::gnc::kMaxEngines);   // (the ideal controllers hear the engine controllers at once)
    for (std::size_t e = 0; e < n; ++e) {
      in.engine_out |= b.veh.engine_failed(e) ? uint64_t{1} << e : uint64_t{0};
    }
    return in;
  }

  void step_body(Body& b, std::size_t index, bool on_pad) {
    const uint32_t body_frame = frame_ - b.born;
    if (b.av != nullptr) {
      b.av->set_pad(on_pad);
      b.ctl = b.av->step(frame_, b.veh, b.ctl);
    } else if (b.ideal != nullptr) {
      ideal_controls(b, on_pad, body_frame);
    }
    if (on_pad) {
      return;   // the main body stands clamped
    }
    b.veh.step(0.01, b.ctl);
    if (body_frame % kEnvelopeEvery == 0U) {
      b.env.note(b.veh.time(), b.veh.current_loads(), b.veh.mass(), b.veh.heating());
    }
    if (b.ideal != nullptr && b.tables != nullptr && b.tables->phase[b.last_phase].kind == tfc::gnc::kind::kGlide) {
      sample_drag(b);
    }
    if (log_ && log_every_ != 0U && frame_ % log_every_ == 0U) {
      BodyLog l;
      l.t = static_cast<double>(flight_frame()) * 0.01;
      l.body = static_cast<int>(index);
      l.phase = b.last_phase;
      l.altitude = b.veh.altitude();
      l.speed = b.veh.speed();
      l.mass = b.veh.mass();
      log_(l);
    }
  }

  // Ideal control: the mission runs on the truth, the attitude is forced to its reference, the engines and events are as it commands, the actuators sit at the trim.
  void ideal_controls(Body& b, bool on_pad, uint32_t body_frame) {
    if (on_pad) {
      return;
    }
    const uint32_t ff = b.stage < 0 ? frame_ - t0_ : body_frame;
    const tfc::gnc::Inputs in = truth_inputs(b, ff);
    if (!b.ideal->started()) {
      b.ideal->start(in);
    }
    const tfc::gnc::Output mo = b.ideal->step(in);
    b.last_phase = mo.phase;
    if (b.tables->phase[mo.phase].kind == tfc::gnc::kind::kPeg && (b.seeded & (1U << mo.phase)) == 0U) {
      b.seeded |= 1U << mo.phase;
      seeds_.push_back(Seed{b.stage, mo.phase, b.ideal->peg_solution()});
    }
    const tfc::gnc::Mixer& mx = b.tables->mixer[mo.mixer < tfc::gnc::kMaxMixers ? mo.mixer : 0U];
    Controls c;
    c.throttle = static_cast<double>(mo.throttle);
    c.group_mask = mo.groups;
    c.engine_count = mo.engines;
    c.events = mo.events;
    for (unsigned k = 0; k < 4U; ++k) {
      c.surface_deg[k] = static_cast<double>(mx.trim[k]);
    }
    b.ctl = c;
    if (mo.control) {   // (under a parachute nothing holds the attitude: the canopy does)
      b.veh.set_attitude(Q4{mo.q_ref.w, mo.q_ref.x, mo.q_ref.y, mo.q_ref.z}, V3{mo.w_ref.x, mo.w_ref.y, mo.w_ref.z});
    }
    // (not for a body that is held at rest, by the arms or the ground: its phase goes on running while the rest of the mission is flown, and a gain track of sixteen points spread over those thousands of seconds
    // has none left for the landing burn that came before: the booster's burn was designed from points 210 s apart in a mission whose ship coasts for an hour)
    if (probe_ && cfg_.probe_every != 0U && frame_ % cfg_.probe_every == 0U && !b.veh.caught() && !b.veh.landed() && !b.veh.crashed()) {
      emit_probe(b, mo, ff);
    }
  }

  // The drag the vehicle really has on its way down, against the model the guidance predicts with (density rho0 exp(-h / H), drag deceleration q / beta): the deceleration along the velocity from the plant's own
  // aerodynamic force, weighted by the dynamic pressure so that the thick air counts for most.
  void sample_drag(Body& b) {
    const State& s = b.veh.state();
    const double speed = norm(s.v);
    const tfc::descent::DragModel& dm = b.tables->drag;
    const double rho = dm.rho0 * std::exp(-std::max(b.veh.altitude(), 0.0) / dm.scale_height);
    const double q = 0.5 * rho * speed * speed;
    if (!(q > 500.0) || !(s.m > 1.0)) {
      return;
    }
    const Loads l = b.veh.current_loads();
    const double a_drag = -dot(l.f_aero, rotate_inv(s.q, s.v) / speed) / s.m;
    if (a_drag > 0.0) {
      b.drag_num += q * q;
      b.drag_den += q * a_drag;
    }
  }

  void emit_probe(Body& b, const tfc::gnc::Output& mo, uint32_t ff) {
    constexpr double kStep = 0.5;   // degrees of an effector, and of attitude, for the differences
    AxisProbe pr;
    pr.t = static_cast<double>(ff) * 0.01;
    pr.body = b.stage < 0 ? 0 : 1;
    pr.stage = b.stage;
    pr.phase = mo.phase;
    pr.t_in_phase = b.ideal->phase_time_s();
    pr.thrust = b.veh.current_loads().thrust;
    const V3 base = b.veh.angular_accel_with(b.ctl);
    const auto diff = [&base, kStep](const V3& v) { return std::array<double, 3>{(v.x - base.x) / kStep, (v.y - base.y) / kStep, (v.z - base.z) / kStep}; };
    Controls c = b.ctl;
    c.pitch_deg += kStep;
    pr.col[0] = diff(b.veh.angular_accel_with(c));
    c = b.ctl;
    c.yaw_deg += kStep;
    pr.col[1] = diff(b.veh.angular_accel_with(c));
    c = b.ctl;
    c.roll_deg += kStep;
    pr.col[2] = diff(b.veh.angular_accel_with(c));
    for (std::size_t k = 0; k < kSurfaceChannels; ++k) {
      c = b.ctl;
      c.surface_deg[k] += kStep;
      pr.col[3U + k] = diff(b.veh.angular_accel_with(c));
    }
    for (int axis = 0; axis < 3; ++axis) {   // 0 roll, 1 yaw, 2 pitch
      const V3 turned = b.veh.angular_accel_turned(b.ctl, axis, kStep * kDeg2Rad);
      const double da = axis == 0 ? turned.x - base.x : (axis == 1 ? turned.y - base.y : turned.z - base.z);
      pr.a[static_cast<std::size_t>(axis)] = da / (kStep * kDeg2Rad);
    }
    probe_(pr);
  }

  void spawn(double parent_time, const Detached& d) {
    auto child = std::make_unique<Body>(Vehicle6::spawn(d, cfg_.params, cfg_.scenario));
    child->name = d.spec.stages.empty() ? "stage" : d.spec.stages[0].name;
    child->stage = d.stage;
    child->born = frame_;
    child->tables = d.stage >= 0 && static_cast<std::size_t>(d.stage) < kMaxStages ? cfg_.stage_tables[static_cast<std::size_t>(d.stage)] : nullptr;
    child->veh.step(parent_time - d.t, Controls{});   // bring it to the instant the parent has reached
    attach(*child);
    if (d.stage >= 0 && static_cast<std::size_t>(d.stage) < kMaxStages) {
      spawn_mass_[static_cast<std::size_t>(d.stage)] = child->veh.mass();
    }
    if (child->ideal != nullptr) {
      child->ideal->set_mass(child->veh.mass());
    }
    if (child->av != nullptr) {
      child->av->align_to(child->veh);
      child->av->set_pad(false);
    }
    bodies_.push_back(std::move(child));
  }

  WorldConfig cfg_;
  std::vector<std::unique_ptr<Body>> bodies_;
  uint32_t frame_ = 0U;
  uint32_t t0_ = 0U;
  bool clamped_ = false;
  std::vector<Seed> seeds_;
  std::array<double, kMaxStages> spawn_mass_{};
  std::function<void(const AxisProbe&)> probe_;
  std::function<void(const BodyLog&)> log_;
  uint32_t log_every_ = 0U;
};

}  // namespace sim
