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
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "avionics.hpp"
#include "vehicle6.hpp"
#include "tfc/mission.hpp"

namespace sim {

struct BodyLog {
  double t = 0.0;
  int body = 0;
  uint8_t phase = 0U;
  double altitude = 0.0;
  double speed = 0.0;
  double mass = 0.0;
};

// The stiffness and effectiveness of the three axes at a point of a design run: angular acceleration per radian of attitude (a) and per degree of demand (b), roll, yaw, pitch.
struct AxisProbe {
  double t = 0.0;               // s since T-zero
  int body = 0;
  uint8_t phase = 0U;
  double t_in_phase = 0.0;
  std::array<double, 3> a{};    // 1/s^2
  std::array<double, 3> b{};    // rad/s^2 per degree of demand
};

struct WorldConfig {
  Params params;
  Scenario scenario;
  const tfc::gnc::Tables* main_tables = nullptr;
  std::array<const tfc::gnc::Tables*, kMaxStages> stage_tables{};   // by the stage's index in the vehicle: the mission of the computers that fly that stage once it is let go (null: it is not flown)
  AvionicsConfig avionics;                                           // the template for every set of computers (its `tables` is replaced)
  bool ideal = false;
  bool main_external = false;                                        // the main body is flown from outside (the rig): the caller supplies its controls
  uint32_t pad_frames = 0U;                                          // frames the main body stands clamped on the pad before T-zero
  uint32_t probe_every = 100U;                                       // frames between the probes of a design run
};

class MissionWorld {
 public:
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
    Body(const Params& p, const Scenario& s) : veh(p, s) {}
    Body(Vehicle6&& v) : veh(std::move(v)) {}
  };

  explicit MissionWorld(const WorldConfig& cfg) : cfg_(cfg) {
    bodies_.push_back(std::make_unique<Body>(cfg.params, cfg.scenario));
    Body& m = *bodies_[0];
    m.name = "main";
    m.tables = cfg.main_tables;
    attach(m);
    clamped_ = cfg.pad_frames > 0U;
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
        spawn(*bodies_[i], d);
      }
    }
    ++frame_;
  }

  [[nodiscard]] uint32_t frame() const { return frame_; }
  [[nodiscard]] uint32_t flight_frame() const { return clamped_ ? 0U : frame_ - t0_; }
  [[nodiscard]] bool clamped() const { return clamped_; }
  [[nodiscard]] std::size_t bodies() const { return bodies_.size(); }
  [[nodiscard]] const Body& body(std::size_t i) const { return *bodies_[i]; }
  [[nodiscard]] Body& body(std::size_t i) { return *bodies_[i]; }
  // The caller's controls for the main body (when it is flown from outside).
  void set_main_controls(const Controls& c) { bodies_[0]->ctl = c; }
  [[nodiscard]] const WorldConfig& config() const { return cfg_; }

 private:
  void attach(Body& b) {
    if (cfg_.ideal) {
      if (b.tables != nullptr) {
        b.ideal = std::make_unique<tfc::gnc::Mission>();
        b.ideal->configure(b.tables);
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
    const tfc::gnc::Mixer& mx = b.tables->mixer[mo.mixer < tfc::gnc::kMaxMixers ? mo.mixer : 0U];
    Controls c;
    c.throttle = static_cast<double>(mo.throttle);
    c.group_mask = mo.groups;
    c.events = mo.events;
    for (unsigned k = 0; k < 4U; ++k) {
      c.surface_deg[k] = static_cast<double>(mx.trim[k]);
    }
    b.ctl = c;
    b.veh.set_attitude(Q4{mo.q_ref.w, mo.q_ref.x, mo.q_ref.y, mo.q_ref.z}, V3{mo.w_ref.x, mo.w_ref.y, mo.w_ref.z});
    if (probe_ && cfg_.probe_every != 0U && frame_ % cfg_.probe_every == 0U) {
      emit_probe(b, mo, mx, ff);
    }
  }

  void emit_probe(Body& b, const tfc::gnc::Output& mo, const tfc::gnc::Mixer& mx, uint32_t ff) {
    constexpr double kStep = 0.5;   // degrees of demand, and half a degree of attitude, for the differences
    AxisProbe pr;
    pr.t = static_cast<double>(ff) * 0.01;
    pr.body = b.stage < 0 ? 0 : 1;
    pr.phase = mo.phase;
    pr.t_in_phase = b.ideal->phase_time_s();
    const V3 base = b.veh.angular_accel_with(b.ctl);
    for (int axis = 0; axis < 3; ++axis) {   // 0 roll, 1 yaw, 2 pitch
      Controls c = b.ctl;
      apply_demand(c, mx, axis, kStep);
      const V3 up = b.veh.angular_accel_with(c);
      const double db = axis == 0 ? up.x - base.x : (axis == 1 ? up.y - base.y : up.z - base.z);
      pr.b[static_cast<std::size_t>(axis)] = db / kStep;
      const V3 turned = b.veh.angular_accel_turned(b.ctl, axis, kStep * kDeg2Rad);
      const double da = axis == 0 ? turned.x - base.x : (axis == 1 ? turned.y - base.y : turned.z - base.z);
      pr.a[static_cast<std::size_t>(axis)] = da / (kStep * kDeg2Rad);
    }
    probe_(pr);
  }

  // The commands that `deg` degrees of demand on an axis (0 roll, 1 yaw, 2 pitch) makes, through the phase's mixer, added to those already in `c`.
  static void apply_demand(Controls& c, const tfc::gnc::Mixer& mx, int axis, double deg) {
    if (axis == 0) {
      c.roll_deg += static_cast<double>(mx.roll) * deg;
    } else if (axis == 1) {
      c.yaw_deg += static_cast<double>(mx.gimbal_yaw) * deg;
    } else {
      c.pitch_deg += static_cast<double>(mx.gimbal_pitch) * deg;
    }
    for (unsigned k = 0; k < 4U; ++k) {
      const float gain = axis == 0 ? mx.from_roll[k] : (axis == 1 ? mx.from_yaw[k] : mx.from_pitch[k]);
      c.surface_deg[k] += static_cast<double>(gain) * deg;
    }
  }

  void spawn(Body& parent, const Detached& d) {
    auto child = std::make_unique<Body>(Vehicle6::spawn(d, cfg_.params, cfg_.scenario));
    child->name = d.spec.stages.empty() ? "stage" : d.spec.stages[0].name;
    child->stage = d.stage;
    child->born = frame_;
    child->tables = d.stage >= 0 && static_cast<std::size_t>(d.stage) < kMaxStages ? cfg_.stage_tables[static_cast<std::size_t>(d.stage)] : nullptr;
    child->veh.step(parent.veh.time() - d.t, Controls{});   // bring it to the instant the parent has reached
    attach(*child);
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
  std::function<void(const AxisProbe&)> probe_;
  std::function<void(const BodyLog&)> log_;
  uint32_t log_every_ = 0U;
};

}  // namespace sim
