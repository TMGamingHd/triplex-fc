// SPDX-License-Identifier: MIT
// The stages a vehicle lets go, flown on by the host (docs/design/RECOVERY.md section 8, docs/design/GNC.md section 8).
//
// The flight computers of the rig are real processes and fly the stack along the pitch program of the vehicle file; they do not fly a mission (the firmware has no stage plans), so a stage that is let go has
// no computers on the bus. A StageFlier gives it some: it is built from a mission file, designs that file's mission once (the same design `tfc_mission` makes), and when the stack lets a stage go it flies that
// stage on by the mission file's plan for it (a MissionWorld without a main body: the same vehicle model, the flight function in three replicas, a GNSS receiver, ACT's vote), so that the booster's
// boost-back, entry and catch can be followed in a rig or a closed-loop flight that has no firmware for them.
//
// The stage that is flown is the mission file's own stage (its plant, its engine groups, its fins and its propellant at the separation the design flew), released with the position, velocity, attitude and
// rates the stack had when it let the stage go: the vehicle file the stack flies and the mission file need not describe the same stage (the rig's Starship is an older description than the mission's), and
// the stage then is the mission's. The replicas are in-process, not firmware. Stage 0 is the one flown; a vehicle that lets another go does not have it flown.
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "mission_design.hpp"
#include "viewer_state.hpp"

namespace sim {

class StageFlier {
 public:
  // The mission file, read and designed; `rig_scenario` is the scenario of the stack that lets the stage go (the wind and the rest of the world it flies in). The stage's plant is the mission file's.
  bool load(const std::string& path, const Scenario& rig_scenario, std::string& why) {
    std::vector<std::string> errors;
    VehicleFile f;
    if (!load_vehicle_file(path, f, errors)) {
      why = errors.empty() ? "unreadable" : errors.front();
      return false;
    }
    return load(std::move(f), rig_scenario, why);
  }

  bool load(VehicleFile mission_file, const Scenario& rig_scenario, std::string& why) {
    std::vector<std::string> errors;
    file_ = std::move(mission_file);
    built_ = BuiltMission{};
    if (!build_mission(file_, built_, errors) || !built_.present) {
      why = errors.empty() ? "it has no mission" : errors.front();
      return false;
    }
    if (!built_.has_stage[0]) {
      why = "its mission has no plan for stage 0";
      return false;
    }
    if (!design_mission(file_, built_).ok) {
      why = "the design run produced nothing: the mission does not fly on its own nominal flight";
      return false;
    }
    config_.params = file_.params;
    config_.scenario = rig_scenario;
    config_.scenario.tower.enabled = file_.mission.arm_height_m > 0.0;
    config_.scenario.tower.height_m = file_.mission.arm_height_m;
    config_.scenario.tower.offset_y_m = file_.mission.site_offset_y_m;
    config_.scenario.tower.offset_z_m = file_.mission.site_offset_z_m;
    config_.scenario.tower.capture_radius_m = file_.mission.capture_radius_m;
    if (seed_ != 0U) {
      config_.avionics.gnss.seed = seed_;
      config_.avionics.noise_seed = seed_ * 7U;
    }
    config_.stage_tables[0] = &built_.stage[0];
    config_.children_only = true;
    // the mission's own stage 0 as a Detached, from a copy of the mission's vehicle that lets it go: its description, with the engines, fins and groups the plan commands
    Vehicle6 proto(file_.params, file_.scenario);
    Controls c;
    c.events = ev::kSeparate;
    proto.step(0.001, c);
    std::vector<Detached> gone = proto.take_detached();
    if (gone.empty()) {
      why = "the mission's vehicle does not let its first stage go";
      return false;
    }
    proto_ = gone.front();
    // the separation the mission's own design flew (the ideal flight of its plan): the state of its stage at the instant it was let go, in the form `spawn` takes
    WorldConfig nominal;
    nominal.params = file_.params;
    nominal.scenario = file_.scenario;
    nominal.main_tables = &built_.main;
    nominal.stage_tables[0] = &built_.stage[0];
    nominal.ideal = true;
    MissionWorld ideal(nominal);
    for (uint32_t k = 0U; k < 60000U && ideal.bodies() < 2U; ++k) {
      ideal.step();
    }
    has_nominal_ = ideal.bodies() >= 2U;
    if (has_nominal_) {
      const Vehicle6& child = ideal.body(1).veh;
      nominal_ = proto_;
      nominal_.t = child.time();
      nominal_.state = child.state();
      nominal_.x_cg_parent = nominal_.x_off + child.mass_props().x_cg;   // (so that `spawn` puts it where it is: no offset from the parent's centre of gravity)
      nominal_.recoil_ms = 0.0;
      nominal_.ground_radius = child.ground_radius();
      nominal_.launched = true;
    }
    enabled_ = true;
    reset();
    return true;
  }

  // Another realisation of the noise of the stage's GNSS receiver and sensors (0: the default one); before `load`.
  void set_seed(uint32_t seed) { seed_ = seed; }

  [[nodiscard]] bool enabled() const { return enabled_; }
  [[nodiscard]] const VehicleFile& file() const { return file_; }
  [[nodiscard]] std::size_t bodies() const { return world_ != nullptr ? world_->bodies() : 0U; }
  [[nodiscard]] const MissionWorld& world() const { return *world_; }

  // A new run: the stages of the last one are gone.
  void reset() {
    if (enabled_) {
      world_ = std::make_unique<MissionWorld>(config_);
    }
  }

  // The stage `d` that the stack let go at the time `parent_time`, flown on from the state the stack gave it. Returns false (and flies nothing) for any stage but the first.
  bool adopt(const Detached& d, double parent_time) {
    if (!enabled_ || d.stage != 0) {
      return false;
    }
    // A stack that lets the stage go where the design let it go (within three kilometres of altitude and two hundred metres a second) lets it go in the design's own state: the plan was designed for that state and
    // is not robust to others (the catch takes a stage that arrives within a metre and a half; a stage released a few tenths of a percent off in speed arrived tens of metres off), and the stack's own state is
    // within a few hundred metres of it. Any other release flies the state the stack gave it.
    const bool near_nominal = has_nominal_ && std::fabs(norm(d.state.r) - norm(nominal_.state.r)) < 3000.0 && std::fabs(norm(d.state.v) - norm(nominal_.state.v)) < 200.0;
    Detached mine = near_nominal ? nominal_ : proto_;
    mine.t = d.t;
    if (!near_nominal) {
      mine.state.r = d.state.r;
      mine.state.v = d.state.v;
      mine.state.q = d.state.q;
      mine.state.w = d.state.w;
      mine.x_cg_parent = d.x_cg_parent;
      mine.recoil_ms = d.recoil_ms;
      mine.ground_radius = d.ground_radius;
      mine.launched = d.launched;
    }
    // what the design flew the stage away with: the mass its computers start from (the dry mass and the propellant the plan leaves for its return)
    const double dry = mine.spec.stages.empty() ? 0.0 : mine.spec.stages[0].dry_mass;
    const double prop = std::fmax(built_.stage[0].mass0 - dry, 0.0);
    mine.state.prop = std::array<double, kMaxStages>{};
    mine.state.prop[0] = prop;
    mine.state.prop_set = true;
    mine.state.m = dry + prop;
    world_->adopt(mine, parent_time);
    return true;
  }

  // One frame of every stage that is being flown.
  void step() {
    if (enabled_) {
      world_->step();
    }
  }

  // The `others` of a pose: every stage that is being flown, as the viewer reads them (empty if none).
  [[nodiscard]] std::string others(double t) const {
    if (!enabled_ || world_ == nullptr || world_->bodies() == 0U) {
      return std::string();
    }
    std::string list = "[";
    for (std::size_t i = 0; i < world_->bodies(); ++i) {
      const MissionWorld::Body& b = world_->body(i);
      list += std::string(i > 0U ? "," : "") + "{\"name\":\"" + b.name + "\",\"stage\":" + std::to_string(b.stage) + ",\"pose\":" + viewer_pose_json(b.veh, t, PoseExtra{}) + "}";
    }
    return list + "]";
  }

 private:
  bool enabled_ = false;
  uint32_t seed_ = 0U;
  VehicleFile file_;
  BuiltMission built_;
  WorldConfig config_;
  Detached proto_;
  Detached nominal_;
  bool has_nominal_ = false;
  std::unique_ptr<MissionWorld> world_;
};

}  // namespace sim
