// SPDX-License-Identifier: MIT
// The flight function of one flight computer: the sensor frames it hears (its own and its peers') go through the sensor consensus, the attitude
// estimator and the scheduled controller to this node's command and state digest. This is the whole of what a node computes between the SYNC and
// the command slot, kept in `core/` so that the firmware, the host tests and the simulator's closed loop run the same code (docs/design/CONTROL_LOOP.md).
// Every replica fed the same frames produces the same bits (the estimator and controller use only + - * / and sqrt).
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

#include "tfc/attitude.hpp"
#include "tfc/consensus.hpp"
#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
#include "tfc/mission.hpp"
#include "tfc/nav.hpp"
#include "tfc/protocol.hpp"
#include "tfc/resync.hpp"

namespace tfc {

constexpr float kFramePeriodS = 0.01F;  // the 100 Hz major frame

class FlightFunction {
 public:
  FlightFunction() noexcept = default;
  FlightFunction(const GainSchedule& gains, const Guidance& guidance) noexcept : gains_(gains), guidance_(guidance) {}
  FlightFunction(const GainSchedule& gains, const Guidance& guidance, const EstimatorConfig& estimator) noexcept
      : gains_(gains), guidance_(guidance), estimator_(estimator) {}

  // Start of frame `frame` (SYNC's number). `usable_nodes` (bit n = node n) says whose sensors may be used: the nodes the fault manager has not
  // excluded.
  void begin_frame(uint32_t frame, uint8_t usable_nodes) noexcept {
    frame_ = frame;
    consensus_.begin_frame(usable_nodes);
  }

  // Offer any frame heard in this frame; only gyro and accelerometer frames of the three nodes that carry THIS frame's number (the low byte, ADR-018) are used.
  // A late frame of an earlier cycle is stale: if it were used, two nodes that happened to receive a different mix of frames would compute different inputs and
  // their estimators would drift apart. True if a frame was used.
  bool on_frame(const Frame& f) noexcept { return f.data[6] == static_cast<uint8_t>(frame_ & 0xFFU) && consensus_.on_frame(f); }

  // The mission state for this frame (docs/design/LAUNCH_SEQUENCE.md). On the pad the schedules sit at their first point; in flight they follow `flight_frame`, the frames since T-zero.
  // Never called: the schedules follow the SYNC frame number, as before.
  void set_mission(bool pad, uint32_t flight_frame) noexcept {
    mission_set_ = true;
    pad_ = pad;
    flight_frame_ = flight_frame;
  }

  // Turn on guidance and navigation (docs/design/GNC.md): the computer then navigates from its accelerometers, the attitude estimator and the GNSS fixes it is offered, flies the mission of `tables` from T-zero
  // (the attitude loop is the three-axis one, the commands are the gimbal's, the surfaces', the propulsion's), and `prop()` and `surf()` carry what its guidance asks beyond the gimbal. Without it nothing changes.
  // `tables` must outlive the function (it is constant data in the firmware).
  void enable_gnc(const gnc::Tables* tables) noexcept {
    gnc_ = tables;
    nav_ = nav::Navigator(tables->nav);
    mission_.configure(tables);
    gnc_started_ = false;
  }
  [[nodiscard]] bool gnc_enabled() const noexcept { return gnc_ != nullptr; }

  // Offer a GNSS frame (0x506 to 0x508, the three of a fix carry the frame number's low byte). True if it completed the fix of this frame.
  bool on_gnss_frame(const Frame& f) noexcept {
    if (gnc_ == nullptr || f.data[6] != static_cast<uint8_t>(frame_ & 0xFFU)) {
      return false;
    }
    if (gnss_.offer(f)) {
      fix_pending_ = true;
      return true;
    }
    return false;
  }

  // This computer's view of its inputs: the sensors give a trustworthy consensus and the attitude is valid. (A computer is ready for launch when this holds and its own IMU's
  // calibration is ready, `ImuCalibrator::ready()`.)
  [[nodiscard]] bool sensors_ok() const noexcept {
    const ConsensusInput c = consensus_.consensus();
    return c.gyro_ok && c.accel_ok && estimator_.attitude().valid;
  }

  // At the command slot: update the estimator, run the controller with this frame's gains and reference, and return the command with the digest.
  [[nodiscard]] Command step() noexcept {
    if (gnc_ != nullptr) {
      estimator_.set_use_accel(mission_set_ && pad_);   // on the pad the accelerometer measures gravity and corrects the tilt; in flight it measures thrust and drag, which are not gravity
    }
    estimator_.update(consensus_.consensus(), kFramePeriodS);
    if (gnc_ != nullptr) {
      return step_gnc();
    }
    const uint32_t idx = !mission_set_ ? frame_ : (pad_ ? 0U : flight_frame_);
    controller_.set_gains(gains_.at(idx));
    Command c = controller_.step(estimator_.attitude(), guidance_.at(idx), kFramePeriodS);
    c.state_digest = static_cast<uint16_t>(estimator_.digest() ^ controller_.digest());
    return c;
  }

  // What the guidance asked for in the last step beyond the gimbal: the engines, the events (the propulsion frame) and the four surface deflections (the surface frame).
  [[nodiscard]] const PropCommand& prop() const noexcept { return prop_; }
  [[nodiscard]] const SurfCommand& surf() const noexcept { return surf_; }
  [[nodiscard]] const nav::Navigator& navigator() const noexcept { return nav_; }
  [[nodiscard]] const gnc::Mission& mission() const noexcept { return mission_; }
  [[nodiscard]] const att::Controller3& attitude_controller() const noexcept { return att_ctrl_; }

  // The state resynchronisation (docs/design/RESYNC.md): this computer's state in the form the replicas exchange, and the adoption of the voted one. False if the voted state is not usable.
  [[nodiscard]] resync::SharedState shared_state() const noexcept { return resync::make_shared(estimator_.state(), controller_.state()); }
  bool adopt_state(const resync::SharedState& s) noexcept {
    AttitudeEstimator::State e = estimator_.state();
    Controller::State c = controller_.state();
    if (!resync::make_state(s, estimator_.steps(), e, c)) {
      return false;
    }
    estimator_.set_state(e);
    controller_.set_state(c);
    return true;
  }

  // Put the computer in the state of a vehicle already in flight (a stage that has been let go, whose computers have navigated since lift-off), or restore a restarted computer from a peer.
  void set_attitude_state(const AttitudeEstimator::State& s) noexcept { estimator_.set_state(s); }
  void set_navigation_state(const nav::Navigator::State& s) noexcept { nav_.set_state(s); }
  [[nodiscard]] AttitudeEstimator::State attitude_state() const noexcept { return estimator_.state(); }

  [[nodiscard]] const AttitudeEstimator& estimator() const noexcept { return estimator_; }
  [[nodiscard]] const Controller& controller() const noexcept { return controller_; }
  [[nodiscard]] ConsensusInput consensus() const noexcept { return consensus_.consensus(); }
  [[nodiscard]] uint32_t frame() const noexcept { return frame_; }

 private:
  // The step of a computer with guidance and navigation: navigate, fly the mission's phase, close the three-axis loop, mix the demand into the effectors' commands.
  Command step_gnc() noexcept {
    const ConsensusInput in = consensus_.consensus();
    nav_.propagate(estimator_.quaternion(), Vec3d(in.accel_g), in.accel_ok, static_cast<double>(kFramePeriodS));
    if (fix_pending_) {
      nav_.apply_fix(gnss_.fix());
      fix_pending_ = false;
      const dm::Vec3 turn = nav_.take_attitude_correction();   // (the level frame's axes are the navigation frame's Y, Z, X)
      estimator_.rotate_level({static_cast<float>(turn.y), static_cast<float>(turn.z), static_cast<float>(turn.x)});
    }
    const bool flight = !mission_set_ || !pad_;
    Command c;
    prop_ = PropCommand{};
    surf_ = SurfCommand{};
    if (flight) {
      if (!gnc_started_) {
        t0_frame_ = frame_;
        gnc_started_ = true;
        mission_.start(mission_inputs());
      }
      run_mission();
      c.pitch_deg = static_cast<float>(cmd_pitch_);
      c.yaw_deg = static_cast<float>(cmd_yaw_);
    }
    c.state_digest = static_cast<uint16_t>(estimator_.digest() ^ att_ctrl_.digest() ^ (static_cast<uint16_t>(mission_.phase()) << 8U));
    return c;
  }

  [[nodiscard]] static dm::Vec3 Vec3d(const Vec3& v) noexcept {
    return dm::Vec3{static_cast<double>(v.v[0]), static_cast<double>(v.v[1]), static_cast<double>(v.v[2])};
  }

  [[nodiscard]] gnc::Inputs mission_inputs() const noexcept {
    gnc::Inputs mi;
    mi.frame = frame_ - t0_frame_;
    mi.r = nav_.position();
    mi.v = nav_.velocity();
    mi.specific_force = nav_.specific_force_mag();
    mi.q = att::body_to_nav(estimator_.quaternion());
    mi.w = att::body_rates_rad(estimator_.body_rates_dps());
    mi.attitude_valid = estimator_.attitude().valid;
    return mi;
  }

  void run_mission() noexcept {
    const gnc::Inputs mi = mission_inputs();
    const gnc::Output mo = mission_.step(mi);
    const gnc::Mixer& mx = gnc_->mixer[mo.mixer < gnc::kMaxMixers ? mo.mixer : 0U];
    // the gimbal turns the vehicle in proportion to the thrust: the gains were designed at a thrust, and a flight at another (a landing burn on one engine, then on thirteen) takes the demand to the gimbal
    // scaled by the ratio, and lets the demand reach as far as the gimbal's own limit does
    const gnc::GainTrack& track = gnc_->gains[mo.phase < gnc::kMaxPhases ? mo.phase : 0U];
    double eff = 1.0;
    const double thrust_design = static_cast<double>(track.thrust_at(phase_frames(mo.phase)));
    if (thrust_design > kThrustFloorN && static_cast<double>(mo.thrust) > kThrustFloorN) {
      eff = dm::clamp_(thrust_design / static_cast<double>(mo.thrust), 0.1, 10.0);
    }
    att::Limits lim_demand = gnc_->limits;
    lim_demand.command_deg = static_cast<float>(static_cast<double>(lim_demand.command_deg) / eff);
    lim_demand.integrator_deg = static_cast<float>(static_cast<double>(lim_demand.integrator_deg) / eff);
    lim_demand.slew_deg_per_frame = static_cast<float>(static_cast<double>(lim_demand.slew_deg_per_frame) / eff);
    att::Demand d{};
    if (mo.control) {
      d = att_ctrl_.step(mi.q, mi.w, mo.q_ref, mo.w_ref, track.at(phase_frames(mo.phase)), lim_demand, kFramePeriodS, mi.attitude_valid);
    }
    const double lim = static_cast<double>(gnc_->limits.command_deg);
    cmd_pitch_ = dm::clamp_(static_cast<double>(mx.gimbal_pitch) * d.pitch * eff, -lim, lim);
    cmd_yaw_ = dm::clamp_(static_cast<double>(mx.gimbal_yaw) * d.yaw * eff, -lim, lim);
    prop_.throttle = mo.throttle;
    prop_.groups = mo.groups;
    prop_.events = mo.events;
    prop_.phase = mo.phase;
    prop_.roll_deg = static_cast<float>(static_cast<double>(mx.roll) * d.roll);
    gnc::SurfAlloc al;
    if (track.has_alloc) {
      al = track.alloc_at(phase_frames(mo.phase));
    } else {
      al = gnc::SurfAlloc{mx.from_pitch, mx.from_yaw, mx.from_roll};
    }
    for (unsigned k = 0; k < 4U; ++k) {
      const double u = static_cast<double>(mx.trim[k]) + (static_cast<double>(al.from_pitch[k]) * d.pitch) + (static_cast<double>(al.from_yaw[k]) * d.yaw) + (static_cast<double>(al.from_roll[k]) * d.roll);
      surf_.deg[k] = static_cast<float>(dm::clamp_(u, static_cast<double>(mx.lo[k]), static_cast<double>(mx.hi[k])));
    }
  }

  // Frames into the current phase, for the gain schedule.
  [[nodiscard]] uint32_t phase_frames(uint8_t phase) const noexcept { return phase == mission_.phase() ? static_cast<uint32_t>(mission_.phase_time_s() * 100.0 + 0.5) : 0U; }

  static constexpr double kThrustFloorN = 1.0e4;   // below this the thrust is nothing to scale by
  GainSchedule gains_{};
  Guidance guidance_{};
  SensorConsensus consensus_{};
  AttitudeEstimator estimator_{};
  Controller controller_{};
  uint32_t frame_ = 0U;
  uint32_t flight_frame_ = 0U;
  bool mission_set_ = false;
  bool pad_ = false;
  // guidance and navigation
  const gnc::Tables* gnc_ = nullptr;
  nav::Navigator nav_{};
  nav::GnssCollector gnss_{};
  gnc::Mission mission_{};
  att::Controller3 att_ctrl_{};
  PropCommand prop_{};
  SurfCommand surf_{};
  double cmd_pitch_ = 0.0;
  double cmd_yaw_ = 0.0;
  uint32_t t0_frame_ = 0U;
  bool gnc_started_ = false;
  bool fix_pending_ = false;
};

}  // namespace tfc
