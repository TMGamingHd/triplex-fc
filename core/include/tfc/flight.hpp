// SPDX-License-Identifier: MIT
// The flight function of one flight computer: the sensor frames it hears (its own and its peers') go through the sensor consensus, the attitude
// estimator and the scheduled controller to this node's command and state digest. This is the whole of what a node computes between the SYNC and
// the command slot, kept in `core/` so that the firmware, the host tests and the simulator's closed loop run the same code (docs/CONTROL_LOOP.md).
// Every replica fed the same frames produces the same bits (the estimator and controller use only + - * / and sqrt).
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

#include "tfc/consensus.hpp"
#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
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

  // The mission state for this frame (docs/LAUNCH_SEQUENCE.md). On the pad the schedules sit at their first point; in flight they follow `flight_frame`, the frames since T-zero.
  // Never called: the schedules follow the SYNC frame number, as before.
  void set_mission(bool pad, uint32_t flight_frame) noexcept {
    mission_set_ = true;
    pad_ = pad;
    flight_frame_ = flight_frame;
  }

  // This computer's view of its inputs: the sensors give a trustworthy consensus and the attitude is valid. (A computer is ready for launch when this holds and its own IMU's
  // calibration is ready, `ImuCalibrator::ready()`.)
  [[nodiscard]] bool sensors_ok() const noexcept {
    const ConsensusInput c = consensus_.consensus();
    return c.gyro_ok && c.accel_ok && estimator_.attitude().valid;
  }

  // At the command slot: update the estimator, run the controller with this frame's gains and reference, and return the command with the digest.
  [[nodiscard]] Command step() noexcept {
    estimator_.update(consensus_.consensus(), kFramePeriodS);
    const uint32_t idx = !mission_set_ ? frame_ : (pad_ ? 0U : flight_frame_);
    controller_.set_gains(gains_.at(idx));
    Command c = controller_.step(estimator_.attitude(), guidance_.at(idx), kFramePeriodS);
    c.state_digest = static_cast<uint16_t>(estimator_.digest() ^ controller_.digest());
    return c;
  }

  // The state resynchronisation (docs/RESYNC.md): this computer's state in the form the replicas exchange, and the adoption of the voted one. False if the voted state is not usable.
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

  [[nodiscard]] const AttitudeEstimator& estimator() const noexcept { return estimator_; }
  [[nodiscard]] const Controller& controller() const noexcept { return controller_; }
  [[nodiscard]] ConsensusInput consensus() const noexcept { return consensus_.consensus(); }
  [[nodiscard]] uint32_t frame() const noexcept { return frame_; }

 private:
  GainSchedule gains_{};
  Guidance guidance_{};
  SensorConsensus consensus_{};
  AttitudeEstimator estimator_{};
  Controller controller_{};
  uint32_t frame_ = 0U;
  uint32_t flight_frame_ = 0U;
  bool mission_set_ = false;
  bool pad_ = false;
};

}  // namespace tfc
