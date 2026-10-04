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

namespace tfc {

constexpr float kFramePeriodS = 0.01F;  // the 100 Hz major frame

class FlightFunction {
 public:
  FlightFunction() noexcept = default;
  FlightFunction(const GainSchedule& gains, const Guidance& guidance) noexcept : gains_(gains), guidance_(guidance) {}

  // Start of frame `frame` (SYNC's number). `usable_nodes` (bit n = node n) says whose sensors may be used: the nodes the fault manager has not
  // excluded.
  void begin_frame(uint32_t frame, uint8_t usable_nodes) noexcept {
    frame_ = frame;
    consensus_.begin_frame(usable_nodes);
  }

  // Offer any frame heard in this frame; only gyro and accelerometer frames of the three nodes are used. True if one was.
  bool on_frame(const Frame& f) noexcept { return consensus_.on_frame(f); }

  // At the command slot: update the estimator, run the controller with this frame's gains and reference, and return the command with the digest.
  [[nodiscard]] Command step() noexcept {
    estimator_.update(consensus_.consensus(), kFramePeriodS);
    controller_.set_gains(gains_.at(frame_));
    Command c = controller_.step(estimator_.attitude(), guidance_.at(frame_), kFramePeriodS);
    c.state_digest = static_cast<uint16_t>(estimator_.digest() ^ controller_.digest());
    return c;
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
};

}  // namespace tfc
