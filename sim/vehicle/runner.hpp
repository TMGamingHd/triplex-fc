// SPDX-License-Identifier: MIT
// The simulator's side of the frame loop, without a socket: the vehicle and the platform advance one frame at a time, and each step returns the frames the
// simulator publishes on the flight bus (docs/VEHICLE_SIM.md section 6, docs/PROTOCOL.md). tools/sim/tfc_simd.cpp puts it on SocketCAN; the host tests put the real
// flight computers and ACT on the other end of the same frames.
//
// The sensor inputs are published one frame ahead: when ACT's output for frame k arrives, the vehicle is stepped over frame k with that command and the sensor frames
// for the START of frame k+1 are sent at once, long before that frame's IMU latch (0.5 ms after its SYNC), so they are waiting in every node's queue and the
// timing of the bus cannot make a sample late. What a node latches is what an IMU would measure at that instant: the state after all the commands so far.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "design.hpp"
#include "platform.hpp"
#include "tfc/protocol.hpp"
#include "vehicle6.hpp"

namespace sim {

struct RunnerConfig {
  Params params;                   // the vehicle that is flown
  Params design;                   // the vehicle the pitch program and gains were designed on (they differ when the flown vehicle is dispersed)
  Scenario scenario;
  bool vehicle_true = false;       // the sensors feel the vehicle itself (rates and specific force) instead of the platform's tilt and gravity
  uint32_t status_every = 10U;     // frames between the state, telemetry and flags frames (10 = 10 Hz)
};

struct SimFrames {
  std::array<tfc::Frame, 5> f{};
  unsigned n = 0;
  void add(const tfc::Frame& fr) { f[n++] = fr; }
};

class SimRunner {
 public:
  explicit SimRunner(const RunnerConfig& cfg = RunnerConfig{})
      : cfg_(cfg), tables_(flight_tables(cfg.design)), vehicle_(cfg.params, cfg.scenario) {}

  // The first SYNC heard carries frame `k0`. Bring the world to the start of that frame (the engines have been running since frame 0, with the gimbal neutral)
  // and return the frames for it. (They reach a node late in frame k0, so a node misses frame k0's sample: that is the price of joining a run in progress.)
  SimFrames start(uint32_t k0) {
    vehicle_ = Vehicle6(cfg_.params, cfg_.scenario);
    platform_ = Platform();
    held_pitch_ = 0.0;
    held_yaw_ = 0.0;
    flags_ = 0U;
    for (uint32_t k = 0; k < k0; ++k) {
      advance(0.0, 0.0);
    }
    frame_ = k0;
    return publish(k0);
  }

  // Frame `k` has ended: apply ACT's command for it (`act` is null if ACT's frame did not come, and then the last command is held), advance the world over the
  // frame, and return the frames for frame k+1.
  SimFrames end_of_frame(uint32_t k, const tfc::ActFrame* act) {
    if (act != nullptr) {
      held_pitch_ = static_cast<double>(act->pitch_deg);
      held_yaw_ = static_cast<double>(act->yaw_deg);
      if (act->state == static_cast<uint8_t>(2U)) {
        flags_ = static_cast<uint8_t>(flags_ | tfc::simflag::kSafed);
      }
    }
    held_ = act == nullptr;
    advance(held_pitch_, held_yaw_);
    frame_ = k + 1U;
    return publish(frame_);
  }

  void abort_run() { flags_ = static_cast<uint8_t>(flags_ | tfc::simflag::kAborted); }

  [[nodiscard]] const Vehicle6& vehicle() const { return vehicle_; }
  [[nodiscard]] const Platform& platform() const { return platform_; }
  [[nodiscard]] uint32_t frame() const { return frame_; }
  [[nodiscard]] bool command_held() const { return held_; }

 private:
  void advance(double pitch_deg, double yaw_deg) {
    vehicle_.step(0.01, pitch_deg, yaw_deg);
    platform_.step(vehicle_.tilts(), 0.01);
  }

  SimFrames publish(uint32_t k) {
    SimFrames out;
    V3 g;
    V3 a;
    if (cfg_.vehicle_true) {
      vehicle_.vehicle_true_imu(g, a);
    } else {
      platform_.imu_truth(g, a);
    }
    const uint8_t seq = static_cast<uint8_t>(k);
    tfc::Vec3 rates;
    tfc::Vec3 accel;
    rates.v = {static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z)};
    accel.v = {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)};
    out.add(tfc::pack_sim_rates(rates, seq));
    out.add(tfc::pack_sim_accel(accel, seq));
    if (cfg_.status_every != 0U && k % cfg_.status_every == 0U) {
      tfc::SimState st;
      st.altitude_m = static_cast<float>(vehicle_.altitude());
      st.speed_ms = static_cast<float>(vehicle_.speed());
      st.mass_kg = static_cast<float>(vehicle_.mass());
      out.add(tfc::pack_sim_state(st, seq));
      const Loads l = vehicle_.current_loads();
      const Tilts t = vehicle_.tilts();
      const tfc::Reference ref = tables_.guidance.at(k);
      tfc::SimTelemetry tm;
      tm.dynamic_pressure_pa = static_cast<float>(l.dynamic_pressure);
      tm.pitch_error_deg = static_cast<float>(t.y_deg) - ref.tilt_y_deg;
      tm.yaw_error_deg = static_cast<float>(t.x_deg) - ref.tilt_x_deg;
      out.add(tfc::pack_sim_telemetry(tm, seq));
      tfc::SimFlags fl;
      fl.flags = flags_;
      if (platform_.saturated()) {
        fl.flags = static_cast<uint8_t>(fl.flags | tfc::simflag::kPlatformSaturated);
      }
      if (vehicle_.engines_on() < vehicle_.params().engines) {
        fl.flags = static_cast<uint8_t>(fl.flags | tfc::simflag::kEngineOut);
      }
      if (held_) {
        fl.flags = static_cast<uint8_t>(fl.flags | tfc::simflag::kCommandHeld);
      }
      fl.engines_on = static_cast<uint8_t>(vehicle_.engines_on());
      fl.time_frames = k;
      out.add(tfc::pack_sim_flags(fl, seq));
    }
    return out;
  }

  RunnerConfig cfg_;
  FlightTables tables_;
  Vehicle6 vehicle_;
  Platform platform_;
  double held_pitch_ = 0.0;
  double held_yaw_ = 0.0;
  uint8_t flags_ = 0U;
  bool held_ = false;
  uint32_t frame_ = 0U;
};

}  // namespace sim
