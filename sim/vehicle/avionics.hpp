// SPDX-License-Identifier: MIT
// A set of three flight computers flying a simulated body, without a socket: the same FlightFunction the firmware runs on each of three computers, each with its own IMU model, the real ACT logic voting their
// gimbal commands, a shared GNSS receiver, and the consumer's vote of the propulsion and surface commands (the middle value of three, two of three for each bit). It is what flies a stage that the rig's own
// computers do not (the booster, once the ship has gone on), what the closed-loop mission tests and the offline mission tool use, and what the design tool checks its gains against
// (docs/design/GNC.md section 8).
//
// One call per frame gives the controls of the vehicle for that frame: the truth of the vehicle goes through the sensor models into the three computers, which navigate, fly the mission of their tables and
// command; the results are voted and returned as `Controls`.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "imu_model.hpp"
#include "vehicle6.hpp"
#include "tfc/act.hpp"
#include "tfc/flight.hpp"
#include "tfc/imu_calibration.hpp"
#include "tfc/mission.hpp"
#include "tfc/nav.hpp"
#include "tfc/protocol.hpp"

namespace sim {

// A GNSS receiver: position and velocity in the navigation frame with Gaussian errors, at a fixed rate, with outages. The same fix goes to every computer (one receiver; docs/design/GNC.md says what that does and
// does not model).
struct GnssModel {
  double pos_sigma_m = 2.0;
  double vel_sigma_ms = 0.05;
  uint32_t period_frames = 10U;
  std::vector<std::array<uint32_t, 2>> outages;   // [from, to) in frames since the start of the run
  uint32_t seed = 4242U;

  // The fix for `frame`, if the receiver delivers one then.
  bool fix(uint32_t frame, const V3& r, const V3& v, tfc::nav::GnssFix& out) {
    if (period_frames == 0U || frame % period_frames != 0U) {
      return false;
    }
    for (const std::array<uint32_t, 2>& o : outages) {
      if (frame >= o[0] && frame < o[1]) {
        return false;
      }
    }
    out.r = tfc::dm::Vec3{r.x + (pos_sigma_m * gauss()), r.y + (pos_sigma_m * gauss()), r.z + (pos_sigma_m * gauss())};
    out.v = tfc::dm::Vec3{v.x + (vel_sigma_ms * gauss()), v.y + (vel_sigma_ms * gauss()), v.z + (vel_sigma_ms * gauss())};
    return true;
  }

 private:
  double uniform() {   // (0, 1)
    state_ ^= state_ << 13U;
    state_ ^= state_ >> 7U;
    state_ ^= state_ << 17U;
    return (static_cast<double>(state_ >> 11U) + 0.5) / 9007199254740992.0;
  }
  double gauss() {
    if (state_ == 0U) {
      state_ = 0x9E3779B97F4A7C15ULL ^ (static_cast<uint64_t>(seed) * 0xBF58476D1CE4E5B9ULL);
    }
    return std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * kPi * uniform());
  }
  uint64_t state_ = 0U;
};

struct AvionicsConfig {
  const tfc::gnc::Tables* tables = nullptr;
  tfc::EstimatorConfig estimator;
  SensorErrors sensors;
  uint32_t noise_seed = 0U;
  GnssModel gnss;
  bool calibrate_on_pad = true;      // each computer measures its gyros' zero-rate level while the vehicle stands on the pad, and subtracts it
};

class Avionics {
 public:
  explicit Avionics(const AvionicsConfig& cfg)
      : cfg_(cfg),
        ff_{tfc::FlightFunction(tfc::GainSchedule{}, tfc::Guidance{}, cfg.estimator), tfc::FlightFunction(tfc::GainSchedule{}, tfc::Guidance{}, cfg.estimator),
            tfc::FlightFunction(tfc::GainSchedule{}, tfc::Guidance{}, cfg.estimator)},
        imu_{ImuModel(cfg.sensors, 0x1234U ^ cfg.noise_seed), ImuModel(cfg.sensors, 0x1235U ^ cfg.noise_seed), ImuModel(cfg.sensors, 0x1236U ^ cfg.noise_seed)} {
    tfc::ActRecord none{};
    act_.boot(tfc::ResetCause::PowerOn, none);
    for (tfc::FlightFunction& f : ff_) {
      f.enable_gnc(cfg.tables);
    }
  }

  // The vehicle stands on the pad (calibrating) or flies. Call every frame before `step`.
  void set_pad(bool on_pad) {
    on_pad_ = on_pad;
    if (cfg_.calibrate_on_pad) {
      for (tfc::ImuCalibrator& c : cal_) {
        c.set_pad(on_pad);
      }
    }
  }

  // Put the three computers in the state a vehicle in flight has at `v`: the attitude estimator aligned with its attitude and the navigator on its place and velocity, as if they had navigated this far.
  void align_to(const Vehicle6& v) {
    const tfc::AttitudeEstimator::State est = estimator_state_of(v.state().q);
    for (tfc::FlightFunction& f : ff_) {
      f.set_attitude_state(est);
      f.set_navigation_state(tfc::nav::Navigator::State{tfc::dm::Vec3{v.state().r.x, v.state().r.y, v.state().r.z}, tfc::dm::Vec3{v.state().v.x, v.state().v.y, v.state().v.z}, true});
    }
  }

  // One frame. `v` is the truth. `frame` counts from the start of the run (the pad included). Returns the controls the vehicle is to be stepped with; `previous` supplies what carries over.
  Controls step(uint32_t frame, const Vehicle6& v, const Controls& previous) {
    const uint8_t seq = static_cast<uint8_t>(frame);
    V3 g;
    V3 a;
    sensors_of(v, g, a);
    std::array<tfc::Frame, 6> sensor{};
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 gs;
      tfc::Vec3 as;
      tfc::Vec3 gt;
      tfc::Vec3 at;
      gt.v = {static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z)};
      at.v = {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)};
      imu_[n].sample(gt, at, gs, as);
      if (cfg_.calibrate_on_pad) {
        gs = cal_[n].process(gs);
      }
      sensor[n] = tfc::pack_gyro(n, gs, seq);
      sensor[3U + n] = tfc::pack_accel(n, as, seq);
    }
    tfc::nav::GnssFix fix;
    const bool have_fix = cfg_.gnss.fix(frame, v.state().r, v.state().v, fix);
    const tfc::GnssRaw raw = tfc::nav::to_raw(fix);
    act_.begin_frame();
    std::array<tfc::Command, 3> cmds{};
    for (unsigned n = 0; n < 3U; ++n) {
      ff_[n].begin_frame(frame, 0x07U);
      ff_[n].set_mission(on_pad_, flight_frame_);
      for (const tfc::Frame& f : sensor) {
        (void)ff_[n].on_frame(f);
      }
      if (have_fix) {
        for (unsigned part = 0; part < 3U; ++part) {
          (void)ff_[n].on_gnss_frame(tfc::pack_gnss(part, raw, seq));
        }
      }
      cmds[n] = ff_[n].step();
      (void)act_.on_frame(tfc::pack_cmd(static_cast<uint8_t>(n), cmds[n], seq));
    }
    act_.safe_request(false);
    const tfc::ActOutput& out = act_.end_frame();
    const tfc::DecodedAct wire = tfc::unpack_act_out(tfc::pack_act_out(tfc::to_act_frame(out), seq));
    last_digests_ = {cmds[0].state_digest, cmds[1].state_digest, cmds[2].state_digest};
    if (!on_pad_) {
      ++flight_frame_;
    }
    return vote(wire.act, previous);
  }

  [[nodiscard]] const tfc::FlightFunction& computer(unsigned n) const { return ff_[n]; }
  [[nodiscard]] const tfc::ActLogic& act() const { return act_; }
  [[nodiscard]] bool digests_agree() const { return last_digests_[0] == last_digests_[1] && last_digests_[1] == last_digests_[2]; }
  [[nodiscard]] bool ready() const { return ff_[0].sensors_ok() && ff_[1].sensors_ok() && ff_[2].sensors_ok() && (!cfg_.calibrate_on_pad || (cal_[0].ready() && cal_[1].ready() && cal_[2].ready())); }
  [[nodiscard]] uint32_t flight_frame() const { return flight_frame_; }

 private:
  // What the sensors would feel: on the pad, nothing turns and the pad's reaction is the local gravity along the vertical; in flight the vehicle's own rates and specific force.
  void sensors_of(const Vehicle6& v, V3& g, V3& a) const {
    if (on_pad_) {
      g = V3{};
      a = V3{0.0, 0.0, norm(v.gravity(v.state().r)) / kG0};
      return;
    }
    v.vehicle_true_imu(g, a);
  }

  // The estimator state for a body at attitude q (body -> inertial): the inverse of the conversion the guidance makes (attitude.hpp), with no bias and no history.
  [[nodiscard]] static tfc::AttitudeEstimator::State estimator_state_of(const Q4& q) {
    const tfc::dm::Quat qd{q.w, q.x, q.y, q.z};
    const tfc::dm::Quat sensor_to_level = tfc::att::estimator_quaternion(qd);
    tfc::AttitudeEstimator::State s;
    s.q = {static_cast<float>(sensor_to_level.w), static_cast<float>(sensor_to_level.x), static_cast<float>(sensor_to_level.y), static_cast<float>(sensor_to_level.z)};
    s.aligned = true;
    s.rates_valid = true;
    s.steps = 1U;
    return s;
  }

  // The consumer's vote: the gimbal from ACT; the throttle and the roll and the surface deflections the middle of the three computers' values; the groups and the events two of three for each bit.
  [[nodiscard]] Controls vote(const tfc::ActFrame& act, const Controls& previous) const {
    Controls c = previous;
    c.pitch_deg = static_cast<double>(act.pitch_deg);
    c.yaw_deg = static_cast<double>(act.yaw_deg);
    const tfc::PropCommand& p0 = ff_[0].prop();
    const tfc::PropCommand& p1 = ff_[1].prop();
    const tfc::PropCommand& p2 = ff_[2].prop();
    c.throttle = static_cast<double>(tfc::mid3(p0.throttle, p1.throttle, p2.throttle));
    c.group_mask = tfc::majority3(p0.groups, p1.groups, p2.groups);
    c.events = tfc::majority3(p0.events, p1.events, p2.events);
    for (unsigned k = 0; k < 4U; ++k) {
      c.surface_deg[k] = static_cast<double>(tfc::mid3(ff_[0].surf().deg[k], ff_[1].surf().deg[k], ff_[2].surf().deg[k]));
    }
    c.roll_deg = static_cast<double>(tfc::mid3(p0.roll_deg, p1.roll_deg, p2.roll_deg));
    return c;
  }

  AvionicsConfig cfg_;
  std::array<tfc::FlightFunction, 3> ff_;
  std::array<ImuModel, 3> imu_;
  std::array<tfc::ImuCalibrator, 3> cal_{};
  tfc::ActLogic act_;
  bool on_pad_ = true;
  uint32_t flight_frame_ = 0U;
  std::array<uint16_t, 3> last_digests_{};
};

}  // namespace sim
