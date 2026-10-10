// SPDX-License-Identifier: MIT
// Navigation: where the vehicle is and how fast it is moving, in the launch-centred inertial frame, from the accelerometers, the attitude estimator and the GNSS fix (docs/design/GNC.md section 3).
//
// The frame. Inertial, non-rotating, centred on the planet, with +X through the launch point at T-zero (up), +Y downrange and +Z crossrange: the frame the vehicle simulator integrates in, and the one
// the guidance targets are written in. A GNSS receiver reports position and velocity in the planet-fixed frame; the receiver's model (the simulator's gateway) hands over the solution already in this frame,
// which is the one modelling shortcut here.
//
// Strapdown. The attitude estimator keeps the quaternion of the sensor frame relative to its level reference (the frame in which gravity is "down" and the heading is the one the vehicle had when it was aligned
// on the pad); the level frame's axes are the navigation frame's (Y, Z, X), so the specific force of the accelerometers, in the sensor frame, is turned into the navigation frame by that quaternion and a permutation.
// Adding gravity gives the acceleration, which is integrated with the velocity-Verlet scheme (second order, and it keeps the energy of a coasting orbit). On the pad the accelerometers read the pad's reaction
// and gravity cancels it, so a vehicle at rest stays where it is without a special case.
//
// Aiding. A GNSS fix (position and velocity) corrects the inertial solution by a fixed fraction of the difference: an alpha-beta observer, the steady-state form of a Kalman filter for the white-acceleration-noise
// model, with the fractions chosen for a fix every tenth of a second. A fix far from the inertial solution is not believed (a gate); a long run of such fixes means the inertial solution is the one that is wrong, and
// the navigator starts again from the fix. With no fix the inertial solution coasts, and the time since the last fix is reported so that the guidance can know how far to trust it.
//
// What it is not: a Kalman filter with a covariance, an estimator of the accelerometers' biases and scale errors, or a model of the earth's rotation inside the navigator (the frame does not rotate).
// All arithmetic is + - * / and sqrt on doubles (dmath.hpp), so three replicas given the same frames compute the same bits.
// No heap, no exceptions, no RTTI.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/dmath.hpp"
#include "tfc/protocol.hpp"

namespace tfc::nav {

using dm::Vec3;

// The gravity of the planet: the inverse-square field and, if it has one, the oblateness term J2 about its pole (the same two terms as the simulator's planet model).
struct Gravity {
  double mu = 3.986004418e14;
  double radius = 6378137.0;
  double j2 = 0.0;
  Vec3 pole{1.0, 0.0, 0.0};

  [[nodiscard]] Vec3 at(Vec3 r) const noexcept {
    const double rn = dm::norm(r);
    const double inv3 = 1.0 / (rn * rn * rn);
    Vec3 g = r * (-mu * inv3);
    if (dm::fabs_(j2) > 0.0) {
      const Vec3 rh = r / rn;
      const double z = dm::dot(rh, pole);
      const double k = 1.5 * j2 * mu * radius * radius / (rn * rn * rn * rn);
      g = g + (((rh * ((5.0 * z * z) - 1.0)) - (pole * (2.0 * z))) * k);
    }
    return g;
  }
};

struct NavConfig {
  Gravity gravity;
  Vec3 r0{6378137.0, 0.0, 0.0};   // the place of the vehicle at start (the pad), navigation frame
  Vec3 v0{};                      // and its velocity (the pad's, if the frame's planet turns)
  double k_pos = 0.35;            // the fraction of the position difference a fix removes
  double k_vel = 0.30;            // and of the velocity difference
  double gate_pos_m = 2000.0;     // a fix further than this from the inertial solution is not believed
  double gate_vel_ms = 60.0;
  uint32_t gate_frames = 30U;     // this many fixes in a row outside the gate: start again from the fix
  // Attitude aiding: under thrust the velocity the fixes keep correcting, across the specific force, is the attitude error (the inertial solution turns the specific force by that error, and the error
  // integrates into velocity). The share of it taken out of the estimated attitude at each fix, and the smallest specific force (m/s^2) at which it is observable.
  double k_att = 0.02;
  double f_att_min = 8.0;
};

struct GnssFix {
  Vec3 r;
  Vec3 v;
};

class Navigator {
 public:
  Navigator() noexcept = default;
  explicit Navigator(const NavConfig& cfg) noexcept : cfg_(cfg), r_(cfg.r0), v_(cfg.v0) {}

  // The turn from the sensor frame to the navigation frame by the estimator's quaternion (w, x, y, z: sensor frame relative to the level reference): rotate into the level frame, then permute its axes
  // (level x, y, z) to the navigation frame's (Y, Z, X).
  [[nodiscard]] static Vec3 sensor_to_nav(const std::array<float, 4>& q, Vec3 sensor) noexcept {
    const dm::Quat qd{static_cast<double>(q[0]), static_cast<double>(q[1]), static_cast<double>(q[2]), static_cast<double>(q[3])};
    const Vec3 level = dm::rotate(dm::normalized(qd), sensor);
    return Vec3{level.z, level.x, level.y};
  }

  // One frame of dt seconds. `accel_g` is the consensus accelerometer in the sensor frame, in g (the specific force); `accel_ok` says it is trustworthy (otherwise the last good value is held for a few frames, then
  // dropped and the vehicle coasts). `q` is the estimator's attitude.
  void propagate(const std::array<float, 4>& q, const Vec3& accel_g, bool accel_ok, double dt) noexcept {
    if (accel_ok) {
      f_sensor_ = accel_g * dm::kG0;
      holds_ = 0U;
    } else if (holds_ < kMaxHolds) {
      ++holds_;
    } else {
      f_sensor_ = Vec3{};
    }
    dt_ = dt;
    const Vec3 f = sensor_to_nav(q, f_sensor_);
    f_nav_ = f;
    const Vec3 a0 = f + cfg_.gravity.at(r_);
    const Vec3 r1 = r_ + (v_ * dt) + (a0 * (0.5 * dt * dt));
    const Vec3 a1 = f + cfg_.gravity.at(r1);
    v_ = v_ + ((a0 + a1) * (0.5 * dt));
    r_ = r1;
    a_ = a1;
    ++since_fix_;
  }

  // A GNSS fix. The first one starts the solution; later ones correct it by the configured fractions of the difference unless they are outside the gate (and a long run outside it starts the solution again).
  void apply_fix(const GnssFix& fix) noexcept {
    const Vec3 dr = fix.r - r_;
    const Vec3 dv = fix.v - v_;
    last_dr_ = dr;
    last_dv_ = dv;
    const double f_mag = dm::norm(f_nav_);
    const double span = static_cast<double>(since_fix_) * dt_;
    if (have_fix_ && cfg_.k_att > 0.0 && f_mag > cfg_.f_att_min && span > 0.0 && dm::norm(dr) <= cfg_.gate_pos_m && dm::norm(dv) <= cfg_.gate_vel_ms) {
      // (in steady state the innovation is -(error x f) T / k_vel, so the error across f is -k_vel (f x dv) / (f^2 T); the estimate is turned back by a share of it)
      att_corr_ = att_corr_ + (dm::cross(f_nav_, dv) * (cfg_.k_att * cfg_.k_vel / (f_mag * f_mag * span)));
    }
    if (!have_fix_) {
      r_ = fix.r;
      v_ = fix.v;
      have_fix_ = true;
      since_fix_ = 0U;
      rejected_ = 0U;
      ++fixes_;
      return;
    }
    if (dm::norm(dr) > cfg_.gate_pos_m || dm::norm(dv) > cfg_.gate_vel_ms) {
      ++rejected_;
      ++rejected_total_;
      if (rejected_ >= cfg_.gate_frames) {
        r_ = fix.r;
        v_ = fix.v;
        rejected_ = 0U;
        since_fix_ = 0U;
        ++restarts_;
      }
      return;
    }
    r_ = r_ + (dr * cfg_.k_pos);
    v_ = v_ + (dv * cfg_.k_vel);
    since_fix_ = 0U;
    rejected_ = 0U;
    ++fixes_;
  }

  // The turn (a rotation vector in the navigation frame, rad) the attitude estimate should be given to come closer to what the fixes say, since the last call.
  [[nodiscard]] Vec3 take_attitude_correction() noexcept {
    const Vec3 c = att_corr_;
    att_corr_ = Vec3{};
    return c;
  }

  [[nodiscard]] Vec3 position() const noexcept { return r_; }
  [[nodiscard]] Vec3 velocity() const noexcept { return v_; }
  [[nodiscard]] Vec3 acceleration() const noexcept { return a_; }            // the whole acceleration (specific force and gravity) of the last frame
  [[nodiscard]] Vec3 specific_force() const noexcept { return f_nav_; }      // the acceleration the engines and the air gave it, navigation frame, m/s^2
  [[nodiscard]] double specific_force_mag() const noexcept { return dm::norm(f_sensor_); }
  [[nodiscard]] uint32_t frames_since_fix() const noexcept { return since_fix_; }
  [[nodiscard]] bool has_fix() const noexcept { return have_fix_; }
  [[nodiscard]] uint32_t fixes() const noexcept { return fixes_; }
  [[nodiscard]] uint32_t rejected_fixes() const noexcept { return rejected_total_; }
  [[nodiscard]] uint32_t restarts() const noexcept { return restarts_; }
  [[nodiscard]] Vec3 last_position_innovation() const noexcept { return last_dr_; }
  [[nodiscard]] Vec3 last_velocity_innovation() const noexcept { return last_dv_; }
  [[nodiscard]] const NavConfig& config() const noexcept { return cfg_; }
  [[nodiscard]] double altitude() const noexcept { return dm::norm(r_) - cfg_.gravity.radius; }

  // What the solution carries from one frame to the next, for the state a restarted computer takes from its peers.
  struct State {
    Vec3 r;
    Vec3 v;
    bool have_fix = false;
  };
  [[nodiscard]] State state() const noexcept { return State{r_, v_, have_fix_}; }
  void set_state(const State& s) noexcept {
    r_ = s.r;
    v_ = s.v;
    have_fix_ = s.have_fix;
    since_fix_ = 0U;
  }

 private:
  static constexpr uint32_t kMaxHolds = 5U;   // frames the last specific force is held when the accelerometer consensus is lost

  NavConfig cfg_{};
  Vec3 r_{6378137.0, 0.0, 0.0};
  Vec3 v_{};
  Vec3 a_{};
  Vec3 f_nav_{};
  Vec3 f_sensor_{};
  Vec3 last_dr_{};
  Vec3 last_dv_{};
  Vec3 att_corr_{};
  double dt_ = 0.01;
  uint32_t since_fix_ = 0U;
  uint32_t holds_ = 0U;
  uint32_t rejected_ = 0U;
  uint32_t rejected_total_ = 0U;
  uint32_t restarts_ = 0U;
  uint32_t fixes_ = 0U;
  bool have_fix_ = false;
};

// Collects the three frames of a GNSS fix (they carry the same sequence byte) and says when a fix is complete.
class GnssCollector {
 public:
  // Offer a frame. True when this frame completed a fix, which `fix()` then returns (until the next fix starts).
  bool offer(const Frame& f) noexcept {
    GnssRaw part = raw_;
    uint8_t seq = 0U;
    unsigned idx = 0U;
    if (!unpack_gnss(f, part, seq, idx)) {
      return false;
    }
    if (!any_ || seq != seq_) {   // a new fix starts: forget the parts of the last one
      have_ = 0U;
      seq_ = seq;
      any_ = true;
    }
    raw_ = part;
    have_ = static_cast<uint8_t>(have_ | (1U << idx));
    if (have_ == 0x7U) {
      have_ = 0U;
      ready_ = true;
      return true;
    }
    return false;
  }

  [[nodiscard]] GnssFix fix() const noexcept {
    GnssFix g;
    g.r = Vec3{static_cast<double>(raw_.pos_m[0]), static_cast<double>(raw_.pos_m[1]), static_cast<double>(raw_.pos_m[2])};
    g.v = Vec3{static_cast<double>(raw_.vel_cms[0]) * 0.01, static_cast<double>(raw_.vel_cms[1]) * 0.01, static_cast<double>(raw_.vel_cms[2]) * 0.01};
    return g;
  }
  [[nodiscard]] bool ready() const noexcept { return ready_; }

 private:
  GnssRaw raw_{};
  uint8_t seq_ = 0U;
  uint8_t have_ = 0U;
  bool any_ = false;
  bool ready_ = false;
};

// The same quantities in the units of the bus: a fix to the three frames of 0x506 to 0x508.
inline GnssRaw to_raw(const GnssFix& g) noexcept {
  GnssRaw r;
  const auto round = [](double x) { return static_cast<int32_t>(x >= 0.0 ? x + 0.5 : x - 0.5); };
  r.pos_m = {round(g.r.x), round(g.r.y), round(g.r.z)};
  r.vel_cms = {round(g.v.x * 100.0), round(g.v.y * 100.0), round(g.v.z * 100.0)};
  return r;
}

}  // namespace tfc::nav
