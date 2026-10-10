// SPDX-License-Identifier: MIT
// The mission sequencer: a table-driven machine of phases that turns the navigated state into what the vehicle is asked to do (docs/design/GNC.md section 7). A phase says what is being done (fly a pitch program, steer a
// burn to an orbit with PEG, coast in an attitude, burn back toward the launch site, fall under the grid fins, land), which engines run, which events are asserted, which mixer and gains the attitude loop uses, and what
// ends it. The machine moves to the next phase when that happens; it never moves back.
//
// What a phase produces each frame is a reference attitude (a quaternion and its rate, rate-limited so a half-turn is a smooth manoeuvre and not a step), a throttle, the engine groups that run, the event levels and the
// indices of the mixer and the gains. The attitude loop (attitude.hpp) turns the reference into commands; the mixer turns those into the gimbal, the surfaces and the roll effector.
//
// Every number a phase needs is in the table (`Tables`), which the vehicle's design tool generates and the flight computer carries as constant data. The machine itself holds only the phase, the time in it, the estimated mass and
// the state of the guidance in use, so a replica that is restarted can take them from its peers.
// No heap, no exceptions, no RTTI. Deterministic: + - * / and sqrt on doubles (dmath.hpp).
#pragma once
#include <array>
#include <cstdint>

#include "tfc/attitude.hpp"
#include "tfc/controller.hpp"
#include "tfc/descent.hpp"
#include "tfc/dmath.hpp"
#include "tfc/nav.hpp"
#include "tfc/peg.hpp"

namespace tfc::gnc {

using dm::Quat;
using dm::Vec3;

constexpr unsigned kMaxPhases = 16U;
constexpr unsigned kMaxMixers = 8U;
constexpr unsigned kParams = 12U;
constexpr unsigned kGainPoints = 16U;
constexpr unsigned kThrottlePoints = 16U;
constexpr unsigned kThrottleTracks = 2U;
constexpr uint8_t kNoTrack = 0xFFU;
constexpr double kDt = 0.01;   // s, one frame

// What a phase is.
namespace kind {
constexpr uint8_t kHold = 0U;       // nothing (before the countdown ends, or a place to wait)
constexpr uint8_t kProgram = 1U;    // fly the pitch program of the tables; the throttle from a track or a constant
constexpr uint8_t kPeg = 2U;        // steer a burn to a target state with PEG; ends when it says the burn is done
constexpr uint8_t kCoast = 3U;      // no engines; an attitude by the hold mode
constexpr uint8_t kBoostback = 4U;  // burn against the horizontal velocity until the predicted impact point reaches the landing site
constexpr uint8_t kGlide = 5U;      // fall engines first, steering by the angle of attack toward the landing site
constexpr uint8_t kLanding = 6U;    // the landing burn
constexpr uint8_t kChute = 7U;      // a parachute descent: the events at their altitudes, no attitude control
constexpr uint8_t kDone = 8U;
}  // namespace kind

// What ends a phase.
namespace end {
constexpr uint8_t kNever = 0U;
constexpr uint8_t kTime = 1U;           // end_value seconds in the phase
constexpr uint8_t kSpeedAbove = 2U;     // inertial speed at or above end_value m/s
constexpr uint8_t kApoapsis = 3U;       // the vehicle stops climbing (the radial velocity is no longer positive) after at least end_value seconds
constexpr uint8_t kAltitudeBelow = 4U;  // descending through end_value metres of altitude
constexpr uint8_t kAltitudeAbove = 5U;  // climbing through end_value metres
constexpr uint8_t kCutoff = 6U;         // the guidance in the phase says the burn is over
constexpr uint8_t kMassBelow = 7U;      // the estimated mass has fallen to end_value kg
constexpr uint8_t kAligned = 8U;        // the attitude is within end_value degrees of the reference (and at least a second in the phase)
constexpr uint8_t kTouchdown = 9U;      // at or below the target point's height, or the burn has run out
constexpr uint8_t kIgnition = 10U;      // the landing burn is due: the height above the target point is under end_value metres and has come down to the stopping height at the planned deceleration
}  // namespace end

// The attitude of a coast.
namespace hold {
constexpr uint8_t kInertial = 0U;         // the reference the previous phase ended with
constexpr uint8_t kPrograde = 1U;         // along the velocity
constexpr uint8_t kRetrograde = 2U;       // against it
constexpr uint8_t kRadial = 3U;           // up
constexpr uint8_t kFixed = 4U;            // the unit vector p[0..2]
constexpr uint8_t kRetroHorizontal = 5U;  // against the horizontal part of the velocity
constexpr uint8_t kProHorizontal = 6U;
}  // namespace hold

struct Phase {
  uint8_t kind = kind::kHold;
  uint8_t end = end::kNever;
  uint8_t hold = hold::kInertial;
  uint8_t groups = 0U;            // engine groups commanded on during the phase (Landing: ignored, the guidance picks them from p)
  uint8_t events = 0U;            // propbit levels asserted from the start of the phase
  uint8_t mixer = 0U;
  uint8_t throttle_track = kNoTrack;
  float throttle = 1.0F;
  float end_value = 0.0F;
  float thrust = 0.0F;            // N at full throttle of the groups that run (vacuum), for PEG and the mass flow
  float mdot = 0.0F;              // kg/s at full throttle
  float mass_set = 0.0F;          // if > 0, the mass estimate takes this value at the start of the phase
  float slew_dps = 20.0F;         // the fastest the reference attitude turns
  std::array<float, kParams> p{};
};

struct Mixer {
  float gimbal_pitch = 0.0F;      // gimbal degrees per degree of pitch demand (0: the gimbal is not used in this phase)
  float gimbal_yaw = 0.0F;
  float roll = 0.0F;              // roll effector degrees per degree of roll demand
  std::array<float, 4> trim{};    // surface deflections (degrees) at zero demand
  std::array<float, 4> from_pitch{};
  std::array<float, 4> from_yaw{};
  std::array<float, 4> from_roll{};
  std::array<float, 4> lo{};      // travel
  std::array<float, 4> hi{};
};

struct GainTrack {
  uint8_t n = 0U;
  std::array<uint32_t, kGainPoints> frame{};
  std::array<att::Gains3, kGainPoints> gains{};
  // The gains at `f` frames into the phase: linear between the points, held beyond them.
  [[nodiscard]] att::Gains3 at(uint32_t f) const noexcept {
    if (n == 0U) {
      return att::Gains3{};
    }
    if (f <= frame[0]) {
      return gains[0];
    }
    for (unsigned i = 1; i < n && i < kGainPoints; ++i) {
      if (f <= frame[i]) {
        const float k = static_cast<float>(f - frame[i - 1U]) / static_cast<float>(frame[i] - frame[i - 1U]);
        return att::Gains3{lerp(gains[i - 1U].roll, gains[i].roll, k), lerp(gains[i - 1U].yaw, gains[i].yaw, k), lerp(gains[i - 1U].pitch, gains[i].pitch, k)};
      }
    }
    return gains[n - 1U];
  }

 private:
  static att::AxisGains lerp(const att::AxisGains& a, const att::AxisGains& b, float k) noexcept {
    return att::AxisGains{a.kp + (k * (b.kp - a.kp)), a.kd + (k * (b.kd - a.kd)), a.ki + (k * (b.ki - a.ki))};
  }
};

struct ThrottleTrack {
  uint8_t n = 0U;
  std::array<uint32_t, kThrottlePoints> frame{};
  std::array<float, kThrottlePoints> value{};
  [[nodiscard]] float at(uint32_t f, float fallback) const noexcept {
    if (n == 0U) {
      return fallback;
    }
    if (f <= frame[0]) {
      return value[0];
    }
    for (unsigned i = 1; i < n && i < kThrottlePoints; ++i) {
      if (f <= frame[i]) {
        const float k = static_cast<float>(f - frame[i - 1U]) / static_cast<float>(frame[i] - frame[i - 1U]);
        return value[i - 1U] + (k * (value[i] - value[i - 1U]));
      }
    }
    return value[n - 1U];
  }
};

// Everything the sequencer is configured with.
struct Tables {
  nav::NavConfig nav;
  uint8_t n_phases = 0U;
  std::array<Phase, kMaxPhases> phase{};
  std::array<Mixer, kMaxMixers> mixer{};
  std::array<GainTrack, kMaxPhases> gains{};     // by phase
  std::array<ThrottleTrack, kThrottleTracks> throttle{};
  Guidance program;                              // the pitch program (planes 0 and 1), in frames from the start of the phase that flies it
  att::Limits limits;
  double mass0 = 1.0e6;                          // kg at T-zero
  Vec3 plane_normal{0.0, 0.0, 1.0};              // the orbit plane (angular momentum direction), and the roll reference of the attitudes
  Vec3 site{6378137.0, 0.0, 0.0};                // the pad (the landing site) on the ground, navigation frame
  double ground_radius = 6378137.0;              // the sphere the ground is on
  descent::LandingTarget landing;                // the catch point is `site` raised by landing.point_height (see below)
  double landing_height = 0.0;                   // the catch point's height above the ground
  descent::Engines engines;
  descent::DragModel drag;
  double ignition_margin = 1.15;
};

// What the sequencer is given each frame.
struct Inputs {
  uint32_t frame = 0U;               // frames since T-zero
  Vec3 r;                            // the navigated state
  Vec3 v;
  double specific_force = 0.0;       // m/s^2, the thrust (and drag) acceleration measured
  Quat q;                            // the attitude, body -> navigation
  Vec3 w;                            // the body rates, rad/s
  bool attitude_valid = true;
};

// What it gives back.
struct Output {
  Quat q_ref;
  Vec3 w_ref;
  float throttle = 0.0F;
  uint8_t groups = 0U;
  uint8_t events = 0U;
  uint8_t phase = 0U;
  uint8_t mixer = 0U;
  bool control = true;               // false: the attitude loop is not used (a parachute descent)
};

class Mission {
 public:
  Mission() noexcept = default;

  void configure(const Tables* t) noexcept {
    tab_ = t;
    mass_ = t != nullptr ? t->mass0 : 1.0;
  }
  void set_mass(double m) noexcept { mass_ = m; }   // (a stage's computers know the stage's mass when it is let go)
  [[nodiscard]] bool configured() const noexcept { return tab_ != nullptr && tab_->n_phases > 0U; }

  // T-zero: the first phase begins with the attitude the vehicle has.
  void start(const Inputs& in) noexcept {
    if (!configured()) {
      return;
    }
    q_ref_ = in.q;
    phase_ = 0U;
    enter(in);
    started_ = true;
  }

  [[nodiscard]] Output step(const Inputs& in) noexcept;

  // For the design of the mission on the host: with this on, a PEG phase that has no seed solves its burn from scratch at its first frame (thousands of predictions at once), so that the solution can be recorded and
  // given to the flight computers as the seed (`peg_solution`). It is never on in flight.
  void set_design_mode(bool on) noexcept { design_ = on; }
  [[nodiscard]] std::array<double, peg::kUnknowns> peg_solution() const noexcept { return peg_.solution(); }

  [[nodiscard]] bool started() const noexcept { return started_; }
  [[nodiscard]] uint8_t phase() const noexcept { return phase_; }
  [[nodiscard]] double phase_time_s() const noexcept { return static_cast<double>(in_phase_) * kDt; }
  [[nodiscard]] double mass() const noexcept { return mass_; }
  [[nodiscard]] const peg::Output& peg_out() const noexcept { return peg_out_; }
  [[nodiscard]] const descent::DescentOut& landing_out() const noexcept { return land_out_; }
  [[nodiscard]] Vec3 miss() const noexcept { return miss_; }
  [[nodiscard]] double aoa_deg() const noexcept { return aoa_deg_; }
  [[nodiscard]] bool cutoff() const noexcept { return cutoff_; }

  // What a restarted replica takes from its peers.
  struct State {
    uint8_t phase = 0U;
    uint32_t in_phase = 0U;
    double mass = 1.0;
    bool started = false;
    bool cutoff = false;
    bool burning = false;
  };
  [[nodiscard]] State state() const noexcept { return State{phase_, in_phase_, mass_, started_, cutoff_, burning_}; }
  void set_state(const State& s) noexcept {
    phase_ = s.phase;
    in_phase_ = s.in_phase;
    mass_ = s.mass;
    started_ = s.started;
    cutoff_ = s.cutoff;
    burning_ = s.burning;
  }

 private:
  static constexpr unsigned kPegPeriod = 5U;        // frames between PEG cycles
  static constexpr unsigned kBoostPeriod = 10U;
  static constexpr unsigned kGlidePeriod = 50U;

  [[nodiscard]] const Phase& ph() const noexcept { return tab_->phase[phase_]; }
  void enter(const Inputs& in) noexcept;
  [[nodiscard]] bool ended(const Inputs& in) const noexcept;
  [[nodiscard]] Vec3 hold_direction(const Phase& p, const Inputs& in) const noexcept;
  void reference_toward(Vec3 axis) noexcept;
  void slew_reference(const Phase& p) noexcept;
  void step_program(const Inputs& in, Output& out) noexcept;
  void step_peg(const Inputs& in, Output& out) noexcept;
  void step_coast(const Inputs& in, Output& out) noexcept;
  void step_boostback(const Inputs& in, Output& out) noexcept;
  void step_glide(const Inputs& in, Output& out) noexcept;
  void step_landing(const Inputs& in, Output& out) noexcept;
  void step_chute(const Inputs& in, Output& out) noexcept;
  [[nodiscard]] double altitude(const Inputs& in) const noexcept { return dm::norm(in.r) - tab_->nav.gravity.radius; }

  const Tables* tab_ = nullptr;
  uint8_t phase_ = 0U;
  uint32_t in_phase_ = 0U;
  double mass_ = 1.0;
  bool started_ = false;
  bool cutoff_ = false;
  bool burning_ = false;
  Quat q_ref_{};
  Quat q_target_{};
  Vec3 w_ref_{};
  peg::Peg peg_{};
  peg::Output peg_out_{};
  bool peg_busy_ = false;
  bool design_ = false;
  descent::PoweredDescent descent_{};
  descent::DescentOut land_out_{};
  Vec3 miss_{};
  Vec3 steer_{1.0, 0.0, 0.0};      // the thrust direction the burn guidance last chose
  Vec3 travel_{};                  // the direction of the first miss of a boost-back (toward the site)
  double aoa_deg_ = 0.0;
  double tof_ = 0.0;
  double flow_ = 0.0;               // the mass flow of this frame, kg/s
  Vec3 aoa_dir_{};
  uint8_t latched_events_ = 0U;
};


// ---- the implementation ----

namespace detail {
// The rotation by `angle` about the unit vector `axis`.
inline Quat about(Vec3 axis, double angle) noexcept {
  double s = 0.0;
  double c = 0.0;
  dm::sincos_(0.5 * angle, s, c);
  return Quat{c, axis.x * s, axis.y * s, axis.z * s};
}
}  // namespace detail

inline void Mission::enter(const Inputs& in) noexcept {
  (void)in;
  in_phase_ = 0U;
  cutoff_ = false;
  burning_ = false;
  const Phase& p = ph();
  if (p.mass_set > 0.0F) {
    mass_ = static_cast<double>(p.mass_set);
  }
  peg_.reset();
  peg_busy_ = false;
  if (p.kind == kind::kPeg && p.p[7] > 0.0F) {   // the design's solution is the start: thrust angles at the start and the end of the burn, the burn time
    peg_.seed({static_cast<double>(p.p[3]), static_cast<double>(p.p[4]), static_cast<double>(p.p[5]), static_cast<double>(p.p[6]), static_cast<double>(p.p[7])});
  }
  descent_.reset();
  miss_ = Vec3{};
  travel_ = Vec3{};
  aoa_deg_ = 0.0;
  latched_events_ = 0U;
  flow_ = 0.0;
  q_target_ = q_ref_;
}

inline bool Mission::ended(const Inputs& in) const noexcept {
  const Phase& p = ph();
  const double ev = static_cast<double>(p.end_value);
  const double t = phase_time_s();
  const double alt = altitude(in);
  const double radial = dm::dot(in.r, in.v);
  if (p.end == end::kTime) {
    return t >= ev;
  }
  if (p.end == end::kSpeedAbove) {
    return dm::norm(in.v) >= ev;
  }
  if (p.end == end::kApoapsis) {
    return t >= ev && radial <= 0.0;
  }
  if (p.end == end::kAltitudeBelow) {
    return alt <= ev && radial < 0.0;
  }
  if (p.end == end::kAltitudeAbove) {
    return alt >= ev && radial > 0.0;
  }
  if (p.end == end::kCutoff) {
    return cutoff_;
  }
  if (p.end == end::kMassBelow) {
    return mass_ <= ev;
  }
  if (p.end == end::kAligned) {
    return t >= 1.0 && dm::norm(att::error_vector(in.q, q_target_)) * dm::kRadToDeg <= ev;
  }
  if (p.end == end::kIgnition) {
    const Vec3 pt = dm::unit(tab_->site, Vec3{1.0, 0.0, 0.0}) * (dm::norm(tab_->site) + tab_->landing_height);
    const Vec3 upt = dm::unit(pt, Vec3{1.0, 0.0, 0.0});
    const double h = dm::dot(in.r - pt, upt);
    const double v_down = dm::max_(-dm::dot(in.v, upt), 0.0);
    return h <= ev && h <= descent::PoweredDescent::stopping_height(v_down, tab_->landing.decel_plan, tab_->landing.sink_ms) * tab_->ignition_margin + 1.0;
  }
  if (p.end == end::kTouchdown) {
    const Vec3 pt = dm::unit(tab_->site, Vec3{1.0, 0.0, 0.0}) * (dm::norm(tab_->site) + tab_->landing_height);
    return dm::dot(in.r - pt, dm::unit(pt, Vec3{1.0, 0.0, 0.0})) <= 0.1;
  }
  return false;
}

inline Vec3 Mission::hold_direction(const Phase& p, const Inputs& in) const noexcept {
  const Vec3 up = dm::unit(in.r, Vec3{1.0, 0.0, 0.0});
  const Vec3 vel = dm::unit(in.v, up);
  const Vec3 horiz = dm::unit(dm::perp(in.v, up), vel);
  if (p.hold == hold::kPrograde) {
    return vel;
  }
  if (p.hold == hold::kRetrograde) {
    return -vel;
  }
  if (p.hold == hold::kRadial) {
    return up;
  }
  if (p.hold == hold::kFixed) {
    return dm::unit(Vec3{static_cast<double>(p.p[0]), static_cast<double>(p.p[1]), static_cast<double>(p.p[2])}, up);
  }
  if (p.hold == hold::kRetroHorizontal) {
    return -horiz;
  }
  if (p.hold == hold::kProHorizontal) {
    return horiz;
  }
  return dm::rotate(q_ref_, Vec3{1.0, 0.0, 0.0});   // inertial: where the reference points now
}

inline void Mission::reference_toward(Vec3 axis) noexcept { q_target_ = att::attitude_from_axes(axis, tab_->plane_normal); }

inline void Mission::step_program(const Inputs& in, Output& out) noexcept {
  (void)in;
  const Phase& p = ph();
  const Reference ref = tab_->program.at(in_phase_);
  const double ty = static_cast<double>(ref.tilt_y_deg) * dm::kDegToRad;
  const double tx = static_cast<double>(ref.tilt_x_deg) * dm::kDegToRad;
  const Vec3 dir{dm::cos_(tx) * dm::cos_(ty), dm::cos_(tx) * dm::sin_(ty), -dm::sin_(tx)};
  reference_toward(dir);
  const float thr = p.throttle_track < kThrottleTracks ? tab_->throttle[p.throttle_track].at(in_phase_, p.throttle) : p.throttle;
  out.throttle = thr;
  flow_ = static_cast<double>(p.mdot) * static_cast<double>(thr);
}

inline void Mission::step_peg(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  const double thrust_model = static_cast<double>(p.thrust) * static_cast<double>(p.throttle) / mass_;
  if (!cutoff_) {
    if (!peg_busy_) {   // a cycle of the guidance begins from the present state; the cycle is spread over the frames, a prediction each
      peg::Target tgt;
      tgt.radius = static_cast<double>(p.p[0]);
      tgt.speed = static_cast<double>(p.p[1]);
      tgt.gamma_rad = static_cast<double>(p.p[2]) * dm::kDegToRad;
      tgt.plane_normal = tab_->plane_normal;
      peg::Burn burn;
      burn.accel = in.specific_force > 0.5 * thrust_model ? in.specific_force : thrust_model;
      burn.exhaust_speed = static_cast<double>(p.thrust) / dm::max_(static_cast<double>(p.mdot), 1.0e-9);
      burn.burn_time_max = dm::max_((mass_ - static_cast<double>(p.p[8])) / dm::max_(static_cast<double>(p.mdot) * static_cast<double>(p.throttle), 1.0e-9), 0.0);
      peg_.begin(in.r, in.v, tgt, burn);
      if (design_ && !peg_.started()) {
        (void)peg_.solve(tab_->nav.gravity, 0.05, 100);
        peg_out_ = peg_.output();
      } else {
        peg_busy_ = true;
      }
    }
    if (peg_.work(tab_->nav.gravity)) {
      peg_busy_ = false;
      peg_out_ = peg_.output();
    }
    peg_.advance(kDt);
    cutoff_ = peg_.output().cutoff;
  }
  reference_toward(peg_.direction(in.r));
  if (cutoff_) {
    out.groups = 0U;
    out.throttle = 0.0F;
  } else {
    out.throttle = p.throttle;
    flow_ = static_cast<double>(p.mdot) * static_cast<double>(p.throttle);
  }
}

inline void Mission::step_coast(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  reference_toward(hold_direction(p, in));
  if (p.groups == 0U) {
    out.throttle = 0.0F;
  } else {
    flow_ = static_cast<double>(p.mdot) * static_cast<double>(p.throttle);   // (a burn at a held attitude: the engines of the hot stage running while the stages are still together)
  }
}

inline void Mission::step_boostback(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  if (!cutoff_ && (in_phase_ % kBoostPeriod) == 0U) {
    const Vec3 up = dm::unit(in.r, Vec3{1.0, 0.0, 0.0});
    const descent::Impact imp = descent::ballistic_impact(in.r, in.v, tab_->nav.gravity.mu, tab_->ground_radius);
    if (imp.valid) {
      const Vec3 target = dm::unit(tab_->site, up) * tab_->ground_radius;
      Vec3 m = dm::perp(target - imp.point, up);
      if (dm::norm(travel_) < 0.5) {
        travel_ = dm::unit(m, Vec3{1.0, 0.0, 0.0});   // the direction from the first impact point to the site: the burn ends when the impact point has come that way as far as the aim point
      }
      m = m + (travel_ * static_cast<double>(p.p[0]));   // aim `bias` metres beyond the site (the air will shorten the way back)
      miss_ = m;
      tof_ = imp.tof;
      if (dm::dot(m, travel_) <= 0.0) {
        cutoff_ = true;
      } else {
        steer_ = dm::unit(m, steer_);
        const double e = static_cast<double>(p.p[2]) * dm::kDegToRad;   // the thrust is lifted this far above the horizontal: a higher arc, a steeper and slower fall
        steer_ = dm::unit((steer_ * dm::cos_(e)) + (up * dm::sin_(e)), steer_);
      }
    }
    if (mass_ <= static_cast<double>(p.p[1])) {
      cutoff_ = true;   // the propellant reserve for the landing is not for this burn
    }
  }
  reference_toward(steer_);
  if (cutoff_) {
    out.groups = 0U;
    out.throttle = 0.0F;
  } else {
    out.throttle = p.throttle;
    flow_ = static_cast<double>(p.mdot) * static_cast<double>(p.throttle);
  }
}

inline void Mission::step_glide(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  const Vec3 up = dm::unit(tab_->site, Vec3{1.0, 0.0, 0.0});
  const Vec3 vel = dm::unit(in.v, -up);
  const Vec3 catch_pt = up * (dm::norm(tab_->site) + tab_->landing_height);
  if ((in_phase_ % kGlidePeriod) == 0U) {
    const descent::Impact pred = descent::descent_predict(in.r, in.v, tab_->nav.gravity, tab_->drag, dm::norm(catch_pt), 1.0, 600U);
    if (pred.valid) {
      miss_ = dm::perp(catch_pt - pred.point, up);
      tof_ = pred.tof;
    }
  }
  // Steer by lift, as a zero-effort miss: the angle of attack that, kept to the end of the fall, gives the sideways acceleration that cancels the miss across the path, a = 2 miss / t^2,
  // from the lift per degree the air at this dynamic pressure gives (p[3] square metres per degree of angle of attack, times q over the mass); a steady angle toward the site (p[2]) adds drag.
  const Vec3 across = dm::perp(miss_, vel);
  const double speed = dm::norm(in.v);
  const double alt = altitude(in);
  const double rho = tab_->drag.rho0 * dm::exp_(-dm::max_(alt, 0.0) / tab_->drag.scale_height);
  const double q = 0.5 * rho * speed * speed;
  const double a_per_deg = dm::max_(q * static_cast<double>(p.p[3]) / mass_, 1.0e-3);   // m/s^2 per degree
  const double t_go = dm::max_(tof_, 5.0);
  const Vec3 a_need = across * (2.0 / (t_go * t_go));
  // and the extra angle that slows the fall to the speed the air at this height should bring a falling body to (the speed at the gate scaled by the square root of the density ratio, as a terminal speed is):
  // the tilt is backward against the horizontal motion, so the lift it makes takes speed out of the sideways motion too
  const double rho_gate = tab_->drag.rho0 * dm::exp_(-static_cast<double>(p.p[5]) / tab_->drag.scale_height);
  const double v_ref = static_cast<double>(p.p[4]) * dm::sqrt_(rho_gate / dm::max_(rho, 1.0e-9));
  const double over = dm::clamp_((speed - v_ref) / dm::max_(v_ref, 1.0), 0.0, 1.0);
  const Vec3 horiz = dm::perp(in.v, up);
  const Vec3 back = dm::unit(dm::perp(-horiz, vel), Vec3{0.0, 1.0, 0.0});
  const Vec3 want = (a_need / a_per_deg) + (back * (over * static_cast<double>(p.p[6])));   // degrees
  const double alpha_deg = dm::min_(dm::norm(want), static_cast<double>(p.p[0]));
  const double alpha = alpha_deg * dm::kDegToRad;
  aoa_deg_ = alpha_deg;
  const Vec3 side = dm::unit(want, back);
  const Vec3 axis = (-vel * dm::cos_(alpha)) - (side * dm::sin_(alpha));   // (flying tail first, the air pushes the vehicle away from the side the nose is tilted to)
  reference_toward(axis);
  out.groups = 0U;
  out.throttle = 0.0F;
}

inline void Mission::step_landing(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  const Vec3 up = dm::unit(tab_->site, Vec3{1.0, 0.0, 0.0});
  descent::LandingTarget tgt = tab_->landing;
  tgt.point = up * (dm::norm(tab_->site) + tab_->landing_height);
  land_out_ = descent_.update(in.r, in.v, mass_, tgt, tab_->engines, tab_->nav.gravity, tab_->ignition_margin, burning_);
  burning_ = burning_ || land_out_.ignite;
  miss_ = land_out_.miss;
  reference_toward(land_out_.direction);
  if (!burning_) {
    out.groups = 0U;
    out.throttle = 0.0F;
    return;
  }
  const unsigned opt = land_out_.option < descent::kEngineOptions ? land_out_.option : 0U;
  out.groups = static_cast<uint8_t>(p.p[opt]);
  out.throttle = static_cast<float>(land_out_.throttle);
  flow_ = static_cast<double>(p.mdot) * static_cast<double>(land_out_.engines) * land_out_.throttle;
}

inline void Mission::step_chute(const Inputs& in, Output& out) noexcept {
  const Phase& p = ph();
  const double alt = altitude(in);
  const bool falling = dm::dot(in.r, in.v) < 0.0;
  if (falling && alt <= static_cast<double>(p.p[0])) {
    latched_events_ = static_cast<uint8_t>(latched_events_ | propbit::kChute0);
  }
  if (falling && alt <= static_cast<double>(p.p[1])) {
    latched_events_ = static_cast<uint8_t>(latched_events_ | propbit::kChute1);
  }
  out.events = static_cast<uint8_t>(out.events | latched_events_);
  out.groups = 0U;
  out.throttle = 0.0F;
  out.control = false;
}

inline Output Mission::step(const Inputs& in) noexcept {
  Output out;
  out.q_ref = q_ref_;
  if (!started_ || !configured()) {
    return out;
  }
  if (ended(in) && static_cast<unsigned>(phase_) + 1U < tab_->n_phases) {
    ++phase_;
    enter(in);
  }
  const Phase& p = ph();
  out.phase = phase_;
  out.mixer = p.mixer;
  out.events = p.events;
  out.groups = p.groups;
  out.throttle = p.throttle;
  flow_ = 0.0;
  if (p.kind == kind::kProgram) {
    step_program(in, out);
  } else if (p.kind == kind::kPeg) {
    step_peg(in, out);
  } else if (p.kind == kind::kBoostback) {
    step_boostback(in, out);
  } else if (p.kind == kind::kGlide) {
    step_glide(in, out);
  } else if (p.kind == kind::kLanding) {
    step_landing(in, out);
  } else if (p.kind == kind::kChute) {
    step_chute(in, out);
  } else {
    step_coast(in, out);
  }
  if (p.kind == kind::kHold || p.kind == kind::kDone) {
    out.groups = 0U;
    out.throttle = 0.0F;
  }
  mass_ = dm::max_(mass_ - (flow_ * kDt), 1.0);
  ++in_phase_;
  slew_reference(p);
  out.q_ref = q_ref_;
  out.w_ref = w_ref_;
  return out;
}

// The reference turns toward the target at the phase's rate, so a flip is a manoeuvre and a change of phase is not a step.
inline void Mission::slew_reference(const Phase& p) noexcept {
  const Quat prev = q_ref_;
  const Vec3 e = att::error_vector(q_ref_, q_target_);
  const double angle = dm::norm(e);
  const double step_max = static_cast<double>(p.slew_dps) * kDt * dm::kDegToRad;
  if (angle <= step_max) {
    q_ref_ = q_target_;
  } else {
    q_ref_ = dm::normalized(q_ref_ * detail::about(e / angle, step_max));
  }
  w_ref_ = att::reference_rate(prev, q_ref_, kDt);
}

}  // namespace tfc::gnc
