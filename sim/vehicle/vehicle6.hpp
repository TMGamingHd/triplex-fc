// SPDX-License-Identifier: MIT
// A 6-DOF launch-vehicle model for the simulator (host only, double precision). See docs/VEHICLE_SIM.md.
//
// Frames. Inertial: non-rotating, centred on a spherical Earth, with the launch point on +X (local vertical), downrange +Y, crossrange +Z (right-handed).
// Body: X along the long axis (nose forward), Y and Z lateral; at lift-off the body axes coincide with the inertial ones (the vehicle stands on the pad).
// Sensor/platform frame: (X, Y, Z) = (body Y, body Z, body X): Z is the long axis, so an accelerometer at rest reads +1 g on Z, as on the rig.
//
// Gimbal convention (chosen so that a positive command gives a positive angular acceleration of the platform tilt it is named for):
//   pitch plane: positive command tilts the nose toward downrange (+Y inertial at lift-off); yaw plane: positive command tilts the nose toward -Z
//   (the platform's tilt about X is positive toward -crossrange).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "atmosphere.hpp"
#include "math3.hpp"

namespace sim {

struct Params {
  // masses
  double m_dry = 6000.0;        // kg
  double m_prop0 = 24000.0;     // kg
  // geometry: x is measured from the aft end, forward positive
  double length = 20.0;         // m
  double diameter = 1.8;        // m
  double x_dry_cg = 10.0;       // m
  double x_tank_bottom = 2.0;   // m
  double prop_density = 900.0;  // kg/m^3
  double x_cp = 12.5;           // centre of pressure, m (ahead of the CG: the vehicle is aerodynamically unstable)
  // propulsion: `engines` identical engines, the first on the axis, the others at `engine_offset` from it
  int engines = 5;
  double thrust_vac_each = 92000.0;  // N
  double exit_area_each = 0.12;      // m^2
  double isp_vac = 310.0;            // s
  double engine_offset = 0.5;        // m
  // thrust-vector control
  double gimbal_limit_deg = 8.0;
  double gimbal_rate_dps = 60.0;
  // aerodynamics
  double c_n_alpha = 2.5;  // normal-force slope per radian, on the cross-section area
  // An ideal roll controller: the roll rate about the long axis is held at zero and roll torques are ignored. The vehicle has some roll control (the
  // flight computers' two-plane gimbal does not provide it), which is not modelled; without it an engine-out's roll torque, acting on an inertia 40 times
  // smaller than the pitch inertia, would turn the vehicle's gimbal planes away from the pad's and end the flight for a reason that is not the subject.
  bool ideal_roll_control = true;
  // tests only: 0 switches gravity off, so that the rocket equation can be checked exactly
  double gravity_scale = 1.0;
  // the largest integration substep, s (tests lower it to check that the answer has converged)
  double max_substep = 0.002;
};

struct Gust {  // a 1-cosine gust of peak velocity `peak` (m/s, inertial frame) lasting `duration` seconds from `t0`
  double t0 = 0.0;
  double duration = 2.0;
  V3 peak{};
};

struct Scenario {
  std::vector<Gust> gusts;
  V3 wind_dir{0.0, 0.0, 1.0};   // direction of the mean wind (inertial), crossrange by default
  double wind_scale = 1.0;      // multiplies the mean wind profile
  double engine_out_time = -1.0;  // seconds; negative: never
  int engine_out_index = 1;       // which engine fails (0 is on the axis)
  double dry_cg_shift = 0.0;      // m, a mass offset: moves the dry centre of gravity forward (+) or aft (-)
};

struct State {
  V3 r{kEarthR, 0.0, 0.0};
  V3 v{};
  Q4 q{};
  V3 w{};
  double m = 30000.0;
};

struct MassProps {
  double mass = 0.0;
  double x_cg = 0.0;  // from the aft end
  double i_t = 0.0;   // transverse inertia about the CG
  double i_x = 0.0;   // roll inertia
};

struct Tilts {  // the vehicle's long axis relative to the pad vertical, as the platform shows it
  double x_deg = 0.0;  // about the platform's X axis (the yaw plane)
  double y_deg = 0.0;  // about Y (the pitch plane)
};

struct Loads {
  V3 f_thrust{};  // body frame
  V3 f_aero{};
  V3 m_thrust{};  // about the CG, body frame
  V3 m_aero{};
  double mdot = 0.0;  // kg/s, positive while burning
  double dynamic_pressure = 0.0;
  double mach = 0.0;
  double alpha = 0.0;  // total angle of attack, rad
  double thrust = 0.0;  // N, total
};

class Vehicle6 {
 public:
  explicit Vehicle6(const Params& p = Params{}, const Scenario& sc = Scenario{}) : p_(p), sc_(sc) {
    s_.m = p.m_dry + p.m_prop0;
    engine_on_.assign(static_cast<std::size_t>(p.engines), true);
  }

  // Advance by dt seconds with the commanded gimbal angles (degrees). Substeps of at most `max_substep` (2 ms).
  void step(double dt, double cmd_pitch_deg, double cmd_yaw_deg) {
    const int n = std::max(1, static_cast<int>(std::ceil((dt / p_.max_substep) - 1e-9)));
    const double h = dt / n;
    for (int i = 0; i < n; ++i) {
      if (sc_.engine_out_time >= 0.0 && t_ >= sc_.engine_out_time && sc_.engine_out_index >= 0 &&
          sc_.engine_out_index < static_cast<int>(engine_on_.size())) {
        engine_on_[static_cast<std::size_t>(sc_.engine_out_index)] = false;
      }
      gimbal_p_ = slew(gimbal_p_, cmd_pitch_deg, h);
      gimbal_y_ = slew(gimbal_y_, cmd_yaw_deg, h);
      rk4(h);
      t_ += h;
    }
  }

  [[nodiscard]] double time() const { return t_; }
  [[nodiscard]] const State& state() const { return s_; }
  [[nodiscard]] double altitude() const { return norm(s_.r) - kEarthR; }
  [[nodiscard]] double speed() const { return norm(s_.v); }
  [[nodiscard]] double mass() const { return s_.m; }
  [[nodiscard]] double gimbal_pitch_deg() const { return gimbal_p_; }
  [[nodiscard]] double gimbal_yaw_deg() const { return gimbal_y_; }
  [[nodiscard]] const Params& params() const { return p_; }
  [[nodiscard]] bool burning() const { return s_.m > p_.m_dry + 1e-6; }
  [[nodiscard]] int engines_on() const {
    int n = 0;
    for (const bool on : engine_on_) {
      n += on ? 1 : 0;
    }
    return n;
  }

  [[nodiscard]] MassProps mass_props(double m) const {
    MassProps mp;
    mp.mass = m;
    const double mprop = std::clamp(m - p_.m_dry, 0.0, p_.m_prop0);
    const double r = 0.5 * p_.diameter;
    const double area = kPi * r * r;
    const double hp = mprop / (p_.prop_density * area);
    const double xp = p_.x_tank_bottom + (0.5 * hp);
    const double xd = p_.x_dry_cg + sc_.dry_cg_shift;
    mp.x_cg = ((p_.m_dry * xd) + (mprop * xp)) / (p_.m_dry + mprop);
    const double i_dry = p_.m_dry * p_.length * p_.length / 12.0 * 0.6;
    const double i_prop = mprop * ((3.0 * r * r) + (hp * hp)) / 12.0;
    mp.i_t = i_dry + (p_.m_dry * (xd - mp.x_cg) * (xd - mp.x_cg)) + i_prop + (mprop * (xp - mp.x_cg) * (xp - mp.x_cg));
    mp.i_x = (p_.m_dry * r * r) + (0.5 * mprop * r * r);
    return mp;
  }

  // The forces and moments on a state at time t (the gimbal angles are the vehicle's current ones).
  [[nodiscard]] Loads loads(const State& s, double t) const {
    Loads l;
    const double alt = norm(s.r) - kEarthR;
    const Air air = air_at(alt);
    const MassProps mp = mass_props(s.m);
    const V3 vrel = rotate_inv(s.q, s.v - wind_at(alt, t));
    const double v_abs = norm(vrel);
    l.mach = v_abs / air.sound;
    l.dynamic_pressure = 0.5 * air.density * v_abs * v_abs;
    const double area = kPi * 0.25 * p_.diameter * p_.diameter;
    if (v_abs > 1.0 && vrel.x > 0.0) {
      const double lat = std::hypot(vrel.y, vrel.z);
      l.alpha = std::atan2(lat, vrel.x);
      l.f_aero.x = -l.dynamic_pressure * area * axial_coefficient(l.mach);
      if (lat > 1e-9) {
        const double fn = -l.dynamic_pressure * area * p_.c_n_alpha * l.alpha;
        l.f_aero.y = fn * vrel.y / lat;
        l.f_aero.z = fn * vrel.z / lat;
      }
      l.m_aero = cross(V3{p_.x_cp - mp.x_cg, 0.0, 0.0}, V3{0.0, l.f_aero.y, l.f_aero.z});
    }
    if (burning()) {
      const double dp = gimbal_p_ * kDeg2Rad;
      const double dy = gimbal_y_ * kDeg2Rad;
      const V3 dir{std::cos(dp) * std::cos(dy), -std::sin(dp) * std::cos(dy), std::sin(dy)};
      for (std::size_t i = 0; i < engine_on_.size(); ++i) {
        if (!engine_on_[i]) {
          continue;
        }
        const double ti = std::max(0.0, p_.thrust_vac_each - (air.pressure * p_.exit_area_each));
        const V3 f = dir * ti;
        l.f_thrust = l.f_thrust + f;
        l.m_thrust = l.m_thrust + cross(V3{-mp.x_cg, engine_y(i), engine_z(i)}, f);
        l.mdot += p_.thrust_vac_each / (p_.isp_vac * kG0);
        l.thrust += ti;
      }
    }
    return l;
  }

  // The platform tilts that show the vehicle's long axis (about X: the yaw plane, positive toward -crossrange; about Y: the pitch plane,
  // positive toward downrange), from the pad vertical.
  [[nodiscard]] Tilts tilts() const {
    const V3 u = rotate(s_.q, V3{1.0, 0.0, 0.0});
    Tilts t;
    t.y_deg = std::atan2(u.y, u.x) * kRad2Deg;
    t.x_deg = -std::asin(std::clamp(u.z, -1.0, 1.0)) * kRad2Deg;
    return t;
  }

  // What IMUs fixed to the vehicle would feel, in the sensor frame (X, Y, Z) = (body Y, body Z, body X): body rates in dps and specific force in g.
  void vehicle_true_imu(V3& gyro_dps, V3& accel_g) const {
    const Loads l = loads(s_, t_);
    const V3 f = (l.f_thrust + l.f_aero) / s_.m / kG0;
    gyro_dps = V3{s_.w.y, s_.w.z, s_.w.x} * kRad2Deg;
    accel_g = V3{f.y, f.z, f.x};
  }

  [[nodiscard]] Loads current_loads() const { return loads(s_, t_); }

  // The mean wind plus gusts, inertial frame, m/s.
  [[nodiscard]] V3 wind_at(double altitude, double t) const {
    V3 w = normalized(sc_.wind_dir) * (sc_.wind_scale * mean_wind_speed(altitude));
    for (const Gust& g : sc_.gusts) {
      if (t >= g.t0 && t <= g.t0 + g.duration && g.duration > 0.0) {
        w = w + (g.peak * (0.5 * (1.0 - std::cos(2.0 * kPi * (t - g.t0) / g.duration))));
      }
    }
    return w;
  }

  // Replace the whole state (for tests and for starting an orbit or a coast).
  void set_state(const State& s) { s_ = s; }

 private:
  static double mean_wind_speed(double altitude) {
    struct Pt {
      double h;
      double v;
    };
    static constexpr std::array<Pt, 9> kProfile{{{0.0, 0.0}, {1000.0, 5.0}, {3000.0, 10.0}, {8000.0, 18.0}, {12000.0, 28.0},
                                                 {16000.0, 20.0}, {25000.0, 10.0}, {40000.0, 5.0}, {60000.0, 0.0}}};
    if (altitude <= kProfile.front().h) {
      return kProfile.front().v;
    }
    for (std::size_t i = 1; i < kProfile.size(); ++i) {
      if (altitude <= kProfile[i].h) {
        const double f = (altitude - kProfile[i - 1].h) / (kProfile[i].h - kProfile[i - 1].h);
        return kProfile[i - 1].v + (f * (kProfile[i].v - kProfile[i - 1].v));
      }
    }
    return 0.0;
  }

  static double axial_coefficient(double mach) {
    const double d = (mach - 1.1) / 0.35;
    return 0.30 + (0.45 * std::exp(-d * d));  // a transonic rise
  }

  [[nodiscard]] double engine_y(std::size_t i) const { return i == 1 ? p_.engine_offset : (i == 2 ? -p_.engine_offset : 0.0); }
  [[nodiscard]] double engine_z(std::size_t i) const { return i == 3 ? p_.engine_offset : (i == 4 ? -p_.engine_offset : 0.0); }

  [[nodiscard]] double slew(double cur, double cmd, double h) const {
    const double target = std::clamp(cmd, -p_.gimbal_limit_deg, p_.gimbal_limit_deg);
    const double d = std::clamp(target - cur, -p_.gimbal_rate_dps * h, p_.gimbal_rate_dps * h);
    return cur + d;
  }

  struct Deriv {
    V3 r{};
    V3 v{};
    Q4 q{0.0, 0.0, 0.0, 0.0};
    V3 w{};
    double m = 0.0;
  };

  [[nodiscard]] Deriv deriv(const State& s, double t) const {
    Deriv d;
    const Loads l = loads(s, t);
    const V3 f_inertial = rotate(s.q, l.f_thrust + l.f_aero);
    const double rn = norm(s.r);
    d.r = s.v;
    d.v = (f_inertial / s.m) + (s.r * (-p_.gravity_scale * kEarthMu / (rn * rn * rn)));
    d.q = q_dot(s.q, s.w);
    const MassProps mp = mass_props(s.m);
    const MassProps mp2 = mass_props(s.m + 1.0);
    const double dit = (mp2.i_t - mp.i_t) * -l.mdot;  // dI/dt = dI/dm * dm/dt, with dm/dt = -mdot
    const double dix = (mp2.i_x - mp.i_x) * -l.mdot;
    const V3 iw{mp.i_x * s.w.x, mp.i_t * s.w.y, mp.i_t * s.w.z};
    const V3 gyro = cross(s.w, iw);
    const V3 m_total = l.m_thrust + l.m_aero;
    d.w = V3{(m_total.x - gyro.x - (dix * s.w.x)) / mp.i_x, (m_total.y - gyro.y - (dit * s.w.y)) / mp.i_t,
             (m_total.z - gyro.z - (dit * s.w.z)) / mp.i_t};
    if (p_.ideal_roll_control) {
      d.w.x = 0.0;
    }
    d.m = -l.mdot;
    return d;
  }

  [[nodiscard]] static State advanced(const State& s, const Deriv& d, double h) {
    State o;
    o.r = s.r + (d.r * h);
    o.v = s.v + (d.v * h);
    o.q = s.q + (d.q * h);
    o.w = s.w + (d.w * h);
    o.m = s.m + (d.m * h);
    return o;
  }

  void rk4(double h) {
    const Deriv k1 = deriv(s_, t_);
    const Deriv k2 = deriv(advanced(s_, k1, 0.5 * h), t_ + (0.5 * h));
    const Deriv k3 = deriv(advanced(s_, k2, 0.5 * h), t_ + (0.5 * h));
    const Deriv k4 = deriv(advanced(s_, k3, h), t_ + h);
    Deriv k;
    k.r = (k1.r + (k2.r * 2.0) + (k3.r * 2.0) + k4.r) / 6.0;
    k.v = (k1.v + (k2.v * 2.0) + (k3.v * 2.0) + k4.v) / 6.0;
    k.q = (k1.q + (k2.q * 2.0) + (k3.q * 2.0) + k4.q) * (1.0 / 6.0);
    k.w = (k1.w + (k2.w * 2.0) + (k3.w * 2.0) + k4.w) / 6.0;
    k.m = (k1.m + (2.0 * k2.m) + (2.0 * k3.m) + k4.m) / 6.0;
    s_ = advanced(s_, k, h);
    s_.q = normalized(s_.q);
    if (p_.ideal_roll_control) {
      s_.w.x = 0.0;
    }
    s_.m = std::max(s_.m, p_.m_dry);  // the propellant cannot go below empty
  }

  Params p_;
  Scenario sc_;
  State s_{};
  double t_ = 0.0;
  double gimbal_p_ = 0.0;
  double gimbal_y_ = 0.0;
  std::vector<bool> engine_on_;
};

}  // namespace sim
