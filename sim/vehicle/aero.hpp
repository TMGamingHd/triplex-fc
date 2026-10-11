// SPDX-License-Identifier: MIT
// The aerodynamics of a vehicle from its shape (host only). See docs/design/VEHICLE_SPEC.md section 2 and docs/design/AERODYNAMICS.md.
//
// An engineering build-up, not a wind tunnel: the normal force and the centre of pressure from the components (Barrowman's method: the nose, the transitions between diameters, the fins,
// with compressibility from the lift slope of the fins' planform), the axial force from skin friction, the waves of the nose, the fins and the transitions, and the base, and the force at any
// angle of attack from the normal force of a slender body plus the cross-flow term of a cylinder in the flow across it. Each formula is a published, simple one (docs/design/AERODYNAMICS.md says
// which and where it stops being good), and the pieces are tested against hand calculations of the same formulas and against the limits they must have.
//
// Coordinates are the vehicle's: x from the aft end of the first stage, forward positive. Coefficients are on the reference area S = pi d^2 / 4.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "math3.hpp"
#include "spec.hpp"

namespace sim {

namespace aero {

constexpr double kGamma = 1.4;
constexpr double kMinReynolds = 1.0e4;

inline double smoothstep(double lo, double hi, double x) {
  const double t = std::clamp((x - lo) / (hi - lo), 0.0, 1.0);
  return t * t * (3.0 - (2.0 * t));
}

// The distance from the tip of a nose to its centre of pressure as a fraction of its length (Barrowman: 0.666 for a cone, 0.466 for a tangent ogive, 0.5 for a parabola, 0.333 for an ellipse), and the
// relative wave drag of the shape at the same fineness (a cone is 1).
inline double nose_cp_from_tip(NoseShape s) {
  switch (s) {
    case NoseShape::Cone: return 0.666;
    case NoseShape::TangentOgive: return 0.466;
    case NoseShape::Parabola: return 0.5;
    case NoseShape::Ellipse: return 0.333;
  }
  return 0.666;
}
inline double nose_wave_factor(NoseShape s) {
  switch (s) {
    case NoseShape::Cone: return 1.0;
    case NoseShape::TangentOgive: return 0.55;
    case NoseShape::Parabola: return 0.7;
    case NoseShape::Ellipse: return 0.4;
  }
  return 1.0;
}

// The lift-curve slope (per radian) of a fin of aspect ratio `ar` and mid-chord sweep `lambda` (rad) at Mach m: the Helmbold-Polhamus form with the compressibility correction below Mach 0.8, the
// linear supersonic theory above Mach 1.2 (4/beta, corrected for a low aspect ratio), and a straight line between the two through the transonic range (which hides the real transonic peak).
inline double wing_lift_slope(double ar, double lambda, double m) {
  const auto subsonic = [&](double mach) {
    const double beta2 = 1.0 - (mach * mach);
    const double t = std::tan(lambda);
    return 2.0 * kPi * ar / (2.0 + std::sqrt(4.0 + (ar * ar * beta2 * (1.0 + (t * t / beta2)))));
  };
  const auto supersonic = [&](double mach) {
    const double beta = std::sqrt((mach * mach) - 1.0);
    const double arb = ar * beta;
    return arb >= 1.0 ? (4.0 / beta) * (1.0 - (1.0 / (2.0 * arb))) : 0.5 * kPi * ar;
  };
  if (m <= 0.8) {
    return subsonic(m);
  }
  if (m >= 1.2) {
    return supersonic(m);
  }
  const double a = subsonic(0.8);
  const double b = supersonic(1.2);
  return a + ((b - a) * (m - 0.8) / 0.4);
}

// Sutherland's law for the viscosity of air, Pa s.
inline double viscosity(double temperature) { return 1.458e-6 * std::pow(temperature, 1.5) / (temperature + 110.4); }

// The drag coefficient of a circular cylinder in cross-flow (on diameter times length), by the Mach number of the flow across it: 1.2 in low-speed flow, rising through the transonic range to a peak
// near Mach 1.5 to 2 and falling to the Newtonian value, 2/3 of the stagnation-pressure coefficient (1.23), at hypersonic speed. The values are in the range of the published cylinder data (Hoerner,
// Fluid-Dynamic Drag, 1965; Jorgensen, NASA TN D-7228, 1973); they are not fitted to any vehicle.
inline double cross_cd(double mach_n) {
  static const double t[][2] = {{0.0, 1.20}, {0.5, 1.25}, {1.0, 1.45}, {1.5, 1.60}, {2.5, 1.55}, {4.0, 1.45}, {8.0, 1.30}, {25.0, 1.23}};
  const std::size_t n = sizeof(t) / sizeof(t[0]);
  if (mach_n <= t[0][0]) {
    return t[0][1];
  }
  for (std::size_t i = 1; i < n; ++i) {
    if (mach_n <= t[i][0]) {
      return t[i - 1][1] + ((mach_n - t[i - 1][0]) / (t[i][0] - t[i - 1][0]) * (t[i][1] - t[i - 1][1]));
    }
  }
  return t[n - 1][1];
}

}  // namespace aero

// What the body is made of right now (the sections and fins of the stages still on the vehicle), reduced to what the force needs. Rebuilt when a stage leaves.
class AeroGeometry {
 public:
  struct Part {
    double cn_alpha_ref;  // the slope this part contributes at Mach 0 (per radian, on S)
    double x_cp;          // its centre of pressure (m)
    int kind;             // 0 nose or transition (slope independent of Mach), 1 fin set (slope follows the planform's lift slope with Mach)
    double ar;            // fins: the aspect ratio of one fin
    double lambda;        // fins: the mid-chord sweep (rad)
    double cl0;           // fins: the planform's lift slope at Mach 0, the value the ratio is taken against
  };

  AeroGeometry() = default;

  // Build from the sections and fins that are on the vehicle. `d_ref_override` > 0 fixes the reference diameter.
  void build(const std::vector<SectionSpec>& sections, const std::vector<FinPlanform>& fins, double d_ref_override) {
    parts_.clear();
    ready_ = !sections.empty();
    if (!ready_) {
      return;
    }
    double x_aft = sections.front().x_start;
    double x_fore = sections.front().x_start + sections.front().length;
    double d_max = 0.0;
    for (const SectionSpec& s : sections) {
      x_aft = std::min(x_aft, s.x_start);
      x_fore = std::max(x_fore, s.x_start + s.length);
      d_max = std::max({d_max, s.d_aft, s.d_fore});
    }
    x_aft_ = x_aft;
    x_fore_ = x_fore;
    d_ref_ = d_ref_override > 0.0 ? d_ref_override : d_max;
    s_ref_ = kPi * d_ref_ * d_ref_ / 4.0;
    length_ = x_fore - x_aft;
    wetted_ = 0.0;
    plan_area_ = 0.0;
    plan_moment_ = 0.0;
    nose_wave_ = 0.0;
    front_face_area_ = 0.0;
    transition_wave_ = 0.0;
    base_area_ = 0.0;
    for (const SectionSpec& s : sections) {
      const double a_aft = kPi * s.d_aft * s.d_aft / 4.0;
      const double a_fore = kPi * s.d_fore * s.d_fore / 4.0;
      const double slant = std::hypot(s.length, 0.5 * (s.d_aft - s.d_fore));
      wetted_ += kPi * 0.5 * (s.d_aft + s.d_fore) * slant;
      const double plan = 0.5 * (s.d_aft + s.d_fore) * s.length;
      plan_area_ += plan;
      plan_moment_ += plan * (s.x_start + (0.5 * s.length));
      if (s.kind == SectionKind::Nose) {
        parts_.push_back(Part{2.0 * (a_aft - a_fore) / s_ref_, s.x_start + s.length - (aero::nose_cp_from_tip(s.nose) * s.length), 0, 0.0, 0.0, 0.0});
        const double half_angle = std::atan2(0.5 * (s.d_aft - s.d_fore), s.length);
        nose_wave_ += 2.0 * std::pow(std::sin(half_angle), 2.0) * aero::nose_wave_factor(s.nose) * (a_aft - a_fore) / s_ref_;
      } else if (s.kind == SectionKind::Transition && std::fabs(s.d_aft - s.d_fore) > 1e-12) {
        const double ratio = s.d_fore / s.d_aft;  // forward over aft
        const double from_fore = (s.length / 3.0) * (1.0 + ((1.0 - ratio) / (1.0 - (ratio * ratio))));
        parts_.push_back(Part{2.0 * (a_aft - a_fore) / s_ref_, s.x_start + s.length - from_fore, 0, 0.0, 0.0, 0.0});
        const double half_angle = std::atan2(0.5 * std::fabs(s.d_aft - s.d_fore), s.length);
        if (s.d_aft > s.d_fore) {  // a flare (a boat-tail's expansion wave costs less and is left to the base drag)
          transition_wave_ += 2.0 * std::pow(std::sin(half_angle), 2.0) * (a_aft - a_fore) / s_ref_;
        }
      }
      if (std::fabs(s.x_start - x_aft) < 1e-9) {
        base_area_ = a_aft;
      }
      if (std::fabs((s.x_start + s.length) - x_fore) < 1e-9) {
        front_face_area_ += a_fore;  // the forward end of the forward-most piece: a point for a pointed nose, a blunt face for a vehicle that ends in a tube
      }
    }
    // the strips for the cross-flow of a turning body: pieces of at most two metres, each with the diameter of its middle
    strips_.clear();
    for (const SectionSpec& sec : sections) {
      const int n = std::max(1, static_cast<int>(std::ceil(sec.length / 2.0)));
      for (int i = 0; i < n; ++i) {
        const double f = (i + 0.5) / n;
        strips_.push_back(Strip{sec.x_start + (f * sec.length), sec.d_aft + (f * (sec.d_fore - sec.d_aft)), sec.length / n});
      }
    }
    fin_wave_ = 0.0;
    for (const FinPlanform& f : fins) {
      if (f.count < 1 || !(f.span > 0.0) || !(f.root_chord > 0.0)) {
        continue;
      }
      double r = 0.5 * d_ref_;  // the body radius at the fins
      for (const SectionSpec& s : sections) {
        if (f.x_le_root >= s.x_start && f.x_le_root <= s.x_start + s.length) {
          const double t = (f.x_le_root - s.x_start) / std::max(s.length, 1e-12);
          r = 0.5 * (s.d_aft + (t * (s.d_fore - s.d_aft)));
        }
      }
      const double ct = f.tip_chord;
      const double cr = f.root_chord;
      const double mid_len = std::hypot(f.span, f.sweep + (0.5 * (ct - cr)));  // the mid-chord line
      const double k_fb = 1.0 + (r / (f.span + r));
      const double n = static_cast<double>(f.count);
      const double cn = (k_fb * 4.0 * n * (f.span / d_ref_) * (f.span / d_ref_)) / (1.0 + std::sqrt(1.0 + std::pow((2.0 * mid_len) / (cr + ct), 2.0)));
      const double x_cp = f.x_le_root - ((((f.sweep) / 3.0) * ((cr + (2.0 * ct)) / (cr + ct))) + ((1.0 / 6.0) * (cr + ct - ((cr * ct) / (cr + ct)))));
      const double area_one = 0.5 * (cr + ct) * f.span;
      const double ar = f.span * f.span / area_one;
      const double lambda = std::atan2(f.sweep + (0.5 * (ct - cr)), f.span);
      Part p;
      p.cn_alpha_ref = cn;
      p.x_cp = x_cp;
      p.kind = 1;
      p.ar = ar;
      p.lambda = lambda;
      p.cl0 = aero::wing_lift_slope(ar, lambda, 0.0);
      parts_.push_back(p);
      wetted_ += 2.0 * n * area_one;
      plan_area_ += 0.5 * n * area_one;
      plan_moment_ += 0.5 * n * area_one * (f.x_le_root - (0.5 * cr));
      fin_wave_ += (n / 2.0) * 4.0 * std::pow(f.thickness / std::max(0.5 * (cr + ct), 1e-9), 2.0) * area_one / s_ref_;  // the supersonic wave drag of thin fins, per beta
    }
  }

  [[nodiscard]] bool ready() const { return ready_; }
  [[nodiscard]] double reference_area() const { return s_ref_; }
  [[nodiscard]] double reference_diameter() const { return d_ref_; }
  [[nodiscard]] double length() const { return length_; }

  // The slope of the normal force at small angles (per radian), and the sum of slope times the arm to x_cg (m), at Mach m: the parts add, a fin set follows its lift slope with Mach.
  void slopes(double mach, double x_cg, double& cn_alpha, double& moment_slope) const {
    cn_alpha = 0.0;
    moment_slope = 0.0;
    for (const Part& p : parts_) {
      const double scale = p.kind == 1 && p.cl0 > 0.0 ? aero::wing_lift_slope(p.ar, p.lambda, mach) / p.cl0 : 1.0;
      cn_alpha += p.cn_alpha_ref * scale;
      moment_slope += p.cn_alpha_ref * scale * (p.x_cp - x_cg);
    }
  }

  // The axial force coefficient flying nose first: skin friction, the waves of the nose, the flares and the fins, the nose tip's blunt face, and the base, which a running engine reduces.
  [[nodiscard]] double axial(double mach, double reynolds, double power_fraction, const AeroSpec& a) const {
    const double re = std::max(reynolds, aero::kMinReynolds);
    const double cf = 0.455 / std::pow(std::log10(re), 2.58) * std::pow(1.0 + (0.144 * mach * mach), -0.65) * a.wetted_roughness;
    const double skin = cf * wetted_ / s_ref_;
    const double supersonic = aero::smoothstep(0.85, 1.2, mach);
    const double transonic_peak = 1.0 + (0.6 * std::exp(-std::pow((mach - 1.15) / 0.2, 2.0)));
    const double beta = std::sqrt(std::max(mach * mach - 1.0, 0.04));
    const double wave = supersonic * transonic_peak * (nose_wave_ + transition_wave_) + (supersonic * fin_wave_ / beta);
    const double tip = front_face_area_ / s_ref_ * (0.4 + (0.45 * supersonic));
    const double base_cd = mach < 1.0 ? 0.12 + (0.13 * mach * mach) : 0.25 / mach;
    const double base = base_cd * (base_area_ / s_ref_) * (1.0 - (a.power_on_base * std::clamp(power_fraction, 0.0, 1.0)));
    return skin + wave + tip + base;
  }

  // The skin-friction part of the axial coefficient alone (the Newtonian model supplies the pressure drag).
  [[nodiscard]] double skin(double mach, double reynolds, const AeroSpec& a) const {
    const double re = std::max(reynolds, aero::kMinReynolds);
    const double cf = 0.455 / std::pow(std::log10(re), 2.58) * std::pow(1.0 + (0.144 * mach * mach), -0.65) * a.wetted_roughness;
    return cf * wetted_ / s_ref_;
  }

  // What turning adds to the aerodynamic load (the damping of the body's rotation), as the difference between the load with the velocity of each piece including the body's own rotation and the load
  // with the velocity of the centre of gravity alone: it is zero for a vehicle that does not turn, so the steady load is whatever the other model says it is. `vrel` is the velocity of the centre of
  // gravity through the air (body frame), `w` the body rates (rad/s). Two parts: the slender-body lift of each component, evaluated at the local angle of attack of its centre of pressure, and the
  // cross-flow drag of each strip of the body (Allen and Perkins), at the local lateral velocity. Force in N and moment about the centre of gravity in N m, body frame.
  void rotation_increment(double mach, double sound, double rho, const V3& vrel, const V3& w, double x_cg, const AeroSpec& a, double cn_scale, V3& d_force, V3& d_moment) const {
    d_force = V3{};
    d_moment = V3{};
    const double q = 0.5 * rho;   // (times the square of the speed, below)
    const auto lateral = [&](double x, V3& u_lat, double& speed_lat) {
      const double rx = x - x_cg;
      u_lat = V3{0.0, vrel.y + (w.z * rx), vrel.z - (w.y * rx)};
      speed_lat = std::hypot(u_lat.y, u_lat.z);
    };
    // the slender-body lift of each part: the force -q S cn sin(alpha) cos(alpha) along the lateral direction of the local velocity
    for (const Part& p : parts_) {
      const double scale = p.kind == 1 && p.cl0 > 0.0 ? aero::wing_lift_slope(p.ar, p.lambda, mach) / p.cl0 : 1.0;
      const double cn_i = p.cn_alpha_ref * scale * cn_scale;
      V3 u;
      double lat = 0.0;
      lateral(p.x_cp, u, lat);
      const double v2_loc = (vrel.x * vrel.x) + (lat * lat);
      const double alpha_loc = std::atan2(lat, vrel.x);
      const double f_loc = lat > 1e-9 ? -q * v2_loc * s_ref_ * cn_i * std::sin(alpha_loc) * std::cos(alpha_loc) : 0.0;
      const double lat0 = std::hypot(vrel.y, vrel.z);
      const double v2_0 = (vrel.x * vrel.x) + (lat0 * lat0);
      const double alpha0 = std::atan2(lat0, vrel.x);
      const double f_0 = lat0 > 1e-9 ? -q * v2_0 * s_ref_ * cn_i * std::sin(alpha0) * std::cos(alpha0) : 0.0;
      const V3 f_vec = (lat > 1e-9 ? V3{0.0, u.y / lat, u.z / lat} * f_loc : V3{}) - (lat0 > 1e-9 ? V3{0.0, vrel.y / lat0, vrel.z / lat0} * f_0 : V3{});
      d_force = d_force + f_vec;
      d_moment = d_moment + cross(V3{p.x_cp - x_cg, 0.0, 0.0}, f_vec);
    }
    // the cross-flow drag of the strips
    for (const Strip& st : strips_) {
      V3 u;
      double lat = 0.0;
      lateral(st.x, u, lat);
      const double lat0 = std::hypot(vrel.y, vrel.z);
      const double k = a.crossflow_eta * st.d * st.dx * q;
      const V3 f_loc = u * (-k * lat * aero::cross_cd(lat / sound));
      const V3 f_0 = V3{0.0, vrel.y, vrel.z} * (-k * lat0 * aero::cross_cd(lat0 / sound));
      const V3 f_vec = f_loc - f_0;
      d_force = d_force + f_vec;
      d_moment = d_moment + cross(V3{st.x - x_cg, 0.0, 0.0}, f_vec);
    }
  }

  // The plan area of the body and its fins, and where its centroid is (for the cross-flow force).
  [[nodiscard]] double plan_area() const { return plan_area_; }
  [[nodiscard]] double plan_centroid() const { return plan_area_ > 0.0 ? plan_moment_ / plan_area_ : 0.5 * (x_aft_ + x_fore_); }

 private:
  struct Strip {
    double x;   // m, the middle of the strip
    double d;   // m, its diameter
    double dx;  // m, its length
  };
  std::vector<Part> parts_;
  std::vector<Strip> strips_;
  bool ready_ = false;
  double x_aft_ = 0.0;
  double x_fore_ = 0.0;
  double d_ref_ = 0.0;
  double s_ref_ = 0.0;
  double length_ = 0.0;
  double wetted_ = 0.0;
  double plan_area_ = 0.0;
  double plan_moment_ = 0.0;
  double nose_wave_ = 0.0;
  double front_face_area_ = 0.0;
  double transition_wave_ = 0.0;
  double fin_wave_ = 0.0;
  double base_area_ = 0.0;
};

}  // namespace sim
