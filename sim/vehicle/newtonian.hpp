// SPDX-License-Identifier: MIT
// Hypersonic aerodynamics of a body of revolution by modified Newtonian impact theory (host only). See docs/design/AERODYNAMICS.md section 8.
//
// At Mach 5 and above the flow does not get out of the way of a body: the pressure on a face that looks into the stream is the stagnation pressure times the square of the cosine of the angle
// between the face's normal and the stream (Newton's "sine-squared law", with Lees' modification for the maximum), and a face that looks away feels nothing. That one rule gives the force and the
// moment on any shape at any angle of attack, which is exactly what a blunt capsule, a stack that flies belly first, or a booster that falls engines first needs, and which the slender-body build-up
// of aero.hpp (a few degrees of angle, a pointed body, flying nose first) cannot give. The body is the one the sections describe (a nose, tubes, transitions, in any shape and order), turned into
// panels (stations along the axis, steps of azimuth); the flat faces at its two ends and at any step in its radius are panels too. Fixed fins are thin plates under the same rule.
//
// The table holds the force and moment coefficients at 37 angles of attack (0 to 180 degrees in steps of 5), computed when the vehicle is built or loses a stage; `at()` interpolates. The panels
// are summed once, not every step.
//
// What it is not: it knows nothing of the boundary layer, of shocks interacting with the body or with each other, of the base pressure (the leeward side is a vacuum: zero), of the heating, of the
// real gas, or of the rarefied flow above 90 km; it is the upper-bound-pressure estimate engineers use before there is data (the answers are within 10 to 20 percent for blunt shapes at Mach 6 and above and
// worse for slender ones at small angles, where the real flow carries more load than the Newtonian one).
//
// Coordinates and signs are those of aero.hpp and vehicle6.hpp: x from the aft end of the first stage, forward positive; the stream arrives from the direction of the vehicle's velocity through the air `u`
// (a unit vector, angle `alpha` from +x), so alpha 0 is flying nose first and alpha 180 degrees tail first. `cx` is the axial force coefficient (positive: a force along -x, drag when flying nose first),
// `cn` the normal force coefficient (positive: a force along -n, against the lateral part of the velocity) and `a0` the moment arm sum about x = 0 (the sum of the normal force's pieces times their place,
// in metres), so that about a centre of gravity at x_cg the arm sum is a0 - x_cg cn: the same quantity as `Slope::moment_slope` for a slender body, so the two models can be blended.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "math3.hpp"
#include "spec.hpp"

namespace sim {

namespace newton {

constexpr double kCpMax = 1.84;        // the stagnation pressure coefficient behind a normal shock at Mach infinity, gamma 1.4 (2.0 is the classical Newtonian value; Lees' modification lowers it)
constexpr int kAngles = 37;            // 0 to 180 degrees in 5 degree steps
constexpr double kAngleStep = kPi / 36.0;
constexpr int kAzimuths = 36;          // panels around the body: 10 degrees each
constexpr int kMaxSegments = 48;       // along one section

// One piece of the outline: the radius of the body at an axial station.
struct Station {
  double x;
  double r;
};

// The radius of a nose at the distance `xi` from its tip (0 at the tip, `len` at the base), by its shape; `rb` is the base radius. Tangent ogive, parabola (K = 1) and ellipse as in Barrowman.
inline double nose_radius(NoseShape shape, double xi, double len, double rb) {
  const double t = std::clamp(xi / len, 0.0, 1.0);
  switch (shape) {
    case NoseShape::Cone:
      return rb * t;
    case NoseShape::TangentOgive: {
      const double rho = ((rb * rb) + (len * len)) / (2.0 * rb);
      return std::sqrt(std::max((rho * rho) - ((len - xi) * (len - xi)), 0.0)) + rb - rho;
    }
    case NoseShape::Parabola:
      return rb * ((2.0 * t) - (t * t));
    case NoseShape::Ellipse:
      return rb * std::sqrt(std::max(1.0 - ((1.0 - t) * (1.0 - t)), 0.0));
  }
  return rb * t;
}

// The outline of one section, aft to fore, at `n` + 1 stations. A nose whose tip has a diameter (d_fore > 0) is a truncated shape: the outline of the full shape between the tip radius and the base.
inline std::vector<Station> outline_of(const SectionSpec& s, int n) {
  std::vector<Station> out;
  out.reserve(static_cast<std::size_t>(n) + 1U);
  for (int i = 0; i <= n; ++i) {
    const double f0 = static_cast<double>(i) / n;  // 0 aft, 1 fore
    // a nose has its steepest slope at the tip, so the stations crowd there (the same curve, sampled where it bends); the other pieces are straight
    const double f = s.kind == SectionKind::Nose ? std::sin(0.5 * kPi * f0) : f0;
    const double x = s.x_start + (f * s.length);
    double r = 0.0;
    if (s.kind == SectionKind::Nose) {
      const double rb = 0.5 * s.d_aft;
      const double rt = 0.5 * s.d_fore;
      if (rt > 0.0) {
        // truncated: the outline of a full nose of length L' = L rb / (rb - rt) (the same curve, the tip cut off where its radius is rt: exact for a cone, an approximation for the curved shapes)
        const double len_full = s.length * rb / (rb - rt);
        const double xi = (len_full - s.length) + ((1.0 - f) * s.length);
        r = nose_radius(s.nose, xi, len_full, rb);
      } else {
        r = nose_radius(s.nose, (1.0 - f) * s.length, s.length, rb);
      }
    } else {
      r = 0.5 * (s.d_aft + (f * (s.d_fore - s.d_aft)));
    }
    out.push_back(Station{x, r});
  }
  return out;
}

}  // namespace newton

class NewtonTable {
 public:
  NewtonTable() = default;

  // Build from the sections (the body) and the fixed fins that are on the vehicle. `s_ref` is the reference area the coefficients are on.
  void build(const std::vector<SectionSpec>& sections, const std::vector<FinPlanform>& fins, double s_ref) {
    ready_ = !sections.empty() && s_ref > 0.0;
    cx_.fill(0.0);
    cn_.fill(0.0);
    a0_.fill(0.0);
    if (!ready_) {
      return;
    }
    s_ref_ = s_ref;
    std::vector<Panel> panels;
    add_body(sections, panels);
    add_fins(sections, fins, panels);
    for (int k = 0; k < newton::kAngles; ++k) {
      const double alpha = k * newton::kAngleStep;
      const double ux = std::cos(alpha);
      const double uy = std::sin(alpha);
      double cx = 0.0;
      double cn = 0.0;
      double a0 = 0.0;
      for (const Panel& p : panels) {
        const double mu = (p.nx * ux) + (p.ny * uy);   // the cosine of the angle between the face's normal and the stream
        if (mu <= 0.0) {
          continue;  // looking away from the stream: in the lee, no pressure
        }
        const double w = newton::kCpMax * mu * mu * p.area / s_ref_;
        cx += w * p.nx;
        cn += w * p.ny;
        a0 += w * ((p.x * p.ny) - (p.y * p.nx));
      }
      cx_[static_cast<std::size_t>(k)] = cx;
      cn_[static_cast<std::size_t>(k)] = cn;
      a0_[static_cast<std::size_t>(k)] = a0;
    }
  }

  [[nodiscard]] bool ready() const { return ready_; }

  // The coefficients at the angle of attack `alpha` (rad; folded into 0 to pi), interpolated linearly between the tabulated angles.
  void at(double alpha, double& cx, double& cn, double& a0) const {
    const double a = std::clamp(std::fabs(alpha), 0.0, kPi) / newton::kAngleStep;
    const int i = std::min(static_cast<int>(a), newton::kAngles - 2);
    const double f = a - i;
    const std::size_t k = static_cast<std::size_t>(i);
    cx = cx_[k] + (f * (cx_[k + 1U] - cx_[k]));
    cn = cn_[k] + (f * (cn_[k + 1U] - cn_[k]));
    a0 = a0_[k] + (f * (a0_[k + 1U] - a0_[k]));
  }

  // The tabulated values themselves, for the tests.
  [[nodiscard]] double cx_at(int k) const { return cx_[static_cast<std::size_t>(k)]; }
  [[nodiscard]] double cn_at(int k) const { return cn_[static_cast<std::size_t>(k)]; }
  [[nodiscard]] double a0_at(int k) const { return a0_[static_cast<std::size_t>(k)]; }

 private:
  // A flat piece of the surface: its outward unit normal in the (x, y) plane of the flow (the part along the other lateral axis cancels over the body, as the panels come in mirror pairs), its
  // area, and where it is (x, and y = the lateral place in the plane of the flow).
  struct Panel {
    double nx;
    double ny;
    double area;
    double x;
    double y;
  };

  static void add_body(const std::vector<SectionSpec>& sections, std::vector<Panel>& panels) {
    const double dphi = 2.0 * kPi / newton::kAzimuths;
    const auto face = [&](double x, double r_lo, double r_hi, double nx_sign) {  // a flat annulus (or disc if r_lo = 0) facing +x or -x
      if (!(r_hi > r_lo)) {
        return;
      }
      const double area_ring = 0.5 * (r_hi * r_hi - r_lo * r_lo) * dphi;
      const double r_mid = (2.0 / 3.0) * ((r_hi * r_hi * r_hi) - (r_lo * r_lo * r_lo)) / ((r_hi * r_hi) - (r_lo * r_lo));
      for (int j = 0; j < newton::kAzimuths; ++j) {
        const double phi = (j + 0.5) * dphi;
        panels.push_back(Panel{nx_sign, 0.0, area_ring, x, r_mid * std::cos(phi)});
      }
    };
    double x_end = 0.0;
    double r_end = 0.0;
    bool first = true;
    for (const SectionSpec& s : sections) {
      const int n = std::clamp(static_cast<int>(std::ceil(s.length / 0.25)), s.kind == SectionKind::Nose ? 24 : 4, newton::kMaxSegments);
      const std::vector<newton::Station> st = newton::outline_of(s, n);
      if (first) {
        face(st.front().x, 0.0, st.front().r, -1.0);  // the aft end: a flat base facing aft
        first = false;
      } else if (std::fabs(st.front().r - r_end) > 1e-9) {  // a step in the radius where two sections meet
        face(st.front().x, std::min(r_end, st.front().r), std::max(r_end, st.front().r), st.front().r < r_end ? 1.0 : -1.0);
      }
      for (std::size_t i = 0; i + 1U < st.size(); ++i) {
        const double dx = st[i + 1U].x - st[i].x;
        const double dr = st[i + 1U].r - st[i].r;
        const double slant = std::hypot(dx, dr);
        if (!(slant > 0.0)) {
          continue;
        }
        const double rm = 0.5 * (st[i].r + st[i + 1U].r);
        const double xm = 0.5 * (st[i].x + st[i + 1U].x);
        const double nxm = -dr / slant;      // the outward normal of the surface of revolution: (-dr, dx cos(phi), dx sin(phi)) / slant
        const double nrm = dx / slant;
        const double area = rm * slant * dphi;
        for (int j = 0; j < newton::kAzimuths; ++j) {
          const double phi = (j + 0.5) * dphi;
          panels.push_back(Panel{nxm, nrm * std::cos(phi), area, xm, rm * std::cos(phi)});
        }
      }
      x_end = st.back().x;
      r_end = st.back().r;
    }
    face(x_end, 0.0, r_end, 1.0);  // the forward end: a flat face (nothing for a pointed nose)
  }

  // Fixed fins: thin flat plates standing out of the body at equal angles in azimuth, from the first fin plane at the flow's own plane. One windward side of each is a panel; the flow sees it at the
  // angle between the stream and the plate's plane.
  static void add_fins(const std::vector<SectionSpec>& sections, const std::vector<FinPlanform>& fins, std::vector<Panel>& panels) {
    (void)sections;
    for (const FinPlanform& f : fins) {
      if (f.count < 1 || !(f.span > 0.0) || !(f.root_chord > 0.0)) {
        continue;
      }
      const double area = 0.5 * (f.root_chord + f.tip_chord) * f.span;
      const double xm = f.x_le_root - (0.5 * (f.root_chord + f.tip_chord) * 0.5);      // the middle of the planform, roughly a quarter chord back of the root's leading edge
      for (int k = 0; k < f.count; ++k) {
        const double phi = 2.0 * kPi * k / f.count;
        // the plate's plane holds the axis and the radial direction (cos phi, sin phi); its normals are +-(-sin phi, cos phi) in the (y, z) plane, so in the plane of the flow (y) they are -+sin phi
        const double ty = -std::sin(phi);
        if (std::fabs(ty) < 1e-9) {
          continue;  // edge-on to a flow in the plane: no load from this fin
        }
        const double y_arm = std::cos(phi) * (0.5 * f.span);
        panels.push_back(Panel{0.0, ty, area, xm, y_arm});
        panels.push_back(Panel{0.0, -ty, area, xm, y_arm});
      }
    }
  }

  bool ready_ = false;
  double s_ref_ = 1.0;
  std::array<double, newton::kAngles> cx_{};
  std::array<double, newton::kAngles> cn_{};
  std::array<double, newton::kAngles> a0_{};
};

}  // namespace sim
