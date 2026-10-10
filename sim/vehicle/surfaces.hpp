// SPDX-License-Identifier: MIT
// Aerodynamic control surfaces and parachutes (host only). See docs/design/AERODYNAMICS.md section 9.
//
// A surface is a flat plate (a flap) or a lattice that acts as one (a grid fin) on a hinge. Deflected, it turns the air that meets it and the force it feels, at its own place on the vehicle, is the
// vehicle's control: the ship of a Starship-class stack steers through the atmosphere on its four flaps, the booster on its grid fins, a rocket with canards on those. The force is that of a plate in
// a stream:
//   * a normal force on the face that meets the air: the linear lift of the planform at small angles and the cross-flow of a flat plate at large ones, C_n(alpha) = CNalpha sin(alpha) cos(alpha) + Cd90 sin^2(alpha),
//     with the slope from the planform's aspect ratio and the Mach number (a flap), or from a table by Mach for a lattice (a grid fin: its lift is lower, it stalls early, and its drag rises
//     sharply through Mach 1 where the air chokes in its cells);
//   * an axial (chordwise) force C_a for the friction and thickness of the plate, and for a grid fin the drag of its lattice;
//   * the dynamic pressure at the place of the surface, from the velocity of that point through the air (the vehicle's velocity plus its turning), so a vehicle that rotates feels the damping of its own
//     surfaces; and a factor for the surface being in the lee of the body (a flap on the far side of a body flying belly first sees the wake, not the stream).
// The numbers are design-level estimates in the range the published wind-tunnel data for flat plates and grid fins give (Hoerner's Fluid-Dynamic Drag for the plate; Washington and Miller,
// "Grid fins: a new concept for missile stability and control", AIAA 93-0035, and Theerthamalai and Nagarajan for the lattice's behaviour through the transonic range): they show the right trends
// (the stall, the transonic drag, the loss of lift at high Mach) and are NOT wind-tunnel or CFD data for any vehicle (docs/design/AERODYNAMICS.md section 9 says where it stops being good).
//
// Frames: the vehicle's body frame (x along the axis, nose forward). A surface at azimuth phi sits in the direction r = (0, cos phi, sin phi); its tangent is t = (0, -sin phi, cos phi).
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "aero.hpp"
#include "math3.hpp"
#include "spec.hpp"

namespace sim {

namespace plate {

// Linear interpolation in a table of (x, y) pairs, held beyond its ends.
template <std::size_t N>
inline double table(const double (&t)[N][2], double x) {
  if (x <= t[0][0]) {
    return t[0][1];
  }
  for (std::size_t i = 1; i < N; ++i) {
    if (x <= t[i][0]) {
      return t[i - 1][1] + ((x - t[i - 1][0]) / (t[i][0] - t[i - 1][0]) * (t[i][1] - t[i - 1][1]));
    }
  }
  return t[N - 1][1];
}

// The drag coefficient of a flat plate broadside to the stream, on its area, by Mach number: 1.17 in low-speed flow (a finite plate of aspect ratio near one), rising through the supersonic range to
// the stagnation-pressure value of Newtonian flow at hypersonic speed.
inline double cd90(double mach) {
  static const double t[][2] = {{0.0, 1.17}, {0.8, 1.20}, {1.0, 1.45}, {2.0, 1.70}, {5.0, 1.82}, {10.0, 1.84}};
  return table(t, mach);
}

// Grid fin (a lattice of thin webs in a frame): the slope of its normal force on the frame area per radian of angle between the stream and the cells' axis, and its axial force coefficient. The lift
// drops through the transonic range (the cells choke) and the drag peaks there.
inline double grid_cn_alpha(double mach) {
  static const double t[][2] = {{0.0, 2.9}, {0.6, 2.9}, {0.9, 2.4}, {1.0, 1.5}, {1.15, 1.6}, {1.5, 2.2}, {3.0, 2.0}, {6.0, 1.5}, {12.0, 1.2}};
  return table(t, mach);
}
inline double grid_ca(double mach) {
  static const double t[][2] = {{0.0, 0.35}, {0.7, 0.40}, {0.9, 0.90}, {1.0, 1.30}, {1.15, 1.10}, {1.5, 0.70}, {3.0, 0.50}, {6.0, 0.45}, {12.0, 0.45}};
  return table(t, mach);
}

// A plate's stall: full lift up to 25 degrees, gone by 50 (a grid fin stalls earlier and harder than a solid plate: the stall factor is applied to the linear term only, the cross-flow term keeps its drag).
inline double stall(double alpha_rad) { return 1.0 - aero::smoothstep(25.0 * kPi / 180.0, 50.0 * kPi / 180.0, alpha_rad); }

}  // namespace plate

// The unit vectors of a surface at a deflection: the chord (along the plate, from the hinge to the free edge), the normal, and the span (along the hinge line). A flap hinges on the tangent to the body;
// a grid fin turns on a shaft along the radius and swings its cells' axis from the body's axis toward the tangent.
struct SurfaceFrame {
  V3 chord;
  V3 normal;
  V3 span;
  V3 radial;
};

inline SurfaceFrame surface_frame(const SurfaceSpec& s, double deflection_deg) {
  const double phi = s.azimuth_deg * kDeg2Rad;
  const double d = deflection_deg * kDeg2Rad;
  const V3 x{1.0, 0.0, 0.0};
  const V3 r{0.0, std::cos(phi), std::sin(phi)};
  const V3 t{0.0, -std::sin(phi), std::cos(phi)};
  SurfaceFrame f;
  f.radial = r;
  if (s.kind == SurfaceKind::GridFin) {
    f.chord = (x * std::cos(d)) + (t * std::sin(d));     // the cells' axis
    f.normal = (x * -std::sin(d)) + (t * std::cos(d));   // the web's normal: the lift is sideways
    f.span = r;
  } else {
    f.chord = (x * (s.chord_dir * std::cos(d))) + (r * std::sin(d));
    f.normal = (x * -std::sin(d)) + (r * (s.chord_dir * std::cos(d)));
    f.span = t;
  }
  return f;
}

// Where the surface's load acts: the middle of its chord from the hinge, at its radius from the axis.
inline V3 surface_point(const SurfaceSpec& s, double deflection_deg) {
  const SurfaceFrame f = surface_frame(s, deflection_deg);
  const double phi = s.azimuth_deg * kDeg2Rad;
  const V3 hinge{s.x_hinge, s.radius * std::cos(phi), s.radius * std::sin(phi)};
  return hinge + (f.chord * (0.5 * s.chord));
}

struct SurfaceLoad {
  V3 force{};        // body frame, N
  V3 point{};        // where it acts (body frame, from the aft end of the first stage: x, y, z)
  double alpha = 0.0;  // the plate's angle of attack, rad
  double q = 0.0;      // the dynamic pressure it saw, Pa
};

// The force of a surface at `deflection_deg` in the air. `v_air` is the velocity of the surface's own point through the air, in the body frame (the vehicle's velocity through the air plus its turning
// at that point), `rho` the density, `sound` the speed of sound, `shadow` the fraction of the dynamic pressure that reaches it (1 in the stream, less in the lee of the body).
inline SurfaceLoad surface_load(const SurfaceSpec& s, double deflection_deg, const V3& v_air, double rho, double sound, double shadow) {
  SurfaceLoad out;
  out.point = surface_point(s, deflection_deg);
  const double speed = norm(v_air);
  if (!(speed > 1.0) || !(rho > 0.0) || !(s.area > 0.0)) {
    return out;
  }
  const SurfaceFrame f = surface_frame(s, deflection_deg);
  const double mach = speed / std::max(sound, 1.0);
  const double q = 0.5 * rho * speed * speed * shadow;
  out.q = q;
  const double vn = dot(v_air, f.normal);
  const double vc = dot(v_air, f.chord);
  const double alpha = std::atan2(std::fabs(vn), std::fabs(vc));   // the angle between the stream and the plate's own plane, 0 to 90 degrees
  out.alpha = alpha;
  double cn_alpha = 0.0;
  double ca = 0.0;
  double cross_scale = 1.0;
  if (s.kind == SurfaceKind::GridFin) {
    cn_alpha = plate::grid_cn_alpha(mach);
    ca = plate::grid_ca(mach);
    cross_scale = 0.35;   // the lattice lets most of a cross-flow through
  } else {
    const double ar = s.span * s.span / s.area;
    cn_alpha = aero::wing_lift_slope(ar, s.sweep_deg * kDeg2Rad, mach);
    ca = 0.02;
  }
  const double sa = std::sin(alpha);
  const double ca_ang = std::cos(alpha);
  const double cn = (cn_alpha * sa * ca_ang * plate::stall(alpha)) + (cross_scale * plate::cd90(mach) * sa * sa);
  const double sgn_n = vn >= 0.0 ? 1.0 : -1.0;
  const double sgn_c = vc >= 0.0 ? 1.0 : -1.0;
  out.force = (f.normal * (-sgn_n * q * s.area * cn)) + (f.chord * (-sgn_c * q * s.area * ca * ca_ang));
  return out;
}

// Is the surface in the stream or in the lee of the body? The velocity of the vehicle through the air, `v`, and the surface's outward radial direction decide: a surface on the side the vehicle moves
// toward is in the stream; one on the far side is behind the body. A smooth change between 0.2 and 1 over a band of about 30 degrees around the side-on direction.
inline double stream_fraction(const V3& v, const V3& radial) {
  const double sp = norm(v);
  if (!(sp > 1.0)) {
    return 1.0;
  }
  const double c = dot(v, radial) / sp;   // +1: moving straight at the surface's side
  return 0.2 + (0.8 * aero::smoothstep(-0.25, 0.25, c));
}

// A parachute: the drag area grows from zero as the canopy fills, over `inflation_s`, with the square of the time (the area of an inflating canopy) and holds at `drag_area` once full. The force is
// along the velocity of the attachment through the air, against it.
struct Parachute {
  double drag_area = 0.0;     // Cd A, m^2, when full
  double inflation_s = 3.0;
  double opened_at = -1.0;    // time the canopy was released; < 0: not yet
};

inline double parachute_drag_area(const Parachute& p, double t) {
  if (p.opened_at < 0.0 || !(p.inflation_s > 0.0)) {
    return p.opened_at < 0.0 ? 0.0 : p.drag_area;
  }
  const double f = std::clamp((t - p.opened_at) / p.inflation_s, 0.0, 1.0);
  return p.drag_area * f * f;
}

inline V3 parachute_force(const Parachute& p, double t, const V3& v_air, double rho) {
  const double speed = norm(v_air);
  const double cda = parachute_drag_area(p, t);
  if (!(speed > 0.1) || !(cda > 0.0)) {
    return V3{};
  }
  return v_air * (-0.5 * rho * speed * cda);   // (0.5 rho V^2 CdA) along -v_air/|v_air|
}

// Stagnation-point heat flux by the Sutton-Graves relation, W/m^2: q = k sqrt(rho / Rn) V^3 with k = 1.7415e-4 (SI units, Earth's air), for a blunt nose of radius Rn; and the temperature a surface comes to
// when it radiates all of a flux q away from one side (radiative equilibrium: q = eps sigma T^4).
inline double stagnation_heat_flux(double rho, double speed, double nose_radius) {
  return nose_radius > 0.0 ? 1.7415e-4 * std::sqrt(rho / nose_radius) * speed * speed * speed : 0.0;
}
inline double radiative_equilibrium_k(double flux, double emissivity) {
  constexpr double kSigma = 5.670374419e-8;
  return flux > 0.0 ? std::pow(flux / (emissivity * kSigma), 0.25) : 0.0;
}

}  // namespace sim
