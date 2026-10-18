// SPDX-License-Identifier: MIT
// The boundary layer on the skin: friction and heating from the inviscid solution's surface state (docs/design/CFD.md section 13). In SI units, host only.
//
// An Euler solution has no boundary layer. What it gives at the wall (the pressure, the temperature and the speed of the air just outside the layer, which is the layer's *edge*) is what a boundary-layer
// calculation needs; the layer's own friction and heating are computed from it by the **reference-temperature method** of Eckert on the flat-plate laws: the incompressible friction law is used at the
// density and the viscosity of the air at a temperature between the edge's, the wall's and the recovery temperature (the temperature an insulated wall reaches), and the Reynolds analogy turns friction
// into heat. The laminar law is Blasius's, 0.664 / sqrt(Re); the turbulent one is Schultz-Grunow's local law, 0.370 / log10(Re)^2.584; a laminar layer turns turbulent over a stretch after the
// Reynolds number (on the length from the stagnation line) has passed a transition value. On a body of revolution a laminar layer's length is Mangler's: the integral of r^2 ds over r^2 (a cone: a third
// of the length, so that its friction is sqrt(3) times a flat plate's).
// It is an integral method on a flat-plate law: right to a few percent on a slender body at a small angle of attack (checked against the exact laminar similarity solution and Van Driest's turbulent
// formula in gasdyn.hpp), and an estimate wherever the flow is not a developing boundary layer along a streamline: the blunt nose, the crossflow of a body at a large angle, separation, shock-boundary-layer
// interaction. The tool says which panels it is not good for.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace sim::cfd::visc {

constexpr double kGasConstant = 287.053;   // J / kg K, air
constexpr double kPrandtl = 0.71;

// Sutherland's law for the viscosity of air, Pa s.
inline double sutherland(double t) { return 1.458e-6 * std::pow(t, 1.5) / (t + 110.4); }

// What is outside the boundary layer.
struct Edge {
  double rho = 1.0;     // kg/m^3
  double p = 1.0;       // Pa
  double t = 288.0;     // K
  double u = 0.0;       // m/s along the wall
  double gamma = 1.4;   // the effective ratio of specific heats there
};

struct Layer {
  double cf = 0.0;          // wall shear over the edge's dynamic pressure, 1/2 rho_e u_e^2
  double stanton = 0.0;     // heat transfer over rho_e u_e (h_aw - h_w)
  double t_aw = 0.0;        // the recovery temperature, K
  double turbulence = 0.0;  // 0 laminar, 1 turbulent (the intermittency between)
  double re_x = 0.0;        // the Reynolds number on the length, at the edge's state
};

inline double recovery_factor(bool turbulent) { return turbulent ? std::cbrt(kPrandtl) : std::sqrt(kPrandtl); }

// The friction and the heat transfer at a station at distance `x` (the effective length, m) along the wall, with the wall at `t_wall` (K). `law` is the viscosity law (Sutherland by default); the
// transition happens at the edge Reynolds number `re_transition` and completes by three times it.
template <class Law>
inline Layer local_layer(const Edge& e, double x, double t_wall, double re_transition, const Law& law) {
  Layer out;
  const double mach2 = e.u * e.u / (e.gamma * kGasConstant * e.t);
  const double rise = 0.5 * (e.gamma - 1.0) * mach2;   // (gamma - 1) M^2 / 2
  auto at = [&](bool turbulent, double& cf, double& st, double& taw) {
    const double r = recovery_factor(turbulent);
    taw = e.t * (1.0 + (r * rise));
    // Eckert's reference temperature (laminar) and Meador and Smart's (turbulent)
    const double t_ref = turbulent ? e.t * (0.5 + (0.5 * t_wall / e.t) + (0.16 * r * rise)) : e.t * (0.5 + (0.5 * t_wall / e.t) + (0.22 * r * rise / r));
    const double rho_ref = e.p / (kGasConstant * t_ref);
    const double re_ref = rho_ref * e.u * x / law(t_ref);
    const double cf_ref = turbulent ? 0.370 / std::pow(std::log10(std::max(re_ref, 1e3)), 2.584) : 0.664 / std::sqrt(std::max(re_ref, 1.0));
    const double scale = rho_ref / e.rho;   // from the reference state's dynamic pressure to the edge's
    cf = cf_ref * scale;
    st = 0.5 * cf_ref * std::pow(kPrandtl, -2.0 / 3.0) * scale;
  };
  double cf_l = 0.0;
  double st_l = 0.0;
  double taw_l = 0.0;
  at(false, cf_l, st_l, taw_l);
  double cf_t = 0.0;
  double st_t = 0.0;
  double taw_t = 0.0;
  at(true, cf_t, st_t, taw_t);
  out.re_x = e.rho * e.u * x / law(e.t);
  const double g = std::clamp((out.re_x - re_transition) / (2.0 * re_transition), 0.0, 1.0);
  const double s = g * g * (3.0 - (2.0 * g));
  out.turbulence = s;
  out.cf = ((1.0 - s) * cf_l) + (s * cf_t);
  out.stanton = ((1.0 - s) * st_l) + (s * st_t);
  out.t_aw = ((1.0 - s) * taw_l) + (s * taw_t);
  return out;
}
inline Layer local_layer(const Edge& e, double x, double t_wall, double re_transition) { return local_layer(e, x, t_wall, re_transition, [](double t) { return sutherland(t); }); }

// The heat flux into the wall, W/m^2, for a perfect-gas enthalpy (c_p = gamma R / (gamma - 1)).
inline double heat_flux(const Edge& e, const Layer& l, double t_wall) {
  const double cp = e.gamma * kGasConstant / (e.gamma - 1.0);
  return l.stanton * e.rho * e.u * cp * (l.t_aw - t_wall);
}

// The wall temperature a surface settles at when it radiates all the heat it takes in (emissivity `eps`): the fixed point of  eps sigma T^4 = q(T), by a few substitutions.
template <class Q>
inline double radiative_equilibrium(const Q& heat_of_wall_temperature, double eps, double start = 1000.0) {
  constexpr double kSigma = 5.670374419e-8;
  double t = start;
  for (int k = 0; k < 30; ++k) {
    const double q = std::max(heat_of_wall_temperature(t), 0.0);
    const double tn = std::pow(q / (eps * kSigma), 0.25);
    t = 0.5 * (t + tn);
  }
  return t;
}

// Mangler's length for a laminar layer on a body of revolution: at each of the stations `s` (arc length from the stagnation point) of radii `r`, the integral of r^2 ds over r^2.
inline std::vector<double> mangler_length(const std::vector<double>& s, const std::vector<double>& r) {
  std::vector<double> out(s.size(), 0.0);
  double integral = 0.0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (i > 0U) {
      integral += 0.5 * ((r[i] * r[i]) + (r[i - 1U] * r[i - 1U])) * (s[i] - s[i - 1U]);
    }
    out[i] = r[i] > 1e-9 ? integral / (r[i] * r[i]) : s[i] / 3.0;   // at the tip (r = 0) the cone's third
  }
  return out;
}

}  // namespace sim::cfd::visc
