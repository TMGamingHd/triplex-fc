// SPDX-License-Identifier: MIT
// Propellant slosh: the first antisymmetric mode of a liquid in an upright circular cylinder, as a sloshing mass on a spring (host only). See docs/design/DYNAMICS.md.
//
// The mass, the frequency and the place are the first-mode results of the linear potential theory of a cylinder (Abramson; the pendulum form of Dodge), written from memory of the usual
// references (a check of the figures against a web source on 7 Oct 2026 agreed on the mass and the pendulum length and place; the original reports were not read): for a liquid of mass m and depth h in a
// cylinder of radius R, with a = 1.84 h / R,
//   m1 = m (R / (2.2 h)) tanh(a)          the mass that sloshes
//   omega^2 = (1.84 g / R) tanh(a)        its frequency, in an acceleration g along the axis
//   l = (R / 1.84) coth(a)                the pendulum length: the mass hangs this far below the pivot
//   d1 = (R / 3.68) / sinh(2 a)           the pivot is this far below the free surface
// The mass is put at the pendulum's bob, kept inside the liquid (for a shallow liquid the pendulum is longer than the liquid is deep, and the model is only a stand-in there), and the force of the
// spring acts there. Baffles damp it (a damping ratio, 0.02 unless the vehicle says otherwise: an assumption); a liquid that is not settled (no axial acceleration) has no spring to speak of
// (the model floors the acceleration at `kSloshMinG`).
#pragma once
#include <algorithm>
#include <cmath>

#include "spec.hpp"

namespace sim {

constexpr double kSloshMinG = 0.05;  // m/s^2: the least axial acceleration the sloshing mass is given a spring for

struct SloshModel {
  double m1 = 0.0;   // the sloshing mass, kg (0: no liquid, nothing sloshes)
  double w2g = 0.0;  // omega^2 / g, per metre (so that omega^2 = w2g x the axial acceleration)
  double x_s = 0.0;  // where the mass sits and the spring acts, m from the aft end of the vehicle
  double h = 0.0;    // the depth of the liquid, m
};

// The model of tank `tk` holding `tm` kilograms of propellant.
inline SloshModel slosh_model(const TankSpec& tk, double tm) {
  SloshModel s;
  if (!(tm > 0.0) || !tk.slosh.enabled) {
    return s;
  }
  const double r = tk.radius;
  const double h = tm / (tk.density * kPi * r * r);
  const double a = 1.84 * h / r;
  const double th = std::tanh(a);
  s.h = h;
  s.m1 = std::min(tm * (r / (2.2 * h)) * th * tk.slosh.mass_scale, 0.95 * tm);
  s.w2g = (1.84 / r) * th * tk.slosh.frequency_scale * tk.slosh.frequency_scale;
  const double pivot = tk.x_bottom + h - ((r / 3.68) / std::sinh(2.0 * a));
  const double bob = pivot - ((r / 1.84) / th);
  s.x_s = std::clamp(bob, tk.x_bottom, tk.x_bottom + h);
  return s;
}

}  // namespace sim
