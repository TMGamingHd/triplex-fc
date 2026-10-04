// SPDX-License-Identifier: MIT
// US Standard Atmosphere 1976 up to 86 km (the seven lower layers), with an exponential tail above. Inputs are geometric altitudes in
// metres; the layer formulas use geopotential altitude. Values at the layer bases are the published ones.
#pragma once
#include <array>
#include <cmath>

namespace sim {

constexpr double kG0 = 9.80665;          // standard gravity, m/s^2
constexpr double kRAir = 287.053;        // specific gas constant of dry air, J/(kg K)
constexpr double kEarthR = 6378137.0;    // equatorial radius, m
constexpr double kEarthMu = 3.986004418e14;
constexpr double kGeopotR = 6356766.0;   // the radius the standard atmosphere uses for geopotential altitude

struct Air {
  double temperature = 288.15;  // K
  double pressure = 101325.0;   // Pa
  double density = 1.225;       // kg/m^3
  double sound = 340.294;       // m/s
};

namespace detail {
struct Layer {
  double h_base;  // geopotential altitude of the base, m
  double t_base;  // K
  double p_base;  // Pa
  double lapse;   // K/m
};
constexpr std::array<Layer, 7> kLayers{{{0.0, 288.15, 101325.0, -0.0065},
                                        {11000.0, 216.65, 22632.06, 0.0},
                                        {20000.0, 216.65, 5474.889, 0.0010},
                                        {32000.0, 228.65, 868.0187, 0.0028},
                                        {47000.0, 270.65, 110.9063, 0.0},
                                        {51000.0, 270.65, 66.93887, -0.0028},
                                        {71000.0, 214.65, 3.956420, -0.0020}}};
}  // namespace detail

// The air at a geopotential altitude H (m), 0 <= H <= 84,852 m.
inline Air air_at_geopotential(double h) {
  if (h < 0.0) {
    h = 0.0;
  }
  const detail::Layer* layer = &detail::kLayers[0];
  for (const detail::Layer& l : detail::kLayers) {
    if (h >= l.h_base) {
      layer = &l;
    }
  }
  const double dh = h - layer->h_base;
  Air a;
  if (std::fabs(layer->lapse) > 0.0) {
    a.temperature = layer->t_base + (layer->lapse * dh);
    a.pressure = layer->p_base * std::pow(a.temperature / layer->t_base, -kG0 / (kRAir * layer->lapse));
  } else {
    a.temperature = layer->t_base;
    a.pressure = layer->p_base * std::exp(-kG0 * dh / (kRAir * layer->t_base));
  }
  a.density = a.pressure / (kRAir * a.temperature);
  a.sound = std::sqrt(1.4 * kRAir * a.temperature);
  return a;
}

// The air at a geometric altitude (m). Above 84.852 km geopotential the density and pressure fall off exponentially (scale height 6.5 km): the
// vehicle is out of the atmosphere for every purpose of this simulator by then.
inline Air air_at(double altitude) {
  const double h = kGeopotR * altitude / (kGeopotR + altitude);  // geopotential
  constexpr double kTop = 84852.0;
  if (h <= kTop) {
    return air_at_geopotential(h);
  }
  Air top = air_at_geopotential(kTop);
  const double f = std::exp(-(h - kTop) / 6500.0);
  top.pressure *= f;
  top.density *= f;
  return top;
}

}  // namespace sim
