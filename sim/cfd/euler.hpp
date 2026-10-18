// SPDX-License-Identifier: MIT
// The Euler equations of a calorically perfect gas in three dimensions: the states, the numerical flux across a face (HLLC, with the HLL flux of Einfeldt where a strong shock sits on the face),
// the slope limiter, and the far-field state of a boundary (docs/design/CFD.md section 2). Host only, double precision, no heap, no exceptions.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace sim::cfd {

// The primitive state: density, the three velocity components, pressure.
struct Prim {
  double rho = 1.0;
  double u = 0.0;
  double v = 0.0;
  double w = 0.0;
  double p = 1.0;
};

// The conserved state: density, momentum, total energy per volume.
using Cons = std::array<double, 5>;

[[nodiscard]] inline double vel(const Prim& s, int d) { return d == 0 ? s.u : (d == 1 ? s.v : s.w); }
inline void set_vel(Prim& s, int d, double c) { (d == 0 ? s.u : (d == 1 ? s.v : s.w)) = c; }

[[nodiscard]] inline Cons to_cons(const Prim& s, double g) {
  const double ke = 0.5 * s.rho * ((s.u * s.u) + (s.v * s.v) + (s.w * s.w));
  return {s.rho, s.rho * s.u, s.rho * s.v, s.rho * s.w, (s.p / (g - 1.0)) + ke};
}

[[nodiscard]] inline Prim to_prim(const Cons& q, double g) {
  Prim s;
  s.rho = q[0];
  const double inv = 1.0 / q[0];
  s.u = q[1] * inv;
  s.v = q[2] * inv;
  s.w = q[3] * inv;
  s.p = (g - 1.0) * (q[4] - (0.5 * q[0] * ((s.u * s.u) + (s.v * s.v) + (s.w * s.w))));
  return s;
}

[[nodiscard]] inline double sound(const Prim& s, double g) { return std::sqrt(g * s.p / s.rho); }

// The flux of the conserved quantities through a face whose normal is the axis d, from the state on its low side (l) and its high side (r): HLLC with the wave speeds of Einfeldt (Roe-averaged),
// and HLL in its place when the two sides differ in pressure by more than `shock_ratio` (a shock lying along the face: HLLC across it can grow the carbuncle, a spurious bump on a bow shock).
[[nodiscard]] inline Cons flux(const Prim& l, const Prim& r, int d, double g, double shock_ratio = 1.8) {
  const int t1 = (d + 1) % 3;
  const int t2 = (d + 2) % 3;
  const double un_l = vel(l, d);
  const double un_r = vel(r, d);
  const double c_l = sound(l, g);
  const double c_r = sound(r, g);
  const double ke_l = 0.5 * ((l.u * l.u) + (l.v * l.v) + (l.w * l.w));
  const double ke_r = 0.5 * ((r.u * r.u) + (r.v * r.v) + (r.w * r.w));
  const double e_l = (l.p / (g - 1.0)) + (l.rho * ke_l);
  const double e_r = (r.p / (g - 1.0)) + (r.rho * ke_r);
  // Roe averages for the wave speeds
  const double sl = std::sqrt(l.rho);
  const double sr = std::sqrt(r.rho);
  const double inv = 1.0 / (sl + sr);
  const double u_roe = ((sl * un_l) + (sr * un_r)) * inv;
  const double h_l = (e_l + l.p) / l.rho;
  const double h_r = (e_r + r.p) / r.rho;
  const double h_roe = ((sl * h_l) + (sr * h_r)) * inv;
  const double v1_roe = ((sl * vel(l, t1)) + (sr * vel(r, t1))) * inv;
  const double v2_roe = ((sl * vel(l, t2)) + (sr * vel(r, t2))) * inv;
  const double q_roe = 0.5 * ((u_roe * u_roe) + (v1_roe * v1_roe) + (v2_roe * v2_roe));
  const double c_roe = std::sqrt(std::max(1e-30, (g - 1.0) * (h_roe - q_roe)));
  const double s_l = std::min(un_l - c_l, u_roe - c_roe);
  const double s_r = std::max(un_r + c_r, u_roe + c_roe);
  // the physical fluxes of each side
  Cons f_l{l.rho * un_l, l.rho * l.u * un_l, l.rho * l.v * un_l, l.rho * l.w * un_l, (e_l + l.p) * un_l};
  Cons f_r{r.rho * un_r, r.rho * r.u * un_r, r.rho * r.v * un_r, r.rho * r.w * un_r, (e_r + r.p) * un_r};
  f_l[static_cast<std::size_t>(1 + d)] += l.p;
  f_r[static_cast<std::size_t>(1 + d)] += r.p;
  if (s_l >= 0.0) {
    return f_l;
  }
  if (s_r <= 0.0) {
    return f_r;
  }
  const Cons u_l{l.rho, l.rho * l.u, l.rho * l.v, l.rho * l.w, e_l};
  const Cons u_r{r.rho, r.rho * r.u, r.rho * r.v, r.rho * r.w, e_r};
  const double p_hi = std::max(l.p, r.p);
  const double p_lo = std::min(l.p, r.p);
  if (p_hi > shock_ratio * p_lo) {   // HLL
    Cons out{};
    const double k = 1.0 / (s_r - s_l);
    for (std::size_t i = 0; i < 5; ++i) {
      out[i] = ((s_r * f_l[i]) - (s_l * f_r[i]) + (s_l * s_r * (u_r[i] - u_l[i]))) * k;
    }
    return out;
  }
  // HLLC: the speed of the contact and the pressure in the star region
  const double s_star = (r.p - l.p + (l.rho * un_l * (s_l - un_l)) - (r.rho * un_r * (s_r - un_r))) / ((l.rho * (s_l - un_l)) - (r.rho * (s_r - un_r)));
  const bool left = s_star >= 0.0;
  const Prim& k = left ? l : r;
  const Cons& uk = left ? u_l : u_r;
  const Cons& fk = left ? f_l : f_r;
  const double sk = left ? s_l : s_r;
  const double unk = left ? un_l : un_r;
  const double ek = left ? e_l : e_r;
  const double fac = k.rho * (sk - unk) / (sk - s_star);
  Cons star{};
  star[0] = fac;
  star[1] = fac * k.u;
  star[2] = fac * k.v;
  star[3] = fac * k.w;
  star[static_cast<std::size_t>(1 + d)] = fac * s_star;
  star[4] = fac * ((ek / k.rho) + ((s_star - unk) * (s_star + (k.p / (k.rho * (sk - unk))))));
  Cons out{};
  for (std::size_t i = 0; i < 5; ++i) {
    out[i] = fk[i] + (sk * (star[i] - uk[i]));
  }
  return out;
}

// The slope limiters, applied to the two one-sided slopes (the van Albada one is smooth and does not clip an extremum as hard as the minmod does; the minmod is the most diffusive and the safest).
enum class Limiter : int { Minmod = 0, VanAlbada = 1, VanLeer = 2 };

[[nodiscard]] inline double limited_slope(double a, double b, Limiter lim) {
  if (a * b <= 0.0) {
    return 0.0;
  }
  switch (lim) {
    case Limiter::Minmod:
      return std::fabs(a) < std::fabs(b) ? a : b;
    case Limiter::VanLeer:
      return 2.0 * a * b / (a + b);
    case Limiter::VanAlbada:
    default:
      return a * b * (a + b) / ((a * a) + (b * b) + 1e-300);
  }
}

// The state a far-field boundary gives the ghost cell: the free stream, and the inside state where the flow leaves, mixed through the Riemann invariants of the normal direction (n_out is the outward
// unit normal, axis d and sign +1 or -1). For a supersonic inflow it is the free stream, for a supersonic outflow the inside; for a subsonic face the invariant that comes from outside is the free stream's.
[[nodiscard]] inline Prim farfield_state(const Prim& inside, const Prim& inf, int d, double sign, double g) {
  const double ci = sound(inside, g);
  const double cf = sound(inf, g);
  const double un_i = sign * vel(inside, d);   // along the outward normal: positive leaves
  const double un_f = sign * vel(inf, d);
  const double m = un_i / ci;
  if (m >= 1.0) {
    return inside;
  }
  if (m <= -1.0) {
    return inf;
  }
  // the invariants R+ = un + 2c/(g-1) arrives from the inside (it travels with the outward normal), R- = un - 2c/(g-1) from the outside
  const double r_plus = un_i + (2.0 * ci / (g - 1.0));
  const double r_minus = un_f - (2.0 * cf / (g - 1.0));
  const double un_b = 0.5 * (r_plus + r_minus);
  const double c_b = 0.25 * (g - 1.0) * (r_plus - r_minus);
  Prim b;
  const bool outflow = un_b > 0.0;
  const Prim& src = outflow ? inside : inf;   // the entropy and the tangential velocity come from upstream
  const double s = src.p / std::pow(src.rho, g);
  b.rho = std::pow((c_b * c_b) / (g * s), 1.0 / (g - 1.0));
  b.p = b.rho * c_b * c_b / g;
  b.u = src.u;
  b.v = src.v;
  b.w = src.w;
  set_vel(b, d, sign * un_b);
  return b;
}

}  // namespace sim::cfd
