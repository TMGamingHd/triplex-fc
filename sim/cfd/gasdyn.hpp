// SPDX-License-Identifier: MIT
// Textbook gas dynamics for a calorically perfect gas: the relations the Euler solver (solver.hpp) is checked against, written without any of its code (docs/design/CFD.md section 5).
// Normal and oblique shocks, the pitot (Rayleigh) pressure, the flow over a cone (Taylor-Maccoll), and the exact solution of a Riemann problem (Toro's iteration, for the shock tube).
#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace sim::cfd::gd {

constexpr double kPi = 3.14159265358979323846;

// ---- shocks ----

// Across a normal shock into a gas at Mach m1 (> 1): the ratios of pressure and density, and the Mach number behind it.
struct Normal {
  double p2_p1;
  double rho2_rho1;
  double m2;
};
inline Normal normal_shock(double m1, double g) {
  const double m1s = m1 * m1;
  Normal n{};
  n.p2_p1 = 1.0 + (2.0 * g / (g + 1.0) * (m1s - 1.0));
  n.rho2_rho1 = (g + 1.0) * m1s / (((g - 1.0) * m1s) + 2.0);
  n.m2 = std::sqrt((1.0 + (0.5 * (g - 1.0) * m1s)) / ((g * m1s) - (0.5 * (g - 1.0))));
  return n;
}

// The pressure a pitot tube at rest in the flow at Mach m1 would read, over the static pressure: the total pressure behind the shock for m1 > 1 (Rayleigh), the isentropic stagnation pressure below.
inline double pitot_ratio(double m1, double g) {
  if (m1 <= 1.0) {
    return std::pow(1.0 + (0.5 * (g - 1.0) * m1 * m1), g / (g - 1.0));
  }
  // Rayleigh's pitot formula: p02/p1 = [(g+1)^2 M^2 / (4 g M^2 - 2 (g-1))]^(g/(g-1)) (1 - g + 2 g M^2) / (g + 1)
  const double a = std::pow(((g + 1.0) * (g + 1.0) * m1 * m1) / ((4.0 * g * m1 * m1) - (2.0 * (g - 1.0))), g / (g - 1.0));
  const double b = (1.0 - g + (2.0 * g * m1 * m1)) / (g + 1.0);
  return a * b;
}

// The deflection angle (rad) a shock at angle beta (rad, from the flow) turns the flow through at Mach m1: tan(theta) = 2 cot(beta) (M^2 sin^2 beta - 1) / (M^2 (g + cos 2 beta) + 2).
inline double deflection(double m1, double beta, double g) {
  const double s = std::sin(beta);
  const double num = 2.0 / std::tan(beta) * ((m1 * m1 * s * s) - 1.0);
  const double den = (m1 * m1 * (g + std::cos(2.0 * beta))) + 2.0;
  return std::atan(num / den);
}

// The weak-shock angle beta (rad) of a wedge of half-angle theta in flow at Mach m1; a negative number when the shock detaches (theta above the maximum deflection).
inline double weak_shock_angle(double m1, double theta, double g) {
  const double mu = std::asin(1.0 / m1);   // the Mach angle: a shock of no strength
  // the deflection rises from 0 at the Mach angle to a maximum and falls to 0 at 90 degrees: find the maximum, then bisect the rising side
  double lo = mu;
  double hi = kPi / 2.0;
  for (int k = 0; k < 80; ++k) {   // golden-section search for the maximum
    const double a = lo + (0.382 * (hi - lo));
    const double b = lo + (0.618 * (hi - lo));
    if (deflection(m1, a, g) < deflection(m1, b, g)) {
      lo = a;
    } else {
      hi = b;
    }
  }
  const double bmax = 0.5 * (lo + hi);
  if (deflection(m1, bmax, g) < theta) {
    return -1.0;
  }
  double a = mu;
  double b = bmax;
  for (int k = 0; k < 100; ++k) {
    const double c = 0.5 * (a + b);
    if (deflection(m1, c, g) < theta) {
      a = c;
    } else {
      b = c;
    }
  }
  return 0.5 * (a + b);
}

// Behind an oblique shock at angle beta in flow at Mach m1 turned by theta: the pressure and density ratios and the Mach number.
inline Normal oblique_shock(double m1, double beta, double theta, double g) {
  const double mn1 = m1 * std::sin(beta);
  const Normal n = normal_shock(mn1, g);
  Normal o{};
  o.p2_p1 = n.p2_p1;
  o.rho2_rho1 = n.rho2_rho1;
  o.m2 = n.m2 / std::sin(beta - theta);
  return o;
}

// ---- the flow over a cone (Taylor-Maccoll) ----

// Along rays from the tip the flow is a function of the angle alone. With the speed made non-dimensional on the stagnation speed of sound, vr and vt the radial and tangential components, the equation is
//   vt' = -vr + vr' ... written as a first-order system in (vr, vr') integrated from the shock to the ray where vt = vr' = 0, which is the cone's surface (Anderson, Modern Compressible Flow, section 10.3).
struct Cone {
  double shock_angle = 0.0;   // beta, rad
  double p_surface_p1 = 0.0;  // the pressure on the surface over the free-stream static pressure
  double rho_surface_rho1 = 0.0;
  double mach_surface = 0.0;
  bool attached = false;
};

namespace detail {
// The derivatives of (vr, vt) with respect to the angle theta (vt = dvr/dtheta): the Taylor-Maccoll equation in the form  vr'' = (vt^2 vr - a^2 (2 vr + vt cot theta)) / (a^2 - vt^2),  a^2 = (g-1)/2 (1 - vr^2 - vt^2).
inline std::array<double, 2> tm_rhs(double theta, const std::array<double, 2>& y, double g) {
  const double vr = y[0];
  const double vt = y[1];
  const double a2 = 0.5 * (g - 1.0) * (1.0 - (vr * vr) - (vt * vt));
  const double dvt = ((vt * vt * vr) - (a2 * ((2.0 * vr) + (vt / std::tan(theta))))) / (a2 - (vt * vt));
  return {vt, dvt};
}
}  // namespace detail

// The attached shock on a cone of half-angle theta_c in flow at Mach m1: beta is found so that the integration from the shock reaches vt = 0 on the cone.
inline Cone cone_flow(double m1, double theta_c, double g) {
  Cone best;
  const double mu = std::asin(1.0 / m1);
  // the surface angle reached for a given shock angle: integrate from beta inward (theta decreasing) until vt changes sign
  auto surface = [&](double beta, double& vr_s, double& vt_end) -> double {
    const double mn1 = m1 * std::sin(beta);
    const Normal n = normal_shock(mn1, g);
    const double dflect = std::atan(2.0 / std::tan(beta) * ((mn1 * mn1) - 1.0) / ((m1 * m1 * (g + std::cos(2.0 * beta))) + 2.0));
    const double m2 = n.m2 / std::sin(beta - dflect);
    // the post-shock speed over the stagnation speed of sound (the stagnation enthalpy is the free stream's)
    const double v_nd = std::sqrt(1.0 / ((2.0 / ((g - 1.0) * m2 * m2)) + 1.0));
    std::array<double, 2> y{v_nd * std::cos(beta - dflect), -v_nd * std::sin(beta - dflect)};
    double th = beta;
    const double h = -2e-4;
    while (th > 0.003) {
      const std::array<double, 2> k1 = detail::tm_rhs(th, y, g);
      const std::array<double, 2> y2{y[0] + (0.5 * h * k1[0]), y[1] + (0.5 * h * k1[1])};
      const std::array<double, 2> k2 = detail::tm_rhs(th + (0.5 * h), y2, g);
      const std::array<double, 2> y3{y[0] + (0.5 * h * k2[0]), y[1] + (0.5 * h * k2[1])};
      const std::array<double, 2> k3 = detail::tm_rhs(th + (0.5 * h), y3, g);
      const std::array<double, 2> y4{y[0] + (h * k3[0]), y[1] + (h * k3[1])};
      const std::array<double, 2> k4 = detail::tm_rhs(th + h, y4, g);
      const std::array<double, 2> yn{y[0] + (h / 6.0 * (k1[0] + (2.0 * k2[0]) + (2.0 * k3[0]) + k4[0])), y[1] + (h / 6.0 * (k1[1] + (2.0 * k2[1]) + (2.0 * k3[1]) + k4[1]))};
      if (yn[1] >= 0.0) {   // vt crosses zero between th and th + h: linear interpolation gives the surface angle
        const double f = y[1] / (y[1] - yn[1]);
        vr_s = y[0] + (f * (yn[0] - y[0]));
        vt_end = 0.0;
        return th + (f * h);
      }
      y = yn;
      th += h;
    }
    vr_s = y[0];
    vt_end = y[1];
    return 0.0;
  };
  // the surface angle grows with the shock angle from 0 at the Mach angle: scan up from it for the first shock angle that reaches the cone's, then bisect inside that step
  double vr_s = 0.0;
  double vt_end = 0.0;
  double lo = mu + 1e-6;
  double hi = lo;
  bool found = false;
  for (double b = mu + 0.002; b < kPi / 2.0 - 0.01; b += 0.002) {
    const double a = surface(b, vr_s, vt_end);
    if (a >= theta_c) {
      hi = b;
      found = true;
      break;
    }
    lo = b;
  }
  if (!found) {
    return best;
  }
  for (int k = 0; k < 60; ++k) {
    const double mid = 0.5 * (lo + hi);
    if (surface(mid, vr_s, vt_end) < theta_c) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const double beta = 0.5 * (lo + hi);
  surface(beta, vr_s, vt_end);
  // on the surface the speed is vr_s (vt = 0): the Mach number and the isentropic pressure from the stagnation state, which the shock leaves at its post-shock total pressure
  const double mn1 = m1 * std::sin(beta);
  const Normal n = normal_shock(mn1, g);
  const double m_s = std::sqrt(2.0 / (g - 1.0) * (vr_s * vr_s) / (1.0 - (vr_s * vr_s)));
  const double p0_over_p = std::pow(1.0 + (0.5 * (g - 1.0) * m_s * m_s), g / (g - 1.0));
  const double dflect = std::atan(2.0 / std::tan(beta) * ((mn1 * mn1) - 1.0) / ((m1 * m1 * (g + std::cos(2.0 * beta))) + 2.0));
  const double m2 = n.m2 / std::sin(beta - dflect);
  const double p0_over_p2 = std::pow(1.0 + (0.5 * (g - 1.0) * m2 * m2), g / (g - 1.0));
  best.shock_angle = beta;
  best.p_surface_p1 = n.p2_p1 * p0_over_p2 / p0_over_p;
  best.rho_surface_rho1 = n.rho2_rho1 * std::pow(p0_over_p2 / p0_over_p, 1.0 / g);
  best.mach_surface = m_s;
  best.attached = true;
  return best;
}

// ---- the exact solution of a Riemann problem ----

struct State1D {
  double rho;
  double u;
  double p;
};

// Toro, "Riemann Solvers and Numerical Methods for Fluid Dynamics", chapter 4: the pressure and velocity in the star region, by Newton's iteration, and the solution sampled at x / t = s.
struct Riemann {
  State1D l;
  State1D r;
  double g = 1.4;
  double p_star = 0.0;
  double u_star = 0.0;

  static double f_side(double p, const State1D& k, double g, double* df) {
    const double a = std::sqrt(g * k.p / k.rho);
    if (p > k.p) {   // a shock
      const double aa = 2.0 / ((g + 1.0) * k.rho);
      const double bb = ((g - 1.0) / (g + 1.0)) * k.p;
      const double q = std::sqrt(aa / (p + bb));
      *df = q * (1.0 - ((p - k.p) / (2.0 * (bb + p))));
      return (p - k.p) * q;
    }
    const double e = (g - 1.0) / (2.0 * g);   // a rarefaction
    const double pr = p / k.p;
    *df = (1.0 / (k.rho * a)) * std::pow(pr, -(g + 1.0) / (2.0 * g));
    return (2.0 * a / (g - 1.0)) * (std::pow(pr, e) - 1.0);
  }

  Riemann(const State1D& left, const State1D& right, double gamma) : l(left), r(right), g(gamma) {
    double p = std::max(1e-12, 0.5 * (l.p + r.p));
    for (int it = 0; it < 100; ++it) {
      double dl = 0.0;
      double dr = 0.0;
      const double fl = f_side(p, l, g, &dl);
      const double fr = f_side(p, r, g, &dr);
      const double f = fl + fr + (r.u - l.u);
      const double pn = std::max(1e-12, p - (f / (dl + dr)));
      const bool done = std::fabs(pn - p) < 1e-13 * (pn + p);
      p = pn;
      if (done) {
        break;
      }
    }
    p_star = p;
    double d1 = 0.0;
    double d2 = 0.0;
    u_star = 0.5 * (l.u + r.u) + (0.5 * (f_side(p, r, g, &d1) - f_side(p, l, g, &d2)));
  }

  [[nodiscard]] State1D sample(double s) const {
    const double gm = (g - 1.0) / (g + 1.0);
    if (s <= u_star) {   // left of the contact
      const double al = std::sqrt(g * l.p / l.rho);
      if (p_star > l.p) {   // a left shock
        const double sl = l.u - (al * std::sqrt(((g + 1.0) / (2.0 * g) * (p_star / l.p)) + ((g - 1.0) / (2.0 * g))));
        if (s <= sl) {
          return l;
        }
        return {l.rho * (((p_star / l.p) + gm) / ((gm * (p_star / l.p)) + 1.0)), u_star, p_star};
      }
      const double shl = l.u - al;   // a left fan
      const double a_star = al * std::pow(p_star / l.p, (g - 1.0) / (2.0 * g));
      const double stl = u_star - a_star;
      if (s <= shl) {
        return l;
      }
      if (s >= stl) {
        return {l.rho * std::pow(p_star / l.p, 1.0 / g), u_star, p_star};
      }
      const double u = (2.0 / (g + 1.0)) * (al + (0.5 * (g - 1.0) * l.u) + s);
      const double c = (2.0 / (g + 1.0)) * (al + (0.5 * (g - 1.0) * (l.u - s)));
      return {l.rho * std::pow(c / al, 2.0 / (g - 1.0)), u, l.p * std::pow(c / al, 2.0 * g / (g - 1.0))};
    }
    const double ar = std::sqrt(g * r.p / r.rho);
    if (p_star > r.p) {   // a right shock
      const double sr = r.u + (ar * std::sqrt(((g + 1.0) / (2.0 * g) * (p_star / r.p)) + ((g - 1.0) / (2.0 * g))));
      if (s >= sr) {
        return r;
      }
      return {r.rho * (((p_star / r.p) + gm) / ((gm * (p_star / r.p)) + 1.0)), u_star, p_star};
    }
    const double shr = r.u + ar;   // a right fan
    const double a_star = ar * std::pow(p_star / r.p, (g - 1.0) / (2.0 * g));
    const double str = u_star + a_star;
    if (s >= shr) {
      return r;
    }
    if (s <= str) {
      return {r.rho * std::pow(p_star / r.p, 1.0 / g), u_star, p_star};
    }
    const double u = (2.0 / (g + 1.0)) * (-ar + (0.5 * (g - 1.0) * r.u) + s);
    const double c = (2.0 / (g + 1.0)) * (ar - (0.5 * (g - 1.0) * (r.u - s)));
    return {r.rho * std::pow(c / ar, 2.0 / (g - 1.0)), u, r.p * std::pow(c / ar, 2.0 * g / (g - 1.0))};
  }
};

}  // namespace sim::cfd::gd
