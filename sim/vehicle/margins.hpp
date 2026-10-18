// SPDX-License-Identifier: MIT
// Stability margins of the attitude loop, by frequency response (docs/design/CONTROL_LOOP.md section 7; docs/design/FUTURE_WORK.md 6a item 2).
//
// The loop of one axis is the rigid body, the servo, the controller's PID and the delay of the frame:
//
//     theta'' = a theta + b delta                           (a: the aerodynamic stiffness, 1/s^2 per radian, positive when the air turns the vehicle away from the wind;
//                                                            b: the effect of one degree of the effector, rad/s^2, from the design probes)
//     delta   = 57.2958 (kp + kd s + ki / s) (theta_ref - theta)   (the gains of attitude.hpp: degrees of the effector per degree of error)
//     the servo: a first-order lag 1 / (1 + T s), or the second-order wn^2 / (s^2 + 2 zeta wn s + wn^2)
//     the delay: exp(-s tau): the sensor's sample, the vote and the zero-order hold of a 100 Hz loop, 15 ms by default (an assumption, not a measurement)
//
// so that L(s) = 57.2958 b (kp + kd s + ki / s) A(s) exp(-s tau) / (s^2 - a). A launcher is unstable in the air (a > 0: the centre of pressure is ahead of the centre of mass), so the loop has a lower gain
// margin (how far the gain can fall before the vehicle falls over) as well as an upper one (how far it can rise before the servo's lag and the delay make it ring), and its phase margin is measured at the highest
// frequency where |L| crosses 1. Stability is decided on the characteristic polynomial (Routh-Hurwitz, the delay a third-order Pade), not on a plot, so the margins are the gain factors and the extra delay at
// which the closed loop really becomes unstable.
//
// What is not in the loop: the slosh of the propellant and the bending of the structure (their frequencies and the gyros' view of the slope are in the vehicle file and in the time-domain plant; a loop with a
// bending mode close to the crossover needs a notch, and this analysis would show a margin the real vehicle does not have), the rate and travel limits of the servo and the slew limit of the controller (nonlinear:
// they cost margin at large amplitude), and the frame's jitter. All of that is why the margins are a first look and the Monte Carlo is the second.
#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace sim::margins {

constexpr double kRadToDeg = 57.29577951308232;

struct Servo {
  int order = 1;
  double lag_s = 0.08;       // order 1
  double natural_hz = 10.0;  // order 2
  double damping = 0.7;
};

struct Loop {
  double a = 0.0;            // 1/s^2
  double b = 0.0;            // rad/s^2 per degree of the effector
  double kp = 0.0;           // the controller's gains: degrees per degree, per degree/second, per degree second
  double kd = 0.0;
  double ki = 0.0;
  Servo servo;
  double delay_s = 0.015;
};

struct Result {
  bool stable = false;
  double crossover_rad_s = 0.0;   // the highest frequency at which |L| crosses 1 going down (0: no crossing)
  double phase_margin_deg = 0.0;
  double delay_margin_s = 0.0;    // the extra delay the loop takes before it is unstable
  double gain_lower_db = 0.0;     // how far the gain can fall (a positive number of dB: 20 log10 of the factor) before the loop is unstable
  double gain_upper_db = 0.0;     // and how far it can rise
};

using Poly = std::vector<double>;   // highest power first

inline Poly mul(const Poly& p, const Poly& q) {
  Poly r(p.size() + q.size() - 1U, 0.0);
  for (std::size_t i = 0; i < p.size(); ++i) {
    for (std::size_t j = 0; j < q.size(); ++j) {
      r[i + j] += p[i] * q[j];
    }
  }
  return r;
}

inline Poly add(const Poly& p, const Poly& q) {
  Poly r(std::max(p.size(), q.size()), 0.0);
  for (std::size_t i = 0; i < p.size(); ++i) {
    r[r.size() - p.size() + i] += p[i];
  }
  for (std::size_t i = 0; i < q.size(); ++i) {
    r[r.size() - q.size() + i] += q[i];
  }
  return r;
}

// True when every root of the polynomial has a negative real part (Routh's array; a zero or a sign change in the first column is instability).
inline bool hurwitz(Poly p) {
  while (p.size() > 1U && std::fabs(p.front()) < 1e-300) {
    p.erase(p.begin());
  }
  const std::size_t n = p.size() - 1U;
  if (n == 0U) {
    return false;
  }
  if (p.front() < 0.0) {
    for (double& c : p) {
      c = -c;
    }
  }
  for (const double c : p) {
    if (!(c > 0.0)) {
      return false;   // a necessary condition: all the coefficients positive
    }
  }
  std::vector<double> prev, cur;
  for (std::size_t i = 0; i < p.size(); i += 2U) {
    prev.push_back(p[i]);
  }
  for (std::size_t i = 1; i < p.size(); i += 2U) {
    cur.push_back(p[i]);
  }
  for (std::size_t row = 2; row <= n; ++row) {
    if (!(cur.front() > 0.0)) {
      return false;
    }
    std::vector<double> next;
    for (std::size_t k = 0; k + 1U < prev.size(); ++k) {
      const double upper = k + 1U < prev.size() ? prev[k + 1U] : 0.0;
      const double right = k + 1U < cur.size() ? cur[k + 1U] : 0.0;
      next.push_back(((cur.front() * upper) - (prev.front() * right)) / cur.front());
    }
    if (next.empty()) {
      next.push_back(0.0);
    }
    prev = cur;
    cur = next;
  }
  return cur.empty() || cur.front() > 0.0;
}

// The characteristic polynomial of the loop with its gain multiplied by `k` and an extra delay.
inline Poly characteristic(const Loop& l, double k, double extra_delay_s) {
  const double tau = l.delay_s + extra_delay_s;
  // exp(-s tau) as a third-order Pade
  const Poly num_d = {-tau * tau * tau / 120.0, tau * tau / 10.0, -tau / 2.0, 1.0};
  const Poly den_d = {tau * tau * tau / 120.0, tau * tau / 10.0, tau / 2.0, 1.0};
  Poly num_a, den_a;
  if (l.servo.order == 2) {
    const double wn = 2.0 * 3.14159265358979323846 * l.servo.natural_hz;
    num_a = {wn * wn};
    den_a = {1.0, 2.0 * l.servo.damping * wn, wn * wn};
  } else {
    num_a = {1.0};
    den_a = {std::max(l.servo.lag_s, 1e-6), 1.0};
  }
  const double g = k * kRadToDeg * l.b;
  // (kd s^2 + kp s + ki) over s; with no integral the s cancels, and a root at the origin that is not there is not left in the polynomial
  const bool integral = l.ki != 0.0;
  const Poly controller = integral ? Poly{g * l.kd, g * l.kp, g * l.ki} : Poly{g * l.kd, g * l.kp};
  const Poly plant_den = integral ? mul(Poly{1.0, 0.0}, Poly{1.0, 0.0, -l.a}) : Poly{1.0, 0.0, -l.a};
  const Poly left = mul(mul(plant_den, den_a), den_d);
  const Poly right = mul(mul(controller, num_a), num_d);
  return add(left, right);
}

inline std::complex<double> response(const Loop& l, double w) {
  const std::complex<double> s(0.0, w);
  std::complex<double> a_s;
  if (l.servo.order == 2) {
    const double wn = 2.0 * 3.14159265358979323846 * l.servo.natural_hz;
    a_s = (wn * wn) / ((s * s) + (2.0 * l.servo.damping * wn * s) + (wn * wn));
  } else {
    a_s = 1.0 / (1.0 + (std::max(l.servo.lag_s, 1e-6) * s));
  }
  const std::complex<double> c = kRadToDeg * l.b * (l.kp + (l.kd * s) + (l.ki / s));
  return c * a_s * std::exp(-s * l.delay_s) / ((s * s) - l.a);
}

inline Result analyse(const Loop& l) {
  Result r;
  r.stable = hurwitz(characteristic(l, 1.0, 0.0));
  // the highest downward crossing of |L| = 1, on a logarithmic grid
  double prev_w = 1e-2;
  double prev_m = std::abs(response(l, prev_w));
  for (int i = 1; i <= 2000; ++i) {
    const double w = 1e-2 * std::pow(1e5, static_cast<double>(i) / 2000.0);
    const double m = std::abs(response(l, w));
    if (prev_m >= 1.0 && m < 1.0) {
      double lo = prev_w;
      double hi = w;
      for (int it = 0; it < 60; ++it) {
        const double mid = std::sqrt(lo * hi);
        (std::abs(response(l, mid)) >= 1.0 ? lo : hi) = mid;
      }
      r.crossover_rad_s = std::sqrt(lo * hi);
    }
    prev_w = w;
    prev_m = m;
  }
  if (r.crossover_rad_s > 0.0) {
    const double w = r.crossover_rad_s;
    const std::complex<double> s(0.0, w);
    // the phase of everything but the plant, then the plant's own: -180 degrees above its pole (and for an unstable one at every frequency)
    const std::complex<double> rest = response(l, w) * ((s * s) - l.a);
    const double plant_deg = (-w * w - l.a) < 0.0 ? -180.0 : 0.0;
    double total = (std::arg(rest) * kRadToDeg) + plant_deg;
    while (total > 180.0) {
      total -= 360.0;
    }
    while (total <= -180.0) {
      total += 360.0;
    }
    r.phase_margin_deg = 180.0 + total;
    if (r.phase_margin_deg > 180.0) {
      r.phase_margin_deg -= 360.0;
    }
    r.delay_margin_s = r.phase_margin_deg > 0.0 ? (r.phase_margin_deg / kRadToDeg) / r.crossover_rad_s : 0.0;
  }
  if (!r.stable) {
    return r;
  }
  // the gain margins: the factors at which the closed loop first goes unstable, up and down, by bisection
  const auto find = [&l](double step) {
    double good = 1.0;
    double bad = 0.0;
    for (int i = 0; i < 400; ++i) {
      const double k = good * step;
      if (!hurwitz(characteristic(l, k, 0.0))) {
        bad = k;
        break;
      }
      good = k;
    }
    if (bad == 0.0) {
      return good * step;   // never unstable inside the scan: report the end of it
    }
    for (int it = 0; it < 60; ++it) {
      const double mid = std::sqrt(good * bad);
      (hurwitz(characteristic(l, mid, 0.0)) ? good : bad) = mid;
    }
    return std::sqrt(good * bad);
  };
  r.gain_upper_db = 20.0 * std::log10(find(1.03));
  r.gain_lower_db = -20.0 * std::log10(find(1.0 / 1.03));
  // the delay margin from the characteristic polynomial too (the crossover's estimate above is the first-order one)
  double good = 0.0;
  double bad = -1.0;
  for (double extra = 0.001; extra < 2.0; extra *= 1.15) {
    if (!hurwitz(characteristic(l, 1.0, extra))) {
      bad = extra;
      break;
    }
    good = extra;
  }
  if (bad > 0.0) {
    for (int it = 0; it < 50; ++it) {
      const double mid = 0.5 * (good + bad);
      (hurwitz(characteristic(l, 1.0, mid)) ? good : bad) = mid;
    }
    r.delay_margin_s = 0.5 * (good + bad);
  } else {
    r.delay_margin_s = good;
  }
  return r;
}

}  // namespace sim::margins
