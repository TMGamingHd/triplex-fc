// SPDX-License-Identifier: MIT
// The gas the solver flies in: a perfect gas (a constant gamma), or **air that vibrates and dissociates**, in equilibrium (docs/design/CFD.md section 11).
//
// The real-gas model is the ideal dissociating gas of Lighthill and Freeman with a harmonic vibrator. One species stands for the air's molecule (gas constant R, mass m); at a density rho and temperature T a
// fraction alpha of it is dissociated into two atoms (each of mass m/2, so the gas constant of an atom is 2R), and per unit mass of the original molecules
//     p = rho R T (1 + alpha)
//     e = (1 - alpha) [5/2 R T + R theta_v / (exp(theta_v / T) - 1)] + alpha [3 R T + R theta_d]
//     alpha^2 / (1 - alpha) = (rho_d / rho) exp(-theta_d / T)
// the energy of the translation and rotation of a molecule, of its vibration (a harmonic oscillator of characteristic temperature theta_v), of the translation of its atoms, and of breaking the bond (theta_d);
// the last line is the equilibrium of the dissociation (Lighthill's law). Cold, the vibration is not excited and alpha is zero: gamma is 1.4 exactly. Hot, the vibration takes energy (gamma falls toward 9/7),
// and the dissociation takes more, so that the temperature behind a strong shock is a half of what a perfect gas has and the shock lies closer to the body.
// The constants stand for oxygen, the first to dissociate (theta_d = 59,500 K, rho_d = 150 g/cm^3), and for air's mean vibration (theta_v = 3,000 K). It is NOT equilibrium air: there is no nitric oxide, no
// nitrogen dissociation (from 9,000 K), no ionisation, and a reaction that is infinitely fast (at the altitudes where the density is low it is not).
//
// The solver is non-dimensional: the free stream has density 1, pressure 1 and the temperature 1, the gas constant is 1; `RealAir` takes theta_v, theta_d and rho_d over the free stream's temperature
// and density. A cold perfect-gas free stream (gamma 1.4) is the p = rho T of this gas at alpha = 0.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace sim::cfd {

class RealAir {
 public:
  // theta_v, theta_d and rho_d over the free stream's temperature and density; `tables` makes the lookup tables the solver uses (about a second): without them every call solves the equations (exact, slow).
  RealAir(double theta_v, double theta_d, double rho_d, bool tables = true) : tv_(theta_v), td_(theta_d), rd_(rho_d) {
    if (tables) {
      build_tables();
    }
  }

  // The dissociated fraction at density rho and temperature T.
  [[nodiscard]] double alpha(double rho, double t) const {
    const double s = (rd_ / rho) * std::exp(-td_ / t);
    if (s < 1e-30) {
      return std::sqrt(s);   // cold: nothing is dissociated (and the formula below would be 0 / 0)
    }
    return 2.0 * s / (std::sqrt((s * s) + (4.0 * s)) + s);   // the root of a^2 + s a - s = 0, written so that it does not lose digits when s is large
  }
  [[nodiscard]] double pressure_rt(double rho, double t) const { return rho * t * (1.0 + alpha(rho, t)); }
  [[nodiscard]] double energy_rt(double rho, double t) const {
    const double a = alpha(rho, t);
    const double x = tv_ / t;
    const double vib = x > 500.0 ? 0.0 : tv_ / std::expm1(x);
    return ((1.0 - a) * ((2.5 * t) + vib)) + (a * ((3.0 * t) + td_));
  }
  // The temperature at which the gas of density rho has the pressure p (p rises with T: Newton's method on a bracket), or the specific internal energy e.
  [[nodiscard]] double t_from_p(double rho, double p) const { return solve(rho, p, true); }
  [[nodiscard]] double t_from_e(double rho, double e) const { return solve(rho, e, false); }
  [[nodiscard]] double e_from_p_exact(double rho, double p) const { return energy_rt(rho, t_from_p(rho, p)); }
  [[nodiscard]] double p_from_e_exact(double rho, double e) const { return pressure_rt(rho, t_from_e(rho, e)); }
  // The same from the tables (to a few parts in 10^5 inside them, and the exact solution outside).
  [[nodiscard]] double e_from_p(double rho, double p) const {
    if (!in_range(rho, p, p_lo_, p_hi_)) {
      return e_from_p_exact(rho, p);
    }
    return (p / rho) * look(psi_, rho, p, p_lo_, p_hi_);
  }
  [[nodiscard]] double p_from_e(double rho, double e) const {
    if (!in_range(rho, e, e_lo_, e_hi_)) {
      return p_from_e_exact(rho, e);
    }
    return rho * e * look(phi_, rho, e, e_lo_, e_hi_);
  }
  // The equilibrium speed of sound squared, (dp/drho) at constant entropy: p_rho at constant e, plus p p_e / rho^2, the derivatives by central differences.
  [[nodiscard]] double sound2(double rho, double p) const {
    if (in_range(rho, p, p_lo_, p_hi_)) {
      return (p / rho) * look(gam_, rho, p, p_lo_, p_hi_);
    }
    return sound2_exact(rho, p);
  }
  [[nodiscard]] double sound2_exact(double rho, double p) const {
    const double e = e_from_p_exact(rho, p);
    const double h = 1e-4;
    const double p_rho = (p_from_e_exact(rho * (1.0 + h), e) - p_from_e_exact(rho * (1.0 - h), e)) / (2.0 * h * rho);
    const double p_e = (p_from_e_exact(rho, e * (1.0 + h)) - p_from_e_exact(rho, e * (1.0 - h))) / (2.0 * h * e);
    return p_rho + (p * p_e / (rho * rho));
  }
  [[nodiscard]] bool has_tables() const { return !psi_.empty(); }

 private:
  static constexpr int kNr = 500;   // points in ln rho
  static constexpr int kNy = 1400;   // points in ln e and in ln p
  static constexpr double kRhoLo = 1e-3;
  static constexpr double kRhoHi = 100.0;

  [[nodiscard]] static bool in_range(double rho, double y, double lo, double hi) { return rho > kRhoLo && rho < kRhoHi && y > lo && y < hi; }
  // bilinear interpolation of a table in (ln rho, ln y)
  [[nodiscard]] double look(const std::vector<double>& tab, double rho, double y, double lo, double hi) const {
    const double fx = (std::log(rho) - std::log(kRhoLo)) / (std::log(kRhoHi) - std::log(kRhoLo)) * (kNr - 1);
    const double fy = (std::log(y) - std::log(lo)) / (std::log(hi) - std::log(lo)) * (kNy - 1);
    const int i = std::min(kNr - 2, static_cast<int>(fx));
    const int j = std::min(kNy - 2, static_cast<int>(fy));
    const double tx = fx - i;
    const double ty = fy - j;
    const std::size_t a = (static_cast<std::size_t>(j) * kNr) + static_cast<std::size_t>(i);
    return ((1.0 - ty) * (((1.0 - tx) * tab[a]) + (tx * tab[a + 1U]))) + (ty * (((1.0 - tx) * tab[a + kNr]) + (tx * tab[a + kNr + 1U])));
  }
  // The tables are made without a single Newton solve: for each density the pressure and the energy are computed on a fine, logarithmic grid of temperatures (both rise with T), and the table's entries at
  // the tabulated energies and pressures are found by interpolating that grid backwards.
  void build_tables() {
    e_lo_ = 1e-3;
    e_hi_ = 5e3;
    p_lo_ = 1e-6;
    p_hi_ = 1e6;
    phi_.assign(static_cast<std::size_t>(kNr) * kNy, 0.0);
    psi_ = phi_;
    gam_ = phi_;
    constexpr int kNt = 6000;
    std::vector<double> ts(kNt);
    std::vector<double> es(kNt);
    std::vector<double> ps(kNt);
    std::vector<double> a2(kNt);
    const double t_lo = 1e-4;
    const double t_hi = 1e4;
    for (int i = 0; i < kNr; ++i) {
      const double rho = std::exp(std::log(kRhoLo) + ((std::log(kRhoHi) - std::log(kRhoLo)) * i / (kNr - 1)));
      for (int k = 0; k < kNt; ++k) {
        const double t = std::exp(std::log(t_lo) + ((std::log(t_hi) - std::log(t_lo)) * k / (kNt - 1)));
        ts[static_cast<std::size_t>(k)] = t;
        es[static_cast<std::size_t>(k)] = energy_rt(rho, t);
        ps[static_cast<std::size_t>(k)] = pressure_rt(rho, t);
      }
      // the sound speed squared at each T: p_rho at constant e + p p_e / rho^2 by the derivatives of p(rho, T) and e(rho, T): p_e = p_T / e_T, p_rho|e = p_rho|T - p_T e_rho / e_T
      for (int k = 0; k < kNt; ++k) {
        const double t = ts[static_cast<std::size_t>(k)];
        const double dt = 1e-5 * t;
        const double dr = 1e-5 * rho;
        const double p_t = (pressure_rt(rho, t + dt) - pressure_rt(rho, t - dt)) / (2.0 * dt);
        const double e_t = (energy_rt(rho, t + dt) - energy_rt(rho, t - dt)) / (2.0 * dt);
        const double p_r = (pressure_rt(rho + dr, t) - pressure_rt(rho - dr, t)) / (2.0 * dr);
        const double e_r = (energy_rt(rho + dr, t) - energy_rt(rho - dr, t)) / (2.0 * dr);
        const double p_e = p_t / e_t;
        a2[static_cast<std::size_t>(k)] = (p_r - (p_t * e_r / e_t)) + (ps[static_cast<std::size_t>(k)] * p_e / (rho * rho));
      }
      auto invert = [&](const std::vector<double>& key, double target, double& t_out, int& k_out) {
        const auto it = std::lower_bound(key.begin(), key.end(), target);
        int k = static_cast<int>(it - key.begin());
        k = std::min(std::max(k, 1), kNt - 1);
        const double f = (target - key[static_cast<std::size_t>(k) - 1U]) / (key[static_cast<std::size_t>(k)] - key[static_cast<std::size_t>(k) - 1U]);
        t_out = ts[static_cast<std::size_t>(k) - 1U] + (f * (ts[static_cast<std::size_t>(k)] - ts[static_cast<std::size_t>(k) - 1U]));
        k_out = k;
      };
      for (int j = 0; j < kNy; ++j) {
        const double e = std::exp(std::log(e_lo_) + ((std::log(e_hi_) - std::log(e_lo_)) * j / (kNy - 1)));
        const double p = std::exp(std::log(p_lo_) + ((std::log(p_hi_) - std::log(p_lo_)) * j / (kNy - 1)));
        const std::size_t at = (static_cast<std::size_t>(j) * kNr) + static_cast<std::size_t>(i);
        double t = 0.0;
        int k = 0;
        invert(es, e, t, k);
        phi_[at] = pressure_rt(rho, t) / (rho * e);                               // p / (rho e) = gamma_eff - 1
        invert(ps, p, t, k);
        psi_[at] = energy_rt(rho, t) * rho / p;                                   // rho e / p = 1 / (gamma_eff - 1)
        const double f = (p - ps[static_cast<std::size_t>(k) - 1U]) / (ps[static_cast<std::size_t>(k)] - ps[static_cast<std::size_t>(k) - 1U]);
        gam_[at] = (a2[static_cast<std::size_t>(k) - 1U] + (f * (a2[static_cast<std::size_t>(k)] - a2[static_cast<std::size_t>(k) - 1U]))) * rho / p;   // rho a^2 / p
      }
    }
  }
  // Finds T with pressure_rt(rho, T) = target (pressure true) or energy_rt(rho, T) = target.
  [[nodiscard]] double solve(double rho, double target, bool pressure) const {
    auto f = [&](double t) { return pressure ? pressure_rt(rho, t) : energy_rt(rho, t); };
    double t = pressure ? target / rho : target / 2.5;   // the cold perfect gas's answer is the first guess
    double lo = 0.0;
    double hi = 1e30;
    double dx = 0.0;
    double dx_old = 0.0;
    for (int it = 0; it < 200; ++it) {
      const double v = f(t) - target;
      if (v > 0.0) {
        hi = std::min(hi, t);
      } else {
        lo = std::max(lo, t);
      }
      if (std::fabs(v) <= 1e-13 * target) {
        break;
      }
      const double dt = 1e-6 * t;
      const double d = (f(t + dt) - f(t - dt)) / (2.0 * dt);
      const bool closed = hi < 1e29;
      double tn = d > 0.0 ? t - (v / d) : t * (v > 0.0 ? 0.5 : 2.0);
      // Newton's step is used while it stays inside the bracket and takes the bracket down fast enough; otherwise bisect (or double while the bracket is open above). Without the second condition Newton
      // can jump from one shoulder of the dissociation to the other and back, closing the bracket by a percent a step: the answer after 80 steps was not a root.
      if (!(tn > lo) || !(tn < hi) || (closed && !(std::fabs(tn - t) <= 0.5 * std::fabs(dx_old)) && it > 1)) {
        tn = closed ? 0.5 * (lo + hi) : 2.0 * t;
      }
      dx_old = dx;
      dx = tn - t;
      t = tn;
    }
    return t;
  }
  double tv_;
  double td_;
  double rd_;
  double e_lo_ = 0.0;
  double e_hi_ = 0.0;
  double p_lo_ = 0.0;
  double p_hi_ = 0.0;
  std::vector<double> phi_;   // p / (rho e) on (ln rho, ln e)
  std::vector<double> psi_;   // rho e / p on (ln rho, ln p)
  std::vector<double> gam_;   // rho a^2 / p on (ln rho, ln p)
};

// The gas of a run: a perfect gas of `gamma`, or (air non-null) the real gas, whose cold limit has the same gamma.
struct Gas {
  double gamma = 1.4;
  const RealAir* air = nullptr;
  Gas() = default;
  Gas(double g) : gamma(g) {}                         // NOLINT: a number is a perfect gas
  Gas(double g, const RealAir* a) : gamma(g), air(a) {}
  [[nodiscard]] double e_from_p(double rho, double p) const { return air != nullptr ? air->e_from_p(rho, p) : p / ((gamma - 1.0) * rho); }
  [[nodiscard]] double p_from_e(double rho, double e) const { return air != nullptr ? air->p_from_e(rho, e) : (gamma - 1.0) * rho * e; }
  [[nodiscard]] double sound(double rho, double p) const { return air != nullptr ? std::sqrt(air->sound2(rho, p)) : std::sqrt(gamma * p / rho); }
  // The effective gamma, 1 + p / (rho e): the perfect gas's, or the real gas's now.
  [[nodiscard]] double gamma_eff(double rho, double p) const { return air != nullptr ? 1.0 + (p / (rho * air->e_from_p(rho, p))) : gamma; }
};

}  // namespace sim::cfd
