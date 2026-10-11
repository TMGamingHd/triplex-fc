// SPDX-License-Identifier: MIT
// Closed-loop guidance for a powered burn to a target state: a numerical predictor-corrector in the family of Powered Explicit Guidance (docs/design/GNC.md section 4). Used for the ascent to orbit and for the burns
// that follow (circularising, leaving the orbit).
//
// What it does. The vehicle burns at the thrust acceleration a(t) = c / (tau - t) of a constant mass flow (c the exhaust speed, tau = m / mdot the time its propellant would last), and steers by a pitch angle and a yaw
// angle that change linearly with time, measured from the local horizontal in the plane of the target orbit:
//     theta(t) = theta0 + (theta1 - theta0) t / T        (above the horizontal)          psi(t) = psi0 + (psi1 - psi0) t / T        (out of the plane)
// It cuts the engines after T seconds. The five numbers (theta0, theta1, psi0, psi1, T), the angles at the start and at the end of the burn, are the unknowns. The predictor integrates the burn from the present state under real gravity (steps of velocity Verlet,
// 12 of them, the thrust and the steering taken at the middle of each) to the state at the end of it; the corrector asks five things of that state, which are the five equations: the target radius, the target speed,
// the target flight-path angle, and the plane (zero position and zero velocity out of it). The solution is found by Newton's method (Levenberg-Marquardt damping, a Jacobian from finite differences of the predictor), warm-started
// from the last solution shifted forward in time, so one cycle of seven predictions a few tenths of a second apart is enough to track it. The position along the track is free: where the burn ends is whatever the other
// five say.
//
// Why not the closed-form integrals of the classical PEG. They are exact for a small steering angle and a short burn; an upper stage that burns most of its propellant while its thrust direction turns through
// 60 degrees leaves their linearisation, and the iteration on them diverged on exactly that case (the tests keep it). The numerical form has no such limit and costs more: a cycle is seven predictors of twelve steps, which
// the caller spreads over seven frames (`work` does one predictor per call), so the cost in any one frame is bounded.
//
// What it is not: the Shuttle's or Centaur's flight software (the stage-by-stage burn sequence, the many error terms of a real vehicle); a staging or a thrust that varies by more than the measured acceleration can see is handled
// by restarting at the new stage (`reset`). All arithmetic is + - * / and sqrt on doubles through dmath.hpp, with fixed-length loops. No heap, no exceptions, no RTTI.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/dmath.hpp"
#include "tfc/nav.hpp"

namespace tfc::peg {

using dm::Vec3;

// The end state of the burn.
struct Target {
  double radius = 6578137.0;      // m from the planet's centre
  double speed = 7784.0;          // m/s, inertial
  double gamma_rad = 0.0;         // flight-path angle above the local horizontal at the end of the burn
  Vec3 plane_normal{0.0, 0.0, 1.0};   // unit normal of the target orbit's plane (angular momentum direction)
};

// The vehicle's burn: the thrust acceleration it has now (m/s^2), its exhaust speed c (m/s) and how long it could burn before the propellant runs out (s; the burn is limited to it).
struct Burn {
  double accel = 10.0;
  double exhaust_speed = 3500.0;
  double burn_time_max = 1.0e9;
};

struct Output {
  Vec3 direction{1.0, 0.0, 0.0};   // the unit vector to thrust along now
  Vec3 rate{};                     // its rate of change, per second
  double time_to_go = 0.0;         // s
  double velocity_to_go = 0.0;     // m/s (the speed the burn still has to add, from the rocket equation of the time to go)
  Vec3 r_cutoff{};                 // where the burn is predicted to end
  Vec3 v_cutoff{};
  double residual = 0.0;           // the size of the scaled miss of the predicted end state (about 1 is a kilometre, ten metres per second, ten milliradians)
  bool converged = false;
  bool cutoff = false;             // the burn is over: cut the engines
};

constexpr unsigned kUnknowns = 5U;
constexpr unsigned kSteps = 12U;

class Peg {
 public:
  Peg() noexcept = default;

  // Forget the solution (a new stage or a new burn): the next cycle starts from a guess.
  void reset() noexcept {
    started_ = false;
    cycle_ = kIdle;
    out_ = Output{};
  }

  // Begin a cycle from the navigated state: the predictions of the cycle all start from this state. The previous solution (shifted forward to now) is the starting point; the first cycle starts from a guess.
  void begin(Vec3 r, Vec3 v, const Target& tgt, const Burn& burn) noexcept {
    r0_ = r;
    v0_ = v;
    tgt_ = tgt;
    tgt_.plane_normal = dm::unit(tgt.plane_normal, Vec3{0.0, 0.0, 1.0});
    burn_ = burn;
    tau_ = burn.exhaust_speed / dm::max_(burn.accel, 1.0e-3);
    t_max_ = dm::min_(burn.burn_time_max, 0.995 * tau_);
    t_min_ = 0.0;
    if (!started_) {
      guess();
    }
    sol_[4] = dm::clamp_(sol_[4], 0.0, t_max_);
    work_ = sol_;   // the cycle works on a snapshot: the live solution moves on every frame while the cycle is under way
    elapsed_ = 0U;
    cycle_ = kBase;
    column_ = 0U;
  }

  // One step of the cycle: one prediction. True when the cycle has just been completed (the output is then new). Does nothing if no cycle has begun.
  bool work(const nav::Gravity& grav) noexcept {
    if (cycle_ == kIdle) {
      return false;
    }
    ++elapsed_;
    if (cycle_ == kBase) {
      base_ = residual(work_, grav);
      cycle_ = kJacobian;
      column_ = 0U;
      return false;
    }
    if (cycle_ == kJacobian) {
      jacobian_column(grav);
      if (++column_ >= kUnknowns) {
        solve_step();
        cycle_ = kTrial;
      }
      return false;
    }
    return finish_trial(grav);
  }

  // The solution moves on one frame (dt s): the steering angles advance by their rates, the time to go falls. Call every frame; the output follows.
  void advance(double dt) noexcept {
    if (!started_) {
      return;
    }
    const double t_old = dm::max_(sol_[4], 1.0e-9);
    const double f = dm::min_(dt / t_old, 1.0);
    sol_[0] += (sol_[1] - sol_[0]) * f;
    sol_[2] += (sol_[3] - sol_[2]) * f;
    sol_[4] = dm::max_(sol_[4] - dt, 0.0);
    out_.time_to_go = sol_[4];
    if (sol_[4] <= 1.0e-9) {
      out_.cutoff = true;
    }
  }

  // The unit thrust direction now, from the solution and the state it is flown in (the local basis is that of the position given).
  [[nodiscard]] Vec3 direction(Vec3 r) const noexcept { return steer(r, sol_[0], sol_[2]); }

  // Solve the burn from scratch, for the design of the mission and for the tests (the flight computers are given the solution as a seed and track it): a coarse search over the steering angles and the burn time
  // for the start with the smallest miss, then Levenberg-Marquardt cycles until the miss is under `tolerance` (the scaled size of `Output::residual`) or `max_cycles` have been run. Returns true if it converged.
  // The cost is a few thousand predictions: it is not run in flight.
  bool solve(const nav::Gravity& grav, double tolerance, unsigned max_cycles) noexcept {
    grid_search(grav);
    for (unsigned c = 0; c < max_cycles; ++c) {
      work_ = sol_;
      elapsed_ = 0U;
      cycle_ = kBase;
      column_ = 0U;
      for (unsigned i = 0; i < kUnknowns + 2U && cycle_ != kIdle; ++i) {
        (void)work(grav);
      }
      sol_ = work_;   // (no time passes between the cycles of a solve)
      if (out_.residual < tolerance) {
        break;
      }
    }
    return out_.residual < tolerance;
  }

  [[nodiscard]] bool started() const noexcept { return started_; }
  [[nodiscard]] const Output& output() const noexcept { return out_; }
  [[nodiscard]] double time_to_go() const noexcept { return sol_[4]; }
  [[nodiscard]] std::array<double, kUnknowns> solution() const noexcept { return sol_; }

 private:
  static constexpr uint8_t kIdle = 0U;
  static constexpr uint8_t kBase = 1U;
  static constexpr uint8_t kJacobian = 2U;
  static constexpr uint8_t kTrial = 3U;
  static constexpr double kCutoffTime = 0.03;

  // The steering direction at position r for pitch theta above the local horizontal and yaw psi out of the plane.
  [[nodiscard]] Vec3 steer(Vec3 r, double theta, double psi) const noexcept {
    const Vec3 up = dm::unit(r, Vec3{1.0, 0.0, 0.0});
    const Vec3 along = dm::unit(dm::cross(tgt_.plane_normal, up), Vec3{0.0, 1.0, 0.0});
    double st = 0.0;
    double ct = 0.0;
    double sp = 0.0;
    double cp = 0.0;
    dm::sincos_(theta, st, ct);
    dm::sincos_(psi, sp, cp);
    return (((along * ct) + (up * st)) * cp) + (tgt_.plane_normal * sp);
  }

  // The state at the end of a burn of sol[4] seconds with the steering sol[0..3] from the snapshot of the cycle.
  void predict(const std::array<double, kUnknowns>& sol, const nav::Gravity& grav, Vec3& r_f, Vec3& v_f) const noexcept {
    const double t_burn = dm::clamp_(sol[4], 0.0, t_max_);
    const double h = t_burn / static_cast<double>(kSteps);
    Vec3 rr = r0_;
    Vec3 vv = v0_;
    for (unsigned i = 0; i < kSteps; ++i) {
      const double tm = (static_cast<double>(i) + 0.5) * h;
      const Vec3 mid = rr + (vv * (0.5 * h));
      const double f = tm / dm::max_(t_burn, 1.0e-9);
      const Vec3 dir = steer(mid, sol[0] + ((sol[1] - sol[0]) * f), sol[2] + ((sol[3] - sol[2]) * f));
      const Vec3 thrust = dir * (burn_.exhaust_speed / (tau_ - tm));
      const Vec3 a0 = thrust + grav.at(rr);
      const Vec3 r1 = rr + (vv * h) + (a0 * (0.5 * h * h));
      const Vec3 a1 = thrust + grav.at(r1);
      vv = vv + ((a0 + a1) * (0.5 * h));
      rr = r1;
    }
    r_f = rr;
    v_f = vv;
  }

  // The five scaled misses of the predicted end state.
  [[nodiscard]] std::array<double, kUnknowns> residual(const std::array<double, kUnknowns>& sol, const nav::Gravity& grav) noexcept {
    Vec3 r_f;
    Vec3 v_f;
    predict(sol, grav, r_f, v_f);
    last_r_ = r_f;
    last_v_ = v_f;
    const double rn = dm::norm(r_f);
    const double vn = dm::norm(v_f);
    const double gamma = dm::asin_(dm::dot(r_f, v_f) / dm::max_(rn * vn, 1.0e-9));
    return {(rn - tgt_.radius) / 1000.0, (vn - tgt_.speed) / 10.0, (gamma - tgt_.gamma_rad) / 0.01, dm::dot(r_f, tgt_.plane_normal) / 1000.0, dm::dot(v_f, tgt_.plane_normal) / 10.0};
  }

  [[nodiscard]] static double size_of(const std::array<double, kUnknowns>& f) noexcept {
    double s = 0.0;
    for (const double x : f) {
      s += x * x;
    }
    return dm::sqrt_(s);
  }

  // The scale of each unknown for the finite differences.
  static double step_of(unsigned j) noexcept {
    constexpr std::array<double, kUnknowns> kStep{1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 0.1};
    return kStep[j];
  }

  void jacobian_column(const nav::Gravity& grav) noexcept {
    std::array<double, kUnknowns> pert = work_;
    const double h = step_of(column_);
    pert[column_] += h;
    const std::array<double, kUnknowns> f = residual(pert, grav);
    for (unsigned i = 0; i < kUnknowns; ++i) {
      jac_[i][column_] = (f[i] - base_[i]) / h;
    }
  }

  // Solve (J^T J + mu diag) delta = -J^T f for the Levenberg-Marquardt step by Gaussian elimination with partial pivoting.
  void solve_step() noexcept {
    std::array<std::array<double, kUnknowns + 1U>, kUnknowns> a{};
    const double mu = mu_;
    for (unsigned i = 0; i < kUnknowns; ++i) {
      for (unsigned j = 0; j < kUnknowns; ++j) {
        double s = 0.0;
        for (unsigned k = 0; k < kUnknowns; ++k) {
          s += jac_[k][i] * jac_[k][j];
        }
        a[i][j] = s;
      }
      a[i][i] += mu * (a[i][i] + 1.0e-12);
      double rhs = 0.0;
      for (unsigned k = 0; k < kUnknowns; ++k) {
        rhs -= jac_[k][i] * base_[k];
      }
      a[i][kUnknowns] = rhs;
    }
    delta_ = {};
    if (!eliminate(a)) {
      return;
    }
    // a step is limited to a fraction of the range each unknown can take
    constexpr std::array<double, kUnknowns> kMax{0.35, 0.35, 0.35, 0.35, 40.0};
    double shrink = 1.0;
    for (unsigned i = 0; i < kUnknowns; ++i) {
      const double ratio = dm::fabs_(delta_[i]) / kMax[i];
      shrink = ratio > shrink ? ratio : shrink;
    }
    for (double& d : delta_) {
      d /= shrink;
    }
  }

  // Gaussian elimination on the augmented matrix; the solution goes to delta_. False if the matrix is singular.
  bool eliminate(std::array<std::array<double, kUnknowns + 1U>, kUnknowns>& a) noexcept {
    for (unsigned col = 0; col < kUnknowns; ++col) {
      unsigned piv = col;
      for (unsigned row = col + 1U; row < kUnknowns; ++row) {
        if (dm::fabs_(a[row][col]) > dm::fabs_(a[piv][col])) {
          piv = row;
        }
      }
      if (piv != col) {
        for (unsigned k = 0; k <= kUnknowns; ++k) {
          const double tmp = a[col][k];
          a[col][k] = a[piv][k];
          a[piv][k] = tmp;
        }
      }
      const double d = a[col][col];
      if (!(dm::fabs_(d) > 1.0e-30)) {
        return false;
      }
      for (unsigned row = col + 1U; row < kUnknowns; ++row) {
        const double f = a[row][col] / d;
        for (unsigned k = col; k <= kUnknowns; ++k) {
          a[row][k] -= f * a[col][k];
        }
      }
    }
    for (unsigned ii = 0; ii < kUnknowns; ++ii) {
      const unsigned i = kUnknowns - 1U - ii;
      double s = a[i][kUnknowns];
      for (unsigned j = i + 1U; j < kUnknowns; ++j) {
        s -= a[i][j] * delta_[j];
      }
      delta_[i] = s / a[i][i];
    }
    return true;
  }

  // The trial prediction at the Levenberg-Marquardt step: accepted if it is closer to the target (and the damping is relaxed), rejected otherwise (and the damping is raised: the next step is shorter and more
  // like a steepest descent).
  bool finish_trial(const nav::Gravity& grav) noexcept {
    std::array<double, kUnknowns> trial = work_;
    for (unsigned i = 0; i < kUnknowns; ++i) {
      trial[i] += delta_[i];
    }
    trial[4] = dm::clamp_(trial[4], t_min_, t_max_);
    const std::array<double, kUnknowns> f = residual(trial, grav);
    if (size_of(f) < size_of(base_)) {
      work_ = trial;
      mu_ = dm::max_(mu_ * 0.3, 1.0e-6);
      finished(f);
    } else {
      mu_ = dm::min_(mu_ * 8.0, 1.0e6);
      finished(base_);
    }
    return true;
  }

  // The cycle is over: the solution is moved on by the frames the cycle took (it was computed for the snapshot), the output filled, the state of the cycle cleared.
  void finished(const std::array<double, kUnknowns>& f) noexcept {
    out_.residual = size_of(f);
    out_.converged = out_.residual < 1.0;
    out_.r_cutoff = last_r_;
    out_.v_cutoff = last_v_;
    started_ = true;
    cycle_ = kIdle;
    const double dt_cycle = static_cast<double>(elapsed_) * 0.01;
    const double f_cycle = dm::min_(dt_cycle / dm::max_(work_[4], 1.0e-9), 1.0);
    sol_ = work_;
    sol_[0] += (work_[1] - work_[0]) * f_cycle;   // the solution was found for the state the cycle began in: it is now the cycle's length later
    sol_[2] += (work_[3] - work_[2]) * f_cycle;
    sol_[4] = dm::max_(work_[4] - dt_cycle, 0.0);
    out_.time_to_go = sol_[4];
    out_.direction = steer(r0_, sol_[0], sol_[2]);
    out_.velocity_to_go = burn_.exhaust_speed * dm::log_(tau_ / dm::max_(tau_ - sol_[4], 1.0e-6));
    out_.cutoff = sol_[4] < kCutoffTime;
  }

  // The first solution: thrust along the present velocity (tilted up by the flight-path angle), levelling off over the burn, for the burn the rocket equation gives for the speed to be added.
  void guess() noexcept {
    const Vec3 up = dm::unit(r0_, Vec3{1.0, 0.0, 0.0});
    const double gamma = dm::asin_(dm::dot(up, dm::unit(v0_, up)));
    const double dv = dm::max_(tgt_.speed - dm::norm(v0_), 0.0) + 800.0;
    const double t = dm::clamp_(tau_ * (1.0 - dm::exp_(-dv / burn_.exhaust_speed)), 1.0, t_max_);
    sol_ = {gamma, 0.0, 0.0, 0.0, t};
  }

  // The coarse search: the pitch at the start and at the end of the burn and the burn time on a grid, the one with the smallest miss kept.
  void grid_search(const nav::Gravity& grav) noexcept {
    constexpr std::array<double, 6> kTheta0{0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    constexpr std::array<double, 6> kTheta1{-0.5, -0.35, -0.2, -0.1, 0.0, 0.2};
    constexpr std::array<double, 3> kTimeScale{0.75, 1.0, 1.25};
    const Vec3 up = dm::unit(r0_, Vec3{1.0, 0.0, 0.0});
    const double dv = dm::max_(tgt_.speed - dm::norm(v0_), 0.0) + 900.0;
    const double t_guess = tau_ * (1.0 - dm::exp_(-dv / burn_.exhaust_speed));
    (void)up;
    double best = 1.0e300;
    std::array<double, kUnknowns> best_sol = sol_;
    for (const double t0 : kTheta0) {
      for (const double t1 : kTheta1) {
        for (const double ts : kTimeScale) {
          const std::array<double, kUnknowns> cand{t0, t1, 0.0, 0.0, dm::clamp_(t_guess * ts, 1.0, t_max_)};
          const double size = size_of(residual(cand, grav));
          if (size < best) {
            best = size;
            best_sol = cand;
          }
        }
      }
    }
    sol_ = best_sol;
    started_ = true;
  }

  // Start the solution from numbers given (the nominal solution of the design, which is close to what flies): theta0, theta1, psi0, psi1 in radians, T in seconds.
 public:
  void seed(const std::array<double, kUnknowns>& s) noexcept {
    sol_ = s;
    started_ = true;
  }

 private:
  bool started_ = false;
  uint8_t cycle_ = kIdle;
  unsigned column_ = 0U;
  unsigned elapsed_ = 0U;
  std::array<double, kUnknowns> sol_{};    // the live solution, for the present frame
  std::array<double, kUnknowns> work_{};   // the cycle's snapshot
  std::array<double, kUnknowns> base_{};
  std::array<double, kUnknowns> delta_{};
  std::array<std::array<double, kUnknowns>, kUnknowns> jac_{};
  Vec3 r0_{};
  Vec3 v0_{};
  Vec3 last_r_{};
  Vec3 last_v_{};
  Target tgt_{};
  Burn burn_{};
  double tau_ = 1.0;
  double t_max_ = 0.0;
  double t_min_ = 0.0;
  double mu_ = 1.0e-2;
  Output out_{};
};

// The state at a point of an orbit of apogee radius r_a and perigee radius r_p, at radius r (r_p <= r <= r_a): the speed, and the flight-path angle above the horizontal on the way up (negative of it on the way down).
struct OrbitPoint {
  double speed = 0.0;
  double gamma_rad = 0.0;
};
inline OrbitPoint orbit_point(double mu, double r_a, double r_p, double r) noexcept {
  OrbitPoint p;
  const double a = 0.5 * (r_a + r_p);
  p.speed = dm::sqrt_(dm::max_(mu * ((2.0 / r) - (1.0 / a)), 0.0));
  const double h = dm::sqrt_(mu * 2.0 * r_a * r_p / (r_a + r_p));   // the angular momentum: sqrt(mu p), p = 2 r_a r_p / (r_a + r_p)
  const double cos_g = dm::clamp_(h / (r * dm::max_(p.speed, 1.0e-9)), 0.0, 1.0);
  p.gamma_rad = dm::acos_(cos_g);
  return p;
}

}  // namespace tfc::peg
