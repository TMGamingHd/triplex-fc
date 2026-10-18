// SPDX-License-Identifier: MIT
// The stability margins of the attitude loop (sim/vehicle/margins.hpp, docs/design/CONTROL_LOOP.md section 7): the Routh-Hurwitz test against polynomials whose roots are known, the margins against the
// ones worked out by hand (an unstable body with a PD and no lag: the lowest gain is a / (57.3 b kp)) and against the frequency response (the upper gain margin is the loop's gain at the phase crossover).
#include <cmath>
#include <complex>

#include "tfc_test.hpp"

#include "margins.hpp"

namespace {

bool near_rel(double a, double b, double tol) { return std::fabs(a - b) <= tol * std::fmax(std::fabs(a), std::fabs(b)); }

sim::margins::Loop launcher() {
  sim::margins::Loop l;
  l.a = 0.4;          // unstable in the air: the centre of pressure is ahead of the centre of mass
  l.b = 0.05;         // rad/s^2 per degree of gimbal
  l.kp = 0.2655;      // degrees per degree, designed for wn = 0.6 rad/s and a damping of 0.8: kp = (wn^2 + a) / b per radian, over 57.3
  l.kd = 0.3351;
  l.ki = 0.02655;
  l.servo.order = 1;
  l.servo.lag_s = 0.08;
  l.delay_s = 0.015;
  return l;
}

}  // namespace

TFC_TEST(margins_the_routh_test_agrees_with_polynomials_of_known_roots) {
  using sim::margins::Poly;
  using sim::margins::hurwitz;
  using sim::margins::mul;
  CHECK(hurwitz(mul(mul(Poly{1.0, 1.0}, Poly{1.0, 2.0}), Poly{1.0, 3.0})));   // (s+1)(s+2)(s+3)
  CHECK(!hurwitz(mul(Poly{1.0, -1.0}, Poly{1.0, 2.0})));                        // (s-1)(s+2)
  CHECK(!hurwitz(Poly{1.0, 1.0, 1.0, 1.0}));                                   // s^3 + s^2 + s + 1: roots -1 and +-j, on the axis
  CHECK(!hurwitz(Poly{1.0, 1.0, 1.0, 2.0}));                                   // a2 a1 = 1 is not above a0 a3 = 2
  CHECK(hurwitz(mul(Poly{1.0, 1.0}, Poly{1.0, 1.0, 1.0})));                    // (s+1)(s^2+s+1)
  CHECK(hurwitz(Poly{1.0, 0.5, 4.0, 1.0, 3.0}));                               // s^4 + 0.5 s^3 + 4 s^2 + s + 3: the first column 1, 0.5, 2, 0.25, 3 stays positive
  CHECK(!hurwitz(Poly{1.0, 1.0, 1.0, 2.0, 3.0}));                              // s^4 + s^3 + s^2 + 2 s + 3: the first column goes 1, 1, -1, ...
  CHECK(hurwitz(Poly{2.0, 3.0}) && !hurwitz(Poly{-2.0, 3.0}) && !hurwitz(Poly{1.0}));
}

TFC_TEST(margins_an_unstable_body_with_a_pd_and_no_lag_has_the_lower_gain_margin_worked_out_by_hand) {
  // theta'' = a theta + b delta, delta = 57.3 (kp e + kd de): stable when 57.3 b kp > a, so the gain may fall to a / (57.3 b kp)
  sim::margins::Loop l;
  l.a = 0.4;
  l.b = 0.05;
  l.kp = 0.2655;
  l.kd = 0.3351;
  l.ki = 0.0;
  l.servo.lag_s = 1e-6;
  l.delay_s = 1e-6;
  const sim::margins::Result r = sim::margins::analyse(l);
  CHECK(r.stable);
  const double k_low = l.a / (sim::margins::kRadToDeg * l.b * l.kp);
  CHECK(near_rel(r.gain_lower_db, -20.0 * std::log10(k_low), 0.01));
  CHECK(r.gain_lower_db > 0.0 && r.gain_upper_db > r.gain_lower_db);   // (with next to no lag the upper margin is huge)
  // below that gain the same loop is unstable, and the margin says so
  sim::margins::Loop weak = l;
  weak.kp = 0.9 * k_low * l.kp;
  CHECK(!sim::margins::analyse(weak).stable);
}

TFC_TEST(margins_the_upper_gain_margin_is_the_gain_of_the_loop_at_the_phase_crossover) {
  // a double integrator with a PD, a servo lag and a delay: the loop goes unstable at the frequency where its phase is -180 degrees, and the gain there is the margin (a check by the frequency response of the
  // margin found from the characteristic polynomial: the two do not share any code)
  sim::margins::Loop l;
  l.a = 0.0;
  l.b = 0.05;
  l.kp = 0.5;
  l.kd = 0.6;
  l.ki = 0.0;
  l.servo.lag_s = 0.1;
  l.delay_s = 0.01;
  const sim::margins::Result r = sim::margins::analyse(l);
  CHECK(r.stable);
  double gain_db = 0.0;
  for (int i = 0; i < 14000; ++i) {                              // the phase of L crosses -180 degrees where L is real and negative: its imaginary part goes from negative to positive
    const double w = 0.5 * std::pow(1.0005, static_cast<double>(i));
    const std::complex<double> v = sim::margins::response(l, w);
    const std::complex<double> v2 = sim::margins::response(l, w * 1.0005);
    if (v.real() < 0.0 && v.imag() < 0.0 && v2.imag() >= 0.0) {
      gain_db = -20.0 * std::log10(std::abs(v));
      break;
    }
  }
  CHECK(gain_db > 10.0);
  CHECK(std::fabs(r.gain_upper_db - gain_db) < 0.4);
}

TFC_TEST(margins_a_launcher_loop_has_both_gain_margins_a_phase_margin_and_a_delay_margin_and_a_late_loop_has_less) {
  const sim::margins::Loop l = launcher();
  const sim::margins::Result r = sim::margins::analyse(l);
  CHECK(r.stable);
  CHECK(r.crossover_rad_s > 0.5 && r.crossover_rad_s < 5.0);
  CHECK(r.phase_margin_deg > 20.0 && r.phase_margin_deg < 120.0);
  CHECK(r.gain_lower_db > 1.0 && r.gain_upper_db > 3.0);
  CHECK(r.delay_margin_s > 0.05 && r.delay_margin_s < 5.0);
  // the delay margin from the polynomial and the one from the phase margin are the same to the accuracy of a first-order estimate
  CHECK(near_rel(r.delay_margin_s, (r.phase_margin_deg / sim::margins::kRadToDeg) / r.crossover_rad_s, 0.5));
  // more delay, less margin; a loop that was stable at its delay margin is not stable beyond it
  sim::margins::Loop late = l;
  late.delay_s = l.delay_s + 0.2;
  const sim::margins::Result rl = sim::margins::analyse(late);
  CHECK(rl.phase_margin_deg < r.phase_margin_deg);
  CHECK(!sim::margins::analyse([&] { sim::margins::Loop x = l; x.delay_s = l.delay_s + r.delay_margin_s * 1.2; return x; }()).stable);
  CHECK(sim::margins::analyse([&] { sim::margins::Loop x = l; x.delay_s = l.delay_s + r.delay_margin_s * 0.8; return x; }()).stable);
}

TFC_TEST(margins_a_second_order_servo_and_an_unstable_loop_are_reported) {
  sim::margins::Loop l = launcher();
  l.servo.order = 2;
  l.servo.natural_hz = 6.0;
  l.servo.damping = 0.6;
  const sim::margins::Result r2 = sim::margins::analyse(l);
  CHECK(r2.stable && r2.gain_upper_db > 0.0);
  // a gain far too low for the body: the report says unstable and gives no margins
  sim::margins::Loop weak = launcher();
  weak.kp = 0.05;     // 57.3 b kp = 0.14, below the stiffness a = 0.4
  weak.kd = 0.05;
  weak.ki = 0.0;
  const sim::margins::Result rw = sim::margins::analyse(weak);
  CHECK(!rw.stable && rw.gain_lower_db == 0.0 && rw.gain_upper_db == 0.0);
}
