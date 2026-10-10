// SPDX-License-Identifier: MIT
// Predictor-corrector guidance for a powered burn (core/include/tfc/peg.hpp, docs/design/GNC.md section 4) flown in a point-mass model of a vehicle with a known thrust and exhaust speed: it must bring the vehicle to
// the target orbit's radius, speed and flight-path angle in the target plane from a range of starting states, on the stiff case of an upper stage that burns most of its propellant while its thrust direction turns
// through more than sixty degrees (a Starship-class ship leaving a hot stage), with errors in the thrust and with a late start; the pieces (the orbit point, the cutoff conditions, the limits, the sliced cycle) are checked on
// their own.
#include <cmath>

#include "tfc_test.hpp"

#include "tfc/peg.hpp"

namespace {
using tfc::dm::Vec3;
using tfc::dm::dot;
using tfc::dm::norm;

struct Flight {
  Vec3 r;
  Vec3 v;
  double t = 0.0;
  double burn_t = 0.0;
  bool cut = false;
  double worst_residual = 0.0;
};

tfc::nav::Gravity earth() { return tfc::nav::Gravity{}; }

// Fly a burn in a point-mass model: thrust acceleration c / (tau - burn time) times `thrust_error` along the guidance's direction, gravity of a point mass. The guidance is solved from scratch once at the start (as the
// design does) and then tracks: a cycle begins whenever none is under way and does one prediction a frame.
Flight fly(Vec3 r0, Vec3 v0, const tfc::peg::Target& tgt, double a0, double c, double burn_max, double thrust_error, double t_limit) {
  const tfc::nav::Gravity g = earth();
  Flight f;
  f.r = r0;
  f.v = v0;
  const double tau0 = c / a0;
  tfc::peg::Peg peg;
  tfc::peg::Burn b0;
  b0.accel = a0 * thrust_error;
  b0.exhaust_speed = c;
  b0.burn_time_max = burn_max;
  peg.begin(r0, v0, tgt, b0);
  (void)peg.solve(g, 0.05, 100);
  peg.reset();
  peg.seed(peg.solution());
  const double dt = 0.01;
  unsigned in_cycle = 0U;
  for (int k = 0; k < static_cast<int>(t_limit / dt) && !f.cut; ++k) {
    const double a = (c / (tau0 - f.burn_t)) * thrust_error;
    if (in_cycle == 0U) {
      tfc::peg::Burn b;
      b.accel = a;
      b.exhaust_speed = c;
      b.burn_time_max = burn_max - f.burn_t;
      peg.begin(f.r, f.v, tgt, b);
    }
    const bool done = peg.work(g);
    in_cycle = done ? 0U : in_cycle + 1U;
    const Vec3 dir = peg.direction(f.r);
    f.worst_residual = std::fmax(f.worst_residual, peg.output().residual);
    const Vec3 a0v = (dir * a) + g.at(f.r);
    const Vec3 r1 = f.r + (f.v * dt) + (a0v * (0.5 * dt * dt));
    const Vec3 a1v = (dir * a) + g.at(r1);
    f.v = f.v + ((a0v + a1v) * (0.5 * dt));
    f.r = r1;
    f.t += dt;
    f.burn_t += dt;
    peg.advance(dt);
    if (peg.time_to_go() <= 1.0e-9) {   // the time to go has run out: the engines are cut
      f.cut = true;
    }
  }
  return f;
}

double flight_path_angle(Vec3 r, Vec3 v) { return std::asin(dot(r, v) / (norm(r) * norm(v))); }
}  // namespace

TFC_TEST(peg_a_burn_from_a_high_climbing_state_ends_in_the_target_circular_orbit) {
  const double r_t = 6378137.0 + 200000.0;
  tfc::peg::Target tgt;
  tgt.radius = r_t;
  tgt.speed = std::sqrt(earth().mu / r_t);
  tgt.gamma_rad = 0.0;
  const double R = 6378137.0 + 80000.0;
  const double g40 = 40.0 * tfc::dm::kDegToRad;
  const Flight f = fly(Vec3{R, 0.0, 0.0}, Vec3{1800.0 * std::sin(g40), 1800.0 * std::cos(g40), 0.0}, tgt, 14.0, 3600.0, 450.0, 1.0, 600.0);
  CHECK(f.cut);
  CHECK(std::fabs(norm(f.r) - r_t) < 1000.0);
  CHECK(std::fabs(norm(f.v) - tgt.speed) < 5.0);
  CHECK(std::fabs(flight_path_angle(f.r, f.v)) < 0.1 * tfc::dm::kDegToRad);
  CHECK(std::fabs(dot(f.r, tgt.plane_normal)) < 500.0 && std::fabs(dot(f.v, tgt.plane_normal)) < 1.0);
}

TFC_TEST(peg_the_stiff_case_of_a_ship_that_burns_most_of_its_propellant_while_its_thrust_turns_through_sixty_degrees) {
  const double R = 6378137.0;
  const double r_t = R + 250000.0;
  tfc::peg::Target tgt;
  tgt.radius = r_t;
  tgt.speed = std::sqrt(earth().mu / r_t);
  // 65 km up, 1.55 km/s, 25 degrees above the horizontal, thrust acceleration 9.1 m/s^2 and exhaust speed 3568 m/s: a burn of about 340 s of a possible 392 s of "tau"
  const double gam = 25.0 * tfc::dm::kDegToRad;
  const Vec3 r{R + 65000.0, 150000.0, 0.0};
  const Vec3 up = tfc::dm::unit(r, Vec3{1.0, 0.0, 0.0});
  const Vec3 east = tfc::dm::cross(Vec3{0.0, 0.0, 1.0}, up);
  const Vec3 v = (up * (1550.0 * std::sin(gam))) + (east * (1550.0 * std::cos(gam)));
  for (const double thrust_error : {1.0, 0.93, 1.06}) {
    const Flight f = fly(r, v, tgt, 9.1, 3568.0, 361.0, thrust_error, 500.0);
    CHECK(f.cut);
    CHECK(std::fabs(norm(f.r) - r_t) < 3000.0);
    CHECK(std::fabs(norm(f.v) - tgt.speed) < 12.0);
    CHECK(std::fabs(flight_path_angle(f.r, f.v)) < 0.3 * tfc::dm::kDegToRad);
    CHECK(std::fabs(dot(f.r, tgt.plane_normal)) < 1500.0 && std::fabs(dot(f.v, tgt.plane_normal)) < 3.0);
  }
}

TFC_TEST(peg_a_target_that_is_an_apogee_on_an_ellipse_is_reached_with_the_flight_path_angle_of_the_ellipse) {
  const double mu = earth().mu;
  const double r_a = 6378137.0 + 400000.0;
  const double r_p = 6378137.0 - 20000.0;       // a suborbital ellipse, perigee below the surface
  const double r_c = 6378137.0 + 150000.0;      // cut off at 150 km on the way up
  const tfc::peg::OrbitPoint p = tfc::peg::orbit_point(mu, r_a, r_p, r_c);
  const double a = 0.5 * (r_a + r_p);
  CHECK(std::fabs(0.5 * p.speed * p.speed - mu / r_c + mu / (2.0 * a)) < 1e-3);
  CHECK(p.gamma_rad > 0.0 && p.gamma_rad < 0.5);
  const double h = std::sqrt(mu * 2.0 * r_a * r_p / (r_a + r_p));
  CHECK(std::fabs(r_c * p.speed * std::cos(p.gamma_rad) - h) < 1e-3 * h);
  CHECK(tfc::peg::orbit_point(mu, r_a, r_p, r_a).gamma_rad < 1e-6 && tfc::peg::orbit_point(mu, r_a, r_p, r_p).gamma_rad < 1e-6);
  tfc::peg::Target tgt;
  tgt.radius = r_c;
  tgt.speed = p.speed;
  tgt.gamma_rad = p.gamma_rad;
  const double g = 35.0 * tfc::dm::kDegToRad;
  const Flight f = fly(Vec3{6378137.0 + 70000.0, 0.0, 0.0}, Vec3{1500.0 * std::sin(g), 1500.0 * std::cos(g), 0.0}, tgt, 15.0, 3700.0, 400.0, 1.0, 600.0);
  CHECK(f.cut);
  CHECK(std::fabs(norm(f.r) - r_c) < 1500.0 && std::fabs(norm(f.v) - p.speed) < 8.0);
  CHECK(std::fabs(flight_path_angle(f.r, f.v) - p.gamma_rad) < 0.3 * tfc::dm::kDegToRad);
  // and the apogee of the orbit it is on is the one asked for
  const double energy = 0.5 * dot(f.v, f.v) - mu / norm(f.r);
  const double a_got = -mu / (2.0 * energy);
  const Vec3 hv = tfc::dm::cross(f.r, f.v);
  const double ecc = std::sqrt(std::fmax(0.0, 1.0 + 2.0 * energy * dot(hv, hv) / (mu * mu)));
  CHECK(std::fabs(a_got * (1.0 + ecc) - r_a) < 8000.0);
}

TFC_TEST(peg_the_cycle_is_sliced_one_prediction_a_call_and_the_solution_moves_on_with_the_frames) {
  tfc::peg::Peg peg;
  tfc::peg::Target tgt;
  tgt.radius = 6578137.0;
  tgt.speed = std::sqrt(earth().mu / tgt.radius);
  tfc::peg::Burn burn;
  CHECK(!peg.started() && peg.time_to_go() == 0.0);
  CHECK(!peg.work(earth()));              // no cycle has begun: nothing to do
  peg.advance(0.01);                      // and nothing to advance
  CHECK(!peg.started());
  const Vec3 r{6478137.0, 0.0, 0.0};
  const Vec3 v{900.0, 1500.0, 0.0};
  peg.begin(r, v, tgt, burn);
  unsigned works = 0U;
  while (!peg.work(earth()) && works < 20U) {
    ++works;
  }
  CHECK(works + 1U == 7U);               // the base prediction, one for each of the five columns of the Jacobian, and the trial: seven
  CHECK(peg.started() && peg.output().time_to_go > 10.0 && std::fabs(norm(peg.output().direction) - 1.0) < 1e-12);
  const double t0 = peg.time_to_go();
  const std::array<double, 5> s0 = peg.solution();
  peg.advance(1.0);
  const std::array<double, 5> s1 = peg.solution();
  CHECK(std::fabs(peg.time_to_go() - (t0 - 1.0)) < 1e-12);
  CHECK(std::fabs(s1[0] - (s0[0] + ((s0[1] - s0[0]) * (1.0 / t0)))) < 1e-12);   // the angle moves by the share of the way to its final value that the second is of the burn
  peg.advance(1.0e6);
  CHECK(peg.time_to_go() == 0.0 && peg.output().cutoff);
  peg.reset();
  CHECK(!peg.started());
  // a zero-length plane normal falls back to a sensible one rather than dividing by zero
  tfc::peg::Target odd = tgt;
  odd.plane_normal = Vec3{};
  tfc::peg::Peg p3;
  p3.begin(r, v, odd, burn);
  while (!p3.work(earth())) {
  }
  CHECK(std::fabs(norm(p3.direction(r)) - 1.0) < 1e-12);
}

TFC_TEST(peg_the_propellant_limits_the_burn_and_a_seeded_solution_is_the_start) {
  tfc::peg::Peg limited;
  tfc::peg::Target tgt;
  tgt.radius = 6578137.0;
  tgt.speed = std::sqrt(earth().mu / tgt.radius);
  tfc::peg::Burn small;
  small.burn_time_max = 30.0;
  limited.begin(Vec3{6578137.0, 0.0, 0.0}, Vec3{0.0, 2000.0, 0.0}, tgt, small);
  (void)limited.solve(earth(), 0.05, 20);
  CHECK(limited.time_to_go() <= 30.0 + 1e-9 && limited.output().residual > 1.0 && !limited.output().converged);   // it cannot get there on 30 s of propellant
  tfc::peg::Peg seeded;
  seeded.seed({0.3, -0.1, 0.0, 0.0, 120.0});
  CHECK(seeded.started() && seeded.time_to_go() == 120.0 && seeded.solution()[0] == 0.3);
  seeded.begin(Vec3{6478137.0, 0.0, 0.0}, Vec3{900.0, 1500.0, 0.0}, tgt, tfc::peg::Burn{});
  CHECK(seeded.solution()[4] <= 0.995 * 3500.0 / 10.0 + 1e-9);   // the burn time is held to what tau allows
  // on the target already, the solution has nothing to do: a zero-length burn, cut off at once
  tfc::peg::Peg done;
  done.begin(Vec3{tgt.radius, 0.0, 0.0}, Vec3{0.0, tgt.speed, 0.0}, tgt, tfc::peg::Burn{});
  (void)done.solve(earth(), 0.05, 30);
  CHECK(done.output().cutoff || done.time_to_go() < 5.0);
}
