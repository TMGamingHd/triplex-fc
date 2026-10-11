// SPDX-License-Identifier: MIT
// The guidance of a vehicle that comes back (core/include/tfc/descent.hpp, docs/design/GNC.md section 6): the closed-form impact point of a coasting vehicle against a numerical integration of the same
// trajectory; the descent predictor with and without air; and the powered-descent law flown in a point-mass model of a booster, which must put it over the catch point slowly and upright from a range of starts.
#include <cmath>

#include "tfc_test.hpp"

#include "tfc/descent.hpp"

namespace {
using tfc::dm::Vec3;
using tfc::dm::dot;
using tfc::dm::norm;
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// The impact by brute force: fly the coast with small steps until the radius falls to `radius`.
bool brute_impact(Vec3 r, Vec3 v, double radius, Vec3& point, double& tof) {
  const tfc::nav::Gravity g;
  const double dt = 0.01;
  double t = 0.0;
  for (int k = 0; k < 4000000; ++k) {
    const Vec3 a0 = g.at(r);
    const Vec3 r1 = r + (v * dt) + (a0 * (0.5 * dt * dt));
    const Vec3 a1 = g.at(r1);
    v = v + ((a0 + a1) * (0.5 * dt));
    if (norm(r1) <= radius && dot(r1, v) < 0.0) {
      const double f = (norm(r) - radius) / (norm(r) - norm(r1));
      point = r + ((r1 - r) * f);
      tof = t + (f * dt);
      return true;
    }
    r = r1;
    t += dt;
  }
  return false;
}
}  // namespace

TFC_TEST(descent_the_closed_form_impact_point_matches_a_numerical_coast) {
  const double R = 6378137.0;
  struct Case {
    double alt;
    double speed;
    double gamma_deg;
    double azimuth_deg;
  };
  const Case cases[] = {{70000.0, 1500.0, 20.0, 0.0}, {120000.0, 2500.0, 5.0, 30.0}, {90000.0, 800.0, -30.0, 100.0}, {60000.0, 300.0, 80.0, 0.0}, {150000.0, 3800.0, 10.0, -60.0}};
  for (const Case& c : cases) {
    const double gm = c.gamma_deg * tfc::dm::kDegToRad;
    const double az = c.azimuth_deg * tfc::dm::kDegToRad;
    // position on +X, velocity: radial part up, horizontal part in the direction of azimuth az in the y-z plane
    const Vec3 r{R + c.alt, 0.0, 0.0};
    const Vec3 v{c.speed * std::sin(gm), c.speed * std::cos(gm) * std::cos(az), c.speed * std::cos(gm) * std::sin(az)};
    const tfc::descent::Impact imp = tfc::descent::ballistic_impact(r, v, tfc::nav::Gravity{}.mu, R);
    Vec3 bp;
    double bt = 0.0;
    CHECK(imp.valid && brute_impact(r, v, R, bp, bt));
    CHECK(norm(imp.point - bp) < 15.0);
    CHECK(near_abs(imp.tof, bt, 0.05));
    CHECK(near_abs(norm(imp.point), R, 1e-6));
  }
}

TFC_TEST(descent_an_orbit_that_stays_above_the_sphere_or_leaves_it_has_no_impact_point) {
  const double R = 6378137.0;
  const double mu = tfc::nav::Gravity{}.mu;
  const double r0 = R + 300000.0;
  CHECK(!tfc::descent::ballistic_impact(Vec3{r0, 0.0, 0.0}, Vec3{0.0, std::sqrt(mu / r0), 0.0}, mu, R).valid);          // a circle
  CHECK(!tfc::descent::ballistic_impact(Vec3{r0, 0.0, 0.0}, Vec3{0.0, 12000.0, 0.0}, mu, R).valid);                      // hyperbolic
  CHECK(!tfc::descent::ballistic_impact(Vec3{r0, 0.0, 0.0}, Vec3{0.0, 7800.0, 0.0}, mu, R).valid);                       // perigee above the sphere
  // straight down: the point under it, and the time of a free fall at about the speed it has
  const tfc::descent::Impact down = tfc::descent::ballistic_impact(Vec3{R + 10000.0, 0.0, 0.0}, Vec3{-100.0, 0.0, 0.0}, mu, R);
  CHECK(down.valid && near_abs(norm(down.point - Vec3{R, 0.0, 0.0}), 0.0, 1e-6) && down.tof > 0.0 && down.tof <= 100.0);
  const tfc::descent::Impact up = tfc::descent::ballistic_impact(Vec3{R + 10000.0, 0.0, 0.0}, Vec3{100.0, 0.0, 0.0}, mu, R);
  CHECK(up.valid);   // straight up and above the sphere: it comes back to it
  CHECK(!tfc::descent::ballistic_impact(Vec3{R - 10.0, 0.0, 0.0}, Vec3{100.0, 0.0, 0.0}, mu, R).valid);   // below the sphere and going up
}

TFC_TEST(descent_the_predictor_with_air_agrees_with_the_coast_without_it_and_falls_short_with_it) {
  const double R = 6378137.0;
  const tfc::nav::Gravity g;
  const Vec3 r{R + 60000.0, 0.0, 0.0};
  const Vec3 v{-200.0, 1200.0, 0.0};
  tfc::descent::DragModel none;
  none.beta = 1.0e12;   // a vehicle the air cannot slow
  const tfc::descent::Impact a = tfc::descent::descent_predict(r, v, g, none, R, 0.5, 4000U);
  const tfc::descent::Impact b = tfc::descent::ballistic_impact(r, v, g.mu, R);
  CHECK(a.valid && b.valid && norm(a.point - b.point) < 30.0 && near_abs(a.tof, b.tof, 0.1));
  tfc::descent::DragModel air;
  air.beta = 3000.0;
  const tfc::descent::Impact c = tfc::descent::descent_predict(r, v, g, air, R, 0.5, 4000U);
  CHECK(c.valid && norm(c.point - r) < norm(a.point - r) - 1000.0 && c.tof > a.tof - 1.0);   // the air shortens the path
  const tfc::descent::Impact d = tfc::descent::descent_predict(r, v, g, air, R, 0.5, 5U);       // too few steps: no answer
  CHECK(!d.valid);
}

namespace {
struct Landing {
  Vec3 r;
  Vec3 v;
  double mass = 290000.0;
  double t = 0.0;
  double h_final = 1e9;
  double sink = 0.0;
  double miss = 0.0;
  double tilt_deg = 0.0;
  double max_tilt_deg = 0.0;
  bool landed = false;
  bool crashed = false;
};

// The booster in a point-mass model: 3 engines of 2.45 MN, mass flow from an exhaust speed of 3250 m/s, thrust along the commanded direction at once.
Landing land(Vec3 offset_h, double h0, double v_down, Vec3 v_h, double mass) {
  const tfc::nav::Gravity g;
  tfc::descent::LandingTarget tgt;
  tgt.point = Vec3{g.radius + 60.0, 0.0, 0.0};
  tfc::descent::Engines eng;
  eng.thrust_each = 2.45e6;
  eng.min_throttle = 0.4;
  tfc::descent::PoweredDescent pd;
  Landing L;
  L.mass = mass;
  const Vec3 up{1.0, 0.0, 0.0};
  L.r = tgt.point + (up * h0) + offset_h;
  L.v = (up * -v_down) + v_h;
  bool burning = false;
  const double dt = 0.01;
  for (int k = 0; k < 20000; ++k) {
    const tfc::descent::DescentOut o = pd.update(L.r, L.v, L.mass, tgt, eng, g, 1.15, burning);
    burning = burning || o.ignite;
    Vec3 acc = g.at(L.r);
    if (burning) {
      const double thrust = o.throttle * eng.thrust_each * static_cast<double>(o.engines);
      acc = acc + (o.direction * (thrust / L.mass));
      L.mass -= thrust / 3250.0 * dt;
      const double tilt = std::acos(dot(o.direction, up)) * tfc::dm::kRadToDeg;
      L.tilt_deg = tilt;
      L.max_tilt_deg = std::fmax(L.max_tilt_deg, tilt);
    }
    const Vec3 r1 = L.r + (L.v * dt) + (acc * (0.5 * dt * dt));
    L.v = L.v + (acc * dt);
    L.r = r1;
    L.t += dt;
    const double h = dot(L.r - tgt.point, up);
    if (h <= 0.0) {
      L.h_final = h;
      L.sink = -dot(L.v, up);
      L.miss = norm(tfc::dm::perp(L.r - tgt.point, up));
      L.landed = true;
      break;
    }
  }
  return L;
}
}  // namespace

TFC_TEST(descent_the_powered_descent_puts_the_booster_over_the_catch_point_slowly_and_upright) {
  struct Start {
    Vec3 offset;
    double h0;
    double v_down;
    Vec3 v_h;
    double mass;
  };
  const Start starts[] = {{Vec3{0.0, 0.0, 0.0}, 2200.0, 250.0, Vec3{}, 290000.0},
                          {Vec3{0.0, 150.0, -80.0}, 2500.0, 270.0, Vec3{0.0, -15.0, 8.0}, 285000.0},
                          {Vec3{0.0, -300.0, 200.0}, 3000.0, 300.0, Vec3{0.0, 25.0, -12.0}, 300000.0},
                          {Vec3{0.0, 40.0, 40.0}, 1500.0, 180.0, Vec3{0.0, -5.0, -5.0}, 280000.0}};
  for (const Start& s : starts) {
    const Landing L = land(s.offset, s.h0, s.v_down, s.v_h, s.mass);
    CHECK(L.landed);
    CHECK(L.miss < 1.5);
    CHECK(L.sink >= 0.0 && L.sink < 2.0);
    CHECK(L.tilt_deg < 4.0);
    CHECK(L.max_tilt_deg < 15.5);
    CHECK(L.mass > 100000.0);
  }
}

TFC_TEST(descent_the_burn_waits_for_the_stopping_height_and_the_engine_count_follows_the_thrust_asked_for) {
  const tfc::nav::Gravity g;
  tfc::descent::LandingTarget tgt;
  tgt.point = Vec3{g.radius + 60.0, 0.0, 0.0};
  tfc::descent::Engines eng;
  eng.thrust_each = 2.45e6;
  // high up and fast: not yet, and the attitude to be in is against the velocity
  tfc::descent::PoweredDescent pd;
  const Vec3 r_high = tgt.point + Vec3{20000.0, 0.0, 0.0};
  const tfc::descent::DescentOut early = pd.update(r_high, Vec3{-300.0, 0.0, 0.0}, 290000.0, tgt, eng, g, 1.15, false);
  CHECK(!early.ignite && early.height > 19000.0 && early.direction.x > 0.99);
  // at the stopping height: go
  const double h_stop = tfc::descent::PoweredDescent::stopping_height(300.0, tgt.decel_plan, tgt.sink_ms);
  CHECK(h_stop > 1500.0 && h_stop < 4000.0);
  const tfc::descent::DescentOut now = pd.update(tgt.point + Vec3{h_stop * 1.1, 0.0, 0.0}, Vec3{-300.0, 0.0, 0.0}, 290000.0, tgt, eng, g, 1.15, false);
  CHECK(now.ignite && now.engines >= 2U && now.engines <= 3U && now.throttle >= eng.min_throttle - 1e-12 && now.throttle <= 1.0);
  // well inside the stopping height (too fast for the height left): all three at nearly full throttle
  tfc::descent::PoweredDescent pd_late;
  const tfc::descent::DescentOut late = pd_late.update(tgt.point + Vec3{h_stop * 0.7, 0.0, 0.0}, Vec3{-300.0, 0.0, 0.0}, 290000.0, tgt, eng, g, 1.15, false);
  CHECK(late.ignite && late.engines == 3U && late.throttle > 0.9);
  // slow and low: one engine at a throttle that holds it, and not more than the three
  tfc::descent::PoweredDescent pd2;
  const tfc::descent::DescentOut slow = pd2.update(tgt.point + Vec3{5.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0}, 150000.0, tgt, eng, g, 1.15, true);
  CHECK(slow.engines == 1U && slow.option == 0U && slow.throttle >= eng.min_throttle - 1e-12 && slow.throttle <= 1.0);
  pd2.reset();
  CHECK(!pd2.running());
  // a gentler plan stops later: the stopping height is inversely proportional to the deceleration planned
  CHECK(std::fabs(tfc::descent::PoweredDescent::stopping_height(300.0, 7.5, 0.5) - (2.0 * tfc::descent::PoweredDescent::stopping_height(300.0, 15.0, 0.5))) < 1e-9);
  // the horizontal error pulls the thrust toward the point, within the tilt limit
  tfc::descent::PoweredDescent pd3;
  const tfc::descent::DescentOut off = pd3.update(tgt.point + Vec3{600.0, 500.0, 0.0}, Vec3{-80.0, 0.0, 0.0}, 290000.0, tgt, eng, g, 1.15, true);
  CHECK(off.direction.y < 0.0 && std::acos(off.direction.x) * tfc::dm::kRadToDeg <= tgt.tilt_max_deg + 1e-6);
  CHECK(near_abs(off.miss.y, 500.0, 1e-6));
}

TFC_TEST(descent_the_impact_point_is_refused_for_no_position_and_for_an_orbit_that_stays_under_the_sphere) {
  const tfc::nav::Gravity g;
  CHECK(!tfc::descent::ballistic_impact(Vec3{}, Vec3{0.0, 100.0, 0.0}, g.mu, g.radius).valid);
  // a circular orbit 100 km above the ground never comes down to a sphere 200 km above it
  const double r = g.radius + 100000.0;
  CHECK(!tfc::descent::ballistic_impact(Vec3{r, 0.0, 0.0}, Vec3{0.0, 0.9 * std::sqrt(g.mu / r), 0.0}, g.mu, g.radius + 200000.0).valid);
  // and more landing options than there are read as the four there are (no reading beyond the table)
  tfc::descent::LandingTarget tgt;
  tgt.point = Vec3{g.radius + 40.0, 0.0, 0.0};
  tfc::descent::Engines eng;
  eng.options = 9U;
  tfc::descent::PoweredDescent pd;
  const tfc::descent::DescentOut o = pd.update(tgt.point + Vec3{500.0, 0.0, 0.0}, Vec3{-100.0, 0.0, 0.0}, 3.0e5, tgt, eng, g, 1.2, true);
  CHECK(o.option < tfc::descent::kEngineOptions && o.engines >= 1U);
}
