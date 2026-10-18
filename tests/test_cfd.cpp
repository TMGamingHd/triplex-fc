// SPDX-License-Identifier: MIT
// The Euler solver (sim/cfd, docs/design/CFD.md): the textbook gas dynamics it is checked against, and the solver against them: a uniform stream stays uniform, Sod's shock tube against its exact solution,
// the oblique shock of a wedge against the theta-beta-Mach relation, the pressure on a cone against Taylor-Maccoll, a body at rest in still air, the surface elements against the areas of the shapes, the
// force of a constant pressure over a closed body being zero, and the result not depending on the number of threads. The grids are small so that this runs in seconds; the grid study that gives the
// accuracy a number is in the document (it is run by `tfc_cfd --validate`).
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include "tfc_test.hpp"

// the solver lives in sim/cfd, found through sim/vehicle, which every tool that builds the tests already has on its include path
#include "../cfd/gasdyn.hpp"
#include "../cfd/solver.hpp"

namespace {

using sim::V3;
namespace cfd = sim::cfd;
namespace gd = sim::cfd::gd;

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool near_rel(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
constexpr double kDeg = 3.14159265358979323846 / 180.0;

}  // namespace

TFC_TEST(cfd_the_gas_dynamics_relations_give_the_textbook_numbers) {
  const gd::Normal n = gd::normal_shock(2.0, 1.4);
  CHECK(near_abs(n.p2_p1, 4.5, 1e-12) && near_abs(n.rho2_rho1, 8.0 / 3.0, 1e-12) && near_abs(n.m2, 0.57735, 1e-5));
  CHECK(near_abs(gd::pitot_ratio(2.0, 1.4), 5.6405, 1e-3) && near_abs(gd::pitot_ratio(3.0, 1.4), 12.061, 1e-3) && near_abs(gd::pitot_ratio(0.5, 1.4), 1.1862, 1e-3));
  // a wedge of 10 degrees at Mach 2: the weak shock at 39.3 degrees, 1.707 times the pressure, Mach 1.64 behind
  const double beta = gd::weak_shock_angle(2.0, 10.0 * kDeg, 1.4);
  CHECK(near_abs(beta / kDeg, 39.31, 0.02) && near_abs(gd::deflection(2.0, beta, 1.4), 10.0 * kDeg, 1e-9));
  const gd::Normal o = gd::oblique_shock(2.0, beta, 10.0 * kDeg, 1.4);
  CHECK(near_abs(o.p2_p1, 1.7066, 2e-3) && near_abs(o.m2, 1.64, 0.01));
  CHECK(gd::weak_shock_angle(2.0, 30.0 * kDeg, 1.4) < 0.0);   // past the largest turn (23 degrees at Mach 2) the shock detaches
  // a cone: a thin one at a high Mach number has its shock close to the surface; the wedge's shock is further out than the cone's of the same angle
  const gd::Cone c = gd::cone_flow(5.0, 10.0 * kDeg, 1.4);
  CHECK(c.attached && near_abs(c.shock_angle / kDeg, 15.6, 0.2) && c.p_surface_p1 > 1.0 && c.mach_surface < 5.0);
  const double bw = gd::weak_shock_angle(5.0, 10.0 * kDeg, 1.4);
  CHECK(c.shock_angle < bw);
  const gd::Cone thin = gd::cone_flow(3.0, 2.0 * kDeg, 1.4);   // a slender cone: its shock is a hair outside the Mach wave (the gap falls exponentially with the cone angle) and the pressure rise is small
  CHECK(thin.attached && thin.shock_angle > std::asin(1.0 / 3.0) && near_abs(thin.shock_angle, std::asin(1.0 / 3.0), 1e-3) && thin.p_surface_p1 > 1.0 && thin.p_surface_p1 < 1.06);
  CHECK(!gd::cone_flow(1.5, 60.0 * kDeg, 1.4).attached);
}

TFC_TEST(cfd_the_exact_riemann_solution_of_sods_tube_has_the_textbook_star_state) {
  const gd::Riemann r({1.0, 0.0, 1.0}, {0.125, 0.0, 0.1}, 1.4);
  CHECK(near_abs(r.p_star, 0.30313, 1e-5) && near_abs(r.u_star, 0.92745, 1e-5));
  CHECK(near_abs(r.sample(0.5).rho, 0.42632, 1e-4) && near_abs(r.sample(1.2).rho, 0.26557, 1e-4));   // between the contact and the shock, and on each side of the contact
  CHECK(near_abs(r.sample(-2.0).rho, 1.0, 1e-12) && near_abs(r.sample(3.0).rho, 0.125, 1e-12));
}

TFC_TEST(cfd_a_uniform_stream_with_no_body_stays_uniform_to_round_off_and_the_flux_of_equal_states_is_the_physical_one) {
  cfd::Config cfg;
  cfg.threads = 3;
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(0.0, 1.0, 12), cfd::uniform_nodes(0.0, 1.0, 9), cfd::uniform_nodes(0.0, 1.0, 7)};
  const cfd::Prim inf{1.2, 300.0, -40.0, 25.0, 1.0e5};
  cfd::Euler3D s(nd, inf, nullptr, cfg);
  for (int it = 0; it < 30; ++it) {
    s.step();
  }
  double worst = 0.0;
  for (int k = 0; k < 7; ++k) {
    for (int j = 0; j < 9; ++j) {
      for (int i = 0; i < 12; ++i) {
        const cfd::Prim p = s.prim(i, j, k);
        worst = std::fmax(worst, std::fabs(p.rho - inf.rho) / inf.rho + std::fabs(p.u - inf.u) / 300.0 + std::fabs(p.p - inf.p) / inf.p);
      }
    }
  }
  cfd::Prim at;
  CHECK(s.sample(V3{0.37, 0.52, 0.61}, at) && near_rel(at.p, inf.p, 1e-12) && near_rel(at.u, inf.u, 1e-12));   // an interpolation of equal states is that state
  CHECK(worst < 1e-12 && s.healthy() && !s.floored());
  // the flux of two equal states along an axis is the physical flux: mass rho u, momentum rho u^2 + p, energy (E + p) u
  const cfd::Prim a{1.2, 300.0, -40.0, 25.0, 1.0e5};
  const cfd::Cons f = cfd::flux(a, a, 0, 1.4);
  const double e = (a.p / 0.4) + (0.5 * a.rho * ((a.u * a.u) + (a.v * a.v) + (a.w * a.w)));
  CHECK(near_rel(f[0], a.rho * a.u, 1e-13) && near_rel(f[1], (a.rho * a.u * a.u) + a.p, 1e-13) && near_rel(f[2], a.rho * a.u * a.v, 1e-13) && near_rel(f[4], (e + a.p) * a.u, 1e-13));
  const cfd::Cons g = cfd::flux(a, a, 2, 1.4);   // along z the pressure is in the third momentum
  CHECK(near_rel(g[3], (a.rho * a.w * a.w) + a.p, 1e-13) && near_rel(g[1], a.rho * a.u * a.w, 1e-13));
}

TFC_TEST(cfd_sods_shock_tube_matches_its_exact_solution) {
  const int n = 200;
  cfd::Config cfg;
  cfg.local_dt = false;
  cfg.cfl = 0.5;
  cfg.bc = {cfd::Bc::Outflow, cfd::Bc::Outflow, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry};
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(0.0, 1.0, n), cfd::uniform_nodes(0.0, 0.02, 2), cfd::uniform_nodes(0.0, 0.02, 2)};
  cfd::Euler3D s(nd, cfd::Prim{1.0, 0.0, 0.0, 0.0, 1.0}, nullptr, cfg);
  for (int k = 0; k < 2; ++k) {
    for (int j = 0; j < 2; ++j) {
      for (int i = 0; i < n; ++i) {
        s.set_prim(i, j, k, s.centre(0, i) < 0.5 ? cfd::Prim{1.0, 0.0, 0.0, 0.0, 1.0} : cfd::Prim{0.125, 0.0, 0.0, 0.0, 0.1});
      }
    }
  }
  while (s.time() < 0.2 - 1e-12) {
    s.step(std::fmin(s.max_dt(), 0.2 - s.time()));
  }
  const gd::Riemann ex({1.0, 0.0, 1.0}, {0.125, 0.0, 0.1}, 1.4);
  double l1_rho = 0.0;
  double l1_p = 0.0;
  double l1_u = 0.0;
  double lo = 1e9;
  for (int i = 0; i < n; ++i) {
    const cfd::Prim p = s.prim(i, 1, 1);
    const gd::State1D e = ex.sample((s.centre(0, i) - 0.5) / 0.2);
    l1_rho += std::fabs(p.rho - e.rho) / n;
    l1_p += std::fabs(p.p - e.p) / n;
    l1_u += std::fabs(p.u - e.u) / n;
    lo = std::fmin(lo, p.rho);
  }
  CHECK(l1_rho < 0.006 && l1_p < 0.004 && l1_u < 0.008);   // measured 0.0033, and about the same for the others, with 200 cells
  CHECK(lo > 0.12 && s.healthy() && !s.floored());           // no undershoot below the right state's density
  // mass is conserved: the walls on either end are far from the waves
  double mass = 0.0;
  for (int i = 0; i < n; ++i) {
    mass += s.prim(i, 0, 0).rho / n;
  }
  CHECK(near_abs(mass, (0.5 * 1.0) + (0.5 * 0.125), 1e-3));
}

TFC_TEST(cfd_the_shock_of_a_wedge_has_the_pressure_the_theta_beta_mach_relation_gives) {
  const double th = 10.0 * kDeg;
  const cfd::HalfSpace top(V3{0, 0, 0}, V3{-std::sin(th), std::cos(th), 0});
  const cfd::HalfSpace aft(V3{0, 0, 0}, V3{-1, 0, 0});
  const cfd::Intersection wedge(top, aft);   // solid under the ramp that starts at the origin (on the floor) and rises at 10 degrees
  cfd::Config cfg;
  cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Symmetry, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Symmetry};   // the floor (y = 0) is a slip wall: the ramp rises from it
  const double m = 2.0;
  const cfd::Prim inf{1.0, m * std::sqrt(1.4), 0.0, 0.0, 1.0};
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(-1.0, 3.0, 40), cfd::uniform_nodes(0.0, 1.7, 18), cfd::uniform_nodes(0.0, 0.1, 2)};
  cfd::Euler3D s(nd, inf, &wedge, cfg);
  CHECK(s.ghost_count() > 40);
  s.run_to_steady(600, 9.0);
  const double beta = gd::weak_shock_angle(m, th, 1.4);
  const gd::Normal ex = gd::oblique_shock(m, beta, th, 1.4);
  cfd::Prim behind;
  CHECK(s.sample(V3{2.5, (2.5 * std::tan(th)) + 0.2, 0.05}, behind));
  CHECK(near_rel(behind.p, ex.p2_p1, 0.01));                                                          // measured 0.02 % on this grid of 0.1
  CHECK(std::fabs(std::atan2(behind.v, behind.u) - th) < 1.5 * kDeg);                                  // the flow runs along the ramp
  const double mach_behind = std::sqrt((behind.u * behind.u) + (behind.v * behind.v)) / std::sqrt(1.4 * behind.p / behind.rho);
  CHECK(near_rel(mach_behind, ex.m2, 0.03));
  // ahead of the shock the stream is untouched, and the shock lies where the angle puts it: across x = 1.5 the pressure is half way up at the height the line from the leading edge gives
  cfd::Prim above;
  CHECK(s.sample(V3{0.5, 1.6, 0.05}, above));
  CHECK(near_abs(above.p, 1.0, 0.02));
  const double y_shock = 1.5 * std::tan(beta);
  cfd::Prim mid;
  double y_half = 0.0;
  for (int k = 0; k < 270; ++k) {   // from the top down in steps of 0.005
    const double y = 1.65 - (0.005 * k);
    if (s.sample(V3{1.5, y, 0.05}, mid) && mid.p > 0.5 * (1.0 + ex.p2_p1)) {
      y_half = y;
      break;
    }
  }
  CHECK(near_abs(y_half, y_shock, 0.2));   // within two cells of the line the angle gives (measured 1.1 against 1.23: the shock starts a cell downstream of the edge on this grid)
  CHECK(s.healthy() && !s.floored());
}

TFC_TEST(cfd_a_cone_in_a_supersonic_stream_raises_the_pressure_on_its_side_and_has_a_drag_and_no_side_force) {
  // a smoke test on a grid far too coarse for the number (2.7 cells across the base radius): the accuracy against Taylor-Maccoll is measured by `tfc_cfd --validate`, on grids of 8 to 30 cells
  const double m = 2.5;
  const double th = 15.0 * kDeg;
  const gd::Cone ex = gd::cone_flow(m, th, 1.4);
  const double len = 2.5;
  const double rb = len * std::tan(th);
  const cfd::Revolution body({{0.0, 0.0}, {len, rb}, {len, 0.0}});
  cfd::Config cfg;
  cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
  const double h = 0.25;
  const std::array<std::vector<double>, 3> nd{cfd::graded_nodes(-1.0, len + 1.0, -0.1, len + 0.3, h, 1.1, 3 * h), cfd::graded_nodes(-2.0, 2.0, -1.3 * rb, 1.3 * rb, h, 1.1, 3 * h),
                                              cfd::graded_nodes(0.0, 2.0, 0.0, 1.3 * rb, h, 1.1, 3 * h)};
  const cfd::Prim inf{1.0, m * std::sqrt(1.4), 0.0, 0.0, 1.0};
  cfd::Euler3D s(nd, inf, &body, cfg);
  s.run_to_steady(300, 9.0);
  std::vector<cfd::Panel> pan;
  body.panels(0.2, pan);
  double sum = 0.0;
  double area = 0.0;
  for (const cfd::Panel& p : pan) {
    if (p.c.x > 0.9 && p.c.x < 1.9 && p.n.x < 0.5) {   // the side, away from the tip and the base
      sum += s.wall_pressure(p) * p.area;
      area += p.area;
    }
  }
  const double p_mean = sum / area;
  CHECK(area > 0.5 && p_mean > 1.05 && p_mean < gd::normal_shock(m, 1.4).p2_p1 && p_mean > 0.5 * ex.p_surface_p1);   // raised, and below what a normal shock would give; this grid reads low (34 %)
  CHECK(s.healthy() && !s.floored());
  const cfd::Forces f = s.integrate(pan, V3{0, 0, 0});
  CHECK(near_abs(f.force.z, 0.0, 1e-9) && std::fabs(f.force.y) < 0.02 * f.force.x && f.force.x > 0.0);   // no force across the plane of symmetry; the drag is along the stream
}

TFC_TEST(cfd_a_body_in_still_air_leaves_it_still) {
  const cfd::Sphere ball(V3{0.5, 0.5, 0.5}, 0.25);
  cfd::Config cfg;
  cfg.bc = {cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry, cfd::Bc::Symmetry};
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(0.0, 1.0, 24), cfd::uniform_nodes(0.0, 1.0, 24), cfd::uniform_nodes(0.0, 1.0, 24)};
  const cfd::Prim still{1.0, 0.0, 0.0, 0.0, 1.0};
  cfd::Euler3D s(nd, still, &ball, cfg);
  CHECK(s.ghost_count() > 200);
  for (int it = 0; it < 60; ++it) {
    s.step();
  }
  double worst_p = 0.0;
  double worst_u = 0.0;
  for (int k = 0; k < 24; ++k) {
    for (int j = 0; j < 24; ++j) {
      for (int i = 0; i < 24; ++i) {
        if (s.is_fluid(i, j, k)) {
          const cfd::Prim p = s.prim(i, j, k);
          worst_p = std::fmax(worst_p, std::fabs(p.p - 1.0));
          worst_u = std::fmax(worst_u, std::fabs(p.u) + std::fabs(p.v) + std::fabs(p.w));
        }
      }
    }
  }
  CHECK(worst_p < 1e-9 && worst_u < 1e-9);   // zero velocity reflected is zero velocity and a uniform pressure is its own image: nothing starts
}

TFC_TEST(cfd_the_surface_elements_add_up_to_the_areas_of_the_shapes_and_a_constant_pressure_makes_no_force) {
  // the half of a sphere (z >= 0): 2 pi r^2
  const cfd::Sphere ball(V3{0, 0, 0}, 2.0);
  std::vector<cfd::Panel> pb;
  ball.panels(0.1, pb);
  double a = 0.0;
  for (const cfd::Panel& p : pb) {
    a += p.area;
    CHECK(near_abs(sim::norm(p.n), 1.0, 1e-12) && near_abs(sim::norm(p.c), 2.0, 1e-9) && p.c.z >= 0.0);
  }
  CHECK(near_rel(a, 2.0 * 3.14159265358979 * 4.0, 2e-3));
  // a cone of slant length 5 (radius 3, length 4) with its base: half of pi r s + pi r^2
  const cfd::Revolution cone({{0.0, 0.0}, {4.0, 3.0}, {4.0, 0.0}});
  std::vector<cfd::Panel> pc;
  cone.panels(0.1, pc);
  double ac = 0.0;
  double base = 0.0;
  for (const cfd::Panel& p : pc) {
    ac += p.area;
    if (p.n.x > 0.99) {
      base += p.area;
    }
    CHECK(near_abs(sim::norm(p.n), 1.0, 1e-12) && cone.phi(p.c + (p.n * 1e-3)) > 0.0 && cone.phi(p.c - (p.n * 1e-3)) < 0.0);   // the normal points out of the body
  }
  CHECK(near_rel(ac, 0.5 * 3.14159265358979 * ((3.0 * 5.0) + (3.0 * 3.0)), 2e-3) && near_rel(base, 0.5 * 3.14159265358979 * 9.0, 1e-3));
  // the signed distance of the cone: on the axis inside, a point 1 above the middle of the side, one beyond the base
  CHECK(cone.phi(V3{2.0, 0.0, 0.0}) < -1.0 && cone.phi(V3{4.5, 0.0, 0.0}) > 0.49 && cone.phi(V3{4.5, 0.0, 0.0}) < 0.51);
  CHECK(near_abs(cone.phi(V3{2.0, 1.5 + (0.8 * 1.0), 0.6 * 1.0}), std::sqrt(((1.5 + 0.8) * (1.5 + 0.8)) + 0.36) - 1.5, 0.2));   // a point off the side (in the meridian plane it is the distance to the line)
  // the force of a constant pressure over a closed body is zero (the mirror half is added)
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(-1.0, 5.0, 12), cfd::uniform_nodes(-4.0, 4.0, 12), cfd::uniform_nodes(0.0, 4.0, 6)};
  cfd::Config cfg;
  cfd::Euler3D s2(nd, cfd::Prim{1.0, 0.0, 0.0, 0.0, 7.0}, nullptr, cfg);
  for (int k = 0; k < 6; ++k) {
    for (int j = 0; j < 12; ++j) {
      for (int i = 0; i < 12; ++i) {
        s2.set_prim(i, j, k, cfd::Prim{1.0, 0.0, 0.0, 0.0, 9.0});
      }
    }
  }
  const cfd::Forces f2 = s2.integrate(pc, V3{0, 0, 0});
  CHECK(sim::norm(f2.force) < 1e-9 * 2.0 * f2.area && f2.area > 0.0);   // 2 over a closed surface integrates to nothing
}

TFC_TEST(cfd_the_result_does_not_depend_on_the_number_of_threads) {
  const double th = 12.0 * kDeg;
  const cfd::Revolution body({{0.0, 0.0}, {2.0, 2.0 * std::tan(th)}, {2.0, 0.0}});
  const std::array<std::vector<double>, 3> nd{cfd::uniform_nodes(-0.5, 3.0, 28), cfd::uniform_nodes(-1.5, 1.5, 24), cfd::uniform_nodes(0.0, 1.5, 12)};
  const cfd::Prim inf{1.0, 3.0 * std::sqrt(1.4), 0.2, 0.0, 1.0};
  std::vector<std::vector<double>> out;
  for (const int threads : {1, 5}) {
    cfd::Config cfg;
    cfg.threads = threads;
    cfg.bc = {cfd::Bc::Farfield, cfd::Bc::Outflow, cfd::Bc::Farfield, cfd::Bc::Farfield, cfd::Bc::Symmetry, cfd::Bc::Farfield};
    cfd::Euler3D s(nd, inf, &body, cfg);
    for (int it = 0; it < 25; ++it) {
      s.step();
    }
    std::vector<double> v;
    for (int k = 0; k < 12; ++k) {
      for (int j = 0; j < 24; ++j) {
        for (int i = 0; i < 28; ++i) {
          const cfd::Prim p = s.prim(i, j, k);
          v.insert(v.end(), {p.rho, p.u, p.v, p.w, p.p});
        }
      }
    }
    v.push_back(s.residual());
    out.push_back(v);
  }
  CHECK(out[0].size() == out[1].size() && std::memcmp(out[0].data(), out[1].data(), out[0].size() * sizeof(double)) == 0);   // bit for bit
}
