// SPDX-License-Identifier: MIT
// Hypersonic aerodynamics by Newtonian impact theory (sim/vehicle/newtonian.hpp, docs/design/AERODYNAMICS.md section 8). The panel sum is checked against the closed-form answers the theory
// gives for the shapes where there is one: a cone's drag (Cpmax sin^2 of its half angle on its base), a hemisphere's (Cpmax / 2), a flat face's (Cpmax), a cylinder across the flow (4/3 Cpmax per
// unit of its diameter-times-length), the sign flip when the same flat-ended body flies tail first, the place of a cone's normal force, and a fin as a flat plate.
#include <cmath>

#include "tfc_test.hpp"

#include "newtonian.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }

sim::SectionSpec section(sim::SectionKind kind, double x, double len, double d_aft, double d_fore, sim::NoseShape shape = sim::NoseShape::Cone) {
  sim::SectionSpec s;
  s.kind = kind;
  s.x_start = x;
  s.length = len;
  s.d_aft = d_aft;
  s.d_fore = d_fore;
  s.nose = shape;
  return s;
}

constexpr double kDeg = sim::kPi / 180.0;

}  // namespace

TFC_TEST(newtonian_cone_drag_is_cpmax_sin_squared_of_the_half_angle_on_the_base) {
  sim::NewtonTable t;
  const double d = 1.0;
  const double len = 2.0;
  t.build({section(sim::SectionKind::Nose, 0.0, len, d, 0.0)}, {}, sim::kPi * d * d / 4.0);
  const double delta = std::atan2(0.5 * d, len);
  CHECK(close(t.cx_at(0), sim::newton::kCpMax * std::pow(std::sin(delta), 2.0), 0.005));
  CHECK(t.cx_at(0) > 0.0);
  CHECK(std::fabs(t.cn_at(0)) < 1e-9);   // at zero angle of attack there is no side force, by symmetry
  CHECK(std::fabs(t.a0_at(0)) < 1e-9);
}

TFC_TEST(newtonian_hemisphere_drag_is_half_cpmax) {
  sim::NewtonTable t;
  const double d = 2.0;
  t.build({section(sim::SectionKind::Nose, 0.0, 1.0, d, 0.0, sim::NoseShape::Ellipse)}, {}, sim::kPi * d * d / 4.0);   // an ellipse as long as it is wide is half a sphere
  CHECK(close(t.cx_at(0), 0.5 * sim::newton::kCpMax, 0.01));
  // broadside (90 degrees) only a quarter of the sphere looks into the stream, and by symmetry it carries half the force of the hemisphere that would: Cpmax / 4 along the stream
  CHECK(close(t.cn_at(18), 0.25 * sim::newton::kCpMax, 0.01));
  CHECK(t.cx_at(6) < t.cx_at(0));   // the drag along the axis falls as the dome turns away from the stream
}

TFC_TEST(newtonian_flat_face_has_the_stagnation_pressure_coefficient) {
  sim::NewtonTable t;
  const double d = 1.0;
  t.build({section(sim::SectionKind::Tube, 0.0, 3.0, d, d)}, {}, sim::kPi * d * d / 4.0);
  CHECK(close(t.cx_at(0), sim::newton::kCpMax, 1e-9));            // the front face is the only thing that looks into the stream
  CHECK(close(t.cx_at(36), -sim::newton::kCpMax, 1e-9));          // flying tail first the base takes it, with the force the other way
  CHECK(std::fabs(t.cx_at(18)) < 1e-6);                           // at 90 degrees neither end sees the stream
}

TFC_TEST(newtonian_cylinder_across_the_flow_has_four_thirds_cpmax_per_plan_area) {
  sim::NewtonTable t;
  const double d = 1.0;
  const double len = 10.0;
  const double s_ref = sim::kPi * d * d / 4.0;
  t.build({section(sim::SectionKind::Tube, 0.0, len, d, d)}, {}, s_ref);
  const double cn_expected = sim::newton::kCpMax * (4.0 / 3.0) * (0.5 * d) * len / s_ref;
  CHECK(close(t.cn_at(18), cn_expected, 0.01));
  // the drag coefficient on the plan area (d x len) is therefore 2/3 Cpmax = 1.23, the cylinder's familiar 1.2 to 1.4
  CHECK(close(t.cn_at(18) * s_ref / (d * len), (2.0 / 3.0) * sim::newton::kCpMax, 0.01));
  // the load is centred on the middle of the tube
  CHECK(close(t.a0_at(18) / t.cn_at(18), 0.5 * len, 0.01));
}

TFC_TEST(newtonian_cone_normal_force_acts_a_third_of_its_length_from_the_base) {
  sim::NewtonTable t;
  const double len = 3.0;
  t.build({section(sim::SectionKind::Nose, 0.0, len, 1.0, 0.0)}, {}, sim::kPi * 0.25);
  // at a small angle only the windward side of the cone carries load and its resultant is at two thirds of the length from the apex
  const double x_cp = t.a0_at(1) / t.cn_at(1);
  CHECK(close(x_cp, len / 3.0, 0.08));   // (the axial force of the unevenly loaded cone sits off the axis and moves the resultant a little)
  CHECK(t.cn_at(1) > 0.0);
}

TFC_TEST(newtonian_fins_are_flat_plates_to_the_stream) {
  sim::NewtonTable t;
  const double d = 1.0;
  sim::FinPlanform f;
  f.count = 4;
  f.x_le_root = 2.0;
  f.root_chord = 0.5;
  f.tip_chord = 0.5;
  f.span = 0.4;
  f.sweep = 0.0;
  f.thickness = 0.01;
  const double s_ref = sim::kPi * d * d / 4.0;
  sim::NewtonTable body;
  body.build({section(sim::SectionKind::Tube, 0.0, 3.0, d, d)}, {}, s_ref);
  t.build({section(sim::SectionKind::Tube, 0.0, 3.0, d, d)}, {f}, s_ref);
  // at 90 degrees the fin whose plane holds the flow's normal axis (two of the four) is broadside to it; the other two are edge-on
  const double fin_cn = t.cn_at(18) - body.cn_at(18);
  CHECK(close(fin_cn, sim::newton::kCpMax * 2.0 * (0.5 * (0.5 + 0.5) * 0.4) / s_ref, 0.01));
  // in the stream (angle 0) the fins are edge-on: no addition to the axial force
  CHECK(close(t.cx_at(0), body.cx_at(0), 1e-9));
}

TFC_TEST(newtonian_steps_in_the_radius_are_faces_and_a_shoulder_faces_forward) {
  sim::NewtonTable t;
  const double s_ref = sim::kPi * 0.25 * 4.0;   // the larger tube, 2 m
  // a 2 m tube behind a 1 m tube: the step between them faces forward and carries the pressure on the annulus
  t.build({section(sim::SectionKind::Tube, 0.0, 3.0, 2.0, 2.0), section(sim::SectionKind::Tube, 3.0, 2.0, 1.0, 1.0)}, {}, s_ref);
  const double annulus = sim::kPi * (1.0 - 0.25) / s_ref;
  const double disc = sim::kPi * 0.25 / s_ref;
  CHECK(close(t.cx_at(0), sim::newton::kCpMax * (annulus + disc), 1e-6));
  // a 1 m tube behind a 2 m tube (the larger forward): the step faces aft and a body flying tail first meets it
  sim::NewtonTable u;
  u.build({section(sim::SectionKind::Tube, 0.0, 2.0, 1.0, 1.0), section(sim::SectionKind::Tube, 2.0, 3.0, 2.0, 2.0)}, {}, s_ref);
  CHECK(close(u.cx_at(36), -sim::newton::kCpMax * (annulus + disc), 1e-6));
}

TFC_TEST(newtonian_interpolates_and_folds_the_angle_and_an_empty_table_is_not_ready) {
  sim::NewtonTable none;
  CHECK(!none.ready());
  sim::NewtonTable none2;
  none2.build({}, {}, 1.0);
  CHECK(!none2.ready());
  sim::NewtonTable t;
  t.build({section(sim::SectionKind::Tube, 0.0, 3.0, 1.0, 1.0)}, {}, sim::kPi * 0.25);
  double cx = 0.0;
  double cn = 0.0;
  double a0 = 0.0;
  t.at(2.5 * kDeg, cx, cn, a0);
  CHECK(close(cx, 0.5 * (t.cx_at(0) + t.cx_at(1)), 1e-12));
  double cx2 = 0.0;
  t.at(-2.5 * kDeg, cx2, cn, a0);   // the table is for the magnitude of the angle
  CHECK(close(cx, cx2, 1e-12));
  t.at(200.0 * kDeg, cx, cn, a0);   // and ends at 180 degrees
  CHECK(close(cx, t.cx_at(36), 1e-12));
}

TFC_TEST(newtonian_truncated_nose_has_the_radius_it_is_given_at_the_tip) {
  const sim::SectionSpec s = section(sim::SectionKind::Nose, 0.0, 1.0, 2.0, 1.0);   // a frustum from 2 m down to 1 m over 1 m
  const std::vector<sim::newton::Station> o = sim::newton::outline_of(s, 4);
  CHECK(o.size() == 5U);
  CHECK(close(o.front().r, 1.0, 1e-12));
  CHECK(close(o.back().r, 0.5, 1e-12));
  for (std::size_t i = 1; i < o.size(); ++i) {
    CHECK(o[i].r < o[i - 1U].r);
  }
  // each shape has its radius at the middle of a unit nose of unit radius: cone 0.5, ogive sqrt(rho^2 - 0.25) + 1 - rho with rho = 1, parabola 0.75, ellipse sqrt(0.75)
  CHECK(close(sim::newton::nose_radius(sim::NoseShape::Cone, 0.5, 1.0, 1.0), 0.5, 1e-12));
  CHECK(close(sim::newton::nose_radius(sim::NoseShape::TangentOgive, 0.5, 1.0, 1.0), std::sqrt(0.75), 1e-12));
  CHECK(close(sim::newton::nose_radius(sim::NoseShape::Parabola, 0.5, 1.0, 1.0), 0.75, 1e-12));
  CHECK(close(sim::newton::nose_radius(sim::NoseShape::Ellipse, 0.5, 1.0, 1.0), std::sqrt(0.75), 1e-12));
}
