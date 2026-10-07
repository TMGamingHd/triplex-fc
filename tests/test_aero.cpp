// SPDX-License-Identifier: MIT
// The aerodynamics that follow the shape (sim/vehicle/aero.hpp, docs/design/AERODYNAMICS.md). Each component of the build-up is checked against a hand calculation of the same published
// formula (Barrowman's normal-force slopes and centres of pressure, the Prandtl-Schlichting skin friction, the Newtonian wave drag of a nose) written out independently here, and against the
// limits the physics demands (a cone's centre of pressure at two thirds of its length from the tip, a transition that narrows to a point being a cone, power-on taking away exactly the base drag it
// is said to, the force at 90 and at 180 degrees, continuity with Mach). Then the vehicle: the force and the moment of a body in the flow, the change when a stage leaves, and the table model against
// the reference model's own numbers.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "aero.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

sim::SectionSpec nose(double x_start, double length, double d_base, sim::NoseShape shape) {
  sim::SectionSpec s;
  s.kind = sim::SectionKind::Nose;
  s.x_start = x_start;
  s.length = length;
  s.d_aft = d_base;
  s.d_fore = 0.0;
  s.nose = shape;
  return s;
}
sim::SectionSpec tube(double x_start, double length, double d) {
  sim::SectionSpec s;
  s.kind = sim::SectionKind::Tube;
  s.x_start = x_start;
  s.length = length;
  s.d_aft = d;
  s.d_fore = d;
  return s;
}
sim::SectionSpec transition(double x_start, double length, double d_aft, double d_fore) {
  sim::SectionSpec s;
  s.kind = sim::SectionKind::Transition;
  s.x_start = x_start;
  s.length = length;
  s.d_aft = d_aft;
  s.d_fore = d_fore;
  return s;
}

// The slope and the centre of pressure (from the origin, x_cg = 0) of what is built.
struct Slope {
  double cn_alpha;
  double x_cp;
};
Slope slope_of(const sim::AeroGeometry& g, double mach) {
  double cn = 0.0;
  double m = 0.0;
  g.slopes(mach, 0.0, cn, m);
  return Slope{cn, cn != 0.0 ? m / cn : 0.0};
}

}  // namespace

// ---- the components ----

TFC_TEST(aero_a_nose_has_the_slope_two_and_the_centre_of_pressure_of_its_shape) {
  const std::array<std::pair<sim::NoseShape, double>, 4> shapes{{{sim::NoseShape::Cone, 0.666}, {sim::NoseShape::TangentOgive, 0.466}, {sim::NoseShape::Parabola, 0.5}, {sim::NoseShape::Ellipse, 0.333}}};
  for (const auto& sh : shapes) {
    sim::AeroGeometry g;
    g.build({nose(0.0, 1.5, 0.2, sh.first)}, {}, 0.0);
    const Slope s = slope_of(g, 0.3);
    CHECK(close(s.cn_alpha, 2.0, 1e-12));                         // a slender body of revolution: 2 per radian on its base area
    CHECK(near_abs(s.x_cp, 1.5 - (sh.second * 1.5), 1e-12));      // measured from the tip, which is forward
  }
  sim::AeroGeometry g2;
  g2.build({nose(0.0, 1.5, 0.2, sim::NoseShape::Cone)}, {}, 0.0);
  CHECK(close(slope_of(g2, 0.1).cn_alpha, slope_of(g2, 4.0).cn_alpha, 1e-12));  // the nose's slope does not depend on Mach
}

TFC_TEST(aero_a_transition_adds_the_slope_of_the_change_in_area_at_the_place_barrowman_gives) {
  // a flare: 0.2 m forward, 0.3 m aft, 0.5 m long, behind a tube of 0.2 m (the reference diameter is the largest, 0.3 m)
  sim::AeroGeometry g;
  g.build({tube(1.0, 1.0, 0.2), transition(0.5, 0.5, 0.3, 0.2)}, {}, 0.0);
  const double s_ref = sim::kPi * 0.09 / 4.0;
  const double expected = 2.0 * ((sim::kPi * 0.09 / 4.0) - (sim::kPi * 0.04 / 4.0)) / s_ref;  // 2 (A_aft - A_fore) / S
  const Slope s = slope_of(g, 0.3);
  CHECK(close(s.cn_alpha, expected, 1e-12) && s.cn_alpha > 0.0);
  const double ratio = 0.2 / 0.3;  // forward over aft
  const double from_fore = (0.5 / 3.0) * (1.0 + ((1.0 - ratio) / (1.0 - (ratio * ratio))));
  CHECK(near_abs(s.x_cp, 0.5 + 0.5 - from_fore, 1e-12));
  // a boat-tail (aft smaller) subtracts
  sim::AeroGeometry b;
  b.build({tube(1.0, 1.0, 0.3), transition(0.5, 0.5, 0.2, 0.3)}, {}, 0.0);
  CHECK(slope_of(b, 0.3).cn_alpha < 0.0);
  // a transition that narrows forward to (nearly) a point is a cone: slope 2 and its centre of pressure two thirds of the length from the tip
  sim::AeroGeometry c;
  c.build({transition(0.0, 1.2, 0.3, 1e-9)}, {}, 0.0);
  CHECK(close(slope_of(c, 0.3).cn_alpha, 2.0, 1e-6) && near_abs(slope_of(c, 0.3).x_cp, 1.2 - (1.2 * 2.0 / 3.0), 1e-6));
  // a tube adds nothing
  sim::AeroGeometry n1;
  n1.build({nose(1.0, 1.0, 0.2, sim::NoseShape::Cone)}, {}, 0.0);
  sim::AeroGeometry n2;
  n2.build({tube(0.0, 1.0, 0.2), nose(1.0, 1.0, 0.2, sim::NoseShape::Cone)}, {}, 0.0);
  CHECK(close(slope_of(n1, 0.3).cn_alpha, slope_of(n2, 0.3).cn_alpha, 1e-12) && close(slope_of(n1, 0.3).x_cp, slope_of(n2, 0.3).x_cp, 1e-12));
}

TFC_TEST(aero_fins_have_the_slope_and_the_centre_of_pressure_of_barrowmans_formulas) {
  // four rectangular fins, root and tip chord 0.1 m, span 0.1 m, on a 0.1 m body: K = 1 + R / (s + R) = 4/3, the mid-chord line is the span itself
  sim::FinPlanform f;
  f.count = 4;
  f.x_le_root = 0.3;
  f.root_chord = 0.1;
  f.tip_chord = 0.1;
  f.span = 0.1;
  f.sweep = 0.0;
  f.thickness = 0.002;
  sim::AeroGeometry g;
  g.build({tube(0.0, 0.5, 0.1)}, {f}, 0.0);
  const double k = 1.0 + (0.05 / (0.1 + 0.05));
  const double hand = (k * 4.0 * 4.0 * 1.0) / (1.0 + std::sqrt(1.0 + std::pow((2.0 * 0.1) / 0.2, 2.0)));  // K 4 N (s/d)^2 / (1 + sqrt(1 + (2 L_f / (C_r + C_t))^2))
  CHECK(close(hand, 8.8365, 1e-4));
  const Slope s = slope_of(g, 0.0);
  CHECK(close(s.cn_alpha, hand, 1e-9));
  CHECK(near_abs(s.x_cp, 0.3 - 0.025, 1e-12));  // a rectangular fin: a quarter of its chord behind its leading edge
  // a swept, tapered fin, worked by hand
  sim::FinPlanform w;
  w.count = 3;
  w.x_le_root = 0.4;
  w.root_chord = 0.2;
  w.tip_chord = 0.1;
  w.span = 0.15;
  w.sweep = 0.1;
  w.thickness = 0.003;
  sim::AeroGeometry gw;
  gw.build({tube(0.0, 0.6, 0.12)}, {w}, 0.0);
  const double kw = 1.0 + (0.06 / (0.15 + 0.06));
  const double lf = std::hypot(0.15, 0.1 + (0.5 * (0.1 - 0.2)));
  const double cn_w = (kw * 4.0 * 3.0 * std::pow(0.15 / 0.12, 2.0)) / (1.0 + std::sqrt(1.0 + std::pow((2.0 * lf) / 0.3, 2.0)));
  const double cp_w = 0.4 - (((0.1 / 3.0) * ((0.2 + (2.0 * 0.1)) / 0.3)) + ((1.0 / 6.0) * (0.3 - ((0.2 * 0.1) / 0.3))));
  const Slope sw = slope_of(gw, 0.0);
  CHECK(close(sw.cn_alpha, cn_w, 1e-9) && near_abs(sw.x_cp, cp_w, 1e-12));
}

TFC_TEST(aero_the_centre_of_pressure_of_the_whole_is_the_slope_weighted_mean_of_its_parts_and_fins_move_it_aft) {
  sim::FinPlanform f;
  f.count = 4;
  f.x_le_root = 0.3;
  f.root_chord = 0.12;
  f.tip_chord = 0.06;
  f.span = 0.1;
  f.sweep = 0.05;
  f.thickness = 0.002;
  sim::AeroGeometry body;
  body.build({tube(0.0, 1.0, 0.1), nose(1.0, 0.4, 0.1, sim::NoseShape::TangentOgive)}, {}, 0.0);
  sim::AeroGeometry both;
  both.build({tube(0.0, 1.0, 0.1), nose(1.0, 0.4, 0.1, sim::NoseShape::TangentOgive)}, {f}, 0.0);
  sim::AeroGeometry fins_only;
  fins_only.build({tube(0.0, 1.4, 0.1)}, {f}, 0.0);
  const Slope sb = slope_of(body, 0.0);
  const Slope sf = slope_of(fins_only, 0.0);
  const Slope sa = slope_of(both, 0.0);
  CHECK(close(sa.cn_alpha, sb.cn_alpha + sf.cn_alpha, 1e-12));
  CHECK(close(sa.x_cp, ((sb.cn_alpha * sb.x_cp) + (sf.cn_alpha * sf.x_cp)) / (sb.cn_alpha + sf.cn_alpha), 1e-12));
  CHECK(sa.x_cp < sb.x_cp);  // the fins pull the centre of pressure aft: that is what they are for
}

TFC_TEST(aero_the_lift_slope_of_a_fin_follows_mach_continuously_and_falls_at_high_mach) {
  const double ar = 1.5;
  const double lambda = 0.4;
  const double c0 = sim::aero::wing_lift_slope(ar, lambda, 0.0);
  CHECK(sim::aero::wing_lift_slope(ar, lambda, 0.6) > c0);                                                       // Prandtl-Glauert: it rises toward Mach 1
  CHECK(sim::aero::wing_lift_slope(ar, lambda, 3.0) < c0 && sim::aero::wing_lift_slope(ar, lambda, 3.0) > 0.0);  // and falls as 1/beta above Mach 1
  CHECK(near_abs(sim::aero::wing_lift_slope(ar, lambda, 0.8 - 1e-9), sim::aero::wing_lift_slope(ar, lambda, 0.8 + 1e-9), 1e-6));   // no jump at either end of the transonic blend
  CHECK(near_abs(sim::aero::wing_lift_slope(ar, lambda, 1.2 - 1e-9), sim::aero::wing_lift_slope(ar, lambda, 1.2 + 1e-9), 1e-6));
  // a very long thin fin approaches the two-dimensional limit 2 pi (low speed), a very short one the slender-wing limit of (pi / 2) AR
  CHECK(sim::aero::wing_lift_slope(40.0, 0.0, 0.0) > 5.5 && sim::aero::wing_lift_slope(40.0, 0.0, 0.0) < 2.0 * sim::kPi);
  CHECK(near_abs(sim::aero::wing_lift_slope(0.1, 0.0, 0.0), 0.5 * sim::kPi * 0.1, 0.03));
  // the fins of a vehicle: the slope of the whole follows it, a body without fins does not
  sim::FinPlanform f;
  f.count = 4;
  f.x_le_root = 0.3;
  f.root_chord = 0.12;
  f.tip_chord = 0.06;
  f.span = 0.1;
  f.sweep = 0.05;
  f.thickness = 0.002;
  sim::AeroGeometry g;
  g.build({tube(0.0, 0.5, 0.1)}, {f}, 0.0);
  CHECK(slope_of(g, 3.0).cn_alpha < slope_of(g, 0.0).cn_alpha && slope_of(g, 0.5).cn_alpha > slope_of(g, 0.0).cn_alpha);
  CHECK(slope_of(g, 0.5).x_cp > 0.0);  // (the centre of pressure of fins alone stays at the fins)
}

TFC_TEST(aero_the_axial_force_is_skin_friction_waves_front_face_and_base_and_a_running_engine_takes_the_base_away) {
  const sim::AeroSpec spec;
  // a plain tube, 0.2 m by 2.0 m: skin friction, a flat front face and the base, at Mach 0.3 and 1e7 Reynolds
  sim::AeroGeometry t;
  t.build({tube(0.0, 2.0, 0.2)}, {}, 0.0);
  const double s_ref = sim::kPi * 0.04 / 4.0;
  const double wetted = sim::kPi * 0.2 * 2.0;
  const double cf = 0.455 / std::pow(std::log10(1.0e7), 2.58) * std::pow(1.0 + (0.144 * 0.09), -0.65);
  const double base_cd = 0.12 + (0.13 * 0.09);
  const double face = 0.4;  // a flat face subsonic: 0.4 on its area, which is the reference area here
  const double expected = (cf * wetted / s_ref) + face + base_cd;
  CHECK(close(t.axial(0.3, 1.0e7, 0.0, spec), expected, 1e-12));
  // a running engine takes away `power_on_base` of the base drag, exactly
  CHECK(close(t.axial(0.3, 1.0e7, 1.0, spec), expected - (0.7 * base_cd), 1e-12));
  CHECK(close(t.axial(0.3, 1.0e7, 0.5, spec), expected - (0.35 * base_cd), 1e-12));
  CHECK(t.axial(0.3, 1.0e8, 0.0, spec) < t.axial(0.3, 1.0e6, 0.0, spec));  // more Reynolds, less skin friction
  // a cone on it, worked by hand at Mach 2: skin on the cone's slant as well, the Newtonian wave drag 2 sin^2(half angle) on the whole base, no front face (a point), the base at 0.25 / M
  sim::AeroGeometry n;
  n.build({tube(0.0, 2.0, 0.2), nose(2.0, 0.6, 0.2, sim::NoseShape::Cone)}, {}, 0.0);
  const double slant = std::hypot(0.6, 0.1);
  const double wet_n = wetted + (sim::kPi * 0.5 * 0.2 * slant);
  const double cf2 = 0.455 / std::pow(std::log10(1.0e7), 2.58) * std::pow(1.0 + (0.144 * 4.0), -0.65);
  const double wave = 2.0 * std::pow(std::sin(std::atan2(0.1, 0.6)), 2.0);
  const double peak = 1.0 + (0.6 * std::exp(-std::pow((2.0 - 1.15) / 0.2, 2.0)));
  CHECK(close(n.axial(2.0, 1.0e7, 0.0, spec), (cf2 * wet_n / s_ref) + (peak * wave) + (0.25 / 2.0), 1e-12));
  CHECK(n.axial(2.0, 1.0e7, 0.0, spec) < t.axial(2.0, 1.0e7, 0.0, spec));  // a nose cone drags less than a flat face
  // the wave drag sets in with Mach: at 0.5 there is none, so the cone's drag is skin, no face and the subsonic base
  CHECK(close(n.axial(0.5, 1.0e7, 0.0, spec), (0.455 / std::pow(std::log10(1.0e7), 2.58) * std::pow(1.0 + (0.144 * 0.25), -0.65) * wet_n / s_ref) + 0.12 + (0.13 * 0.25), 1e-12));
  // a blunter nose drags more, and an ogive less than a cone of the same fineness
  sim::AeroGeometry blunt;
  blunt.build({tube(0.0, 2.0, 0.2), nose(2.0, 0.2, 0.2, sim::NoseShape::Cone)}, {}, 0.0);
  sim::AeroGeometry ogive;
  ogive.build({tube(0.0, 2.0, 0.2), nose(2.0, 0.6, 0.2, sim::NoseShape::TangentOgive)}, {}, 0.0);
  CHECK(blunt.axial(2.0, 1.0e7, 0.0, spec) > n.axial(2.0, 1.0e7, 0.0, spec) && ogive.axial(2.0, 1.0e7, 0.0, spec) < n.axial(2.0, 1.0e7, 0.0, spec));
  // a plausible rocket across the range: between 0.15 and 1.2 everywhere
  for (int i = 0; i <= 48; ++i) {
    const double m = 0.2 + (0.1 * static_cast<double>(i));
    const double ca = n.axial(m, 1.0e7, 0.0, spec);
    CHECK(ca > 0.15 && ca < 1.2);
  }
  // the transonic rise peaks near Mach 1.15 above both sides
  CHECK(n.axial(1.15, 1.0e7, 0.0, spec) > n.axial(0.7, 1.0e7, 0.0, spec) && n.axial(1.15, 1.0e7, 0.0, spec) > n.axial(4.0, 1.0e7, 0.0, spec));
}

// ---- the vehicle in the flow ----

namespace {

// A 0.1 m rocket body (a nose, a tube and four fins) of 200 kg, without engines, high in the air and without gravity, so that the only force is the air's.
sim::Params shaped_body() {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = 200.0;
  st.length = 2.0;
  st.radius = 0.05;
  st.x_cg_dry = 0.8;
  st.sections = {tube(0.0, 1.5, 0.1), nose(1.5, 0.5, 0.1, sim::NoseShape::TangentOgive)};
  sim::FinPlanform f;
  f.count = 4;
  f.x_le_root = 0.4;
  f.root_chord = 0.2;
  f.tip_chord = 0.1;
  f.span = 0.12;
  f.sweep = 0.1;
  f.thickness = 0.003;
  st.stabilizers.push_back(f);
  p.spec.stages.push_back(st);
  return p;
}

sim::Scenario at_speed(const sim::V3& v_body_frame_inertial, double altitude) {
  sim::Scenario sc;
  sc.wind_scale = 0.0;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + altitude, 0.0, 0.0};
  sc.initial.v = v_body_frame_inertial;
  return sc;
}

}  // namespace

TFC_TEST(aero_vehicle_the_force_at_small_angles_is_the_slope_times_the_angle_and_the_moment_is_the_arm_to_the_centre_of_pressure) {
  const sim::Params p = shaped_body();
  sim::Scenario sc = at_speed(sim::V3{300.0, 300.0 * std::tan(0.01), 0.0}, 5000.0);  // 0.01 rad of angle of attack, nose into the flow
  const sim::Vehicle6 v(p, sc);
  const sim::Loads l = v.current_loads();
  double cn = 0.0;
  double xcp = 0.0;
  double ca = 0.0;
  double s_ref = 0.0;
  v.aero_summary(l.mach, cn, xcp, ca, s_ref);
  const double alpha = std::atan2(300.0 * std::tan(0.01), 300.0);
  const double q_s = l.dynamic_pressure * s_ref;
  CHECK(near_abs(l.alpha, alpha, 1e-12));
  // the normal force opposes the lateral wind: the vehicle moves toward +Y relative to the air, so the force on it is toward -Y, of q S CN_alpha sin(alpha) cos(alpha) plus the small cross-flow term
  CHECK(l.f_aero.y < 0.0 && near_abs(l.f_aero.z, 0.0, 1e-9));
  CHECK(near_abs(-l.f_aero.y, q_s * cn * std::sin(alpha) * std::cos(alpha), 0.06 * q_s * cn * alpha));  // (the cross-flow term is of the order of alpha squared: a few percent here)
  // the moment about the centre of gravity: positive normal force at a centre of pressure behind the centre of gravity (x_cp < x_cg) turns the nose into the wind, i.e. a restoring moment
  const double x_cg = v.mass_props().x_cg;
  CHECK(xcp < x_cg);  // the fins put the centre of pressure behind the centre of gravity: a stable body
  const double arm = x_cg - xcp;
  CHECK(l.m_aero.z > 0.0);  // the body is moving toward +Y relative to the air, the force on it is toward -Y behind the centre of gravity: the nose is turned toward +Y, into the wind
  CHECK(near_abs(l.m_aero.z, q_s * cn * std::sin(alpha) * std::cos(alpha) * arm, 0.10 * q_s * cn * alpha * arm));
  CHECK(v.divergence() < 0.0);  // negative divergence: the body is aerodynamically stable
}

TFC_TEST(aero_vehicle_at_ninety_degrees_the_body_makes_the_crossflow_force_and_no_axial_force_and_tail_first_it_drags_at_the_rear_coefficient) {
  const sim::Params p = shaped_body();
  const sim::Vehicle6 side(p, at_speed(sim::V3{0.0, 150.0, 0.0}, 5000.0));  // the whole wind across the body
  const sim::Loads l = side.current_loads();
  double cn = 0.0;
  double xcp = 0.0;
  double ca = 0.0;
  double s_ref = 0.0;
  side.aero_summary(l.mach, cn, xcp, ca, s_ref);
  CHECK(near_abs(l.alpha, 0.5 * sim::kPi, 1e-12));
  CHECK(near_abs(l.f_aero.x, 0.0, 1e-6 * l.dynamic_pressure * s_ref));  // sin(90) cos(90) = 0 and the axial force goes as cos
  // the cross-flow force: q S eta Cdc (plan area / S): a body of 0.1 m by 2.0 m and four fins, plan area from the geometry
  const double plan = (0.1 * 1.5) + (0.5 * 0.1 * 0.5) + (0.5 * 4.0 * (0.5 * (0.2 + 0.1) * 0.12));
  CHECK(near_abs(-l.f_aero.y, l.dynamic_pressure * 0.7 * 1.2 * plan, 0.02 * l.dynamic_pressure * 0.7 * 1.2 * plan));
  // tail first: the wind comes from behind, the force pushes forward at the rear coefficient
  const sim::Vehicle6 back(p, at_speed(sim::V3{-150.0, 0.0, 0.0}, 5000.0));
  const sim::Loads lb = back.current_loads();
  CHECK(near_abs(lb.alpha, sim::kPi, 1e-12));
  CHECK(near_abs(lb.f_aero.x, lb.dynamic_pressure * s_ref * 1.1, 1e-6 * lb.dynamic_pressure * s_ref * 1.1));  // +q S CA_rear: toward +X, against the motion
  CHECK(near_abs(lb.f_aero.y, 0.0, 1e-6) && near_abs(lb.f_aero.z, 0.0, 1e-6));
}

TFC_TEST(aero_vehicle_the_force_is_continuous_through_ninety_degrees_and_its_normal_part_is_odd_about_it) {
  const sim::Params p = shaped_body();
  double previous_x = 0.0;
  double previous_y = 0.0;
  bool first = true;
  double max_jump = 0.0;
  for (int deg = 0; deg <= 180; deg += 3) {
    const double a = deg * sim::kDeg2Rad;
    const sim::Vehicle6 v(p, at_speed(sim::V3{150.0 * std::cos(a), 150.0 * std::sin(a), 0.0}, 5000.0));
    const sim::Loads l = v.current_loads();
    const double q = l.dynamic_pressure;
    if (!first) {
      max_jump = std::fmax(max_jump, std::fmax(std::fabs(l.f_aero.x - previous_x), std::fabs(l.f_aero.y - previous_y)) / (q * 0.0078));
    }
    previous_x = l.f_aero.x;
    previous_y = l.f_aero.y;
    first = false;
  }
  CHECK(max_jump < 2.0);  // no step of a 3 degree sweep larger than a coefficient of about two (the cross-flow term alone changes by 1.1): the force goes smoothly around the whole circle
}

TFC_TEST(aero_vehicle_the_aerodynamics_change_when_a_stage_separates) {
  sim::Params p = shaped_body();
  p.spec.stages[0].separate_time_s = 1.0;  // the whole first stage goes (the second is a bare tube with a nose)
  sim::StageSpec upper;
  upper.name = "upper";
  upper.dry_mass = 50.0;
  upper.x_start = 2.0;
  upper.length = 1.0;
  upper.radius = 0.05;
  upper.x_cg_dry = 2.5;
  upper.sections = {tube(2.0, 0.5, 0.1), nose(2.5, 0.5, 0.1, sim::NoseShape::Cone)};
  p.spec.stages.push_back(upper);
  p.gravity_scale = 0.0;
  sim::Vehicle6 v(p, at_speed(sim::V3{300.0, 0.0, 0.0}, 5000.0));
  double cn0 = 0.0;
  double cp0 = 0.0;
  double ca0 = 0.0;
  double s0 = 0.0;
  v.aero_summary(0.9, cn0, cp0, ca0, s0);
  while (v.time() < 1.5) {
    v.step(0.01, 0.0, 0.0);
  }
  double cn1 = 0.0;
  double cp1 = 0.0;
  double ca1 = 0.0;
  double s1 = 0.0;
  v.aero_summary(0.9, cn1, cp1, ca1, s1);
  CHECK(!v.stage_active(0) && v.stage_active(1));
  CHECK(cn1 < cn0 && cp1 > cp0 + 0.3);  // without the fins and the ogive of the first stage: less slope, and the centre of pressure forward, up in the cone
  CHECK(close(cn1, 2.0, 1e-9) && near_abs(cp1, 3.0 - (0.666 * 0.5), 1e-9));  // just a cone: slope 2, two thirds from the tip
}

TFC_TEST(aero_vehicle_a_table_by_mach_is_the_reference_models_own_aerodynamics_when_it_holds_the_same_numbers) {
  // the reference model's axial coefficient is 0.30 + 0.45 exp(-((M - 1.1)/0.35)^2), its slope 2.5 per radian at a centre of pressure of 12.5 m: tabulate that
  sim::Params simple;
  simple.ground_contact = false;
  simple.gravity_scale = 0.0;
  sim::Params tabled = simple;
  tabled.spec = sim::legacy_spec(tabled);
  tabled.spec.aero.full_angle = false;  // the reference model's own limits: linear in the angle, nothing past 90 degrees
  for (int i = 0; i <= 300; ++i) {
    const double m = 0.02 * i;
    tabled.spec.aero.table.push_back(sim::AeroTablePoint{m, 0.30 + (0.45 * std::exp(-std::pow((m - 1.1) / 0.35, 2.0))), 2.5, 12.5});
  }
  for (const double speed : {120.0, 250.0, 350.0, 500.0, 900.0}) {
    for (const double ang : {0.0, 0.02, 0.1}) {
      const sim::Scenario sc = at_speed(sim::V3{speed, speed * std::tan(ang), 0.0}, 8000.0);
      const sim::Vehicle6 a(simple, sc);
      const sim::Vehicle6 b(tabled, sc);
      const sim::Loads la = a.current_loads();
      const sim::Loads lb = b.current_loads();
      const double q_s = la.dynamic_pressure * sim::kPi * 0.25 * 1.8 * 1.8;
      CHECK(near_abs(la.f_aero.x, lb.f_aero.x, 2e-3 * q_s) && near_abs(la.f_aero.y, lb.f_aero.y, 2e-3 * q_s) && near_abs(la.m_aero.z, lb.m_aero.z, 2e-3 * q_s * 12.0));
    }
  }
  const sim::Vehicle6 a(simple, at_speed(sim::V3{-100.0, 0.0, 0.0}, 8000.0));
  const sim::Vehicle6 b(tabled, at_speed(sim::V3{-100.0, 0.0, 0.0}, 8000.0));
  CHECK(a.current_loads().f_aero.x == 0.0 && b.current_loads().f_aero.x == 0.0);  // and tail first both give nothing
}

// ---- the files ----

TFC_TEST(aero_files_read_and_write_a_shape_a_table_and_refuse_a_bad_one) {
  const std::string base = R"({
    "aero": {"diameter_m": 0.3, "c_n_alpha": 2.5, "x_cp_m": 1.5, "crossflow_cd": 1.0, "full_angle": true},
    "stages": [{"dry_mass_kg": 100, "length_m": 4, "radius_m": 0.15, "x_cg_dry_m": 2,
                "tanks": [{"propellant_kg": 50, "x_bottom_m": 0.3, "radius_m": 0.14, "density_kg_m3": 1750}],
                "sections": [{"kind": "tube", "x_start_m": 0, "length_m": 3, "d_aft_m": 0.3, "d_fore_m": 0.3},
                             {"kind": "nose", "x_start_m": 3, "length_m": 1, "d_aft_m": 0.3, "d_fore_m": 0, "shape": "ogive"}],
                "stabilizers": [{"count": 4, "x_le_root_m": 0.6, "root_chord_m": 0.4, "tip_chord_m": 0.2, "span_m": 0.25, "sweep_m": 0.2, "thickness_m": 0.005}]}],
    "engines": [{"stage": 0, "thrust_vac_n": 10000, "isp_vac_s": 250}]
  })";
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(base, v, errors) && errors.empty());
  const sim::StageSpec& st = v.params.spec.stages[0];
  CHECK(st.sections.size() == 2U && st.sections[1].kind == sim::SectionKind::Nose && st.sections[1].nose == sim::NoseShape::TangentOgive);
  CHECK(st.stabilizers.size() == 1U && st.stabilizers[0].count == 4 && near_abs(st.stabilizers[0].sweep, 0.2, 1e-12));
  CHECK(v.params.spec.aero.crossflow_cd == 1.0 && v.params.spec.aero.full_angle);
  const std::string text = sim::write_vehicle(v);
  sim::VehicleFile w;
  CHECK(sim::read_vehicle(text, w, errors) && errors.empty() && sim::write_vehicle(w) == text);
  // errors
  const auto bad_with = [&](const std::string& from, const std::string& to, const char* what) {
    std::string t = base;
    t.replace(t.find(from), from.size(), to);
    sim::VehicleFile x;
    std::vector<std::string> e;
    const bool ok = sim::read_vehicle(t, x, e);
    for (const std::string& m : e) {
      if (m.find(what) != std::string::npos) {
        return !ok;
      }
    }
    return false;
  };
  CHECK(bad_with("\"shape\": \"ogive\"", "\"shape\": \"spike\"", "expected \"cone\""));
  CHECK(bad_with("\"kind\": \"nose\"", "\"kind\": \"cone\"", "expected \"nose\""));
  CHECK(bad_with("\"d_fore_m\": 0, ", "\"d_fore_m\": 0.4, ", "is a nose"));
  CHECK(bad_with("\"d_fore_m\": 0.3}", "\"d_fore_m\": 0.2}", "is a tube"));
  CHECK(bad_with("\"span_m\": 0.25", "\"span_m\": 0", "stabilizers[0]"));
  CHECK(bad_with("\"crossflow_cd\": 1.0", "\"crossflow_cd\": -1", "aero:"));
  // a table by Mach
  const std::string tabled = R"({
    "aero": {"diameter_m": 0.3, "table": [[0, 0.3, 2.5, 1.5], [1, 0.6, 3.0, 1.6], [3, 0.4, 2.0, 1.7]]},
    "stages": [{"dry_mass_kg": 100, "length_m": 4, "radius_m": 0.15}]
  })";
  sim::VehicleFile t;
  errors.clear();
  CHECK(sim::read_vehicle(tabled, t, errors) && errors.empty() && t.params.spec.aero.table.size() == 3U);
  std::string unsorted = tabled;
  unsorted.replace(unsorted.find("[1, 0.6"), 7, "[0, 0.6");
  errors.clear();
  CHECK(!sim::read_vehicle(unsorted, t, errors));
  CHECK(!errors.empty() && errors[0].find("must increase") != std::string::npos);
  std::string both = base;
  both.replace(both.find("\"diameter_m\": 0.3,"), 18, "\"diameter_m\": 0.3, \"table\": [[0, 0.3, 2.5, 1.5]],");
  errors.clear();
  CHECK(!sim::read_vehicle(both, t, errors) && !errors.empty() && errors[0].find("not both") != std::string::npos);
}

// ---- the formulas at the points the first mutation sweep found nothing pinning ----

TFC_TEST(aero_the_fin_lift_slope_matches_the_formulas_by_hand_with_sweep_and_above_mach_one) {
  // subsonic, swept: 2 pi AR / (2 + sqrt(4 + AR^2 beta^2 (1 + tan^2(Lambda) / beta^2))) at AR 1.5, sweep 0.4 rad, Mach 0.5
  const double ar = 1.5;
  const double lambda = 0.4;
  const double beta2 = 1.0 - 0.25;
  const double t = std::tan(lambda);
  const double hand = 2.0 * sim::kPi * ar / (2.0 + std::sqrt(4.0 + (ar * ar * beta2 * (1.0 + (t * t / beta2)))));
  CHECK(close(sim::aero::wing_lift_slope(ar, lambda, 0.5), hand, 1e-12));
  CHECK(sim::aero::wing_lift_slope(ar, lambda, 0.5) < sim::aero::wing_lift_slope(ar, 0.0, 0.5));  // sweep lowers the slope
  // supersonic: (4 / beta)(1 - 1 / (2 AR beta)) for AR beta >= 1: aspect ratio 3, Mach 3
  const double beta = std::sqrt(8.0);
  CHECK(close(sim::aero::wing_lift_slope(3.0, 0.0, 3.0), (4.0 / beta) * (1.0 - (1.0 / (2.0 * 3.0 * beta))), 1e-12));
  // and the straight line through the transonic range: half way between the two ends at Mach 1.0
  CHECK(close(sim::aero::wing_lift_slope(ar, lambda, 1.0), 0.5 * (sim::aero::wing_lift_slope(ar, lambda, 0.8) + sim::aero::wing_lift_slope(ar, lambda, 1.2)), 1e-12));
}

TFC_TEST(aero_a_flat_front_face_and_a_flare_cost_what_they_are_said_to_at_mach_two) {
  const sim::AeroSpec spec;
  const double s_ref = sim::kPi * 0.04 / 4.0;
  // a plain tube, 0.2 m by 2.0 m, at Mach 2: skin, a flat face at 0.4 + 0.45 on its area, the supersonic base
  sim::AeroGeometry t;
  t.build({tube(0.0, 2.0, 0.2)}, {}, 0.0);
  const double cf = 0.455 / std::pow(std::log10(1.0e7), 2.58) * std::pow(1.0 + (0.144 * 4.0), -0.65);
  CHECK(close(t.axial(2.0, 1.0e7, 0.0, spec), (cf * sim::kPi * 0.2 * 2.0 / s_ref) + 0.85 + (0.25 / 2.0), 1e-12));
  // a flare (aft end larger) adds Newtonian wave drag 2 sin^2(half angle) (A_aft - A_fore) / S; a boat-tail (aft end smaller) adds none
  sim::AeroGeometry flare;
  flare.build({tube(1.0, 1.0, 0.2), transition(0.5, 0.5, 0.3, 0.2)}, {}, 0.0);  // reference diameter 0.3
  sim::AeroGeometry plain;
  plain.build({tube(1.0, 1.0, 0.2), tube(0.5, 0.5, 0.3)}, {}, 0.0);
  sim::AeroGeometry boat;
  boat.build({tube(1.0, 1.0, 0.3), transition(0.5, 0.5, 0.2, 0.3)}, {}, 0.0);
  sim::AeroGeometry boat_plain;
  boat_plain.build({tube(1.0, 1.0, 0.3), tube(0.5, 0.5, 0.2)}, {}, 0.0);
  const double s3 = sim::kPi * 0.09 / 4.0;
  const double half = std::atan2(0.5 * 0.1, 0.5);
  const double wave = 2.0 * std::pow(std::sin(half), 2.0) * ((sim::kPi * 0.09 / 4.0) - (sim::kPi * 0.04 / 4.0)) / s3;
  // (the skin of the slanted transition differs a little from a tube's: compare through the wave term, at Mach 2 where it is full: it is positive for the flare and absent for the boat-tail)
  CHECK(flare.axial(2.0, 1.0e7, 0.0, spec) - plain.axial(2.0, 1.0e7, 0.0, spec) > 0.5 * wave);
  CHECK(boat.axial(2.0, 1.0e7, 0.0, spec) - boat_plain.axial(2.0, 1.0e7, 0.0, spec) < 0.5 * wave);
}

TFC_TEST(aero_the_scale_factors_the_running_engines_the_table_and_the_fairing_act_on_the_force) {
  // cn_scale multiplies the normal force and cd_scale the axial force
  sim::Params base = shaped_body();
  sim::Params scaled = base;
  scaled.cn_scale = 1.5;
  scaled.cd_scale = 1.3;
  const sim::Scenario sc = at_speed(sim::V3{300.0, 300.0 * std::tan(0.01), 0.0}, 5000.0);
  const sim::Loads l0 = sim::Vehicle6(base, sc).current_loads();
  const sim::Loads l1 = sim::Vehicle6(scaled, sc).current_loads();
  CHECK(close(l1.f_aero.x, 1.3 * l0.f_aero.x, 1e-9));
  CHECK(l1.f_aero.y < 1.4 * l0.f_aero.y && l1.f_aero.y > 1.5 * l0.f_aero.y);  // the linear part is 1.5 times, the cross-flow part is not scaled: between 1 and 1.5 (both are negative)
  // power fraction is weighted by the thrust of the engines that run: 3000 N and 1000 N, the small one fails
  sim::Params p = shaped_body();
  p.spec.stages[0].tanks.push_back(sim::TankSpec{2.0, 0.1, 0.04, 1000.0, {}});
  sim::EngineSpec big;
  big.thrust_vac = 3000.0;
  big.exit_area = 0.0;
  sim::EngineSpec small = big;
  small.thrust_vac = 1000.0;
  p.spec.engines = {big, small};
  sim::Scenario sc2 = at_speed(sim::V3{300.0, 0.0, 0.0}, 5000.0);
  sim::Vehicle6 whole(p, sc2);
  sc2.engine_failures.push_back(sim::EngineFailure{0.0, 1});
  sim::Vehicle6 broken(p, sc2);
  broken.step(0.001, 0.0, 0.0);
  double cn = 0.0;
  double xcp = 0.0;
  double ca_whole = 0.0;
  double ca_broken = 0.0;
  double s_ref = 0.0;
  whole.aero_summary(0.9, cn, xcp, ca_whole, s_ref);
  broken.aero_summary(0.9, cn, xcp, ca_broken, s_ref);
  sim::Params cold = p;
  cold.spec.stages[0].ignite_time_s = 1.0e9;  // never lit: no power
  double ca_cold = 0.0;
  sim::Vehicle6(cold, sc2).aero_summary(0.9, cn, xcp, ca_cold, s_ref);
  const double base_term = (0.12 + (0.13 * 0.81)) * (sim::kPi * 0.25 * 0.01 / s_ref);  // base drag at Mach 0.9 on the base area (0.1 m body)
  CHECK(near_abs(ca_cold - ca_whole, 0.7 * base_term, 1e-9));           // all the thrust: 0.7 of the base drag gone
  CHECK(near_abs(ca_cold - ca_broken, 0.7 * 0.75 * base_term, 1e-9));   // three quarters of it running: three quarters of that
  // a table by Mach: the centre of pressure interpolates and is held beyond the ends; it is not the reference model's numbers
  sim::Params tp;
  tp.ground_contact = false;
  tp.gravity_scale = 0.0;
  tp.spec = sim::legacy_spec(tp);
  tp.spec.aero.table = {{0.0, 0.30, 4.0, 10.0}, {2.0, 0.50, 6.0, 14.0}};
  const sim::Vehicle6 tv(tp, at_speed(sim::V3{300.0, 0.0, 0.0}, 5000.0));
  double tcn = 0.0;
  double tcp = 0.0;
  double tca = 0.0;
  double ts = 0.0;
  tv.aero_summary(1.0, tcn, tcp, tca, ts);
  CHECK(close(tcn, 5.0, 1e-12) && close(tcp, 12.0, 1e-12) && close(tca, 0.40, 1e-12));  // half way
  tv.aero_summary(5.0, tcn, tcp, tca, ts);
  CHECK(close(tcn, 6.0, 1e-12) && close(tcp, 14.0, 1e-12) && close(tca, 0.50, 1e-12));  // held beyond the last point
  tv.aero_summary(-1.0, tcn, tcp, tca, ts);
  CHECK(close(tcn, 4.0, 1e-12) && close(tcp, 10.0, 1e-12));                              // and before the first
  // a fairing's shape leaves with it
  sim::Params fp;
  fp.ground_contact = false;
  fp.gravity_scale = 0.0;
  sim::StageSpec st;
  st.dry_mass = 200.0;
  st.length = 1.5;
  st.radius = 0.05;
  st.x_cg_dry = 0.7;
  st.sections = {tube(0.0, 1.5, 0.1)};
  fp.spec.stages.push_back(st);
  sim::PayloadSpec fairing;
  fairing.name = "fairing";
  fairing.mass = 10.0;
  fairing.x = 1.7;
  fairing.jettison_time_s = 1.0;
  fairing.sections = {nose(1.5, 0.5, 0.1, sim::NoseShape::Cone)};
  fp.spec.payloads.push_back(fairing);
  sim::Vehicle6 fv(fp, at_speed(sim::V3{300.0, 0.0, 0.0}, 5000.0));
  fv.aero_summary(0.9, tcn, tcp, tca, ts);
  CHECK(close(tcn, 2.0, 1e-9));  // with its fairing: a cone's slope
  while (fv.time() < 1.5) {
    fv.step(0.01, 0.0, 0.0);
  }
  fv.aero_summary(0.9, tcn, tcp, tca, ts);
  CHECK(near_abs(tcn, 0.0, 1e-12));  // without it: a bare tube has none
}
