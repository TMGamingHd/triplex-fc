// SPDX-License-Identifier: MIT
// The three-axis attitude controller (core/include/tfc/attitude.hpp, docs/design/GNC.md section 5): the conversion from the estimator's quaternion to the body-to-navigation attitude, the reference attitude from a
// thrust direction and a roll hint, the error vector (the short way round, including through a half turn), the reference rate, the control law's signs and limits, and the closed loop against a rigid body that it must
// turn through 180 degrees without winding up.
#include <cmath>

#include "tfc_test.hpp"

#include "tfc/attitude.hpp"

namespace {
using tfc::dm::Quat;
using tfc::dm::Vec3;
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
double deg(double x) { return x * tfc::dm::kRadToDeg; }
Quat about(Vec3 axis, double angle) {
  double s = 0.0;
  double c = 0.0;
  tfc::dm::sincos_(0.5 * angle, s, c);
  const Vec3 a = tfc::dm::unit(axis, Vec3{1.0, 0.0, 0.0});
  return Quat{c, a.x * s, a.y * s, a.z * s};
}
}  // namespace

TFC_TEST(attitude_the_estimators_quaternion_gives_the_body_to_navigation_attitude) {
  // on the pad the sensor frame is the level frame: the body x axis (the long axis) is the navigation +X (up), body y downrange (+Y), body z crossrange (+Z)
  const Quat q = tfc::att::body_to_nav({1.0F, 0.0F, 0.0F, 0.0F});
  const Vec3 x = tfc::dm::rotate(q, Vec3{1.0, 0.0, 0.0});
  const Vec3 y = tfc::dm::rotate(q, Vec3{0.0, 1.0, 0.0});
  const Vec3 z = tfc::dm::rotate(q, Vec3{0.0, 0.0, 1.0});
  CHECK(near_abs(x.x, 1.0, 1e-12) && near_abs(y.y, 1.0, 1e-12) && near_abs(z.z, 1.0, 1e-12));
  // pitched over 90 degrees toward downrange (the sensor frame turned about its own Y by 90 degrees puts the long axis along the sensor X, downrange)
  const float s = static_cast<float>(std::sqrt(0.5));
  const Quat p = tfc::att::body_to_nav({s, 0.0F, s, 0.0F});
  const Vec3 px = tfc::dm::rotate(p, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(px.y, 1.0, 1e-6) && near_abs(px.x, 0.0, 1e-6));
  // the rates: the estimator's sensor-frame rates (x, y, z) are the body's (y, z, x)
  const Vec3 w = tfc::att::body_rates_rad({10.0F, 20.0F, 30.0F});
  CHECK(near_abs(w.x, 30.0 * tfc::dm::kDegToRad, 1e-12) && near_abs(w.y, 10.0 * tfc::dm::kDegToRad, 1e-12) && near_abs(w.z, 20.0 * tfc::dm::kDegToRad, 1e-12));
}

TFC_TEST(attitude_the_reference_puts_the_long_axis_where_asked_and_the_roll_where_the_hint_says) {
  const Vec3 dir = tfc::dm::unit(Vec3{1.0, 1.0, 0.0}, Vec3{});
  const Quat q = tfc::att::attitude_from_axes(dir, Vec3{0.0, 0.0, 1.0});
  const Vec3 x = tfc::dm::rotate(q, Vec3{1.0, 0.0, 0.0});
  const Vec3 z = tfc::dm::rotate(q, Vec3{0.0, 0.0, 1.0});
  const Vec3 y = tfc::dm::rotate(q, Vec3{0.0, 1.0, 0.0});
  CHECK(tfc::dm::norm(x - dir) < 1e-12 && near_abs(z.z, 1.0, 1e-12));
  CHECK(tfc::dm::norm(tfc::dm::cross(x, y) - z) < 1e-12);   // right-handed
  // the long axis along the hint: any perpendicular will do, and it is still a proper rotation
  const Quat a = tfc::att::attitude_from_axes(Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 1.0});
  const Vec3 ax = tfc::dm::rotate(a, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(ax.z, 1.0, 1e-12) && near_abs(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z, 1.0, 1e-12));
  const Quat b = tfc::att::attitude_from_axes(Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(tfc::dm::rotate(b, Vec3{1.0, 0.0, 0.0}).x, 1.0, 1e-12));
  const Quat c = tfc::att::attitude_from_axes(Vec3{}, Vec3{0.0, 0.0, 1.0});   // no direction: the default one
  CHECK(near_abs(tfc::dm::rotate(c, Vec3{1.0, 0.0, 0.0}).x, 1.0, 1e-12));
}

TFC_TEST(attitude_the_error_vector_is_the_short_rotation_in_body_axes_including_through_a_half_turn) {
  const Quat q = about(Vec3{0.0, 0.0, 1.0}, 0.3);
  const Quat r = about(Vec3{0.0, 0.0, 1.0}, 0.5);
  const Vec3 e = tfc::att::error_vector(q, r);
  CHECK(near_abs(e.z, 0.2, 1e-12) && near_abs(e.x, 0.0, 1e-12) && near_abs(e.y, 0.0, 1e-12));
  // the same rotation expressed in the body frame of a turned vehicle: about its own y axis
  const Quat tilt = about(Vec3{1.0, 0.0, 0.0}, 1.0);   // rolled by a radian
  const Quat qa = tilt;
  const Quat qr = tilt * about(Vec3{0.0, 1.0, 0.0}, 0.1);
  const Vec3 e2 = tfc::att::error_vector(qa, qr);
  CHECK(near_abs(e2.y, 0.1, 1e-12) && near_abs(e2.x, 0.0, 1e-12) && near_abs(e2.z, 0.0, 1e-12));
  // a negative scalar part (the quaternion's other sign for the same rotation) gives the same error, not the long way round
  const Quat neg{-r.w, -r.x, -r.y, -r.z};
  const Vec3 e3 = tfc::att::error_vector(q, neg);
  CHECK(near_abs(e3.z, 0.2, 1e-12));
  // 180 degrees: the vector has the length of the half turn (pi), along the axis
  const Vec3 half = tfc::att::error_vector(Quat{}, about(Vec3{0.0, 1.0, 0.0}, tfc::dm::kPi - 1e-6));
  CHECK(near_abs(half.y, tfc::dm::kPi - 1e-6, 1e-4));
  // the rate of a reference
  const Vec3 w = tfc::att::reference_rate(q, r, 0.1);
  CHECK(near_abs(w.z, 2.0, 1e-12));
  CHECK(tfc::dm::norm(tfc::att::reference_rate(q, r, 0.0)) == 0.0);
}

TFC_TEST(attitude_the_control_law_has_the_signs_and_limits_it_is_written_with) {
  tfc::att::Controller3 c;
  tfc::att::Gains3 g;
  g.pitch = {2.0F, 1.0F, 0.0F};
  g.yaw = {3.0F, 0.0F, 0.0F};
  g.roll = {1.0F, 0.0F, 0.0F};
  tfc::att::Limits lim;
  lim.slew_deg_per_frame = 100.0F;
  // a reference 2 degrees ahead about z (pitch), 1 about y (yaw), 4 about x (roll): the commands are positive and proportional
  const Quat q;
  const Quat r = about(Vec3{0.0, 0.0, 1.0}, 2.0 * tfc::dm::kDegToRad) * about(Vec3{0.0, 1.0, 0.0}, 1.0 * tfc::dm::kDegToRad) * about(Vec3{1.0, 0.0, 0.0}, 4.0 * tfc::dm::kDegToRad);
  const tfc::att::Demand d = c.step(q, Vec3{}, r, Vec3{}, g, lim, 0.01, true);
  CHECK(near_abs(d.pitch, 2.0 * c.error_deg().z, 1e-9) && near_abs(d.yaw, 3.0 * c.error_deg().y, 1e-9) && near_abs(d.roll, 1.0 * c.error_deg().x, 1e-9));   // the gains times the errors
  CHECK(d.pitch > 0.0 && d.yaw > 0.0 && d.roll > 0.0);
  CHECK(near_abs(c.error_deg().z, 2.0, 0.15) && near_abs(c.error_deg().y, 1.0, 0.15) && near_abs(c.error_deg().x, 4.0, 0.15));
  // rate damping: spinning at +1 deg/s about z against a still reference gives a negative pitch command of kd times that
  tfc::att::Controller3 c2;
  const tfc::att::Demand d2 = c2.step(q, Vec3{0.0, 0.0, 1.0 * tfc::dm::kDegToRad}, q, Vec3{}, g, lim, 0.01, true);
  CHECK(near_abs(d2.pitch, -1.0, 1e-9) && near_abs(d2.yaw, 0.0, 1e-12));
  // a reference that is itself turning at that rate asks for nothing
  tfc::att::Controller3 c3;
  CHECK(near_abs(c3.step(q, Vec3{0.0, 0.0, 1.0 * tfc::dm::kDegToRad}, q, Vec3{0.0, 0.0, 1.0 * tfc::dm::kDegToRad}, g, lim, 0.01, true).pitch, 0.0, 1e-9));
  // limits: the command stops at the limit and the integrator does not wind up behind it
  tfc::att::Controller3 c4;
  tfc::att::Gains3 gi;
  gi.pitch = {10.0F, 0.0F, 5.0F};
  tfc::att::Limits tight;
  tight.command_deg = 4.0F;
  tight.integrator_deg = 3.0F;
  tight.slew_deg_per_frame = 100.0F;
  const Quat far = about(Vec3{0.0, 0.0, 1.0}, 0.5);
  for (int k = 0; k < 500; ++k) {
    (void)c4.step(q, Vec3{}, far, Vec3{}, gi, tight, 0.01, true);
  }
  CHECK(near_abs(c4.output().pitch, 4.0, 1e-12) && c4.saturated_frames() > 0U && std::fabs(c4.state().integ[2]) < 1e-9);
  // the slew limit
  tfc::att::Controller3 c5;
  tfc::att::Limits slow;
  slow.slew_deg_per_frame = 0.5F;
  const tfc::att::Demand ds = c5.step(q, Vec3{}, far, Vec3{}, gi, slow, 0.01, true);
  CHECK(near_abs(ds.pitch, 0.5, 1e-12));
  // an invalid estimate holds the command and counts; the state can be taken and put back; the integrators can be cleared
  const tfc::att::Demand held = c5.step(q, Vec3{}, far, Vec3{}, gi, slow, 0.01, false);
  CHECK(held.pitch == ds.pitch && c5.holds() == 1U);
  tfc::att::Controller3 c6;
  c6.set_state(c5.state());
  CHECK(c6.output().pitch == c5.output().pitch && c6.digest() == c5.digest());
  c6.clear_integrators();
  CHECK(c6.state().integ[2] == 0.0);
  const tfc::att::Demand moved = c6.step(q, Vec3{}, far, Vec3{}, gi, slow, 0.01, true);
  CHECK(moved.pitch != held.pitch && c6.digest() != c5.digest());
}

TFC_TEST(attitude_a_rigid_body_is_turned_through_a_half_circle_and_held_without_winding_up) {
  // a body with an inertia and a control effectiveness: angular acceleration = b * command (rad/s^2 per degree), command limited to 8 degrees, about its z axis; the reference is 170 degrees away
  const double b = 0.02;                       // rad/s^2 per degree of command
  const double wn = 0.5;
  const double kp = wn * wn / b;               // degrees of command per degree of error... the loop is s^2 + b kd s + b kp (in these units)
  const double kd = 2.0 * 0.9 * wn / b;
  tfc::att::Gains3 g;
  g.pitch = {static_cast<float>(kp), static_cast<float>(kd), static_cast<float>(0.1 * kp)};
  tfc::att::Limits lim;
  lim.command_deg = 8.0F;
  lim.slew_deg_per_frame = 100.0F;
  tfc::att::Controller3 c;
  Quat q;
  Vec3 w{};
  const Quat target = about(Vec3{0.0, 0.0, 1.0}, 170.0 * tfc::dm::kDegToRad);
  double worst_after = 0.0;
  for (int k = 0; k < 6000; ++k) {
    const tfc::att::Demand d = c.step(q, w, target, Vec3{}, g, lim, 0.01, true);
    w.z += b * d.pitch * 0.01;
    q = tfc::dm::normalized(q * about(Vec3{0.0, 0.0, 1.0}, w.z * 0.01));
    if (k > 4000) {
      worst_after = std::fmax(worst_after, std::fabs(deg(tfc::att::error_vector(q, target).z)));
    }
  }
  CHECK(worst_after < 0.2);
  CHECK(std::fabs(deg(w.z)) < 0.05);
}
