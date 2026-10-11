// SPDX-License-Identifier: MIT
// The deterministic double-precision maths of the guidance (core/include/tfc/dmath.hpp): each function against the library over the range the guidance uses, the identities they must keep
// (sin^2 + cos^2, exp(log x) = x, atan2 over the full circle), the quadrant handling and the clamps; the vector, quaternion and matrix helpers against hand calculations.
#include <cmath>

#include "tfc_test.hpp"

#include "tfc/dmath.hpp"

namespace {
bool rel(double a, double b, double r) { return std::fabs(a - b) <= r * std::fmax(1.0e-300, std::fmax(std::fabs(a), std::fabs(b))); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
}  // namespace

TFC_TEST(dmath_sine_and_cosine_agree_with_the_library_over_many_turns) {
  double worst = 0.0;
  for (int i = -4000; i <= 4000; ++i) {
    const double x = i * 0.0173;   // about +-70 rad
    double s = 0.0;
    double c = 0.0;
    tfc::dm::sincos_(x, s, c);
    worst = std::fmax(worst, std::fmax(std::fabs(s - std::sin(x)), std::fabs(c - std::cos(x))));
    CHECK(near_abs((s * s) + (c * c), 1.0, 1e-14));
  }
  CHECK(worst < 2e-14);
  CHECK(tfc::dm::sin_(0.5) == tfc::dm::sin_(0.5) && near_abs(tfc::dm::cos_(tfc::dm::kPi), -1.0, 1e-15) && near_abs(tfc::dm::sin_(tfc::dm::kHalfPi), 1.0, 1e-15));
}

TFC_TEST(dmath_arctangent_and_the_inverse_sine_and_cosine_cover_the_circle) {
  double worst = 0.0;
  for (int i = -180; i <= 180; ++i) {
    const double a = i * tfc::dm::kDegToRad;
    const double y = std::sin(a) * 3.7;
    const double x = std::cos(a) * 3.7;
    worst = std::fmax(worst, std::fabs(tfc::dm::atan2_(y, x) - std::atan2(y, x)));
  }
  CHECK(worst < 2e-15 * 4.0);
  CHECK(tfc::dm::atan2_(0.0, 0.0) == 0.0);
  CHECK(near_abs(tfc::dm::atan2_(0.0, -1.0), tfc::dm::kPi, 1e-15) && near_abs(tfc::dm::atan2_(-0.0, 1.0), 0.0, 1e-15));
  CHECK(near_abs(tfc::dm::atan2_(1.0, 0.0), tfc::dm::kHalfPi, 1e-15) && near_abs(tfc::dm::atan2_(-1.0, 0.0), -tfc::dm::kHalfPi, 1e-15));
  for (int i = -100; i <= 100; ++i) {
    const double x = i * 0.01;
    CHECK(near_abs(tfc::dm::asin_(x), std::asin(x), 1e-14) && near_abs(tfc::dm::acos_(x), std::acos(x), 1e-14));
  }
  CHECK(near_abs(tfc::dm::asin_(1.5), tfc::dm::kHalfPi, 1e-15) && near_abs(tfc::dm::acos_(-1.5), tfc::dm::kPi, 1e-15));   // clamped, not NaN
}

TFC_TEST(dmath_exponential_and_logarithm_agree_with_the_library_and_invert_each_other) {
  for (int i = -600; i <= 600; ++i) {
    const double x = i * 0.1;
    CHECK(rel(tfc::dm::exp_(x), std::exp(x), 5e-15));
  }
  for (int i = -300; i <= 300; ++i) {
    const double x = std::pow(10.0, i * 0.1);
    CHECK(near_abs(tfc::dm::log_(x), std::log(x), 5e-14 * std::fmax(1.0, std::fabs(std::log(x)))));
    CHECK(rel(tfc::dm::exp_(tfc::dm::log_(x)), x, 5e-13));
  }
  CHECK(tfc::dm::log_(0.0) < -1e299 && tfc::dm::log_(-3.0) < -1e299);
  CHECK(tfc::dm::exp_(1000.0) == tfc::dm::exp_(700.0) && tfc::dm::exp_(-1000.0) == tfc::dm::exp_(-700.0));
  CHECK(tfc::dm::exp_(0.0) == 1.0 && rel(tfc::dm::pow2_(10), 1024.0, 1e-15) && rel(tfc::dm::pow2_(-3), 0.125, 1e-15) && tfc::dm::pow2_(0) == 1.0);
}

TFC_TEST(dmath_floor_clamp_min_max_behave) {
  CHECK(tfc::dm::floor_(2.7) == 2.0 && tfc::dm::floor_(-2.7) == -3.0 && tfc::dm::floor_(-3.0) == -3.0 && tfc::dm::floor_(0.0) == 0.0);
  CHECK(tfc::dm::floor_(1.0e30) == 1.0e30);
  CHECK(tfc::dm::clamp_(5.0, 0.0, 3.0) == 3.0 && tfc::dm::clamp_(-5.0, -1.0, 3.0) == -1.0 && tfc::dm::clamp_(1.0, 0.0, 3.0) == 1.0);
  CHECK(tfc::dm::min_(1.0, 2.0) == 1.0 && tfc::dm::max_(1.0, 2.0) == 2.0 && tfc::dm::fabs_(-2.0) == 2.0 && tfc::dm::fabs_(2.0) == 2.0 && tfc::dm::sq_(3.0) == 9.0);
}

TFC_TEST(dmath_vectors_quaternions_and_matrices) {
  using namespace tfc::dm;
  const Vec3 a{1.0, 2.0, 3.0};
  const Vec3 b{-2.0, 0.5, 4.0};
  CHECK(dot(a, b) == 11.0);
  const Vec3 c = cross(a, b);
  CHECK(c.x == 6.5 && c.y == -10.0 && c.z == 4.5 && dot(c, a) == 0.0 && dot(c, b) == 0.0);
  CHECK((a + b).x == -1.0 && (a - b).y == 1.5 && (a * 2.0).z == 6.0 && (2.0 * a).z == 6.0 && (a / 2.0).y == 1.0 && (-a).x == -1.0);
  CHECK(near_abs(norm(unit(a, Vec3{})), 1.0, 1e-15));
  const Vec3 fb = unit(Vec3{}, Vec3{0.0, 0.0, 1.0});
  CHECK(fb.z == 1.0);
  const Vec3 p = perp(a, Vec3{0.0, 0.0, 1.0});
  CHECK(p.x == 1.0 && p.y == 2.0 && p.z == 0.0);
  // a quarter turn about z takes x to y
  double s = 0.0;
  double cc = 0.0;
  sincos_(0.25 * kPi, s, cc);
  const Quat q{cc, 0.0, 0.0, s};
  const Vec3 r = rotate(q, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(r.x, 0.0, 1e-15) && near_abs(r.y, 1.0, 1e-15) && near_abs(r.z, 0.0, 1e-15));
  const Vec3 back = rotate_inv(q, r);
  CHECK(near_abs(back.x, 1.0, 1e-15) && near_abs(back.y, 0.0, 1e-15));
  const Mat3 m = matrix_of(q);
  const Vec3 rm = mul(m, Vec3{1.0, 0.0, 0.0});
  CHECK(near_abs(rm.x, r.x, 1e-15) && near_abs(rm.y, r.y, 1e-15));
  CHECK(near_abs(normalized(Quat{2.0, 0.0, 0.0, 0.0}).w, 1.0, 1e-15) && normalized(Quat{0.0, 0.0, 0.0, 0.0}).w == 1.0);
  // the shortest rotation between two directions, including opposite ones and the second branch of the choice of the axis
  for (const Vec3 from : {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.6, 0.0, 0.8}}) {
    for (const Vec3 to : {Vec3{0.0, 0.0, 1.0}, Vec3{-1.0, 0.0, 0.0}, Vec3{0.0, -1.0, 0.0}, Vec3{0.6, 0.0, 0.8}, Vec3{-0.6, 0.0, -0.8}}) {
      const Vec3 got = rotate(between(from, to), from);
      CHECK(near_abs(got.x, to.x, 1e-12) && near_abs(got.y, to.y, 1e-12) && near_abs(got.z, to.z, 1e-12));
    }
  }
}

TFC_TEST(dmath_a_quaternion_round_trips_through_its_matrix_in_every_branch_of_the_conversion) {
  using namespace tfc::dm;
  // attitudes chosen so that each of the four numerators is the largest in turn
  const Quat cases[] = {{1.0, 0.0, 0.0, 0.0},       normalized(Quat{0.1, 0.9, 0.3, 0.2}), normalized(Quat{0.1, 0.2, 0.9, 0.3}),
                        normalized(Quat{0.1, 0.2, 0.3, 0.9}), normalized(Quat{0.7, 0.3, -0.5, 0.4}), normalized(Quat{0.0, 0.0, 1.0, 0.0})};
  for (const Quat& q : cases) {
    const Mat3 m = matrix_of(q);
    const Vec3 cx = mul(m, Vec3{1.0, 0.0, 0.0});
    const Vec3 cy = mul(m, Vec3{0.0, 1.0, 0.0});
    const Vec3 cz = mul(m, Vec3{0.0, 0.0, 1.0});
    const Quat r = quat_of_columns(cx, cy, cz);
    const double same = std::fabs(r.w * q.w + r.x * q.x + r.y * q.y + r.z * q.z);   // q and -q are the same rotation
    CHECK(near_abs(same, 1.0, 1e-12));
  }
}

TFC_TEST(dmath_the_exponential_of_a_number_that_is_not_one_is_not_one) {
  CHECK(!(tfc::dm::exp_(std::nan("")) == tfc::dm::exp_(std::nan(""))));   // NaN in, NaN out, and no undefined conversion on the way
  CHECK(tfc::dm::exp_(-1.0e300) < 1e-300 && tfc::dm::exp_(1.0e300) > 1e300);
}
