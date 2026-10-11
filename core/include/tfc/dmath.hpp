// SPDX-License-Identifier: MIT
// Deterministic double-precision maths for the navigation and guidance of the flight computers: sine and cosine, arctangent, exponential and logarithm built from + - * / and sqrt only,
// with fixed-length loops, so that three replicas, the host and the target compute the same bits (ADR-006; the library's transcendental functions are not required to agree between
// implementations, and a guidance law that integrates a trajectory would amplify the difference). Small three-vectors, quaternions and 3x3 matrices of doubles, for the same reason.
// Accuracy is better than 1e-14 relative over the ranges the guidance uses (tests/test_dmath.cpp checks each function against the library).
// No heap, no exceptions, no RTTI, no global state. Written to docs/verification/CODING_STANDARD.md: every loop has a compile-time bound.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace tfc::dm {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.57079632679489661923;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kLn2 = 0.69314718055994530942;
constexpr double kLn2Hi = 6.93147180369123816490e-01;   // ln 2 in two parts (Cody and Waite): the first has trailing zero bits, so k * kLn2Hi is exact for whole k up to 2^11
constexpr double kLn2Lo = 1.90821492927058770002e-10;
constexpr double kG0 = 9.80665;              // standard gravity, m/s^2: the specific impulse's conversion and the accelerometer's g
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;


namespace detail {
// 1/n! for n = 0 .. N-1, computed at compile time (the Taylor coefficients of the series below).
template <std::size_t N>
constexpr std::array<double, N> inverse_factorials() noexcept {
  std::array<double, N> a{};
  a[0] = 1.0;
  double f = 1.0;
  for (std::size_t i = 1; i < N; ++i) {
    f *= static_cast<double>(i);
    a[i] = 1.0 / f;
  }
  return a;
}
constexpr std::array<double, 40> kInvFact = inverse_factorials<40>();

// The coefficients, in powers of x^2, of sin(x)/x and cos(x): (-1)^k / (2k+1)! and (-1)^k / (2k)!.
constexpr std::array<double, 9> sin_coefficients() noexcept {
  std::array<double, 9> c{};
  for (std::size_t k = 0; k < c.size(); ++k) {
    c[k] = ((k % 2U) == 0U ? 1.0 : -1.0) * kInvFact[(2U * k) + 1U];
  }
  return c;
}
constexpr std::array<double, 9> cos_coefficients() noexcept {
  std::array<double, 9> c{};
  for (std::size_t k = 0; k < c.size(); ++k) {
    c[k] = ((k % 2U) == 0U ? 1.0 : -1.0) * kInvFact[2U * k];
  }
  return c;
}
// atan(x)/x in powers of x^2: (-1)^k / (2k+1); atanh(x)/x: 1 / (2k+1).
template <bool Alternating>
constexpr std::array<double, 12> odd_reciprocals() noexcept {
  std::array<double, 12> c{};
  for (std::size_t k = 0; k < c.size(); ++k) {
    c[k] = (Alternating && (k % 2U) != 0U ? -1.0 : 1.0) / static_cast<double>((2U * k) + 1U);
  }
  return c;
}
constexpr std::array<double, 9> kSinC = sin_coefficients();
constexpr std::array<double, 9> kCosC = cos_coefficients();
constexpr std::array<double, 12> kAtanC = odd_reciprocals<true>();
constexpr std::array<double, 12> kAtanhC = odd_reciprocals<false>();
constexpr std::array<double, 17> kInvFactExp = [] {
  std::array<double, 17> c{};
  for (std::size_t k = 0; k < c.size(); ++k) {
    c[k] = kInvFact[k];
  }
  return c;
}();

// The polynomial c[0] + c[1] x + ... by Horner's rule.
template <std::size_t N>
constexpr double horner(const std::array<double, N>& c, double x) noexcept {
  double r = c[N - 1U];
  for (std::size_t i = 1; i < N; ++i) {
    r = (r * x) + c[N - 1U - i];
  }
  return r;
}
}  // namespace detail

constexpr double fabs_(double x) noexcept { return x < 0.0 ? -x : x; }
constexpr double min_(double a, double b) noexcept { return b < a ? b : a; }
constexpr double max_(double a, double b) noexcept { return a < b ? b : a; }
constexpr double clamp_(double x, double lo, double hi) noexcept { return x < lo ? lo : (x > hi ? hi : x); }
constexpr double sq_(double x) noexcept { return x * x; }
inline double sqrt_(double x) noexcept { return std::sqrt(x); }   // (correctly rounded by IEEE 754: the same on every implementation)

// floor for |x| < 2^52 (every argument the guidance forms is a few thousand at most); larger magnitudes are returned unchanged.
constexpr double floor_(double x) noexcept {
  if (!(fabs_(x) < 4503599627370496.0)) {
    return x;
  }
  const double t = static_cast<double>(static_cast<int64_t>(x));
  return t > x ? t - 1.0 : t;
}

// Sine and cosine: reduce to [-pi/4, pi/4] by whole quarter turns, then the Taylor series to degree 17 (error under 1e-19 on that interval).
constexpr void sincos_(double x, double& s, double& c) noexcept {
  const double turns = floor_((x / kTwoPi) + 0.5);
  const double r = x - (turns * kTwoPi);                  // -pi .. pi
  const double q = floor_((r / kHalfPi) + 0.5);           // -2 .. 2
  const double a = r - (q * kHalfPi);
  const double a2 = a * a;
  const double sn = a * detail::horner(detail::kSinC, a2);
  const double cs = detail::horner(detail::kCosC, a2);
  const int n = static_cast<int>(q) & 3;   // the quarter turns, counted modulo four (two's complement: -1 is 3, -2 is 2)
  if (n == 0) {
    s = sn;
    c = cs;
  } else if (n == 1) {   // a quarter turn
    s = cs;
    c = -sn;
  } else if (n == 2) {   // a half turn
    s = -sn;
    c = -cs;
  } else {               // three quarters
    s = -cs;
    c = sn;
  }
}

constexpr double sin_(double x) noexcept {
  double s = 0.0;
  double c = 0.0;
  sincos_(x, s, c);
  return s;
}
constexpr double cos_(double x) noexcept {
  double s = 0.0;
  double c = 0.0;
  sincos_(x, s, c);
  return c;
}

// Arctangent of t in [0, 1]: two halvings of the angle (atan t = 2 atan(t / (1 + sqrt(1 + t^2)))) bring the argument under 0.2, then the series to t^25.
inline double atan_unit_(double t) noexcept {
  const double h1 = t / (1.0 + std::sqrt(1.0 + (t * t)));
  const double h2 = h1 / (1.0 + std::sqrt(1.0 + (h1 * h1)));
  const double x2 = h2 * h2;
  const double s = h2 * detail::horner(detail::kAtanC, x2);
  return 4.0 * s;
}

// atan2(y, x) in (-pi, pi]; atan2(0, 0) is 0.
inline double atan2_(double y, double x) noexcept {
  const double ax = fabs_(x);
  const double ay = fabs_(y);
  const double mx = max_(ax, ay);
  if (!(mx > 0.0)) {
    return 0.0;
  }
  double r = atan_unit_(min_(ax, ay) / mx);
  if (ay > ax) {
    r = kHalfPi - r;
  }
  if (x < 0.0) {
    r = kPi - r;
  }
  return y < 0.0 ? -r : r;
}

inline double asin_(double x) noexcept {
  const double c = clamp_(x, -1.0, 1.0);
  return atan2_(c, std::sqrt(max_(1.0 - (c * c), 0.0)));
}
inline double acos_(double x) noexcept {
  const double c = clamp_(x, -1.0, 1.0);
  return atan2_(std::sqrt(max_(1.0 - (c * c), 0.0)), c);
}

// 2^k for a whole k, by squaring (|k| <= 1023).
constexpr double pow2_(int k) noexcept {
  const bool neg = k < 0;
  int n = neg ? -k : k;
  double base = 2.0;
  double p = 1.0;
  for (int i = 0; i < 11; ++i) {
    if ((n & 1) != 0) {
      p *= base;
    }
    base *= base;
    n >>= 1;
  }
  return neg ? 1.0 / p : p;
}

// exp(x): x = k ln2 + r with |r| <= ln2 / 2, the series to degree 16, scaled by 2^k. Arguments beyond +-700 are clamped there.
constexpr double exp_(double x) noexcept {
  const double xc = clamp_(x, -700.0, 700.0);
  if (!(xc >= -700.0)) {
    return xc;   // not a number: it stays one (a conversion of it to an integer below would be undefined)
  }
  const double k = floor_((xc / kLn2) + 0.5);
  const double r = (xc - (k * kLn2Hi)) - (k * kLn2Lo);
  const double s = detail::horner(detail::kInvFactExp, r);
  return s * pow2_(static_cast<int>(k));
}

// ln(x) for x > 0 (and a large negative number for anything else): x = m 2^e with m in [sqrt(1/2), sqrt(2)), ln m = 2 atanh((m - 1) / (m + 1)) by its series.
constexpr double log_(double x) noexcept {
  if (!(x > 0.0)) {
    return -1.0e300;
  }
  double m = x;
  int e = 0;
  for (int i = 0; i < 2200; ++i) {
    if (m >= 1.4142135623730951) {
      m *= 0.5;
      ++e;
    } else if (m < 0.7071067811865476) {
      m *= 2.0;
      --e;
    } else {
      break;
    }
  }
  const double s = (m - 1.0) / (m + 1.0);
  const double s2 = s * s;
  const double ln_m = 2.0 * s * detail::horner(detail::kAtanhC, s2);
  return (static_cast<double>(e) * kLn2) + ln_m;
}

// ---- three-vectors, quaternions, 3x3 matrices of doubles ----

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr Vec3 operator+(Vec3 a, Vec3 b) noexcept { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(Vec3 a, Vec3 b) noexcept { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator*(Vec3 a, double s) noexcept { return {a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator*(double s, Vec3 a) noexcept { return {a.x * s, a.y * s, a.z * s}; }
constexpr Vec3 operator/(Vec3 a, double s) noexcept { return {a.x / s, a.y / s, a.z / s}; }
constexpr Vec3 operator-(Vec3 a) noexcept { return {-a.x, -a.y, -a.z}; }
constexpr double dot(Vec3 a, Vec3 b) noexcept { return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); }
constexpr Vec3 cross(Vec3 a, Vec3 b) noexcept { return {(a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x)}; }
inline double norm(Vec3 a) noexcept { return std::sqrt(dot(a, a)); }
// The unit vector along a, or `fallback` if a is (nearly) zero.
inline Vec3 unit(Vec3 a, Vec3 fallback) noexcept {
  const double n = norm(a);
  return n > 1.0e-300 ? a / n : fallback;
}
// The part of a perpendicular to the unit vector u.
constexpr Vec3 perp(Vec3 a, Vec3 u) noexcept { return a - (u * dot(a, u)); }

// A unit quaternion: v_out = q v q*. w is the scalar part.
struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr Quat operator*(Quat a, Quat b) noexcept {
  return {(a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z), (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y), (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
          (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)};
}
constexpr Quat conj(Quat a) noexcept { return {a.w, -a.x, -a.y, -a.z}; }
inline Quat normalized(Quat a) noexcept {
  const double n = std::sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
  return n > 1.0e-300 ? Quat{a.w / n, a.x / n, a.y / n, a.z / n} : Quat{};
}
constexpr Vec3 rotate(Quat q, Vec3 v) noexcept {
  const Quat p = q * Quat{0.0, v.x, v.y, v.z} * conj(q);
  return {p.x, p.y, p.z};
}
constexpr Vec3 rotate_inv(Quat q, Vec3 v) noexcept { return rotate(conj(q), v); }

// The shortest rotation that takes the unit vector a to the unit vector b (a half turn about any axis perpendicular to a if they are opposite).
inline Quat between(Vec3 a, Vec3 b) noexcept {
  const double d = dot(a, b);
  if (d < -0.999999999999) {
    const Vec3 ax = unit(fabs_(a.x) < 0.9 ? cross(a, Vec3{1.0, 0.0, 0.0}) : cross(a, Vec3{0.0, 1.0, 0.0}), Vec3{0.0, 0.0, 1.0});
    return Quat{0.0, ax.x, ax.y, ax.z};
  }
  const Vec3 c = cross(a, b);
  return normalized(Quat{1.0 + d, c.x, c.y, c.z});
}

// A 3x3 matrix, row-major: m[3 * row + column].
struct Mat3 {
  std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};
constexpr Vec3 mul(const Mat3& a, Vec3 v) noexcept {
  return {(a.m[0] * v.x) + (a.m[1] * v.y) + (a.m[2] * v.z), (a.m[3] * v.x) + (a.m[4] * v.y) + (a.m[5] * v.z), (a.m[6] * v.x) + (a.m[7] * v.y) + (a.m[8] * v.z)};
}
// The rotation matrix of a unit quaternion (v_out = R v_in).
constexpr Mat3 matrix_of(Quat q) noexcept {
  Mat3 r;
  r.m = {1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))), 2.0 * ((q.x * q.y) - (q.w * q.z)), 2.0 * ((q.x * q.z) + (q.w * q.y)),
         2.0 * ((q.x * q.y) + (q.w * q.z)), 1.0 - (2.0 * ((q.x * q.x) + (q.z * q.z))), 2.0 * ((q.y * q.z) - (q.w * q.x)),
         2.0 * ((q.x * q.z) - (q.w * q.y)), 2.0 * ((q.y * q.z) + (q.w * q.x)), 1.0 - (2.0 * ((q.x * q.x) + (q.y * q.y)))};
  return r;
}
// The quaternion of a rotation matrix with columns the images of the axes (Shepperd's method: the largest of the four numerators is the one divided by).
inline Quat quat_of_columns(Vec3 cx, Vec3 cy, Vec3 cz) noexcept {
  const double m00 = cx.x;
  const double m11 = cy.y;
  const double m22 = cz.z;
  const double tr = m00 + m11 + m22;
  if (tr > m00 && tr > m11 && tr > m22) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    return normalized(Quat{0.25 * s, (cy.z - cz.y) / s, (cz.x - cx.z) / s, (cx.y - cy.x) / s});
  }
  if (m00 >= m11 && m00 >= m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    return normalized(Quat{(cy.z - cz.y) / s, 0.25 * s, (cy.x + cx.y) / s, (cz.x + cx.z) / s});
  }
  if (m11 >= m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    return normalized(Quat{(cz.x - cx.z) / s, (cy.x + cx.y) / s, 0.25 * s, (cz.y + cy.z) / s});
  }
  const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
  return normalized(Quat{(cx.y - cy.x) / s, (cz.x + cx.z) / s, (cz.y + cy.z) / s, 0.25 * s});
}

}  // namespace tfc::dm
