// SPDX-License-Identifier: MIT
// Small vector and quaternion maths for the vehicle simulator (host only, double precision).
#pragma once
#include <cmath>

namespace sim {

struct V3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return a * s; }
inline V3 operator/(V3 a, double s) { return {a.x / s, a.y / s, a.z / s}; }
inline V3 operator-(V3 a) { return {-a.x, -a.y, -a.z}; }
inline double dot(V3 a, V3 b) { return (a.x * b.x) + (a.y * b.y) + (a.z * b.z); }
inline V3 cross(V3 a, V3 b) { return {(a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x)}; }
inline double norm(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 normalized(V3 a) {
  const double n = norm(a);
  return n > 0.0 ? a / n : V3{};
}

// A unit quaternion that takes body coordinates to inertial coordinates: v_inertial = q v_body q*.
struct Q4 {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Q4 operator*(Q4 a, Q4 b) {
  return {(a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z), (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
          (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x), (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)};
}
inline Q4 operator*(Q4 a, double s) { return {a.w * s, a.x * s, a.y * s, a.z * s}; }
inline Q4 operator+(Q4 a, Q4 b) { return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Q4 conj(Q4 a) { return {a.w, -a.x, -a.y, -a.z}; }
inline Q4 normalized(Q4 a) {
  const double n = std::sqrt((a.w * a.w) + (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
  return n > 0.0 ? a * (1.0 / n) : Q4{};
}
inline V3 rotate(Q4 q, V3 v) {  // body -> inertial
  const Q4 p = q * Q4{0.0, v.x, v.y, v.z} * conj(q);
  return {p.x, p.y, p.z};
}
inline V3 rotate_inv(Q4 q, V3 v) { return rotate(conj(q), v); }  // inertial -> body
inline Q4 from_axis_angle(V3 axis, double angle) {
  const V3 a = normalized(axis);
  const double s = std::sin(0.5 * angle);
  return {std::cos(0.5 * angle), a.x * s, a.y * s, a.z * s};
}
// The quaternion derivative for body rates w (rad/s, body frame).
inline Q4 q_dot(Q4 q, V3 w) { return (q * Q4{0.0, w.x, w.y, w.z}) * 0.5; }

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

}  // namespace sim
