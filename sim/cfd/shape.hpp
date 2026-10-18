// SPDX-License-Identifier: MIT
// What the flow solver's body is: a signed distance to its surface (negative inside) and a list of the surface elements the pressure is integrated on (docs/design/CFD.md section 3).
// A body of revolution from a profile (the vehicle's own sections), a sphere, a half-space and an axis-aligned box, and their union and intersection. The surface elements of a body that is symmetric about
// the plane z = 0 are given for z >= 0 only: the solver adds their mirror images.
#pragma once
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "math3.hpp"

namespace sim::cfd {

// A piece of the surface: where it is, which way it faces (out of the body, into the fluid), and how big.
struct Panel {
  V3 c;
  V3 n;
  double area = 0.0;
  int part = 0;   // 0: the skin; 1: a flat end (the base of a stack, where an Euler solution has no wake and its pressure means little)
};

class Shape {
 public:
  Shape() = default;
  Shape(const Shape&) = delete;
  Shape& operator=(const Shape&) = delete;
  virtual ~Shape() = default;
  // The distance to the surface, negative inside the body.
  [[nodiscard]] virtual double phi(V3 p) const = 0;
  // The surface elements, about `h` across, for z >= 0 (the solver mirrors them).
  virtual void panels(double h, std::vector<Panel>& out) const = 0;
  // The unit normal of the surface at the closest point, from the gradient of the distance.
  [[nodiscard]] virtual V3 normal(V3 p) const {
    constexpr double e = 1e-4;
    const V3 g{phi(V3{p.x + e, p.y, p.z}) - phi(V3{p.x - e, p.y, p.z}), phi(V3{p.x, p.y + e, p.z}) - phi(V3{p.x, p.y - e, p.z}), phi(V3{p.x, p.y, p.z + e}) - phi(V3{p.x, p.y, p.z - e})};
    return normalized(g);
  }
};

// A sphere.
class Sphere final : public Shape {
 public:
  Sphere(V3 centre, double radius) : c_(centre), r_(radius) {}
  [[nodiscard]] double phi(V3 p) const override { return norm(p - c_) - r_; }
  [[nodiscard]] V3 normal(V3 p) const override { return normalized(p - c_); }
  void panels(double h, std::vector<Panel>& out) const override {
    const int nt = std::max(8, static_cast<int>(std::ceil(3.14159265358979 * r_ / h)));   // latitude bands from pole to pole; the longitude over half a turn (z >= 0)
    for (int i = 0; i < nt; ++i) {
      const double a0 = 3.14159265358979 * i / nt;
      const double a1 = 3.14159265358979 * (i + 1) / nt;
      const double am = 0.5 * (a0 + a1);
      const int np = std::max(4, static_cast<int>(std::ceil(3.14159265358979 * r_ * std::sin(am) / h)));
      for (int j = 0; j < np; ++j) {
        const double b0 = 3.14159265358979 * j / np;
        const double b1 = 3.14159265358979 * (j + 1) / np;
        const double bm = 0.5 * (b0 + b1);
        const V3 n{std::cos(am), std::sin(am) * std::cos(bm), std::sin(am) * std::sin(bm)};
        out.push_back(Panel{c_ + (n * r_), n, r_ * r_ * (b1 - b0) * (std::cos(a0) - std::cos(a1))});
      }
    }
  }

 private:
  V3 c_;
  double r_;
};

// The ground on one side of a plane: solid where (p - point) . normal < 0.
class HalfSpace final : public Shape {
 public:
  HalfSpace(V3 point, V3 normal_out) : p0_(point), n_(normalized(normal_out)) {}
  [[nodiscard]] double phi(V3 p) const override { return dot(p - p0_, n_); }
  [[nodiscard]] V3 normal(V3) const override { return n_; }
  void panels(double, std::vector<Panel>&) const override {}

 private:
  V3 p0_;
  V3 n_;
};

// An axis-aligned box.
class Box final : public Shape {
 public:
  Box(V3 lo, V3 hi) : lo_(lo), hi_(hi) {}
  [[nodiscard]] double phi(V3 p) const override {
    const V3 c = (lo_ + hi_) * 0.5;
    const V3 h = (hi_ - lo_) * 0.5;
    const V3 q{std::fabs(p.x - c.x) - h.x, std::fabs(p.y - c.y) - h.y, std::fabs(p.z - c.z) - h.z};
    const V3 o{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
    return norm(o) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0);
  }
  void panels(double h, std::vector<Panel>& out) const override {
    const V3 s = hi_ - lo_;
    auto face = [&](int axis, double sign) {
      const int a1 = (axis + 1) % 3;
      const int a2 = (axis + 2) % 3;
      auto comp = [](const V3& v, int a) { return a == 0 ? v.x : (a == 1 ? v.y : v.z); };
      const int n1 = std::max(1, static_cast<int>(std::ceil(comp(s, a1) / h)));
      const int n2 = std::max(1, static_cast<int>(std::ceil(comp(s, a2) / h)));
      for (int i = 0; i < n1; ++i) {
        for (int j = 0; j < n2; ++j) {
          V3 c;
          auto put = [](V3& v, int a, double x) { (a == 0 ? v.x : (a == 1 ? v.y : v.z)) = x; };
          put(c, axis, sign > 0.0 ? comp(hi_, axis) : comp(lo_, axis));
          put(c, a1, comp(lo_, a1) + ((i + 0.5) * comp(s, a1) / n1));
          put(c, a2, comp(lo_, a2) + ((j + 0.5) * comp(s, a2) / n2));
          V3 n;
          put(n, axis, sign);
          if (c.z < 0.0) {
            continue;   // the mirror image of the half z >= 0 is added by the solver (a box that straddles the plane has its z < 0 half cut off here and the other half doubled: it must be symmetric)
          }
          out.push_back(Panel{c, n, comp(s, a1) / n1 * (comp(s, a2) / n2)});
        }
      }
    };
    for (int a = 0; a < 3; ++a) {
      face(a, 1.0);
      face(a, -1.0);
    }
  }

 private:
  V3 lo_;
  V3 hi_;
};

// A body of revolution about the x axis: `profile` is the radius at stations along x from the first to the last (the tip, the sides, the base). The closed outline is the profile and the axis.
class Revolution final : public Shape {
 public:
  explicit Revolution(std::vector<std::pair<double, double>> profile) : pr_(std::move(profile)), poly_(pr_) {
    if (pr_.back().second > 0.0) {
      poly_.emplace_back(pr_.back().first, 0.0);
    }
    if (pr_.front().second > 0.0) {
      poly_.emplace_back(pr_.front().first, 0.0);
    }
  }

  [[nodiscard]] double phi(V3 p) const override {
    const double rho = std::sqrt((p.y * p.y) + (p.z * p.z));
    double best = 1e300;
    for (std::size_t i = 0; i + 1 < pr_.size(); ++i) {
      best = std::min(best, seg_dist(p.x, rho, pr_[i], pr_[i + 1]));
    }
    // the cap at each end where the profile does not close on the axis
    if (pr_.front().second > 0.0) {
      best = std::min(best, seg_dist(p.x, rho, std::make_pair(pr_.front().first, 0.0), pr_.front()));
    }
    if (pr_.back().second > 0.0) {
      best = std::min(best, seg_dist(p.x, rho, pr_.back(), std::make_pair(pr_.back().first, 0.0)));
    }
    return inside(p.x, rho) ? -best : best;
  }

  void panels(double h, std::vector<Panel>& out) const override {
    auto band = [&](std::pair<double, double> a, std::pair<double, double> b, int part) {
      const double dx = b.first - a.first;
      const double dr = b.second - a.second;
      const double len = std::hypot(dx, dr);
      if (len <= 0.0) {
        return;
      }
      const int ns = std::max(1, static_cast<int>(std::ceil(len / h)));
      for (int s = 0; s < ns; ++s) {
        const double f0 = static_cast<double>(s) / ns;
        const double f1 = static_cast<double>(s + 1) / ns;
        const double r0 = a.second + (f0 * dr);
        const double r1 = a.second + (f1 * dr);
        const double xm = a.first + (0.5 * (f0 + f1) * dx);
        const double rm = 0.5 * (r0 + r1);
        if (rm <= 0.0) {
          continue;
        }
        // the normal in the meridian plane, the way that leaves the body
        double nx = dr / len;
        double nr = -dx / len;
        if (phi(V3{xm + (nx * 1e-3 * h), rm + (nr * 1e-3 * h), 0.0}) < 0.0) {
          nx = -nx;
          nr = -nr;
        }
        const int nt = std::max(8, static_cast<int>(std::ceil(3.14159265358979 * rm / h)));
        const double seg_len = len / ns;
        for (int t = 0; t < nt; ++t) {
          const double th = 3.14159265358979 * (t + 0.5) / nt;
          out.push_back(Panel{V3{xm, rm * std::cos(th), rm * std::sin(th)}, V3{nx, nr * std::cos(th), nr * std::sin(th)}, (3.14159265358979 / nt) * rm * seg_len, part});
        }
      }
    };
    for (std::size_t i = 0; i + 1 < pr_.size(); ++i) {
      band(pr_[i], pr_[i + 1], 0);
    }
    if (pr_.front().second > 0.0) {
      band(std::make_pair(pr_.front().first, 0.0), pr_.front(), 1);
    }
    if (pr_.back().second > 0.0) {
      band(pr_.back(), std::make_pair(pr_.back().first, 0.0), 1);
    }
  }

  [[nodiscard]] double x_min() const { return pr_.front().first; }
  [[nodiscard]] double x_max() const { return pr_.back().first; }
  [[nodiscard]] double r_max() const {
    double r = 0.0;
    for (const auto& q : pr_) {
      r = std::max(r, q.second);
    }
    return r;
  }

 private:
  // The distance from the point (x, rho) in the meridian plane to the segment a-b.
  static double seg_dist(double x, double rho, std::pair<double, double> a, std::pair<double, double> b) {
    const double ex = b.first - a.first;
    const double er = b.second - a.second;
    const double l2 = (ex * ex) + (er * er);
    double t = l2 > 0.0 ? (((x - a.first) * ex) + ((rho - a.second) * er)) / l2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    return std::hypot(x - (a.first + (t * ex)), rho - (a.second + (t * er)));
  }
  // Even-odd: a ray from the point toward +x crosses the closed outline (the profile, then down to the axis at each end that is not already on it, and along the axis) an odd number of times
  // when the point is inside.
  [[nodiscard]] bool inside(double x, double rho) const {
    bool in = false;
    const std::size_t n = poly_.size();
    for (std::size_t i = 0; i < n; ++i) {
      const std::pair<double, double> a = poly_[i];
      const std::pair<double, double> b = poly_[(i + 1) % n];
      if ((a.second > rho) != (b.second > rho)) {
        const double xc = a.first + ((rho - a.second) * (b.first - a.first) / (b.second - a.second));
        if (x < xc) {
          in = !in;
        }
      }
    }
    return in;
  }
  std::vector<std::pair<double, double>> pr_;
  std::vector<std::pair<double, double>> poly_;
};

// Solid where any of two shapes is.
class Union final : public Shape {
 public:
  Union(const Shape& a, const Shape& b) : a_(a), b_(b) {}
  [[nodiscard]] double phi(V3 p) const override { return std::min(a_.phi(p), b_.phi(p)); }
  void panels(double h, std::vector<Panel>& out) const override {
    a_.panels(h, out);
    b_.panels(h, out);
  }

 private:
  const Shape& a_;
  const Shape& b_;
};

// Solid where both are.
class Intersection final : public Shape {
 public:
  Intersection(const Shape& a, const Shape& b) : a_(a), b_(b) {}
  [[nodiscard]] double phi(V3 p) const override { return std::max(a_.phi(p), b_.phi(p)); }
  void panels(double, std::vector<Panel>&) const override {}

 private:
  const Shape& a_;
  const Shape& b_;
};

}  // namespace sim::cfd
