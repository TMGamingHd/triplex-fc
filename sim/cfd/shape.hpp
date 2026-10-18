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

// A box of any orientation: its centre, three orthonormal axes and the half length along each (a flap, a fin, a plate at an angle to the stream). `part` marks its surface elements, so that the force on this
// piece alone can be told from the rest of the body's.
class OrientedBox final : public Shape {
 public:
  OrientedBox(V3 centre, V3 a1, V3 a2, V3 a3, V3 half, int part = 0) : c_(centre), a_{normalized(a1), normalized(a2), normalized(a3)}, h_(half), part_(part) {}
  [[nodiscard]] double phi(V3 p) const override {
    const V3 d = p - c_;
    const V3 q{std::fabs(dot(d, a_[0])) - h_.x, std::fabs(dot(d, a_[1])) - h_.y, std::fabs(dot(d, a_[2])) - h_.z};
    const V3 o{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
    return norm(o) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0);
  }
  void panels(double h, std::vector<Panel>& out) const override {
    const double half[3] = {h_.x, h_.y, h_.z};
    for (int axis = 0; axis < 3; ++axis) {
      const int i1 = (axis + 1) % 3;
      const int i2 = (axis + 2) % 3;
      const int n1 = std::max(1, static_cast<int>(std::ceil(2.0 * half[i1] / h)));
      const int n2 = std::max(1, static_cast<int>(std::ceil(2.0 * half[i2] / h)));
      for (const double sign : {1.0, -1.0}) {
        for (int i = 0; i < n1; ++i) {
          for (int j = 0; j < n2; ++j) {
            const double u = (-half[i1]) + ((i + 0.5) * 2.0 * half[i1] / n1);
            const double v = (-half[i2]) + ((j + 0.5) * 2.0 * half[i2] / n2);
            const V3 n = a_[axis] * sign;
            const V3 c = c_ + (n * half[axis]) + (a_[i1] * u) + (a_[i2] * v);
            if (c.z < 0.0) {
              continue;   // the solver adds the mirror image of what lies at z >= 0
            }
            out.push_back(Panel{c, n, (2.0 * half[i1] / n1) * (2.0 * half[i2] / n2), part_});
          }
        }
      }
    }
  }
  [[nodiscard]] V3 centre() const { return c_; }

 private:
  V3 c_;
  V3 a_[3];
  V3 h_;
  int part_;
};

// A plate with pointed ends: the cross-section in the plane of `ax` (along the length) and `ay` (the thickness) is a hexagon, a rectangle of thickness t whose two ends taper to points over the
// length `bevel`, and it is extruded along `az` from -extrusion to +extrusion (the chord of a flap: the pointed ends are the edges that face the stream). Real flaps and fins have such edges; a blunt end
// would stand a detached shock in front of it. `part` marks its surface elements.
class BeveledPlate final : public Shape {
 public:
  BeveledPlate(V3 centre, V3 ax, V3 ay, V3 az, double length, double thickness, double bevel, double extrusion, int part = 0)
      : c_(centre), ax_(normalized(ax)), ay_(normalized(ay)), az_(normalized(az)), e_(extrusion), part_(part) {
    const double l = 0.5 * length;
    const double t = 0.5 * thickness;
    const double b = std::min(bevel, length * 0.49);
    poly_ = {{-l, 0.0}, {-l + b, t}, {l - b, t}, {l, 0.0}, {l - b, -t}, {-l + b, -t}};
  }
  [[nodiscard]] double phi(V3 p) const override {
    const V3 d = p - c_;
    const double x = dot(d, ax_);
    const double y = dot(d, ay_);
    const double z = dot(d, az_);
    const double d2 = polygon_distance(x, y);
    const double dz = std::fabs(z) - e_;
    const double o = std::hypot(std::max(d2, 0.0), std::max(dz, 0.0));
    return o + std::min(std::max(d2, dz), 0.0);
  }
  void panels(double h, std::vector<Panel>& out) const override {
    const std::size_t n = poly_.size();
    // the six faces of the cross-section, each extruded
    for (std::size_t i = 0; i < n; ++i) {
      const auto a = poly_[i];
      const auto b = poly_[(i + 1U) % n];
      const double ex = b.first - a.first;
      const double ey = b.second - a.second;
      const double len = std::hypot(ex, ey);
      const double nx = ey / len;     // outward for a polygon listed clockwise as seen from +az... the sign is fixed below by the distance
      const double ny = -ex / len;
      const int ns = std::max(1, static_cast<int>(std::ceil(len / h)));
      const int nz = std::max(1, static_cast<int>(std::ceil(2.0 * e_ / h)));
      for (int u = 0; u < ns; ++u) {
        for (int w = 0; w < nz; ++w) {
          const double f = (u + 0.5) / ns;
          const double zz = (-e_) + ((w + 0.5) * 2.0 * e_ / nz);
          const double px = a.first + (f * ex);
          const double py = a.second + (f * ey);
          V3 nrm = (ax_ * nx) + (ay_ * ny);
          V3 c = c_ + (ax_ * px) + (ay_ * py) + (az_ * zz);
          if (phi(c + (nrm * 1e-3)) < 0.0) {
            nrm = nrm * -1.0;
          }
          if (c.z < 0.0) {
            continue;
          }
          out.push_back(Panel{c, nrm, (len / ns) * (2.0 * e_ / nz), part_});
        }
      }
    }
    // the two ends of the extrusion: the polygon, on a grid
    double xmin = 1e300;
    double xmax = -1e300;
    double ymin = 1e300;
    double ymax = -1e300;
    for (const auto& q : poly_) {
      xmin = std::min(xmin, q.first);
      xmax = std::max(xmax, q.first);
      ymin = std::min(ymin, q.second);
      ymax = std::max(ymax, q.second);
    }
    const int nx = std::max(1, static_cast<int>(std::ceil((xmax - xmin) / h)));
    const int ny = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / h)));
    for (const double sign : {1.0, -1.0}) {
      for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
          const double px = xmin + ((i + 0.5) * (xmax - xmin) / nx);
          const double py = ymin + ((j + 0.5) * (ymax - ymin) / ny);
          if (polygon_distance(px, py) >= 0.0) {
            continue;
          }
          const V3 c = c_ + (ax_ * px) + (ay_ * py) + (az_ * (sign * e_));
          if (c.z < 0.0) {
            continue;
          }
          out.push_back(Panel{c, az_ * sign, ((xmax - xmin) / nx) * ((ymax - ymin) / ny), part_});
        }
      }
    }
  }

 private:
  // The signed distance (negative inside) from (x, y) to the hexagon.
  [[nodiscard]] double polygon_distance(double x, double y) const {
    double best = 1e300;
    bool inside = false;
    const std::size_t n = poly_.size();
    for (std::size_t i = 0; i < n; ++i) {
      const auto a = poly_[i];
      const auto b = poly_[(i + 1U) % n];
      const double ex = b.first - a.first;
      const double ey = b.second - a.second;
      const double l2 = (ex * ex) + (ey * ey);
      const double t = std::min(1.0, std::max(0.0, (((x - a.first) * ex) + ((y - a.second) * ey)) / l2));
      best = std::min(best, std::hypot(x - (a.first + (t * ex)), y - (a.second + (t * ey))));
      if ((a.second > y) != (b.second > y) && x < a.first + ((y - a.second) * ex / ey)) {
        inside = !inside;
      }
    }
    return inside ? -best : best;
  }
  V3 c_;
  V3 ax_;
  V3 ay_;
  V3 az_;
  double e_;
  int part_;
  std::vector<std::pair<double, double>> poly_;
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

// The union of any number of shapes: solid where any is, the surface of each except where it lies inside another (a flap's face against the hull is not surface).
class Assembly final : public Shape {
 public:
  Assembly() = default;
  void add(const Shape* s) { parts_.push_back(s); }
  [[nodiscard]] double phi(V3 p) const override {
    double d = 1e300;
    for (const Shape* s : parts_) {
      d = std::min(d, s->phi(p));
    }
    return d;
  }
  void panels(double h, std::vector<Panel>& out) const override {
    for (std::size_t i = 0; i < parts_.size(); ++i) {
      std::vector<Panel> pi;
      parts_[i]->panels(h, pi);
      for (const Panel& p : pi) {
        bool inside = false;
        for (std::size_t j = 0; j < parts_.size() && !inside; ++j) {
          inside = j != i && parts_[j]->phi(p.c) < -1e-6;
        }
        if (!inside) {
          out.push_back(p);
        }
      }
    }
  }

 private:
  std::vector<const Shape*> parts_;
};

// Solid where any of two shapes is.
class Union final : public Shape {
 public:
  Union(const Shape& a, const Shape& b) : a_(a), b_(b) {}
  [[nodiscard]] double phi(V3 p) const override { return std::min(a_.phi(p), b_.phi(p)); }
  // the surface of each piece except where it lies inside the other (a flap's face on the hull is not surface)
  void panels(double h, std::vector<Panel>& out) const override {
    std::vector<Panel> pa;
    std::vector<Panel> pb;
    a_.panels(h, pa);
    b_.panels(h, pb);
    for (const Panel& p : pa) {
      if (b_.phi(p.c) > -1e-6) {
        out.push_back(p);
      }
    }
    for (const Panel& p : pb) {
      if (a_.phi(p.c) > -1e-6) {
        out.push_back(p);
      }
    }
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
