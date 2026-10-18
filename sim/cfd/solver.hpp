// SPDX-License-Identifier: MIT
// A three-dimensional Euler solver for a body in a flow, on a Cartesian grid with the body immersed in it (docs/design/CFD.md; ADR-050).
//
//  * The grid is a tensor product of three lines of cell faces, so it can be fine near the body and coarse far away; the body is a signed distance (shape.hpp), not a mesh.
//  * A cell is solid when its centre is inside the body. The solid cells next to the fluid (two deep: the second-order stencil reaches two cells) are ghost cells: each takes the state of the fluid at
//    its image point, the point mirrored in the surface, with the velocity's normal component reversed, so that the flow through the surface is zero (the slip wall of the Euler equations).
//  * The fluxes are HLLC (HLL on a strong shock) from a MUSCL reconstruction of the primitive variables with a slope limiter; the time step is a two-stage Runge-Kutta (SSP) with a local step
//    toward a steady state, or one global step for a time-accurate run (the shock tube).
//  * The forces come from the pressure at the body's surface elements, read at probe points off the surface.
// Calorically perfect gas, inviscid: no skin friction, no wake, no real-gas effects; docs/design/CFD.md section 6 says what that does to a number.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "euler.hpp"
#include "shape.hpp"

namespace sim::cfd {

// The faces of the cells along one axis from `lo` to `hi`: `n` equal cells.
inline std::vector<double> uniform_nodes(double lo, double hi, int n) {
  std::vector<double> v(static_cast<std::size_t>(n) + 1U);
  for (int i = 0; i <= n; ++i) {
    v[static_cast<std::size_t>(i)] = lo + ((hi - lo) * i / n);
  }
  return v;
}

// Cells of about `h` across the core [core_lo, core_hi], and outside it cells that grow by `ratio` each (up to `h_max`) until `lo` and `hi`: fine where the body is, coarse where only waves go.
inline std::vector<double> graded_nodes(double lo, double hi, double core_lo, double core_hi, double h, double ratio, double h_max) {
  const int nc = std::max(1, static_cast<int>(std::lround((core_hi - core_lo) / h)));
  const double hc = (core_hi - core_lo) / nc;
  std::vector<double> left;    // from the core's edge outward
  std::vector<double> right;
  double w = hc;
  for (double x = core_lo; x > lo;) {
    w = std::min(h_max, w * ratio);
    x -= w;
    left.push_back(std::max(x, lo));
    if (x <= lo) {
      break;
    }
  }
  w = hc;
  for (double x = core_hi; x < hi;) {
    w = std::min(h_max, w * ratio);
    x += w;
    right.push_back(std::min(x, hi));
    if (x >= hi) {
      break;
    }
  }
  std::vector<double> v;
  v.reserve(left.size() + right.size() + static_cast<std::size_t>(nc) + 1U);
  for (auto it = left.rbegin(); it != left.rend(); ++it) {
    v.push_back(*it);
  }
  for (int i = 0; i <= nc; ++i) {
    v.push_back(core_lo + (hc * i));
  }
  for (const double x : right) {
    v.push_back(x);
  }
  return v;
}

// Worker threads that stay alive between loops: a range of an index is cut into equal parts, one per thread (the caller does the first), so a result does not depend on the thread count when every
// index writes only its own cells (the solver's loops all do).
class Pool {
 public:
  explicit Pool(int threads) : n_(std::max(1, threads)) {
    for (int id = 1; id < n_; ++id) {
      th_.emplace_back([this, id]() { work(id); });
    }
  }
  Pool(const Pool&) = delete;
  Pool& operator=(const Pool&) = delete;
  ~Pool() {
    {
      const std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_start_.notify_all();
    for (std::thread& t : th_) {
      t.join();
    }
  }
  void run(int lo, int hi, const std::function<void(int, int)>& f) {
    if (n_ == 1 || hi - lo < 2) {
      f(lo, hi);
      return;
    }
    {
      const std::lock_guard<std::mutex> lk(m_);
      job_ = &f;
      lo_ = lo;
      hi_ = hi;
      pending_ = n_ - 1;
      ++gen_;
    }
    cv_start_.notify_all();
    f(lo, lo + ((hi - lo) / n_));
    std::unique_lock<std::mutex> lk(m_);
    cv_done_.wait(lk, [this]() { return pending_ == 0; });
  }

 private:
  void work(int id) {
    int seen = 0;
    for (;;) {
      std::unique_lock<std::mutex> lk(m_);
      cv_start_.wait(lk, [this, &seen]() { return stop_ || gen_ != seen; });
      if (stop_) {
        return;
      }
      seen = gen_;
      const std::function<void(int, int)>* job = job_;
      const int lo = lo_;
      const int hi = hi_;
      lk.unlock();
      (*job)(lo + ((hi - lo) * id / n_), lo + ((hi - lo) * (id + 1) / n_));
      lk.lock();
      if (--pending_ == 0) {
        cv_done_.notify_one();
      }
    }
  }
  int n_;
  std::vector<std::thread> th_;
  std::mutex m_;
  std::condition_variable cv_start_;
  std::condition_variable cv_done_;
  const std::function<void(int, int)>* job_ = nullptr;
  int lo_ = 0;
  int hi_ = 0;
  int gen_ = 0;
  int pending_ = 0;
  bool stop_ = false;
};

enum class Bc : int { Farfield = 0, Symmetry = 1, Outflow = 2 };

struct Config {
  double gamma = 1.4;
  double cfl = 0.6;
  Limiter limiter = Limiter::VanAlbada;
  double shock_ratio = 1.8;
  int order = 2;                  // 1: first order (piecewise constant); 2: MUSCL
  int threads = 1;
  bool local_dt = true;           // true: each cell steps at its own limit (a steady state); false: all at the smallest (time-accurate)
  std::array<Bc, 6> bc{Bc::Farfield, Bc::Farfield, Bc::Farfield, Bc::Farfield, Bc::Farfield, Bc::Farfield};   // x low, x high, y low, y high, z low, z high
  double probe_cells = 0.75;      // how far off the surface (in cells) the pressure for the force is read, in the single-probe form
  bool probe_extrapolate = true;  // read at 0.5 and 1 cell off and extrapolate linearly to the surface (the pressure falls with distance in the layer behind a cone's shock: a single probe reads it low)
};

struct Forces {
  V3 force;       // N, from the pressure over the free stream's (gauge pressure), the whole body (the mirror half added)
  V3 moment;      // N m about the reference point
  double area = 0.0;
};

class Euler3D {
 public:
  Euler3D(const std::array<std::vector<double>, 3>& nodes, const Prim& free_stream, const Shape* shape, const Config& cfg) : cfg_(cfg), pool_(cfg.threads), inf_(free_stream), shape_(shape) {
    for (std::size_t a = 0; a < 3; ++a) {
      const std::vector<double>& nd = nodes[a];
      n_[a] = static_cast<int>(nd.size()) - 1;
      const std::size_t tot = static_cast<std::size_t>(n_[a]) + 4U;
      xf_[a].assign(tot + 1U, 0.0);
      xc_[a].assign(tot, 0.0);
      dx_[a].assign(tot, 0.0);
      for (int i = 0; i <= n_[a]; ++i) {
        xf_[a][static_cast<std::size_t>(i) + 2U] = nd[static_cast<std::size_t>(i)];
      }
      // two ghost cells on each side as wide as the cell next to them
      const double w0 = nd[1] - nd[0];
      const double w1 = nd[static_cast<std::size_t>(n_[a])] - nd[static_cast<std::size_t>(n_[a]) - 1U];
      xf_[a][1] = nd[0] - w0;
      xf_[a][0] = nd[0] - (2.0 * w0);
      xf_[a][static_cast<std::size_t>(n_[a]) + 3U] = nd[static_cast<std::size_t>(n_[a])] + w1;
      xf_[a][static_cast<std::size_t>(n_[a]) + 4U] = nd[static_cast<std::size_t>(n_[a])] + (2.0 * w1);
      for (std::size_t i = 0; i < tot; ++i) {
        dx_[a][i] = xf_[a][i + 1U] - xf_[a][i];
        xc_[a][i] = 0.5 * (xf_[a][i] + xf_[a][i + 1U]);
      }
    }
    stride_[0] = 1U;
    stride_[1] = static_cast<std::size_t>(n_[0]) + 4U;
    stride_[2] = stride_[1] * (static_cast<std::size_t>(n_[1]) + 4U);
    total_ = stride_[2] * (static_cast<std::size_t>(n_[2]) + 4U);
    u_.assign(total_, Cons{});
    u0_.assign(total_, Cons{});
    w_.assign(total_, Cons{});
    r_.assign(total_, Cons{});
    for (std::size_t d = 0; d < 3; ++d) {
      f_[d].assign(total_, Cons{});
      ic_[d].assign(xc_[d].size(), 0.0);
      for (std::size_t i = 0; i + 1 < xc_[d].size(); ++i) {
        ic_[d][i] = 1.0 / (xc_[d][i + 1U] - xc_[d][i]);
      }
    }
    dt_.assign(total_, 0.0);
    kind_.assign(total_, 3);
    initialise();
  }

  // ---- the grid ----
  [[nodiscard]] int cells(int axis) const { return n_[static_cast<std::size_t>(axis)]; }
  [[nodiscard]] double centre(int axis, int i) const { return xc_[static_cast<std::size_t>(axis)][static_cast<std::size_t>(i + 2)]; }
  [[nodiscard]] double width(int axis, int i) const { return dx_[static_cast<std::size_t>(axis)][static_cast<std::size_t>(i + 2)]; }
  [[nodiscard]] std::size_t cell_count() const { return static_cast<std::size_t>(n_[0]) * static_cast<std::size_t>(n_[1]) * static_cast<std::size_t>(n_[2]); }
  [[nodiscard]] const Prim& free_stream() const { return inf_; }
  [[nodiscard]] std::size_t index(int i, int j, int k) const {
    return static_cast<std::size_t>(i + 2) + (stride_[1] * static_cast<std::size_t>(j + 2)) + (stride_[2] * static_cast<std::size_t>(k + 2));
  }
  [[nodiscard]] bool is_fluid(int i, int j, int k) const { return kind_[index(i, j, k)] == 0; }
  [[nodiscard]] Prim prim(int i, int j, int k) const { return load(index(i, j, k)); }
  void set_prim(int i, int j, int k, const Prim& s) {
    const std::size_t c = index(i, j, k);
    store_w(c, s);
    u_[c] = to_cons(s, cfg_.gamma);
  }

  // Every cell at the free stream, the solid cells found from the shape, the ghost cells and their image points made.
  void initialise() {
    for (std::size_t c = 0; c < total_; ++c) {
      store_w(c, inf_);
      u_[c] = to_cons(inf_, cfg_.gamma);
    }
    std::fill(kind_.begin(), kind_.end(), static_cast<std::uint8_t>(3));
    for (int k = 0; k < n_[2]; ++k) {
      for (int j = 0; j < n_[1]; ++j) {
        for (int i = 0; i < n_[0]; ++i) {
          kind_[index(i, j, k)] = 0;
        }
      }
    }
    ghosts_.clear();
    if (shape_ != nullptr) {
      std::vector<double> ph(total_, 1.0);
      parallel(0, n_[2], [&](int k0, int k1) {
        for (int k = k0; k < k1; ++k) {
          for (int j = 0; j < n_[1]; ++j) {
            for (int i = 0; i < n_[0]; ++i) {
              ph[index(i, j, k)] = shape_->phi(V3{centre(0, i), centre(1, j), centre(2, k)});
            }
          }
        }
      });
      for (int k = 0; k < n_[2]; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            if (ph[index(i, j, k)] < 0.0) {
              kind_[index(i, j, k)] = 2;
            }
          }
        }
      }
      // a solid cell with a fluid cell one or two cells away along an axis is a ghost cell
      std::vector<std::uint8_t> mark(kind_);
      for (int k = 0; k < n_[2]; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            if (kind_[index(i, j, k)] != 2) {
              continue;
            }
            bool near_fluid = false;
            for (int s = -2; s <= 2 && !near_fluid; ++s) {
              if (s == 0) {
                continue;
              }
              near_fluid = fluid_at(i + s, j, k) || fluid_at(i, j + s, k) || fluid_at(i, j, k + s);
            }
            if (near_fluid) {
              mark[index(i, j, k)] = 1;
            }
          }
        }
      }
      kind_ = mark;
      for (int k = 0; k < n_[2]; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            if (kind_[index(i, j, k)] == 1) {
              ghosts_.push_back(make_ghost(i, j, k, ph[index(i, j, k)]));
            }
          }
        }
      }
    }
    iteration_ = 0;
    time_ = 0.0;
    res0_ = 0.0;
    res_ = 0.0;
  }

  [[nodiscard]] std::size_t ghost_count() const { return ghosts_.size(); }
  [[nodiscard]] int iteration() const { return iteration_; }
  [[nodiscard]] double time() const { return time_; }
  [[nodiscard]] double residual() const { return res_; }
  // How far the density residual has fallen from the first iteration's, in orders of magnitude.
  [[nodiscard]] double residual_drop() const { return (res0_ > 0.0 && res_ > 0.0) ? std::log10(res0_ / res_) : 0.0; }
  [[nodiscard]] bool healthy() const { return healthy_; }
  [[nodiscard]] bool floored() const { return floored_; }   // a density or pressure had to be raised to its floor at some point

  // The largest time step every fluid cell would take at the CFL number.
  [[nodiscard]] double max_dt() {
    refresh_primitives();
    fill_ghosts();
    double dt = 1e300;
    for (int k = 0; k < n_[2]; ++k) {
      for (int j = 0; j < n_[1]; ++j) {
        for (int i = 0; i < n_[0]; ++i) {
          if (kind_[index(i, j, k)] == 0) {
            dt = std::min(dt, cell_dt(i, j, k));
          }
        }
      }
    }
    return dt;
  }

  // One iteration (two Runge-Kutta stages) at the local time step, or at `dt` for every cell when it is given. Returns the root-mean-square density residual.
  double step(double dt_global = 0.0) {
    parallel(0, n_[2] + 4, [&](int k0, int k1) {
      const std::size_t a = static_cast<std::size_t>(k0) * stride_[2];
      const std::size_t b = std::min(total_, static_cast<std::size_t>(k1) * stride_[2]);
      std::copy(u_.begin() + static_cast<std::ptrdiff_t>(a), u_.begin() + static_cast<std::ptrdiff_t>(b), u0_.begin() + static_cast<std::ptrdiff_t>(a));
    });
    refresh_primitives();
    fill_ghosts();
    set_time_steps(dt_global);
    residual_into_r();
    // the root-mean-square density residual: a sum per plane, added in order, so that it does not depend on the number of threads
    std::vector<double> plane(static_cast<std::size_t>(n_[2]), 0.0);
    std::vector<std::size_t> plane_n(static_cast<std::size_t>(n_[2]), 0U);
    parallel(0, n_[2], [&](int k0, int k1) {
      for (int k = k0; k < k1; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            const std::size_t c = index(i, j, k);
            if (kind_[c] == 0) {
              plane[static_cast<std::size_t>(k)] += r_[c][0] * r_[c][0];
              ++plane_n[static_cast<std::size_t>(k)];
            }
          }
        }
      }
    });
    double sum = 0.0;
    std::size_t cnt = 0;
    for (std::size_t k = 0; k < plane.size(); ++k) {
      sum += plane[k];
      cnt += plane_n[k];
    }
    res_ = cnt > 0U ? std::sqrt(sum / static_cast<double>(cnt)) : 0.0;
    advance(1.0, 0.0);              // U1 = U0 + dt R(U0)
    refresh_primitives();
    fill_ghosts();
    residual_into_r();
    advance(0.5, 0.5);              // U = 1/2 U0 + 1/2 (U1 + dt R(U1))
    if (iteration_ == 0) {
      res0_ = res_;
    }
    ++iteration_;
    if (dt_global > 0.0) {
      time_ += dt_global;
    } else if (!cfg_.local_dt) {
      time_ += step_dt_;
    }
    check_health();
    return res_;
  }

  // Iterate until the residual has fallen by `orders` or `max_iter` iterations have run; returns the number run.
  int run_to_steady(int max_iter, double orders) {
    int it = 0;
    while (it < max_iter) {
      step();
      ++it;
      if (!healthy_ || residual_drop() >= orders) {
        break;
      }
    }
    return it;
  }

  // The state at a point by trilinear interpolation of the fluid cells around it; false when none is near.
  bool sample(V3 x, Prim& out) {
    refresh_primitives();
    return interpolate(x, out);
  }

  // The same for many points: make the primitive variables current once, then read as often as needed (`sample` does the first every time).
  void prepare_sampling() { refresh_primitives(); }
  [[nodiscard]] bool sample_prepared(V3 x, Prim& out) const { return interpolate(x, out); }

  // How the surface pressure is read from now on: at `cells` cells off the surface, or (extrapolate) from 0.5 and 1 cell off, linearly to the surface.
  void set_probe(double cells, bool extrapolate) {
    cfg_.probe_cells = cells;
    cfg_.probe_extrapolate = extrapolate;
  }

  // The pressure at the surface under a panel: read off the surface along the normal, at `probe_cells` cells, or extrapolated linearly from 0.5 and 1 cell.
  [[nodiscard]] double wall_pressure(const Panel& pn) {
    const double s = std::max({local_width(0, pn.c.x), local_width(1, pn.c.y), local_width(2, pn.c.z)});
    Prim a;
    if (cfg_.probe_extrapolate) {
      Prim b;
      const bool ok1 = interpolate(pn.c + (pn.n * (0.5 * s)), a);
      const bool ok2 = interpolate(pn.c + (pn.n * (1.0 * s)), b);
      if (ok1 && ok2) {
        return std::max(1e-6 * inf_.p, (2.0 * a.p) - b.p);
      }
      return ok1 ? a.p : inf_.p;
    }
    return interpolate(pn.c + (pn.n * (cfg_.probe_cells * s)), a) ? a.p : inf_.p;
  }

  // The force and the moment of the pressure over a set of panels and their mirror images in the plane z = 0 (the body is symmetric there); `part` >= 0 takes only the panels of that part.
  Forces integrate(const std::vector<Panel>& panels, V3 ref, bool mirror = true, int part = -1) {
    refresh_primitives();
    Forces out;
    for (const Panel& pn : panels) {
      if (part >= 0 && pn.part != part) {
        continue;
      }
      const double dp = wall_pressure(pn) - inf_.p;
      const V3 df = pn.n * (-dp * pn.area);
      out.force = out.force + df;
      out.moment = out.moment + cross(pn.c - ref, df);
      out.area += pn.area;
      if (mirror) {
        const Panel m{V3{pn.c.x, pn.c.y, -pn.c.z}, V3{pn.n.x, pn.n.y, -pn.n.z}, pn.area};
        const V3 dm = m.n * (-dp * m.area);
        out.force = out.force + dm;
        out.moment = out.moment + cross(m.c - ref, dm);
        out.area += m.area;
      }
    }
    return out;
  }

 private:
  struct Ghost {
    std::size_t cell = 0;
    std::array<std::size_t, 8> src{};
    std::array<double, 8> wt{};
    int count = 0;
    V3 normal;
  };

  [[nodiscard]] bool fluid_at(int i, int j, int k) const {
    if (i < 0 || j < 0 || k < 0 || i >= n_[0] || j >= n_[1] || k >= n_[2]) {
      return false;
    }
    return kind_[index(i, j, k)] == 0;
  }

  [[nodiscard]] Prim load(std::size_t c) const {
    const Cons& q = w_[c];
    return Prim{q[0], q[1], q[2], q[3], q[4]};
  }
  void store_w(std::size_t c, const Prim& s) { w_[c] = Cons{s.rho, s.u, s.v, s.w, s.p}; }

  template <class F>
  void parallel(int lo, int hi, F&& f) {
    const std::function<void(int, int)> fn(std::forward<F>(f));
    pool_.run(lo, hi, fn);
  }

  // The cell of an axis a coordinate lies in (clamped to the cells that have a neighbour on the high side), and where it is between that cell's centre and the next.
  void locate(std::size_t a, double x, int& i, double& f) const {
    const std::vector<double>& c = xc_[a];
    const int n = n_[a];
    int lo = 0;
    int hi = n - 1;
    while (lo < hi) {
      const int mid = (lo + hi + 1) / 2;
      if (c[static_cast<std::size_t>(mid) + 2U] <= x) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    i = std::min(lo, n - 2);
    const double c0 = c[static_cast<std::size_t>(i) + 2U];
    const double c1 = c[static_cast<std::size_t>(i) + 3U];
    f = std::min(1.0, std::max(0.0, (x - c0) / (c1 - c0)));
  }

  [[nodiscard]] double local_width(std::size_t a, double x) const {
    int i = 0;
    double f = 0.0;
    locate(a, x, i, f);
    return dx_[a][static_cast<std::size_t>(i) + 2U];
  }

  // Trilinear weights of the fluid cells around `x` (the 2 x 2 x 2 block): a cell that is not fluid weighs nothing and the rest are renormalised; the block grows by one cell on each side
  // if there is none.
  int weights(V3 x, std::array<std::size_t, 8>& src, std::array<double, 8>& wt) const {
    int i0 = 0;
    int j0 = 0;
    int k0 = 0;
    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    locate(0, x.x, i0, fx);
    locate(1, x.y, j0, fy);
    locate(2, x.z, k0, fz);
    int cnt = 0;
    double sum = 0.0;
    for (int dk = 0; dk < 2; ++dk) {
      for (int dj = 0; dj < 2; ++dj) {
        for (int di = 0; di < 2; ++di) {
          const int i = i0 + di;
          const int j = j0 + dj;
          const int k = k0 + dk;
          if (!fluid_at(i, j, k)) {
            continue;
          }
          const double w = (di == 0 ? 1.0 - fx : fx) * (dj == 0 ? 1.0 - fy : fy) * (dk == 0 ? 1.0 - fz : fz);
          if (w <= 1e-12) {
            continue;
          }
          src[static_cast<std::size_t>(cnt)] = index(i, j, k);
          wt[static_cast<std::size_t>(cnt)] = w;
          sum += w;
          ++cnt;
        }
      }
    }
    if (cnt > 0 && sum > 1e-9) {
      for (int q = 0; q < cnt; ++q) {
        wt[static_cast<std::size_t>(q)] /= sum;
      }
      return cnt;
    }
    // no fluid weighs anything: the nearest fluid cell in the block that is one cell wider on every side
    double best = 1e300;
    cnt = 0;
    for (int dk = -1; dk <= 2; ++dk) {
      for (int dj = -1; dj <= 2; ++dj) {
        for (int di = -1; di <= 2; ++di) {
          const int i = i0 + di;
          const int j = j0 + dj;
          const int k = k0 + dk;
          if (!fluid_at(i, j, k)) {
            continue;
          }
          const double d = norm(V3{centre(0, i), centre(1, j), centre(2, k)} - x);
          if (d < best) {
            best = d;
            src[0] = index(i, j, k);
            wt[0] = 1.0;
            cnt = 1;
          }
        }
      }
    }
    return cnt;
  }

  bool interpolate(V3 x, Prim& out) const {
    std::array<std::size_t, 8> src{};
    std::array<double, 8> wt{};
    const int n = weights(x, src, wt);
    if (n == 0) {
      return false;
    }
    Prim s{0.0, 0.0, 0.0, 0.0, 0.0};
    for (int q = 0; q < n; ++q) {
      const Prim c = load(src[static_cast<std::size_t>(q)]);
      const double w = wt[static_cast<std::size_t>(q)];
      s.rho += w * c.rho;
      s.u += w * c.u;
      s.v += w * c.v;
      s.w += w * c.w;
      s.p += w * c.p;
    }
    out = s;
    return true;
  }

  Ghost make_ghost(int i, int j, int k, double phi_c) const {
    Ghost g;
    g.cell = index(i, j, k);
    const V3 p{centre(0, i), centre(1, j), centre(2, k)};
    g.normal = shape_->normal(p);
    const V3 image = p - (g.normal * (2.0 * phi_c));   // phi_c < 0: the point as far outside the surface as the ghost is inside
    g.count = weights(image, g.src, g.wt);
    return g;
  }

  void refresh_primitives() {
    parallel(0, n_[2], [&](int k0, int k1) {
      for (int k = k0; k < k1; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            const std::size_t c = index(i, j, k);
            if (kind_[c] != 0) {
              continue;
            }
            store_w(c, to_prim(u_[c], cfg_.gamma));
          }
        }
      }
    });
  }

  // The domain's ghost cells from its boundaries, and the body's ghost cells from their image points.
  void fill_ghosts() {
    for (std::size_t face = 0; face < 6U; ++face) {
      const std::size_t a = face / 2U;
      const bool high = (face % 2U) == 1U;
      const double sign = high ? 1.0 : -1.0;   // the outward normal along the axis
      const std::size_t a1 = (a + 1U) % 3U;
      const std::size_t a2 = (a + 2U) % 3U;
      for (int layer = 0; layer < 2; ++layer) {
        const int ig = high ? n_[a] + layer : -1 - layer;     // the ghost cell's index along the axis
        const int ii = high ? n_[a] - 1 - layer : layer;      // the interior cell it mirrors
        for (int q = 0; q < n_[a2]; ++q) {
          for (int p = 0; p < n_[a1]; ++p) {
            std::array<int, 3> gi{};
            std::array<int, 3> ni{};
            gi[a] = ig;
            ni[a] = ii;
            gi[a1] = ni[a1] = p;
            gi[a2] = ni[a2] = q;
            const std::size_t cg = index(gi[0], gi[1], gi[2]);
            const std::size_t cn = index(ni[0], ni[1], ni[2]);
            if (kind_[cn] != 0 && kind_[cn] != 1) {
              store_w(cg, inf_);
              continue;
            }
            const Prim in = load(cn);
            switch (cfg_.bc[face]) {
              case Bc::Farfield:
                store_w(cg, farfield_state(in, inf_, static_cast<int>(a), sign, cfg_.gamma));
                break;
              case Bc::Symmetry: {
                Prim m = in;
                set_vel(m, static_cast<int>(a), -vel(in, static_cast<int>(a)));
                store_w(cg, m);
                break;
              }
              case Bc::Outflow:
              default:
                store_w(cg, in);
                break;
            }
          }
        }
      }
    }
    parallel(0, static_cast<int>(ghosts_.size()), [&](int g0, int g1) {
      for (int gi = g0; gi < g1; ++gi) {
        const Ghost& g = ghosts_[static_cast<std::size_t>(gi)];
        if (g.count == 0) {
          store_w(g.cell, inf_);
          continue;
        }
        Prim s{0.0, 0.0, 0.0, 0.0, 0.0};
        for (int q = 0; q < g.count; ++q) {
          const Prim c = load(g.src[static_cast<std::size_t>(q)]);
          const double w = g.wt[static_cast<std::size_t>(q)];
          s.rho += w * c.rho;
          s.u += w * c.u;
          s.v += w * c.v;
          s.w += w * c.w;
          s.p += w * c.p;
        }
        const double vn = (s.u * g.normal.x) + (s.v * g.normal.y) + (s.w * g.normal.z);   // the normal velocity is turned round: zero through the surface
        s.u -= 2.0 * vn * g.normal.x;
        s.v -= 2.0 * vn * g.normal.y;
        s.w -= 2.0 * vn * g.normal.z;
        store_w(g.cell, s);
      }
    });
  }

  [[nodiscard]] double cell_dt(int i, int j, int k) const {
    const Prim s = load(index(i, j, k));
    const double c = sound(s, cfg_.gamma);
    const double rate = ((std::fabs(s.u) + c) / width(0, i)) + ((std::fabs(s.v) + c) / width(1, j)) + ((std::fabs(s.w) + c) / width(2, k));
    return cfg_.cfl / rate;
  }

  void set_time_steps(double dt_global) {
    parallel(0, n_[2], [&](int k0, int k1) {
      for (int k = k0; k < k1; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            const std::size_t c = index(i, j, k);
            if (kind_[c] == 0) {
              dt_[c] = cell_dt(i, j, k);
            }
          }
        }
      }
    });
    if (dt_global > 0.0 || !cfg_.local_dt) {
      double d = dt_global;
      if (d <= 0.0) {
        d = 1e300;
        for (int k = 0; k < n_[2]; ++k) {
          for (int j = 0; j < n_[1]; ++j) {
            for (int i = 0; i < n_[0]; ++i) {
              const std::size_t c = index(i, j, k);
              if (kind_[c] == 0) {
                d = std::min(d, dt_[c]);
              }
            }
          }
        }
        step_dt_ = d;
      }
      parallel(0, n_[2], [&](int k0, int k1) {
        for (int k = k0; k < k1; ++k) {
          for (int j = 0; j < n_[1]; ++j) {
            for (int i = 0; i < n_[0]; ++i) {
              dt_[index(i, j, k)] = d;
            }
          }
        }
      });
    }
  }

  // The state on each side of the face on the low side of cell (i, j, k) along axis d, from the cells on either side and one beyond each.
  void face_states(int d, int i, int j, int k, Prim& left, Prim& right) const {
    const std::size_t ud = static_cast<std::size_t>(d);
    const std::size_t cb = index(i, j, k);
    const std::size_t ca = cb - stride_[ud];
    const std::size_t caa = ca - stride_[ud];
    const std::size_t cbb = cb + stride_[ud];
    const int pos = d == 0 ? i : (d == 1 ? j : k);   // the index along the axis
    const std::size_t pa = static_cast<std::size_t>(pos - 1 + 2);
    const std::vector<double>& xc = xc_[ud];
    const double x_a = xc[pa];
    const double x_b = xc[pa + 1U];
    const double x_f = xf_[ud][pa + 1U];
    const double i_aa = ic_[ud][pa - 1U];   // 1 / (x_a - x_aa)
    const double i_ab = ic_[ud][pa];
    const double i_bb = ic_[ud][pa + 1U];   // 1 / (x_bb - x_b)
    const Cons& wa = w_[ca];
    const Cons& wb = w_[cb];
    Cons l;
    Cons r;
    if (cfg_.order <= 1) {
      l = wa;
      r = wb;
    } else {
      const Cons& waa = w_[caa];
      const Cons& wbb = w_[cbb];
      for (std::size_t q = 0; q < 5; ++q) {
        const double d_ab = (wb[q] - wa[q]) * i_ab;
        const double sa = limited_slope((wa[q] - waa[q]) * i_aa, d_ab, cfg_.limiter);
        const double sb = limited_slope(d_ab, (wbb[q] - wb[q]) * i_bb, cfg_.limiter);
        l[q] = wa[q] + (sa * (x_f - x_a));
        r[q] = wb[q] + (sb * (x_f - x_b));
      }
    }
    left = Prim{l[0], l[1], l[2], l[3], l[4]};
    right = Prim{r[0], r[1], r[2], r[3], r[4]};
    if (!(left.rho > 0.0 && left.p > 0.0 && right.rho > 0.0 && right.p > 0.0)) {   // a reconstruction that is not a state: the cells' own
      left = load(ca);
      right = load(cb);
    }
  }

  void residual_into_r() {
    const int nk = n_[2];
    for (std::size_t d = 0; d < 3U; ++d) {
      const int ni = n_[0] + (d == 0U ? 1 : 0);
      const int nj = n_[1] + (d == 1U ? 1 : 0);
      const int nkk = nk + (d == 2U ? 1 : 0);
      parallel(0, nkk, [&](int k0, int k1) {
        for (int k = k0; k < k1; ++k) {
          for (int j = 0; j < nj; ++j) {
            for (int i = 0; i < ni; ++i) {
              const std::size_t cb = index(i, j, k);
              const std::size_t ca = cb - stride_[d];
              if (kind_[ca] != 0 && kind_[cb] != 0) {
                continue;
              }
              Prim l;
              Prim r;
              face_states(static_cast<int>(d), i, j, k, l, r);
              const Cons fl = flux(l, r, static_cast<int>(d), cfg_.gamma, cfg_.shock_ratio);
              f_[d][cb] = fl;
            }
          }
        }
      });
    }
    parallel(0, nk, [&](int k0, int k1) {
      for (int k = k0; k < k1; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            const std::size_t c = index(i, j, k);
            if (kind_[c] != 0) {
              continue;
            }
            const double inv0 = 1.0 / width(0, i);
            const double inv1 = 1.0 / width(1, j);
            const double inv2 = 1.0 / width(2, k);
            const Cons& x0 = f_[0][c];
            const Cons& x1 = f_[0][c + stride_[0]];
            const Cons& y0 = f_[1][c];
            const Cons& y1 = f_[1][c + stride_[1]];
            const Cons& z0 = f_[2][c];
            const Cons& z1 = f_[2][c + stride_[2]];
            for (std::size_t q = 0; q < 5; ++q) {
              r_[c][q] = -(((x1[q] - x0[q]) * inv0) + ((y1[q] - y0[q]) * inv1) + ((z1[q] - z0[q]) * inv2));
            }
          }
        }
      }
    });
  }

  // U = a U0 + b (U + dt R) for the fluid cells: (1, 0) the first stage from U0; (1/2, 1/2) the second.
  void advance(double a, double b) {
    parallel(0, n_[2], [&](int k0, int k1) {
      for (int k = k0; k < k1; ++k) {
        for (int j = 0; j < n_[1]; ++j) {
          for (int i = 0; i < n_[0]; ++i) {
            const std::size_t c = index(i, j, k);
            if (kind_[c] != 0) {
              continue;
            }
            Cons q;
            for (std::size_t m = 0; m < 5; ++m) {
              const double stepped = u_[c][m] + (dt_[c] * r_[c][m]);
              q[m] = b > 0.0 ? (a * u0_[c][m]) + (b * stepped) : stepped;
            }
            // keep the state physical: a floor on the density and on the pressure
            const double floor_rho = 1e-6 * inf_.rho;
            if (!(q[0] > floor_rho)) {
              q[0] = floor_rho;
              floored_ = true;
            }
            const double ke = 0.5 * ((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3])) / q[0];
            const double e_min = ke + (1e-6 * inf_.p / (cfg_.gamma - 1.0));
            if (!(q[4] > e_min)) {
              q[4] = e_min;
              floored_ = true;
            }
            u_[c] = q;
          }
        }
      }
    });
  }

  void check_health() {
    if (!std::isfinite(res_)) {
      healthy_ = false;
    }
  }

  Config cfg_;
  Pool pool_;
  Prim inf_;
  const Shape* shape_;
  std::array<int, 3> n_{};
  std::array<std::vector<double>, 3> xf_;
  std::array<std::vector<double>, 3> xc_;
  std::array<std::vector<double>, 3> dx_;
  std::array<std::size_t, 3> stride_{};
  std::size_t total_ = 0;
  std::vector<Cons> u_;    // the conserved state of each cell, the five numbers together: a stencil then touches a few cache lines, not twenty
  std::vector<Cons> u0_;
  std::vector<Cons> w_;    // the primitive state, in the order of Prim
  std::vector<Cons> r_;
  std::array<std::vector<Cons>, 3> f_;   // the flux through the low face of each cell, along each axis
  std::array<std::vector<double>, 3> ic_;   // 1 / (distance between the centres of a cell and the next)
  std::vector<double> dt_;
  std::vector<std::uint8_t> kind_;   // 0 fluid, 1 ghost of the body, 2 solid, 3 outside the domain
  std::vector<Ghost> ghosts_;
  int iteration_ = 0;
  double time_ = 0.0;
  double res0_ = 0.0;
  double res_ = 0.0;
  double step_dt_ = 0.0;
  bool healthy_ = true;
  bool floored_ = false;
};

}  // namespace sim::cfd
