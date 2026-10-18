// SPDX-License-Identifier: MIT
// The flow solver's aerodynamic tables in the plant (docs/design/CFD.md section 14): the lookup by Mach number and angle of attack, the choice of the table for the stages that are on the vehicle, and the slopes at
// a small angle that the control design needs. Host only.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include "spec.hpp"

namespace sim::cfdaero {

struct Coeffs {
  double ca = 0.0;
  double ca_base = 0.0;
  double cn = 0.0;
  double cm = 0.0;
};

// Where `x` falls on an ascending grid: the cell and the fraction in it (held at the ends).
inline void locate(const std::vector<double>& g, double x, std::size_t& i, double& f) {
  if (x <= g.front()) {
    i = 0U;
    f = 0.0;
    return;
  }
  if (x >= g.back()) {
    i = g.size() - 2U;
    f = 1.0;
    return;
  }
  i = static_cast<std::size_t>(std::upper_bound(g.begin(), g.end(), x) - g.begin()) - 1U;
  f = (x - g[i]) / (g[i + 1U] - g[i]);
}

inline Coeffs lookup(const CfdAeroDb& db, double mach, double alpha_deg) {
  std::size_t i = 0;
  std::size_t j = 0;
  double fm = 0.0;
  double fa = 0.0;
  locate(db.mach, mach, i, fm);
  locate(db.alpha_deg, std::fabs(alpha_deg), j, fa);
  const std::size_t na = db.alpha_deg.size();
  auto at = [&](const std::vector<double>& t) {
    const double a = (t[(i * na) + j] * (1.0 - fa)) + (t[(i * na) + j + 1U] * fa);
    const double b = (t[((i + 1U) * na) + j] * (1.0 - fa)) + (t[((i + 1U) * na) + j + 1U] * fa);
    return (a * (1.0 - fm)) + (b * fm);
  };
  return Coeffs{at(db.ca), at(db.ca_base), at(db.cn), at(db.cm)};
}

// The table for the stages that are on the vehicle, or null.
inline const CfdAeroDb* select(const std::vector<CfdAeroDb>& dbs, const std::vector<bool>& stage_active) {
  for (const CfdAeroDb& db : dbs) {
    bool same = true;
    for (std::size_t s = 0; s < stage_active.size(); ++s) {
      bool listed = false;
      for (const int x : db.stages) {
        listed = listed || static_cast<std::size_t>(x) == s;
      }
      same = same && listed == stage_active[s];
    }
    if (same) {
      return &db;
    }
  }
  return nullptr;
}

// The normal-force slope (per radian, on s_ref) and the sum of slope times arm to x_cg (m), from the table's first positive angle of attack.
inline void slopes(const CfdAeroDb& db, double mach, double x_cg, double& cn_alpha, double& moment_slope) {
  const double a1 = db.alpha_deg[1];
  const Coeffs c = lookup(db, mach, a1);
  const double a1r = a1 * 3.14159265358979323846 / 180.0;
  cn_alpha = c.cn / a1r;
  moment_slope = ((-c.cm * db.d_ref) + ((db.x_ref - x_cg) * c.cn)) / a1r;
}

}  // namespace sim::cfdaero
