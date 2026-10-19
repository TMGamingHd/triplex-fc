// SPDX-License-Identifier: MIT
// The vehicle's outer body as the flow solver's shape: the sections of the stages that are on it (spec.hpp), as the body of revolution their outlines make (docs/design/CFD.md section 3).
// Flaps, grid fins and fixed fins are not in it yet: the shape is the body alone.
#pragma once
#include <algorithm>
#include <utility>
#include <vector>

#include "newtonian.hpp"
#include "shape.hpp"
#include "spec.hpp"

namespace sim::cfd {

// The radius of the body at stations along it, from the aft end (the smallest x) to the nose, from the sections of the stages from `first_stage` on and the payloads (when the whole stack is
// meant): each section's outline at `per_section` + 1 stations, a nose with its stations crowded at the tip (newtonian.hpp's outline).
inline std::vector<std::pair<double, double>> body_profile(const VehicleSpec& g, std::size_t first_stage, int per_section, std::size_t last_stage = 1000U) {
  std::vector<SectionSpec> sections;
  for (std::size_t s = first_stage; s < g.stages.size() && s <= last_stage; ++s) {
    sections.insert(sections.end(), g.stages[s].sections.begin(), g.stages[s].sections.end());
  }
  if (first_stage == 0U && last_stage >= g.stages.size() - 1U) {
    for (const PayloadSpec& p : g.payloads) {
      sections.insert(sections.end(), p.sections.begin(), p.sections.end());
    }
  }
  std::sort(sections.begin(), sections.end(), [](const SectionSpec& a, const SectionSpec& b) { return a.x_start < b.x_start; });
  std::vector<std::pair<double, double>> out;
  for (const SectionSpec& sec : sections) {
    for (const newton::Station& st : newton::outline_of(sec, per_section)) {
      if (!out.empty() && st.x - out.back().first < 1e-9 && std::fabs(st.r - out.back().second) < 1e-9) {
        continue;   // the end of one piece is the start of the next
      }
      out.emplace_back(st.x, st.r);
    }
  }
  return out;
}

}  // namespace sim::cfd
