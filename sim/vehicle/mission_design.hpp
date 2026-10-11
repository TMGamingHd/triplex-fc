// SPDX-License-Identifier: MIT
// The design of a mission (docs/design/GNC.md section 9): the vehicle is flown once with the attitude forced to the one each phase asks for (the nominal flight), and along it the aerodynamic stiffness of each axis and the
// effect of each effector on the angular acceleration are measured by differences on the vehicle model itself. From those come, for every phase, the gains of the attitude loop (the same second-order loop whatever moves the
// vehicle: kp = (wn^2 + a) / b, kd = 2 zeta wn / b, as in design.hpp for the pitch program) and the allocation of the demand to the surfaces; from the flight itself the guidance's starting solution (a burn solved from
// scratch, which the flight computers then track) and the mass each stage had when it was let go.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "mission_build.hpp"
#include "mission_world.hpp"

namespace sim {

struct DesignReport {
  bool ok = false;
  double end_time_s = 0.0;
  uint32_t probes = 0U;
  std::vector<std::string> notes;
};

namespace detail {

// One axis (0 roll, 1 yaw, 2 pitch) of the angular acceleration of a probe column.
inline double axis_of(const std::array<double, 3>& c, int axis) { return c[static_cast<std::size_t>(axis)]; }

// The effect of one unit of demand on an axis through a mixer, from a probe (rad/s^2 per unit: per degree of gimbal, or per unit of the allocated surface demand).
inline double effectiveness(const AxisProbe& pr, const tfc::gnc::Mixer& mx, int axis) {
  double b = 0.0;
  if (axis == 0) {
    b += static_cast<double>(mx.roll) * axis_of(pr.col[2], 0);
  } else if (axis == 1) {
    b += static_cast<double>(mx.gimbal_yaw) * axis_of(pr.col[1], 1);
  } else {
    b += static_cast<double>(mx.gimbal_pitch) * axis_of(pr.col[0], 2);
  }
  for (std::size_t k = 0; k < kSurfaceChannels; ++k) {
    const double gain = axis == 0 ? static_cast<double>(mx.from_roll[k]) : (axis == 1 ? static_cast<double>(mx.from_yaw[k]) : static_cast<double>(mx.from_pitch[k]));
    b += gain * axis_of(pr.col[3U + k], axis);
  }
  return b;
}

// The minimum-effort allocation of a demand on each axis to the surface channels in use: u = B^T (B B^T)^-1 e for the 3 x m matrix B of effects (rows roll, yaw, pitch), normalised so that a unit demand is
// one radian per second squared. Solved with a small regularisation so that a rank-deficient set (fewer than three useful channels) gives the least-squares share.
inline void allocate(const std::array<std::array<double, 3>, 4>& cols, const std::vector<int>& used, tfc::gnc::Mixer& mx, const std::array<double, 4>& travel_deg, double command_deg) {
  const std::size_t m = used.size();
  if (m == 0U) {
    return;
  }
  // G = B B^T + eps I (3 x 3)
  double g[3][3] = {};
  double scale = 0.0;
  for (std::size_t k = 0; k < m; ++k) {
    for (int i = 0; i < 3; ++i) {
      scale = std::max(scale, std::fabs(cols[static_cast<std::size_t>(used[k])][static_cast<std::size_t>(i)]));
    }
  }
  if (!(scale > 1e-12)) {
    return;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (std::size_t k = 0; k < m; ++k) {
        s += cols[static_cast<std::size_t>(used[k])][static_cast<std::size_t>(i)] * cols[static_cast<std::size_t>(used[k])][static_cast<std::size_t>(j)];
      }
      g[i][j] = s + (i == j ? 1e-4 * scale * scale : 0.0);
    }
  }
  // invert G by cofactors
  const double det = g[0][0] * (g[1][1] * g[2][2] - g[1][2] * g[2][1]) - g[0][1] * (g[1][0] * g[2][2] - g[1][2] * g[2][0]) + g[0][2] * (g[1][0] * g[2][1] - g[1][1] * g[2][0]);
  if (!(std::fabs(det) > 1e-300)) {
    return;
  }
  double inv[3][3];
  inv[0][0] = (g[1][1] * g[2][2] - g[1][2] * g[2][1]) / det;
  inv[0][1] = (g[0][2] * g[2][1] - g[0][1] * g[2][2]) / det;
  inv[0][2] = (g[0][1] * g[1][2] - g[0][2] * g[1][1]) / det;
  inv[1][0] = (g[1][2] * g[2][0] - g[1][0] * g[2][2]) / det;
  inv[1][1] = (g[0][0] * g[2][2] - g[0][2] * g[2][0]) / det;
  inv[1][2] = (g[0][2] * g[1][0] - g[0][0] * g[1][2]) / det;
  inv[2][0] = (g[1][0] * g[2][1] - g[1][1] * g[2][0]) / det;
  inv[2][1] = (g[0][1] * g[2][0] - g[0][0] * g[2][1]) / det;
  inv[2][2] = (g[0][0] * g[1][1] - g[0][1] * g[1][0]) / det;
  for (int axis = 0; axis < 3; ++axis) {
    std::array<double, 4> u{};
    double worst = 0.0;   // the largest share of its travel that a unit of demand asks of any surface
    for (std::size_t k = 0; k < m; ++k) {
      for (int i = 0; i < 3; ++i) {
        u[k] += cols[static_cast<std::size_t>(used[k])][static_cast<std::size_t>(i)] * inv[i][axis];
      }
      worst = std::max(worst, std::fabs(u[k]) / std::max(travel_deg[static_cast<std::size_t>(used[k])], 1e-3));
    }
    // the demand of the loop is limited to `command_deg`: scale so that the full demand takes the most loaded surface to 80% of its travel (a unit of demand is then a fraction of a degree of the surface, not a
    // radian per second squared, and the effectiveness the gains are designed from is measured with this allocation)
    const double share = worst > 0.0 ? 0.8 / (command_deg * worst) : 0.0;
    for (std::size_t k = 0; k < m; ++k) {
      const std::size_t ch = static_cast<std::size_t>(used[k]);
      (axis == 0 ? mx.from_roll : (axis == 1 ? mx.from_yaw : mx.from_pitch))[ch] = static_cast<float>(u[k] * share);
    }
  }
}

}  // namespace detail

// The gains and the surface allocation of one set of computers (a plan), from the probes of its body.
inline void fit_plan(const MissionSpec& ms, const VehicleSpec& spec, tfc::gnc::Tables& tab, const std::vector<AxisProbe>& all, int owner) {
  using namespace tfc::gnc;
  std::vector<const AxisProbe*> mine;
  for (const AxisProbe& p : all) {
    if (p.stage == owner) {
      mine.push_back(&p);
    }
  }
  // 1. the surface allocation of each mixer that names channels, from the average (normalised) effect of the channels over the probes of the phases that use it
  for (std::size_t idx = 0; idx < ms.mixers.size() && idx < kMaxMixers; ++idx) {
    const std::vector<int>& used = ms.mixers[idx].surfaces;
    if (used.empty()) {
      continue;
    }
    std::array<std::array<double, 3>, 4> avg{};
    int n = 0;
    for (const AxisProbe* p : mine) {
      if (tab.phase[p->phase].mixer != idx) {
        continue;
      }
      double scale = 0.0;
      for (const int ch : used) {
        for (std::size_t i = 0; i < 3U; ++i) {
          scale = std::max(scale, std::fabs(p->col[3U + static_cast<std::size_t>(ch)][i]));
        }
      }
      if (!(scale > 1e-12)) {
        continue;
      }
      for (const int ch : used) {
        for (std::size_t i = 0; i < 3U; ++i) {
          avg[static_cast<std::size_t>(ch)][i] += p->col[3U + static_cast<std::size_t>(ch)][i] / scale;
        }
      }
      ++n;
    }
    std::array<double, 4> travel{};
    for (const int ch : used) {
      for (const SurfaceSpec& sf : spec.surfaces) {
        if (sf.channel == ch) {
          tab.mixer[idx].lo[static_cast<std::size_t>(ch)] = static_cast<float>(sf.min_deg);
          tab.mixer[idx].hi[static_cast<std::size_t>(ch)] = static_cast<float>(sf.max_deg);
          travel[static_cast<std::size_t>(ch)] = std::min(std::fabs(sf.min_deg), std::fabs(sf.max_deg));
          break;
        }
      }
    }
    if (n > 0) {
      detail::allocate(avg, used, tab.mixer[idx], travel, static_cast<double>(tab.limits.command_deg));
    }
  }
  // 2. the gains of each phase, at every probe, thinned to what a track holds
  for (unsigned ph = 0; ph < tab.n_phases; ++ph) {
    std::vector<const AxisProbe*> pts;
    for (const AxisProbe* p : mine) {
      if (p->phase == ph) {
        pts.push_back(p);
      }
    }
    GainTrack& tr = tab.gains[ph];
    tr = GainTrack{};
    if (pts.empty()) {
      continue;
    }
    const std::size_t mixer_idx = tab.phase[ph].mixer < kMaxMixers ? tab.phase[ph].mixer : 0U;
    const Mixer& mx = tab.mixer[mixer_idx];
    const std::vector<int> no_surfaces;
    const std::vector<int>& used = mixer_idx < ms.mixers.size() ? ms.mixers[mixer_idx].surfaces : no_surfaces;
    std::array<double, 4> travel{};
    for (const int ch : used) {
      for (const SurfaceSpec& sf : spec.surfaces) {
        if (sf.channel == ch) {
          travel[static_cast<std::size_t>(ch)] = std::min(std::fabs(sf.min_deg), std::fabs(sf.max_deg));
          break;
        }
      }
    }
    tr.has_alloc = !used.empty();
    std::vector<tfc::att::Gains3> g;
    std::vector<SurfAlloc> allocs;
    for (const AxisProbe* p : pts) {
      tfc::att::Gains3 gs;
      tfc::att::AxisGains* ax[3] = {&gs.roll, &gs.yaw, &gs.pitch};
      // the allocation at this point of the flight: from the effect of the surfaces here, not on average
      Mixer here = mx;
      SurfAlloc al{};
      if (!used.empty()) {
        std::array<std::array<double, 3>, 4> cols{};
        for (const int ch : used) {
          cols[static_cast<std::size_t>(ch)] = p->col[3U + static_cast<std::size_t>(ch)];
        }
        here.from_pitch.fill(0.0F);
        here.from_yaw.fill(0.0F);
        here.from_roll.fill(0.0F);
        detail::allocate(cols, used, here, travel, static_cast<double>(tab.limits.command_deg));
        al = SurfAlloc{here.from_pitch, here.from_yaw, here.from_roll};
      }
      allocs.push_back(al);
      for (int a = 0; a < 3; ++a) {
        const double b = detail::effectiveness(*p, here, a);
        const double stiff = std::max(0.0, p->a[static_cast<std::size_t>(a)]);
        const double bb = std::fabs(b) > ms.b_min ? b : (b < 0.0 ? -ms.b_min : ms.b_min);
        const std::vector<PhaseSpec>& plan = owner < 0 ? ms.main : ms.stage[static_cast<std::size_t>(owner)];
        const double wn = ph < plan.size() && plan[ph].wn_rad_s > 0.0 ? plan[ph].wn_rad_s : ms.wn;
        double kp = (wn * wn + stiff) / bb;
        double kd = 2.0 * ms.zeta * wn / bb;
        kp = std::clamp(kp, -ms.kp_max, ms.kp_max);
        kd = std::clamp(kd, -ms.kp_max, ms.kp_max);
        // the design is in degrees of effector per radian of error (b is per degree of effector); the loop takes its error in degrees and its rate error in degrees per second
        constexpr double kPerDeg = kDeg2Rad;
        ax[a]->kp = static_cast<float>(kp * kPerDeg);
        ax[a]->kd = static_cast<float>(kd * kPerDeg);
        ax[a]->ki = static_cast<float>(ms.ki_over_kp * kp * kPerDeg);
      }
      g.push_back(gs);
    }
    const std::size_t n = pts.size();
    const std::size_t take = std::min<std::size_t>(n, kGainPoints);
    uint32_t last_frame = 0U;
    for (std::size_t i = 0; i < take; ++i) {
      const std::size_t src = take == 1U ? 0U : i * (n - 1U) / (take - 1U);
      uint32_t frame = static_cast<uint32_t>(pts[src]->t_in_phase * 100.0 + 0.5);
      if (i > 0U && frame <= last_frame) {
        frame = last_frame + 1U;
      }
      tr.frame[i] = frame;
      tr.gains[i] = g[src];
      tr.alloc[i] = allocs[src];
      tr.thrust[i] = static_cast<float>(pts[src]->thrust);
      last_frame = frame;
      ++tr.n;
    }
  }
}

// Run the design on the mission built from the file. Fills the gains, the mixers' allocations, the guidance seeds and the stage masses of `m`.
inline DesignReport design_mission(const VehicleFile& f, BuiltMission& m, const std::function<void(const MissionWorld&)>& watch = nullptr) {
  using namespace tfc::gnc;
  DesignReport rep;
  const MissionSpec& ms = f.mission;
  WorldConfig wc;
  wc.params = f.params;
  wc.scenario = f.scenario;
  wc.scenario.tower.enabled = true;
  wc.scenario.tower.height_m = ms.arm_height_m;
  wc.scenario.tower.offset_y_m = ms.site_offset_y_m;
  wc.scenario.tower.offset_z_m = ms.site_offset_z_m;
  wc.scenario.tower.capture_radius_m = ms.capture_radius_m;
  wc.main_tables = &m.main;
  for (std::size_t s = 0; s < kMaxStages; ++s) {
    wc.stage_tables[s] = m.has_stage[s] ? &m.stage[s] : nullptr;
  }
  wc.ideal = true;
  wc.probe_every = std::max<uint32_t>(1U, static_cast<uint32_t>(ms.sample_s * 100.0));
  const uint32_t max_frames = static_cast<uint32_t>(ms.max_time_s * 100.0);
  const auto tables_of = [&m](int stage) -> Tables& { return stage < 0 ? m.main : m.stage[static_cast<std::size_t>(stage)]; };
  // Two passes: the first flies the mission on the guidance's own model of the air and measures the drag the vehicle really has on its way down; the second flies it with the model made to match, which is the
  // flight the seeds, the masses and the probes are taken from.
  bool glides = false;
  for (std::size_t s = 0; s < kMaxStages; ++s) {
    for (unsigned ph = 0; m.has_stage[s] && ph < m.stage[s].n_phases; ++ph) {
      glides = glides || m.stage[s].phase[ph].kind == kind::kGlide;
    }
  }
  const unsigned kPasses = glides ? 2U : 1U;
  for (unsigned pass = 0; pass < kPasses; ++pass) {
    MissionWorld world(wc);
    std::vector<AxisProbe> probes;
    world.set_probe([&probes](const AxisProbe& p) { probes.push_back(p); });
    uint32_t tail = 0U;
    uint32_t k = 0U;
    for (; k < max_frames; ++k) {
      world.step();
      if (watch && pass + 1U == kPasses && k % 100U == 0U) {
        watch(world);
      }
      tail = world.finished() ? tail + 1U : 0U;
      if (tail > static_cast<uint32_t>(ms.tail_s * 100.0)) {
        break;
      }
    }
    if (pass + 1U < kPasses) {
      for (std::size_t s = 0; s < kMaxStages; ++s) {
        const double beta = m.has_stage[s] ? world.drag_beta(s) : 0.0;
        if (beta > 0.0) {
          m.stage[s].drag.beta = beta;
        }
      }
      continue;
    }
    rep.end_time_s = static_cast<double>(k) * 0.01;
    rep.probes = static_cast<uint32_t>(probes.size());
    for (const MissionWorld::Seed& sd : world.seeds()) {
      Phase& ph = tables_of(sd.stage).phase[sd.phase];
      for (std::size_t i = 0; i < tfc::peg::kUnknowns; ++i) {
        ph.p[3U + i] = static_cast<float>(sd.sol[i]);
      }
    }
    for (std::size_t s = 0; s < kMaxStages; ++s) {
      if (m.has_stage[s] && world.spawn_mass(s) > 0.0) {
        m.stage[s].mass0 = world.spawn_mass(s);
      }
    }
    fit_plan(ms, f.params.spec, m.main, probes, -1);
    for (std::size_t s = 0; s < kMaxStages; ++s) {
      if (m.has_stage[s]) {
        fit_plan(ms, f.params.spec, m.stage[s], probes, static_cast<int>(s));
      }
    }
    rep.ok = rep.probes > 0U;
  }
  return rep;
}

}  // namespace sim
