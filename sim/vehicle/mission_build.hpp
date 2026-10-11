// SPDX-License-Identifier: MIT
// From a vehicle file's mission description to the constant tables the flight computers carry (docs/design/GNC.md section 9): the phases with the thrust and mass flow of the engines they run, the targets of the guidance
// worked out from the orbits that were asked for, the mixers (their surface allocation is designed from the vehicle, mission_design.hpp), the place of the landing site. The gains and the guidance's seed are not here: they
// come from the design run, which flies the mission once with the attitude forced.
#pragma once
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "mission_spec.hpp"
#include "spec_io.hpp"
#include "vehicle6.hpp"
#include "tfc/mission.hpp"
#include "tfc/peg.hpp"

namespace sim {

struct BuiltMission {
  tfc::gnc::Tables main;
  std::array<tfc::gnc::Tables, kMaxStages> stage{};
  std::array<bool, kMaxStages> has_stage{};
  bool present = false;
};

namespace detail {

inline uint8_t kind_of(const std::string& k, bool& ok) {
  using namespace tfc::gnc::kind;
  ok = true;
  if (k == "hold") return kHold;
  if (k == "program") return kProgram;
  if (k == "peg") return kPeg;
  if (k == "coast") return kCoast;
  if (k == "boostback") return kBoostback;
  if (k == "glide") return kGlide;
  if (k == "landing") return kLanding;
  if (k == "chute") return kChute;
  if (k == "done") return kDone;
  ok = false;
  return kHold;
}

inline uint8_t end_of(const std::string& k, bool& ok) {
  using namespace tfc::gnc::end;
  ok = true;
  if (k == "never") return kNever;
  if (k == "time_s") return kTime;
  if (k == "speed_ms") return kSpeedAbove;
  if (k == "apoapsis_after_s") return kApoapsis;
  if (k == "altitude_below_m") return kAltitudeBelow;
  if (k == "altitude_above_m") return kAltitudeAbove;
  if (k == "cutoff") return kCutoff;
  if (k == "mass_below_kg") return kMassBelow;
  if (k == "aligned_deg") return kAligned;
  if (k == "touchdown") return kTouchdown;
  if (k == "ignition") return kIgnition;
  ok = false;
  return kNever;
}

inline uint8_t hold_of(const std::string& k, bool& ok) {
  using namespace tfc::gnc::hold;
  ok = true;
  if (k == "inertial") return kInertial;
  if (k == "prograde") return kPrograde;
  if (k == "retrograde") return kRetrograde;
  if (k == "radial") return kRadial;
  if (k == "fixed") return kFixed;
  if (k == "retro_horizontal") return kRetroHorizontal;
  if (k == "pro_horizontal") return kProHorizontal;
  ok = false;
  return kInertial;
}

inline uint8_t event_of(const std::string& k, bool& ok) {
  ok = true;
  if (k == "separate") return tfc::propbit::kSeparate;
  if (k == "deploy_surfaces") return tfc::propbit::kDeploySurfaces;
  if (k == "extend_legs") return tfc::propbit::kExtendLegs;
  if (k == "chute0") return tfc::propbit::kChute0;
  if (k == "chute1") return tfc::propbit::kChute1;
  if (k == "jettison") return tfc::propbit::kJettison;
  ok = false;
  return 0U;
}

inline uint8_t mask_of(const std::vector<int>& groups) {
  uint8_t m = 0U;
  for (const int g : groups) {
    if (g >= 0 && g < static_cast<int>(kEngineGroups)) {
      m = static_cast<uint8_t>(m | (1U << g));
    }
  }
  return m;
}

// The thrust and the mass flow at full throttle of the main engines of the given groups (all stages: a group number belongs to one stage in a vehicle file that means to be flown).
inline void group_totals(const VehicleSpec& g, uint8_t mask, double& thrust, double& mdot, double& per_engine_mdot) {
  thrust = 0.0;
  mdot = 0.0;
  per_engine_mdot = 0.0;
  for (const EngineSpec& e : g.engines) {
    if (e.control == Control::None && e.group >= 0 && ((mask >> e.group) & 1U) != 0U) {
      thrust += e.thrust_vac;
      mdot += e.thrust_vac / (e.isp_vac * kG0);
      per_engine_mdot = e.thrust_vac / (e.isp_vac * kG0);
    }
  }
}

// The mean time constant (s) with which the main engines of the given groups die away after a shutdown command: the guidance ends a burn early by the impulse they still give.
inline double mean_tail_s(const VehicleSpec& g, uint8_t mask) {
  double tail = 0.0;
  unsigned n = 0U;
  for (const EngineSpec& e : g.engines) {
    if (e.control == Control::None && e.group >= 0 && ((mask >> e.group) & 1U) != 0U) {
      tail += e.tail_s;
      ++n;
    }
  }
  return n > 0U ? tail / static_cast<double>(n) : 0.0;
}

}  // namespace detail

// Build the tables of one plan. `v` is the vehicle at T-zero (for the planet, the ground, the masses).
inline bool build_plan(const VehicleFile& f, const Vehicle6& v, const std::vector<PhaseSpec>& plan, const std::string& who, tfc::gnc::Tables& t, std::vector<std::string>& errors) {
  using namespace tfc::gnc;
  const MissionSpec& m = f.mission;
  const VehicleSpec& g = v.spec();
  const PlanetSpec& pl = g.planet;
  t = Tables{};
  t.nav.gravity.mu = pl.mu;
  t.nav.gravity.radius = pl.radius;
  t.nav.gravity.j2 = pl.j2;
  t.nav.gravity.pole = tfc::dm::Vec3{v.pole().x, v.pole().y, v.pole().z};
  t.nav.r0 = tfc::dm::Vec3{v.state().r.x, v.state().r.y, v.state().r.z};
  t.nav.v0 = tfc::dm::Vec3{v.state().v.x, v.state().v.y, v.state().v.z};
  t.mass0 = v.mass();
  t.ground_radius = v.ground_radius();
  t.site = tfc::dm::Vec3{v.ground_radius(), m.site_offset_y_m, m.site_offset_z_m};
  t.plane_normal = tfc::dm::Vec3{0.0, 0.0, 1.0};
  t.limits.command_deg = static_cast<float>(m.command_limit_deg);
  t.limits.integrator_deg = static_cast<float>(m.integrator_limit_deg);
  t.limits.slew_deg_per_frame = static_cast<float>(m.slew_deg_per_frame);
  t.ignition_margin = m.ignition_margin;
  t.drag.scale_height = m.scale_height_m;
  t.drag.beta = m.ballistic_coefficient;
  t.landing.sink_ms = m.sink_ms;
  t.landing.aim_below_m = m.aim_below_m;
  t.landing.approach_s = m.approach_s;
  t.landing.horizon_min_s = m.horizon_min_s;
  t.landing.tilt_max_deg = m.tilt_max_deg;
  t.landing.tilt_final_deg = m.tilt_final_deg;
  t.landing.final_height_m = m.final_height_m;
  t.landing.decel_plan = m.decel_plan_ms2;
  t.engines.thrust_each = m.engine_thrust_n;
  t.engines.min_throttle = m.engine_min_throttle;
  if (plan.size() > kMaxPhases) {
    errors.push_back("mission." + who + ": at most " + std::to_string(kMaxPhases) + " phases");
    return false;
  }
  t.n_phases = static_cast<uint8_t>(plan.size());
  std::size_t track = 0;
  bool have_program = false;
  bool good = true;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const PhaseSpec& ps = plan[i];
    const std::string at = "mission." + who + "[" + std::to_string(i) + "]";
    Phase& ph = t.phase[i];
    bool ok = true;
    ph.kind = detail::kind_of(ps.kind, ok);
    if (!ok) {
      errors.push_back(at + ".kind: unknown \"" + ps.kind + "\"");
      good = false;
    }
    ph.end = detail::end_of(ps.end_kind, ok);
    ph.hold = detail::hold_of(ps.hold, ok);
    if (!ok) {
      errors.push_back(at + ".hold: unknown \"" + ps.hold + "\"");
      good = false;
    }
    ph.end_value = static_cast<float>(ps.end_value);
    ph.groups = detail::mask_of(ps.groups);
    for (const std::string& e : ps.events) {
      bool eok = true;
      ph.events = static_cast<uint8_t>(ph.events | detail::event_of(e, eok));
      if (!eok) {
        errors.push_back(at + ".events: unknown event \"" + e + "\"");
      good = false;
      }
    }
    ph.mixer = static_cast<uint8_t>(ps.mixer);
    ph.throttle = static_cast<float>(ps.throttle);
    ph.slew_dps = static_cast<float>(ps.slew_dps);
    ph.mass_set = static_cast<float>(ps.mass_set_kg);
    double thrust = 0.0;
    double mdot = 0.0;
    double per = 0.0;
    detail::group_totals(g, ph.groups, thrust, mdot, per);
    ph.thrust = static_cast<float>(thrust);
    ph.mdot = static_cast<float>(mdot);
    if (!ps.throttle_track.empty() && track < kThrottleTracks) {
      ThrottleTrack& tr = t.throttle[track];
      for (const std::array<double, 2>& p : ps.throttle_track) {
        if (tr.n < kThrottlePoints) {
          tr.frame[tr.n] = static_cast<uint32_t>(p[0] * 100.0 + 0.5);
          tr.value[tr.n] = static_cast<float>(p[1]);
          ++tr.n;
        }
      }
      ph.throttle_track = static_cast<uint8_t>(track);
      ++track;
    }
    if (ph.kind == kind::kProgram && !ps.program.empty() && !have_program) {
      have_program = true;
      for (const std::array<double, 2>& p : ps.program) {
        (void)t.program.add(1U, static_cast<uint32_t>(p[0] * 100.0 + 0.5), static_cast<float>(p[1]));
      }
    }
    if (ph.kind == kind::kCoast && ph.hold == hold::kFixed) {
      ph.p[0] = static_cast<float>(ps.fixed.x);
      ph.p[1] = static_cast<float>(ps.fixed.y);
      ph.p[2] = static_cast<float>(ps.fixed.z);
    }
    if (ph.kind == kind::kPeg) {
      double r = ps.target_radius_m;
      double vs = ps.target_speed_ms;
      double gamma = ps.target_gamma_deg;
      if (ps.circular_km >= 0.0) {
        r = pl.radius + (ps.circular_km * 1000.0);
        vs = std::sqrt(pl.mu / r);
        gamma = 0.0;
      } else if (ps.apogee_km >= 0.0) {
        const double rc = pl.radius + (ps.cutoff_km * 1000.0);
        const tfc::peg::OrbitPoint op = tfc::peg::orbit_point(pl.mu, pl.radius + (ps.apogee_km * 1000.0), pl.radius + (ps.perigee_km * 1000.0), rc);
        r = rc;
        vs = op.speed;
        gamma = op.gamma_rad * kRad2Deg;
      }
      ph.p[0] = static_cast<float>(r);
      ph.p[1] = static_cast<float>(vs);
      ph.p[2] = static_cast<float>(gamma);
      ph.p[8] = static_cast<float>(ps.burnout_mass_kg);
      ph.p[9] = static_cast<float>(detail::mean_tail_s(g, ph.groups));   // (the engines go on pushing for their decay time after the cutoff: the burn is ended that much speed early)
    } else if (ph.kind == kind::kBoostback) {
      ph.p[0] = static_cast<float>(ps.bias_m);
      ph.p[1] = static_cast<float>(ps.reserve_mass_kg);
      ph.p[2] = static_cast<float>(ps.pitch_up_deg);
      ph.p[3] = static_cast<float>(detail::mean_tail_s(g, ph.groups));   // (the impulse the engines still give while they die away is taken off the burn)
    } else if (ph.kind == kind::kGlide) {
      ph.p[0] = static_cast<float>(ps.alpha_max_deg);
      ph.p[1] = static_cast<float>(ps.gain_deg_per_km / 1000.0);
      ph.p[2] = static_cast<float>(ps.alpha_brake_deg);
      ph.p[3] = static_cast<float>(ps.lift_area_per_deg_m2);
      ph.p[4] = static_cast<float>(ps.gate_speed_ms);
      ph.p[5] = static_cast<float>(ps.gate_height_m);
      ph.p[6] = static_cast<float>(ps.brake_max_deg);
    } else if (ph.kind == kind::kLanding) {
      unsigned options = 0U;
      for (std::size_t n = 0; n < 4U; ++n) {
        const uint8_t mk = detail::mask_of(ps.landing_groups[n]);
        ph.p[n] = static_cast<float>(mk);
        if (mk != 0U) {
          unsigned cnt = 0U;
          for (const EngineSpec& e : g.engines) {
            cnt += (e.control == Control::None && e.group >= 0 && ((mk >> e.group) & 1U) != 0U) ? 1U : 0U;
          }
          t.engines.counts[n] = cnt;
          options = static_cast<unsigned>(n) + 1U;
        }
      }
      t.engines.options = options;
      ph.groups = detail::mask_of(ps.landing_groups[options > 0U ? options - 1U : 0U]);
      double th = 0.0;
      double md = 0.0;
      double pe = 0.0;
      detail::group_totals(g, ph.groups, th, md, pe);
      ph.mdot = static_cast<float>(pe);   // per engine
      ph.thrust = static_cast<float>(th);
    } else if (ph.kind == kind::kChute) {
      ph.p[0] = static_cast<float>(ps.drogue_altitude_m);
      ph.p[1] = static_cast<float>(ps.main_altitude_m);
    }
  }
  return good;
}

// Build the tables of every plan in the file (the surfaces' allocation, the gains and the seeds come from the design run).
inline bool build_mission(const VehicleFile& f, BuiltMission& out, std::vector<std::string>& errors) {
  out = BuiltMission{};
  if (!f.mission.present) {
    return true;
  }
  out.present = true;
  const Vehicle6 v(f.params, f.scenario);
  const MissionSpec& m = f.mission;
  bool ok = build_plan(f, v, m.main, "main", out.main, errors);
  const auto set_mixers = [&m](tfc::gnc::Tables& t) {
    for (std::size_t i = 0; i < m.mixers.size() && i < tfc::gnc::kMaxMixers; ++i) {
      t.mixer[i].gimbal_pitch = m.mixers[i].gimbal ? 1.0F : 0.0F;
      t.mixer[i].gimbal_yaw = m.mixers[i].gimbal ? 1.0F : 0.0F;
      t.mixer[i].roll = m.mixers[i].roll_thrusters ? 1.0F : 0.0F;
    }
  };
  set_mixers(out.main);
  // the catch point's height above the ground, for the centre of gravity: the pins at the arms' height, the pins ahead of the centre of gravity by the distance between them
  for (std::size_t s = 0; s < kMaxStages; ++s) {
    if (m.stage[s].empty()) {
      continue;
    }
    out.has_stage[s] = true;
    ok = build_plan(f, v, m.stage[s], "stage" + std::to_string(s), out.stage[s], errors) && ok;
    set_mixers(out.stage[s]);
    const StageSpec& st = f.params.spec.stages[s];
    const double cg = v.stage_cg_from_aft(s, m.landing_propellant_kg);
    out.stage[s].landing_height = m.arm_height_m - (st.catch_pin_x - cg);
    // the slope: the same at another amount of propellant, up to a hundred tonnes more (or less, if the stage is already as full as it goes)
    double capacity = 0.0;
    for (const TankSpec& tk : st.tanks) {
      capacity += tk.propellant;
    }
    const double room = capacity - m.landing_propellant_kg;
    const double step = room > 1.0 ? std::min(100000.0, room) : -std::min(100000.0, std::max(m.landing_propellant_kg, 0.0));
    if (std::fabs(step) > 1.0) {
      const double cg_more = v.stage_cg_from_aft(s, m.landing_propellant_kg + step);
      out.stage[s].landing_height_slope = ((m.arm_height_m - (st.catch_pin_x - cg_more)) - out.stage[s].landing_height) / step;
    }
    out.stage[s].landing_mass_ref = st.dry_mass + m.landing_propellant_kg;
    out.stage[s].arm_height = m.arm_height_m;
  }
  return ok;
}

}  // namespace sim
