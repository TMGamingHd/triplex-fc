// SPDX-License-Identifier: MIT
// The plant of a vehicle that comes back (sim/vehicle/surfaces.hpp, the parts of vehicle6.hpp that use it; docs/design/RECOVERY.md, docs/design/AERODYNAMICS.md sections 7 to 9):
// the plate model of a flap and a grid fin against hand calculations of the same formulas and the physics they must obey (a plate takes energy out of the flow, a stowed flap is nearly
// invisible, a plate broadside to the stream has the flat-plate drag), the parachute's inflation and the terminal speed it gives, the engines the flight computers drive (a group mask, a
// throttle, a lowest throttle, a limit on starts, a start that fails), the surfaces' deployment and rate limit, the separation that hands a stage on as a vehicle of its own with the momentum
// and the centre of gravity of the whole conserved, the touchdown judged on speed and attitude, the catch in the tower's arms, and the full-speed-range aerodynamics.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "atmosphere.hpp"
#include "newtonian.hpp"
#include "spec.hpp"
#include "surfaces.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

sim::SurfaceSpec flap_at(double azimuth_deg) {
  sim::SurfaceSpec f;
  f.kind = sim::SurfaceKind::Flap;
  f.x_hinge = 40.0;
  f.azimuth_deg = azimuth_deg;
  f.radius = 4.5;
  f.area = 10.0;
  f.chord = 2.5;
  f.span = 4.0;
  f.chord_dir = -1.0;
  f.channel = 0;
  return f;
}

sim::SurfaceSpec grid_fin_at(double azimuth_deg) {
  sim::SurfaceSpec f = flap_at(azimuth_deg);
  f.kind = sim::SurfaceKind::GridFin;
  f.area = 6.0;
  f.chord = 0.6;
  f.span = 3.0;
  f.min_deg = -40.0;
  f.max_deg = 40.0;
  return f;
}

// One stage, no tanks unless asked; the planet has an atmosphere of constant density and no engines act unless given.
sim::Params bare_stage(double dry, double length, double radius) {
  sim::Params p;
  p.ground_contact = false;
  p.cd_scale = 0.0;
  p.cn_scale = 0.0;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = dry;
  st.x_start = 0.0;
  st.length = length;
  st.radius = radius;
  st.inertia_factor = 0.6;
  st.x_cg_dry = 0.5 * length;
  p.spec.stages.push_back(st);
  return p;
}

sim::Scenario aloft(double altitude, double down_speed) {
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + altitude, 0.0, 0.0};
  sc.initial.v = sim::V3{-down_speed, 0.0, 0.0};
  return sc;
}

sim::EngineSpec engine(double thrust, int group, double min_throttle = 0.0) {
  sim::EngineSpec e;
  e.pos = sim::V3{0.0, 0.0, 0.0};
  e.thrust_vac = thrust;
  e.exit_area = 0.0;
  e.isp_vac = 300.0;
  e.group = group;
  e.min_throttle = min_throttle;
  e.gimbal = false;
  return e;
}

}  // namespace

// ---- the plate model ----

TFC_TEST(plate_frames_are_orthonormal_for_flaps_and_grid_fins_at_any_deflection) {
  for (const sim::SurfaceKind kind : {sim::SurfaceKind::Flap, sim::SurfaceKind::GridFin}) {
    for (const double az : {0.0, 70.0, 180.0, 250.0}) {
      for (const double d : {-30.0, 0.0, 25.0, 90.0}) {
        sim::SurfaceSpec s = flap_at(az);
        s.kind = kind;
        const sim::SurfaceFrame f = sim::surface_frame(s, d);
        CHECK(near_abs(sim::norm(f.chord), 1.0, 1e-12) && near_abs(sim::norm(f.normal), 1.0, 1e-12) && near_abs(sim::norm(f.span), 1.0, 1e-12));
        CHECK(near_abs(sim::dot(f.chord, f.normal), 0.0, 1e-12) && near_abs(sim::dot(f.chord, f.span), 0.0, 1e-12) && near_abs(sim::dot(f.normal, f.span), 0.0, 1e-12));
        CHECK(near_abs(sim::norm(f.radial), 1.0, 1e-12));
      }
    }
  }
}

TFC_TEST(plate_a_flap_broadside_to_the_stream_has_the_flat_plate_drag) {
  const sim::SurfaceSpec s = flap_at(0.0);
  const double rho = 1.2;
  const double v = 200.0;
  const sim::SurfaceLoad l = sim::surface_load(s, 90.0, sim::V3{v, 0.0, 0.0}, rho, 340.0, 1.0);
  const double q = 0.5 * rho * v * v;
  const double expect = q * s.area * sim::plate::cd90(v / 340.0);   // (the axial term vanishes at 90 degrees)
  CHECK(close(sim::norm(l.force), expect, 1e-9));
  CHECK(l.force.x < 0.0 && near_abs(l.force.y, 0.0, 1e-6 * expect) && near_abs(l.force.z, 0.0, 1e-6 * expect));   // along the axis, against the motion
  CHECK(near_abs(l.alpha, sim::kPi / 2.0, 1e-12));
  CHECK(close(l.q, q, 1e-12));
}

TFC_TEST(plate_a_stowed_flap_in_axial_flow_carries_only_its_friction) {
  const sim::SurfaceSpec s = flap_at(90.0);
  const sim::SurfaceLoad l = sim::surface_load(s, 0.0, sim::V3{200.0, 0.0, 0.0}, 1.2, 340.0, 1.0);
  const double q = 0.5 * 1.2 * 200.0 * 200.0;
  CHECK(close(sim::norm(l.force), q * s.area * 0.02, 1e-9));
  CHECK(l.force.x < 0.0);
  CHECK(near_abs(l.alpha, 0.0, 1e-12));
}

TFC_TEST(plate_a_grid_fin_turned_to_the_stream_makes_a_side_force_by_the_plate_law_and_it_stalls) {
  const sim::SurfaceSpec s = grid_fin_at(90.0);
  const double rho = 0.9;
  const double v = 180.0;
  const double a = 340.0;
  const double q = 0.5 * rho * v * v;
  const double d = 10.0 * sim::kDeg2Rad;
  const sim::SurfaceLoad l = sim::surface_load(s, 10.0, sim::V3{v, 0.0, 0.0}, rho, a, 1.0);
  const sim::SurfaceFrame f = sim::surface_frame(s, 10.0);
  const double mach = v / a;
  const double cn = (sim::plate::grid_cn_alpha(mach) * std::sin(d) * std::cos(d) * sim::plate::stall(d)) + (0.35 * sim::plate::cd90(mach) * std::sin(d) * std::sin(d));
  const double ca = sim::plate::grid_ca(mach) * std::cos(d);
  const sim::V3 expect = (f.normal * (q * s.area * cn)) + (f.chord * (-q * s.area * ca));   // vn < 0 and vc > 0
  CHECK(sim::norm(l.force - expect) < 1e-9 * sim::norm(expect));
  CHECK(l.force.y < 0.0);                    // the cells' axis was turned toward -y (the tangent at 90 degrees), so the fin pushes that way... on the air; the vehicle feels the reaction
  // a stalled plate (50 degrees and beyond) keeps only its cross-flow term
  CHECK(sim::plate::stall(50.0 * sim::kDeg2Rad) == 0.0 && sim::plate::stall(10.0 * sim::kDeg2Rad) == 1.0);
}

TFC_TEST(plate_the_lattice_chokes_through_mach_one) {
  CHECK(sim::plate::grid_ca(1.0) > sim::plate::grid_ca(0.5) && sim::plate::grid_ca(1.0) > sim::plate::grid_ca(2.0));
  CHECK(sim::plate::grid_cn_alpha(1.0) < sim::plate::grid_cn_alpha(0.5) && sim::plate::grid_cn_alpha(1.0) < sim::plate::grid_cn_alpha(1.5));
  CHECK(sim::plate::cd90(0.0) == 1.17 && sim::plate::cd90(99.0) == 1.84 && sim::plate::cd90(1.0) > sim::plate::cd90(0.5));
}

TFC_TEST(plate_a_surface_always_takes_energy_out_of_the_flow) {
  for (const sim::SurfaceKind kind : {sim::SurfaceKind::Flap, sim::SurfaceKind::GridFin}) {
    sim::SurfaceSpec s = flap_at(30.0);
    s.kind = kind;
    for (const double d : {-35.0, -10.0, 0.0, 15.0, 45.0, 80.0}) {
      for (int i = -3; i <= 3; ++i) {
        for (int j = -3; j <= 3; ++j) {
          for (const double sign : {1.0, -1.0}) {
            const sim::V3 v{sign * 150.0, 40.0 * i, 40.0 * j};
            const sim::SurfaceLoad l = sim::surface_load(s, d, v, 0.8, 330.0, 1.0);
            CHECK(sim::dot(l.force, v) <= 1e-9 * sim::norm(l.force) * sim::norm(v));
          }
        }
      }
    }
  }
}

TFC_TEST(plate_no_air_or_no_speed_or_no_area_gives_no_force_and_the_wake_takes_most_of_the_dynamic_pressure) {
  sim::SurfaceSpec s = flap_at(0.0);
  CHECK(sim::norm(sim::surface_load(s, 30.0, sim::V3{0.5, 0.0, 0.0}, 1.2, 340.0, 1.0).force) == 0.0);
  CHECK(sim::norm(sim::surface_load(s, 30.0, sim::V3{100.0, 0.0, 0.0}, 0.0, 340.0, 1.0).force) == 0.0);
  s.area = 0.0;
  CHECK(sim::norm(sim::surface_load(s, 30.0, sim::V3{100.0, 0.0, 0.0}, 1.2, 340.0, 1.0).force) == 0.0);
  CHECK(sim::stream_fraction(sim::V3{0.0, 100.0, 0.0}, sim::V3{0.0, 1.0, 0.0}) == 1.0);
  CHECK(close(sim::stream_fraction(sim::V3{0.0, 100.0, 0.0}, sim::V3{0.0, -1.0, 0.0}), 0.2, 1e-12));
  const double side = sim::stream_fraction(sim::V3{0.0, 100.0, 0.0}, sim::V3{0.0, 0.0, 1.0});
  CHECK(side > 0.2 && side < 1.0);
  CHECK(sim::stream_fraction(sim::V3{0.1, 0.0, 0.0}, sim::V3{0.0, 1.0, 0.0}) == 1.0);   // at rest there is no wake
}

TFC_TEST(parachute_inflates_with_the_square_of_the_time_and_pulls_against_the_motion) {
  sim::Parachute p;
  p.drag_area = 100.0;
  p.inflation_s = 4.0;
  CHECK(sim::parachute_drag_area(p, 10.0) == 0.0);     // not released
  p.opened_at = 10.0;
  CHECK(sim::parachute_drag_area(p, 10.0) == 0.0);
  CHECK(close(sim::parachute_drag_area(p, 12.0), 25.0, 1e-12));   // half way: a quarter of the area
  CHECK(close(sim::parachute_drag_area(p, 14.0), 100.0, 1e-12) && close(sim::parachute_drag_area(p, 99.0), 100.0, 1e-12));
  const sim::V3 f = sim::parachute_force(p, 20.0, sim::V3{0.0, 30.0, 40.0}, 1.0);
  CHECK(close(sim::norm(f), 0.5 * 1.0 * 2500.0 * 100.0, 1e-12));
  CHECK(close(sim::dot(f, sim::V3{0.0, 30.0, 40.0}), -sim::norm(f) * 50.0, 1e-12));
  CHECK(sim::norm(sim::parachute_force(p, 20.0, sim::V3{0.0, 0.01, 0.0}, 1.0)) == 0.0);   // too slow to pull
  p.inflation_s = 0.0;
  CHECK(sim::parachute_drag_area(p, 10.0) == 100.0);   // an instant canopy
}

TFC_TEST(plate_heating_follows_sutton_graves_and_the_wall_radiates_it_away) {
  const double q = sim::stagnation_heat_flux(0.001, 7000.0, 1.0);
  CHECK(close(q, 1.7415e-4 * std::sqrt(0.001) * 7000.0 * 7000.0 * 7000.0, 1e-12));
  CHECK(sim::stagnation_heat_flux(0.001, 7000.0, 0.0) == 0.0);
  const double t = sim::radiative_equilibrium_k(q, 0.85);
  CHECK(close(0.85 * 5.670374419e-8 * std::pow(t, 4.0), q, 1e-9));
  CHECK(sim::radiative_equilibrium_k(0.0, 0.85) == 0.0);
}

// ---- the vehicle with parachutes, surfaces, engines it is told to run ----

TFC_TEST(plant_a_vehicle_under_a_parachute_falls_at_the_terminal_speed_the_drag_area_gives) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.spec.planet.atmosphere = sim::AtmosphereKind::Exponential;
  p.spec.planet.scale_height = 1.0e9;   // constant density 1.225
  sim::ParachuteSpec pc;
  pc.drag_area = 40.0;
  pc.inflation_s = 3.0;
  pc.x_attach = 6.0;
  p.spec.parachutes.push_back(pc);
  sim::Scenario still = aloft(3000.0, 0.0);
  still.wind_scale = 0.0;   // still air
  sim::Vehicle6 v(p, still);
  sim::Controls c;
  c.events = sim::ev::kChute0;
  for (int k = 0; k < 6000; ++k) {
    v.step(0.01, c);
  }
  const double expect = std::sqrt(2.0 * 1000.0 * 9.81 / (1.225 * 40.0));
  CHECK(v.chute_open(0) && close(v.chute_fill(0), 1.0, 1e-9));
  CHECK(close(v.speed(), expect, 0.02));
  // it hangs from the canopy: the nose (where it is attached) is up
  CHECK(sim::rotate(v.state().q, sim::V3{1.0, 0.0, 0.0}).x > 0.9);
}

TFC_TEST(plant_the_engines_a_stage_is_driven_to_run_follow_the_group_mask_and_the_throttle) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.gravity_scale = 0.0;
  p.spec.stages[0].guided = true;
  sim::TankSpec tk;
  tk.propellant = 500.0;
  tk.x_bottom = 0.5;
  tk.radius = 0.5;
  tk.density = 1000.0;
  p.spec.stages[0].tanks.push_back(tk);
  p.spec.engines = {engine(10000.0, 0), engine(20000.0, 1, 0.4)};
  sim::Vehicle6 v(p, aloft(300000.0, 0.0));
  sim::Controls c;
  c.group_mask = 0U;
  v.step(0.01, c);
  CHECK(v.current_loads().thrust == 0.0 && v.engines_on() == 2);   // none commanded
  c.group_mask = 1U;
  c.throttle = 1.0;
  v.step(0.01, c);
  CHECK(close(v.current_loads().thrust, 10000.0, 1e-9));
  c.group_mask = 2U;
  c.throttle = 0.7;
  v.step(0.01, c);
  CHECK(close(v.current_loads().thrust, 0.7 * 20000.0, 1e-9));
  c.throttle = 0.1;   // below the engine's lowest: it holds 40 %
  v.step(0.01, c);
  CHECK(close(v.current_loads().thrust, 0.4 * 20000.0, 1e-9));
  c.group_mask = 3U;
  c.throttle = 1.0;
  v.step(0.01, c);
  CHECK(close(v.current_loads().thrust, 30000.0, 1e-9));
  c.throttle = 0.0;   // no throttle: nothing runs
  v.step(0.01, c);
  CHECK(v.current_loads().thrust == 0.0);
  CHECK(v.stage_ignited(0) && v.engine_starts(0) == 2 && v.engine_starts(1) == 1);   // (the first engine was started twice: groups 1, 2, then 3)
}

TFC_TEST(plant_an_engine_with_one_start_does_not_light_again_and_a_start_can_fail) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.gravity_scale = 0.0;
  p.spec.stages[0].guided = true;
  sim::TankSpec tk;
  tk.propellant = 500.0;
  tk.radius = 0.5;
  tk.density = 1000.0;
  p.spec.stages[0].tanks.push_back(tk);
  sim::EngineSpec e = engine(10000.0, 0);
  e.max_starts = 1;
  p.spec.engines = {e};
  sim::Vehicle6 v(p, aloft(300000.0, 0.0));
  sim::Controls c;
  c.throttle = 1.0;
  c.group_mask = 1U;
  v.step(0.01, c);
  CHECK(v.current_loads().thrust > 0.0);
  c.group_mask = 0U;
  v.step(0.01, c);
  CHECK(v.current_loads().thrust == 0.0);
  c.group_mask = 1U;
  v.step(0.01, c);
  CHECK(v.current_loads().thrust == 0.0 && v.engine_starts(0) == 1);   // out of starts
  // a start that fails: with probability 1 nothing lights, however often it is commanded
  sim::Params p2 = p;
  p2.spec.engines[0].max_starts = 0;
  sim::Scenario sc = aloft(300000.0, 0.0);
  sc.start_failure_prob = 1.0;
  sim::Vehicle6 w(p2, sc);
  sim::Controls c2;
  c2.throttle = 1.0;
  c2.group_mask = 1U;
  for (int k = 0; k < 5; ++k) {
    w.step(0.01, c2);
  }
  CHECK(w.current_loads().thrust == 0.0 && w.engine_starts(0) == 1);   // one attempt, then it waits to be commanded off and on
  c2.group_mask = 0U;
  w.step(0.01, c2);
  c2.group_mask = 1U;
  w.step(0.01, c2);
  CHECK(w.engine_starts(0) == 2);
  // and with probability 0.5 the failures are about half of the attempts (the scenario's generator is deterministic)
  sim::Scenario half = aloft(300000.0, 0.0);
  half.start_failure_prob = 0.5;
  half.start_seed = 99U;
  sim::Vehicle6 h(p2, half);
  int lit = 0;
  for (int k = 0; k < 200; ++k) {
    sim::Controls on;
    on.throttle = 1.0;
    on.group_mask = 1U;
    h.step(0.01, on);
    lit += h.engine_on_command(0) ? 1 : 0;
    sim::Controls off;
    off.group_mask = 0U;
    h.step(0.01, off);
  }
  CHECK(lit > 70 && lit < 130);
}

TFC_TEST(plant_surfaces_stay_stowed_until_deployed_then_follow_the_command_at_their_rate) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.gravity_scale = 0.0;
  sim::SurfaceSpec s = flap_at(0.0);
  s.deployed = false;
  s.stow_deg = 2.0;
  s.rate_dps = 20.0;
  s.min_deg = 0.0;
  s.max_deg = 80.0;
  p.spec.surfaces.push_back(s);
  sim::Vehicle6 v(p, aloft(300000.0, 0.0));
  sim::Controls c;
  c.surface_deg[0] = 60.0;
  v.step(0.1, c);
  CHECK(near_abs(v.surface_deg(0), 2.0, 1e-9) && !v.surface_deployed(0));
  c.events = sim::ev::kDeploySurfaces;
  v.step(1.0, c);
  CHECK(v.surface_deployed(0) && near_abs(v.surface_deg(0), 22.0, 0.05));   // 20 degrees per second from 2
  c.surface_deg[0] = 200.0;   // beyond the travel
  for (int k = 0; k < 500; ++k) {
    v.step(0.01, c);
  }
  CHECK(near_abs(v.surface_deg(0), 80.0, 1e-6));
  c.surface_deg[0] = -50.0;
  for (int k = 0; k < 600; ++k) {
    v.step(0.01, c);
  }
  CHECK(near_abs(v.surface_deg(0), 0.0, 1e-6));
  CHECK(v.surface_deg(99) == 0.0 && !v.surface_deployed(99));
}

TFC_TEST(plant_a_flap_in_the_stream_turns_the_vehicle) {
  // a vehicle flying nose first at 150 m/s with a flap out on one side: the flap's force at its arm turns it about the axis it pushes around
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.gravity_scale = 0.0;
  sim::SurfaceSpec s = flap_at(90.0);   // on the +Z side
  s.x_hinge = 1.0;                      // aft of the centre of gravity (at 3 m)
  s.deployed = true;
  p.spec.surfaces.push_back(s);
  sim::Scenario sc = aloft(2000.0, 0.0);
  sc.initial.v = sim::V3{150.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  sim::Controls c;
  c.surface_deg[0] = 45.0;
  for (int k = 0; k < 20; ++k) {
    v.step(0.01, c);
  }
  const sim::Loads l = v.current_loads();
  CHECK(l.f_aero.x < 0.0);
  CHECK(std::fabs(l.m_aero.y) > 100.0);   // a flap on +Z, off the centre of gravity along x, makes a yaw (about Y) moment
}

TFC_TEST(plant_separation_hands_the_stage_on_with_momentum_and_centre_of_gravity_conserved) {
  // two stages: 10 t at the bottom with 20 t of propellant, 5 t above; at 10 s the lower is let go with a 2 m/s push
  sim::Params p;
  p.ground_contact = false;
  p.landing_model = true;
  sim::StageSpec lo;
  lo.name = "lower";
  lo.dry_mass = 10000.0;
  lo.x_start = 0.0;
  lo.length = 20.0;
  lo.radius = 2.0;
  lo.x_cg_dry = 8.0;
  sim::TankSpec tk;
  tk.propellant = 20000.0;
  tk.x_bottom = 2.0;
  tk.radius = 2.0;
  tk.density = 1000.0;
  lo.tanks.push_back(tk);
  lo.separate_time_s = 10.0;
  lo.separation_dv_ms = 2.0;
  lo.catch_pin_x = 12.0;
  sim::StageSpec hi;
  hi.name = "upper";
  hi.dry_mass = 5000.0;
  hi.x_start = 20.0;
  hi.length = 10.0;
  hi.radius = 2.0;
  hi.x_cg_dry = 25.0;
  p.spec.stages = {lo, hi};
  p.gravity_scale = 0.0;   // nothing acts on the two but the push between them
  sim::Scenario sc0 = aloft(100000.0, 0.0);
  sc0.initial.v = sim::V3{0.0, 100.0, 20.0};
  sim::Vehicle6 v(p, sc0);
  for (int k = 0; k < 990; ++k) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.take_detached().empty());
  const sim::State before = v.state();
  const sim::MassProps mp_before = v.mass_props();
  int nsteps = 0;
  for (; nsteps < 12 && v.stage_active(0); ++nsteps) {
    v.step(0.01, 0.0, 0.0);
  }
  std::vector<sim::Detached> d = v.take_detached();
  CHECK(d.size() == 1U && d[0].stage == 0 && d[0].launched);
  if (d.size() != 1U) {
    return;
  }
  sim::Vehicle6 child = sim::Vehicle6::spawn(d[0], p, sc0);
  child.step(v.time() - d[0].t, sim::Controls{});   // the child is made at the instant of the separation, the parent has gone on to the end of the step: bring the child there
  const sim::State& sp = v.state();
  const sim::State& sc = child.state();
  // the masses add up and nothing was lost
  CHECK(close(sp.m + sc.m, before.m, 1e-12));
  CHECK(child.spec().stages.size() == 1U && child.spec().stages[0].x_start == 0.0 && child.spec().engines.empty());
  CHECK(child.spec().stages[0].guided);
  // the centre of gravity of the two is where the whole one was (after one step's motion) and the momentum is the whole one's, plus nothing: the springs act between the two
  const sim::V3 cg = ((sp.r * sp.m) + (sc.r * sc.m)) / (sp.m + sc.m);
  const sim::V3 mom = (sp.v * sp.m) + (sc.v * sc.m);
  const sim::V3 mom_before = before.v * before.m;
  const sim::V3 cg_before = before.r + (before.v * (0.01 * nsteps));   // (the whole has moved on at its constant velocity for the steps it took)
  CHECK(sim::norm(cg - cg_before) < 1e-4);   // (the air at 100 km takes a few micrometres per second off the two)
  CHECK(sim::norm(mom - mom_before) < 0.5);   // the springs act between the two: they add nothing to the whole's momentum
  (void)mp_before;
  // the parent got the push forward and the child the opposite, in proportion to the masses
  const double dv_parent = sim::dot(sp.v - before.v, sim::V3{1.0, 0.0, 0.0});
  const double dv_child = sim::dot(sc.v - before.v, sim::V3{1.0, 0.0, 0.0});
  CHECK(near_abs(dv_parent, 2.0, 1e-6));
  CHECK(close(dv_parent * sp.m, -dv_child * sc.m, 1e-6));
  // the child is further down than the parent
  CHECK(sc.r.x < sp.r.x);
}

TFC_TEST(plant_the_separation_event_lets_the_lowest_stage_go_once) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  p.spec.stages[0].guided = true;
  sim::StageSpec hi = p.spec.stages[0];
  hi.name = "upper";
  hi.x_start = 6.0;
  hi.x_cg_dry = 9.0;
  p.spec.stages.push_back(hi);
  sim::Vehicle6 v(p, aloft(100000.0, 0.0));
  sim::Controls c;
  v.step(0.01, c);
  CHECK(v.take_detached().empty() && v.stage_active(0));
  c.events = sim::ev::kSeparate;
  v.step(0.01, c);
  CHECK(!v.stage_active(0) && v.stage_active(1) && v.take_detached().size() == 1U);
  v.step(0.01, c);   // the bit still set: nothing more happens
  CHECK(v.take_detached().empty() && v.stage_active(1));
  c.events = 0U;
  v.step(0.01, c);
  c.events = sim::ev::kSeparate;   // and the last stage cannot be let go
  v.step(0.01, c);
  CHECK(v.stage_active(1) && v.take_detached().empty());
}

// ---- the ground and the tower ----

namespace {
// A 30 m booster-like body with the pad model off, its centre of gravity 15 m up, a foot 3 m below its aft end when the legs are out; placed `h_base` metres above the ground, falling at `sink`.
sim::Vehicle6 booster_at(double h_base, double sink, bool legs, sim::Tower tower, double tilt_deg = 0.0, double lateral = 0.0) {
  sim::Params p = bare_stage(20000.0, 30.0, 2.0);
  p.ground_contact = true;
  p.landing_model = true;
  p.gravity_scale = 0.0;   // a constant speed through the arms and on to the ground: the touchdown speed is the one given
  p.spec.stages[0].leg_x = -3.0;
  p.spec.stages[0].catch_pin_x = 20.0;
  sim::Scenario sc;
  sc.has_initial = true;
  sc.tower = tower;
  const double t = tilt_deg * sim::kDeg2Rad;
  const sim::Q4 q = sim::from_axis_angle(sim::V3{0.0, 0.0, 1.0}, t);   // tilted toward downrange
  sc.initial.r = sim::V3{sim::kEarthR + h_base + 15.0, 0.0, 0.0};
  sc.initial.v = sim::V3{-sink, lateral, 0.0};
  sc.initial.q = q;
  sim::Vehicle6 v(p, sc);
  if (legs) {
    sim::Controls c;
    c.events = sim::ev::kExtendLegs;
    v.step(0.0001, c);
  }
  return v;
}
void fall(sim::Vehicle6& v, int steps) {
  sim::Controls c = v.controls();
  for (int k = 0; k < steps && !v.landed() && !v.crashed() && !v.caught(); ++k) {
    v.step(0.01, c);
  }
}
}  // namespace

TFC_TEST(ground_a_slow_upright_touchdown_lands_and_a_fast_one_or_a_tilted_one_crashes) {
  sim::Tower none;
  sim::Vehicle6 ok = booster_at(2.0, 2.0, false, none);
  CHECK(near_abs(ok.base_height(), 2.0, 1e-3));
  fall(ok, 300);
  CHECK(ok.landed() && !ok.crashed() && near_abs(ok.base_height(), 0.0, 1e-6) && ok.state().v.x == 0.0);
  sim::Vehicle6 fast = booster_at(2.0, 12.0, false, none);
  fall(fast, 300);
  CHECK(fast.crashed() && !fast.landed());
  sim::Vehicle6 tilted = booster_at(3.0, 1.0, false, none, 25.0);
  fall(tilted, 600);
  CHECK(tilted.crashed());
  sim::Vehicle6 sideways = booster_at(2.0, 1.0, false, none, 0.0, 6.0);
  fall(sideways, 300);
  CHECK(sideways.crashed());
  // a held vehicle stays held
  const double x = ok.state().r.x;
  sim::Controls c;
  ok.step(1.0, c);
  CHECK(ok.landed() && near_abs(ok.state().r.x, x, 1e-9));
}

TFC_TEST(ground_the_legs_put_the_foot_below_the_base_and_the_height_is_that_of_the_lowest_point) {
  sim::Tower none;
  sim::Vehicle6 no_legs = booster_at(10.0, 1.0, false, none);
  sim::Vehicle6 legs = booster_at(10.0, 1.0, true, none);
  CHECK(near_abs(no_legs.base_height(), 10.0, 1e-3) && near_abs(legs.base_height(), 7.0, 1e-3) && legs.legs_out() && !no_legs.legs_out());
  // lying down, the body's ends are what is lowest: a body on its side 20 m up has its axis 20 m + the radius off the ground at the lower end
  sim::Params p = bare_stage(20000.0, 30.0, 2.0);
  p.ground_contact = true;
  p.landing_model = true;
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 20.0, 0.0, 0.0};
  sc.initial.q = sim::from_axis_angle(sim::V3{0.0, 0.0, 1.0}, sim::kPi / 2.0);   // the axis along downrange
  sim::Vehicle6 side(p, sc);
  CHECK(near_abs(side.base_height(), 20.0 - 2.0, 1e-3));
}

TFC_TEST(tower_the_arms_catch_a_stage_whose_pins_come_down_through_them_close_enough_and_slowly_enough) {
  sim::Tower t;
  t.enabled = true;
  t.height_m = 30.0;
  t.offset_y_m = 0.0;
  t.offset_z_m = 0.0;
  // the pins are 20 m up the stage: the base is 10 m up when they pass 30 m
  sim::Vehicle6 good = booster_at(10.0 + 1.5, 1.0, false, t);
  fall(good, 400);
  CHECK(good.caught() && !good.catch_missed() && !good.crashed());
  CHECK(near_abs(good.pin_height(), 30.0, 1e-6) && near_abs(good.base_height(), 10.0, 0.01));
  // 3 m off to the side: the arms cannot reach it; it falls on to the ground
  sim::Vehicle6 wide = booster_at(10.0 + 1.5, 1.0, false, t);
  sim::State st = wide.state();
  st.r.y += 3.0;
  wide.set_state(st);
  fall(wide, 2000);
  CHECK(wide.catch_missed() && !wide.caught() && (wide.crashed() || wide.landed()));
  // too fast through the arms
  sim::Vehicle6 fast = booster_at(10.0 + 3.5, 5.0, false, t);
  fall(fast, 2000);
  CHECK(fast.catch_missed() && !fast.caught());
  // too sideways
  sim::Vehicle6 skew = booster_at(10.0 + 1.5, 1.0, false, t, 0.0, 3.0);
  fall(skew, 2000);
  CHECK(skew.catch_missed() && !skew.caught());
  // too tilted
  sim::Vehicle6 lean = booster_at(10.0 + 1.5, 1.0, false, t, 6.0);
  fall(lean, 2000);
  CHECK(lean.catch_missed() && !lean.caught());
  // a tower that is off catches nothing
  sim::Tower off;
  sim::Vehicle6 none = booster_at(10.0 + 1.5, 1.0, false, off);
  fall(none, 2000);
  CHECK(!none.caught() && !none.catch_missed());
}

TFC_TEST(ground_the_vehicle_that_has_not_left_the_pad_keeps_the_pad_model_even_with_the_landing_model_on) {
  sim::Params p = bare_stage(20000.0, 30.0, 2.0);
  p.landing_model = true;
  p.ground_contact = true;
  sim::Vehicle6 v(p, sim::Scenario{});
  CHECK(!v.launched());
  for (int k = 0; k < 10; ++k) {
    v.step(0.01, 0.0, 0.0);
  }
  CHECK(v.on_ground() && !v.launched() && near_abs(v.altitude(), 0.0, 1e-6));   // no thrust, standing: held at the pad as before
  CHECK(close(v.ground_radius(), sim::kEarthR - 15.0, 1e-9));                    // and the ground is where the base is: 15 m below the centre of gravity
}

// ---- the aerodynamics over the full speed range ----

namespace {
sim::Params shaped_body(bool full) {
  sim::Params p = bare_stage(20000.0, 24.0, 1.0);
  p.cd_scale = 1.0;
  p.cn_scale = 1.0;
  p.diameter = 2.0;
  sim::SectionSpec tube;
  tube.kind = sim::SectionKind::Tube;
  tube.x_start = 0.0;
  tube.length = 20.0;
  tube.d_aft = 2.0;
  tube.d_fore = 2.0;
  sim::SectionSpec nose;
  nose.kind = sim::SectionKind::Nose;
  nose.x_start = 20.0;
  nose.length = 4.0;
  nose.d_aft = 2.0;
  nose.d_fore = 0.0;
  nose.nose = sim::NoseShape::TangentOgive;
  p.spec.stages[0].sections = {tube, nose};
  p.spec.stages[0].x_cg_dry = 11.0;
  p.spec.aero.full_regime = full;
  return p;
}
// The aerodynamic load on the body flying at `mach` at 20 km with its axis `alpha_deg` off the velocity, turning at `w` (rad/s about body Z).
sim::Loads load_at(const sim::Params& p, double mach, double alpha_deg, double w_z = 0.0, double sign = 1.0) {
  sim::Scenario sc;
  sc.has_initial = true;
  const double alt = 20000.0;
  const sim::Air air = sim::air_at(alt);
  const double speed = mach * air.sound;
  const double a = alpha_deg * sim::kDeg2Rad;
  // the velocity in the body frame is speed (cos a, sin a, 0); the body is level (identity attitude), so that is the inertial velocity too
  sc.initial.r = sim::V3{sim::kEarthR + alt, 0.0, 0.0};
  sc.initial.v = sim::V3{sign * speed * std::cos(a), speed * std::sin(a), 0.0};
  sc.initial.w = sim::V3{0.0, 0.0, w_z};
  sim::Vehicle6 v(p, sc);
  return v.current_loads();
}
}  // namespace

TFC_TEST(aero_full_regime_off_changes_nothing_and_on_it_agrees_with_the_build_up_at_low_speed_without_turning) {
  const sim::Params off = shaped_body(false);
  const sim::Params on = shaped_body(true);
  // subsonic, no rotation: the cross-flow coefficient differs only by its Mach dependence (1.2 to 1.25 over the range), so the answers agree closely at small angles and within the coefficient's change at large ones
  const sim::Loads a = load_at(off, 0.5, 3.0);
  const sim::Loads b = load_at(on, 0.5, 3.0);
  CHECK(close(a.f_aero.x, b.f_aero.x, 1e-9));
  CHECK(close(a.f_aero.y, b.f_aero.y, 0.03));
  const sim::Loads c = load_at(off, 0.5, 60.0);
  const sim::Loads d = load_at(on, 0.5, 60.0);
  CHECK(close(c.f_aero.y, d.f_aero.y, 0.08));
}

TFC_TEST(aero_at_hypersonic_speed_the_body_follows_the_newtonian_table) {
  const sim::Params p = shaped_body(true);
  sim::VehicleSpec g = p.spec;
  sim::NewtonTable t;
  t.build(g.stages[0].sections, {}, sim::kPi * 1.0 * 1.0);
  const sim::Air air = sim::air_at(20000.0);
  for (const double alpha_deg : {0.0, 20.0, 60.0, 90.0, 150.0}) {
    const sim::Loads l = load_at(p, 12.0, alpha_deg);
    const double q_s = l.dynamic_pressure * sim::kPi;
    double cx = 0.0;
    double cn = 0.0;
    double a0 = 0.0;
    t.at(alpha_deg * sim::kDeg2Rad, cx, cn, a0);
    // the lateral force opposes the lateral velocity (+y here): f_y = -q S cn; the axial force is -q S (cx + skin) so the Newtonian part is within the skin friction (small) of it
    CHECK(near_abs(l.f_aero.y, -q_s * cn, 0.002 * q_s * (1.0 + std::fabs(cn))));
    CHECK(near_abs(l.f_aero.x, -q_s * cx, 0.03 * q_s * (1.0 + std::fabs(cx))));
  }
  (void)air;
}

TFC_TEST(aero_the_blend_between_the_models_is_continuous_in_mach) {
  const sim::Params p = shaped_body(true);
  for (const double alpha_deg : {5.0, 40.0, 90.0}) {
    double last_y = load_at(p, 2.0, alpha_deg).f_aero.y / load_at(p, 2.0, alpha_deg).dynamic_pressure;
    for (double m = 2.1; m <= 7.0; m += 0.1) {
      const sim::Loads l = load_at(p, m, alpha_deg);
      const double y = l.f_aero.y / l.dynamic_pressure;
      CHECK(std::fabs(y - last_y) < 0.12 * (std::fabs(last_y) + 1.0));   // no jump larger than a few percent per tenth of a Mach number
      last_y = y;
    }
  }
}

TFC_TEST(aero_flying_tail_first_the_force_along_the_axis_changes_sign_and_a_turning_body_is_damped) {
  const sim::Params p = shaped_body(true);
  const sim::Loads nose_first = load_at(p, 8.0, 0.0);
  const sim::Loads tail_first = load_at(p, 8.0, 0.0, 0.0, -1.0);
  CHECK(nose_first.f_aero.x < 0.0 && tail_first.f_aero.x > 0.0);
  // a body turning about Z at +0.1 rad/s feels a moment about Z against the turn, at every speed
  for (const double mach : {0.4, 2.0, 8.0}) {
    const sim::Loads still = load_at(p, mach, 20.0, 0.0);
    const sim::Loads turning = load_at(p, mach, 20.0, 0.1);
    CHECK(turning.m_aero.z - still.m_aero.z < 0.0);
    const sim::Loads other = load_at(p, mach, 20.0, -0.1);
    CHECK(other.m_aero.z - still.m_aero.z > 0.0);
  }
  // and no turning means no addition
  const sim::Loads a = load_at(p, 2.0, 20.0, 0.0);
  const sim::Loads b = load_at(shaped_body(true), 2.0, 20.0, 0.0);
  CHECK(a.m_aero.z == b.m_aero.z);
}

// ---- the rest of the plant: jettison by event, surfaces of a stage that has gone, a hop, a vehicle held on a turning planet ----

TFC_TEST(plant_the_jettison_event_lets_the_first_payload_go_once_and_takes_its_mass) {
  sim::Params p = bare_stage(1000.0, 6.0, 0.5);
  sim::PayloadSpec pl;
  pl.name = "satellite";
  pl.mass = 200.0;
  pl.x = 5.0;
  p.spec.payloads.push_back(pl);
  sim::PayloadSpec second = pl;
  second.name = "second";
  second.mass = 50.0;
  p.spec.payloads.push_back(second);
  sim::Vehicle6 v(p, aloft(100000.0, 0.0));
  const double m0 = v.mass();
  sim::Controls c;
  v.step(0.01, c);
  CHECK(near_abs(v.mass(), m0, 1e-9));
  c.events = sim::ev::kJettison;
  v.step(0.01, c);
  CHECK(near_abs(v.mass(), m0 - 200.0, 1e-6));
  v.step(0.01, c);   // the level stays up: only a rising edge lets one go
  CHECK(near_abs(v.mass(), m0 - 200.0, 1e-6));
  c.events = 0U;
  v.step(0.01, c);
  c.events = sim::ev::kJettison;
  v.step(0.01, c);
  CHECK(near_abs(v.mass(), m0 - 250.0, 1e-6));
  c.events = 0U;
  v.step(0.01, c);
  c.events = sim::ev::kJettison;
  v.step(0.01, c);   // nothing left to let go
  CHECK(near_abs(v.mass(), m0 - 250.0, 1e-6));
}

TFC_TEST(plant_the_surfaces_of_a_stage_that_has_gone_leave_with_it_and_the_probe_of_a_surface_command_changes_the_acceleration) {
  sim::Params p;
  p.ground_contact = false;
  p.landing_model = true;
  p.gravity_scale = 0.0;
  sim::StageSpec lo;
  lo.name = "lower";
  lo.dry_mass = 4000.0;
  lo.length = 10.0;
  lo.radius = 1.5;
  lo.x_cg_dry = 5.0;
  lo.separate_time_s = 1.0;
  sim::StageSpec hi;
  hi.name = "upper";
  hi.dry_mass = 2000.0;
  hi.x_start = 10.0;
  hi.length = 8.0;
  hi.radius = 1.5;
  hi.x_cg_dry = 14.0;
  p.spec.stages = {lo, hi};
  sim::SurfaceSpec flap = flap_at(0.0);
  flap.stage = 0;
  flap.x_hinge = 8.0;
  flap.radius = 1.5;
  flap.channel = 0;
  flap.min_deg = -30.0;
  flap.max_deg = 30.0;
  flap.rate_dps = 100.0;
  p.spec.surfaces.push_back(flap);
  sim::SurfaceSpec grid = grid_fin_at(90.0);
  grid.stage = 1;
  grid.x_hinge = 12.0;
  grid.radius = 1.5;
  grid.channel = 1;
  grid.deployed = false;
  p.spec.surfaces.push_back(grid);
  sim::FinSpec fin;
  fin.stage = 0;
  fin.x_hinge = 1.0;
  fin.area_each = 0.5;
  fin.lift_slope = 3.0;
  p.spec.fins.push_back(fin);
  sim::Scenario sc = aloft(20000.0, 0.0);
  sc.initial.v = sim::V3{0.0, 600.0, 0.0};   // flying downrange fast in thin air, the nose along +X: a large angle of attack
  sim::Vehicle6 v(p, sc);
  // the probe: the command moves the flap, so the angular acceleration it gives is not the one with the flap at zero
  sim::Controls c0;
  sim::Controls c1;
  c1.surface_deg[0] = 15.0;
  v.step(0.001, c0);
  const sim::V3 a0 = v.angular_accel_with(c0);
  const sim::V3 a1 = v.angular_accel_with(c1);
  CHECK(sim::norm(a1 - a0) > 0.0);
  // the stowed grid fin of the upper stage does nothing; the flap and the fins of the lower stage go with it at the separation
  for (int k = 0; k < 150; ++k) {
    v.step(0.01, c0);
  }
  CHECK(!v.stage_active(0) && v.stage_active(1));
  std::vector<sim::Detached> d = v.take_detached();
  CHECK(d.size() == 1U);
  if (d.size() == 1U) {
    CHECK(d[0].spec.surfaces.size() == 1U && d[0].spec.surfaces[0].stage == 0 && near_abs(d[0].spec.surfaces[0].x_hinge, 8.0, 1e-9));
  }
  for (int k = 0; k < 50; ++k) {
    v.step(0.01, c1);   // the remaining vehicle goes on with a surface command for a surface that left
  }
  CHECK(v.mass() < 2000.0 + 1.0);
}

TFC_TEST(ground_a_vehicle_that_touches_the_ground_moving_up_hops_and_a_stage_held_on_a_turning_planet_goes_round_with_it) {
  sim::Tower none;
  // a touch with an upward speed is not a landing
  sim::Vehicle6 hop = booster_at(0.1, -3.0, false, none);   // (a negative sink is up)
  sim::Controls c = hop.controls();
  for (int k = 0; k < 10; ++k) {
    hop.step(0.01, c);
  }
  CHECK(!hop.landed() && !hop.crashed());
  // on a planet that turns, the stage that has landed is carried round: its velocity is the ground's
  sim::Params p = bare_stage(20000.0, 30.0, 2.0);
  p.ground_contact = true;
  p.landing_model = true;
  p.spec.planet.rotation_rate = 1.0e-7;   // (slowly: the ground goes round at 0.6 m/s, inside what a touchdown tolerates)
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 15.3, 0.0, 0.0};   // (the base is 0.3 m up)
  sc.initial.v = sim::V3{-1.0, 0.0, 0.0};
  sim::Vehicle6 v(p, sc);
  for (int k = 0; k < 1200 && !v.landed() && !v.crashed(); ++k) {
    v.step(0.01, sim::Controls{});
  }
  CHECK(v.landed());
  const sim::V3 where = v.state().r;
  v.step(1.0, sim::Controls{});
  CHECK(sim::norm(v.state().r - where) > 0.0 && sim::norm(v.state().r - where) < 1.0);   // carried round by the planet's turning
  CHECK(v.landed());
}
