// SPDX-License-Identifier: MIT
// The environment of the vehicle (docs/design/ENVIRONMENT.md): other planets, a rotating planet, the oblateness J2, the atmosphere models and their dispersions, a wind profile, and random turbulence.
// Each is checked against an answer that does not come from the code: the speed of a point on a rotating sphere (omega R cos(latitude)), a vehicle held on the pad going round the pole exactly as the
// planet does, the nodal regression of an inclined orbit (-3/2 J2 n (R/p)^2 cos i), an exponential atmosphere against its definition, the hover threshold on the Moon (thrust equal to the weight
// there), a hot day's density, and the statistics of the turbulence (its variance and its 1/e correlation length). With every setting at its default none of it changes anything.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "spec_io.hpp"
#include "vehicle6.hpp"

namespace {

bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }
bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A bare body of 1000 kg with engines and a tank as asked, no gravity or ground unless the test turns them on.
sim::Params body(const sim::PlanetSpec& planet) {
  sim::Params p;
  p.ground_contact = false;
  p.gravity_scale = 0.0;
  sim::StageSpec st;
  st.name = "body";
  st.dry_mass = 1000.0;
  st.length = 4.0;
  st.radius = 0.5;
  st.x_cg_dry = 2.0;
  p.spec.stages.push_back(st);
  p.spec.planet = planet;
  return p;
}

sim::PlanetSpec preset(const char* name) {
  sim::PlanetSpec p;
  CHECK(sim::planet_preset(name, p));
  return p;
}

}  // namespace

// ---- the planets and their rotation ----

TFC_TEST(environment_the_presets_are_the_planets_and_an_unknown_name_is_refused) {
  CHECK(close(preset("earth").mu / (preset("earth").radius * preset("earth").radius), 9.798, 1e-3));   // surface gravity
  CHECK(close(preset("moon").mu / (preset("moon").radius * preset("moon").radius), 1.624, 5e-3));
  CHECK(close(preset("mars").mu / (preset("mars").radius * preset("mars").radius), 3.713, 5e-3));
  CHECK(preset("moon").atmosphere == sim::AtmosphereKind::None && preset("mars").atmosphere == sim::AtmosphereKind::Exponential && preset("earth").atmosphere == sim::AtmosphereKind::Us1976);
  CHECK(preset("reference").rotation_rate == 0.0 && preset("reference").j2 == 0.0);  // the reference model's own world
  sim::PlanetSpec p;
  CHECK(!sim::planet_preset("jupiter", p));
}

TFC_TEST(environment_a_point_on_a_rotating_planet_moves_at_omega_r_cos_latitude_and_feels_no_wind_from_it) {
  for (const double lat : {0.0, 28.5, 60.0}) {
    sim::Params p = body(preset("earth"));
    sim::Scenario sc;
    sc.site.latitude_deg = lat;
    sc.site.azimuth_deg = 90.0;
    sc.wind_scale = 0.0;
    const sim::Vehicle6 v(p, sc);
    const double expected = 7.2921159e-5 * sim::kEarthR * std::cos(lat * sim::kDeg2Rad);
    CHECK(close(sim::norm(v.state().v), expected, 1e-9));
    CHECK(near_abs(sim::dot(v.state().v, v.state().r), 0.0, 1e-3));                               // along the surface
    CHECK(near_abs(sim::norm(v.state().w), 7.2921159e-5, 1e-12));                                  // and turning with it
    CHECK(near_abs(v.pole().x, std::sin(lat * sim::kDeg2Rad), 1e-12));                             // the pole's inclination to the local vertical
    CHECK(near_abs(v.pole().y, 0.0, 1e-12) && near_abs(v.pole().z, std::cos(lat * sim::kDeg2Rad), 1e-12));  // launching due east: the pole is crossrange, not downrange
    // in the planet's air it is at rest: no wind, no dynamic pressure
    CHECK(v.current_loads().dynamic_pressure < 1e-9);
  }
  // any azimuth: the pole is (sin lat, cos lat cos az, cos lat sin az) in the frame of X up, Y downrange, Z crossrange
  {
    sim::Scenario sa;
    sa.site.latitude_deg = 40.0;
    sa.site.azimuth_deg = 30.0;
    const sim::V3 pole = sim::Vehicle6(body(preset("earth")), sa).pole();
    const double lat = 40.0 * sim::kDeg2Rad;
    const double az = 30.0 * sim::kDeg2Rad;
    CHECK(near_abs(pole.x, std::sin(lat), 1e-12) && near_abs(pole.y, std::cos(lat) * std::cos(az), 1e-12) && near_abs(pole.z, std::cos(lat) * std::sin(az), 1e-12));
    CHECK(near_abs(sim::norm(pole), 1.0, 1e-12));
  }
  // the equator at 465 m/s: the textbook figure
  sim::Params p = body(preset("earth"));
  sim::Scenario sc;
  sc.site.latitude_deg = 0.0;
  CHECK(near_abs(sim::norm(sim::Vehicle6(p, sc).state().v), 465.1, 0.2));
  // a non-rotating planet: at rest in inertial space
  CHECK(sim::norm(sim::Vehicle6(body(preset("reference")), sc).state().v) == 0.0);
}

TFC_TEST(environment_a_vehicle_held_on_the_pad_goes_round_the_pole_with_the_planet) {
  sim::Params p = body(preset("earth"));
  p.ground_contact = true;
  p.gravity_scale = 1.0;
  p.spec.stages[0].tanks.push_back(sim::TankSpec{200.0, 0.5, 0.5, 1000.0});
  sim::EngineSpec e;
  e.thrust_vac = 5000.0;  // far less than the weight: it cannot lift off
  e.exit_area = 0.0;
  p.spec.engines.push_back(e);
  sim::Scenario sc;
  sc.site.latitude_deg = 28.5;
  sc.site.azimuth_deg = 90.0;
  sc.wind_scale = 0.0;
  sim::Vehicle6 v(p, sc);
  const sim::V3 r0 = v.state().r;
  const sim::V3 pole = v.pole();
  for (int k = 0; k < 600; ++k) {
    v.step(1.0, 0.0, 0.0);  // ten minutes
  }
  CHECK(v.on_ground() && near_abs(v.altitude(), 0.0, 1e-6));
  const double angle = 7.2921159e-5 * 600.0;
  const sim::Q4 turn = sim::from_axis_angle(pole, angle);
  const sim::V3 expected = sim::rotate(turn, r0);
  CHECK(sim::norm(v.state().r - expected) < 0.05);  // where the planet's rotation has carried the pad (to 8e-9 of the radius after 300,000 steps of rotation)
  CHECK(near_abs(sim::dot(v.state().r, pole), sim::dot(r0, pole), 0.05));  // at the same distance along the pole
  CHECK(close(sim::norm(v.state().v), 7.2921159e-5 * sim::kEarthR * std::cos(28.5 * sim::kDeg2Rad), 1e-9));
  CHECK(near_abs(sim::norm(v.state().w), 7.2921159e-5, 1e-12));  // the gyros read the planet's rotation
}

// A pad on the equator is carried round a circle, so the pad needs a force toward the centre of only g - omega^2 R, and a lift-off needs less than the weight: omega^2 R / g = 0.35 % for the Earth.
// A thrust of 0.994 times the weight stays down there, 0.999 lifts (it would be 1.0 on a planet that did not turn).
TFC_TEST(environment_on_the_equator_a_lift_off_needs_less_than_the_weight_by_the_centripetal_acceleration_of_the_pad) {
  const sim::PlanetSpec earth = preset("earth");
  const double g = earth.mu / (earth.radius * earth.radius);
  const double need = earth.rotation_rate * earth.rotation_rate * earth.radius / g;
  CHECK(near_abs(need, 0.00346, 0.00002));  // the hand figure
  const auto lifts = [&](double thrust_over_weight) {
    sim::Params p = body(earth);
    p.ground_contact = true;
    p.gravity_scale = 1.0;
    p.spec.planet.j2 = 0.0;  // (J2 changes the weight at the equator by 0.2 %: out of the way of the threshold)
    p.spec.stages[0].tanks.push_back(sim::TankSpec{200.0, 0.5, 0.5, 1000.0});
    sim::EngineSpec e;
    e.thrust_vac = thrust_over_weight * 1200.0 * g;
    e.isp_vac = 1.0e6;  // (burning no propellant to speak of: the weight stays the weight)
    e.exit_area = 0.0;
    p.spec.engines.push_back(e);
    sim::Scenario sc;
    sc.site.latitude_deg = 0.0;
    sc.site.azimuth_deg = 90.0;
    sc.wind_scale = 0.0;
    sim::Vehicle6 v(p, sc);
    for (int k = 0; k < 100; ++k) {
      v.step(0.1, 0.0, 0.0);
    }
    return !v.on_ground() && v.altitude() > 0.0;
  };
  CHECK(!lifts(0.994) && lifts(0.999) && lifts(1.01));
}

// The pad's own velocity is perpendicular to the vertical; its dot product with it is rounding noise of either sign, and the ground must not read that as "moving away" (a vehicle once let go that way
// flew off along the tangent). Held at many sites, with a thrust far under the weight, every one must stay down.
TFC_TEST(environment_a_vehicle_held_on_a_turning_pad_is_never_released_by_rounding_noise) {
  int held = 0;
  int sites = 0;
  for (int i = -5; i <= 5; ++i) {
    for (int j = 0; j < 8; ++j) {
      const double lat = 15.0 * i;
      const double az = 10.0 + (47.0 * j);
      sim::Params p = body(preset("earth"));
      p.ground_contact = true;
      p.gravity_scale = 1.0;
      p.spec.stages[0].tanks.push_back(sim::TankSpec{200.0, 0.5, 0.5, 1000.0});
      sim::EngineSpec e;
      e.thrust_vac = 5000.0;
      e.exit_area = 0.0;
      p.spec.engines.push_back(e);
      sim::Scenario sc;
      sc.site.latitude_deg = lat;
      sc.site.azimuth_deg = az;
      sc.wind_scale = 0.0;
      sim::Vehicle6 v(p, sc);
      for (int k = 0; k < 20; ++k) {
        v.step(0.5, 0.0, 0.0);
      }
      ++sites;
      held += (v.on_ground() && near_abs(v.altitude(), 0.0, 1e-6)) ? 1 : 0;
    }
  }
  CHECK(sites == 88 && held == sites);
}

// ---- J2 ----

TFC_TEST(environment_j2_regresses_the_node_of_an_inclined_orbit_at_the_textbook_rate) {
  sim::PlanetSpec pl = preset("reference");
  pl.j2 = 1.08263e-3;
  pl.atmosphere = sim::AtmosphereKind::None;
  sim::Params p = body(pl);
  p.gravity_scale = 1.0;
  p.max_substep = 1.0;
  const double a = pl.radius + 700000.0;
  const double vc = std::sqrt(pl.mu / a);
  const double inc = 60.0 * sim::kDeg2Rad;
  sim::Scenario sc;
  sc.site.latitude_deg = 90.0;  // the pole along the vehicle's X axis
  sc.has_initial = true;
  sc.initial.r = sim::V3{0.0, a, 0.0};
  sc.initial.v = sim::V3{vc * std::sin(inc), 0.0, vc * std::cos(inc)};
  sim::Vehicle6 v(p, sc);
  CHECK(near_abs(v.pole().x, 1.0, 1e-12));
  const auto node = [&v]() {  // the angle of the ascending node about the pole: the pole x h, measured in the equatorial plane
    const sim::V3 h = sim::cross(v.state().r, v.state().v);
    return std::atan2(h.y, -h.z);
  };
  const double n0 = node();
  const double period = 2.0 * sim::kPi * std::sqrt(a * a * a / pl.mu);
  const int steps = static_cast<int>(2.0 * period);
  for (int k = 0; k < steps; ++k) {
    v.step(1.0, 0.0, 0.0);
  }
  const double elapsed = static_cast<double>(steps);
  const double rate = -1.5 * pl.j2 * std::sqrt(pl.mu / (a * a * a)) * std::pow(pl.radius / a, 2.0) * std::cos(inc);  // rad/s
  const double measured = (node() - n0) / elapsed;
  CHECK(rate < 0.0 && close(measured, rate, 0.06));
  // inclination is held (J2 does not change it, on average)
  const sim::V3 h = sim::cross(v.state().r, v.state().v);
  CHECK(near_abs(std::acos(h.x / sim::norm(h)), inc, 5e-4));
  // with J2 zero the gravity is the point mass's, exactly
  sim::Params q = body(preset("reference"));
  q.gravity_scale = 1.0;
  const sim::Vehicle6 w(q, sc);
  const sim::V3 r{7000000.0, 1000000.0, 500000.0};
  const double rn = sim::norm(r);
  CHECK(w.gravity(r).x == r.x * (-1.0 * q.spec.planet.mu / (rn * rn * rn)));
}

// ---- the atmosphere ----

TFC_TEST(environment_an_exponential_atmosphere_follows_its_definition_and_none_leaves_the_engines_their_vacuum_thrust) {
  sim::PlanetSpec pl = preset("reference");
  pl.atmosphere = sim::AtmosphereKind::Exponential;
  pl.surface_density = 1.0;
  pl.scale_height = 7000.0;
  pl.temperature = 250.0;
  const sim::Vehicle6 v(body(pl));
  const sim::Air a = v.atmosphere(14000.0);
  CHECK(close(a.density, std::exp(-2.0), 1e-12));
  CHECK(close(a.pressure, a.density * 287.053 * 250.0, 1e-12) && close(a.sound, std::sqrt(1.4 * 287.053 * 250.0), 1e-12) && a.temperature == 250.0);
  // Mars: 0.02 kg/m^3 at the surface, 11.1 km scale height
  const sim::Vehicle6 mars(body(preset("mars")));
  CHECK(close(mars.atmosphere(11100.0).density, 0.020 / std::exp(1.0), 1e-12));
  // no atmosphere: no pressure, no density, and the engines give their vacuum thrust at the surface
  sim::PlanetSpec none = preset("reference");
  none.atmosphere = sim::AtmosphereKind::None;
  sim::Params p = body(none);
  p.spec.stages[0].tanks.push_back(sim::TankSpec{100.0, 0.5, 0.5, 1000.0});
  sim::EngineSpec e;
  e.thrust_vac = 20000.0;
  e.exit_area = 1.0;
  p.spec.engines.push_back(e);
  const sim::Vehicle6 w(p);
  CHECK(w.atmosphere(0.0).density == 0.0 && w.atmosphere(0.0).pressure == 0.0);
  CHECK(close(w.current_loads().thrust, 20000.0, 1e-12) && w.current_loads().dynamic_pressure == 0.0);
  // with air, the same engine loses p_a A_e
  sim::Params q = p;
  q.spec.planet = preset("reference");
  CHECK(close(sim::Vehicle6(q).current_loads().thrust, 20000.0 - 101325.0, 1e-9) || sim::Vehicle6(q).current_loads().thrust == 0.0);
}

TFC_TEST(environment_a_denser_or_a_hotter_day_changes_the_air_as_the_gas_law_says) {
  sim::PlanetSpec base = preset("reference");
  sim::PlanetSpec dense = base;
  dense.density_scale = 1.2;
  sim::PlanetSpec hot = base;
  hot.temperature_offset = 30.0;
  const sim::Vehicle6 vb(body(base));
  const sim::Vehicle6 vd(body(dense));
  const sim::Vehicle6 vh(body(hot));
  for (const double alt : {0.0, 5000.0, 12000.0}) {
    const sim::Air b = vb.atmosphere(alt);
    const sim::Air d = vd.atmosphere(alt);
    const sim::Air h = vh.atmosphere(alt);
    CHECK(close(d.density, 1.2 * b.density, 1e-12) && close(d.pressure, 1.2 * b.pressure, 1e-12) && d.temperature == b.temperature);
    CHECK(close(h.density, b.density * b.temperature / (b.temperature + 30.0), 1e-12));  // the same pressure in warmer air
    CHECK(close(h.sound, b.sound * std::sqrt((b.temperature + 30.0) / b.temperature), 1e-12) && close(h.pressure, b.pressure, 1e-12));
  }
}

// ---- the wind ----

TFC_TEST(environment_a_wind_profile_replaces_the_built_in_one_and_is_held_beyond_its_ends) {
  sim::Params p = body(preset("reference"));
  sim::Scenario sc;
  sc.wind_profile = {{1000.0, 2.0}, {3000.0, 12.0}};
  sc.wind_dir = sim::V3{0.0, 1.0, 0.0};
  const sim::Vehicle6 v(p, sc);
  CHECK(close(v.wind_at(2000.0, 0.0).y, 7.0, 1e-12) && v.wind_at(2000.0, 0.0).z == 0.0);
  CHECK(close(v.wind_at(0.0, 0.0).y, 2.0, 1e-12) && close(v.wind_at(9000.0, 0.0).y, 12.0, 1e-12));  // held beyond its ends
  sc.wind_scale = 2.0;
  CHECK(close(sim::Vehicle6(p, sc).wind_at(2000.0, 0.0).y, 14.0, 1e-12));
  // the built-in profile is untouched when none is given: a jet stream of 28 m/s at 12 km
  sim::Scenario none;
  CHECK(close(sim::Vehicle6(p, none).wind_at(12000.0, 0.0).z, 28.0, 1e-12));
}

TFC_TEST(environment_turbulence_has_the_variance_and_the_correlation_length_given_and_is_the_same_every_run) {
  const double sigma = 3.0;
  const double length = 500.0;
  const double speed = 200.0;
  sim::Params p = body(preset("reference"));
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{sim::kEarthR + 100000.0, 0.0, 0.0};
  sc.initial.v = sim::V3{speed, 0.0, 0.0};
  sc.turbulence.sigma_ms = sigma;
  sc.turbulence.scale_length_m = length;
  sc.turbulence.seed = 7U;
  sim::Vehicle6 v(p, sc);
  const int n = 60000;  // 600 s: 240 correlation times
  const int lag = static_cast<int>((length / speed) / 0.01);  // L / V seconds
  std::vector<double> y;
  y.reserve(static_cast<std::size_t>(n));
  double sum = 0.0;
  double sum2 = 0.0;
  for (int k = 0; k < n; ++k) {
    v.step(0.01, 0.0, 0.0);
    y.push_back(v.turbulence().y);
    sum += v.turbulence().y;
    sum2 += v.turbulence().y * v.turbulence().y;
  }
  const double mean = sum / n;
  const double var = (sum2 / n) - (mean * mean);
  CHECK(std::fabs(mean) < 0.35 && close(std::sqrt(var), sigma, 0.12));
  double cov = 0.0;
  for (int k = lag; k < n; ++k) {
    cov += (y[static_cast<std::size_t>(k)] - mean) * (y[static_cast<std::size_t>(k - lag)] - mean);
  }
  cov /= (n - lag);
  CHECK(std::fabs((cov / var) - std::exp(-1.0)) < 0.1);  // at a lag of L / V the correlation is 1/e
  // the wind the vehicle feels includes it
  CHECK(near_abs(v.wind_at(100000.0, 0.0).y - v.turbulence().y, 0.0, 1e-9));
  // the same seed gives the same wind, another seed another one, none gives none
  sim::Vehicle6 w(p, sc);
  for (int k = 0; k < 1000; ++k) {
    w.step(0.01, 0.0, 0.0);
  }
  CHECK(w.turbulence().y == y[999]);
  sc.turbulence.seed = 8U;
  sim::Vehicle6 x(p, sc);
  for (int k = 0; k < 1000; ++k) {
    x.step(0.01, 0.0, 0.0);
  }
  CHECK(x.turbulence().y != y[999]);
  sc.turbulence.sigma_ms = 0.0;
  sim::Vehicle6 z(p, sc);
  z.step(1.0, 0.0, 0.0);
  CHECK(z.turbulence().x == 0.0 && z.turbulence().y == 0.0 && z.turbulence().z == 0.0);
}

// ---- other worlds ----

TFC_TEST(environment_on_the_moon_the_hover_threshold_is_the_weight_there_and_an_orbit_stays_an_orbit) {
  const sim::PlanetSpec moon = preset("moon");
  const double g = moon.mu / (moon.radius * moon.radius);
  const auto lifts = [&](double thrust_over_weight) {
    sim::Params p = body(moon);
    p.ground_contact = true;
    p.gravity_scale = 1.0;
    p.spec.stages[0].tanks.push_back(sim::TankSpec{200.0, 0.5, 0.5, 1000.0});
    sim::EngineSpec e;
    e.thrust_vac = thrust_over_weight * 1200.0 * g;  // the vehicle weighs 1200 kg
    e.exit_area = 0.0;
    p.spec.engines.push_back(e);
    p.spec.planet.rotation_rate = 0.0;  // (the Moon's own rotation is a thirtieth of a degree per minute: out of the way of the threshold)
    sim::Vehicle6 v(p);
    for (int k = 0; k < 100; ++k) {
      v.step(0.1, 0.0, 0.0);
    }
    return !v.on_ground() && v.altitude() > 0.0;
  };
  CHECK(!lifts(0.99) && lifts(1.01));
  // a circular orbit 100 km up keeps its radius and its energy (with the Moon's own J2 it would wander by kilometres: that is what J2 is, so it is off here)
  sim::Params p = body(moon);
  p.spec.planet.j2 = 0.0;
  p.gravity_scale = 1.0;
  p.max_substep = 1.0;
  const double r0 = moon.radius + 100000.0;
  sim::Scenario sc;
  sc.has_initial = true;
  sc.initial.r = sim::V3{r0, 0.0, 0.0};
  sc.initial.v = sim::V3{0.0, std::sqrt(moon.mu / r0), 0.0};
  sim::Vehicle6 v(p, sc);
  const double e0 = (0.5 * sim::dot(v.state().v, v.state().v)) - (moon.mu / r0);
  for (int k = 0; k < 4000; ++k) {
    v.step(1.0, 0.0, 0.0);
  }
  CHECK(near_abs(sim::norm(v.state().r), r0, 5.0) && close((0.5 * sim::dot(v.state().v, v.state().v)) - (moon.mu / sim::norm(v.state().r)), e0, 1e-7));
}

// ---- the files ----

TFC_TEST(environment_files_read_a_preset_with_overrides_the_site_the_turbulence_and_the_wind_profile_and_write_them_back) {
  const std::string text = R"({
    "planet": {"preset": "mars", "j2": 0, "density_scale": 1.1, "temperature_offset_k": -10},
    "stages": [{"dry_mass_kg": 100, "length_m": 2, "radius_m": 0.2}],
    "scenario": {"site": {"latitude_deg": 10, "azimuth_deg": 45}, "turbulence": {"sigma_ms": 2.5, "scale_length_m": 300, "seed": 5},
                 "wind_profile": [[0, 1], [5000, 9]], "start": {"altitude_m": 200000, "circular_orbit": true}}
  })";
  sim::VehicleFile v;
  std::vector<std::string> errors;
  CHECK(sim::read_vehicle(text, v, errors) && errors.empty());
  const sim::PlanetSpec& pl = v.params.spec.planet;
  CHECK(pl.radius == 3396200.0 && pl.j2 == 0.0 && pl.density_scale == 1.1 && pl.temperature_offset == -10.0 && pl.atmosphere == sim::AtmosphereKind::Exponential);
  CHECK(v.scenario.site.latitude_deg == 10.0 && v.scenario.site.azimuth_deg == 45.0 && v.scenario.turbulence.seed == 5U && v.scenario.wind_profile.size() == 2U);
  CHECK(near_abs(v.scenario.initial.r.x, 3396200.0 + 200000.0, 1e-6) && near_abs(v.scenario.initial.v.y, std::sqrt(4.282837e13 / (3396200.0 + 200000.0)), 1e-9));  // the orbit is Mars's, not the Earth's
  const std::string once = sim::write_vehicle(v);
  sim::VehicleFile w;
  CHECK(sim::read_vehicle(once, w, errors) && errors.empty() && sim::write_vehicle(w) == once);
  CHECK(w.params.spec.planet.surface_density == 0.020 && w.scenario.turbulence.sigma_ms == 2.5);
  // refusals
  std::string bad = text;
  bad.replace(bad.find("mars"), 4, "venus");
  CHECK(!sim::read_vehicle(bad, v, errors) && !errors.empty() && errors.back().find("expected \"reference\"") != std::string::npos);
  bad = text;
  bad.replace(bad.find("\"j2\": 0,"), 8, "\"j2\": 0, \"atmosphere\": \"thin\",");
  errors.clear();
  CHECK(!sim::read_vehicle(bad, v, errors) && !errors.empty() && errors[0].find("expected \"us1976\"") != std::string::npos);
  bad = text;
  bad.replace(bad.find("\"density_scale\": 1.1"), 20, "\"density_scale\": 0");
  errors.clear();
  CHECK(!sim::read_vehicle(bad, v, errors) && !errors.empty() && errors[0].find("planet:") != std::string::npos);
}
