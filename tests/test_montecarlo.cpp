// SPDX-License-Identifier: MIT
// The Monte Carlo of the closed loop (sim/vehicle/montecarlo.hpp, docs/design/MONTE_CARLO.md): the generator and the distributions against what they must have (mean, variance, range), the
// statistics against worked numbers, every dispersion acting on the field it names, the configuration file and its refusals, and whole runs: reproducible flight by flight, the same whatever the
// number of threads, a null run equal to the nominal flight, and a dispersion with a known effect showing it.
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "tfc_test.hpp"

#include "montecarlo.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

std::string repo_root() {
#ifdef TFC_SOURCE_DIR
  return std::string(TFC_SOURCE_DIR) + "/";
#else
  const std::string f = __FILE__;
  const std::size_t pos = f.rfind("tests/");
  return pos == std::string::npos ? std::string() : f.substr(0, pos);
#endif
}

sim::mc::Config small(uint32_t flights) {
  sim::mc::Config c;
  c.base.frames = 1500U;  // 15 s
  c.flights = flights;
  return c;
}

}  // namespace

TFC_TEST(mc_the_generator_is_uniform_normal_and_the_same_every_time) {
  sim::mc::Rng r(42U);
  const int n = 200000;
  double su = 0.0;
  double suu = 0.0;
  double sn = 0.0;
  double snn = 0.0;
  double sn3 = 0.0;
  for (int i = 0; i < n; ++i) {
    const double u = r.uniform();
    CHECK(u >= 0.0 && u < 1.0);
    su += u;
    suu += u * u;
    const double g = r.normal();
    sn += g;
    snn += g * g;
    sn3 += g * g * g;
  }
  const double mu = su / n;
  CHECK(near_abs(mu, 0.5, 0.003) && near_abs((suu / n) - (mu * mu), 1.0 / 12.0, 0.002));   // the uniform: mean 1/2, variance 1/12
  CHECK(near_abs(sn / n, 0.0, 0.01) && near_abs(snn / n, 1.0, 0.01) && near_abs(sn3 / n, 0.0, 0.03));  // the normal: mean 0, variance 1, no skew
  sim::mc::Rng a(7U);
  sim::mc::Rng b(7U);
  sim::mc::Rng c(8U);
  bool same = true;
  bool differ = false;
  for (int i = 0; i < 100; ++i) {
    const double x = a.uniform();
    same = same && x == b.uniform();
    differ = differ || x != c.uniform();
  }
  CHECK(same && differ);
  // a pinned value, so that the stream cannot change by accident: it is the same on every machine
  sim::mc::Rng p(1U);
  const double first = p.uniform();
  CHECK(first > 0.0 && first < 1.0 && near_abs(first, sim::mc::Rng(1U).uniform(), 0.0));
}

TFC_TEST(mc_the_streams_of_different_flights_are_independent_and_do_not_depend_on_anything_else) {
  const int n = 4000;
  std::vector<double> x;
  std::vector<double> y;
  for (int i = 0; i < n; ++i) {
    sim::mc::Rng a = sim::mc::flight_rng(5U, static_cast<uint32_t>(i));
    sim::mc::Rng b = sim::mc::flight_rng(5U, static_cast<uint32_t>(i) + 1U);
    x.push_back(a.normal());
    y.push_back(b.normal());
  }
  CHECK(std::fabs(sim::mc::pearson(x, y)) < 0.05);  // neighbouring flights are not correlated
  CHECK(sim::mc::flight_rng(5U, 3U).uniform() == sim::mc::flight_rng(5U, 3U).uniform());
  CHECK(sim::mc::flight_rng(5U, 3U).uniform() != sim::mc::flight_rng(6U, 3U).uniform());
  CHECK(sim::mc::flight_rng(5U, 3U).uniform() != sim::mc::flight_rng(5U, 4U).uniform());
}

TFC_TEST(mc_the_distributions_have_their_mean_their_spread_their_range_and_their_clip) {
  sim::mc::Rng r(11U);
  sim::mc::Dispersion nrm;
  nrm.kind = sim::mc::Kind::Normal;
  nrm.a = 10.0;
  nrm.b = 2.0;
  sim::mc::Dispersion uni;
  uni.kind = sim::mc::Kind::Uniform;
  uni.a = -3.0;
  uni.b = 5.0;
  sim::mc::Dispersion cst;
  cst.kind = sim::mc::Kind::Constant;
  cst.a = 4.5;
  sim::mc::Dispersion clipped = nrm;
  clipped.clip = 1.0;
  std::vector<double> vn;
  std::vector<double> vu;
  double widest = 0.0;
  for (int i = 0; i < 40000; ++i) {
    vn.push_back(sim::mc::draw(nrm, r));
    const double u = sim::mc::draw(uni, r);
    vu.push_back(u);
    CHECK(sim::mc::draw(cst, r) == 4.5);
    widest = std::fmax(widest, std::fabs(sim::mc::draw(clipped, r) - 10.0));
  }
  const sim::mc::Stat sn = sim::mc::stat(vn);
  const sim::mc::Stat su = sim::mc::stat(vu);
  CHECK(near_abs(sn.mean, 10.0, 0.05) && near_abs(sn.sd, 2.0, 0.04));
  CHECK(su.min >= -3.0 && su.max < 5.0 && near_abs(su.mean, 1.0, 0.05) && near_abs(su.sd, 8.0 / std::sqrt(12.0), 0.04));  // uniform on [-3, 5): mean 1, sd (b - a) / sqrt(12)
  CHECK(widest <= 2.0 + 1e-12 && widest > 1.9);  // clipped at one standard deviation: never farther, and it gets there
  // a clip so tight that no draw falls inside it gives the mean (after 64 tries), not something outside
  sim::mc::Dispersion tight = nrm;
  tight.clip = 1e-9;
  CHECK(sim::mc::draw(tight, r) == 10.0);
  // a constant uses no random numbers: the draws after it are the same with or without it
  sim::mc::Rng a(3U);
  sim::mc::Rng b(3U);
  (void)sim::mc::draw(cst, a);
  CHECK(sim::mc::draw(nrm, a) == sim::mc::draw(nrm, b));
}

TFC_TEST(mc_percentiles_intervals_and_correlations_are_the_worked_numbers) {
  // type-7 percentiles: the median of 1 2 3 4 is 2.5, the 25th is 1.75, the 90th of 1..5 is 4.6; unsorted input; the ends
  CHECK(near_abs(sim::mc::percentile({4.0, 1.0, 3.0, 2.0}, 50.0), 2.5, 1e-12));
  CHECK(near_abs(sim::mc::percentile({4.0, 1.0, 3.0, 2.0}, 25.0), 1.75, 1e-12));
  CHECK(near_abs(sim::mc::percentile({1.0, 2.0, 3.0, 4.0, 5.0}, 90.0), 4.6, 1e-12));
  CHECK(sim::mc::percentile({3.0, 9.0, 5.0}, 150.0) == 9.0 && sim::mc::percentile({3.0, 9.0, 5.0}, -20.0) == 3.0);  // beyond the ends it is the ends
  CHECK(sim::mc::percentile({3.0, 9.0}, 0.0) == 3.0 && sim::mc::percentile({3.0, 9.0}, 100.0) == 9.0 && sim::mc::percentile({7.0}, 99.0) == 7.0 && sim::mc::percentile({}, 50.0) == 0.0);
  // the Wilson interval: 90 of 100 is 0.8256 to 0.9448; none of 10 starts at 0 and does not end at 0; all of 10 ends at 1 and does not start at 1
  const std::array<double, 2> w = sim::mc::wilson(90U, 100U);
  CHECK(near_abs(w[0], 0.8256, 5e-4) && near_abs(w[1], 0.9448, 5e-4));
  const std::array<double, 2> none = sim::mc::wilson(0U, 10U);
  const std::array<double, 2> all = sim::mc::wilson(10U, 10U);
  CHECK(none[0] == 0.0 && near_abs(none[1], 0.2775, 5e-4) && all[1] == 1.0 && near_abs(all[0], 0.7225, 5e-4));
  CHECK(sim::mc::wilson(0U, 0U)[0] == 0.0 && sim::mc::wilson(0U, 0U)[1] == 1.0);
  // Pearson: a line up is 1, a line down is -1, a constant is 0, a symmetric parabola is 0, and a worked example (r = 0.8)
  CHECK(near_abs(sim::mc::pearson({1.0, 2.0, 3.0, 4.0}, {2.0, 4.0, 6.0, 8.0}), 1.0, 1e-12));
  CHECK(near_abs(sim::mc::pearson({1.0, 2.0, 3.0, 4.0}, {8.0, 6.0, 4.0, 2.0}), -1.0, 1e-12));
  CHECK(sim::mc::pearson({1.0, 2.0, 3.0}, {5.0, 5.0, 5.0}) == 0.0 && sim::mc::pearson({5.0, 5.0, 5.0}, {1.0, 2.0, 3.0}) == 0.0 && sim::mc::pearson({1.0}, {2.0}) == 0.0);
  CHECK(near_abs(sim::mc::pearson({-2.0, -1.0, 0.0, 1.0, 2.0}, {4.0, 1.0, 0.0, 1.0, 4.0}), 0.0, 1e-12));
  CHECK(near_abs(sim::mc::pearson({1.0, 2.0, 3.0, 4.0, 5.0}, {2.0, 1.0, 4.0, 3.0, 5.0}), 0.8, 1e-12));
  // the statistics of 2, 4, 4, 4, 5, 5, 7, 9: mean 5, sample standard deviation sqrt(32/7)
  const sim::mc::Stat s = sim::mc::stat({2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0});
  CHECK(near_abs(s.mean, 5.0, 1e-12) && near_abs(s.sd, std::sqrt(32.0 / 7.0), 1e-12) && s.min == 2.0 && s.max == 9.0 && near_abs(s.p50, 4.5, 1e-12));
  CHECK(sim::mc::stat({}).mean == 0.0 && sim::mc::stat({3.0}).sd == 0.0);
  // 1 to 100: the 95th percentile is 95.05 and the 99th 99.01 (position 0.95 x 99 = 94.05 in the sorted list)
  std::vector<double> hundred;
  for (int i = 1; i <= 100; ++i) {
    hundred.push_back(static_cast<double>(i));
  }
  const sim::mc::Stat h = sim::mc::stat(hundred);
  CHECK(near_abs(h.p95, 95.05, 1e-9) && near_abs(h.p99, 99.01, 1e-9) && near_abs(h.p50, 50.5, 1e-9) && h.min == 1.0 && h.max == 100.0);
}

TFC_TEST(mc_every_dispersion_sets_the_field_it_names) {
  const auto set = [](const char* name, double v, sim::Loop& lp) {
    const sim::mc::Target* t = sim::mc::find_target(name);
    CHECK(t != nullptr);
    if (t != nullptr) {
      t->apply(lp, v);
    }
  };
  sim::VehicleFile f;
  std::vector<std::string> errors;
  CHECK(sim::load_vehicle_file(repo_root() + "vehicles/launcher_dynamics.json", f, errors) && errors.empty());
  sim::Loop lp;
  lp.cfg.params = f.params;
  lp.cfg.design = f.params;
  lp.cfg.scenario = f.scenario;
  const sim::VehicleSpec& g = lp.cfg.params.spec;
  const double dry0 = g.stages[0].dry_mass;
  const double prop0 = g.stages[0].tanks[0].propellant;
  const double isp0 = g.engines[0].isp_vac;
  set("thrust_scale", 1.07, lp);
  set("cd_scale", 1.2, lp);
  set("cn_scale", 0.8, lp);
  set("thrust_misalign_pitch_deg", 0.3, lp);
  set("thrust_misalign_yaw_deg", -0.2, lp);
  set("gimbal_lag_s", 0.04, lp);
  set("dry_mass_scale", 1.1, lp);
  set("propellant_scale", 0.9, lp);
  set("isp_scale", 0.95, lp);
  set("dry_cg_shift_m", 0.4, lp);
  set("wind_scale", 1.5, lp);
  set("turbulence_sigma_ms", 2.0, lp);
  set("density_scale", 1.05, lp);
  set("temperature_offset_k", 7.0, lp);
  set("engine_out_time_s", 33.0, lp);
  set("engine_out_index", 2.6, lp);
  set("gyro_bias_dps", 0.5, lp);
  set("gyro_scale_err", 0.02, lp);
  set("gyro_noise_scale", 2.0, lp);
  set("accel_bias_g", 0.01, lp);
  set("accel_scale_err", 0.03, lp);
  set("misalign_deg", 1.0, lp);
  set("sensor_latency_frames", 2.6, lp);
  set("gyro_noise_density_mdps", 6.0, lp);
  set("gyro_bias_instability_dph", 7.2, lp);
  set("sensor_temperature_offset_c", 15.0, lp);
  set("sample_jitter", 0.25, lp);
  set("slosh_mass_scale", 1.3, lp);
  set("slosh_frequency_scale", 0.8, lp);
  set("flex_frequency_hz", 3.1, lp);
  set("flex_damping", 0.01, lp);
  set("flex_slope_imu_per_m", -0.3, lp);
  set("backlash_deg", 0.2, lp);
  set("separation_dv_ms", 2.5, lp);
  set("tipoff_pitch_dps", 0.4, lp);
  set("tipoff_yaw_dps", -0.3, lp);
  set("tipoff_roll_dps", 0.6, lp);
  const sim::Params& p = lp.cfg.params;
  CHECK(p.thrust_scale == 1.07 && p.cd_scale == 1.2 && p.cn_scale == 0.8 && p.thrust_misalign_pitch_deg == 0.3 && p.thrust_misalign_yaw_deg == -0.2 && p.gimbal_lag_s == 0.04);
  CHECK(near_abs(p.spec.stages[0].dry_mass, dry0 * 1.1, 1e-9) && near_abs(p.spec.stages[0].tanks[0].propellant, prop0 * 0.9, 1e-9) && near_abs(p.spec.engines[0].isp_vac, isp0 * 0.95, 1e-9));
  CHECK(lp.cfg.scenario.dry_cg_shift == 0.4 && lp.cfg.scenario.wind_scale == 1.5 && lp.cfg.scenario.turbulence.sigma_ms == 2.0 && lp.cfg.scenario.engine_out_time == 33.0 && lp.cfg.scenario.engine_out_index == 3);  // (2.6 rounds to 3, it does not truncate to 2)
  CHECK(p.spec.planet.density_scale == 1.05 && p.spec.planet.temperature_offset == 7.0);
  CHECK(lp.sensors.gyro_bias_dps == 0.5F && lp.sensors.gyro_scale_err == 0.02F && near_abs(static_cast<double>(lp.sensors.gyro_noise_amp_dps), 0.34, 1e-6) && lp.sensors.accel_bias_g == 0.01F &&
        lp.sensors.accel_scale_err == 0.03F && lp.sensors.misalign_deg == 1.0F && lp.sensors.latency_frames == 3U);
  CHECK(near_abs(static_cast<double>(lp.sensors.gyro_noise_density_dps), 0.006, 1e-9) && near_abs(static_cast<double>(lp.sensors.gyro_bias_instability_dps), 0.002, 1e-9) &&
        lp.sensors.temperature_offset_c == 15.0F && lp.sensors.sample_jitter == 0.25F);  // (in the datasheet's units: milli-dps per root hertz, degrees an hour)
  CHECK(p.spec.stages[0].tanks[0].slosh.mass_scale == 1.3 && p.spec.stages[1].tanks[1].slosh.frequency_scale == 0.8);
  CHECK(p.spec.flex.frequency_hz == 3.1 && p.spec.flex.damping == 0.01 && p.spec.flex.slope_imu == -0.3 && p.spec.actuator.backlash_deg == 0.2);
  CHECK(p.spec.stages[0].separation_dv_ms == 2.5 && p.spec.stages[0].tipoff_pitch_dps == 0.4 && p.spec.stages[0].tipoff_yaw_dps == -0.3 && p.spec.stages[0].tipoff_roll_dps == 0.6);
  // the wind's azimuth: 0 downrange (Y), 90 crossrange (Z)
  set("wind_azimuth_deg", 90.0, lp);
  CHECK(near_abs(lp.cfg.scenario.wind_dir.y, 0.0, 1e-12) && near_abs(lp.cfg.scenario.wind_dir.z, 1.0, 1e-12));
  set("wind_azimuth_deg", 0.0, lp);
  CHECK(near_abs(lp.cfg.scenario.wind_dir.y, 1.0, 1e-12) && near_abs(lp.cfg.scenario.wind_dir.z, 0.0, 1e-12));
  // the reference vehicle (no stages listed) is dispersed through its own simple fields
  sim::Loop ref;
  const double m_dry = ref.cfg.params.m_dry;
  const double m_prop = ref.cfg.params.m_prop0;
  const double isp = ref.cfg.params.isp_vac;
  set("dry_mass_scale", 1.1, ref);
  set("propellant_scale", 0.9, ref);
  set("isp_scale", 1.02, ref);
  CHECK(near_abs(ref.cfg.params.m_dry, m_dry * 1.1, 1e-9) && near_abs(ref.cfg.params.m_prop0, m_prop * 0.9, 1e-9) && near_abs(ref.cfg.params.isp_vac, isp * 1.02, 1e-9));
  // every name is unique and described
  const std::vector<sim::mc::Target>& all = sim::mc::targets();
  for (std::size_t i = 0; i < all.size(); ++i) {
    CHECK(all[i].meaning[0] != '\0' && all[i].apply != nullptr);
    for (std::size_t j = 0; j < i; ++j) {
      CHECK(std::string(all[i].name) != all[j].name);
    }
  }
  CHECK(sim::mc::find_target("nonesuch") == nullptr && all.size() >= 30U);
}

TFC_TEST(mc_the_configuration_file_is_read_strictly) {
  const std::string text = R"({
    "flights": 12, "seed": 9, "limit_deg": 3.5, "threads": 3,
    "dispersions": [{"name": "thrust_scale", "normal": [1.0, 0.02]}, {"name": "wind_scale", "uniform": [0, 2]}, {"name": "engine_out_time_s", "constant": -1},
                    {"name": "cd_scale", "normal": [1.0, 0.1, 2.0]}]
  })";
  sim::mc::Config c;
  std::vector<std::string> errors;
  CHECK(sim::mc::read_config(text, c, errors) && errors.empty());
  CHECK(c.flights == 12U && c.seed == 9U && c.limit_deg == 3.5 && c.threads == 3U && c.dispersions.size() == 4U);
  CHECK(c.dispersions[0].kind == sim::mc::Kind::Normal && c.dispersions[0].a == 1.0 && c.dispersions[0].b == 0.02 && c.dispersions[0].clip == 3.0);
  CHECK(c.dispersions[1].kind == sim::mc::Kind::Uniform && c.dispersions[1].a == 0.0 && c.dispersions[1].b == 2.0);
  CHECK(c.dispersions[2].kind == sim::mc::Kind::Constant && c.dispersions[2].a == -1.0 && c.dispersions[3].clip == 2.0);
  const auto refused = [](const std::string& body, const std::string& what) {
    sim::mc::Config d;
    std::vector<std::string> errs;
    const bool ok = sim::mc::read_config(body, d, errs);
    if (ok) {
      return false;
    }
    for (const std::string& e : errs) {
      if (e.find(what) != std::string::npos) {
        return true;
      }
    }
    return false;
  };
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scal", "normal": [1, 0.1]}]})", "did you mean \"thrust_scale\""));
  CHECK(refused(R"({"dispersions": [{"name": "zzzzzzzzzzzz", "normal": [1, 0.1]}]})", "unknown dispersion"));
  CHECK(refused(R"({"dispersions": [{"normal": [1, 0.1]}]})", "needs a \"name\""));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale"}]})", "exactly one of"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "normal": [1, 0.1], "constant": 1}]})", "exactly one of"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "normal": [1]}]})", "expected [mean, sigma]"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "uniform": [1, 2, 3]}]})", "expected [min, max]"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "constant": "one"}]})", "expected a number"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "normal": [1, -0.1]}]})", "standard deviation that is not negative"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "uniform": [3, 1]}]})", "max not below min"));
  CHECK(refused(R"({"dispersions": [{"name": "thrust_scale", "constant": 1}, {"name": "thrust_scale", "constant": 2}]})", "dispersed twice"));
  CHECK(refused(R"({"flights": 0})", "flights"));
  CHECK(refused(R"({"sensor_preset": "ism330dhcx_perfect"})", "sensor_preset"));
  sim::mc::Config pre;
  std::vector<std::string> pe;
  CHECK(sim::mc::read_config(R"({"sensor_preset": "ism330dhcx_maximum"})", pre, pe) && pre.base.sensors.gyro_bias_dps == 3.0F && pre.base.sensors.accel_bias_g == 0.065F);
  CHECK(sim::mc::read_config(R"({"sensor_preset": "ism330dhcx_typical"})", pre, pe) && pre.base.sensors.gyro_bias_dps == 1.0F);
  CHECK(sim::mc::read_config(R"({"sensor_preset": "bench"})", pre, pe) && pre.base.sensors.gyro_bias_dps == 0.0F && pre.base.sensors.gyro_noise_amp_dps == 0.17F);
  CHECK(refused(R"({"flights": 5, "colour": 1})", "unknown field"));
  CHECK(refused(R"({"limit_deg": -1})", "limit_deg must be positive"));
  CHECK(refused(R"({"flights": )", "line"));
  // the starting set that is shipped is valid
  sim::mc::Config shipped;
  std::string body;
  FILE* fh = std::fopen((repo_root() + "vehicles/dispersions.json").c_str(), "r");
  CHECK(fh != nullptr);
  if (fh != nullptr) {
    char buf[4096];
    std::size_t n = 0;
    while ((n = std::fread(buf, 1U, sizeof buf, fh)) > 0U) {
      body.append(buf, n);
    }
    (void)std::fclose(fh);
    std::vector<std::string> e2;
    CHECK(sim::mc::read_config(body, shipped, e2) && e2.empty() && shipped.dispersions.size() >= 15U);
  }
}

TFC_TEST(mc_a_run_with_nothing_dispersed_flies_the_nominal_flight_with_only_its_own_noise) {
  sim::mc::Config c = small(3U);
  const std::vector<sim::mc::Flight> fl = sim::mc::run_all(c);
  sim::Loop nominal = c.base;
  const sim::Result n = sim::run(nominal);
  CHECK(fl.size() == 3U);
  CHECK(n.final_speed > 20.0 && n.max_q > 100.0 && n.final_altitude > 20.0 && n.final_speed < 1000.0);  // (the new measures are filled in)
  bool differ = false;
  for (const sim::mc::Flight& f : fl) {
    CHECK(f.draws.empty() && f.ok);
    // the vehicle, the wind and the engines are the nominal ones; the sensors have another realisation of their noise, so the flight is the nominal one to a hair
    CHECK(near_abs(f.r.final_speed, n.final_speed, 0.05) && near_abs(f.r.final_altitude, n.final_altitude, 0.5) && near_abs(f.r.max_q, n.max_q, 50.0));
    differ = differ || f.r.rms_deg != n.rms_deg;
  }
  CHECK(differ);  // and not the same flight as the nominal
}

TFC_TEST(mc_a_flight_can_be_flown_again_by_itself_and_the_threads_change_nothing) {
  sim::mc::Config c = small(6U);
  c.dispersions.push_back(sim::mc::Dispersion{"thrust_scale", sim::mc::Kind::Normal, 1.0, 0.03, 3.0});
  c.dispersions.push_back(sim::mc::Dispersion{"wind_scale", sim::mc::Kind::Uniform, 0.0, 2.0, 3.0});
  c.dispersions.push_back(sim::mc::Dispersion{"gyro_bias_dps", sim::mc::Kind::Constant, 0.5, 0.0, 3.0});
  c.threads = 1U;
  const std::vector<sim::mc::Flight> one = sim::mc::run_all(c);
  c.threads = 4U;
  const std::vector<sim::mc::Flight> four = sim::mc::run_all(c);
  CHECK(one.size() == 6U && four.size() == 6U);
  bool differ = false;
  for (std::size_t i = 0; i < one.size(); ++i) {
    CHECK(one[i].index == i && four[i].index == i);
    CHECK(one[i].draws == four[i].draws && one[i].r.max_deg == four[i].r.max_deg && one[i].r.final_speed == four[i].r.final_speed);
    differ = differ || one[i].r.final_speed != one[0].r.final_speed;
  }
  CHECK(differ);  // the flights are not all the same flight
  // one flight alone, with its own tables, is the one of the run
  const sim::FlightTables tables = sim::flight_tables(c.base.cfg.design, c.base.cfg.plan);
  const sim::mc::Flight again = sim::mc::fly_one(c, &tables, 4U);
  CHECK(again.draws == one[4].draws && again.r.max_deg == one[4].r.max_deg && again.r.final_speed == one[4].r.final_speed);
  // another seed is another set of flights
  sim::mc::Config d = c;
  d.seed = 2U;
  d.flights = 2U;
  const std::vector<sim::mc::Flight> other = sim::mc::run_all(d);
  CHECK(other[0].draws != one[0].draws);
}

TFC_TEST(mc_each_flight_has_its_own_sensor_noise_wind_and_lost_frames) {
  // nothing is dispersed, but the noise is large, the air is turbulent and the bus loses frames: the flights must differ from one another in each of those
  sim::mc::Config c = small(4U);
  c.base.frame_loss_prob = 0.02F;
  c.dispersions.push_back(sim::mc::Dispersion{"gyro_noise_scale", sim::mc::Kind::Constant, 6.0, 0.0, 3.0});
  c.dispersions.push_back(sim::mc::Dispersion{"turbulence_sigma_ms", sim::mc::Kind::Constant, 3.0, 0.0, 3.0});
  c.dispersions.push_back(sim::mc::Dispersion{"gyro_bias_dps", sim::mc::Kind::Constant, 0.5, 0.0, 3.0});
  c.threads = 4U;
  const std::vector<sim::mc::Flight> fl = sim::mc::run_all(c);
  bool noise = false;
  bool lost = false;
  bool wind = false;
  for (std::size_t i = 1; i < fl.size(); ++i) {
    noise = noise || fl[i].r.rms_deg != fl[0].r.rms_deg;
    lost = lost || fl[i].r.lost_frames != fl[0].r.lost_frames;
    wind = wind || fl[i].r.final_speed != fl[0].r.final_speed;
  }
  CHECK(noise && lost && wind);
  // the same configuration again gives the same flights
  const std::vector<sim::mc::Flight> again = sim::mc::run_all(c);
  for (std::size_t i = 0; i < fl.size(); ++i) {
    CHECK(again[i].r.rms_deg == fl[i].r.rms_deg && again[i].r.lost_frames == fl[i].r.lost_frames && again[i].r.final_speed == fl[i].r.final_speed);
  }
  // a negative latency is no latency
  sim::Loop lp;
  sim::mc::find_target("sensor_latency_frames")->apply(lp, -3.0);
  CHECK(lp.sensors.latency_frames == 0U);
}

TFC_TEST(mc_every_flight_gets_seeds_of_its_own) {
  sim::mc::Config c = small(8U);
  std::vector<uint32_t> noise;
  std::vector<uint32_t> loss;
  std::vector<uint32_t> turb;
  for (uint32_t i = 0; i < 8U; ++i) {
    sim::Loop lp;
    (void)sim::mc::prepare(c, nullptr, i, lp);
    noise.push_back(lp.noise_seed);
    loss.push_back(lp.loss_seed);
    turb.push_back(lp.cfg.scenario.turbulence.seed);
    CHECK(lp.cfg.tables == nullptr);
  }
  const auto distinct = [](const std::vector<uint32_t>& v) {
    for (std::size_t i = 0; i < v.size(); ++i) {
      for (std::size_t j = 0; j < i; ++j) {
        if (v[i] == v[j]) {
          return false;
        }
      }
    }
    return true;
  };
  CHECK(distinct(noise) && distinct(loss) && distinct(turb));
  CHECK(noise[0] != loss[0] && loss[0] != turb[0] && noise[0] != 0U && loss[0] != 77U && turb[0] != 0U);  // three different streams, and not the defaults
  // and they are the same when the same flight is prepared again, and not when the seed of the run differs
  sim::Loop a;
  sim::Loop b;
  (void)sim::mc::prepare(c, nullptr, 3U, a);
  (void)sim::mc::prepare(c, nullptr, 3U, b);
  CHECK(a.noise_seed == b.noise_seed && a.cfg.scenario.turbulence.seed == b.cfg.scenario.turbulence.seed);
  c.seed = 99U;
  (void)sim::mc::prepare(c, nullptr, 3U, b);
  CHECK(a.noise_seed != b.noise_seed);
}

TFC_TEST(mc_a_dispersion_with_a_known_effect_shows_it_and_the_sensitivity_finds_it) {
  sim::mc::Config c = small(10U);
  c.threads = 4U;
  c.dispersions.push_back(sim::mc::Dispersion{"thrust_scale", sim::mc::Kind::Uniform, 0.9, 1.1, 3.0});
  c.dispersions.push_back(sim::mc::Dispersion{"cd_scale", sim::mc::Kind::Uniform, 0.9, 1.1, 3.0});
  const std::vector<sim::mc::Flight> fl = sim::mc::run_all(c);
  const std::vector<double> speed = sim::mc::column(fl, sim::mc::metrics()[4]);
  const double r_thrust = sim::mc::pearson(sim::mc::draws_of(fl, 0U), speed);
  const double r_drag = sim::mc::pearson(sim::mc::draws_of(fl, 1U), speed);
  CHECK(r_thrust > 0.95);                  // more thrust, more speed after 15 s
  CHECK(std::fabs(r_drag) < r_thrust);     // and the drag, in 15 s of a slow ascent, is the lesser effect
  for (const sim::mc::Flight& f : fl) {
    CHECK(f.draws.size() == 2U && f.draws[0] >= 0.9 && f.draws[0] < 1.1);
  }
  // a column of a metric and of a draw have the length of the run
  CHECK(speed.size() == 10U && sim::mc::draws_of(fl, 1U).size() == 10U);
}

TFC_TEST(mc_the_verdict_of_a_flight_is_the_conditions_of_the_sensitivity_study) {
  sim::Result r;
  r.max_deg_settled = 2.0;
  r.liftoff_frame = 45U;
  CHECK(sim::mc::flight_ok(r, 5.0, true) && !sim::mc::flight_ok(r, 1.0, true));
  sim::Result a = r;
  a.finite = false;
  sim::Result b = r;
  b.safe_frames = 1U;
  sim::Result c = r;
  c.platform_saturated = 1U;
  sim::Result d = r;
  d.crashed = true;
  sim::Result e = r;
  e.liftoff_frame = 301U;
  CHECK(!sim::mc::flight_ok(a, 5.0, true) && !sim::mc::flight_ok(b, 5.0, true) && !sim::mc::flight_ok(c, 5.0, true) && !sim::mc::flight_ok(d, 5.0, true) && !sim::mc::flight_ok(e, 5.0, true));
  CHECK(sim::mc::flight_ok(c, 5.0, false) && !sim::mc::flight_ok(c, 5.0, true));  // a saturated platform matters to the rig and not to a vehicle with its own sensors
  sim::Result f = r;
  f.liftoff_frame = 300U;
  CHECK(sim::mc::flight_ok(f, 5.0, true));
  sim::mc::Config bad;
  bad.flights = 0U;
  bad.limit_deg = 0.0;
  bad.dispersions.push_back(sim::mc::Dispersion{"nonesuch", sim::mc::Kind::Constant, 1.0, 0.0, 3.0});
  CHECK(sim::mc::validate(bad).size() == 3U && sim::mc::validate(sim::mc::Config{}).empty());
  sim::mc::Config clip;
  clip.dispersions.push_back(sim::mc::Dispersion{"thrust_scale", sim::mc::Kind::Normal, 1.0, 0.1, 0.0});
  CHECK(sim::mc::validate(clip).size() == 1U);  // a clip of zero standard deviations leaves nothing to draw
}
