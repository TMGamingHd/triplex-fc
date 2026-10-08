// SPDX-License-Identifier: MIT
// The IMU model of the datasheet (sim/vehicle/imu_model.hpp, docs/design/IMU_MODEL.md): Gaussian noise of a given density, a wandering bias, drift with temperature, cross-axis sensitivity and
// sample jitter, each checked against what it must have: a noise spectrum (the Allan deviation of white noise falls as one over the square root of the averaging time), the stationary variance and
// the correlation time of a Gauss-Markov process, a drift that is exactly the coefficient times the temperature, a cross-axis term within its limit, a sample that is early by a known fraction of a
// frame. With nothing of it switched on the model is the bench model of the first weeks, sample for sample.
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "tfc_test.hpp"

#include "closed_loop.hpp"
#include "imu_model.hpp"

namespace {

bool near_abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool close(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b)); }

tfc::Vec3 vec(float x, float y, float z) {
  tfc::Vec3 v;
  v.v = {x, y, z};
  return v;
}

// The Allan deviation of a series sampled every `dt` seconds, at the averaging time m dt (non-overlapping clusters).
double allan(const std::vector<double>& y, std::size_t m) {
  const std::size_t n = y.size() / m;
  std::vector<double> means(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    double s = 0.0;
    for (std::size_t j = 0; j < m; ++j) {
      s += y[(i * m) + j];
    }
    means[i] = s / static_cast<double>(m);
  }
  double acc = 0.0;
  for (std::size_t i = 1; i < n; ++i) {
    acc += (means[i] - means[i - 1U]) * (means[i] - means[i - 1U]);
  }
  return std::sqrt(0.5 * acc / static_cast<double>(n - 1U));
}

}  // namespace

TFC_TEST(imu_nothing_switched_on_is_the_bench_model_sample_for_sample) {
  sim::SensorErrors e;  // the defaults: uniform noise of +-0.17 dps and +-3.5 mg
  sim::ImuModel m(e, 0x1234U);
  sim::Lcg noise(0x1234U);
  for (int k = 0; k < 200; ++k) {
    const tfc::Vec3 g_true = vec(static_cast<float>(k) * 0.1F, -2.0F, 0.5F);
    const tfc::Vec3 a_true = vec(0.0F, 0.25F, 1.0F);
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(g_true, a_true, g, a);
    for (unsigned i = 0; i < 3U; ++i) {
      const float eg = (g_true.v[i] * 1.0F) + 0.0F + (0.17F * noise.uniform());
      const float ea = (a_true.v[i] * 1.0F) + 0.0F + (0.0035F * noise.uniform());
      CHECK(g.v[i] == eg && a.v[i] == ea);
    }
  }
}

TFC_TEST(imu_white_noise_of_a_given_density_has_the_rms_density_times_root_bandwidth_and_is_gaussian) {
  sim::SensorErrors e;
  e.gyro_noise_density_dps = 0.005F;
  e.accel_noise_density_g = 0.00006F;
  e.noise_bandwidth_hz = 416.0F;
  sim::ImuModel m(e, 77U);
  const int n = 300000;
  double sg = 0.0;
  double sgg = 0.0;
  double sg4 = 0.0;
  double sa = 0.0;
  double saa = 0.0;
  int beyond = 0;
  const double sigma_g = 0.005 * std::sqrt(416.0);
  for (int k = 0; k < n; ++k) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    const double x = static_cast<double>(g.v[1]);
    sg += x;
    sgg += x * x;
    sg4 += x * x * x * x;
    sa += static_cast<double>(a.v[2]);
    saa += static_cast<double>(a.v[2]) * static_cast<double>(a.v[2]);
    beyond += std::fabs(x) > 3.0 * sigma_g ? 1 : 0;
  }
  const double rms_g = std::sqrt(sgg / n);
  const double rms_a = std::sqrt(saa / n);
  CHECK(close(rms_g, sigma_g, 0.01) && close(rms_a, 0.00006 * std::sqrt(416.0), 0.01));  // 0.102 dps and 1.22 mg
  CHECK(near_abs(sg / n, 0.0, 4.0 * sigma_g / std::sqrt(static_cast<double>(n))) && near_abs(sa / n, 0.0, 4.0 * rms_a / std::sqrt(static_cast<double>(n))));
  CHECK(near_abs((sg4 / n) / (sgg / n * sgg / n), 3.0, 0.08));                         // the kurtosis of a Gaussian
  CHECK(near_abs(static_cast<double>(beyond) / n, 0.0027, 0.0006));                    // 0.27 % beyond three sigma
  // the bench's uniform noise of 0.17 dps is a standard deviation of 0.098 dps: the datasheet's 5 mdps per root hertz over 416 Hz is 0.102
  CHECK(close(0.17 / std::sqrt(3.0), sigma_g, 0.05));
}

TFC_TEST(imu_a_channel_without_a_density_keeps_the_benchs_uniform_noise) {
  // a gyro density with the accelerometer's left at the bench's uniform +-3.5 mg: the accelerometer's noise is uniform (bounded, rms 2.0 mg); and the converse
  for (int variant = 0; variant < 2; ++variant) {
    sim::SensorErrors e;
    if (variant == 0) {
      e.gyro_noise_density_dps = 0.005F;
    } else {
      e.accel_noise_density_g = 0.00006F;
    }
    sim::ImuModel m(e, 8U);
    double sa = 0.0;
    double sg = 0.0;
    double widest_a = 0.0;
    double widest_g = 0.0;
    const int n = 40000;
    for (int k = 0; k < n; ++k) {
      tfc::Vec3 g;
      tfc::Vec3 a;
      m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
      sa += static_cast<double>(a.v[1]) * static_cast<double>(a.v[1]);
      sg += static_cast<double>(g.v[1]) * static_cast<double>(g.v[1]);
      widest_a = std::fmax(widest_a, std::fabs(static_cast<double>(a.v[1])));
      widest_g = std::fmax(widest_g, std::fabs(static_cast<double>(g.v[1])));
    }
    if (variant == 0) {
      CHECK(widest_a <= 0.0035 + 1e-7 && close(std::sqrt(sa / n), 0.0035 / std::sqrt(3.0), 0.03));  // uniform: bounded by its amplitude
      CHECK(close(std::sqrt(sg / n), 0.005 * std::sqrt(416.0), 0.03) && widest_g > 0.3);              // Gaussian: 3 sigma is 0.3 dps and it gets past
    } else {
      CHECK(widest_g <= 0.17 + 1e-6 && close(std::sqrt(sg / n), 0.17 / std::sqrt(3.0), 0.03));
      CHECK(close(std::sqrt(sa / n), 0.00006 * std::sqrt(416.0), 0.03));
    }
  }
}

TFC_TEST(imu_the_allan_deviation_of_white_noise_falls_as_one_over_the_square_root_of_the_averaging_time) {
  sim::SensorErrors e;
  e.gyro_noise_density_dps = 0.005F;
  e.noise_bandwidth_hz = 416.0F;
  sim::ImuModel m(e, 5U);
  std::vector<double> y;
  const int n = 1000000;  // 10 000 s at 100 Hz
  y.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    y.push_back(static_cast<double>(g.v[0]));
  }
  const double sigma = 0.005 * std::sqrt(416.0);
  for (const std::size_t mm : {1U, 10U, 100U, 1000U}) {
    const double tau = 0.01 * static_cast<double>(mm);
    const double expected = sigma * std::sqrt(0.01 / tau);  // sigma_y(tau) = sigma sqrt(dt / tau)
    CHECK(close(allan(y, mm), expected, 0.08));
  }
}

TFC_TEST(imu_the_bias_instability_is_a_gauss_markov_process_with_the_variance_and_the_correlation_time_given) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.gyro_bias_instability_dps = 0.01F;
  e.bias_correlation_s = 10.0F;
  sim::ImuModel m(e, 9U);
  const int n = 1500000;  // 15 000 s: 1500 correlation times
  std::vector<double> b;
  b.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    b.push_back(static_cast<double>(g.v[2]));
  }
  double mean = 0.0;
  for (double x : b) {
    mean += x;
  }
  mean /= n;
  double var = 0.0;
  for (double x : b) {
    var += (x - mean) * (x - mean);
  }
  var /= n;
  CHECK(close(std::sqrt(var), 0.01, 0.1));  // the standard deviation of the process
  const std::size_t lag = 1000U;            // one correlation time
  double c = 0.0;
  for (std::size_t i = 0; i + lag < b.size(); ++i) {
    c += (b[i] - mean) * (b[i + lag] - mean);
  }
  c /= static_cast<double>(b.size() - lag);
  CHECK(near_abs(c / var, std::exp(-1.0), 0.06));  // and it has fallen to 1/e after one correlation time
  // the three axes wander on their own
  sim::ImuModel m2(e, 9U);
  double x0 = 0.0;
  double x1 = 0.0;
  for (int k = 0; k < 2000; ++k) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    m2.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    x0 += static_cast<double>(g.v[0]);
    x1 += static_cast<double>(g.v[1]);
  }
  CHECK(x0 != x1);
}

TFC_TEST(imu_a_wandering_bias_starts_from_its_stationary_distribution_and_each_setting_alone_switches_the_model_on) {
  // the first sample of 600 sensors with a bias instability of 0.01 dps: their spread is 0.01
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.gyro_bias_instability_dps = 0.01F;
  double ss = 0.0;
  const int nodes = 600;
  for (int s = 0; s < nodes; ++s) {
    sim::ImuModel m(e, static_cast<uint32_t>(s) + 100U);
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    ss += static_cast<double>(g.v[0]) * static_cast<double>(g.v[0]);
  }
  CHECK(close(std::sqrt(ss / nodes), 0.01, 0.12));
  // every setting of the datasheet model alone makes the model differ from the bench's zero-error one (whose output, with no noise, is the input)
  const auto differs = [](const sim::SensorErrors& base) {
    sim::ImuModel m(base, 31U);
    tfc::Vec3 g;
    tfc::Vec3 a;
    bool any = false;
    for (int k = 0; k < 4; ++k) {
      m.sample(vec(10.0F + static_cast<float>(k), 20.0F, 30.0F), vec(0.1F, 0.2F, 1.0F + static_cast<float>(k)), g, a);
      any = any || g.v[0] != 10.0F + static_cast<float>(k) || g.v[1] != 20.0F || g.v[2] != 30.0F || a.v[0] != 0.1F || a.v[1] != 0.2F || a.v[2] != 1.0F + static_cast<float>(k);
    }
    return any;
  };
  sim::SensorErrors q;
  q.gyro_noise_amp_dps = 0.0F;
  q.accel_noise_amp_g = 0.0F;
  CHECK(!differs(q));  // (nothing on: the input)
  sim::SensorErrors a = q;
  a.gyro_noise_density_dps = 0.001F;
  CHECK(differs(a));
  a = q;
  a.accel_noise_density_g = 0.00001F;
  CHECK(differs(a));
  a = q;
  a.gyro_bias_instability_dps = 0.01F;
  CHECK(differs(a));
  a = q;
  a.temperature_offset_c = 10.0F;
  a.gyro_bias_tc_dps_per_c = 0.01F;
  CHECK(differs(a));
  a = q;
  a.gyro_cross_axis = 0.01F;
  CHECK(differs(a));
  a = q;
  a.accel_cross_axis = 0.01F;
  CHECK(differs(a));
  a = q;
  a.sample_jitter = 0.5F;
  CHECK(differs(a));
}

TFC_TEST(imu_drift_with_temperature_is_the_coefficient_times_the_degrees_and_the_sensitivity_changes_by_its_own) {
  sim::SensorErrors cold;
  cold.gyro_noise_amp_dps = 0.0F;
  cold.accel_noise_amp_g = 0.0F;
  cold.gyro_bias_tc_dps_per_c = 0.005F;
  cold.accel_bias_tc_g_per_c = 0.0001F;
  cold.gyro_sens_tc_per_c = 0.00007F;
  cold.accel_sens_tc_per_c = 0.00005F;
  cold.gyro_cross_axis = 1e-9F;  // (switches the datasheet model on)
  sim::SensorErrors hot = cold;
  hot.temperature_offset_c = 20.0F;
  double widest_g = 0.0;
  double widest_a = 0.0;
  double sum_g = 0.0;
  double sum_a = 0.0;
  const int nodes = 400;
  for (int s = 0; s < nodes; ++s) {
    sim::ImuModel a(cold, static_cast<uint32_t>(s) + 1U);
    sim::ImuModel b(hot, static_cast<uint32_t>(s) + 1U);
    tfc::Vec3 g0;
    tfc::Vec3 a0;
    tfc::Vec3 g1;
    tfc::Vec3 a1;
    a.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g0, a0);
    b.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g1, a1);
    for (unsigned i = 0; i < 3U; ++i) {
      const double dg = static_cast<double>(g1.v[i]) - static_cast<double>(g0.v[i]);  // the drift of the zero-rate level over 20 degrees
      CHECK(std::fabs(dg) <= 0.005 * 20.0 + 1e-6 && std::fabs(static_cast<double>(a1.v[i]) - static_cast<double>(a0.v[i])) <= 0.0001 * 20.0 + 1e-7);
      widest_g = std::fmax(widest_g, std::fabs(dg));
      sum_g += dg;
      const double da = static_cast<double>(a1.v[i]) - static_cast<double>(a0.v[i]);
      widest_a = std::fmax(widest_a, std::fabs(da));
      sum_a += da;
    }
  }
  CHECK(widest_g > 0.095);                                    // the limit is reached (the coefficient is drawn uniformly within it)
  CHECK(near_abs(sum_g / (3.0 * nodes), 0.0, 0.01));          // and has no preferred sign
  CHECK(widest_a > 0.0019 && near_abs(sum_a / (3.0 * nodes), 0.0, 0.0002));  // the accelerometer's: 0.1 mg per degree over 20 degrees is up to 2 mg
  // the sensitivity: with 100 dps in, the output rises by 100 x 0.00007 x 20 = 0.14 dps beyond the zero-rate drift, and 1 g by 1 x 0.00005 x 20 = 1 mg
  sim::ImuModel a(cold, 3U);
  sim::ImuModel b(hot, 3U);
  tfc::Vec3 g0;
  tfc::Vec3 a0;
  tfc::Vec3 g1;
  tfc::Vec3 a1;
  tfc::Vec3 gz0;
  tfc::Vec3 az0;
  tfc::Vec3 gz1;
  tfc::Vec3 az1;
  sim::ImuModel az(cold, 3U);
  sim::ImuModel bz(hot, 3U);
  a.sample(vec(100.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 1.0F), g0, a0);
  b.sample(vec(100.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 1.0F), g1, a1);
  az.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), gz0, az0);
  bz.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), gz1, az1);
  const double dg100 = (static_cast<double>(g1.v[0]) - static_cast<double>(g0.v[0])) - (static_cast<double>(gz1.v[0]) - static_cast<double>(gz0.v[0]));
  const double da1 = (static_cast<double>(a1.v[2]) - static_cast<double>(a0.v[2])) - (static_cast<double>(az1.v[2]) - static_cast<double>(az0.v[2]));
  CHECK(near_abs(dg100, 0.14, 2e-4) && near_abs(da1, 0.001, 1e-6));
}

TFC_TEST(imu_cross_axis_terms_are_within_their_limit_zero_mean_and_leave_the_diagonal_alone) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.gyro_cross_axis = 0.01F;
  e.accel_cross_axis = 0.005F;
  double widest_g = 0.0;
  double widest_a = 0.0;
  double sum = 0.0;
  const int nodes = 2000;
  for (int s = 0; s < nodes; ++s) {
    sim::ImuModel m(e, static_cast<uint32_t>(s) + 11U);
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(100.0F, 0.0F, 0.0F), vec(1.0F, 0.0F, 0.0F), g, a);
    CHECK(g.v[0] == 100.0F && a.v[0] == 1.0F);  // the axis the input is on is not changed
    for (unsigned i = 1; i < 3U; ++i) {
      CHECK(std::fabs(g.v[i]) <= 1.0F + 1e-5F && std::fabs(a.v[i]) <= 0.005F + 1e-7F);  // 1 % of 100 dps, 0.5 % of 1 g
      widest_g = std::fmax(widest_g, std::fabs(static_cast<double>(g.v[i])));
      widest_a = std::fmax(widest_a, std::fabs(static_cast<double>(a.v[i])));
      sum += static_cast<double>(g.v[i]);
    }
  }
  CHECK(widest_g > 0.98 && widest_a > 0.0049);
  CHECK(near_abs(sum / (2.0 * nodes), 0.0, 0.03));
  // a cross-axis term acts on the other axes' input: a rate about Y leaks into X and Z in proportion
  sim::ImuModel m(e, 11U);
  tfc::Vec3 gx;
  tfc::Vec3 ax;
  m.sample(vec(0.0F, 50.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), gx, ax);
  CHECK(gx.v[1] == 50.0F && (gx.v[0] != 0.0F || gx.v[2] != 0.0F) && std::fabs(gx.v[0]) <= 0.5F + 1e-5F && std::fabs(gx.v[2]) <= 0.5F + 1e-5F);
}

// The three computers' IMU models are seeded with neighbouring numbers. In the bench model that makes their constant errors almost the same (found 7 Oct 2026: the generator's first outputs for
// seeds that differ by one differ by a thousandth of their range), which a real set of parts does not do; the datasheet model draws them independently. The first half is a characterisation of
// the bench model, left as it was so that every earlier flight and every documented number stays; if it is ever changed this test says so.
TFC_TEST(imu_neighbouring_seeds_give_the_bench_model_almost_the_same_constant_error_and_the_datasheet_model_independent_ones) {
  const auto bias_of = [](const sim::SensorErrors& e, uint32_t seed) {
    sim::ImuModel m(e, seed);
    tfc::Vec3 g;
    tfc::Vec3 a;
    m.sample(vec(0.0F, 0.0F, 0.0F), vec(0.0F, 0.0F, 0.0F), g, a);
    return static_cast<double>(g.v[0]);
  };
  sim::SensorErrors bench;
  bench.gyro_noise_amp_dps = 0.0F;
  bench.gyro_bias_dps = 1.0F;
  sim::SensorErrors sheet = bench;
  sheet.gyro_cross_axis = 1e-9F;  // (switches the datasheet model on)
  std::vector<double> x_bench;
  std::vector<double> y_bench;
  std::vector<double> x_sheet;
  std::vector<double> y_sheet;
  for (uint32_t s = 0; s < 300U; ++s) {
    x_bench.push_back(bias_of(bench, 0x1234U + (s * 7U)));
    y_bench.push_back(bias_of(bench, 0x1235U + (s * 7U)));
    x_sheet.push_back(bias_of(sheet, 0x1234U + (s * 7U)));
    y_sheet.push_back(bias_of(sheet, 0x1235U + (s * 7U)));
  }
  const auto corr = [](const std::vector<double>& a, const std::vector<double>& b) {
    double ma = 0.0;
    double mb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      ma += a[i];
      mb += b[i];
    }
    ma /= static_cast<double>(a.size());
    mb /= static_cast<double>(b.size());
    double sab = 0.0;
    double saa = 0.0;
    double sbb = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      sab += (a[i] - ma) * (b[i] - mb);
      saa += (a[i] - ma) * (a[i] - ma);
      sbb += (b[i] - mb) * (b[i] - mb);
    }
    return sab / std::sqrt(saa * sbb);
  };
  CHECK(corr(x_bench, y_bench) > 0.99);            // the bench model: node B's bias is node A's
  CHECK(std::fabs(corr(x_sheet, y_sheet)) < 0.15);  // the datasheet model: independent
  double widest = 0.0;
  for (double v : x_sheet) {
    widest = std::fmax(widest, std::fabs(v));
    CHECK(std::fabs(v) <= 1.0 + 1e-6);
  }
  CHECK(widest > 0.95);
}

TFC_TEST(imu_the_sample_jitter_takes_the_sample_early_by_a_fraction_of_a_frame) {
  sim::SensorErrors e;
  e.gyro_noise_amp_dps = 0.0F;
  e.accel_noise_amp_g = 0.0F;
  e.sample_jitter = 0.5F;
  sim::ImuModel m(e, 21U);
  double sum = 0.0;
  double widest = 0.0;
  const int n = 5000;
  for (int k = 0; k < n; ++k) {
    tfc::Vec3 g;
    tfc::Vec3 a;
    const float truth = 5.0F + (static_cast<float>(k) * 0.1F);  // a ramp of 0.1 dps a frame from 5 dps
    m.sample(vec(truth, 0.0F, 0.0F), vec(0.0F, 0.0F, static_cast<float>(k) * 0.001F), g, a);
    const double early = (static_cast<double>(truth) - static_cast<double>(g.v[0])) / 0.1;  // in frames
    if (k == 0) {
      CHECK(g.v[0] == 5.0F && a.v[2] == 0.0F);  // nothing before the first frame: it is the first frame
      continue;
    }
    CHECK(early >= -2e-3 && early < 0.5 + 2e-3);
    CHECK(near_abs((static_cast<double>(k) * 0.001 - static_cast<double>(a.v[2])) / 0.001, early, 5e-3));  // the accelerometer is early by the same fraction of the frame: one draw serves both
    sum += early;
    widest = std::fmax(widest, early);
  }
  CHECK(near_abs(sum / (n - 1), 0.25, 0.01) && widest > 0.49);  // uniform on [0, 0.5): mean 0.25
}

TFC_TEST(imu_the_presets_are_the_datasheets_numbers_and_the_worst_case_is_worse_in_every_one) {
  const sim::SensorErrors t = sim::ism330dhcx_typical();
  const sim::SensorErrors w = sim::ism330dhcx_maximum();
  CHECK(t.gyro_noise_density_dps == 0.005F && t.accel_noise_density_g == 0.00006F && t.gyro_bias_dps == 1.0F && t.accel_bias_g == 0.010F && t.gyro_scale_err == 0.02F &&
        t.accel_scale_err == 0.02F && t.gyro_cross_axis == 0.01F && t.accel_cross_axis == 0.005F);
  CHECK(near_abs(static_cast<double>(t.gyro_bias_instability_dps), 3.0 / 3600.0, 1e-9) && t.gyro_bias_tc_dps_per_c == 0.005F && t.accel_bias_tc_g_per_c == 0.0001F &&
        t.gyro_sens_tc_per_c == 0.00007F && t.accel_sens_tc_per_c == 0.00005F);
  CHECK(t.gyro_noise_amp_dps == 0.0F && t.accel_noise_amp_g == 0.0F);  // (the datasheet's density replaces the bench's uniform noise)
  CHECK(w.gyro_noise_density_dps == 0.008F && w.accel_noise_density_g == 0.0001F && w.gyro_bias_dps == 3.0F && w.accel_bias_g == 0.065F && w.gyro_bias_tc_dps_per_c == 0.015F &&
        w.accel_bias_tc_g_per_c == 0.0005F && w.gyro_sens_tc_per_c == 0.00015F && w.accel_sens_tc_per_c == 0.0001F);
  CHECK(w.gyro_noise_density_dps > t.gyro_noise_density_dps && w.accel_noise_density_g > t.accel_noise_density_g && w.gyro_bias_dps > t.gyro_bias_dps && w.accel_bias_g > t.accel_bias_g &&
        w.gyro_bias_tc_dps_per_c > t.gyro_bias_tc_dps_per_c && w.accel_bias_tc_g_per_c > t.accel_bias_tc_g_per_c && w.gyro_sens_tc_per_c > t.gyro_sens_tc_per_c &&
        w.accel_sens_tc_per_c > t.accel_sens_tc_per_c);
  CHECK(w.gyro_scale_err == t.gyro_scale_err && w.gyro_cross_axis == t.gyro_cross_axis && w.gyro_bias_instability_dps == t.gyro_bias_instability_dps);  // (the limits the datasheet gives only once)
}

TFC_TEST(imu_the_closed_loop_flies_the_reference_vehicle_with_the_datasheets_sensors) {
  sim::Loop lp;
  lp.frames = 3000U;
  lp.pad_frames = 1500U;  // the pad phase calibrates the gyro bias out
  lp.sensors = sim::ism330dhcx_typical();
  lp.sensors.temperature_offset_c = 20.0F;
  lp.sensors.sample_jitter = 0.2F;
  const sim::Result r = sim::run(lp);
  CHECK(r.finite && r.safe_frames == 0U && !r.crashed && r.max_deg_settled < 3.0);
  const sim::Result again = sim::run(lp);
  CHECK(again.max_deg == r.max_deg && again.rms_deg == r.rms_deg);  // and the same every time
  sim::Loop worst = lp;
  worst.sensors = sim::ism330dhcx_maximum();
  worst.sensors.temperature_offset_c = 40.0F;
  const sim::Result rw = sim::run(worst);
  CHECK(rw.finite && rw.safe_frames == 0U);
}
