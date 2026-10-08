// SPDX-License-Identifier: MIT
// A Monte Carlo of the closed loop (host only). See docs/design/MONTE_CARLO.md.
//
// One flight is the whole software chain of closed_loop.hpp (the simulated vehicle, three flight functions with their IMU models, the real ACT logic) with a random draw of departures from
// the nominal: thrust, drag, masses, winds, sensor errors, a failed engine, and, when the vehicle has them, the slosh, the bending mode and the separation. The draws are a function of the seed
// and the flight's index alone, so a flight can be flown again by itself and the results do not depend on how many threads flew them.
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "closed_loop.hpp"
#include "json.hpp"
#include "spec_io.hpp"

namespace sim::mc {

// ---- random numbers ----

inline uint64_t splitmix(uint64_t& x) {
  x += 0x9E3779B97F4A7C15ULL;
  uint64_t z = x;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// xorshift64* seeded through splitmix64: the same on every machine.
class Rng {
 public:
  explicit Rng(uint64_t seed) {
    uint64_t x = seed;
    s_ = splitmix(x);
    if (s_ == 0U) {
      s_ = 0x2545F4914F6CDD1DULL;
    }
  }
  double uniform() {  // [0, 1)
    s_ ^= s_ >> 12;
    s_ ^= s_ << 25;
    s_ ^= s_ >> 27;
    return static_cast<double>((s_ * 0x2545F4914F6CDD1DULL) >> 11) * (1.0 / 9007199254740992.0);
  }
  double normal() {  // Box-Muller
    double u1 = uniform();
    while (u1 <= 1e-300) {
      u1 = uniform();
    }
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * uniform());
  }

 private:
  uint64_t s_ = 1U;
};

// The stream of flight `index` of a run with `seed`: independent of every other flight's, and of the number of threads.
inline Rng flight_rng(uint64_t seed, uint32_t index) {
  uint64_t x = seed;
  (void)splitmix(x);
  x += static_cast<uint64_t>(index) * 0xD1B54A32D192ED03ULL;
  return Rng(splitmix(x));
}

// ---- distributions ----

enum class Kind : int { Constant = 0, Normal = 1, Uniform = 2 };

struct Dispersion {
  std::string name;  // a Target's name
  Kind kind = Kind::Constant;
  double a = 0.0;    // constant: the value; normal: the mean; uniform: the lower end
  double b = 0.0;    // normal: the standard deviation; uniform: the upper end
  double clip = 3.0; // normal: draws farther than this many standard deviations are drawn again
};

inline double draw(const Dispersion& d, Rng& r) {
  switch (d.kind) {
    case Kind::Normal:
      for (int tries = 0; tries < 64; ++tries) {
        const double x = r.normal();
        if (std::fabs(x) <= d.clip) {
          return d.a + (d.b * x);
        }
      }
      return d.a;
    case Kind::Uniform:
      return d.a + ((d.b - d.a) * r.uniform());
    case Kind::Constant:
      break;
  }
  return d.a;
}

// ---- what can be dispersed ----

struct Target {
  const char* name;
  const char* unit;
  const char* meaning;
  void (*apply)(Loop&, double);
};

inline void scale_dry_mass(Params& p, double v) {
  if (p.spec.empty()) {
    p.m_dry *= v;
  }
  for (StageSpec& st : p.spec.stages) {
    st.dry_mass *= v;
  }
}
inline void scale_propellant(Params& p, double v) {
  if (p.spec.empty()) {
    p.m_prop0 *= v;
  }
  for (StageSpec& st : p.spec.stages) {
    for (TankSpec& t : st.tanks) {
      t.propellant *= v;
    }
  }
}
inline void scale_isp(Params& p, double v) {
  if (p.spec.empty()) {
    p.isp_vac *= v;
  }
  for (EngineSpec& e : p.spec.engines) {
    e.isp_vac *= v;
  }
}

// The names a dispersion file may use. Each acts on the vehicle that is FLOWN; the tables the flight computers carry were designed on the nominal one. Those about the dynamics (slosh,
// bending, the separation, backlash) do nothing to a vehicle without them.
inline const std::vector<Target>& targets() {
  using L = Loop&;
  static const std::vector<Target> table = {
      {"thrust_scale", "x", "every engine's thrust and mass flow times this", [](L l, double v) { l.cfg.params.thrust_scale = v; }},
      {"isp_scale", "x", "every engine's specific impulse times this", [](L l, double v) { scale_isp(l.cfg.params, v); }},
      {"cd_scale", "x", "the axial force coefficient times this", [](L l, double v) { l.cfg.params.cd_scale = v; }},
      {"cn_scale", "x", "the normal-force slope times this", [](L l, double v) { l.cfg.params.cn_scale = v; }},
      {"thrust_misalign_pitch_deg", "deg", "the thrust direction's constant offset, pitch plane", [](L l, double v) { l.cfg.params.thrust_misalign_pitch_deg = v; }},
      {"thrust_misalign_yaw_deg", "deg", "the thrust direction's constant offset, yaw plane", [](L l, double v) { l.cfg.params.thrust_misalign_yaw_deg = v; }},
      {"gimbal_lag_s", "s", "the gimbal servo's first-order lag", [](L l, double v) { l.cfg.params.gimbal_lag_s = v; }},
      {"dry_mass_scale", "x", "every stage's dry mass times this", [](L l, double v) { scale_dry_mass(l.cfg.params, v); }},
      {"propellant_scale", "x", "every tank's propellant times this", [](L l, double v) { scale_propellant(l.cfg.params, v); }},
      {"dry_cg_shift_m", "m", "the structure's centre of gravity moved forward by this", [](L l, double v) { l.cfg.scenario.dry_cg_shift = v; }},
      {"wind_scale", "x", "the mean wind profile times this", [](L l, double v) { l.cfg.scenario.wind_scale = v; }},
      {"wind_azimuth_deg", "deg", "the direction of the mean wind in the horizontal plane (0: downrange, 90: crossrange)",
       [](L l, double v) { l.cfg.scenario.wind_dir = V3{0.0, std::cos(v * kDeg2Rad), std::sin(v * kDeg2Rad)}; }},
      {"turbulence_sigma_ms", "m/s", "the standard deviation of the random wind", [](L l, double v) { l.cfg.scenario.turbulence.sigma_ms = v; }},
      {"density_scale", "x", "the atmosphere's density and pressure times this", [](L l, double v) { l.cfg.params.spec.planet.density_scale = v; }},
      {"temperature_offset_k", "K", "the atmosphere's temperature plus this", [](L l, double v) { l.cfg.params.spec.planet.temperature_offset = v; }},
      {"engine_out_time_s", "s", "the time one engine fails (negative: none)", [](L l, double v) { l.cfg.scenario.engine_out_time = v; }},
      {"engine_out_index", "", "which engine fails (rounded to a whole number)", [](L l, double v) { l.cfg.scenario.engine_out_index = static_cast<int>(std::lround(v)); }},
      {"gyro_bias_dps", "dps", "each gyro's constant bias, up to this on each axis", [](L l, double v) { l.sensors.gyro_bias_dps = static_cast<float>(v); }},
      {"gyro_scale_err", "fraction", "each gyro's gain error, up to this", [](L l, double v) { l.sensors.gyro_scale_err = static_cast<float>(v); }},
      {"gyro_noise_scale", "x", "the gyro noise times this (times the 0.17 dps of the bench model)", [](L l, double v) { l.sensors.gyro_noise_amp_dps = static_cast<float>(0.17 * v); }},
      {"gyro_noise_density_mdps", "mdps/rtHz", "the gyro's white-noise density (the datasheet model; replaces the uniform noise)", [](L l, double v) { l.sensors.gyro_noise_density_dps = static_cast<float>(v * 0.001); }},
      {"gyro_bias_instability_dph", "deg/h", "the standard deviation of the gyro's wandering bias", [](L l, double v) { l.sensors.gyro_bias_instability_dps = static_cast<float>(v / 3600.0); }},
      {"sensor_temperature_offset_c", "C", "the sensors' temperature above 25 degrees", [](L l, double v) { l.sensors.temperature_offset_c = static_cast<float>(v); }},
      {"sample_jitter", "frames", "the sensors' samples are early by up to this fraction of a frame", [](L l, double v) { l.sensors.sample_jitter = static_cast<float>(v); }},
      {"accel_bias_g", "g", "each accelerometer's constant bias, up to this", [](L l, double v) { l.sensors.accel_bias_g = static_cast<float>(v); }},
      {"accel_scale_err", "fraction", "each accelerometer's gain error, up to this", [](L l, double v) { l.sensors.accel_scale_err = static_cast<float>(v); }},
      {"misalign_deg", "deg", "each IMU's mounting error, up to this about each axis", [](L l, double v) { l.sensors.misalign_deg = static_cast<float>(v); }},
      {"sensor_latency_frames", "frames", "how old the sensor sample is (rounded)", [](L l, double v) { l.sensors.latency_frames = static_cast<unsigned>(std::max(0L, std::lround(v))); }},
      {"slosh_mass_scale", "x", "every sloshing tank's sloshing mass times this", [](L l, double v) {
         for (StageSpec& st : l.cfg.params.spec.stages) {
           for (TankSpec& t : st.tanks) {
             t.slosh.mass_scale = v;
           }
         }
       }},
      {"slosh_frequency_scale", "x", "every sloshing tank's frequency times this", [](L l, double v) {
         for (StageSpec& st : l.cfg.params.spec.stages) {
           for (TankSpec& t : st.tanks) {
             t.slosh.frequency_scale = v;
           }
         }
       }},
      {"flex_frequency_hz", "Hz", "the bending mode's frequency", [](L l, double v) { l.cfg.params.spec.flex.frequency_hz = v; }},
      {"flex_damping", "", "the bending mode's damping ratio", [](L l, double v) { l.cfg.params.spec.flex.damping = v; }},
      {"flex_slope_imu_per_m", "1/m", "the bending mode's slope where the IMUs are", [](L l, double v) { l.cfg.params.spec.flex.slope_imu = v; }},
      {"backlash_deg", "deg", "the gimbal servo's play", [](L l, double v) { l.cfg.params.spec.actuator.backlash_deg = v; }},
      {"separation_dv_ms", "m/s", "the push of the first stage's separation", [](L l, double v) {
         if (!l.cfg.params.spec.stages.empty()) {
           l.cfg.params.spec.stages[0].separation_dv_ms = v;
         }
       }},
      {"tipoff_pitch_dps", "dps", "the pitch rate the first stage's separation leaves", [](L l, double v) {
         if (!l.cfg.params.spec.stages.empty()) {
           l.cfg.params.spec.stages[0].tipoff_pitch_dps = v;
         }
       }},
      {"tipoff_yaw_dps", "dps", "the yaw rate the first stage's separation leaves", [](L l, double v) {
         if (!l.cfg.params.spec.stages.empty()) {
           l.cfg.params.spec.stages[0].tipoff_yaw_dps = v;
         }
       }},
      {"tipoff_roll_dps", "dps", "the roll rate the first stage's separation leaves", [](L l, double v) {
         if (!l.cfg.params.spec.stages.empty()) {
           l.cfg.params.spec.stages[0].tipoff_roll_dps = v;
         }
       }},
  };
  return table;
}

inline const Target* find_target(const std::string& name) {
  for (const Target& t : targets()) {
    if (name == t.name) {
      return &t;
    }
  }
  return nullptr;
}

// ---- a run ----

struct Config {
  Loop base;                          // the flight, with the nominal vehicle (cfg.design stays the nominal one)
  std::vector<Dispersion> dispersions;
  uint32_t flights = 100U;
  uint64_t seed = 1U;
  double limit_deg = 5.0;             // a flight fails if, after 3 s, the attitude strays farther than this from the program (and on the other conditions of flight_ok)
  unsigned threads = 1U;
};

// Whether a flight is a good one: the conditions of tfc_sens. The platform's travel is a limit of the rig: a flight with the vehicle's own sensors (`platform_sensors` false) does not have it.
inline bool flight_ok(const Result& r, double limit_deg, bool platform_sensors) {
  return r.finite && r.max_deg_settled <= limit_deg && r.safe_frames == 0U && (r.platform_saturated == 0U || !platform_sensors) && !r.crashed && r.liftoff_frame <= 300U;
}

struct Flight {
  uint32_t index = 0U;
  std::vector<double> draws;  // one per dispersion, in the order of Config::dispersions
  Result r;
  bool ok = false;
};

// The problems with a configuration, in words (empty: none).
inline std::vector<std::string> validate(const Config& c) {
  std::vector<std::string> bad;
  if (c.flights == 0U) {
    bad.push_back("flights must be at least 1");
  }
  if (!(c.limit_deg > 0.0)) {
    bad.push_back("limit_deg must be positive");
  }
  for (std::size_t i = 0; i < c.dispersions.size(); ++i) {
    const Dispersion& d = c.dispersions[i];
    if (find_target(d.name) == nullptr) {
      bad.push_back("dispersions[" + std::to_string(i) + "]: unknown name \"" + d.name + "\"");
    }
    if (d.kind == Kind::Normal && (!(d.b >= 0.0) || !(d.clip > 0.0))) {
      bad.push_back("dispersions[" + std::to_string(i) + "] (" + d.name + "): a normal distribution needs a standard deviation that is not negative and a positive clip");
    }
    if (d.kind == Kind::Uniform && !(d.b >= d.a)) {
      bad.push_back("dispersions[" + std::to_string(i) + "] (" + d.name + "): a uniform distribution needs max not below min");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (c.dispersions[j].name == d.name) {
        bad.push_back("dispersions[" + std::to_string(i) + "]: \"" + d.name + "\" is dispersed twice");
      }
    }
  }
  return bad;
}

// Flight `index` up to the moment it flies: the draws made and applied, and the seeds of its own noise, lost frames and turbulence set. `lp` is the loop to fly.
inline Flight prepare(const Config& c, const FlightTables* tables, uint32_t index, Loop& lp) {
  Flight f;
  f.index = index;
  lp = c.base;
  lp.cfg.tables = tables;
  Rng rng = flight_rng(c.seed, index);
  for (const Dispersion& d : c.dispersions) {
    const double v = draw(d, rng);
    f.draws.push_back(v);
    find_target(d.name)->apply(lp, v);
  }
  uint64_t x = c.seed ^ (0xA24BAED4963EE407ULL * (static_cast<uint64_t>(index) + 1U));
  lp.noise_seed = static_cast<uint32_t>(splitmix(x));  // each flight has its own sensor noise
  lp.loss_seed = static_cast<uint32_t>(splitmix(x));
  lp.cfg.scenario.turbulence.seed = static_cast<uint32_t>(splitmix(x));
  return f;
}

// Flight `index`: draw, apply, fly.
inline Flight fly_one(const Config& c, const FlightTables* tables, uint32_t index) {
  Loop lp;
  Flight f = prepare(c, tables, index, lp);
  f.r = run(lp);
  f.ok = flight_ok(f.r, c.limit_deg, !c.base.cfg.vehicle_true);
  return f;
}

// All the flights, in the order of their index whatever the number of threads.
inline std::vector<Flight> run_all(const Config& c) {
  const FlightTables tables = flight_tables(c.base.cfg.design, c.base.cfg.plan);  // designed once
  std::vector<Flight> out(c.flights);
  std::atomic<uint32_t> next{0U};
  const auto worker = [&]() {
    for (;;) {
      const uint32_t i = next.fetch_add(1U);
      if (i >= c.flights) {
        return;
      }
      out[i] = fly_one(c, &tables, i);
    }
  };
  const unsigned n = std::max(1U, std::min(c.threads, c.flights));
  std::vector<std::thread> pool;
  for (unsigned t = 1U; t < n; ++t) {
    pool.emplace_back(worker);
  }
  worker();
  for (std::thread& t : pool) {
    t.join();
  }
  return out;
}

// ---- statistics ----

// The p-th percentile (0 to 100) by linear interpolation between the order statistics (the usual "type 7": the median of 1, 2, 3, 4 is 2.5).
inline double percentile(std::vector<double> v, double p) {
  if (v.empty()) {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const double pos = std::clamp(p, 0.0, 100.0) / 100.0 * static_cast<double>(v.size() - 1U);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = std::min(lo + 1U, v.size() - 1U);
  return v[lo] + ((pos - static_cast<double>(lo)) * (v[hi] - v[lo]));
}

// The Wilson score interval for k successes in n trials (z = 1.96: 95 %): honest at the ends, where k = n does not mean a probability of 1.
inline std::array<double, 2> wilson(unsigned k, unsigned n, double z = 1.96) {
  if (n == 0U) {
    return {0.0, 1.0};
  }
  const double nn = static_cast<double>(n);
  const double p = static_cast<double>(k) / nn;
  const double den = 1.0 + (z * z / nn);
  const double centre = (p + (z * z / (2.0 * nn))) / den;
  const double half = z * std::sqrt((p * (1.0 - p) / nn) + (z * z / (4.0 * nn * nn))) / den;
  return {std::max(0.0, centre - half), std::min(1.0, centre + half)};
}

// Pearson's correlation coefficient; 0 when either series does not vary.
inline double pearson(const std::vector<double>& x, const std::vector<double>& y) {
  const std::size_t n = std::min(x.size(), y.size());
  if (n < 2U) {
    return 0.0;
  }
  double mx = 0.0;
  double my = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    mx += x[i];
    my += y[i];
  }
  mx /= static_cast<double>(n);
  my /= static_cast<double>(n);
  double sxy = 0.0;
  double sxx = 0.0;
  double syy = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    sxy += (x[i] - mx) * (y[i] - my);
    sxx += (x[i] - mx) * (x[i] - mx);
    syy += (y[i] - my) * (y[i] - my);
  }
  if (!(sxx > 0.0) || !(syy > 0.0)) {
    return 0.0;
  }
  return sxy / std::sqrt(sxx * syy);
}

struct Stat {
  double mean = 0.0;
  double sd = 0.0;  // the sample standard deviation (n - 1)
  double min = 0.0;
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double max = 0.0;
};

inline Stat stat(const std::vector<double>& v) {
  Stat s;
  if (v.empty()) {
    return s;
  }
  double sum = 0.0;
  for (double x : v) {
    sum += x;
  }
  s.mean = sum / static_cast<double>(v.size());
  double ss = 0.0;
  for (double x : v) {
    ss += (x - s.mean) * (x - s.mean);
  }
  s.sd = v.size() > 1U ? std::sqrt(ss / static_cast<double>(v.size() - 1U)) : 0.0;
  s.min = *std::min_element(v.begin(), v.end());
  s.max = *std::max_element(v.begin(), v.end());
  s.p50 = percentile(v, 50.0);
  s.p95 = percentile(v, 95.0);
  s.p99 = percentile(v, 99.0);
  return s;
}

// What a flight measures, for the report and the sensitivity ranking.
struct Metric {
  const char* name;
  double (*get)(const Result&);
};
inline const std::vector<Metric>& metrics() {
  static const std::vector<Metric> m = {
      {"max error after 3 s (deg)", [](const Result& r) { return r.max_deg_settled; }},
      {"lift-off transient (deg)", [](const Result& r) { return r.max_deg_liftoff; }},
      {"rms error (deg)", [](const Result& r) { return r.rms_deg; }},
      {"final altitude (m)", [](const Result& r) { return r.final_altitude; }},
      {"final speed (m/s)", [](const Result& r) { return r.final_speed; }},
      {"max dynamic pressure (Pa)", [](const Result& r) { return r.max_q; }},
  };
  return m;
}

// The column of one metric over the flights, or of one dispersion's draws.
inline std::vector<double> column(const std::vector<Flight>& fl, const Metric& m) {
  std::vector<double> v;
  v.reserve(fl.size());
  for (const Flight& f : fl) {
    v.push_back(m.get(f.r));
  }
  return v;
}
inline std::vector<double> draws_of(const std::vector<Flight>& fl, std::size_t dispersion) {
  std::vector<double> v;
  v.reserve(fl.size());
  for (const Flight& f : fl) {
    v.push_back(f.draws[dispersion]);
  }
  return v;
}

// ---- the configuration file ----

inline bool read_numbers(const Json& j, std::size_t lo, std::size_t hi, std::vector<double>& out) {
  if (j.type != Json::Type::Array || j.items.size() < lo || j.items.size() > hi) {
    return false;
  }
  out.clear();
  for (const Json& it : j.items) {
    if (it.type != Json::Type::Number) {
      return false;
    }
    out.push_back(it.number);
  }
  return true;
}

// {"flights": 200, "seed": 1, "limit_deg": 5, "dispersions": [{"name": "thrust_scale", "normal": [1.0, 0.02]}, {"name": "wind_scale", "uniform": [0, 2]}, {"name": "engine_out_time_s", "constant": -1}]}
// normal: [mean, sigma] or [mean, sigma, clip in sigmas]; uniform: [min, max]; constant: a number. Strict: an unknown field or name is refused, with the field's place.
inline bool read_config(const std::string& text, Config& cfg, std::vector<std::string>& errors) {
  using detail::ObjectReader;
  Json root;
  std::string perr;
  if (!parse_json(text, root, perr)) {
    errors.push_back(perr);
    return false;
  }
  const std::size_t before = errors.size();
  ObjectReader top(root, "", errors);
  int flights = static_cast<int>(cfg.flights);
  if (top.integer("flights", flights)) {
    if (flights < 1) {
      errors.push_back("flights: must be at least 1");
    } else {
      cfg.flights = static_cast<uint32_t>(flights);
    }
  }
  int seed = 0;
  if (top.integer("seed", seed)) {
    cfg.seed = static_cast<uint64_t>(seed);
  }
  top.num("limit_deg", cfg.limit_deg);
  std::string preset;
  if (top.str("sensor_preset", preset)) {
    if (preset == "bench") {
      cfg.base.sensors = SensorErrors{};
    } else if (preset == "ism330dhcx_typical") {
      cfg.base.sensors = ism330dhcx_typical();
    } else if (preset == "ism330dhcx_maximum") {
      cfg.base.sensors = ism330dhcx_maximum();
    } else {
      errors.push_back("sensor_preset: \"" + preset + "\" is not one of \"bench\", \"ism330dhcx_typical\", \"ism330dhcx_maximum\"");
    }
  }
  int threads = static_cast<int>(cfg.threads);
  if (top.integer("threads", threads) && threads >= 1) {
    cfg.threads = static_cast<unsigned>(threads);
  }
  std::vector<const Json*> list;
  if (top.list("dispersions", list)) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      const std::string path = detail::item("", "dispersions", i);
      ObjectReader o(*list[i], path, errors);
      Dispersion d;
      if (!o.str("name", d.name)) {
        errors.push_back(path + ": a dispersion needs a \"name\"");
      } else if (find_target(d.name) == nullptr) {
        std::string best;
        std::size_t best_d = 4U;
        for (const Target& t : targets()) {
          const std::size_t e = detail::edit_distance(d.name, t.name);
          if (e < best_d) {
            best_d = e;
            best = t.name;
          }
        }
        errors.push_back(path + ".name: unknown dispersion \"" + d.name + "\"" + (best.empty() ? std::string(" (tfc_mc --list says which there are)") : " (did you mean \"" + best + "\"?)"));
      }
      int given = 0;
      std::vector<double> nums;
      if (const Json* v = o.get("normal")) {
        ++given;
        if (read_numbers(*v, 2U, 3U, nums)) {
          d.kind = Kind::Normal;
          d.a = nums[0];
          d.b = nums[1];
          d.clip = nums.size() > 2U ? nums[2] : 3.0;
        } else {
          errors.push_back(path + ".normal: expected [mean, sigma] or [mean, sigma, clip]");
        }
      }
      if (const Json* v = o.get("uniform")) {
        ++given;
        if (read_numbers(*v, 2U, 2U, nums)) {
          d.kind = Kind::Uniform;
          d.a = nums[0];
          d.b = nums[1];
        } else {
          errors.push_back(path + ".uniform: expected [min, max]");
        }
      }
      if (const Json* v = o.get("constant")) {
        ++given;
        if (v->type == Json::Type::Number) {
          d.kind = Kind::Constant;
          d.a = v->number;
        } else {
          errors.push_back(path + ".constant: expected a number");
        }
      }
      if (given != 1) {
        errors.push_back(path + ": give exactly one of \"normal\", \"uniform\", \"constant\"");
      }
      o.finish();
      cfg.dispersions.push_back(d);
    }
  }
  top.finish();
  if (errors.size() == before) {
    for (const std::string& b : validate(cfg)) {
      errors.push_back(b);
    }
  }
  return errors.size() == before;
}

}  // namespace sim::mc
