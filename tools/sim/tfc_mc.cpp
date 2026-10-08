// SPDX-License-Identifier: MIT
// A Monte Carlo of the closed loop: many flights of the whole software chain (the simulated vehicle, three flight functions, the real ACT logic), each with a random draw of departures from
// the nominal, and what fraction of them fly, how far they stray, and which departure explains it. See docs/design/MONTE_CARLO.md.
//   tfc_mc [--vehicle FILE] [--config FILE] [--flights N] [--seed S] [--threads T] [--limit DEG] [--frames N] [--sensors platform|vehicle] [--imu bench|ism330dhcx_typical|ism330dhcx_maximum] [--no-accel] [--pad FRAMES]
//          [--csv FILE] [--only INDEX] [--list]
// --config: the dispersions (vehicles/dispersions.json is a starting point; every number in it is an assumption). --only re-flies one flight of the run and prints its draws and its result:
// the draws are a function of the seed and the index alone. --list names what can be dispersed.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "montecarlo.hpp"

namespace {

bool read_file(const std::string& path, std::string& out) {
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

const char* why_not(const sim::Result& r, double limit, bool platform_sensors) {
  if (!r.finite) return "not finite";
  if (r.crashed) return "destroyed";
  if (r.liftoff_frame > 300U) return "never left the pad";
  if (r.safe_frames > 0U) return "ACT went to Safe";
  if (r.platform_saturated > 0U && platform_sensors) return "platform saturated";
  if (r.max_deg_settled > limit) return "strayed from the program";
  return "";
}

}  // namespace

int main(int argc, char** argv) {
  sim::mc::Config cfg;
  std::string vehicle_path;
  std::string config_path;
  std::string csv_path;
  bool vehicle_true = false;
  bool no_accel = false;
  std::string imu;
  bool list = false;
  long only = -1;
  uint32_t frames = 0U;
  uint32_t pad = 0U;
  bool flights_given = false;
  bool seed_given = false;
  bool limit_given = false;
  uint32_t flights_arg = 0U;
  uint64_t seed_arg = 0U;
  double limit_arg = 0.0;
  unsigned threads_arg = 0U;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--vehicle" && more) {
      vehicle_path = argv[++i];
    } else if (a == "--config" && more) {
      config_path = argv[++i];
    } else if (a == "--flights" && more) {
      flights_arg = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
      flights_given = true;
    } else if (a == "--seed" && more) {
      seed_arg = std::strtoull(argv[++i], nullptr, 10);
      seed_given = true;
    } else if (a == "--threads" && more) {
      threads_arg = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--limit" && more) {
      limit_arg = std::atof(argv[++i]);
      limit_given = true;
    } else if (a == "--frames" && more) {
      frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--pad" && more) {
      pad = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--sensors" && more) {
      vehicle_true = std::string(argv[++i]) == "vehicle";
    } else if (a == "--imu" && more) {
      imu = argv[++i];
    } else if (a == "--no-accel") {
      no_accel = true;
    } else if (a == "--csv" && more) {
      csv_path = argv[++i];
    } else if (a == "--only" && more) {
      only = std::strtol(argv[++i], nullptr, 10);
    } else if (a == "--list") {
      list = true;
    } else {
      std::fprintf(stderr,
                   "usage: tfc_mc [--vehicle FILE] [--config FILE] [--flights N] [--seed S] [--threads T] [--limit DEG] [--frames N] [--sensors platform|vehicle] [--imu bench|ism330dhcx_typical|ism330dhcx_maximum] [--no-accel] [--pad FRAMES] "
                   "[--csv FILE] [--only INDEX] [--list]\n");
      return 2;
    }
  }
  if (list) {
    for (const sim::mc::Target& t : sim::mc::targets()) {
      std::printf("%-28s %-7s %s\n", t.name, t.unit, t.meaning);
    }
    return 0;
  }
  std::vector<std::string> errors;
  sim::VehicleFile vf;
  if (!vehicle_path.empty()) {
    if (!sim::load_vehicle_file(vehicle_path, vf, errors)) {
      for (const std::string& e : errors) {
        std::fprintf(stderr, "%s\n", e.c_str());
      }
      return 1;
    }
    cfg.base.cfg.params = vf.params;
    cfg.base.cfg.design = vf.params;
    cfg.base.cfg.scenario = vf.scenario;
    cfg.base.cfg.plan = vf.plan;
    cfg.base.frames = static_cast<uint32_t>(vf.plan.trajectory.t_end / 0.01);
  } else {
    cfg.base.frames = 10000U;
  }
  if (!config_path.empty()) {
    std::string text;
    if (!read_file(config_path, text)) {
      std::fprintf(stderr, "cannot read %s\n", config_path.c_str());
      return 1;
    }
    if (!sim::mc::read_config(text, cfg, errors)) {
      for (const std::string& e : errors) {
        std::fprintf(stderr, "%s: %s\n", config_path.c_str(), e.c_str());
      }
      return 1;
    }
  }
  if (flights_given) cfg.flights = flights_arg;
  if (seed_given) cfg.seed = seed_arg;
  if (limit_given) cfg.limit_deg = limit_arg;
  if (threads_arg != 0U) cfg.threads = threads_arg;
  if (frames != 0U) cfg.base.frames = frames;
  if (imu == "bench") {
    cfg.base.sensors = sim::SensorErrors{};
  } else if (imu == "ism330dhcx_typical") {
    cfg.base.sensors = sim::ism330dhcx_typical();
  } else if (imu == "ism330dhcx_maximum") {
    cfg.base.sensors = sim::ism330dhcx_maximum();
  } else if (!imu.empty()) {
    std::fprintf(stderr, "--imu: \"%s\" is not bench, ism330dhcx_typical or ism330dhcx_maximum\n", imu.c_str());
    return 2;
  }
  cfg.base.pad_frames = pad;
  cfg.base.cfg.vehicle_true = vehicle_true;
  cfg.base.estimator.use_accel = !no_accel;
  for (const std::string& b : sim::mc::validate(cfg)) {
    std::fprintf(stderr, "%s\n", b.c_str());
    return 1;
  }

  if (only >= 0) {
    const sim::FlightTables tables = sim::flight_tables(cfg.base.cfg.design, cfg.base.cfg.plan);
    const sim::mc::Flight f = sim::mc::fly_one(cfg, &tables, static_cast<uint32_t>(only));
    std::printf("flight %ld of seed %llu:\n", only, static_cast<unsigned long long>(cfg.seed));
    for (std::size_t i = 0; i < cfg.dispersions.size(); ++i) {
      std::printf("  %-28s %.6g\n", cfg.dispersions[i].name.c_str(), f.draws[i]);
    }
    std::printf("  %s%s%s\n", f.ok ? "flew" : "FAILED", f.ok ? "" : ": ", f.ok ? "" : why_not(f.r, cfg.limit_deg, !cfg.base.cfg.vehicle_true));
    for (const sim::mc::Metric& m : sim::mc::metrics()) {
      std::printf("  %-28s %.6g\n", m.name, m.get(f.r));
    }
    return f.ok ? 0 : 1;
  }

  const std::vector<sim::mc::Flight> fl = sim::mc::run_all(cfg);
  unsigned ok = 0U;
  unsigned causes[6] = {0U, 0U, 0U, 0U, 0U, 0U};
  const char* cause_names[6] = {"not finite", "destroyed", "never left the pad", "ACT went to Safe", "platform saturated", "strayed from the program"};
  for (const sim::mc::Flight& f : fl) {
    if (f.ok) {
      ++ok;
      continue;
    }
    const std::string w = why_not(f.r, cfg.limit_deg, !cfg.base.cfg.vehicle_true);
    for (int c = 0; c < 6; ++c) {
      if (w == cause_names[c]) {
        ++causes[c];
      }
    }
  }
  const auto iv = sim::mc::wilson(ok, static_cast<unsigned>(fl.size()));
  std::printf("Monte Carlo of %zu flights, seed %llu, %u frames, %s sensors%s; a flight is good if, after 3 s, it stays within %.1f degrees of the program and is never in Safe, saturated, destroyed or non-finite.\n",
              fl.size(), static_cast<unsigned long long>(cfg.seed), static_cast<unsigned>(cfg.base.frames), vehicle_true ? "vehicle" : "platform", no_accel ? " (accelerometer correction off)" : "",
              cfg.limit_deg);
  std::printf("Good flights: %u of %zu (%.1f %%; the 95 %% Wilson interval for the probability is %.1f to %.1f %%)\n", ok, fl.size(), 100.0 * ok / static_cast<double>(fl.size()), 100.0 * iv[0], 100.0 * iv[1]);
  for (int c = 0; c < 6; ++c) {
    if (causes[c] > 0U) {
      std::printf("  failed, %s: %u\n", cause_names[c], causes[c]);
    }
  }
  std::printf("\n| Measured | mean | sd | min | median | 95th | 99th | max |\n|---|---|---|---|---|---|---|---|\n");
  for (const sim::mc::Metric& m : sim::mc::metrics()) {
    const sim::mc::Stat s = sim::mc::stat(sim::mc::column(fl, m));
    std::printf("| %s | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g |\n", m.name, s.mean, s.sd, s.min, s.p50, s.p95, s.p99, s.max);
  }
  if (!cfg.dispersions.empty() && fl.size() >= 3U) {
    std::printf("\nWhich departure explains the error: the correlation of each draw with the largest error after 3 s (and with the final speed), over the %zu flights\n\n| Departure | corr. with max error | corr. with final speed |\n|---|---|---|\n", fl.size());
    const std::vector<double> err = sim::mc::column(fl, sim::mc::metrics()[0]);
    const std::vector<double> spd = sim::mc::column(fl, sim::mc::metrics()[4]);
    std::vector<std::size_t> order(cfg.dispersions.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
      order[i] = i;
    }
    std::vector<double> ce(order.size());
    std::vector<double> cs(order.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
      const std::vector<double> d = sim::mc::draws_of(fl, i);
      ce[i] = sim::mc::pearson(d, err);
      cs[i] = sim::mc::pearson(d, spd);
    }
    std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return std::fabs(ce[x]) > std::fabs(ce[y]); });
    for (std::size_t i : order) {
      if (cfg.dispersions[i].kind == sim::mc::Kind::Constant) {
        std::printf("| %s | (a constant: its effect is a random draw per flight, in the sensors) | |\n", cfg.dispersions[i].name.c_str());
      } else {
        std::printf("| %s | %+.2f | %+.2f |\n", cfg.dispersions[i].name.c_str(), ce[i], cs[i]);
      }
    }
  }
  std::vector<std::size_t> worst(fl.size());
  for (std::size_t i = 0; i < worst.size(); ++i) {
    worst[i] = i;
  }
  std::sort(worst.begin(), worst.end(), [&](std::size_t x, std::size_t y) { return fl[x].r.max_deg_settled > fl[y].r.max_deg_settled; });
  std::printf("\nThe five worst flights (fly one again with --only INDEX):\n");
  for (std::size_t k = 0; k < std::min<std::size_t>(5U, worst.size()); ++k) {
    const sim::mc::Flight& f = fl[worst[k]];
    std::printf("  flight %u: max error %.3f deg, %s\n", f.index, f.r.max_deg_settled, f.ok ? "flew" : why_not(f.r, cfg.limit_deg, !cfg.base.cfg.vehicle_true));
  }
  if (!csv_path.empty()) {
    FILE* out = std::fopen(csv_path.c_str(), "w");
    if (out == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
      return 1;
    }
    std::fprintf(out, "index,good");
    for (const sim::mc::Dispersion& d : cfg.dispersions) {
      std::fprintf(out, ",%s", d.name.c_str());
    }
    for (const sim::mc::Metric& m : sim::mc::metrics()) {
      std::string n = m.name;
      std::replace(n.begin(), n.end(), ',', ' ');
      std::fprintf(out, ",%s", n.c_str());
    }
    std::fprintf(out, "\n");
    for (const sim::mc::Flight& f : fl) {
      std::fprintf(out, "%u,%d", f.index, f.ok ? 1 : 0);
      for (double v : f.draws) {
        std::fprintf(out, ",%.9g", v);
      }
      for (const sim::mc::Metric& m : sim::mc::metrics()) {
        std::fprintf(out, ",%.9g", m.get(f.r));
      }
      std::fprintf(out, "\n");
    }
    std::fclose(out);
    std::printf("\nwrote %zu rows to %s\n", fl.size(), csv_path.c_str());
  }
  return 0;
}
