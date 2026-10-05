// SPDX-License-Identifier: MIT
// How often should the replicas resynchronise their state (docs/design/RESYNC.md, TS-23)? For each resync period and each frame-loss rate, fly the closed loop (sim/vehicle/closed_loop.hpp:
// three flight functions, the real ACT logic, the simulated vehicle) over several random loss patterns and report what the period buys and what it costs.
//   tfc_resync [--frames N] [--seeds N] [--periods a,b,c] [--loss p,q,r]
// Output: a markdown table per loss rate. A flight is LOST if ACT enters Safe or the attitude strays more than 5 degrees from the program after the first 3 s.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <string>
#include <vector>

#include "closed_loop.hpp"

namespace {

std::vector<double> parse_list(const char* s) {
  std::vector<double> out;
  std::string cur;
  for (const char* c = s;; ++c) {
    if (*c == ',' || *c == '\0') {
      if (!cur.empty()) {
        out.push_back(std::atof(cur.c_str()));
      }
      cur.clear();
      if (*c == '\0') {
        break;
      }
    } else {
      cur += *c;
    }
  }
  return out;
}

struct Cell {
  double period = 0.0;
  double loss = 0.0;
  unsigned lost = 0U;
  double over_tol = 0.0;        // mean frames over ACT's tolerance
  double spread = 0.0;          // mean of the largest command difference, degrees
  double err = 0.0;             // mean of the largest attitude error after 3 s, degrees
  double skipped = 0.0;         // mean fraction of resyncs a computer could not do
  double mismatch = 0.0;        // mean fraction of frames with the three digests not all equal
  uint32_t longest_run = 0U;    // the longest digest mismatch of any seed
  unsigned would_flag = 0U;     // seeds in which a digest persistence of period + 50 would have counted a mismatch
  unsigned would_flag2 = 0U;    // ... of two periods + 50 (which rides out one skipped resync)
};

Cell run_cell(double period, double loss, uint32_t frames, unsigned seeds) {
  Cell c;
  c.period = period;
  c.loss = loss;
  for (unsigned s = 0; s < seeds; ++s) {
    sim::Loop lp;
    lp.frames = frames;
    lp.estimator.use_accel = false;
    lp.frame_loss_prob = static_cast<float>(loss);
    lp.loss_seed = 1000U + (37U * s);
    lp.resync_period = static_cast<uint32_t>(period);
    const sim::Result r = sim::run(lp);
    const bool lost = !r.finite || r.safe_frames > 0U || r.max_deg_settled > 5.0;
    c.lost += lost ? 1U : 0U;
    c.over_tol += static_cast<double>(r.frames_over_tol);
    c.spread += r.max_command_spread_deg;
    c.err += r.max_deg_settled;
    const double tries = static_cast<double>(r.resyncs_adopted + r.resyncs_skipped);
    c.skipped += tries > 0.0 ? static_cast<double>(r.resyncs_skipped) / tries : 0.0;
    c.mismatch += static_cast<double>(r.digest_mismatch_frames) / static_cast<double>(frames);
    c.longest_run = std::max(c.longest_run, r.longest_mismatch_run);
    c.would_flag += (period > 0.0 && static_cast<double>(r.longest_mismatch_run) > period + 50.0) ? 1U : 0U;
    c.would_flag2 += (period > 0.0 && static_cast<double>(r.longest_mismatch_run) > (2.0 * period) + 50.0) ? 1U : 0U;
  }
  const double n = static_cast<double>(seeds);
  c.over_tol /= n;
  c.spread /= n;
  c.err /= n;
  c.skipped /= n;
  c.mismatch /= n;
  return c;
}

}  // namespace

int main(int argc, char** argv) {
  uint32_t frames = 6000U;
  unsigned seeds = 8U;
  std::vector<double> periods{0, 10, 20, 50, 100, 200, 500, 1000};
  std::vector<double> losses{0.0, 0.0001, 0.001, 0.01, 0.05};
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--frames") {
      frames = static_cast<uint32_t>(std::strtoul(argv[i + 1], nullptr, 10));
    } else if (a == "--seeds") {
      seeds = static_cast<unsigned>(std::strtoul(argv[i + 1], nullptr, 10));
    } else if (a == "--periods") {
      periods = parse_list(argv[i + 1]);
    } else if (a == "--loss") {
      losses = parse_list(argv[i + 1]);
    } else {
      std::fprintf(stderr, "usage: tfc_resync [--frames N] [--seeds N] [--periods a,b,c] [--loss p,q,r]\n");
      return 2;
    }
  }
  std::printf("%u frames per flight, %u loss patterns per cell. LOST: ACT in Safe, or more than 5 degrees from the program after 3 s.\n", frames, seeds);
  for (const double loss : losses) {
    std::vector<std::future<Cell>> jobs;
    for (const double period : periods) {
      jobs.push_back(std::async(std::launch::async, run_cell, period, loss, frames, seeds));
    }
    std::printf("\n### Frame loss %g per frame\n\n| Period (frames) | Flights lost | Frames over ACT's tolerance | Largest command difference (deg) | Largest attitude error (deg) | Resyncs a computer skipped | Frames with digests unequal | Longest unequal run | Seeds flagged by a digest persistence of P + 50 | ... of 2P + 50 |\n|---|---|---|---|---|---|---|---|---|---|\n", loss);
    for (auto& j : jobs) {
      const Cell c = j.get();
      std::printf("| %s | %u of %u | %.1f | %.3f | %.2f | %.1f %% | %.1f %% | %u | %u of %u | %u of %u |\n", c.period == 0.0 ? "none" : std::to_string(static_cast<unsigned>(c.period)).c_str(), c.lost, seeds, c.over_tol, c.spread, c.err,
                  100.0 * c.skipped, 100.0 * c.mismatch, c.longest_run, c.would_flag, seeds, c.would_flag2, seeds);
    }
    std::fflush(stdout);
  }
  return 0;
}
