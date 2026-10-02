// SPDX-License-Identifier: MIT
// Cost of one 10 ms frame of the redundancy manager on the host: on_frame x 9 + end_frame, healthy triplex and under
// fault. Host numbers are a sanity check on algorithmic cost (no heap, no loops over unbounded data), NOT the flight
// WCET: that must be measured on the target (docs/CODING_STANDARD.md section 6). Build target: tfc_bench.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "tfc/redundancy.hpp"

using namespace tfc;
using Clock = std::chrono::steady_clock;

namespace {

struct Cycle {
  std::array<Frame, 9> f;
};

std::vector<Cycle> make_cycles(int bias_node, bool dropout_c) {
  std::vector<Cycle> out(256);
  for (unsigned k = 0; k < 256U; ++k) {
    unsigned i = 0;
    const auto seq = static_cast<uint8_t>(k);
    const float g = 5.0F + 3.0F * static_cast<float>(k % 16U) * 0.1F;
    for (uint8_t n = 0; n < 3U; ++n) {
      const float b = static_cast<int>(n) == bias_node ? 3.0F : 0.0F;
      out[k].f[i++] = pack_gyro(n, Vec3{{g + b, -2.0F, 1.0F}}, seq);
      out[k].f[i++] = pack_accel(n, Vec3{{0.0F, 0.0F, 1.0F}}, seq);
      out[k].f[i++] = pack_cmd(n, Command{0.5F, -0.25F, 0x1234U}, seq);
    }
    if (dropout_c) {
      for (unsigned j = 6; j < 9; ++j) out[k].f[j].id = 0x7F0;  // C's three frames never arrive (out-of-schedule noise instead)
    }
  }
  return out;
}

void run(const char* name, const std::vector<Cycle>& cycles, int frames) {
  RedundancyManager m;
  std::vector<double> ns;
  ns.reserve(static_cast<std::size_t>(frames));
  unsigned sink = 0U;
  for (int k = 0; k < frames; ++k) {
    const Cycle& c = cycles[static_cast<std::size_t>(k) % 256U];
    const auto t0 = Clock::now();
    m.begin_frame();
    for (const Frame& f : c.f) (void)m.on_frame(f);
    sink += static_cast<unsigned>(m.end_frame().mode);
    const auto t1 = Clock::now();
    ns.push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()));
  }
  std::sort(ns.begin(), ns.end());
  const auto pct = [&](double p) { return ns[static_cast<std::size_t>(p * static_cast<double>(ns.size() - 1U))]; };
  std::printf("%-34s median %6.0f ns   p99 %6.0f ns   p99.9 %6.0f ns   max %7.0f ns   (%u)\n", name, pct(0.5), pct(0.99), pct(0.999),
              ns.back(), sink & 1U);
}

}  // namespace

int main() {
  std::printf("sizeof(RedundancyManager)=%zu B  sizeof(FrameReport)=%zu B  sizeof(RedundancyConfig)=%zu B  sizeof(Counters)=%zu B\n",
              sizeof(RedundancyManager), sizeof(FrameReport), sizeof(RedundancyConfig), sizeof(Counters));
  constexpr int kFrames = 400000;
  run("healthy triplex", make_cycles(-1, false), kFrames);
  run("node B biased (latched, duplex)", make_cycles(1, false), kFrames);
  run("node C silent (latched, duplex)", make_cycles(-1, true), kFrames);
  std::printf("a 10 ms frame is 10,000,000 ns\n");
  return 0;
}
