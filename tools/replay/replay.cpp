// SPDX-License-Identifier: MIT
// tfc_replay: feed recorded flight-bus traffic (candump -L format) through the real core/
// code the way a flight computer would, and report what its FDIR decided.
//
//   tfc_replay LOG [--t0 SECONDS] [--verbose]
//              [--expect-latch NODE:FRAME | NODE:MIN-MAX] [--expect-no-latch NODE]
//              [--expect-mode triplex|duplex|simplex|safe] [--expect-min STAT:N]
//
// Each 10 ms frame of the log is fed to tfc::RedundancyManager, the same class the firmware
// runs: it decodes gyro/accel/command frames, checks CRC and sequence, votes every channel,
// cross-checks the state digest, runs the stuck detector, and feeds one 3-of-5 ChannelMonitor
// per node (docs/ARCHITECTURE.md sections 4-5). This tool only reads the log and reports.
// Exit status: 0 ok, 1 an --expect-* failed, 2 usage/input error.
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"

namespace {

constexpr uint64_t kFrameUs = 10000U;

struct Logged {
  uint64_t t_us = 0;
  tfc::Frame frame;
};

bool parse_line(const std::string& s, Logged& out) {
  const char* p = s.c_str();
  if (*p != '(') {
    return false;
  }
  char* end = nullptr;
  const unsigned long long sec = std::strtoull(p + 1, &end, 10);
  if (end == p + 1 || *end != '.') {
    return false;
  }
  const char* q = end + 1;
  const unsigned long long usec = std::strtoull(q, &end, 10);
  if (end == q || *end != ')') {
    return false;
  }
  p = end + 1;
  while (*p == ' ') {
    ++p;
  }
  while (*p != '\0' && *p != ' ') {  // interface name
    ++p;
  }
  while (*p == ' ') {
    ++p;
  }
  const unsigned long id = std::strtoul(p, &end, 16);
  if (end == p || *end != '#' || id > 0x7FFUL) {
    return false;
  }
  p = end + 1;
  const std::size_t hex_len = std::strlen(p);
  if (hex_len % 2U != 0U || hex_len > 16U) {
    return false;
  }
  out.t_us = static_cast<uint64_t>(sec) * 1000000U + static_cast<uint64_t>(usec);
  out.frame = tfc::Frame{};
  out.frame.id = static_cast<uint32_t>(id);
  out.frame.len = static_cast<uint8_t>(hex_len / 2U);
  for (std::size_t i = 0; i < hex_len / 2U; ++i) {
    const char byte[3] = {p[2U * i], p[2U * i + 1U], '\0'};
    const unsigned long v = std::strtoul(byte, &end, 16);
    if (*end != '\0') {
      return false;
    }
    out.frame.data[i] = static_cast<uint8_t>(v);
  }
  return true;
}

std::string reason_text(uint8_t bits) {
  std::array<char, 96> buf{};
  tfc::format_reasons(bits, buf.data(), buf.size());
  return std::string(buf.data());
}

const char* mode_name(tfc::Mode m) {
  switch (m) {
    case tfc::Mode::Triplex: return "triplex";
    case tfc::Mode::Duplex: return "duplex";
    case tfc::Mode::Simplex: return "simplex";
    case tfc::Mode::Safe: return "safe";
  }
  return "?";
}

struct Expect {
  struct Latch {
    unsigned node;
    long lo;
    long hi;
  };
  std::vector<Latch> latches;
  std::vector<unsigned> no_latch;
  std::string mode;
  std::vector<std::pair<std::string, unsigned long>> min_stats;
};

bool parse_node(const std::string& s, unsigned& node) {
  if (s.size() != 1U || s[0] < 'A' || s[0] > 'C') {
    return false;
  }
  node = static_cast<unsigned>(s[0] - 'A');
  return true;
}

int usage() {
  (void)std::fprintf(stderr,
               "usage: tfc_replay LOG [--t0 SECONDS] [--verbose] [--expect-latch NODE:FRAME|NODE:MIN-MAX]\n"
               "                      [--expect-no-latch NODE] [--expect-mode MODE] [--expect-min STAT:N]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    return usage();
  }
  const std::string path = argv[1];
  bool verbose = false;
  uint64_t t0_us = 0;
  Expect ex;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_val = i + 1 < argc;
    if (a == "--verbose") {
      verbose = true;
    } else if (a == "--t0" && has_val) {
      t0_us = static_cast<uint64_t>(std::strtod(argv[++i], nullptr) * 1e6);
    } else if (a == "--expect-latch" && has_val) {
      const std::string v = argv[++i];
      unsigned node = 0;
      const std::size_t colon = v.find(':');
      if (colon == std::string::npos || !parse_node(v.substr(0, colon), node)) {
        return usage();
      }
      const std::string range = v.substr(colon + 1U);
      const std::size_t dash = range.find('-');
      const long lo = std::strtol(range.c_str(), nullptr, 10);
      const long hi = dash == std::string::npos ? lo : std::strtol(range.c_str() + dash + 1U, nullptr, 10);
      ex.latches.push_back({node, lo, hi});
    } else if (a == "--expect-no-latch" && has_val) {
      unsigned node = 0;
      if (!parse_node(argv[++i], node)) {
        return usage();
      }
      ex.no_latch.push_back(node);
    } else if (a == "--expect-mode" && has_val) {
      ex.mode = argv[++i];
    } else if (a == "--expect-min" && has_val) {
      const std::string v = argv[++i];
      const std::size_t colon = v.find(':');
      if (colon == std::string::npos) {
        return usage();
      }
      ex.min_stats.emplace_back(v.substr(0, colon), std::strtoul(v.c_str() + colon + 1U, nullptr, 10));
    } else {
      return usage();
    }
  }

  // ---- load and bucket frames by 10 ms major frame ----
  std::ifstream in(path);
  if (!in) {
    (void)std::fprintf(stderr, "tfc_replay: cannot open %s\n", path.c_str());
    return 2;
  }
  std::vector<std::vector<tfc::Frame>> frames;
  std::string line;
  unsigned long line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (line.empty()) {
      continue;
    }
    Logged lg;
    if (!parse_line(line, lg) || lg.t_us < t0_us) {
      (void)std::fprintf(stderr, "tfc_replay: %s:%lu: cannot parse or before --t0: %s\n", path.c_str(), line_no,
                   line.c_str());
      return 2;
    }
    const std::size_t k = static_cast<std::size_t>((lg.t_us - t0_us) / kFrameUs);
    if (k >= frames.size()) {
      frames.resize(k + 1U);
    }
    frames[k].push_back(lg.frame);
  }
  if (frames.empty()) {
    (void)std::fprintf(stderr, "tfc_replay: %s has no frames\n", path.c_str());
    return 2;
  }

  // ---- feed the log to the redundancy manager, one 10 ms frame at a time ----
  tfc::RedundancyManager mgr;
  std::array<long, tfc::kNodes> latch_frame{-1, -1, -1};
  tfc::Mode mode = tfc::Mode::Triplex;
  unsigned healthy = tfc::kNodes;
  for (std::size_t k = 0; k < frames.size(); ++k) {
    mgr.begin_frame();
    for (const tfc::Frame& f : frames[k]) {
      mgr.on_frame(f);
    }
    const tfc::FrameReport& rep = mgr.end_frame();
    mode = rep.mode;
    healthy = rep.healthy;
    for (unsigned n = 0; n < tfc::kNodes; ++n) {
      if (((rep.newly_latched >> n) & 1U) == 0U) {
        continue;
      }
      latch_frame[n] = static_cast<long>(k);
      if (verbose) {
        std::printf("frame %zu: node %c latched because: %s\n", k, static_cast<char>('A' + n),
                    reason_text(rep.reason[n]).c_str());
      }
    }
  }

  const tfc::Counters& c = mgr.counters();
  std::printf("frames=%zu\n", frames.size());
  for (unsigned n = 0; n < tfc::kNodes; ++n) {
    if (latch_frame[n] < 0) {
      std::printf("latch.%c=-\n", static_cast<char>('A' + n));
    } else {
      std::printf("latch.%c=%ld\n", static_cast<char>('A' + n), latch_frame[n]);
    }
  }
  std::printf("healthy=%u\nmode=%s\n", healthy, mode_name(mode));
  const std::map<std::string, unsigned long> stats{{"crc_bad", c.crc_bad},
                                                   {"seq_bad", c.seq_bad},
                                                   {"missing", c.missing},
                                                   {"out_of_schedule", c.out_of_schedule},
                                                   {"stuck_flags", c.stuck_flags},
                                                   {"digest_flags", c.digest_flags},
                                                   {"vote_disagreements", c.vote_disagreements}};
  for (const auto& kv : stats) {
    std::printf("%s=%lu\n", kv.first.c_str(), kv.second);
  }

  // ---- expectations ----
  int failed = 0;
  for (const Expect::Latch& e : ex.latches) {
    const long got = latch_frame[e.node];
    if (got < e.lo || got > e.hi) {
      std::printf("EXPECT FAILED: node %c latch frame %ld not in [%ld, %ld]\n", static_cast<char>('A' + e.node), got,
                  e.lo, e.hi);
      ++failed;
    }
  }
  for (const unsigned n : ex.no_latch) {
    if (latch_frame[n] >= 0) {
      std::printf("EXPECT FAILED: node %c latched at frame %ld but should not have\n", static_cast<char>('A' + n),
                  latch_frame[n]);
      ++failed;
    }
  }
  if (!ex.mode.empty() && ex.mode != mode_name(mode)) {
    std::printf("EXPECT FAILED: mode %s, expected %s\n", mode_name(mode), ex.mode.c_str());
    ++failed;
  }
  for (const auto& m : ex.min_stats) {
    const auto it = stats.find(m.first);
    if (it == stats.end() || it->second < m.second) {
      std::printf("EXPECT FAILED: %s=%lu, expected at least %lu\n", m.first.c_str(),
                  it == stats.end() ? 0UL : it->second, m.second);
      ++failed;
    }
  }
  return failed == 0 ? 0 : 1;
}
