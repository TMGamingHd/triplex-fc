// SPDX-License-Identifier: MIT
// tfc_replay: feed recorded flight-bus traffic (candump -L format) through the real core/
// code the way a flight computer would, and report what its FDIR decided.
//
//   tfc_replay LOG [--t0 SECONDS] [--verbose]
//              [--expect-latch NODE:FRAME | NODE:MIN-MAX] [--expect-no-latch NODE]
//              [--expect-mode triplex|duplex|simplex|safe] [--expect-min STAT:N]
//
// Per 10 ms frame, per node (A/B/C): decode gyro, accel and command frames with
// tfc::unpack_*, check CRC and sequence, vote every channel with tfc::vote3, cross-check the
// state digest, run a tfc::StuckDetector, and feed a tfc::ChannelMonitor (3-of-5). Missing,
// CRC-bad and out-of-sequence data count as a miscompare, as in docs/ARCHITECTURE.md.
// This is a host model of FC-A's consensus + FDIR step, not the firmware; it makes the same
// calls the firmware will make. Exit status: 0 ok, 1 an --expect-* failed, 2 usage/input error.
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace {

constexpr unsigned kNodes = 3;
constexpr uint64_t kFrameUs = 10000U;
constexpr uint16_t kStuckLimit = 20U;
// Vote tolerances: gyro x3 (dps), accel x3 (g), command pitch/yaw (deg).
constexpr std::array<float, 8> kTol = {1.0F, 1.0F, 1.0F, 0.02F, 0.02F, 0.02F, 0.01F, 0.01F};

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

uint32_t fnv1a(const uint8_t* d, std::size_t n) {
  uint32_t h = 2166136261U;
  for (std::size_t i = 0; i < n; ++i) {
    h = (h ^ d[i]) * 16777619U;
  }
  return h;
}

struct NodeRx {
  bool gyro = false;
  bool accel = false;
  bool cmd = false;
  bool crc_bad = false;
  bool seq_bad = false;
  std::array<float, 8> x{};  // gyro[3], accel[3], pitch, yaw
  uint16_t digest = 0;
  std::array<uint8_t, 12> raw{};  // gyro+accel payload bytes, for the stuck detector
};

// Mask of nodes whose digest disagrees with the majority (or both, when only two can be compared).
uint8_t digest_outliers(uint8_t valid, const std::array<NodeRx, kNodes>& rx) {
  const unsigned n = tfc::count_channels(valid);
  if (n == 3U) {
    const uint16_t a = rx[0].digest;
    const uint16_t b = rx[1].digest;
    const uint16_t c = rx[2].digest;
    if (a == b && b == c) {
      return 0U;
    }
    if (a == b) {
      return 0x4U;
    }
    if (a == c) {
      return 0x2U;
    }
    return b == c ? 0x1U : 0x7U;
  }
  if (n == 2U) {
    uint16_t first = 0;
    bool have = false;
    for (unsigned i = 0; i < kNodes; ++i) {
      if (((valid >> i) & 1U) == 0U) {
        continue;
      }
      if (!have) {
        first = rx[i].digest;
        have = true;
      } else if (rx[i].digest != first) {
        return valid;
      }
    }
  }
  return 0U;
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

  // ---- the flight computer's per-frame step ----
  std::array<tfc::ChannelMonitor, kNodes> mon{tfc::ChannelMonitor(3, 5, 100, 3), tfc::ChannelMonitor(3, 5, 100, 3),
                                              tfc::ChannelMonitor(3, 5, 100, 3)};
  std::array<tfc::StuckDetector, kNodes> stuck{tfc::StuckDetector(kStuckLimit), tfc::StuckDetector(kStuckLimit),
                                               tfc::StuckDetector(kStuckLimit)};
  std::array<std::array<bool, 3>, kNodes> have_seq{};
  std::array<std::array<uint8_t, 3>, kNodes> last_seq{};
  std::array<long, kNodes> latch_frame{-1, -1, -1};
  std::map<std::string, unsigned long> stats{{"crc_bad", 0}, {"seq_bad", 0},       {"missing", 0},
                                             {"out_of_schedule", 0}, {"stuck_flags", 0}, {"digest_flags", 0},
                                             {"vote_disagreements", 0}};

  for (std::size_t k = 0; k < frames.size(); ++k) {
    std::array<NodeRx, kNodes> rx{};
    for (const tfc::Frame& f : frames[k]) {
      const uint32_t id = f.id;
      unsigned stream = 0;
      unsigned node = 0;
      if (id >= tfc::id::kGyroBase && id < tfc::id::kGyroBase + kNodes) {
        stream = 0;
        node = id - tfc::id::kGyroBase;
      } else if (id >= tfc::id::kAccelBase && id < tfc::id::kAccelBase + kNodes) {
        stream = 1;
        node = id - tfc::id::kAccelBase;
      } else if (id >= tfc::id::kCmdBase && id < tfc::id::kCmdBase + kNodes) {
        stream = 2;
        node = id - tfc::id::kCmdBase;
      } else {
        const bool known = id == tfc::id::kSync || id == tfc::id::kActOut ||
                           (id >= tfc::id::kHeartbeat && id < tfc::id::kHeartbeat + kNodes) || id >= tfc::id::kSim;
        if (!known) {
          ++stats["out_of_schedule"];
        }
        continue;
      }
      NodeRx& r = rx[node];
      uint8_t seq = 0;
      bool ok = false;
      if (stream == 2U) {
        const tfc::DecodedCommand d = tfc::unpack_cmd(f);
        ok = d.ok;
        seq = d.seq;
        if (ok) {
          r.x[6] = d.cmd.pitch_deg;
          r.x[7] = d.cmd.yaw_deg;
          r.digest = d.cmd.state_digest;
          r.cmd = true;
        }
      } else {
        const bool is_gyro = stream == 0U;
        const tfc::DecodedVec3 d = tfc::unpack_vec3(f, is_gyro ? tfc::kGyroLsbDps : tfc::kAccelLsbG);
        ok = d.ok;
        seq = d.seq;
        if (ok) {
          for (unsigned i = 0; i < 3U; ++i) {
            r.x[(is_gyro ? 0U : 3U) + i] = d.x.v[i];
          }
          std::memcpy(r.raw.data() + (is_gyro ? 0U : 6U), f.data.data(), 6U);
          (is_gyro ? r.gyro : r.accel) = true;
        }
      }
      if (!ok) {
        r.crc_bad = true;
        ++stats["crc_bad"];
        continue;
      }
      if (have_seq[node][stream] && !tfc::seq_is_next(last_seq[node][stream], seq)) {
        r.seq_bad = true;
        ++stats["seq_bad"];
      }
      have_seq[node][stream] = true;
      last_seq[node][stream] = seq;
    }

    uint8_t valid = 0U;
    std::array<bool, kNodes> good{};
    for (unsigned n = 0; n < kNodes; ++n) {
      const NodeRx& r = rx[n];
      const bool present = r.gyro && r.accel && r.cmd;
      if (!present && !r.crc_bad) {
        ++stats["missing"];
      }
      good[n] = present && !r.crc_bad && !r.seq_bad;
      if (good[n] && !mon[n].latched()) {
        valid = static_cast<uint8_t>(valid | (1U << n));
      }
    }

    uint8_t disagree = 0U;
    for (unsigned ch = 0; ch < kTol.size(); ++ch) {
      const std::array<float, 3> x = {rx[0].x[ch], rx[1].x[ch], rx[2].x[ch]};
      disagree = static_cast<uint8_t>(disagree | tfc::vote3(x, valid, kTol[ch]).disagree_mask);
    }
    const uint8_t digest_bad = digest_outliers(valid, rx);
    if (disagree != 0U) {
      ++stats["vote_disagreements"];
    }
    if (digest_bad != 0U) {
      ++stats["digest_flags"];
    }

    for (unsigned n = 0; n < kNodes; ++n) {
      bool stuck_now = false;
      if (good[n]) {
        stuck_now = stuck[n].update(static_cast<int32_t>(fnv1a(rx[n].raw.data(), rx[n].raw.size())));
      }
      if (stuck_now) {
        ++stats["stuck_flags"];
      }
      const bool bad = !good[n] || ((disagree >> n) & 1U) != 0U || ((digest_bad >> n) & 1U) != 0U || stuck_now;
      if (mon[n].update(bad) && latch_frame[n] < 0) {
        latch_frame[n] = static_cast<long>(k);
        if (verbose) {
          std::printf("frame %zu: node %c latched (missing/crc/seq=%d vote=%d digest=%d stuck=%d)\n", k,
                      static_cast<char>('A' + n), good[n] ? 0 : 1, ((disagree >> n) & 1U) != 0U ? 1 : 0,
                      ((digest_bad >> n) & 1U) != 0U ? 1 : 0, stuck_now ? 1 : 0);
        }
      }
    }
  }

  unsigned healthy = 0;
  for (unsigned n = 0; n < kNodes; ++n) {
    healthy += mon[n].latched() ? 0U : 1U;
  }
  const tfc::Mode mode = tfc::mode_from_healthy(healthy);
  std::printf("frames=%zu\n", frames.size());
  for (unsigned n = 0; n < kNodes; ++n) {
    if (latch_frame[n] < 0) {
      std::printf("latch.%c=-\n", static_cast<char>('A' + n));
    } else {
      std::printf("latch.%c=%ld\n", static_cast<char>('A' + n), latch_frame[n]);
    }
  }
  std::printf("healthy=%u\nmode=%s\n", healthy, mode_name(mode));
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
