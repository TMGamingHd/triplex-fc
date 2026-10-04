// SPDX-License-Identifier: MIT
// tfc_replay: feed recorded flight-bus traffic (candump -L format) through the real core/
// code the way a flight computer would, and report what its FDIR decided.
//
//   tfc_replay LOG [--t0 SECONDS] [--vote-us MICROSECONDS] [--policy manual|auto] [--verbose] [--dump CSV]
//              [--expect-latch NODE:FRAME | NODE:MIN-MAX] [--expect-no-latch NODE]
//              [--expect-state NODE:healthy|latched|probation|disabled]
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
constexpr uint64_t kDefaultVoteUs = 7000U;  // FC-A votes 7.0 ms into the frame (docs/ARCHITECTURE.md section 3)

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
  std::vector<std::pair<unsigned, std::string>> states;
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
               "usage: tfc_replay LOG [--t0 SECONDS] [--first-frame N] [--startup-grace N] [--vote-us US] [--policy manual|auto] [--verbose] [--dump CSV]\n"
               "                      [--expect-latch NODE:FRAME|NODE:MIN-MAX]\n"
               "                      [--expect-no-latch NODE] [--expect-state NODE:STATE] [--expect-mode MODE]\n"
               "                      [--expect-min STAT:N]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    return usage();
  }
  const std::string path = argv[1];
  bool verbose = false;
  int64_t t0_us = 0;  // signed: a live log that starts mid-run is aligned to a frame boundary before its first line
  bool have_first_frame = false;
  uint32_t first_frame = 0U;
  uint64_t vote_us = kDefaultVoteUs;
  std::string dump_path;
  tfc::RedundancyConfig cfg;
  Expect ex;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_val = i + 1 < argc;
    if (a == "--verbose") {
      verbose = true;
    } else if (a == "--t0" && has_val) {
      t0_us = static_cast<int64_t>(std::strtod(argv[++i], nullptr) * 1e6);
    } else if (a == "--startup-grace" && has_val) {
      cfg.startup_grace_frames = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (a == "--first-frame" && has_val) {
      first_frame = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
      have_first_frame = true;
    } else if (a == "--vote-us" && has_val) {
      vote_us = static_cast<uint64_t>(std::strtoull(argv[++i], nullptr, 10));
      if (vote_us == 0U || vote_us > kFrameUs) {
        return usage();
      }
    } else if (a == "--dump" && has_val) {
      dump_path = argv[++i];
    } else if (a == "--policy" && has_val) {
      const std::string v = argv[++i];
      if (v == "auto") {
        cfg.policy = tfc::ReintegrationPolicy::AutoTransient;
      } else if (v == "manual") {
        cfg.policy = tfc::ReintegrationPolicy::Manual;
      } else {
        return usage();
      }
    } else if (a == "--expect-state" && has_val) {
      const std::string v = argv[++i];
      unsigned node = 0;
      const std::size_t colon = v.find(':');
      if (colon == std::string::npos || !parse_node(v.substr(0, colon), node)) {
        return usage();
      }
      ex.states.emplace_back(node, v.substr(colon + 1U));
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
    const bool parsed = parse_line(line, lg);
    const int64_t rel = static_cast<int64_t>(lg.t_us) - t0_us + static_cast<int64_t>(kFrameUs - vote_us);
    if (!parsed || rel < 0) {
      (void)std::fprintf(stderr, "tfc_replay: %s:%lu: cannot parse or before --t0: %s\n", path.c_str(), line_no,
                   line.c_str());
      return 2;
    }
    // Frame k = everything received between the previous vote and this one, as FC-A drains it.
    const std::size_t k = static_cast<std::size_t>(static_cast<uint64_t>(rel) / kFrameUs);
    if (k >= frames.size()) {
      frames.resize(k + 1U);
    }
    frames[k].push_back(lg.frame);
  }
  if (frames.empty()) {
    (void)std::fprintf(stderr, "tfc_replay: %s has no frames\n", path.c_str());
    return 2;
  }

  // Optional per-frame CSV for offline analysis (the fault campaign checks safety properties on it).
  std::FILE* dump = nullptr;
  if (!dump_path.empty()) {
    dump = std::fopen(dump_path.c_str(), "w");
    if (dump == nullptr) {
      (void)std::fprintf(stderr, "tfc_replay: cannot write %s\n", dump_path.c_str());
      return 2;
    }
    (void)std::fprintf(dump,
                       "frame,mode,healthy,valid,latched,probation,disabled,safe,alarm,held,unresolved,newly_latched,"
                       "newly_started,newly_readmitted,newly_disabled,probation_failed,integrity,reason_a,reason_b,reason_c,out0,out1,out2,out3,out4,out5,out6,out7\n");
  }

  // ---- feed the log to the redundancy manager, one 10 ms frame at a time ----
  tfc::RedundancyManager mgr(cfg);
  std::array<long, tfc::kNodes> latch_frame{-1, -1, -1};
  tfc::Mode mode = tfc::Mode::Triplex;
  unsigned healthy = tfc::kNodes;
  bool prev_safe_request = false;
  bool prev_bus_alarm = false;
  for (std::size_t k = 0; k < frames.size(); ++k) {
    if (have_first_frame) {
      mgr.begin_frame(first_frame + static_cast<uint32_t>(k));  // a live log: the frame numbers on the bus, not the position in the log
    } else {
      mgr.begin_frame();
    }
    for (const tfc::Frame& f : frames[k]) {
      mgr.on_frame(f);
    }
    const tfc::FrameReport& rep = mgr.end_frame();
    mode = rep.mode;
    healthy = rep.healthy;
    if (verbose) {
      if (rep.safe_request != prev_safe_request) {
        std::printf("frame %zu: SAFE REQUEST %s (disagreement nobody can be blamed for)\n", k,
                    rep.safe_request ? "raised" : "cleared");
      }
      if (rep.bus_alarm != prev_bus_alarm) {
        std::printf("frame %zu: BUS ALARM %s (%u out-of-schedule frames in this 10 ms frame)\n", k,
                    rep.bus_alarm ? "raised" : "cleared", static_cast<unsigned>(rep.out_of_schedule_in_frame));
      }
    }
    if (verbose && rep.integrity_mask != 0U) {
      std::printf("frame %zu: INTEGRITY FAULT repaired (mask %u: 1 node state, 2 Safe flag, 4 configuration, 8 invariant)\n", k,
                  static_cast<unsigned>(rep.integrity_mask));
    }
    prev_safe_request = rep.safe_request;
    prev_bus_alarm = rep.bus_alarm;
    if (dump != nullptr) {
      (void)std::fprintf(dump, "%zu,%u,%u,%u,%u,%u,%u,%d,%d,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u", k, static_cast<unsigned>(rep.mode),
                         rep.healthy, static_cast<unsigned>(rep.valid_mask), static_cast<unsigned>(rep.latched_mask),
                         static_cast<unsigned>(rep.probation_mask), static_cast<unsigned>(rep.disabled_mask),
                         rep.safe_request ? 1 : 0, rep.bus_alarm ? 1 : 0, static_cast<unsigned>(rep.held_mask),
                         rep.unresolved ? 1 : 0, static_cast<unsigned>(rep.newly_latched),
                         static_cast<unsigned>(rep.probation_started), static_cast<unsigned>(rep.newly_reintegrated),
                         static_cast<unsigned>(rep.newly_disabled), static_cast<unsigned>(rep.probation_failed),
                         static_cast<unsigned>(rep.integrity_mask),
                         static_cast<unsigned>(rep.reason[0]), static_cast<unsigned>(rep.reason[1]),
                         static_cast<unsigned>(rep.reason[2]));
      for (unsigned ch = 0; ch < tfc::kVoteChannels; ++ch) {
        (void)std::fprintf(dump, ",%.6f", static_cast<double>(rep.output[ch]));
      }
      (void)std::fprintf(dump, "\n");
    }
    if (verbose) {
      for (unsigned i = 0; i < rep.command_count; ++i) {
        const tfc::CommandEvent& ce = rep.commands[i];
        std::printf("frame %zu: ground command %s%s %c: %s%s\n", k, (ce.flags & tfc::cmdflag::kArm) != 0U ? "ARM " : "",
                    tfc::op_text(ce.op), static_cast<char>('A' + ce.node), tfc::result_text(ce.result),
                    (ce.flags & tfc::cmdflag::kCritical) != 0U ? "  *** CRITICAL: the last voting node was removed ***" : "");
      }
      for (unsigned n = 0; n < tfc::kNodes; ++n) {
        const unsigned bit = 1U << n;
        const char c = static_cast<char>('A' + n);
        if ((rep.probation_started & bit) != 0U) {
          std::printf("frame %zu: node %c on probation (shadow vote against the healthy nodes)\n", k, c);
        }
        if ((rep.probation_failed & bit) != 0U) {
          std::printf("frame %zu: node %c FAILED probation, back to latched: %s\n", k, c,
                      reason_text(rep.reason[n]).c_str());
        }
        if ((rep.newly_reintegrated & bit) != 0U) {
          std::printf("frame %zu: node %c REINTEGRATED (strikes on record: %u)\n", k, c,
                      static_cast<unsigned>(rep.strikes[n]));
        }
        if ((rep.newly_disabled & bit) != 0U) {
          std::printf("frame %zu: node %c DISABLED for the run (strikes: %u)\n", k, c,
                      static_cast<unsigned>(rep.strikes[n]));
        }
      }
    }
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

  if (dump != nullptr) {
    (void)std::fclose(dump);
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
  for (unsigned n = 0; n < tfc::kNodes; ++n) {
    std::printf("state.%c=%s\nstrikes.%c=%u\n", static_cast<char>('A' + n), tfc::state_text(mgr.state(n)),
                static_cast<char>('A' + n), mgr.strikes(n));
  }
  const std::map<std::string, unsigned long> stats{{"crc_bad", c.crc_bad},
                                                   {"seq_bad", c.seq_bad},
                                                   {"missing", c.missing},
                                                   {"out_of_schedule", c.out_of_schedule},
                                                   {"stuck_flags", c.stuck_flags},
                                                   {"digest_flags", c.digest_flags},
                                                   {"vote_disagreements", c.vote_disagreements},
                                                   {"unresolved_frames", c.unresolved_frames},
                                                   {"held_frames", c.held_frames},
                                                   {"safe_request_frames", c.safe_request_frames},
                                                   {"bus_alarm_frames", c.bus_alarm_frames},
                                                   {"probations_started", c.probations_started},
                                                   {"probation_failures", c.probation_failures},
                                                   {"reintegrations", c.reintegrations},
                                                   {"nodes_disabled", c.nodes_disabled},
                                                   {"commands_accepted", c.commands_accepted},
                                                   {"commands_refused", c.commands_refused},
                                                   {"commands_bad", c.commands_bad},
                                                   {"integrity_faults", c.integrity_faults},
                                                   {"invariant_violations", c.invariant_violations},
                                                   {"commands_unauthentic", c.commands_unauthentic},
                                                   {"commands_replayed", c.commands_replayed},
                                                   {"arms_expired", c.arms_expired},
                                                   {"critical_commands", c.critical_commands}};
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
  for (const auto& st : ex.states) {
    const char* got = tfc::state_text(mgr.state(st.first));
    if (st.second != got) {
      std::printf("EXPECT FAILED: node %c is %s, expected %s\n", static_cast<char>('A' + st.first), got,
                  st.second.c_str());
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
