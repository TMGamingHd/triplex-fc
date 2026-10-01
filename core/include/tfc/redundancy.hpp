// SPDX-License-Identifier: MIT
// One flight computer's view of its two peers (and itself): collect a major frame of bus
// traffic, judge every node, vote every channel, and update the FDIR state.
//
//   mgr.begin_frame();                 // start of each 10 ms major frame
//   mgr.on_frame(f);                   // every flight-bus frame received (and this node's own)
//   const FrameReport& r = mgr.end_frame();   // vote + FDIR step; r says who latched and why
//
// Per node it checks that gyro, accel and command arrived, CRC-good and in sequence; votes the
// 8 channels (gyro x3, accel x3, command pitch/yaw) with vote3; cross-checks the estimator
// digest; runs a StuckDetector on the raw sensor bytes; and feeds one ChannelMonitor per node
// (M-of-N persistence, latch, reintegration). Absent means invalid: a node that sends nothing
// is just a node whose frames are missing. No heap, no exceptions, deterministic.
#pragma once
#include <array>
#include <cstdint>
#include <cstring>

#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace tfc {

constexpr unsigned kNodes = kChannels;      // flight computers A, B, C
constexpr unsigned kVoteChannels = 8;       // gyro x3, accel x3, command pitch, command yaw
constexpr unsigned kStreams = 3;            // per node: gyro, accel, command

// Why a node's data was judged bad in a frame (bit mask in FrameReport::reason).
namespace reason {
constexpr uint8_t kMissing = 1U;  // a gyro/accel/command frame did not arrive
constexpr uint8_t kCrc = 2U;      // a frame arrived but failed its CRC
constexpr uint8_t kSeq = 4U;      // sequence number out of order
constexpr uint8_t kVote = 8U;     // value disagreed with the vote on some channel
constexpr uint8_t kDigest = 16U;  // estimator-state digest disagreed
constexpr uint8_t kStuck = 32U;   // sensor bytes bit-identical for too many frames
}  // namespace reason

// Writes the set reason bits as words, e.g. "vote disagreement + digest mismatch", into `out`
// (NUL-terminated, truncated to `cap`). No heap; for logs on the host and on the target.
inline void format_reasons(uint8_t bits, char* out, std::size_t cap) noexcept {
  struct Name {
    uint8_t bit;
    const char* text;
  };
  constexpr std::array<Name, 6> names = {{{reason::kMissing, "frame missing"},
                                          {reason::kCrc, "CRC failure"},
                                          {reason::kSeq, "sequence error"},
                                          {reason::kVote, "vote disagreement"},
                                          {reason::kDigest, "digest mismatch"},
                                          {reason::kStuck, "stuck sensor"}}};
  if (cap == 0U) {
    return;
  }
  std::size_t n = 0;
  auto put = [&](const char* text) {
    for (const char* c = text; *c != '\0' && n + 1U < cap; ++c) {
      out[n++] = *c;
    }
  };
  for (const Name& nm : names) {
    if ((bits & nm.bit) != 0U) {
      if (n != 0U) {
        put(" + ");
      }
      put(nm.text);
    }
  }
  if (n == 0U) {
    put("(none)");
  }
  out[n] = '\0';
}

struct RedundancyConfig {
  // Vote tolerances per channel: gyro (dps) x3, accel (g) x3, command (deg) x2.
  std::array<float, kVoteChannels> tol{{1.0F, 1.0F, 1.0F, 0.02F, 0.02F, 0.02F, 0.01F, 0.01F}};
  uint8_t persist_m = 3;            // latch when M of the last N frames are bad
  uint8_t persist_n = 5;
  uint16_t reintegrate_clean = 100;  // clean frames needed after an explicit reintegration request
  uint8_t max_latches = 3;           // latches before a node is permanently excluded
  uint16_t stuck_limit = 20;         // identical sensor frames before "stuck"
  // A node that has never delivered a good sample is not judged for this many frames: peers boot
  // in any order, and "not here yet" is not "failed". 0 = judge from the first frame (absent means
  // invalid). Once a node has been seen, it is always judged.
  uint32_t startup_grace_frames = 0;
};

struct Counters {
  uint32_t frames = 0;
  uint32_t crc_bad = 0;             // frames discarded for a failed CRC
  uint32_t seq_bad = 0;             // frames whose sequence number was not the expected next
  uint32_t missing = 0;             // node-frames where gyro/accel/command did not all arrive
  uint32_t out_of_schedule = 0;     // frames on IDs that are not part of the schedule
  uint32_t stuck_flags = 0;
  uint32_t digest_flags = 0;
  uint32_t vote_disagreements = 0;  // frames where any channel vote flagged a node
};

struct FrameReport {
  uint8_t valid_mask = 0U;      // nodes whose data took part in the vote this frame
  uint8_t newly_latched = 0U;   // nodes that latched on this frame
  uint8_t newly_seen = 0U;      // nodes whose first good sample arrived on this frame
  uint8_t latched_mask = 0U;    // nodes currently latched out
  std::array<uint8_t, kNodes> reason{};  // reason:: bits, per node, this frame
  Mode mode = Mode::Safe;
  unsigned healthy = 0U;        // nodes that have been seen and are not latched out
  std::array<VoteResult, kVoteChannels> votes{};  // voted value + status per channel
};

class RedundancyManager {
 public:
  explicit RedundancyManager(const RedundancyConfig& cfg = RedundancyConfig{}) noexcept
      : cfg_(cfg),
        mon_{ChannelMonitor(cfg.persist_m, cfg.persist_n, cfg.reintegrate_clean, cfg.max_latches),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, cfg.reintegrate_clean, cfg.max_latches),
             ChannelMonitor(cfg.persist_m, cfg.persist_n, cfg.reintegrate_clean, cfg.max_latches)},
        stuck_{StuckDetector(cfg.stuck_limit), StuckDetector(cfg.stuck_limit),
               StuckDetector(cfg.stuck_limit)} {}

  void begin_frame() noexcept { rx_ = {}; }

  // Offer one received frame. Returns false if its ID is not part of the flight-bus schedule
  // (it is then only counted). SYNC, actuator, heartbeat and sim frames are accepted and ignored.
  bool on_frame(const Frame& f) noexcept {
    unsigned stream = 0U;
    unsigned node = 0U;
    if (f.id >= id::kGyroBase && f.id < id::kGyroBase + kNodes) {
      stream = 0U;
      node = f.id - id::kGyroBase;
    } else if (f.id >= id::kAccelBase && f.id < id::kAccelBase + kNodes) {
      stream = 1U;
      node = f.id - id::kAccelBase;
    } else if (f.id >= id::kCmdBase && f.id < id::kCmdBase + kNodes) {
      stream = 2U;
      node = f.id - id::kCmdBase;
    } else {
      const bool known = f.id == id::kSync || f.id == id::kActOut ||
                         (f.id >= id::kHeartbeat && f.id < id::kHeartbeat + kNodes) || f.id >= id::kSim;
      if (!known) {
        ++counters_.out_of_schedule;
      }
      return known;
    }
    NodeRx& r = rx_[node];
    bool ok = false;
    uint8_t seq = 0U;
    if (stream == 2U) {
      const DecodedCommand d = unpack_cmd(f);
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
      const DecodedVec3 d = unpack_vec3(f, is_gyro ? kGyroLsbDps : kAccelLsbG);
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
      ++counters_.crc_bad;
      seq_[node][stream].note_damaged();  // it was sent, so it used up a sequence number
      return true;
    }
    if (!seq_[node][stream].accept(seq)) {
      r.seq_bad = true;
      ++counters_.seq_bad;
    }
    return true;
  }

  // Close the frame: judge nodes, vote, cross-check, update FDIR. The returned reference is
  // valid until the next end_frame().
  const FrameReport& end_frame() noexcept {
    FrameReport& rep = report_;
    rep = FrameReport{};
    ++counters_.frames;

    std::array<bool, kNodes> good{};
    uint8_t valid = 0U;
    for (unsigned n = 0; n < kNodes; ++n) {
      const NodeRx& r = rx_[n];
      const bool present = r.gyro && r.accel && r.cmd;
      const bool in_grace = !seen_[n] && counters_.frames <= cfg_.startup_grace_frames;
      if (!present && !r.crc_bad && !in_grace) {
        ++counters_.missing;
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kMissing);
      }
      if (r.crc_bad) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kCrc);
      }
      if (r.seq_bad) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kSeq);
      }
      good[n] = present && !r.crc_bad && !r.seq_bad;
      if (good[n] && !seen_[n]) {
        seen_[n] = true;
        rep.newly_seen = static_cast<uint8_t>(rep.newly_seen | (1U << n));
      }
      if (good[n] && !mon_[n].latched()) {
        valid = static_cast<uint8_t>(valid | (1U << n));
      }
    }
    rep.valid_mask = valid;

    uint8_t disagree = 0U;
    for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
      const std::array<float, kNodes> x = {rx_[0].x[ch], rx_[1].x[ch], rx_[2].x[ch]};
      rep.votes[ch] = vote3(x, valid, cfg_.tol[ch]);
      disagree = static_cast<uint8_t>(disagree | rep.votes[ch].disagree_mask);
    }
    const uint8_t digest_bad = digest_outliers(valid);
    if (disagree != 0U) {
      ++counters_.vote_disagreements;
    }
    if (digest_bad != 0U) {
      ++counters_.digest_flags;
    }

    for (unsigned n = 0; n < kNodes; ++n) {
      bool stuck_now = false;
      if (good[n]) {
        stuck_now = stuck_[n].update(static_cast<int32_t>(fnv1a(rx_[n].raw.data(), rx_[n].raw.size())));
      }
      if (stuck_now) {
        ++counters_.stuck_flags;
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kStuck);
      }
      if (((disagree >> n) & 1U) != 0U) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kVote);
      }
      if (((digest_bad >> n) & 1U) != 0U) {
        rep.reason[n] = static_cast<uint8_t>(rep.reason[n] | reason::kDigest);
      }
      if (mon_[n].update(rep.reason[n] != 0U)) {
        rep.newly_latched = static_cast<uint8_t>(rep.newly_latched | (1U << n));
      }
    }

    for (unsigned n = 0; n < kNodes; ++n) {
      if (mon_[n].latched()) {
        rep.latched_mask = static_cast<uint8_t>(rep.latched_mask | (1U << n));
      } else if (seen_[n]) {
        ++rep.healthy;
      }
    }
    rep.mode = mode_from_healthy(rep.healthy);
    return rep;
  }

  // Operator/ground command: start counting clean frames toward reintegrating `node`.
  void request_reintegration(unsigned node) noexcept {
    if (node < kNodes) {
      mon_[node].request_reintegration();
    }
  }

  bool seen(unsigned node) const noexcept { return node < kNodes && seen_[node]; }
  bool latched(unsigned node) const noexcept { return node < kNodes && mon_[node].latched(); }
  bool permanent(unsigned node) const noexcept { return node < kNodes && mon_[node].permanent(); }
  const Counters& counters() const noexcept { return counters_; }
  const FrameReport& last_report() const noexcept { return report_; }

 private:
  struct NodeRx {
    bool gyro = false;
    bool accel = false;
    bool cmd = false;
    bool crc_bad = false;
    bool seq_bad = false;
    std::array<float, kVoteChannels> x{};
    uint16_t digest = 0U;
    std::array<uint8_t, 12> raw{};  // gyro + accel payload bytes, for the stuck detector
  };

  static uint32_t fnv1a(const uint8_t* d, std::size_t n) noexcept {
    uint32_t h = 2166136261U;
    for (std::size_t i = 0; i < n; ++i) {
      h = (h ^ d[i]) * 16777619U;
    }
    return h;
  }

  // Nodes whose digest disagrees with the majority (both, when only two can be compared).
  uint8_t digest_outliers(uint8_t valid) const noexcept {
    const unsigned n = count_channels(valid);
    if (n == 3U) {
      const uint16_t a = rx_[0].digest;
      const uint16_t b = rx_[1].digest;
      const uint16_t c = rx_[2].digest;
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
      uint16_t first = 0U;
      bool have = false;
      for (unsigned i = 0; i < kNodes; ++i) {
        if (((valid >> i) & 1U) == 0U) {
          continue;
        }
        if (!have) {
          first = rx_[i].digest;
          have = true;
        } else if (rx_[i].digest != first) {
          return valid;
        }
      }
    }
    return 0U;
  }

  RedundancyConfig cfg_;
  std::array<ChannelMonitor, kNodes> mon_;
  std::array<StuckDetector, kNodes> stuck_;
  std::array<bool, kNodes> seen_{};
  std::array<std::array<SeqTracker, kStreams>, kNodes> seq_{};
  std::array<NodeRx, kNodes> rx_{};
  Counters counters_{};
  FrameReport report_{};
};

}  // namespace tfc
