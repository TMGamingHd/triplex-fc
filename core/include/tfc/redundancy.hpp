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
#include <cmath>
#include <cstdint>
#include <cstring>

#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace tfc {

constexpr unsigned kNodes = kChannels;      // flight computers A, B, C
constexpr unsigned kVoteChannels = 8;       // gyro x3, accel x3, command pitch, command yaw
constexpr unsigned kStreams = 3;            // per node: gyro, accel, command
constexpr unsigned kChPitch = 6;            // indices into FrameReport::output / votes
constexpr unsigned kChYaw = 7;

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
  // Duplex arbitration (two valid nodes that disagree): blame a node only if it jumped more than
  // `factor` x tolerance away from the last agreed value (set by the previous frame) while the other
  // stayed within one tolerance of it. Otherwise the disagreement is unresolved: nobody is blamed,
  // the last good output is held (and, being stale, never used to blame anyone later), and a
  // persistent run of them requests Safe. 0 disables arbitration.
  float duplex_arbitration_factor = 2.0F;
  // This many out-of-schedule frames inside one 10 ms frame raises the bus alarm (babbling node).
  uint32_t bus_alarm_per_frame = 3;
};

struct Counters {
  uint32_t frames = 0;
  uint32_t crc_bad = 0;             // frames discarded for a failed CRC
  uint32_t seq_bad = 0;             // frames whose sequence number was not the expected next
  uint32_t missing = 0;             // node-frames where gyro/accel/command did not all arrive
  uint32_t out_of_schedule = 0;     // frames on IDs that are not part of the schedule
  uint32_t stuck_flags = 0;
  uint32_t digest_flags = 0;
  uint32_t vote_disagreements = 0;  // frames where any channel vote blamed a node
  uint32_t unresolved_frames = 0;   // frames with a disagreement nobody could be blamed for
  uint32_t held_frames = 0;         // frames in which some output channel held its last good value
  uint32_t safe_request_frames = 0; // frames spent requesting Safe
  uint32_t bus_alarm_frames = 0;    // frames with the out-of-schedule flood alarm raised
};

struct FrameReport {
  uint8_t valid_mask = 0U;      // nodes whose data took part in the vote this frame
  uint8_t newly_latched = 0U;   // nodes that latched on this frame
  uint8_t newly_seen = 0U;      // nodes whose first good sample arrived on this frame
  uint8_t latched_mask = 0U;    // nodes currently latched out
  std::array<uint8_t, kNodes> reason{};  // reason:: bits, per node, this frame
  Mode mode = Mode::Safe;
  unsigned healthy = 0U;        // nodes that have been seen and are not latched out
  std::array<VoteResult, kVoteChannels> votes{};  // raw vote result per channel (value undefined on a miscompare)
  // What a downstream consumer should use: the voted value, or, when the vote could not produce a
  // trustworthy one (duplex disagreement nobody can be blamed for, no majority, no data), the last
  // good value. 0 before there is any history. `held_mask` has bit ch set when output[ch] is held.
  std::array<float, kVoteChannels> output{};
  uint8_t held_mask = 0U;
  bool unresolved = false;      // a disagreement this frame that no node could be blamed for
  bool safe_request = false;    // unresolved disagreements persisted (M of the last N frames): Safe is
                                // requested and STAYS requested until clear_safe_request(); outputs are held
  bool bus_alarm = false;       // out-of-schedule frames this frame >= RedundancyConfig::bus_alarm_per_frame
  uint32_t out_of_schedule_in_frame = 0U;
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

  void begin_frame() noexcept {
    rx_ = {};
    oos_in_frame_ = 0U;
  }

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
        ++oos_in_frame_;
      }
      return known;
    }
    NodeRx& r = rx_[node];
    r.arrived[stream] = true;
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
      for (unsigned st = 0; st < kStreams; ++st) {
        if (!r.arrived[st]) {
          seq_[n][st].note_missing();  // slot passed with nothing: the sender's counter still advanced
        }
      }
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
    bool unresolved = false;
    for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
      const std::array<float, kNodes> x = {rx_[0].x[ch], rx_[1].x[ch], rx_[2].x[ch]};
      const VoteResult v = vote3(x, valid, cfg_.tol[ch]);
      rep.votes[ch] = v;
      uint8_t blame = v.disagree_mask;
      bool trusted = true;
      float out = v.value;
      switch (v.status) {
        case VoteStatus::Triplex:
        case VoteStatus::Duplex:
        case VoteStatus::Simplex:
          break;
        case VoteStatus::DuplexMiscompare:
          // Two nodes disagree: the vote alone cannot say who is wrong. Use continuity with the last
          // agreed value; if that does not single one out, blame nobody.
          blame = 0U;
          trusted = arbitrate(ch, x, valid, out, blame);
          if (!trusted) {
            unresolved = true;
          }
          break;
        case VoteStatus::NoMajority:
        case VoteStatus::NoData:
          trusted = false;
          break;
      }
      if (trusted && !safe_latched_) {
        last_good_[ch] = out;
        have_last_[ch] = true;
        ref_fresh_[ch] = true;
        rep.output[ch] = out;
      } else {
        ref_fresh_[ch] = false;  // a held reference goes stale: it must not be used to blame anyone
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;  // hold the last good value
        rep.held_mask = static_cast<uint8_t>(rep.held_mask | (1U << ch));
      }
      disagree = static_cast<uint8_t>(disagree | blame);
    }
    const DigestVerdict dv = digest_outliers(valid);
    const uint8_t digest_bad = dv.blame;
    unresolved = unresolved || dv.unresolved;
    if (disagree != 0U) {
      ++counters_.vote_disagreements;
    }
    if (digest_bad != 0U || dv.unresolved) {
      ++counters_.digest_flags;
    }
    rep.unresolved = unresolved;
    unres_hist_ = (unres_hist_ << 1) | (unresolved ? 1U : 0U);
    const uint32_t window = cfg_.persist_n >= 32U ? 0xFFFFFFFFU : ((1U << cfg_.persist_n) - 1U);
    if (popcount32(unres_hist_ & window) >= cfg_.persist_m) {
      safe_latched_ = true;  // sticky: only clear_safe_request() (operator/ground) lifts it
    }
    rep.safe_request = safe_latched_;
    if (safe_latched_) {  // hold the last voted values for the whole time Safe is requested
      for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
        rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;
      }
      rep.held_mask = 0xFFU;
    }
    rep.out_of_schedule_in_frame = oos_in_frame_;
    rep.bus_alarm = oos_in_frame_ >= cfg_.bus_alarm_per_frame && cfg_.bus_alarm_per_frame != 0U;
    counters_.unresolved_frames += unresolved ? 1U : 0U;
    counters_.safe_request_frames += rep.safe_request ? 1U : 0U;
    counters_.held_frames += rep.held_mask != 0U ? 1U : 0U;
    counters_.bus_alarm_frames += rep.bus_alarm ? 1U : 0U;

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
    rep.mode = rep.safe_request ? Mode::Safe : mode_from_healthy(rep.healthy);
    return rep;
  }

  // Operator/ground command: start counting clean frames toward reintegrating `node`.
  void request_reintegration(unsigned node) noexcept {
    if (node < kNodes) {
      mon_[node].request_reintegration();
    }
  }

  // Operator/ground command: lift a Safe request (the disagreement history is forgotten too).
  void clear_safe_request() noexcept {
    safe_latched_ = false;
    unres_hist_ = 0U;
  }

  bool safe_requested() const noexcept { return safe_latched_; }
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
    std::array<bool, kStreams> arrived{};  // a frame (good or damaged) came in on this stream
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

  struct DigestVerdict {
    uint8_t blame = 0U;       // nodes in the minority (only when three can be compared)
    bool unresolved = false;  // two comparable nodes disagree: nobody can be blamed
  };

  // Cross-check the estimator-state digests of the voting nodes. With three, the odd one out is
  // blamed. With two, a mismatch cannot be attributed.
  DigestVerdict digest_outliers(uint8_t valid) const noexcept {
    DigestVerdict d;
    const unsigned n = count_channels(valid);
    if (n == 3U) {
      const uint16_t a = rx_[0].digest;
      const uint16_t b = rx_[1].digest;
      const uint16_t c = rx_[2].digest;
      if (a == b && b == c) {
        return d;
      }
      if (a == b) {
        d.blame = 0x4U;
      } else if (a == c) {
        d.blame = 0x2U;
      } else {
        d.blame = b == c ? 0x1U : 0x7U;
      }
    } else if (n == 2U) {
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
          d.unresolved = true;
        }
      }
    }
    return d;
  }

  // Two valid nodes disagree on channel `ch`. Blame the one that jumped away from the last agreed
  // value (more than factor x tolerance) if the other stayed within one tolerance of it. Returns
  // true and sets `out` (the consistent node's value) and `blame` when resolved.
  bool arbitrate(unsigned ch, const std::array<float, kNodes>& x, uint8_t valid, float& out,
                 uint8_t& blame) const noexcept {
    // Only a fresh reference (the previous frame was trusted) can single a node out. After a hold the
    // vehicle may have moved arbitrarily far from the stale value, and a healthy node would look like
    // the outlier.
    if (!have_last_[ch] || !ref_fresh_[ch] || cfg_.duplex_arbitration_factor <= 0.0F) {
      return false;
    }
    unsigned idx[2] = {0U, 0U};
    unsigned cnt = 0U;
    for (unsigned i = 0; i < kNodes && cnt < 2U; ++i) {
      if (((valid >> i) & 1U) != 0U) {
        idx[cnt++] = i;
      }
    }
    const float tol = cfg_.tol[ch];
    const float far = cfg_.duplex_arbitration_factor * tol;
    const float dev0 = std::fabs(x[idx[0]] - last_good_[ch]);
    const float dev1 = std::fabs(x[idx[1]] - last_good_[ch]);
    if (dev0 > far && dev1 <= tol) {
      blame = static_cast<uint8_t>(1U << idx[0]);
      out = x[idx[1]];
      return true;
    }
    if (dev1 > far && dev0 <= tol) {
      blame = static_cast<uint8_t>(1U << idx[1]);
      out = x[idx[0]];
      return true;
    }
    return false;
  }

  RedundancyConfig cfg_;
  std::array<ChannelMonitor, kNodes> mon_;
  std::array<StuckDetector, kNodes> stuck_;
  std::array<bool, kNodes> seen_{};
  std::array<std::array<SeqTracker, kStreams>, kNodes> seq_{};
  std::array<NodeRx, kNodes> rx_{};
  std::array<float, kVoteChannels> last_good_{};   // last trustworthy voted value per channel
  std::array<bool, kVoteChannels> have_last_{};
  std::array<bool, kVoteChannels> ref_fresh_{};    // last_good_ was set by the previous frame's trusted vote
  uint32_t unres_hist_ = 0U;                       // 1 bit per frame: an unresolved disagreement
  bool safe_latched_ = false;
  uint32_t oos_in_frame_ = 0U;
  Counters counters_{};
  FrameReport report_{};
};

}  // namespace tfc
