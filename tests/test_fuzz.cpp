// SPDX-License-Identifier: MIT
// Property and fuzz tests. Random (but deterministic, seeded) fault processes and malformed traffic are thrown at the
// protocol decoders, the voter and the RedundancyManager; after EVERY frame a set of invariants must hold. Run under
// ASan + UBSan in CI, so any out-of-bounds access, overflow or undefined behaviour on hostile input fails the build.
//
// Invariants checked every frame (the list is in docs/verification/FAULT_CAMPAIGN.md section "Properties"):
//   I1  every output finite and bounded                   I5  one node on probation at most
//   I2  mode == Safe if a Safe request, else healthy count  I6  legal node-state transitions
//   I3  a node out of the vote last frame does not vote     I7  Safe request is sticky until a clear-safe command
//   I4  safe request => every output channel held           I8  counters never decrease; frames counted exactly once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "tfc/fault_monitor.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/voter.hpp"
#include "ground.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

// xorshift64*: tiny, deterministic, identical on every platform.
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0x1234567ULL) {
    for (int i = 0; i < 4; ++i) next();
  }
  uint64_t next() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545F4914F6CDD1DULL;
  }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>((next() >> 33) % n); }
  bool chance(uint32_t per_thousand) { return below(1000) < per_thousand; }
  float range(float lo, float hi) { return lo + (hi - lo) * static_cast<float>(next() >> 40) / 16777216.0F; }
};

#define FUZZ_CHECK(cond)                                                                       \
  do {                                                                                         \
    if (!(cond)) {                                                                             \
      std::printf("    FUZZ seed=%llu frame=%d: %s\n", static_cast<unsigned long long>(seed), k, #cond); \
      CHECK(cond);                                                                             \
      return false;                                                                            \
    }                                                                                          \
  } while (0)

bool legal(NodeState from, NodeState to, bool clear_disabled_cmd) {
  switch (from) {
    case NodeState::Healthy: return to == NodeState::Healthy || to == NodeState::Latched || to == NodeState::Disabled;
    case NodeState::Latched: return to == NodeState::Latched || to == NodeState::Probation || to == NodeState::Disabled;
    case NodeState::Probation: return true;  // readmitted, bounced back, still on probation, or disabled by command
    case NodeState::Disabled: return to == NodeState::Disabled || (to == NodeState::Latched && clear_disabled_cmd);
  }
  return false;
}

// The per-frame invariant checker, shared by every fuzzer. Returns false (after printing the seed and frame) on a violation.
struct Checker {
  uint64_t seed;
  std::array<NodeState, 3> prev_state{NodeState::Healthy, NodeState::Healthy, NodeState::Healthy};
  uint8_t prev_latched = 0;
  bool prev_safe = false;
  Counters prev_counters{};
  unsigned safe_frames = 0;
  unsigned readmissions = 0;
  unsigned disables = 0;

  bool after_frame(RedundancyManager& m, const FrameReport& r, int k, bool clear_disabled_cmd, bool clear_safe_cmd) {
    for (unsigned ch = 0; ch < kVoteChannels; ++ch) {
      FUZZ_CHECK(std::isfinite(r.output[ch]));
      FUZZ_CHECK(std::fabs(r.output[ch]) < 1.0e5F);
    }
    FUZZ_CHECK(r.healthy <= 3U);
    FUZZ_CHECK((r.mode == Mode::Safe) == (r.safe_request || r.healthy == 0U));
    if (!r.safe_request) {
      FUZZ_CHECK(static_cast<unsigned>(r.mode) == r.healthy);
    }
    FUZZ_CHECK((r.valid_mask & prev_latched) == 0U);
    if (r.safe_request) {
      FUZZ_CHECK(r.held_mask == 0xFFU);
      FUZZ_CHECK(r.mode == Mode::Safe);
      ++safe_frames;
    }
    // A probation never starts while another one is running, except in total loss (no node Healthy at that moment),
    // when the probationers judge each other (ADR-014). The rest of a cohort may outlive the first member's readmission.
    FUZZ_CHECK(r.probation_started == 0U || count_channels(r.probation_mask) <= 1U || r.latched_mask == 0x7U);
    FUZZ_CHECK((r.disabled_mask & r.probation_mask) == 0U);
    FUZZ_CHECK((r.probation_mask & ~r.latched_mask) == 0U);   // probation and disabled are subsets of "out of the vote"
    FUZZ_CHECK((r.disabled_mask & ~r.latched_mask) == 0U);
    for (unsigned n = 0; n < 3; ++n) {
      const NodeState st = m.state(n);
      FUZZ_CHECK(legal(prev_state[n], st, clear_disabled_cmd));
      // every state change is reported as an event (the console and the operator rely on these)
      const uint8_t bit = static_cast<uint8_t>(1U << n);
      if (prev_state[n] != NodeState::Disabled && st == NodeState::Disabled) {
        FUZZ_CHECK((r.newly_disabled & bit) != 0U);
      }
      if (prev_state[n] != NodeState::Healthy && st == NodeState::Healthy) {
        FUZZ_CHECK((r.newly_reintegrated & bit) != 0U);
      }
      if (prev_state[n] == NodeState::Healthy && st == NodeState::Latched) {
        FUZZ_CHECK((r.newly_latched & bit) != 0U);
      }
      if (prev_state[n] != NodeState::Probation && st == NodeState::Probation) {
        FUZZ_CHECK((r.probation_started & bit) != 0U);
      }
      if (prev_state[n] == NodeState::Probation && st == NodeState::Latched) {
        FUZZ_CHECK((r.probation_failed & bit) != 0U);
      }
      prev_state[n] = st;
      FUZZ_CHECK(r.strikes[n] == m.strikes(n));
      FUZZ_CHECK(((r.newly_reintegrated >> n) & 1U) == 0U || st == NodeState::Healthy);
      FUZZ_CHECK(((r.newly_disabled >> n) & 1U) == 0U || st == NodeState::Disabled);
    }
    readmissions += count_channels(r.newly_reintegrated);
    disables += count_channels(r.newly_disabled);
    if (prev_safe && !clear_safe_cmd) {
      FUZZ_CHECK(r.safe_request);  // sticky
    }
    // the effect of an accepted command is visible in the same frame
    FUZZ_CHECK(r.command_count <= kMaxCommandsPerFrame);
    for (unsigned i = 0; i < r.command_count; ++i) {
      const CommandEvent& e = r.commands[i];
      if (e.result == CommandResult::Accepted && e.node < 3U && (e.flags & cmdflag::kArm) == 0U) {  // an executed command, not an ARM
        if (e.op == static_cast<uint8_t>(GroundOp::Disable)) {
          FUZZ_CHECK(m.state(e.node) == NodeState::Disabled);
        }
        if (e.op == static_cast<uint8_t>(GroundOp::ClearDisabled)) {
          FUZZ_CHECK(m.state(e.node) != NodeState::Disabled && m.strikes(e.node) == 0U);
        }
        if ((e.flags & cmdflag::kArm) == 0U) {  // an executed command: the dangerous ones only ever run under an ARM (ADR-019)
          if (e.op == static_cast<uint8_t>(GroundOp::ClearDisabled) || e.op == static_cast<uint8_t>(GroundOp::ClearSafe)) {
            FUZZ_CHECK((e.flags & cmdflag::kArmed) != 0U);
          }
          if ((e.flags & cmdflag::kCritical) != 0U) {
            FUZZ_CHECK((e.flags & cmdflag::kArmed) != 0U && e.op == static_cast<uint8_t>(GroundOp::Disable));
          }
        }
      }
    }
    const Counters& c = m.counters();
    FUZZ_CHECK(c.frames == static_cast<uint32_t>(k) + 1U);
    FUZZ_CHECK(c.crc_bad >= prev_counters.crc_bad && c.seq_bad >= prev_counters.seq_bad &&
               c.missing >= prev_counters.missing && c.out_of_schedule >= prev_counters.out_of_schedule &&
               c.reintegrations >= prev_counters.reintegrations && c.probations_started >= prev_counters.probations_started &&
               c.nodes_disabled >= prev_counters.nodes_disabled && c.held_frames >= prev_counters.held_frames &&
               c.safe_request_frames >= prev_counters.safe_request_frames);
    prev_counters = c;
    prev_safe = r.safe_request;
    prev_latched = r.latched_mask;
    return true;
  }
};

// An operator (and the hostile bus around them): authentic commands with a fresh counter, usually ARMed first when the operation
// is dangerous, plus forged tags, stale counters, corrupted frames and nonsense opcodes. Sets the flags the checker needs
// (a clear-disabled / clear-safe EXECUTE was sent in this frame).
struct Operator {
  uint8_t counter = 0;
  uint8_t last_op = 0;
  uint8_t last_node = 0;
  bool last_arm = false;
};

void operator_acts(RedundancyManager& m, Rng& rng, Operator& st, bool hostile, bool& clear_disabled_cmd, bool& clear_safe_cmd) {
  const uint8_t op = static_cast<uint8_t>(hostile ? rng.below(7) : 1 + rng.below(4));
  const uint8_t node = static_cast<uint8_t>(hostile ? rng.below(6) : rng.below(3));
  const unsigned flavour = rng.below(100);
  const bool dangerous = op == static_cast<uint8_t>(GroundOp::ClearDisabled) || op == static_cast<uint8_t>(GroundOp::ClearSafe) ||
                         op == static_cast<uint8_t>(GroundOp::Disable);
  if (dangerous && rng.chance(600)) {  // the careful operator arms first
    m.on_frame(tfct::gcmd_raw(static_cast<uint8_t>(op | kArmFlag), node, ++st.counter));
  }
  Frame f = tfct::gcmd_raw(op, node, ++st.counter);
  if (flavour < 4) {
    f.data[2] = static_cast<uint8_t>(f.data[2] ^ 0x10U);  // a forged tag
    f.data[7] = crc8(f.data.data(), 7);
  } else if (flavour < 8) {
    f.data[6] = static_cast<uint8_t>(st.counter - 40U);  // a stale counter (tag now wrong too: replay of an old frame is modelled below)
    f.data[7] = crc8(f.data.data(), 7);
  } else if (flavour < 12) {
    f = tfct::gcmd_raw(st.last_op, st.last_node, st.counter);  // re-send the previous command with the NEW counter
    f.data[6] = static_cast<uint8_t>(st.counter - 1U);         // ... but an old one: a replay (tag mismatch or stale)
    f.data[7] = crc8(f.data.data(), 7);
  } else if (flavour < 15) {
    f.data[7] = static_cast<uint8_t>(f.data[7] ^ 1U);  // corrupted
  }
  st.last_op = op;
  st.last_node = node;
  clear_disabled_cmd = clear_disabled_cmd || op == static_cast<uint8_t>(GroundOp::ClearDisabled);
  clear_safe_cmd = clear_safe_cmd || op == static_cast<uint8_t>(GroundOp::ClearSafe);
  m.on_frame(f);
}

// Chaos fuzzer: `frames` frames at level `chaos` (0 = calm .. 3 = storm), every behaviour drawn at random each frame.
bool run_fuzz(uint64_t seed, int frames, unsigned chaos, ReintegrationPolicy policy) {
  Rng rng(seed);
  RedundancyConfig cfg;
  cfg.policy = policy;
  cfg.startup_grace_frames = rng.below(4) == 0 ? rng.below(50) : 0U;
  RedundancyManager m(cfg);
  Checker chk{seed};
  Operator op_state{};
  std::array<Frame, 8> late_pool{};
  unsigned late_n = 0;
  float tr = 5.0F;

  for (int k = 0; k < frames; ++k) {
    m.begin_frame();
    bool clear_disabled_cmd = false;
    bool clear_safe_cmd = false;
    const uint8_t seq = static_cast<uint8_t>(k);
    tr = 5.0F + 3.0F * std::sin(0.2F * static_cast<float>(k));

    for (unsigned i = 0; i < late_n; ++i) {  // frames that missed the previous vote arrive now
      m.on_frame(late_pool[i]);
    }
    late_n = 0;

    for (unsigned n = 0; n < 3; ++n) {
      uint32_t mode = rng.below(100);
      const uint32_t storm = chaos * 12;  // percent of frames with something wrong, per node
      mode = mode >= storm ? 0U : 1U + rng.below(12);
      Frame g = pack_gyro(static_cast<uint8_t>(n), Vec3{{tr, -2.0F, 1.0F}}, seq);
      Frame a = pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F + 0.001F * static_cast<float>(k % 7)}}, seq);
      Frame c = pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, 0x1234U}, seq);
      switch (mode) {
        case 1: continue;                       // node silent
        case 2: g = Frame{}; g.id = id::kGyroBase + n; break;  // empty frame (len 0)
        case 3: g.data[rng.below(8)] ^= static_cast<uint8_t>(1U << rng.below(8)); break;   // bit flip, CRC now wrong
        case 4: g = pack_gyro(static_cast<uint8_t>(n), Vec3{{rng.range(-9000.0F, 9000.0F), rng.range(-9000.0F, 9000.0F), 0.0F}}, seq); break;
        case 5: c = pack_cmd(static_cast<uint8_t>(n), Command{rng.range(-90.0F, 90.0F), rng.range(-90.0F, 90.0F), static_cast<uint16_t>(rng.next())}, seq); break;
        case 6: g = pack_gyro(static_cast<uint8_t>(n), Vec3{{tr, -2.0F, 1.0F}}, static_cast<uint8_t>(rng.next())); break;  // wrong sequence
        case 7: m.on_frame(g); break;           // duplicate: g is sent twice
        case 8:                                  // g misses the vote and arrives next frame
          if (late_n < late_pool.size()) late_pool[late_n++] = g;
          g = Frame{};
          g.id = 0x7FF;  // a placeholder id outside every range; the real frame comes next frame
          break;
        case 9: {                                // random payload with a VALID CRC: any int16 values at all
          for (unsigned i = 0; i < 6; ++i) g.data[i] = static_cast<uint8_t>(rng.next());
          g.data[6] = seq;
          g.data[7] = crc8(g.data.data(), 7);
          break;
        }
        case 10: g.len = static_cast<uint8_t>(rng.below(256)); break;  // length garbage
        case 11: a = Frame{}; a.id = id::kAccelBase + n; a.len = 8; break;  // all-zero payload, wrong CRC
        default: c.id = static_cast<uint32_t>(rng.next());             // command with a random (possibly huge) id
      }
      m.on_frame(g);
      m.on_frame(a);
      m.on_frame(c);
    }

    const unsigned junk = chaos == 0 ? 0 : rng.below(chaos * 3);  // hostile bus traffic
    for (unsigned i = 0; i < junk; ++i) {
      Frame f;
      f.id = rng.chance(500) ? rng.below(0x800) : static_cast<uint32_t>(rng.next());
      f.len = static_cast<uint8_t>(rng.below(256));
      for (auto& b : f.data) b = static_cast<uint8_t>(rng.next());
      m.on_frame(f);
    }
    if (rng.chance(chaos == 0 ? 10 : 60)) {  // operator commands, valid and not
      operator_acts(m, rng, op_state, true, clear_disabled_cmd, clear_safe_cmd);
    }
    if (!chk.after_frame(m, m.end_frame(), k, clear_disabled_cmd, clear_safe_cmd)) {
      return false;
    }
  }
  return true;
}

// Structured fuzzer: campaign-style fault processes (a node dies, a survivor then drifts / diverges / is biased just above
// tolerance, commands arrive at random moments). This is what actually drives Duplex into unresolved disagreements, Safe
// requests, probations, strikes and disabling, which pure per-frame chaos latches too quickly to reach.
struct Fault {
  unsigned node;
  int start;
  int end;                 // exclusive
  unsigned kind;           // 0 dropout 1 bias 2 drift 3 digest 4 cmd offset 5 stuck 6 intermittent corrupt 7 saturate 8 late
  float mag;
  int period;
};

bool run_structured(uint64_t seed, int frames, ReintegrationPolicy policy, Checker& chk_out) {
  Rng rng(seed);
  RedundancyConfig cfg;
  cfg.policy = policy;
  RedundancyManager m(cfg);
  Checker chk{seed};
  Operator op_state{};
  std::array<Fault, 5> faults{};
  const unsigned nf = 1 + rng.below(5);
  for (unsigned i = 0; i < nf; ++i) {
    Fault& f = faults[i];
    f.node = rng.below(3);
    f.start = static_cast<int>(rng.below(static_cast<uint32_t>(frames / 2)));
    f.end = f.start + 1 + static_cast<int>(rng.below(rng.chance(300) ? 40U : static_cast<uint32_t>(frames)));
    f.kind = rng.below(9);
    static const float mags[] = {0.3F, 0.9F, 1.3F, 1.8F, 3.0F, 30.0F, 0.05F, 0.01F};
    f.mag = mags[rng.below(8)];
    f.period = 2 + static_cast<int>(rng.below(9));
  }
  float frozen = 0.0F;
  std::array<Frame, 3> late_g{};
  std::array<bool, 3> have_late{};
  for (int k = 0; k < frames; ++k) {
    m.begin_frame();
    bool clear_disabled_cmd = false;
    bool clear_safe_cmd = false;
    const uint8_t seq = static_cast<uint8_t>(k);
    const float tr = 5.0F + 3.0F * std::sin(0.2F * static_cast<float>(k));
    for (unsigned n = 0; n < 3; ++n) {
      float gyro = tr;
      uint16_t dig = 0x1234U;
      float pitch = 0.5F;
      bool silent = false;
      bool corrupt = false;
      bool late = false;
      float accel_z = 1.0F + 0.001F * static_cast<float>(k % 7);
      for (unsigned i = 0; i < nf; ++i) {
        const Fault& f = faults[i];
        if (f.node != n || k < f.start || k >= f.end) continue;
        switch (f.kind) {
          case 0: silent = true; break;
          case 1: gyro += f.mag; break;
          case 2: gyro += f.mag * 0.05F * static_cast<float>(k - f.start + 1); break;
          case 3: dig = static_cast<uint16_t>(dig ^ static_cast<uint16_t>(1U + static_cast<unsigned>(f.mag))); break;
          case 4: pitch += f.mag * 0.05F; break;
          case 5: if (k == f.start) frozen = tr; gyro = frozen; accel_z = 1.0F; break;
          case 6: corrupt = ((k - f.start) % f.period) == 0; break;
          case 7: gyro = (k % 2 == 0) ? 9000.0F : -9000.0F; break;
          default: late = true; break;
        }
      }
      if (silent) continue;
      Frame g = pack_gyro(static_cast<uint8_t>(n), Vec3{{gyro, -2.0F, 1.0F}}, seq);
      Frame a = pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, accel_z}}, seq);
      Frame c = pack_cmd(static_cast<uint8_t>(n), Command{pitch, -0.25F, dig}, seq);
      if (corrupt) g.data[0] = static_cast<uint8_t>(g.data[0] ^ 1U);
      if (have_late[n]) {  // last frame's gyro arrives a frame late
        m.on_frame(late_g[n]);
        have_late[n] = false;
      }
      if (late) {
        late_g[n] = g;
        have_late[n] = true;
      } else {
        m.on_frame(g);
      }
      m.on_frame(a);
      m.on_frame(c);
    }
    if (rng.chance(25)) {  // an operator acts at a random moment on a random node
      operator_acts(m, rng, op_state, false, clear_disabled_cmd, clear_safe_cmd);
    }
    if (!chk.after_frame(m, m.end_frame(), k, clear_disabled_cmd, clear_safe_cmd)) {
      return false;
    }
  }
  chk_out.safe_frames += chk.safe_frames;
  chk_out.readmissions += chk.readmissions;
  chk_out.disables += chk.disables;
  return true;
}

}  // namespace

TFC_TEST(fuzz_manager_invariants_hold_under_random_fault_processes) {
  int bad_runs = 0;
  for (uint64_t seed = 1; seed <= 120; ++seed) {
    const unsigned chaos = static_cast<unsigned>(seed % 4);
    const ReintegrationPolicy pol = (seed % 3 == 0) ? ReintegrationPolicy::AutoTransient : ReintegrationPolicy::Manual;
    if (!run_fuzz(seed, 1500, chaos, pol)) {
      if (++bad_runs >= 3) break;  // enough evidence; stop flooding the log
    }
  }
  CHECK(bad_runs == 0);
}

TFC_TEST(fuzz_structured_fault_processes_reach_every_state_and_keep_every_invariant) {
  // 600 campaign-style runs. The test also proves the fuzzer is not vacuous: it must actually reach Safe requests,
  // readmissions and disabled nodes, otherwise the invariants about those states were never exercised.
  Checker total{0};
  int bad = 0;
  for (uint64_t seed = 1; seed <= 600; ++seed) {
    const ReintegrationPolicy pol = (seed % 2 == 0) ? ReintegrationPolicy::AutoTransient : ReintegrationPolicy::Manual;
    if (!run_structured(seed, 1200, pol, total) && ++bad >= 3) break;
  }
  CHECK(bad == 0);
  CHECK(total.safe_frames > 1000U);     // Safe requests were reached, and held
  CHECK(total.readmissions > 20U);      // probation led to readmissions
  CHECK(total.disables > 20U);          // strikes led to disabling
}

TFC_TEST(fuzz_manager_survives_a_long_storm_without_counter_overflow_problems) {
  CHECK(run_fuzz(0xC0FFEEULL, 40000, 3, ReintegrationPolicy::AutoTransient));
}

TFC_TEST(fuzz_protocol_roundtrip_crc_and_quantisation_properties) {
  Rng rng(42);
  for (int i = 0; i < 20000; ++i) {
    const float v = rng.range(-5000.0F, 5000.0F);
    const int16_t q = quantize(v, kGyroLsbDps);
    // within range: the round trip is within half an LSB, and saturation is monotonic
    if (std::fabs(v) < 4000.0F) {
      CHECK(std::fabs(static_cast<float>(q) * kGyroLsbDps - v) <= 0.5F * kGyroLsbDps + 1e-3F);
    }
    const int16_t q2 = quantize(v + 1.0F, kGyroLsbDps);
    CHECK(q2 >= q);
    // any frame built by pack_* verifies; any single flipped bit does not
    Frame f = pack_gyro(static_cast<uint8_t>(rng.below(3)), Vec3{{v, v * 0.5F, -v}}, static_cast<uint8_t>(rng.next()));
    CHECK(unpack_vec3(f, kGyroLsbDps).ok);
    const unsigned bit = rng.below(64);
    Frame g = f;
    g.data[bit / 8U] = static_cast<uint8_t>(g.data[bit / 8U] ^ (1U << (bit % 8U)));
    CHECK(!unpack_vec3(g, kGyroLsbDps).ok);
  }
  // non-finite inputs never produce a garbage frame or UB
  const float nan = std::nanf("");
  CHECK(unpack_vec3(pack_gyro(0, Vec3{{nan, INFINITY, -INFINITY}}, 1), kGyroLsbDps).ok);
  CHECK(quantize(nan, 1.0F) == 0);
  CHECK(quantize(INFINITY, 1.0F) == 32767 && quantize(-INFINITY, 1.0F) == -32768);
}

TFC_TEST(fuzz_decoders_reject_every_malformed_frame_without_misbehaving) {
  Rng rng(7);
  for (int i = 0; i < 50000; ++i) {
    Frame f;
    f.id = static_cast<uint32_t>(rng.next());
    f.len = static_cast<uint8_t>(rng.below(256));
    for (auto& b : f.data) b = static_cast<uint8_t>(rng.next());
    // decoders only accept len == 8 with a good CRC; anything else is rejected, never read out of bounds
    const bool good = f.len == 8U && f.data[7] == crc8(f.data.data(), 7);
    CHECK(unpack_vec3(f, kGyroLsbDps).ok == good);
    CHECK(unpack_cmd(f).ok == good);
    CHECK(unpack_sync(f).ok == (good && f.id == id::kSync));
    CHECK(unpack_ground(f).ok == (good && f.id == id::kGround));
  }
}

TFC_TEST(fuzz_voter_properties) {
  Rng rng(99);
  for (int i = 0; i < 50000; ++i) {
    std::array<float, 3> x{};
    for (auto& v : x) {
      const uint32_t kind = rng.below(20);
      v = kind == 0 ? std::nanf("") : kind == 1 ? INFINITY : kind == 2 ? -INFINITY : rng.range(-100.0F, 100.0F);
    }
    const uint8_t mask = static_cast<uint8_t>(rng.below(8));
    const float tol = rng.range(0.001F, 5.0F);
    const VoteResult r = vote3(x, mask, tol);
    CHECK(std::isfinite(r.value));                 // never propagates NaN/Inf
    CHECK((r.disagree_mask & ~kAllChannels) == 0U);
    // a non-finite input is never counted as valid: it is blamed
    for (unsigned n = 0; n < 3; ++n) {
      if (((mask >> n) & 1U) != 0U && !std::isfinite(x[n])) {
        CHECK(((r.disagree_mask >> n) & 1U) != 0U);
      }
    }
    // a triplex vote returns one of the inputs exactly (the median: no arithmetic)
    if (r.status == VoteStatus::Triplex) {
      CHECK(r.value == x[0] || r.value == x[1] || r.value == x[2]);
    }
    // the median does not depend on the order of its inputs
    const float a = rng.range(-9.0F, 9.0F);
    const float b = rng.range(-9.0F, 9.0F);
    const float c = rng.range(-9.0F, 9.0F);
    const float m = median3(a, b, c);
    CHECK(m == median3(b, a, c) && m == median3(c, b, a) && m == median3(a, c, b));
    CHECK((m <= std::fmax(a, std::fmax(b, c))) && (m >= std::fmin(a, std::fmin(b, c))));
    // the defining property of a median: at least two inputs are <= it and at least two are >= it (a min or a max fails this)
    const int le = (a <= m ? 1 : 0) + (b <= m ? 1 : 0) + (c <= m ? 1 : 0);
    const int ge = (a >= m ? 1 : 0) + (b >= m ? 1 : 0) + (c >= m ? 1 : 0);
    CHECK(le >= 2 && ge >= 2);
  }
}

TFC_TEST(fuzz_channel_monitor_and_alpha_count_properties) {
  Rng rng(5);
  for (int trial = 0; trial < 300; ++trial) {
    ChannelMonitor mon(3, 5, 100, 3);
    AlphaCount alpha(0.9F, 3.0F);
    int bad_total = 0;
    for (int k = 0; k < 400; ++k) {
      const bool bad = rng.chance(60 + (trial % 5) * 80);
      bad_total += bad ? 1 : 0;
      const bool by_window = mon.update(bad);
      const bool by_alpha = alpha.update(bad);
      CHECK(!by_window || bad_total >= 3);          // 3-of-5 can never fire with fewer than 3 bad frames
      CHECK(alpha.score() >= 0.0F && alpha.score() < 66000.0F);
      if (by_alpha) {
        CHECK(alpha.score() >= 3.0F);
      }
      if (mon.latched()) break;
    }
  }
}
