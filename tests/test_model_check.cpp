// SPDX-License-Identifier: MIT
// A bounded exhaustive check of the voter and the fault manager (core/include/tfc/redundancy.hpp; docs/design/FUTURE_WORK.md 6a item 6).
//
// The state of the manager is explored the way an explicit-state model checker explores a model: every sequence of three phases, each a choice of what the three computers do (nothing wrong; or exactly one of them
// silent, with a bias on its gyro, with a wrong digest, or repeating its last sample) held for 3 or 25 frames, is run on a copy of the manager that the sequence before it left, and the properties are
// checked on every frame of every sequence (17,576 sequences, about 256,000 frames: the ones that get a node latched, serving its dwell, on probation and readmitted or thrown back are among them; the dwell
// and the probation are shortened to 10 frames so that three phases reach them, the code that counts them is the same). It is the real code and not a model of it, and the bound is the depth: no sequence
// of more than three phases, and none with two computers wrong in the same frame (that is the campaign's, which samples it).
//
// The properties, each a thing that must hold whatever the sequence:
//   P1  a computer that was out of the vote before the frame (latched, on probation or disabled) did not take part in it;
//   P2  the masks nest: disabled, on probation and newly latched are all within latched;
//   P3  no data that did not arrive is voted: the nodes that took part are among those that sent a frame (no phantom data);
//   P4  once Safe is requested it stays requested (nothing here clears it: that takes an operator's command);
//   P5  the output is finite, and when it is not held it lies between the smallest and the largest of the values that took part;
//   P6  while two computers are still voting, the output stays near the truth whichever one is wrong: a fault does not reach it (the claim of TFC-SYS-004, at depth); once the vote has been
//       down to one computer a lone survivor with a bias is the output, which is two faults and not one, and the property is not asked of it;
//   P7  two copies of the manager given the same frames give the same reports (determinism).
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

enum Fault : int { kNone = 0, kSilent = 1, kBias = 2, kDigest = 3, kFrozen = 4 };

float truth(int k) { return 5.0F + 0.125F * static_cast<float>(k % 16); }

struct Explorer {
  uint64_t sequences = 0U;
  uint64_t frames = 0U;
  uint64_t latch_events = 0U;
  uint64_t probation_events = 0U;
  uint64_t reintegration_events = 0U;
  uint64_t violations = 0U;
  char first[240] = {};
  int path[8] = {};         // the choices and durations of the sequence being run (for the report of the first violation)
  int path_len = 0;
  float last_out = 0.0F;
  int last_k = 0;

  void fail(const char* what) {
    if (violations++ == 0U) {
      (void)std::snprintf(first, sizeof first, "%s [frame %d, output %.3f, truth %.3f, choices", what, last_k, static_cast<double>(last_out), static_cast<double>(truth(last_k)));
      for (int i = 0; i < path_len; ++i) {
        (void)std::snprintf(first + std::strlen(first), sizeof first - std::strlen(first), " %d", path[i]);
      }
      (void)std::snprintf(first + std::strlen(first), sizeof first - std::strlen(first), "]");
    }
  }

  // One frame; which computer is wrong (-1 none) and how. Returns the report; checks the properties.
  void step(RedundancyManager& m, RedundancyManager& twin, int k, int bad, Fault fault, bool& safe_seen, bool& degraded) {
    for (RedundancyManager* mm : {&m, &twin}) {
      mm->begin_frame();
      const uint8_t seq = static_cast<uint8_t>(k);
      for (unsigned n = 0; n < 3; ++n) {
        const bool is_bad = static_cast<int>(n) == bad;
        if (is_bad && fault == kSilent) {
          continue;
        }
        int kk = k;
        if (is_bad && fault == kFrozen) {
          kk = 0;
        }
        const float bias = is_bad && fault == kBias ? 3.0F : 0.0F;
        mm->on_frame(pack_gyro(static_cast<uint8_t>(n), Vec3{{truth(kk) + bias, -2.0F, 1.0F}}, seq));
        mm->on_frame(pack_accel(static_cast<uint8_t>(n), Vec3{{0.0F, 0.0F, 1.0F}}, seq));
        mm->on_frame(pack_cmd(static_cast<uint8_t>(n), Command{0.5F, -0.25F, static_cast<uint16_t>(0x1234U ^ (is_bad && fault == kDigest ? 0x00FFU : 0U))}, seq));
      }
    }
    const FrameReport& r = m.end_frame();
    const FrameReport& rt = twin.end_frame();
    ++frames;
    uint8_t sent = 0U;
    for (unsigned n = 0; n < 3; ++n) {
      if (!(static_cast<int>(n) == bad && fault == kSilent)) {
        sent = static_cast<uint8_t>(sent | (1U << n));
      }
    }
    if ((r.valid_mask & r.latched_mask & static_cast<uint8_t>(~r.newly_latched)) != 0U) {
      fail("P1: a node that is out of the vote took part in it");
    }
    if ((r.disabled_mask & static_cast<uint8_t>(~r.latched_mask)) != 0U || (r.probation_mask & static_cast<uint8_t>(~r.latched_mask)) != 0U || (r.newly_latched & static_cast<uint8_t>(~r.latched_mask)) != 0U) {
      fail("P2: the masks do not nest");
    }
    if ((r.valid_mask & static_cast<uint8_t>(~sent)) != 0U) {
      fail("P3: a node that sent nothing took part in the vote");
    }
    if (safe_seen && !r.safe_request) {
      fail("P4: Safe was requested and is not any more");
    }
    safe_seen = safe_seen || r.safe_request;
    const float out = r.output[0];
    last_out = out;
    last_k = k;
    if (!std::isfinite(out)) {
      fail("P5: the output is not finite");
    } else if ((r.held_mask & 1U) == 0U) {
      float lo = 1e30F;
      float hi = -1e30F;
      for (unsigned n = 0; n < 3; ++n) {
        if (((r.valid_mask >> n) & 1U) == 0U) {
          continue;
        }
        const bool is_bad = static_cast<int>(n) == bad;
        const float v = truth(is_bad && fault == kFrozen ? 0 : k) + (is_bad && fault == kBias ? 3.0F : 0.0F);
        lo = std::fmin(lo, v);
        hi = std::fmax(hi, v);
      }
      if (r.valid_mask != 0U && (out < lo - 1e-3F || out > hi + 1e-3F)) {
        fail("P5: the output is outside the values that took part");
      }
    }
    // P6 holds until the vote has been down to one computer (a lone survivor with a bias is the whole output, and no vote can help it: that is two faults, not one)
    degraded = degraded || r.healthy < 2U;
    if (!degraded && std::fabs(out - truth(k)) > 2.5F) {
      fail("P6: the output is far from the truth with one computer wrong at a time");
    }
    if (r.mode != rt.mode || r.valid_mask != rt.valid_mask || r.latched_mask != rt.latched_mask || r.output[0] != rt.output[0] || r.healthy != rt.healthy || r.safe_request != rt.safe_request) {
      fail("P7: two managers given the same frames gave different reports");
    }
    latch_events += r.newly_latched != 0U ? 1U : 0U;
    probation_events += r.probation_started != 0U ? 1U : 0U;
    reintegration_events += r.newly_reintegrated != 0U ? 1U : 0U;
  }

  // One phase on copies of the manager and its twin: `choice` 0 is all good, 1 to 12 is computer (choice - 1) / 4 with the fault (choice - 1) % 4 + 1, held for `length` frames from frame `k`.
  void phase(RedundancyManager& a, RedundancyManager& b, int& k, int choice, int length, bool& safe, bool& degraded) {
    const int bad = choice == 0 ? -1 : (choice - 1) / 4;
    const Fault fault = choice == 0 ? kNone : static_cast<Fault>(((choice - 1) % 4) + 1);
    for (int i = 0; i < length; ++i, ++k) {
      step(a, b, k, bad, fault, safe, degraded);
    }
  }

  // Every sequence of three phases, each of 13 choices held for 3 or 25 frames, from the managers `m0` and `t0` at frame `k0` (the three levels are three loops: the checker's own code has no recursion either).
  void explore3(const RedundancyManager& m0, const RedundancyManager& t0, int k0, bool safe0, bool degraded0) {
    constexpr int kDurations[2] = {3, 25};
    for (int c1 = 0; c1 < 13; ++c1) {
      for (const int d1 : kDurations) {
        RedundancyManager m1 = m0;
        RedundancyManager t1 = t0;
        bool s1 = safe0;
        bool g1 = degraded0;
        int k1 = k0;
        path[0] = (c1 * 100) + d1;
        path_len = 1;
        phase(m1, t1, k1, c1, d1, s1, g1);
        for (int c2 = 0; c2 < 13; ++c2) {
          for (const int d2 : kDurations) {
            RedundancyManager m2 = m1;
            RedundancyManager t2 = t1;
            bool s2 = s1;
            bool g2 = g1;
            int k2 = k1;
            path[1] = (c2 * 100) + d2;
            path_len = 2;
            phase(m2, t2, k2, c2, d2, s2, g2);
            for (int c3 = 0; c3 < 13; ++c3) {
              for (const int d3 : kDurations) {
                RedundancyManager m3 = m2;
                RedundancyManager t3 = t2;
                bool s3 = s2;
                bool g3 = g2;
                int k3 = k2;
                path[2] = (c3 * 100) + d3;
                path_len = 3;
                phase(m3, t3, k3, c3, d3, s3, g3);
                ++sequences;
              }
            }
          }
        }
      }
    }
  }
};

}  // namespace

TFC_TEST(modelcheck_every_sequence_of_three_phases_with_one_computer_wrong_at_a_time_keeps_the_manager_within_its_properties) {
  Explorer x;
  RedundancyConfig cfg;
  cfg.policy = ReintegrationPolicy::AutoTransient;
  cfg.min_dwell_frames_transient = 10;
  cfg.min_dwell_frames = 20;
  cfg.probation_frames = 10;
  cfg.probation_frames_repeat = 20;
  RedundancyManager m(cfg);
  RedundancyManager twin(cfg);
  bool safe = false;
  bool degraded = false;
  for (int k = 0; k < 20; ++k) {                       // twenty good frames first: every computer seen, the history full
    x.step(m, twin, k, -1, kNone, safe, degraded);
  }
  x.explore3(m, twin, 20, safe, degraded);
  if (x.violations != 0U) {
    std::printf("    first violation: %s (%llu of them)\n", x.first, static_cast<unsigned long long>(x.violations));
  }
  CHECK(x.violations == 0U);
  CHECK(x.sequences == 17576U);                         // (13 choices x 2 durations) cubed
  CHECK(x.frames > 200000U);
  CHECK(x.latch_events > 1000U);                        // the exploration reached a node being latched out, serving its dwell and coming back on probation...
  CHECK(x.probation_events > 100U);
  CHECK(x.reintegration_events > 0U);                   // ... and being readmitted
  std::printf("    %llu sequences, %llu frames, %llu latches, %llu probations, %llu readmissions\n", static_cast<unsigned long long>(x.sequences), static_cast<unsigned long long>(x.frames),
              static_cast<unsigned long long>(x.latch_events), static_cast<unsigned long long>(x.probation_events), static_cast<unsigned long long>(x.reintegration_events));
}
