// SPDX-License-Identifier: MIT
// State resynchronisation (TS-16 option C, docs/design/RESYNC.md). The replicas' estimators and controllers are stateful: two computers that were given a different set of sensor
// frames in one frame (one lost or late) compute different states, and the difference does not heal by itself. Every `period` frames each computer therefore broadcasts its state
// (quantised, 12 words, in four frames) and every computer adopts the mid-value of the states it received, component by component. After an adoption all computers that got the same
// shares hold the same bits.
//
// What the adoption also does is measure: how far each computer's own state was from the vote. A small difference is what a lost frame leaves; a large one is a state that has gone
// wrong, and `ResyncOutcome::large` reports it to the fault manager (`RedundancyManager::report_state_correction`) so that resynchronisation heals the replicas without hiding a failing one.
//
// Everything is integer arithmetic on the shared words except the (de)quantisation, which is the same on every replica. No heap, no exceptions, no RTTI. Every loop is bounded.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "tfc/controller.hpp"
#include "tfc/estimator.hpp"
#include "tfc/protocol.hpp"

namespace tfc::resync {

constexpr unsigned kWords = 12U;  // 4 quaternion, 3 bias, 2 integrators, 2 outputs, 1 meta
constexpr float kQuatLsb = 1.0F / 32767.0F;  // components are in [-1, 1]
constexpr float kBiasLsb = 1.0e-5F;          // rad/s: the integrator range of +-0.32 rad/s (18 degrees per second) is far beyond the 5 degrees per second limit
constexpr float kCtrlLsb = 0.001F;           // degrees: +-32.7, the controller's own digest resolution

// Which words are which, for the tolerances.
constexpr unsigned kQuatFirst = 0U;
constexpr unsigned kBiasFirst = 4U;
constexpr unsigned kCtrlFirst = 7U;
constexpr unsigned kMeta = 11U;

struct SharedState {
  std::array<int16_t, kWords> w{};
};

[[nodiscard]] constexpr int16_t to_word(float v, float lsb) noexcept {
  const float q = v / lsb;
  if (!(q < 32767.0F)) {
    return q > 0.0F ? int16_t{32767} : int16_t{0};  // saturates (and a NaN becomes 0: a shared word must be defined)
  }
  if (q <= -32767.0F) {
    return int16_t{-32767};
  }
  return static_cast<int16_t>(q >= 0.0F ? q + 0.5F : q - 0.5F);
}

[[nodiscard]] constexpr float from_word(int16_t w, float lsb) noexcept { return static_cast<float>(w) * lsb; }

// The shared form of a computer's state. The quaternion is taken with a non-negative scalar part (q and -q are the same attitude, and a mid-value of two signs would be nonsense).
[[nodiscard]] inline SharedState make_shared(const AttitudeEstimator::State& e, const Controller::State& c) noexcept {
  SharedState s;
  const float sign = e.q[0] < 0.0F ? -1.0F : 1.0F;
  for (unsigned i = 0; i < 4U; ++i) {
    s.w[kQuatFirst + i] = to_word(sign * e.q[i], kQuatLsb);
  }
  for (unsigned i = 0; i < 3U; ++i) {
    s.w[kBiasFirst + i] = to_word(e.bias[i], kBiasLsb);
  }
  s.w[kCtrlFirst] = to_word(c.integ[0], kCtrlLsb);
  s.w[kCtrlFirst + 1U] = to_word(c.integ[1], kCtrlLsb);
  s.w[kCtrlFirst + 2U] = to_word(c.out[0], kCtrlLsb);
  s.w[kCtrlFirst + 3U] = to_word(c.out[1], kCtrlLsb);
  s.w[kMeta] = static_cast<int16_t>((e.steps & 0xFFU) | (e.aligned ? 0x100U : 0U) | (e.rates_valid ? 0x200U : 0U));
  return s;
}

// The state a shared form stands for. The quaternion is renormalised (the quantisation moves its length by a few parts in 10^4); false if it is not usable (and then nothing is to be applied).
// `steps_now` is the receiver's own update count: only its low byte is shared, the rest stays.
[[nodiscard]] inline bool make_state(const SharedState& s, uint32_t steps_now, AttitudeEstimator::State& e, Controller::State& c) noexcept {
  std::array<float, 4> q{};
  float n2 = 0.0F;
  for (unsigned i = 0; i < 4U; ++i) {
    q[i] = from_word(s.w[kQuatFirst + i], kQuatLsb);
    n2 += q[i] * q[i];
  }
  if (!(n2 > 0.5F) || !(n2 < 2.0F)) {
    return false;
  }
  const float inv = 1.0F / std::sqrt(n2);
  for (unsigned i = 0; i < 4U; ++i) {
    e.q[i] = q[i] * inv;
  }
  for (unsigned i = 0; i < 3U; ++i) {
    e.bias[i] = from_word(s.w[kBiasFirst + i], kBiasLsb);
  }
  c.integ = {from_word(s.w[kCtrlFirst], kCtrlLsb), from_word(s.w[kCtrlFirst + 1U], kCtrlLsb)};
  c.out = {from_word(s.w[kCtrlFirst + 2U], kCtrlLsb), from_word(s.w[kCtrlFirst + 3U], kCtrlLsb)};
  const uint16_t meta = static_cast<uint16_t>(s.w[kMeta]);
  e.steps = (steps_now & 0xFFFFFF00U) | (meta & 0xFFU);
  e.aligned = (meta & 0x100U) != 0U;
  e.rates_valid = (meta & 0x200U) != 0U;
  return true;
}

// The frames of a shared state, and the schedule: the state is shared in the last frame of every period, after the command slot.
[[nodiscard]] inline std::array<Frame, kResyncChunks> pack_state(unsigned node, const SharedState& s, uint8_t seq) noexcept {
  std::array<Frame, kResyncChunks> out{};
  for (std::size_t k = 0; k < kResyncChunks; ++k) {
    const std::size_t i = 3U * k;
    out[k] = pack_resync(static_cast<uint8_t>(node), static_cast<uint8_t>(k), {s.w[i], s.w[i + 1U], s.w[i + 2U]}, seq);
  }
  return out;
}

[[nodiscard]] constexpr bool due(uint32_t frame, uint32_t period) noexcept { return period != 0U && (frame % period) == period - 1U; }

// What the receiver collects in one cycle: the chunks of the three computers that carry this frame's number.
class Collector {
 public:
  void begin(uint32_t frame) noexcept {
    frame_low_ = static_cast<uint8_t>(frame & 0xFFU);
    seen_ = {};
    words_ = {};
  }

  // True if the frame was a resync chunk of this cycle (valid CRC, this frame's number) and was kept.
  bool on_frame(const Frame& f) noexcept {
    const DecodedResync d = unpack_resync(f);
    if (!d.ok || d.seq != frame_low_) {
      return false;
    }
    for (unsigned i = 0; i < 3U; ++i) {
      words_[d.node][(3U * d.chunk) + i] = d.words[i];
    }
    seen_[d.node] = static_cast<uint8_t>(seen_[d.node] | (1U << d.chunk));
    return true;
  }

  // Nodes of which all four chunks arrived.
  [[nodiscard]] uint8_t complete() const noexcept {
    uint8_t m = 0U;
    for (unsigned n = 0; n < 3U; ++n) {
      m = static_cast<uint8_t>(m | (seen_[n] == 0x0FU ? (1U << n) : 0U));
    }
    return m;
  }

  [[nodiscard]] SharedState share(unsigned node) const noexcept {
    SharedState s;
    if (node < 3U) {
      s.w = words_[node];
    }
    return s;
  }

 private:
  uint8_t frame_low_ = 0U;
  std::array<uint8_t, 3> seen_{};
  std::array<std::array<int16_t, kWords>, 3> words_{};
};

// Tolerances in least significant bits of the words, per class.
struct Config {
  // Two computers' shares are averaged only if no word differs by more than this (otherwise nobody can say which is right, and nothing is adopted).
  uint16_t pair_quat = 330U;   // about 1 degree of attitude
  uint16_t pair_bias = 1200U;  // about 0.7 degree per second
  uint16_t pair_ctrl = 1000U;  // 1 degree
  // A computer's own state differing from the vote by more than this in any word is a large correction: it says the state has gone wrong, which a lost frame does not do.
  uint16_t large_quat = 150U;  // about 0.5 degree
  uint16_t large_bias = 500U;  // about 0.3 degree per second
  uint16_t large_ctrl = 500U;  // 0.5 degree
};

enum class Why : uint8_t {
  Adopted = 0,
  TooFew = 1,      // fewer than two voting computers
  Incomplete = 2,  // a voting computer's state did not arrive whole at this receiver: nothing is adopted (see vote)
  Disagree = 3     // two computers delivered states further apart than the pair tolerance, or states with different meta words
};

struct Outcome {
  bool adopted = false;
  Why why = Why::TooFew;
  uint8_t voters = 0U;   // the computers whose shares made the vote
  uint8_t changed = 0U;  // voters whose own state was not the vote, in any word
  uint8_t large = 0U;    // voters whose own state was far from the vote (Config::large_*)
  SharedState state{};   // the vote
};

[[nodiscard]] constexpr int16_t median3(int16_t a, int16_t b, int16_t c) noexcept {
  const int16_t lo = a < b ? a : b;
  const int16_t hi = a < b ? b : a;
  return c < lo ? lo : (c > hi ? hi : c);
}

[[nodiscard]] constexpr uint16_t pair_tol(const Config& cfg, unsigned word) noexcept {
  if (word < kBiasFirst) {
    return cfg.pair_quat;
  }
  return word < kCtrlFirst ? cfg.pair_bias : cfg.pair_ctrl;
}

[[nodiscard]] constexpr uint16_t large_tol(const Config& cfg, unsigned word) noexcept {
  if (word < kBiasFirst) {
    return cfg.large_quat;
  }
  return word < kCtrlFirst ? cfg.large_bias : cfg.large_ctrl;
}

[[nodiscard]] constexpr int32_t distance(int16_t a, int16_t b) noexcept {
  const int32_t d = static_cast<int32_t>(a) - static_cast<int32_t>(b);
  return d < 0 ? -d : d;
}

// The vote over the shares of the computers in `healthy`, all of which must have delivered a complete state (a receiver that missed a chunk adopts nothing: a vote over fewer states than
// the others take would put it in a state no other computer holds, which is the problem this module solves): three give the mid-value of each word; two give their mean if they agree
// within tolerance (a latched or restarted computer can then be brought back from the two that remain); fewer give nothing. The meta word (the step count's low byte and two flags) is voted as a whole:
// it is a count, and the mid-value of a count that wrapped is not a count.
[[nodiscard]] inline Outcome vote(const Collector& col, uint8_t healthy, const Config& cfg) noexcept {
  Outcome o;
  const uint8_t voters = static_cast<uint8_t>(col.complete() & healthy & 0x07U);
  o.voters = voters;
  std::array<unsigned, 3> idx{};
  unsigned n = 0U;
  for (unsigned i = 0; i < 3U; ++i) {
    if (((voters >> i) & 1U) != 0U) {
      idx[n++] = i;
    }
  }
  if (n < 2U) {
    o.why = Why::TooFew;
    return o;
  }
  if (voters != (healthy & 0x07U)) {
    o.why = Why::Incomplete;  // a vote over some of the computers is not the vote the others take: it would move this computer to a state no one else holds
    return o;
  }
  std::array<SharedState, 3> s{};
  for (unsigned k = 0; k < n; ++k) {
    s[k] = col.share(idx[k]);
  }
  for (unsigned wd = 0; wd < kWords; ++wd) {
    if (n == 3U) {
      o.state.w[wd] = median3(s[0].w[wd], s[1].w[wd], s[2].w[wd]);
    } else {
      const int32_t d = distance(s[0].w[wd], s[1].w[wd]);
      if ((wd == kMeta && d != 0) || (wd != kMeta && d > static_cast<int32_t>(pair_tol(cfg, wd)))) {
        o.why = Why::Disagree;
        return o;
      }
      o.state.w[wd] = static_cast<int16_t>((static_cast<int32_t>(s[0].w[wd]) + static_cast<int32_t>(s[1].w[wd])) / 2);  // exact: |sum| < 2^16
    }
  }
  for (unsigned k = 0; k < n; ++k) {
    bool changed = false;
    bool large = false;
    for (unsigned wd = 0; wd < kWords; ++wd) {
      const int32_t d = distance(s[k].w[wd], o.state.w[wd]);
      changed = changed || d != 0;
      large = large || (wd != kMeta && d > static_cast<int32_t>(large_tol(cfg, wd)));
    }
    o.changed = static_cast<uint8_t>(o.changed | (changed ? (1U << idx[k]) : 0U));
    o.large = static_cast<uint8_t>(o.large | (large ? (1U << idx[k]) : 0U));
  }
  o.adopted = true;
  o.why = Why::Adopted;
  return o;
}

}  // namespace tfc::resync
