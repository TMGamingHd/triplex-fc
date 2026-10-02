// SPDX-License-Identifier: MIT
// Redundancy management: 3-channel mid-value-select voter with a validity mask.
// No heap, no exceptions, no RTTI. Deterministic: same inputs -> same outputs.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace tfc {

constexpr unsigned kChannels = 3;
constexpr uint8_t kAllChannels = 0x07U;

enum class VoteStatus : uint8_t {
  Triplex,           // 3 valid inputs, at least 2 agree within tolerance
  Duplex,            // 2 valid inputs that agree
  DuplexMiscompare,  // 2 valid inputs that disagree: cannot tell which is bad
  Simplex,           // 1 valid input: no cross-check possible
  NoMajority,        // 3 valid inputs but no two agree
  NoData             // nothing usable
};

struct VoteResult {
  float value = 0.0F;
  VoteStatus status = VoteStatus::NoData;
  uint8_t disagree_mask = 0U;  // bit i set => channel i deviates (or is non-finite)
};

// Number of set bits in the low three bits of a channel mask.
constexpr unsigned count_channels(uint8_t mask) noexcept {
  return ((mask & 1U) != 0U ? 1U : 0U) + ((mask & 2U) != 0U ? 1U : 0U) +
         ((mask & 4U) != 0U ? 1U : 0U);
}

// Median of three finite values; returns one of the inputs exactly (no arithmetic).
[[nodiscard]] constexpr float median3(float a, float b, float c) noexcept {
  const float lo_ab = a < b ? a : b;
  const float hi_ab = a < b ? b : a;
  const float mid = hi_ab < c ? hi_ab : c;  // min(max(a,b), c)
  return lo_ab > mid ? lo_ab : mid;         // max(min(a,b), mid)
}

namespace detail {
// Three valid values: the median is the voted value; anything farther than `tol` from it disagrees.
inline VoteResult vote_of_three(const std::array<float, kChannels>& x, float tol, uint8_t blamed) noexcept {
  VoteResult r;
  r.disagree_mask = blamed;
  r.value = median3(x[0], x[1], x[2]);  // one of the inputs exactly, no arithmetic
  unsigned agree = 0;
  for (unsigned i = 0; i < kChannels; ++i) {
    if (std::fabs(x[i] - r.value) > tol) {
      r.disagree_mask = static_cast<uint8_t>(r.disagree_mask | (1U << i));
    } else {
      ++agree;
    }
  }
  r.status = (agree >= 2U) ? VoteStatus::Triplex : VoteStatus::NoMajority;
  return r;
}

// Two valid values: they either agree (their mean is the value) or they cannot be told apart.
inline VoteResult vote_of_two(const std::array<float, kChannels>& x, uint8_t mask, float tol, uint8_t blamed) noexcept {
  VoteResult r;
  r.disagree_mask = blamed;
  unsigned first = 0U;
  unsigned second = 0U;
  bool have_first = false;
  for (unsigned i = 0; i < kChannels; ++i) {
    if (((mask >> i) & 1U) == 0U) {
      continue;
    }
    if (!have_first) {
      first = i;
      have_first = true;
    } else {
      second = i;
    }
  }
  if (std::fabs(x[first] - x[second]) <= tol) {
    r.value = 0.5F * (x[first] + x[second]);
    r.status = VoteStatus::Duplex;
  } else {
    r.value = 0.0F;
    r.status = VoteStatus::DuplexMiscompare;
    r.disagree_mask = static_cast<uint8_t>(r.disagree_mask | mask);
  }
  return r;
}
}  // namespace detail

// Votes one scalar. `valid_mask` marks channels that delivered a fresh, CRC-good,
// non-latched sample this frame. Non-finite values are treated as invalid and
// reported in `disagree_mask`.
[[nodiscard]] inline VoteResult vote3(const std::array<float, kChannels>& x, uint8_t valid_mask, float tol) noexcept {
  uint8_t mask = static_cast<uint8_t>(valid_mask & kAllChannels);
  uint8_t non_finite = 0U;
  for (unsigned i = 0; i < kChannels; ++i) {
    if (((mask >> i) & 1U) != 0U && !std::isfinite(x[i])) {
      mask = static_cast<uint8_t>(mask & ~(1U << i));
      non_finite = static_cast<uint8_t>(non_finite | (1U << i));
    }
  }
  VoteResult r;
  switch (count_channels(mask)) {
    case 3:
      r = detail::vote_of_three(x, tol, non_finite);
      break;
    case 2:
      r = detail::vote_of_two(x, mask, tol, non_finite);
      break;
    case 1:
      for (unsigned i = 0; i < kChannels; ++i) {
        if (((mask >> i) & 1U) != 0U) {
          r.value = x[i];
        }
      }
      r.status = VoteStatus::Simplex;
      r.disagree_mask = non_finite;
      break;
    default:
      r.status = VoteStatus::NoData;
      r.disagree_mask = non_finite;
      break;
  }
  return r;
}

}  // namespace tfc
