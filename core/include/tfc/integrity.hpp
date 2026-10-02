// SPDX-License-Identifier: MIT
// Protection of the manager's own state against single-event upsets and logic errors.
//
// A flight computer's RAM is not immune to radiation (a cosmic ray or a heavy ion can flip a bit) and a
// redundancy manager that silently believes a flipped "node is healthy" or "no Safe request" flag is worse than
// having no manager. NASA-STD-8719.13 and NPR 7150.2 ask for exactly this kind of self-protection: critical
// state is stored redundantly, checked every cycle, and on a mismatch the software falls back to the safe side
// and says so, instead of continuing on corrupted data.
//
//  * GuardedByte stores a byte and its bitwise complement. A single flipped bit in either copy makes them
//    disagree, which is detected on the next scrub. (Two flips that happen to cancel are not detected; that is
//    far less likely and is what the periodic scrub interval is for.)
//  * ensure() is the project's runtime assertion. A flight computer must keep flying, so it never aborts: it
//    counts the failure and returns false so the caller can take a safe fall-back.
#pragma once
#include <cstdint>

namespace tfc {

struct ManagerTestAccess;  // fault injection in the host tests only (tests/test_recovery.cpp)

class GuardedByte {
 public:
  constexpr GuardedByte() noexcept = default;
  constexpr explicit GuardedByte(uint8_t v) noexcept : v_(v), n_(static_cast<uint8_t>(~v)) {}

  [[nodiscard]] constexpr uint8_t get() const noexcept { return v_; }
  [[nodiscard]] constexpr bool intact() const noexcept { return static_cast<uint8_t>(~v_) == n_; }
  constexpr void set(uint8_t v) noexcept {
    v_ = v;
    n_ = static_cast<uint8_t>(~v);
  }

 private:
  friend struct ManagerTestAccess;
  uint8_t v_ = 0U;
  uint8_t n_ = 0xFFU;
};

// Runtime assertion with safe recovery: true if `cond` holds; otherwise counts the failure (saturating) and
// returns false. Used for invariants that can only fail through memory corruption or a coding error.
[[nodiscard]] inline bool ensure(bool cond, uint32_t& failures) noexcept {
  if (!cond && failures < 0xFFFFFFFFU) {
    ++failures;
  }
  return cond;
}

}  // namespace tfc
