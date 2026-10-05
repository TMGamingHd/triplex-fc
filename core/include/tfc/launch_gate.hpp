// SPDX-License-Identifier: MIT
// Go or no-go for launch (docs/LAUNCH_SEQUENCE.md section 5): the list of conditions the sync master checks before it accepts a `launch`, and keeps checking during the countdown (a failure scrubs).
// It only decides; the caller supplies the facts and acts on the answer.
//   three healthy flight computers (Triplex), none latched or on probation
//   no Safe request
//   every computer ready (its heartbeat bit: IMU calibration, sensors, attitude), all three
//   ACT in Nominal (its status field in `0x300`)
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <cstdint>

namespace tfc {

struct LaunchFacts {
  uint8_t healthy_nodes = 0U;     // bit n: node n is healthy in the fault manager (voting)
  bool safe_requested = false;    // a sticky Safe request is up
  uint8_t ready_nodes = 0U;       // bit n: node n reports ready for launch
  bool act_nominal = false;       // ACT's output frame says Nominal
};

namespace nogo {
constexpr uint8_t kNotTriplex = 0x01U;
constexpr uint8_t kSafeRequested = 0x02U;
constexpr uint8_t kNodeNotReady = 0x04U;
constexpr uint8_t kActNotNominal = 0x08U;
}  // namespace nogo

// The failed conditions as a bit mask; 0 means go.
[[nodiscard]] constexpr uint8_t launch_check(const LaunchFacts& f) noexcept {
  uint8_t m = 0U;
  if ((f.healthy_nodes & 0x07U) != 0x07U) {
    m = static_cast<uint8_t>(m | nogo::kNotTriplex);
  }
  if (f.safe_requested) {
    m = static_cast<uint8_t>(m | nogo::kSafeRequested);
  }
  if ((f.ready_nodes & 0x07U) != 0x07U) {
    m = static_cast<uint8_t>(m | nogo::kNodeNotReady);
  }
  if (!f.act_nominal) {
    m = static_cast<uint8_t>(m | nogo::kActNotNominal);
  }
  return m;
}

// Text for the first failed condition ("go" if none), for the console.
[[nodiscard]] constexpr const char* nogo_text(uint8_t mask) noexcept {
  if ((mask & nogo::kNotTriplex) != 0U) {
    return "not three healthy flight computers";
  }
  if ((mask & nogo::kSafeRequested) != 0U) {
    return "a Safe request is up";
  }
  if ((mask & nogo::kNodeNotReady) != 0U) {
    return "a flight computer is not ready (IMU calibration, sensors or attitude)";
  }
  if ((mask & nogo::kActNotNominal) != 0U) {
    return "ACT is not Nominal";
  }
  return "go";
}

}  // namespace tfc
