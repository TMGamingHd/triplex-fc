// SPDX-License-Identifier: MIT
// Simulated IMU for host runs: the same motion the virtual peers feel (sim/tfc_peers/peers.py,
// truth()) plus this node's own noise. On the target this file is replaced by the ISM330DHCX
// driver; nothing else in the app changes.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>

#include "tfc/protocol.hpp"

namespace fc::sim {

constexpr double kTau = 6.283185307179586;
constexpr uint32_t kFrameUs = 10000U;

struct Truth {
  std::array<double, 3> gyro{};   // dps
  std::array<double, 3> accel{};  // g
};

// Must stay operation-for-operation identical to peers.py so command and digest match bit for bit.
inline Truth truth(uint32_t frame) {
  const double t = static_cast<double>(frame) * static_cast<double>(kFrameUs) / 1e6 + 0.0005;
  Truth r;
  r.gyro = {10.0 * std::sin(kTau * 0.8 * t), 6.0 * std::cos(kTau * 0.5 * t), 3.0 * std::sin(kTau * 0.3 * t)};
  r.accel = {0.05 * std::sin(kTau * 0.4 * t), 0.03 * std::cos(kTau * 0.6 * t), 1.0};
  return r;
}

inline double clamp(double v, double lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

// Healthy replicas compute the same command from the same truth, so the commands and the
// estimator-state digest agree exactly (ADR-006).
inline tfc::Command command(const Truth& t, uint32_t frame) {
  tfc::Command c;
  const double pitch = clamp(0.1 * t.gyro[0], 30.0);
  const double yaw = clamp(0.1 * t.gyro[1], 30.0);
  c.pitch_deg = static_cast<float>(pitch);
  c.yaw_deg = static_cast<float>(yaw);
  const int32_t qp = tfc::quantize(c.pitch_deg, tfc::kCmdLsbDeg);
  const int32_t qy = tfc::quantize(c.yaw_deg, tfc::kCmdLsbDeg);
  const uint32_t mix = static_cast<uint32_t>(qp) * 31U + static_cast<uint32_t>(qy) * 17U + frame * 40503U;
  c.state_digest = static_cast<uint16_t>(mix & 0xFFFFU);
  return c;
}

// Deterministic per-node noise (same LCG as the SIL tests).
class Noise {
 public:
  explicit Noise(uint32_t seed) : s_(seed) {}
  float uniform() {  // [-1, 1)
    s_ = s_ * 1664525U + 1013904223U;
    return static_cast<float>(s_ >> 8) / 8388608.0F - 1.0F;
  }

 private:
  uint32_t s_;
};

}  // namespace fc::sim
