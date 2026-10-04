// SPDX-License-Identifier: MIT
// Sensor consensus: the single set of gyro and accelerometer values every replica computes its attitude from. Each node
// broadcasts its own IMU sample (frames 0x100+n and 0x110+n); once the exchange slot is over, every computer votes the three
// samples per axis (mid-value select with a tolerance, `vote3`) and gets the same answer, so the estimators that follow
// start from identical inputs and stay bit-identical (ADR-006). The set of nodes whose sensors may be used is given each
// frame by the caller (the manager's view: a latched node is excluded), which is exactly the "channels with a valid flag"
// interface the sensor/compute health split of ADR-020 will drive.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cstdint>

#include "tfc/protocol.hpp"
#include "tfc/voter.hpp"

namespace tfc {

struct ConsensusInput {
  Vec3 gyro_dps;
  Vec3 accel_g;
  bool gyro_ok = false;     // the three gyro axes have a trustworthy value
  bool accel_ok = false;    // likewise for the accelerometer
  uint8_t gyro_nodes = 0U;  // bit n: node n's gyro sample took part
  uint8_t accel_nodes = 0U;
};

class SensorConsensus {
 public:
  SensorConsensus() noexcept = default;
  SensorConsensus(float gyro_tol_dps, float accel_tol_g) noexcept : gyro_tol_(gyro_tol_dps), accel_tol_(accel_tol_g) {}

  // Start of a frame: forget the samples, and say which nodes' sensors may be used (bit n = node n).
  void begin_frame(uint8_t usable_nodes) noexcept {
    usable_ = static_cast<uint8_t>(usable_nodes & kAllChannels);
    gyro_have_ = 0U;
    accel_have_ = 0U;
  }

  // Offer a frame. Returns true if it was a gyro or accelerometer frame of node 0..2 with a good CRC and was stored.
  bool on_frame(const Frame& f) noexcept {
    const uint32_t base = f.id & ~0x3U;
    const unsigned node = f.id & 0x3U;
    if (node >= kChannels || (base != id::kGyroBase && base != id::kAccelBase)) {
      return false;
    }
    const bool gyro = base == id::kGyroBase;
    const DecodedVec3 d = unpack_vec3(f, gyro ? kGyroLsbDps : kAccelLsbG);
    if (!d.ok) {
      return false;
    }
    (gyro ? gyro_ : accel_)[node] = d.x;
    uint8_t& have = gyro ? gyro_have_ : accel_have_;
    have = static_cast<uint8_t>(have | (1U << node));
    return true;
  }

  [[nodiscard]] ConsensusInput consensus() const noexcept {
    ConsensusInput out;
    out.gyro_nodes = static_cast<uint8_t>(usable_ & gyro_have_);
    out.accel_nodes = static_cast<uint8_t>(usable_ & accel_have_);
    out.gyro_ok = vote_axes(gyro_, out.gyro_nodes, gyro_tol_, out.gyro_dps);
    out.accel_ok = vote_axes(accel_, out.accel_nodes, accel_tol_, out.accel_g);
    return out;
  }

 private:
  // Votes the three axes. True only if every axis has a trustworthy value (Triplex, Duplex or Simplex); an unresolved
  // disagreement or no data is not a value, and the caller then does not use it.
  static bool vote_axes(const std::array<Vec3, kChannels>& s, uint8_t mask, float tol, Vec3& out) noexcept {
    bool ok = true;
    for (unsigned axis = 0; axis < 3U; ++axis) {
      const std::array<float, kChannels> x{s[0].v[axis], s[1].v[axis], s[2].v[axis]};
      const VoteResult r = vote3(x, mask, tol);
      const bool good = r.status == VoteStatus::Triplex || r.status == VoteStatus::Duplex || r.status == VoteStatus::Simplex;
      out.v[axis] = good ? r.value : 0.0F;
      ok = ok && good;
    }
    return ok;
  }

  float gyro_tol_ = 1.0F;
  float accel_tol_ = 0.02F;
  uint8_t usable_ = 0U;
  uint8_t gyro_have_ = 0U;
  uint8_t accel_have_ = 0U;
  std::array<Vec3, kChannels> gyro_{};
  std::array<Vec3, kChannels> accel_{};
};

}  // namespace tfc
