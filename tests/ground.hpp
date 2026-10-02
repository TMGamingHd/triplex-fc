// SPDX-License-Identifier: MIT
// Helpers for tests that drive the manager with ground-command frames (host only).
#pragma once
#include "tfc/protocol.hpp"

namespace tfct {

// An authenticated command frame under the public bench key (what the default configuration expects).
inline tfc::Frame gcmd(tfc::GroundOp op, uint8_t node, uint8_t counter, bool arm = false) {
  return tfc::pack_ground_auth(op, node, counter, tfc::kBenchKey, arm);
}

// A frame whose tag is VALID for an arbitrary opcode byte (so the manager's own checks on the opcode and node are reached).
inline tfc::Frame gcmd_raw(uint8_t op_byte, uint8_t node, uint8_t counter) {
  tfc::Frame f = tfc::pack_ground(tfc::GroundOp::Reintegrate, node, counter);
  f.data[0] = op_byte;
  const uint32_t tag = tfc::ground_mac(tfc::kBenchKey, tfc::id::kGround, op_byte, node, counter);
  for (unsigned i = 0; i < 4U; ++i) f.data[2U + i] = static_cast<uint8_t>((tag >> (8U * i)) & 0xFFU);
  f.data[7] = tfc::crc8(f.data.data(), 7);
  return f;
}

}  // namespace tfct
