// SPDX-License-Identifier: MIT
// CRC-8/SAE-J1850: poly 0x1D, init 0xFF, xorout 0xFF, no reflection.
// Check value for ASCII "123456789" is 0x4B (asserted in tests).
#pragma once
#include <cstddef>
#include <cstdint>

namespace tfc {

constexpr uint8_t crc8(const uint8_t* data, std::size_t len) noexcept {
  uint8_t crc = 0xFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc = static_cast<uint8_t>(crc ^ data[i]);
    for (int b = 0; b < 8; ++b) {
      crc = static_cast<uint8_t>((crc & 0x80U) != 0U ? ((crc << 1) ^ 0x1DU) : (crc << 1));
    }
  }
  return static_cast<uint8_t>(crc ^ 0xFFU);
}

}  // namespace tfc
