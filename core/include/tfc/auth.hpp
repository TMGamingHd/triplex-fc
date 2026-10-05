// SPDX-License-Identifier: MIT
// Message authentication for ground commands: SipHash-2-4 (Aumasson and Bernstein, 2012), a keyed 64-bit PRF designed
// as a MAC for short messages. Pure functions, no state, no heap, fixed loop bounds. A command frame carries the low 32
// bits of it (ADR-019). The key below is a PUBLIC bench key: it exists so the host tests, the virtual peers and the
// firmware agree out of the box. A real uplink must provision its own (docs/verification/FAULT_CAMPAIGN.md E10, docs/design/FUTURE_WORK.md).
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace tfc {

using AuthKey = std::array<uint8_t, 16>;

// The bench key 00 01 02 ... 0f: the key of the SipHash reference test vectors. Public; never use it for anything real.
inline constexpr AuthKey kBenchKey = {{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}};

namespace detail {
[[nodiscard]] constexpr uint64_t rotl64(uint64_t x, unsigned b) noexcept { return (x << b) | (x >> (64U - b)); }

[[nodiscard]] constexpr uint64_t load_le64(const uint8_t* p, std::size_t n) noexcept {
  uint64_t v = 0U;
  for (std::size_t i = 0; i < n && i < 8U; ++i) {
    v |= static_cast<uint64_t>(p[i]) << (8U * i);
  }
  return v;
}

struct SipState {
  uint64_t v0, v1, v2, v3;
};

constexpr void sip_round(SipState& s) noexcept {
  s.v0 += s.v1;
  s.v1 = rotl64(s.v1, 13U);
  s.v1 ^= s.v0;
  s.v0 = rotl64(s.v0, 32U);
  s.v2 += s.v3;
  s.v3 = rotl64(s.v3, 16U);
  s.v3 ^= s.v2;
  s.v0 += s.v3;
  s.v3 = rotl64(s.v3, 21U);
  s.v3 ^= s.v0;
  s.v2 += s.v1;
  s.v1 = rotl64(s.v1, 17U);
  s.v1 ^= s.v2;
  s.v2 = rotl64(s.v2, 32U);
}
}  // namespace detail

// SipHash-2-4 of `len` bytes. `len` is the caller's buffer size (at most a few bytes in this project).
[[nodiscard]] constexpr uint64_t siphash24(const AuthKey& key, const uint8_t* data, std::size_t len) noexcept {
  const uint64_t k0 = detail::load_le64(key.data(), 8U);
  const uint64_t k1 = detail::load_le64(key.data() + 8, 8U);
  detail::SipState s{k0 ^ 0x736f6d6570736575ULL, k1 ^ 0x646f72616e646f6dULL, k0 ^ 0x6c7967656e657261ULL, k1 ^ 0x7465646279746573ULL};
  const std::size_t words = len / 8U;
  for (std::size_t w = 0; w < words; ++w) {
    const uint64_t m = detail::load_le64(data + 8U * w, 8U);
    s.v3 ^= m;
    detail::sip_round(s);
    detail::sip_round(s);
    s.v0 ^= m;
  }
  const uint64_t tail = detail::load_le64(data + 8U * words, len - 8U * words);
  const uint64_t b = (static_cast<uint64_t>(len) << 56U) | tail;
  s.v3 ^= b;
  detail::sip_round(s);
  detail::sip_round(s);
  s.v0 ^= b;
  s.v2 ^= 0xFFU;
  for (unsigned i = 0; i < 4U; ++i) {
    detail::sip_round(s);
  }
  return s.v0 ^ s.v1 ^ s.v2 ^ s.v3;
}

// The 32-bit tag of a ground command: the low 32 bits of SipHash-2-4 over (frame id high byte, frame id low byte,
// opcode byte, node, counter). The id binds the tag to this frame type; the counter binds it to one command.
[[nodiscard]] constexpr uint32_t ground_mac(const AuthKey& key, uint32_t can_id, uint8_t op_byte, uint8_t node, uint8_t counter) noexcept {
  const std::array<uint8_t, 5> msg = {{static_cast<uint8_t>((can_id >> 8U) & 0xFFU), static_cast<uint8_t>(can_id & 0xFFU), op_byte, node, counter}};
  return static_cast<uint32_t>(siphash24(key, msg.data(), msg.size()) & 0xFFFFFFFFULL);
}

}  // namespace tfc
