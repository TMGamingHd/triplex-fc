// SPDX-License-Identifier: MIT
// The fault injector's relay logic (ADR-026, docs/design/PICO.md): four channels, one power cut each for flight computers A, B, C and the actuator node. A channel is
// energised (the node cut) only for a time the PC names, and ends by itself; so a PC that crashes, a link that drops, or a Pico that is reset leaves every node powered.
//   cut(channel, ms)   energise the channel for `ms` (at most `max_cut_ms`); `ms` = 0 releases it; the PC's refresh restarts the time
//   link_lost()        release everything (the caller says so when nothing has come from the PC for the link timeout)
// `energised()` is the set of channels to drive; the caller turns it into active-low pin levels (a de-energised relay is a powered node: normally-closed contacts).
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cstdint>

namespace tfc {

constexpr unsigned kInjectorChannels = 4U;  // A, B, C, ACT

class InjectorLogic {
 public:
  static constexpr uint16_t kMaxCutMs = 30000U;

  // False if the channel does not exist.
  bool cut(unsigned channel, uint16_t cut_ms) noexcept {
    if (channel >= kInjectorChannels) {
      return false;
    }
    left_[channel] = cut_ms > kMaxCutMs ? kMaxCutMs : cut_ms;
    return true;
  }

  void release_all() noexcept { left_ = {}; }

  void tick(uint16_t dt_ms) noexcept {
    for (uint16_t& l : left_) {
      l = l > dt_ms ? static_cast<uint16_t>(l - dt_ms) : 0U;
    }
  }

  // Bit n set: channel n is energised (the node is cut).
  [[nodiscard]] uint8_t energised() const noexcept {
    uint8_t m = 0U;
    for (unsigned i = 0; i < kInjectorChannels; ++i) {
      m = static_cast<uint8_t>(m | (left_[i] > 0U ? (1U << i) : 0U));
    }
    return m;
  }

  [[nodiscard]] uint16_t remaining_ms(unsigned channel) const noexcept { return channel < kInjectorChannels ? left_[channel] : 0U; }

 private:
  std::array<uint16_t, kInjectorChannels> left_{};
};

}  // namespace tfc
