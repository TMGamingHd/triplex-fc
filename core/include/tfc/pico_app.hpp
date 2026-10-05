// SPDX-License-Identifier: MIT
// The Pico's frame loop (docs/PICO.md): everything `firmware/pico/src/main.cpp` does every period, with the hardware behind a small interface so that the same loop runs on the
// board and under a fake in the host tests. `Hal` provides (no virtual functions: the choice is made at compile time)
//   bool read_byte(uint8_t&)                     the next byte from the PC, if there is one
//   void write(const uint8_t*, std::size_t)      bytes to the PC
//   void set_servo_us(unsigned axis, float us)   the pulse of servo 0 (tilt about X) or 1 (about Y)
//   void set_relays(uint8_t energised)           bit n set: channel n energised (the caller turns it into active-low pin levels)
//   void feed_watchdog()
// One `step()` is one period (10 ms): read what has come, apply it, advance the platform driver and the injector, set the outputs, answer, feed the watchdog. `boot()` sets the safe
// outputs before anything else: the servos level and every relay off.
// No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "tfc/injector.hpp"
#include "tfc/pico_link.hpp"
#include "tfc/platform_driver.hpp"

namespace tfc {

// The most bytes one step reads (a bound on the loop, as every loop in the core has one). USB full speed carries about 1.2 kbyte in 10 ms; a 100 Hz platform stream and its status are 20 bytes,
// so this is a flood limit: bytes beyond it wait for the next step.
constexpr std::size_t kMaxBytesPerStep = 256U;

struct PicoAppConfig {
  PlatformConfig platform{};
  ServoMap servo_x{};
  ServoMap servo_y{};
  uint16_t period_ms = 10U;
  uint32_t link_timeout_ms = 2000U;  // nothing from the PC for this long: every relay is released
  uint8_t status_every_steps = 2U;   // 50 Hz at a 10 ms period
  bool after_watchdog_reset = false;  // the reason for this boot, reported in the status
};

template <typename Hal>
class PicoApp {
 public:
  PicoApp(Hal& hal, const PicoAppConfig& cfg) noexcept : hal_(hal), cfg_(cfg), platform_(cfg.platform) {}

  // The safe state, before anything else can go wrong: level, and every node powered.
  void boot() noexcept {
    hal_.set_relays(0U);
    hal_.set_servo_us(0U, cfg_.servo_x.pulse_us(0.0F));
    hal_.set_servo_us(1U, cfg_.servo_y.pulse_us(0.0F));
  }

  void step() noexcept {
    // ---- the bytes from the PC ----
    uint8_t byte = 0U;
    bool heard = false;
    bool ping = false;
    for (std::size_t i = 0; i < kMaxBytesPerStep && hal_.read_byte(byte); ++i) {
      pico::Message msg;
      if (!parser_.feed(byte, msg)) {
        continue;
      }
      heard = true;
      pico::PlatformCommand pc;
      pico::RelayCommand rc;
      if (pico::unpack_platform(msg, pc)) {
        if (platform_.command(pc.tilt_x_deg, pc.tilt_y_deg)) {
          last_seq_ = pc.seq;
        }
      } else if (pico::unpack_relay(msg, rc)) {
        (void)injector_.cut(rc.channel, rc.cut_ms);
      } else if (msg.type == pico::Type::Ping) {
        ping = true;
      }
    }
    quiet_ms_ = heard ? 0U : (quiet_ms_ + cfg_.period_ms);
    if (quiet_ms_ >= cfg_.link_timeout_ms) {
      injector_.release_all();  // a PC that went quiet leaves every node powered
      link_lost_ = true;
    } else if (heard) {
      link_lost_ = false;
    }

    // ---- the platform, the injector, the outputs ----
    platform_.tick(cfg_.period_ms);
    injector_.tick(cfg_.period_ms);
    hal_.set_servo_us(0U, cfg_.servo_x.pulse_us(platform_.output().x_deg));
    hal_.set_servo_us(1U, cfg_.servo_y.pulse_us(platform_.output().y_deg));
    hal_.set_relays(injector_.energised());

    // ---- the status back to the PC ----
    ++steps_;
    if (ping || (cfg_.status_every_steps != 0U && steps_ % cfg_.status_every_steps == 0U)) {
      pico::Status st;
      st.seq_echo = last_seq_;
      st.out_x_deg = platform_.output().x_deg;
      st.out_y_deg = platform_.output().y_deg;
      st.flags = flags();
      st.relays = injector_.energised();
      st.command_age_ms = platform_.command_age_ms();
      std::array<uint8_t, pico::kMaxFrame> out{};
      const std::size_t n = pico::encode(pico::pack_status(st), out.data());
      hal_.write(out.data(), n);
    }
    hal_.feed_watchdog();
  }

  [[nodiscard]] const PlatformDriver& platform() const noexcept { return platform_; }
  [[nodiscard]] const InjectorLogic& injector() const noexcept { return injector_; }
  [[nodiscard]] const pico::Parser& parser() const noexcept { return parser_; }
  [[nodiscard]] bool link_lost() const noexcept { return link_lost_; }

 private:
  [[nodiscard]] uint8_t flags() const noexcept {
    uint8_t f = 0U;
    f = static_cast<uint8_t>(f | (platform_.output().saturated ? pico::statusflag::kSaturated : 0U));
    f = static_cast<uint8_t>(f | (platform_.output().holding ? pico::statusflag::kHolding : 0U));
    f = static_cast<uint8_t>(f | (platform_.output().levelling ? pico::statusflag::kLevelling : 0U));
    f = static_cast<uint8_t>(f | (link_lost_ ? pico::statusflag::kLinkLost : 0U));
    f = static_cast<uint8_t>(f | (cfg_.after_watchdog_reset ? pico::statusflag::kWatchdogReset : 0U));
    f = static_cast<uint8_t>(f | (platform_.rejected() != 0U ? pico::statusflag::kRejected : 0U));
    return f;
  }

  Hal& hal_;
  PicoAppConfig cfg_;
  PlatformDriver platform_;
  InjectorLogic injector_{};
  pico::Parser parser_{};
  uint8_t last_seq_ = 0U;
  uint32_t quiet_ms_ = 0U;
  bool link_lost_ = false;
  uint32_t steps_ = 0U;
};

}  // namespace tfc
