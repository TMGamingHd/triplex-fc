// SPDX-License-Identifier: MIT
// Hardware seams that every node of the system needs, shared by the flight-computer app (firmware/app) and the actuator app (firmware/act):
//   Watchdog       the independent hardware watchdog                      | nothing, where the board has none
//   reset_cause()  why the chip reset, from the hardware                  | power-on
//   ActLines       the actuator node's FRAME and KICK out and SAFE in     | no lines: SAFE reads as not asserted
// No virtual functions: the choice is made at compile time (tools/check_elf.sh forbids vtables in the flight binary).
// Include the core headers BEFORE this file (Zephyr defines a `__unused` macro that breaks a glibc header).
#pragma once
#include <cstdint>

#include "tfc/resetlog.hpp"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

namespace fc::hw {

// ---------------------------------------------------------------- watchdog

#if DT_HAS_ALIAS(watchdog0) && defined(CONFIG_WATCHDOG)

// Started just before the first frame, serviced from one place only: the end of a completed frame (TFC-FDIR-038).
class Watchdog {
 public:
  bool start(uint32_t timeout_ms) {
    if (!device_is_ready(dev_)) {
      return false;
    }
    wdt_timeout_cfg cfg{};
    cfg.flags = WDT_FLAG_RESET_SOC;
    cfg.window.min = 0U;
    cfg.window.max = timeout_ms;
    channel_ = wdt_install_timeout(dev_, &cfg);
    if (channel_ < 0) {
      return false;
    }
    started_ = wdt_setup(dev_, WDT_OPT_PAUSE_HALTED_BY_DBG) == 0;
    return started_;
  }
  void feed() {
    if (started_) {
      (void)wdt_feed(dev_, channel_);
    }
  }
  [[nodiscard]] bool started() const { return started_; }

 private:
  const device* dev_ = DEVICE_DT_GET(DT_ALIAS(watchdog0));
  int channel_ = -1;
  bool started_ = false;
};

#else

class Watchdog {
 public:
  bool start(uint32_t) { return false; }
  void feed() {}
  [[nodiscard]] bool started() const { return false; }
};

#endif

// ---------------------------------------------------------------- the actuator node's lines to the supervisor

#if DT_NODE_EXISTS(DT_NODELABEL(tfc_act_lines))

class ActLines {
 public:
  void init() {
    configure(frame_, GPIO_OUTPUT_INACTIVE);
    configure(kick_, GPIO_OUTPUT_INACTIVE);
    configure(safe_, GPIO_INPUT);
  }
  void frame(bool on) { (void)gpio_pin_set_dt(&frame_, on ? 1 : 0); }
  void kick(bool on) { (void)gpio_pin_set_dt(&kick_, on ? 1 : 0); }
  // The supervisor's (or the FORCE-SAFE switch's) hardware Safe: high is Safe. A pin that cannot be read is "not asserted".
  [[nodiscard]] bool safe() const { return gpio_pin_get_dt(&safe_) > 0; }

 private:
  static void configure(const gpio_dt_spec& s, gpio_flags_t flags) {
    if (gpio_is_ready_dt(&s)) {
      (void)gpio_pin_configure_dt(&s, flags);
    }
  }
  gpio_dt_spec frame_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_act_lines), frame_gpios);
  gpio_dt_spec kick_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_act_lines), kick_gpios);
  gpio_dt_spec safe_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_act_lines), safe_gpios);
};

#else

class ActLines {
 public:
  void init() {}
  void frame(bool) {}
  void kick(bool) {}
  [[nodiscard]] bool safe() const { return false; }
};

#endif

// ---------------------------------------------------------------- why the chip reset

// Note: on the STM32 family a power-up reports a brown-out and a pin reset together. That is harmless here: after a power-up the
// record in RAM is garbage, fails its check, and is discarded as a fresh log whatever cause is reported.
inline tfc::ResetCause reset_cause() {
  uint32_t flags = 0U;
  if (hwinfo_get_reset_cause(&flags) != 0) {
    return tfc::ResetCause::Unknown;
  }
  (void)hwinfo_clear_reset_cause();
  if ((flags & RESET_POR) != 0U) {
    return tfc::ResetCause::PowerOn;
  }
  if ((flags & RESET_WATCHDOG) != 0U) {
    return tfc::ResetCause::Watchdog;
  }
  if ((flags & RESET_BROWNOUT) != 0U) {
    return tfc::ResetCause::Brownout;
  }
  if ((flags & RESET_CPU_LOCKUP) != 0U) {
    return tfc::ResetCause::Fault;
  }
  if ((flags & RESET_SOFTWARE) != 0U) {
    return tfc::ResetCause::Software;
  }
  if ((flags & RESET_PIN) != 0U) {
    return tfc::ResetCause::Pin;
  }
  return tfc::ResetCause::Unknown;
}

}  // namespace fc::hw
