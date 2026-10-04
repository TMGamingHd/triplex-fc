// SPDX-License-Identifier: MIT
// Hardware seams that every node of the system needs, shared by the flight-computer app (firmware/app) and the actuator app (firmware/act):
//   Watchdog       the independent hardware watchdog                      | nothing, where the board has none
//   reset_cause()  why the chip reset, from the hardware                  | power-on
// No virtual functions: the choice is made at compile time (tools/check_elf.sh forbids vtables in the flight binary).
// Include the core headers BEFORE this file (Zephyr defines a `__unused` macro that breaks a glibc header).
#pragma once
#include <cstdint>

#include "tfc/resetlog.hpp"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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
