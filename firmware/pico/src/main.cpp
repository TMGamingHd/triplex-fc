// SPDX-License-Identifier: MIT
// The platform driver and fault injector on the Pico 2 (docs/design/PICO.md; ADR-026): the board's side of tfc::PicoApp (core/include/tfc/pico_app.hpp, tested on the host with a fake
// board). Every 10 ms the loop in the core:
//   - the bytes from the PC (USB serial) go through the link parser: a platform command goes to the platform driver, a relay command to the injector, a ping asks for a
//     status; a frame that does not check is dropped;
//   - the platform driver applies its limit, rate limit and timeout (TFC-PLAT-001, 002, 004) and the servo pulses follow: two PWM channels at 50 Hz;
//   - the injector's cuts run down by themselves, and if nothing has come from the PC for the link timeout every relay is released;
//   - a status (the output, the flags, the relays, the command age) goes back to the PC;
//   - the hardware watchdog is fed.
// The relays are active low with external pull-ups: a pin that is not driven (reset, boot, a crash) is a relay off, which is a powered node. Up to the first command the
// servos are held at level. None of this has been run on a board yet.
//
// Include core/ (and so the C++ standard library) BEFORE Zephyr headers: Zephyr defines an `__unused` macro that breaks a glibc header.
#include <array>
#include <cstdint>

#include "tfc/pico_app.hpp"
#include "tfc/resetlog.hpp"

#include "usb_init.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>

namespace {

const struct device* const g_uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
const struct pwm_dt_spec g_servo[2] = {PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(servos), 0), PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(servos), 1)};
const struct gpio_dt_spec g_relays[tfc::kInjectorChannels] = {
    GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 2), GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 3)};

void usb_message(struct usbd_context* const ctx, const struct usbd_msg* msg) {
  if (usbd_can_detect_vbus(ctx)) {
    if (msg->type == USBD_MSG_VBUS_READY) {
      (void)usbd_enable(ctx);
    } else if (msg->type == USBD_MSG_VBUS_REMOVED) {
      (void)usbd_disable(ctx);
    }
  }
}

tfc::ResetCause reset_cause() {
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
  return tfc::ResetCause::Unknown;
}

// The board behind tfc::PicoApp's interface.
class Board {
 public:
  Board(const struct device* wdt, int wdt_channel) : wdt_(wdt), wdt_channel_(wdt_channel) {}

  bool read_byte(uint8_t& b) { return uart_poll_in(g_uart, &b) == 0; }
  void write(const uint8_t* data, std::size_t n) {
    (void)uart_fifo_fill(g_uart, data, static_cast<int>(n));  // a short write is a lost status: the next one follows in 20 ms
  }
  void set_servo_us(unsigned axis, float us) { (void)pwm_set_dt(&g_servo[axis], PWM_USEC(20000U), PWM_USEC(static_cast<uint32_t>(us + 0.5F))); }
  void set_relays(uint8_t energised) {
    for (unsigned i = 0; i < tfc::kInjectorChannels; ++i) {
      (void)gpio_pin_set_dt(&g_relays[i], ((energised >> i) & 1U) != 0U ? 1 : 0);  // the devicetree says active low
    }
  }
  void feed_watchdog() {
    if (wdt_channel_ >= 0) {
      (void)wdt_feed(wdt_, wdt_channel_);
    }
  }

 private:
  const struct device* wdt_;
  int wdt_channel_;
};

}  // namespace

int main() {
  // The relays first: every output is configured "inactive" (the pin high, the relay off, the node powered) before anything else can go wrong.
  for (const struct gpio_dt_spec& r : g_relays) {
    if (gpio_is_ready_dt(&r)) {
      (void)gpio_pin_configure_dt(&r, GPIO_OUTPUT_INACTIVE);
    }
  }
  const tfc::ResetCause cause = reset_cause();
  struct usbd_context* usb = tfc_usbd_init(usb_message);
  if (usb != nullptr && !usbd_can_detect_vbus(usb)) {
    (void)usbd_enable(usb);
  }
  if (!device_is_ready(g_uart) || !pwm_is_ready_dt(&g_servo[0]) || !pwm_is_ready_dt(&g_servo[1])) {
    return 1;
  }

  tfc::PicoAppConfig cfg;
  cfg.platform.limit_deg = static_cast<float>(CONFIG_TFC_PLATFORM_LIMIT_DEG);
  cfg.platform.rate_limit_dps = static_cast<float>(CONFIG_TFC_PLATFORM_RATE_DPS);
  cfg.link_timeout_ms = static_cast<uint32_t>(CONFIG_TFC_LINK_TIMEOUT_MS);
  cfg.after_watchdog_reset = cause == tfc::ResetCause::Watchdog;
  cfg.servo_x.neutral_us = static_cast<float>(CONFIG_TFC_SERVO_NEUTRAL_US);
  cfg.servo_x.us_per_deg = static_cast<float>(CONFIG_TFC_SERVO_US_PER_DEG_X100) * 0.01F;
  cfg.servo_x.sign = static_cast<float>(CONFIG_TFC_SERVO_X_SIGN);
  cfg.servo_x.trim_deg = static_cast<float>(CONFIG_TFC_SERVO_X_TRIM_X100) * 0.01F;
  cfg.servo_y = cfg.servo_x;
  cfg.servo_y.sign = static_cast<float>(CONFIG_TFC_SERVO_Y_SIGN);
  cfg.servo_y.trim_deg = static_cast<float>(CONFIG_TFC_SERVO_Y_TRIM_X100) * 0.01F;

  const struct device* const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
  int wdt_channel = -1;
  if (device_is_ready(wdt)) {
    wdt_timeout_cfg wcfg{};
    wcfg.flags = WDT_FLAG_RESET_SOC;
    wcfg.window.min = 0U;
    wcfg.window.max = CONFIG_TFC_WATCHDOG_TIMEOUT_MS;
    wdt_channel = wdt_install_timeout(wdt, &wcfg);
    if (wdt_channel >= 0 && wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) != 0) {
      wdt_channel = -1;
    }
  }

  Board board(wdt, wdt_channel);
  tfc::PicoApp<Board> app(board, cfg);
  app.boot();  // level, and every node powered, before the first byte is read

  int64_t next = k_uptime_get() + cfg.period_ms;
  for (;;) {
    app.step();
    const int64_t now = k_uptime_get();
    if (next > now) {
      k_msleep(static_cast<int32_t>(next - now));
    }
    next += cfg.period_ms;
  }
}
