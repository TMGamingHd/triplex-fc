// SPDX-License-Identifier: MIT
// The platform driver and fault injector on the Pico 2 (docs/PICO.md; ADR-026). Every 10 ms:
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

#include "tfc/injector.hpp"
#include "tfc/pico_link.hpp"
#include "tfc/platform_driver.hpp"
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

constexpr uint16_t kPeriodMs = 10U;
constexpr unsigned kStatusEveryTicks = 2U;  // 50 Hz

const struct device* const g_uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
const struct pwm_dt_spec g_servo_x = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(servos), 0);
const struct pwm_dt_spec g_servo_y = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(servos), 1);
const struct gpio_dt_spec g_relays[tfc::kInjectorChannels] = {
    GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 0), GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 2), GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(relays), relay_gpios, 3)};

bool g_usb_ready = false;

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

// The servo pulse for a tilt, through the map of this axis.
bool set_servo(const struct pwm_dt_spec& spec, const tfc::ServoMap& map, float angle_deg) {
  const float us = map.pulse_us(angle_deg);
  return pwm_set_dt(&spec, PWM_USEC(20000U), PWM_USEC(static_cast<uint32_t>(us + 0.5F))) == 0;
}

void apply_relays(uint8_t energised) {
  for (unsigned i = 0; i < tfc::kInjectorChannels; ++i) {
    (void)gpio_pin_set_dt(&g_relays[i], ((energised >> i) & 1U) != 0U ? 1 : 0);  // the devicetree says active low
  }
}

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
    g_usb_ready = usbd_enable(usb) == 0;
  }
  if (!device_is_ready(g_uart) || !pwm_is_ready_dt(&g_servo_x) || !pwm_is_ready_dt(&g_servo_y)) {
    return 1;
  }

  tfc::PlatformConfig pcfg;
  pcfg.limit_deg = static_cast<float>(CONFIG_TFC_PLATFORM_LIMIT_DEG);
  pcfg.rate_limit_dps = static_cast<float>(CONFIG_TFC_PLATFORM_RATE_DPS);
  tfc::PlatformDriver platform(pcfg);
  tfc::InjectorLogic injector;
  tfc::ServoMap map_x;
  map_x.neutral_us = static_cast<float>(CONFIG_TFC_SERVO_NEUTRAL_US);
  map_x.us_per_deg = static_cast<float>(CONFIG_TFC_SERVO_US_PER_DEG_X100) * 0.01F;
  map_x.sign = static_cast<float>(CONFIG_TFC_SERVO_X_SIGN);
  map_x.trim_deg = static_cast<float>(CONFIG_TFC_SERVO_X_TRIM_X100) * 0.01F;
  tfc::ServoMap map_y = map_x;
  map_y.sign = static_cast<float>(CONFIG_TFC_SERVO_Y_SIGN);
  map_y.trim_deg = static_cast<float>(CONFIG_TFC_SERVO_Y_TRIM_X100) * 0.01F;

  // Level from the start: the servos are told neutral before the first command.
  (void)set_servo(g_servo_x, map_x, 0.0F);
  (void)set_servo(g_servo_y, map_y, 0.0F);

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

  tfc::pico::Parser parser;
  uint8_t last_seq = 0U;
  uint32_t quiet_ms = 0U;
  bool link_lost = false;
  bool ping_pending = false;
  unsigned tick = 0U;
  int64_t next = k_uptime_get() + kPeriodMs;

  for (;;) {
    // ---- the bytes from the PC ----
    uint8_t byte = 0U;
    bool heard = false;
    while (uart_poll_in(g_uart, &byte) == 0) {
      tfc::pico::Message msg;
      if (!parser.feed(byte, msg)) {
        continue;
      }
      heard = true;
      tfc::pico::PlatformCommand pc;
      tfc::pico::RelayCommand rc;
      if (tfc::pico::unpack_platform(msg, pc)) {
        if (platform.command(pc.tilt_x_deg, pc.tilt_y_deg)) {
          last_seq = pc.seq;
        }
      } else if (tfc::pico::unpack_relay(msg, rc)) {
        (void)injector.cut(rc.channel, rc.cut_ms);
      } else if (msg.type == tfc::pico::Type::Ping) {
        ping_pending = true;
      }
    }
    quiet_ms = heard ? 0U : (quiet_ms + kPeriodMs);
    if (quiet_ms >= static_cast<uint32_t>(CONFIG_TFC_LINK_TIMEOUT_MS)) {
      injector.release_all();  // a PC that went quiet leaves every node powered
      link_lost = true;
    } else if (heard) {
      link_lost = false;
    }

    // ---- the platform, the injector, the outputs ----
    platform.tick(kPeriodMs);
    injector.tick(kPeriodMs);
    (void)set_servo(g_servo_x, map_x, platform.output().x_deg);
    (void)set_servo(g_servo_y, map_y, platform.output().y_deg);
    apply_relays(injector.energised());

    // ---- the status back to the PC ----
    ++tick;
    if (ping_pending || tick % kStatusEveryTicks == 0U) {
      ping_pending = false;
      tfc::pico::Status st;
      st.seq_echo = last_seq;
      st.out_x_deg = platform.output().x_deg;
      st.out_y_deg = platform.output().y_deg;
      st.flags = static_cast<uint8_t>((platform.output().saturated ? tfc::pico::statusflag::kSaturated : 0U) | (platform.output().holding ? tfc::pico::statusflag::kHolding : 0U) |
                                      (platform.output().levelling ? tfc::pico::statusflag::kLevelling : 0U) | (link_lost ? tfc::pico::statusflag::kLinkLost : 0U) |
                                      (cause == tfc::ResetCause::Watchdog ? tfc::pico::statusflag::kWatchdogReset : 0U) |
                                      (platform.rejected() != 0U ? tfc::pico::statusflag::kRejected : 0U));
      st.relays = injector.energised();
      st.command_age_ms = platform.command_age_ms();
      std::array<uint8_t, tfc::pico::kMaxFrame> out{};
      const std::size_t n = tfc::pico::encode(tfc::pico::pack_status(st), out.data());
      (void)uart_fifo_fill(g_uart, out.data(), static_cast<int>(n));  // a short write is a lost status: the next one follows in 20 ms
    }

    if (wdt_channel >= 0) {
      (void)wdt_feed(wdt, wdt_channel);
    }
    const int64_t now = k_uptime_get();
    if (next > now) {
      k_msleep(static_cast<int32_t>(next - now));
    }
    next += kPeriodMs;
  }
}
