// SPDX-License-Identifier: MIT
// The supervisor on the Pico 2 (SUP-Lite, docs/SUPERVISOR.md; ADR-022). The board's side of sup::Supervisor (supervisor/include/sup/supervisor.hpp, tested on the host with a simulation of the
// units it watches). Every millisecond this loop
//   - reads the cumulative FRAME and KICK edge counts and the tick of the latest edge of each of the four units (the edges are counted in the GPIO interrupts),
//   - reads a command line if one has come in over USB serial (reset, cycle, hold, release, safe-now, safe-clear, launch, scrub, t0, status),
//   - gives both to the logic and applies the levels it returns: NRST (driven open-drain by hand), the power relays (active low, external pull-ups), SAFE and T0,
//   - answers the command and reports what happened (a reset, a power-cycle, a unit declared DEAD, a period or phase out of limit, T-zero),
//   - feeds the hardware watchdog.
// At power-up every output is at its default before anything else can go wrong: no reset, no relay, SAFE and T0 low. The record of T-zero is kept in the last flash sector, so that after a
// reset or a power cut the mission clock is resumed from the battery-backed RTC (docs/MISSION_CLOCK.md). It shares no code with core/ (TFC-SUP-001). None of this has been run on a board.
//
// Include the C++ standard library BEFORE Zephyr headers: Zephyr defines an `__unused` macro that breaks a glibc header.
#include <array>
#include <cstdint>
#include <cstring>

#include "sup/rtc.hpp"
#include "sup/supervisor.hpp"

#include "usb_init.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>

namespace {

constexpr unsigned kUnits = sup::kUnits;
const struct device* const g_uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
const struct device* const g_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
constexpr uint16_t kRtcAddress = 0x68;

#define LINE(prop, i) GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(sup_lines), prop, i)
const struct gpio_dt_spec g_frame[kUnits] = {LINE(frame_gpios, 0), LINE(frame_gpios, 1), LINE(frame_gpios, 2), LINE(frame_gpios, 3)};
const struct gpio_dt_spec g_kick[kUnits] = {LINE(kick_gpios, 0), LINE(kick_gpios, 1), LINE(kick_gpios, 2), LINE(kick_gpios, 3)};
const struct gpio_dt_spec g_nrst[kUnits] = {LINE(nrst_gpios, 0), LINE(nrst_gpios, 1), LINE(nrst_gpios, 2), LINE(nrst_gpios, 3)};
const struct gpio_dt_spec g_pwr[kUnits] = {LINE(pwr_gpios, 0), LINE(pwr_gpios, 1), LINE(pwr_gpios, 2), LINE(pwr_gpios, 3)};
const struct gpio_dt_spec g_safe = GPIO_DT_SPEC_GET(DT_NODELABEL(sup_lines), safe_gpios);
const struct gpio_dt_spec g_t0 = GPIO_DT_SPEC_GET(DT_NODELABEL(sup_lines), t0_gpios);

// The edge counters, written in the GPIO interrupt and read with interrupts off.
struct Edges {
  uint32_t frames = 0U;
  uint64_t frame_tick = 0U;
  uint32_t kicks = 0U;
  uint64_t kick_tick = 0U;
};
Edges g_edges[kUnits];

struct EdgeCallback {
  struct gpio_callback cb;
  unsigned unit;
  bool kick;
};
EdgeCallback g_callbacks[2U * kUnits];

uint64_t now_ticks() { return k_cyc_to_us_floor64(k_cycle_get_64()); }  // 1 MHz, as sup::SupConfig::tick_hz

void on_edge(const struct device*, struct gpio_callback* cb, uint32_t) {
  EdgeCallback* e = CONTAINER_OF(cb, EdgeCallback, cb);
  const uint64_t t = now_ticks();
  if (e->kick) {
    ++g_edges[e->unit].kicks;
    g_edges[e->unit].kick_tick = t;
  } else {
    ++g_edges[e->unit].frames;
    g_edges[e->unit].frame_tick = t;
  }
}

bool watch(const struct gpio_dt_spec& pin, unsigned index, unsigned unit, bool kick) {
  if (!gpio_is_ready_dt(&pin) || gpio_pin_configure_dt(&pin, GPIO_INPUT) != 0) {
    return false;
  }
  EdgeCallback& e = g_callbacks[index];
  e.unit = unit;
  e.kick = kick;
  gpio_init_callback(&e.cb, on_edge, BIT(pin.pin));
  return gpio_add_callback(pin.port, &e.cb) == 0 && gpio_pin_interrupt_configure_dt(&pin, GPIO_INT_EDGE_RISING) == 0;
}

// NRST: an output pulled low to reset, an input (high impedance) otherwise.
void set_nrst(unsigned u, bool low) {
  static bool last[kUnits];
  static bool known[kUnits];
  if (known[u] && last[u] == low) {
    return;
  }
  known[u] = true;
  last[u] = low;
  if (low) {
    (void)gpio_pin_configure_dt(&g_nrst[u], GPIO_OUTPUT_ACTIVE);
  } else {
    (void)gpio_pin_configure_dt(&g_nrst[u], GPIO_INPUT);
  }
}

void apply(const sup::Outputs& o) {
  for (unsigned u = 0; u < kUnits; ++u) {
    set_nrst(u, o.nrst_low[u]);
    (void)gpio_pin_set_dt(&g_pwr[u], o.power_cut[u] ? 1 : 0);  // the devicetree says active low: logical 1 is the relay energised
  }
  (void)gpio_pin_set_dt(&g_safe, o.safe_line ? 1 : 0);
  (void)gpio_pin_set_dt(&g_t0, o.t0_line ? 1 : 0);
}

void say(const char* text) {
  const std::size_t n = std::strlen(text);
  std::size_t sent = 0U;
  for (int tries = 0; tries < 4 && sent < n; ++tries) {  // a short write is a lost line: the operator asks again
    sent += static_cast<std::size_t>(uart_fifo_fill(g_uart, reinterpret_cast<const uint8_t*>(text) + sent, static_cast<int>(n - sent)));
  }
}

// The real-time clock: seconds since 2000-01-01, or invalid.
sup::RtcReading read_rtc() {
  std::array<uint8_t, 7> r{};
  uint8_t reg = 0x00;
  uint8_t status = 0x80;
  if (!device_is_ready(g_i2c) || i2c_write_read(g_i2c, kRtcAddress, &reg, 1, r.data(), static_cast<uint32_t>(r.size())) != 0) {
    return sup::RtcReading{};
  }
  reg = 0x0F;
  if (i2c_write_read(g_i2c, kRtcAddress, &reg, 1, &status, 1) != 0) {
    return sup::RtcReading{};
  }
  return sup::ds3231_seconds(r, status);
}

// The record of T-zero in the last flash sector.
bool load_record(sup::MetRecord& rec) {
  const struct flash_area* fa = nullptr;
  if (flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa) != 0) {
    return false;
  }
  const bool ok = flash_area_read(fa, 0, &rec, sizeof rec) == 0;
  flash_area_close(fa);
  return ok;
}

void store_record(const sup::MetRecord& rec) {
  const struct flash_area* fa = nullptr;
  if (flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa) != 0) {
    return;
  }
  if (flash_area_erase(fa, 0, 4096U) == 0) {
    (void)flash_area_write(fa, 0, &rec, sizeof rec);
  }
  flash_area_close(fa);
}

void usb_message(struct usbd_context* const ctx, const struct usbd_msg* msg) {
  if (usbd_can_detect_vbus(ctx)) {
    if (msg->type == USBD_MSG_VBUS_READY) {
      (void)usbd_enable(ctx);
    } else if (msg->type == USBD_MSG_VBUS_REMOVED) {
      (void)usbd_disable(ctx);
    }
  }
}

const char* const kUnitNames[kUnits] = {"A", "B", "C", "ACT"};

}  // namespace

int main() {
  // Every output at its default first: no relay energised, NRST released, SAFE and T0 low (TFC-SUP-007).
  for (unsigned u = 0; u < kUnits; ++u) {
    if (gpio_is_ready_dt(&g_pwr[u])) {
      (void)gpio_pin_configure_dt(&g_pwr[u], GPIO_OUTPUT_INACTIVE);
    }
    if (gpio_is_ready_dt(&g_nrst[u])) {
      (void)gpio_pin_configure_dt(&g_nrst[u], GPIO_INPUT);
    }
  }
  if (gpio_is_ready_dt(&g_safe)) {
    (void)gpio_pin_configure_dt(&g_safe, GPIO_OUTPUT_INACTIVE);
  }
  if (gpio_is_ready_dt(&g_t0)) {
    (void)gpio_pin_configure_dt(&g_t0, GPIO_OUTPUT_INACTIVE);
  }
  struct usbd_context* usb = tfc_usbd_init(usb_message);
  if (usb != nullptr && !usbd_can_detect_vbus(usb)) {
    (void)usbd_enable(usb);
  }
  bool lines_ok = device_is_ready(g_uart);
  for (unsigned u = 0; u < kUnits; ++u) {
    lines_ok = watch(g_frame[u], 2U * u, u, false) && watch(g_kick[u], (2U * u) + 1U, u, true) && lines_ok;
  }
  if (!lines_ok) {
    return 1;
  }

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

  sup::Supervisor supervisor;
  sup::RtcReading rtc = read_rtc();
  {
    sup::MetRecord rec;
    if (rtc.valid && load_record(rec) && supervisor.restore(rec, now_ticks(), rtc.seconds)) {
      say("T-zero record found: the mission clock is resumed from the RTC (good to a second)\n");
    }
  }
  say("supervisor up\n");

  std::array<char, 48> line{};
  std::size_t line_len = 0U;
  uint64_t next_rtc = now_ticks() + 1000000U;
  int64_t next = k_uptime_get() + 1;
  for (;;) {
    sup::Inputs in;
    const unsigned key = irq_lock();
    for (unsigned u = 0; u < kUnits; ++u) {
      in.unit[u].frames = g_edges[u].frames;
      in.unit[u].frame_tick = g_edges[u].frame_tick;
      in.unit[u].kicks = g_edges[u].kicks;
      in.unit[u].kick_tick = g_edges[u].kick_tick;
    }
    irq_unlock(key);
    in.ticks = now_ticks();
    if (in.ticks >= next_rtc) {  // the RTC once a second
      next_rtc = in.ticks + 1000000U;
      rtc = read_rtc();
    }
    in.rtc_s = rtc.valid ? rtc.seconds : 0U;

    sup::Command cmd;
    bool have_cmd = false;
    uint8_t ch = 0U;
    for (unsigned i = 0; i < 16U && !have_cmd && uart_poll_in(g_uart, &ch) == 0; ++i) {  // at most 16 characters per millisecond
      if (ch == '\n' || ch == '\r') {
        if (line_len > 0U) {
          cmd = sup::parse_command(line.data(), line_len);
          have_cmd = true;
          line_len = 0U;
        }
      } else if (line_len < line.size()) {
        line[line_len++] = static_cast<char>(ch);
      } else {
        line_len = line.size() + 1U;  // too long: parse_command will refuse it when the line ends
      }
    }
    if (line_len > line.size()) {
      line_len = 0U;
      cmd = sup::Command{};
      cmd.parse = sup::Parse::TooLong;
      have_cmd = true;
    }

    sup::Events ev;
    sup::Response resp = sup::Response::Done;
    const sup::Outputs out = supervisor.step(in, have_cmd ? &cmd : nullptr, ev, resp);
    apply(out);

    if (have_cmd) {
      if (resp == sup::Response::Done && cmd.kind == sup::Kind::Status) {
        std::array<char, 400> text{};
        (void)supervisor.status_text(text.data(), text.size());
        say(text.data());
      } else {
        say(resp == sup::Response::Done ? "ok\n" : (resp == sup::Response::Refused ? "refused\n" : "unknown command\n"));
      }
    }
    for (unsigned u = 0; u < kUnits; ++u) {
      std::array<char, 64> msg{};
      if (ev.reset[u]) {
        (void)snprintk(msg.data(), msg.size(), "RESET %s (kicks stopped)\n", kUnitNames[u]);
        say(msg.data());
      }
      if (ev.cycle[u]) {
        (void)snprintk(msg.data(), msg.size(), "POWER-CYCLE %s\n", kUnitNames[u]);
        say(msg.data());
      }
      if (ev.dead[u]) {
        (void)snprintk(msg.data(), msg.size(), "DEAD %s: held in reset; `release %s` to try again\n", kUnitNames[u], kUnitNames[u]);
        say(msg.data());
      }
      if (ev.period_bad[u]) {
        (void)snprintk(msg.data(), msg.size(), "PERIOD %s out of limit (%d ppm)\n", kUnitNames[u], static_cast<int>(supervisor.period_ppm(static_cast<sup::Unit>(u))));
        say(msg.data());
      }
      if (ev.mission_bad[u]) {
        (void)snprintk(msg.data(), msg.size(), "MISSION TIME %s disagrees with the clock\n", kUnitNames[u]);
        say(msg.data());
      }
    }
    if (ev.warm_start) {
      say("warm start: units are running, nothing is touched\n");
    }
    if (ev.cold_start) {
      say("cold start: every unit held in reset, released in order ACT, A, B, C\n");
    }
    if (ev.phase_bad) {
      say("PHASE: two flight computers' frame starts are more than 200 us apart\n");
    }
    if (ev.total_loss) {
      say("TOTAL LOSS: no flight computer is kicking\n");
    }
    if (ev.countdown_started) {
      say("countdown: T-10 s\n");
    }
    if (ev.scrubbed) {
      say("countdown scrubbed\n");
    }
    if (ev.t0) {
      say("T-ZERO\n");
      store_record(ev.record);
    }

    if (wdt_channel >= 0) {
      (void)wdt_feed(wdt, wdt_channel);
    }
    const int64_t now = k_uptime_get();
    if (next > now) {
      k_msleep(static_cast<int32_t>(next - now));
    }
    ++next;
  }
}
