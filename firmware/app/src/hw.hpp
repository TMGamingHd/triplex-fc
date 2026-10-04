// SPDX-License-Identifier: MIT
// The seams between the frame loop and the hardware. Each one has a real implementation (chosen by what the board's
// devicetree provides) and a stand-in for `native_sim`, so main.cpp is the same on both and `core/` never sees a driver:
//   Imu       the ISM330DHCX over SPI (core/include/tfc/ism330dhcx.hpp)   | the simulated IMU (sim_imu.hpp)
//   Lines     FRAME and KICK pulses to the supervisor, the node-id straps | nothing
//   Watchdog  the independent hardware watchdog                           | nothing
//   reset_cause()  why the chip reset, from the hardware                   | power-on
// No virtual functions: the choice is made at compile time, as the flight binary may contain no vtables (tools/check_elf.sh).
// Include the core headers BEFORE this file (Zephyr defines a `__unused` macro that breaks a glibc header).
#pragma once
#include <array>
#include <cstdint>

#include "sim_imu.hpp"
#include "tfc/ism330dhcx.hpp"
#include "tfc/protocol.hpp"
#include "tfc/resetlog.hpp"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

namespace fc::hw {

// ---------------------------------------------------------------- IMU

// The simulated IMU: the same motion the virtual peers feel plus this node's own noise.
class SimImu {
 public:
  explicit SimImu(unsigned node) : noise_(0x1234U + node) {}
  bool init() { return true; }
  bool sample(uint32_t frame, tfc::Vec3& gyro, tfc::Vec3& accel) {
    const fc::sim::Truth tr = fc::sim::truth(frame);
    for (unsigned i = 0; i < 3U; ++i) {
      gyro.v[i] = static_cast<float>(tr.gyro[i]) + 0.17F * noise_.uniform();
      accel.v[i] = static_cast<float>(tr.accel[i]) + 0.0035F * noise_.uniform();
    }
    return true;
  }
  [[nodiscard]] uint32_t stale() const { return 0U; }
  [[nodiscard]] uint32_t errors() const { return 0U; }

 private:
  fc::sim::Noise noise_;
};

#if DT_NODE_EXISTS(DT_NODELABEL(imu_home))

// Raw SPI to the part. The chip uses SPI mode 3, as in Zephyr's own driver for it.
class ZephyrSpi {
 public:
  bool ready() const { return device_is_ready(spec().bus); }
  bool transfer(const uint8_t* tx, uint8_t* rx, uint8_t len) {
    const spi_buf txb{const_cast<uint8_t*>(tx), len};
    const spi_buf rxb{rx, len};
    const spi_buf_set txs{&txb, 1U};
    const spi_buf_set rxs{&rxb, 1U};
    return spi_transceive_dt(&spec(), &txs, &rxs) == 0;
  }
  void delay_us(uint32_t us) { k_usleep(static_cast<int32_t>(us)); }

 private:
  static const spi_dt_spec& spec() {
    static const spi_dt_spec s =
        SPI_DT_SPEC_GET(DT_NODELABEL(imu_home), SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA);
    return s;
  }
};

// The ISM330DHCX: configured once, then read once per frame in a single burst.
class Ism330Imu {
 public:
  explicit Ism330Imu(unsigned) : regs_(spi_), chip_(regs_, tfc::ism::Config{}) {}

  bool init() {
    if (!spi_.ready()) {
      printk("IMU: SPI bus not ready\n");
      return false;
    }
    const tfc::ism::Status s = chip_.init();
    if (s != tfc::ism::Status::Ok) {
      printk("IMU: init failed, status %u (1 bus, 2 wrong id, 3 reset timeout, 4 config mismatch)\n", static_cast<unsigned>(s));
    }
    return s == tfc::ism::Status::Ok;
  }

  // False if the bus failed: the caller then sends nothing for this frame, which peers see as a missing sample.
  bool sample(uint32_t, tfc::Vec3& gyro, tfc::Vec3& accel) {
    tfc::ism::RawSample raw;
    if (chip_.read(raw) != tfc::ism::Status::Ok) {
      ++errors_;
      return false;
    }
    if (!raw.gyro_new || !raw.accel_new) {
      ++stale_;  // the previous sample again: counted here, and left to the stuck detector on the bus
    }
    tfc::ism::convert(raw, chip_.config(), gyro, accel);
    return true;
  }
  [[nodiscard]] uint32_t stale() const { return stale_; }
  [[nodiscard]] uint32_t errors() const { return errors_; }

 private:
  ZephyrSpi spi_;
  tfc::ism::SpiRegisters<ZephyrSpi> regs_;
  tfc::ism::Ism330<tfc::ism::SpiRegisters<ZephyrSpi>> chip_;
  uint32_t stale_ = 0U;
  uint32_t errors_ = 0U;
};

using Imu = Ism330Imu;
#else
using Imu = SimImu;
#endif

// ---------------------------------------------------------------- lines to the supervisor

#if DT_NODE_EXISTS(DT_NODELABEL(tfc_lines))

class Lines {
 public:
  void init() {
    configure(frame_, GPIO_OUTPUT_INACTIVE);
    configure(kick_, GPIO_OUTPUT_INACTIVE);
    configure(adopt_, GPIO_INPUT);
    configure(id0_, GPIO_INPUT);
    configure(id1_, GPIO_INPUT);
  }
  void frame(bool on) { set(frame_, on); }
  void kick(bool on) { set(kick_, on); }
  [[nodiscard]] bool adopt() const { return gpio_pin_get_dt(&adopt_) > 0; }
  // The node id on the two straps (a strap tied to ground reads as 1), or -1 if a pin cannot be read.
  [[nodiscard]] int node_id() const {
    const int b0 = gpio_pin_get_dt(&id0_);
    const int b1 = gpio_pin_get_dt(&id1_);
    return (b0 < 0 || b1 < 0) ? -1 : (b0 + (2 * b1));
  }

 private:
  static void configure(const gpio_dt_spec& s, gpio_flags_t flags) {
    if (gpio_is_ready_dt(&s)) {
      (void)gpio_pin_configure_dt(&s, flags);
    }
  }
  static void set(const gpio_dt_spec& s, bool on) { (void)gpio_pin_set_dt(&s, on ? 1 : 0); }

  gpio_dt_spec frame_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_lines), frame_gpios);
  gpio_dt_spec kick_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_lines), kick_gpios);
  gpio_dt_spec adopt_ = GPIO_DT_SPEC_GET(DT_NODELABEL(tfc_lines), adopt_gpios);
  gpio_dt_spec id0_ = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tfc_lines), id_gpios, 0);
  gpio_dt_spec id1_ = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(tfc_lines), id_gpios, 1);
};

#else

class Lines {
 public:
  void init() {}
  void frame(bool) {}
  void kick(bool) {}
  [[nodiscard]] bool adopt() const { return false; }
  [[nodiscard]] int node_id() const { return -1; }
};

#endif

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
