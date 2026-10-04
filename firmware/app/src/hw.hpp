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

#include "hw_common.hpp"
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
  void set_frame(uint32_t) {}
  void feed(const tfc::Frame&) {}
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

// The simulated IMU fed by the vehicle simulator over the bus: the body rates and acceleration of frame k arrive as 0x501 and 0x502 just after SYNC; this
// node adds its own noise. If either frame did not arrive for this frame the sample is missing, as for any dead sensor.
class BusImu {
 public:
  explicit BusImu(unsigned node) : noise_(0x1234U + node) {}
  bool init() { return true; }
  // The simulator's frames carry the frame number (low byte) they are for; a frame of another cycle is stale and is not used.
  void set_frame(uint32_t k) { expect_ = static_cast<uint8_t>(k); }
  void feed(const tfc::Frame& f) {
    if (f.data[6] != expect_) {
      ++stale_frames_;
      return;
    }
    if (f.id == tfc::id::kSimRates) {
      const tfc::DecodedVec3 d = tfc::unpack_vec3(f, tfc::kGyroLsbDps);
      have_rates_ = d.ok;
      rates_ = d.x;
    } else if (f.id == tfc::id::kSimAccel) {
      const tfc::DecodedVec3 d = tfc::unpack_vec3(f, tfc::kAccelLsbG);
      have_accel_ = d.ok;
      accel_ = d.x;
    }
  }
  bool sample(uint32_t, tfc::Vec3& gyro, tfc::Vec3& accel) {
    const bool ok = have_rates_ && have_accel_;
    if (!ok) {
      ++missing_;
    } else {
      for (unsigned i = 0; i < 3U; ++i) {
        gyro.v[i] = rates_.v[i] + (0.17F * noise_.uniform());
        accel.v[i] = accel_.v[i] + (0.0035F * noise_.uniform());
      }
    }
    have_rates_ = false;  // each frame needs its own
    have_accel_ = false;
    return ok;
  }
  [[nodiscard]] uint32_t stale() const { return stale_frames_; }
  [[nodiscard]] uint32_t errors() const { return missing_; }

 private:
  fc::sim::Noise noise_;
  uint8_t expect_ = 0U;
  uint32_t stale_frames_ = 0U;
  tfc::Vec3 rates_{};
  tfc::Vec3 accel_{};
  bool have_rates_ = false;
  bool have_accel_ = false;
  uint32_t missing_ = 0U;
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
  void set_frame(uint32_t) {}
  void feed(const tfc::Frame&) {}
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
#elif IS_ENABLED(CONFIG_TFC_SIM_BUS_IMU)
using Imu = BusImu;
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

}  // namespace fc::hw
