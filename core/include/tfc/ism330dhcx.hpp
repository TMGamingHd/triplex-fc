// SPDX-License-Identifier: MIT
// ISM330DHCX 6-axis IMU: the chip logic (identity check, reset, configuration with read-back, one burst read of status,
// temperature, gyro and accelerometer) over an abstract bus, so it can be tested on the host against a model of the
// register file and runs unchanged over the board's SPI. Register addresses, field layouts and sensitivities are those of
// STMicroelectronics' own driver (ism330dhcx_reg.h and .c); nothing is copied from memory.
//
// A `Bus` provides:  bool read_regs(uint8_t reg, uint8_t* out, uint8_t n);  bool write_reg(uint8_t reg, uint8_t value);
//                    void delay_us(uint32_t us);
// `SpiRegisters<Spi>` turns a raw SPI transfer into such a bus (the address byte, the read bit, the burst).
// No heap, no exceptions, no RTTI, no loops without a bound. Deterministic.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "tfc/protocol.hpp"

namespace tfc::ism {

namespace reg {
constexpr uint8_t kWhoAmI = 0x0FU;
constexpr uint8_t kCtrl1Xl = 0x10U;
constexpr uint8_t kCtrl2G = 0x11U;
constexpr uint8_t kCtrl3C = 0x12U;
constexpr uint8_t kCtrl5C = 0x14U;
constexpr uint8_t kStatus = 0x1EU;  // then 0x1F reserved, 0x20/0x21 temperature, 0x22.. gyro X Y Z, 0x28.. accel X Y Z
}  // namespace reg

constexpr uint8_t kDeviceId = 0x6BU;
constexpr uint8_t kCtrl3SwReset = 0x01U;
constexpr uint8_t kCtrl3IfInc = 0x04U;
constexpr uint8_t kCtrl3Bdu = 0x40U;
constexpr uint8_t kStatusXlda = 0x01U;  // new accelerometer sample
constexpr uint8_t kStatusGda = 0x02U;   // new gyroscope sample
constexpr uint8_t kCtrl5StXlPositive = 0x01U;
constexpr uint8_t kCtrl5StGPositive = 0x04U;
constexpr uint8_t kBurstBytes = 16U;  // status, reserved, temperature (2), gyro (6), accel (6)

// The values of the FS_XL field of CTRL1_XL and the FS_G field of CTRL2_G (ST's enumerations).
enum class AccelRange : uint8_t { G2 = 0, G16 = 1, G4 = 2, G8 = 3 };
enum class GyroRange : uint8_t { Dps250 = 0, Dps4000 = 1, Dps125 = 2, Dps500 = 4, Dps1000 = 8, Dps2000 = 12 };

enum class Status : uint8_t { Ok = 0, BusError, WrongId, ResetTimeout, ConfigMismatch, SelfTestFailed };

struct Config {
  AccelRange accel_range = AccelRange::G16;  // the protocol carries +-16 g (kAccelLsbG)
  GyroRange gyro_range = GyroRange::Dps500;  // a proposal for the rig; the protocol carries up to +-4096 dps
  uint8_t odr_code = 7U;                     // ST's code for both sensors: 7 = 833 Hz, 8 = 1666 Hz
  uint32_t reset_poll_us = 1000U;
  uint8_t reset_polls = 20U;
};

struct RawSample {
  std::array<int16_t, 3> gyro{};
  std::array<int16_t, 3> accel{};
  int16_t temp = 0;
  bool gyro_new = false;   // a gyroscope sample was ready when read
  bool accel_new = false;  // an accelerometer sample was ready when read
};

// Self-test (ST's procedure: average before and after switching the self-test on, compare the change). The limits come from
// the datasheet's self-test table and are deliberately NOT guessed here: the default limits are an empty interval, so the
// test fails, and cannot pass by accident. `discard` samples are dropped after every change and `average` are averaged.
struct SelfTestSetup {
  Config test_config;                 // the ranges and data rate the datasheet specifies for the test
  uint8_t discard = 5U;
  uint8_t average = 5U;
  uint32_t sample_period_us = 5000U;  // wait between reads, longer than one sample period at test_config.odr_code
  uint32_t settle_us = 100000U;       // wait after switching the self-test on
  int32_t accel_min_lsb = 1;          // |change| of every accelerometer axis must lie in [min, max]; the default is an
  int32_t accel_max_lsb = 0;          // empty interval, so nothing passes until real limits are set
  int32_t gyro_min_lsb = 1;
  int32_t gyro_max_lsb = 0;
};

constexpr float accel_g_per_lsb(AccelRange r) noexcept {
  switch (r) {
    case AccelRange::G2: return 0.061F * 0.001F;
    case AccelRange::G16: return 0.488F * 0.001F;
    case AccelRange::G4: return 0.122F * 0.001F;
    case AccelRange::G8: return 0.244F * 0.001F;
    default: return 0.488F * 0.001F;
  }
}

constexpr float gyro_dps_per_lsb(GyroRange r) noexcept {
  switch (r) {
    case GyroRange::Dps250: return 8.75F * 0.001F;
    case GyroRange::Dps4000: return 140.0F * 0.001F;
    case GyroRange::Dps125: return 4.375F * 0.001F;
    case GyroRange::Dps500: return 17.5F * 0.001F;
    case GyroRange::Dps1000: return 35.0F * 0.001F;
    case GyroRange::Dps2000: return 70.0F * 0.001F;
    default: return 17.5F * 0.001F;
  }
}

// Raw counts to the physical units of the bus protocol (dps and g).
inline void convert(const RawSample& s, const Config& c, Vec3& gyro_dps, Vec3& accel_g) noexcept {
  const float gs = gyro_dps_per_lsb(c.gyro_range);
  const float as = accel_g_per_lsb(c.accel_range);
  for (std::size_t i = 0; i < 3U; ++i) {
    gyro_dps.v[i] = static_cast<float>(s.gyro[i]) * gs;
    accel_g.v[i] = static_cast<float>(s.accel[i]) * as;
  }
}

constexpr uint8_t kMaxSpiBurst = 16U;

// A register bus over raw SPI: the first byte is the register address with bit 7 set for a read; the chip auto-increments.
// `Spi` provides:  bool transfer(const uint8_t* tx, uint8_t* rx, uint8_t len);  void delay_us(uint32_t us);
template <typename Spi>
class SpiRegisters {
 public:
  explicit SpiRegisters(Spi& spi) noexcept : spi_(spi) {}

  [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* out, uint8_t n) noexcept {
    if (n == 0U || n > kMaxSpiBurst) {
      return false;
    }
    std::array<uint8_t, kMaxSpiBurst + 1U> tx{};
    std::array<uint8_t, kMaxSpiBurst + 1U> rx{};
    tx[0] = static_cast<uint8_t>(reg | 0x80U);
    if (!spi_.transfer(tx.data(), rx.data(), static_cast<uint8_t>(n + 1U))) {
      return false;
    }
    for (uint8_t i = 0; i < n; ++i) {
      out[i] = rx[static_cast<std::size_t>(i) + 1U];
    }
    return true;
  }

  [[nodiscard]] bool write_reg(uint8_t reg, uint8_t value) noexcept {
    const std::array<uint8_t, 2> tx{static_cast<uint8_t>(reg & 0x7FU), value};
    std::array<uint8_t, 2> rx{};
    return spi_.transfer(tx.data(), rx.data(), 2U);
  }

  void delay_us(uint32_t us) noexcept { spi_.delay_us(us); }

 private:
  Spi& spi_;
};

namespace detail {
constexpr int16_t le16(uint8_t lo, uint8_t hi) noexcept {
  return static_cast<int16_t>(static_cast<uint16_t>(static_cast<uint16_t>(lo) | static_cast<uint16_t>(static_cast<uint16_t>(hi) << 8U)));
}
constexpr int32_t abs_diff(int32_t a, int32_t b) noexcept { return a > b ? a - b : b - a; }
}  // namespace detail

template <typename Bus>
class Ism330 {
 public:
  Ism330(Bus& bus, const Config& config) noexcept : bus_(bus), cfg_(config) {}

  // Check the identity, reset, and configure: block data update, address auto-increment, data rate and ranges, each written
  // and read back. A part that answers with the wrong identity, never finishes its reset, or does not keep what was written
  // is reported, never used.
  [[nodiscard]] Status init() noexcept {
    uint8_t id = 0U;
    if (!bus_.read_regs(reg::kWhoAmI, &id, 1U)) {
      return Status::BusError;
    }
    if (id != kDeviceId) {
      return Status::WrongId;
    }
    const Status r = reset_chip();
    if (r != Status::Ok) {
      return r;
    }
    return configure(cfg_);
  }

  // One burst read: status, temperature, gyro, accelerometer. The status bits say whether each sample is new since the
  // last read (a sample that is not new is the previous one repeated).
  [[nodiscard]] Status read(RawSample& out) noexcept {
    std::array<uint8_t, kBurstBytes> b{};
    if (!bus_.read_regs(reg::kStatus, b.data(), kBurstBytes)) {
      return Status::BusError;
    }
    out.accel_new = (b[0] & kStatusXlda) != 0U;
    out.gyro_new = (b[0] & kStatusGda) != 0U;
    out.temp = detail::le16(b[2], b[3]);
    for (std::size_t i = 0; i < 3U; ++i) {
      out.gyro[i] = detail::le16(b[4U + (2U * i)], b[5U + (2U * i)]);
      out.accel[i] = detail::le16(b[10U + (2U * i)], b[11U + (2U * i)]);
    }
    return Status::Ok;
  }

  // ST's self-test: the change in the average of the outputs when the self-test is switched on must lie inside the limits
  // on every axis. The normal configuration is restored afterwards.
  [[nodiscard]] Status self_test(const SelfTestSetup& t) noexcept {
    Status s = configure(t.test_config);
    if (s == Status::Ok) {
      s = run_self_test(t);
    }
    const Status restore = configure(cfg_);
    return s != Status::Ok ? s : restore;
  }

  [[nodiscard]] const Config& config() const noexcept { return cfg_; }

 private:
  using Mean = std::array<int32_t, 6>;  // gyro x y z, accel x y z

  [[nodiscard]] Status reset_chip() noexcept {
    if (!bus_.write_reg(reg::kCtrl3C, kCtrl3SwReset)) {
      return Status::BusError;
    }
    for (uint8_t i = 0; i < cfg_.reset_polls; ++i) {
      bus_.delay_us(cfg_.reset_poll_us);
      uint8_t c3 = 0U;
      if (!bus_.read_regs(reg::kCtrl3C, &c3, 1U)) {
        return Status::BusError;
      }
      if ((c3 & kCtrl3SwReset) == 0U) {
        return Status::Ok;
      }
    }
    return Status::ResetTimeout;
  }

  [[nodiscard]] Status write_verified(uint8_t r, uint8_t value) noexcept {
    uint8_t back = 0U;
    if (!bus_.write_reg(r, value) || !bus_.read_regs(r, &back, 1U)) {
      return Status::BusError;
    }
    return back == value ? Status::Ok : Status::ConfigMismatch;
  }

  [[nodiscard]] Status configure(const Config& c) noexcept {
    const uint8_t odr = static_cast<uint8_t>(c.odr_code << 4U);
    const uint8_t xl = static_cast<uint8_t>(odr | static_cast<uint8_t>(static_cast<uint8_t>(c.accel_range) << 2U));
    const uint8_t g = static_cast<uint8_t>(odr | static_cast<uint8_t>(c.gyro_range));
    Status s = write_verified(reg::kCtrl3C, static_cast<uint8_t>(kCtrl3Bdu | kCtrl3IfInc));
    if (s == Status::Ok) {
      s = write_verified(reg::kCtrl1Xl, xl);
    }
    if (s == Status::Ok) {
      s = write_verified(reg::kCtrl2G, g);
    }
    return s;
  }

  // Wait, read, and average `n` samples after dropping `discard` of them.
  [[nodiscard]] Status mean_of(const SelfTestSetup& t, Mean& mean) noexcept {
    Mean sum{};
    const uint8_t total = static_cast<uint8_t>(t.discard + t.average);
    for (uint8_t k = 0; k < total; ++k) {
      bus_.delay_us(t.sample_period_us);
      RawSample s;
      if (read(s) != Status::Ok) {
        return Status::BusError;
      }
      if (k >= t.discard) {
        for (std::size_t i = 0; i < 3U; ++i) {
          sum[i] += s.gyro[i];
          sum[3U + i] += s.accel[i];
        }
      }
    }
    const int32_t div = t.average > 0U ? static_cast<int32_t>(t.average) : 1;
    for (std::size_t i = 0; i < mean.size(); ++i) {
      mean[i] = sum[i] / div;
    }
    return Status::Ok;
  }

  [[nodiscard]] Status run_self_test(const SelfTestSetup& t) noexcept {
    Mean off{};
    Mean on{};
    Status s = mean_of(t, off);
    if (s == Status::Ok) {
      s = write_verified(reg::kCtrl5C, static_cast<uint8_t>(kCtrl5StXlPositive | kCtrl5StGPositive));
    }
    if (s == Status::Ok) {
      bus_.delay_us(t.settle_us);
      s = mean_of(t, on);
    }
    const Status off_again = write_verified(reg::kCtrl5C, 0U);  // always switch it off again
    if (s != Status::Ok) {
      return s;
    }
    if (off_again != Status::Ok) {
      return off_again;
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      const int32_t dg = detail::abs_diff(on[i], off[i]);
      const int32_t da = detail::abs_diff(on[3U + i], off[3U + i]);
      if (dg < t.gyro_min_lsb || dg > t.gyro_max_lsb || da < t.accel_min_lsb || da > t.accel_max_lsb) {
        return Status::SelfTestFailed;
      }
    }
    return Status::Ok;
  }

  Bus& bus_;
  Config cfg_;
};

}  // namespace tfc::ism
