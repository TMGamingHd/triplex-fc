// SPDX-License-Identifier: MIT
// The hardware-facing logic of the core: the ISM330DHCX driver against a model of its register file (over a fake SPI that
// can fail at any call), the watchdog progress monitor, and the reset log.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

#include "tfc/ism330dhcx.hpp"
#include "tfc/progress.hpp"
#include "tfc/resetlog.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

// A model of the chip's registers: identity, software reset that takes a number of polls, a register that ignores writes,
// sample output (with a change when the self-test bits are set), and status flags. Behind the SPI framing of the real part.
struct FakeSpi {
  std::array<uint8_t, 128> r{};
  uint8_t id = ism::kDeviceId;
  int fail_at = -1;  // index of the transfer that fails
  int calls = 0;
  int polls_needed = 1;
  int polls_seen = 0;
  bool resetting = false;
  int stuck = -1;  // register that ignores writes
  std::array<int16_t, 3> gyro{100, -200, 300};
  std::array<int16_t, 3> accel{-1, 2, 4000};
  int16_t temp = -7;
  bool fresh = true;
  int16_t gyro_st = 0;   // change of the gyro output when the gyro self-test is on
  int16_t accel_st = 0;  // change of the accelerometer output when the accelerometer self-test is on
  uint32_t delay_total = 0;

  void delay_us(uint32_t us) { delay_total += us; }

  static void put(std::array<uint8_t, 16>& b, std::size_t at, int16_t v) {
    b[at] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFFU);
    b[at + 1U] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8U) & 0xFFU);
  }

  uint8_t read_one(unsigned a) {
    if (a == ism::reg::kWhoAmI) {
      return id;
    }
    if (a == ism::reg::kCtrl3C && resetting) {
      if (++polls_seen >= polls_needed) {
        resetting = false;
        r[a] = 0x04U;  // reset value: address auto-increment on
      }
    }
    return r[a];
  }

  // The 16-byte burst starting at the status register, built from the current state.
  void burst(unsigned a, uint8_t* rx, unsigned n) {
    std::array<uint8_t, 16> b{};
    b[0] = fresh ? static_cast<uint8_t>(ism::kStatusXlda | ism::kStatusGda) : 0U;
    put(b, 2, temp);
    const bool gst = (r[ism::reg::kCtrl5C] & 0x0CU) != 0U;
    const bool ast = (r[ism::reg::kCtrl5C] & 0x03U) != 0U;
    for (std::size_t i = 0; i < 3U; ++i) {
      put(b, 4U + (2U * i), static_cast<int16_t>(gyro[i] + (gst ? gyro_st : 0)));
      put(b, 10U + (2U * i), static_cast<int16_t>(accel[i] + (ast ? accel_st : 0)));
    }
    for (unsigned i = 0; i < n; ++i) {
      rx[i] = b[(a - ism::reg::kStatus) + i];
    }
  }

  bool transfer(const uint8_t* tx, uint8_t* rx, uint8_t len) {
    if (calls++ == fail_at) {
      return false;
    }
    const unsigned a = tx[0] & 0x7FU;
    if ((tx[0] & 0x80U) != 0U) {
      if (a == ism::reg::kStatus && len == ism::kBurstBytes + 1U) {
        burst(a, rx + 1, ism::kBurstBytes);
      } else {
        for (unsigned i = 1; i < len; ++i) {
          rx[i] = read_one(a + i - 1U);
        }
      }
      return true;
    }
    if (static_cast<int>(a) != stuck) {
      r[a] = tx[1];
      if (a == ism::reg::kCtrl3C && (tx[1] & ism::kCtrl3SwReset) != 0U) {
        resetting = true;
        polls_seen = 0;
      }
    }
    return true;
  }
};

using Regs = ism::SpiRegisters<FakeSpi>;

ism::Config default_config() { return ism::Config{}; }

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// Number of SPI transfers a successful run of `fn` makes.
template <typename F>
int calls_of(F fn) {
  FakeSpi spi;
  spi.accel_st = 50;
  spi.gyro_st = 100;
  Regs bus(spi);
  fn(bus, spi);
  return spi.calls;
}

ism::SelfTestSetup good_setup() {
  ism::SelfTestSetup t;
  t.test_config.accel_range = ism::AccelRange::G4;
  t.test_config.gyro_range = ism::GyroRange::Dps2000;
  t.test_config.odr_code = 3U;
  t.accel_min_lsb = 20;
  t.accel_max_lsb = 100;
  t.gyro_min_lsb = 50;
  t.gyro_max_lsb = 200;
  return t;
}

}  // namespace

TFC_TEST(ism_init_configures_the_chip) {
  FakeSpi spi;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  CHECK(spi.r[ism::reg::kCtrl3C] == (ism::kCtrl3Bdu | ism::kCtrl3IfInc));
  CHECK(spi.r[ism::reg::kCtrl1Xl] == 0x74U);  // 833 Hz, +-16 g
  CHECK(spi.r[ism::reg::kCtrl2G] == 0x74U);   // 833 Hz, +-500 dps
  CHECK(imu.config().odr_code == 7U);
  CHECK(spi.delay_total >= 1000U);  // waited for the reset
}

TFC_TEST(ism_init_rejects_wrong_identity) {
  FakeSpi spi;
  spi.id = 0x6AU;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::WrongId);
  CHECK(spi.r[ism::reg::kCtrl1Xl] == 0U);  // nothing was configured
}

TFC_TEST(ism_init_times_out_when_the_reset_never_finishes) {
  FakeSpi spi;
  spi.polls_needed = 1000;
  Regs bus(spi);
  ism::Config c = default_config();
  c.reset_polls = 3U;
  ism::Ism330<Regs> imu(bus, c);
  CHECK(imu.init() == ism::Status::ResetTimeout);
}

TFC_TEST(ism_init_detects_a_register_that_does_not_keep_its_value) {
  for (const unsigned stuck : {static_cast<unsigned>(ism::reg::kCtrl3C), static_cast<unsigned>(ism::reg::kCtrl1Xl),
                               static_cast<unsigned>(ism::reg::kCtrl2G)}) {
    FakeSpi spi;
    spi.stuck = static_cast<int>(stuck);
    Regs bus(spi);
    ism::Ism330<Regs> imu(bus, default_config());
    const ism::Status s = imu.init();
    // CTRL3_C is also written by the reset, which a stuck register ignores: that surfaces as a timeout, not a mismatch.
    CHECK(s == ism::Status::ConfigMismatch || s == ism::Status::ResetTimeout);
  }
}

TFC_TEST(ism_init_reports_a_bus_failure_at_every_step) {
  const int total = calls_of([](Regs& bus, FakeSpi&) {
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::Ok);
  });
  CHECK(total > 8);
  for (int i = 0; i < total; ++i) {
    FakeSpi spi;
    spi.fail_at = i;
    Regs bus(spi);
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::BusError);
  }
}

TFC_TEST(ism_read_decodes_a_burst) {
  FakeSpi spi;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  ism::RawSample s;
  CHECK(imu.read(s) == ism::Status::Ok);
  CHECK(s.gyro_new && s.accel_new);
  CHECK(s.gyro[0] == 100 && s.gyro[1] == -200 && s.gyro[2] == 300);
  CHECK(s.accel[0] == -1 && s.accel[1] == 2 && s.accel[2] == 4000);
  CHECK(s.temp == -7);
  spi.fresh = false;
  CHECK(imu.read(s) == ism::Status::Ok);
  CHECK(!s.gyro_new && !s.accel_new);
}

TFC_TEST(ism_read_reports_a_bus_failure) {
  FakeSpi spi;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  spi.fail_at = spi.calls;
  ism::RawSample s;
  CHECK(imu.read(s) == ism::Status::BusError);
}

TFC_TEST(ism_convert_uses_the_sensitivity_of_each_range) {
  ism::RawSample s;
  s.gyro = {1000, -1000, 0};
  s.accel = {1000, -1000, 2048};
  Vec3 g;
  Vec3 a;
  ism::Config c = default_config();
  ism::convert(s, c, g, a);  // +-500 dps (17.5 mdps), +-16 g (0.488 mg)
  CHECK(near(g.v[0], 17.5F, 1e-3F) && near(g.v[1], -17.5F, 1e-3F) && near(g.v[2], 0.0F, 1e-6F));
  CHECK(near(a.v[0], 0.488F, 1e-4F) && near(a.v[2], 2048.0F * 0.000488F, 1e-3F));
  const std::array<std::pair<ism::GyroRange, float>, 6> gyro = {{{ism::GyroRange::Dps125, 4.375F},
                                                                  {ism::GyroRange::Dps250, 8.75F},
                                                                  {ism::GyroRange::Dps500, 17.5F},
                                                                  {ism::GyroRange::Dps1000, 35.0F},
                                                                  {ism::GyroRange::Dps2000, 70.0F},
                                                                  {ism::GyroRange::Dps4000, 140.0F}}};
  for (const auto& e : gyro) {
    c.gyro_range = e.first;
    ism::convert(s, c, g, a);
    CHECK(near(g.v[0], e.second, 1e-3F));
  }
  const std::array<std::pair<ism::AccelRange, float>, 4> accel = {{{ism::AccelRange::G2, 0.061F},
                                                                    {ism::AccelRange::G4, 0.122F},
                                                                    {ism::AccelRange::G8, 0.244F},
                                                                    {ism::AccelRange::G16, 0.488F}}};
  for (const auto& e : accel) {
    c.accel_range = e.first;
    ism::convert(s, c, g, a);
    CHECK(near(a.v[0], e.second, 1e-4F));
  }
  // An invalid field value falls back to the widest range (the safe reading), never to zero.
  const uint8_t invalid = 0x7FU;  // copied in, so no cast to an out-of-range enumerator is written
  std::memcpy(&c.gyro_range, &invalid, 1U);
  std::memcpy(&c.accel_range, &invalid, 1U);
  ism::convert(s, c, g, a);
  CHECK(near(g.v[0], 17.5F, 1e-3F) && near(a.v[0], 0.488F, 1e-4F));
}

TFC_TEST(ism_spi_adapter_frames_reads_and_writes) {
  FakeSpi spi;
  Regs bus(spi);
  CHECK(bus.write_reg(ism::reg::kCtrl1Xl, 0x5AU));
  CHECK(spi.r[ism::reg::kCtrl1Xl] == 0x5AU);
  std::array<uint8_t, 2> out{};
  CHECK(bus.read_regs(ism::reg::kCtrl1Xl, out.data(), 1U));
  CHECK(out[0] == 0x5AU);
  CHECK(!bus.read_regs(ism::reg::kCtrl1Xl, out.data(), 0U));
  CHECK(!bus.read_regs(ism::reg::kCtrl1Xl, out.data(), static_cast<uint8_t>(ism::kMaxSpiBurst + 1U)));
  spi.fail_at = spi.calls;
  CHECK(!bus.write_reg(ism::reg::kCtrl1Xl, 1U));
  spi.fail_at = spi.calls;
  CHECK(!bus.read_regs(ism::reg::kCtrl1Xl, out.data(), 1U));
}

TFC_TEST(ism_self_test_passes_inside_the_limits_and_restores_the_configuration) {
  FakeSpi spi;
  spi.accel_st = 50;
  spi.gyro_st = 100;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  CHECK(imu.self_test(good_setup()) == ism::Status::Ok);
  CHECK(spi.r[ism::reg::kCtrl5C] == 0U);      // self-test switched off again
  CHECK(spi.r[ism::reg::kCtrl1Xl] == 0x74U);  // normal configuration restored
  CHECK(spi.r[ism::reg::kCtrl2G] == 0x74U);
}

TFC_TEST(ism_self_test_fails_outside_the_limits_on_any_axis_or_with_no_change) {
  const std::array<std::array<int16_t, 2>, 4> deltas = {{{0, 0}, {10, 100}, {50, 10}, {500, 100}}};  // gyro, accel
  for (const auto& d : deltas) {
    FakeSpi spi;
    spi.gyro_st = d[0];
    spi.accel_st = d[1];
    Regs bus(spi);
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::Ok);
    CHECK(imu.self_test(good_setup()) == ism::Status::SelfTestFailed);
    CHECK(spi.r[ism::reg::kCtrl5C] == 0U);
  }
  // The default limits are an empty interval: nothing passes, not even a chip whose self-test does nothing.
  FakeSpi spi;
  spi.accel_st = 50;
  spi.gyro_st = 100;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  CHECK(imu.self_test(ism::SelfTestSetup{}) == ism::Status::SelfTestFailed);
  spi.gyro_st = 0;
  spi.accel_st = 0;
  CHECK(imu.self_test(ism::SelfTestSetup{}) == ism::Status::SelfTestFailed);
}

TFC_TEST(ism_self_test_reports_a_bus_failure_at_every_step) {
  const int init_calls = calls_of([](Regs& bus, FakeSpi&) {
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::Ok);
  });
  const int total = calls_of([](Regs& bus, FakeSpi&) {
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::Ok);
    CHECK(imu.self_test(good_setup()) == ism::Status::Ok);
  });
  CHECK(total > init_calls + 10);
  for (int i = init_calls; i < total; ++i) {
    FakeSpi spi;
    spi.accel_st = 50;
    spi.gyro_st = 100;
    Regs bus(spi);
    ism::Ism330<Regs> imu(bus, default_config());
    CHECK(imu.init() == ism::Status::Ok);
    spi.fail_at = i;
    CHECK(imu.self_test(good_setup()) != ism::Status::Ok);
  }
}

TFC_TEST(ism_self_test_with_no_samples_to_average_divides_by_one) {
  FakeSpi spi;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  ism::SelfTestSetup t = good_setup();
  t.average = 0U;
  t.gyro_min_lsb = 0;
  t.gyro_max_lsb = 0;
  t.accel_min_lsb = 0;
  t.accel_max_lsb = 0;
  CHECK(imu.self_test(t) == ism::Status::Ok);  // nothing summed, nothing changes: all means are zero
}

TFC_TEST(ism_self_test_stops_when_the_test_configuration_is_not_kept) {
  FakeSpi spi;
  Regs bus(spi);
  ism::Ism330<Regs> imu(bus, default_config());
  CHECK(imu.init() == ism::Status::Ok);
  spi.stuck = ism::reg::kCtrl1Xl;
  CHECK(imu.self_test(good_setup()) == ism::Status::ConfigMismatch);
}

// ---- progress monitor ----

TFC_TEST(progress_services_the_watchdog_only_when_every_task_reported_and_the_vote_ran) {
  ProgressMonitor m(0x07U);
  m.report(0);
  m.report(1);
  m.report(2);
  CHECK(m.end_of_frame(true));
  CHECK(m.missing() == 0U && m.refusals() == 0U);
  m.report(0);
  m.report(2);
  CHECK(!m.end_of_frame(true));  // task 1 did not report
  CHECK(m.missing() == 0x02U && m.refusals() == 1U);
  m.report(0);
  m.report(1);
  m.report(2);
  CHECK(!m.end_of_frame(false));  // the vote did not run
  CHECK(m.missing() == 0U && m.refusals() == 2U);
}

TFC_TEST(progress_one_frame_cannot_cover_for_the_next) {
  ProgressMonitor m(0x03U);
  m.report(0);
  m.report(1);
  CHECK(m.end_of_frame(true));
  CHECK(!m.end_of_frame(true));  // nobody reported in the second frame
  CHECK(m.missing() == 0x03U);
}

TFC_TEST(progress_ignores_and_counts_a_task_that_does_not_exist) {
  ProgressMonitor m(0x01U);
  m.report(8);
  m.report(200);
  m.report(0);
  CHECK(m.end_of_frame(true));
  CHECK(m.bad_reports() == 2U);
  ProgressMonitor none(0U);
  CHECK(none.end_of_frame(true));  // nothing required: the vote alone decides
  CHECK(!none.end_of_frame(false));
}

// ---- reset log ----

TFC_TEST(resetlog_a_power_on_starts_a_fresh_log) {
  ResetRecord rec;
  rec.boots = 99U;  // garbage in RAM
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  CHECK(log.valid() && log.boots() == 1U && log.short_boots() == 0U);
  CHECK(log.last_cause() == ResetCause::PowerOn && !log.loop_detected());
}

TFC_TEST(resetlog_three_short_boots_in_a_row_are_a_loop) {
  ResetRecord rec;
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  log.running(200U);
  log.boot(ResetCause::Watchdog);
  CHECK(log.short_boots() == 1U && !log.loop_detected());
  log.running(100U);
  log.boot(ResetCause::Watchdog);
  CHECK(log.short_boots() == 2U && !log.loop_detected());
  log.running(50U);
  log.boot(ResetCause::Brownout);
  CHECK(log.short_boots() == 3U && log.loop_detected() && log.boots() == 4U);
  CHECK(log.last_cause() == ResetCause::Brownout);
}

TFC_TEST(resetlog_a_long_boot_clears_the_count) {
  ResetRecord rec;
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  log.running(10U);
  log.boot(ResetCause::Watchdog);
  log.running(10U);
  log.boot(ResetCause::Fault);
  CHECK(log.short_boots() == 2U);
  log.running(6000U);  // ran for 60 s
  log.boot(ResetCause::Watchdog);
  CHECK(log.short_boots() == 0U && !log.loop_detected());
}

TFC_TEST(resetlog_a_deliberate_software_reset_neither_counts_nor_clears) {
  ResetRecord rec;
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  log.running(10U);
  log.boot(ResetCause::Watchdog);
  CHECK(log.short_boots() == 1U);
  log.running(10U);
  log.boot(ResetCause::Software);
  CHECK(log.short_boots() == 1U && log.boots() == 3U);
  log.running(10U);
  log.boot(ResetCause::Pin);
  CHECK(log.short_boots() == 2U);
  log.running(10U);
  log.boot(ResetCause::Unknown);
  CHECK(log.short_boots() == 3U && log.loop_detected());
}

TFC_TEST(resetlog_a_damaged_record_is_discarded_not_trusted) {
  ResetRecord rec;
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  log.running(10U);
  log.boot(ResetCause::Watchdog);
  CHECK(log.valid());
  rec.short_boots ^= 4U;  // a bit flipped in RAM
  CHECK(!log.valid());
  log.boot(ResetCause::Watchdog);  // starts again from one boot, no short boots
  CHECK(log.valid() && log.boots() == 1U && log.short_boots() == 0U);
  rec.magic = 0U;  // wiped
  CHECK(!log.valid());
}

TFC_TEST(resetlog_counters_saturate_instead_of_wrapping) {
  ResetRecord rec;
  ResetLog log(rec);
  log.boot(ResetCause::PowerOn);
  rec.boots = 0xFFFFFFFFU;
  rec.short_boots = 0xFFFFFFFFU;
  rec.frames_last_boot = 1U;
  rec.crc = detail::record_crc(rec);
  log.boot(ResetCause::Watchdog);
  CHECK(log.boots() == 0xFFFFFFFFU && log.short_boots() == 0xFFFFFFFFU && log.loop_detected());
}

TFC_TEST(resetlog_policy_changes_what_counts_as_a_loop) {
  ResetRecord rec;
  ResetPolicy p;
  p.short_boot_frames = 100U;
  p.loop_boots = 1U;
  ResetLog log(rec, p);
  log.boot(ResetCause::PowerOn);
  log.running(99U);
  log.boot(ResetCause::Watchdog);
  CHECK(log.loop_detected());
  log.running(100U);
  log.boot(ResetCause::Watchdog);
  CHECK(!log.loop_detected());
}
