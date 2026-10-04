// SPDX-License-Identifier: MIT
// Build gate for the flight core: this file is compiled (CMake target tfc_strict_check) with the strictest warning set
// the project uses, as errors, and exercises the main entry points so that code which only inlines at -O2 is analysed
// too (null dereference, stack usage). It is never linked into anything. Each header is also compiled on its own
// (see CMakeLists.txt) so it cannot rely on another header having been included first.
#include "tfc/crc8.hpp"
#include "tfc/fault_monitor.hpp"
#include "tfc/integrity.hpp"
#include "tfc/ism330dhcx.hpp"
#include "tfc/progress.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/resetlog.hpp"
#include "tfc/voter.hpp"

namespace tfc_check {
namespace {
// A bus that answers every request with zeros: enough to compile and analyse the driver's code paths.
struct NullSpi {
  bool transfer(const uint8_t* tx, uint8_t* rx, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) {
      rx[i] = static_cast<uint8_t>(tx[0] & 0U);
    }
    return true;
  }
  void delay_us(uint32_t) {}
};
}  // namespace

int exercise(unsigned frames);  // declared here only to satisfy -Wmissing-declarations: nothing calls it
int exercise(unsigned frames) {
  tfc::RedundancyManager mgr;
  unsigned acc = 0U;
  for (unsigned k = 0; k < frames; ++k) {
    mgr.begin_frame();
    const auto seq = static_cast<uint8_t>(k);
    for (uint8_t n = 0; n < 3U; ++n) {
      (void)mgr.on_frame(tfc::pack_gyro(n, tfc::Vec3{{1.0F, 2.0F, 3.0F}}, seq));
      (void)mgr.on_frame(tfc::pack_accel(n, tfc::Vec3{{0.0F, 0.0F, 1.0F}}, seq));
      (void)mgr.on_frame(tfc::pack_cmd(n, tfc::Command{0.5F, -0.5F, 0x1234U}, seq));
    }
    (void)mgr.on_frame(tfc::pack_ground_auth(tfc::GroundOp::Reintegrate, 1U, seq, tfc::kBenchKey));
    acc += static_cast<unsigned>(mgr.end_frame().mode);
    acc += static_cast<unsigned>(mgr.command(tfc::GroundOp::ClearSafe, 0U));
  }
  NullSpi spi;
  tfc::ism::SpiRegisters<NullSpi> regs(spi);
  tfc::ism::Ism330<tfc::ism::SpiRegisters<NullSpi>> imu(regs, tfc::ism::Config{});
  tfc::ism::RawSample raw;
  acc += static_cast<unsigned>(imu.init());
  acc += static_cast<unsigned>(imu.read(raw));
  acc += static_cast<unsigned>(imu.self_test(tfc::ism::SelfTestSetup{}));
  tfc::ProgressMonitor progress(0x03U);
  progress.report(0U);
  acc += progress.end_of_frame(true) ? 1U : 0U;
  tfc::ResetRecord record;
  tfc::ResetLog resets(record);
  resets.boot(tfc::ResetCause::Watchdog);
  acc += resets.loop_detected() ? 1U : 0U;
  char text[64];
  tfc::format_reasons(static_cast<uint8_t>(acc), text, sizeof text);
  return static_cast<int>(acc) + static_cast<int>(tfc::validate_config(mgr.config())) + static_cast<int>(text[0]);
}
}  // namespace tfc_check
