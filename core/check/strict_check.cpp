// SPDX-License-Identifier: MIT
// Build gate for the flight core: this file is compiled (CMake target tfc_strict_check) with the strictest warning set
// the project uses, as errors, and exercises the main entry points so that code which only inlines at -O2 is analysed
// too (null dereference, stack usage). It is never linked into anything. Each header is also compiled on its own
// (see CMakeLists.txt) so it cannot rely on another header having been included first.
#include "tfc/crc8.hpp"
#include "tfc/fault_monitor.hpp"
#include "tfc/integrity.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/voter.hpp"

namespace tfc_check {
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
  char text[64];
  tfc::format_reasons(static_cast<uint8_t>(acc), text, sizeof text);
  return static_cast<int>(acc) + static_cast<int>(tfc::validate_config(mgr.config())) + static_cast<int>(text[0]);
}
}  // namespace tfc_check
