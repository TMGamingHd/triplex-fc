// SPDX-License-Identifier: MIT
// The behaviour of a release, as the commands its flight function gives on a fixed set of scenarios (ADR-021, TFC-ARCH-006). Three flight functions (each with its own IMU model) fly the
// 6-DOF vehicle and their commands are voted by a plain mid-value; the voted command of every frame is printed. `tools/compat/compat_gate.py` builds this file against two releases of
// `core/include` (the current one and the golden one) and requires their commands to agree within the version tolerance.
// The harness uses only the stable surface of the core: the protocol's frames, `FlightFunction` (consensus, estimator, controller, guidance) and its tables. It does not use the fault manager, ACT or
// the resynchronisation, so a golden release older than those still builds.
//   tfc_compat [--list] [SCENARIO ...]        prints `scenario,frame,pitch_deg,yaw_deg` lines
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "design.hpp"
#include "imu_model.hpp"
#include "runner.hpp"
#include "tfc/flight.hpp"
#include "tfc/protocol.hpp"

namespace {

struct Scenario {
  const char* name;
  uint32_t frames;
  sim::SensorErrors sensors;
  double thrust_misalign_deg;
  bool node_b_dead;
  bool gyro_fault_b;
};

float median3(float a, float b, float c) { return std::max(std::min(a, b), std::min(std::max(a, b), c)); }

void fly(const Scenario& sc) {
  sim::RunnerConfig rcfg;
  rcfg.params.thrust_misalign_pitch_deg = sc.thrust_misalign_deg;
  sim::SimRunner runner(rcfg);
  const sim::FlightTables tables = sim::flight_tables(rcfg.design);
  tfc::EstimatorConfig est;
  est.use_accel = false;
  std::array<tfc::FlightFunction, 3> ff{tfc::FlightFunction(tables.gains, tables.guidance, est), tfc::FlightFunction(tables.gains, tables.guidance, est),
                                        tfc::FlightFunction(tables.gains, tables.guidance, est)};
  std::array<sim::ImuModel, 3> imu{sim::ImuModel(sc.sensors, 0x1234U), sim::ImuModel(sc.sensors, 0x1235U), sim::ImuModel(sc.sensors, 0x1236U)};
  sim::SimFrames pending = runner.start(0U);
  for (uint32_t k = 0; k < sc.frames; ++k) {
    const tfc::DecodedVec3 dr = tfc::unpack_vec3(pending.f[0], tfc::kGyroLsbDps);
    const tfc::DecodedVec3 da = tfc::unpack_vec3(pending.f[1], tfc::kAccelLsbG);
    const uint8_t seq = static_cast<uint8_t>(k);
    std::array<tfc::Frame, 6> sensor{};
    const std::array<bool, 3> alive{true, !(sc.node_b_dead && k >= sc.frames / 2U), true};
    for (uint8_t n = 0; n < 3U; ++n) {
      tfc::Vec3 g;
      tfc::Vec3 a;
      imu[n].sample(dr.x, da.x, g, a);
      if (sc.gyro_fault_b && n == 1U) {
        g.v[0] += 15.0F;
        g.v[1] += 15.0F;
      }
      sensor[n] = tfc::pack_gyro(n, g, seq);
      sensor[3U + n] = tfc::pack_accel(n, a, seq);
    }
    std::array<tfc::Command, 3> cmd{};
    for (unsigned n = 0; n < 3U; ++n) {
      if (!alive[n]) {
        continue;
      }
      ff[n].begin_frame(k, alive[1] ? 0x07U : 0x05U);
      for (unsigned m = 0; m < 3U; ++m) {
        if (alive[m]) {
          (void)ff[n].on_frame(sensor[m]);
          (void)ff[n].on_frame(sensor[3U + m]);
        }
      }
      cmd[n] = ff[n].step();
    }
    tfc::ActFrame act;
    if (alive[1]) {
      act.pitch_deg = median3(cmd[0].pitch_deg, cmd[1].pitch_deg, cmd[2].pitch_deg);
      act.yaw_deg = median3(cmd[0].yaw_deg, cmd[1].yaw_deg, cmd[2].yaw_deg);
    } else {
      act.pitch_deg = 0.5F * (cmd[0].pitch_deg + cmd[2].pitch_deg);
      act.yaw_deg = 0.5F * (cmd[0].yaw_deg + cmd[2].yaw_deg);
    }
    act.state = 1U;
    std::printf("%s,%u,%.6f,%.6f\n", sc.name, k, static_cast<double>(act.pitch_deg), static_cast<double>(act.yaw_deg));
    pending = runner.end_of_frame(k, &act);
  }
}

}  // namespace

int main(int argc, char** argv) {
  sim::SensorErrors dispersed;
  dispersed.gyro_bias_dps = 0.05F;
  dispersed.gyro_scale_err = 0.002F;
  dispersed.misalign_deg = 0.1F;
  sim::SensorErrors quiet;
  quiet.gyro_noise_amp_dps = 0.0F;
  quiet.accel_noise_amp_g = 0.0F;
  const std::vector<Scenario> all = {{"nominal", 6000U, sim::SensorErrors{}, 0.0, false, false},
                                     {"quiet", 4000U, quiet, 0.0, false, false},
                                     {"dispersed_imus", 6000U, dispersed, 0.0, false, false},
                                     {"thrust_misalignment", 6000U, sim::SensorErrors{}, 0.5, false, false},
                                     {"gyro_fault_in_b", 4000U, sim::SensorErrors{}, 0.0, false, true},
                                     {"computer_b_lost", 4000U, sim::SensorErrors{}, 0.0, true, false}};
  if (argc > 1 && std::strcmp(argv[1], "--list") == 0) {
    for (const Scenario& s : all) {
      std::printf("%s\n", s.name);
    }
    return 0;
  }
  for (const Scenario& s : all) {
    bool wanted = argc <= 1;
    for (int i = 1; i < argc; ++i) {
      wanted = wanted || std::strcmp(argv[i], s.name) == 0;
    }
    if (wanted) {
      fly(s);
    }
  }
  return 0;
}
