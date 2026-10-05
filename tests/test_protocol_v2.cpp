// SPDX-License-Identifier: MIT
// Protocol version 2 (docs/design/PROTOCOL.md): ACT's output, the heartbeat, the state share and the simulator's frames. The golden bytes below are pinned in
// the Python mirror too (sim/tests/test_protocol.py): if one side changes, both tests fail until both agree.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

std::string hex(const Frame& f) {
  std::string s;
  char b[3];
  for (unsigned i = 0; i < 8U; ++i) {
    (void)std::snprintf(b, sizeof b, "%02x", f.data[i]);
    s += b;
  }
  return s;
}

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

ActFrame act_example() {
  ActFrame a;
  a.pitch_deg = 1.234F;
  a.yaw_deg = -5.678F;
  a.state = 3U;
  a.held = true;
  a.vote_status = 2U;
  a.voted_nodes = 5U;
  a.excluded_nodes = 2U;
  a.cause = 1U;
  return a;
}

}  // namespace

TFC_TEST(protocol_v2_golden_frames_match_the_pinned_bytes) {
  CHECK(hex(pack_act_out(act_example(), 9U)) == "d204d2e9ab2a092d");
  CHECK(pack_act_out(act_example(), 9U).id == 0x300U);
  CHECK(hex(pack_act_out(ActFrame{}, 0U)) == "000000000000000a");
  ActFrame sat;
  sat.pitch_deg = -40.0F;
  sat.yaw_deg = 40.0F;
  sat.state = 7U;
  sat.held = true;
  sat.vote_status = 7U;
  sat.voted_nodes = 7U;
  sat.excluded_nodes = 7U;
  sat.cause = 7U;
  CHECK(hex(pack_act_out(sat, 255U)) == "0080ff7fffffff5d");  // the angles saturate, every field is all ones

  Heartbeat h;
  h.mode = 2U;
  h.safe_requested = true;
  h.role = 1U;
  h.quarantined = true;
  h.node_state = {0U, 1U, 3U};
  h.reset_count = 7U;
  h.release_hash = 0xBEEFU;
  const Frame hb = pack_heartbeat(1U, h, 4U);
  CHECK(hb.id == 0x401U && hex(hb) == "02563407efbe0496");
  CHECK(hex(pack_heartbeat(0U, Heartbeat{}, 0U)) == "02000000000000b0");  // protocol version 2, everything else zero

  StateShare s;
  s.strikes = {1U, 15U, 2U};
  s.command_counter = 200U;
  const Frame sf = pack_state_share(2U, s, 6U);
  CHECK(sf.id == 0x412U && hex(sf) == "f102c800000006f7");

  CHECK(hex(pack_sim_rates(Vec3{{1.5F, -2.0F, 0.125F}}, 3U)) == "0c00f0ff0100032c");
  CHECK(pack_sim_rates(Vec3{}, 0U).id == 0x501U && pack_sim_accel(Vec3{}, 0U).id == 0x502U);
  CHECK(hex(pack_sim_accel(Vec3{{0.0F, 0.5F, 1.0F}}, 255U)) == "000000040008ffda");  // the same bytes as the accel frame of node C: same scale
  SimState ss;
  ss.altitude_m = 31963.0F;
  ss.speed_ms = 995.4F;
  ss.mass_kg = 14869.0F;
  CHECK(hex(pack_sim_state(ss, 17U)) == "7c0ce303153a118d");
  SimState over;
  over.altitude_m = 1.0e9F;
  over.speed_ms = -5.0F;
  over.mass_kg = 70000.0F;
  CHECK(hex(pack_sim_state(over, 0U)) == "ffff0000ffff00b9");  // saturates high, negative is zero
  SimTelemetry t;
  t.dynamic_pressure_pa = 31500.0F;
  t.pitch_error_deg = -0.31F;
  t.yaw_error_deg = 1.25F;
  CHECK(hex(pack_sim_telemetry(t, 18U)) == "4e0ccafee20412b7");
  SimFlags sfl;
  sfl.flags = simflag::kSafed | simflag::kEngineOut;
  sfl.engines_on = 4U;
  sfl.time_frames = 6500U;
  CHECK(hex(pack_sim_flags(sfl, 19U)) == "050464190000131e");
  SimFlags mx;
  mx.flags = 0xFFU;
  mx.engines_on = 5U;
  mx.time_frames = 0xFFFFFFFFU;
  CHECK(hex(pack_sim_flags(mx, 255U)) == "ff05ffffffffff7e");
}

TFC_TEST(protocol_v2_frames_round_trip) {
  const DecodedAct a = unpack_act_out(pack_act_out(act_example(), 9U));
  CHECK(a.ok && a.seq == 9U && near(a.act.pitch_deg, 1.234F, 1e-6F) && near(a.act.yaw_deg, -5.678F, 1e-6F));
  CHECK(a.act.state == 3U && a.act.held && a.act.vote_status == 2U && a.act.voted_nodes == 5U && a.act.excluded_nodes == 2U && a.act.cause == 1U);
  Heartbeat h;
  h.mode = 3U;
  h.bus_alarm = true;
  h.role = 2U;
  h.node_state = {3U, 2U, 1U};
  h.reset_count = 255U;
  h.release_hash = 0x1234U;
  const DecodedHeartbeat d = unpack_heartbeat(pack_heartbeat(2U, h, 77U));
  CHECK(d.ok && d.seq == 77U && d.hb.protocol_version == kProtocolVersion && d.hb.mode == 3U && !d.hb.safe_requested && d.hb.bus_alarm);
  CHECK(d.hb.role == 2U && !d.hb.quarantined && d.hb.node_state[0] == 3U && d.hb.node_state[1] == 2U && d.hb.node_state[2] == 1U);
  CHECK(d.hb.reset_count == 255U && d.hb.release_hash == 0x1234U);
  StateShare s;
  s.strikes = {15U, 0U, 7U};
  s.command_counter = 9U;
  const DecodedStateShare ds = unpack_state_share(pack_state_share(0U, s, 1U));
  CHECK(ds.ok && ds.share.strikes[0] == 15U && ds.share.strikes[1] == 0U && ds.share.strikes[2] == 7U && ds.share.command_counter == 9U);
  SimState ss;
  ss.altitude_m = 31960.0F;
  ss.speed_ms = 995.0F;
  ss.mass_kg = 14869.0F;
  const DecodedSimState dss = unpack_sim_state(pack_sim_state(ss, 2U));
  CHECK(dss.ok && near(dss.s.altitude_m, 31960.0F, 1e-3F) && near(dss.s.speed_ms, 995.0F, 1e-3F) && near(dss.s.mass_kg, 14869.0F, 1e-3F));
  SimTelemetry t;
  t.dynamic_pressure_pa = 31500.0F;
  t.pitch_error_deg = -0.31F;
  t.yaw_error_deg = 1.25F;
  const DecodedSimTelemetry dt = unpack_sim_telemetry(pack_sim_telemetry(t, 3U));
  CHECK(dt.ok && near(dt.t.dynamic_pressure_pa, 31500.0F, 1e-3F) && near(dt.t.pitch_error_deg, -0.31F, 1e-6F) && near(dt.t.yaw_error_deg, 1.25F, 1e-6F));
  SimFlags f;
  f.flags = simflag::kPlatformSaturated | simflag::kCommandHeld | simflag::kAborted;
  f.engines_on = 3U;
  f.time_frames = 123456U;
  const DecodedSimFlags df = unpack_sim_flags(pack_sim_flags(f, 4U));
  CHECK(df.ok && df.s.flags == f.flags && df.s.engines_on == 3U && df.s.time_frames == 123456U);
}

TFC_TEST(protocol_v2_fields_do_not_overlap) {
  // each field of the ACT status word, set alone, shows up alone
  for (unsigned bit = 0; bit < 16U; ++bit) {
    Frame f = pack_act_out(ActFrame{}, 0U);
    f.data[4U + (bit / 8U)] = static_cast<uint8_t>(1U << (bit % 8U));
    f.data[7] = crc8(f.data.data(), 7);
    const ActFrame a = unpack_act_out(f).act;
    const unsigned set = (a.state != 0U ? 1U : 0U) + (a.held ? 1U : 0U) + (a.vote_status != 0U ? 1U : 0U) + (a.voted_nodes != 0U ? 1U : 0U) +
                         (a.excluded_nodes != 0U ? 1U : 0U) + (a.cause != 0U ? 1U : 0U);
    CHECK(set == 1U);
  }
  // values wider than their fields are masked, never spilled into the next field
  ActFrame w;
  w.state = 0xFFU;
  w.vote_status = 0xFFU;
  w.voted_nodes = 0xFFU;
  w.excluded_nodes = 0U;
  w.cause = 0U;
  const ActFrame back = unpack_act_out(pack_act_out(w, 0U)).act;
  CHECK(back.state == 7U && back.vote_status == 7U && back.voted_nodes == 7U && back.excluded_nodes == 0U && back.cause == 0U && !back.held);
  Heartbeat h;
  h.mode = 0xFFU;
  h.role = 0xFFU;
  h.node_state = {0xFFU, 0U, 0U};
  const Heartbeat hb = unpack_heartbeat(pack_heartbeat(0U, h, 0U)).hb;
  CHECK(hb.mode == 3U && hb.role == 3U && hb.node_state[0] == 3U && hb.node_state[1] == 0U && !hb.safe_requested && !hb.quarantined);
}

TFC_TEST(protocol_v2_decoders_reject_damage_the_wrong_id_and_a_node_that_does_not_exist) {
  Frame a = pack_act_out(act_example(), 1U);
  a.data[7] ^= 1U;
  CHECK(!unpack_act_out(a).ok);
  Frame wrong = pack_act_out(act_example(), 1U);
  wrong.id = id::kHeartbeat;
  CHECK(!unpack_act_out(wrong).ok);
  Frame hb = pack_heartbeat(0U, Heartbeat{}, 0U);
  hb.id = id::kHeartbeat + 3U;
  CHECK(!unpack_heartbeat(hb).ok);
  Frame hb_bad = pack_heartbeat(1U, Heartbeat{}, 0U);
  hb_bad.data[2] ^= 0x10U;
  CHECK(!unpack_heartbeat(hb_bad).ok);
  Frame st = pack_state_share(0U, StateShare{}, 0U);
  st.id = id::kState + 3U;
  CHECK(!unpack_state_share(st).ok);
  Frame st_bad = pack_state_share(1U, StateShare{}, 0U);
  st_bad.data[0] ^= 1U;
  CHECK(!unpack_state_share(st_bad).ok);
  Frame sim = pack_sim_state(SimState{}, 0U);
  sim.id = id::kSimTelemetry;
  CHECK(!unpack_sim_state(sim).ok);
  Frame sim_bad = pack_sim_state(SimState{}, 0U);
  sim_bad.data[1] ^= 1U;
  CHECK(!unpack_sim_state(sim_bad).ok);
  Frame tel = pack_sim_telemetry(SimTelemetry{}, 0U);
  tel.id = id::kSimState;
  CHECK(!unpack_sim_telemetry(tel).ok);
  Frame tel_bad = pack_sim_telemetry(SimTelemetry{}, 0U);
  tel_bad.data[0] ^= 4U;
  CHECK(!unpack_sim_telemetry(tel_bad).ok);
  Frame fl = pack_sim_flags(SimFlags{}, 0U);
  fl.id = id::kSimTelemetry;
  CHECK(!unpack_sim_flags(fl).ok);
  Frame fl_bad = pack_sim_flags(SimFlags{}, 0U);
  fl_bad.data[2] ^= 8U;
  CHECK(!unpack_sim_flags(fl_bad).ok);
}

TFC_TEST(protocol_v2_ids_keep_the_priority_order_and_the_simulator_range_is_known_to_the_manager) {
  CHECK(id::kSync < id::kGyroBase && id::kAccelBase < id::kCmdBase && id::kCmdBase < id::kActOut);
  CHECK(id::kActOut < id::kHeartbeat && id::kHeartbeat + 2U < id::kState && id::kState + 2U < id::kSim);
  CHECK(id::kSim < id::kSimRates && id::kSimFlags <= id::kSimLast && id::kSimLast < id::kGround);
  RedundancyManager m;
  m.begin_frame();
  Frame f;
  f.len = 8;
  for (uint32_t idv : {id::kSimRates, id::kSimAccel, id::kSimState, id::kSimTelemetry, id::kSimFlags, id::kSimLast, id::kState, id::kState + 1U, id::kState + 2U,
                       id::kHeartbeat + 1U, id::kActOut}) {
    f.id = idv;
    CHECK(m.on_frame(f));
  }
  CHECK(m.counters().out_of_schedule == 0U);  // the simulator's frames and the state share are not stray traffic, so they never raise the bus alarm
  for (uint32_t idv : {id::kSimLast + 2U, id::kState + 3U, id::kHeartbeat + 3U}) {  // 0x510 is the ground command, so one past it
    f.id = idv;
    CHECK(!m.on_frame(f));
  }
  CHECK(m.counters().out_of_schedule == 3U);
}
