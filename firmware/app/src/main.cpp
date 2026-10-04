// SPDX-License-Identifier: MIT
// One of the three flight computers (node A, B or C, from CONFIG_TFC_NODE_ID). Node A is the sync master at power-up; B and C follow its SYNC, keep
// counting through a gap, and the lowest of them takes over if SYNC stops (tfc::SyncClock). Every 10 ms major frame a node
//   t = 0.0 ms  the master broadcasts SYNC (frame number); the others lock their frame to it
//   t = 1.5 ms  samples its (simulated) IMU and broadcasts gyro + accel
//   t = 5.0 ms  broadcasts its command + estimator digest
//   t = 7.0 ms  hands everything received to tfc::RedundancyManager, which votes, cross-checks
//               and runs FDIR; latch events and a once-a-second status line go to the console
// (schedule: docs/ARCHITECTURE.md section 3). On native_sim the bus is the host's vcan0 and the other nodes are either the virtual peers (sim/) or more
// instances of this image; on the Nucleo it is FDCAN1.
// With CONFIG_TFC_FLIGHT_FUNCTION the command comes from tfc::FlightFunction (consensus, estimator, controller) instead of a scripted function of the frame
// number; with CONFIG_TFC_SIM_BUS_IMU the sensor input is the simulator's 0x501/0x502 frames instead of the scripted motion.
//
// Include core/ (and so the C++ standard library) BEFORE Zephyr headers: Zephyr defines an
// `__unused` macro that breaks a glibc header (struct_mutex.h) on the native_sim host build.
#include <array>
#include <cstdint>
#include <cstring>

#include "sim_imu.hpp"
#include "flight_tables.hpp"
#include "tfc/flight.hpp"
#include "tfc/progress.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"
#include "tfc/resetlog.hpp"
#include "tfc/sync_clock.hpp"

#include "hw.hpp"

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

namespace {

constexpr unsigned kNodeId = CONFIG_TFC_NODE_ID;
BUILD_ASSERT(kNodeId <= 2U, "the node id is 0 (A), 1 (B) or 2 (C)");
constexpr bool kFlightFunction = IS_ENABLED(CONFIG_TFC_FLIGHT_FUNCTION);
constexpr bool kSimBusImu = IS_ENABLED(CONFIG_TFC_SIM_BUS_IMU);

constexpr int64_t kSampleUs = 500;   // the IMU sample is latched here (ARCHITECTURE section 3) ...
constexpr int64_t kGyroUs = 1500;     // ... and sent here
constexpr int64_t kCmdUs = 5000;   // the command goes out here; the peers' sensor frames (sent by 3 ms) are drained just before
constexpr int64_t kVoteUs = CONFIG_TFC_VOTE_US;
constexpr uint32_t kStatusEveryFrames = 100;
// The phases of a frame that must each have run before the watchdog may be serviced (tfc::ProgressMonitor, FDIR-038).
constexpr unsigned kTaskSample = 0U;
constexpr unsigned kTaskSend = 1U;
constexpr unsigned kTaskVote = 2U;
constexpr uint8_t kProgressRequired = 0x07U;
// Peers may boot after this node; judge a never-seen peer only after 5 s (RedundancyConfig).
constexpr uint32_t kStartupGraceFrames = 500;

const struct device* const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
// Bus state changes reported by the CAN controller (interrupt context: only atomic counters). A bus-off is recovered by the controller on
// its own (TFC-FDIR-010); these make it visible on the console and in the log.
atomic_t g_bus_off_events = ATOMIC_INIT(0);
atomic_t g_error_passive_events = ATOMIC_INIT(0);

void can_state_changed(const struct device*, enum can_state state, struct can_bus_err_cnt, void*) {
  if (state == CAN_STATE_BUS_OFF) {
    atomic_inc(&g_bus_off_events);
  } else if (state == CAN_STATE_ERROR_PASSIVE) {
    atomic_inc(&g_error_passive_events);
  }
}

// Survives a reset (no-init RAM, kept intact because the record has no constructor): how often and why this node reset.
tfc::ResetRecord g_reset_record __noinit;
CAN_MSGQ_DEFINE(rx_msgq, 32);      // the three schedule slots (gyro, accel, command of every node)
CAN_MSGQ_DEFINE(rx_all_msgq, 64);  // everything: only used to see what is NOT in the schedule (babbling)
CAN_MSGQ_DEFINE(rx_sync_msgq, 4);  // SYNC, for the nodes that follow it (and for a master to see another's)
CAN_MSGQ_DEFINE(rx_sim_msgq, 8);   // the simulator's sensor inputs (0x501, 0x502; the mask also lets 0x500, 0x503 through)

// Statics, not locals: the flight function holds the tables and the filter state, about a kilobyte.
tfc::FlightFunction g_flight;

void sleep_until_us(int64_t base_ticks, int64_t offset_us) {
  k_sleep(K_TIMEOUT_ABS_TICKS(base_ticks + k_us_to_ticks_ceil64(offset_us)));
}

bool send(const tfc::Frame& f) {
  can_frame cf{};
  cf.id = f.id;
  cf.dlc = f.len;
  std::memcpy(cf.data, f.data.data(), 8);
  return can_send(can_dev, &cf, K_MSEC(2), nullptr, nullptr) == 0;
}

// True if `id` is one of this node's own data frames (the driver reports our own transmissions
// as TX confirmations, but never trust that: own samples are fed to the manager directly).
// True for the nine schedule-slot IDs (gyro/accel/command of nodes A, B, C).
bool is_schedule_slot(uint32_t id) {
  return (id & ~0x3U) == tfc::id::kGyroBase || (id & ~0x3U) == tfc::id::kAccelBase ||
         (id & ~0x3U) == tfc::id::kCmdBase;
}

bool is_own(uint32_t id) {
  return id == tfc::id::kGyroBase + kNodeId || id == tfc::id::kAccelBase + kNodeId ||
         id == tfc::id::kCmdBase + kNodeId;
}

bool bus_init() {
  if (!device_is_ready(can_dev)) {
    printk("CAN device not ready\n");
    return false;
  }
  // 1 Mbit/s classic CAN. The host SocketCAN driver has no bit timing and may refuse; ignore that.
  const int br = can_set_bitrate(can_dev, 1000000);
  if (br != 0 && br != -ENOTSUP) {
    printk("can_set_bitrate: %d (continuing)\n", br);
  }
  // Acceptance filters: only the three schedule slots (each matches nodes 0..2). Out-of-schedule
  // IDs such as a babbling node's never reach the application.
  for (const uint32_t base : {tfc::id::kGyroBase, tfc::id::kAccelBase, tfc::id::kCmdBase}) {
    const can_filter filter{.id = base, .mask = 0x7FCU, .flags = 0U};
    if (can_add_rx_filter_msgq(can_dev, &rx_msgq, &filter) < 0) {
      printk("cannot add CAN rx filter for id 0x%03x\n", static_cast<unsigned>(base));
      return false;
    }
  }
  // Catch-all: lets stray high-priority traffic reach the manager's out-of-schedule counter and bus
  // alarm (TFC-FDIR-009). Frames that match a schedule slot arrive here too and are skipped on drain.
  // On the target this is the expensive part (every frame costs an interrupt): the FDIR rate monitor is
  // worth it, and a hardware acceptance filter plus the controller's lost-message counter is the
  // cheaper alternative to evaluate at bring-up.
  const can_filter all{.id = 0U, .mask = 0U, .flags = 0U};
  if (can_add_rx_filter_msgq(can_dev, &rx_all_msgq, &all) < 0) {
    printk("cannot add catch-all CAN rx filter\n");
    return false;
  }
  const can_filter sync{.id = tfc::id::kSync, .mask = 0x7FFU, .flags = 0U};
  if (can_add_rx_filter_msgq(can_dev, &rx_sync_msgq, &sync) < 0) {
    printk("cannot add the SYNC rx filter\n");
    return false;
  }
  if (kSimBusImu) {
    const can_filter sim{.id = tfc::id::kSim, .mask = 0x7FCU, .flags = 0U};
    if (can_add_rx_filter_msgq(can_dev, &rx_sim_msgq, &sim) < 0) {
      printk("cannot add the simulator rx filter\n");
      return false;
    }
  }
  can_set_state_change_callback(can_dev, can_state_changed, nullptr);
  const int rc = can_start(can_dev);
  if (rc != 0) {
    printk("can_start: %d\n", rc);
    return false;
  }
  return true;
}

const char* mode_text(tfc::Mode m) {
  switch (m) {
    case tfc::Mode::Triplex: return "TRIPLEX";
    case tfc::Mode::Duplex: return "DUPLEX";
    case tfc::Mode::Simplex: return "SIMPLEX";
    case tfc::Mode::Safe: return "SAFE";
  }
  return "?";
}

// Per-node state letter for the status line: '+' voting, 'X' latched out, 'p' on probation,
// 'D' disabled for the run, '?' no good data this frame.
char node_state(const tfc::FrameReport& r, unsigned n) {
  if (((r.disabled_mask >> n) & 1U) != 0U) {
    return 'D';
  }
  if (((r.probation_mask >> n) & 1U) != 0U) {
    return 'p';
  }
  if (((r.latched_mask >> n) & 1U) != 0U) {
    return 'X';
  }
  return ((r.valid_mask >> n) & 1U) != 0U ? '+' : '?';
}

// Parse the 32 hex digits of CONFIG_TFC_GROUND_KEY; returns false (and leaves `key` untouched) if it is not exactly that.
bool parse_key(const char* hex, tfc::AuthKey& key) {
  tfc::AuthKey out{};
  for (unsigned i = 0; i < 32U; ++i) {
    const char c = hex[i];
    unsigned v = 0U;
    if (c >= '0' && c <= '9') {
      v = static_cast<unsigned>(c - '0');
    } else if (c >= 'a' && c <= 'f') {
      v = static_cast<unsigned>(c - 'a') + 10U;
    } else if (c >= 'A' && c <= 'F') {
      v = static_cast<unsigned>(c - 'A') + 10U;
    } else {
      return false;  // too short, or not hex
    }
    out[i / 2U] = static_cast<uint8_t>((i % 2U == 0U) ? (v << 4U) : (out[i / 2U] | v));
  }
  if (hex[32] != '\0') {
    return false;  // too long
  }
  key = out;
  return true;
}

}  // namespace

int main() {
  if (!bus_init()) {
    return 1;
  }
  tfc::RedundancyConfig cfg;
  cfg.startup_grace_frames = kStartupGraceFrames;
  cfg.policy = IS_ENABLED(CONFIG_TFC_AUTO_REINTEGRATE) ? tfc::ReintegrationPolicy::AutoTransient
                                                       : tfc::ReintegrationPolicy::Manual;
  if (!parse_key(CONFIG_TFC_GROUND_KEY, cfg.ground_key)) {
    printk("CONFIG ERROR: TFC_GROUND_KEY must be exactly 32 hex digits; using the PUBLIC bench key\n");
  }
  tfc::RedundancyManager mgr(cfg);
  if (mgr.config_errors() != 0U) {  // a bad configuration is replaced by defaults, never run silently
    printk("CONFIG ERROR: invalid fields (mask 0x%x) replaced by defaults\n", static_cast<unsigned>(mgr.config_errors()));
  }
  fc::hw::Imu imu(kNodeId);
  const bool imu_ok = imu.init();
  fc::hw::Lines lines;
  lines.init();
  const int strap_id = lines.node_id();
  if (strap_id >= 0 && static_cast<unsigned>(strap_id) != kNodeId) {
    printk("WARNING: the node-id straps read %d but this image is built for node %u\n", strap_id, kNodeId);
  }
  fc::hw::Watchdog wdt;
  tfc::ResetPolicy reset_policy;
  reset_policy.short_boot_frames = static_cast<uint32_t>(CONFIG_TFC_SHORT_BOOT_S) * 100U;
  reset_policy.loop_boots = static_cast<uint32_t>(CONFIG_TFC_RESET_LOOP_BOOTS);
  tfc::ResetLog resets(g_reset_record, reset_policy);
  resets.boot(fc::hw::reset_cause());
  printk("boot %u since power-on, last reset cause %u (1 power-on, 2 pin, 3 watchdog, 4 software, 5 brown-out, 6 fault), %u short boots in a row\n",
         static_cast<unsigned>(resets.boots()), static_cast<unsigned>(resets.last_cause()), static_cast<unsigned>(resets.short_boots()));
  const bool quarantined = IS_ENABLED(CONFIG_TFC_QUARANTINE_ON_RESET_LOOP) && resets.loop_detected();
  tfc::ProgressMonitor progress(kProgressRequired);
  constexpr int64_t kPeriodUs = fc::sim::kFrameUs;
  const int64_t period = k_us_to_ticks_ceil64(kPeriodUs);
  const int64_t window = k_us_to_ticks_ceil64(tfc::sync_window_us(kNodeId, CONFIG_TFC_SYNC_STAGGER_US));
  // Node A listens first, even after a power-up (it may be the one that was off while B took over: two masters would collide on one id), and
  // claims the bus if nobody is sending after a few frames; B and C only follow, and take over only after they have heard a master and lost it.
  const tfc::SyncStart sync_start = kNodeId != 0U ? tfc::SyncStart::FollowOnly : tfc::SyncStart::Listen;
  tfc::SyncClock sync_clock(sync_start);
  int64_t nominal = k_uptime_ticks() + k_ms_to_ticks_ceil64(200);  // when the next frame should start
  tfc::Mode last_mode = tfc::Mode::Triplex;
  bool last_bus_alarm = false;
  bool last_safe_request = false;
  uint32_t tx_errors = 0;
  uint32_t sync_missed_total = 0;
  uint8_t usable_nodes = 0x07U;  // whose sensors the flight function may use: the nodes the fault manager has not excluded
  if (kFlightFunction) {
    tfc::GainSchedule gains;
    tfc::Guidance guidance;
    fc::tables::load(gains, guidance);
    g_flight = tfc::FlightFunction(gains, guidance);
  }

  printk("FC-%c (node %u): %s, 100 Hz frame loop%s%s.\n", 'A' + static_cast<char>(kNodeId), kNodeId,
         sync_start == tfc::SyncStart::Listen ? "listens for a master, then claims SYNC" : "following SYNC",
         kFlightFunction ? ", flight function on" : "", kSimBusImu ? ", sensors from the simulator" : "");
  printk("status: '+' voting, 'X' latched out, 'p' on probation, 'D' disabled, '?' no good data this frame\n");

  // Hand every schedule-slot frame that has arrived to the manager (and the sensor frames to the flight function); our own frames are not repeated back.
  auto drain_schedule_slots = [&]() {
    can_frame cf;
    while (k_msgq_get(&rx_msgq, &cf, K_NO_WAIT) == 0) {
      tfc::Frame f;
      f.id = cf.id;
      f.len = static_cast<uint8_t>(can_dlc_to_bytes(cf.dlc));
      std::memcpy(f.data.data(), cf.data, 8);
      if (!is_own(f.id)) {
        mgr.on_frame(f);
        if (kFlightFunction) {
          (void)g_flight.on_frame(f);
        }
      }
    }
  };

  for (uint32_t cycle = 0;; ++cycle) {
    if (quarantined) {  // a reset loop (TFC-FDIR-042): stay silent, keep the watchdog serviced, say so once a second
      k_sleep(K_TIMEOUT_ABS_TICKS(nominal));
      nominal += period;
      if (cycle == 0U) {
        (void)wdt.start(static_cast<uint32_t>(CONFIG_TFC_WATCHDOG_TIMEOUT_MS));
      }
      wdt.feed();
      if (cycle % kStatusEveryFrames == 0U) {
        printk("[cycle %u] RESET LOOP: %u short boots in a row; this node is held out and silent until it is power-cycled\n", cycle,
               static_cast<unsigned>(resets.short_boots()));
      }
      continue;
    }

    // ---- the start of the frame: the master wakes on its own clock, a follower waits for SYNC (up to its window) ----
    bool heard = false;
    uint32_t heard_number = 0U;
    int64_t base = nominal;
    can_frame sf;
    if (sync_clock.master()) {
      k_sleep(K_TIMEOUT_ABS_TICKS(nominal));
      while (k_msgq_get(&rx_sync_msgq, &sf, K_NO_WAIT) == 0) {  // another master's SYNC: this node yields
        tfc::Frame f;
        f.id = sf.id;
        f.len = static_cast<uint8_t>(can_dlc_to_bytes(sf.dlc));
        std::memcpy(f.data.data(), sf.data, 8);
        const tfc::DecodedSync d = tfc::unpack_sync(f);
        if (d.ok) {
          heard = true;
          heard_number = d.frame_no;
        }
      }
    } else {
      while (k_msgq_get(&rx_sync_msgq, &sf, K_TIMEOUT_ABS_TICKS(nominal + window)) == 0) {
        tfc::Frame f;
        f.id = sf.id;
        f.len = static_cast<uint8_t>(can_dlc_to_bytes(sf.dlc));
        std::memcpy(f.data.data(), sf.data, 8);
        const tfc::DecodedSync d = tfc::unpack_sync(f);
        if (d.ok) {
          heard = true;
          heard_number = d.frame_no;
          base = k_uptime_ticks();  // the frame is locked to the arrival of SYNC
          break;
        }
      }
    }
    const tfc::SyncTick tick = sync_clock.cycle(heard, heard_number);
    const uint32_t k = tick.frame;
    if (tick.took_over) {
      base = k_uptime_ticks();  // the new master's frame starts now, at the end of its wait; its SYNC tells the others
    }
    nominal = base + period;  // (for a frame without SYNC, `base` is still the nominal start, so the cadence does not drift)
    if (tick.took_over) {
      printk("[frame %u] SYNC lost for %u frames: this node takes over as sync master\n", k, static_cast<unsigned>(tick.missed));
    }
    if (tick.yielded) {
      printk("[frame %u] another node is sending SYNC: this node follows it\n", k);
    }
    if (tick.synced && !tick.locked && !tick.master) {
      ++sync_missed_total;
    }

    if (cycle == 0U && !wdt.start(static_cast<uint32_t>(CONFIG_TFC_WATCHDOG_TIMEOUT_MS)) && DT_HAS_ALIAS(watchdog0)) {
      printk("WATCHDOG: could not be started\n");
    }
    if (!tick.synced) {  // no SYNC heard yet and not the master: there is no frame number to stamp frames with, so stay silent and keep the watchdog serviced
      wdt.feed();
      if (cycle % kStatusEveryFrames == 0U) {
        printk("[cycle %u] waiting for SYNC (listening: nothing is sent until a master is heard or this node claims the bus)\n", cycle);
      }
      continue;
    }
    if (!tick.master && !tick.locked) {
      // SYNC did not come in this frame and this node is not taking over: the frame is skipped, by every follower alike. Running it on the node's own
      // clock would put each follower's frame at a different time (their windows differ), so two nodes could receive different mixes of the same frames,
      // compute different inputs and drift apart for good (docs/DECISIONS.md ADR-025). The number still counts, so it stays continuous.
      wdt.feed();
      continue;
    }
    lines.kick(false);
    lines.frame(true);  // FRAME: a pulse at the start of every frame, cleared when the sample is latched
    if (cycle % kStatusEveryFrames == 0U) {
      resets.running(cycle);  // how long this boot has lasted, for the next boot's loop check
    }
    const uint8_t seq = static_cast<uint8_t>(k);
    mgr.begin_frame(k);  // SYNC's frame number: the number every node stamps its frames with (ADR-018)
    if (kFlightFunction) {
      g_flight.begin_frame(k, usable_nodes);
    }
    if (cycle == 0U) {
      k_msgq_purge(&rx_msgq);  // anything queued before the first frame belongs to no frame
      k_msgq_purge(&rx_all_msgq);
      k_msgq_purge(&rx_sim_msgq);
    }
    if (tick.master) {
      tx_errors += send(tfc::pack_sync(k, seq)) ? 0U : 1U;
    }

    // ---- t = 0.5 ms: latch the IMU sample ----
    sleep_until_us(base, kSampleUs);
    if (kSimBusImu) {
      can_frame sim_cf;
      while (k_msgq_get(&rx_sim_msgq, &sim_cf, K_NO_WAIT) == 0) {  // the simulator's inputs for this frame (sent just after SYNC)
        tfc::Frame f;
        f.id = sim_cf.id;
        f.len = static_cast<uint8_t>(can_dlc_to_bytes(sim_cf.dlc));
        std::memcpy(f.data.data(), sim_cf.data, 8);
        imu.feed(f);
      }
    }
    tfc::Vec3 gyro{};
    tfc::Vec3 accel{};
    const bool have_sample = imu_ok && imu.sample(k, gyro, accel);
    lines.frame(false);
    progress.report(kTaskSample);

    // ---- t = 1.5 ms: send it (nothing if the sensor could not be read: peers see a missing sample, as for any dead sensor) ----
    sleep_until_us(base, kGyroUs);
    if (have_sample) {
      const tfc::Frame g = tfc::pack_gyro(kNodeId, gyro, seq);
      const tfc::Frame a = tfc::pack_accel(kNodeId, accel, seq);
      mgr.on_frame(g);
      mgr.on_frame(a);
      if (kFlightFunction) {
        (void)g_flight.on_frame(g);
        (void)g_flight.on_frame(a);
      }
      tx_errors += send(g) ? 0U : 1U;
      tx_errors += send(a) ? 0U : 1U;
    }
    progress.report(kTaskSend);

    // ---- t = 5.0 ms: command + digest ----
    sleep_until_us(base, kCmdUs);
    drain_schedule_slots();  // the peers' sensor frames, which the flight function needs now
    tfc::Command cmd;
    if (kFlightFunction) {
      cmd = g_flight.step();
    } else {
      // Without the flight function the command is a fixed function of the frame number, the same on every replica and on the PC, so the digest can be
      // checked against a golden run; it does not come from the IMU.
      cmd = fc::sim::command(fc::sim::truth(k), k);
    }
    const tfc::Frame c = tfc::pack_cmd(kNodeId, cmd, seq);
    mgr.on_frame(c);
    tx_errors += send(c) ? 0U : 1U;

    // ---- t = 7.0 ms: everything from the peers has arrived; vote ----
    sleep_until_us(base, kVoteUs);
    drain_schedule_slots();
    can_frame cf;
    while (k_msgq_get(&rx_all_msgq, &cf, K_NO_WAIT) == 0) {
      if (is_schedule_slot(cf.id)) {
        continue;  // already handled through rx_msgq
      }
      tfc::Frame f;
      f.id = cf.id;
      f.len = static_cast<uint8_t>(can_dlc_to_bytes(cf.dlc));
      std::memcpy(f.data.data(), cf.data, 8);
      mgr.on_frame(f);  // counts it as out-of-schedule (or ignores SYNC/ACT/heartbeat/sim IDs)
    }
    const tfc::FrameReport& rep = mgr.end_frame();
    usable_nodes = static_cast<uint8_t>(~rep.latched_mask & 0x07U);  // next frame's flight function uses only the nodes that are voting
    progress.report(kTaskVote);
    {  // the heartbeat: what ACT and the supervisor need to know (protocol v2): mode, Safe request, bus alarm, how this node sees the three
      tfc::Heartbeat hb;
      hb.mode = static_cast<uint8_t>(rep.mode);
      hb.safe_requested = rep.safe_request;
      hb.bus_alarm = rep.bus_alarm;
      for (unsigned n = 0; n < tfc::kNodes; ++n) {
        const unsigned bit = 1U << n;
        hb.node_state[n] = (rep.disabled_mask & bit) != 0U ? 3U : ((rep.probation_mask & bit) != 0U ? 2U : ((rep.latched_mask & bit) != 0U ? 1U : 0U));
      }
      hb.reset_count = static_cast<uint8_t>(resets.boots() > 255U ? 255U : resets.boots());
      tx_errors += send(tfc::pack_heartbeat(kNodeId, hb, seq)) ? 0U : 1U;
    }
    if (progress.end_of_frame(true)) {  // the one place the watchdog is serviced, and the one place KICK is raised
      wdt.feed();
      lines.kick(true);
    }

    for (unsigned i = 0; i < rep.command_count; ++i) {
      const tfc::CommandEvent& ce = rep.commands[i];
      printk("[frame %u] GROUND COMMAND %s%s %c: %s\n", k, (ce.flags & tfc::cmdflag::kArm) != 0U ? "ARM " : "", tfc::op_text(ce.op),
             'A' + static_cast<char>(ce.node), tfc::result_text(ce.result));
      if ((ce.flags & tfc::cmdflag::kCritical) != 0U) {
        printk("[frame %u] !!! CRITICAL: the last voting node was removed by operator command !!!\n", k);
      }
    }
    for (unsigned n = 0; n < tfc::kNodes; ++n) {
      const unsigned bit = 1U << n;
      if ((rep.probation_started & bit) != 0U) {
        printk("[frame %u] node %c ON PROBATION: shadow vote against the healthy nodes (strikes: %u)\n", k,
               'A' + static_cast<char>(n), static_cast<unsigned>(rep.strikes[n]));
      }
      if ((rep.probation_failed & bit) != 0U) {
        std::array<char, 96> why{};
        tfc::format_reasons(rep.reason[n], why.data(), why.size());
        printk("[frame %u] node %c FAILED PROBATION, back to latched: %s\n", k, 'A' + static_cast<char>(n),
               why.data());
      }
      if ((rep.newly_reintegrated & bit) != 0U) {
        printk("[frame %u] node %c REINTEGRATED into the vote (strikes on record: %u)\n", k,
               'A' + static_cast<char>(n), static_cast<unsigned>(rep.strikes[n]));
      }
      if ((rep.newly_disabled & bit) != 0U) {
        printk("[frame %u] node %c DISABLED for the run (strikes: %u)\n", k, 'A' + static_cast<char>(n),
               static_cast<unsigned>(rep.strikes[n]));
      }
    }
    for (unsigned n = 0; n < tfc::kNodes; ++n) {
      if (((rep.newly_seen >> n) & 1U) != 0U && n != kNodeId) {
        printk("[frame %u] node %c joined the bus\n", k, 'A' + static_cast<char>(n));
      }
      if (((rep.newly_latched >> n) & 1U) != 0U) {
        std::array<char, 96> why{};
        tfc::format_reasons(rep.reason[n], why.data(), why.size());
        printk("[frame %u] node %c LATCHED OUT: %s\n", k, 'A' + static_cast<char>(n), why.data());
      }
    }
    if (rep.integrity_mask != 0U) {
      printk("[frame %u] INTEGRITY FAULT repaired (mask 0x%x: 1 node state, 2 Safe flag, 4 configuration, 8 invariant)\n", k,
             static_cast<unsigned>(rep.integrity_mask));
    }
    if (rep.bus_alarm != last_bus_alarm) {
      printk("[frame %u] BUS ALARM %s: %u out-of-schedule frames in this 10 ms frame\n", k,
             rep.bus_alarm ? "RAISED" : "cleared", static_cast<unsigned>(rep.out_of_schedule_in_frame));
      last_bus_alarm = rep.bus_alarm;
    }
    if (rep.safe_request && !last_safe_request) {
      printk("[frame %u] SAFE REQUESTED: the two voting nodes disagree and nobody can be blamed; "
             "output held at the last good value\n", k);
    }
    last_safe_request = rep.safe_request;
    if (rep.mode != last_mode) {
      printk("[frame %u] MODE %s -> %s\n", k, mode_text(last_mode), mode_text(rep.mode));
      last_mode = rep.mode;
    }
    if (cycle % kStatusEveryFrames == 0U) {
      const tfc::Counters& cn = mgr.counters();
      printk("[frame %u] %s  A%c B%c C%c  | crc=%u seq=%u missing=%u vote=%u digest=%u stuck=%u oos=%u tx_err=%u imu_err=%u imu_stale=%u wdt_refused=%u bus_off=%u err_passive=%u sync_missed=%u\n",
             k, mode_text(rep.mode), node_state(rep, 0), node_state(rep, 1), node_state(rep, 2),
             static_cast<unsigned>(cn.crc_bad), static_cast<unsigned>(cn.seq_bad),
             static_cast<unsigned>(cn.missing), static_cast<unsigned>(cn.vote_disagreements),
             static_cast<unsigned>(cn.digest_flags), static_cast<unsigned>(cn.stuck_flags),
             static_cast<unsigned>(cn.out_of_schedule), tx_errors, static_cast<unsigned>(imu.errors()),
             static_cast<unsigned>(imu.stale()), static_cast<unsigned>(progress.refusals()),
             static_cast<unsigned>(atomic_get(&g_bus_off_events)), static_cast<unsigned>(atomic_get(&g_error_passive_events)),
             static_cast<unsigned>(sync_missed_total));
    }
  }
}
