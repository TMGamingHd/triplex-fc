// SPDX-License-Identifier: MIT
// FC-A: the sync master and one of three flight computers. Every 10 ms major frame it
//   t = 0.0 ms  broadcasts SYNC (frame number)
//   t = 1.5 ms  samples its (simulated) IMU and broadcasts gyro + accel
//   t = 5.0 ms  broadcasts its command + estimator digest
//   t = 7.0 ms  hands everything received to tfc::RedundancyManager, which votes, cross-checks
//               and runs FDIR; latch events and a once-a-second status line go to the console
// (schedule: docs/ARCHITECTURE.md section 3). On native_sim the bus is the host's vcan0 and the
// peers B and C are the virtual peers (sim/); on the Nucleo it is FDCAN1.
//
// Include core/ (and so the C++ standard library) BEFORE Zephyr headers: Zephyr defines an
// `__unused` macro that breaks a glibc header (struct_mutex.h) on the native_sim host build.
#include <array>
#include <cstdint>
#include <cstring>

#include "sim_imu.hpp"
#include "tfc/protocol.hpp"
#include "tfc/redundancy.hpp"

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

namespace {

constexpr unsigned kNodeId = CONFIG_TFC_NODE_ID;
BUILD_ASSERT(kNodeId == 0, "this stage implements node A, the sync master; B and C need SYNC following");

constexpr int64_t kGyroUs = 1500;
constexpr int64_t kCmdUs = 5000;
constexpr int64_t kVoteUs = 7000;
constexpr uint32_t kStatusEveryFrames = 100;
// Peers may boot after this node; judge a never-seen peer only after 5 s (RedundancyConfig).
constexpr uint32_t kStartupGraceFrames = 500;

const struct device* const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
CAN_MSGQ_DEFINE(rx_msgq, 32);      // the three schedule slots (gyro, accel, command of every node)
CAN_MSGQ_DEFINE(rx_all_msgq, 64);  // everything: only used to see what is NOT in the schedule (babbling)

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

}  // namespace

int main() {
  if (!bus_init()) {
    return 1;
  }
  tfc::RedundancyConfig cfg;
  cfg.startup_grace_frames = kStartupGraceFrames;
  cfg.policy = IS_ENABLED(CONFIG_TFC_AUTO_REINTEGRATE) ? tfc::ReintegrationPolicy::AutoTransient
                                                       : tfc::ReintegrationPolicy::Manual;
  tfc::RedundancyManager mgr(cfg);
  if (mgr.config_errors() != 0U) {  // a bad configuration is replaced by defaults, never run silently
    printk("CONFIG ERROR: invalid fields (mask 0x%x) replaced by defaults\n", static_cast<unsigned>(mgr.config_errors()));
  }
  fc::sim::Noise noise(0x1234U + kNodeId);
  constexpr int64_t kPeriodUs = fc::sim::kFrameUs;
  const int64_t period = k_us_to_ticks_ceil64(kPeriodUs);
  const int64_t start = k_uptime_ticks() + k_ms_to_ticks_ceil64(200);
  tfc::Mode last_mode = tfc::Mode::Triplex;
  bool last_bus_alarm = false;
  bool last_safe_request = false;
  uint32_t tx_errors = 0;

  printk("FC-A (node %u): sync master, 100 Hz frame loop. Waiting for peers B and C on the bus.\n", kNodeId);
  printk("status: '+' voting, 'X' latched out, 'p' on probation, 'D' disabled, '?' no good data this frame\n");

  for (uint32_t k = 0;; ++k) {
    const int64_t base = start + static_cast<int64_t>(k) * period;
    k_sleep(K_TIMEOUT_ABS_TICKS(base));
    const uint8_t seq = static_cast<uint8_t>(k);
    mgr.begin_frame();
    if (k == 0U) {
      k_msgq_purge(&rx_msgq);  // anything queued before the first SYNC belongs to no frame
      k_msgq_purge(&rx_all_msgq);
    }
    tx_errors += send(tfc::pack_sync(k, seq)) ? 0U : 1U;

    // ---- t = 1.5 ms: own IMU sample ----
    sleep_until_us(base, kGyroUs);
    const fc::sim::Truth tr = fc::sim::truth(k);
    tfc::Vec3 gyro{};
    tfc::Vec3 accel{};
    for (unsigned i = 0; i < 3U; ++i) {
      gyro.v[i] = static_cast<float>(tr.gyro[i]) + 0.17F * noise.uniform();
      accel.v[i] = static_cast<float>(tr.accel[i]) + 0.0035F * noise.uniform();
    }
    const tfc::Frame g = tfc::pack_gyro(kNodeId, gyro, seq);
    const tfc::Frame a = tfc::pack_accel(kNodeId, accel, seq);
    mgr.on_frame(g);
    mgr.on_frame(a);
    tx_errors += send(g) ? 0U : 1U;
    tx_errors += send(a) ? 0U : 1U;

    // ---- t = 5.0 ms: command + digest ----
    sleep_until_us(base, kCmdUs);
    const tfc::Frame c = tfc::pack_cmd(kNodeId, fc::sim::command(tr, k), seq);
    mgr.on_frame(c);
    tx_errors += send(c) ? 0U : 1U;

    // ---- t = 7.0 ms: everything from the peers has arrived; vote ----
    sleep_until_us(base, kVoteUs);
    can_frame cf;
    while (k_msgq_get(&rx_msgq, &cf, K_NO_WAIT) == 0) {
      tfc::Frame f;
      f.id = cf.id;
      f.len = static_cast<uint8_t>(can_dlc_to_bytes(cf.dlc));
      std::memcpy(f.data.data(), cf.data, 8);
      if (!is_own(f.id)) {
        mgr.on_frame(f);
      }
    }
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

    for (unsigned i = 0; i < rep.command_count; ++i) {
      printk("[frame %u] GROUND COMMAND %s %c: %s\n", k, tfc::op_text(rep.commands[i].op),
             'A' + static_cast<char>(rep.commands[i].node), tfc::result_text(rep.commands[i].result));
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
    if (k % kStatusEveryFrames == 0U) {
      const tfc::Counters& cn = mgr.counters();
      printk("[frame %u] %s  A%c B%c C%c  | crc=%u seq=%u missing=%u vote=%u digest=%u stuck=%u oos=%u tx_err=%u\n",
             k, mode_text(rep.mode), node_state(rep, 0), node_state(rep, 1), node_state(rep, 2),
             static_cast<unsigned>(cn.crc_bad), static_cast<unsigned>(cn.seq_bad),
             static_cast<unsigned>(cn.missing), static_cast<unsigned>(cn.vote_disagreements),
             static_cast<unsigned>(cn.digest_flags), static_cast<unsigned>(cn.stuck_flags),
             static_cast<unsigned>(cn.out_of_schedule), tx_errors);
    }
  }
}
