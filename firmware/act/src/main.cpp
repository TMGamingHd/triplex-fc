// SPDX-License-Identifier: MIT
// The actuator node (ADR-023, ADR-024, docs/ACT_LOGIC.md). It has no servo behind it: the platform is driven by the Pico, and ACT's job is the vote.
// Every 10 ms frame it
//   t = 0.0 ms  hears SYNC from the sync master and locks its frame to it (it only follows: it is never the master)
//   t = 6.5 ms  takes the three flight computers' commands, votes, judges the nodes, runs its mode (Standby, Nominal, Safe) and sends the voted
//               output and its status on 0x300 (docs/PROTOCOL.md)
// From the flight computers' heartbeats it reads their Safe request; ground commands (authenticated, ARMed) lift its Safe and readmit nodes it excluded.
// After a reset that is not a power-on it starts in Safe, holding the output it had, from a record kept in no-init RAM.
// Not here yet: the hardware SAFE line from the supervisor (docs/SUPERVISOR.md), which waits for the supervisor.
//
// Include core/ (and so the C++ standard library) BEFORE Zephyr headers: Zephyr defines an `__unused` macro that breaks a glibc header.
#include <array>
#include <cstdint>
#include <cstring>

#include "tfc/act.hpp"
#include "tfc/act_ground.hpp"
#include "tfc/protocol.hpp"
#include "tfc/resetlog.hpp"
#include "tfc/sync_clock.hpp"

#include "hw_common.hpp"

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

namespace {

constexpr unsigned kActNode = 3U;  // ACT's place after A, B and C
constexpr int64_t kPeriodUs = 10000;
constexpr int64_t kVoteUs = CONFIG_TFC_ACT_VOTE_US;
constexpr uint32_t kStatusEveryFrames = 100U;

const struct device* const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// Survive a reset (no-init RAM, intact because the records have no constructors).
tfc::ResetRecord g_reset_record __noinit;
tfc::ActRecord g_act_record __noinit;

CAN_MSGQ_DEFINE(rx_cmd_msgq, 16);     // the flight computers' commands (0x200 to 0x203)
CAN_MSGQ_DEFINE(rx_sync_msgq, 4);     // SYNC
CAN_MSGQ_DEFINE(rx_ground_msgq, 8);   // ground commands
CAN_MSGQ_DEFINE(rx_hb_msgq, 16);      // heartbeats (0x400 to 0x403)

tfc::Frame to_frame(const can_frame& cf) {
  tfc::Frame f;
  f.id = cf.id;
  f.len = static_cast<uint8_t>(can_dlc_to_bytes(cf.dlc));
  std::memcpy(f.data.data(), cf.data, 8);
  return f;
}

bool send(const tfc::Frame& f) {
  can_frame cf{};
  cf.id = f.id;
  cf.dlc = f.len;
  std::memcpy(cf.data, f.data.data(), 8);
  return can_send(can_dev, &cf, K_MSEC(2), nullptr, nullptr) == 0;
}

bool add_filter(struct k_msgq* q, uint32_t id, uint32_t mask) {
  const can_filter filter{.id = id, .mask = mask, .flags = 0U};
  if (can_add_rx_filter_msgq(can_dev, q, &filter) < 0) {
    printk("cannot add CAN rx filter for id 0x%03x\n", static_cast<unsigned>(id));
    return false;
  }
  return true;
}

bool bus_init() {
  if (!device_is_ready(can_dev)) {
    printk("CAN device not ready\n");
    return false;
  }
  const int br = can_set_bitrate(can_dev, 1000000);
  if (br != 0 && br != -ENOTSUP) {
    printk("can_set_bitrate: %d (continuing)\n", br);
  }
  if (!add_filter(&rx_cmd_msgq, tfc::id::kCmdBase, 0x7FCU) || !add_filter(&rx_sync_msgq, tfc::id::kSync, 0x7FFU) ||
      !add_filter(&rx_ground_msgq, tfc::id::kGround, 0x7FFU) || !add_filter(&rx_hb_msgq, tfc::id::kHeartbeat, 0x7FCU)) {
    return false;
  }
  const int rc = can_start(can_dev);
  if (rc != 0) {
    printk("can_start: %d\n", rc);
    return false;
  }
  return true;
}

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
      return false;
    }
    out[i / 2U] = static_cast<uint8_t>((i % 2U == 0U) ? (v << 4U) : (out[i / 2U] | v));
  }
  if (hex[32] != '\0') {
    return false;
  }
  key = out;
  return true;
}

const char* mode_text(tfc::ActMode m) {
  switch (m) {
    case tfc::ActMode::Standby: return "STANDBY";
    case tfc::ActMode::Nominal: return "NOMINAL";
    case tfc::ActMode::Safe: return "SAFE";
  }
  return "?";
}

const char* phase_text(tfc::SafePhase p) {
  switch (p) {
    case tfc::SafePhase::None: return "-";
    case tfc::SafePhase::Hold: return "hold";
    case tfc::SafePhase::Ramp: return "ramp";
    case tfc::SafePhase::Neutral: return "neutral";
  }
  return "?";
}

const char* cause_text(tfc::SafeCause c) {
  switch (c) {
    case tfc::SafeCause::None: return "none";
    case tfc::SafeCause::LostVotes: return "lost votes";
    case tfc::SafeCause::FcRequest: return "the flight computers' request";
    case tfc::SafeCause::HardwareLine: return "the SAFE line";
    case tfc::SafeCause::Reset: return "a reset";
  }
  return "?";
}

const char* ground_text(tfc::ActGroundResult r) {
  switch (r) {
    case tfc::ActGroundResult::None: return "not a ground frame";
    case tfc::ActGroundResult::Bad: return "damaged";
    case tfc::ActGroundResult::Unauthentic: return "unauthentic";
    case tfc::ActGroundResult::Replayed: return "replayed";
    case tfc::ActGroundResult::NotForAct: return "not for ACT";
    case tfc::ActGroundResult::NeedsArm: return "needs an ARM first";
    case tfc::ActGroundResult::Armed: return "ARMed";
    case tfc::ActGroundResult::ClearSafe: return "clear-safe";
    case tfc::ActGroundResult::ClearExclusions: return "readmit excluded nodes";
  }
  return "?";
}

}  // namespace

int main() {
  if (!bus_init()) {
    return 1;
  }
  tfc::ActGroundConfig gcfg;
  if (!parse_key(CONFIG_TFC_GROUND_KEY, gcfg.key)) {
    printk("CONFIG ERROR: TFC_GROUND_KEY must be exactly 32 hex digits; using the PUBLIC bench key\n");
  }
  tfc::ActGround ground(gcfg);
  tfc::ActLogic act;
  tfc::HeartbeatMonitor heartbeats;

  fc::hw::Watchdog wdt;
  tfc::ResetPolicy reset_policy;
  reset_policy.short_boot_frames = static_cast<uint32_t>(CONFIG_TFC_SHORT_BOOT_S) * 100U;
  reset_policy.loop_boots = static_cast<uint32_t>(CONFIG_TFC_RESET_LOOP_BOOTS);
  tfc::ResetLog resets(g_reset_record, reset_policy);
  const tfc::ResetCause cause = fc::hw::reset_cause();
  resets.boot(cause);
  act.boot(cause, g_act_record);
  printk("ACT boot %u since power-on, last reset cause %u, starts in %s%s\n", static_cast<unsigned>(resets.boots()), static_cast<unsigned>(resets.last_cause()),
         mode_text(act.output().mode), act.output().mode == tfc::ActMode::Safe ? " (after a reset: holding the stored output, then ramping to neutral)" : "");

  const int64_t period = k_us_to_ticks_ceil64(kPeriodUs);
  const int64_t window = k_us_to_ticks_ceil64(tfc::sync_window_us(kActNode - 1U, CONFIG_TFC_SYNC_STAGGER_US));  // as long as node C's: ACT never takes over
  tfc::SyncClock sync_clock(tfc::SyncStart::Observer);
  int64_t nominal = k_uptime_ticks() + k_ms_to_ticks_ceil64(200);
  tfc::ActMode last_mode = act.output().mode;
  tfc::SafePhase last_phase = act.output().phase;
  uint8_t last_excluded = 0U;
  uint32_t tx_errors = 0U;

  printk("ACT: following SYNC, voting at %d us. Silent until the first SYNC.\n", static_cast<int>(kVoteUs));

  for (uint32_t cycle = 0;; ++cycle) {
    // ---- the start of the frame: wait for SYNC, up to this node's window ----
    bool heard = false;
    uint32_t heard_number = 0U;
    int64_t base = nominal;
    can_frame sf;
    while (k_msgq_get(&rx_sync_msgq, &sf, K_TIMEOUT_ABS_TICKS(nominal + window)) == 0) {
      const tfc::DecodedSync d = tfc::unpack_sync(to_frame(sf));
      if (d.ok) {
        heard = true;
        heard_number = d.frame_no;
        base = k_uptime_ticks();
        break;
      }
    }
    if (!heard) {
      base = k_uptime_ticks();
    }
    const tfc::SyncTick tick = sync_clock.cycle(heard, heard_number);
    const uint32_t k = tick.frame;
    nominal = base + period;
    if (cycle == 0U && !wdt.start(static_cast<uint32_t>(CONFIG_TFC_WATCHDOG_TIMEOUT_MS)) && DT_HAS_ALIAS(watchdog0)) {
      printk("WATCHDOG: could not be started\n");
    }
    if (!tick.synced) {
      wdt.feed();
      if (cycle % kStatusEveryFrames == 0U) {
        printk("[cycle %u] waiting for SYNC\n", cycle);
      }
      continue;
    }
    if (cycle % kStatusEveryFrames == 0U) {
      resets.running(cycle);
    }
    const uint8_t seq = static_cast<uint8_t>(k);
    act.begin_frame();
    heartbeats.begin_frame();
    ground.tick();
    if (cycle == 0U) {
      k_msgq_purge(&rx_cmd_msgq);
      k_msgq_purge(&rx_hb_msgq);
      k_msgq_purge(&rx_ground_msgq);
    }

    // ---- t = 6.5 ms: the commands are in; vote and publish ----
    k_sleep(K_TIMEOUT_ABS_TICKS(base + k_us_to_ticks_ceil64(kVoteUs)));
    can_frame cf;
    while (k_msgq_get(&rx_cmd_msgq, &cf, K_NO_WAIT) == 0) {
      (void)act.on_frame(to_frame(cf));
    }
    while (k_msgq_get(&rx_hb_msgq, &cf, K_NO_WAIT) == 0) {
      (void)heartbeats.on_frame(to_frame(cf));
    }
    while (k_msgq_get(&rx_ground_msgq, &cf, K_NO_WAIT) == 0) {
      const tfc::ActGroundResult r = ground.on_frame(to_frame(cf));
      if (r == tfc::ActGroundResult::ClearSafe) {
        printk("[frame %u] GROUND clear-safe: %s\n", k, act.clear_safe() ? "ACT leaves Safe" : "REFUSED (the votes have not been good for long enough, or a request is still active)");
      } else if (r == tfc::ActGroundResult::ClearExclusions) {
        act.clear_exclusions();
        printk("[frame %u] GROUND reintegrate: ACT readmits the nodes it excluded (they must agree again)\n", k);
      } else if (r != tfc::ActGroundResult::None) {
        printk("[frame %u] GROUND command %s\n", k, ground_text(r));
      }
    }
    act.safe_request(heartbeats.safe_requested(act.output().excluded_nodes));
    act.hardware_safe(false);  // the supervisor's SAFE line is not wired yet
    const tfc::ActOutput& out = act.end_frame();
    tx_errors += send(tfc::pack_act_out(tfc::to_act_frame(out), seq)) ? 0U : 1U;
    g_act_record = act.record();
    wdt.feed();

    if (out.mode != last_mode || out.phase != last_phase) {
      printk("[frame %u] ACT %s -> %s (%s), cause: %s\n", k, mode_text(last_mode), mode_text(out.mode), phase_text(out.phase), cause_text(out.cause));
      last_mode = out.mode;
      last_phase = out.phase;
    }
    for (unsigned n = 0; n < tfc::kChannels; ++n) {
      const unsigned bit = 1U << n;
      if (((out.excluded_nodes ^ last_excluded) & bit) != 0U) {
        printk("[frame %u] ACT %s node %c\n", k, (out.excluded_nodes & bit) != 0U ? "EXCLUDED" : "readmitted", 'A' + static_cast<char>(n));
      }
    }
    last_excluded = out.excluded_nodes;
    if (cycle % kStatusEveryFrames == 0U) {
      printk("[frame %u] %s %s pitch %.3f yaw %.3f  voted %u excluded %u held %u  | held_frames %u safe_entries %u refused_clears %u tx_err %u\n", k, mode_text(out.mode),
             phase_text(out.phase), static_cast<double>(out.pitch_deg), static_cast<double>(out.yaw_deg), static_cast<unsigned>(out.voted_nodes),
             static_cast<unsigned>(out.excluded_nodes), out.held ? 1U : 0U, static_cast<unsigned>(act.held_frames()), static_cast<unsigned>(act.safe_entries()),
             static_cast<unsigned>(act.refused_clears()), static_cast<unsigned>(tx_errors));
    }
  }
}
