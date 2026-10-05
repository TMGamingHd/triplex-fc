# Firmware (Zephyr)

> Status: **built, not run on a board** (reviewed 5 Oct 2026). Four Zephyr v4.4.2 applications consume `core/` and `supervisor/` as they are; only drivers, threads and ISRs belong here. They build for `native_sim` (real time, the flight bus on the host's `vcan0`), for the Nucleo and for the Pico, they pass
> the ELF check (no heap, exceptions, RTTI or vtables), and live tests run real instances against each other (`tools/bench/sil_triplex.sh --test`). What waits for the hardware is in `docs/STATUS.md` section 4.

```
firmware/
  west.yml       # pins Zephyr (v4.4.2) and the few modules we use
  env.sh         # source this: activates the venv, sets ZEPHYR_BASE and the SDK path
  app/           # the flight computer: node A, B or C from CONFIG_TFC_NODE_ID (nucleo_g474re, native_sim)
  act/           # the actuator node (nucleo_g474re, native_sim)
  pico/          # the Pico 2: platform driver and fault injector (rpi_pico2/rp2350a/m33)
  supervisor/    # the second Pico 2: the supervisor (rpi_pico2/rp2350a/m33)
  common/        # hardware seams shared by app and act (watchdog, reset cause, the actuator's lines)
```

| App | What it is | Design | Flash (board build) |
|---|---|---|---|
| `app` | The flight computer: SYNC following and takeover, acquisition, consensus, estimator, controller (with `TFC_FLIGHT_FUNCTION`), the fault manager, the launch sequence, resynchronisation | `docs/design/ARCHITECTURE.md` | 59.1 KB of 512 KB (60.5 KB with the flight function); RAM 10.0 KB of 128 KB |
| `act` | The actuator node: the vote, the output latch, Safe (including the hardware `SAFE` line), `FRAME` and `KICK` | `docs/design/ACT_LOGIC.md` | 39.5 KB of 512 KB |
| `pico` | The platform driver (two servos with their own limits) and the fault injector (relays) | `docs/design/PICO.md` | 58.8 KB of 4 MB |
| `supervisor` | The watchdog and reset ladder, boot sequencing, T-zero and the `T0` line, the clock record, the override sense lines, the `time` command | `docs/design/SUPERVISOR.md` | 69.1 KB of 4 MB |

## What a flight computer does every 10 ms frame
| t | Every node (the sync master sends SYNC; the others lock to it) |
|---|---|
| 0.0 ms | SYNC (master) or lock to it (follower); FRAME line rises, KICK falls |
| 0.5 ms | latches its IMU sample (one SPI burst on the board); FRAME line falls |
| 1.5 ms | broadcasts gyro + accel (nothing if the sensor could not be read) |
| 5.0 ms | broadcasts its command + estimator digest |
| 5.5 to 7.1 ms | once per resync period only: its state in four frames; the others' are collected at 8 ms and the mid-value adopted before the next frame |
| 7.0 ms | hands every frame received to `tfc::RedundancyManager` (decode, CRC/sequence, the votes, digest check, stuck detector, 3-of-5 latch); if the frame completed (every phase ran) services the watchdog and raises KICK; then sends the heartbeat (and the state share every 10 frames), samples the `T0` line, and prints what it decided |

A node that has heard no SYNC stays silent (ADR-025). The sync master is the lowest healthy node; a follower that misses SYNC for two frames takes over, in staggered windows.

## Console
A status line each second: `A+ B+ C+` = all voting; `X` latched out; `p` on probation; `w` resting as WARM; `D` disabled for the run; `?` no good data; then the counters (`oos=` frames on IDs outside the schedule, `imu_err`, `wdt_refused`, `resync=adopted/skipped`, `far=`, `wcet_step`, `wcet_vote`, `wcet_frame` in microseconds, `mission=`, `ready=`, and `phase=`/`warm=` last).
Event lines whenever a node joins, latches out (with the reason in words), goes on probation, fails probation, is reintegrated or disabled; a ground command is applied (`GROUND COMMAND <op> <node>: accepted` or `refused: <why>`); the mode changes; the bus alarm is raised or cleared; a Safe request is raised; the countdown runs (`COUNTDOWN`, `T-n s`, `T0 LINE`, `T-ZERO`, `SCRUB`, `LAUNCH REFUSED, no-go: ...`); a resync adopts or skips; a restart takes the others' state (`STATE RESTORED`); a reset loop silences the node (`RESET LOOP`). The boot line prints the release (`release 0x....`).

## Operator commands, reintegration
Commands arrive as authenticated CAN ground frames (`0x510`; SipHash tag with `CONFIG_TFC_GROUND_KEY`, default the public bench key, plus a counter that rejects replays): `reintegrate`, `disable`, `clear-disabled`, `clear-safe`, `launch`, `scrub`, `phase`, `noop`, `warm` (`docs/design/PROTOCOL.md`). Dangerous ones need an ARM first.
Send one with `python3 -m tfc_peers command reintegrate B`, or script it at an exact frame with `run --follow-sync --command 450:reintegrate:B`.
A latched node is out of the vote until an operator readmits it (after the dwell it goes on probation and 100 agreeing frames readmit it; `docs/decisions/DECISIONS.md` ADR-010); `CONFIG_TFC_AUTO_REINTEGRATE=y` also readmits a first, transient-looking latch.

## Configuration reference
**`app`** (`firmware/app/Kconfig`; the board and `native_sim` share them)
| Option | Default | What it does |
|---|---|---|
| `TFC_NODE_ID` | 0 | 0 = A, 1 = B, 2 = C (the two strap pins PC2 and PC3 are only checked: a mismatch prints a warning) |
| `TFC_FLIGHT_FUNCTION` | n | Compute the command with the flight function (consensus, estimator, controller) instead of the scripted function of the frame number |
| `TFC_SIM_BUS_IMU` | n | Take the sensor input from the simulator's frames on the bus |
| `TFC_LAUNCH_SEQUENCE` | n | The pad calibration, the mission frame, `launch` and `scrub`, the go/no-go, the `T0` line |
| `TFC_PHASES` | n | The mission phase and the WARM role (ADR-023) |
| `TFC_SENSOR_SPLIT` | with the flight function | Judge each IMU apart from its computer (ADR-020 case 1) |
| `TFC_RESYNC_PERIOD` / `TFC_RESYNC_GROUP` | 100 / 0x07 | State resynchronisation period in frames (0 off) and who takes part (bit n = node n; leave the diverse computer out) |
| `TFC_RELEASE_ID` | 0 | The release the heartbeat reports (0: the first four hex digits of the git hash) |
| `TFC_GROUND_KEY` | the public bench key | 32 hex digits; provision your own before any real use |
| `TFC_AUTO_REINTEGRATE` | n | Readmit a first, transient-looking latch automatically |
| `TFC_SYNC_STAGGER_US`, `TFC_VOTE_US` | 500, 7000 | The SYNC wait windows and the vote time (the host board file lengthens both: the native CAN driver polls every millisecond) |
| `TFC_WATCHDOG_TIMEOUT_MS`, `TFC_QUARANTINE_ON_RESET_LOOP`, `TFC_RESET_LOOP_BOOTS`, `TFC_SHORT_BOOT_S` | 100, y, 3, 60 | The watchdog and the reset-loop rule (FDIR-038, FDIR-042) |

**Test knobs** (bench aids for the procedures and the live tests; **never set in a flight build**; all default to 0 = off)
| Option | Used by | What it does |
|---|---|---|
| `TFC_TEST_DROP_PEERS_FIRST`, `_FRAMES` | live resync test | This computer withholds every sensor frame from its flight function for a window of frames |
| `TFC_TEST_HANG_AT_FRAME` | `P-S2-03` | From this frame the KICK stops while everything else goes on |
| `TFC_TEST_CMD_OFFSET_MDEG` | live release test, `P-REL-01` | Milli-degrees added to this computer's pitch command |
| `TFC_TEST_GYRO_BIAS_MDPS`, `TFC_TEST_GYRO_DRIFT_MDPS_PER_FRAME` | live split test | A gyro bias or a cumulative drift in this computer's IMU |
| `TFC_TEST_CLOCK_PPM` | `P-S2-03` | The sync master's frame period is this many ppm long |
| `TFC_TEST_T0_AT_FRAMES_TO_ZERO` | live T0 test | The `T0` line reads high from this many frames before T-zero |

**`act`**: `TFC_GROUND_KEY`, `TFC_SYNC_STAGGER_US`, `TFC_ACT_VOTE_US` (6500), `TFC_LAUNCH_SEQUENCE` (n: take pad or flight from SYNC's mission frame, so that Safe on the pad goes to neutral at once; set it with the flight computers'), `TFC_WATCHDOG_TIMEOUT_MS`, `TFC_RESET_LOOP_BOOTS`, `TFC_SHORT_BOOT_S`; test knob `TFC_TEST_SAFE_LINE_AT_FRAME` (the hardware SAFE line reads as asserted from this SYNC frame).
**`pico`**: `TFC_USB_VID`/`PID`, `TFC_LINK_TIMEOUT_MS` (2000: nothing from the PC for this long releases every relay), the servo calibration (`TFC_SERVO_NEUTRAL_US`, `TFC_SERVO_US_PER_DEG_X100`, `_X_SIGN`, `_Y_SIGN`, `_X_TRIM_X100`, `_Y_TRIM_X100`), `TFC_PLATFORM_LIMIT_DEG` (45), `TFC_PLATFORM_RATE_DPS` (300), `TFC_WATCHDOG_TIMEOUT_MS`.
**`supervisor`**: `TFC_USB_VID`/`PID`, `TFC_WATCHDOG_TIMEOUT_MS`, `TFC_OVERRIDE_LINES_FITTED` (which override sense lines are wired, 0 = none).

## The images, and how to build them
`tools/bench/sil_triplex.sh` builds and runs the `native_sim` images the live tests use (it sources `firmware/env.sh`):

| Command | Images | For |
|---|---|---|
| `--build` | `build/triplex_a,b,c`, `build/act_native` | Three scripted flight computers and ACT: the triplex test |
| `--build --flight` | `build/flight_a,b,c` | The real flight function |
| `--build --flight --sim-imu` | `build/loop_a,b,c` | The closed loop with `tfc_simd` |
| `--build --launch` | `build/launch_a,b,c` | The launch sequence |
| `--build-drop`, `--build-bias`, `--build-mixed`, `--build-t0`, `--build-phases`, `--build-safeline` | `flight_b_drop`, `flight_b_bias`, `mixed_*`, `launch_t0_*`, `phases_*`, `act_safeline` | The resync, split, mixed-release, T0, phases and hardware-Safe live tests |
| `--test` | | Runs the live tests (needs `vcan0`: `sim/scripts/setup_vcan.sh`) |

For the boards: `west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re` (add `-- -DCONFIG_TFC_FLIGHT_FUNCTION=y` for the flight function), `firmware/act` likewise, and `west build -p always -b rpi_pico2/rp2350a/m33 firmware/pico -d build/pico` or `firmware/supervisor -d build/supervisor`; then `tools/check_elf.sh` on each `zephyr.elf`.

## On the board (nucleo_g474re)
**Not yet run on hardware.** What the board build adds, all chosen by the devicetree (`boards/nucleo_g474re.overlay`) so that `native_sim` keeps its simulated stand-ins (`app/src/hw.hpp`):

| Seam | Board | `native_sim` |
|---|---|---|
| IMU | ISM330DHCX on SPI2 through `core/include/tfc/ism330dhcx.hpp` (identity check, reset, configuration with read-back, one burst read per frame; +-16 g, +-500 dps, 833 Hz) | the simulated IMU |
| Lines to the supervisor | FRAME, KICK, ADOPT, T0, two node-id straps | none |
| Watchdog | the independent watchdog, started at frame 0, serviced only at the end of a completed frame | none |
| Reset record | reset count and cause in no-init RAM; a reset loop silences the node | power-on every run |
| CAN | FDCAN1 from the board file, 1 Mbit/s, receive timestamps enabled | the host's `vcan0` |

**Pins** (looked up in the morpho-connector map that Zephyr takes from ST's data; **check each against ST's user manual (UM2505) and the board in hand before wiring**):

| Signal | Pin | Connector | Notes |
|---|---|---|---|
| FDCAN1 RX / TX | PA11 / PA12 | CN10 14 / 12 | from the board file |
| IMU SPI2 SCK / MISO / MOSI / CS | PB13 / PB14 / PB15 / PB12 | CN10 30 / 28 / 26 / 16 | CS is a GPIO; 4 MHz, SPI mode 3 |
| Backup IMU SPI3 (reserved, disabled) | PC10 / PC11 / PC12 / PA15 | CN7 1 / 2 / 3 / 17 | the ring re-homing stretch (ADR-020, outside v1) |
| FRAME (node to supervisor) | PC8 | CN10 2 | pulse at the start of a frame |
| KICK (node to supervisor) | PC9 | CN10 1 | pulse at the end of a completed frame only |
| ADOPT (supervisor to node) | PC6 | CN10 4 | input, pull-down (the stretch) |
| T0 (supervisor to node) | PC5 | CN10 6 | input, pull-down; high from the supervisor's T-zero |
| Node-id straps bit 0 / bit 1 | PC2 / PC3 | CN7 35 / 37 | to ground reads as 1 |
| Reset from the supervisor | NRST | the board's reset pin | open-drain from the supervisor, no firmware |
| ACT: FRAME / KICK / SAFE | PC8 / PC9 / PC6 | CN10 2 / 1 / 4 | `act/boards/nucleo_g474re.overlay`; SAFE is an input with a pull-down |

Left alone on purpose: PA5 (the LED), PA2/PA3 (the ST-LINK serial port), PA13/PA14 (SWD), PC13 (the button), PC4, PB8/PB9. The supervisor's pins are in `supervisor/boards/rpi_pico2_rp2350a_m33.overlay` (GP2 to GP5 `FRAME`, GP6 to GP9 `KICK`, GP10 to GP13 `NRST`, GP14 to GP17 `PWR`, GP18 `SAFE`, GP19 `T0`, GP20/GP21 I2C, GP22, GP26 to GP28, GP0 and GP1 the override sense inputs).

**What to know before the first run.**
- Without `TFC_FLIGHT_FUNCTION` the command is a fixed function of the frame number (`app/src/sim_imu.hpp`), so the digest can be checked against a golden run; it does not come from the IMU. The gyro and accelerometer frames are the real sensor's. Run **without the virtual peers** on the bench (they feel the simulated motion, not the bench's, so the vote would correctly disagree), with the USB-CAN adapter as the second node (`procedures/P-S1-01`).
- The sensor's self-test is not called: the datasheet's limits are not set (`core/include/tfc/ism330dhcx.hpp`, `docs/design/FUTURE_WORK.md` 1.2).
- A sensor that cannot be read costs the frame its gyro and accelerometer frames (peers see a missing sample); `imu_err` counts them, `imu_stale` counts reads that returned the previous sample.
- The console is slow on the board (115200 baud): a status line takes several milliseconds. The watchdog is serviced before anything is printed, and its default timeout is ten frames for that reason; measure the frame time on the board and tighten it (the requirement proposes three frames).

## One-time setup (Ubuntu)
```bash
sudo apt install -y ninja-build device-tree-compiler gperf ccache

# 1. Zephyr SDK (minimal + ARM toolchain + host tools, ~230 MB)
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz -C ~
~/zephyr-sdk-1.0.1/setup.sh -t arm-zephyr-eabi -h -c

# 2. west + Zephyr, from the repo root. The workspace is the repo root, so zephyr/ and
#    modules/ land inside it (git-ignored, ~1.8 GB). The venv lives next to the repo.
python3 -m venv ../.venv && ../.venv/bin/pip install west
../.venv/bin/west init -l firmware
../.venv/bin/west update --narrow -o=--depth=1
../.venv/bin/pip install -r zephyr/scripts/requirements-base.txt
```

## Every session
```bash
cd ~/SpaceX/triplex-fc
. firmware/env.sh            # once per terminal; puts `west` on PATH
```

## Run FC-A against the virtual peers (no hardware)
```bash
sim/scripts/setup_vcan.sh                                   # once per boot (sudo): creates vcan0
west build -p always -b native_sim/native/64 firmware/app -d build/native_sim

# terminal 1: the flight computer (runs in real time; -stop_at=<s> ends it, Ctrl+C also works)
build/native_sim/zephyr/zephyr.exe -stop_at=12

# terminal 2, within a second or two: fake FC-B and FC-C that follow FC-A's SYNC; B gets a bias at frame 400.
# --frames 0 = keep sending until Ctrl+C (a fixed count would stop the peers early and FC-A would then report them missing)
cd sim && python3 -m tfc_peers run --follow-sync --nodes B,C --frames 0 --fault B:bias:start=400,mag=3
# optional terminal 3, from any directory: watch the bus   ~/SpaceX/triplex-fc/sim/tfc-peers listen
```
Expected FC-A console (frame numbers are absolute; the peers use SYNC's frame number):
```
[frame 0] MODE TRIPLEX -> SIMPLEX
[frame 44] node B joined the bus
[frame 44] node C joined the bus
[frame 44] MODE SIMPLEX -> TRIPLEX
[frame 100] TRIPLEX  A+ B+ C+  | crc=0 seq=0 missing=0 vote=0 digest=0 stuck=0 tx_err=0
[frame 402] node B LATCHED OUT: vote disagreement
[frame 402] MODE TRIPLEX -> DUPLEX
...
[frame 796] node C LATCHED OUT: frame missing      <- only after you press Ctrl+C on the peers
```
Try any fault from `python3 -m tfc_peers faults`. Without `--follow-sync` the peers free-run on their own
clock and will not line up with FC-A's frames. Automated version: `cd sim && python3 -m unittest tests.test_live_fc -v`
(skipped unless `vcan0` exists and the binary is built).

## Build for the real board (`nucleo_g474re`)
```bash
west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re
```
Flashing needs the board plugged in over USB (ST-LINK): `west flash -d build/nucleo_g474re`.
Serial console: `picocom -b 115200 /dev/ttyACM0` (first check `dmesg` for the device name).
The board file already puts FDCAN1 on PA11/PA12; the bring-up checklist still has to confirm that against the wiring.

## Gotchas
- **Include order:** in any `.cpp` that uses `core/`, include the `tfc/*.hpp` headers (and so the
  C++ standard library) *before* any `<zephyr/...>` header. Zephyr defines an `__unused` macro that
  breaks a glibc header (`struct_mutex.h`) on the native_sim host build.
- `core/` needs the full libstdc++ (`CONFIG_REQUIRES_FULL_LIBCPP=y`); Zephyr's default minimal C++
  library has no `<array>`/`<cmath>`. Exceptions and RTTI stay off.
- **Kernel tick:** the default is 100 ticks/s (10 ms), which rounds every sleep up to 10 ms and wrecked the
  schedule (60 ms frames). `CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000` is required.
- **Catch-all filter:** besides the three schedule-slot filters there is a catch-all filter into a second queue, only so stray traffic (a babbling node) reaches the manager's out-of-schedule counter and bus alarm (ADR-009). On the target that makes every frame an interrupt; evaluate hardware filters plus the controller's lost-message counters at bring-up.
- The host's native CAN driver reports its own transmissions back as TX confirmations, so own samples are fed
  to the manager directly and received frames with this node's own IDs are dropped.
- `printk` has no float support; print fixed-point integers, or enable picolibc float I/O deliberately.
