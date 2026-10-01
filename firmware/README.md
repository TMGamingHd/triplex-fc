# Firmware (Zephyr)

Zephyr v4.4.2 app that consumes `core/` as-is. Only drivers, threads and ISRs belong here.
Status: **FC-A runs the 100 Hz frame loop** as sync master against virtual peers on `vcan0` (`native_sim`),
and the same app builds for `nucleo_g474re` (FDCAN1 from the board file; not yet run on hardware, and it
still uses a simulated IMU until the ISM330DHCX driver lands).

```
firmware/
  west.yml       # pins Zephyr (v4.4.2) and the few modules we use
  env.sh         # source this: activates the venv, sets ZEPHYR_BASE and the SDK path
  app/
    Kconfig                  # TFC_NODE_ID (this stage: 0 = A, the sync master)
    prj.conf                 # C++17, no exceptions/RTTI, CAN, 100 us kernel tick
    boards/native_sim_native_64.{conf,overlay}   # real-time clock; bus = host vcan0
    src/main.cpp             # the frame loop (below)
    src/sim_imu.hpp          # simulated IMU: same motion the peers feel (replace with the ISM330 driver)
```

## What the app does (every 10 ms major frame)
| t | FC-A |
|---|---|
| 0.0 ms | broadcasts **SYNC** (32-bit frame number) |
| 1.5 ms | samples its IMU, broadcasts gyro + accel |
| 5.0 ms | broadcasts its command + estimator digest |
| 7.0 ms | hands every frame received to `tfc::RedundancyManager` (decode, CRC/sequence, 8-channel vote, digest check, stuck detector, 3-of-5 latch) and prints what it decided |

Console: a status line each second (`A+ B+ C+` = all voting; `X` = latched out; `?` = no good data; `oos=` counts frames on IDs outside the schedule), and an
event line whenever a node joins, latches out (with the reason in words), the mode changes, the bus alarm is raised or cleared (`BUS ALARM`, 3+ stray frames per 10 ms frame), or Safe is requested (`SAFE REQUESTED`, sticky: two voting nodes disagree and nobody can be blamed; output held).
A peer that has never been seen is not judged for the first 5 s (`startup_grace_frames`); once seen, it always is.

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
