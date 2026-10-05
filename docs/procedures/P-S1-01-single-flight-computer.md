# P-S1-01: stage S1, one flight computer on real hardware

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Exit test of stage S1 (`docs/hardware/STAGED_BUILD.md`): "10 min at 100 Hz, zero frame errors; WCET and jitter measured;
digest matches the PC golden run".

| Field | Entry |
|---|---|
| Procedure ID | P-S1-01 |
| Requirements verified | TFC-SYS-001 (frame-start jitter), TFC-FDIR-010 (bus-off recovery, PC side), the S1 exit test; prepares TFC-FDIR-038 (watchdog), TFC-SUP-003 (kick) |
| Fault-matrix rows | F14 (bus-off), F57 (watchdog, partly) |
| Firmware under test | git hash of the commit built (`git rev-parse HEAD`), built for `nucleo_g474re` |
| Hardware configuration | one Nucleo (FC-A) with CAN Pal and IMU, USB-CAN adapter as the second node on the bus (a lone CAN node gets no ACK and goes bus-off), the PC, the logic analyser on FRAME (PC8) and KICK (PC9) |
| Tools | `tools/bench/*.sh`, `tools/bench/frame_jitter.py`, `tools/bench/check_golden.py`, the Kingst analyser |

## 1. Description
Show that the flight-computer firmware runs the 100 Hz frame loop on the board for ten minutes with no frame errors, that the frame start is steady to
within 100 microseconds at the 99th percentile, that the watchdog is serviced every frame, and that the command and digest stream is bit-identical to the PC's golden run.
**Pass** = all of: `tx_err=0`, `crc=0`, `seq=0` in the last status line; `wdt_refused=0`; FRAME jitter p99 at most 100 us with no missed frame; every command frame matches the golden run.
**Run without the virtual peers** (they feel the simulated motion, not the bench's).

## 2. Initial set-up
P-M1-01 done and passed. `tools/bench/check_pc.sh` clean. Board wired per the pin table in `firmware/README.md`. IMU powered per P-M1-01 step 12. Adapter up (`tools/bench/can_up.sh`).

## 3. Steps

| Step | Who | Action (exact) | Expected | Actual (as run) | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | `. firmware/env.sh && west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re && tools/check_elf.sh build/nucleo_g474re/zephyr/zephyr.elf` | Builds; "no heap, exception, RTTI or vtable symbols" | | |
| 2 | Operator | `tools/bench/flash.sh` | Flashes without error | | |
| 3 | Operator | Open the serial console: `screen /dev/ttyACM0 115200` | The boot line `boot 1 since power-on, last reset cause 1`, then `FC-A (node 0)...` and `IMU` shows no `init failed` | | |
| 4 | Operator | Read the status line for a few seconds | `imu_err=0`, `imu_stale` stays at or near 0, `tx_err=0` | | |
| 5 | Operator | In another terminal: `python3 -m tfc_peers log --iface can0 --out logs/s1.log --duration 620 &` | Log file growing | | |
| 6 | Operator | Start the logic-analyser capture of FRAME and KICK, 10 min, rising edges | Capture running | | |
| 7 | Operator | Wait 10 minutes. Do not touch the board | The console prints a status line every second | | |
| 8 | Operator | Export the FRAME edges to `frame.csv`, then `python3 tools/bench/frame_jitter.py frame.csv --unit s` | `PASS`: p99 at most 100 us, `missed frames: 0` | | |
| 9 | Operator | Export the KICK edges, run the same command | One edge per frame: no missed frames, mean period 10000 us | | |
| 10 | Operator | `python3 tools/bench/check_golden.py logs/s1.log --node A` | `all match the golden run` (about 60,000 command frames) | | |
| 11 | Operator | In the console's last status line | `crc=0 seq=0`, `wdt_refused=0`, `tx_err=0`, `bus_off=0` | | |
| 12 | Operator | Bus-off recovery (PC side): short CAN_H to CAN_L for one second, then remove the short; watch `ip -details link show can0` | The kernel restarts `can0` within 100 ms (`restart-ms 100`); frames resume (FDIR-010, partly) | | |
| 13 | Operator | Watchdog: hold the board in reset with the ST-LINK for a second, release; read the boot line | `last reset cause 2` (pin) or as reported; a loop is not reported | | |

## 4. Shutdown
Stop the logger and the capture; unplug the adapter last; leave the Nucleo powered down. Keep the log and both exports.

## 5. Records
This file as run; `logs/s1.log`; `frame.csv` and the KICK export; the console text; the git hash; the fault-matrix rows F14 and F57 updated with the test id; `SYS-001` and `SYS-002` figures into `docs/verification/REQUIREMENTS.md` if they differ from the proposal.

## 6. Not covered here
WCET of the estimator and controller (they do not exist yet); the self-test of the IMU (limits not set); arrival-time margins (the CAN receive timestamps are enabled in the build, but nothing reads them yet).
