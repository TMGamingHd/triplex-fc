# P-S2-03: stage S2b, the supervisor (SUP-Lite)

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Exit test of S2b (`docs/hardware/STAGED_BUILD.md`): "A node made to hang is reset within 3 frames; the supervisor unplugged leaves FC-A and ACT running (F57, F58, F66)". Design: `docs/design/SUPERVISOR.md`; logic and its host tests: `supervisor/include/sup/`, `tests/test_supervisor*.cpp`.

| Field | Entry |
|---|---|
| Procedure ID | P-S2-03 |
| Requirements verified | TFC-SUP-001 to 004, 006 to 010, 012 to 014 (the clock and T0 parts as far as one flight computer shows them), TFC-FDIR-038, TFC-FDIR-039 |
| Fault-matrix rows | F57, F58, F59, F66, F67, F68 |
| Firmware under test | git hash of FC-A, ACT and the supervisor (`west build -b rpi_pico2/rp2350a/m33 firmware/supervisor`, UF2 to the Pico by drag and drop) |
| Hardware configuration | FC-A and ACT with FRAME, KICK and NRST on the same pins as every node (STAGED_BUILD rule 10); the supervisor Pico on a perfboard with 8.2 kOhm pull-downs on every input and pull-ups on the relay lines; the second relay module's four spare channels on the nodes' feeds (JD-VCC jumper removed); the DS3231 module with its coin cell on I2C0 (GP20, GP21); the PC on the supervisor's USB |
| Tools | the serial port of the supervisor (`screen /dev/ttyACM1 115200`), the logic analyser on FRAME, KICK, NRST and PWR of FC-A, `tfc_peers listen` |

## 1. Description
Show that the supervisor watches the end-of-frame KICK, resets a node that stops (and no other), escalates to a power-cycle and to DEAD as specified, boots the nodes in order, never touches a running node when it starts or restarts, executes the operator's hardware
commands without any flight computer, and keeps mission time across its own reset. **Pass** = every step as expected.

## 2. Initial set-up
P-S1-01 and P-S2-01 passed. Supervisor wiring checked with a meter (no input floating; no output shorted). Power order: supervisor first, then the nodes. USB console open on the supervisor: it says `supervisor up`, then `warm start` or `cold start`.

## 3. Steps
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | Power the supervisor with both nodes **off** | `cold start: every unit held in reset, released in order ACT, A, B, C`; NRST of every unit low, then released 0.5 s apart in that order (logic analyser) | | |
| 2 | Operator | `status` | Four lines, every unit `running`, `resets=0 cycles=0`, the FRAME periods within 200 ppm | | |
| 3 | Operator | Reset the supervisor with the nodes **running** (unplug and replug its USB) | `warm start: units are running, nothing is touched`; no NRST edge on any node, no relay (F66) | | |
| 4 | Operator | Make FC-A stop kicking without stopping FRAME: build it with `-DCONFIG_TFC_TEST_HANG_AT_FRAME=3000` (the KICK stops from SYNC frame 3000, everything else goes on) | Within 40 ms `RESET A (kicks stopped)`; NRST low 100 ms; the node boots and rejoins; `resets=1` (F57, F58) | | |
| 5 | Operator | Repeat step 4 three times inside a minute | The third is followed by `POWER-CYCLE A`: PWR relay open 500 ms (analyser) | | |
| 6 | Operator | Make A fail to recover (hold it halted) for two power-cycles | `DEAD A: held in reset`; NRST stays low; the other node untouched (F68) | | |
| 7 | Operator | `release A` | `ok`; the node boots; the record is clear (`resets=0` after it runs) | | |
| 8 | Operator | `hold A`, then `release A`; `reset B` while B is held; `cycle ACT` | `ok`, then `refused` for a held unit's reset, then ACT's power cycle | | |
| 9 | Operator | `safe-now`, check ACT's console, `safe-clear` | ACT reports Safe with cause *hardware line* and **stays Safe** after `safe-clear` until an operator `clear-safe` over the bus (SAFE_MODE.md) | | |
| 10 | Operator | Unplug the supervisor entirely | FC-A and ACT keep running with every output at its default (TFC-SUP-007) | | |
| 11 | Operator | Skew FC-A's clock by 300 ppm: `-DCONFIG_TFC_TEST_CLOCK_PPM=300` (FC-A is the sync master) | `PERIOD A out of limit (+300 ppm)` after about 10 s and nothing else: A is not reset (F67) | | |
| 12 | Operator | `launch`, then at T-5 s `scrub`; `launch` again and wait | `countdown: T-10 s`; `countdown scrubbed`; then `T-ZERO` after 10 s and the T0 pulse (50 ms) on the line | | |
| 13 | Operator | After T-zero unplug the supervisor's USB power for 30 s and restore it | `T-zero record found: the mission clock is resumed from the RTC (good to a second)`; `status` shows launched | | |
| 13b | Operator | With the `T0` line wired to FC-A's input (PC5, CN10 pin 6, checked against ST's manual): type `launch` on the supervisor, then within one second send the flight computers' authenticated `launch` (`python3 -m tfc_peers launch`) | The supervisor's T0 pulse arrives in the last second of FC-A's countdown: `T0 LINE: the supervisor's T-zero; the next frame is T-zero` on FC-A, and every computer's `T-ZERO` is the same frame. If the supervisor is more than a second late, FC-A's own countdown runs out first (the fallback): no harm, note it | | |
| 14 | Operator | Over a night: `python3 tools/bench/clock_corr.py --port /dev/ttyACM0 --interval 10 --duration 28800 --out logs/clock.csv` (it asks `time` and stamps each answer with the PC's UTC) | The oscillator's drift in ppm with its error bound now and five years out (the first data of MISSION_CLOCK.md section 2); `time` also shows `met_us` after T-zero | | |

## 4. Shutdown
Nodes off first, supervisor last; relays released; the coin cell left in. Keep the console text, the analyser captures and the correlation pairs.

## 5. Records
This file as run; the console log; the analyser captures; rows F57 to F59, F66 to F68 of `docs/verification/FAULT_MATRIX.md` updated with this test id; the measured drift into `docs/design/MISSION_CLOCK.md`.

## 6. Not covered here
The hardware overrides and their sense lines (P-HWO-01); the launch sequence on the whole rig (P-S2-02); the supervisor as a power-cycle of the *real* injector's cut (the two are in series, check P-HWO-01 step 4).
