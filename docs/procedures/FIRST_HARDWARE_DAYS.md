# The first hardware days: the order of work, what to measure, and what each number decides

Parts are due **9 October 2026**. This page is the schedule for the first days after they arrive. It adds nothing to the design: each line points to the procedure that has the exact
steps, and says which number measured that day settles which open setting. Days are a sequence, not dates; a stage that fails stops the sequence until it is understood.

## Before the parts arrive (software side, done in the repository)
| Item | Check |
|---|---|
| The PC | `tools/bench/check_pc.sh` reports nothing missing; `vcan0` rules; `can-utils`, `openocd`, the ST-LINK rule, the `gs_usb` driver |
| The images | `. firmware/env.sh`; build `firmware/app` for `nucleo_g474re` with and without `-DCONFIG_TFC_FLIGHT_FUNCTION=y`, `firmware/act` for `nucleo_g474re`, `firmware/pico` and `firmware/supervisor` for `rpi_pico2/rp2350a/m33`; `tools/check_elf.sh` on each ELF |
| The tests | `build/host/tfc_tests`, `python3 -m unittest discover` in `sim`, the live tests of `tools/bench/sil_triplex.sh --test` |
| Printouts | P-M1-01, P-S1-01 and the others below, with room to write: they are filled in as run and kept (`docs/procedures/`) |
| **Open question for the order** | the hardware overrides (`docs/HARDWARE_OVERRIDE.md`, about 25 USD of switches, a servo-tester board) were not on the parts list; if they are not in the order they wait, and the checks of P-HWO-01 are skipped until they are |

## Day 1: unpack and measure (M1) — P-M1-01
Every part against the sheet; JP5 on every Nucleo; adapter outputs under 5.25 V; relay at 3.3 V drive and with a floating input; the servo with a 3.3 V signal and with **no** signal; mounting holes; the IMU's logic level.
**What the numbers decide:** the servo's behaviour with no signal answers `FAULT_RESPONSE.md` question 2 (what ACT's output does after a reset); the IMU's high level decides whether the IMUs are powered from 3.3 V only (TFC-ARCH-010); the relay and
floating-input results decide whether the supervisor's `PWR` pull-ups are mandatory.
**Go on if:** nothing exceeded a rating and the USB-CAN adapter comes up at 1 Mbit/s (`tools/bench/can_up.sh`).

## Day 2: one flight computer (S1) — P-S1-01
FC-A alone on the bus with the USB-CAN adapter as its second node. Ten minutes at 100 Hz, FRAME jitter from the logic analyser, the digest against the golden run.
**Also read off the console** (the status line ends with them): `wcet_step` and `wcet_vote` in microseconds, the longest the flight function and the manager's frame took so far, and `wcet_frame`, the longest time from the start of a frame to the end of its last action (it includes the waits to the 7 ms vote, so it must stay under 10 ms; on a quiet bus it is about 7.1 ms, and a larger value is the slack that was lost) (`TFC-SYS-002`).
Run it once with the flight function on and the IMU's real samples; compare the command trace with the PC's closed loop (the same estimator on the same frames gives the same bits: this is the first test of the target's FPU against the host's).
**What the numbers decide:** the worst-case compute time sets the real margin of the 7 ms vote (TS-1, ADR-003); a digest that differs from the host's by more than the quantisation means the target's floating point is not the host's and ADR-006 needs the tolerance for it.
**Go on if:** P-S1-01 passes, `imu_err=0`, `wdt_refused=0`.

## Day 3: the actuator node, the platform and the supervisor (S2, S2b) — P-S2-01, P-S2-03
ACT on its own Nucleo; the Pico's servo outputs against a ramp with the hard stops fitted; the closed loop with one computer; Safe by ACT alone; an ACT reset. Then the supervisor on a perfboard watching FC-A and ACT: a hung node reset within three frames,
the power-cycle and DEAD rules, the supervisor unplugged leaving both running.
**What the numbers decide:** the platform's real bandwidth and lag against `SIM_FIDELITY.md` section 3 (the vehicle is time-scaled if it cannot follow); the servo's current under load against the rail's fuse; the RTC's drift against the PC (the first correlation data of
`MISSION_CLOCK.md`: `python3 -c "from tfc_peers.timecorr import Correlator"` over a night of pairs).
**Go on if:** the E-stop and the platform-level check work (P-HWO-01, if fitted) before anything is run unattended.

## Day 4: Duplex (S3) — P-S3-01
FC-B joins. Unplug B (back to Simplex without a glitch); inject a bad value (miscompare flagged, nobody blamed, Safe requested after the persistence); the IMU of one computer faulted with the sensor split on (the computer stays a voter).
**What the numbers decide:** the real detection times against `FAULT_MATRIX.md`; whether the sensor split's behaviour on real IMU noise matches the host's (the consensus tolerance of 1 dps against the real bias spread of three IMUs: record each IMU's calibration result on the pad, `ready` and the bias, before the run).

## Day 5: Triplex and the bus (S4) — P-S4-01
FC-C joins. The fault campaign on the real bus (F01 to F18 and the rows of the matrix that have a HIL column). **A one-hour quiet run with the resync on, then `python3 tools/bench/bus_loss.py logs/s4.log --resync-period 100 --console A=a.txt --console B=b.txt --console C=c.txt`.**
**What the numbers decide:** this is the **real bus loss rate**, the number TS-23, the digest persistence and the manager's command tolerance have been waiting for (`docs/RESYNC.md` sections 5 and 6b); the arrival time of the twelve resync frames against the 7 ms and 8 ms
slots (the burst budget, `RESYNC.md` section 6); whether three Nucleos' states stay bit-identical after a resync (the digests equal on the frame after, as in the live test on `vcan0`).

## Then
The launch checklist on the real rig (P-S2-02), the overrides (P-HWO-01), the sensitivity sweeps against the real platform (`tfc_sens --mode platform`), and the studies that need the rig (TS-5, TS-8, TS-9; TS-17's measured part). Each stage ends with a tagged commit and a short log or video (`STAGED_BUILD.md`).

## If something fails
Stop at the stage, keep the log, write the symptom in the as-run copy, and open the matching fault-matrix row as *failed*, not passing. Do not carry on to the next stage on a hope: the stages are cumulative.
