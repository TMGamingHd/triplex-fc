# P-S4-01: stage S4, Triplex, the fault campaign on the real bus, and the resync

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Exit test of S4: "Fault matrix filled with measured data".

| Field | Entry |
|---|---|
| Procedure ID | P-S4-01 |
| Requirements verified | TFC-SYS-003, 004, TFC-FDIR-001 to 023, TFC-FDIR-044 to 046, TFC-ARCH-001, TFC-LAUN-001 to 007 |
| Fault-matrix rows | F01 to F18, F24 to F45 (as the rig allows), F88 to F95, F81 to F87 |
| Firmware under test | git hash of A, B, C (same image), ACT, Pico, supervisor; the flight function, the sensor split and the resync on (defaults); the launch sequence on (`CONFIG_TFC_LAUNCH_SEQUENCE=y`) |
| Hardware configuration | A, B, C with IMUs on the platform, ACT, the Pico, the supervisor, the USB-CAN adapter, the PC with `tfc_simd --hold --pico` |
| Tools | `tfc_peers`, the injector Pico (`tfc_peers pico`), the campaign's scenario list (`python3 -m campaign.run --list`), `tools/bench/bus_loss.py`, the analyser |

## 1. Description
Run the faults of the matrix on the real bus and fill the HIL column with measured detection times; show the resync on three real computers; measure the real bus loss rate. **Pass** = every step as expected and no anomaly code in the replay of the recorded log.

## 2. Initial set-up
P-S3-01 passed with the same three images. The three computers' pad calibrations ready. The injector Pico's four relays disarmed by the `INJECTOR-DISARM` switch until a step says otherwise.

## 3. Steps
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | One hour quiet with the resync on; `bus_loss.py` with all three consoles | The **real loss rate**; the setting it implies (the report's last section) is applied or left | | |
| 2 | Operator | Three computers, Triplex, the digests of A, B, C equal in every frame; stop at every resync | After each resync `RESYNC:` lines appear only when something differed; **the three digests are equal on the frame after** (the live test on `vcan0`, now on three Nucleos) | | |
| 3 | Operator | Replay the hour's log: `tfc_replay logs/s4.log --sensor-split --first-frame N --startup-grace 500 ...` (`tools/bench/log_t0.py`) | No anomaly; the same decisions as the live run | | |
| 4 | Operator | The campaign's scenarios one at a time through the virtual peers and the injector: dropout, stuck, bias, drift, cmd_offset, digest, late, babble, reboot of each computer | Each detected within its latency; the host's number and the measured number side by side in the matrix | | |
| 5 | Operator | Cut each node's power with the injector (relays armed) in turn | The supervisor sees it dead, tries its resets, gives up at the limit and reports `DEAD`; the others vote on | | |
| 6 | Operator | Kill the sync master (A) mid-run | B takes over within 2 frames, the frame number continuous (the live test's check), C follows | | |
| 7 | Operator | A computer's IMU and another computer's commands faulted together | The IMU channel and the other computer latched; two voters and two IMUs remain; no Safe request (F94) | | |
| 8 | Operator | The launch checklist (P-S2-02) on three real computers with the platform | As in that procedure | | |
| 9 | Operator | Make the fault manager's command tolerance bite: inject a 0.02 degree command offset on one computer for 3 frames | Whether the manager's 0.01 degree tolerance latches it: **record it**; it is the number that decides whether the tolerance is widened (RESYNC.md 6b) | | |
| 10 | Operator | Fill `docs/verification/FAULT_MATRIX.md`: for each row run, the measured detection time and this test id; failures as *failed* | The HIL column filled | | |

## 4. Shutdown
Injector disarmed first; the supervisor last; the platform to level; E-stop reset.

## 5. Records
As-run copy; all logs and analyser captures; the fault matrix; `docs/design/RESYNC.md` and `docs/decisions/TRADE_STUDIES.md` TS-23 updated with the loss rate and the decision it implies; `docs/project/PROOF.md` updated with the measured figures that can be claimed.
