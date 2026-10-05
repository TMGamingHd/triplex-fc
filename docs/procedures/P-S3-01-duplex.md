# P-S3-01: stage S3, Duplex (FC-A and FC-B)

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Exit test of S3: "Unplug B: back to Simplex without a glitch. Inject a bad value: miscompare flagged".

| Field | Entry |
|---|---|
| Procedure ID | P-S3-01 |
| Requirements verified | TFC-FDIR-001 to 008, TFC-FDIR-020 to 023, TFC-ARCH-001, TFC-SYS-003, TFC-SYS-004 |
| Fault-matrix rows | F01, F02, F04 to F10, F15, F17, F65 with the real IMUs, F29 to F36 as the rig allows |
| Firmware under test | git hash of FC-A, FC-B (same image, node id by strap), ACT, the Pico, the supervisor if present; `CONFIG_TFC_FLIGHT_FUNCTION=y`, `CONFIG_TFC_SENSOR_SPLIT=y` (the default with it), the resync period default 100 |
| Hardware configuration | FC-A, FC-B with IMUs, ACT, the full backbone with the USB-CAN adapter, the platform if the run is closed-loop |
| Tools | `tfc_peers command`, `tfc_peers listen`, the virtual peer for C (`tfc_peers run --follow-sync`), the consoles |

## 1. Description
With two real computers and a virtual C: continuity when a computer is removed, the Duplex rules of ADR-008 and ADR-017 against real noise, the sensor split on real IMUs, and the resync with two voters. **Pass** = every step as expected.

## 2. Initial set-up
P-S1-01 for FC-A and the same for FC-B (its own run, same limits). Both calibrated on the pad (`ready` set). The virtual peer for C running so that the bus has three participants.

## 3. Steps
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | Run ten minutes, no fault | `A+ B+ C+` (C virtual), `vote=0 digest=0 stuck=0`, `missing` and `crc` at or near 0; the digests of A and B equal in every frame (`check_golden` on each) | | |
| 2 | Operator | Unplug B's CAN stub | Within 3 frames `node B LATCHED OUT`, mode Duplex or Simplex per the virtual C; no step in ACT's output larger than the slew limit | | |
| 3 | Operator | Replug; `reintegrate B` | After the dwell `ON PROBATION`, after 100 agreeing frames `REINTEGRATED` (ADR-010) | | |
| 4 | Operator | Give B's IMU a bias of 3 dps: `-DCONFIG_TFC_TEST_GYRO_BIAS_MDPS=3000` (or warm it by hand until its gyro moves more than 1 dps from A's) | `IMU B LATCHED OUT (computer B stays in the command vote)`; B keeps voting commands (F65 on real hardware) | | |
| 5 | Operator | Record the pad calibration of both IMUs (`ready`, bias, the standard deviation) | Both ready; the bias spread between the two IMUs written down (it sets the consensus tolerance, ADR-028) | | |
| 6 | Operator | Make B's command wrong by 1 degree: `-DCONFIG_TFC_TEST_CMD_OFFSET_MDEG=1000` | `LATCHED OUT: vote disagreement` within 5 frames | | |
| 7 | Operator | With only A and B, drift A's IMU by 0.05 dps per frame: `-DCONFIG_TFC_TEST_GYRO_DRIFT_MDPS_PER_FRAME=50` | A Safe request after the persistence, held outputs, **no computer blamed** (ADR-008) | | |
| 8 | Operator | Wait for a resync frame with the bus analyser armed on `0x420` to `0x42B` | Twelve frames in the last frame of every 100, in the 5.5 to 7.1 ms window; B's console `RESYNC: adopted the vote` only when something differed | | |
| 9 | Operator | One hour quiet; `python3 tools/bench/bus_loss.py logs/s3.log --resync-period 100 --console A=a.txt --console B=b.txt` | The first real loss figure (two computers): written into the as-run copy | | |

## 4. Shutdown
Virtual peer first, then the nodes; relays released.

## 5. Records
As-run copy; logs; the fault-matrix rows with the test id; the loss figure and the bias spread into `docs/design/RESYNC.md` and `docs/design/LAUNCH_SEQUENCE.md` if they differ from the assumptions.
