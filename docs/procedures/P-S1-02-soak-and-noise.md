# P-S1-02: the noise of the real IMUs and a fault-free soak

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. It settles the half of the detection story that the fault campaign cannot: **how often a computer with nothing wrong is put out of the vote**, and what the real IMUs' noise is against the model the detection thresholds were tuned on.

| Field | Entry |
|---|---|
| Procedure ID | P-S1-02 (the noise at S1 with one computer; the soak at S4 with three) |
| Title | The IMUs at rest (Allan deviation) and a fault-free soak (false-isolation bound) |
| Requirements verified | TFC-FDIR-047 (the false-isolation rate), TFC-FDIR-004 (a single disagreement does not isolate a healthy channel), and the thresholds of TFC-FDIR-002 against the real noise |
| Fault-matrix rows | F103 (no fault at all) |
| Firmware under test | git hash of every node; the same release on all three (a mixed-release run is P-REL-01) |
| Hardware configuration | Part A: one flight computer (S1), its IMU at rest. Part B: three flight computers and ACT on the bus, the IMUs mounted as for the campaign, no platform motion, no injector, no operator command |
| Tools | `python3 -m tfc_peers log` (the capture), `tools/bench/allan.py` (Part A), `tools/bench/soak_report.py` (Part B), `docs/design/IMU_MODEL.md` (the datasheet figures) |
| Prepared by, date | 19 Oct 2026 (the first procedure written from the FMECA: REC-10) |

## 1. Description
**Part A, the noise.** Record the IMU at rest for at least one hour (four, for the bias instability) and compute the Allan deviation of each axis. **Pass** = the noise density of every axis of every IMU is at or below the datasheet's limit (gyro 8 mdps/rtHz, accelerometer 100 ug/rtHz), the three IMUs agree with each other to within a factor of 1.5 in noise density, and the bias instability, if the record is long enough to see it, is not above three times the datasheet's 3 deg/h. A figure above the limit is not a failure of the firmware: it is the reason to re-tune the vote tolerances (`docs/decisions/TRADE_STUDIES.md` TS-1) before the soak.
**Part B, the soak.** Run the three flight computers and ACT, undisturbed, for as long as can be had and count every isolation, Safe request, bus alarm, reset and departure of ACT from Nominal. **Pass** = none of them in the run; the bound on the rate is then 3 / hours at 95 %, and the target (under one false isolation in 100 hours, TS-1) is shown at 300 hours. **Any isolation is a finding to be explained before anything else** (read the nodes' consoles for the reason: `vote disagreement`, `frame missing`, `CRC failure`...); the tool prints the bound with it.

## 2. Test cases
| Case | What is varied | Pass criterion |
|---|---|---|
| A1 | the gyro of each of the three IMUs at rest, 1 h | noise density at or below 8 mdps/rtHz; the three within 1.5x of one another |
| A2 | the accelerometer of each, same record | at or below 100 ug/rtHz (bus resolution: 1/2048 g, see section 6) |
| A3 | the same IMUs after the platform has been run (warm) | not worse than A1 by more than 1.5x |
| B1 | three computers, ACT, no motion, 1 h (a smoke run) | zero events |
| B2 | the same, overnight (8 to 10 h) | zero events, or each one explained |
| B3 | the same at the length that demonstrates the target (300 h, in pieces of at least one night) | zero events in the total |

## 3. Initial set-up
P-S1-01 done and passed (A), P-S4-01 done and passed (B). `tools/bench/check_pc.sh` clean. The bench still: no draught on the IMUs, the platform stopped and levelled (a vibration is not noise), the mains-powered equipment that shares the table switched off for the run if it can be. Part B: the controller of the platform **not** driven, the injector disarmed (its switch open), the supervisor present and its `status` clean. The room temperature is written down at the start and the end (the datasheet's figures are at 25 degC).

## 4. Steps
| Step | Who | Action (exact) | Expected | Actual (as run) | Pass/fail |
|---|---|---|---|---|---|
| A1 | Operator | Power the flight computer, wait 10 minutes for the IMU to settle. `python3 -m tfc_peers log --iface can0 --out logs/rest-a.log --duration 3700` | the log grows by 100 frames a second per stream | | |
| A2 | PC | `python3 tools/bench/allan.py --log logs/rest-a.log --node A --sensor gyro` and `--sensor accel` | a table for x, y and z; the noise density against the datasheet | | |
| A3 | Operator | Repeat for B and C (S3, S4), or for each IMU in turn on the one board if there is only one | the three noise densities | | |
| A4 | Operator | Record the bias instability's tau and value; if the report says "not seen", record for four hours | a floor, or the statement that the record was short | | |
| B1 | Operator | Bring up the three computers and ACT as in P-S4-01 step 1 to 6; check `tfc_peers listen` shows Triplex, no Safe request | all three healthy, ACT Nominal | | |
| B2 | Operator | `python3 -m tfc_peers log --iface can0 --out logs/soak-1.log --duration 3600`; do not touch anything | the log grows | | |
| B3 | PC | `python3 tools/bench/soak_report.py logs/soak-1.log` | zero events; the bound; "target not demonstrated (not enough hours)" | | |
| B4 | Operator | Repeat overnight (`--duration 36000`), then in nights until 300 hours are in | `TARGET DEMONSTRATED` on the sum (add the hours; the tool reads one log, so run it on each and add the hours by hand, or `cat` logs whose timestamps are continuous) | | |
| B5 | Operator | At each isolation: stop, copy the nodes' console text and the log, do not restart the node until the reason is read | the reason named; a finding opened | | |

## 5. Shutdown
Stop the logger; leave the nodes powered down. Keep the logs, the console text and the temperature notes. Anything unusual that happened during a run (a door, a phone charger plugged in) goes into the as-run copy with the time.

## 6. Records and what the numbers decide
This file as run; the logs; the tools' output; an entry in `docs/hardware/BENCH_LOG.md` with the figures; `docs/verification/REQUIREMENTS.md` (FDIR-047) and the fault-matrix row F103 updated with the test id and the result; the FMECA's REC-10 closed or reopened.
* **Noise above the limit:** the vote tolerances (1 dps gyro, 0.02 g accelerometer) were set against the datasheet; if a part is noisier, the false-isolation rate is where it will show (B), and the tolerances are what to look at.
* **Isolations in the soak:** the bound is not the issue; the reason is. Each one is either a threshold set too tight for the real noise (re-tune with the Allan result), a real bus loss (`tools/bench/bus_loss.py`), or a defect.
* **The bus is not the sensor:** a gyro count on the bus is 1/32 dps and an accelerometer count 1/2048 g. If the noise is within about twice that, the Allan report says the figure is the bus's, not the sensor's. To measure below it, read the IMU's raw output through the node's own console or a debug build (not done here).

## 7. Not covered here
The temperature dependence (the datasheet's zero-rate change per degree needs a thermal chamber or a long drift of the room); vibration (the platform's own, which P-S2-01 measures); and anything the real vehicle's motion does, which no run at rest can say.
