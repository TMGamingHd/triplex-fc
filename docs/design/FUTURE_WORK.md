# Future work: designs that are written down and not built

> Status: **reference** (reviewed 5 Oct 2026). This page replaces the old "deferred work" register. Everything that was on it and has since been built is described in the design page it
> belongs to; what is left is here, each with the thing that has to exist before it can be done and the design notes written now so nobody has to re-derive them. Items that only wait for
> the hardware to arrive are listed with their procedures in [`../STATUS.md`](../STATUS.md); this page is for work that is **not** just a measurement.

## 1. Waits for the board

### 1.1 Arrival-time monitoring (E3 and E11 option B; TFC-FDIR-037)
**Trigger:** firmware on the real board with a CAN receive timestamp (the build enables the FDCAN timestamps; the thresholds need the rig).
**What:** for every received frame, the arrival time relative to the SYNC of its cycle, per node and stream. Three uses, in this order, and the order matters:
1. **Margin telemetry:** per node and stream, the minimum and mean margin to the 7 ms vote deadline over a window, and a count of frames inside 20 % of their slack. Logged, no effect on voting.
2. **Slot-window check:** compare the arrival with the slot (`1.5 + 0.2 n` ms for gyro, `2.3 + 0.2 n` for accel, `5.0 + 0.3 n` for the command) plus the legitimate bus delay. This closes the blind band that the frame-number check
   leaves (a frame up to about 4.5 ms early, a clock drifting by a few hundred microseconds). Enforce it only after the margins have been measured under bus load: nine frames queue per cycle at about 0.11 ms each, so a frame can
   legitimately be late by around a millisecond, and a window that is too tight isolates healthy nodes.
3. **Warning flag:** "timing degraded" for a node whose margin stays under the threshold for M of N frames; reported, never a latch.
**Design notes:** the core knows no time. Compute the margin in the application layer and pass it in the report, or give `on_frame` an optional receive timestamp. The peers' `early`, `jitter`, `late` and `clockdrift` faults exercise the logic
in simulation; the thresholds cannot come from there. If the 1.4 ms slack of node C's command proves thin, move the vote later or the command slots earlier (this trades against the compute window).
The supervisor's Lite build cannot see arrival times (`SUPERVISOR.md`); the Full build would (TFC-SUP-005).

### 1.2 Sensor self-test at power-up (TFC-FDIR-036)
**Trigger:** the part, and the datasheet's limits for the self-test output change. The mechanism exists and is host-tested (`ism330dhcx.hpp`: it fails closed when no limits are set), and it is not called until the limits are. A node whose sensor fails
the self-test reports "not ready" and does not join the vote. Belongs to stage S1.

### 1.3 A defined response to a persistent bus alarm (TFC-FDIR-040)
**Trigger:** the rig. The alarm is raised and reported today; what to do about one that lasts more than N frames (proposal 10) is defined and tested on the bus. The supervisor's `hold X` is the operator's way to find a babbler by elimination.
It never holds a node automatically on an alarm alone: an out-of-schedule id cannot be attributed to a node (ADR-009).

## 2. Needs a decision or a use case, not hardware

### 2.1 Analytical redundancy for Duplex attribution (E6 option A, TS-11)
The estimator now exists, so the trigger is met; it was not built because the accepted answer to a Duplex disagreement nobody can attribute is Safe, which is the fail-safe outcome (ADR-008), and the exposure to it is short (a first transient latch waits
50 frames, not 200, before probation). The idea: check each node's gyro against the other sensors and the model (gyro-integrated attitude against the tilt from the accelerometers). It resolves faults larger than the model's own error (slow drifts yes; a 1.5 dps
bias just above a 1 dps tolerance, probably not). On the desk rig, with a platform of known accelerations, it can be shown honestly; in flight thrust corrupts the accelerometer's gravity reference. New failure mode to test for: blaming the wrong node.
Revisit if the rig shows that Safe in Duplex is reached too often.

### 2.2 Telemetry and events in a fixed format (IF-004 to IF-006)
`noop` (FDIR-043) is built. What is not: **a dictionary** (one file listing commands, telemetry channels and events, with argument limits, ARM needs and the modes that refuse a command, and a test that checks the C++ and the Python against it, so the
opcode tables in `protocol.hpp` and `tfc_peers/protocol.py` cannot drift); **housekeeping** (the counters as numbered channels at a fixed rate with caution and warning limits); **events** (one record per event with frame number, kind, node, reason and a
per-kind sequence number, so a gap shows a lost record). Today the report struct, the counters and the console lines carry the information. Reuse the column names of the campaign's `--dump` CSV. Start with the dictionary and the event records.

### 2.3 Ground-changeable monitor parameters
The lecture's rule: the condition and the response are code, the threshold and the persistence are parameters. Today they are build-time configuration (validated, guarded and scrubbed, ADR-015). Making them settable by an authenticated, ARM-protected
command is possible once the board exists, with the rule that no parameter may disable a monitor.

### 2.4 Before any real uplink
The ground-command security is bench-grade (ADR-019): the default key is the public SipHash test key and protects nothing; the tag comparison is not constant-time; the internal bus is a trusted segment and the heartbeat and state share are not
authenticated; nothing models the uplink. A real uplink needs key provisioning and storage (`CONFIG_TFC_GROUND_KEY` exists), authentication at the vehicle gateway, and a constant-time comparison. The command-counter gap after a reboot is closed by the state share
(FDIR-041); a ground counter that is more than 32 ahead of a flight computer's still needs that computer restarted.

### 2.5 Membership agreement
Each flight computer decides alone, so two can hold different views of a node (ADR-010's limit). The state share restores strike counts and the command counter after a restart; it is deliberately not an agreement protocol. A full membership
agreement is a research topic of its own and stays out of v1.

## 3. Architecture stretches (outside the v1 scope)

| Item | What | Why it is not in v1 |
|---|---|---|
| **A second ACT** with a supervisor-driven output selector (ADR-023) | Extends fail-operational to the actuator node | One Nucleo, one CAN Pal and a selector chip, about 24 USD plus the chip; the budget is already over its ceiling |
| **Ring re-homing of an orphaned IMU** (ADR-020 case 2, TFC-ARCH-003) | After a double fault a backup computer adopts the IMU of a failed one, one owner at a time | Needs the supervisor's `ADOPT` lines and a second SPI port per computer; the harness reserves them (`STAGED_BUILD.md` rule 11); decide at S4 |
| **Dual-slot boot with a golden image** | A tiny unchangeable boot program chooses between two firmware images; the golden release of ADR-021 is the natural second image | Board bring-up; ties to FDIR-042 |
| **Bus and memory sizing analyses** | Worst-case bus load with bit stuffing, stack and flash budgets | Measured on the target (WCET: the status line already reports `wcet_step`, `wcet_vote`, `wcet_frame`) |
| **A second bus, a self-checking pair, CAN FD** | Removes the shared bus as a common cause | TS-5 compares them on the rig after S3 |

## 4. Design note: accelerometer calibration and alignment
The estimator's `align()` takes the initial attitude from gravity on the pad, and the pad calibration (`ImuCalibrator`) removes each gyro's stationary bias before the consensus. What is **not** done is a two-position (or six-position) accelerometer calibration
for scale and misalignment. That is a procedure on the rig (hold the platform at two known attitudes, fit the matrix), not a flight-software feature, and it matters only once the real IMUs are on the platform (`P-S1-01`). If the measured accelerometer error exceeds
what the sensitivity study (`tfc_sens`) says the closed loop tolerates, add the matrix to the per-channel calibration constants that every computer already carries (ADR-020).

## 5. Out of the flight path: AI-assisted tooling
**The rule:** no learned component is in the flight path and none holds the ground-command key. The replicas must produce bit-identical state and digests (ADR-006); the fault management is argued by a deterministic design with oracles that are
mutation-tested (ADR-016), which a learned model cannot be; and an inference time that varies is a timing fault in a time-triggered schedule. Saying where AI was *not* used, and why, is the point an avionics interviewer will probe (TS-14).
**Where it would help, outside flight, in order of value:** (1) falsification search, an optimiser looking for fault parameters and two-fault combinations that violate an oracle beyond the campaign's grids, each find becoming a regression scenario;
(2) log triage; (3) a shadow anomaly detector on the telemetry, advisory only, compared with the deterministic design for detection time and false alarms (TS-14); (4) offline tuning of gains, persistence and Safe parameters, whose result is a fixed table.
**Not before** the first hardware measurements (M4), and with boundary tests written with the first use: byte-identical campaign dumps with the AI parts disabled and enabled, no inference code in the flight binary (`tools/check_elf.sh` extended), no access to the key.
Requirements TFC-AI-001 to 003 (proposed boundary).

## 6. Considered and not added
Rad-hard parts, MIL-STD-1553, memory types, data processing and encryption units, pyro boxes, the uplink-loss timer (a rocket does not need the ground to survive). Added after the second reading of the lecture and now built: the time-seed plausibility check
(TFC-SUP-009), the parameters of the Safe configuration (TFC-SAFE-005) and the response ladder (TFC-SAFE-007).
