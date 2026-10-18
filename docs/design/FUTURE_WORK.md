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

**An aiding source for a vehicle in flight** (added 7 Oct 2026). A flying vehicle's accelerometer reads no gravity, so its attitude is the integral of its gyros and nothing corrects them (`SIM_FIDELITY.md` 3.5): the Monte Carlo shows the gyros' zero-rate error left after the pad calibration to be the largest single contributor to the large attitude errors of a launcher flown on the gyros alone (`MONTE_CARLO.md` section 5). An aiding source (a GPS or inertial navigation solution, a star tracker, a horizon sensor or a magnetometer) would give the estimator something to correct against. It is a decision about hardware and an input on the bus; nothing on the rig provides it today, and the rig, which has gravity on its platform, does not need it.

## 5. Out of the flight path: AI-assisted tooling
**The rule:** no learned component is in the flight path and none holds the ground-command key. The replicas must produce bit-identical state and digests (ADR-006); the fault management is argued by a deterministic design with oracles that are
mutation-tested (ADR-016), which a learned model cannot be; and an inference time that varies is a timing fault in a time-triggered schedule. Saying where AI was *not* used, and why, is the point an avionics interviewer will probe (TS-14).
**Where it would help, outside flight, in order of value:** (1) falsification search, an optimiser looking for fault parameters and two-fault combinations that violate an oracle beyond the campaign's grids, each find becoming a regression scenario;
(2) log triage; (3) a shadow anomaly detector on the telemetry, advisory only, compared with the deterministic design for detection time and false alarms (TS-14); (4) offline tuning of gains, persistence and Safe parameters, whose result is a fixed table.
**Not before** the first hardware measurements (M4), and with boundary tests written with the first use: byte-identical campaign dumps with the AI parts disabled and enabled, no inference code in the flight binary (`tools/check_elf.sh` extended), no access to the key.
Requirements TFC-AI-001 to 003 (proposed boundary).

## 6a. A review against the practice of the field (18 Oct 2026)

> Written after the owner asked "what am I missing and what can be done better, thinking about what someone in this field should do". Each line was checked against the repository before it was written (a search of `sim/`, `core/`, `tools/` and `docs/`, and flying the missions); what I know of the field's practice is from memory and is marked as such. The ranking is by what it would change in a reader's confidence per week of work.

**What is strong, so that the list is not read as a verdict:** a voted three-computer design with a fault campaign, a mutation-tested oracle, 99.2 % line and 90.1 % branch coverage of the simulator, requirements with identifiers, an FMEA and an STPA, deterministic replicas, a coding standard that CI enforces, and a habit of writing the limits down. Few projects of this size have any of these.

| # | What the field does | What is here (checked) | What to build | Size |
|---|---|---|---|---|
| 1 | **Time the flight code on the target.** The first question about a flight computer is how long a cycle takes and with what margin, on the part that will fly. | Nothing has run on a board (`LIMITATIONS.md` 3, 10); the guidance and navigation are host-only and the PEG predictors are double precision on a single-precision FPU; the parts have arrived (`STATUS.md`). | Build the guidance and navigation into the image, run the PEG cycle and the descent predictor on the G474, report the worst case with the margin to the 10 ms frame (`P-S1-*`), and decide float or fixed point from the measurement. | days, with the board |
| 2 | **Stability margins of the attitude loop**, by frequency response: gain and phase margin at several points of the flight (lift-off, max q, staging, entry), with the servo, the delay, the slosh and the bending modes in the loop. A launcher's control law is accepted on them (about 6 dB and 30 degrees is the usual ask, from memory). | None: a search for gain margin, phase margin, Nichols, Bode, crossover finds nothing in the code, the tools or the docs. The study is in the time domain (`tfc_sens`: how far each departure goes before the flight is lost; `tfc_mc`). **The data is already there:** the gain design (`mission_design.hpp`) probes the plant for the aerodynamic stiffness and the effectiveness of every axis at points along the flight. | A tool that turns each probe into the rigid-body open loop (stiffness, effectiveness, the controller's gains, the servo's second-order lag, the frame delay, the slosh pendulum and the first bending mode from the spec), and prints the margins along the flight and where they are smallest. | about a week |
| 3 | **An estimator with a covariance.** An error-state Kalman filter with accelerometer and gyro bias states, innovation gating on a chi-square test and a consistency check (NIS and NEES) in the Monte Carlo. | An alpha-beta observer with fixed gains, an attitude correction taken from the velocity innovation, one GNSS receiver whose fix all three computers share, no covariance and no bias states (`nav.hpp` header, `LIMITATIONS.md` 10). The Monte Carlo found the gyros' zero-rate error to be the largest single contributor to the large attitude errors (`MONTE_CARLO.md` 5). | The filter, in the same header-only, no-heap style, with the covariance reported so that the guidance can know how far to trust it (it is asked for now by `time since fix`), and a Monte Carlo that checks its consistency. | 1 to 2 weeks |
| 4 | **A Monte Carlo of the whole mission**, not of the ascent: the probability that the stage is caught and the ship is in orbit, over dispersions of masses, thrust, aerodynamics, wind and sensors together, with the Wilson interval. | `tfc_mc` disperses the ascent of a launcher (34 draws); the mission's evidence is 40 noise seeds that move only the receiver's and sensors' noise (39 of 40 caught, 40 of 40 in orbit): the 40 seeds say nothing about a thrust that is 3 % low, a drag coefficient that is 15 % off or a day with a jet stream; ADR-049's "revisit" already lists it. | `tfc_mc` over a mission file: the dispersion machinery, the statistics and the seeds exist; the runner would be `MissionWorld`. The result is the figure the catch should have been quoted with. | about a week |
| 5 | **Load relief and a limit on q alpha.** The structure's design load on the way up is the bending moment from q alpha, so the guidance flies the vehicle into the wind (or biases the pitch program on a balloon's wind) to keep it down, and a trajectory is checked against the allowable. | Nothing: the pitch program is fixed (a search for load relief and q alpha finds nothing in the simulator). **The envelope added with this review** (ADR-053) shows the stack's maximum q sin(alpha) at 4.4 kPa (about 250 kPa degrees) and the booster's descent at **183 kPa** of dynamic pressure, which is very likely far harder on a structure than a real stage's (from memory: launchers keep q alpha to a few kPa, a returning booster to a few tens of kPa). The Loads lens has no allowables. | Allowables in the vehicle file from a source, a failed run when a mission crosses them, and a wind-biased pitch program or an accelerometer load relief; then the descent's q (the gate, the entry burn) is a design question and the catch has to be re-won under it. | about a week |
| 6 | **Verify the voter and the fault manager with a model checker.** A bounded model checker (CBMC, ESBMC) proves properties for every input inside a bound, not for the inputs a test chose; the code is header-only, with no heap, no exceptions, no recursion, which is what such a tool wants. | Mutation testing of the oracle, a fault campaign, 90 % branch coverage. No model checker, no proof, no property-based test (searches for CBMC, model checking, TLA, property-based find nothing). The checker is not installed on this machine. | Properties for `Voter` and `RedundancyManager`: no out-of-range access, the vote is a function of the inputs, Safe is left only by an operator command, a latched node does not vote, the counters do not wrap into a false state; run in CI for a bounded number of frames. | 1 to 2 weeks |
| 7 | **Convergence and validation of the CFD.** A grid-convergence index (Roache's) from three grids, and a comparison with a public case with data (the blunt bodies and cones with published pressure coefficients and drag, and the Apollo capsule's and the Orbiter's aerodynamic data books are public). | The solver is checked against exact solutions (Sod's tube on three grids, a wedge, a cone, a sphere's stagnation pressure, similarity solutions: ADR-050, ADR-051), but the vehicle's own tables were each run on one grid, the flaps and fins are not converged (ADR-051), and nothing here is compared with the flow of a real vehicle. | A three-grid study of each table node with the index in the table's file, and one public data case; then dispersions of the table (its uncertainty) in the Monte Carlo of item 4. | about a week |
| 8 | **The ship's return.** | The experimental ship return comes down 131 km from the site with the solver's tables and 26 km with the build-up: its entry guidance was tuned to the build-up and has not been retuned (ADR-051). The mission files fly a planet without rotation (the poses' spec says 0): the planet presets have the rotation, the flights do not use it. | Retune the entry on the table; fly the missions on the rotating Earth (the launch azimuth and the inertial speed change by 400 m/s). | about a week |
| 9 | **An abort and a range-safety logic**, or the explicit statement of why there is none. | An abort is a launch rule, not the computer's (`LIMITATIONS.md` 2: the owner's decision, 4 Oct 2026); a search of the docs for flight termination, AFTS and range safety finds nothing. | A page that says what a termination system is, what independent hardware the real one runs on, and what the flight computer does and does not do about a vehicle outside its corridor; a corridor check that only reports. | days |
| 10 | **A telemetry dictionary and a recorded-data format** (CCSDS or a plain fixed dictionary) so that the C++, the Python and the console cannot disagree about a channel. | `FUTURE_WORK.md` 2.2, written down and not built. | The dictionary, with the test that checks both sides against it. | days |

**What I would do first, and why:** 1 (the board is on the desk and the whole claim is untested without it), then 2 and 4 together (they use machinery that exists, and between them turn "it flew" into "it is robust, with margins and a probability"), then 5 (the one that may change a result: the booster's descent q), then 3. 6 and 7 are what an interviewer in the field would ask next and are the best use of a long evening.

## 6. Considered and not added
Rad-hard parts, MIL-STD-1553, memory types, data processing and encryption units, pyro boxes, the uplink-loss timer (a rocket does not need the ground to survive). Added after the second reading of the lecture and now built: the time-seed plausibility check
(TFC-SUP-009), the parameters of the Safe configuration (TFC-SAFE-005) and the response ladder (TFC-SAFE-007).
