# Concept of operations: who does what, from power-up to the end of a run

> Status: **written** (18 Oct 2026), from the design pages it summarises (`LAUNCH_SEQUENCE.md`, `FAULT_RESPONSE.md`, `SAFE_MODE.md`, `MISSION_PHASES.md`, `SUPERVISOR.md`, `HARDWARE_OVERRIDE.md`, `CONSOLE.md`). It adds nothing to the design: it says in one place how the system is **operated**, so that what the operator decides and what they are never asked to decide is stated and not left to be found in the code. Every number is a design number of those pages; nothing here has been exercised on a board.

## 1. Purpose and scope

The system is a **fault-tolerant flight computer demonstrator**: three flight computers, an actuator node that votes, a supervisor, a simulator that flies a vehicle, and a flight console. "Operating" it means running a simulated (or, on the rig, a platform-driven) ascent and watching the system survive faults injected into it. It is not a launch vehicle, there is no range, and **no learned or remote component takes part in a flight** (`FUTURE_WORK.md` 5). The statements about a real launch (abort, range safety) are in `RANGE_SAFETY.md`.

## 2. Who and what takes part

| Actor | Does | Never does |
|---|---|---|
| **Operator** (a person at the console, `tfc_peers` or the web page) | Decides to launch or scrub, resolves a Safe request by naming the computer to trust, readmits or disables a computer, changes the phase | Flies the vehicle; changes a computer's output; acts without the authenticated command path |
| **Flight computers A, B, C** | Sense (each its own IMU), agree on the sensor values, estimate, compute the command, judge each other, send a heartbeat; hold the launch gate | Decide a phase change on their own (`TFC-PHASE-004`); abort (`TFC-SAFE-008`, owner's decision of 4 Oct 2026) |
| **ACT** (actuator node) | Votes the three commands (2-of-3 mid-value), latches the output, runs the Safe sequence (freeze, then null after `T_hold`), drives the `SAFE` line | Trust a flight computer's judgement of another |
| **Supervisor** (SUP-Lite, a Pico) | Watches each node's `KICK` and `FRAME` lines, resets, power-cycles and holds a node that stops, counts down and asserts `T0`, keeps the clock of record | Transmit on the flight bus; decide whether a working node's data are good (`TFC-SUP-010`); reset the last computer whose output agrees with the vote (`TFC-RESP-004`) |
| **Hardware overrides** (H1 to H5, not yet built) | Cut power or force a line by a switch, with no program in the path | Depend on anything running |
| **Simulator, platform, Pico** | The vehicle's physics and the motion the IMUs feel; the fault injector | Are part of the flight system (they are the test environment) |

## 3. Phases of an operation

| Phase | What happens | Operator | Ends by |
|---|---|---|---|
| **Bench** | The parts are checked (`P-M1-01`), each node on its own (`P-S1-01`), then in pairs and as the triple (`P-S3-01`, `P-S4-01`) | Runs the procedures; records `BENCH_LOG.md` | A stage passed |
| **Power-up** | The nodes boot and claim SYNC (the lowest healthy node is the master); sensors flow; the supervisor releases the units in order | Starts the rig; waits for the nodes to join | SYNC is up |
| **Pad** | Each computer calibrates its own IMU at rest for 10 s (1,000 samples), the estimators align to gravity, ACT goes Standby to Nominal with a neutral output | Watches the checklist (the go/no-go) | `launch` accepted |
| **Countdown** | 10 s from the `launch` command to `T0`; the sync master (or the supervisor) asserts the `T0` line; anything that stops being ready **scrubs** | May `scrub` (only before `T0`; there is no hold) | `T0`, or a scrub back to the pad |
| **Flight** | The clamps open at `T0`; the guidance and gain schedules follow the frames since `T0`; the calibration is frozen | Watches; resolves a Safe request; may reintegrate a latched computer once it is ready | The end of the run |
| **After** | The log, the campaign table and the replay are kept (`console/demo/`, `logs/`) | Reads them; a recording replays to the same events | |

## 4. The go/no-go, and what the operator may decide

The sync master accepts `launch` only if **all four** hold (`launch_gate.hpp`): three healthy flight computers in Triplex, none latched or on probation; no Safe request up; every computer ready (its calibration, sensors and attitude); ACT Nominal. The console shows the same checklist (its answer equals the command line's over 400 random states: `TFC-CON-002`). A `launch` that is refused names the reasons; the console refuses to send one while the checklist is NO-GO unless forced (`TFC-CON-005`).

**Commands** are authenticated ground frames (a SipHash tag over the key, the operation, the node, a counter and an ARM flag; a replayed or stale counter is refused; `ground.hpp`). `reintegrate`, `launch`, `scrub`, `phase`, `warm` and `noop` are plain; `disable`, `clear-disabled` and `clear-safe` need an ARM (a first frame that arms, a second that executes within a window of frames, one ARM for one execute). The key is the public bench key unless provisioned: **the uplink is bench-grade** (`LIMITATIONS.md` 5).

## 5. What the flight does with a fault, by phase

The principle (`FAULT_RESPONSE.md` 1): respond to the consequence, not the cause. In Triplex the 2-of-3 mid-value select masks a single fault in the same frame; detection and isolation matter for the second. The phase decides what losing redundancy means:

| | Pad | Ascent (Triplex) | Coast (Duplex + warm spare) | Burn |
|---|---|---|---|---|
| one computer or sensor lost | **no go**: the launch is held until it is repaired | continue; no replacement while a burn runs | promote the warm spare (probation first) | continue |
| down to Simplex | no go | continue and **alert**: no automatic abort | alert; the operator decides | continue and alert |
| an unattributable disagreement | Safe: neutral, platform level | freeze, then null after `T_hold` | freeze | freeze, then null |
| no valid vote | neutral | freeze, then null | freeze | freeze, then null |

**What the operator is asked to decide:** to launch or scrub; which computer to trust when the system cannot tell (a Duplex disagreement with no attribution holds the last good value and requests Safe, which **only an operator ends**, ADR-008); when to readmit a computer (after its dwell, 50 or 200 frames, and 100 or 300 agreeing frames of probation); to disable a repeat offender (the system disables a computer after three latches, two if the cause is physical, until a maintenance `clear-disabled`). **What the operator is never asked to decide in flight:** an abort, a reset of a working computer, or a change of the control law.

## 6. Anomalies the operator can see, and what to do

| The console says | It means | The operator |
|---|---|---|
| a node `LATCHED OUT` with a reason | the manager isolated it (vote, digest, stuck, missing frames) | nothing in Triplex; read the reason; `reintegrate` after the dwell if the cause is gone |
| Duplex | one computer or sensor is out | continue (ascent); on the pad, no-go |
| `SAFE REQUESTED` | an unresolved disagreement persisted (3 of the last 5 frames) | decide which computer to trust (`disable` under an ARM), then `clear-safe` under an ARM once the exit conditions hold (at least two voting) |
| `BUS ALARM` | three out-of-schedule frames in one frame time | find the babbler by elimination (`hold` a node at the supervisor); the manager never holds a node on this alone |
| `RESET LOOP` | three short boots in a row | the node is held out and silent until it is power-cycled |
| `PREVIOUS RUN ENDED IN A FATAL FAULT` (new: `firmware/README.md`) | a fatal fault stopped the last run: the console prints the reason, the program counter and the last frames | keep the console text with the bench log |

## 7. What the system does not do

No automatic abort and no range-safety function (`RANGE_SAFETY.md`); no recovery from two unattributable faults except Safe; no operation without the authenticated command path; nothing here protects against a fault that is the same in all three computers (`LIMITATIONS.md` 2). The single points are the actuator node, the shared bus, the supervisor (fail-passive) and the single 5 V adapter (`LIMITATIONS.md` 1).
