# Safe mode: research and proposal

> Status: **accepted for Safe mode, 4 Oct 2026** (the open questions in section 10 were answered by the owner); the phase table it refers to is still a proposal (resolves the "define the Safe action" item of `DEFERRED.md` section 3 as far as it can be resolved before the
> actuator node exists). All numbers are proposals. Sources were read as search summaries, not in full; check them before quoting them in
> the write-up (section 9).

## 1. What Safe means today

When the flight computers cannot produce a trustworthy output (an unresolved Duplex disagreement, no majority, no data) the manager holds the
last good value and raises a **sticky Safe request** that only an operator clears (ADR-008, FDIR-008, FDIR-018). What the *vehicle* does in
Safe is not decided: "hold the last gimbal command" is the placeholder. This page decides it.

## 2. What the lecture says safe mode is

For a spacecraft: all non-critical devices off, the most basic attitude control (typically an IMU and Sun sensors), a low-gain antenna for
robust contact with the ground, a safe attitude with the arrays at the Sun. Its purpose is to be survivable, not useful: it "buys the
ground time". It is entered by fault monitors, by an uplink-loss timer, or on a software reset, from a configuration file that is already
written. The deck's test of a good safe mode is that the spacecraft **stays safe through a fault, including a computer swap with loss of
attitude and thermal control**. Its other warnings apply here: a monitor with no persistence trips on one noisy sample, and a disabled
monitor protects nothing; so prefer a lower response first (standby before safe), and make thresholds and persistence parameters.

A launch vehicle on ascent differs in one way that matters: there is **no safe attitude to coast in** and no ground that can act in
seconds, so Safe cannot be "stop and wait for help". The rig is a desk model of ascent, so the design goes by the control engineering
definitions, not the spacecraft ones.

## 3. Definitions we use

| Term | Meaning | Source |
|---|---|---|
| Fail-operational | After a first failure the system carries on doing its job | flight-control references; the lecture ("needs a spare already running") |
| Fail-passive | After a failure the system stops doing its job but does not freeze, run away or cause a sudden, undesirable motion | flight-control references |
| Fail-safe | The system drops to a protective state (neutral, retract, mechanical mode) | flight-control references; the lecture |

The project is **fail-operational for the first fault** in the computing and sensing chain (Triplex to Duplex) and **fail-passive for the
second** unresolved fault (hold, then Safe). Safe is therefore the fail-passive state, and what it must avoid above all is a **runaway**.

## 4. How a thrust-vector actuator fails

Thrust-vector-control references describe three failure modes of the actuator or its command: **hard-over** (to a limit),
**fail in place** (frozen), and **fail to null** (to neutral). A hard-over is the worst case for the vehicle; frozen is survivable for a
short time; null is a known position. This is why the Safe action must (a) never step, (b) not hold an off-centre command forever without a
decision, and (c) end in a position everyone agrees on.

## 5. Options for the Safe action

| Option | What the output does | For | Against |
|---|---|---|---|
| S0 hold forever (today) | Last good command, indefinitely | No motion at all | A held off-centre command can be wrong for long on a real vehicle; no defined end |
| **S1 hold, then null (proposed)** | Freeze at once; after `T_hold`, ramp to neutral at a limited rate; hold neutral | No step; gives a transient and the operator time; ends in a known position | A frozen command costs `T_hold` of wrong steering if the last value was bad |
| S2 null at once | Ramp to neutral immediately | Shortest exposure to a bad held value | A fast move can itself be a disturbance; loses the chance that a transient clears |
| S3 depower the servos | Remove drive | Simplest, nothing can run away | The platform goes where gravity and friction take it; fine on the ground, not on a rig carrying an IMU you want to keep calibrated |
| S4 abort the run | Tell the simulator to end the scenario | Matches range-safety logic | Not a flight-computer decision; a message to the simulator |

**Proposal: S1 in ascent, S3 on the pad, S4 as an additional message to the simulator.** Per phase (`MISSION_PHASES.md`):

| Phase | Safe action |
|---|---|
| Power-up, checkout, pad hold | Servos depowered or neutral; nothing moves |
| Ascent (all three hot) | Freeze, then null after `T_hold` at the rate limit; message to the simulator |
| Coast / burn-free | Freeze; null only on operator command (no steering needed) |
| Recovery after a Safe | The operator clears Safe (ARM) once the exit conditions hold |

## 6. Safe configuration (parameters, not code)

Per the lecture, the thresholds and timers are parameters with defaults in non-volatile memory, validated like FDIR-026 (a bad value is
replaced by its default and reported). Defaults:

| Parameter | Proposal | Why |
|---|---|---|
| `T_hold` | 50 frames (0.5 s) | long enough for a transient and the operator's reflex, short enough that a bad held value cannot do much |
| neutral command | pitch 0, yaw 0 | mechanical centre of the gimbal |
| rate limit to neutral | one quarter of full travel per second | slow enough to be no step; fast enough to arrive in 4 s |
| votes lost before ACT enters Safe alone | 3 frames | matches FDIR-001 |

## 7. Who enters Safe, and who leaves it

| Enters Safe | How |
|---|---|
| The flight computers | The sticky Safe request in the vote status (ADR-008) |
| ACT, by itself | No valid vote for 3 frames (all computers lost, or the bus lost), with no help from anyone |
| The supervisor or the operator | The hardware `SAFE` line (`SUPERVISOR.md`), which needs no bus and no flight software |
| The E-stop | Cuts the servo rail (already on the parts list); outside the electronics |

Leaving Safe takes **all** of: at least two healthy, agreeing flight computers for 100 frames, ACT seeing valid votes again, and an operator
`clear-safe` under an ARM (ADR-019). Nothing leaves Safe by itself, so a flapping fault cannot toggle the output.

## 8. The response ladder

The lecture's advice on persistence and on a lower response first becomes an explicit ladder; each rung needs its own persistence and the
lowest adequate rung is used:

1. Ignore a single glitch (3-of-5 window, leaky count: ADR-007, ADR-013).
2. Flag and count it (telemetry, IF-005).
3. Isolate the node or sensor channel (ADR-010).
4. Degrade: Triplex to Duplex to Simplex.
5. Hold the output and request Safe (an unresolved disagreement).
6. Safe: freeze, then null (this page).

## 9. Sources (search summaries; verify before citing)

- ASTE-331 lecture "Avionics, C&DH and Flight Software" (2 Oct 2026): safe mode, fault monitors, redundancy, fail-operational. Not for redistribution.
- Fail-operational and fail-passive definitions: [MIT OCW 16.885, Flight Controls](https://ocw.mit.edu/courses/16-885j-aircraft-systems-engineering-fall-2004/de14f682fa69f4e7379d7ae861f7f4a2_flight_controls_2.pdf); [US patent 4612844, fail-passive actuator control](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4612844).
- TVC actuator failure modes (hard-over, fail in place, fail to null) came out of a search over several documents and are not tied to one source here; find the primary reference before citing. A related read: [Georgia Tech repository record, SRB control limitations on loss of a hydraulic power unit](https://repository.gatech.edu/entities/publication/4f8cb417-f6d3-4780-8585-b4083b76d39e) (about command-position deltas; not read in full).

## 10. Decisions (owner, 4 Oct 2026)

| # | Question | Decision | Why |
|---|---|---|---|
| 1 | Null the gimbal, or depower the servos, on the rig? | **Null in ascent; depower on the pad** | A depowered 2-axis platform droops onto its stops and the IMU on it is thrown about; nulling keeps it under control |
| 2 | Should Safe stop the simulated run? | **No.** The simulator is told (a flag), keeps flying with the frozen or nulled gimbal, and marks the run "safed"; only an operator **abort** ends it (TFC-SAFE-008) | The continued run is the data for TS-4: how long the vehicle stays recoverable |
| 3 | A 0.5 s hold, or null at once? | **0.5 s as a parameter** (`T_hold`); TS-4 chooses the value from data | A parameter, not a belief |

## 11. Requirements and tests

TFC-SAFE-001 to 008 (`REQUIREMENTS.md`); fault-matrix rows F69 (ACT alone enters Safe) and F70 (Safe exit refused without its conditions).
