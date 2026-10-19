# Hazard analysis: losses, hazards, and the unsafe control actions of the control structure

> Status: **written** (18 Oct 2026): a system-level hazard analysis in the manner of STPA (Leveson, "Engineering a Safer World", 2011, and the STPA Handbook, from memory), which the project did not have: the FMEA (`FMEA.md`) is by fault, this is by hazard and by control action. It was written from the design pages and checked against the requirements and tests they name; it is **one person's analysis, not reviewed by anyone else**, and it analyses the demonstrator as if it were a flight system, which it is not. Where the design has no control for a scenario, the table says so: those are the findings.

## 1. Losses and hazards

**Losses** (what the system must not allow, for the purpose of the analysis, a flight vehicle's): L1 the vehicle is lost or leaves its corridor; L2 loss of control of the attitude (a runaway); L3 an uncommanded motion at the pad (people, the platform); L4 loss of the mission's data (a run that cannot be understood afterwards); L5 an unintended release of the vehicle (launch).

| Hazard | Leads to | Statement |
|---|---|---|
| H1 | L1, L2 | The actuator's output is wrong (a step, a hard-over, a frozen value) while the vehicle is flying |
| H2 | L2 | The output is **correct but too late** (a stale command applied) |
| H3 | L1, L2 | The system continues on a degraded vote without the operator knowing |
| H4 | L5 | The vehicle is released (T0) when the system is not ready |
| H5 | L3 | The platform or a servo moves when it should not (the rig's only physical hazard) |
| H6 | L4 | The record of what happened is missing or cannot be trusted |
| H7 | L1, L2 | A fault that is the same in all replicas takes all of them at once |
| H8 | L1, L2 | An engine fails and the guidance does not know it (it counts thrust and flow that are not there, and lights engines that cannot run): the engine controllers' health word (`0x509`, `0x50A`, ADR-058) is the defence, with the loop's integrator behind it |

## 2. The control structure

Operator -> console / `tfc_peers` -> **authenticated ground frames** -> flight computers (judge, vote, command) -> **ACT** (vote, latch, Safe) -> actuator / simulator. Beside it: the **supervisor** (discrete lines to every node and to ACT's `SAFE`) and the **hardware overrides** (switches in the power and signal paths). Feedback: heartbeats, the voted output frame, the simulator's truth, the console's reading of the bus.

## 3. Unsafe control actions

Each row: a controller, a control action, how it is unsafe (not given / given when it should not be / too early or late / stopped too soon or applied too long), the hazard, and what in the design controls it (requirement and test where they exist). **GAP** marks a scenario with no control.

| # | Controller: action | Unsafe when | Hazard | Control in the design | Status |
|---|---|---|---|---|---|
| U1 | ACT: output the voted command | **given when the three disagree beyond tolerance** | H1 | mid-value select masks one wrong command in the same frame (`TFC-SYS-004`); ACT excludes a node after persistent disagreement; `act_votes_out_a_wrong_node_and_excludes_it_after_persistent_disagreement` | controlled for one fault |
| U2 | ACT: output | given from **two** agreeing wrong computers (a common cause) | H1, H7 | none by voting; node C runs the previous release (ADR-021) against a regression, not a bug in both | **GAP** (`LIMITATIONS.md` 2) |
| U3 | ACT: output | **not given** (ACT silent, hung) | H1, H2 | fail-passive: the servo holds or goes to neutral per `SAFE_MODE.md`; the supervisor watches ACT's `KICK` and `FRAME` and resets it | partly: the ACT node is a single point (`LIMITATIONS.md` 1) |
| U4 | ACT: output | **applied too long** (a stale value kept after the vote is gone) | H2 | ACT's Safe sequence freezes then nulls after `T_hold` at a rate limit; `TFC-RESP-003`, `act_starts_in_safe_after_a_reset_and_resumes_from_the_stored_output` | controlled |
| U5 | ACT: Safe (freeze, null) | null **too fast** (a step to neutral is itself a hard-over) | H1 | the rate limit on the ramp (`SAFE_MODE.md` 4) | controlled by design, rig pending |
| U6 | Flight computer: command | wrong but valid (a frozen or inverted command) | H1 | peer judging by vote and digest; in Duplex by continuity with the motion (ADR-017) | controlled; Duplex unattributable: Safe |
| U7 | Flight computer: heartbeat | **not given** while the computer is outvoted (it hides its state) | H3 | ACT and the supervisor judge by frames, not by the computer's own claim | controlled |
| U8 | Manager: isolate a node | **given for a healthy node** (a false latch) | H3 | persistence 3 of 5, dwell, probation; strikes; the campaign (12,362 scenarios) has no property violated | controlled at the campaign's coverage |
| U9 | Manager: isolate a node | **not given** for a faulty node (a missed detection) | H1 | the vote still masks it; the blind band of an early frame (`LIMITATIONS.md` 2) is not seen | **GAP** (waits for arrival-time monitoring on the board) |
| U10 | Manager: Safe request | **cleared by something other than an operator** | H1 | the request is sticky; `clear-safe` needs an ARM and the exit conditions; `bounded check P4` (`tests/test_model_check.cpp`) | controlled |
| U11 | Operator: `launch` | given when not ready | H4 | the go/no-go (four conditions), checked again every frame of the countdown (a failure scrubs); the console refuses unless forced | controlled; a forced launch is the operator's |
| U12 | Operator: `launch` | given **twice** (a second launch after a reset) | H4 | the supervisor resumes the mission clock after its own reset and refuses a second launch (`sup_after_its_own_reset_...`) | controlled |
| U13 | Operator: `scrub` | given **after** T0 | H4 | refused after T0 (there is no hold or scrub in flight) | controlled |
| U14 | Operator: `reintegrate` | given for a node that is still bad | H3 | probation: 100 (300 for a repeat) agreeing frames before it votes again; a repeat offender is disabled | controlled |
| U15 | Operator: `clear-safe` | given while the cause persists | H1 | refused unless two voters and no request active; ARM needed | controlled |
| U16 | Operator: any command | forged, replayed or damaged | H1, H4 | SipHash tag, counter window, ARM expiry (`ground_*` tests); **the key is the public bench key and the comparison is not constant-time** | **GAP** for a real uplink (`FUTURE_WORK.md` 2.4) |
| U17 | Supervisor: reset or power-cycle a node | given for a **working** node | H3 | it acts only on a node that has stopped (`TFC-SUP-010`); never on the last agreeing computer (`TFC-RESP-004`) | controlled by construction |
| U18 | Supervisor: reset | **given in a loop** (a reset storm) | H3 | three resets in a minute become a power cycle, two cycles make it DEAD; the node's own reset-loop rule | controlled |
| U19 | Supervisor: `T0` line | asserted without a `launch` | H4, L5 | only after the countdown from an accepted `launch` or its own authenticated command; the nodes also take T0 from the sync master | controlled on the host; the line is not wired |
| U20 | Hardware override | **engaged by accident** | H5, H3 | a launch is refused while an override is engaged that nobody acknowledged | controlled in software; the parts are **not ordered** |
| U21 | Hardware override | **dead when needed** (a contact that never made) | H5 | `P-HWO-01` operates each one; the supervisor reports the untested ones | waits for the hardware |
| U22 | Platform driver: servo motion | beyond the platform's limits, or continuing after the PC link is lost | H5 | rate and travel limits; holds after 100 ms without a command and levels after 1 s (`TFC-PLAT-002`) | controlled; the E-stop and the stall current wait for the rig |
| U23 | Flight computer: guidance command (host only) | wrong because the model of the stage's descent is wrong | H1 | none flown on a board; the whole-mission Monte Carlo finds **no catch with the assumed dispersions** | **GAP** (`MONTE_CARLO.md` 7) |
| U24 | Recorder | no record of a fatal fault | H6 | the fault record and the last 128 frames (`firmware/README.md`) | built; unchecked on a board |
| U25 | Flight computer: timing | a frame **late** past the vote (a long guidance cycle on the target) | H2 | a missed vote is a missing frame; the WCET is not measured on the target | **GAP** (`LIMITATIONS.md` 3, 10) |

## 4. Reading it

Of 25 unsafe control actions, 16 are controlled by a mechanism with a requirement and a test, one is controlled in part (U3: the actuator node is a single point), 3 are controlled only on the host or until the hardware exists (U19, U20, U21: the line and the override parts), and **5 have no control or an incomplete one: U2 and H7 (a common-cause bug), U9 (the early frame the check cannot see), U16 (the uplink), U23 (the descent guidance), U25 (the execution time on the target)**. The first is the hazard a reviewer will ask about, and the one that no design of this kind can close with voting: the argument for it is diversity, which is one previous release.

**What it did not do:** no quantitative probabilities (the Markov model of TS-2 uses assumed relative rates); no analysis of the physical rig's own hazards beyond U22; one analyst.
