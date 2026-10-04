# Fault response: what the flight computers do, and what the vehicle does

> Status: **proposed**. It joins the fault matrix (`FAULT_MATRIX.md`: what is detected and how) with the vehicle's side: what the
> output does, who acts, what the operator does, and how it ends. Rows marked "today" describe the current software; everything else
> is the intended behaviour and needs the Safe action, the supervisor or the phases (`SAFE_MODE.md`, `SUPERVISOR.md`,
> `MISSION_PHASES.md`). Detection times are those of the matrix (10 ms frames). Numbers are proposals.

## 1. The principle: respond to the consequence, not the cause

Two questions decide the response, whatever broke:
1. **Is the output still trustworthy?** If yes, keep flying on it. If no, it must not move the vehicle.
2. **Is redundancy reduced?** If yes, say so loudly and decide what the phase allows. A second fault of the same kind is the case to survive.

In Triplex, the 2-of-3 mid-value select **masks a single fault in the same frame**, before any detection: the output stays within one
tolerance of the truth (oracle S2) and, for the loss of a computer, within 0.5 degrees per frame (TFC-SYS-004). Detection and isolation
matter for the *second* fault, not the first. This is why detection times of 2 to 40 frames are acceptable.

## 2. Response classes

| Class | Name | Output (what the vehicle sees) | Who acts | Operator |
|---|---|---|---|---|
| R0 | Mask | Unchanged | The vote and the 3-of-5 filter | Nothing; counted |
| R1 | Flag | Unchanged | The manager, the supervisor | Informed through telemetry and events (IF-005, IF-006) |
| R2 | Isolate and continue | Unchanged; Triplex to Duplex (fail-operational) | The manager | Informed; may `reintegrate` later |
| R3 | Degrade and alert | Continues on fewer nodes (Duplex to Simplex, or below the phase minimum) | The manager; the phase table decides what the phase allows | **Alerted**; decides (see phase rules, section 5) |
| R4 | Hold and request Safe | **Frozen** at the last good value, flagged held; Safe requested (sticky) once the disagreement persists 3 of 5 frames | The manager | Resolves: names the node to trust (`disable` under an ARM), then `clear-safe` |
| R5 | Safe | Freeze at once, then null after `T_hold` at the rate limit (`SAFE_MODE.md`) | ACT, alone if needed | `clear-safe` under an ARM once the exit conditions hold |
| R6 | Hardware action | Unchanged by itself; the node concerned is reset, power-cycled or held in reset | The supervisor | Informed; may `release` |

R6 is an action on a *node*, and combines with R2 (a silent node is both isolated by the others and reset by the supervisor). R3 to R5
are the ladder of `SAFE_MODE.md` section 8: the lowest adequate rung is used.

## 3. By fault family

"Today" = behaviour of the current manager (checked by the campaign). Matrix ids in brackets.

### A. A computer is lost, silent or hung
F01 power loss, F02 hang, F12 cut from the bus, F24 reboot, F39 partial (some frame types missing), F57 partial hang, F58 wedged
firmware, F68 reset loop; and F19 a single lost frame.

| | |
|---|---|
| Detected | 2 to 3 frames of missing frames (FDIR-001); a single lost frame is one bad sample and nothing more (FDIR-016); the supervisor sees missing `KICK`s within 3 frames |
| Class | **R2** (+ **R6** from the supervisor); a single lost frame is **R0** |
| Vehicle | Unchanged in Triplex. In Duplex the loss of a second computer is R3 (Simplex), and with the last one gone ACT sees no votes and enters Safe by itself (F69) |
| Operator | Informed. After a reset the computer comes back through dwell and probation; `reintegrate` when ready. Three latches (two if physical) disable it until a maintenance `clear-disabled` under an ARM |
| Recovery | Latched, then dwell (50 or 200 frames), operator request, 100 agreeing frames (FDIR-006). A loop of resets ends in `hold` and a report (FDIR-042) |

### B. A sensor channel reads wrong
F03 stuck, F04 bias, F05 drift, F06 spike, F07 wild or non-finite, F27 to F36 (scale, noise, inversion, swap, zero, clip, vibration,
rate mismatch, bit flip, stuck bit), F65 an IMU fails with a healthy computer.

| | |
|---|---|
| Detected | Vote disagreement, 2 frames for a step; a drift after it passes the tolerance (about 25 frames for 0.05 per frame); a stuck sensor 7 frames (bound 40); a lone spike is filtered (3-of-5) |
| Class | Today **R2** (the whole computer is isolated). After ARCH-001: **R2 on the sensor channel only**; the computer stays a command voter |
| Vehicle | Unchanged in Triplex. Sensing falls to two channels (Duplex sensing) while commands stay Triplex; a second bad channel is R3, and two sensor channels that cannot be told apart are R4 |
| Operator | Informed; the channel can be reintegrated after its dwell like a node (ADR-010) |
| Recovery | As A; a physical cause (stuck, intermittent) disables the channel on its second strike |

### C. A computer's command is wrong
F09 wrong-but-valid command, F37 frozen, F38 inverted, F50 wrong or frozen in Duplex while the vehicle moves.

| | |
|---|---|
| Detected | Vote on the command, 2 frames; in Duplex by continuity with the motion (ADR-017) |
| Class | Triplex **R2**. Duplex, attributable: **R2/R3** (the consistent node is followed). Duplex, not attributable: **R4** |
| Vehicle | Triplex: unchanged. Duplex: **held** until the persistence window decides, then the consistent node, or Safe. Never follows the node that stopped following the motion (FDIR-028) |
| Operator | R4: names the node to trust (`disable` under an ARM in Duplex), then `clear-safe` |

### D. Timing and sequence faults
F25 late, F40 duplicates, F41 replays, F42 frozen frame number, F43 early, F44 jitter, F45 clock drift, F67 slow drift, F08 corruption.

| | |
|---|---|
| Detected | Sequence errors (frame-number phase check, ADR-018): 2 to 3 frames; late data by the vote and digest; jitter as missing frames when a frame crosses the 7 ms vote; slow drift by the supervisor's clock (SUP-004) |
| Class | **R2** (with **R1** reports from the supervisor). A frame that is early *inside its own window* (about 4.5 ms) is the known blind band: today it is not seen (**R1** once the arrival-time telemetry exists, FDIR-037) |
| Vehicle | As A: unchanged in Triplex |
| Operator | Informed |

### E. The estimator state diverges
F10 digest mismatch, F51 three different digests, F47 an upset in the manager's own state, F48 invalid configuration.

| | |
|---|---|
| Detected | Digest cross-check, 1 frame (FDIR-011); the manager's own scrub, 1 frame (FDIR-025); configuration at construction (FDIR-026) |
| Class | Triplex with a majority: **R2**. No majority (three digests) or Duplex: nobody can be blamed, **R4**. A manager upset: the node is excluded on the safe side and Safe is requested if needed; an invalid configuration is replaced by its default (**R1**) |
| Vehicle | R2: unchanged. R4: frozen, then the operator decides |
| Operator | R4: names the node to trust, then `clear-safe` |

### F. The bus
F11 babbling, F13 bus fully lost, F14 bus-off, F15 sync master lost, F49 an unlisted id, F59 an alarm that persists.

| | |
|---|---|
| F11, F49, F59 | Detected by the out-of-schedule filter (3 per 10 ms frame, FDIR-019). **R1**: an alarm, nobody blamed (an id cannot be attributed to a node, ADR-009); ACT ignores unlisted ids; vehicle unchanged. If it persists, the operator holds nodes in reset one at a time with the supervisor (`hold X`) to find the source (FDIR-040) |
| F14 bus-off | The controller recovers on its own within 100 ms (FDIR-010). Meanwhile ACT sees no votes: it **freezes** for up to 10 frames (within `T_hold`, so no null is commanded for a recovery this short) |
| F13 bus lost | ACT sees no valid vote for 3 frames and runs Safe on its own: **R5** (F69). The computers cannot help; the supervisor sees their `KICK`s still arriving, so it does *not* reset them (the fault is not theirs) |
| F15 sync master lost | The next computer takes over within 2 frames, with the frame number continuous (ADR-018): **R2**-like, unchanged output |

### G. The actuator node
F17 reset or brownout, and the loss of ACT's own output.

| | |
|---|---|
| Detected | The supervisor's missing `KICK` from ACT, 3 frames; ACT's own watchdog |
| Class | **R6** on ACT, then **R5**: after any ACT reset ACT starts **in Safe**, as the lecture's reset pattern does, not in the nominal mode |
| Vehicle | **Open problem.** A servo with no signal may hold or go limp (not known for the D85MG; measure it). On boot ACT has no last command, so naive start-up would jump the output to neutral: a step. Proposal: ACT keeps its last output in memory that survives a reset (no-init RAM, CRC-checked) and resumes from it, then nulls at the rate limit; if that memory is invalid it starts from neutral. |
| Operator | `clear-safe` under an ARM when the exit conditions hold |
| Limit | While ACT is single (ADR-023) this is fail-passive, not fail-operational |

### H. Operator and command faults
F52 forged tag, F53 replay, F54 missing ARM, F55 disabling too many nodes, F56 an upset in the command state, F62 path check.

| | |
|---|---|
| Class | **R1**: dropped or refused, counted, nothing changes on the vehicle |
| Vehicle | Unchanged. A disable that would remove the last voter is allowed under an ARM and reported CRITICAL (ADR-019); the vehicle then has one voter (R3) |

### I. Recovery and total loss
F21 transient fault, F22 persistent fault with an operator request, F23 repeat offender, F46 total loss, F60 forgotten state after a restart.

| | |
|---|---|
| Behaviour | As ADR-010 and ADR-014: latched, dwell, probation, readmission or refusal, disabling on the third latch. Total loss: with all nodes latched the probationers judge each other. Vehicle: during a total loss ACT has no valid votes, so it runs Safe (**R5**); readmission needs the operator and the exit conditions of `SAFE_MODE.md` |

### J. The new cases of the proposals
| Fault | Class | Vehicle | Notes |
|---|---|---|---|
| F63 two nodes of one release wrong the same way | **R4** (ADR-021) | Frozen; the operator picks the side | Today the healthy node is latched and the vehicle follows the wrong pair (measured) |
| F66 the supervisor fails | none (fail-passive, SUP-007) | Unchanged | Protection is lost, function is not; detectable on the PC |
| F70 Safe exit before its conditions hold | refused | Stays in Safe | |
| F71, F72 a phase change or a promotion that cannot be met | refused | Phase held | |
| F73 IMU A and computer B both fail | **R2/R3** on sensing | Unchanged; sensing on one IMU (or two after a re-homing, ADR-020) | The vehicle needs only one good sensor channel and a computer pair |
| F18 an engine out | not a computer fault | The controller saturates and compensates; no Safe | Needs a test in Duplex: a legitimate large step in the command must not look like a fault to the motion-aware arbitration |

## 4. What the vehicle must never do
These are the properties the responses exist to protect; the campaign oracles check the ones that exist today.

1. Follow a node that is wrong (S2, FDIR-028).
2. Step by more than 0.5 degrees in a frame because one computer failed (SYS-004).
3. Isolate a healthy node for a single fault (S1); **and, for the proposals, not isolate the healthy node when two nodes of one release agree on a wrong value** (F63, ARCH-007).
4. Change the output while held, except by repeating the last good value exactly (M7).
5. Leave Safe without the operator (M4; SAFE-003).
6. Move on the pad (phase P2).
7. Run away while in Safe: freeze, then a rate-limited ramp, never a step (SAFE-001).

A new oracle is proposed for the campaign once the Safe action exists: **S5, the output step per frame stays within the rate limit in
Safe and within SYS-004 for the loss of one computer.** The per-frame dump (`tfc_replay --dump`) already carries the voted output and the
held and Safe flags.

## 5. The phase decides what R3 and R4 mean

| Event | P2 Pad hold | P3 Ascent (Triplex) | P4 Coast (Duplex + warm spare) | P5/P6 Pre-burn, burn |
|---|---|---|---|---|
| R2, one computer or sensor lost | **No go**: hold the launch until it is repaired (go/no-go needs 3) | Continue; no replacement while a burn runs | Promote the warm spare (probation first) | P5: promote and re-check before the burn; P6: continue |
| R3, down to Simplex | No go | Continue; alert; **abort criteria are a decision for the owner** (open question 1) | Alert; operator decides, spare promoted | Continue; alert |
| R4, unattributable disagreement | Safe: servos neutral or depowered | Freeze, null after `T_hold` | Freeze only | Freeze, null after `T_hold` |
| R5, no valid votes | Neutral or depowered | Freeze, then null | Freeze | Freeze, then null |

## 6. Open questions for the owner
1. **Abort.** Should Simplex in ascent trigger a hold or abort message to the simulator, or only an alert? (A real vehicle has abort rules; the rig can model one as a message.)
2. **ACT reset.** Resume from the stored last output (proposed) or always start at neutral? And what does the D85MG do with no signal? (a measurement for M2).
3. **Which phase minimums** are right, with the pad needing three and ascent needing two? (`MISSION_PHASES.md`.)
4. **Whether the supervisor may reset a computer while its output is the only one agreeing** (a Duplex where one computer is the last good one): proposed no, only the operator may (`hold`).

## 7. What this adds to the requirements
TFC-RESP-001 to 004 in `REQUIREMENTS.md`: every fault-matrix row names a response class; the table of section 5 is the phase rule; ACT starts in Safe after a reset and resumes from a stored output; the supervisor never resets the last agreeing computer on its own.
