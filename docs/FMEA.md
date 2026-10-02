# Fault coverage: an FMEA-style gap analysis

> Status: working document, last updated with the campaign run recorded in `docs/FAULT_CAMPAIGN.md`. "Modelled" means the virtual peers can inject it; "Covered"
> means a test checks the response. SpaceX-related statements are from public material or inference.

## 1. How real programs think about this

Space and aerospace programs do not start from "which faults can I inject" but from a **Failure Modes and Effects Analysis**
(FMEA/FMECA): for every function and every component, list the ways it can fail, what the failure looks like at the
interface, what it does to the system, how it is detected, and what the response is. The common reference points are
**ECSS-Q-ST-30-02C** (FMEA/FMECA in space projects: functional and hardware analysis, highest-criticality failures first),
**SAE ARP4761** and **MIL-STD-1629A** (aircraft and military FMECA), and for software **NASA-STD-8719.13** and NPR 7150.2
(fault tolerance, safety-critical software requirements, memory scrubbing against single-event upsets) and
**DO-178C** (airborne software, structural coverage). Launch-vehicle programs then verify the response in software-in-the-loop
and hardware-in-the-loop rigs with **fault injection** (NASA's SLS has a dedicated Avionics and Software Integration Lab for
exactly this). This project is the desk-scale version of that loop: the fault matrix is the FMEA, the virtual peers and
the campaign are the fault-injection rig.

What a flight-computer FMEA must cover, by interface:

| Interface | Typical failure classes |
|---|---|
| **Sensor (IMU)** | bias step, bias instability/drift, **scale-factor error**, **noise increase**, stuck output, **zero/null output**, saturation and **wrong full-scale range (clipping)**, spikes, **vibration-induced oscillation/aliasing**, **sample repetition (output data rate mismatch)**, **sign inversion / axis swap (misalignment)**, **bit errors in the sample before it is checksummed (SEU)**, stuck bits |
| **Compute node** | crash, hang, reboot (power glitch), **late/early/jittering output (timing faults)**, **clock drift**, wrong command (software bug), **frozen command output**, **inverted command sign**, state divergence (SEU in the estimator), **partial failure (only some frames sent)** |
| **Bus (CAN)** | bit errors, lost frames, **duplicated frames**, **stale replayed frames**, **frozen sequence counter**, babbling idiot, bus-off, open/short, ID collisions (two nodes with one ID), SYNC loss |
| **Redundancy manager itself** | wrong tolerance configuration, **memory upsets in its own state (flags, counters)**, a latent bug that only shows with two faults |
| **Operator / ground** | wrong command, command at the wrong moment, command that would remove redundancy |
| **Multi-fault** | two simultaneous independent faults, **common-cause faults (both peers wrong the same way)**, a fault during recovery |

Items in **bold** were missing from the first version of the virtual peers; the campaign work added them (section 3).

## 2. What was already covered (13 kinds)

dropout, stuck, bias, drift, spike, saturate, corrupt, cmd_offset, digest, babble, seqgap, reboot, late; plus the
`period`/`duty` intermittent option on all of them, scripted operator commands, and node targeting on A, B and C.

## 3. Gaps found and what was added (now 32 kinds)

Every bold item in section 1 became a fault kind (matrix rows F27-F45). The campaign (`docs/FAULT_CAMPAIGN.md`) then found
three *response* gaps that no new fault kind could have shown, and a coverage gap in the checks themselves:

| Class from section 1 | Kind | Row | Measured response |
|---|---|---|---|
| Scale-factor error | `scale` | F27 | isolated when the error exceeds the tolerance; a 0.5% error on gravity is left alone |
| Noise increase | `noise` | F28 | isolated from about 20x the healthy noise; 3x and below is ignored |
| Sign inversion, axis swap | `invert`, `swap` | F29, F30 | isolated in 3 frames |
| Zero / null output | `zero` | F31 | isolated in 3 frames, also at rest (gravity disappears) |
| Wrong full-scale range (clipping) | `clip` | F32 | isolated only when the limit clips real signal |
| Vibration / aliasing | `oscillate` | F33 | isolated in about 4 frames above 1.2 tolerances; a 100 Hz oscillation aliases to a constant offset |
| Sample repetition (data-rate mismatch) | `repeat` | F34 | isolated after 7 frames for n of 12 or more |
| Bit errors in the sample before the CRC (SEU), stuck bits | `bitflip`, `stuckbit` | F35, F36 | the CRC cannot see them; the vote does, from bit 4 (gyro) / bit 6 (accel) upward |
| Frozen / inverted command | `cmdstuck`, `cmdinvert` | F37, F38 | isolated in 3 frames; in Duplex see below |
| Partial failure | `partial` | F39 | isolated in 3 frames (missing) |
| Duplicated frames, replayed frames, frozen sequence counter | `duplicate`, `replay`, `seqstuck` | F40-F42 | isolated in 3 frames (sequence / stale data) |
| Early / jittering output, clock drift | `early`, `jitter`, `clockdrift` | F43-F45 | jitter and drift isolated once frames cross the 7 ms deadline; a consistently early gyro stream is **not** detectable (E11) |
| Common-cause faults | `correlated` group | - | two nodes with the same error: the healthy node is outvoted and blamed (voting cannot help; needs design diversity) |
| Multi-fault and fault during recovery | `pairs`, `new_pairs`, `cascades` groups | - | 1,788 combinations, no property violated |

Response gaps found by the campaign and fixed (details: `docs/FAULT_CAMPAIGN.md` section 6):

| Gap | Where it shows | Fix |
|---|---|---|
| **A frozen or stale command in Duplex got the *healthy* node isolated** and the faulty node's command onto the output | F37, F50: 63 of 80 start phases | motion-aware arbitration reference (ADR-017) |
| **Total loss was unrecoverable**: all nodes latched, no reference, only a reset could help | F46 | cohort probation (ADR-014) |
| **Three different digests latched every node**: a self-inflicted total loss on a no-consensus situation | F51 | unresolved, hold, Safe request (ADR-008 amended) |
| **No protection of the manager's own state**; a zero or NaN tolerance silently disabled detection | F47, F48 | guarded state, scrub, validated configuration (ADR-015) |
| Dwell after `clear-disabled` / failed probation counted the frame of the event | E1 | one-frame off-by-one |
| Any CAN id above 0x500 counted as "known" (a babbler there was invisible) | F49 | only the ids in the schedule are known |

## 4. Known classes that cannot be modelled in software alone (need the rig)

| Class | Why |
|---|---|
| ID collision / duplicate node ID | On real CAN two transmitters with the same ID corrupt each other's frames (bit errors, error frames, possibly bus-off): a hardware effect, not two clean valid frames |
| Inconsistent frame omission (one receiver misses a frame others got) | Needs more than one receiver; FC-A is the only receiver in the model |
| Bus-off, short, open circuit, ground offsets | Electrical |
| Real latency and jitter, WCET | Measured on the target, never from Python on a desktop |
| Common-mode software bug in all replicas | Voting cannot see it; needs design diversity (see ARCHITECTURE section 7) |
| SEU in the manager's own memory | Modelled by corrupting the manager's state in a unit test (`seu_*`), not by a bus fault; the repair is ADR-015. Protection of the *code* (flash ECC, watchdogs) is hardware |
