# Decisions taken and work deferred

> Each campaign edge case (docs/FAULT_CAMPAIGN.md section 6) was either fixed, accepted, or **deliberately postponed until the
> thing it depends on exists**. This page is the register of the postponed work: what it is, why it waits, what triggers it, and
> the design notes written down now so nobody has to re-derive them. "Trigger" is the milestone or artefact whose arrival
> makes the work possible. Nothing here is a hidden requirement: items with a requirement id are listed in `docs/REQUIREMENTS.md`
> with status "deferred".

## 1. Decisions taken for the edge cases

| Edge case | Decision | Where it landed |
|---|---|---|
| **E11** an early or late stream is invisible | **Option A now** (frame number from SYNC, receiver checks the phase), **option B later** (arrival time against the slot, telemetry only) | ADR-018 (done); section 2 below (B) |
| **E10** ground commands unauthenticated, replayable | **A + B + C**: counter with replay window, 32-bit SipHash tag, ARM/EXECUTE for the dangerous operations | ADR-019 (done) |
| **Interlock** disabling nodes | Triplex to Duplex plain; Duplex to Simplex refused unless armed; the last voter refused unless armed and logged loudly | ADR-019 (done) |
| **E3** lateness is a cliff | Keep the cliff (a node outside its window is failed, as in time-triggered systems); **do margin telemetry (A) and a non-latching warning (B) once the board exists** | section 2 below |
| **E6** Duplex cannot attribute small faults | **Accept Safe** for this project; **add B** (self-test at power-up) and **D** (shorter Duplex exposure); define the Safe action when the actuator node exists; **revisit A** (analytical redundancy) when the estimator exists | D done (ADR-010 amended); the rest in section 3 |
| **E19** the Duplex decision band moved | **Option B**: use the motion-aware reference only where the signal moves more than a tolerance per frame | ADR-017 amended (done) |

## 2. Timing: arrival-time monitoring (E3 and E11 option B)

**Trigger:** firmware for the real board exists and a CAN receive timestamp (or a cycle-counter read in the receive interrupt) is
available. Needs the rig: the thresholds come from measurements, not from a desktop.

**What:** for every received frame record the arrival time relative to the SYNC of its cycle, per node and stream. Two uses:
1. **Margin telemetry (E3 A):** keep, per node and stream, the minimum and the average margin to the 7 ms vote deadline over a
   window (and a count of frames inside, say, 20% of their slack). Log it; no effect on voting.
2. **Slot-window check (E11 B):** compare the arrival time with the slot (`1.5 + 0.2 n` ms for gyro, `2.3 + 0.2 n` for accel,
   `5.0 + 0.3 n` for the command) plus the legitimate bus delay, within a window. This closes the blind band that the frame-number check
   leaves: a frame up to about 4.5 ms early, or a clock drifting by a few hundred microseconds, still arrives in the right window
   with the right number.
3. **Warning flag (E3 B):** "timing degraded" for a node whose margin stays under the threshold for M of N frames; reported, never a latch.

**Order:** telemetry first and no enforcement; enforce the slot window only after the margins have been measured on the rig under
bus load (too tight a window causes false isolations; nine frames queue per cycle and each takes about 0.11 ms, so a frame can
legitimately be delayed by around a millisecond). The leaky alpha-count could take "almost late" as an input (E3 option C): only if the
measurements show that margin trends predict failures.

**Design notes:** the core does not know about time today (frame bucketing is outside it). Either compute the margin in the
application layer and pass it in the report, or give `on_frame` an optional receive timestamp. The replay log has virtual timestamps,
so the logic can be written and tested in simulation first (the peers' `early`, `jitter`, `late`, `clockdrift` faults exercise it);
the thresholds cannot.

**If measurements show the 1.4 ms slack of node C's command is thin** (E3 option D): move the vote later or the command slots earlier;
this trades against the compute window before the next frame.

**Requirement:** TFC-FDIR-037 (deferred).

## 3. Duplex attribution and the Safe action (E6)

**Accepted:** when two healthy nodes disagree and continuity cannot say who is wrong (a bias between 1x and about 2.3x tolerance, a slow
drift, a digest mismatch, a signal that barely moves) the manager holds the last good output and requests Safe (sticky). This is the
fail-safe outcome and it is what ADR-008 specified.

**Done (D):** the exposure to it is short. A node latched for a first, transient-looking cause waits 50 frames instead of 200 before it
may go on probation, so the system is back in Triplex sooner. The operator can also resolve a Safe request by isolating the node they
know is bad (`armed-disable` in Duplex, then `armed-clear-safe`): the interlock tiers (ADR-019) were chosen to allow exactly that.

**Deferred, with triggers:**
- **Define the Safe action** (**trigger: the actuator node (ACT) exists**). Today Safe means "the output holds the last good value".
  What the vehicle should *do* in Safe (hold, null the gimbal, a fixed attitude, abort) is not decided, and holding the last gimbal
  command is not obviously safe for long on a real rocket. This is a requirement to write with ACT: what ACT outputs when the
  flight computers request Safe, for how long, and who can leave Safe. **Proposed in `docs/SAFE_MODE.md` (ADR-023): freeze at once, null after a hold time at a limited rate, ACT able to do it alone; the owner's three open questions are at the end of that page.**
- **Pre-flight self-test (B)** (**trigger: the ISM330DHCX driver exists**). The part has a self-test that produces a known output
  change; it works at rest, so it protects the power-up check rather than the second fault in flight. A node whose sensor fails
  it should report "not ready" and not join the vote. Requirement TFC-FDIR-036 (deferred, verification by measurement). Belongs in
  stage S1 of `docs/STAGED_BUILD.md`.
- **Analytical redundancy (A)** (**trigger: the estimator exists, M3**). Check each node's gyro against the other sensors and the
  model (gyro-integrated attitude against the tilt from the accelerometers). It resolves faults larger than the model's own error:
  slow drifts, yes; a 1.5 dps bias just above a 1 dps tolerance, probably not. On the desk rig (a platform with known accelerations)
  it can be demonstrated honestly; in flight thrust corrupts the accelerometer's gravity reference. New failure mode to test for:
  blaming the wrong node.

## 4. Other work this change makes necessary

| Item | Why | Trigger |
|---|---|---|
| **Firmware for nodes B and C takes the frame number from SYNC** | ADR-018: the number is the label of the cycle, not a private counter | FC-B/C firmware |
| **SYNC loss and sync-master takeover keep the number continuous** | a node that loses SYNC must keep counting, and the takeover must not skip or repeat a number (F15) | sync-master takeover |
| **ACT applies the same phase check** and, if it accepts ground commands, the same authentication | ACT is the single point that votes the actuator output | ACT firmware |
| **Command-counter resynchronisation after a long outage or a flight-computer reboot** (section 5.3 shares the counter between the nodes) | the first command a flight computer accepts after a reboot sets its counter, so a recorded command could be replayed into that gap; and after more than 32 lost commands the ground counter is out of the window until the flight computer is restarted. A real uplink needs an authenticated counter-sync step | real uplink |
| **Key provisioning and storage for ground commands** | the default key is the public SipHash test key and protects nothing (ADR-019) | before any real uplink |
| **Authentication of the real uplink at the vehicle gateway** | the internal bus is a trusted segment; the link is not | real uplink |
| **A constant-time tag comparison** | `==` on the tag leaks timing; irrelevant on a bench | real uplink |
| **Command telemetry**: log accepted, refused, replayed, unauthentic and critical commands with their counters (section 5.4 defines the telemetry and the event records) | the operator and the post-flight analysis need them; the counters exist, the downlink does not | telemetry |
| **Membership agreement between the flight computers** | each decides alone; commands and frame loss can leave their views different (ADR-010 limit) | after FC-B/C |
| **Heartbeat (`0x400+n`) and actuator-output (`0x300`) payloads** | not defined in `protocol.hpp` yet | with ACT |
| **WCET and stack of `end_frame` on the target** | host figures only (0.5 us, 1,080 bytes of state) | M2, cycle counter |
| **Real CAN effects**: inconsistent omission and duplication, error rates for re-tuning the leaky count | no software model reproduces them | the rig |

## 5. Fault-management additions from the ASTE-331 avionics lecture

Five additions that follow from comparing this project with the course lecture "Avionics, C&DH and Flight Software" (2 Oct 2026; the
slides are not for redistribution, so only the ideas are recorded here). Each has requirement ids in `docs/REQUIREMENTS.md` and a row in
`docs/FAULT_MATRIX.md`. None changes the voter. Each needs an ADR when it is built. The first two wait for hardware, the third and the
fourth for node firmware and the heartbeat payload, the fifth can start in the core.

### 5.1 A watchdog that cannot be kept alive by the wrong task (FDIR-038, F57)
**Trigger:** the frame loop runs on the board (M2).
**Why:** on the Mars rover's Sol 200 anomaly, enough tasks kept running to keep servicing the watchdog while others were hung, so the
watchdog never fired. `ARCHITECTURE.md` already lists a per-node watchdog and a frame-deadline monitor; this fixes *what services it*.
**What:** every monitored task reports progress once per frame (a bit in a progress word, cleared at the end of the frame). The watchdog
is serviced in exactly one place, at the end of the frame, and only if the vote ran and every bit is set. The same end-of-frame event drives the `KICK` line to the supervisor (`docs/SUPERVISOR.md`), so the outside watchdog and the inside one see the same thing. Not from a timer, not from a
task. A task that stops costs the servicing within one frame; the hardware timeout then resets the node, which the others see as F01.
**Design notes:** the checkpoint logic is plain C++ and belongs in `core/` or next to it, so it can be tested on the host with a task
that never reports (T) before it is tested on the board with a blocked task (M). Pick the watchdog timeout so a healthy frame never
trips it and a hang is caught in about 3 frames (FDIR-001's bound). Decide whether a *window* watchdog (servicing too early is also a fault) is worth having, and check what the G474's watchdogs offer
in the datasheet.
**Test:** F57 needs a firmware build in which one task blocks while SYNC and the frame sender carry on.

### 5.2 An independent way to isolate a node, and what to do about a bus alarm (FDIR-039, FDIR-040, F58, F59)
**Trigger:** the supervisor exists (section 6.3, `docs/SUPERVISOR.md`).
**Why:** a software watchdog cannot help when the software is the problem, and the lecture's answer is a separate part with its own
hardware that can reset the computer and accepts hardware commands that bypass flight software. The first version of this item was a
bare line from another node; the supervisor replaces it, and gives the operator `hold X` to find a babbler by elimination (an
out-of-schedule id cannot be attributed to a node, ADR-009, so a bus alarm never holds a node in reset on its own).

### 5.3 Peers share their view of the state (FDIR-041, F60)
**Trigger:** nodes B and C have firmware, and the heartbeat (`0x400+n`) payload is defined (section 4, with ACT).
**Why:** the rover's recovery depended on critical telemetry being copied to the backup computer where it could be read without the
failed computer's software. Here the state worth sharing is small, and sharing it closes two gaps already on this page: a restarted
node forgets its strike counts (so a flaky node could start again from zero strikes), and the first command after a restart sets the
counter window (section 4, the command-counter row).
**What:** each heartbeat carries mode, the state and strike count of each node, and the last accepted command counter. By a rough
count that is about 4 of the 6 data bytes (to be checked against `protocol.hpp`). A restarted node rebuilds its state from what two
nodes agree on and, where they differ, takes the more conservative value (higher strikes, a Disabled node stays Disabled, higher
counter). It never ends up more trusted than the others consider it.
**Risks:** a faulty node can broadcast a wrong state, hence the two-agree rule; and this is the start of membership agreement
(the membership-agreement row in section 4), so keep it to restoring state after a restart and leave the full agreement problem alone. The
internal bus is a trusted segment (ADR-019): the heartbeat is not authenticated.
**Test:** the virtual peers must send state records and a restart (`reboot` already exists) must come back with them. Needs a
protocol change on both sides (`protocol.hpp` and `tfc_peers/protocol.py`).

### 5.4 Telemetry and events in a fixed format, and a command dictionary (IF-004, IF-005, IF-006, FDIR-043)
**Trigger:** the first telemetry output exists (console today; CAN or the serial port later). `noop` (FDIR-043) has no trigger; it can
be added to the core, the peers and the campaign now, as a command that changes nothing.
**Why:** today the report struct, the counters and the console prints carry the information, but there is no fixed list of channels, the
prints have no timestamp or sequence number, and the opcodes live in two places (`protocol.hpp` and `tfc_peers/protocol.py`).
**What:**
- **Dictionary (IF-004):** one file listing commands, telemetry channels and events, with argument limits, ARM needs and the modes that
  refuse a command. The C++ and the Python are checked against it by a test, so the two tables cannot drift.
- **Housekeeping (IF-005):** the existing counters as numbered channels at a fixed rate, with caution and warning limits.
- **Events (IF-006):** one record per event with frame number, kind, node, reason code and a per-kind sequence number (a gap shows a lost
  record). Every command that is accepted for processing gets exactly one success-or-refused record.
- **`noop` (FDIR-043):** a command that does nothing, needs no ARM, and proves the path.
**Design notes:** the fault campaign's per-frame CSV (`--dump`) is already close to the housekeeping channels; reuse its column names.
Start with the dictionary and the event records because the tests can use them straight away.

### 5.5 What a node does after a reset, and a reboot loop (FDIR-042, F61)
**Trigger:** node firmware exists (the reset counter and cause need a place that survives a reset: no-init RAM or a backup register).
**Why:** the lecture's reset pattern is: come up in a minimal known state, count the reset, and run the safe configuration if resets
repeat. Here the existing `reboot` fault covers one reboot and its rejoin, not a node that resets every few seconds.
**What:** after any reset a node does not vote and sends only what it needs to rejoin; it rejoins through the normal probation (FDIR-006),
not on its own say. It records the cause and a reset count. More than R resets in a window (proposal 3 in 60 s) keeps it out and needs a
maintenance command. The peers' strike counts (ADR-010) already disable a node that is latched three times, so this rule matters mostly
where the node itself can tell a loop is happening before the others have counted three latches.
**Open:** which reset causes count (a deliberate software reset should probably not); and the interplay with the 5 s startup grace
(ARCHITECTURE, boot order), which must not hide a loop.

### Not added from the lecture (decided, for now)
Rad-hard parts, 1553, memory types, data processing and encryption units, pyro boxes, the uplink-loss timer (a rocket does not need the
ground to survive), and a multi-image boot loader (dual-slot boot is listed in section 6.7 as a stretch). Now **added** after the second
reading: the time-seed plausibility check (TFC-SUP-009), parameters for the Safe configuration (TFC-SAFE-005), and the response ladder
(TFC-SAFE-007). Still left for a later decision: ground-changeable *monitor* thresholds and persistence as parameters (the lecture's rule:
the condition and the response are code, the threshold and persistence are parameters; today they are build-time configuration), settable
by an authenticated, ARM-protected command once the board exists, with the rule that no parameter may disable a monitor.

## 6. Architecture proposals that wait for a decision or a trigger (ADR-020 to ADR-023)

Written on 3 Oct 2026 from the owner's questions on cross-strapping, release diversity, the supervisor, fail-operational scope, Safe and
phases. Each is *proposed*: the ADR says what and why, the document it points to has the detail. **Decisions needed from the owner are in
bold.**

| # | Item | Trigger | Decision needed |
|---|---|---|---|
| 6.1 | **Sensor strapping** (ADR-020). **Case 1, accepted:** separate sensor health from compute health (TFC-ARCH-001). **Order decided 4 Oct:** after the loop (P1), before S3 (3 Nov), as its own PR; TS-15 decides the degradation rule first. **Case 2, deferred stretch:** ring re-homing of an orphaned IMU (TFC-ARCH-003). Direct simultaneous wiring rejected | Case 1: after P1; the estimator takes channels with a valid flag from the start. Case 2: after S4 and the supervisor | Case 2 only: wanted as a stretch? Decide at S4. The cheap preparations are decided (STAGED_BUILD rule 11) |
| 6.2 | **Release diversity** (ADR-021). Node C on the previous known-good release. **Decided 4 Oct: hold, request Safe, operator picks.** Automatic takeover by the old release stays open | FC-B and FC-C firmware; a tagged golden release; the `common_mode` campaign group can be written now | Automatic takeover (option c): after the TS-3 data (about November) |
| 6.3 | **Supervisor** (ADR-022, `docs/SUPERVISOR.md`). **Decided 4 Oct: SUP-Lite** (Pico 2 plus TCXO module); Full remains an upgrade | **The parts order closes 6 Oct: one more Pico 2, a clock module and resistors must be in it** | None |
| 6.4 | **Safe mode** (ADR-023, `docs/SAFE_MODE.md`). Freeze, then null; ACT does it alone. **Questions answered 4 Oct** (SAFE_MODE section 10; FAULT_RESPONSE section 6) | ACT firmware | None; the servo no-signal measurement (P-M1-01 step 10) confirms the ACT-reset rule |
| 6.5 | **Phases and roles** (ADR-023, `docs/MISSION_PHASES.md`). HOT, WARM, COLD; phase table; `phase` command. **Phase minimums accepted 4 Oct** | Estimator and the simulator's scenario events | None for the minimums; the rest is a proposal |
| 6.6 | **A second ACT** with a supervisor-driven output selector, to extend fail-operational to the actuator node (ADR-023) | After S4, if the budget allows: one Nucleo, one CAN Pal (about 24 USD) and a selector chip | **Stretch yes or no?** |
| 6.7 | **Other items from the lecture:** dual-slot boot with a golden image (the lecture's multiple FSW images chosen by a tiny unchangeable boot program; the project's golden release is the natural second image; ties to FDIR-042 and to ADR-021); a verification-procedure template (`docs/VERIFICATION_PROCEDURE_TEMPLATE.md`, written now); bus and memory sizing analyses (ARCHITECTURE section 7) | Board bring-up | None |

**What can be done now without hardware:** the `common_mode` campaign scenario (F63) against the current voter, which shows the
weakness the golden-release rule is for (measured: the healthy node is latched at frame 102); the `noop` command (FDIR-043); the
supervisor's decision table as host-tested logic; a three-process `native_sim` triplex on one `vcan0` to test mixed releases.

## 7. Later step: AI-assisted tooling, outside the flight path

Added 4 Oct 2026 from the owner's question. **Not scheduled before M4;** it is the kind of extra that is worth doing only if the core
is finished and measured.

**The rule.** No learned component is in the flight path, and none holds the ground-command key. Reasons that are specific to this
project: the replicas must produce bit-identical state and digests (ADR-006), the fault management is argued by a deterministic design
with oracles that are mutation-tested (ADR-016), and a learned model cannot be tested the same way; the project's coding standard is
built for code that can be read and bounded; and an inference time that varies is a timing fault in a time-triggered schedule. A learned
part would multiply the verification burden to gain little on a fault set that the deterministic design already covers. Saying where AI
was *not* used, and why, is the point an avionics interviewer will probe.

**Where it is useful, in order of value for the effort:**

| # | Use | Where it runs | Risk to flight | What it gives |
|---|---|---|---|---|
| 1 | **Falsification search**: an optimiser (evolutionary, Bayesian) looks for fault parameters, start times and two-fault combinations that violate an oracle, beyond the campaign's grids | PC, offline | none | Finds the case nobody listed; each find becomes a regression scenario; a good partner to mutation testing |
| 2 | **Log triage**: an assistant reads decoded logs, events and housekeeping (IF-005, IF-006), summarises what happened and why, and points at the first anomalous frame | PC, offline | none | Faster analysis of campaign failures and rig runs |
| 3 | **Shadow anomaly detector**: a learned model on the telemetry runs *next to* the deterministic fault management and never acts; its alarms are compared with it for detection time and false alarms | PC, advisory | none | A data-based comparison (TS-14); an honest answer to "why not machine learning?" |
| 4 | **Duplex attribution advisor**: for the case nobody can be blamed (E6, the Safe request), a model ranks which node is more likely wrong and shows it to the operator, who decides under an ARM | PC, advisory | none, the operator decides | Targets the weakest point of Duplex; compare with analytical redundancy (TS-11) |
| 5 | **Offline tuning**: an optimiser or learner picks the estimator and controller gains, the persistence settings (TS-1) and the Safe parameters (TS-4) in simulation; the result is a fixed table | PC, offline; fixed numbers in flight | none if the table is verified like any other | Better settings, argued by data |
| 6 | **Operator assistant**: explains events, drafts the command, **cannot send it**; the key stays with the human | PC | none by construction | Lower operator load in the Safe and recovery cases |
| 7 | **Requirement and trace checking**, review of documents against the matrix | PC, offline | none | Catches stale references |

**What stays out until a case is made:** a learned residual inside the estimator or the voter, a learned arbiter that decides who is
right, anything whose output reaches ACT.

**Boundary tests (to write with the first use).** The campaign gives byte-identical dumps with the AI components disabled and enabled;
the flight binary contains no inference code (`tools/check_elf.sh` extended); the AI client has no access to the key.

**Cost and order.** Items 1 and 3 are the two worth doing: they need only the PC, the existing campaign, and the telemetry of section 5.4.
Item 5 comes free with TS-1 and TS-4. Do not start before the software-ready gate (`SOFTWARE_READINESS.md`) and the first hardware
measurements. **Requirements:** TFC-AI-001 to 003 (proposed boundary). **Study:** TS-14.
