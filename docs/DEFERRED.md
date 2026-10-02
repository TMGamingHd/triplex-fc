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
  flight computers request Safe, for how long, and who can leave Safe.
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
| **Command-counter resynchronisation after a long outage or a flight-computer reboot** | the first command a flight computer accepts after a reboot sets its counter, so a recorded command could be replayed into that gap; and after more than 32 lost commands the ground counter is out of the window until the flight computer is restarted. A real uplink needs an authenticated counter-sync step | real uplink |
| **Key provisioning and storage for ground commands** | the default key is the public SipHash test key and protects nothing (ADR-019) | before any real uplink |
| **Authentication of the real uplink at the vehicle gateway** | the internal bus is a trusted segment; the link is not | real uplink |
| **A constant-time tag comparison** | `==` on the tag leaks timing; irrelevant on a bench | real uplink |
| **Command telemetry**: log accepted, refused, replayed, unauthentic and critical commands with their counters | the operator and the post-flight analysis need them; the counters exist, the downlink does not | telemetry |
| **Membership agreement between the flight computers** | each decides alone; commands and frame loss can leave their views different (ADR-010 limit) | after FC-B/C |
| **Heartbeat (`0x400+n`) and actuator-output (`0x300`) payloads** | not defined in `protocol.hpp` yet | with ACT |
| **WCET and stack of `end_frame` on the target** | host figures only (0.5 us, 1,080 bytes of state) | M2, cycle counter |
| **Real CAN effects**: inconsistent omission and duplication, error rates for re-tuning the leaky count | no software model reproduces them | the rig |
