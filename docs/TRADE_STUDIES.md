# Trade studies: plan

> Status: **plan**. None of the studies below is done except where a section says what already exists. Numbers are proposals. The point of
> this page is to choose studies that (a) the repository can answer with its own data, (b) end in a decision someone could disagree with,
> and (c) are things an avionics or flight-software interviewer will want to argue about.

## 1. What a good study looks like here

The lecture's design process names the trades an avionics engineer is expected to run: redundancy, processor type, serial against bus
interface, interface throughput sizing, memory sizing, and ground-commanded against autonomous functions. Each study here follows one shape:

| Part | Content |
|---|---|
| Question | One sentence that ends in a decision |
| Options | Three to five, including "do nothing" |
| Criteria and weights | Written **before** the data, with the reason for each weight |
| Method | What is measured, what is modelled, what is assumed, and which is which |
| Evidence we already have | Repository data and its limits |
| Output | One figure or table that carries the argument |
| Decision rule | The condition that picks the winner, written in advance |
| Sensitivity | What would change the answer |
| Talking point | The sentence that explains it to an interviewer |

Rules: label every number as **measured**, **modelled** or **assumed**; never present a modelled number as a measurement; keep the raw data
and the script that makes the figure in the repository so the figure can be regenerated; report the result that went against the
hypothesis, if there is one.

## 2. The catalogue

Appeal: how likely an avionics interviewer is to want to discuss it (3 = very). Needs: SIL means it can run now on the simulator and the
campaign; HIL needs the rig.

| ID | Study | Question | Needs | Appeal | Effort |
|---|---|---|---|---|---|
| TS-0 | Supervisor build: Lite or Full | Which supervisor do we order by 6 Oct? | analysis | 2 | small, **urgent** |
| TS-1 | Persistence and detection tuning | What M-of-N and leaky-count settings give the best detection time for an acceptable false-isolation rate? | SIL, then HIL timing data | 3 | medium |
| TS-2 | Redundancy architecture, availability against coverage | Which architecture (simplex with cold spare, duplex, triplex, triplex plus monitor, self-checking pairs) loses the output least, once coverage and common cause are counted? | model + campaign | 3 | medium |
| TS-3 | Release diversity: how to arbitrate between releases | Vote only, hold and ask, or automatic takeover? | SIL (new fault group) | 3 | medium |
| TS-4 | Safe action in the closed loop | Hold, null, null after a hold, depower: which keeps the vehicle recoverable? | SIL toy model now, 6-DOF sim later | 3 | medium |
| TS-5 | Bus transport | Classic CAN, CAN FD, a second bus, or Ethernet for the exchange? | analysis + HIL | 3 | medium |
| TS-6 | Reintegration policy | Manual, automatic for transients, or automatic for all; dwell and probation lengths? | SIL | 2 | small |
| TS-7 | Sensor strapping | None, health split, ring re-homing, smart IMU nodes? | model | 2 | small |
| TS-8 | Time and schedule | Where to place the vote deadline and the slots; how much slack? | HIL | 3 | medium |
| TS-9 | Watchdog and supervision coverage | Which hang and reset faults does each watchdog arrangement catch? | HIL | 3 | medium |
| TS-10 | Digest and tag design | How many bits for the state digest and the command tag? | analysis | 1 | small |
| TS-11 | Duplex attribution | How should a Duplex disagreement be attributed (continuity, analytical redundancy, none)? | SIL, estimator | 2 | large |
| TS-12 | Autonomy against ground command | Which responses are autonomous and which need the operator? | analysis | 2 | small |
| TS-13 | Toolchain and RTOS | C++17 subset against C or Rust; Zephyr against bare metal | analysis (partly decided in ADR-001, ADR-002) | 2 | small |
| TS-15 | Sensor and compute health split: the degradation rule | Once a sensor channel is excluded, how should the computer degrade, and what does the split buy? | SIL (campaign groups, after the split) | 3 | medium, **before S3** |
| TS-16 | Keeping replicated estimators together on a lossy bus | When two nodes receive different sets of sensor frames in one frame their estimators diverge for good: agree on inputs, resynchronise state, or tolerate the digest mismatch? | SIL, live triplex | 3 | medium, **before the closed loop is relied on** |
| TS-17 | Hardware overrides: how many switches, which, and what can go wrong | When every program is down or wrong, which manual hardware overrides are worth their cost and their own failure modes? | paper FMEA, then HIL on the rig | 3 | medium, **before the override parts are ordered** |
| TS-18 | Testing hardware-facing firmware without the hardware | Fake board under the real loop, the host emulation of the board's drivers, an instruction-set emulator, or the board only: which finds how many bugs per hour of work, and where does each go blind? | seeded bugs and mutation scores | 3 | small, **data already exists** |
| TS-19 | The injector's cut semantics | A cut that ends by itself (today), a latched cut with an explicit restore, or a cut that needs a heartbeat: which is safest when the PC dies and still allows the long outages the supervisor's tests need? | HIL scenarios | 2 | small, before the supervisor tests |
| TS-20 | The platform's behaviour when commands stop | Hold then level (today), hold for ever, level at once, stop the pulse and go limp, hold then stop: which has the smallest mechanical shock and the least confusing view for the IMUs and the flight computers? | rig measurements | 2 | medium, after E1 to E6 of `PICO_TESTS.md` |
| TS-21 | Where the mission clock lives | The supervisor as the continuous source, the supervisor deciding T-zero with the flight computers as the clock (hybrid), mission time in SYNC from the sync master alone, or each computer counting for itself: which survives a failure of the supervisor, of the sync master and of a node's link, and keeps every computer on the same schedule? | SIL with fault injection, live triplex | 3 | medium, before the mission time goes into SYNC |
| TS-22 | The independent time reference | A TCXO RTC, an oven crystal, a GPS-disciplined oscillator, a chip-scale atomic clock, with or without correlation against the PC's UTC: what error does each leave over a run, what does it cost, and what happens when it fails? | bench measurement (Allan variance, drift against NTP and GPS) | 3 | small to medium, after the parts arrive |
| TS-23 | How often the replicas resynchronise (the period) | Every 10, 20, 50, 100, 200, 500 or 1000 frames: what does the period change in flight survival, the digest check, healing and rejoin time, bus load and the detection of a failing state? | SIL closed loop (`tfc_resync`) | 3 | medium, **before the firmware default is fixed** |
| TS-14 | Learned against deterministic anomaly detection | Does a learned detector, run in shadow mode on the telemetry, beat the 3-of-5 plus leaky-count design on detection time or false alarms, and what does it cost to verify? | SIL, later HIL (after the telemetry exists) | 3 | medium; **later step**, `DEFERRED.md` section 7 |

Recommended order, by value and by when the data exist: **TS-0** now; **TS-1, TS-2, TS-3, TS-4** on the simulator in October and
November, while the hardware is built; **TS-5, TS-8, TS-9** on the rig after S3; the others as short write-ups.

## 3. TS-0: supervisor, Lite or Full (**decided 4 Oct 2026: Lite**)

> **Outcome.** The owner chose Lite, as recommended. The study stays as the record of why, and of what would change the choice: wanting per-stream arrival-time telemetry from outside the computers.

**Question.** One more Pico 2 and a clock module (Lite, discrete lines only) or a Nucleo, a CAN Pal and a clock module (Full, adds a
listen-only bus tap)? `SUPERVISOR.md` section 3 has the build. The parts order closes on 6 Oct.

**Criteria and weights (my proposal; change them and the answer may change).**

| Criterion | Weight | Why this weight | Lite | Full |
|---|---|---|---|---|
| Independence from the bus | 3 | the point of the supervisor | 5 | 4 |
| Capability (arrival margins per stream, frame-number plausibility) | 2 | closes the deferred FDIR-037 outside the computers | 3 | 5 |
| Cost | 3 | the budget is already over its ceiling | 5 (6.00 USD) | 4 (24.08 USD) |
| Build effort | 3 | the time between 20 Oct and 17 Nov is the schedule risk | 4 | 3 |
| Toolchain match with the rest (Zephyr, FDCAN timestamps) | 1 | one toolchain is less to learn | 3 | 5 |
| Timing precision on the bus | 2 | hardware receive timestamps against GPIO capture | 3 | 5 |
| Risk of adding a node to the bus | 2 | one more tap, one more thing to babble | 5 | 3 |

Weighted totals with these scores: **Lite 67, Full 64**. If capability is weighted 4 instead of 2, Lite 73 and Full 74: a tie. The
scores are my judgement, not data. The study shows that the choice turns on one question: **is per-stream arrival-time telemetry from outside
the computers worth 18 USD and a node on the bus?**

**Decision rule.** Take Lite unless the owner wants the arrival-time telemetry from outside the flight computers, in which case take
Full. **Recommendation: Lite**, because the same telemetry can be produced inside the computers (FDIR-037 as written) and every other
supervisor function is the same. The cost of choosing Lite and later wanting Full is a second board and a firmware port, not a
redesign: the lines are identical.

**Talking point.** "I made the supervisor independent of the bus on purpose, and I can say what that cost me and what it would take to
change my mind."

## 4. TS-1: persistence and detection tuning

**Question.** For the 3-of-5 window, the leaky alpha-count (decay 0.9, latch at 3), the vote tolerance and the stuck limit: what setting
gives the shortest detection time for the faults of FDIR-001 to 003 and 023, at an acceptable rate of isolating a healthy node?

**Options.** M-of-N in {2-of-3, 3-of-5, 4-of-7, 5-of-9}; alpha decay in {0.8, 0.9, 0.95} and threshold in {2, 3, 4}; tolerance multiples
of 1x, 1.5x, 2x the noise sigma; each with and without the alpha-count.

**Criteria.** Worst-case detection latency per fault class (frames); false-isolation rate under the real timing jitter (events per hour of
flight); fraction of intermittent faults caught; operator burden (false alarms per hour).

**Method.** (1) *Model*: each filter is a small Markov chain, so the probability of a false latch per frame for a bad-frame probability
p can be computed exactly, with no need for hundreds of hours of simulation to see a rare event. (2) *Check the model* against the real
manager with the campaign runner: the existing grids already sweep fault magnitude, start frame and seed. (3) *Replace the assumed p*
with a measured one: `missing` was seen at 4 to 8 in 400 frames on the busy CI runner, and the rig will give the real rate. The
sweep is a new campaign group; the parameters are in `core/` configuration.

**Evidence already here.** The detection-time table in `sim/README.md` and `FAULT_CAMPAIGN.md`; the false-isolation oracle S1 on 10,713
scenarios; ADR-013 on why the alpha-count exists.

**Output.** A detection-latency against false-isolation plot with one point per setting and the chosen setting circled (a Pareto front).

**Decision rule.** Among settings that keep the false-isolation rate under one per 100 hours of flight time (proposal), choose the one
with the lowest worst-case latency for the FDIR-001 to 003 faults; break ties by fewer parameters.

**Sensitivity.** The measured jitter on the rig; a different fault mix; whether a missing frame under bus load is more common than modelled.

**Talking point.** "I computed the false-isolation probability exactly and checked it against the real code, so the setting is argued,
not guessed."

## 5. TS-2: redundancy architecture against availability, with coverage and common cause

**Question.** Which architecture loses the output least over a mission, when detection is imperfect and some faults hit every replica at
once?

**Options.** (a) one computer with a cold spare; (b) Duplex, compare only; (c) Triplex, the project; (d) Triplex plus a dissimilar
monitor (the Shuttle's arrangement; the project's golden release); (e) two self-checking pairs (as published descriptions of Orion's
computers; verify before citing).

**Criteria.** Probability that the output is lost; probability that a wrong output is used; number of computers (mass, power and cost
proxy); software and procedural complexity; operator workload.

**Method.** *Model*: a continuous-time Markov chain for each architecture, with a failure rate per computer (assumed, relative values
only), a **coverage factor** c (the fraction of faults detected and isolated correctly), a latent-fault interval, and a common-cause
fraction beta. *Measure*: c comes from the campaign (the fraction of scenarios where the fault was isolated or masked with every
oracle passing), per fault class. *Sweep*: c from 0.9 to 0.9999 and beta from 0.01 to 0.1.

**Expected finding (to be shown or refuted).** Coverage and common cause dominate the result; the failure rate per computer barely
matters. If so, effort is better spent on detection coverage and diversity than on a fourth computer, which is what the project does.

**Evidence already here.** The campaign's per-class results (`FAULT_CAMPAIGN.md`); ADR-008, ADR-017 and ADR-021 on the limits of voting.

**Output.** Output-loss probability against coverage, one curve per architecture; a second plot against beta.

**Decision rule.** The architecture with the lowest output-loss probability at the measured coverage and a stated beta, unless it needs
more than the budget allows. The decision is already Triplex; the study says how much of the benefit is Triplex and how much is coverage.

**Sensitivity.** The failure-rate ratios between hardware and software; the common-cause fraction (a judgement, bracket it).

**Talking point.** "Redundancy only helps as much as the detection behind it, and I measured the detection."

## 6. TS-3: release diversity, how to arbitrate

**Question.** When the two computers on the current release agree and the one on the golden release disagrees, what should the system
do?

**Options.** (a) Vote only; (b) hold and request Safe, the operator chooses (ADR-021); (c) automatic takeover by the golden release when
the pair leaves a plausibility envelope; (d) same release on all three, for reference.

**Fault scenarios.** A regression common to the pair; a bug only in the golden release (it lacks a fix); an independent hardware fault
on one node; a timing fault; a behavioural change that is legitimate (the compatibility gate's case).

**Criteria.** The wrong output followed (frames and magnitude); an unnecessary Safe request (availability cost); time to a correct
output; operator actions needed.

**Method.** *Measured on the real voter*: a new campaign group `common_mode` (F63) with the peers set to the same wrong output on two
nodes; a new peer behaviour for "the other release". The measured baseline already exists: with the same `cmd_offset` on B and C the
healthy node A is latched at frame 102 and the vehicle follows the wrong pair. Options (b) and (c) need the manager change.

**Output.** A table of outcomes, rule against scenario, with the failure it prevents and the one it causes.

**Decision rule.** Prefer the rule that never follows a wrong output for more than the persistence window and never isolates the healthy
node, accepting the availability cost; (c) wins only if its envelope can be shown not to cause a new failure.

**Talking point.** "A majority vote cannot see a bug the majority shares. I measured that on my own system, then changed the rule."

## 7. TS-4: Safe action in the closed loop

**Question.** After an unrecoverable disagreement, what should the output do, so that the vehicle stays recoverable?

**Options.** Hold forever (today); freeze then null after `T_hold`; null at once; depower; freeze then null with different rate limits.

**Criteria.** Peak attitude error after the fault; time before the attitude error leaves a recoverable envelope; peak gimbal rate; the
operator's time to recover.

**Method.** *SIL with a toy model first*: a linearised unstable pitch-plane model of the vehicle (few states) driven by the controller,
with a fault at a chosen time, sweeping `T_hold`, the rate limit and the fault time across the ascent. *Later* the 6-DOF simulator and the
platform (M3). The toy model is **modelled**, never to be shown as the platform's behaviour.

**Output.** A heat map of "recoverable or not" against `T_hold` and fault time, per strategy.

**Decision rule.** The strategy with the largest recoverable region that never commands a step; the owner's open questions
(`SAFE_MODE.md` section 10) set the weights.

**Talking point.** "I did not pick the safe state, I showed where each one stops being safe."

## 8. TS-5: bus transport

**Question.** Classic CAN at 1 Mbit/s (ADR-004), CAN FD, a second CAN bus, or Ethernet?

**Options.** As stated; the stretch goals in the project overview mention FD and Ethernet; the parts sheet notes the Nucleo and the CAN
transceiver already support FD.

**Criteria.** Worst-case message latency (against the 7 ms vote); bus load with the added traffic (IF-005, IF-006, state records); the
babbling-node and common-cause exposure; hardware on hand; code change.

**Method.** *Analysis*: worst-case response-time analysis of the frame schedule (standard CAN schedulability analysis; the 9 frames at
about 0.11 ms each are the known figure); *measurement* on the rig after S3 with the logic analyser; for FD, whether the USB-CAN adapter
supports it is unconfirmed (ADR-004).

**Output.** Latency and load against option; a table of what each option does against a babbling node (F11) and against a lost bus (F13).

**Decision rule.** Keep classic CAN unless the analysis or the measurement shows the vote deadline or the 40% load limit (SYS-003)
cannot be met; a second bus is chosen only to remove the bus as a single point of failure, at a stated cost.

**Talking point.** "I measured whether the faster bus was needed before I used it."

## 9. TS-6 to TS-13: short plans

| ID | Options and criteria | Method and output |
|---|---|---|
| TS-6 Reintegration | Manual, auto for transients, auto for all; dwell 50 or 200, probation 100 or 300 frames. Availability, risk of readmitting a flaky node, operator load | Campaign recovery groups plus a Markov availability model; a table of mean time in a reduced mode |
| TS-7 Sensor strapping | None, health split, ring, smart IMU nodes. Probability of a double fault that costs a sensor channel; cost; complexity | A small reliability model; a cost and probability table (ADR-020) |
| TS-8 Time and schedule | Vote deadline 6, 7 or 8 ms; slot spacing; slack against jitter. Missing-frame rate, compute window, slack | Measurements on the rig: p99 jitter, WCET (SYS-001, SYS-002); a histogram and a chosen deadline |
| TS-9 Watchdog coverage | Internal only; internal plus frame-deadline; plus the supervisor. Hang types: full hang, partial hang, reset loop, a slow task | A fault-injection table on the rig, hang type against arrangement, with detection time |
| TS-10 Digest and tag | Digest 16 or 32 bits (probability a divergence is missed, 2^-16 per frame); tag 32 or 64 bits (forgery 2^-32). Bytes of bus load against probability | Analysis; a table; how many frames pass before an undetected divergence is expected |
| TS-11 Duplex attribution | Continuity (today), analytical redundancy (gyro against accelerometer tilt), none. False-blame rate, Safe rate | Needs the estimator; the existing `duplex_boundary` sweep is the baseline |
| TS-12 Autonomy | For each response (isolate, reintegrate, Safe, clear), autonomous or operator. Time available, risk of a wrong action, operator availability in each phase | A table with the reasoning; ADR-010 and ADR-019 already take positions |
| TS-13 Toolchain and RTOS | Language subset, RTOS, build and static-analysis tools. Determinism, certification evidence, ecosystem | A short write-up; decided in ADR-001 and ADR-002, add the rejected options with reasons |
| TS-14 Learned against deterministic detection | A learned detector (for example residual-based) against the persistence filters; detection time, false-alarm rate, training data needed, verification effort, determinism | The campaign scenarios as labelled data, held-out fault kinds as the test; shadow mode only; a table and one plot; the result may well be that the deterministic design wins, and that is a finding |

## 9a. TS-15: how a computer degrades when its sensing does (decides ARCH-001's rule)

**Question.** The owner chose to separate sensor health from compute health (ADR-020, case 1). What exactly should the manager do when one sensor channel is excluded and the computer is healthy, and when a second channel goes? The study chooses the **degradation rule** before it is written into the manager, and measures what the split buys.

**Options.**
| | Rule | In words |
|---|---|---|
| A | Today | A computer is one unit: a sensor fault latches the whole computer |
| B | Split | The sensor channel is excluded; the computer stays a command voter at its usual tolerance |
| C | Split, stricter | As B, but while only two sensor channels remain, that computer's command is judged against a tighter tolerance (it now computes from two sensors, and a median of two cannot arbitrate) |
| D | Split, demote | As B, but when sensing falls below two channels the computer is demoted to WARM (shadow-voted, not voting) until a channel returns |
Variants for what the consensus is with two channels left: the mean, or the continuity rule of ADR-017 (follow the channel that agrees with the motion).

**Criteria and weights (proposed; fix them before the data).** Command-voter availability, in computer-frames in the vote under a fault mix (weight 3); false isolation of a healthy computer or channel (weight 3: S1 must hold); Safe requests caused by two sensor channels that cannot be told apart (weight 2); wrong-output exposure, measured as frames with the output beyond tolerance (weight 3); complexity, in lines changed in the manager, new states, mutants needed and the effect on the coverage gate (weight 1).

**Method.** Build the split behind a configuration switch (A is today's behaviour, so the baseline is exact) and run the campaign on three new groups: sensor-only faults on one and on two channels, computer-only faults, and IMU-plus-computer double faults (F73), each at several start frames, in Triplex and in Duplex, plus ten-minute runs. Metrics come from the per-frame dump (`tfc_replay --dump`). A small Markov model with the measured coverage turns the per-fault results into availability over a mission, so the gain is stated in the same units as TS-2.

**Output.** One table: option against fault class, with the availability of command voters, false isolations (expected zero), Safe requests and output exposure; one plot of availability against the share of sensor faults in the fault mix.

**Decision rule.** The option with the highest command-voter availability that adds **no** false isolation and violates no oracle; on a tie, the simpler. The expected finding, to be shown or refuted: B gives nearly all of the gain for a single sensor fault, and C or D matter only for two sensor faults at once.

**Dependencies and timing.** Needs the estimator's channel interface (P1) and the campaign groups; run **after the loop and before S3 (3 Nov)**, in the same window as the split itself. The ring re-homing of ADR-020 is TS-7's question, not this one.

**Talking point.** "I measured what splitting sensor health from computer health buys, and which rule for the degraded case was worth its complexity."

## 9b. TS-16: keeping replicated estimators together on a lossy bus

**Question.** Found by running three firmware instances with the real estimator (ADR-025): the estimator is stateful, so two nodes that receive a different set of sensor frames in one frame (a frame lost or late at one of them) compute different inputs, and their states drift apart for good, because the attitude is integrated from the gyro and only the accelerometer pulls it back, slowly. The state digest then disagrees, the two voting nodes cannot blame each other, and the system goes to Safe on one lost frame. How should the replicas stay together?

**Options.**
| | Rule | In words |
|---|---|---|
| A | Today | Nothing: a digest mismatch is a fault (frames are skipped alike when SYNC is lost, which removes only that cause) |
| B | Agree on inputs | Each node broadcasts which sensor frames it used (a mask, or the consensus value itself) and all use the agreed set or value; costs a round of messages inside the frame |
| C | Resynchronise state | Each node shares its quantised estimator state (the state-share frames) and every few frames adopts the mid-value state; a divergence heals in a frame or two, and the digest compares what is left |
| D | Tolerate | Compare digests with persistence (a mismatch that persists for N frames), and let the filter's own gains pull the states together; no new traffic |
| E | Contractive estimator | Choose the filter gains so that a difference in state decays in well under the persistence window, so D works |

**Criteria and weights (proposed).** False isolation or Safe from one lost frame (weight 3: the single-fault tolerance of S1); time to reconverge after a lost frame; bus load and frame time (weight 2); detection of a *real* estimator fault (weight 3: the digest must still catch a corrupt node); complexity and verification effort (weight 2).

**Method.** On the host: the live triplex with a bus fault injector (drop one sensor frame at one node in frame N, for several N and for each frame type) and the closed-loop flights with the real vehicle; for each option count false isolations, Safe requests, the time for the states to agree again and the detection time of an injected state corruption. The campaign's replay tool cannot show this by itself, because it feeds every node the same frames.

**Decision rule.** The cheapest option with no false isolation or Safe from a single lost or late frame in any frame type and no loss of detection of a real estimator fault. Expected finding, to be shown or refuted: D with E is enough for gyro-bias and attitude differences, which decay, but not for a difference in the *controller's integrator*, which is held by anti-windup and does not decay on its own; that one needs C.

**First measurement (4 Oct 2026, host closed loop, option A as built).** `sim::Loop::frame_loss_prob` drops each sensor frame of another computer independently at each receiver (a computer always gets its own), over a 60 s ascent with the real flight function (accelerometer correction off, as flown under thrust), three replicas and ACT (`tests/test_realism.cpp`, `ts16_*`; the sweep itself was a scratch program, not kept):
| Loss per frame | Frames lost (of 72 000 receptions) | Frames with the three digests not all equal | Largest command difference between two computers | Flight |
|---|---|---|---|---|
| 0 | 0 | 0 | 0 | flies |
| 0.01 % | 4 | 82 % | 0.06 deg | flies (0.61 deg max error) |
| 0.1 % | 64 | 99 % | 0.07 deg | flies (0.61 deg max error) |
| 1 % | 725 | 99 % | 0.17 deg | lost: ACT in Safe for 2 219 frames |
| 5 % | 3 589 | 99.8 % | 0.15 deg | lost: ACT in Safe for 2 730 frames |

What it shows: (1) **a handful of lost frames in a flight makes the digests differ for the rest of it** (4 losses, 82 % of the frames, one run of 4 873 frames), so the permanent divergence of the question is confirmed and is not a rare corner; (2) the *commands* stay within 0.2 degree of each other, so the damage is the digest mismatch and what the manager and ACT do with it, not a control error: the digest is a fingerprint of quantised state, so any difference, however small, persists; (3) at 1 % loss the flight is lost. This loop has no fault manager and no state-share frames (it was built to test the control chain and the vehicle, and the manager lives in the node firmware: a gap in the loop, not a design choice), so the Safe at 1 % is ACT's own response, and it is now explained: `SafeCause::LostVotes`. ACT accepts three commands as agreeing if they are within `tol_deg` = 0.05 degree; at 1 % loss the replicas' commands differ by more than that on 1 408 frames (28 at 0.3 %, 8 at 0.1 %, 1 at 0.01 %), and three such frames in a row (`lost_votes` = 3) enter Safe, at the first time with a spread of 0.075 degree. The step from 28 to 1 408 over-tolerance frames between 0.3 % and 1 % shows the difference **growing** with each further loss and not just persisting, which is the controller-integrator case the expected finding predicted. The real system adds the manager's reaction to the digest mismatch on top of this.

*Caveat on the loss model.* The drops here are independent at each receiver. Classic CAN is built so that a frame damaged at one node is flagged and re-sent for all (an atomic broadcast, with a known exception for errors in the last bits of a frame), so a lost frame at one node and not the others is **not** the likely cause on a healthy wired bus. The likely causes are local: a receive FIFO that overflows, a frame that arrives after the node has already run its step (late, in a time-triggered frame), a node that was reset and rejoined, or a software drop. The model stands for those; the rate to use for them is a measurement still to be made on the rig (the live triplex with a counter of frames each node used in each frame). Consequence for the options: **D alone (tolerate with persistence) cannot work**, because the mismatch does not decay inside any persistence window; the digest must either compare something that converges (E) or the states must be pulled together (C) or the inputs agreed (B). The bus loss rate this needs to survive is also now a number to ask for: the CAN fault campaign should say what loss a real bus shows, since a good wired bus loses far fewer than 0.01 %.

**Decision (5 Oct 2026, owner: option C with persistence-based detection; ADR-030).** Built on the host and measured (`docs/RESYNC.md`): with the resync every 100 frames the 60 s flight that was lost at 1 % frame loss flies (max error 0.55 degree), and still flies at 10 %; a 2 degree state corruption in one computer is healed and reported as one large correction; the digest check counts a mismatch only after a configured persistence. Not yet in the firmware, so not yet on the bus. The expected finding (D with E is not enough for the controller's integrator) was not tested separately: option D alone is ruled out by the first measurement.

**Dependencies and timing.** Needs the fault injector on the bus (P1-5, the Pico, or a host injector) and the runner's closed loop (P1-4d). Before the closed loop is used for the demonstration.

**Talking point.** "I found that replicated estimators diverge permanently on a single lost frame, measured it, and compared four ways of keeping them together."

## 9c. TS-17: hardware overrides, how many and which (docs/HARDWARE_OVERRIDE.md)

**Question.** The software has three layers (flight software, the supervisor, and nothing); the owner asked for a layer of hardware switches that works when every
program is down or wrong. How many overrides does the rig need, which functions should they have, and what can go wrong with each? More switches cover more
failures, and every switch is itself a part that can fail to act (a latent loss of protection) or act when it should not (a lost run, a mechanical shock).

**Scenarios (the software-down cases, SD).**
| | Scenario | Why it is hard for software |
|---|---|---|
| SD1 | The PC or simulator hangs | Nothing commands the platform; the Pico's own timeout (PLAT-002) is software |
| SD2 | The Pico platform driver hangs or runs away | Its PWM is stuck or wrong; the Pico's watchdog is software too |
| SD3 | ACT hangs or votes wrong | The simulator holds the last command; the vehicle flies on and the platform follows it to its stops |
| SD4 | All flight computers are wrong the same way (a common software bug) | ACT's vote agrees with them; no software layer sees a fault |
| SD5 | The supervisor hangs or resets a healthy node again and again | The layer meant to help is the fault |
| SD6 | The injector's relay stays on (a crashed Pico, a failed driver) | A node is cut and looks faulty |
| SD7 | A node babbles on the bus or hangs it | The bus is the only way the others can see it |
| SD8 | A servo stalls or runs to its stop (mechanical) | Current rises; software may be the cause or the victim |
| SD9 | The node rail droops (relay coils, a failing adapter) | Every node resets together |

**Options (cumulative; H-numbers as in `HARDWARE_OVERRIDE.md` section 3).**
| | Set | Count | What it adds |
|---|---|---|---|
| O0 | None; the supervisor alone | 0 | The baseline: layers 2 and 3 only |
| O1 | The parts list: E-stop (H1) and the free manual controls (Nucleo reset buttons, unpluggable CAN stubs, adapter plugs) | 1 | Platform stop; manual node reset and bus isolation |
| O2 | O1 + FORCE-SAFE (H2) + PLATFORM-LEVEL (H3) | 3 | A command to ACT, and a platform that goes level with the servos still holding |
| O3 | O2 + INJECTOR-DISARM (H4) + SUPERVISOR-DISARM (H5) | 5 | The two layers that can cut power get a switch that makes them unable to |
| O4 | O3 + four per-node POWER-KILL (H6) + MASTER-POWER (H7) | 11 | A manual node loss that needs neither the Pico nor the supervisor |

**Criteria and weights (proposed; fix them before the data).**
- **Coverage:** for each scenario SD1 to SD9, is there an override that needs no program, and how long does it take to reach the safe state (weight 3).
- **Harm done by the override itself:** a mechanical shock (G1, G2), a lost run, a node cut for no reason, rated per override from the failure table (G1 to G12) (weight 3).
- **Latent failure:** the chance that the override is dead when needed, and what it takes to test it (weight 3; testability matters more than cost here).
- **Independence:** does the override share a supply, a ground or a part with what it overrides (weight 2; G10).
- **Cost and panel space** against a budget that is over its ceiling (weight 1).
- **Demonstration value:** an override a viewer can see work (weight 1; stated, not hidden).

**Method.**
1. *Paper first.* Complete the failure analysis of every candidate (the G table), a coverage matrix (scenario by override: covers, partly, does not), and a simple model of the
   probability that the safe state is reached given a failure probability per demand for each override and for each program; the result is a table and a sensitivity plot (how
   the answer changes if overrides are ten times less reliable than assumed).
2. *Then the rig.* With the platform and the injector built: for each scenario cause it (a killed process, a held GPIO, an overloaded rail) and time the override with the logic analyzer: from
   the action to the platform at rest, to a node powered, to ACT in Safe. Each override is operated 20 times for its own failure rate (does it always act).
3. *Failure of the override.* Break each wire in turn (open a lead, lift a ground) and record the resulting state against the one designed in section 4.

**Output.** One table (option against scenario: time to safe state, or "none"), the G table with measured effects, one coverage figure, one line on cost per scenario covered.

**Decision rule.** The smallest option in which every scenario has at least one override that needs no program, with no override whose own harm (a spurious activation, a snap, a cut) is worse than the failure it covers; on a tie, the cheaper and the easier to test.
Expected finding, to be shown or refuted: **O2 + H4** covers every scenario that a person can reach at the bench, and the per-node kill switches of O4 add demonstration and a way to cause F01 without the Pico, but cover no scenario that
the Nucleo reset buttons, the pluggable stubs and H4 do not already cover; and **testing** the overrides is a larger part of their value than counting them.

**Dependencies and timing.** The paper part needs only `HARDWARE_OVERRIDE.md` and can be done now; it decides the parts to order. The measured part needs the platform, the injector (P1-5) and the supervisor (S2b). Before the override parts are bought.

**Talking point.** "I asked what the system does when all the software is wrong, listed the ways the safety switches could themselves fail, and chose how many to build from that."

## 9d. TS-18: testing hardware-facing firmware without the hardware (docs/PICO_TESTS.md)

**Question.** The Pico's loop talks to a USB port, two PWM channels, four relay pins and a watchdog. Four ways to test it before the board exists: **(1)** the real loop on the host under a fake
board (done: `tests/test_pico_app.cpp`), **(2)** the real application on `native_sim` with Zephyr's emulated GPIO, a fake PWM and the UART on a pseudo-terminal, **(3)** an instruction-set
emulator running the real binary (not verified to exist for the RP2350 with USB and PWM), **(4)** the board only. Each finds some bugs and is blind to others: (1) cannot see a wrong pin number, a wrong
devicetree line, a PWM period that does not fit, a USB descriptor error; (2) sees the devicetree and driver wiring but not the real timing or the electrical side; (3) sees the machine code but not the
board's analogue behaviour; (4) sees everything and is slow, scarce and cannot be put in CI.

**Method: seeded bugs.** Plant a list of realistic bugs of three kinds (logic in the loop, wiring in the glue and the devicetree, timing and electrical), run each way of testing, and count which bugs it finds and the
effort (work to build the method, run time, upkeep). The first data is in hand: **mutation testing of the loop and its portable parts** (`tools/mutation`, 40 mutants of the platform driver, the injector, the link and the loop)
killed 36 of 40 on the first draft of the tests; of the four survivors three were real gaps (a saturation threshold, a length check, a servo map that was never given different values) and one was equivalent (a path the link cannot reach).
After three tests were added all 39 non-equivalent mutants are killed. So the fake board reaches full mutation score on logic, **by construction not on glue**: a planted wrong relay pin or wrong PWM channel is invisible to it.

**Criteria and weights.** Bugs found, by kind (weight 3); effort to build and keep (weight 2); speed and whether it runs in CI (weight 2); how much of the real code path is exercised (weight 2).

**Output.** A table: method against bug kind, with the number found of the number planted, and the cost; one line saying where each method stops being worth its cost.

**Decision rule.** The cheapest set of methods that finds every planted logic and wiring bug before the board; whatever remains goes on the bring-up list.

**Dependencies and timing.** Method (1) exists. (2) needs a small refactor (the link on a chosen UART instead of the CDC device) and a test-only debug message. Run the seeded-bug experiment before the board arrives.

**Talking point.** "I measured what each way of testing firmware without the hardware can and cannot see, with planted bugs, and used mutation testing as the yardstick."

## 9e. TS-19: the injector's cut semantics

**Question.** A relay cut can end by itself after a time the PC names (today, at most 30 s, refreshed by the PC; ADR-026), stay until an explicit restore command, or stay only while a heartbeat from the PC keeps arriving.
What is safest, and what do the tests need? The supervisor's tests ask for outages of minutes (two power-cycles in 5 minutes mean `DEAD`, `SUPERVISOR.md` section 5), and a latched cut is the more faithful power loss; but a crashed PC with a
latched cut leaves a node dead, which looks like a node fault (G6).

**Options.** A: timed, self-ending, refreshed (today). B: latched, ended by a restore command or a power cycle of the Pico. C: latched but released after a link timeout (a heartbeat). D: A with a longer limit.

**Criteria.** Safe when the PC dies (weight 3); can reproduce the long outages the supervisor and FDIR tests need (weight 2); how easy it is to leave a node cut by mistake (weight 3); how faithful the cut is to a real power loss (weight 1); effort (weight 1).

**Method.** Scripted scenarios: kill the PC tool during a cut; pull the cable; reset the Pico; a 5-minute outage; the supervisor's `DEAD` sequence. Count nodes left cut and time to restore. The analysis is largely on paper: A and C are fail-passive by construction, B is not; the question for the experiment is whether the PC's refresh at 1 Hz is reliable enough for 5-minute outages.

**Decision rule.** The most faithful option that leaves no node cut after any PC or link failure; expected: A with the PC refreshing, which needs no change and no longer limit.

**Dependencies and timing.** Needs the injector on the bench and the supervisor's logic; small.

**Talking point.** "I chose how a fault injector fails, so that the tool that breaks things cannot leave things broken."

## 9f. TS-20: what the platform does when the commands stop

**Question.** PLAT-002 says: hold after 100 ms, level after 1 s (at 30 degrees per second). Other choices are possible: hold for ever, level at once, stop the pulse (the servo goes limp or holds depending on its electronics), or hold and then stop the pulse. Which has the smallest mechanical
shock, the least confusing motion for the IMUs, and a state the flight computers read correctly? Levelling while the flight computers are flying produces a motion they will treat as a fault or as vehicle motion.

**Options.** H: hold for ever. L: level at once, rate-limited (a few rates). T (today): hold 100 ms then level at 30 degrees per second after 1 s. S: stop the pulse after the hold. TS: hold then level, then stop the pulse.

**Criteria and weights.** Shock and servo current (weight 3); what the IMUs and the flight computers see: a slow ramp should not trip their checks (weight 3); servo heating or stall when holding for ever against a load (weight 2); what the servo does with no pulse (weight 2, unknown: PICO_TESTS E1); operator surprise (weight 1).

**Method.** On the platform: stop the stream at several tilts and times, for each option measure the platform's motion (the IMUs and a camera), the servo current, and the flight computers' reaction (vote disagreements, Safe). A parameter sweep of the hold time (50 to 500 ms) and the level rate (10 to 100 degrees per second). Needs the real servo.

**Decision rule.** The option with the smallest peak shock and no false fault in the flight computers; tie: the one that does not depend on the servo's no-pulse behaviour. Expected: T with a slower level rate, once E1 shows what the servo does without a pulse.

**Dependencies and timing.** After E1 to E6 of `PICO_TESTS.md`; the platform rig.

**Talking point.** "I tuned what a platform does when it loses its commander, from measured shock and from what the flight computers made of it."

## 9g. TS-21: where the mission clock lives (docs/LAUNCH_SEQUENCE.md section 3)

**Question.** The guidance and gain schedules need one number that every computer agrees on: the time since T-zero. It can come from four places. The owner's first idea is the supervisor, which is independent and cannot be affected by the rest of the system.
What does each option do when the supervisor, the sync master, or one computer's link fails during the ascent, and when two computers see the launch command a frame apart?

**Options.** A: the supervisor sends mission time continuously (needs a bus transmitter, or a discrete encoding). B (recommended, hybrid): the supervisor decides T-zero with a discrete line, the sync master latches it and carries mission time in SYNC, the supervisor
checks it with its own clock. C: mission time in SYNC from the sync master alone, started by the command. D: each computer counts from its own sight of the command.

**Criteria and weights.** Agreement of all computers on T (weight 3: a frame of difference diverges the replicas, TS-16); survives a supervisor failure after T-zero (weight 3: TFC-SUP-007); survives a sync-master takeover without a jump (weight 3);
a restarted computer rejoins on the right schedule (weight 2); independence: an error in the time is detectable from outside (weight 2); complexity and parts (weight 1).

**Method.** On the live triplex with the closed loop and fault injection: kill the sync master at 20 s, at 60 s (max-Q); reset a follower at 40 s; delay the launch command to one computer by one frame; reset the (virtual or real) supervisor at 60 s; drop SYNC frames for 3, 10, 50 frames; measure the largest difference in the schedule index between computers, the attitude error, and whether Safe was entered.

**Expected finding, to be shown or refuted.** A fails the supervisor-reset case by construction; D fails the one-frame case; C and B pass all, and B adds the independent check and the supervisor's authority over T-zero at the price of one discrete line.

**Dependencies and timing.** Needs the mission time in SYNC (the next step of the launch sequence); the supervisor part waits for its parts.

**Talking point.** "I put the authority for launch in an independent computer and the clock in the replicated ones, and tested what each choice does when each part fails."

## 9h. TS-22: the independent time reference (docs/MISSION_CLOCK.md)

**Question.** The clock of record must be independent of the flight computers and good enough for the mission length. The five-year drift is 316 s for a TCXO class part, 16 s for an oven crystal and milliseconds for an atomic clock (datasheet-class figures, not checked).
Which reference gives an error the project can state and defend at an acceptable cost, and does time correlation against the PC's UTC (the way a spacecraft's clock is correlated against ground time) make the cheap one enough?

**Options.** A: the TCXO RTC module (chosen for the parts list), alone. B: A with correlation against the PC's NTP-disciplined UTC. C: B with a GPS receiver's pulse per second as an extra reference (needs a sky view). D: an oven-controlled oscillator with B. E: a chip-scale atomic clock (not priced; expected to be out of budget).

**Criteria and weights.** The time error after 24 h, 30 days and an extrapolated five years, stated with its uncertainty (weight 3); behaviour through a power cut and a reset (weight 2); cost and parts (weight 2); how the error shows up when the reference fails (weight 2); complexity of the correlation software (weight 1).

**Method.** On the bench, with the supervisor and its clock module: log the oscillator's counter against the PC's UTC and against a GPS pulse for several days; compute the drift, the Allan deviation and the aging; fit the correlation (offset and drift) and measure the residual after prediction over 1 h, 1 day and a week; power-cut the rig and compare the RTC's time after.
The time-error budget is then extrapolated to five years with the measured aging, and compared with the table of section 2 of the mission-clock page.

**Output.** A table: option against error at 24 h, 30 days and five years (extrapolated), with the uncertainty; one plot of the correlation residual against the prediction interval; one line on what each extra cost buys.

**Decision rule.** The cheapest option whose stated error over the mission length stays inside the requirement, where the requirement is set by what the time is used for (time-tagging to a second is easier than coordinating an event to a millisecond). Expected: B is enough for a desk demonstration and for a five-year claim *with* correlation; a real five-year, uncorrelated mission needs D or E.

**Dependencies and timing.** The supervisor's firmware and parts (the TCXO module, the second Pico). The correlation software is host-side and can be written first.

**Talking point.** "I asked what keeps the time of a mission that outlasts any one computer, measured what an oscillator drifts, and designed the correlation a spacecraft uses against ground time."

## 9i. TS-23: how often the replicas resynchronise (docs/RESYNC.md, ADR-030)

**Question.** The state resynchronisation (TS-16 option C) has one number to choose, the period. A short one heals a lost frame sooner and costs more bus time; a long one costs nothing and leaves the states apart for longer. What does the period actually change, how much, and where is the best value?

**What the period touches (the criteria).**
1. *Flight survival* on a lossy bus: is a flight lost? (weight 3)
2. *The digest check*: how long the three digests stay different, which decides whether a digest persistence can be set that never counts a healed mismatch. (weight 2)
3. *Time to heal and to rejoin*: a lost frame leaves the states apart for about half a period on average, a computer that restarted rejoins within one period, two if one resync is skipped. A probation lasts 1 s (100 frames), so a period well beyond that stretches every rejoin. (weight 2)
4. *Bus load and the schedule*: each resync is a burst of 12 frames (about 130 microseconds each at 1 Mbit/s, so 1.6 ms, in a frame that carries about 1.8 ms already: 34 % busy that frame, 18 % in every other) and the manager's hand-over moves in that frame. Average added load = 1.6 ms / (period x 10 ms). (weight 1: it is small at every period tried, but it is paid in the frame budget of the target, which is not measured)
5. *Detecting a failing state*: a sudden corruption is healed at the next resync and reported if it is larger than the large-correction limit (0.5 degree), at any period; but a state that **drifts** away at rate r is healed before it is reported if r x period is below that limit. At period 100 (1 s) a drift below 0.5 degree per second is healed in silence; at 1000 (10 s) below 0.05. A *short* period therefore hides slow faults that a long one would show. (weight 1: sudden upsets are the expected fault; a slow drift of the estimator alone has no known cause here)
6. *Cost of adopting at all*: the state is quantised (0.0035 degree of attitude) and mid-valued every time. Measured on a clean bus at every period: no change in the flight and no digest difference (first table), so the cost is zero in this model.

**Method.** `tools/sim/tfc_resync.cpp`: the closed loop of the 60 s ascent (three flight functions, the real ACT logic, the simulated vehicle) for each period and each frame-loss rate, 32 independent random loss patterns per cell (the loss is independent at each receiver for every sensor frame and every resync chunk, the pessimistic model of TS-16). A flight is lost if ACT enters Safe or the attitude strays more than 5 degrees from the program after 3 s. "Digest persistence P + 50 / 2P + 50" counts the seeds in which a digest mismatch lasted longer than that, i.e. in which a manager set to that persistence would have counted it.

**Results** (`./build/rel/tfc_resync --seeds 32`; the numbers are means over the 32 patterns unless a longest value or a count is named).

*Flights lost, of 32:*
| Loss per frame | none | 10 | 20 | 50 | 100 | 200 | 500 | 1000 |
|---|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 0.01 % | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 0.1 % | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 % | **11** | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 5 % | **28** | 0 | 0 | 1 | 1 | 1 | 2 | 3 |

*Share of the frames in which the three digests are not all equal (the lower, the more useful the digest check):*
| Loss per frame | none | 10 | 20 | 50 | 100 | 200 | 500 | 1000 |
|---|---|---|---|---|---|---|---|---|
| 0.01 % | 69 % | 0.5 % | 0.7 % | 1.3 % | 2.3 % | 3.9 % | 10.5 % | 20.1 % |
| 0.1 % | 95 % | 4.1 % | 5.7 % | 10.5 % | 17.7 % | 29 % | 56 % | 75 % |
| 1 % | 99 % | 34 % | 44 % | 63 % | 77 % | 87 % | 95 % | 97 % |
| 5 % | 99.9 % | 86 % | 92 % | 96 % | 98 % | 99 % | 99 % | 99.7 % |

*Seeds (of 32) in which a digest persistence of 2P + 50 would have counted a mismatch (a false flag: nothing was wrong that the resync had not healed):*
| Loss per frame | 10 | 20 | 50 | 100 | 200 | 500 | 1000 |
|---|---|---|---|---|---|---|---|
| 0.01 % and 0.1 % | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 % | 0 | 3 | 9 | 21 | 16 | 8 | 4 |
| 5 % | 32 | 32 | 32 | 32 | 32 | 32 | 30 |

(With persistence P + 50 instead, the 0.1 % row already flags 1 seed at period 100, 2 at 200 and 6 at 500: one skipped resync makes a mismatch two periods long, so **2P + 50 is the persistence to use**.) The resyncs a computer skips (a chunk of a peer's state lost) are a property of the loss and not of the period: 0.1 % at 0.8 %, 1 % at 7.8 %, 5 % at 33 %. The command difference between computers and the frames beyond ACT's agreement tolerance (about 5 at 0.1 % and 48 at 1 %) also do not depend on the period, and the flight's attitude error stays at 0.4 to 0.65 degree: over minutes, not seconds, is when an unhealed difference grows enough to matter, which is why even a period of 1000 flies at 1 %.

*Bus load* (analytic): the burst is 1.6 ms at every period; the average added load is 1.6 % at period 10, 0.8 % at 20, 0.31 % at 50, 0.16 % at 100, 0.08 % at 200 and below 0.03 % beyond 500.

**What the table says.**
- *Flight survival* is flat up to 1 % loss at every period, because a divergence has to grow over many seconds to matter; only at 5 % loss does a long period lose flights (1 to 3 of 32 beyond 50).
- *The digest check* is where the period matters. At a realistic loss (0.1 % and below) period 100 leaves the digests equal 82 % of the time and a persistence of 2P + 50 = 250 never false-flags; at 1 % loss only periods up to 20 keep the digest check mostly clean, and at 5 % none does.
- *Healing and rejoin time* scale directly with the period: 0.1 s at 10, 1 s at 100, 10 s at 1000 (twice that if a resync is skipped).
- *The cost* is small everywhere; period 10 is the only one that adds more than 1 % average bus load.
- *Drift detection* runs the other way: it prefers the long periods.

*What no period fixes* (the fault manager's command tolerance, `docs/RESYNC.md` section 6b): frames in which two computers' commands differ by more than the manager's 0.01 degree are caused by the immediate effect of a lost frame, so they do not depend on the period (28 per flight at 0.1 % loss at period 10 and at 100, 284 and 281 at 1 %), but the resync is what keeps them from lasting: without it every flight at 0.1 % loss has a run of three such frames in a row, with it none of 16 does; at 1 % loss 7 to 8 of 16 flights still do, at any period. At that loss rate the tolerance, or option B, has to change.

**Decision rule.** The *longest* period for which (a) no flight is lost at 1 % loss, (b) a digest persistence of 2P + 50 never false-flags at 0.1 % loss and (c) a lost frame is healed, and a restarted computer rejoins, within one probation (1 s). That is **period 100**: (a) holds up to period 1000, (b) holds up to 1000 at 0.1 % (0 of 32), (c) fails from 200 up. If the rig measures a loss rate near 1 % the digest check is only usable at period 20 or below and the rule's (b) should be re-run at that rate; the choice then trades 0.8 % of the bus for a usable digest.

**Decision (5 Oct 2026, proposed): default period 100 frames and a digest persistence of 2P + 50 = 250 frames, both configurable in the firmware (`TFC_RESYNC_PERIOD`), to be revisited with the measured loss of the real bus.** What would change it: a measured loss above about 0.3 % (shorten to 20 to 50); a frame budget on the target that cannot take the burst every second (lengthen); a failing-state fault that drifts rather than jumps (needs a second rule, below).

**Open question this exposed: slow drift.** Resynchronisation heals a drift below `large_limit / period` per second without a report. A cheap second rule would count how many resyncs *in a row* changed the same computer's state while the others' did not (on a bus with a loss of 0.1 % or less this is rare for a healthy computer, and a drifting one is changed every time), and report it as a bad frame. It was not built: the measured loss rate is needed to set its count. Recorded as a follow-up with the real-bus measurement.

**The diverse node.** It is left out of the resync by a switch (`CONFIG_TFC_RESYNC_GROUP`) and does not appear in these tables; the reasons, the measured command difference it then shows and the option of a slew-limited adoption are in `docs/RESYNC.md` section 6.

**Talking point.** "I measured what the resync period buys: flight survival does not depend on it up to 1 % loss, the usefulness of the digest check depends on it strongly, and the cost is a 1.6 ms burst; the best value is the longest one that still heals within a probation."

## 10. Schedule

| When | Study | Why then |
|---|---|---|
| Done 4 Oct | TS-0 | Decided: Lite |
| Late October, with the split (before S3, 3 Nov) | TS-15 | Chooses the degradation rule that the split implements |
| October, on the simulator | TS-1, TS-3, TS-6 | Everything they need exists or is a small campaign group; hardware is on order |
| October to November | TS-2, TS-4 | The model and the toy vehicle are independent of the hardware; TS-4 is re-run on the 6-DOF simulator at M3 |
| Now (paper), then with the injector and supervisor | TS-17 | The paper part decides which override parts to order; the measured part needs P1-5 and S2b |
| After S3 (Nov) | TS-5, TS-8, TS-9 | Need the rig |
| As time allows | TS-7, TS-10 to TS-13 | Short write-ups |

M4 (17 Nov to 1 Dec) and M5 (write-up) are where these become the project's argument. Each finished study is one page in `docs/`, one figure,
and one line in the write-up; **if time runs short, finish fewer studies completely** rather than starting all.
