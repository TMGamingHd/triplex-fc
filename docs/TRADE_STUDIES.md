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

## 10. Schedule

| When | Study | Why then |
|---|---|---|
| Done 4 Oct | TS-0 | Decided: Lite |
| Late October, with the split (before S3, 3 Nov) | TS-15 | Chooses the degradation rule that the split implements |
| October, on the simulator | TS-1, TS-3, TS-6 | Everything they need exists or is a small campaign group; hardware is on order |
| October to November | TS-2, TS-4 | The model and the toy vehicle are independent of the hardware; TS-4 is re-run on the 6-DOF simulator at M3 |
| After S3 (Nov) | TS-5, TS-8, TS-9 | Need the rig |
| As time allows | TS-7, TS-10 to TS-13 | Short write-ups |

M4 (17 Nov to 1 Dec) and M5 (write-up) are where these become the project's argument. Each finished study is one page in `docs/`, one figure,
and one line in the write-up; **if time runs short, finish fewer studies completely** rather than starting all.
