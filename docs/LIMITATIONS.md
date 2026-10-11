# Limitations

> Status: **reference**, the honest list. Reviewed 8 Oct 2026. Each entry says what is *not* claimed and why, so that a reader (or an interviewer) does not have to find it out. Where a limit has a remedy, the entry names it. Nothing here is hidden elsewhere:
> the per-fault gaps are in [`verification/FMEA.md`](verification/FMEA.md) and the campaign's edge cases in [`verification/FAULT_CAMPAIGN.md`](verification/FAULT_CAMPAIGN.md) section 6.

## 1. What the claim is, and what it is not

**Claimed (ADR-023, TFC-SYS-006):** after any one fault in a flight computer, in an IMU, or in a computer's link to the bus, the voted output continues within 0.5 degree with no interruption longer than one frame; after the next fault the system degrades to Duplex and then
to Simplex, and holds its last good output and asks for the operator when a disagreement cannot be attributed. This is *fail-operational through sensing and computing*.

**Not claimed: fail-operational beyond that.** These are single points, and the system is fail-passive about them (hold, then null):
- **ACT**, the actuator node (a second ACT with a supervisor-driven selector is a stretch goal, `design/FUTURE_WORK.md`);
- **the shared CAN bus** (one segment; a babbler or a short takes it all; a second bus is a stretch goal);
- **the supervisor** (it is software too, on one Pico; unplugged or hung it must leave every node running, TFC-SUP-007, and that is tested on the host, not yet on the board);
- **the power rails** (one 5 V adapter feeds every node, the relay coils and both Picos; one star ground; scenario SD9 of TS-17 has no override that removes a failing adapter);
- **the PC and the simulator**, which are the test environment, not the flight system.
The servo, its rail and the platform are the plant, not the flight computer, and are outside the claim.

## 2. Faults the design cannot see or cannot decide

- **A bug common to every replica.** The three computers run the same code on the same inputs; a shared software bug is a shared failure. The only defence is diversity: node C on the previous known-good release (ADR-021), which protects against a
  regression, **not** against a bug present in both releases. A WARM computer shares the inputs of the others and so shares any bug that depends only on its inputs (MISSION_PHASES). Release awareness needs all three heartbeats and exactly one computer on another release.
- **A Duplex disagreement that nobody can attribute** (a bias between one and about 2.3 tolerances, a slow drift, a digest mismatch, a signal that barely moves) is not settled by the system: it holds the last good value and requests Safe, which only an operator ends.
  That is the fail-safe outcome (ADR-008), not a fix. Analytical redundancy would settle more of these cases and is not built (`FUTURE_WORK.md` 2.1).
- **A frame a few milliseconds early inside its own window is not seen.** Frames carry SYNC's number, so whole-frame errors are seen; the arrival-time check against the slot waits for the board (TFC-FDIR-037).
- **Real CAN effects are not modelled:** inconsistent omission and duplication, error-frame storms, arbitration under load, the real harm of a babbler, wiring faults. The digest cross-check and the state resync are the designed answer to the first; nothing proves it on a real bus.
- **Simplex in ascent alerts only.** There is no automatic abort: an abort is a launch rule, not a flight-computer rule (owner's decision, 4 Oct 2026). Below a phase's minimum number of voters the manager reports; it does not act.
- **Single-fault focus.** The campaign covers every fault kind alone, in pairs, as correlated faults, in cascades, and with operator commands, but not every combination; the claim for two simultaneous independent faults is "degrades as designed in the cases tried".
- **No radiation or EMI test.** Single-event upsets are modelled by bit flips in the manager's own state (ADR-015); that is a software model, not a measurement.

## 3. What is measured, and what is a target

- **Nothing here has run on a board.** Every timing number (jitter, WCET, bus load, frame cost) is either a host figure or a *target*; SYS-001 to SYS-003 and every `M` verification wait for the rig (`STATUS.md` section 4).
- **The persistence constants** (3-of-5, the leaky count, dwell and probation lengths, the strike limits) and the Safe hold time are tuned on a simulation of frame-level faults; real error rates will differ and they are re-tuned from measurements (TS-1, TS-4).
- **The resynchronisation period (100 frames) and the digest persistence (250) were measured on the simulator, not on a real bus**, whose loss rate cannot be known until the hardware is set up. One finding limits what resync can do: the manager's 0.01 degree command tolerance
  still latches healthy computers at about 1 % frame loss at any period, while ACT's 0.05 degree tolerance does not; the remedy is a measured loss rate and a decision on the tolerance (`design/RESYNC.md` 6b).
- **The mission clock's drift figures** (a TCXO is about 2 ppm, an oscillator about 20 ppm) are datasheet-class numbers from memory and **not checked**; the clock of record is correlated against the PC's UTC and its error is bounded by that, not by the datasheet (`design/MISSION_CLOCK.md`).

## 4. The simulator is not the real world

- **The vehicles are invented or estimated** (the reference vehicle and the examples of `vehicles/` are generic, with plausible numbers; `starship.json` is a Starship V3 class vehicle built from publicly reported numbers and *my estimates* of the unpublished ones: its dry masses, its vacuum engine's thrust, its pitch program and its ship's throttle, listed in `design/VIEWER.md` section 7); "real life" in `design/SIM_FIDELITY.md` means the behaviour a real vehicle and IMU have, not agreement with a particular rocket (Flight 12's trajectory is matched in kind, not in numbers). The IMU error model is not calibrated to the real part.
- **The platform is bandwidth-limited by its servos**, so the simulated ascent is time-scaled and the platform clamps rate and travel; the closed loop on the rig is not the closed loop of the simulator.
- **The virtual peers are Python on desktop Linux**: no latency guarantee, and they send a synthetic digest (the real firmware computes a real one).
- **Thrust corrupts the accelerometer as a gravity reference** on a real ascent; the estimator coasts on the gyro, and the accelerometer correction is off in flight.

## 5. Security is bench-grade

The ground-command key is the public SipHash test key by default and protects nothing; there is no key provisioning; the tag comparison is not constant-time; the internal bus is trusted and the heartbeat and the state share are not authenticated; the uplink itself is not
modelled. Fine for a bench, not for a flight uplink (`FUTURE_WORK.md` 2.4). One ground counter more than 32 ahead of a computer's needs that computer restarted. Every flight computer decides alone: there is no membership agreement.

## 6. The supervisor and the overrides

- **SUP-Lite sees discrete lines only.** It cannot see a vote, a bus frame or an arrival margin, so it can only act on a unit that has *stopped*; it never decides whether a working node's data are good (TFC-SUP-010). It does not distribute time to the nodes (TFC-SUP-013 is not built).
- **A hardware override can itself fail.** Each is a latent loss of protection if it is dead when needed, or a lost run if it is operated by accident; that is why `P-HWO-01` operates each one and the supervisor reports the untested ones. The override parts are **not yet ordered**, and no
  override has been wired or tested.
- **COLD is manual.** A node is cold when the operator tells the supervisor to `hold` it; there is no automatic cold standby.

## 7. Scope, and how the evidence was produced

- **Educational scale.** This shows the technique. It is not flight-qualified, not certified to DO-178C or any standard, uses no qualified tools, and has had no independent review: one person wrote the design, the code, the tests and the oracles.
- **Mutation testing proves the tests catch the bugs that were injected**, not all bugs. Equivalent mutants (changes that cannot alter behaviour) are judged by reasoning and listed in `tools/mutation/mutations.py` with the reason.
- **Coverage is structural** (every line, 98.4 % of branches); it does not say the right thing is checked at each line.
- **Public sources only.** Statements about SpaceX or any real program come from public material or inference; nothing here is insider knowledge and the project does not claim to replicate any SpaceX design.

## 8. The flight console

- **A bench tool, not flight software.** It shows what the computers say and sends the operator's authenticated commands; no flight decision depends on it. It is not a ground station for a real vehicle (the key is the public bench key, section 5), and nothing in it is qualified.
- **Its serial paths have not met a board.** The Nucleos' consoles, the supervisor and the Pico are tested against pseudo-terminals and a fake Pico only (`design/CONSOLE.md` section 10).
- **It judges nothing.** The deviation bars and the go/no-go are the console's own reading of the bus, labelled as such; the computers' verdicts are their heartbeats and ACT's frame. A reason (why a node was latched) is on the node's console and is shown only when a console is attached.
- **The fault lab's virtual B and C are not full nodes** (no heartbeat, so no mode, role or view), and the rig flies another vehicle only after the Launch tab has built the flight computers for it (ADR-036, `design/CONSOLE.md` section 8): that is a build of the real application for `native_sim`, not a flash of a board, and a vehicle whose design the table generator cannot close is refused, not flown.
- **The virtual rig is sensitive to a loaded host**, like the live tests: starting a browser during a run made the computers latch one another out (reproduced here, and so did twelve busy loops for four seconds), because the rig's processes have no real-time priority and switch threads about 14 000 times a second. The console now keeps the user's other programs off the rig's CPUs while it runs (ADR-040): the same burn and a browser start no longer latch anything, and a full flight under a browser and a test run stayed in Triplex. That is a measurement on one 12-thread host, not a guarantee: a stall from the kernel, the GPU driver or another user is not prevented, a host with fewer than 8 logical CPUs gets no isolation, and real-time scheduling (the better answer) was not tested for lack of a privilege.
- **Range is inertial** (a rotating planet's own motion is not taken out), the bus-load figure is arithmetic (111 bits a frame, no stuffing) and not a measurement, and the nominal overlay is the reference vehicle's.

## 9. The 3D viewer and its pictures

- **It draws the simulator's state and nothing it does not have.** After separation the spent stage is carried on by the viewer, not simulated; there is **no re-entry, no boost-back, no landing and no catch**, no orbital insertion burn or coast beyond the vehicle's cut-off, and no payload deployment.
- **The flow pictures are not CFD.** The pressure map (modified Newtonian), the streamlines (slender body) and the shock (Taylor-Maccoll) are labelled illustrations of a model, and the loads are a lumped-mass estimate, not a structural analysis (`design/VIEWER.md` sections 6, 9).
- **The Starship's grid fins and flaps are a picture** (the vehicle file and the physics have none); the Block 3 numbers are public or stated estimates, and the simulator's aerodynamics are a slender-body model, not the vehicle's.
- **The Earth is a picture of the ground, not terrain**: Landsat over the United States is the year 2000's composite, Blue Marble is a monthly composite, there is no height, the sun is set by two sliders and not by a date and a place, and the weather is the viewer's own. Outside the United States the finest picture is 4.9 km a pixel. The pad is placed at the known launch site nearest the simulator's latitude, because the simulator's launch point has no longitude.
- **Performance was measured on one GPU and one browser** (`design/VIEWER.md` section 13); GPU memory for the most detailed imagery is computed (about 400 MB), not measured, and the pictures' GPU upload and their load over a network were not measured (the fetch and decode from the same machine took 50 to 70 ms).
- **The event list is the page's own**: the viewer derives lift-off, staging, max-Q and the rest from the poses it has seen, so a page opened (or a flight sought) after an event does not list it, and the server does not replay the earlier ones.

## 10. Guidance, navigation and the stage's return (ADR-039)

* **Host only.** The guidance, the navigation and the mission have run in the simulator and nowhere else. **How long they take on the STM32G474 is not known**: software double precision on a single-precision FPU, a PEG cycle of seven predictors of twelve steps, a descent predictor of 600 to 900 steps. The firmware does not call them, does not receive the GNSS frames (`0x506` to `0x508`) or send the propulsion and surface frames (`0x210`, `0x220`), and the tables are not generated into the image. The rig, the console and the viewer do not fly or show a mission.
* **A model of a vehicle, not a vehicle.** The Starship V3 class stack uses public numbers and my estimates (`RECOVERY.md` section 2: the dry masses, the vacuum Raptor, the grid fins, the tower, every tuning number are mine); its aerodynamics are engineering estimates (Newtonian impact theory, plate and grid-fin formulas), **not wind-tunnel or CFD data**. Nothing in the results says anything about SpaceX's vehicle or software.
* **The catch is marginal.** Four of the first 36 GNSS noise seeds miss (3.0 to 5.4 m from the site), and the catch needs a receiver of 3 m or better; the model has no differential correction near the tower. It was tuned on one vehicle in the nominal atmosphere with the built-in mean wind: no gusts, no turbulence, no engine that fails to light, no sensor fault in flight. A change of the booster's propellant at staging moves the gate by kilometres.
* **The ship stops in orbit** (234 x 257 km for a target of 250, an error of the engines' tail-off model). It does not come back.
* **One GNSS receiver** feeds all three computers: a bad fix is common to them. The navigation has no covariance and no sensor-bias states.
* **The consumer's vote** of the propulsion and surface commands is in the host loop in memory; the frames' quantisation is not in the results.
* **The ideal mode is not a result.** The design flies the mission with the attitude forced; a result is only the closed loop's.
