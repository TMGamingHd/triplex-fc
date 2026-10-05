# State resynchronisation (TS-16 option C): keeping the replicas together on a lossy bus

> Status: **built on the host and in the firmware** (core, closed loop, manager hooks, the node loop, a live test on `vcan0`; ADR-030). **Not run on the target or the rig.** The tolerances and the
> period are proposals measured on the simulator (TS-23), not on the real CAN bus, whose loss rate cannot be measured until the hardware is set up.

## 1. The problem, in one paragraph
The estimator and the controller carry state from frame to frame. Two computers that were given a different set of sensor frames in one frame (one lost, one late) compute different
inputs, and their states differ from then on: the attitude is the gyro's integral, and only the accelerometer's slow correction pulls it back (and under thrust the accelerometer is not used at all). `TS-16`
measured it: with four lost frames in a 60 s flight the three state digests were different for 82 % of the frames, and with 1 % loss the flight was lost, because ACT, which accepts three
commands as agreeing only within 0.05 degree, lost its vote and went to Safe.

## 2. What it does
Every `period` frames (proposal: 100, once a second), in the last frame of the period, after the command slot:
1. Each computer takes its state *as it is after this frame's step*, quantises it to twelve 16-bit words (the quaternion, the gyro-bias integrator, the controller's integrators and last outputs, the low byte of the update count with two flags) and broadcasts it in four frames (`0x420 + 4n + k`, `PROTOCOL.md`).
2. Each computer collects the chunks that carry this frame's number. It adopts a state only if **every healthy computer's state arrived whole**.
3. The adopted state is the **mid-value of each word** of the three (or the mean of two that agree, within a tolerance, if only two are healthy). Every computer that got all the shares computes the same integers, de-quantises them the same way, renormalises the quaternion the same way (only + − × ÷ and √, as everywhere in the flight function), and so holds **the same bits**.
4. Before the next frame the adopted state replaces its own. Nothing else changes: the schedules, the gains and the sensor inputs of the next frame are as before.

Why the mid-value and not the mean: one computer whose state has gone wrong, by any amount, does not move the vote (the same reason the sensor and command votes are mid-values).
Why only a *complete* vote: a computer that missed a chunk and voted over the rest would adopt a state that no other computer holds, which is the original problem again; it
adopts nothing and the next period tries again (`Why::Incomplete`). Why the quaternion's scalar part is made non-negative: q and −q are the same attitude, and the mid-value of two signs would be a nonsense attitude.

## 3. What it also measures, so that healing does not hide a failing computer
Resynchronisation would make a corrupted estimator state vanish at the next period. A fault that heals itself without anyone being told is a latent fault (it is the criterion that carries weight 3 in TS-16), so the vote also reports, per computer, how far its own state was from the result:
- **changed**: any word differed (what a lost frame leaves; a count, for the telemetry);
- **large**: a word differed by more than `Config::large_*` (proposal: about 0.5 degree of attitude or controller state, 0.3 degree per second of bias). A lost frame does not do this; a corrupted state does.

The firmware passes the large ones to the manager (`RedundancyManager::report_state_correction`). They are **bad frames for that computer through the same detectors as any other reason** (`reason::kResync`, "state far from the vote"): one is not a latch; three in five frames, or a leaky count of them, is. So a computer that needs correcting again and again is isolated, and one that was hit once is healed and noted.

## 4. The digest and its persistence
The state digest in every command is a fingerprint of the quantised state; it differs for any difference, however small, so on a lossy bus it is almost never equal for long (table below). Its rule in the manager becomes **persistence-based**: `digest_persist_frames` (default 1, as before) is how many frames in a row a mismatch must last before it counts, for the computer it blames or as an unresolved disagreement. With the resync, set it to a little more than the period (proposal 150): a mismatch that the resync healed within the period never counts, one that survives a resync does. It is a secondary check now; the primary one is the vote on the commands (ACT and the manager), and the correction report above.

## 5. Measured on the simulator (closed loop, 60 s ascent, `sim::Loop::frame_loss_prob`, resync period 100)
Each sensor frame of another computer is lost independently at each receiver; so is each resync chunk. A computer always gets its own frames.

| Loss per frame | Without resync: flight | Frames over ACT's tolerance (no resync → resync) | With resync: flight | Resyncs adopted / skipped (of 180) | Frames with the three digests not all equal |
|---|---|---|---|---|---|
| 0 | flies | 0 → 0 | flies, nothing to correct | 180 / 0 | 0 % |
| 0.1 % | flies | 8 → 7 | flies | 179 / 1 | 17 % (longest run 94) |
| 1 % | **lost (Safe)** | 1 408 → 59 | **flies**, max error 0.55 degree | 165 / 15 | 79 % (longest 247) |
| 5 % | lost | 1 761 → 257 | flies | 116 / 64 | 98 % |
| 10 % | lost | 1 913 → 558 | flies | 73 / 107 | 99.7 % |

(With period 50, the flight at 10 % is lost again: the loop's resync chunks are lost too, so a shorter period does not help once most resyncs fail; the table's period of 100 is a proposal, not an optimum.)
Reading it: the resync turns a flight lost at 1 % loss into one that flies to 10 %. It does **not** make the digests equal at those loss rates, because a new difference arises every few frames; at 1 % the digest persistence of section 4 would still false-flag (longest run 247 > 150), so at that loss rate the digest check is not usable and the bus is the problem. A healthy wired CAN bus loses far fewer frames (the caveat in TS-16 of `TRADE_STUDIES.md`: classic CAN is built to deliver a frame to all nodes or to none), so the realistic regime is the first two rows, where the digests are equal again at once after the resync and the commands never leave ACT's tolerance.
The injected state corruption (about 2 degrees of attitude in node B, `Loop::corrupt_b_at`) was healed at the next resync and reported as exactly one large correction (`tests/test_resync.cpp`).

## 6. In the firmware, and what is not done
**The node loop** (`firmware/app/src/main.cpp`, `CONFIG_TFC_RESYNC_PERIOD`, default 100, 0 turns it off; needs the flight function). In the last frame of each period: at 5.5 ms, after the three commands, the computer sends its state in four frames (and keeps its own copy for the vote); at vote time + 1 ms (8 ms on the target, 9 ms on the host) it collects the peers' frames from a queue of its own (filter `0x420` to `0x42F`), takes the vote over the healthy computers (the manager's view: not latched, and in the group below), adopts it, and reports the far ones to the manager. The report reaches the manager in the *next* frame's judgement, because the manager handed over its frame at 7 ms. The manager's digest persistence is set to twice the period plus 50 (TS-23). The console says `RESYNC: adopted the vote of N computers; state corrected on mask .., far from the vote on mask ..` when anything changed, and `RESYNC: nothing adopted (...)` when it could not; the status line ends with `resync=<adopted>/<skipped> far=<count>`.

**Live** (`sim/tests/test_live_resync.py`, three real firmware images on `vcan0`): a test knob (`TFC_TEST_DROP_PEERS_FIRST/_FRAMES`, never set in a flight build) makes B withhold every sensor frame (its own and its peers') from its flight function for frames 370 to 398, so that it holds its rates (with the sensor split a computer that is latched out still uses its own IMU, so withholding only the peers' frames no longer diverges it). The test shows on the bus that every computer sends its state in four frames in frame 99, 199, 299 and 399 and in no other frame; that B's state at 399 is far from the other two's, which are the same words; that the fault manager latches B out (its commands left the vote's tolerance, which is the right answer for a computer degrees off); that at the resync B takes the vote of the two healthy computers, so the three digests are equal from frame 400; and that an operator's `reintegrate B` then succeeds, which needs equal digests in the probation, and the system is Triplex again. Three runs in a row passed.

**The diverse computer** (ADR-021: node C on the golden release). Proposed and built as a switch: `CONFIG_TFC_RESYNC_GROUP` (default `0x07`, the same on every computer) says who takes part; a computer outside the group neither sends its state nor adopts the vote, and the others vote without it (two shares: the mean if they agree, nothing if not). Why it is left out and not made a witness or a learner: the diverse computer exists to show the differences between releases, and adopting a vote every second would erase any difference that builds up more slowly than a period, so a regression that drifts at 0.1 degree per second would never be seen. What it costs, measured (`Loop::resync_nodes`, period 100, 16 loss patterns, 60 s): the largest command difference between C and the other two is 0.09 degree on a clean bus (that is not loss: A and B adopt a state quantised to about 0.0035 degree of attitude at every resync and C does not, so C walks away from them at random, 0.09 degree over 60 s), 0.16 at 0.1 % loss, 0.29 at 1 % and 0.58 at 5 % (the two that are resynchronised differ from each other by 0.06, 0.12 and 0.12). The same quantisation is also why a group of two that resynchronises is exactly together while a computer outside it is not. C's *version tolerance* has to absorb that, so it must be larger than that figure with margin: 0.5 degree is enough at 1 % loss for a 60 s flight; over minutes without the accelerometer (a vehicle under thrust) the difference keeps growing as a random walk, and the rig's platform is never in that case because its accelerometer pulls the attitude back. Option for later, not built: a **slew-limited adoption** for C, which moves each word toward the vote by at most a fixed small step per period (smaller than the version tolerance divided by the period), so the random walk is bounded but a regression faster than the step still shows. It needs the version tolerance to exist first (ADR-021 is paper: nobody applies it yet).

**What is not done**
- **The target.** The burst is twelve frames (about 1.6 ms) on a bus that is 18 % busy in a normal frame; the manager's hand-over is at 7 ms and the collection at 8 ms, so the budget is there on paper; it has not been measured on the Nucleo, and the native_sim CAN driver (1 ms polling) says nothing about it.
- **The two-healthy rejoin without a command.** A restarted computer's probation compares digests; the resync now makes them equal, but the live test uses the operator's command. Whether `AutoTransient` should also use it is a policy question (ADR-010).
- **A second chance after a skipped resync** (a request frame) was not built: the table shows that a skip costs one more period of difference, and that is not what limits the flight.
- **The slow-drift rule** (TS-23): a computer whose state is changed at every resync while the others' are not.
- **Bit-for-bit agreement on the target.** Equal bits after an adoption assume that every computer evaluates `sqrt` and the float arithmetic identically (`-ffp-contract=off`; the Nucleo's FPU is IEEE). The live test on native_sim shows it on the host; the target is where it is shown for real.

## 6b. A finding that limits what the resync can do: the fault manager's command tolerance
The manager votes the commands with a tolerance of 0.01 degree (ACT's is 0.05), and latches a computer that is the odd one out in 3 of 5 frames. A lost frame changes a computer's command by more than that for a few frames, because the derivative term sees it at once. `Loop::frames_over_manager_tol` counts the frames in which two computers' commands differ by more than 0.01 degree and `runs_of_three_over_manager_tol` the times that lasted three frames in a row (a necessary condition for the 3-of-5 detector, not the detector itself), over 16 patterns of 60 s:

| Loss per frame | No resync: frames over / flights with a run of 3 | Resync 100: frames over / flights with a run of 3 | Resync 10: same |
|---|---|---|---|
| 0.01 % | 344 / 5 of 16 | 2.7 / 0 of 16 | 3.1 / 0 of 16 |
| 0.1 % | 2 689 / 16 of 16 | 28.8 / 0 of 16 | 28.0 / 0 of 16 |
| 1 % | 4 524 / 16 of 16 | 284 / 7 of 16 | 281 / 8 of 16 |

So the resync is what keeps the manager's command vote from latching healthy computers at a realistic loss rate (0.1 % and below: no run of three in 16 flights, against every flight without it). At 1 % it is not enough, at any period, because what exceeds the tolerance is the immediate effect of a lost frame, not an accumulated difference; at that loss rate the command tolerance (or the agreement of inputs, TS-16 option B) is what has to change. Recorded as an open point of TS-23.

## 7. Requirements (statuses in `../verification/REQUIREMENTS.md`)
| ID | Requirement |
|---|---|
| TFC-FDIR-044 | The replicated flight computers shall exchange their estimator and controller state every N frames (proposal 100) and each shall adopt the mid-value of the states of the healthy computers when it has all of them, and nothing otherwise, so that a computer that missed or received late a sensor frame is brought back to the same state as the others within N frames. |
| TFC-FDIR-045 | A computer whose own state is found, at a resynchronisation, further from the vote than a stated tolerance shall be reported to the fault manager as a bad frame for that computer; resynchronisation shall never hide a failing computer (docs/design/RESYNC.md section 3). |
| TFC-FDIR-046 | A state-digest mismatch shall count against a computer, or as an unresolved disagreement, only after it has lasted a configured number of frames in a row (default 1; with resynchronisation, more than N). |
