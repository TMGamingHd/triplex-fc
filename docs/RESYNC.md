# State resynchronisation (TS-16 option C): keeping the replicas together on a lossy bus

> Status: **built on the host** (core, closed loop, manager hooks; ADR-030). **Not in the firmware yet**, so nothing here has run on the bus or the rig. The tolerances and the
> period are proposals measured on the simulator, not on the real CAN bus.

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

## 6. What is not done
- **The firmware.** The node loop must send the four frames after the command slot in the last frame of each period, collect the others', vote, adopt, and call `report_state_correction`; the manager's hand-over time (7 ms into the frame) moves on that frame, and the bus load of the burst (twelve 8-byte frames, about 1.6 ms) has to be checked against the frame budget on the rig. The native_sim triplex test then shows it live (`TFC_RESYNC_PERIOD` proposed).
- **Two healthy computers.** The vote averages two shares that agree within tolerance, so a latched or restarted computer can be brought back from the two that remain; the manager's probation of a restarted computer does not yet use it (the digest includes the update count, and a restarted computer's count differs from the first frame until it is resynchronised: this is the first thing to try on the rig).
- **The resync's own loss.** A computer that missed a chunk adopts nothing that period. The table shows it, and a second chance in the next frame (a request frame) would shorten the unhealed time; it costs a new frame type and was not judged worth it before measuring the real loss on the bus.
- **Replicas of different software** (the diverse node of ADR-021) cannot share this state: their state is not the same variables. They stay out of the resync and are compared by their commands only.
- **The state's bit-for-bit agreement on the target.** Equal bits after an adoption assume that every computer evaluates `sqrt` and the float arithmetic identically (the build uses `-ffp-contract=off`, and the Nucleo's FPU is IEEE); the live test on the target is where this is shown.

## 7. Requirements (proposed)
| ID | Requirement |
|---|---|
| TFC-FDIR-044 | The replicated flight computers shall exchange their estimator and controller state every N frames (proposal 100) and each shall adopt the mid-value of the states of the healthy computers when it has all of them, and nothing otherwise, so that a computer that missed or received late a sensor frame is brought back to the same state as the others within N frames. |
| TFC-FDIR-045 | A computer whose own state is found, at a resynchronisation, further from the vote than a stated tolerance shall be reported to the fault manager as a bad frame for that computer; resynchronisation shall never hide a failing computer (docs/RESYNC.md section 3). |
| TFC-FDIR-046 | A state-digest mismatch shall count against a computer, or as an unresolved disagreement, only after it has lasted a configured number of frames in a row (default 1; with resynchronisation, more than N). |
