# Launch sequence: the pad, the countdown and T-zero

> Status: **the pad phase, the mission clock in SYNC, the launch and scrub commands and the go/no-go are built and tested on the host.** The firmware wiring, the simulator's release from the mission frame, the live test and the checklist are not built. The design is
> accepted by the owner (4 Oct 2026: schedule time in the flight computers, the supervisor the independent authority for T-zero and the clock of record, `MISSION_CLOCK.md`).
> Decided by the owner (4 Oct 2026): the sync master acts on the launch command; a 10 s countdown; a 10 s pad calibration; scrub only (no hold).

## 1. Why: what the audit measured

The vehicle was released at frame 0 while ACT was in Standby with a neutral gimbal, the guidance and gain tables were indexed by the frame number since SYNC began, and nothing calibrated a gyro (`SIM_FIDELITY.md` 3.1, 3.2).

| Measured departure | Without a pad | With a 15 s pad |
|---|---|---|
| Lift-off transient, 1 degree of thrust misalignment (worst error in the first 3 s) | 5.3 degrees | 1.2 degrees (the gimbal answers at once: ACT is already Nominal) |
| Lift-off transient, 3 degrees of thrust misalignment | 22 degrees | 3.6 degrees |
| Gyro bias a vehicle flies with, accelerometer correction off | 0.19 dps | at least 8 dps (the largest tried) |
| Gyro bias the rig flies with | 2.5 dps | at least 8 dps |
| Residual bias after 60 s of the estimator's own learning at 1 dps | about 0.4 dps | (not needed: a 10 s average is good to 0.003 dps) |

## 2. Why the calibration is per IMU, and before the consensus

The first try averaged the gyro of the **consensus** inside the estimator. It did not help: three real IMUs differ from one another by more than the consensus tolerance (1 dps) long before any is faulty, so with biases of
a few tenths of a degree per second the sensor consensus itself failed on the pad (it cannot tell a spread of biases from a fault) and the vehicle's bias tolerance stayed near 0.5 dps. Each computer must therefore calibrate its **own** IMU
and subtract the bias before it sends its sample, so that the consensus sees corrected values and keeps its job, detecting a real fault. This is also where STAGED_BUILD rule 11 already puts it: "the calibration of every channel is in every computer's configuration".

`ImuCalibrator` (`core/include/tfc/imu_calibration.hpp`): on the pad it keeps a running mean and variance of the gyro (Welford, floats, deterministic); from 100 samples the running mean is subtracted; it is **ready**
after 1,000 samples (10 s) if the platform really was at rest (the spread of every axis under 0.6 dps) and the bias is plausible (under 5 dps); at lift-off the bias is frozen and a real rotation can no longer move it.
A platform that is being moved on the pad is therefore **not ready** (a no-go), which is the intended check.

What the pad does *not* remove: the IMU's **mounting** error. The estimator aligns to gravity on the pad, so a mounting error is imported as a pointing error of the same size (the sweep shows the flight lost at about 2.7 degrees of
misalignment with a pad, where without one the estimator started at the true vertical by luck). A real vehicle aligns the IMU to its frame at integration; the rig does it by levelling the platform and measuring.

## 3. Where the mission clock lives (ADR-028, TS-21: **owner to confirm**)

The owner's proposal: the supervisor ("lizard brain") holds the mission time, because it is independent and cannot be impacted by the rest of the system.

**Agreed:** the supervisor should be the **independent truth** about the launch: it should decide *when* T-zero happens and it should be able to check, from its own TCXO clock, that the system's mission time is right.

**Why it should not be the *only runtime source* of mission time:** the design principles of `SUPERVISOR.md` (ADR-022) say it is fail-passive: unpowered, in reset or hung, it must leave every node running (TFC-SUP-007), it never transmits on the bus, and "the supervisor as the time master" was considered and left
out because it makes the supervisor a single point of failure for time. If the guidance schedules depended on the supervisor, a supervisor that resets or hangs 60 s into the ascent would take away the schedule of every computer at once. Independence cuts both ways: what must not depend on the rest of the system must not be depended on by it either.
The Lite supervisor also has no bus tap and cannot send a countdown number; it has discrete lines.

**Recommended: a hybrid, which keeps the supervisor as the authority and the flight computers as the clock:**
1. **The supervisor decides and signals T-zero**, with a discrete **`T0` line** to each node (like a launch vehicle's liftoff discrete). Until it exists (the parts are not ordered), the authenticated `launch` command from the PC plays that role through the same interface.
2. **The sync master latches the frame of the `T0` edge and distributes mission time in SYNC** (two currently unused bytes: frames since T-zero offset by the countdown; 0xFFFF = not launched). Every follower takes it from SYNC exactly as it takes the frame number, counts through a gap, and the takeover master continues it. All computers therefore agree on T by construction (without it, nodes that saw the command one frame apart would index their schedules one frame apart, and the replicas would diverge: TS-16).
3. **The supervisor keeps its own mission clock from the `T0` edge** and compares it, through the master's `FRAME` pulses, with the system's: a drift or a jump is reported (and is the supervisor's evidence of a SYNC fault). It never overrides the computers' mission time, in line with "never takes a voting decision".
4. After T-zero the system **does not need** the supervisor to keep flying.

**Two clocks.** What is decided here is *schedule time* (frames since T-zero, minutes). The long-lived *clock of record* (mission elapsed time over hours to years, on the supervisor's own oscillator, correlated against UTC) is `docs/MISSION_CLOCK.md` (ADR-029); the two are linked at T-zero: the supervisor's epoch is the `T0` edge.

Alternatives (TS-21): the supervisor as the continuous time source (rejected above); the command carrying a 32-bit T-zero frame (the ground frame has no room); an ACT broadcast (ACT has no spare bytes in `0x300`); each computer counting from its own sight of the command (T-zero differs by a frame between computers).

## 4. Phases and states

| Phase | What happens | Leaves by |
|---|---|---|
| **Power-up** | Nodes boot, SYNC is claimed, sensors flow. Mission time = not launched | The pad (automatically, when SYNC is up) |
| **Pad** (P2) | The vehicle is clamped, at rest. Each computer calibrates its IMU, the estimators align to gravity, ACT goes Standby to Nominal (neutral output), the schedules sit at their first point | `launch` accepted |
| **Countdown** | Mission time counts from T-10 s. Anything that is no longer ready scrubs | T-zero, or **scrub** |
| **Flight** (P3) | The clamps open at T-zero; the schedules follow flight time; the calibration is frozen; in vehicle-sensor mode the accelerometer correction is off | the end of the run; Safe (abort) |
| **Scrub** | Allowed only before T-zero: back to the pad, the calibration starts again | the pad |

## 5. Go/no-go (the launch command is accepted only if all hold)
Each computer reports `ready` in its heartbeat (a bit that is free): its IMU calibration is ready (10 s of rest, plausible bias), its consensus is trustworthy, its attitude is valid, no Safe request. And: three healthy computers (Triplex), ACT Nominal,
no digest disagreement. The command is an authenticated ARM then EXECUTE (ADR-019), so two deliberate steps. During the countdown the same list is watched; a failure scrubs.

## 6. The interfaces (to build)
| Where | Change |
|---|---|
| `SYNC` (`0x010`) | The two reserved bytes carry the mission frame (offset), 0xFFFF = not launched. Python mirror and golden bytes with it |
| Ground commands | `launch` (needs an ARM) and `scrub` (plain), authenticated, counter-checked; applied by the sync master only |
| Heartbeat | Bit 7 of byte 1: `ready` |
| `SyncClock` | Carries the mission frame, counts through a gap, continues on takeover |
| Firmware | The calibrator between the IMU read and the gyro frame; the flight function given the pad flag and the flight frame |
| Simulator | `tfc_simd` clamps the vehicle on the pad and releases it when the mission frame passes T-zero; its time becomes mission time |
| Supervisor (later) | The `T0` line out, the mission clock check |
| The checklist | `docs/procedures/` (the human steps) and `tfc_peers launch` (polls readiness, runs the countdown, scrubs on a no-go) |

## 7. What is built (host, tested)
- **Pad phase (PR 38):** `ImuCalibrator`; `FlightFunction::set_mission` (schedules by flight frame, held at the first point on the pad) and `sensors_ok`; the runner's clamp, release and flight frame; the closed loop with `pad_frames`; `tfc_sens --pad`.
- **Mission time on the bus:** the mission frame in SYNC (C++ and Python, goldens pinned in both); `SyncClock` carries it (follows SYNC on the pad and in the countdown, counts and verifies in flight, a late joiner adopts it, continuous through a takeover, saturating); `launch()` and `scrub()` on the sync master only.
- **Launch commands and the gate:** ground operations `launch` (always needs an ARM) and `scrub`, reported to the firmware as events; `launch_check` (the go/no-go list); the heartbeat's `ready` bit (C++ and Python).
- Tests: `tests/test_pad.cpp`, `tests/test_launch.cpp`, the mission tests in `tests/test_sync_clock.cpp`; 34 mutants of the new code, all killed.

## 8. Still to do
The firmware: the calibrator between the IMU read and the gyro frame, the flight function given the pad flag and the flight frame, the master acting on the launch event (gate, `launch()`), the countdown, the ready bit set; `tfc_simd` clamping and releasing from the mission frame; the live test (a 10 s pad, a launch, a scrub, a takeover during the countdown); `tfc_peers launch` and the human checklist; the supervisor's `T0` line and mission clock check (waits for the supervisor).
