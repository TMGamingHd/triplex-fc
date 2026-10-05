# Actuator node logic

> Status: **built and tested on the host** (`core/include/tfc/act.hpp`, `act_ground.hpp`, `tests/test_act.cpp`, `tests/test_act_ground.cpp`) **and running as an application**
> (`firmware/act`, Zephyr; native_sim on `vcan0` and the Nucleo; `sim/tests/test_live_triplex.py`). Per ADR-024, ACT has **no servo output**: its output is the voted gimbal command on the bus (`0x300`),
> and its Safe action is the value of that command.

## What it does, each frame
1. **Votes** the three flight computers' commands (frames `0x200+n`), pitch and yaw separately, with `vote3` (mid-value select, tolerance 0.05 degree). A node counts only if
   its frame arrived this frame with a good CRC and ACT has not excluded it. Three agreeing: that value. Two agreeing: their mean. One alone: its value. Two that disagree,
   three with no majority, or none: **not a trustworthy command**, and the last output is held.
2. **Judges the nodes itself**, not trusting their own verdicts: a missing frame, or a command that is the odd one out of a three-way vote, is a bad frame; **3 bad of the last 5**
   excludes the node (`ChannelMonitor`). Two nodes that cannot be told apart blame nobody. Excluded nodes are readmitted only by the operator (`clear_exclusions()`).
3. **Latches the output** with a bound: in Standby and Nominal it moves no more than `normal_slew` (1 degree) per frame toward the voted command, whatever the vote says.

## Modes
| Mode | Output | Enters | Leaves |
|---|---|---|---|
| **Standby** (the pad) | neutral | a power-on, or a stored record that fails its check | 100 trustworthy votes in a row go to Nominal; a Safe request or the SAFE line goes to Safe |
| **Nominal** | follows the vote, slew-limited; a frame without a trustworthy vote **holds** | Standby, or an accepted `clear_safe()` | 3 such frames in a row, the flight computers' Safe request, or the SAFE line: **Safe** |
| **Safe** | **freeze at once** (no step), hold 50 frames (0.5 s), then ramp to neutral at 0.02 degree a frame (4 s from 8 degrees), then hold neutral | the three causes above, or a reset | **only** `clear_safe()` |

`clear_safe()` is the operator's authenticated, ARMed command (the caller checks the ground-command frame; ACT does not) and is **accepted only if** the votes have been trustworthy for 100 frames
**since Safe was entered**, from at least two nodes, with no Safe request or SAFE line active. Otherwise it is refused and counted. Nothing leaves Safe by itself, however good the votes
(a test runs 1000 good frames). The flight computers' Safe request and the SAFE line are inputs (`safe_request()`, `hardware_safe()`); the first comes from the heartbeat, the second from the supervisor
or the operator (`SUPERVISOR.md`).

## After a reset (TFC-RESP-003)
After a reset that is not a power-on, ACT **starts in Safe** (cause `Reset`), holding the output it had stored before the reset (`ActRecord`: the last output and mode, with a CRC, kept in memory that
survives a reset), so the command does not step across the reset; then the hold, then the ramp. With `resume_from_stored = false` it starts from neutral. A power-on, or a record that fails its check,
starts in Standby. A corrupted mode value is treated as Safe.

## What the tests show (`tests/test_act.cpp`, 19 tests)
The pad stays neutral; a wrong node is out-voted and excluded after persistent disagreement and readmitted by the operator; missing frames count; Duplex agreement uses the mean and a miscompare holds and blames nobody; one or two lost votes
are held through and the third enters Safe; the output never moves faster than the slew bound; Safe freezes with no step, holds, ramps without exceeding the ramp rate, waits for both planes, and stays; the SAFE line has priority; `clear_safe()` is refused
for each missing condition and accepted when all hold; the reset behaviour; the stored record detects damage in every field; and **a 20,000-frame deterministic fuzz** (random commands, drops, wild nodes, Safe requests) shows the output finite,
never stepping by more than the bound, and never leaving Safe without a clear. Coverage of `act.hpp`: 100% of lines, 99.1% of branches.

## The application (`firmware/act`)
It follows SYNC and never becomes the master (`SyncStart::Observer`); it is silent until the first SYNC. At 6.5 ms (7.5 ms on the host, whose CAN driver polls every millisecond) it drains the
commands, the heartbeats and any ground frames, votes, and sends `0x300` (`to_act_frame`). The flight computers' Safe request comes from their heartbeats through `HeartbeatMonitor`: **a majority
of the fresh heartbeats of the nodes ACT has not excluded must ask** (so one faulty computer cannot safe the vehicle; two computers that disagree and cannot blame each other both ask; a lone survivor decides).
Ground commands go through `ActGround`: the same tag, counter window and ARM-then-EXECUTE as the flight computers (ADR-019); `clear-safe` needs an ARM and lifts Safe only if `clear_safe()` accepts
(votes good for 100 frames since Safe was entered, from two nodes, no request active); `reintegrate` readmits the nodes ACT excluded, whatever its node field says. The output survives a reset in
no-init RAM (`ActRecord`). Live, with three flight-computer instances: Standby goes to Nominal after 100 good frames (frame 103), a computer that dies is excluded at the same frame the flight computers latch it,
ACT keeps flying on two and then on one, and with none it enters Safe on lost votes.

## Not yet
The supervisor's `KICK`, `FRAME` and `SAFE` lines in the app (`hardware_safe()` is wired to nothing); the hardware test of a real ACT reset (F17); an oracle S5 for the campaign (the output step bound) once the
campaign drives ACT; a live test of the heartbeat-driven Safe request and of `clear-safe` (needs a fault that makes the firmware's flight computers request Safe: fault injection in the firmware or the Pico, P1-5 and later;
the logic is covered on the host).
