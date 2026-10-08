# Simulation and virtual peers

| Part | Status |
|---|---|
| `tfc_peers/`: **virtual peers**, fake flight computers that put real, fault-injectable traffic on the flight bus | Done |
| `../tools/replay`: `tfc_replay`, runs recorded traffic through the real `core/` redundancy code | Done |
| Vehicle simulator (6-DOF ascent, platform model, SocketCAN gateway) | **Done**: `../sim/vehicle` (the C++ model, the runner, the closed loop), `../tools/sim/tfc_simd` (on SocketCAN, with `--pico`), `../tools/sim/tfc_sens` (sensitivity), `../tools/sim/tfc_mc` (a Monte Carlo of the closed loop: `../docs/design/MONTE_CARLO.md`), `../tools/sim/tfc_fly` (fly any vehicle described in a file: `../vehicles/`, `../docs/design/VEHICLE_SPEC.md`). See `../docs/design/VEHICLE_SIM.md` and `../docs/design/SIM_FIDELITY.md` |
| Pico client (`tfc_peers pico`, `pico_link.py`) | Done, not run on a board (`../docs/design/PICO.md`) |
| Fault-campaign runner (`campaign/`) | Done |

Contents: [What the virtual peers are](#what-the-virtual-peers-are) ·
[Which command do I want?](#which-command-do-i-want) ·
[Quick start](#quick-start) ·
[Command reference](#command-reference) ·
[Fault reference](#fault-reference) ·
[What the peers send](#what-the-peers-send) ·
[How the decisions are made](#how-the-decisions-are-made-and-the-replay-output) ·
[Live CAN in depth](#live-can-in-depth) ·
[Offline vs live](#offline-vs-live) ·
[Logs](#logs) ·
[Tests](#tests) ·
[Limits](#limits-and-what-is-not-covered)

## What the virtual peers are
The real system is three flight computers that vote over a CAN bus. Until the boards arrive, and later for
failures that are dangerous or awkward to cause for real, the PC plays the other flight computers.
`tfc_peers` generates exactly the frames a real FC-B or FC-C would send every 10 ms (gyro, accel, command;
real CAN IDs, CRC-8, sequence numbers, sensor noise) and can break any of them on purpose with 32 kinds of
fault, and can send the flight computers operator commands (reintegrate a node, disable it, clear a Safe request). Everything is standard-library Python 3.10+; nothing to install.

There are two ways to use the traffic, and they differ in **who judges it**:

| | **Offline** (`record` then `tfc_replay`) | **Live** (`run` / `listen` on a CAN interface, with FC-A) |
|---|---|---|
| What happens | Python writes a log file in virtual time; the C++ replay tool feeds it to `tfc::RedundancyManager` | Python sends frames onto `vcan0` in real time; the real FC-A firmware (Zephyr `native_sim`) judges them as they arrive |
| Judge | `tfc_replay` (host process, same core class as the firmware) | FC-A firmware on its own 100 Hz clock, as it will run on the Nucleo |
| Time | virtual, instant, byte-for-byte repeatable | real, 10 ms per frame |
| Needs | `build/host/tfc_replay` | `vcan0` (needs `sudo` once per boot) and the built `native_sim` binary |
| Best for | sweeps, regression tests, CI, "what exactly does `core/` decide?" | seeing the real loop run, SYNC and timing, boot order, the bus view, and later the real hardware |

Details of the difference are in [Offline vs live](#offline-vs-live).

## Which command do I want?
| I want to... | Run |
|---|---|
| See which faults exist and their options | `python3 -m tfc_peers faults` |
| Move the platform, cut a node's power, or read the Pico's status (USB serial) | `python3 -m tfc_peers pico status`, `pico platform 10 -5 --for 3`, `pico cut B 3000`, `pico restore all` |
| Generate traffic into a file | `python3 -m tfc_peers record ... --out FILE` |
| Read a log file in plain English | `python3 -m tfc_peers decode FILE` |
| Ask "what would a flight computer decide about this log?" | `../build/host/tfc_replay FILE --verbose` |
| Assert an outcome (pass/fail, for tests) | `tfc_replay FILE --expect-latch B:102 --expect-mode duplex` |
| Watch a CAN bus in decoded form | `python3 -m tfc_peers listen --iface vcan0` |
| Run fake peers next to the **real FC-A firmware** | `python3 -m tfc_peers run --follow-sync --nodes B,C ...` |
| Run fake peers with **no** flight computer (to test a monitor or other receiver) | `python3 -m tfc_peers run --nodes B,C ...` (free-running) |
| Script an operator command in a scenario | `--command 450:reintegrate:B` (on `record` or `run`) |
| Send one operator command to a running flight computer | `python3 -m tfc_peers command reintegrate B` |
| Create the virtual CAN interface | `./scripts/setup_vcan.sh` |

## Quick start
Run from `sim/` (the commands below assume that). Build the replay tool once from the repo root:
`cmake -S . -B build/host -G Ninja -DTFC_SANITIZE=ON && cmake --build build/host`.

**1. Offline: one fault, and what the flight computer decides**
```bash
python3 -m tfc_peers record --nodes A,B,C --frames 400 --fault B:bias:start=100,mag=3 --out /tmp/t.log
../build/host/tfc_replay /tmp/t.log --verbose
```
```
frame 102: node B latched because: vote disagreement
frames=400
latch.A=-
latch.B=102
latch.C=-
healthy=2
mode=duplex
crc_bad=0
...                      (the other counters follow, all 0 here)
```

**2. Offline: the same thing as a pass/fail check**
```bash
../build/host/tfc_replay /tmp/t.log --expect-latch B:102 --expect-no-latch A --expect-no-latch C --expect-mode duplex; echo $?
```
`0` = all expectations held, `1` = at least one failed (the failed ones are printed).

**3. Live: the real FC-A firmware against fake peers** (two terminals; one-time `./scripts/setup_vcan.sh` and
`west build -b native_sim/native/64 firmware/app -d build/native_sim` from the repo root first)
```bash
# terminal 1, repo root: the flight computer (real time; Ctrl+C or -stop_at=<s> to end)
build/native_sim/zephyr/zephyr.exe -stop_at=12
# terminal 2, within a second or two: fake FC-B and FC-C that follow FC-A's SYNC; B gets a bias at frame 400.
# --frames 0 = keep sending until Ctrl+C, so the peers outlive FC-A's 12 s (see "why C latches" below)
cd sim && python3 -m tfc_peers run --follow-sync --nodes B,C --frames 0 --fault B:bias:start=400,mag=3
```
FC-A's console shows `node B joined the bus`, then `node B LATCHED OUT: vote disagreement` at frame 402.
Add a third terminal to watch every frame on the bus (from **any** directory, with the launcher; see
[Running from any directory](#running-from-any-directory)): `~/SpaceX/triplex-fc/sim/tfc-peers listen`.

**Why does C latch out "frame missing" after B?** Only if the peers stopped. With a fixed `--frames N` the peers send
for N frames and stop, and a flight computer that is still running then (correctly) reports both peers missing; the
one you faulted is already out, so you only see the other one. `--frames 0`, or any N larger than FC-A's run
(`-stop_at=12` is 1200 frames, and the peers join around frame 40-60), avoids it.

## Command reference

### `python3 -m tfc_peers record`: generate traffic into a log (offline)
Virtual time, no sleeping: 400 frames take milliseconds. Output is identical on every run for the same inputs.

| Flag | Default | Meaning |
|---|---|---|
| `--out FILE` | (required) | Log file to write, `candump -L` format. Overwritten if it exists. |
| `--nodes LIST` | `B,C` | Which flight computers to simulate, comma separated from `A,B,C` (or `0,1,2`; case and spaces ignored). Use **`A,B,C`** for a fully virtual triplex: `tfc_replay` needs all three present, otherwise it correctly treats the missing one as dead. |
| `--fault SPEC` | none | `NODE:KIND[:key=value,...]`. **Repeatable**; any number of faults, on any nodes, overlapping or not. A fault must target a node listed in `--nodes`. See [Fault reference](#fault-reference). |
| `--frames N` | `1000` | Number of 10 ms major frames. 100 = 1 s, 400 = 4 s. |
| `--command SPEC` | none | Scripted operator command `FRAME:OP[:NODE]`, **repeatable**: `OP` is `reintegrate`, `disable`, `clear-disabled`, `warm` (these need a node), `phase` (its NODE is a phase: `0`..`7`, `p3`, or a name such as `ascent`) or `clear-safe`, `noop` (no node), optionally prefixed `arm-` (only the ARM frame), `armed-` (the whole two-step: ARM in that frame, EXECUTE two frames later) or `forged-` (a wrong tag: refused without a trace); `FRAME:replay` re-sends the previous command frame unchanged (a replay). Sent as an authenticated ground-command frame in that frame number, 6.5 ms in, so the flight computer applies it in that very frame; counters are assigned 1, 2, 3... in time order. `clear-disabled`, `clear-safe` and a `disable` that would leave fewer than two healthy nodes need the `armed-` form. See [Operator commands](#operator-commands-authentication-arm-and-the-interlock) and [Recovery](#recovery-latch-probation-readmission-and-disabling). |
| `--seed N` | `1` | Seed for sensor noise and for the random faults (`spike`, `corrupt`, `babble`). Same seed = byte-identical log. Change it to see how a result varies. |
| `--iface NAME` | `vcan0` | Only the interface name written into each log line; has no other effect. |

`record` never emits SYNC (the peers are not the sync master). Frame numbers start at 0.

### `python3 -m tfc_peers log`: record the live bus into a log
| Flag | Default | Meaning |
|---|---|---|
| `--iface NAME` | `vcan0` | Interface to record (`can0` for the USB-CAN adapter). |
| `--out FILE` | (required) | Log to write, `candump -L` format, timestamps counted from 0. Overwritten if it exists. |
| `--duration S` | until Ctrl+C | Stop after S seconds. |

The same file that `record` writes, so `decode` prints it and `tfc_replay` judges it: the way to ask, offline and repeatably, what a flight computer
*would* have decided about what really happened on the bus. `tools/bench/hil_run.sh` does this in one command.

### `python3 -m tfc_peers decode LOG`: read a log in plain English
| Argument | Meaning |
|---|---|
| `LOG` | A `candump -L` format file, e.g. one written by `record`. |
| `--limit N` | Print only the first N frames. |

Each line: time, CAN ID in hex, then name, node, sequence number, decoded values, units; `CRC-BAD` for damaged
frames and `out-of-schedule` for IDs that are not part of the flight-bus schedule. Pipe it through `grep`/`awk`/`less`.

### `python3 -m tfc_peers run`: send traffic on a CAN interface (live)
Real time, 100 Hz. Prints its own send-lateness statistics when finished.

| Flag | Default | Meaning |
|---|---|---|
| `--iface NAME` | `vcan0` | SocketCAN interface to send on. `can0` for the USB-CAN adapter (once it has arrived and is configured). |
| `--follow-sync` | off | Phase-lock to the flight computer's SYNC frames and use SYNC's frame number as the frame number. **Use this whenever a real FC-A is on the bus.** Without it the peers free-run on their own clock. `--frames` then counts SYNC frames. Exits with an error if no SYNC arrives for 2 s. |
| `--frames N` | `1000` | As for `record`, but **`0` means keep going until Ctrl+C**. With `--follow-sync` it counts SYNC frames. A fixed N ends the peers' traffic after N frames; see "Why does C latch" above. |
| `--nodes`, `--fault`, `--seed`, `--command` | as for `record` | With a real FC-A on the bus use `--nodes B,C`. Fault `start`/`end` frame numbers are then **FC-A's frame numbers** (SYNC's), not "seconds since the peers started". |

Ctrl+C stops it cleanly and prints the statistics (exit status 0 with `--frames 0`, 130 with a fixed count). When a finite run ends it prints a reminder that a running flight computer will now report the peers missing.

### `python3 -m tfc_peers listen`: bus monitor (live)
| Flag | Default | Meaning |
|---|---|---|
| `--iface NAME` | `vcan0` | Interface to monitor. |
| `--duration S` | until Ctrl+C | Stop after S seconds. |

Prints every frame seen, decoded, with the time since `listen` started. It is a pure observer: it does not judge
anything and what it prints is not a replayable log.

### `python3 -m tfc_peers command OP [NODE]`: send one operator command now (live)
| Argument | Default | Meaning |
|---|---|---|
| `OP` | (required) | `reintegrate`, `disable`, `clear-disabled`, `clear-safe`, `launch`, `scrub`, `warm`, `phase` or `noop` |
| `NODE` | none | `A`, `B` or `C` (for `phase`: the phase, `0`..`7` or its name); not needed for `clear-safe`, `launch`, `scrub` and `noop` |
| `--iface NAME` | `vcan0` | SocketCAN interface to send on |
| `--arm` | off | Send the ARM frame, then the EXECUTE frame 50 ms later. Needed for `clear-disabled`, `clear-safe` and a `disable` that would leave fewer than two healthy nodes |
| `--counter N` | next after the last this tool sent | Command counter of the first frame. The flight computer accepts only counters that are 1 to 32 ahead of the last one it accepted, so the tool remembers its last counter in `~/.cache/tfc_peers/ground_counter` (override the directory with `$XDG_CACHE_HOME`) |

Sends an authenticated ground-command frame (`0x510`; tag key from `$TFC_GROUND_KEY`, 32 hex digits, default the public bench key) to
whichever flight computers are on the bus and exits. It does not wait for an answer: the flight computer prints the outcome on its
console (`GROUND COMMAND reintegrate B: accepted`, or `refused: <why>`; a frame with a wrong tag or a stale counter is dropped without
a trace and only counted). If the flight computer was restarted while this tool kept running, the counters still increase and are
accepted (the first command after a restart is accepted whatever its counter); if you delete the counter file while the flight
computer keeps running, restart it or pass `--counter` above its last one. It is the interactive twin of `--command`; use
`--command` when the command must land in an exact frame.

### `python3 -m tfc_peers launch`: the launch checklist, automated
Watches the bus (the flight computers' heartbeats, ACT's output, SYNC's mission frame), shows the go/no-go once a second, and when every item holds sends the launch (an authenticated ARM, then the EXECUTE) and runs the countdown. It exits **0 at T-zero**, 1 on a no-go or `--check`, 2 on a scrub, 3 if the command was not accepted, 4 if the countdown never completed. It asks you to type `LAUNCH` first unless `--yes`. The human checklist that goes with it is `docs/procedures/P-S2-02-launch-checklist.md`; the design is `docs/design/LAUNCH_SEQUENCE.md`.

| Flag | Default | Meaning |
|---|---|---|
| `--iface IFACE` | `vcan0` | SocketCAN interface. |
| `--wait SECONDS` | `60` | How long to wait for a go before giving up (a no-go, exit 1, with the reasons). |
| `--check` | off | Only report the go/no-go once and exit (0 go, 1 no-go); sends nothing. |
| `--yes` | off | Do not ask for the typed confirmation. |
| `--counter N` | the next one | Command counter of the ARM frame, as for `command`. |

A scrub is `python3 -m tfc_peers command scrub` (before T-zero only); `command launch --arm` sends the launch by hand.

### `python3 -m tfc_peers pico OP [OPERANDS]`: the Pico (platform driver and fault injector) over USB serial
Talks to the board of `docs/design/PICO.md`; needs `pyserial` (`pip install pyserial`) and a Pico running `firmware/pico`. Each call sends one command and prints the board's status.

| Operation | Operands | Meaning |
|---|---|---|
| `status` | none | Ask for the status: the platform's output tilts, the command age, the flags (saturated, holding, levelling, link lost, watchdog reset, rejected command) and which nodes are cut. |
| `platform` | `X_DEG Y_DEG` | Command the platform's tilts about X and Y. The board limits the travel (45 degrees) and the rate (300 degrees per second) whatever is sent, holds after 100 ms without a command and levels after 1 s, so one command alone does not stay: add `--for SECONDS` to keep sending at 100 Hz. |
| `cut` | `NODE MS` | Cut the power of node `A`, `B`, `C` or `ACT` for `MS` milliseconds (at most 30,000). It ends by itself, so a crash of this tool cannot leave a node cut. |
| `restore` | `NODE` or `all` | Power the node (or every node) at once. |

Flags: `--port PORT` (default `/dev/ttyACM0`), `--for SECONDS` (default 0).

### `python3 -m tfc_peers faults`
Lists every fault kind with its fault-matrix row and options. Same information as the next section.

### `tfc_replay LOG ...` (C++, `build/host/tfc_replay`): what would a flight computer decide?
Feeds each 10 ms frame of the log to `tfc::RedundancyManager`, the same class the firmware runs, and reports.

| Flag | Meaning |
|---|---|
| `LOG` | `candump -L` log. Frames are grouped into 10 ms frames by timestamp, so logs from `record` (which start at t=0) work as-is. |
| `--verbose` | Print one line per latch event (`frame N: node X latched because: <reasons in words>`), plus `SAFE REQUEST raised`, `BUS ALARM raised/cleared`, ground commands and their outcomes, and probation / reintegration / disabling events. |
| `--t0 SECONDS` | Subtract this from every timestamp before grouping (for logs that do not start at 0). May be negative, so that frame 0 starts before the first line of a log that begins mid-run. |
| `--startup-grace N` | Frames during which a peer that has never been seen is not judged (default 0, as in a `record` log, where every node is present from frame 0). A live log needs it: the peers join some frames after the flight computer starts (FC-A uses 500). |
| `--first-frame N` | The frame number of the first 10 ms frame, for a log of a **live** bus that starts at a SYNC frame number other than 0 (the frames carry SYNC's number, ADR-018, and the phase check compares against it). Without it frames are numbered 0, 1, 2... as in a `record` log. `tools/bench/log_t0.py` prints the `--t0` and `--first-frame` that line a live log up. |
| `--dump FILE` | Write one CSV row per 10 ms frame: mode, healthy count, masks (valid/latched/probation/disabled/held), Safe and bus-alarm flags, the nodes newly latched and newly on probation, the integrity mask, per-node reason bits, and all 8 voted outputs. Used by the fault campaign (`docs/verification/FAULT_CAMPAIGN.md`) to check safety properties, e.g. that the output never follows a faulty node. |
| `--vote-us MICROSECONDS` | Where in the 10 ms frame the flight computer votes (default 7000, as FC-A). A frame received after it belongs to the *next* frame, so a late frame is judged stale. |
| `--release A=ID,B=ID,C=ID` | The release each computer reports in its heartbeat (ADR-021; a live bus carries it in the heartbeat, a recorded log does not). When two computers share a release and the third does not, the lone computer's commands are compared with the pair's within the version tolerance (1.5 times the vote tolerance), its digest is not compared with theirs, and a disagreement beyond the tolerance isolates nobody: the outputs are held and Safe is requested. The campaign group `common_mode` runs with it. |
| `--phases` | Keep the mission phase and the WARM role (ADR-023, `docs/design/MISSION_PHASES.md`): the manager starts in power-up, `phase` changes it (refused if fewer computers vote than the new phase's minimum), `warm` rests a computer, and the `disable` tiers follow the phase (plain above the nominal, an ARM below it, refused below the minimum). Below the minimum is reported, never acted on. The summary then prints `phase`, `warm` and `below_minimum_frames`. |
| `--sensor-split` | Judge each IMU's gyro and accelerometer pair apart from its computer (ADR-020 case 1, TS-15): a bad IMU is excluded from the sensor consensus and its computer stays in the command vote. The summary then also prints `slatch.X`, `sstate.X` and `sstrikes.X` per IMU channel, and the `--dump` CSV carries `s_*` columns (valid, latched, probation, disabled, healthy, mode, newly latched, reason per channel). Operator commands address an IMU channel by node 4 to 6. The campaign groups `split_sensor`, `split_command`, `split_pairs` and `split_all` run with it; `python3 -m campaign.ts15` is the TS-15 comparison with and without. |
| `--policy manual\|auto` | Reintegration policy (default `manual`: only an operator command readmits a node). `auto` also readmits, after the dwell, a node whose first latch looked transient. See [Recovery](#recovery-latch-probation-readmission-and-disabling). |
| `--expect-latch NODE:FRAME` or `NODE:MIN-MAX` | Node (A/B/C) must have latched at exactly that frame, or within the range (inclusive). Repeatable. |
| `--expect-no-latch NODE` | Node must never have latched. Repeatable. |
| `--expect-state NODE:STATE` | Final state of a node must be `healthy`, `latched`, `probation` or `disabled`. Repeatable. |
| `--expect-mode MODE` | Final mode must be `triplex`, `duplex`, `simplex` or `safe`. |
| `--expect-min STAT:N` | A counter must be at least N. `STAT` is one of `crc_bad`, `seq_bad`, `missing`, `out_of_schedule`, `stuck_flags`, `digest_flags`, `vote_disagreements`. Repeatable. |

Exit status: `0` ran and every expectation held, `1` an expectation failed, `2` usage or input error.
Output format is described in [the replay output](#how-the-decisions-are-made-and-the-replay-output).

### `./scripts/setup_vcan.sh [IFACE]`
Loads the `vcan` kernel module and creates and brings up a virtual CAN interface (default `vcan0`). Needs `sudo`.
Safe to re-run. It does not survive a reboot. Remove it with `sudo ip link del vcan0`.
`ip -brief link show vcan0` should show `UNKNOWN <NOARP,UP,LOWER_UP>`; `UNKNOWN` is normal for a virtual interface.

### Running from any directory
`python3 -m tfc_peers ...` (underscore) only works from inside `sim/` (that is where the package lives); from anywhere
else it says `No module named tfc_peers`. The launcher `sim/tfc-peers` (hyphen, a script: **no** `python3 -m`) takes the
same subcommands and flags and works from any directory: `~/SpaceX/triplex-fc/sim/tfc-peers listen`, `sim/tfc-peers run ...` from the repo root. (Add `sim/` to your
`PATH`, or `alias tfc-peers=~/SpaceX/triplex-fc/sim/tfc-peers`, to type just `tfc-peers listen`.)

### Exit status of `tfc_peers`
`0` success (including Ctrl+C of a `run --frames 0`), `2` error (bad argument or fault spec, unknown node, interface or file not found, no SYNC), `130` Ctrl+C of a `run` with a fixed `--frames`.

## Fault reference
Syntax: `NODE:KIND[:key=value,key=value,...]`, for example `B:bias:start=100,mag=3`. `NODE` is `A`, `B` or `C`.
Every fault also accepts `start=N` (first frame it is active, default 0) and `end=N` (first frame it is *not*
active, default never), and the intermittent pair `period=N,duty=K`: active only `K` frames out of every `N`, counted
from `start` (so `B:corrupt:start=100,period=3,duty=1` damages one frame in three). Frames are 10 ms; `start=100` is 1.0 s in. Faults only change what the node *sends*.

| Kind | Matrix row | What it simulates | Options (default) | What it changes on the wire |
|---|---|---|---|---|
| `dropout` | F01 | A node that dies: fail-silent | none | Node sends no frames at all |
| `stuck` | F03 | A frozen sensor | none | Gyro and accel values freeze at the node's last good sample (frames still valid, sequence still counting) |
| `bias` | F04 | A constant sensor offset | `sensor`=`gyro`\|`accel` (gyro), `axis`=0\|1\|2 (0), `mag` (3.0; dps for gyro, g for accel) | Adds `mag` to that axis |
| `drift` | F05 | A slowly growing offset | `sensor` (gyro), `axis` (0), `rate` (0.05; per frame, dps or g) | Adds `rate` x (frames since start + 1) to that axis |
| `spike` | F06 | Random single-frame outliers | `sensor` (gyro), `mag` (20.0), `p` (0.05; probability per frame) | With probability `p`, adds +-`mag` to one random axis for that frame |
| `saturate` | F07 | Sensor pinned to full scale | none | Gyro and accel at the 16-bit limits (+1023.97/-1024 dps, +-16 g), sign alternating each frame |
| `corrupt` | F08 | A flaky link | `p` (0.2; probability per frame, per frame type) | Flips one random bit of a gyro/accel/command frame; its CRC then fails |
| `cmd_offset` | F09 | A wrong-but-valid command (software bug) | `mag` (1.0; degrees) | Adds `mag` to the pitch command; everything else, including the CRC, stays valid |
| `digest` | F10 | Silent internal state divergence | `xor` (1; 16-bit mask) | XORs the estimator-state digest in the command frame; pitch and yaw unchanged |
| `babble` | F11 | A node flooding the bus | `n` (5; extra frames per 10 ms), `id` (32 = 0x020) | Adds `n` extra frames per cycle on the 16 out-of-schedule IDs from `id` up (default `0x020`-`0x02F`, higher priority than every sensor ID). `id` may be written `0x520`; ids that overlap the flight-bus schedule are rejected. `n` of 3 or more raises the bus alarm, whatever the id |
| `seqgap` | IF | A node whose frame number is wrong (it lost SYNC, or its counter is off) | `gap` (3) | The frame number is off by `gap` for as long as the fault is active (frames carry the number of the SYNC frame of their cycle). With no `end` it stays wrong: the node is out of phase and is isolated; give an `end` for a one-frame glitch, which 3-of-5 tolerates |
| `reboot` | F24 | A node that resets | `down` (50; frames of silence), `resync` (1) | Silent for `down` frames, then back **in phase** (it takes the frame number from SYNC), so a restart is no sequence break. `resync=0` is a node that does not resync and restarts its frame number at 0: every frame is then the wrong frame, for good |
| `late` | F25 | Stale data from a late node | `us` (4000; 1-9000) | Every scheduled frame leaves `us` microseconds later. The command frame (due at ~5.3 ms) then misses the 7 ms vote and is judged a frame late, with a stale digest |
| `scale` | F27 | A scale-factor error | `sensor` (gyro), `axis` (0), `factor` (1.2) | Multiplies that axis by `factor` (1.0 is healthy, -1 inverts, 0 kills the axis) |
| `noise` | F28 | A noisy sensor | `sensor` (gyro), `mult` (20.0) | The noise standard deviation grows to `mult` x the healthy value (0.1 dps, 0.002 g) on all three axes of that sensor |
| `invert` | F29 | Wrong polarity or mounting | `sensor` (gyro), `axis` (0) | Negates that axis |
| `swap` | F30 | Two axes exchanged (a misalignment) | `sensor` (gyro), `axis` (0), `other` (1) | Swaps `axis` and `other` |
| `zero` | F31 | A dead sensor | none | Gyro and accel read exactly 0 on every axis (gravity disappears) |
| `clip` | F32 | The wrong full-scale range selected | `sensor` (gyro), `limit` (4.0) | The sensor's output is clipped to +-`limit` (dps or g) on all axes |
| `oscillate` | F33 | Vibration, or aliasing of it | `sensor` (gyro), `axis` (0), `amp` (3.0), `hz` (33.0) | Adds `amp` x sin(2 pi `hz` t) to that axis, sampled at 100 Hz: above 50 Hz it aliases, and 100 Hz looks like a constant offset |
| `repeat` | F34 | An output data rate that does not match the frame rate | `n` (3) | The sensors are refreshed only every `n` frames and held in between (valid frames, old data) |
| `bitflip` | F35 | A single-event upset in the sample register | `sensor` (gyro), `bit` (12), `p` (0.1) | With probability `p` per frame, flips bit `bit` (0-15) of one random axis' 16-bit sample **before** the CRC is computed, so the frame stays valid |
| `stuckbit` | F36 | A stuck data line | `sensor` (gyro), `bit` (12), `value` (1) | Forces bit `bit` of every axis' sample to `value` (0 or 1); CRC valid |
| `cmdstuck` | F37 | A frozen command output | none | Pitch and yaw freeze at their last value while digest and sequence keep counting |
| `cmdinvert` | F38 | A wrong-sign command | none | Negates pitch and yaw (the digest still matches the true command) |
| `partial` | F39 | Only some frame types are sent | `mask` (4) | Bit mask of the frame types **not** sent: 1 gyro, 2 accel, 4 command (1-7) |
| `duplicate` | F40 | Every frame sent twice | `gap_us` (300) | Each scheduled frame is repeated `gap_us` microseconds later (0-9000) |
| `replay` | F41 | Stale frames re-sent | `age` (10) | Sends the frames of `age` frames ago in place of the current ones: valid CRC, old sequence number (1-300) |
| `seqstuck` | F42 | A frozen frame-number counter | none | The frame number stays at the value it had when the fault began |
| `early` | F43 | Frames sent early | `us` (2000) | Every scheduled frame leaves `us` microseconds earlier (never before time 0; 1-9000) |
| `jitter` | F44 | Random timing jitter | `us` (1500) | Every frame leaves up to +-`us` microseconds from its slot, random but fixed by `--seed` (1-9000) |
| `clockdrift` | F45 | A drifting clock | `us_per_frame` (20) | Frames leave `us_per_frame` later each frame, cumulatively (negative = earlier; 1-1000) |

The first twelve kinds are the original set; `scale` onward come from the FMEA gap analysis (`docs/verification/FMEA.md`). Bad values are rejected
with a message (a NaN, a bit number above 15, `age=0`, an axis of 3, ...) rather than becoming a silent no-op. Bit faults act on the
16-bit sample before the CRC is computed, which is how a real upset in a sensor register looks: the frame is valid, the data are wrong.

Notes: multiple faults can be combined, including two of the same kind on one node (their effects add). `babble`
has no effect on the scheduled frames themselves; the flight computer *detects* it (bus alarm, ADR-009), but its real
harm, bus load and arbitration starvation, only shows on real CAN hardware.
`dropout` suppresses a node's scheduled frames, so a node that is both `dropout` and `babble` still floods.

**What to expect** (400 frames, fault at frame 100, `--nodes A,B,C`, seed 1, replayed with `tfc_replay`):

| Fault | Node B latches at | Caught by | Mode after |
|---|---|---|---|
| `dropout` | 102 | missing frames | duplex |
| `stuck` | 107 | vote (the stuck detector fires from frame 120 as a backup) | duplex |
| `bias` (3 dps) | 102 | vote | duplex |
| `drift` (0.05/frame) | 125 | vote, once past the 1.0 dps tolerance | duplex |
| `spike` (p=0.05) | never | vote flags 15 spikes; the 3-of-5 filter rejects them | triplex |
| `saturate` | 102 | vote | duplex |
| `corrupt` (p=0.2) | 111 | CRC failures piling up (seed-dependent: seeds 1-12 give 102-126) | duplex |
| `cmd_offset` (1.0 deg) | 102 | vote on the command | duplex |
| `digest` | 102 | digest cross-check | duplex |
| `babble` | never | out-of-schedule frames counted; `bus_alarm_frames` (BUS ALARM) raised, no node blamed | triplex |
| `seqgap` (a number that stays wrong, gap=3) | 102 | sequence errors (every frame is out of phase) | duplex |
| `seqgap` with an `end` one frame later | never | one bad frame (`seq_bad=3`: once per frame type) | triplex |
| `reboot` (down=50) | 102 | missing frames; it comes back in phase and is readmitted if the operator asks (see Recovery) | duplex, then triplex |
| `reboot` (down=2) | never | two lost frames are below 3-of-5 | triplex |
| `reboot` (down=50, resync=0) | 102 | missing frames, and after it is back every frame is the wrong frame: it stays isolated | duplex |
| `late` (us=4000) | 102 | stale data (vote + digest) and a frame numbered for the previous cycle (sequence) | duplex |
| `scale` (factor 1.5) | 102 | vote (a 50% error on a 10 dps axis is 5 dps) | duplex |
| `noise` (mult 30) | 102 | vote (sigma 3 dps against a 1 dps tolerance) | duplex |
| `invert`, `swap`, `zero`, `clip` (limit 4) | 102 | vote | duplex |
| `oscillate` (3 dps at 33 Hz) | 104 | vote | duplex |
| `repeat` (n=10) | 107 | vote (the held sample falls behind the motion) | duplex |
| `bitflip` (bit 9, p=0.5) | 116 | vote, then the leaky count ("intermittent": the upset hits only some frames) | duplex |
| `stuckbit` (bit 9 = 1) | 102 | vote | duplex |
| `cmdstuck`, `cmdinvert` | 102 | vote on the command | duplex |
| `partial` (gyro + accel not sent) | 102 | missing frames | duplex |
| `duplicate` | 102 | sequence errors (the second copy repeats a number) | duplex |
| `replay` (age 3) | 102 | stale data: vote + digest | duplex |
| `seqstuck` | 103 | sequence errors | duplex |
| `early` (6000 us) | 101 | the gyro and accel frames arrive in the *previous* frame's window carrying the next frame's number: sequence errors (this used to be invisible; ADR-018). Up to about 4.5 ms early (e.g. `us=4500`) the frame is still in its own window with the right number: not detected until arrival times are checked (`docs/design/FUTURE_WORK.md`) | duplex |
| `jitter` (3000 us) | 106 | missing frames (some frames miss the 7 ms vote) | duplex |
| `clockdrift` (+40 us/frame) | 144 | the command frame crosses the 7 ms vote deadline once the drift reaches about 1.7 ms: vote + digest | duplex |
| `corrupt` at 1 frame in 3 (`period=3,duty=1`) | 112 | the leaky count ("intermittent fault"); 3-of-5 alone never fills | duplex |
| `corrupt` at 2 frames in 5 / 2 in 10 | 106 / 121 | the leaky count | duplex |
| `corrupt` at 1 frame in 5, 10 or 20 | never | sparse trouble is left alone (no false isolation) | triplex |

**Two faults in a row** (after B is out the system is in *duplex*: two nodes can only compare, not outvote). What happens
next depends on how clear-cut the second fault is (ADR-008):

| Second fault (on C, after B is out) | Outcome | Why |
|---|---|---|
| `--fault B:bias:start=100 --fault C:bias:start=200,axis=1` (a 3 dps step) | C latches at 202, **simplex** on A | C jumped away from the last agreed value, A did not: attributable |
| three single `spike`s on C inside 5 frames | C latches at 104, **simplex** | each spike is attributable; before ADR-008 both survivors were latched and the system fell to Safe |
| `C:bias:start=200,axis=1,mag=1.5` (just above tolerance), a slow `drift`, or `C:digest` | **Safe requested**, output held, nobody blamed | not attributable; a stale reference is never used later to blame anyone |

A Safe request is *sticky*: it stays until an operator clears it (`clear_safe_request()`), and the output stays frozen.

## Recovery: latch, probation, readmission and disabling
A node that latches is out of the vote, but it is not necessarily gone for good. The life cycle (ADR-010; state diagram
in `docs/design/ARCHITECTURE.md`) is **Healthy → Latched → Probation → Healthy**, or **→ Disabled**:

1. **Latched:** excluded from the vote; a *strike* is counted against it. It must wait out a **dwell**: 50 frames (0.5 s) after a first, transient-looking latch (missing or damaged frames, one vote episode), 200 frames (2 s) after a digest mismatch, a stuck or intermittent node, and after every repeat latch.
2. **Probation:** starts when the dwell is over *and* it is asked: by an operator command (default), or automatically
   with `--policy auto` for a first latch whose cause looks transient (missing/damaged frames or a vote episode; never a
   stuck sensor or digest mismatch, never a repeat offender). The node is still out of the vote, but every frame its data is
   compared on all 8 channels with the **voted output of the healthy nodes**, and its digest with theirs: a *shadow vote*.
   It needs **100 consecutive agreeing frames** (300 after a repeat latch). One bad or disagreeing frame sends it back to
   Latched (the dwell restarts and a new request is needed), without a new strike. Only one node is on probation at a time.
3. **Disabled:** the **3rd** latch of a node (the **2nd** if the cause is physical: a stuck sensor or an intermittent fault) disables it for
   the run. A disabled node refuses `reintegrate`; only a maintenance `clear-disabled` brings it back to Latched.

| Setting | Default | Meaning |
|---|---|---|
| `policy` | Manual | Manual: only an operator command starts probation. AutoTransient: also automatic for a first, transient-looking latch |
| `min_dwell_frames` / `min_dwell_frames_transient` | 200 / 50 | Minimum time latched before probation can start; the shorter one applies to a first, transient-looking latch (it shortens the time the system runs with less redundancy; the node still has to pass the whole probation) |
| `ground_auth`, `ground_key` | on, the public bench key | Require a valid tag and a fresh counter on ground frames (`ground_key` is replaced if it is all zeros) |
| `command_window`, `arm_window_frames` | 32, 250 | How far ahead a command counter may be, and how long an ARM stays valid |
| `probation_frames` / `probation_frames_repeat` | 100 / 300 | Agreeing frames needed after the first / a repeat latch |
| `max_strikes` / `max_strikes_physical` | 3 / 2 | Latches before the node is disabled; for a physical cause (`physical_causes`, default stuck sensor or intermittent fault) |
| `strike_window_frames` | 0 (whole run) | Strikes older than this are forgotten; 0 = never |
| `auto_eligible_causes`, `auto_max_attempts` | frame problems + vote; 3 | Which causes the auto policy may readmit, and failed probations tolerated |

### Operator commands: authentication, ARM and the interlock
Commands are ground-command frames (`0x510`, ADR-019): `opcode (+ ARM flag) | node | 32-bit SipHash-2-4 tag | counter | CRC-8`.
A frame with a wrong tag, or a counter that is not 1 to 32 ahead of the last accepted one (a replay), is dropped and counted
(`commands_unauthentic`, `commands_replayed`) with no trace on the bus. Accepted commands are answered with `accepted`, `already done`
(idempotent repeat) or `refused: <reason>`. Use `--command FRAME:OP[:NODE]` (scripted) or `tfc_peers command OP [NODE]` (live):

| Command | Effect | Refused when |
|---|---|---|
| `reintegrate B` | Queue probation for latched node B (starts after its dwell). Never needs an ARM: the shadow vote is its check | B is healthy (`not latched`) or disabled |
| `disable B` | Exclude B for the rest of the run. **Plain** while it leaves at least two healthy nodes (Triplex to Duplex) or if B is not voting; **needs an ARM** to leave one voter (Duplex to Simplex); removing the **last voter** needs an ARM and is reported `CRITICAL` | `needs an ARM frame first` |
| `clear-disabled B` | Maintenance: disabled → latched, strikes cleared. **Always needs an ARM** | B is not disabled; no ARM |
| `clear-safe` | Lift a sticky Safe request. **Always needs an ARM** | (`already done` if none); no ARM |
| `warm B` | Rest B as WARM (docs/design/MISSION_PHASES.md): it leaves the vote, is judged by the shadow vote every frame and stays ready; `reintegrate B` promotes it by the probation criteria. Same tiers as `disable` (with `--phases` those of the phase) | B is not a healthy voter; the phase minimum; no ARM below the nominal |
| `phase ascent` | Change the mission phase (only with `--phases` / `CONFIG_TFC_PHASES`). Never needs an ARM | fewer healthy voters than the new phase's minimum (`the mission phase needs more computers`) |
| `noop` | Changes nothing and needs no ARM; it is answered like any command, to test the command path end to end | (never; a stale counter is dropped as a replay) |

An **ARM** is the same frame with bit 7 of the opcode set; the matching EXECUTE must follow within 250 frames, one ARM covers one
EXECUTE of one operation and node, and an ARM that is not followed expires (`arms_expired`). In a script, `armed-OP` does both
(`600:armed-clear-safe`); live, `tfc_peers command clear-safe --arm`. The interlock exists so one corrupted frame, one replay or one
script slip cannot remove redundancy, while an operator who knows a node is bad (the case Duplex cannot decide by itself) still can.

**Try it offline** (real output, `--frames 700`; the fault ends at frame 130, so the node is healthy again but stays out until asked):
```bash
python3 -m tfc_peers record --nodes A,B,C --frames 700 --fault B:bias:start=100,end=130 \
    --command 450:reintegrate:B --out /tmp/t.log
../build/host/tfc_replay /tmp/t.log --verbose
```
```
frame 102: node B latched because: vote disagreement
frame 450: ground command reintegrate B: accepted
frame 450: node B on probation (shadow vote against the healthy nodes)
frame 550: node B REINTEGRATED (strikes on record: 1)
mode=triplex        state.B=healthy     strikes.B=1
```
**A node that is still faulty is refused** (`B:bias:start=100` with no end, commands at 450 and 800):
```
frame 450: node B on probation (shadow vote against the healthy nodes)
frame 451: node B FAILED probation, back to latched: vote disagreement
frame 800: node B on probation (shadow vote against the healthy nodes)
frame 801: node B FAILED probation, back to latched: vote disagreement
mode=duplex   state.B=latched   strikes.B=1   probation_failures=2
```
**A repeat offender is disabled** (three fault windows at 100, 600 and 1200, `reintegrate` at 450, 700 and 1300):
```
frame 102: node B latched (strike 1)  -> 450 command accepted, on probation at once -> 550 REINTEGRATED
frame 602: node B latched (strike 2)  -> 700 command queued (dwell ends 802) -> 802 on probation -> 1102 REINTEGRATED (300 frames)
frame 1202: node B DISABLED for the run (strikes: 3)
frame 1300: ground command reintegrate B: refused: node is disabled
mode=duplex   state.B=disabled   strikes.B=3
```
(The first dwell is the short one, 50 frames, so the command at 450 finds it already served; the second latch is a repeat and waits the full 200.)
**Automatic policy** (`--policy auto`, `B:dropout:start=100,end=120`, no command): latched 102, on probation 152, REINTEGRATED 252,
`mode=triplex`. The same run without `--policy auto` leaves B latched for ever.

**Live:** the same `--command` works with `run --follow-sync`, and FC-A prints `GROUND COMMAND`, `ON PROBATION`,
`FAILED PROBATION`, `REINTEGRATED` and `DISABLED` events (see the live section). FC-A's policy is chosen at build time
(`CONFIG_TFC_AUTO_REINTEGRATE`, default off). Why these rules: ADR-010.

## What the peers send
Every 10 ms major frame, each simulated node sends three frames (CAN IDs from `core/include/tfc/protocol.hpp`):

| ID | Frame | Peer send time in the frame | Payload (8 bytes: 6 data, 1 sequence, 1 CRC-8) |
|---|---|---|---|
| `0x010` | SYNC (sent by the sync master, FC-A; the peers only *listen* for it) | 0.0 ms | frame number (u32, little endian), 2 reserved bytes |
| `0x100+n` | GYRO | 1.5 ms + 0.2 ms x n | 3 x int16, 1/32 dps per count |
| `0x110+n` | ACCEL | 2.3 ms + 0.2 ms x n | 3 x int16, 1/2048 g per count |
| `0x510` | GROUND (operator command; sent only by `--command` / `tfc_peers command`) | 6.5 ms in the frame it is scripted for | opcode (1 reintegrate, 2 disable, 3 clear-disabled, 4 clear-safe, 5 launch, 6 scrub, 7 phase, 8 noop, 9 warm; +0x80 = ARM), node, 32-bit SipHash tag, command counter |
| `0x200+n` | CMD (command + digest) | 5.0 ms + 0.3 ms x n | pitch int16 and yaw int16 (0.001 deg per count), digest u16 |

`n` is the node number: A=0, B=1, C=2. The sequence number counts frames (wraps at 256). FC-A itself sends its
gyro and accel back to back at 1.5 ms and its command at 5.0 ms.

**The simulated motion** every healthy IMU feels, at `t = frame x 0.01 s + 0.0005 s`:
gyro = (10 sin(2pi 0.8 t), 6 cos(2pi 0.5 t), 3 sin(2pi 0.3 t)) dps; accel = (0.05 sin(2pi 0.4 t),
0.03 cos(2pi 0.6 t), 1.0) g. Each node adds its own independent noise (gyro 0.1 dps, accel 0.002 g, 1-sigma).
The command is 0.1 x gyro axis 0 (pitch) and 0.1 x gyro axis 1 (yaw), clamped to +-30 deg, and is **noise free**
so healthy replicas send bit-identical commands (the design's deterministic replicas, ADR-006). The digest is a
synthetic function of the command and the frame number, identical across healthy nodes.

Example log line: `(0.001500) vcan0 100#010031000100009D` = at 1.5 ms, node A's gyro: bytes
`0100 3100 0100` = 1, 49, 1 counts = (0.031, 1.531, 0.031) dps, sequence `00`, CRC `9D`.

## How the decisions are made, and the replay output
`tfc::RedundancyManager` (`core/include/tfc/redundancy.hpp`) runs once per 10 ms frame:
1. Decode each node's gyro, accel and command frames; check CRC and sequence number.
2. Vote each of the 8 channels (gyro x3, accel x3, command pitch, yaw) with `vote3` (median of three). With only two voting nodes that disagree, try to blame the one that jumped away from the last agreed value; otherwise hold the last good output and, if it persists, request Safe (sticky).
3. Cross-check the estimator-state digests; run a stuck detector on the raw sensor bytes.
4. Feed each node's verdict to its own `ChannelMonitor` and a leaky `AlphaCount`. A node is **latched out** when 3 of its last 5 frames
   were bad, or when the leaky score (+1 per bad frame, x0.9 per good frame) reaches 3; from then on its data is excluded from the vote until explicitly reintegrated.

| Setting | Value | Meaning |
|---|---|---|
| Vote tolerance | gyro 1.0 dps, accel 0.02 g, command 0.01 deg | A value further than this from the median counts as disagreeing |
| Persistence | 3 of the last 5 frames | One glitch never latches |
| Leaky count (intermittent faults) | K=0.9, threshold 3 | +1 per bad frame, x0.9 per good one. Catches 1 bad frame in 3 (isolated 12 frames after it starts) and 2 in 5 (6 frames) that 3-of-5 never sees; 1 in 5 or sparser is left alone (ADR-013) |
| Stuck limit | 20 identical sensor frames | Backup for a stuck sensor when the vehicle is at rest |
| Reintegration | explicit request + 100 clean frames | The replay never requests it (no operator command in a log) |
| Permanent failure | 3 latches | Then never reintegrated |
| Startup grace | 0 frames in `tfc_replay` | A missing node is bad from the first frame. FC-A uses 500 (5 s) so peers may boot late |
| Duplex arbitration | culprit must be > 2.0 x tolerance from a fresh last-agreed value, the other within 1 x | Else unresolved: nobody blamed, output held; 3 of 5 unresolved frames request Safe (sticky) |
| Bus alarm | 3 out-of-schedule frames in one 10 ms frame | Raised while it continues; blames no node (a CAN ID is not a sender) |

**What happens to a latched node** is described in [Recovery](#recovery-latch-probation-readmission-and-disabling): dwell, probation by shadow vote, strikes, disabling.

**Why a node was judged bad** (named in `--verbose` output and in FC-A's console): `intermittent fault` (the leaky count latched it; 3-of-5 never fired), `frame missing`,
`CRC failure`, `sequence error` (a damaged frame, or a slot in which nothing arrived, still consumes a sequence number, so one corrupted or lost frame costs one
bad sample, not two; ADR-007), `vote disagreement`, `digest mismatch`, `stuck sensor`.

**`tfc_replay` output** (`key=value`, one per line):

| Key | Meaning |
|---|---|
| `frames` | 10 ms frames in the log |
| `latch.A/B/C` | Frame on which that node *first* latched out, or `-` if never |
| `state.A/B/C`, `strikes.A/B/C` | Final state (`healthy`/`latched`/`probation`/`disabled`) and strikes on record per node |
| `healthy`, `mode` | Nodes still voting at the end, and `triplex`/`duplex`/`simplex`/`safe` |
| `crc_bad` | Frames discarded for a failed CRC, over the whole run, all nodes |
| `seq_bad` | Frames whose sequence number was not the expected next |
| `missing` | Node-frames where gyro, accel or command did not all arrive (keeps counting after a latch) |
| `out_of_schedule` | Frames on IDs that are not part of the schedule |
| `vote_disagreements` | Frames in which any channel vote flagged a node |
| `digest_flags`, `stuck_flags` | Frames flagged by the digest cross-check / stuck detector |
| `probations_started`, `probation_failures` | Probations begun, and those ended by a bad or disagreeing frame |
| `reintegrations`, `nodes_disabled` | Nodes readmitted to the vote; disable events (strikes used up, or by command) |
| `commands_accepted`, `commands_refused`, `commands_bad` | Ground commands applied; refused (with the reason in `--verbose`); frames dropped for a failed CRC |
| `unresolved_frames` | Frames with a duplex disagreement nobody could be blamed for |
| `held_frames` | Frames in which some output channel held its last good value (unresolved, no majority, no data, or Safe requested) |
| `safe_request_frames` | Frames spent with the (sticky) Safe request raised |
| `bus_alarm_frames` | Frames with the out-of-schedule flood alarm raised |
| `commands_unauthentic`, `commands_replayed` | Ground frames dropped because their tag did not verify, or because their counter was a repeat or too old (a replay); neither leaves a trace on the bus or reaches the command queue (ADR-019) |
| `phase_changes`, `below_minimum_frames` | `phase` commands that changed the mission phase; frames spent with fewer voters than the phase's minimum (ADR-023, `--phases`) |
| `state_restores` | restarts that took strike counts or the command counter from the others' state shares (FDIR-041; the replay never restores, so this stays 0 there) |
| `arms_expired`, `critical_commands` | ARM frames never followed by their EXECUTE in time; commands that removed the last voting node (`--verbose` prints `*** CRITICAL ***`) |
| `integrity_faults`, `invariant_violations` | Upsets found and repaired in the manager's own state (node states, Safe flag, configuration, command queue), and internal invariants that did not hold. Both are 0 in any normal run; `--verbose` prints `INTEGRITY FAULT` when one happens (ADR-015) |

## Live CAN in depth

### Setup
```bash
./scripts/setup_vcan.sh                        # sudo; once per boot
west build -p always -b native_sim/native/64 firmware/app -d build/native_sim    # repo root, after `. firmware/env.sh`
```
`vcan0` is a virtual CAN interface in the Linux kernel: every frame written to it is delivered to every other
socket bound to it, like a bus with no wires. FC-A (the `native_sim` build) is bound to `vcan0` through the
board overlay (`firmware/app/boards/native_sim_native_64.overlay`), and `tfc_peers` binds to it with a standard
SocketCAN socket. Nothing else is needed.

### The three live modes
1. **`listen` only.** A read-only monitor. Run it in its own terminal next to the others.
2. **`run` free-running** (no `--follow-sync`): the peers send on their own 100 Hz clock starting when you
   launch them, with frame numbers counting from 0. Use it when **no** flight computer is on the bus, for example
   to test a monitor or another receiver (`--nodes A,B,C` is fine here).
3. **`run --follow-sync`** with FC-A: the peers wait for each SYNC frame from FC-A, take its frame number as theirs,
   and send that frame's traffic at the schedule offsets *measured from the moment SYNC arrived*, exactly as a
   time-triggered node would. Use `--nodes B,C`: FC-A supplies A itself.

### Why `--follow-sync` exists
FC-A sends SYNC every 10 ms and expects its peers' frames within the same frame, before it votes at 7 ms.
Free-running peers have their own phase and their own frame count. Measured, with **no fault injected**:

```
[frame 49] node B joined the bus ... node C joined the bus
[frame 51] node A LATCHED OUT: vote disagreement + digest mismatch
[frame 100] DUPLEX  AX B+ C+  | ... vote=3 digest=3
```
B and C agree with each other (they share a frame count) but not with FC-A (different frame number, so a
different "truth" and a different digest), so the 2-of-3 vote trusts the majority and isolates the healthy FC-A.
That is the voter doing its job; the inputs were simply not synchronised. With `--follow-sync` the same run has
zero alarms.

### What you see
FC-A's console (events plus a status line each second; `+` voting, `X` latched out, `?` no good data this frame):
```
[frame 44] node B joined the bus
[frame 44] MODE SIMPLEX -> TRIPLEX
[frame 100] TRIPLEX  A+ B+ C+  | crc=0 seq=0 missing=0 vote=0 digest=0 stuck=0 tx_err=0
[frame 402] node B LATCHED OUT: vote disagreement
[frame 402] MODE TRIPLEX -> DUPLEX
```
`listen` (all nodes' frames interleaved, 10 ms apart; here with FC-A and synced peers, `C:bias` on accel):
```
   0.6728  010  SYNC  -  frame=51 seq=51
   0.6744  100  GYRO  A  seq=51  [   +5.500    -0.125    +2.250] dps
   0.6746  101  GYRO  B  seq=51  [   +5.500    +0.000    +2.500] dps
   0.6748  102  GYRO  C  seq=51  [   +5.500    -0.250    +2.500] dps
   0.6754  111  ACCEL B  seq=51  ...
```
**Timing jitter shows up as `missing`.** FC-A votes at 7.0 ms; the Python peers' last frame is due at about 5.6 ms,
so there is only ~1.4 ms of margin for desktop-Linux scheduling. On an idle machine `missing` stays 0; on a busy one
(the shared GitHub CI runner) a few peer frames arrive after the vote and are counted as `missing` (4-8 in 400
frames was seen there). Nobody latched and the mode stayed TRIPLEX: that is exactly what the 3-of-5 filter is for. A late frame costs only that one `missing`: when it turns up in the next frame its sequence number is recognised as the slot it was assumed missing for, so `seq=` stays 0 (ADR-007).
The live tests therefore allow a few `missing` and a latch a few frames either side of the ideal fault + 2.

Both Python's `run` lateness line (typically p50 3 us, p99 15-45 us, max under 1 ms on this machine) and FC-A's
timing are desktop-Linux numbers. Treat them as "good enough for logic", never as a flight-computer measurement.

### Recovery events live
FC-A prints each step of the life cycle (real output of `--fault B:bias:start=300,end=330 --command 450:reintegrate:B`):
```
[frame 302] node B LATCHED OUT: vote disagreement
[frame 302] MODE TRIPLEX -> DUPLEX
[frame 450] GROUND COMMAND reintegrate B: accepted
[frame 450] node B ON PROBATION: shadow vote against the healthy nodes (strikes: 1)
[frame 550] node B REINTEGRATED into the vote (strikes on record: 1)
[frame 550] MODE DUPLEX -> TRIPLEX
```
The command at 450 was accepted and acted on at once: the dwell after a first transient latch is 50 frames (302 + 50 = 352), long over. In the status line `p` means on probation and `D`
disabled. To send a command at an arbitrary moment instead of a scripted frame: `python3 -m tfc_peers command reintegrate B`.

### Typical sessions
```bash
# healthy system: expect no alarms; --frames 0 runs until you press Ctrl+C
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 0
# a dead node
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 0 --fault C:dropout:start=300
# several faults, different seeds
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 0 --seed 3 \
    --fault B:corrupt:start=200,end=260,p=0.3 --fault C:drift:start=400,rate=0.1
# watch the bus while it happens (third terminal)
python3 -m tfc_peers listen --duration 10
```
When the peers stop (their `--frames` ran out, or you pressed Ctrl+C), FC-A latches them out as `frame missing` within 3 frames: that is correct.

### Pointing it at real hardware later
With the USB-CAN adapter, the same commands work with `--iface can0` after `sudo ip link set can0 type can
bitrate 1000000 && sudo ip link set up can0`. This path is **untested**: the adapter has not arrived. Flight
computers on the Nucleo use FDCAN1 (see `firmware/README.md`).

### Troubleshooting
| Message or symptom | Cause and fix |
|---|---|
| `No module named tfc-peers` (with a **hyphen**) | Two different names: **`tfc_peers`** (underscore) is the Python package, run as `python3 -m tfc_peers ...` from inside `sim/`; **`tfc-peers`** (hyphen) is the launcher *script*, run directly as `./tfc-peers ...` (no `python3 -m`). |
| `No module named tfc_peers` | You are not in `sim/`. `cd sim`, or use the launcher from anywhere: `sim/tfc-peers ...`. |
| One node latched from your fault, then the other latches `frame missing` | The peers stopped (fixed `--frames` ran out) while FC-A kept running. Use `--frames 0` (until Ctrl+C) or a larger N. Not a fault. |
| `cannot open CAN interface 'vcan0'` | The interface does not exist. `./scripts/setup_vcan.sh` (it is gone after a reboot). |
| `no SYNC on the bus for 2.0 s: start the flight computer first` | `--follow-sync` needs FC-A running. Start `zephyr.exe` first, or drop `--follow-sync` if there is no flight computer. |
| `note: SYNC frame number went back ...` | FC-A restarted; the peers restarted their scenario too. Not an error. |
| FC-A latched itself or reports `vote=` / `digest=` counts with no fault | Peers are not following SYNC. Add `--follow-sync`. |
| Healthy nodes latch out (even before your fault) while the machine is under heavy load | Both the Python peers and the simulated firmware are ordinary Linux processes, not real-time. Starved for tens of ms (seen with 24 busy loops on 12 cores) frames are genuinely late and the voter correctly reacts. Run live sessions on an otherwise idle machine; use offline `record` + `tfc_replay` for anything that must be exact. |
| FC-A: `GROUND COMMAND ...: refused: node is not latched / is disabled / is not disabled` | The command did not apply: `reintegrate` needs a latched node; `clear-disabled` needs a disabled one; a disabled node refuses `reintegrate` (use `clear-disabled` first). Nothing is wrong. |
| FC-A: `node B FAILED PROBATION, back to latched: vote disagreement` (or `digest mismatch`, `frame missing`...) | The node's data still disagree with the healthy nodes (or its frames are missing), so it is correctly refused. Fix or remove the fault (check your `--fault` has an `end`), wait out the dwell and ask again. |
| FC-A: `SAFE REQUESTED: the two voting nodes disagree and nobody can be blamed` | Two nodes are voting (the third is out) and they disagree in a way that cannot be attributed (slow drift, small step, digest mismatch). Output is held and Safe stays requested until cleared. Expected for those faults; see the F16 table. |
| FC-A: `BUS ALARM RAISED` | Three or more out-of-schedule frames in one 10 ms frame (your `babble` fault, or a real flooding node). Clears when it stops. No node is blamed. |
| Small `missing=` count on a healthy run, no latch | Timing jitter: a peer frame landed after FC-A's 7 ms vote on a busy machine. Absorbed by the 3-of-5 filter; close other heavy programs if it bothers you. |
| FC-A: `[frame 2] node B LATCHED OUT: frame missing` | Peers not running within the 5 s startup grace, or `--nodes` omits them. |
| `listen` prints nothing | Nothing is sending on that interface, or the wrong `--iface`. |
| `BrokenPipeError` after piping `listen` into `head` | Harmless: `head` closed the pipe. |
| `error: --nodes is empty` / `unknown node 'D'` / `fault ... targets a node that is not simulated` | Fix `--nodes` or the fault's node letter. |
| FC-A runs forever | Pass `-stop_at=<seconds>` or press Ctrl+C. |

## Offline vs live
Same traffic generator, same faults, same `core/` decision code. The differences:

| | Offline | Live |
|---|---|---|
| Who generates A's data | Peers play A too (`--nodes A,B,C`) | FC-A really samples its own (simulated) IMU with its own noise; peers play only B, C |
| Frame numbers | 0, 1, 2, ... | FC-A's frame numbers, from SYNC; a late joiner fast-forwards so its state matches a run from 0 |
| Fault timing | Exactly at frame `start` | Exactly at frame `start` of FC-A's count; reproducible to the frame (the tests assert it) |
| Timing and jitter | None; time is just a number in the log | Real: SYNC lock, 1.5/5.0/7.0 ms slots, kernel ticks, Python send jitter |
| Boot order, startup grace, joins | Not modelled | Real: `joined the bus`, grace period, restarts |
| Operator commands | Scripted into the log at an exact frame (`--command`) | Scripted the same way, or typed at any moment with `tfc_peers command` |
| Speed | Thousands of times real time | 10 ms per frame |
| What you keep | A log file you can re-run, decode and diff | FC-A's console (use `tee`); `listen` output is not replayable |
| Sweeps over seeds and faults | Yes, the right tool | Possible, but slow; the live test suite covers 5 scenarios |
| What neither can show | Arbitration, bus load, bus-off, wiring faults, real latency | (needs the real nodes: F11 timing, F12-F15, F17) |

## Logs
`record` writes the standard `candump -L` format, one frame per line: `(seconds.microseconds) interface ID#HEXDATA`.
- Where: exactly the `--out` path you gave. `/tmp` is cleared at reboot, so use a permanent path (for example
  `~/logs/x.log`) for anything you want to keep.
- Look: `python3 -m tfc_peers decode FILE | less`, `... | grep CRC-BAD`, `... | awk '$2==101'` (only node B's gyro),
  or `less FILE` for the raw bytes. Compatible with `can-utils` (`canplayer`) if you install it.
- Live runs write nothing by themselves. Save FC-A's console with `zephyr.exe ... | tee /tmp/fca.log`.
  Replaying a live capture is not supported yet: the replay's 10 ms frame grouping would need to be aligned to SYNC.

## Fault campaign: every fault, many inputs, safety properties checked on every frame
`campaign/` (this directory) runs thousands of scenarios through the real `core/` code and checks properties that must
always hold. The methodology, the properties, the measured response of every fault kind and the edge cases found are in
[`docs/verification/FAULT_CAMPAIGN.md`](../docs/verification/FAULT_CAMPAIGN.md). In short: a *scenario* is a set of faults (kind, node, start frame,
magnitude, duration, intermittency), operator commands, a redundancy context (Triplex, or Duplex/Simplex made by dropping
nodes) and a seed; an *oracle* is a property checked on the per-frame CSV from `tfc_replay --dump`.

```bash
cmake -S .. -B ../build/rel -DCMAKE_BUILD_TYPE=Release && cmake --build ../build/rel --target tfc_replay
export TFC_REPLAY_BIN=$PWD/../build/rel/tfc_replay
python3 -m campaign.run --list                                   # the groups and their sizes
python3 -m campaign.run --out /tmp/campaign.jsonl                # everything (about 11,300 scenarios, 2 minutes on 12 cores)
python3 -m campaign.run --group bias_gyro --group phase_sweep    # only some groups (repeatable); --limit N for a quick look
python3 -m campaign.report /tmp/campaign.jsonl bias_gyro drift   # the Markdown tables of docs/verification/FAULT_CAMPAIGN.md, plus response curves
python3 -m campaign.mutate                                       # does the campaign notice deliberate bugs in core/? (about 90 minutes)
```
| Flag | Meaning |
|---|---|
| `--group NAME` | Run only this group (repeatable). `--list` shows them all. |
| `--workers N` | Parallel scenarios (default 8). |
| `--out FILE` | Where the per-scenario results go (JSON lines: latch frames, reasons, counters, metrics, anomalies, a hash of every per-frame decision). |
| `--limit N` | At most N scenarios per group. |
| `--every K` | Every K-th scenario of each group, always including the first: a spread-out sample (CI runs `--every 8` on pull requests and the full set on `main`). |
| `--quiet` | No progress lines. |
| `--strict` | Exit status 1 if any anomaly was raised (CI uses this). Without it the status is 0 and the anomalies are only printed and stored. |

With `--strict` the exit status is 1 if any scenario raised an anomaly. Two further tools prove the *tests* can fail:
`python3 tools/mutation/run_unit.py` (390 deliberate bugs in `core/` and `supervisor/`, each must be caught by the C++ tests) and
`python3 -m campaign.mutate` (the 58 of them that this campaign can see, against this campaign). The decision hash in the results makes a refactor
provable: run the campaign before and after, and the hashes of every scenario must be identical.

## Tests
```bash
cmake -S .. -B ../build/host -G Ninja -DTFC_SANITIZE=ON && cmake --build ../build/host
python3 -m unittest discover -s tests -t .                  # everything below
python3 -m unittest tests.test_live_fc -v                   # only the live FC-A tests
```
| File | What it covers |
|---|---|
| `tests/test_protocol.py` | CRC, golden frames generated by the C++ code (incl. SYNC), every single-bit flip detected, decode |
| `tests/test_peers.py` | Healthy traffic and schedule, every fault's effect on the wire, fault-spec parsing, logs, follow-SYNC logic |
| `tests/test_replay.py`, `tests/test_replay_fmea.py`, `tests/test_faults_fmea.py` | Peers -> log -> **C++ `tfc_replay`**: every fault kind through the real `core/` code against the requirements; the node life cycle; the phases and the WARM role (`ReplayPhases`) |
| `tests/test_live_fc.py`, `test_live_triplex.py`, `test_live_closed_loop.py` | The **real firmware** on `vcan0`: one flight computer against the peers; three computers with the sync master killed; the closed loop with the simulator and ACT through max-Q |
| `tests/test_live_resync.py`, `test_live_split.py`, `test_live_release.py` | State resynchronisation after withheld frames, the sensor split with a biased IMU, mixed releases (hold and ask) |
| `tests/test_live_launch.py`, `test_live_phases.py`, `test_live_act_safe.py`, `test_live_simd_edges.py`, `test_live_pico_stream.py` | The launch sequence (scrub, dying master, the `T0` edge), phases and WARM, ACT's hardware Safe line, the simulator's edge cases, the platform stream |
| `tests/test_launch_cli.py`, `test_pico_link.py`, `test_timecorr.py`, `test_bench_tools.py`, `test_compat_gate.py`, `test_release_record.py` | The launch checklist with a scripted bus, the Pico link, time correlation, the bench tools (bus loss, jitter, clock correlation), the compatibility gate, the release record |
| `tests/test_ts17.py` | The arithmetic behind the TS-17 decision |
| `tests/test_docs.py`, `tests/test_docs_consistency.py` | Fails if this README stops documenting a flag, fault kind or fault option; fails on a broken link, a document missing from the index, a missing status line or a wrong section number in `docs/` |

`ctest --test-dir ../build/host` runs the same suite (`peers_e2e`). Replay tests skip if `tfc_replay` is not
built; live tests skip unless `vcan0` exists and their images are built (`tools/bench/sil_triplex.sh`; `firmware/README.md`).

## Using it as a library
```python
from tfc_peers import bus as B
from tfc_peers.faults import parse_fault
from tfc_peers.peers import Scenario
from tfc_peers.protocol import describe

sc = Scenario([0, 1, 2], [parse_fault("B:bias:start=100,mag=3")], seed=1)
mem = B.ListBus()
B.record(sc, mem, 400)                       # 3600 (t_us, Frame) pairs in mem.sent
print(describe(mem.sent[1][1]))              # GYRO  B  seq=0 [...] dps
lb = B.LogBus("x.log"); B.record(Scenario([1, 2]), lb, 100); lb.close()   # or write a log
bus = B.SocketCanBus("vcan0"); B.run_realtime(Scenario([1, 2]), bus, 100)   # or send live
```
`Scenario.frames(k)` returns frame `k`'s traffic; requesting a later frame fast-forwards deterministically,
going backwards needs `reset()`.

## Limits of these tools
The system's limits are in `docs/LIMITATIONS.md`; these are the tools'.
- **Not real-time:** Python on desktop Linux gives no latency guarantee; the lateness it prints is for information.
- **Synthetic digest:** the peers' estimator-state digest is a function of the command and frame number (a scripted firmware image computes the same function in C++ bit for bit). Checking a real estimator's digest is the closed loop's job (three firmware images with the flight function).
- **The peers send sensor and command frames only:** no heartbeat, state share or resync frames, so a flight computer listening to them sees no release, no shares and no resync (the live tests with several firmware images do).
- **Bus effects:** arbitration, bus load, babbling's real harm, bus-off and wiring faults need hardware.
