# Simulation and virtual peers

| Part | Status |
|---|---|
| `tfc_peers/`: **virtual peers**, fake flight computers that put real, fault-injectable traffic on the flight bus | Done |
| `../tools/replay`: `tfc_replay`, runs recorded traffic through the real `core/` redundancy code | Done |
| Vehicle simulator (6-DOF ascent, platform-rate limiter, SocketCAN gateway, fault-campaign runner) | Not started (milestone M3) |

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
real CAN IDs, CRC-8, sequence numbers, sensor noise) and can break any of them on purpose with 11 kinds of
fault. Everything is standard-library Python 3.10+; nothing to install.

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
| Generate traffic into a file | `python3 -m tfc_peers record ... --out FILE` |
| Read a log file in plain English | `python3 -m tfc_peers decode FILE` |
| Ask "what would a flight computer decide about this log?" | `../build/host/tfc_replay FILE --verbose` |
| Assert an outcome (pass/fail, for tests) | `tfc_replay FILE --expect-latch B:102 --expect-mode duplex` |
| Watch a CAN bus in decoded form | `python3 -m tfc_peers listen --iface vcan0` |
| Run fake peers next to the **real FC-A firmware** | `python3 -m tfc_peers run --follow-sync --nodes B,C ...` |
| Run fake peers with **no** flight computer (to test a monitor or other receiver) | `python3 -m tfc_peers run --nodes B,C ...` (free-running) |
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
# terminal 2, within a second or two: fake FC-B and FC-C that follow FC-A's SYNC; B gets a bias at frame 400
cd sim && python3 -m tfc_peers run --follow-sync --nodes B,C --frames 750 --fault B:bias:start=400,mag=3
```
FC-A's console shows `node B joined the bus`, then `node B LATCHED OUT: vote disagreement` at frame 402.
Add a third terminal with `python3 -m tfc_peers listen` to watch every frame on the bus.

## Command reference

### `python3 -m tfc_peers record`: generate traffic into a log (offline)
Virtual time, no sleeping: 400 frames take milliseconds. Output is identical on every run for the same inputs.

| Flag | Default | Meaning |
|---|---|---|
| `--out FILE` | (required) | Log file to write, `candump -L` format. Overwritten if it exists. |
| `--nodes LIST` | `B,C` | Which flight computers to simulate, comma separated from `A,B,C` (or `0,1,2`; case and spaces ignored). Use **`A,B,C`** for a fully virtual triplex: `tfc_replay` needs all three present, otherwise it correctly treats the missing one as dead. |
| `--fault SPEC` | none | `NODE:KIND[:key=value,...]`. **Repeatable**; any number of faults, on any nodes, overlapping or not. A fault must target a node listed in `--nodes`. See [Fault reference](#fault-reference). |
| `--frames N` | `1000` | Number of 10 ms major frames. 100 = 1 s, 400 = 4 s. |
| `--seed N` | `1` | Seed for sensor noise and for the random faults (`spike`, `corrupt`, `babble`). Same seed = byte-identical log. Change it to see how a result varies. |
| `--iface NAME` | `vcan0` | Only the interface name written into each log line; has no other effect. |

`record` never emits SYNC (the peers are not the sync master). Frame numbers start at 0.

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
| `--nodes`, `--fault`, `--frames`, `--seed` | as for `record` | With a real FC-A on the bus use `--nodes B,C`. Fault `start`/`end` frame numbers are then **FC-A's frame numbers** (SYNC's), not "seconds since the peers started". |

Ctrl+C stops it cleanly (exit status 130).

### `python3 -m tfc_peers listen`: bus monitor (live)
| Flag | Default | Meaning |
|---|---|---|
| `--iface NAME` | `vcan0` | Interface to monitor. |
| `--duration S` | until Ctrl+C | Stop after S seconds. |

Prints every frame seen, decoded, with the time since `listen` started. It is a pure observer: it does not judge
anything and what it prints is not a replayable log.

### `python3 -m tfc_peers faults`
Lists every fault kind with its fault-matrix row and options. Same information as the next section.

### `tfc_replay LOG ...` (C++, `build/host/tfc_replay`): what would a flight computer decide?
Feeds each 10 ms frame of the log to `tfc::RedundancyManager`, the same class the firmware runs, and reports.

| Flag | Meaning |
|---|---|
| `LOG` | `candump -L` log. Frames are grouped into 10 ms frames by timestamp, so logs from `record` (which start at t=0) work as-is. |
| `--verbose` | Print one line per latch event: `frame N: node X latched because: <reasons in words>`. |
| `--t0 SECONDS` | Subtract this from every timestamp before grouping (for logs that do not start at 0). |
| `--expect-latch NODE:FRAME` or `NODE:MIN-MAX` | Node (A/B/C) must have latched at exactly that frame, or within the range (inclusive). Repeatable. |
| `--expect-no-latch NODE` | Node must never have latched. Repeatable. |
| `--expect-mode MODE` | Final mode must be `triplex`, `duplex`, `simplex` or `safe`. |
| `--expect-min STAT:N` | A counter must be at least N. `STAT` is one of `crc_bad`, `seq_bad`, `missing`, `out_of_schedule`, `stuck_flags`, `digest_flags`, `vote_disagreements`. Repeatable. |

Exit status: `0` ran and every expectation held, `1` an expectation failed, `2` usage or input error.
Output format is described in [the replay output](#how-the-decisions-are-made-and-the-replay-output).

### `./scripts/setup_vcan.sh [IFACE]`
Loads the `vcan` kernel module and creates and brings up a virtual CAN interface (default `vcan0`). Needs `sudo`.
Safe to re-run. It does not survive a reboot. Remove it with `sudo ip link del vcan0`.
`ip -brief link show vcan0` should show `UNKNOWN <NOARP,UP,LOWER_UP>`; `UNKNOWN` is normal for a virtual interface.

### Exit status of `tfc_peers`
`0` success, `2` error (bad argument or fault spec, unknown node, interface or file not found, no SYNC), `130` Ctrl+C.

## Fault reference
Syntax: `NODE:KIND[:key=value,key=value,...]`, for example `B:bias:start=100,mag=3`. `NODE` is `A`, `B` or `C`.
Every fault also accepts `start=N` (first frame it is active, default 0) and `end=N` (first frame it is *not*
active, default never). Frames are 10 ms; `start=100` is 1.0 s in. Faults only change what the node *sends*.

| Kind | Matrix row | What it simulates | Options (default) | What it changes on the wire |
|---|---|---|---|---|
| `dropout` | F01 | A node that dies: fail-silent | none | Node sends no frames at all |
| `stuck` | F03 | A frozen sensor | none | Gyro and accel values freeze at the node's last good sample (frames still valid, sequence still counting) |
| `bias` | F04 | A constant sensor offset | `sensor`=`gyro`\|`accel` (gyro), `axis`=0\|1\|2 (0), `mag` (3.0; dps for gyro, g for accel) | Adds `mag` to that axis |
| `drift` | F05 | A slowly growing offset | `sensor` (gyro), `axis` (0), `rate` (0.05; per frame, dps or g) | Adds `rate` x (frames since start + 1) to that axis |
| `spike` | F06 | Random single-frame outliers | `sensor` (gyro), `mag` (20.0), `p` (0.05; probability per frame) | With probability `p`, adds +-`mag` to one random axis for that frame |
| `saturate` | F07 | Sensor pinned to full scale | none | Gyro and accel at the 16-bit limits (+4095.875/-4096 dps, +-16 g), sign alternating each frame |
| `corrupt` | F08 | A flaky link | `p` (0.2; probability per frame, per frame type) | Flips one random bit of a gyro/accel/command frame; its CRC then fails |
| `cmd_offset` | F09 | A wrong-but-valid command (software bug) | `mag` (1.0; degrees) | Adds `mag` to the pitch command; everything else, including the CRC, stays valid |
| `digest` | F10 | Silent internal state divergence | `xor` (1; 16-bit mask) | XORs the estimator-state digest in the command frame; pitch and yaw unchanged |
| `babble` | F11 | A node flooding the bus | `n` (5; extra frames per 10 ms) | Adds `n` extra frames per cycle on out-of-schedule IDs `0x020`-`0x02F` (higher priority than every sensor ID) |
| `seqgap` | IF | A node that skipped sequence numbers | `gap` (3) | At `start`, the sequence counter jumps ahead by `gap`, once, then counts normally |

Notes: multiple faults can be combined, including two of the same kind on one node (their effects add). `babble`
has no effect on the scheduled frames themselves; its harm is bus load, which only real CAN hardware shows.
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
| `babble` | never | out-of-schedule frames ignored and counted | triplex |
| `seqgap` | never | one bad frame (`seq_bad=3`: once per frame type) | triplex |

Two faults in a row (`--fault B:bias:start=100 --fault C:bias:start=200,axis=1`) end in `safe`: after B is
out, a second fault leaves two survivors that disagree and cannot be attributed, so both are dropped.

## What the peers send
Every 10 ms major frame, each simulated node sends three frames (CAN IDs from `core/include/tfc/protocol.hpp`):

| ID | Frame | Peer send time in the frame | Payload (8 bytes: 6 data, 1 sequence, 1 CRC-8) |
|---|---|---|---|
| `0x010` | SYNC (sent by the sync master, FC-A; the peers only *listen* for it) | 0.0 ms | frame number (u32, little endian), 2 reserved bytes |
| `0x100+n` | GYRO | 1.5 ms + 0.2 ms x n | 3 x int16, 0.125 dps per count |
| `0x110+n` | ACCEL | 2.3 ms + 0.2 ms x n | 3 x int16, 1/2048 g per count |
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
`0100 3100 0100` = 1, 49, 1 counts = (0.125, 6.125, 0.125) dps, sequence `00`, CRC `9D`.

## How the decisions are made, and the replay output
`tfc::RedundancyManager` (`core/include/tfc/redundancy.hpp`) runs once per 10 ms frame:
1. Decode each node's gyro, accel and command frames; check CRC and sequence number.
2. Vote each of the 8 channels (gyro x3, accel x3, command pitch, yaw) with `vote3` (median of three).
3. Cross-check the estimator-state digests; run a stuck detector on the raw sensor bytes.
4. Feed each node's verdict to its own `ChannelMonitor`. A node is **latched out** when 3 of its last 5 frames
   were bad; from then on its data is excluded from the vote until explicitly reintegrated.

| Setting | Value | Meaning |
|---|---|---|
| Vote tolerance | gyro 1.0 dps, accel 0.02 g, command 0.01 deg | A value further than this from the median counts as disagreeing |
| Persistence | 3 of the last 5 frames | One glitch never latches |
| Stuck limit | 20 identical sensor frames | Backup for a stuck sensor when the vehicle is at rest |
| Reintegration | explicit request + 100 clean frames | The replay never requests it (no operator command in a log) |
| Permanent failure | 3 latches | Then never reintegrated |
| Startup grace | 0 frames in `tfc_replay` | A missing node is bad from the first frame. FC-A uses 500 (5 s) so peers may boot late |

**Why a node was judged bad** (named in `--verbose` output and in FC-A's console): `frame missing`,
`CRC failure`, `sequence error` (a damaged frame still consumes a sequence number, so one corrupted frame costs one
bad sample, not two), `vote disagreement`, `digest mismatch`, `stuck sensor`.

**`tfc_replay` output** (`key=value`, one per line):

| Key | Meaning |
|---|---|
| `frames` | 10 ms frames in the log |
| `latch.A/B/C` | Frame on which that node latched out, or `-` if never |
| `healthy`, `mode` | Nodes still voting at the end, and `triplex`/`duplex`/`simplex`/`safe` |
| `crc_bad` | Frames discarded for a failed CRC, over the whole run, all nodes |
| `seq_bad` | Frames whose sequence number was not the expected next |
| `missing` | Node-frames where gyro, accel or command did not all arrive (keeps counting after a latch) |
| `out_of_schedule` | Frames on IDs that are not part of the schedule |
| `vote_disagreements` | Frames in which any channel vote flagged a node |
| `digest_flags`, `stuck_flags` | Frames flagged by the digest cross-check / stuck detector |

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
Both Python's `run` lateness line (typically p50 3 us, p99 15-45 us, max under 1 ms on this machine) and FC-A's
timing are desktop-Linux numbers. Treat them as "good enough for logic", never as a flight-computer measurement.

### Typical sessions
```bash
# healthy system: expect zero alarms, then FC-A latches the peers only when they stop
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 500
# a dead node
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 750 --fault C:dropout:start=300
# several faults, different seeds
python3 -m tfc_peers run --follow-sync --nodes B,C --frames 750 --seed 3 \
    --fault B:corrupt:start=200,end=260,p=0.3 --fault C:drift:start=400,rate=0.1
# watch the bus while it happens (third terminal)
python3 -m tfc_peers listen --duration 10
```
When the peers finish their `--frames`, FC-A latches them out as `frame missing` within 3 frames: that is correct.

### Pointing it at real hardware later
With the USB-CAN adapter, the same commands work with `--iface can0` after `sudo ip link set can0 type can
bitrate 1000000 && sudo ip link set up can0`. This path is **untested**: the adapter has not arrived. Flight
computers on the Nucleo use FDCAN1 (see `firmware/README.md`).

### Troubleshooting
| Message or symptom | Cause and fix |
|---|---|
| `cannot open CAN interface 'vcan0'` | The interface does not exist. `./scripts/setup_vcan.sh` (it is gone after a reboot). |
| `no SYNC on the bus for 2.0 s: start the flight computer first` | `--follow-sync` needs FC-A running. Start `zephyr.exe` first, or drop `--follow-sync` if there is no flight computer. |
| `note: SYNC frame number went back ...` | FC-A restarted; the peers restarted their scenario too. Not an error. |
| FC-A latched itself or reports `vote=` / `digest=` counts with no fault | Peers are not following SYNC. Add `--follow-sync`. |
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
| `tests/test_replay.py` | Peers -> log -> **C++ `tfc_replay`**: every fault kind through the real `core/` code against the requirements |
| `tests/test_live_fc.py` | The **real FC-A firmware** against the peers on `vcan0`: healthy run with zero alarms, bias isolated at fault+2, digest and command faults labelled, silence within 3 frames |
| `tests/test_docs.py` | Fails if this README stops documenting a flag, fault kind or fault option |

`ctest --test-dir ../build/host` runs the same suite (`peers_e2e`). Replay tests skip if `tfc_replay` is not
built; live tests skip unless `vcan0` exists and `build/native_sim/zephyr/zephyr.exe` is built (or `TFC_FC_BIN`).

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

## Limits and what is not covered
- **Not real-time:** Python on desktop Linux gives no latency guarantee; the lateness it prints is for information.
- **Synthetic digest:** the peers' estimator-state digest is a function of the command and frame number (FC-A's
  simulated IMU computes the same function in C++ bit for bit). Checking a real estimator's digest needs M3.
- **Undefined payloads:** heartbeat (`0x400+n`) and actuator-output (`0x300`) frames are not defined in
  `protocol.hpp` yet, so the peers do not send them.
- **No reintegration through the CLI:** a latched node stays out in replays; the manager supports it
  (`request_reintegration`, 100 clean frames) and the unit tests cover it.
- **Bus effects:** arbitration, bus load, babbling's real harm, bus-off and wiring faults need hardware.
- **FC-A only:** the firmware implements node A, the sync master. Firmware for B and C and sync-master takeover
  (F15) are not built; until then B and C are always the virtual peers.
