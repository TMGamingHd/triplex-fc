# Flight-bus protocol, version 2

> Source of truth: `core/include/tfc/protocol.hpp`. The Python mirror is `sim/tfc_peers/protocol.py`; the golden bytes below are pinned in both
> (`tests/test_protocol_v2.cpp`, `sim/tests/test_protocol.py`), so a change on one side fails a test until both agree.
> Classic CAN, 1 Mbit/s. Every payload is **8 bytes: 6 data bytes, a sequence byte (the low byte of the frame number of the cycle, ADR-018), and a CRC-8**
> (SAE J1850, polynomial 0x1D). Multi-byte fields are little endian. A lower id wins arbitration.

## Identifiers
| Id | Name | From | Content | Version |
|---|---|---|---|---|
| `0x010` | SYNC | the sync master | 32-bit frame number and the 16-bit mission frame (0 = not launched, 1 to 1000 the countdown, 1001 T-zero, then flight; `docs/LAUNCH_SEQUENCE.md`) | 1, mission frame added (old senders put 0 there) |
| `0x100+n` | GYRO | node n | three axes, 0.125 dps per count | 1 |
| `0x110+n` | ACCEL | node n | three axes, 1/2048 g per count | 1 |
| `0x200+n` | CMD | node n | pitch and yaw gimbal command, 0.001 degree; a 16-bit state digest | 1 |
| `0x300` | ACT out | ACT | the voted gimbal command and the vote status | **2** |
| `0x400+n` | Heartbeat | node n | protocol version, mode, role, view of the nodes, reset count, release hash | **2** |
| `0x410+n` | State share | node n | strike counts and the last accepted command counter | **2** |
| `0x420+4n+k` | State resync | node n, chunk k (0-3) | a quarter of the node's estimator and controller state: three 16-bit words, once per resync period | **2** |
| `0x501` | Sim rates | the simulator | the sensor inputs' body rates, as a gyro frame | **2** |
| `0x502` | Sim accel | the simulator | the sensor inputs' acceleration, as an accel frame | **2** |
| `0x503` | Sim state | the simulator | altitude, speed, mass | **2** |
| `0x504` | Sim telemetry | the simulator | dynamic pressure, attitude error in the two planes | **2** |
| `0x505` | Sim flags | the simulator | flags, engines on, time | **2** |
| `0x510` | Ground command | the operator | authenticated opcode, node, tag, counter (ADR-019) | 1 |

**The simulator's range is `0x500` to `0x50F`** (`id::kSim` to `id::kSimLast`); the ground command is `0x510`. The fault manager treats the simulator's range, the state share
and the heartbeats as part of the schedule; every other id is **out-of-schedule traffic** and is counted, and three of them in one frame raise the bus alarm (FDIR-019, FDIR-027).
In version 1 only `0x500` was in the schedule; the range was widened for the simulator's frames, and the tests and the requirement were changed with it.

## Version 2 frames
**ACT out (`0x300`).** Bytes 0-1: the pitch-plane command; bytes 2-3: the yaw-plane command (0.001 degree, signed). Bytes 4-5, a 16-bit field:
bits 0-2 state (0 Standby, 1 Nominal, 2 Safe-hold, 3 Safe-ramp, 4 Safe-neutral); bit 3 held (no trustworthy vote this frame); bits 4-6 the pitch vote's status (`VoteStatus`);
bits 7-9 the nodes whose command took part (bit n = node n); bits 10-12 the nodes ACT has excluded; bits 13-15 the cause of Safe (0 none, 1 lost votes, 2 the flight computers' request, 3 the hardware line, 4 reset).
Example: pitch 1.234, yaw -5.678, Safe-ramp, held, Duplex, nodes A and C, B excluded, lost votes, seq 9: `d204d2e9ab2a092d`.

**Heartbeat (`0x400+n`).** Byte 0: protocol version. Byte 1: bits 0-1 mode (`Mode`), bit 2 Safe requested, bit 3 bus alarm, bits 4-5 role (0 hot, 1 warm, 2 cold), bit 6 quarantined (a reset loop), bit 7 ready for launch.
Byte 2: this node's view of A, B and C, two bits each (0 healthy, 1 latched, 2 probation, 3 disabled). Byte 3: the reset count since power-on, saturating. Bytes 4-5: the first 16 bits of the release's
source hash, so that a node on the golden release (ADR-021) can be told from one on the current release. Example (node B): `02563407efbe0496`.

**State share (`0x410+n`).** Byte 0: strikes of node A (low nibble) and B (high nibble); byte 1: strikes of node C; byte 2: the last accepted ground-command counter. This is what a restarted node needs from the others
to rebuild its strike record and close the counter gap (FDIR-041). Example: strikes 1, 15, 2 and counter 200: `f102c800000006f7`.

**State resync (`0x420 + 4n + k`).** Once per resync period (the last frame of it, after the command slot) each computer sends its quantised estimator and controller state in four frames of three signed 16-bit words, little-endian, bytes 0 to 5; byte 6 is the low byte of the frame number the state belongs to (a chunk of another cycle is not used), byte 7 the CRC. The twelve words: 0 to 3 the attitude quaternion (1/32767 per count, scalar part non-negative), 4 to 6 the gyro bias integrator (1e-5 rad/s per count), 7 and 8 the controller's integrators and 9 and 10 its last outputs (0.001 degree per count), 11 the low byte of the update count with *aligned* (bit 8) and *rates valid* (bit 9). Example, node B chunk 2 carrying 0x0102, -2, 0x7FFF for frame 0x55: id 0x426, data `0201feffff7f55e4` (`tests/test_resync.cpp` and `sim/tests/test_protocol.py` pin it). What the receivers do with it is in `docs/RESYNC.md`.

**Simulator frames.** `0x501` and `0x502` are the vehicle's sensor inputs for the frame, in the same scales as the gyro and accel frames (each node's simulated IMU adds its own noise and faults).
`0x503`: altitude (10 m per count), speed (1 m/s per count), mass (1 kg per count), each a 16-bit unsigned number that saturates. `0x504`: dynamic pressure (10 Pa per count, unsigned),
then the attitude error of the pitch and yaw planes (0.001 degree, signed). `0x505`: a flags byte (1 safed, 2 platform saturated, 4 engine out, 8 command held, 16 aborted), the engines on, and the
simulation time in 10 ms frames (32 bits). Their timing and use: `VEHICLE_SIM.md` section 6 .

**Ground commands** gained two operations: `launch` (5; always needs an ARM) and `scrub` (6; plain); the node field is ignored. Examples: SYNC frame 0x01020304, seq 9, mission 1001: `04030201e90309d2`; mission 65535, frame 0, seq 0: `00000000ffff0045`. Heartbeat B, mode 3, ready, resets 5, hash 0xBEEF, seq 4: `02830005efbe048c`.

## Rules
- A decoder checks the CRC and the id range, never trusts a field wider than its bits (a wider value is masked, not spilled into the next field: tested), and a node number outside 0 to 2 is not a node.
- A quantity that does not fit saturates; NaN becomes zero (an unsigned quantity: a negative number is zero too).
- A frame that does not decode is a bad frame and costs its sender one bad sample (ADR-007), exactly like a corrupted sensor frame.

## Not yet
The `noop` and `phase` ground opcodes (FDIR-043, PHASE-004), which touch the fault manager; the state share (`0x410+n`) and the release hash in the heartbeat (the flight computers send heartbeats since P1-4c,
with the release hash zero).
