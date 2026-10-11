# Flight-bus protocol, version 2

> Status: **built** (reviewed 5 Oct 2026; every frame below is implemented in C++ and in the Python mirror, and the golden examples are pinned in both). Source of truth: `core/include/tfc/protocol.hpp`. The Python mirror is `sim/tfc_peers/protocol.py`; the golden bytes below are pinned in both
> (`tests/test_protocol_v2.cpp`, `sim/tests/test_protocol.py`), so a change on one side fails a test until both agree.
> Classic CAN, 1 Mbit/s. Every payload is **8 bytes: 6 data bytes, a sequence byte (the low byte of the frame number of the cycle, ADR-018), and a CRC-8**
> (SAE J1850, polynomial 0x1D). Multi-byte fields are little endian. A lower id wins arbitration.

## Identifiers
| Id | Name | From | Content | Version |
|---|---|---|---|---|
| `0x010` | SYNC | the sync master | 32-bit frame number and the 16-bit mission frame (0 = not launched, 1 to 1000 the countdown, 1001 T-zero, then flight; `docs/design/LAUNCH_SEQUENCE.md`) | 1, mission frame added (old senders put 0 there) |
| `0x100+n` | GYRO | node n | three axes, **1/32 dps per count** (range +-1024 dps; it was 0.125 and +-4096 until 7 Oct 2026, ADR-032) | 1, scale changed |
| `0x110+n` | ACCEL | node n | three axes, 1/2048 g per count | 1 |
| `0x200+n` | CMD | node n | pitch and yaw gimbal command, 0.001 degree; a 16-bit state digest | 1 |
| `0x210+n` | Propulsion command | node n | throttle (0.5 % per count), the engine-group mask, the event bits, the mission phase, a roll command (0.25 degree per count); sent after CMD by a flight computer that flies a mission (`GNC.md` section 8) | 2, **host only so far** |
| `0x220+n` | Surface command | node n | four control-surface deflections, 12 bits each, 0.05 degree per count, signed | 2, **host only so far** |
| `0x300` | ACT out | ACT | the voted gimbal command and the vote status | **2** |
| `0x400+n` | Heartbeat | node n | protocol version, mode, role, view of the nodes, reset count, release hash | **2** |
| `0x410+n` | State share | node n | strike counts and the last accepted command counter | **2** |
| `0x420+4n+k` | State resync | node n, chunk k (0-3) | a quarter of the node's estimator and controller state: three 16-bit words, once per resync period | **2** |
| `0x501` | Sim rates | the simulator | the sensor inputs' body rates, as a gyro frame | **2** |
| `0x502` | Sim accel | the simulator | the sensor inputs' acceleration, as an accel frame | **2** |
| `0x503` | Sim state | the simulator | altitude, speed, mass | **2** |
| `0x504` | Sim telemetry | the simulator | dynamic pressure, attitude error in the two planes | **2** |
| `0x505` | Sim flags | the simulator | flags, engines on, time | **2** |
| `0x506`-`0x508` | Sim GNSS | the simulator | the GNSS fix in three frames (position, then velocity), each component a signed 24-bit number (metres; 0.01 m/s), in the launch-centred inertial frame (`GNC.md` section 3) | 2, **host only so far** |
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

**State resync (`0x420 + 4n + k`).** Once per resync period (the last frame of it, after the command slot) each computer sends its quantised estimator and controller state in four frames of three signed 16-bit words, little-endian, bytes 0 to 5; byte 6 is the low byte of the frame number the state belongs to (a chunk of another cycle is not used), byte 7 the CRC. The twelve words: 0 to 3 the attitude quaternion (1/32767 per count, scalar part non-negative), 4 to 6 the gyro bias integrator (1e-5 rad/s per count), 7 and 8 the controller's integrators and 9 and 10 its last outputs (0.001 degree per count), 11 the low byte of the update count with *aligned* (bit 8) and *rates valid* (bit 9). Example, node B chunk 2 carrying 0x0102, -2, 0x7FFF for frame 0x55: id 0x426, data `0201feffff7f55e4` (`tests/test_resync.cpp` and `sim/tests/test_protocol.py` pin it). What the receivers do with it is in `docs/design/RESYNC.md`.

**Simulator frames.** `0x501` and `0x502` are the vehicle's sensor inputs for the frame, in the same scales as the gyro and accel frames (each node's simulated IMU adds its own noise and faults).
`0x503`: altitude (10 m per count), speed (1 m/s per count), mass (1 kg per count), each a 16-bit unsigned number that saturates. `0x504`: dynamic pressure (10 Pa per count, unsigned),
then the attitude error of the pitch and yaw planes (0.001 degree, signed). `0x505`: a flags byte (1 safed, 2 platform saturated, 4 engine out, 8 command held, 16 aborted), the engines on, and the
simulation time in 10 ms frames (32 bits). Their timing and use: `VEHICLE_SIM.md` section 6 .

**The node field of a ground command** is 0 to 2 for a flight computer; **with the sensor split (ADR-020), 4 to 6 address IMU channel 0 to 2** for `reintegrate`, `disable` and `clear-disabled` (any other value is refused as a bad node, and without the split so is 4 to 6). The ARM code of a command is its operation and the whole node field, so an ARM for computer B does not cover IMU B.

**Ground commands.** The opcode byte carries the operation (low 7 bits) and the ARM flag (bit 7); the node field is as above except where stated:

| Op | Name | Node field | ARM |
|---|---|---|---|
| 1 | `reintegrate` | the computer (or, with the split, 4 to 6: an IMU channel) | never |
| 2 | `disable` | the computer or IMU | without phases: when it would leave fewer than two voters; with phases: when it would leave fewer than the phase's nominal number, and refused below its minimum |
| 3 | `clear-disabled` | the computer or IMU | always |
| 4 | `clear-safe` | ignored | always |
| 5 | `launch` | ignored | always (the sync master acts, after the go/no-go) |
| 6 | `scrub` | ignored | never; only before T-zero |
| 7 | `phase` | **the phase number 0 to 7**, not a node (docs/design/MISSION_PHASES.md) | never; refused if fewer computers vote than the new phase's minimum; needs `phases` on |
| 8 | `noop` | ignored | never; changes nothing and is answered like any command |
| 9 | `warm` | the computer (not an IMU) | as `disable`; rests a voting computer as WARM, `reintegrate` promotes it |

The answers (`accepted`, `already done`, `refused: <reason>`, with `RefusedPhase` and `RefusedNotHealthy` for the new operations) are in the frame report and on the console. Examples: SYNC frame 0x01020304, seq 9, mission 1001: `04030201e90309d2`; mission 65535, frame 0, seq 0: `00000000ffff0045`. Heartbeat B, mode 3, ready, resets 5, hash 0xBEEF, seq 4: `02830005efbe048c`.

## Rules
- A decoder checks the CRC and the id range, never trusts a field wider than its bits (a wider value is masked, not spilled into the next field: tested), and a node number outside 0 to 2 is not a node.
- A quantity that does not fit saturates; NaN becomes zero (an unsigned quantity: a negative number is zero too).
- A frame that does not decode is a bad frame and costs its sender one bad sample (ADR-007), exactly like a corrupted sensor frame.

## Not defined
Telemetry and event records in a fixed format and a machine-readable command dictionary (IF-004 to IF-006): today the report struct, the counters and the console lines carry the information (`docs/design/FUTURE_WORK.md` section 2.2). The supervisor's USB commands are a separate, human-typed interface (`docs/design/SUPERVISOR.md` section 6).

## Frames of a flight computer that flies a mission (host only so far)

`0x210+n` and `0x220+n` carry what the guidance asks for beyond the gimbal; a consumer (the vehicle) takes the **middle value of the three nodes'** throttle, roll and deflections and **two of three for each bit** of the group mask and the events (`mid3`, `majority3` in `protocol.hpp`), as ACT votes the gimbal. A frame whose CRC or sequence is wrong, or whose node is not 0 to 2, is refused (`unpack_prop`, `unpack_surf`). `0x506` to `0x508` are the receiver's three frames of one fix (they carry the same sequence byte), assembled by `nav::GnssCollector`. The ids are in `redundancy.hpp`'s list of the traffic that belongs to the schedule. **The firmware does not send or receive them yet**; the host loop (`avionics.hpp`) passes the commands in memory, so their quantisation is not in the results of `GNC.md` section 10. The Python mirror (`sim/tfc_peers/protocol.py`) does not know these ids.
