# SPDX-License-Identifier: MIT
"""What the console knows about the system that is not on the bus: names, the voter's tolerances, the timeouts it judges "alive" by.

The tolerances and windows are copies of `core/include/tfc/redundancy_types.hpp` and `act.hpp`; `sim/tests/test_console_constants.py` reads those headers and fails if a copy differs, so a change in
`core/` cannot leave the console drawing the wrong band.
"""
from __future__ import annotations

NODE_NAMES = ("A", "B", "C")
MODE_NAMES = {3: "Triplex", 2: "Duplex", 1: "Simplex", 0: "Safe"}   # fault_monitor.hpp, Mode
ROLE_NAMES = ("hot", "warm", "cold", "?")
STATE_NAMES_SAFE = ("Standby", "Nominal", "Safe-hold", "Safe-ramp", "Safe-neutral", "?", "?", "?")   # ACT's state field (PROTOCOL.md): 2 to 4 are the stages of Safe
VIEW_NAMES = ("healthy", "latched", "probation", "disabled")        # a node's view of a node (heartbeat byte 2)

# the eight voted channels: gyro x, y, z (dps), accelerometer x, y, z (g), the pitch and the yaw command (deg)
CHANNELS = (
    ("gyro X", "dps"), ("gyro Y", "dps"), ("gyro Z", "dps"),
    ("accel X", "g"), ("accel Y", "g"), ("accel Z", "g"),
    ("cmd pitch", "deg"), ("cmd yaw", "deg"),
)
VOTE_TOL = (1.0, 1.0, 1.0, 0.02, 0.02, 0.02, 0.01, 0.01)            # RedundancyConfig::tol
ACT_TOL_DEG = 0.05                                                  # ActConfig::tol_deg
ARM_WINDOW_FRAMES = 250                                             # RedundancyConfig::arm_window_frames: an ARM stays valid 2.5 s
COMMAND_WINDOW = 32                                                 # RedundancyConfig::command_window: a counter must be 1..32 ahead
PERSIST_M, PERSIST_N = 3, 5                                         # latch when 3 of the last 5 frames are bad
BUS_ALARM_PER_FRAME = 3                                             # out-of-schedule frames in one 10 ms frame
STRIKES_MAX, STRIKES_MAX_PHYSICAL = 3, 2

# seconds without a frame after which the console calls its sender silent (the launch checklist's limit, tfc_peers/launch.py MAX_AGE_S)
HEARTBEAT_TIMEOUT_S = 0.5
SYNC_TIMEOUT_S = 0.5
ACT_TIMEOUT_S = 0.5
SIM_TIMEOUT_S = 2.0          # the simulator's status frames come at 10 Hz
TRUTH_TIMEOUT_S = 2.0

FRAME_S = 0.01
MISSION_COUNTDOWN_FRAMES = 1000   # 1 to 1000 the countdown, 1001 T-zero (protocol.hpp, mission)

PHASE_NAMES = ("off", "power-up", "pre-launch", "ascent", "coast", "pre-burn", "burn", "safed")  # MISSION_PHASES.md
PHASE_RULES = ((0, 0), (3, 1), (3, 3), (3, 2), (2, 2), (3, 3), (3, 2), (3, 1))                    # (nominal, minimum) HOT computers per phase

# the ids of the flight bus (docs/design/PROTOCOL.md), for the bus table
SCHEDULE = (
    (0x010, "SYNC", "the sync master: frame number and mission frame"),
    (0x100, "GYRO", "node 0-2: three axes, 1/32 dps per count"),
    (0x110, "ACCEL", "node 0-2: three axes, 1/2048 g per count"),
    (0x200, "CMD", "node 0-2: pitch and yaw command, state digest"),
    (0x300, "ACT", "ACT: the voted command and the vote status"),
    (0x400, "HEARTBEAT", "node 0-2: mode, role, view of A, B, C, release"),
    (0x410, "STATE", "node 0-2: strikes, last command counter"),
    (0x420, "RESYNC", "node 0-2, chunk 0-3: a quarter of the estimator and controller state"),
    (0x501, "SIM_RATES", "simulator: body rates the IMUs feel"),
    (0x502, "SIM_ACCEL", "simulator: acceleration the IMUs feel"),
    (0x503, "SIM_STATE", "simulator: altitude, speed, mass"),
    (0x504, "SIM_TELEM", "simulator: dynamic pressure, attitude error"),
    (0x505, "SIM_FLAGS", "simulator: flags, engines on, time"),
    (0x510, "GROUND", "the operator: an authenticated command"),
)


def expected_hz(can_id: int) -> float | None:
    """How often an id should be on the bus (docs/design/PROTOCOL.md and ARCHITECTURE.md: the 100 Hz frame, the state share and the simulator's status at 10 Hz, one resync frame per node chunk per second), or None for an id with no fixed rate."""
    if can_id == 0x010 or can_id == 0x300 or can_id in (0x501, 0x502):
        return 100.0
    if any(base <= can_id <= base + 2 for base in (0x100, 0x110, 0x200, 0x400)):
        return 100.0
    if 0x410 <= can_id <= 0x412 or can_id in (0x503, 0x504, 0x505):
        return 10.0
    if 0x420 <= can_id < 0x42C:
        return 1.0
    return None


def id_name(can_id: int) -> str:
    """A short name for a bus id: `GYRO B`, `SYNC`, `RESYNC A.2`, or `?0x123` for an id outside the schedule."""
    if can_id == 0x010:
        return "SYNC"
    for base, name in ((0x100, "GYRO"), (0x110, "ACCEL"), (0x200, "CMD"), (0x400, "HEARTBEAT"), (0x410, "STATE")):
        if base <= can_id <= base + 2:
            return f"{name} {NODE_NAMES[can_id - base]}"
    if 0x420 <= can_id < 0x42C:
        k = can_id - 0x420
        return f"RESYNC {NODE_NAMES[k // 4]}.{k % 4}"
    named = {0x300: "ACT", 0x501: "SIM_RATES", 0x502: "SIM_ACCEL", 0x503: "SIM_STATE", 0x504: "SIM_TELEM", 0x505: "SIM_FLAGS", 0x510: "GROUND"}
    if can_id in named:
        return named[can_id]
    return f"SIM {can_id:#05x}" if 0x500 <= can_id <= 0x50F else f"?{can_id:#05x}"


def in_schedule(can_id: int) -> bool:
    """The ids the fault manager does not count as out-of-schedule traffic (core: the schedule, the state share, the heartbeats, the simulator's range 0x500 to 0x50F); the ground command is judged apart."""
    return id_name(can_id)[0] != "?"
