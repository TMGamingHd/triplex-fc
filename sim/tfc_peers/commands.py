# SPDX-License-Identifier: MIT
"""Scripted operator commands: `FRAME:OP[:NODE]`, e.g. `450:reintegrate:B` or `600:clear-safe`.

They are sent as authenticated ground-command frames (core protocol.hpp, id 0x510, ADR-019) in frame FRAME of the scenario,
6.5 ms in (before the 7 ms vote), so a flight computer applies them in that very frame. OP may be prefixed:
  arm-OP     only the ARM frame (the flight computer then waits for the EXECUTE)
  armed-OP   the whole two-step: ARM in frame FRAME, EXECUTE two frames later (what clear-safe, clear-disabled and a disable
             that would leave fewer than two healthy nodes need)
  forged-OP  a frame with a wrong tag (what a node without the key sends): refused without a trace
and `FRAME:replay` re-sends the previous command frame unchanged (a stale counter): refused as a replay.
Counters are assigned in time order, 1, 2, 3, ..., one per frame sent.
"""
from __future__ import annotations

from dataclasses import dataclass

from . import protocol as P
from .faults import FaultSpecError, parse_node

COMMAND_SEND_US = 6500
ARMED_GAP_FRAMES = 2


@dataclass(frozen=True)
class GroundCommand:
    frame: int
    op: str
    node: int = 0
    arm: bool = False
    forged: bool = False
    replay: bool = False

    def __str__(self) -> str:
        if self.replay:
            return f"{self.frame}:replay"
        who = "" if self.op == "clear-safe" else f":{P.NODE_NAMES[self.node]}"
        prefix = ("arm-" if self.arm else "") + ("forged-" if self.forged else "")
        return f"{self.frame}:{prefix}{self.op}{who}"

    def frame_for(self, counter: int, key: bytes | None = None) -> P.Frame:
        return P.pack_ground(P.GROUND_OPS[self.op], self.node, counter, key, self.arm, self.forged)


def parse_commands(spec: str) -> list[GroundCommand]:
    """One spec, one or two commands (`armed-OP` is an ARM and, ARMED_GAP_FRAMES later, its EXECUTE)."""
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise FaultSpecError(f"bad command {spec!r}: expected FRAME:OP[:NODE], e.g. 450:reintegrate:B")
    try:
        frame = int(parts[0])
    except ValueError:
        raise FaultSpecError(f"bad command {spec!r}: FRAME must be a frame number") from None
    if frame < 0:
        raise FaultSpecError(f"bad command {spec!r}: FRAME must be >= 0")
    op = parts[1].strip().lower()
    if op == "replay":
        if len(parts) != 2:
            raise FaultSpecError(f"bad command {spec!r}: replay takes no node")
        return [GroundCommand(frame, "reintegrate", 0, replay=True)]
    armed = forged = arm_only = False
    for _ in range(3):
        for prefix in ("armed-", "arm-", "forged-"):
            if op.startswith(prefix):
                op = op[len(prefix):]
                armed |= prefix == "armed-"
                arm_only |= prefix == "arm-"
                forged |= prefix == "forged-"
    if op not in P.GROUND_OPS:
        raise FaultSpecError(f"unknown command {op!r}; known: {', '.join(P.GROUND_OPS)} (optionally prefixed arm-, armed- or forged-)")
    if op == "clear-safe":
        node = 0
    else:
        if len(parts) != 3:
            raise FaultSpecError(f"command {op!r} needs a node: FRAME:{op}:A|B|C")
        node = parse_node(parts[2])
    if armed:
        return [GroundCommand(frame, op, node, arm=True, forged=forged), GroundCommand(frame + ARMED_GAP_FRAMES, op, node, forged=forged)]
    return [GroundCommand(frame, op, node, arm=arm_only, forged=forged)]


def parse_command(spec: str) -> GroundCommand:
    """A spec that is exactly one frame (everything except `armed-OP`)."""
    cmds = parse_commands(spec)
    if len(cmds) != 1:
        raise FaultSpecError(f"{spec!r} is a two-frame command; use parse_commands")
    return cmds[0]
