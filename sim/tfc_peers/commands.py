# SPDX-License-Identifier: MIT
"""Scripted operator commands: `FRAME:OP[:NODE]`, e.g. `450:reintegrate:B` or `600:clear-safe`.

They are sent as ground-command frames (core protocol.hpp, id 0x510) in frame FRAME of the scenario,
6.5 ms in (before the 7 ms vote), so a flight computer applies them in that very frame.
"""
from __future__ import annotations

from dataclasses import dataclass

from . import protocol as P
from .faults import FaultSpecError, parse_node

COMMAND_SEND_US = 6500


@dataclass(frozen=True)
class GroundCommand:
    frame: int
    op: str
    node: int = 0

    def __str__(self) -> str:
        who = "" if self.op == "clear-safe" else f":{P.NODE_NAMES[self.node]}"
        return f"{self.frame}:{self.op}{who}"

    def frame_for(self, seq: int) -> P.Frame:
        return P.pack_ground(P.GROUND_OPS[self.op], self.node, seq)


def parse_command(spec: str) -> GroundCommand:
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise FaultSpecError(f"bad command {spec!r}: expected FRAME:OP[:NODE], e.g. 450:reintegrate:B")
    try:
        frame = int(parts[0])
    except ValueError:
        raise FaultSpecError(f"bad command {spec!r}: FRAME must be a frame number") from None
    op = parts[1].strip().lower()
    if op not in P.GROUND_OPS:
        raise FaultSpecError(f"unknown command {op!r}; known: {', '.join(P.GROUND_OPS)}")
    if frame < 0:
        raise FaultSpecError(f"bad command {spec!r}: FRAME must be >= 0")
    if op == "clear-safe":
        return GroundCommand(frame, op, 0)
    if len(parts) != 3:
        raise FaultSpecError(f"command {op!r} needs a node: FRAME:{op}:A|B|C")
    return GroundCommand(frame, op, parse_node(parts[2]))
