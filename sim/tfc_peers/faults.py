# SPDX-License-Identifier: MIT
"""Fault specifications for virtual flight computers.

A fault is written `NODE:KIND[:key=value,...]`, e.g. `B:bias:start=100,mag=3`.
`start`/`end` are 10 ms frame indices (`end` is exclusive, default: forever). Each kind maps
to a row of docs/FAULT_MATRIX.md so a campaign can be described the same way as the matrix.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Union

from .protocol import NODE_NAMES

Value = Union[int, float, str]

# kind -> (fault-matrix row, description, default parameters)
KINDS: dict[str, tuple[str, str, dict[str, Value]]] = {
    "dropout": ("F01", "node goes fail-silent: sends no frames", {}),
    "stuck": ("F03", "sensor outputs freeze at their last good value", {}),
    "bias": ("F04", "constant offset added to one sensor axis",
             {"sensor": "gyro", "axis": 0, "mag": 3.0}),
    "drift": ("F05", "offset that grows by `rate` per frame on one sensor axis",
              {"sensor": "gyro", "axis": 0, "rate": 0.05}),
    "spike": ("F06", "random single-frame outliers of +-`mag` on one axis with probability `p`",
              {"sensor": "gyro", "mag": 20.0, "p": 0.05}),
    "saturate": ("F07", "sensor output pinned to full scale, alternating sign", {}),
    "corrupt": ("F08", "flip one random payload bit in a frame with probability `p`",
                {"p": 0.2}),
    "cmd_offset": ("F09", "wrong-but-valid command: pitch offset by `mag` degrees",
                   {"mag": 1.0}),
    "digest": ("F10", "estimator-state digest diverges (XOR with `xor`)", {"xor": 1}),
    "babble": ("F11", "`n` extra frames per cycle on out-of-schedule high-priority ids",
               {"n": 5}),
    "seqgap": ("IF", "sequence counter jumps ahead by `gap` once, at `start`", {"gap": 3}),
    "reboot": ("F24", "node restarts: silent for `down` frames, then back with its sequence counter restarted at 0",
               {"down": 50}),
    "late": ("F25", "every scheduled frame arrives `us` microseconds late (after the 7 ms vote it is stale data)",
             {"us": 4000}),
}


class FaultSpecError(ValueError):
    pass


@dataclass
class Fault:
    node: int
    kind: str
    start: int = 0
    end: int | None = None
    params: dict[str, Value] = field(default_factory=dict)
    period: int | None = None  # intermittent: active `duty` frames out of every `period`, from `start`
    duty: int | None = None

    def active(self, k: int) -> bool:
        if k < self.start or (self.end is not None and k >= self.end):
            return False
        if self.period is None:
            return True
        return (k - self.start) % self.period < int(self.duty or 1)

    def __str__(self) -> str:
        extra = ",".join(f"{k}={v}" for k, v in self.params.items())
        window = f"start={self.start}" + (f",end={self.end}" if self.end is not None else "")
        if self.period is not None:
            window += f",period={self.period},duty={self.duty}"
        return f"{NODE_NAMES[self.node]}:{self.kind}:{window}" + (f",{extra}" if extra else "")


def parse_node(text: str) -> int:
    t = text.strip().upper()
    if t in tuple(NODE_NAMES):
        return NODE_NAMES.index(t)
    if t in ("0", "1", "2"):
        return int(t)
    raise FaultSpecError(f"unknown node {text!r} (use A, B or C)")


def _coerce(text: str) -> Value:
    for conv in (int, float):
        try:
            return conv(text)
        except ValueError:
            pass
    return text


def parse_fault(spec: str) -> Fault:
    parts = spec.split(":")
    if len(parts) not in (2, 3):
        raise FaultSpecError(f"bad fault {spec!r}: expected NODE:KIND[:key=value,...]")
    node = parse_node(parts[0])
    kind = parts[1].strip().lower()
    if kind not in KINDS:
        raise FaultSpecError(f"unknown fault kind {kind!r}; known: {', '.join(KINDS)}")
    params = dict(KINDS[kind][2])
    start, end, period, duty = 0, None, None, None
    if len(parts) == 3 and parts[2].strip():
        for item in parts[2].split(","):
            if "=" not in item:
                raise FaultSpecError(f"bad option {item!r} in {spec!r}: expected key=value")
            key, val = (s.strip() for s in item.split("=", 1))
            if key == "start":
                start = int(val)
            elif key == "end":
                end = int(val)
            elif key == "period":
                period = int(val)
            elif key == "duty":
                duty = int(val)
            elif key in params:
                params[key] = _coerce(val)
            else:
                allowed = ["start", "end", "period", "duty", *params]
                raise FaultSpecError(f"{kind} has no option {key!r}; allowed: {', '.join(allowed)}")
    if start < 0 or (end is not None and end <= start):
        raise FaultSpecError(f"bad window in {spec!r}: need 0 <= start < end")
    if (period is None) != (duty is None) and not (period is not None and duty is None):
        raise FaultSpecError(f"in {spec!r}: duty needs period")
    if period is not None:
        duty = 1 if duty is None else duty
        if period < 2 or not 1 <= duty < period:
            raise FaultSpecError(f"bad intermittent pattern in {spec!r}: need period >= 2 and 1 <= duty < period")
    if kind == "reboot" and int(params["down"]) < 1:
        raise FaultSpecError("reboot needs down >= 1")
    if kind == "late" and not 0 < int(params["us"]) <= 9000:
        raise FaultSpecError("late needs 0 < us <= 9000 (frames must still leave inside the 10 ms frame)")
    if params.get("sensor", "gyro") not in ("gyro", "accel"):
        raise FaultSpecError("sensor must be gyro or accel")
    if int(params.get("axis", 0)) not in (0, 1, 2):
        raise FaultSpecError("axis must be 0, 1 or 2")
    return Fault(node, kind, start, end, params, period, duty)
