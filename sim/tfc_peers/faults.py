# SPDX-License-Identifier: MIT
"""Fault specifications for virtual flight computers.

A fault is written `NODE:KIND[:key=value,...]`, e.g. `B:bias:start=100,mag=3`.
`start`/`end` are 10 ms frame indices (`end` is exclusive, default: forever). Each kind maps
to a row of docs/FAULT_MATRIX.md so a campaign can be described the same way as the matrix.
Kinds F27 onward come from the FMEA gap analysis (docs/FMEA.md); every parameter is validated here so a typo
cannot silently become "no fault" or crash the generator half way through a run.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Union

from . import protocol as P
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
    "babble": ("F11", "`n` extra frames per cycle on out-of-schedule ids `id` .. `id`+15 (default 0x020, above every sensor id in priority)",
               {"n": 5, "id": 32}),
    "seqgap": ("IF", "sequence counter jumps ahead by `gap` once, at `start`", {"gap": 3}),
    "reboot": ("F24", "node restarts: silent for `down` frames, then back with its sequence counter restarted at 0",
               {"down": 50}),
    "late": ("F25", "every scheduled frame arrives `us` microseconds late (after the 7 ms vote it is stale data)",
             {"us": 4000}),
    # ---- kinds added by the FMEA gap analysis (docs/FMEA.md) ----
    "scale": ("F27", "scale-factor error: one axis multiplied by `factor`",
              {"sensor": "gyro", "axis": 0, "factor": 1.2}),
    "noise": ("F28", "noise grows to `mult` times the healthy sigma on all axes of one sensor",
              {"sensor": "gyro", "mult": 20.0}),
    "invert": ("F29", "sign inversion of one axis (wrong mounting or polarity)",
               {"sensor": "gyro", "axis": 0}),
    "swap": ("F30", "two axes exchanged (misalignment of 90 degrees)",
             {"sensor": "gyro", "axis": 0, "other": 1}),
    "zero": ("F31", "sensor reads exactly zero on all axes (dead sensor, lost supply)", {}),
    "clip": ("F32", "output clipped at +-`limit` (wrong full-scale range selected)",
             {"sensor": "gyro", "limit": 4.0}),
    "oscillate": ("F33", "sinusoid of amplitude `amp` at `hz` added to one axis (vibration, aliasing)",
                  {"sensor": "gyro", "axis": 0, "amp": 3.0, "hz": 33.0}),
    "repeat": ("F34", "sample refreshed only every `n` frames, held in between (output data rate mismatch)",
               {"n": 3}),
    "bitflip": ("F35", "single-event upset: bit `bit` of one axis' 16-bit sample flipped with probability `p`, "
                       "before the CRC is computed",
                {"sensor": "gyro", "bit": 12, "p": 0.1}),
    "stuckbit": ("F36", "bit `bit` of every 16-bit sample forced to `value` (stuck data line)",
                 {"sensor": "gyro", "bit": 12, "value": 1}),
    "cmdstuck": ("F37", "command output frozen at its last value while the node otherwise runs", {}),
    "cmdinvert": ("F38", "both command axes sent with the wrong sign (software or wiring bug)", {}),
    "partial": ("F39", "only some frame types are sent: `mask` bits 1=gyro 2=accel 4=cmd are suppressed",
                {"mask": 4}),
    "duplicate": ("F40", "every scheduled frame is sent twice, the copy `gap_us` microseconds later",
                  {"gap_us": 300}),
    "replay": ("F41", "frames from `age` frames ago are re-sent in place of the current ones (valid CRC, old counter)",
               {"age": 10}),
    "seqstuck": ("F42", "sequence counter frozen at its value when the fault began", {}),
    "early": ("F43", "every scheduled frame leaves `us` microseconds early (before its slot, possibly before SYNC)",
              {"us": 2000}),
    "jitter": ("F44", "every frame leaves up to +-`us` microseconds from its slot (random, deterministic by seed)",
               {"us": 1500}),
    "clockdrift": ("F45", "local clock drifts: frames leave `us_per_frame` microseconds later each frame, cumulative",
                   {"us_per_frame": 20}),
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
    for conv in (lambda t: int(t, 0), float):  # int(t, 0) also accepts 0x520
        try:
            return conv(text)
        except ValueError:
            pass
    return text


def _num(params: dict[str, Value], key: str, spec: str) -> float:
    v = params[key]
    if isinstance(v, str) or not math.isfinite(float(v)):
        raise FaultSpecError(f"in {spec!r}: option {key!r} must be a finite number, got {v!r}")
    return float(v)


def _validate(kind: str, params: dict[str, Value], spec: str) -> None:
    """Reject every parameter combination that would be a typo, a crash or a silent no-op."""
    defaults = KINDS[kind][2]
    for key, default in defaults.items():
        if isinstance(default, str):
            continue
        x = _num(params, key, spec)
        if isinstance(default, int):
            if x != int(x):
                raise FaultSpecError(f"in {spec!r}: option {key!r} must be a whole number, got {params[key]!r}")
            params[key] = int(x)
        else:
            params[key] = x
    if params.get("sensor", "gyro") not in ("gyro", "accel"):
        raise FaultSpecError("sensor must be gyro or accel")
    if int(params.get("axis", 0)) not in (0, 1, 2) or int(params.get("other", 1)) not in (0, 1, 2):
        raise FaultSpecError("axis must be 0, 1 or 2")
    rules: dict[str, tuple[bool, str]] = {
        "reboot": (int(params.get("down", 1)) >= 1, "reboot needs down >= 1"),
        "late": (0 < int(params.get("us", 1)) <= 9000,
                 "late needs 0 < us <= 9000 (frames must still leave inside the 10 ms frame)"),
        "early": (0 < int(params.get("us", 1)) <= 9000, "early needs 0 < us <= 9000"),
        "jitter": (0 < int(params.get("us", 1)) <= 9000, "jitter needs 0 < us <= 9000"),
        "clockdrift": (params.get("us_per_frame", 1) != 0 and abs(int(params.get("us_per_frame", 1))) <= 1000,
                       "clockdrift needs 0 < |us_per_frame| <= 1000"),
        "duplicate": (0 <= int(params.get("gap_us", 0)) <= 9000, "duplicate needs 0 <= gap_us <= 9000"),
        "replay": (1 <= int(params.get("age", 1)) <= 300, "replay needs 1 <= age <= 300"),
        "repeat": (int(params.get("n", 2)) >= 2, "repeat needs n >= 2"),
        "partial": (1 <= int(params.get("mask", 1)) <= 7, "partial needs 1 <= mask <= 7"),
        "bitflip": (0 <= int(params.get("bit", 0)) <= 15 and 0.0 <= float(params.get("p", 0)) <= 1.0,
                    "bitflip needs 0 <= bit <= 15 and 0 <= p <= 1"),
        "stuckbit": (0 <= int(params.get("bit", 0)) <= 15 and int(params.get("value", 0)) in (0, 1),
                     "stuckbit needs 0 <= bit <= 15 and value 0 or 1"),
        "noise": (float(params.get("mult", 1)) >= 1.0, "noise needs mult >= 1"),
        "clip": (float(params.get("limit", 1)) > 0.0, "clip needs limit > 0"),
        "oscillate": (float(params.get("hz", 1)) > 0.0 and float(params.get("amp", 0)) >= 0.0,
                      "oscillate needs hz > 0 and amp >= 0"),
        "swap": (int(params.get("axis", 0)) != int(params.get("other", 1)), "swap needs two different axes"),
        "spike": (0.0 <= float(params.get("p", 0)) <= 1.0, "spike needs 0 <= p <= 1"),
        "corrupt": (0.0 <= float(params.get("p", 0)) <= 1.0, "corrupt needs 0 <= p <= 1"),
        "babble": (0 <= int(params.get("n", 1)) <= 200, "babble needs 0 <= n <= 200"),
    }
    ok, message = rules.get(kind, (True, ""))
    if not ok:
        raise FaultSpecError(f"in {spec!r}: {message}")
    if kind == "babble":
        lo = int(params["id"])
        if not 0 <= lo <= 0x7F0:
            raise FaultSpecError(f"in {spec!r}: babble needs 0 <= id <= 0x7F0 (an 11-bit CAN id, 16 ids from `id`)")
        scheduled = {P.ID_SYNC, P.ID_ACT_OUT, P.ID_SIM, P.ID_GROUND} | {b + n for b in (P.ID_GYRO_BASE, P.ID_ACCEL_BASE, P.ID_CMD_BASE, P.ID_HEARTBEAT) for n in range(3)}
        if scheduled & set(range(lo, lo + 16)):
            raise FaultSpecError(f"in {spec!r}: babble ids {lo:#x}-{lo + 15:#x} overlap the flight-bus schedule; "
                                 "babble is out-of-schedule traffic (forging a scheduled frame is not what this fault models)")


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
    _validate(kind, params, spec)
    return Fault(node, kind, start, end, params, period, duty)
