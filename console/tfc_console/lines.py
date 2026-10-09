# SPDX-License-Identifier: MIT
"""Reading what a node says on its console.

A flight computer, ACT or the supervisor prints a line for every event and a status line every 100 frames (`firmware/app/src/main.cpp`, `firmware/act/src/main.cpp`,
`firmware/supervisor/src/main.cpp`). Only the console carries the *reasons* (why a node was latched out, what a ground command answered), because the bus carries the outcome and not the cause.
`parse_line` turns one line into an event: a kind, a level, the node it is about, and for a status line the counters. A line it does not know is kept as an `info` text, never dropped:
firmware changes, and a console that hid what it could not parse would be hiding exactly the line that matters.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

_FRAME = re.compile(r"^\[(?:frame|cycle) (\d+)\]\s*(.*)$")
_STATUS = re.compile(r"^(TRIPLEX|DUPLEX|SIMPLEX|SAFE)\s+A(\S) B(\S) C(\S)\s*\|\s*(.*)$")
_ACT_STATUS = re.compile(r"^(STANDBY|NOMINAL|SAFE)\s+(\S+)\s+pitch (-?[\d.]+) yaw (-?[\d.]+)\s+voted (\d+) excluded (\d+) held (\d+)\s*\|\s*(.*)$")
_KV = re.compile(r"([a-z_]+)=(\S+)")
_NODE = re.compile(r"\bnode ([ABC])\b")
_IMU = re.compile(r"\bIMU ([ABC])\b")
_GROUND = re.compile(r"^GROUND COMMAND (ARM )?([a-z-]+)(?: ([ABC]|P\d))?: (.*)$")
_ACT_GROUND = re.compile(r"^GROUND ([a-z-]+): (.*)$")
_ACT_FRAME = re.compile(r"^GROUND command (.*)$")        # ACT's view of a ground frame that was not a command for it: damaged, unauthentic, replayed, ARMed...

# the letter the status line prints for each node state (firmware/app/src/main.cpp: node_state)
STATUS_LETTERS = {"+": "voting", "X": "latched", "p": "probation", "w": "warm", "D": "disabled", "?": "no data"}


@dataclass
class Parsed:
    kind: str
    level: str                      # info, ok, warn or crit
    text: str
    frame: int | None = None
    node: str | None = None         # the node the line is about (A, B, C), when it names one
    fields: dict[str, object] = field(default_factory=dict)


def _status_fields(tail: str) -> dict[str, object]:
    out: dict[str, object] = {}
    for key, val in _KV.findall(tail):
        if "/" in val:                                  # resync=3/0: adopted/skipped
            a, _, b = val.partition("/")
            if a.isdigit() and b.isdigit():
                out["resync_adopted"], out["resync_skipped"] = int(a), int(b)
            continue
        try:
            out[key] = int(val, 0)
        except ValueError:
            out[key] = val
    return out


def up_boot(body: str) -> bool:
    """The lines a node prints once at start: its boot count and reset cause, its release, which node it is, the legend of the status line."""
    return body.upper().startswith(("BOOT ", "RELEASE ", "FC-", "ACT BOOT", "ACT: FOLLOWING", "STATUS:", "*** BOOTING", "ZEPHYR"))


def parse_line(line: str) -> Parsed | None:
    """One console line as an event, or None for an empty line."""
    text = line.rstrip("\r\n").strip()
    if not text:
        return None
    m = _FRAME.match(text)
    frame = int(m.group(1)) if m else None
    body = m.group(2) if m else text

    s = _STATUS.match(body)
    if s:
        letters = {n: s.group(2 + i) for i, n in enumerate("ABC")}
        fields = _status_fields(s.group(5))
        fields["mode"] = s.group(1).lower()
        fields["view"] = {n: STATUS_LETTERS.get(c, c) for n, c in letters.items()}
        return Parsed("status", "info", body, frame, None, fields)
    a = _ACT_STATUS.match(body)
    if a:
        fields = {"mode": a.group(1).lower(), "phase": a.group(2), "pitch": float(a.group(3)), "yaw": float(a.group(4)), "voted": int(a.group(5)), "excluded": int(a.group(6)), "held": int(a.group(7))}
        for key, val in re.findall(r"([a-z_]+) (\d+)", a.group(8)):
            fields[key] = int(val)
        return Parsed("act-status", "info", body, frame, None, fields)

    g = _GROUND.match(body)
    if g:
        arm, op, who, result = g.groups()
        level = "ok" if result.startswith(("accepted", "already")) else "warn" if result.startswith("refused") else "info"
        return Parsed("ground-command", level, body, frame, who if who in ("A", "B", "C") else None,
                      {"op": op, "arm": bool(arm), "target": who, "result": result})
    ag = _ACT_GROUND.match(body)
    if ag:
        level = "warn" if "REFUSED" in ag.group(2) or "refused" in ag.group(2) else "ok"
        return Parsed("ground-command", level, body, frame, None, {"op": ag.group(1), "arm": False, "target": None, "result": ag.group(2), "by": "ACT"})

    if up_boot(body):  # before the rules below: a boot line lists "watchdog" and "probation" as words, and is not an event
        return Parsed("boot", "info", body, frame)
    af = _ACT_FRAME.match(body)
    if af:
        what = af.group(1)
        return Parsed("act-ground-frame", "warn" if what in ("damaged", "unauthentic", "replayed", "needs an ARM first") else "info", body, frame, None, {"result": what})
    node = (_NODE.search(body) or _IMU.search(body))
    who = node.group(1) if node else None
    up = body.upper()
    # (kind, level) by the words the firmware uses; the order matters: the most specific first
    rules = (
        ("CRITICAL", "critical", "crit"), ("RESET LOOP", "reset-loop", "crit"), ("SAFE REQUESTED", "safe-request", "crit"), ("WATCHDOG", "watchdog", "crit"),
        ("FAILED PROBATION", "probation-failed", "warn"), ("ON PROBATION", "probation", "info"), ("REINTEGRATED", "reintegrated", "ok"),
        ("LATCHED OUT", "latched-out", "warn"), ("DISABLED", "disabled", "warn"), ("JOINED THE BUS", "joined", "ok"),
        ("BUS ALARM RAISED", "bus-alarm", "warn"), ("BUS ALARM CLEARED", "bus-alarm-cleared", "ok"), ("INTEGRITY FAULT", "integrity", "warn"),
        ("LAUNCH REFUSED", "launch-refused", "warn"), ("SCRUB REFUSED", "scrub-refused", "warn"), ("COUNTDOWN SCRUBBED", "scrubbed", "warn"), ("SCRUB", "scrub", "warn"),
        ("T0 LINE", "t0-line", "info"), ("LAUNCH: GO", "launch-go", "ok"), ("LAUNCH: ACCEPTED", "launch-accepted", "ok"), ("T-ZERO", "t-zero", "ok"),
        ("COUNTDOWN", "countdown", "info"), ("RESYNC", "resync", "info"), ("STATE RESTORED", "state-restored", "info"),
        ("TAKES OVER AS SYNC MASTER", "sync-master", "warn"), ("ANOTHER NODE IS SENDING SYNC", "sync-follow", "info"),
        ("EXCLUDED NODE", "act-excluded", "warn"), ("READMITTED NODE", "act-readmitted", "ok"),
    )
    for needle, kind, level in rules:
        if needle in up:
            if kind == "resync" and ("NOTHING ADOPTED" in up):
                level = "warn"
            fields: dict[str, object] = {}
            if kind in ("latched-out", "disabled", "probation-failed", "launch-refused", "scrubbed") and ": " in body:
                fields["reason"] = body.split(": ", 1)[1]
            return Parsed(kind, level, body, frame, who, fields)
    mm = re.match(r"^(?:ACT )?(?:MODE )?([A-Z-]+) -> ([A-Z-]+)", body)
    if mm and ("MODE" in up or up.startswith("ACT ")):
        to = mm.group(2)
        level = "ok" if to in ("TRIPLEX", "NOMINAL") else "warn" if to in ("DUPLEX", "SIMPLEX") else "crit" if to == "SAFE" else "info"
        return Parsed("mode-change", level, body, frame, None, {"from": mm.group(1), "to": mm.group(2)})
    if re.match(r"^T-\d+ s$", body):
        return Parsed("countdown", "info", body, frame)
    if re.match(r"^(RESET|POWER-CYCLE|DEAD|PERIOD|MISSION TIME|OVERRIDE|OVERRIDES)\b", body, re.I):  # the supervisor
        level = "crit" if body.startswith("DEAD") else "warn"
        return Parsed("supervisor", level, body, frame, None)
    return Parsed("text", "info", body, frame, who)
