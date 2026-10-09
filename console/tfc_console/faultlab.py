# SPDX-License-Identifier: MIT
"""Injecting faults: the virtual peers of the fault lab take their faults from the console while they run (`tfc_peers run --control`, `tfc_peers/control.py`).

The console never builds the faulty traffic itself; it asks the peers' process to add or clear a fault, in the peers' own spec language (`B:bias:mag=3`), and reads the answer line. That keeps the fault engine in
one place (`tfc_peers/faults.py`, mutation-tested through the campaign) and the console a client of it, and keeps the timing-critical frame generation out of the web server's process.

`EXPECT` is what the project says each fault should look like to the flight computers (`sim/README.md`, "What to expect": 400 frames, a fault at frame 100, node B, seed 1): the page shows it next to the live
reaction, so the operator sees whether the system did what the matrix says. It is the documentation's table, not a measurement of this run; `sim/tests/test_console_faultlab.py` checks it covers every fault kind.
"""
from __future__ import annotations

import queue
import threading
from collections import deque

from tfc_peers.faults import FaultSpecError, parse_fault

from .hub import Hub
from .rig import Rig

# kind -> (what catches it, what the mode is afterwards)
EXPECT: dict[str, tuple[str, str]] = {
    "dropout": ("missing frames, about 2 frames after", "Duplex"), "stuck": ("the vote (the stuck detector is the backup, after 20 frames)", "Duplex"),
    "bias": ("the vote, about 2 frames after", "Duplex"), "drift": ("the vote, once the offset passes the 1 dps tolerance", "Duplex"),
    "spike": ("the vote flags each spike; the 3-of-5 filter rejects them, so nobody is latched", "Triplex"), "saturate": ("the vote", "Duplex"),
    "corrupt": ("CRC failures piling up (seed-dependent: 2 to 26 frames)", "Duplex"), "cmd_offset": ("the vote on the command", "Duplex"), "digest": ("the digest cross-check", "Duplex"),
    "babble": ("out-of-schedule frames are counted and the bus alarm is raised; nobody is blamed", "Triplex"), "seqgap": ("sequence errors (every frame is out of phase)", "Duplex"),
    "reboot": ("missing frames; the node returns in phase and is readmitted when the operator asks", "Duplex, then Triplex"), "late": ("stale data (vote and digest) and a frame numbered for the previous cycle", "Duplex"),
    "scale": ("the vote", "Duplex"), "noise": ("the vote", "Duplex"), "invert": ("the vote", "Duplex"), "swap": ("the vote", "Duplex"), "zero": ("the vote", "Duplex"), "clip": ("the vote", "Duplex"),
    "oscillate": ("the vote, about 4 frames after", "Duplex"), "repeat": ("the vote (the held sample falls behind the motion)", "Duplex"),
    "bitflip": ("the vote, then the leaky count (the upset hits only some frames)", "Duplex"), "stuckbit": ("the vote", "Duplex"), "cmdstuck": ("the vote on the command", "Duplex"), "cmdinvert": ("the vote on the command", "Duplex"),
    "partial": ("missing frames", "Duplex"), "duplicate": ("sequence errors (the second copy repeats a number)", "Duplex"), "replay": ("stale data: the vote and the digest", "Duplex"),
    "seqstuck": ("sequence errors", "Duplex"), "early": ("sequence errors (frames arrive in the previous frame's window)", "Duplex"), "jitter": ("missing frames (some miss the 7 ms vote)", "Duplex"),
    "clockdrift": ("the command frame crosses the 7 ms vote deadline once the drift reaches about 1.7 ms: the vote and the digest", "Duplex"),
}


class FaultLab:
    def __init__(self, hub: Hub, rig: Rig) -> None:
        self.hub, self.rig = hub, rig
        self.replies: queue.Queue[str] = queue.Queue()
        self.table: list[dict] = []
        self.history: deque[dict] = deque(maxlen=100)
        self.lock = threading.Lock()
        rig.line_hooks.append(self._on_line)
        hub.event_listeners.append(self._on_event)

    def _on_line(self, name: str, text: str) -> None:
        if name == "PEERS" and (text.startswith(("ok ", "error:", "fault "))):
            self.replies.put(text)

    def _on_event(self, e) -> None:
        """Measure the detection: the first time after a fault started that the computers call its node latched (the heartbeats' view, or the node's own console), and why (the console)."""
        if e.kind not in ("view", "latched-out") or e.node is None:
            return
        if e.kind == "view" and e.fields.get("to") not in ("latched", "disabled"):
            return
        with self.lock:
            for f in self.table:
                if f["node"] != e.node or e.frame is None or e.frame < f["start"]:
                    continue
                d = f.get("detected")
                if d is None:
                    f["detected"] = {"frame": e.frame, "frames_after": e.frame - f["start"], "by": "heartbeats" if e.kind == "view" else "console", "reason": e.fields.get("reason")}
                elif e.kind == "latched-out" and d.get("reason") is None:
                    d["reason"] = e.fields.get("reason")
                    d["frames_after"] = min(d["frames_after"], max(0, e.frame - f["start"]))

    def available(self) -> bool:
        p = self.rig.procs.get("PEERS")
        return p is not None and p.state == "running" and p.spec.control

    def _ask(self, line: str, timeout: float = 2.0) -> list[str]:
        """Send one control line and collect its answer (a command is applied between frames, so the answer takes a frame or two)."""
        with self.lock:
            while not self.replies.empty():
                self.replies.get_nowait()
            self.rig.write_line("PEERS", line)
            out: list[str] = []
            try:
                first = self.replies.get(timeout=timeout)
            except queue.Empty:
                raise TimeoutError("the virtual peers did not answer (no SYNC to follow, or they have stopped)") from None
            out.append(first)
            if first.startswith("ok list"):
                while True:
                    try:
                        out.append(self.replies.get(timeout=0.2))
                    except queue.Empty:
                        break
            return out

    def add(self, spec: str, frames: int | None = None) -> dict:
        if not self.available():
            raise ValueError("the fault lab is not running: start it on the Rig tab (the virtual peers take the faults)")
        try:
            fault = parse_fault(spec)                        # the same parser the peers use: a bad spec is refused here, with its reason, before it is sent
        except FaultSpecError as e:
            raise ValueError(str(e)) from e
        if fault.node == 0:
            raise ValueError("node A is the real flight computer in the fault lab: the virtual peers are B and C. To break A, use a process fault (kill, freeze, restart) on the Faults tab")
        line = f"add {spec}" + (f" for {int(frames)}" if frames else "")
        reply = self._ask(line)[0]
        if reply.startswith("error"):
            raise ValueError(reply.removeprefix("error: "))
        _ok, _add, number, rest = reply.split(" ", 3)
        placed = parse_fault(rest)                          # the peers answer with the spec as they applied it (a start of its own, an end from `for`)
        rec = {"id": int(number), "spec": rest, "kind": placed.kind, "node": "ABC"[placed.node], "start": placed.start, "end": placed.end, "frames": frames, "detected": None}
        with self.lock:
            self.table.append(rec)
        self.history.append(rec)
        expect = EXPECT.get(fault.kind)
        self.hub.note("warn", "OP", "fault", f"FAULT INJECTED on node {rec['node']}: {rec['spec']}" + (f" (expected: {expect[0]}; then {expect[1]})" if expect else ""), {"id": rec["id"], "spec": rec["spec"]})
        return rec

    def clear(self, which: str) -> None:
        if not self.available():
            raise ValueError("the fault lab is not running")
        reply = self._ask(f"clear {which}")[0]
        if reply.startswith("error"):
            raise ValueError(reply.removeprefix("error: "))
        with self.lock:
            self.table = [] if which == "all" else [f for f in self.table if str(f["id"]) != str(which)]
        self.hub.note("info", "OP", "fault", f"fault {which} cleared (what it already did stays done)")

    def snapshot(self) -> dict:
        frame = self.hub.model.sync_no
        with self.lock:
            table = [dict(f) for f in self.table]
        for f in table:                                       # a fault with a duration ends by itself: it stays in the list, marked, until the operator clears it
            f["active"] = f["end"] is None or (frame is not None and frame < f["end"])
        return {"available": self.available(), "table": table, "frame": frame}
