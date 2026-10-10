# SPDX-License-Identifier: MIT
"""Recording a session: the bus as a `candump -L` log (the format `tfc_peers decode`, `tfc_replay` and this console's own replay read), and beside it a sidecar of what the bus does not carry.

`console-<stamp>.log` is the bus; `console-<stamp>.side.jsonl` is one JSON object per line: the nodes' console lines (`k: line`), the simulator's telemetry (`k: truth`), the 3D viewer's poses and specs (`k: pose`, `k: spec`: the whole state fifty times a second) and what the operator did (`k: note`).
Both count time from the start of the recording, so a replay lines them up. A log without a sidecar still replays (the bus alone is enough for the state, the votes and the commands seen on it).
"""
from __future__ import annotations

import gzip
import json
import threading
import time
from pathlib import Path

from tfc_peers import protocol as P

NOTE_SOURCES = ("OP", "CON")      # the events the console itself made: the only ones the replay cannot derive again from the frames and the lines


class Recorder:
    def __init__(self, directory: Path, t0: float, iface: str = "vcan0", label: str = "") -> None:
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.base = directory / f"console-{stamp}{('-' + label) if label else ''}"
        self.iface = iface
        self.t0 = t0
        self.path = self.base.with_suffix(".log")
        self.side_path = Path(str(self.base) + ".side.jsonl")
        self._log = open(self.path, "w", encoding="ascii")
        self._side = open(self.side_path, "w", encoding="utf-8")
        self.frames = 0
        self.lines = 0
        self.started = time.time()
        self._lock = threading.Lock()

    def _rel(self, t: float) -> float:
        return max(0.0, t - self.t0)

    def frame(self, t: float, f: P.Frame) -> None:
        sec, usec = divmod(int(self._rel(t) * 1_000_000), 1_000_000)
        with self._lock:
            self._log.write(f"({sec}.{usec:06d}) {self.iface} {f.id:03X}#{f.data.hex().upper()}\n")
            self.frames += 1

    def _side_write(self, t: float, k: str, fields: dict) -> None:
        """One sidecar record: `k` says what it is (line, truth, note); `fields` are its content (a note has a field called `kind` of its own, which is why they are not keyword arguments)."""
        with self._lock:
            self._side.write(json.dumps({"t": round(self._rel(t), 4), "k": k, **fields}, separators=(",", ":")) + "\n")
            self.lines += 1

    def line(self, t: float, src: str, text: str) -> None:
        self._side_write(t, "line", {"src": src, "text": text.rstrip("\r\n")})

    def truth(self, t: float, d: dict) -> None:
        self._side_write(t, "truth", {"d": d})

    def pose(self, t: float, raw: str) -> None:
        """A pose or a spec of the 3D viewer, as the simulator sent it (already compact JSON): written as it is, without parsing it again."""
        k = "spec" if raw.startswith('{"k":"spec"') else "pose"
        with self._lock:
            self._side.write('{"t":%.4f,"k":"%s","d":%s}\n' % (self._rel(t), k, raw))
            self.lines += 1

    def event(self, t: float, e: dict) -> None:
        if e.get("src") in NOTE_SOURCES:
            self._side_write(t, "note", {"level": e["level"], "src": e["src"], "kind": e["kind"], "text": e["text"], "fields": e.get("fields", {})})

    def describe(self) -> dict:
        return {"file": str(self.path), "frames": self.frames, "side": self.lines, "seconds": round(time.time() - self.started, 1)}

    def close(self) -> dict:
        info = self.describe()
        with self._lock:
            self._log.close()
            self._side.close()
        return info


def open_text(path: Path):
    """A log or sidecar for reading, gzip or plain."""
    return gzip.open(path, "rt", encoding="utf-8") if str(path).endswith(".gz") else open(path, encoding="utf-8")


def sidecar_of(path: Path) -> Path | None:
    name = str(path)
    for ext in (".log.gz", ".log"):
        if name.endswith(ext):
            for cand in (name[: -len(ext)] + ".side.jsonl", name[: -len(ext)] + ".side.jsonl.gz"):
                if Path(cand).exists():
                    return Path(cand)
    return None
