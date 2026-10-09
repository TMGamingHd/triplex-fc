# SPDX-License-Identifier: MIT
"""Where the console's data comes from: a CAN interface (live), a recorded log (replay), and the simulator's telemetry (UDP).

All three deliver to a `Hub`; none of them judges anything. The live source is a reader thread on a raw SocketCAN socket (standard library, as in `tfc_peers/bus.py`); it never transmits. Operator
commands go out through `commands.py`, on a socket of their own.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path

from tfc_peers import protocol as P
from tfc_peers.bus import SocketCanBus

from .hub import Hub
from .recorder import open_text, sidecar_of


class BusSource(threading.Thread):
    """Reads every frame of a SocketCAN interface (vcan0 for the virtual rig, can0 for the USB-CAN adapter) and delivers it with a monotonic time."""

    def __init__(self, hub: Hub, iface: str) -> None:
        super().__init__(name=f"bus-{iface}", daemon=True)
        self.hub, self.iface = hub, iface
        self.bus = SocketCanBus(iface)                                   # raises OSError, with the way to create vcan0, when the interface does not exist
        try:
            self.bus._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)   # a burst while the GIL is busy must not be dropped by the kernel
        except OSError:
            pass
        self._stop_flag = threading.Event()
        hub.status.update(kind="live", iface=iface, state="listening")

    def run(self) -> None:
        got = False
        while not self._stop_flag.is_set():
            f = self.bus.recv(0.2)
            if f is None:
                continue
            if not got:
                got = True
                self.hub.status["state"] = "receiving"
            self.hub.ingest_frame(time.monotonic(), f)

    def stop(self) -> None:
        self._stop_flag.set()
        self.join(timeout=1.0)
        self.bus.close()


class TruthSource(threading.Thread):
    """The simulator's own telemetry: one JSON object per UDP datagram from `tfc_simd --telemetry PORT`."""

    DEFAULT_PORT = 45679

    def __init__(self, hub: Hub, port: int = DEFAULT_PORT) -> None:
        super().__init__(name="truth-udp", daemon=True)
        self.hub = hub
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", port))
        except OSError:                                          # the port is taken (a second console): any free one; the rig the console starts is told which
            self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.bad = 0
        self._stop_flag = threading.Event()

    def run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                d = json.loads(data)
                if not isinstance(d, dict):
                    raise ValueError("not an object")
            except ValueError:
                self.bad += 1
                continue
            self.hub.ingest_truth(d)

    def stop(self) -> None:
        self._stop_flag.set()
        self.join(timeout=1.0)
        self.sock.close()


class LogSource(threading.Thread):
    """Plays a recorded session (a `candump -L` log, plain or gzip, with its sidecar if there is one) through the same hub, at a speed of the operator's choosing.

    The clock is the log's own. Seeking forgets the model and feeds it again from the start, which is the only way to a state that is true (the votes, the strikes and the mission clock are histories, not values).
    """

    def __init__(self, hub: Hub, path: Path, speed: float = 1.0, loop: bool = False) -> None:
        super().__init__(name="replay", daemon=True)
        self.hub, self.path = hub, Path(path)
        self.speed, self.loop = speed, loop
        self.frames: list[tuple[float, P.Frame]] = []
        self.side: list[dict] = []
        self._load()
        self.pos = self.t_start
        self.playing = True
        self._i = 0
        self._j = 0
        self._cv = threading.Condition()
        self._stop_flag = threading.Event()
        self._seek_to: float | None = None
        hub.set_replay(lambda: self.pos)
        self._status()

    def _load(self) -> None:
        with open_text(self.path) as fh:
            self.frames = [(t / 1e6, fr) for t, _iface, fr in _parse_stream(fh)]
        self.frames.sort(key=lambda x: x[0])
        if not self.frames:
            raise ValueError(f"{self.path}: no frames")
        side = sidecar_of(self.path)
        if side is not None:
            with open_text(side) as fh:
                self.side = [json.loads(line) for line in fh if line.strip()]
            self.side.sort(key=lambda r: r["t"])
        self.t_start = self.frames[0][0]
        self.t_end = self.frames[-1][0]

    def _status(self) -> None:
        self.hub.status.update(kind="replay", file=self.path.name, start=self.t_start, end=self.t_end, pos=round(self.pos, 2), speed=self.speed, playing=self.playing, sidecar=bool(self.side),
                               frames=len(self.frames), state="playing" if self.playing else "paused")

    # ---- controls (called from the web handlers)
    def play(self) -> None:
        with self._cv:
            if self.pos >= self.t_end:
                self._seek_to = self.t_start
            self.playing = True
            self._cv.notify()
        self._status()

    def pause(self) -> None:
        self.playing = False
        self._status()

    def set_speed(self, speed: float) -> None:
        self.speed = max(0.05, min(32.0, float(speed)))
        self._status()

    def seek(self, t: float) -> None:
        with self._cv:
            self._seek_to = max(self.t_start, min(self.t_end, t))
            self._cv.notify()

    def stop(self) -> None:
        self._stop_flag.set()
        with self._cv:
            self._cv.notify()
        self.join(timeout=2.0)

    # ---- the player
    def _feed_until(self, t: float) -> None:
        frames, side = self.frames, self.side
        i, j = self._i, self._j
        hub = self.hub
        while True:
            tf = frames[i][0] if i < len(frames) else None
            ts = side[j]["t"] if j < len(side) else None            # the sidecar counts from the same start as the log
            if tf is None and ts is None:
                break
            take_frame = ts is None or (tf is not None and tf <= ts)
            when = tf if take_frame else ts
            if when > t:
                break
            if take_frame:
                hub.ingest_frame(when, frames[i][1])
                i += 1
            else:
                self._apply_side(side[j], when)
                j += 1
            hub.tick_to(when)
        self._i, self._j = i, j
        hub.tick_to(t)

    def _apply_side(self, rec: dict, when: float) -> None:
        k = rec.get("k")
        if k == "line":
            self.hub.ingest_line(rec["src"], rec["text"], when)
        elif k == "truth":
            self.hub.ingest_truth(rec["d"], when)
        elif k == "note":
            with self.hub.lock:
                self.hub.model.emit(when, rec.get("level", "info"), rec.get("src", "OP"), rec.get("kind", "note"), rec.get("text", ""), None, rec.get("fields") or {})

    def _rewind_to(self, t: float) -> None:
        self.hub.reset_model()
        self._i = self._j = 0
        self.pos = self.t_start
        self._feed_until(t)
        self.pos = t

    def run(self) -> None:
        last = time.monotonic()
        self._feed_until(self.pos)
        while not self._stop_flag.is_set():
            with self._cv:
                if self._seek_to is None and not self.playing:
                    self._cv.wait(0.2)
                target, self._seek_to = self._seek_to, None
            now = time.monotonic()
            dt, last = now - last, now
            if target is not None:
                self._rewind_to(target)
                self._status()
                continue
            if not self.playing:
                continue
            self.pos = min(self.t_end, self.pos + dt * self.speed)
            self._feed_until(self.pos)
            if self.pos >= self.t_end:
                if self.loop:
                    self._rewind_to(self.t_start)
                else:
                    self.playing = False
            self._status()
            time.sleep(0.005)


def _parse_stream(fh):
    """`tfc_peers.bus.read_log` for an open text stream (so a gzip log reads too): (t_us, iface, Frame)."""
    for n, line in enumerate(fh, 1):
        line = line.strip()
        if not line:
            continue
        try:
            stamp, iface, body = line.split()
            sec, usec = stamp.strip("()").split(".")
            can_id, data = body.split("#")
            yield int(sec) * 1_000_000 + int(usec), iface, P.Frame(int(can_id, 16), bytes.fromhex(data))
        except ValueError as e:
            raise ValueError(f"line {n}: cannot parse {line!r}: {e}") from e
