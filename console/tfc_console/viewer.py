# SPDX-License-Identifier: MIT
"""The 3D viewer's data: the vehicle's whole state, fifty times a second, from the simulator (live), from a recording, or from a flight the console flies on request.

The console's own telemetry is thirty numbers at 10 Hz (`hub.ingest_truth`): enough for charts, not for drawing a vehicle. The simulator also sends a **pose** (`sim/vehicle/viewer_state.hpp`): where it is, which way it
points, the air, the forces, the engines, the tanks, and once in a while a **spec** (the vehicle as the simulator reads it and the world around it). They arrive on a UDP port of their own and go to the pages that
ask for them on a stream of their own (`/api/pose-stream`), so a console tab that does not show the viewer pays nothing for it: with no viewer subscribed a pose is parsed, kept as the latest, and dropped.

Three sources feed it, one at a time:
* the live simulator (`tfc_simd --viewer PORT`; the rig the console starts is told the port),
* a recording: the sidecar of a console recording carries the poses (`k: pose`) and the bus replay plays them with the rest, or a pose file (`tfc_fly --pose FILE`, `.pose.jsonl` or `.pose.jsonl.gz`) that `PoseReplay` plays
  with its own transport (play, pause, seek, speed) and that needs no bus at all,
* a flight: `fly` runs `tfc_fly` on any vehicle file with the real flight software and plays the result. This is how a vehicle the rig cannot fly (the rig's flight computers carry the tables of the reference vehicle)
  is seen in the viewer.
"""
from __future__ import annotations

import json
import queue
import socket
import subprocess
import threading
import time
from pathlib import Path

from .recorder import open_text


def dumps(o: object) -> str:
    return json.dumps(o, separators=(",", ":"), allow_nan=False, default=str)


class ViewerHub:
    """The latest pose and spec, and the pages subscribed to them."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.subs: set[queue.Queue] = set()
        self.spec_raw: str | None = None
        self.pose_raw: str | None = None
        self.source: dict = {"kind": "none"}          # what feeds the viewer now, for the page: kind, file, start, end, pos, speed, playing
        self.muted = False                              # a replay that is seeking feeds the model from the start: nothing of that goes to the pages, only the end of it
        self.count = 0
        self.bad = 0
        self.last_at = 0.0

    # ------------------------------------------------------------------ in
    def ingest(self, raw: str, kind: str | None = None) -> None:
        """One datagram or one recorded line, as the simulator wrote it. `kind` is "spec" or "pose" if the caller already knows."""
        if kind is None:
            kind = "spec" if raw.startswith('{"k":"spec"') else "pose" if raw.startswith('{"k":"pose"') else None
        if kind is None:
            self.bad += 1
            return
        with self.lock:
            self.last_at = time.monotonic()
            if kind == "spec":
                changed = raw != self.spec_raw
                self.spec_raw = raw
                if not changed or self.muted:
                    return
                self._publish("spec", raw)
            else:
                self.pose_raw = raw
                self.count += 1
                if not self.muted:
                    self._publish("pose", raw)

    def reset(self) -> None:
        """A replay jumped (or a new run began): the pages forget the poses they hold."""
        with self.lock:
            self.pose_raw = None
            self._publish("reset", "{}")

    def flush(self) -> None:
        """After a seek: the page gets the spec and the pose the seek ended on."""
        with self.lock:
            self.muted = False
            if self.spec_raw:
                self._publish("spec", self.spec_raw)
            if self.pose_raw:
                self._publish("pose", self.pose_raw)

    def set_source(self, **kw) -> None:
        with self.lock:
            self.source = kw
            self._publish("source", dumps(kw))

    # ------------------------------------------------------------------ out
    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=400)
        with self.lock:
            self.subs.add(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self.lock:
            self.subs.discard(q)

    def hello(self) -> str:
        with self.lock:
            return '{"spec":%s,"pose":%s,"source":%s}' % (self.spec_raw or "null", self.pose_raw or "null", dumps(self.source))

    def _publish(self, kind: str, raw: str) -> None:
        if not self.subs:
            return
        msg = f"event: {kind}\ndata: {raw}\n\n"
        for q in list(self.subs):
            try:
                q.put_nowait(msg)
            except queue.Full:                      # a page that cannot keep up loses its oldest pose: the newer one supersedes it
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except (queue.Empty, queue.Full):
                    pass

    def status(self) -> dict:
        with self.lock:
            return {"source": self.source, "poses": self.count, "bad": self.bad, "have_spec": self.spec_raw is not None, "age_s": round(time.monotonic() - self.last_at, 2) if self.last_at else None,
                    "subscribers": len(self.subs)}


class PoseSource(threading.Thread):
    """The simulator's poses and specs: one JSON object per UDP datagram from `tfc_simd --viewer PORT`."""

    DEFAULT_PORT = 45680

    def __init__(self, hub, port: int = DEFAULT_PORT) -> None:
        super().__init__(name="pose-udp", daemon=True)
        self.hub = hub
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        try:
            self.sock.bind(("127.0.0.1", port))
        except OSError:                                          # taken (a second console): any free one; the rig the console starts is told which
            self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self._stop_flag = threading.Event()

    def run(self) -> None:
        while not self._stop_flag.is_set():
            try:
                data = self.sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                self.hub.ingest_pose(data.decode("utf-8"))
            except UnicodeDecodeError:
                self.hub.viewer.bad += 1

    def stop(self) -> None:
        self._stop_flag.set()
        self.join(timeout=1.0)
        self.sock.close()


class PoseReplay(threading.Thread):
    """Plays a pose file (`tfc_fly --pose`) into the viewer at the speed the operator wants, with a transport of its own. No bus, no flight computers: the file is a flight that has already happened."""

    def __init__(self, hub, path: Path, speed: float = 1.0, loop: bool = False, autoplay: bool = True) -> None:
        super().__init__(name="pose-replay", daemon=True)
        self.hub, self.path = hub, Path(path)
        self.speed, self.loop = speed, loop
        self.spec: str | None = None
        self.poses: list[tuple[float, str]] = []        # (the flight time, the line)
        self._load()
        self.pos = self.poses[0][0]
        self.playing = autoplay
        self._i = 0
        self._cv = threading.Condition()
        self._seek_to: float | None = self.pos
        self._stop_flag = threading.Event()

    def _load(self) -> None:
        with open_text(self.path) as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                if line.startswith('{"k":"spec"'):
                    self.spec = line
                elif line.startswith('{"k":"pose"'):
                    i = line.index('"t":') + 4                     # the flight time (negative on the pad): the clock the page shows and the transport seeks in
                    j = line.index(",", i)
                    self.poses.append((float(line[i:j]), line))
        if not self.poses:
            raise ValueError(f"{self.path.name}: no poses (a pose file comes from `tfc_fly VEHICLE --pose FILE`)")
        if self.spec is None:
            raise ValueError(f"{self.path.name}: no spec line (the first line of a pose file describes the vehicle)")
        self.t_start, self.t_end = self.poses[0][0], self.poses[-1][0]

    def _status(self) -> None:
        self.hub.viewer.set_source(kind="pose-file", file=self.path.name, start=self.t_start, end=self.t_end, pos=round(self.pos, 2), speed=self.speed, playing=self.playing, loop=self.loop,
                                   state="playing" if self.playing else "paused")

    # ---- controls
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
    def _jump(self, t: float) -> None:
        v = self.hub.viewer
        v.reset()
        v.ingest(self.spec, "spec")
        lo, hi = 0, len(self.poses)
        while lo < hi:                                  # the first pose after t
            mid = (lo + hi) // 2
            if self.poses[mid][0] <= t:
                lo = mid + 1
            else:
                hi = mid
        self._i = max(0, lo - 1)
        v.ingest(self.poses[self._i][1], "pose")
        self._i += 1
        self.pos = t

    def run(self) -> None:
        last = time.monotonic()
        self.hub.viewer.ingest(self.spec, "spec")
        while not self._stop_flag.is_set():
            with self._cv:
                if self._seek_to is None and not self.playing:
                    self._cv.wait(0.2)
                target, self._seek_to = self._seek_to, None
            now = time.monotonic()
            dt, last = now - last, now
            if target is not None:
                self._jump(target)
                self._status()
                continue
            if not self.playing:
                continue
            self.pos = min(self.t_end, self.pos + dt * self.speed)
            v = self.hub.viewer
            while self._i < len(self.poses) and self.poses[self._i][0] <= self.pos:
                v.ingest(self.poses[self._i][1], "pose")
                self._i += 1
            if self.pos >= self.t_end:
                if self.loop:
                    self._seek_to = self.t_start
                else:
                    self.playing = False
            self._status()
            time.sleep(0.004)


class ViewerService:
    """The viewer's controls: which pose file or flight feeds it, and the models a user has dropped in."""

    def __init__(self, hub, repo: Path, state_dir: Path, vehicles, models_dir: Path | None = None) -> None:
        self.hub, self.repo = hub, repo
        self.vehicles = vehicles
        self.scratch = state_dir / "viewer"
        self.models = models_dir or repo / "console" / "models"
        self.player: PoseReplay | None = None
        self.lock = threading.RLock()
        self.flying: dict | None = None

    # ------------------------------------------------------------------ files
    def pose_files(self) -> list[dict]:
        out = []
        for d, tag in ((self.repo / "console" / "demo", "demo"), (self.repo / "logs", "recorded"), (self.scratch, "flown")):
            if d.is_dir():
                for p in sorted(d.glob("*.pose.jsonl*"), reverse=True):
                    if p.name.endswith((".pose.jsonl", ".pose.jsonl.gz")):
                        out.append({"name": p.name, "where": tag, "path": str(p), "bytes": p.stat().st_size})
        return out

    def resolve(self, name: str) -> Path:
        for f in self.pose_files():
            if f["name"] == name:
                return Path(f["path"])
        raise ValueError(f"no pose file named {name!r} in console/demo/, logs/ or the console's own flights")

    def model_files(self) -> list[dict]:
        out = []
        if self.models.is_dir():
            for p in sorted(self.models.iterdir()):
                if p.suffix.lower() in (".glb", ".gltf") and p.is_file():
                    out.append({"name": p.name, "bytes": p.stat().st_size})
        return out

    def model_bytes(self, name: str) -> tuple[bytes, str]:
        for f in self.model_files():
            if f["name"] == name:
                p = self.models / name
                return p.read_bytes(), "model/gltf-binary" if p.suffix.lower() == ".glb" else "model/gltf+json"
        raise ValueError(f"no model named {name!r} in console/models/")

    # ------------------------------------------------------------------ the player
    def _stop_player(self) -> None:
        if self.player is not None:
            self.player.stop()
            self.player = None

    def open(self, name: str, speed: float = 1.0, loop: bool = False, autoplay: bool = True) -> dict:
        return self.open_path(self.resolve(name), speed, loop, autoplay)

    def open_path(self, path: Path, speed: float = 1.0, loop: bool = False, autoplay: bool = True) -> dict:
        with self.lock:
            self._stop_player()
            self.hub.viewer.reset()
            self.hub.viewer.spec_raw = None
            self.player = PoseReplay(self.hub, path, speed, loop, autoplay)
            self.player.start()
        return self.hub.viewer.status()

    def control(self, action: str, value=None) -> dict:
        p = self.player
        if p is None:
            raise ValueError("no pose file is open")
        if action == "play":
            p.play()
        elif action == "pause":
            p.pause()
        elif action == "seek":
            p.seek(float(value))
        elif action == "speed":
            p.set_speed(float(value))
        elif action == "loop":
            p.loop = bool(value)
            p._status()
        else:
            raise ValueError("action is play, pause, seek, speed or loop")
        return self.hub.viewer.status()

    def close(self) -> None:
        with self.lock:
            self._stop_player()
            self.hub.viewer.set_source(kind="none")

    # ------------------------------------------------------------------ fly a vehicle with the real flight software, then play it
    def fly(self, text: str, label: str, sensors: str = "vehicle", pad: int = 300, hz: int = 50) -> dict:
        """Runs `tfc_fly` on a vehicle text and writes its poses; the caller (a job) opens the result."""
        fly = self.vehicles.fly
        if not fly.exists():
            raise ValueError("build/host/tfc_fly is not built (cmake --build build/host)")
        if hz < 1 or hz > 100:
            raise ValueError("the pose rate is 1 to 100 per second")
        self.scratch.mkdir(parents=True, exist_ok=True)
        veh = self.vehicles._write_tmp(text)
        safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in label)[:40] or "flight"
        out = self.scratch / f"{safe}.pose.jsonl"
        argv = [str(fly), str(veh), "--pose", str(out), "--pose-hz", str(hz), "--sensors", "vehicle" if sensors == "vehicle" else "platform"]
        if sensors == "vehicle":
            argv.append("--no-accel")
        if pad:
            argv += ["--pad", str(int(pad))]
        with self.lock:
            self.flying = {"label": label, "started": time.time()}
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=600)
        finally:
            with self.lock:
                self.flying = None
        output = (r.stdout + r.stderr).replace(str(veh), "this file")
        if not out.exists() or out.stat().st_size == 0:
            raise ValueError("tfc_fly wrote no poses:\n" + output.strip())
        return {"file": out.name, "ok": r.returncode == 0, "output": output.strip(), "bytes": out.stat().st_size}
