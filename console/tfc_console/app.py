# SPDX-License-Identifier: MIT
"""The console's composition root: the hub, the sources, the services and the routes that put them on the web.

The services (`commands`, `rig`, `faults`, `hardware`, `vehicles`) each own one thing the operator can do; this module owns what they share (where the repository is, where recordings go, which source is
connected) and the routes of the parts that have no service of their own (the bus table, the history, the recordings, the replay's transport).
"""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path

from tfc_peers import faults as F
from tfc_peers import protocol as P

from . import constants as K
from .hub import Hub
from .recorder import Recorder, sidecar_of
from .server import ApiError, App, Request
from .sources import BusSource, LogSource, TruthSource

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "console" / "web"


class SourceManager:
    """Which data the hub is fed from: a live interface or a replayed log, one at a time."""

    def __init__(self, hub: Hub, root: Path = ROOT) -> None:
        self.hub, self.root = hub, root
        self.source: BusSource | LogSource | None = None
        self.iface: str | None = None
        self.lock = threading.RLock()
        self.log_dir = root / "logs"
        self.demo_dir = root / "console" / "demo"

    def connect_live(self, iface: str) -> None:
        with self.lock:
            self._drop()
            self.hub.set_live()
            self.hub.reset_model()
            self.hub.status.clear()
            src = BusSource(self.hub, iface)          # OSError if the interface is not there: the old source is already gone, and the page says "no source"
            self.source, self.iface = src, iface
            src.start()

    def open_replay(self, path: Path, speed: float = 1.0) -> None:
        with self.lock:
            self._drop()
            self.hub.reset_model()
            self.hub.status.clear()
            src = LogSource(self.hub, path, speed)
            self.source, self.iface = src, None
            src.start()

    def _drop(self) -> None:
        if self.hub.recorder is not None:
            self.stop_record()
        if self.source is not None:
            self.source.stop()
            self.source = None
        self.hub.status.clear()
        self.hub.status.update(kind="none", state="no source")

    def disconnect(self) -> None:
        with self.lock:
            self._drop()

    def start_record(self, label: str = "") -> dict:
        with self.lock:
            if self.hub.recorder is not None:
                raise ApiError(409, "already recording")
            if self.iface is None:
                raise ApiError(409, "nothing to record: no live interface is connected (a replay is already a recording)")
            rec = Recorder(self.log_dir, self.hub.clock(), self.iface, label)
            self.hub.recorder = rec
            self.hub.note("info", "CON", "record", f"recording to {rec.path.name}")
            return rec.describe()

    def stop_record(self) -> dict | None:
        with self.lock:
            rec, self.hub.recorder = self.hub.recorder, None
            if rec is None:
                return None
            info = rec.close()
            self.hub.note("info", "CON", "record", f"recording stopped: {info['frames']} frames in {rec.path.name}")
            return info

    def recordings(self) -> list[dict]:
        out = []
        for d, tag in ((self.log_dir, "recorded"), (self.demo_dir, "demo")):
            if not d.is_dir():
                continue
            for p in sorted(d.glob("*.log*"), reverse=True):
                if not p.name.endswith((".log", ".log.gz")):            # not the sidecars (.side.jsonl), not anything else
                    continue
                out.append({"name": p.name, "where": tag, "path": str(p), "bytes": p.stat().st_size, "sidecar": sidecar_of(p) is not None})
        return out

    def resolve_recording(self, name: str) -> Path:
        """A recording by file name, from logs/ or console/demo/ only (a path from the page is never opened as given)."""
        for r in self.recordings():
            if r["name"] == name:
                return Path(r["path"])
        raise ApiError(404, f"no recording named {name!r} in logs/ or console/demo/")


def fault_catalog() -> list[dict]:
    out = []
    for kind, (row, desc, params) in F.KINDS.items():
        out.append({"kind": kind, "row": row, "desc": desc, "params": params})
    return out


def config_payload() -> dict:
    return {
        "nodes": list(K.NODE_NAMES), "channels": [{"name": n, "unit": u, "tol": t} for (n, u), t in zip(K.CHANNELS, K.VOTE_TOL)],
        "act_tol": K.ACT_TOL_DEG, "arm_window_frames": K.ARM_WINDOW_FRAMES, "command_window": K.COMMAND_WINDOW, "persist": [K.PERSIST_M, K.PERSIST_N], "strikes_max": [K.STRIKES_MAX, K.STRIKES_MAX_PHYSICAL],
        "phases": [{"n": i, "name": n, "nominal": r[0], "minimum": r[1]} for i, (n, r) in enumerate(zip(K.PHASE_NAMES, K.PHASE_RULES))],
        "schedule": [{"id": i, "name": n, "what": w} for i, n, w in K.SCHEDULE],
        "ground_ops": {k: v for k, v in P.GROUND_OPS.items()}, "faults": fault_catalog(),
        "view_names": list(K.VIEW_NAMES), "mode_names": {str(k): v for k, v in K.MODE_NAMES.items()},
    }


def register_core(app: App, mgr: SourceManager) -> None:
    hub = app.hub

    @app.route("GET", "/api/config")
    def _config(req: Request):
        return 200, config_payload()

    @app.route("GET", "/api/hold")
    def _hold(req: Request):
        """Answers after `ms` milliseconds. A page that loads this as an image does not finish loading until it answers, which is how a headless browser is made to wait for the first state before it takes a screenshot (`?hold=ms`)."""
        time.sleep(min(120.0, req.num("ms", 0) / 1000.0))
        return 200, {"held": True}

    @app.route("GET", "/api/snapshot")
    def _snapshot(req: Request):
        return 200, hub.snapshot()

    @app.route("GET", "/api/history")
    def _history(req: Request):
        return 200, hub.history_columns(req.num("seconds", 180.0))

    @app.route("GET", "/api/events")
    def _events(req: Request):
        return 200, {"events": hub.recent_events(int(req.num("limit", 400)))}

    @app.route("GET", "/api/lines")
    def _lines(req: Request):
        return 200, hub.recent_lines(int(req.num("limit", 300)))

    @app.route("GET", "/api/bus")
    def _bus(req: Request):
        return 200, hub.bus_table()

    @app.route("GET", "/api/frames")
    def _frames(req: Request):
        ids = None
        if req.arg("ids"):
            try:
                ids = {int(x, 16) for x in req.arg("ids").split(",") if x}
            except ValueError:
                raise ApiError(400, "ids must be comma-separated hex ids") from None
        return 200, hub.recent_frames(int(req.num("after", 0)), min(int(req.num("limit", 300)), 1000), ids)

    @app.route("GET", "/api/source")
    def _source(req: Request):
        return 200, {"status": dict(hub.status), "iface": mgr.iface, "recordings": mgr.recordings(), "recording": None if hub.recorder is None else hub.recorder.describe(),
                     "interfaces": sorted(p.name for p in Path("/sys/class/net").iterdir() if p.name.startswith(("vcan", "can", "slcan")))}

    @app.route("POST", "/api/source")
    def _source_set(req: Request):
        b = req.body
        if b.get("iface"):
            iface = str(b["iface"])
            if not iface.replace("_", "").isalnum() or len(iface) > 15:
                raise ApiError(400, "not an interface name")
            try:
                mgr.connect_live(iface)
            except OSError as e:
                raise ApiError(409, str(e)) from e
        elif b.get("replay"):
            try:
                mgr.open_replay(mgr.resolve_recording(str(b["replay"])), float(b.get("speed", 1.0)))
            except (ValueError, OSError) as e:
                raise ApiError(400, str(e)) from e
        elif b.get("disconnect"):
            mgr.disconnect()
        else:
            raise ApiError(400, "give iface, replay or disconnect")
        return 200, {"status": dict(hub.status)}

    @app.route("POST", "/api/replay")
    def _replay(req: Request):
        src = mgr.source
        if not isinstance(src, LogSource):
            raise ApiError(409, "no replay is open")
        b = req.body
        if "speed" in b:
            src.set_speed(float(b["speed"]))
        if "seek" in b:
            src.seek(float(b["seek"]))
        if b.get("play") is True:
            src.play()
        elif b.get("play") is False:
            src.pause()
        return 200, {"status": dict(hub.status)}

    @app.route("POST", "/api/record")
    def _record(req: Request):
        if req.body.get("stop"):
            return 200, {"recording": mgr.stop_record()}
        label = "".join(c for c in str(req.body.get("label", "")) if c.isalnum() or c in "-_")[:24]
        return 200, {"recording": mgr.start_record(label)}


def default_state_dir() -> Path:
    base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "tfc_console"


__all__ = ["SourceManager", "register_core", "ROOT", "WEB", "TruthSource"]
