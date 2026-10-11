# SPDX-License-Identifier: MIT
"""The vehicle: choose a vehicle file, change the numbers the operator is allowed to change, check it with the simulator's own reader, fly a preview of it with the real flight software, and give the rig what it needs.

Everything that *knows* what a vehicle is stays in the C++ simulator. The console never reads physics out of a file: it asks `tfc_fly --check` whether a file is valid (the reader's own messages, with line and column) and
`tfc_fly FILE --csv` for a flight, and `tfc_simd` flies the one the rig is given. What the console adds is the editing: a set of **knobs** (the departures and the tunable numbers: thrust, drag, the gimbal's limits,
the wind, the gains' design) that are written into a copy of the file, and a raw editor for everything else.

Two honest limits are on the page, not hidden here. (1) The flight computers carry the pitch program and gain tables of the *reference* vehicle, compiled in; `tfc_simd` flying another vehicle needs
`tfc_gen_tables --vehicle` and a rebuild of the firmware, so the rig's own vehicle is the reference one (with whatever plant departures the knobs apply, which the gains are robust to by design), while the preview flies any vehicle,
because `tfc_fly` designs the tables for it. (2) A file edited here is saved as plain JSON: the `//` comments of the committed examples are not kept, so a copy never overwrites the example it came from.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
import re
import subprocess
import threading
import time
from pathlib import Path

# the knobs: (path in the file, label, unit, step, minimum, maximum, what it does, the default if the file does not say)
KNOBS = (
    ("vehicle.thrust_scale", "Thrust scale", "x", 0.01, 0.5, 1.5, "Multiplies every engine's thrust: a motor that is stronger or weaker than the design.", 1.0),
    ("vehicle.cd_scale", "Drag scale", "x", 0.05, 0.3, 3.0, "Multiplies the axial force coefficient.", 1.0),
    ("vehicle.cn_scale", "Normal-force scale", "x", 0.05, 0.3, 3.0, "Multiplies the normal-force slope: how strongly the air turns the vehicle.", 1.0),
    ("vehicle.thrust_misalign_pitch_deg", "Thrust misalignment, pitch", "deg", 0.05, -3.0, 3.0, "The engines point this far off the axis in the pitch plane.", 0.0),
    ("vehicle.thrust_misalign_yaw_deg", "Thrust misalignment, yaw", "deg", 0.05, -3.0, 3.0, "The same in the yaw plane.", 0.0),
    ("vehicle.gimbal_limit_deg", "Gimbal limit", "deg", 0.5, 1.0, 20.0, "How far the engines can swing.", 8.0),
    ("vehicle.gimbal_rate_dps", "Gimbal rate", "deg/s", 5.0, 10.0, 300.0, "How fast they can swing.", 60.0),
    ("vehicle.gimbal_lag_s", "Gimbal lag", "s", 0.005, 0.0, 0.3, "A first-order lag between the command and the engines.", 0.0),
    ("scenario.wind_scale", "Wind scale", "x", 0.1, 0.0, 4.0, "Scales the mean wind profile (0: calm).", 1.0),
    ("scenario.dry_cg_shift_m", "Dry CG shift", "m", 0.05, -1.5, 1.5, "Moves every stage's dry centre of gravity along the vehicle.", 0.0),
    ("scenario.turbulence.sigma_ms", "Turbulence", "m/s", 0.5, 0.0, 15.0, "A random wind of this standard deviation (Dryden-style).", 0.0),
    ("design.gains.wn", "Loop bandwidth (design)", "rad/s", 0.1, 0.5, 8.0, "The natural frequency the controller's gains are designed for.", 2.5),
    ("design.gains.zeta", "Loop damping (design)", "", 0.05, 0.3, 1.5, "The damping ratio the gains are designed for.", 0.8),
)
_NAME = re.compile(r"^[A-Za-z0-9_.-]{1,48}$")


def strip_comments(text: str) -> str:
    """The `//` comments the vehicle files allow, taken out (outside strings), so `json` can read them."""
    out: list[str] = []
    i, n, in_str, esc = 0, len(text), False, False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
            out.append(c)
        elif c == "/" and text[i:i + 2] == "//":
            while i < n and text[i] != "\n":
                i += 1
            continue
        else:
            out.append(c)
        i += 1
    return "".join(out)


def get_path(d: dict, path: str, default=None):
    cur = d
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def set_path(d: dict, path: str, value) -> None:
    cur = d
    keys = path.split(".")
    for k in keys[:-1]:
        cur = cur.setdefault(k, {})
    cur[keys[-1]] = value


class Jobs:
    """Long runs (a preview flight can take several seconds) in a thread each; the page polls for the result."""

    def __init__(self) -> None:
        self.jobs: dict[str, dict] = {}
        self.lock = threading.Lock()
        self._n = 0

    def start(self, kind: str, fn) -> str:
        with self.lock:
            self._n += 1
            jid = f"{kind}-{self._n}"
            self.jobs[jid] = {"id": jid, "kind": kind, "state": "running", "started": time.time()}
            for old in [k for k, v in self.jobs.items() if v["state"] != "running" and time.time() - v["started"] > 600]:
                del self.jobs[old]

        def run() -> None:
            try:
                result = fn()
                res = {"state": "done", "result": result}
            except Exception as e:  # noqa: BLE001 - the job's error is the job's result
                res = {"state": "failed", "error": f"{type(e).__name__}: {e}"}
            with self.lock:
                self.jobs[jid].update(res, seconds=round(time.time() - self.jobs[jid]["started"], 1))

        threading.Thread(target=run, name=jid, daemon=True).start()
        return jid

    def get(self, jid: str) -> dict | None:
        with self.lock:
            return dict(self.jobs[jid]) if jid in self.jobs else None


class Vehicles:
    def __init__(self, repo: Path, state_dir: Path) -> None:
        self.repo = repo
        self.dir = repo / "vehicles"
        self.scratch = state_dir / "vehicles"
        self.fly = Path(os.environ.get("TFC_FLY_BIN") or repo / "build" / "host" / "tfc_fly")      # as TFC_REPLAY_BIN: where the simulator's tool is when the build is not build/host
        self.jobs = Jobs()
        self.active: dict = {"name": "reference", "knobs": {}}      # what the rig's simulator is given
        self._nominal_cache: dict[str, dict] = {}

    # ------------------------------------------------------------------ files
    def listing(self) -> list[dict]:
        out = [{"name": "reference", "where": "built in", "description": "The built-in reference vehicle: five engines, 30 t, a 100 s ascent through max-Q (the one the flight computers' tables are made for)."}]
        for d, tag in ((self.dir, "vehicles/"), (self.scratch, "console")):
            if d.is_dir():
                for p in sorted(d.glob("*.json")):
                    if p.stem.startswith("_"):             # the console's own working files (_tmp-*, _rig, _reference): not vehicles to choose
                        continue
                    if d == self.dir and p.stem == "reference":      # the built-in vehicle above is this file (`tfc_fly reference --dump`): one entry, not two
                        continue
                    desc = ""
                    try:
                        data = json.loads(strip_comments(p.read_text()))
                        if "stages" not in data:           # not a vehicle: the Monte Carlo's dispersion sets (tfc_mc --config) live in the same directory
                            continue
                        desc = data.get("description", "")
                    except (OSError, ValueError):
                        desc = "(cannot be read: see Validate)"
                    out.append({"name": p.stem if tag == "console" else p.stem, "file": p.name, "where": tag, "description": desc, "path": str(p)})
        return out

    def _resolve(self, name: str) -> Path | None:
        if name == "reference":
            return None
        if not _NAME.match(name):
            raise ValueError("not a vehicle name")
        for d in (self.scratch, self.dir):
            p = d / f"{name}.json"
            if p.is_file():
                return p
        raise ValueError(f"no vehicle file named {name!r}")

    def read(self, name: str) -> dict:
        p = self._resolve(name)
        if p is None:
            text = self._reference_text()
        else:
            text = p.read_text()
        try:
            data = json.loads(strip_comments(text))
        except ValueError as e:
            return {"name": name, "text": text, "data": None, "parse_error": str(e)}
        knobs = {path: get_path(data, path, default) for path, *_r, default in KNOBS}
        return {"name": name, "text": text, "data": data, "knobs": knobs, "where": "built in" if p is None else str(p.relative_to(self.repo)) if self.repo in p.parents else str(p)}

    def _reference_text(self) -> str:
        """The built-in vehicle written out by the simulator itself (`tfc_fly reference --dump`), so what is edited is exactly what it flies."""
        committed = self.dir / "reference.json"
        if committed.is_file():
            return committed.read_text()
        out = self.scratch / "_reference.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([str(self.fly), "reference", "--dump", str(out)], check=True, capture_output=True, timeout=30)
        return out.read_text()

    def knob_table(self) -> list[dict]:
        return [{"path": p, "label": l, "unit": u, "step": st, "min": lo, "max": hi, "doc": d, "default": df} for p, l, u, st, lo, hi, d, df in KNOBS]

    def apply_knobs(self, data: dict, knobs: dict) -> dict:
        out = copy.deepcopy(data)
        allowed = {k[0]: k for k in KNOBS}
        for path, value in knobs.items():
            if path not in allowed:
                raise ValueError(f"{path!r} is not a knob")
            if value is None or value == "":
                continue
            v = float(value)
            lo, hi = allowed[path][4], allowed[path][5]
            if not lo <= v <= hi:
                raise ValueError(f"{allowed[path][1]}: {v:g} is outside {lo:g} to {hi:g}")
            set_path(out, path, v)
        return out

    def save(self, name: str, text: str) -> dict:
        if not _NAME.match(name) or name == "reference":
            raise ValueError("a name is letters, digits, dot, dash and underscore, up to 48, and not `reference`")
        if (self.dir / f"{name}.json").exists():
            raise ValueError(f"vehicles/{name}.json is one of the repository's examples: save a copy under another name (the console never overwrites them)")
        try:
            data = json.loads(strip_comments(text))
        except ValueError as e:
            raise ValueError(f"not valid JSON: {e}") from e
        self.scratch.mkdir(parents=True, exist_ok=True)
        p = self.scratch / f"{name}.json"
        p.write_text(json.dumps(data, indent=2) + "\n")
        return {"saved": str(p)}

    # ------------------------------------------------------------------ the simulator's own reader and flight
    def _write_tmp(self, text: str) -> Path:
        self.scratch.mkdir(parents=True, exist_ok=True)
        h = hashlib.sha1(text.encode()).hexdigest()[:12]
        p = self.scratch / f"_tmp-{h}.json"
        p.write_text(text)
        return p

    def check(self, text: str) -> dict:
        """What `tfc_fly --check` says about the text: the reader's messages with their lines, or the description of the vehicle."""
        if not self.fly.exists():
            return {"ok": False, "output": "build/host/tfc_fly is not built (cmake --build build/host)"}
        p = self._write_tmp(text)
        r = subprocess.run([str(self.fly), str(p), "--check"], capture_output=True, text=True, timeout=60)
        out = (r.stdout + r.stderr).replace(str(p), "this file")
        return {"ok": r.returncode == 0, "output": out.strip()}

    def preview(self, text: str, sensors: str = "platform", pad: int = 0) -> dict:
        """Fly the vehicle with the real flight software (`tfc_fly`) and return the flight as columns, with the simulator's own verdict."""
        if not self.fly.exists():
            raise ValueError("build/host/tfc_fly is not built")
        p = self._write_tmp(text)
        csv = p.with_suffix(".csv")
        argv = [str(self.fly), str(p), "--csv", str(csv), "--every", "10", "--sensors", sensors if sensors in ("platform", "vehicle") else "platform"]
        if sensors == "vehicle":
            argv.append("--no-accel")
        if pad:
            argv += ["--pad", str(int(pad))]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=300)
        out = (r.stdout + r.stderr).replace(str(p), "this file")
        cols: dict[str, list[float]] = {}
        if csv.exists():
            rows = [ln.split(",") for ln in csv.read_text().splitlines()]
            head, body = rows[0], rows[1:]
            cols = {h: [float(r_[i]) for r_ in body] for i, h in enumerate(head)}
            csv.unlink()
        return {"ok": r.returncode == 0, "output": out.strip(), "columns": cols, "seconds_of_flight": cols.get("t_s", [0])[-1] if cols else 0}

    def nominal(self, name: str = "reference") -> dict:
        """The nominal flight of a vehicle, for the Flight tab's overlay: one preview, kept (it is the same every time)."""
        if name in self._nominal_cache:
            return self._nominal_cache[name]
        info = self.read(name)
        if info.get("data") is None:
            raise ValueError("the vehicle file cannot be read")
        res = self.preview(info["text"], "platform" if name == "reference" else "vehicle")
        self._nominal_cache[name] = res
        return res

    # ------------------------------------------------------------------ what the rig's simulator is given
    def set_active(self, name: str, knobs: dict) -> dict:
        info = self.read(name)
        if info.get("data") is None:
            raise ValueError("the vehicle file cannot be read")
        self.apply_knobs(info["data"], knobs)       # validated now, so a bad knob is refused here and not when the rig starts
        self.active = {"name": name, "knobs": {k: v for k, v in knobs.items() if v not in (None, "")}}
        return self.active

    def sim_args(self) -> list[str]:
        """The `tfc_simd` arguments for the active vehicle: nothing for the untouched reference; for it with plant departures, a file written with the knobs applied (never one of the committed examples, and
        the platform's sensors as always); for any other vehicle that file and `--vehicle-true`: the sensors are the vehicle's own, because a platform that tilts a few degrees cannot follow a launcher that
        pitches past 45."""
        name, knobs = self.active["name"], self.active["knobs"]
        if name == "reference" and not knobs:
            return []
        info = self.read(name)
        data = self.apply_knobs(info["data"], knobs)
        text = json.dumps(data, indent=2) + "\n"
        p = self.scratch / "_rig.json"
        self.scratch.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        return ["--vehicle", str(p)] + ([] if name == "reference" else ["--vehicle-true"])

    def snapshot(self) -> dict:
        return {"active": self.active, "tables_note": "reference" if self.active["name"] == "reference" else "other"}
