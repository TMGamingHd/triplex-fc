# SPDX-License-Identifier: MIT
"""Which vehicle the rig flies, and the flight computers that are built for it.

The flight computers carry, compiled in, the pitch program and the gain schedule that were designed on their vehicle's nominal ascent (`flight_tables.hpp`, written by `tfc_gen_tables --vehicle FILE`). To put another
vehicle on the rig, the rig needs three flight-computer images with *that vehicle's* tables: the same flight code, other numbers. This module does what a person would do by hand, for the vehicle the operator chose:

1. take the vehicle file with only the **design** knobs applied (`design.*`: the loop bandwidth and damping the gains are designed for). The **plant** knobs (thrust, drag, misalignment, wind...) are departures
   from the design, which the flight computers must not know, so they go to the simulator and not to the tables;
2. check it with the simulator's own reader (`tfc_fly --check`), so a file it refuses is refused here with its messages;
3. write its tables, and build the three launch images (`west build`, Zephyr's native_sim, about ten seconds each) with `-DTFC_TABLES_HEADER` pointing at them, and with the attitude estimator told not to use
   the accelerometer (a vehicle under thrust: SIM_FIDELITY.md 3.2);
4. keep them under `build/vehicles/NAME-KEY/`, where KEY is a hash of the design file and of the firmware's sources: the same vehicle with the same firmware is not built twice, and a firmware that has changed
   is noticed.

The reference vehicle, untouched, has its images already (`build/launch_a`, `_b`, `_c`: the committed tables); nothing is built for it. The simulator side (`tfc_simd --vehicle FILE --vehicle-true`) is `Vehicles.sim_args`.
The build is the real build: nothing here fakes a flight computer, and a vehicle whose tables the design cannot make (an unstable plant it cannot stabilise) fails in `tfc_gen_tables`, with its message.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from .hub import Hub

NODES = "abc"


class RigBuilder:
    def __init__(self, hub: Hub, repo: Path, vehicles, state_dir: Path | None = None) -> None:
        self.hub, self.repo, self.vehicles = hub, Path(repo), vehicles
        self.root = self.repo / "build" / "vehicles"
        self.gen = Path(os.environ.get("TFC_GEN_TABLES_BIN") or self.repo / "build" / "host" / "tfc_gen_tables")
        self.lock = threading.RLock()
        self.job: threading.Thread | None = None
        self.log: deque[str] = deque(maxlen=400)
        self.steps: list[dict] = []
        self.state = "idle"                      # idle, building, failed
        self.error = ""
        self.started = 0.0
        self.seconds = 0.0
        self.building: tuple[str, str] | None = None      # (name, key) of the build in progress
        self._fw_cache: tuple[float, str] | None = None
        self._status_cache: tuple | None = None

    # ------------------------------------------------------------------ what the rig is given
    def reference_untouched(self, name: str, knobs: dict) -> bool:
        return name == "reference" and not any(k.startswith("design.") for k in knobs)

    def design_text(self, name: str, knobs: dict) -> str:
        """The vehicle as the tables are designed on it: the design knobs applied, the plant's departures not."""
        info = self.vehicles.read(name)
        if info.get("data") is None:
            raise ValueError("the vehicle file cannot be read: " + str(info.get("parse_error", "")))
        design = {k: v for k, v in knobs.items() if k.startswith("design.")}
        return json.dumps(self.vehicles.apply_knobs(info["data"], design), indent=2) + "\n"

    def firmware_hash(self) -> str:
        """A hash of what the images are built from (the flight computer's application and the flight core), so that a changed firmware is not mistaken for a built one."""
        paths = [p for d in ("firmware/app", "firmware/common", "core/include") for p in sorted((self.repo / d).rglob("*")) if p.is_file() and "/build" not in str(p)]
        stamp = max((p.stat().st_mtime for p in paths), default=0.0)
        if self._fw_cache and self._fw_cache[0] == stamp:
            return self._fw_cache[1]
        h = hashlib.sha256()
        for p in paths:
            h.update(str(p.relative_to(self.repo)).encode())
            h.update(p.read_bytes())
        self._fw_cache = (stamp, h.hexdigest()[:12])
        return self._fw_cache[1]

    def key(self, name: str, knobs: dict) -> str:
        return hashlib.sha256((self.design_text(name, knobs) + self.firmware_hash()).encode()).hexdigest()[:10]

    def dir_of(self, name: str, knobs: dict) -> Path:
        return self.root / f"{name}-{self.key(name, knobs)}"

    def images(self, name: str, knobs: dict) -> list[Path] | None:
        """The three flight-computer images for this vehicle, if they exist: the reference's own for the untouched reference, the built ones for anything else."""
        if self.reference_untouched(name, knobs):
            exes = [self.repo / "build" / f"launch_{n}" / "zephyr" / "zephyr.exe" for n in NODES]
        else:
            d = self.dir_of(name, knobs)
            exes = [d / f"launch_{n}" / "zephyr" / "zephyr.exe" for n in NODES]
            if not (d / "stamp.json").is_file():
                return None
        return exes if all(p.is_file() and os.access(p, os.X_OK) for p in exes) else None

    def toolchain(self) -> str | None:
        """None if `west` can be run the way firmware/env.sh sets it up; else what is missing."""
        venv = Path(os.environ.get("TFC_VENV") or self.repo.parent / ".venv")
        if not (venv / "bin" / "west").exists():
            return f"the Zephyr environment is not there: no west in {venv}/bin (firmware/README.md, firmware/env.sh)"
        if not self.gen.exists():
            return f"{self.gen.relative_to(self.repo) if self.gen.is_relative_to(self.repo) else self.gen} is not built (cmake --build build/host)"
        if not (self.repo / "build" / "act_native" / "zephyr" / "zephyr.exe").exists():
            return "the actuator node's image build/act_native is not built (tools/bench/sil_triplex.sh --build --launch)"
        return None

    # ------------------------------------------------------------------ the build
    def status(self, name: str, knobs: dict) -> dict:
        """What the page shows for the chosen vehicle: ready, needs building, building, failed. (Asked ten times a second by the hub: the answer is kept for a second unless a build is running.)"""
        ck = (name, json.dumps(knobs, sort_keys=True))
        now = time.monotonic()
        with self.lock:
            hit = self._status_cache
            if hit and hit[0] == ck and now - hit[1] < 1.0 and self.state != "building":
                return hit[2]
        out = self._status(name, knobs)
        with self.lock:
            self._status_cache = (ck, now, out)
        return out

    def _status(self, name: str, knobs: dict) -> dict:
        with self.lock:
            out: dict = {"vehicle": name, "reference": self.reference_untouched(name, knobs), "state": "idle", "steps": list(self.steps), "log": list(self.log)[-30:], "error": self.error, "seconds": round(self.seconds, 1)}
            try:
                imgs = self.images(name, knobs)
                key = None if out["reference"] else self.key(name, knobs)
            except (ValueError, OSError) as e:
                return {**out, "state": "failed", "error": str(e)}
            out["key"] = key
            if self.state == "building" and self.building == (name, key):
                out["state"] = "building"
                out["seconds"] = round(time.monotonic() - self.started, 1)
            elif imgs is not None:
                out["state"] = "ready"
                out["images"] = [str(p.relative_to(self.repo)) for p in imgs]
            elif self.state == "failed" and self.building == (name, key):
                out["state"] = "failed"
            else:
                why = None if out["reference"] else self.toolchain()
                out["state"] = "cannot-build" if why else "needs-build"
                out["error"] = why or ""
                if out["reference"]:
                    out["error"] = "the reference vehicle's images are not built (tools/bench/sil_triplex.sh --build --launch)"
            return out

    def prepare(self, name: str, knobs: dict) -> dict:
        """Start building the images for this vehicle if they are not there. Returns the status. One build at a time."""
        with self.lock:
            st = self.status(name, knobs)
            if st["state"] in ("ready", "building", "cannot-build") or st["reference"]:
                return st
            if self.job is not None and self.job.is_alive():
                raise ValueError("a build is already running: wait for it")
            self.state, self.error, self.steps, self.started = "building", "", [], time.monotonic()
            self.log.clear()
            self._status_cache = None
            self.building = (name, self.key(name, knobs))
            design = self.design_text(name, knobs)
            target = self.dir_of(name, knobs)
            self.job = threading.Thread(target=self._build, args=(name, design, target), name="rig-build", daemon=True)
            self.job.start()
        self.hub.note("info", "CON", "rig", f"building the flight computers for {name} (their pitch program and gains are designed on it)")
        return self.status(name, knobs)

    def _step(self, name: str, state: str, detail: str = "") -> None:
        with self.lock:
            for s in self.steps:
                if s["name"] == name:
                    s.update(state=state, detail=detail)
                    return
            self.steps.append({"name": name, "state": state, "detail": detail})

    def _run(self, argv: list[str], what: str, cwd: Path | None = None, shell_env: bool = False) -> tuple[int, str]:
        cmd = ["bash", "-c", ". firmware/env.sh && exec \"$@\"", "bash", *argv] if shell_env else argv
        p = subprocess.Popen(cmd, cwd=cwd or self.repo, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        tail: deque[str] = deque(maxlen=12)
        assert p.stdout is not None
        with p.stdout:
            for line in p.stdout:
                line = line.rstrip()
                if line:
                    tail.append(line)
                    with self.lock:
                        self.log.append(f"[{what}] {line}"[:300])
        return p.wait(), "\n".join(tail)

    def _build(self, name: str, design: str, target: Path) -> None:
        try:
            target.mkdir(parents=True, exist_ok=True)
            vfile = target / "vehicle.json"
            vfile.write_text(design)
            fly = self.vehicles.fly
            self._step("check the vehicle file", "running")
            rc, out = self._run([str(fly), str(vfile), "--check"], "check")
            if rc != 0:
                raise ValueError("the simulator's reader refuses the vehicle:\n" + out.replace(str(vfile), "the vehicle file"))
            self._step("check the vehicle file", "done")
            self._step("design the tables", "running")
            header = target / "flight_tables.hpp"
            rc, out = self._run([str(self.gen), "--vehicle", str(vfile), str(header)], "tables")
            if rc != 0:
                raise ValueError("the design could not make tables for this vehicle:\n" + out)
            self._step("design the tables", "done", f"{header.stat().st_size} bytes")
            for n, letter in enumerate(NODES):
                label = f"build flight computer {letter.upper()}"
                self._step(label, "running")
                d = target / f"launch_{letter}"
                rc, out = self._run(["west", "build", "-p", "auto", "-b", "native_sim/native/64", "firmware/app", "-d", str(d), "--", f"-DCONFIG_TFC_NODE_ID={n}", "-DCONFIG_TFC_FLIGHT_FUNCTION=y",
                                     "-DCONFIG_TFC_SIM_BUS_IMU=y", "-DCONFIG_TFC_LAUNCH_SEQUENCE=y", "-DCONFIG_TFC_ESTIMATOR_NO_ACCEL=y", f"-DTFC_TABLES_HEADER={header}"], f"FC-{letter.upper()}", shell_env=True)
                if rc != 0:
                    raise ValueError(f"the build of flight computer {letter.upper()} failed:\n{out}")
                self._step(label, "done")
            (target / "stamp.json").write_text(json.dumps({"vehicle": name, "firmware": self.firmware_hash(), "built": time.strftime("%Y-%m-%d %H:%M:%S")}) + "\n")
            with self.lock:
                self.state, self.error = "idle", ""
                self._status_cache = None
            self.hub.note("ok", "CON", "rig", f"the flight computers for {name} are built ({time.monotonic() - self.started:.0f} s)")
        except (ValueError, OSError) as e:
            with self.lock:
                self._status_cache = None
                self.state, self.error = "failed", str(e)
                for s in self.steps:
                    if s["state"] == "running":
                        s["state"] = "failed"
            self.hub.note("crit", "CON", "rig", f"the flight computers for {name} could not be built: {str(e).splitlines()[0]}")
        finally:
            with self.lock:
                self.seconds = time.monotonic() - self.started

    def forget_failure(self) -> None:
        with self.lock:
            if self.state == "failed":
                self.state, self.error = "idle", ""
