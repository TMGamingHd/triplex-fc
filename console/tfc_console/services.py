# SPDX-License-Identifier: MIT
"""The services that act on the rig, and their routes. Installed by `cli.main` after the core routes."""
from __future__ import annotations

from pathlib import Path

from .app import config_payload, default_state_dir
from .commands import CommandService, catalog
from .faultlab import EXPECT, FaultLab
from .hardware import Hardware
from .hub import Hub
from .rig import Rig
from .server import ApiError, App, Request
from .vehicles import Vehicles


def _bad(e: Exception) -> ApiError:
    return ApiError(504 if isinstance(e, TimeoutError) else 400, str(e))


def install(app: App, hub: Hub, mgr, truth, repo: Path, rig: bool = True, state_dir: Path | None = None) -> dict:
    vehicles = Vehicles(repo, state_dir or default_state_dir())
    commands = CommandService(hub, lambda: mgr.iface)
    hardware = Hardware(hub)
    rig_svc = Rig(hub, repo, lambda: mgr.iface, lambda: truth.port, vehicles.sim_args) if rig else None
    lab = FaultLab(hub, rig_svc) if rig_svc else None

    hub.extra["commands"] = commands.snapshot
    hub.extra["hardware"] = hardware.snapshot
    hub.extra["vehicle"] = vehicles.snapshot
    hub.extra["rig"] = rig_svc.snapshot if rig_svc else (lambda: {"disabled": True})
    hub.extra["faults"] = lab.snapshot if lab else (lambda: {"available": False, "table": [], "disabled": True})
    app.hello_extra = lambda: {"config": {**config_payload(), "commands": catalog(), "knobs": vehicles.knob_table(), "expect": {k: list(v) for k, v in EXPECT.items()}, "truth_port": truth.port}, "vehicles": vehicles.listing()}
    app.on_close.append(hardware.shutdown)
    if rig_svc:
        app.on_close.insert(0, rig_svc.shutdown)

    # ---------------------------------------------------------------- commands
    @app.route("POST", "/api/command")
    def _command(req: Request):
        b = req.body
        rec = commands.send(str(b.get("op", "")), b.get("target"), str(b.get("mode", "auto")), bool(b.get("confirmed")), bool(b.get("force")))
        return 200, rec

    # ---------------------------------------------------------------- the rig and the faults
    def need_rig() -> Rig:
        if rig_svc is None:
            raise ApiError(409, "the console was started with --no-rig")
        return rig_svc

    @app.route("POST", "/api/rig/start")
    def _rig_start(req: Request):
        try:
            need_rig().start_profile(str(req.body.get("profile", "")))
        except ValueError as e:
            raise _bad(e) from e
        return 200, rig_svc.snapshot()

    @app.route("POST", "/api/rig/stop")
    def _rig_stop(req: Request):
        try:
            need_rig().stop(req.body.get("name") or None)
        except ValueError as e:
            raise _bad(e) from e
        return 200, rig_svc.snapshot()

    @app.route("POST", "/api/rig/proc")
    def _rig_proc(req: Request):
        r = need_rig()
        action = str(req.body.get("action", ""))
        name = str(req.body.get("name", ""))
        fn = {"kill": r.kill, "freeze": r.freeze, "resume": r.resume, "restart": r.restart}.get(action)
        if fn is None:
            raise ApiError(400, "action is kill, freeze, resume or restart")
        try:
            fn(name)
        except ValueError as e:
            raise _bad(e) from e
        return 200, rig_svc.snapshot()

    @app.route("POST", "/api/faults/add")
    def _fault_add(req: Request):
        if lab is None:
            raise ApiError(409, "the console was started with --no-rig")
        try:
            frames = req.body.get("frames")
            return 200, lab.add(str(req.body.get("spec", "")), int(frames) if frames else None)
        except (ValueError, TimeoutError) as e:
            raise _bad(e) from e

    @app.route("POST", "/api/faults/clear")
    def _fault_clear(req: Request):
        if lab is None:
            raise ApiError(409, "the console was started with --no-rig")
        try:
            lab.clear(str(req.body.get("id", "all")))
        except (ValueError, TimeoutError) as e:
            raise _bad(e) from e
        return 200, lab.snapshot()

    # ---------------------------------------------------------------- the serial hardware
    @app.route("POST", "/api/hardware/connect")
    def _hw_connect(req: Request):
        b = req.body
        try:
            hardware.connect(str(b.get("kind", "")), str(b.get("port", "")), b.get("role"))
        except ImportError:
            raise ApiError(409, "pyserial is not installed (pip install pyserial)") from None
        except (ValueError, OSError) as e:
            raise _bad(e) from e
        return 200, hardware.snapshot()

    @app.route("POST", "/api/hardware/disconnect")
    def _hw_disconnect(req: Request):
        hardware.disconnect(str(req.body.get("kind", "")), req.body.get("role"))
        return 200, hardware.snapshot()

    @app.route("POST", "/api/supervisor")
    def _sup(req: Request):
        try:
            hardware.supervisor_command(str(req.body.get("line", "")))
        except ValueError as e:
            raise _bad(e) from e
        return 200, {"ok": True}

    @app.route("POST", "/api/pico")
    def _pico(req: Request):
        b = req.body
        try:
            if b.get("op") == "cut":
                hardware.pico_cut(str(b.get("node", "")), int(b.get("ms", 0)))
            elif b.get("op") == "restore":
                hardware.pico_restore(str(b.get("node", "all")))
            else:
                raise ValueError("op is cut or restore")
        except ValueError as e:
            raise _bad(e) from e
        return 200, {"ok": True}

    # ---------------------------------------------------------------- the vehicle
    @app.route("GET", "/api/vehicles")
    def _vehicles(req: Request):
        return 200, {"vehicles": vehicles.listing(), "knobs": vehicles.knob_table()}

    @app.route("GET", "/api/vehicle")
    def _vehicle(req: Request):
        try:
            return 200, vehicles.read(req.arg("name", "reference"))
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    def _text_of(b: dict) -> str:
        """The vehicle text a request means: the raw text, or a named vehicle with the knobs applied."""
        import json
        if b.get("text") is not None and not b.get("knobs"):
            return str(b["text"])
        if b.get("text") is not None:
            from .vehicles import strip_comments
            data = json.loads(strip_comments(str(b["text"])))
        else:
            data = vehicles.read(str(b.get("name", "reference")))["data"]
        return json.dumps(vehicles.apply_knobs(data, b.get("knobs") or {}), indent=2) + "\n"

    @app.route("POST", "/api/vehicle/check")
    def _vcheck(req: Request):
        try:
            return 200, vehicles.check(_text_of(req.body))
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    @app.route("POST", "/api/vehicle/preview")
    def _vpreview(req: Request):
        try:
            text = _text_of(req.body)
        except (ValueError, OSError) as e:
            raise _bad(e) from e
        sensors, pad = str(req.body.get("sensors", "platform")), int(req.body.get("pad", 0) or 0)
        return 202, {"job": vehicles.jobs.start("preview", lambda: vehicles.preview(text, sensors, pad))}

    @app.route("GET", "/api/job")
    def _job(req: Request):
        j = vehicles.jobs.get(req.arg("id", ""))
        if j is None:
            raise ApiError(404, "no such job")
        return 200, j

    @app.route("GET", "/api/vehicle/nominal")
    def _nominal(req: Request):
        name = req.arg("name", vehicles.active["name"]) or "reference"
        return 202, {"job": vehicles.jobs.start("nominal", lambda: vehicles.nominal(name))}

    @app.route("POST", "/api/vehicle/save")
    def _vsave(req: Request):
        try:
            return 200, vehicles.save(str(req.body.get("name", "")), _text_of(req.body))
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    @app.route("POST", "/api/vehicle/active")
    def _vactive(req: Request):
        try:
            return 200, vehicles.set_active(str(req.body.get("name", "reference")), req.body.get("knobs") or {})
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    return {"vehicles": vehicles, "commands": commands, "hardware": hardware, "rig": rig_svc, "lab": lab}
