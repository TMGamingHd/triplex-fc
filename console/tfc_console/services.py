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
from .rigbuild import RigBuilder
from .server import ApiError, App, Raw, Request
from .vehicles import Vehicles
from .viewer import ViewerService


def _safe(builder, vehicles) -> bool:
    """Whether the chosen vehicle can be asked about (a file the reader cannot read has no images)."""
    try:
        builder.key(vehicles.active["name"], vehicles.active["knobs"])
        return True
    except (ValueError, OSError):
        return False


def _bad(e: Exception) -> ApiError:
    return ApiError(504 if isinstance(e, TimeoutError) else 400, str(e))


def install(app: App, hub: Hub, mgr, truth, repo: Path, rig: bool = True, state_dir: Path | None = None, pose=None) -> dict:
    vehicles = Vehicles(repo, state_dir or default_state_dir())
    commands = CommandService(hub, lambda: mgr.iface)
    hardware = Hardware(hub)
    builder = RigBuilder(hub, repo, vehicles)
    rig_svc = Rig(hub, repo, lambda: mgr.iface, lambda: truth.port, vehicles.sim_args, (lambda: pose.port) if pose is not None else None,
                  lambda: (builder.images(vehicles.active["name"], vehicles.active["knobs"]) if _safe(builder, vehicles) else None, vehicles.active["name"])) if rig else None
    viewer = ViewerService(hub, repo, (state_dir or default_state_dir()), vehicles)
    app.viewer = hub.viewer
    app.on_close.append(viewer.close)
    lab = FaultLab(hub, rig_svc) if rig_svc else None

    hub.extra["commands"] = commands.snapshot
    hub.extra["hardware"] = hardware.snapshot
    hub.extra["vehicle"] = vehicles.snapshot
    hub.extra["rigbuild"] = lambda: builder.status(vehicles.active["name"], vehicles.active["knobs"])
    hub.extra["rig"] = rig_svc.snapshot if rig_svc else (lambda: {"disabled": True})
    hub.extra["faults"] = lab.snapshot if lab else (lambda: {"available": False, "table": [], "disabled": True})
    app.hello_extra = lambda: {"config": {**config_payload(), "commands": catalog(), "knobs": vehicles.knob_table(), "expect": {k: list(v) for k, v in EXPECT.items()}, "truth_port": truth.port, "pose_port": pose.port if pose is not None else None}, "vehicles": vehicles.listing()}
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

    @app.route("GET", "/api/rig/vehicle")
    def _rig_vehicle(req: Request):
        return 200, {"active": vehicles.active, "build": builder.status(vehicles.active["name"], vehicles.active["knobs"]), "vehicles": vehicles.listing()}

    @app.route("POST", "/api/rig/vehicle")
    def _rig_vehicle_set(req: Request):
        """Choose the vehicle the rig flies and, if its flight computers are not built, start building them. The rig must be stopped: a running one has the vehicle it started with."""
        if rig_svc is not None and any(p.state in ("running", "frozen") for p in rig_svc.procs.values()):
            raise ApiError(409, "the rig is running with its vehicle: stop it first, then choose another")
        try:
            vehicles.set_active(str(req.body.get("name", "reference")), req.body.get("knobs") or {})
            builder.forget_failure()
            return 200, {"active": vehicles.active, "build": builder.prepare(vehicles.active["name"], vehicles.active["knobs"])}
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    # ---------------------------------------------------------------- the 3D viewer
    @app.route("GET", "/api/viewer/state")
    def _viewer_state(req: Request):
        return 200, {**hub.viewer.status(), "pose_files": viewer.pose_files(), "models": viewer.model_files(), "vehicles": vehicles.listing(), "pose_port": pose.port if pose is not None else None,
                     "flying": viewer.flying, "fly_available": vehicles.fly.exists()}

    @app.route("POST", "/api/viewer/open")
    def _viewer_open(req: Request):
        b = req.body
        try:
            return 200, viewer.open(str(b.get("name", "")), float(b.get("speed", 1.0)), bool(b.get("loop")), b.get("autoplay", True) is not False)
        except (ValueError, OSError) as e:
            raise _bad(e) from e

    @app.route("POST", "/api/viewer/control")
    def _viewer_control(req: Request):
        try:
            return 200, viewer.control(str(req.body.get("action", "")), req.body.get("value"))
        except (ValueError, TypeError) as e:
            raise _bad(e) from e

    @app.route("POST", "/api/viewer/close")
    def _viewer_close(req: Request):
        viewer.close()
        return 200, hub.viewer.status()

    @app.route("POST", "/api/viewer/fly")
    def _viewer_fly(req: Request):
        try:
            text = _text_of(req.body)
        except (ValueError, OSError) as e:
            raise _bad(e) from e
        if viewer.flying is not None:
            raise ApiError(409, "a flight is being flown already")
        label = str(req.body.get("name") or "flight")
        sensors, pad = str(req.body.get("sensors", "vehicle")), int(req.body.get("pad", 300) or 0)

        def job():
            res = viewer.fly(text, label, sensors, pad)
            if mgr.source is not None and getattr(mgr.source, "playing", False) is True and hub.replay:
                mgr.disconnect()                    # a bus replay is playing: the flown flight takes the viewer, the bus replay is stopped
            viewer.open(res["file"], 1.0, False, True)
            return res
        return 202, {"job": vehicles.jobs.start("fly", job)}

    @app.route("POST", "/api/viewer/log")
    def _viewer_log(req: Request):
        """The page's own console, for a browser that has none at hand (a headless one under test): the line goes to this process's standard error, and nowhere else."""
        import sys
        print("viewer:", str(req.body.get("text", ""))[:2000], file=sys.stderr, flush=True)
        return 200, {"ok": True}

    @app.route("GET", "/api/viewer/model")
    def _viewer_model(req: Request):
        try:
            body, ctype = viewer.model_bytes(req.arg("name", "") or "")
        except ValueError as e:
            raise ApiError(404, str(e)) from e
        return 200, Raw(body, ctype)

    @app.route("GET", "/api/viewer/imagery")
    def _viewer_imagery(req: Request):
        return 200, viewer.imagery.manifest()

    @app.route("GET", "/api/viewer/imagery/file")
    def _viewer_imagery_file(req: Request):
        try:
            body, ctype = viewer.imagery.file_bytes(req.arg("name", "") or "")
        except ValueError as e:
            raise ApiError(404, str(e)) from e
        return 200, Raw(body, ctype)

    return {"vehicles": vehicles, "commands": commands, "hardware": hardware, "rig": rig_svc, "lab": lab, "viewer": viewer, "builder": builder}
