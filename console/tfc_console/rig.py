# SPDX-License-Identifier: MIT
"""The rig's processes: start the virtual rig (three flight computers, ACT and the simulator, as separate processes on one `vcan0`), watch them, and break them on purpose.

A *profile* is a list of processes and the order to start them in (the order of `docs/procedures/P-S2-02-launch-checklist.md`: the simulator first, then the computers and ACT). Every process runs on a pseudo-terminal,
so its console is line-buffered, and each line it prints goes to the hub tagged with its role (A, B, C, ACT, SIM, PEERS): the page shows the same console a terminal would, and the event log gets the reasons
(`node B LATCHED OUT: vote disagreement`) that the bus does not carry. Children are started in their own session and die with the console (`PR_SET_PDEATHSIG`), so a console that is killed does not leave a
rig running that nobody can see.

Process faults are the virtual rig's counterpart of the injector's relays: `kill` (a power cut: the node falls silent at once), `freeze` (SIGSTOP: a hung node, still powered), `resume`, `restart` (a reboot).
They are what the live tests do by hand, and, like the relays, they are test actions: the console says so on the event log.
"""
from __future__ import annotations

import ctypes
import os
import pty
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .hub import Hub


def _die_with_parent() -> None:
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)    # PR_SET_PDEATHSIG
    except (OSError, AttributeError):
        pass


@dataclass
class ProcSpec:
    name: str                   # the role: A, B, C, ACT, SIM, PEERS
    argv: list[str]
    cwd: Path
    binary: Path | None = None  # what has to exist for it to start
    delay_s: float = 0.0        # wait this long after the previous process before starting
    control: bool = False       # keep stdin as a pipe for commands (the peers' --control)
    title: str = ""


@dataclass
class Proc:
    spec: ProcSpec
    popen: subprocess.Popen | None = None
    master: int | None = None
    state: str = "stopped"      # stopped, running, frozen, exited
    started: float = 0.0
    exit_code: int | None = None
    restarts: int = 0
    lines: int = 0
    reader: threading.Thread | None = None


class Rig:
    def __init__(self, hub: Hub, repo: Path, iface_of: Callable[[], str | None], telemetry_port: Callable[[], int], sim_args: Callable[[], list[str]], viewer_port: Callable[[], int] | None = None, images: Callable[[], tuple[list[Path] | None, str]] | None = None) -> None:
        self.hub, self.repo = hub, repo
        self.iface_of, self.telemetry_port, self.sim_args = iface_of, telemetry_port, sim_args
        self.viewer_port = viewer_port
        self.images = images                                       # the three flight-computer images for the vehicle the rig is given, and its name (rigbuild.RigBuilder); None: the reference vehicle's, as always
        self.procs: dict[str, Proc] = {}
        self.profile: str | None = None
        self.lock = threading.RLock()
        self.line_hooks: list[Callable[[str, str], None]] = []     # called with (process name, line) for every line: how the fault service reads the peers' answers
        self._starter: threading.Thread | None = None
        self.message = ""
        # Every child is started by this one thread. The parent-death signal (PR_SET_PDEATHSIG) fires when the *thread* that started the child ends, not the process, so a child started from a short-lived
        # request thread would be killed as soon as that thread returned. This thread lives as long as the console does.
        self._spawn_jobs: queue.Queue = queue.Queue()
        threading.Thread(target=self._spawner, name="rig-spawner", daemon=True).start()

    # ------------------------------------------------------------------ profiles
    def _exe(self, *candidates: str) -> Path | None:
        for c in candidates:
            p = self.repo / c / "zephyr" / "zephyr.exe"
            if p.exists() and os.access(p, os.X_OK):
                return p
        return None

    def profiles(self) -> dict[str, dict]:
        iface = self.iface_of() or "vcan0"
        simd = self.repo / "build" / "host" / "tfc_simd"
        act = self._exe("build/act_native")
        out: dict[str, dict] = {}
        # ---- the closed loop with the launch sequence: three real flight computers, ACT, the simulator clamped on the pad
        built, vname = self.images() if self.images else (None, "reference")
        fcs = list(built) if built else ([None, None, None] if self.images and vname != "reference" else [self._exe(f"build/launch_{n}") for n in "abc"])
        procs = [ProcSpec("SIM", [str(simd), "--iface", iface, "--hold", "--quiet", "--telemetry", str(self.telemetry_port()), *(["--viewer", str(self.viewer_port())] if self.viewer_port else []), *self.sim_args()], self.repo, simd if simd.exists() else None, 0.0, title="vehicle simulator (tfc_simd --hold)")]
        procs += [ProcSpec(n, [str(p) if p else ""], self.repo, p, 0.4 if i == 0 else 0.0, title=f"flight computer {n} (launch image)") for i, (n, p) in enumerate(zip("ABC", fcs))]
        procs.append(ProcSpec("ACT", [str(act) if act else ""], self.repo, act, 0.0, title="actuator node"))
        out["closed-loop"] = {"title": "Closed loop with the launch sequence", "doc": f"Three real flight-computer processes (nodes A, B and C, each calibrating its own IMU on the pad), the actuator node and the vehicle simulator clamped on the pad, flying: {vname}. "
                              "Launch it from the Launch tab after about 12 s. Needs vcan0 and the launch images (the reference vehicle's: tools/bench/sil_triplex.sh --build --launch; any other vehicle's are built from the Launch tab).", "procs": procs}
        # ---- the fault lab: FC-A is real, B and C are virtual peers whose faults the console changes while it runs
        a = self._exe("build/triplex_a", "build/native_sim")
        py = sys.executable
        lab = [ProcSpec("A", [str(a) if a else ""], self.repo, a, 0.0, title="flight computer A (scripted command)")]
        lab.append(ProcSpec("ACT", [str(act) if act else ""], self.repo, act, 0.0, title="actuator node"))
        lab.append(ProcSpec("PEERS", [py, "-m", "tfc_peers", "run", "--follow-sync", "--nodes", "B,C", "--frames", "0", "--control", "--iface", iface], self.repo / "sim", Path(py), 1.2, control=True, title="virtual FC-B and FC-C (tfc_peers --control)"))
        out["fault-lab"] = {"title": "Fault lab: FC-A and virtual B and C", "doc": "One real flight-computer process (node A) and virtual B and C that follow its SYNC. The Faults tab adds and clears any of the 32 fault kinds on B and C while this runs, and the Voting tab shows what the real FC-A "
                            "and ACT do about each. No vehicle: the sensors are the peers' scripted motion. ACT starts with B and C excluded (the peers join a second after it) and never readmits by itself: send reintegrate once.", "procs": lab}
        for p in out.values():
            p["missing"] = [s.name for s in p["procs"] if s.binary is None or not Path(s.binary).exists()]
            p["ready"] = not p["missing"]
        return out

    # ------------------------------------------------------------------ lifecycle
    def start_profile(self, name: str) -> None:
        with self.lock:
            prof = self.profiles().get(name)
            if prof is None:
                raise ValueError(f"unknown profile {name!r}")
            if not prof["ready"]:
                raise ValueError(f"{prof['title']}: not built yet ({', '.join(prof['missing'])}); the reference vehicle's images: tools/bench/sil_triplex.sh --build --launch; another vehicle's are built from the Launch tab)")
            if self.iface_of() is None or self.hub.replay:
                raise ValueError("attach the console to a live interface (vcan0) before starting a rig on it")
            if any(p.state in ("running", "frozen") for p in self.procs.values()):
                raise ValueError("a rig is already running: stop it first")
            self.procs = {s.name: Proc(s) for s in prof["procs"]}
            self.profile = name
            self.message = f"starting {prof['title']}"
        self.hub.note("info", "CON", "rig", f"starting the rig: {prof['title']}")
        self._starter = threading.Thread(target=self._start_all, args=([s for s in prof["procs"]],), name="rig-start", daemon=True)
        self._starter.start()

    def _start_all(self, specs: list[ProcSpec]) -> None:
        for s in specs:
            if s.delay_s:
                time.sleep(s.delay_s)
            if self.procs.get(s.name) is None:
                return                                 # stopped while starting
            self._spawn(self.procs[s.name])
        self.message = "running"

    def _spawner(self) -> None:
        while True:
            fn, done = self._spawn_jobs.get()
            try:
                fn()
            finally:
                done.set()

    def _spawn(self, p: Proc) -> None:
        done = threading.Event()
        self._spawn_jobs.put((lambda: self._spawn_here(p), done))
        done.wait(5.0)

    def _spawn_here(self, p: Proc) -> None:
        spec = p.spec
        master, slave = pty.openpty()
        stdin = subprocess.PIPE if spec.control else subprocess.DEVNULL
        try:
            p.popen = subprocess.Popen(spec.argv, cwd=str(spec.cwd), stdin=stdin, stdout=slave, stderr=slave, start_new_session=True, close_fds=True, preexec_fn=_die_with_parent, env={**os.environ, "PYTHONUNBUFFERED": "1"})  # noqa: PLW1509
        except OSError as e:
            os.close(master)
            os.close(slave)
            p.state, p.exit_code = "exited", -1
            self.hub.note("crit", "CON", "rig", f"could not start {spec.name}: {e}")
            return
        os.close(slave)
        p.master, p.state, p.started, p.exit_code = master, "running", time.monotonic(), None
        p.reader = threading.Thread(target=self._read, args=(p,), name=f"rig-{spec.name}", daemon=True)
        p.reader.start()

    def _read(self, p: Proc) -> None:
        buf = b""
        fd = p.master
        while True:
            try:
                chunk = os.read(fd, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").rstrip("\r")
                if text.strip():
                    p.lines += 1
                    self.hub.ingest_line(p.spec.name, text)
                    for fn in list(self.line_hooks):
                        fn(p.spec.name, text)
        code = p.popen.wait() if p.popen is not None else None
        with self.lock:
            if p.state != "stopped":
                p.state, p.exit_code = "exited", code
        try:
            os.close(fd)
        except OSError:
            pass
        self.hub.note("info" if p.state == "stopped" or code in (0, None, -signal.SIGTERM, -signal.SIGKILL) else "warn", "CON", "rig", f"{p.spec.name} ended (exit code {code})")

    def _signal(self, p: Proc, sig: int) -> None:
        if p.popen is not None and p.popen.poll() is None:
            try:
                os.killpg(os.getpgid(p.popen.pid), sig)
            except (ProcessLookupError, PermissionError):
                pass

    def stop(self, name: str | None = None) -> None:
        """Stop one process, or all of them (SIGTERM, then SIGKILL after a second)."""
        with self.lock:
            targets = [self.procs[name]] if name else list(self.procs.values())
            if name and name not in self.procs:
                raise ValueError(f"no process {name!r} in the rig")
        for p in targets:
            if p.state == "frozen":
                self._signal(p, signal.SIGCONT)
            if p.state in ("running", "frozen"):
                p.state = "stopped"
                self._signal(p, signal.SIGTERM)
        deadline = time.monotonic() + 1.0
        for p in targets:
            while p.popen is not None and p.popen.poll() is None and time.monotonic() < deadline:
                time.sleep(0.02)
            if p.popen is not None and p.popen.poll() is None:
                self._signal(p, signal.SIGKILL)
        if name is None:
            self.hub.note("info", "CON", "rig", "the rig was stopped")
            self.profile = None
            self.message = "stopped"

    def kill(self, name: str) -> None:
        p = self._proc(name)
        self.hub.note("warn", "CON", "fault", f"TEST ACTION: node {name} killed (a power cut: it falls silent at once)")
        p.state = "exited"
        self._signal(p, signal.SIGKILL)

    def freeze(self, name: str) -> None:
        p = self._proc(name)
        if p.state != "running":
            raise ValueError(f"{name} is {p.state}, not running")
        self.hub.note("warn", "CON", "fault", f"TEST ACTION: node {name} frozen (SIGSTOP: a hung node, still powered)")
        p.state = "frozen"
        self._signal(p, signal.SIGSTOP)

    def resume(self, name: str) -> None:
        p = self._proc(name)
        if p.state != "frozen":
            raise ValueError(f"{name} is {p.state}, not frozen")
        self.hub.note("info", "CON", "fault", f"node {name} resumed (SIGCONT)")
        p.state = "running"
        self._signal(p, signal.SIGCONT)

    def restart(self, name: str) -> None:
        p = self._proc(name)
        self.hub.note("warn", "CON", "fault", f"TEST ACTION: node {name} restarted (a reboot: it comes back as a new boot)")
        if p.state in ("running", "frozen"):
            p.state = "stopped"
            self._signal(p, signal.SIGCONT)
            self._signal(p, signal.SIGKILL)
            if p.popen is not None:
                try:
                    p.popen.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
        p.restarts += 1
        self._spawn(p)

    def _proc(self, name: str) -> Proc:
        with self.lock:
            p = self.procs.get(name)
        if p is None:
            raise ValueError(f"no process {name!r} in the rig")
        return p

    def write_line(self, name: str, text: str) -> None:
        p = self._proc(name)
        if p.popen is None or p.popen.stdin is None or p.state != "running":
            raise ValueError(f"{name} is not running with a control channel")
        p.popen.stdin.write((text.strip() + "\n").encode())
        p.popen.stdin.flush()

    def shutdown(self) -> None:
        try:
            self.stop(None)
        except ValueError:
            pass

    def snapshot(self) -> dict:
        with self.lock:
            procs = []
            for p in self.procs.values():
                if p.popen is not None and p.state == "running" and p.popen.poll() is not None:
                    p.state, p.exit_code = "exited", p.popen.returncode
                procs.append({"name": p.spec.name, "title": p.spec.title, "state": p.state, "pid": p.popen.pid if p.popen else None, "uptime": round(time.monotonic() - p.started, 1) if p.state in ("running", "frozen") else None,
                              "exit_code": p.exit_code, "restarts": p.restarts, "lines": p.lines})
            profs = {k: {"title": v["title"], "doc": v["doc"], "ready": v["ready"], "missing": v["missing"], "procs": [s.name for s in v["procs"]]} for k, v in self.profiles().items()}
            return {"profile": self.profile, "procs": procs, "profiles": profs, "message": self.message}
