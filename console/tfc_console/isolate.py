# SPDX-License-Identifier: MIT
"""Keep the rest of the desktop off the CPUs the rig runs on (ADR-040).

The virtual rig is five to seven processes with a 10 ms frame, and `native_sim` hands control between a node's threads about 14 000 times a second, each hand-off a Linux wake-up. When the host has no idle CPU
(a browser starting, a WebGL page drawing a sky, a compile) those wake-ups queue behind other work, a frame arrives late, and the computers do what they are built to do: they latch each other out. This
was measured, not assumed (`docs/design/CONSOLE.md` section 8): twelve busy loops for four seconds on a twelve-thread host latched the nodes whatever their priority or the rig's cgroup weight, six busy loops did
not, and the rig pinned to its own CPUs with the load on the others stayed in Triplex. Real-time scheduling would be the usual cure and needs a privilege this tool does not ask for.

So, without any privilege, two things while a rig runs:
* the rig's processes are pinned to a reserved set of CPUs (whole physical cores, so that no other program shares a core with them through hyper-threading), and
* the other processes **of the same user** are moved to the remaining CPUs, and every process they start inherits that.
Nothing else is touched (other users' processes and the kernel's are not ours to move). The original masks are remembered and put back when the rig stops or the console exits; a process that was started
in the meantime and inherited the other CPUs gets every CPU back. A host with fewer than eight logical CPUs has nothing to share, and the console says so instead of pretending.
"""
from __future__ import annotations

import os
import threading
from pathlib import Path
from typing import Callable, Iterable

MIN_CPUS = 8                 # the rig needs three physical cores of its own and the desktop at least as many
SWEEP_S = 3.0                # how often a process that escaped the move (started by something we did not move) is found


def cpu_list(cpus: Iterable[int]) -> str:
    """{0,1,2,6,7,8} -> '0-2,6-8'."""
    s = sorted(cpus)
    out, i = [], 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and s[j + 1] == s[j] + 1:
            j += 1
        out.append(str(s[i]) if i == j else f"{s[i]}-{s[j]}")
        i = j + 1
    return ",".join(out)


def parse_cpu_list(text: str) -> set[int]:
    out: set[int] = set()
    for part in text.strip().split(","):
        if not part:
            continue
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def core_groups(allowed: set[int], sysfs: Path = Path("/sys/devices/system/cpu")) -> list[frozenset[int]]:
    """The logical CPUs of each physical core (hyper-threads together), lowest first, restricted to the CPUs this process may use. Without a topology every CPU is its own core."""
    groups: dict[frozenset[int], None] = {}
    for c in sorted(allowed):
        try:
            sib = parse_cpu_list((sysfs / f"cpu{c}" / "topology" / "thread_siblings_list").read_text())
        except (OSError, ValueError):
            sib = {c}
        groups[frozenset(sib & allowed or {c})] = None
    return sorted(groups, key=min)


def make_plan(groups: list[frozenset[int]]) -> tuple[frozenset[int], frozenset[int]] | str:
    """(the rig's CPUs, everyone else's), or the reason there is no plan: the rig gets the first half of the physical cores."""
    total = sum(len(g) for g in groups)
    if total < MIN_CPUS or len(groups) < 4:
        return f"only {total} logical CPUs ({len(groups)} cores): the rig and the desktop would have to share them"
    k = len(groups) // 2
    rig = frozenset().union(*groups[:k])
    rest = frozenset().union(*groups[k:])
    return rig, rest


class Isolation:
    """The state and the work. `processes` says which pids may be moved (default: every process of this user); tests give it a list."""

    def __init__(self, hub, enabled: bool = False, processes: Callable[[], Iterable[int]] | None = None, proc_root: Path = Path("/proc"),
                 sysfs: Path = Path("/sys/devices/system/cpu"), sweep_s: float = SWEEP_S, state_file: Path | None = None, plan: tuple[frozenset[int], frozenset[int]] | str | None = None) -> None:
        self.hub = hub
        self.wanted = enabled
        self.proc_root = proc_root
        self.sweep_s = sweep_s
        self.all_cpus = frozenset(os.sched_getaffinity(0))
        self.plan = plan if plan is not None else make_plan(core_groups(set(self.all_cpus), sysfs))      # `plan`: tests give their own
        self._processes = processes
        self.engaged = False
        self.saved: dict[int, frozenset[int]] = {}       # tid -> the mask it had before we moved it
        self.moved_procs: set[int] = set()
        self.left: int = 0                                # processes that could not or need not move
        self._rig_pids: Callable[[], Iterable[int]] = lambda: ()
        self.lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.state_file = state_file
        self._rescue_after_crash()

    # ------------------------------------------------------------------ what it is
    @property
    def available(self) -> bool:
        return not isinstance(self.plan, str)

    @property
    def rig_cpus(self) -> frozenset[int]:
        return self.plan[0] if self.available else frozenset()

    @property
    def rest_cpus(self) -> frozenset[int]:
        return self.plan[1] if self.available else frozenset()

    def status(self) -> dict:
        with self.lock:
            return {"available": self.available, "wanted": self.wanted, "engaged": self.engaged,
                    "reason": self.plan if isinstance(self.plan, str) else "",
                    "rig_cpus": cpu_list(self.rig_cpus), "other_cpus": cpu_list(self.rest_cpus),
                    "moved": len(self.moved_procs), "threads": len(self.saved), "left": self.left}

    def _rescue_after_crash(self) -> None:
        """A console that was killed while it held the isolation left the user's programs on the other CPUs. Its pid file says so; if that process is gone, give those programs the machine back."""
        if self.state_file is None or not self.available:
            return
        try:
            pid = int(self.state_file.read_text().strip())
        except (OSError, ValueError):
            return
        if pid != os.getpid() and (self.proc_root / str(pid)).exists():
            return                                                # that console is alive and holds it
        n = self._give_back_stragglers(set())
        try:
            self.state_file.unlink()
        except OSError:
            pass
        self.hub.note("warn", "CON", "rig", f"isolation: a console (pid {pid}) ended without releasing it; {n} threads of yours have every CPU again")

    def _give_back_stragglers(self, keep: set[int]) -> int:
        """Every thread of this user's that is confined to exactly the rig's CPUs or exactly the others gets all CPUs."""
        n = 0
        for pid in list(self._candidates()):
            for tid in self._tids(pid):
                if tid in keep:
                    continue
                try:
                    cur = frozenset(os.sched_getaffinity(tid))
                except OSError:
                    continue
                if cur in (self.rest_cpus, self.rig_cpus):
                    n += self._set(tid, self.all_cpus)
        return n

    def _mark(self, on: bool) -> None:
        if self.state_file is None:
            return
        try:
            if on:
                self.state_file.parent.mkdir(parents=True, exist_ok=True)
                self.state_file.write_text(str(os.getpid()))
            else:
                self.state_file.unlink()
        except OSError:
            pass

    def pin_self(self) -> None:
        """For the rig's children, between fork and exec: confine this process to the rig's CPUs (a no-op when the isolation is not engaged)."""
        if self.engaged and self.available:
            try:
                os.sched_setaffinity(0, self.rig_cpus)
            except OSError:
                pass

    # ------------------------------------------------------------------ engage and release
    def set_wanted(self, on: bool, rig_running: bool, rig_pids: Callable[[], Iterable[int]]) -> None:
        with self.lock:
            self.wanted = bool(on)
        if on and rig_running:
            self.engage(rig_pids)
        elif not on:
            self.release()

    def engage(self, rig_pids: Callable[[], Iterable[int]] = lambda: ()) -> None:
        """Move the others away, then keep a watch for stragglers. Does nothing when not wanted, not available or already engaged."""
        with self.lock:
            if not self.wanted or not self.available or self.engaged:
                return
            self.engaged = True
            self._rig_pids = rig_pids
        self._mark(True)
        self._sweep()
        self.pin_rig()
        self._stop.clear()
        self._thread = threading.Thread(target=self._watch, name="rig-isolation", daemon=True)
        self._thread.start()
        s = self.status()
        self.hub.note("info", "CON", "rig", f"isolation: the rig runs on CPUs {s['rig_cpus']}; {s['moved']} of your processes ({s['threads']} threads) moved to {s['other_cpus']}, put back when the rig stops")

    def pin_rig(self) -> None:
        """Pin every thread of the rig's processes (some are already, from their start; this catches the rest, and a rig that was running when the isolation was switched on)."""
        if not self.engaged:
            return
        for pid in list(self._rig_pids()):
            for tid in self._tids(pid):
                self._set(tid, self.rig_cpus)

    def release(self) -> None:
        with self.lock:
            if not self.engaged:
                return
            self.engaged = False
            saved, self.saved = self.saved, {}
            moved = len(self.moved_procs)
            self.moved_procs = set()
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self._thread = None
        back = 0
        for tid, mask in saved.items():
            back += self._set(tid, mask)
        # whatever inherited the other CPUs in the meantime and is still confined to exactly them gets the whole machine back, as does the rig's own children
        self._give_back_stragglers(set(saved))
        self._mark(False)
        self.hub.note("info", "CON", "rig", f"isolation released: {moved} processes ({back} threads) have their CPUs back")

    # ------------------------------------------------------------------ the work
    def _candidates(self) -> Iterable[int]:
        if self._processes is not None:
            return list(self._processes())
        me = os.getuid()
        out = []
        try:
            names = os.listdir(self.proc_root)
        except OSError:
            return out
        for n in names:
            if n.isdigit():
                try:
                    if (self.proc_root / n).stat().st_uid == me:
                        out.append(int(n))
                except OSError:
                    pass
        return out

    def _tids(self, pid: int) -> list[int]:
        try:
            return [int(t) for t in os.listdir(self.proc_root / str(pid) / "task") if t.isdigit()]
        except OSError:
            return []

    def _pgrp(self, pid: int) -> int | None:
        try:
            return int((self.proc_root / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[2])
        except (OSError, ValueError, IndexError):
            return None

    @staticmethod
    def _set(tid: int, mask: Iterable[int]) -> int:
        try:
            os.sched_setaffinity(tid, set(mask))
            return 1
        except OSError:
            return 0

    def _sweep(self) -> None:
        """Move every thread of every candidate process that can still run on the rig's CPUs onto the other CPUs. The rig's own processes (by process group) stay where they are."""
        rig_groups = {g for g in (self._pgrp(p) for p in self._rig_pids()) if g}
        me = os.getpid()
        left = 0
        for pid in self._candidates():
            if pid != me and self._pgrp(pid) in rig_groups:
                continue
            tids = self._tids(pid)
            did = False
            for tid in tids:
                try:
                    cur = frozenset(os.sched_getaffinity(tid))
                except OSError:
                    continue
                if not (cur & self.rig_cpus):
                    continue                                  # already off the rig's CPUs
                new = cur & self.rest_cpus
                if not new:
                    left += 1                                 # confined to the rig's CPUs by someone else: not ours to undo
                    continue
                with self.lock:
                    self.saved.setdefault(tid, cur)
                if self._set(tid, new):
                    did = True
                else:
                    left += 1
                    with self.lock:
                        self.saved.pop(tid, None)
            if did:
                with self.lock:
                    self.moved_procs.add(pid)
        self.left = left

    def _watch(self) -> None:
        while not self._stop.wait(self.sweep_s):
            try:
                self._sweep()
                self.pin_rig()
            except Exception as e:                            # a watcher must never die of a process that vanished
                self.hub.note("warn", "CON", "rig", f"isolation sweep: {e}")
