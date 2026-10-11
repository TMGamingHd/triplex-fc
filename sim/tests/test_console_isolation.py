# SPDX-License-Identifier: MIT
"""CPU isolation of the rig (ADR-040): the plan from the topology, moving other programs off the rig's CPUs and putting them back, what it leaves alone, the crash rescue, and that the rig's children are pinned.

The moves are real (`sched_setaffinity` on child processes started here), on whatever CPUs the host gives this test; a host with fewer than two skips them."""
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.hub import Hub
from tfc_console.isolate import Isolation, core_groups, cpu_list, make_plan, parse_cpu_list

CPUS = sorted(os.sched_getaffinity(0))


def mask(pid: int) -> frozenset[int]:
    return frozenset(os.sched_getaffinity(pid))


def topology(root: Path, siblings: dict[int, str]) -> None:
    for c, text in siblings.items():
        d = root / f"cpu{c}" / "topology"
        d.mkdir(parents=True, exist_ok=True)
        (d / "thread_siblings_list").write_text(text + "\n")


class Plan(unittest.TestCase):
    def test_cpu_lists_round_trip(self):
        for s in ({0, 1, 2, 6, 7, 8}, {5}, {0, 2, 4}, {3, 4, 5, 9, 10, 11}, set()):
            self.assertEqual(parse_cpu_list(cpu_list(s)), s)
        self.assertEqual(cpu_list({0, 1, 2, 6, 7, 8}), "0-2,6-8")
        self.assertEqual(parse_cpu_list("0-2,6\n"), {0, 1, 2, 6})

    def test_the_rig_gets_the_first_half_of_the_physical_cores_with_their_hyper_threads(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            topology(root, {c: f"{c % 6},{c % 6 + 6}" for c in range(12)})
            groups = core_groups(set(range(12)), root)
            self.assertEqual([sorted(g) for g in groups], [[0, 6], [1, 7], [2, 8], [3, 9], [4, 10], [5, 11]])
            rig, rest = make_plan(groups)
            self.assertEqual(sorted(rig), [0, 1, 2, 6, 7, 8])
            self.assertEqual(sorted(rest), [3, 4, 5, 9, 10, 11])
            self.assertFalse(rig & rest)

    def test_a_cpu_the_process_may_not_use_is_left_out(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            topology(root, {c: f"{c % 4},{c % 4 + 4}" for c in range(8)})
            groups = core_groups({0, 1, 2, 3, 4, 5, 6}, root)               # CPU 7 is not ours: core 3 has one thread
            self.assertEqual(sorted(sum((sorted(g) for g in groups), [])), [0, 1, 2, 3, 4, 5, 6])

    def test_without_a_topology_every_cpu_is_a_core(self):
        with tempfile.TemporaryDirectory() as d:
            groups = core_groups(set(range(8)), Path(d))
            self.assertEqual(len(groups), 8)
            rig, rest = make_plan(groups)
            self.assertEqual((sorted(rig), sorted(rest)), ([0, 1, 2, 3], [4, 5, 6, 7]))

    def test_a_small_host_gets_no_plan_and_a_reason(self):
        for n in (1, 2, 4, 6):
            why = make_plan([frozenset({c}) for c in range(n)])
            self.assertIsInstance(why, str)
            self.assertIn("share", why)
        self.assertIsInstance(make_plan([frozenset({0, 1}), frozenset({2, 3}), frozenset({4, 5})]), str)       # six threads, three cores
        self.assertNotIsInstance(make_plan([frozenset({c, c + 4}) for c in range(4)]), str)                  # eight threads, four cores


@unittest.skipIf(len(CPUS) < 2, "needs two CPUs to move anything")
class Moves(unittest.TestCase):
    def setUp(self):
        self.rig, self.rest = frozenset(CPUS[:1]), frozenset(CPUS[1:])
        self.hub = Hub()
        self.kids: list[subprocess.Popen] = []

    def tearDown(self):
        for k in self.kids:
            k.kill()
            k.wait()

    def kid(self, **kw) -> subprocess.Popen:
        p = subprocess.Popen(["sleep", "60"], **kw)
        self.kids.append(p)
        return p

    def iso(self, pids, **kw) -> Isolation:
        return Isolation(self.hub, enabled=True, processes=lambda: [k.pid for k in self.kids] if pids is None else pids, sweep_s=0.05, plan=(self.rig, self.rest), **kw)

    def test_engaging_moves_a_program_off_the_rig_cpus_and_releasing_puts_it_back(self):
        k = self.kid()
        before = mask(k.pid)
        self.assertTrue(before & self.rig)
        i = self.iso(None)
        i.engage()
        try:
            self.assertEqual(mask(k.pid), before & self.rest)
            s = i.status()
            self.assertTrue(s["engaged"] and s["moved"] == 1 and s["threads"] >= 1)
            self.assertEqual((s["rig_cpus"], s["other_cpus"]), (cpu_list(self.rig), cpu_list(self.rest)))
        finally:
            i.release()
        self.assertEqual(mask(k.pid), before)
        self.assertFalse(i.status()["engaged"])
        self.assertEqual(i.status()["threads"], 0)

    def test_a_program_started_after_the_move_is_caught_and_gets_every_cpu_back_at_the_end(self):
        i = self.iso(None)
        i.engage()
        try:
            late = self.kid()
            deadline = time.monotonic() + 3
            while mask(late.pid) & self.rig and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertFalse(mask(late.pid) & self.rig)                    # the watcher found it
        finally:
            i.release()
        self.assertEqual(mask(late.pid), frozenset(CPUS))                   # it was not ours to remember, so it gets the whole machine

    def test_a_program_the_user_confined_to_the_rig_cpus_is_left_alone(self):
        k = self.kid()
        os.sched_setaffinity(k.pid, self.rig)
        i = self.iso(None)
        i.engage()
        try:
            self.assertEqual(mask(k.pid), self.rig)
            self.assertGreaterEqual(i.status()["left"], 1)
        finally:
            i.release()
        self.assertEqual(mask(k.pid), frozenset(CPUS))                      # confined to exactly the rig's CPUs, it is not distinguishable from one we pinned: it goes back to all

    def test_the_rigs_own_processes_are_pinned_not_moved(self):
        rigp = self.kid(start_new_session=True)
        other = self.kid()
        i = self.iso([rigp.pid, other.pid])
        i.engage(lambda: [rigp.pid])
        try:
            deadline = time.monotonic() + 3
            while mask(rigp.pid) != self.rig and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertEqual(mask(rigp.pid), self.rig)
            self.assertFalse(mask(other.pid) & self.rig)
        finally:
            i.release()
        self.assertEqual(mask(rigp.pid), frozenset(CPUS))

    def test_it_does_nothing_unless_wanted_and_possible(self):
        k = self.kid()
        before = mask(k.pid)
        off = Isolation(self.hub, enabled=False, processes=lambda: [k.pid])
        off.plan = (self.rig, self.rest)
        off.engage()
        self.assertFalse(off.status()["engaged"])
        self.assertEqual(mask(k.pid), before)
        small = Isolation(self.hub, enabled=True, processes=lambda: [k.pid])
        small.plan = "too small"
        small.engage()
        self.assertFalse(small.status()["engaged"])
        self.assertEqual((small.status()["available"], small.status()["reason"], small.status()["rig_cpus"]), (False, "too small", ""))
        self.assertEqual(mask(k.pid), before)
        small.release()                                                      # releasing what was never engaged is not an error

    def test_pin_self_confines_only_while_engaged(self):
        i = self.iso([])
        keep = mask(os.getpid())
        i.pin_self()
        self.assertEqual(mask(os.getpid()), keep)
        i.engaged = True
        try:
            i.pin_self()
            self.assertEqual(mask(os.getpid()), self.rig)
        finally:
            os.sched_setaffinity(0, keep)
            i.engaged = False

    def test_switching_it_off_on_a_running_rig_puts_everything_back(self):
        k = self.kid()
        before = mask(k.pid)
        i = self.iso(None)
        i.set_wanted(True, True, lambda: [])
        self.assertTrue(i.status()["engaged"])
        i.set_wanted(False, True, lambda: [])
        self.assertFalse(i.status()["engaged"] or i.status()["wanted"])
        self.assertEqual(mask(k.pid), before)
        i.set_wanted(True, False, lambda: [])                                 # wanted, but no rig is running: nothing moves yet
        self.assertFalse(i.status()["engaged"])
        self.assertTrue(i.status()["wanted"])

    def test_a_console_that_died_holding_it_is_cleaned_up_by_the_next(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "isolation.pid"
            k = self.kid()
            os.sched_setaffinity(k.pid, self.rest)                            # left on the other CPUs by the dead console
            dead = subprocess.Popen(["true"])
            dead.wait()
            f.write_text(str(dead.pid))
            i = self.iso(None, state_file=f)
            self.assertEqual(mask(k.pid), frozenset(CPUS))
            self.assertFalse(f.exists())
            self.assertTrue(any("ended without releasing" in e.text for e in self.hub.model.events))
            # a console that is alive keeps what it holds
            os.sched_setaffinity(k.pid, self.rest)
            f.write_text(str(os.getppid()))
            self.iso(None, state_file=f)
            self.assertEqual(mask(k.pid), self.rest)
            self.assertTrue(f.exists())

    def test_the_pid_file_exists_exactly_while_engaged(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "sub" / "isolation.pid"
            i = self.iso([], state_file=f)
            i.engage()
            self.assertEqual(f.read_text(), str(os.getpid()))
            i.release()
            self.assertFalse(f.exists())


if __name__ == "__main__":
    unittest.main()
