# SPDX-License-Identifier: MIT
"""The rig's processes, with stand-ins for the firmware (shell scripts that print what a node prints): start, watch, break, stop, and never leave one behind."""
import os
import signal
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.hub import Hub
from tfc_console.isolate import Isolation
from tfc_console.rig import Rig
from tfc_console.server import wait_for

SCRIPT = """#!/bin/sh
printf '\\033[0;39m[frame 5] node B joined the bus\\033[0m\\n'
printf '[frame 100] TRIPLEX  A+ B+ C+  | crc=0 seq=0 wcet_frame=9100\\n'
exec sleep 60
"""


def fake_repo(root: Path, missing=()):
    for rel in ("build/host/tfc_simd", "build/launch_a/zephyr/zephyr.exe", "build/launch_b/zephyr/zephyr.exe", "build/launch_c/zephyr/zephyr.exe", "build/act_native/zephyr/zephyr.exe"):
        if rel in missing:
            continue
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(SCRIPT)
        p.chmod(p.stat().st_mode | stat.S_IXUSR)


def state_of(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0]
    except OSError:
        return None


class RigTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name)
        fake_repo(self.repo)
        self.hub = Hub()
        self.iface = "vcan0"
        self.rig = Rig(self.hub, self.repo, lambda: self.iface, lambda: 45679, lambda: [])

    def tearDown(self):
        self.rig.shutdown()
        self.tmp.cleanup()

    def up(self):
        self.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: len(self.rig.procs) == 5 and all(p.state == "running" for p in self.rig.procs.values()), 6.0), self.rig.snapshot())


class Profiles(RigTest):
    def test_a_profile_is_ready_when_its_binaries_are_there_and_says_what_is_missing(self):
        profs = self.rig.profiles()
        self.assertTrue(profs["closed-loop"]["ready"])
        self.assertFalse(profs["fault-lab"]["ready"])                      # no build/triplex_a
        self.assertEqual(profs["fault-lab"]["missing"], ["A"])                  # ACT is there; the peers are Python, which is always there
        fake_repo(self.repo / "x", missing=("build/launch_b/zephyr/zephyr.exe",))
        r2 = Rig(self.hub, self.repo / "x", lambda: "vcan0", lambda: 1, lambda: [])
        self.assertEqual(r2.profiles()["closed-loop"]["missing"], ["B"])

    def test_the_simulator_is_given_the_pad_hold_the_interface_and_the_telemetry_port_and_starts_first(self):
        spec = {s.name: s for s in self.rig.profiles()["closed-loop"]["procs"]}
        argv = spec["SIM"].argv
        self.assertIn("--hold", argv)
        self.assertEqual(argv[argv.index("--iface") + 1], "vcan0")
        self.assertEqual(argv[argv.index("--telemetry") + 1], "45679")
        order = [s.name for s in self.rig.profiles()["closed-loop"]["procs"]]
        self.assertEqual(order[0], "SIM")
        self.assertEqual(sorted(order[1:]), ["A", "ACT", "B", "C"])

    def test_the_vehicle_arguments_reach_the_simulator(self):
        rig = Rig(self.hub, self.repo, lambda: "vcan0", lambda: 1, lambda: ["--vehicle", "/x/v.json"])
        argv = rig.profiles()["closed-loop"]["procs"][0].argv
        self.assertEqual(argv[-2:], ["--vehicle", "/x/v.json"])

    def test_the_fault_labs_peers_follow_sync_and_take_commands(self):
        fake_repo(self.repo)
        (self.repo / "build/triplex_a/zephyr").mkdir(parents=True)
        exe = self.repo / "build/triplex_a/zephyr/zephyr.exe"
        exe.write_text(SCRIPT)
        exe.chmod(0o755)
        peers = {s.name: s for s in self.rig.profiles()["fault-lab"]["procs"]}["PEERS"]
        for flag in ("--follow-sync", "--control", "--nodes", "B,C", "--frames", "0"):
            self.assertIn(flag, peers.argv)
        self.assertTrue(peers.control)
        self.assertTrue(self.rig.profiles()["fault-lab"]["ready"])


class Lifecycle(RigTest):
    def test_start_watch_and_stop(self):
        self.up()
        self.assertTrue(wait_for(lambda: all(self.hub.lines.get(n) for n in ("A", "B", "C", "ACT", "SIM")), 4.0))
        texts = [r["text"] for r in self.hub.recent_lines()["A"]]
        self.assertIn("[frame 5] node B joined the bus", texts)                  # the colour codes are gone
        self.assertTrue(wait_for(lambda: self.rig.procs["B"].lines >= 2, 2.0))
        self.assertEqual(self.hub.model.nodes[1].console["wcet_frame"], 9100)    # the status line reached the node's state
        self.assertTrue(any(e.kind == "joined" and e.src == "A" for e in self.hub.model.events))
        pids = [p.popen.pid for p in self.rig.procs.values()]
        self.rig.stop()
        self.assertTrue(all(wait_for(lambda q=q: state_of(q) in (None, "Z"), 3.0) for q in pids))
        self.assertTrue(all(p.state == "stopped" for p in self.rig.procs.values()))

    def test_a_second_start_while_running_is_refused(self):
        self.up()
        with self.assertRaisesRegex(ValueError, "already running"):
            self.rig.start_profile("closed-loop")

    def test_a_rig_needs_a_live_bus_and_a_built_profile(self):
        self.iface = None
        with self.assertRaisesRegex(ValueError, "live interface"):
            self.rig.start_profile("closed-loop")
        self.iface = "vcan0"
        with self.assertRaisesRegex(ValueError, "not built"):
            self.rig.start_profile("fault-lab")
        with self.assertRaisesRegex(ValueError, "unknown profile"):
            self.rig.start_profile("nope")
        self.hub.replay = True
        with self.assertRaisesRegex(ValueError, "live interface"):
            self.rig.start_profile("closed-loop")

    def test_kill_freeze_resume_restart(self):
        self.up()
        b = self.rig.procs["B"]
        pid = b.popen.pid
        self.rig.freeze("B")
        self.assertTrue(wait_for(lambda: state_of(pid) == "T", 2.0), state_of(pid))
        self.assertEqual(self.rig.procs["B"].state, "frozen")
        with self.assertRaises(ValueError):
            self.rig.freeze("B")
        self.rig.resume("B")
        self.assertTrue(wait_for(lambda: state_of(pid) in ("S", "R"), 2.0))
        self.rig.kill("B")
        self.assertTrue(wait_for(lambda: state_of(pid) in (None, "Z"), 3.0))
        self.assertTrue(wait_for(lambda: self.rig.procs["B"].state == "exited", 3.0))
        self.rig.restart("B")
        self.assertTrue(wait_for(lambda: self.rig.procs["B"].state == "running" and self.rig.procs["B"].popen.pid != pid, 3.0))
        self.assertEqual(self.rig.procs["B"].restarts, 1)
        self.assertEqual(self.rig.procs["A"].state, "running")                  # the others were not touched
        msgs = [e.text for e in self.hub.model.events if e.src == "CON"]
        self.assertTrue(any("TEST ACTION: node B killed" in m for m in msgs))
        self.assertTrue(any("frozen" in m for m in msgs))

    def test_stopping_a_frozen_process_still_stops_it(self):
        self.up()
        pid = self.rig.procs["C"].popen.pid
        self.rig.freeze("C")
        self.rig.stop("C")
        self.assertTrue(wait_for(lambda: state_of(pid) in (None, "Z"), 3.0))

    def test_a_missing_process_name_is_an_error_not_a_crash(self):
        self.up()
        for fn in (self.rig.kill, self.rig.freeze, self.rig.resume, self.rig.restart):
            with self.assertRaises(ValueError):
                fn("Z")

    def test_a_process_that_ends_by_itself_is_shown_as_exited_with_its_code(self):
        (self.repo / "build/launch_c/zephyr/zephyr.exe").write_text("#!/bin/sh\necho bye\nexit 3\n")
        self.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: self.rig.procs.get("C") is not None and self.rig.procs["C"].state == "exited", 5.0))
        self.assertEqual(self.rig.procs["C"].exit_code, 3)
        snap = self.rig.snapshot()
        self.assertEqual([p["exit_code"] for p in snap["procs"] if p["name"] == "C"], [3])

    def test_the_snapshot_describes_the_profiles_for_the_page(self):
        snap = self.rig.snapshot()
        self.assertEqual(set(snap["profiles"]), {"closed-loop", "fault-lab"})
        self.assertTrue(snap["profiles"]["closed-loop"]["ready"])

    def test_write_line_needs_a_process_with_a_control_channel(self):
        self.up()
        with self.assertRaises(ValueError):
            self.rig.write_line("A", "list")


@unittest.skipIf(len(os.sched_getaffinity(0)) < 2, "needs two CPUs")
class Isolated(unittest.TestCase):
    """The rig's processes run on the CPUs the isolation reserves, and it lets go when the rig stops (ADR-040). `processes` is empty so that nothing of the host's is moved by a test."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name)
        fake_repo(self.repo)
        self.hub = Hub()
        cpus = sorted(os.sched_getaffinity(0))
        self.rigcpus = frozenset(cpus[:1])
        self.iso = Isolation(self.hub, enabled=True, processes=lambda: [], sweep_s=0.05, plan=(self.rigcpus, frozenset(cpus[1:])))
        self.rig = Rig(self.hub, self.repo, lambda: "vcan0", lambda: 45679, lambda: [], isolation=self.iso)

    def tearDown(self):
        self.rig.shutdown()
        self.tmp.cleanup()

    def test_every_process_starts_on_the_rig_cpus_and_the_snapshot_says_so(self):
        self.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: len(self.rig.pids()) == 5, 6.0), self.rig.snapshot())
        for pid in self.rig.pids():
            self.assertTrue(wait_for(lambda pid=pid: frozenset(os.sched_getaffinity(pid)) == self.rigcpus, 3.0), pid)
        iso = self.rig.snapshot()["isolation"]
        self.assertTrue(iso["engaged"] and iso["available"])
        self.assertEqual(iso["rig_cpus"], str(sorted(self.rigcpus)[0]))
        self.rig.stop()
        self.assertFalse(self.rig.snapshot()["isolation"]["engaged"])
        self.assertTrue(any("isolation released" in e.text for e in self.hub.model.events))

    def test_it_can_be_switched_off_and_on_while_the_rig_runs(self):
        self.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: len(self.rig.pids()) == 5, 6.0))
        self.rig.isolate(False)
        self.assertFalse(self.rig.snapshot()["isolation"]["engaged"])
        self.rig.isolate(True)
        self.assertTrue(self.rig.snapshot()["isolation"]["engaged"])
        for pid in self.rig.pids():
            self.assertTrue(wait_for(lambda pid=pid: frozenset(os.sched_getaffinity(pid)) == self.rigcpus, 3.0), pid)

    def test_a_rig_without_it_or_a_host_that_cannot_refuses_to_switch_it_on(self):
        bare = Rig(self.hub, self.repo, lambda: "vcan0", lambda: 45679, lambda: [])
        self.assertIsNone(bare.snapshot()["isolation"])
        with self.assertRaises(ValueError):
            bare.isolate(True)
        self.iso.plan = "only 4 logical CPUs"
        with self.assertRaises(ValueError) as cm:
            self.rig.isolate(True)
        self.assertIn("only 4 logical CPUs", str(cm.exception))


class NothingIsLeftBehind(unittest.TestCase):
    def test_children_die_with_a_console_that_is_killed(self):
        """PR_SET_PDEATHSIG: a console that is killed outright must not leave a rig running that nobody can see."""
        with tempfile.TemporaryDirectory() as d:
            repo = Path(d)
            fake_repo(repo)
            code = textwrap.dedent(f"""
                import sys, time
                sys.path.insert(0, {str(ROOT / 'console')!r}); sys.path.insert(0, {str(ROOT / 'sim')!r})
                from pathlib import Path
                from tfc_console.hub import Hub
                from tfc_console.rig import Rig
                rig = Rig(Hub(), Path({d!r}), lambda: "vcan0", lambda: 1, lambda: [])
                rig.start_profile("closed-loop")
                time.sleep(1.5)
                print(" ".join(str(p.popen.pid) for p in rig.procs.values()), flush=True)
                time.sleep(60)
            """)
            parent = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
            try:
                pids = [int(x) for x in parent.stdout.readline().split()]
                self.assertEqual(len(pids), 5)
                self.assertTrue(all(state_of(p) not in (None, "Z") for p in pids))
                parent.send_signal(signal.SIGKILL)
                parent.wait()
                self.assertTrue(all(wait_for(lambda p=p: state_of(p) in (None, "Z"), 5.0) for p in pids), [state_of(p) for p in pids])
            finally:
                if parent.poll() is None:
                    parent.kill()
                parent.stdout.close()


if __name__ == "__main__":
    unittest.main()
