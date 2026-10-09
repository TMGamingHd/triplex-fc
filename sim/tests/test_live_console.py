# SPDX-License-Identifier: MIT
"""Live: the console's own modules driving the real firmware on one vcan0 (no browser, no HTTP): the rig started from a profile, the hub reading the bus and the nodes' consoles, a launch sent by the command service,
a node killed, a fault injected and measured, a node readmitted. These are the flows the page performs, with the firmware that will be on the boards.
Skipped unless vcan0 exists and the launch images, ACT, tfc_simd and the fault-lab images are built (tools/bench/sil_triplex.sh --build --launch)."""
import time
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.commands import CommandService
from tfc_console.faultlab import FaultLab
from tfc_console.hub import Hub
from tfc_console.rig import Rig
from tfc_console.server import wait_for
from tfc_console.sources import BusSource, TruthSource

NEEDED = [ROOT / "build" / "launch_a" / "zephyr" / "zephyr.exe", ROOT / "build" / "act_native" / "zephyr" / "zephyr.exe", ROOT / "build" / "host" / "tfc_simd"]
LAB = [ROOT / "build" / "triplex_a" / "zephyr" / "zephyr.exe"]
VCAN = Path("/sys/class/net/vcan0").exists()


class Console:
    """The console's parts, wired as `tfc_console.cli` wires them, without the web server."""

    def __init__(self):
        self.hub = Hub()
        self.truth = TruthSource(self.hub, 0)
        self.truth.start()
        self.src = BusSource(self.hub, "vcan0")
        self.src.start()
        self.hub.start()
        self.rig = Rig(self.hub, ROOT, lambda: "vcan0", lambda: self.truth.port, lambda: [])
        self.commands = CommandService(self.hub, lambda: "vcan0")
        self.lab = FaultLab(self.hub, self.rig)

    def close(self):
        self.rig.shutdown()
        self.src.stop()
        self.truth.stop()
        self.hub.stop()

    def snap(self):
        return self.hub.snapshot()

    def health(self):
        return [n["health"] for n in self.snap()["nodes"]]

    def starved(self, allowed=()):
        for e in self.hub.model.events:
            if e.kind == "latched-out" and e.node not in allowed and e.src in "ABC":
                return f"the machine was too loaded: node {e.node} was latched out ({e.fields.get('reason')})"
        return None


@unittest.skipUnless(VCAN and all(p.exists() for p in NEEDED), "vcan0 or the launch images are not there (sim/scripts/setup_vcan.sh; tools/bench/sil_triplex.sh --build --launch)")
class ClosedLoop(unittest.TestCase):
    def setUp(self):
        self.c = Console()
        self.addCleanup(self.c.close)

    def test_launch_flight_and_the_loss_of_a_node_as_the_page_shows_them(self):
        c = self.c
        c.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: c.snap()["go_nogo"]["go"], 45.0), c.snap()["go_nogo"]["reasons"])
        if (why := c.starved()):
            self.skipTest(why)
        s = c.snap()
        self.assertEqual(s["phase"]["name"], "pad")
        self.assertTrue(wait_for(lambda: c.snap()["truth"] is not None, 5.0), "no telemetry from tfc_simd")
        self.assertEqual(c.snap()["truth"]["clamped"], 1)
        # every console is attached: the nodes' boot lines and status lines arrived
        self.assertTrue(all(c.hub.lines.get(n) for n in ("A", "B", "C", "ACT", "SIM")))
        self.assertTrue(wait_for(lambda: all(n["console"].get("wcet_frame") for n in c.snap()["nodes"]), 5.0))
        # the launch, as the page sends it
        rec = c.commands.send("launch", mode="armed", confirmed=True)
        self.assertTrue(wait_for(lambda: c.snap()["phase"]["name"] == "countdown", 4.0))
        self.assertTrue(wait_for(lambda: c.snap()["phase"]["name"] == "flight", 15.0), c.snap()["phase"])
        c.commands._settle(rec["id"])
        self.assertEqual(rec["status"], "accepted", rec["responses"])
        self.assertGreaterEqual(len({r["src"] for r in rec["responses"]}), 3)
        ms = c.snap()["milestones"]
        self.assertAlmostEqual(ms["t0"] - ms["countdown"], 10.0, delta=0.4)
        # T-zero as the three computers print it
        self.assertTrue(wait_for(lambda: sum(1 for e in c.hub.model.events if e.kind == "t-zero" and e.src in ("A", "B", "C")) == 3, 3.0))
        # the vehicle leaves the pad and the simulator's truth follows the nominal flight
        self.assertTrue(wait_for(lambda: c.snap()["truth"]["alt"] > 50.0, 10.0))
        self.assertEqual(c.snap()["truth"]["clamped"], 0)
        self.assertGreater(c.snap()["sim"]["alt"], 0)                           # the bus's own copy agrees
        # a computer is lost in flight (the live tests' "kill B at T+30 s")
        c.rig.kill("B")
        self.assertTrue(wait_for(lambda: c.snap()["nodes"][1]["health"] == "latched", 3.0))
        self.assertTrue(wait_for(lambda: c.snap()["act"]["excluded"][1], 3.0))
        self.assertEqual(c.snap()["act"]["vote_status"], "Duplex")
        self.assertTrue(wait_for(lambda: not c.snap()["nodes"][1]["alive"], 3.0))
        self.assertTrue(any(a["key"] == "silent-B" for a in c.snap()["alerts"]))
        killed = next(e for e in c.hub.model.events if e.kind == "fault" and "killed" in e.text)
        latched = next(e for e in c.hub.model.events if e.kind == "view" and e.fields.get("to") == "latched")
        self.assertLessEqual(latched.frame - killed.frame, 6)                    # detection: a few frames (the matrix says 2, plus the heartbeat's one)
        # the flight goes on in Duplex
        self.assertEqual(c.snap()["phase"]["name"], "flight")
        self.assertFalse(c.snap()["truth"]["crashed"])

    def test_a_scrub_in_the_countdown_returns_to_the_pad(self):
        c = self.c
        c.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: c.snap()["go_nogo"]["go"], 45.0))
        if (why := c.starved()):
            self.skipTest(why)
        c.commands.send("launch", mode="armed", confirmed=True)
        self.assertTrue(wait_for(lambda: c.snap()["phase"]["name"] == "countdown", 4.0))
        time.sleep(2.0)
        rec = c.commands.send("scrub")
        self.assertTrue(wait_for(lambda: c.snap()["phase"]["name"] == "pad", 4.0))
        self.assertTrue(any(e.kind == "scrub" for e in c.hub.model.events))
        c.commands._settle(rec["id"])
        self.assertEqual(rec["status"], "accepted")
        self.assertEqual(c.snap()["truth"]["clamped"], 1)

    def test_a_launch_is_refused_by_the_console_before_the_calibration_and_by_the_firmware_when_forced(self):
        c = self.c
        c.rig.start_profile("closed-loop")
        self.assertTrue(wait_for(lambda: c.snap()["sync"]["alive"], 15.0))
        time.sleep(3.0)                                                          # the calibration needs 10 s
        from tfc_console.server import ApiError
        with self.assertRaises(ApiError) as cm:
            c.commands.send("launch", mode="armed", confirmed=True)
        self.assertIn("NO-GO", cm.exception.message)
        rec = c.commands.send("launch", mode="armed", confirmed=True, force=True)
        self.assertTrue(wait_for(lambda: any(e.kind == "launch-refused" for e in c.hub.model.events), 5.0))
        self.assertEqual(c.snap()["phase"]["name"], "pad")
        c.commands._settle(rec["id"])
        self.assertEqual(c.snap()["truth"]["clamped"], 1)


@unittest.skipUnless(VCAN and all(p.exists() for p in NEEDED + LAB), "vcan0 or the fault-lab images are not there")
class FaultLabLive(unittest.TestCase):
    def setUp(self):
        self.c = Console()
        self.addCleanup(self.c.close)

    def test_a_fault_is_injected_detected_measured_cleared_and_the_node_readmitted(self):
        c = self.c
        c.rig.start_profile("fault-lab")
        self.assertTrue(wait_for(lambda: c.snap()["nodes"][1]["alive"] and c.snap()["nodes"][2]["alive"] and c.snap()["nodes"][0]["heartbeat"], 20.0), "FC-A and the peers did not come up")
        self.assertTrue(wait_for(lambda: c.lab.available(), 5.0))
        time.sleep(2.0)
        if (why := c.starved()):
            self.skipTest(why)
        self.assertEqual(c.health(), ["healthy"] * 3)
        rec = c.lab.add("B:bias:mag=3")
        self.assertTrue(wait_for(lambda: c.lab.table[0]["detected"] is not None, 5.0), "the fault was never detected")
        d = c.lab.table[0]["detected"]
        self.assertEqual(d["frames_after"], 2, d)                                # sim/README.md: a 3 dps bias on one node latches it 2 frames after it starts
        self.assertTrue(wait_for(lambda: c.lab.table[0]["detected"].get("reason"), 3.0), "the node's console never gave the reason")
        self.assertEqual(c.lab.table[0]["detected"]["reason"], "vote disagreement")
        self.assertTrue(wait_for(lambda: c.snap()["nodes"][1]["health"] == "latched", 2.0))            # the page's state follows its events by up to a tick
        self.assertTrue(wait_for(lambda: min(n["mode"] for n in c.snap()["nodes"] if n["heartbeat"]) == 2, 2.0))
        c.lab.clear("all")
        time.sleep(2.5)                                                          # the dwell
        r = c.commands.send("reintegrate", "B")
        self.assertTrue(wait_for(lambda: any(x["result"].startswith("accepted") for x in r["responses"]), 5.0), r["responses"])
        self.assertTrue(wait_for(lambda: c.health() == ["healthy"] * 3, 10.0), c.health())
        self.assertEqual(c.snap()["act"]["vote_status"], "Triplex")

    def test_every_one_of_a_sample_of_fault_kinds_is_detected_as_the_readme_says(self):
        c = self.c
        c.rig.start_profile("fault-lab")
        self.assertTrue(wait_for(lambda: c.lab.available() and c.snap()["nodes"][1]["alive"], 25.0))
        time.sleep(2.0)
        if (why := c.starved()):
            self.skipTest(why)
        # (spec, the most frames after its start that the matrix allows plus the heartbeat's one). The strikes are spread over the nodes: a node latched twice for a physical cause is disabled for the run.
        for spec, latest in (("C:dropout", 6), ("B:stuck", 30), ("B:cmd_offset:mag=1", 6), ("C:digest:xor=1", 8)):
            node = spec[0]
            i = "ABC".index(node)
            c.lab.add(spec)
            tail = "\n".join(f"{e.frame} {e.src} {e.text[:100]}" for e in list(c.hub.model.events)[-12:])
            self.assertTrue(wait_for(lambda: c.lab.table[-1]["detected"] is not None, 6.0), f"{spec} was not detected; health {c.health()}; table {c.lab.table}; last events:\n{tail}")
            self.assertLessEqual(c.lab.table[-1]["detected"]["frames_after"], latest, (spec, c.lab.table[-1]["detected"]))
            self.assertTrue(wait_for(lambda: c.snap()["nodes"][i]["health"] in ("latched", "disabled"), 2.0))
            c.lab.clear("all")                                                     # recover: clear the fault, wait out the dwell, readmit, wait for the probation
            time.sleep(2.5)
            c.commands.send("reintegrate", node)
            self.assertTrue(wait_for(lambda: c.snap()["nodes"][i]["health"] == "healthy", 12.0), (spec, c.health()))


if __name__ == "__main__":
    unittest.main()
