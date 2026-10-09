# SPDX-License-Identifier: MIT
"""Injecting faults through the peers' control channel, and measuring what the computers do about them."""
import unittest
from types import SimpleNamespace

from . import console_support  # noqa: F401  (first: puts console/ on the path)
from tfc_peers import control as CTL
from tfc_peers import peers as PE
from tfc_peers import protocol as P
from tfc_peers.faults import KINDS, parse_fault
from tfc_console.faultlab import EXPECT, FaultLab
from tfc_console.hub import Hub


class FakeRig:
    """The rig as the fault lab sees it: a process called PEERS that answers control lines. The answers come from the real control module and a real scenario."""

    def __init__(self, scenario, frame):
        self.scenario, self.frame = scenario, frame
        self.procs = {"PEERS": SimpleNamespace(state="running", spec=SimpleNamespace(control=True))}
        self.line_hooks = []
        self.sent = []

    def write_line(self, name, text):
        self.sent.append(text)
        for reply in CTL.handle(self.scenario, text, self.frame[0]):
            for fn in self.line_hooks:
                fn("PEERS", reply)


class Base(unittest.TestCase):
    def setUp(self):
        self.hub = Hub(clock=lambda: 1.0)
        self.scenario = PE.Scenario([1, 2], [], 1)
        self.frame = [400]
        self.rig = FakeRig(self.scenario, self.frame)
        self.lab = FaultLab(self.hub, self.rig)


class Injection(Base):
    def test_a_fault_added_through_the_lab_changes_the_traffic_of_that_node_from_that_frame_on(self):
        rec = self.lab.add("B:bias:mag=3")
        self.assertEqual((rec["node"], rec["kind"], rec["start"], rec["end"]), ("B", "bias", 400, None))
        before = {f.frame.id: P.unpack_vec3(f.frame, P.GYRO_LSB_DPS).values for f in self.scenario.frames(400) if f.frame.id in (0x101, 0x102)}
        self.assertGreater(before[0x101][0] - before[0x102][0], 2.0)             # B reads about 3 dps above C

    def test_a_fault_before_its_start_does_nothing_and_a_cleared_one_stops(self):
        rec = self.lab.add("B:bias:start=450,mag=3")
        self.assertEqual(rec["start"], 450)
        self.lab.clear(str(rec["id"]))
        self.assertEqual(self.lab.table, [])
        self.assertEqual(self.scenario.fault_table(), [])

    def test_a_duration_ends_the_fault_and_the_table_says_so(self):
        rec = self.lab.add("C:dropout", frames=30)
        self.assertEqual((rec["start"], rec["end"]), (400, 430))
        self.hub.model.sync_no = 420
        self.assertTrue(self.lab.snapshot()["table"][0]["active"])
        self.hub.model.sync_no = 431
        self.assertFalse(self.lab.snapshot()["table"][0]["active"])

    def test_a_bad_spec_is_refused_with_the_parsers_reason_and_nothing_is_sent(self):
        for spec, why in (("B:nope", "unknown fault kind"), ("B:bias:mag=abc", "finite number"), ("B:late:us=99999", "late needs"), ("A:bias", "real flight computer"), ("B", "expected NODE:KIND")):
            with self.assertRaises(ValueError) as cm:
                self.lab.add(spec)
            self.assertIn(why, str(cm.exception), spec)
        self.assertEqual(self.rig.sent, [])
        self.assertEqual(self.lab.table, [])

    def test_a_peer_that_is_not_there_is_a_clear_error(self):
        self.rig.procs["PEERS"].state = "exited"
        with self.assertRaisesRegex(ValueError, "not running"):
            self.lab.add("B:bias")
        self.assertFalse(self.lab.snapshot()["available"])

    def test_faults_are_numbered_and_clear_all_empties_the_scenario(self):
        a = self.lab.add("B:bias")
        b = self.lab.add("C:stuck")
        self.assertEqual((a["id"], b["id"]), (1, 2))
        self.lab.clear("all")
        self.assertEqual((self.lab.table, self.scenario.fault_table()), ([], []))
        with self.assertRaises(ValueError):
            self.lab.clear("7")

    def test_injection_is_an_event_with_what_the_matrix_expects(self):
        self.lab.add("B:bias")
        e = self.hub.model.events[-1]
        self.assertEqual((e.src, e.kind, e.level), ("OP", "fault", "warn"))
        self.assertIn("FAULT INJECTED on node B", e.text)
        self.assertIn(EXPECT["bias"][0], e.text)


class Detection(Base):
    def view_event(self, frame, observer, subject, to="latched"):
        self.hub.model.emit(1.0, "warn", observer, "view", "x", subject, {"observer": observer, "subject": subject, "to": to, "from": "healthy"}, frame=frame)

    def test_the_first_latch_after_the_start_is_the_detection_and_the_console_adds_the_reason(self):
        rec = self.lab.add("B:bias:mag=3")
        self.view_event(402, "A", "B")
        self.view_event(402, "C", "B")
        d = self.lab.table[0]["detected"]
        self.assertEqual((d["frame"], d["frames_after"], d["by"]), (402, 2, "heartbeats"))
        self.hub.model.emit(1.0, "warn", "A", "latched-out", "[A] node B LATCHED OUT: vote disagreement", "B", {"reason": "vote disagreement"}, frame=402)
        self.assertEqual(self.lab.table[0]["detected"]["reason"], "vote disagreement")

    def test_a_latch_of_another_node_or_before_the_fault_is_not_its_detection(self):
        self.lab.add("B:bias")
        self.view_event(402, "A", "C")                                   # another node
        self.view_event(390, "A", "B")                                   # before the fault began
        self.view_event(405, "A", "B", to="probation")                   # not a latch
        self.assertIsNone(self.lab.table[0]["detected"])

    def test_two_faults_each_have_their_own_measurement(self):
        self.lab.add("B:bias")
        self.frame[0] = 500
        self.lab.add("C:dropout")
        self.view_event(503, "A", "C")
        self.assertIsNone(self.lab.table[0]["detected"])
        self.assertEqual(self.lab.table[1]["detected"]["frames_after"], 3)


class TheCatalogue(unittest.TestCase):
    def test_every_fault_kind_has_an_expectation_from_the_documents_table(self):
        self.assertEqual(set(EXPECT) - set(KINDS), set(), "an expectation for a kind that does not exist")
        missing = set(KINDS) - set(EXPECT)
        self.assertEqual(missing, set(), f"no expectation for {sorted(missing)}: add it from sim/README.md, 'What to expect'")

    def test_the_expectations_are_the_readmes(self):
        from .console_support import ROOT
        text = (ROOT / "sim" / "README.md").read_text()
        for kind in EXPECT:
            self.assertIn(f"| `{kind}`", text)
        self.assertEqual(EXPECT["spike"][1], "Triplex")                  # the README: a spike is flagged and filtered, the node is not latched
        self.assertEqual(EXPECT["babble"][1], "Triplex")

    def test_the_presets_of_the_page_are_valid_specs(self):
        import re
        from .console_support import ROOT
        js = (ROOT / "console" / "web" / "js" / "tabs" / "faults.js").read_text()
        specs = re.findall(r"\['[^']+', '([A-C]:[a-z_]+[^']*)'\]", js)
        self.assertGreaterEqual(len(specs), 8)
        for spec in specs:
            parse_fault(spec)


if __name__ == "__main__":
    unittest.main()
