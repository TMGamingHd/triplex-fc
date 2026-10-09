# SPDX-License-Identifier: MIT
"""The operator's commands: the frames the console puts on the bus, the discipline around them, and what is made of the nodes' answers."""
import os
import tempfile
import unittest
from unittest import mock

from .console_support import act, hb  # first: puts console/ on the path
from tfc_peers import protocol as P
from tfc_console import commands as C
from tfc_console.hub import Hub
from tfc_console.server import ApiError


class FakeBus:
    sent = []

    def __init__(self, iface):
        self.iface = iface

    def send(self, t_us, frame):
        FakeBus.sent.append((self.iface, frame))

    def close(self):
        pass


def go_state(hub):
    """A bus on which the launch checklist is go."""
    for n in range(3):
        hub.ingest_frame(100.0, hb(n))
    hub.ingest_frame(100.0, act())
    hub.ingest_frame(100.0, P.pack_sync(100, 0, 0))


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = mock.patch.dict(os.environ, {"XDG_CACHE_HOME": self.tmp.name})
        self.env.start()
        self.bus = mock.patch.object(C, "SocketCanBus", FakeBus)
        self.bus.start()
        FakeBus.sent = []
        self.hub = Hub(clock=lambda: 100.0)
        self.svc = C.CommandService(self.hub, lambda: "vcan0")
        self.timer = mock.patch("threading.Timer")                  # the three-second settle is called by hand
        self.timer.start()

    def tearDown(self):
        self.timer.stop()
        self.bus.stop()
        self.env.stop()
        self.tmp.cleanup()

    def frames(self):
        return [P.unpack_ground(f) for _i, f in FakeBus.sent]


class TheFrames(Base):
    def test_a_plain_command_is_one_authenticated_frame(self):
        self.svc.send("reintegrate", "B")
        (g,) = self.frames()
        self.assertEqual((g.op, g.node, g.arm), (P.GROUND_OPS["reintegrate"], 1, False))
        self.assertEqual(g.tag, P.ground_mac(P.ground_key(), g.op, g.node, g.counter))
        self.assertEqual(FakeBus.sent[0][0], "vcan0")

    def test_a_two_step_command_is_an_arm_then_the_execute_with_the_next_counter(self):
        rec = self.svc.send("clear-disabled", "C", "armed", confirmed=True)
        a, e = self.frames()
        self.assertEqual((a.arm, e.arm, a.op, e.op, a.node, e.node), (True, False, 3, 3, 2, 2))
        self.assertEqual(e.counter, (a.counter + 1) & 0xFF)
        self.assertEqual(a.tag, P.ground_mac(P.ground_key(), 3 | P.ARM_FLAG, 2, a.counter))
        self.assertEqual([f["arm"] for f in rec["frames"]], [True, False])

    def test_counters_increase_and_are_the_command_lines_counters(self):
        self.svc.send("noop")
        self.svc.send("noop")
        c1, c2 = (g.counter for g in self.frames())
        self.assertEqual(c2, c1 + 1)
        from tfc_peers.cli import _counter_file
        self.assertEqual(int(_counter_file().read_text()), c2)       # `tfc_peers command` continues from here

    def test_an_imu_channel_is_node_4_to_6_and_a_phase_is_its_number(self):
        self.svc.send("reintegrate", "IMU-C")
        self.svc.send("phase", "coast")
        i, p = self.frames()
        self.assertEqual((i.node, p.node), (6, 4))
        self.assertEqual(C.target_text("reintegrate", 6), "IMU-C")
        self.assertEqual(C.target_text("phase", 4), "coast")

    def test_the_test_kinds(self):
        self.svc.send("reintegrate", "B", "forged", confirmed=True)
        self.svc.send("reintegrate", "B", "arm", confirmed=True)
        self.svc.send("reintegrate", "B", "execute", confirmed=True)
        f, a, e = self.frames()
        self.assertNotEqual(f.tag, P.ground_mac(P.ground_key(), f.op, f.node, f.counter))     # a tag that does not verify
        self.assertTrue(a.arm)
        self.assertFalse(e.arm)
        self.svc.send("noop", mode="replay")
        r = self.frames()[-1]
        self.assertEqual((r.counter, r.op), (e.counter, e.op))                                  # the last frame again, unchanged

    def test_the_key_is_the_environments(self):
        key = bytes(range(100, 116))
        with mock.patch.dict(os.environ, {"TFC_GROUND_KEY": key.hex()}):
            self.svc.send("noop")
        (g,) = self.frames()
        self.assertEqual(g.tag, P.ground_mac(key, g.op, g.node, g.counter))
        self.assertNotEqual(g.tag, P.ground_mac(P.BENCH_KEY, g.op, g.node, g.counter))


class TheDiscipline(Base):
    def refused(self, *a, **kw):
        with self.assertRaises(ApiError) as cm:
            self.svc.send(*a, **kw)
        self.assertEqual(FakeBus.sent, [], "a refused command must put nothing on the bus")
        return cm.exception

    def test_an_arm_command_needs_the_operators_confirmation(self):
        e = self.refused("launch", mode="armed")
        self.assertEqual(e.status, 409)
        self.refused("clear-safe", mode="auto")
        self.refused("disable", "B", "armed")

    def test_a_command_that_always_needs_an_arm_cannot_be_sent_plain(self):
        e = self.refused("clear-safe", mode="single", confirmed=True)
        self.assertIn("ARM", e.message)

    def test_a_launch_is_refused_while_the_checklist_is_no_go_and_names_why(self):
        e = self.refused("launch", mode="armed", confirmed=True)
        self.assertIn("NO-GO", e.message)
        self.assertIn("SYNC", e.message)

    def test_a_launch_with_a_go_is_sent_and_force_exists_to_test_the_refusal(self):
        go_state(self.hub)
        rec = self.svc.send("launch", mode="armed", confirmed=True)
        self.assertEqual(len(FakeBus.sent), 2)
        FakeBus.sent = []
        hub2 = Hub(clock=lambda: 100.0)
        svc2 = C.CommandService(hub2, lambda: "vcan0")
        svc2.send("launch", mode="armed", confirmed=True, force=True)
        self.assertEqual(len(FakeBus.sent), 2)
        self.assertTrue(svc2.log[-1]["forced"])

    def test_unknown_commands_targets_and_modes_are_refused(self):
        self.assertEqual(self.refused("fly").status, 400)
        self.assertEqual(self.refused("reintegrate").status, 400)                # needs a target
        self.assertEqual(self.refused("reintegrate", "D").status, 400)
        self.assertEqual(self.refused("warm", "IMU-A").status, 400)               # a computer, not an IMU
        self.assertEqual(self.refused("phase", "orbit").status, 400)
        self.assertEqual(self.refused("noop", mode="sideways").status, 400)
        self.assertEqual(self.refused("noop", mode="replay").status, 409)         # nothing to replay yet

    def test_a_replay_of_the_console_cannot_send(self):
        self.hub.replay = True
        e = self.refused("noop")
        self.assertEqual(e.status, 409)
        self.assertIn("read-only", e.message)

    def test_no_interface_no_command(self):
        svc = C.CommandService(self.hub, lambda: None)
        with self.assertRaises(ApiError):
            svc.send("noop")

    def test_a_bus_that_is_not_there_is_a_409_not_a_crash(self):
        class Gone(FakeBus):
            def __init__(self, iface):
                raise OSError("cannot open CAN interface 'vcan0': no such device. For a virtual bus run sim/scripts/setup_vcan.sh")
        with mock.patch.object(C, "SocketCanBus", Gone), self.assertRaises(ApiError) as cm:
            self.svc.send("noop")
        self.assertEqual(cm.exception.status, 409)
        self.assertIn("setup_vcan", cm.exception.message)

    def test_every_command_of_the_protocol_is_in_the_catalogue_with_the_right_arm_rule(self):
        names = {c["op"] for c in C.catalog()}
        self.assertEqual(names, set(P.GROUND_OPS))
        for c in C.catalog():
            self.assertEqual(c["id"], P.GROUND_OPS[c["op"]])
        arm = {c["op"]: c["arm"] for c in C.catalog()}
        for op in ("launch", "clear-safe", "clear-disabled"):
            self.assertEqual(arm[op], "always")                                  # docs/design/PROTOCOL.md, the table of ground commands
        for op in ("reintegrate", "scrub", "phase", "noop"):
            self.assertEqual(arm[op], "never")
        for op in ("disable", "warm"):
            self.assertEqual(arm[op], "tiered")


class TheAnswers(Base):
    def answer(self, src, text, t=100.5):
        self.hub.ingest_line(src, text, t)

    def test_each_nodes_answer_is_attached_to_the_command_and_the_status_follows(self):
        rec = self.svc.send("reintegrate", "B")
        for src in "ABC":
            self.answer(src, "[frame 500] GROUND COMMAND reintegrate B: accepted")
        self.answer("ACT", "[frame 500] GROUND reintegrate: ACT readmits the nodes it excluded (they must agree again)")
        self.assertEqual(len(rec["responses"]), 4)
        self.assertEqual(rec["status"], "answered")
        self.svc._settle(rec["id"])
        self.assertEqual(rec["status"], "accepted")

    def test_a_refusal_is_shown_with_its_reason(self):
        rec = self.svc.send("reintegrate", "B")
        self.answer("A", "[frame 500] GROUND COMMAND reintegrate B: refused: node is not latched")
        self.svc._settle(rec["id"])
        self.assertEqual(rec["status"], "refused")
        self.assertIn("not latched", rec["responses"][0]["result"])

    def test_with_no_console_attached_it_says_no_answer(self):
        rec = self.svc.send("noop")
        self.svc._settle(rec["id"])
        self.assertEqual(rec["status"], "no answer")

    def test_the_arm_frames_acceptance_is_not_the_commands(self):
        rec = self.svc.send("clear-disabled", "C", "armed", confirmed=True)
        self.answer("A", "[frame 500] GROUND COMMAND ARM clear-disabled C: accepted")
        self.answer("A", "[frame 502] GROUND COMMAND clear-disabled C: refused: node is not disabled")
        self.svc._settle(rec["id"])
        self.assertEqual(rec["status"], "refused")

    def test_an_answer_to_another_command_is_not_attached(self):
        rec = self.svc.send("noop")
        self.answer("A", "[frame 500] GROUND COMMAND reintegrate B: accepted")
        self.assertEqual(rec["responses"], [])

    def test_the_sending_is_an_event_with_what_was_sent(self):
        self.svc.send("clear-disabled", "C", "armed", confirmed=True)
        e = self.hub.model.events[-1]
        self.assertEqual((e.src, e.kind), ("OP", "command"))
        self.assertIn("ARM + clear-disabled C", e.text)


if __name__ == "__main__":
    unittest.main()
