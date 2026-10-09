# SPDX-License-Identifier: MIT
"""Changing a running scenario's faults (`tfc_peers run --control`, tfc_peers/control.py): the commands, the effect on the traffic, and the rule that a command is applied between frames."""
import io
import unittest
from contextlib import redirect_stderr
from unittest import mock

from tfc_peers import cli
from tfc_peers import control as CTL
from tfc_peers import peers as PE
from tfc_peers import protocol as P
from tfc_peers.bus import ListBus, run_synced
from tfc_peers.faults import parse_fault


def gyro_of(scenario, k, node):
    return next(P.unpack_vec3(tf.frame, P.GYRO_LSB_DPS).values for tf in scenario.frames(k) if tf.frame.id == P.ID_GYRO_BASE + node)


class Commands(unittest.TestCase):
    def setUp(self):
        self.sc = PE.Scenario([1, 2], [], 1)

    def h(self, line, k=10):
        return CTL.handle(self.sc, line, k)

    def test_add_without_a_start_begins_in_the_frame_it_is_applied_in(self):
        (r,) = self.h("add B:bias:mag=3", k=250)
        self.assertEqual(r, "ok add 1 B:bias:start=250,sensor=gyro,axis=0,mag=3.0")

    def test_add_with_a_start_keeps_it_and_for_gives_the_end(self):
        self.assertEqual(self.h("add C:dropout:start=300 for 50", k=250), ["ok add 1 C:dropout:start=300,end=350"])
        self.assertEqual(self.h("add C:stuck for 20", k=250), ["ok add 2 C:stuck:start=250,end=270"])

    def test_clear_one_and_all(self):
        self.h("add B:bias")
        self.h("add C:bias")
        self.assertEqual(self.h("clear 1"), ["ok clear 1"])
        self.assertEqual([i for i, _f in self.sc.fault_table()], [2])
        self.assertEqual(self.h("clear all"), ["ok clear all 1"])
        self.assertEqual(self.sc.fault_table(), [])
        self.assertEqual(self.h("clear all"), ["ok clear all 0"])

    def test_list_and_frame(self):
        self.h("add B:bias")
        self.assertEqual(self.h("list", k=33)[0], "ok list")
        self.assertRegex(self.h("list")[1], r"^fault 1 B:bias:start=10,")
        self.assertEqual(self.h("frame", k=77), ["ok frame 77"])

    def test_every_error_is_one_line_that_says_why_and_changes_nothing(self):
        for line, why in (("add", "usage"), ("add B:bias for", "usage"), ("add B:bias until 5", "usage"), ("add B:nope", "unknown fault kind"), ("add A:bias", "not simulated"), ("add B:bias for 0", "at least 1"),
                          ("add B:bias for x", "invalid literal"), ("clear", "usage"), ("clear 99", "no fault 99"), ("clear x", "invalid literal"), ("jump", "unknown command"), ("add B:bias:start=5,end=3", "bad window")):
            (r,) = self.h(line)
            self.assertTrue(r.startswith("error: "), (line, r))
            self.assertIn(why, r, line)
        self.assertEqual(self.sc.fault_table(), [])
        self.assertEqual(self.h(""), [])
        self.assertEqual(self.h("   "), [])

    def test_a_fault_added_with_a_duration_that_would_end_before_it_starts_is_refused(self):
        (r,) = self.h("add B:bias:start=200,end=210", k=300)
        self.assertTrue(r.startswith("ok"))                            # an explicit window is the operator's own business, even in the past
        (r,) = self.h("add B:bias:start=200 for 0", k=300)
        self.assertTrue(r.startswith("error"))


class TheEffectOnTheTraffic(unittest.TestCase):
    def test_a_fault_added_in_frame_k_changes_frame_k_and_not_the_ones_before(self):
        sc = PE.Scenario([1, 2], [], 1)
        base = [gyro_of(sc, k, 1) for k in range(3)]
        CTL.handle(sc, "add B:bias:mag=5", 3)
        after = gyro_of(sc, 3, 1)
        ref = PE.Scenario([1, 2], [], 1)
        clean = [gyro_of(ref, k, 1) for k in range(4)]
        self.assertEqual(base, clean[:3])
        self.assertAlmostEqual(after[0] - clean[3][0], 5.0, delta=0.05)
        self.assertEqual(after[1:], clean[3][1:])

    def test_clearing_restores_the_nodes_traffic_exactly(self):
        sc, ref = PE.Scenario([1, 2], [], 1), PE.Scenario([1, 2], [], 1)
        CTL.handle(sc, "add C:bias:mag=5", 0)
        got = [gyro_of(sc, 0, 2), gyro_of(sc, 1, 2)]
        CTL.handle(sc, "clear 1", 2)
        got += [gyro_of(sc, k, 2) for k in range(2, 5)]
        clean = [gyro_of(ref, k, 2) for k in range(5)]
        self.assertGreater(got[0][0] - clean[0][0], 4.9)
        self.assertGreater(got[1][0] - clean[1][0], 4.9)
        self.assertEqual(got[2:], clean[2:])

    def test_the_faults_survive_a_restart_of_the_sync_master(self):
        sc = PE.Scenario([1, 2], [], 1)
        CTL.handle(sc, "add B:bias:mag=5", 0)
        sc.frames(5)
        sc.reset()                                                     # SYNC went back: the scenario starts over, with the faults it has now
        self.assertEqual(len(sc.fault_table()), 1)
        self.assertGreater(gyro_of(sc, 0, 1)[0] - gyro_of(PE.Scenario([1, 2], [], 1), 0, 1)[0], 4.9)

    def test_a_node_the_scenario_does_not_simulate_cannot_be_given_a_fault(self):
        sc = PE.Scenario([1], [], 1)
        with self.assertRaises(ValueError):
            sc.add_fault(parse_fault("C:bias"))

    def test_a_scenario_that_started_with_faults_numbers_them_from_one(self):
        sc = PE.Scenario([1, 2], [parse_fault("B:bias"), parse_fault("C:stuck")], 1)
        self.assertEqual([i for i, _f in sc.fault_table()], [1, 2])
        self.assertEqual(sc.add_fault(parse_fault("B:drift")), 3)
        sc.clear_fault(1)
        self.assertEqual(sc.add_fault(parse_fault("B:noise")), 4)          # numbers are never reused


class AppliedBetweenFrames(unittest.TestCase):
    def test_run_synced_calls_the_poll_once_per_sync_before_the_frames_of_that_frame(self):
        sc = PE.Scenario([1], [], 1)
        seen = []

        class Bus(ListBus):
            def __init__(self):
                super().__init__()
                self.n = 0

            def recv(self, timeout=None):
                self.n += 1
                return P.pack_sync(self.n, self.n & 255, 0) if self.n <= 3 else None

            def set_filter(self, filters):
                pass

        bus = Bus()

        def poll(k):
            seen.append((k, len(bus.sent)))

        with mock.patch("tfc_peers.bus._wait_until"):
            try:
                run_synced(sc, bus, 3, sync_timeout_s=0.01, poll=poll)
            except TimeoutError:
                pass
        self.assertEqual([k for k, _n in seen], [1, 2, 3])
        self.assertEqual([n for _k, n in seen], [0, 3, 6])                 # each poll came before its frame's three frames were sent


class TheCommandLine(unittest.TestCase):
    def test_control_needs_follow_sync(self):
        args = cli.build_parser().parse_args(["run", "--control"])
        err = io.StringIO()
        with redirect_stderr(err):
            self.assertEqual(cli.cmd_run(args), 2)
        self.assertIn("--follow-sync", err.getvalue())

    def test_the_flag_exists_and_is_off_by_default(self):
        p = cli.build_parser()
        self.assertFalse(p.parse_args(["run"]).control)
        self.assertTrue(p.parse_args(["run", "--control", "--follow-sync"]).control)


if __name__ == "__main__":
    unittest.main()
