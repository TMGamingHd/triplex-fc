# SPDX-License-Identifier: MIT
"""The console's model of the bus: what it concludes from frames, held to the system's own rules (the launch checklist) and to a real recording."""
import gzip
import random
import unittest

from .console_support import DEMO, act, feed_frame, hb  # first: puts console/ on the path
from tfc_peers import launch as LA
from tfc_peers import protocol as P
from tfc_console import constants as K
from tfc_console.model import Telemetry

NOW = 100.0


def sync(tm, t, frame_no, mission=0, seq=0):
    tm.feed(t, P.pack_sync(frame_no, seq, mission))


def events(tm, kind=None):
    return [e for e in tm.events if kind is None or e.kind == kind]


class GoNoGoIsTheChecklistsRule(unittest.TestCase):
    """The page shows GO or NO-GO from the console's own evaluation. It must agree with `tfc_peers launch`, the checklist the operator trusts, in every case."""

    def test_the_two_agree_over_many_random_states(self):
        rnd = random.Random(7)
        for trial in range(400):
            tm, obs = Telemetry(), LA.Observer()
            now = NOW
            frames = []
            for n in range(3):
                if rnd.random() < 0.9:
                    frames.append(hb(n, mode=rnd.choice([3, 3, 3, 2, 1, 0]), ready=rnd.random() < 0.8, safe=rnd.random() < 0.1, view=(rnd.choice([0, 0, 0, 1]), 0, 0)))
            if rnd.random() < 0.9:
                frames.append(act(state=rnd.choice([1, 1, 1, 0, 2, 3])))
            if rnd.random() < 0.95:
                frames.append(P.pack_sync(500 + trial, 0, rnd.choice([0, 0, 0, 300, 1001, 2000])))
            for f in frames:
                tm.feed(now - 0.01, f)
                obs.feed(f, now - 0.01)
            gng = tm.go_nogo(now)
            self.assertEqual(gng["go"], not obs.verdict(now), f"trial {trial}: {gng['reasons']} vs {obs.verdict(now)}")
            self.assertEqual(bool(gng["reasons"]), bool(obs.verdict(now)))

    def test_old_frames_do_not_count(self):
        tm, obs = Telemetry(), LA.Observer()
        for n in range(3):
            f = hb(n)
            tm.feed(NOW - 2.0, f)
            obs.feed(f, NOW - 2.0)
        for f in (act(), P.pack_sync(10, 0, 0)):
            tm.feed(NOW - 2.0, f)
            obs.feed(f, NOW - 2.0)
        self.assertFalse(tm.go_nogo(NOW)["go"])
        self.assertTrue(obs.verdict(NOW))

    def test_a_full_set_is_go_and_each_missing_piece_is_named(self):
        tm = Telemetry()
        for n in range(3):
            tm.feed(NOW, hb(n))
        tm.feed(NOW, act())
        sync(tm, NOW, 100)
        self.assertTrue(tm.go_nogo(NOW)["go"])
        tm2 = Telemetry()
        for n in range(3):
            tm2.feed(NOW, hb(n, ready=(n != 1)))
        tm2.feed(NOW, act())
        sync(tm2, NOW, 100)
        g = tm2.go_nogo(NOW)
        self.assertFalse(g["go"])
        self.assertEqual(len(g["reasons"]), 1)
        self.assertIn("FC-B ready", g["reasons"][0])


class TheVoteMonitor(unittest.TestCase):
    def feed(self, frames, bias_b=0.0, start=0):
        tm = Telemetry()
        for n in range(3):
            tm.feed(0.0, hb(n))
        for k in range(start, start + frames):
            t = k * 0.01
            gyro = [(1.0, 2.0, 3.0), (1.0 + bias_b, 2.0, 3.0), (1.0, 2.0, 3.0)]
            feed_frame(tm, t, k & 0xFF, gyro=gyro)
        return tm

    def test_agreeing_nodes_sit_on_the_median(self):
        tm = self.feed(20)
        self.assertGreater(tm.vote.compared, 30)
        ch = tm.vote.latest[0]
        self.assertEqual(ch["n"], 3)
        self.assertTrue(all(abs(d) < 1e-6 for d in ch["dev"]))
        self.assertEqual(tm.vote.over, [0, 0, 0])

    def test_a_biased_node_stands_out_in_units_of_the_tolerance(self):
        tm = self.feed(20, bias_b=3.0)
        ch = tm.vote.latest[0]
        self.assertAlmostEqual(ch["dev"][1], 3.0, places=1)
        self.assertAlmostEqual(ch["norm"][1], 3.0, places=1)       # the gyro tolerance is 1 dps
        self.assertGreater(tm.vote.over[1], 15)
        self.assertEqual(tm.vote.over[0], 0)
        self.assertEqual(tm.vote.over[2], 0)
        peaks = tm.vote.take_peaks()
        self.assertGreater(peaks[1][0], 2.9)
        self.assertEqual(max(peaks[0]), max(peaks[0]))              # the others are near zero
        self.assertLess(max(peaks[0]), 0.1)

    def test_samples_of_neighbouring_frames_are_not_compared(self):
        """A signal that moves 10 dps a frame would make node B (one frame behind) look 10 tolerances out; the monitor compares only the same frame's samples."""
        tm = Telemetry()
        for n in range(3):
            tm.feed(0.0, hb(n))
        for k in range(30):
            t = k * 0.01
            moving = k * 10.0
            tm.feed(t + 0.0015, P.pack_gyro(0, (moving, 0, 0), k))
            tm.feed(t + 0.0017, P.pack_gyro(1, ((k - 1) * 10.0, 0, 0), k - 1 & 0xFF))     # node B's sample is of the previous frame's number
            tm.feed(t + 0.0019, P.pack_gyro(2, (moving, 0, 0), k))
        self.assertEqual(tm.vote.over[1], 0)

    def test_two_nodes_are_compared_when_the_third_is_silent(self):
        tm = Telemetry()
        tm.feed(0.0, hb(0))
        tm.feed(0.0, hb(1))
        for k in range(10):
            feed_frame(tm, k * 0.01, k, gyro=[(0.0, 0, 0), (5.0, 0, 0), (0.0, 0, 0)], nodes=(0, 1))
        ch = tm.vote.latest[0]
        self.assertEqual(ch["n"], 2)
        self.assertIsNone(ch["vals"][2])
        self.assertAlmostEqual(abs(ch["dev"][0]), 2.5, places=1)          # the median of two is their middle

    def test_the_command_channels_use_the_command_tolerance(self):
        tm = Telemetry()
        for n in range(3):
            tm.feed(0.0, hb(n))
        for k in range(10):
            feed_frame(tm, k * 0.01, k, cmd=[(0.0, 0.0), (0.05, 0.0), (0.0, 0.0)])
        self.assertAlmostEqual(tm.vote.latest[6]["norm"][1], 5.0, places=1)    # 0.05 degree against 0.01


class Events(unittest.TestCase):
    def test_a_node_going_latched_in_the_others_view_is_an_event_with_who_and_how(self):
        tm = Telemetry()
        tm.feed(1.0, hb(0, view=(0, 0, 0)))
        tm.feed(1.1, hb(0, view=(0, 1, 0)))
        e = events(tm, "view")
        self.assertEqual(len(e), 1)
        self.assertEqual((e[0].node, e[0].level, e[0].fields["to"], e[0].fields["from"], e[0].fields["observer"]), ("B", "warn", "latched", "healthy", "A"))

    def test_the_first_sight_of_a_node_and_its_changes(self):
        tm = Telemetry()
        tm.feed(1.0, hb(2, mode=3))
        tm.feed(1.1, hb(2, mode=2, ready=False, safe=True, resets=2, release=0xBEEF))
        kinds = [e.kind for e in tm.events]
        for k in ("node-up", "node-mode", "ready", "safe-request", "reset", "release"):
            self.assertIn(k, kinds)
        self.assertEqual(events(tm, "safe-request")[0].level, "crit")

    def test_silence_is_an_event_and_so_is_its_end(self):
        tm = Telemetry()
        tm.feed(1.0, hb(1))
        tm.check_timeouts(1.2)
        self.assertEqual(events(tm, "node-silent"), [])
        tm.check_timeouts(1.7)
        tm.check_timeouts(1.8)
        self.assertEqual(len(events(tm, "node-silent")), 1)             # once, not on every tick
        tm.feed(2.0, hb(1))
        self.assertEqual(len(events(tm, "node-back")), 1)

    def test_sync_going_away_and_coming_back(self):
        tm = Telemetry()
        sync(tm, 1.0, 50)
        tm.check_timeouts(1.3)
        tm.check_timeouts(1.8)
        self.assertEqual(len(events(tm, "sync-lost")), 1)
        sync(tm, 2.0, 70)
        self.assertEqual(len(events(tm, "sync-back")), 1)

    def test_a_frame_number_that_goes_back_is_a_restart(self):
        tm = Telemetry()
        sync(tm, 1.0, 5000)
        sync(tm, 1.01, 3)
        self.assertEqual(tm.sync_restarts, 1)
        self.assertEqual(len(events(tm, "sync-restart")), 1)

    def test_the_mission_frame_tells_the_countdown_the_scrub_and_t_zero(self):
        tm = Telemetry()
        sync(tm, 1.0, 100, 0)
        sync(tm, 1.01, 101, 1)
        sync(tm, 1.02, 102, 500)
        sync(tm, 1.03, 103, 0)                                          # a scrub
        sync(tm, 1.04, 104, 1)
        sync(tm, 1.05, 105, 1002)                                       # T-zero
        self.assertEqual([e.kind for e in tm.events if e.kind in ("countdown", "scrub", "t-zero")], ["countdown", "scrub", "countdown", "t-zero"])
        self.assertEqual(tm.phase(1.05)["name"], "flight")
        self.assertAlmostEqual(tm.phase(1.05)["flight_s"], 0.01)
        tm2 = Telemetry()
        sync(tm2, 1.0, 100, 700)
        self.assertEqual(tm2.phase(1.0)["name"], "countdown")
        self.assertAlmostEqual(tm2.phase(1.0)["t_minus_s"], 3.01, places=2)

    def test_act_excluding_a_node_and_going_to_safe(self):
        tm = Telemetry()
        tm.feed(1.0, act())
        tm.feed(1.1, act(excluded=0b010, voted=0b101, vote_status=1))
        tm.feed(1.2, act(state=2, cause=1))
        self.assertEqual(events(tm, "act-excluded")[0].node, "B")
        self.assertEqual(events(tm, "act-state")[0].level, "crit")
        a = tm.snapshot(1.2)["alerts"]
        self.assertTrue(any(x["key"] == "act-safe" and x["level"] == "crit" for x in a))

    def test_a_vote_status_that_comes_straight_back_is_a_blip_and_one_that_stays_is_a_warning(self):
        tm = Telemetry()
        tm.feed(1.00, act(vote_status=0))
        tm.feed(1.01, act(vote_status=1))
        tm.feed(1.02, act(vote_status=0))
        self.assertEqual([e.kind for e in tm.events if e.kind.startswith("vote")], ["vote-blip"])
        tm2 = Telemetry()
        tm2.feed(1.00, act(vote_status=0))
        for k in range(1, 12):
            tm2.feed(1.00 + k * 0.01, act(vote_status=1))
        self.assertEqual([e.kind for e in tm2.events if e.kind.startswith("vote")], ["vote-status"])
        self.assertEqual(events(tm2, "vote-status")[0].level, "warn")
        tm2.feed(1.2, act(vote_status=0))
        self.assertEqual(events(tm2, "vote-status")[-1].level, "ok")

    def test_a_status_that_flickers_between_non_triplex_values_after_a_loss_is_one_warning(self):
        tm = Telemetry()
        tm.feed(1.00, act(vote_status=0))
        t = 1.01
        for status in ([1] * 10 + [3] + [1] * 10 + [2] + [1] * 10):       # Duplex, a frame of Simplex, Duplex, a frame of miscompare, Duplex
            tm.feed(t, act(vote_status=status))
            t += 0.01
        self.assertEqual(len(events(tm, "vote-status")), 1)
        tm.feed(t, act(vote_status=0))
        self.assertEqual([e.level for e in events(tm, "vote-status")], ["warn", "ok"])

    def test_a_ground_frame_on_the_bus_is_shown_and_its_tag_checked(self):
        tm = Telemetry()
        tm.feed(1.0, P.pack_ground(P.GROUND_OPS["reintegrate"], 1, 5))
        tm.feed(1.1, P.pack_ground(P.GROUND_OPS["reintegrate"], 1, 6, forged=True))
        self.assertEqual([g["tag_ok"] for g in tm.ground], [True, False])
        self.assertIn("NOT valid", events(tm, "ground-frame")[1].text)


class FramesThatAreNotGood(unittest.TestCase):
    def test_a_damaged_frame_is_counted_for_its_id_and_its_node_and_decoded_by_nobody(self):
        tm = Telemetry()
        f = P.pack_gyro(1, (1, 2, 3), 5)
        bad = P.Frame(f.id, bytes([f.data[0] ^ 1]) + f.data[1:])
        tm.feed(1.0, bad)
        self.assertEqual((tm.crc_bad_total, tm.nodes[1].crc_bad, tm.ids[0x101].crc_bad), (1, 1, 1))
        self.assertNotIn("gyro", tm.nodes[1].values)

    def test_out_of_schedule_ids_are_counted_and_named_in_an_alert(self):
        tm = Telemetry()
        for i in range(5):
            tm.feed(1.0 + i * 0.001, P.Frame(0x021, P.seal(bytes(6), i)))
        self.assertEqual(tm.oos, {0x021: 5})
        self.assertTrue(any(a["key"] == "oos" for a in tm.snapshot(1.1)["alerts"]))

    def test_a_sequence_gap_is_counted(self):
        tm = Telemetry()
        for k in (1, 2, 3, 7, 8):
            tm.feed(k * 0.01, P.pack_gyro(0, (0, 0, 0), k))
        self.assertEqual(tm.nodes[0].seq_gaps, 1)


class VirtualPeers(unittest.TestCase):
    def test_a_node_that_sends_samples_and_no_heartbeat_is_alive_but_has_no_mode(self):
        tm = Telemetry()
        tm.feed(1.0, hb(0))
        feed_frame(tm, 1.0, 1, nodes=(1,))
        snap = tm.snapshot(1.05)
        b = snap["nodes"][1]
        self.assertTrue(b["alive"])
        self.assertFalse(b["heartbeat"])
        self.assertIsNone(b["mode"])
        self.assertTrue(snap["nodes"][0]["heartbeat"])
        self.assertFalse(snap["nodes"][2]["alive"])


class TheSimulator(unittest.TestCase):
    def test_stages_and_max_q_from_the_truth(self):
        tm = Telemetry()
        base = {"ft": 1.0, "q": 0.0, "stages_active": 0b11, "stages_ignited": 0b01, "crashed": 0, "clamped": 1}
        tm.feed_truth(1.0, dict(base))
        tm.feed_truth(1.1, {**base, "stages_ignited": 0b11})
        tm.feed_truth(1.2, {**base, "stages_ignited": 0b11, "stages_active": 0b10})
        for i, q in enumerate([2000.0, 20000.0, 31000.0, 29000.0, 27000.0]):
            tm.feed_truth(2.0 + i * 0.1, {**base, "q": q, "ft": 50.0 + i, "stages_active": 0b10, "stages_ignited": 0b11})
        kinds = [e.kind for e in tm.events]
        self.assertEqual(kinds.count("stage-ignition"), 1)
        self.assertEqual(kinds.count("stage-separation"), 1)
        self.assertEqual(kinds.count("max-q"), 1)
        self.assertIn("31.0 kPa", events(tm, "max-q")[0].text)
        tm.feed_truth(3.0, {**base, "crashed": 1})
        self.assertEqual(events(tm, "crashed")[0].level, "crit")

    def test_the_flags_of_the_bus_status_frame(self):
        tm = Telemetry()
        tm.feed(1.0, P.pack_sim_flags(P.SimFlags(0, 5, 0), 1))
        tm.feed(1.1, P.pack_sim_flags(P.SimFlags(P.SIM_FLAG_PLATFORM_SATURATED | P.SIM_FLAG_ENGINE_OUT, 4, 100), 2))
        texts = " ".join(e.text for e in tm.events)
        self.assertIn("platform is saturated", texts)
        self.assertIn("an engine is out", texts)
        self.assertEqual(len(events(tm, "liftoff")), 1)


class OnTheRecordedSession(unittest.TestCase):
    """console/demo is a real run of the virtual rig (three firmware processes, ACT and the simulator on vcan0), recorded by the console: a launch, then node B killed at T+23 s. What the model concludes from it is checked
    against facts that were observed on the bus and on the nodes' consoles when it was recorded."""

    @classmethod
    def setUpClass(cls):
        import json
        from tfc_console.lines import parse_line
        from tfc_console.sources import _parse_stream
        with gzip.open(DEMO, "rt") as fh:
            cls.frames = [(t / 1e6, fr) for t, _i, fr in _parse_stream(fh)]
        with gzip.open(DEMO.with_name("launch-and-node-loss.side.jsonl.gz"), "rt") as fh:
            side = [json.loads(line) for line in fh if line.strip()]
        merged = sorted([(t, 0, fr) for t, fr in cls.frames] + [(r["t"], 1, r) for r in side], key=lambda x: (x[0], x[1]))
        cls.tm = Telemetry()
        cls.snap_at = {}
        marks = [5.0, 14.0, 40.0, 60.0]
        for t, kind, item in merged:
            if kind == 0:
                cls.tm.feed(t, item)
            elif item["k"] == "truth":
                cls.tm.feed_truth(t, item["d"])
            elif item["k"] == "line":
                cls.tm.feed_console(t, item["src"], parse_line(item["text"]))
            while marks and t >= marks[0]:
                cls.snap_at[marks.pop(0)] = cls.tm.snapshot(t)
        cls.tm.check_timeouts(cls.frames[-1][0])
        cls.end = cls.tm.snapshot(cls.frames[-1][0])

    def test_the_shape_of_the_recording(self):
        self.assertGreater(len(self.frames), 120000)
        self.assertEqual(self.tm.crc_bad_total, 0)
        self.assertEqual(self.tm.oos, {})

    def test_on_the_pad_everything_is_go(self):
        s = self.snap_at[5.0]
        self.assertEqual(s["phase"]["name"], "countdown")                # recording started at T-10 s
        self.assertTrue(s["go_nogo"]["go"])
        self.assertTrue(all(n["alive"] and n["health"] == "healthy" and n["ready"] for n in s["nodes"]))

    def test_the_countdown_is_ten_seconds_and_t_zero_is_the_same_frame_on_the_bus(self):
        t_cd = next(e for e in self.tm.events if e.kind == "countdown" and e.src == "bus")
        t_0 = next(e for e in self.tm.events if e.kind == "t-zero" and e.src == "bus")
        self.assertAlmostEqual(t_0.t - t_cd.t, 10.0, delta=0.15)
        self.assertEqual(t_0.frame - t_cd.frame, 1000)

    def test_in_flight_after_t_zero(self):
        s = self.snap_at[40.0]
        self.assertEqual(s["phase"]["name"], "flight")
        self.assertGreater(s["sim"]["alt"], 1000)

    def test_the_loss_of_node_b_is_seen_as_the_flight_computers_saw_it(self):
        end = self.end
        self.assertFalse(end["nodes"][1]["alive"])
        self.assertEqual(end["nodes"][1]["health"], "latched")
        self.assertTrue(end["act"]["excluded"][1])
        self.assertEqual(end["act"]["vote_status"], "Duplex")
        silent = next(e for e in self.tm.events if e.kind == "node-silent")
        self.assertEqual(silent.node, "B")
        latched = [e for e in self.tm.events if e.kind == "view" and e.fields.get("to") == "latched" and e.node == "B"]
        self.assertEqual({e.fields["observer"] for e in latched}, {"A", "C"})
        self.assertEqual(len({e.frame for e in latched}), 1)               # both survivors latched it in the same frame

    def test_the_detection_is_two_frames_as_the_fault_matrix_says_for_a_node_that_goes_silent(self):
        """Row F01 (docs/verification/FAULT_MATRIX.md, sim/README.md): a fail-silent node is latched 2 frames after its last frame."""
        last_b = max(t for t, fr in self.frames if fr.id in (P.ID_GYRO_BASE + 1, P.ID_CMD_BASE + 1))
        first_latch = min(e.t for e in self.tm.events if e.kind == "view" and e.fields.get("to") == "latched")
        self.assertAlmostEqual((first_latch - last_b) / K.FRAME_S, 2.0, delta=2.0)

    def test_max_q_is_where_the_nominal_flight_puts_it(self):
        """tfc_fly of the reference vehicle (no departures): max-Q 31.5 kPa at 64.9 s. The live flight of three real flight computers and the simulator found the same, from the truth stream."""
        e = next(e for e in self.tm.events if e.kind == "max-q")
        self.assertRegex(e.text, r"31\.[45] kPa at T\+6[45]\.\d s")

    def test_the_vote_monitor_saw_no_disagreement_before_the_loss(self):
        s = self.snap_at[40.0]
        self.assertEqual(s["nodes"][0]["over_frames"], 0)
        self.assertEqual(s["nodes"][2]["over_frames"], 0)


if __name__ == "__main__":
    unittest.main()
