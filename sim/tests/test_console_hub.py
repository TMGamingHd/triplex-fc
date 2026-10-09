# SPDX-License-Identifier: MIT
"""The hub, the recorder and the replay: that what is recorded is what is replayed, and that a replay that seeks lands in the same state as one that plays straight through."""
import json
import tempfile
import time
import unittest
from pathlib import Path

from .console_support import DEMO, act, hb  # first: puts console/ on the path
from tfc_peers import protocol as P
from tfc_console.hub import Hub, dumps
from tfc_console.recorder import Recorder, sidecar_of
from tfc_console.sources import LogSource


class TheHub(unittest.TestCase):
    def test_ticks_fill_the_history_and_publish_state(self):
        hub = Hub(clock=lambda: 10.0)
        q = hub.subscribe()
        hub.ingest_frame(10.0, hb(0))
        hub.tick(10.0)
        hub.tick(10.1)
        cols = hub.history_columns()
        self.assertEqual(cols["t"], [10.0, 10.1])
        self.assertIn("act_p", cols["series"])
        msgs = []
        while not q.empty():
            msgs.append(q.get_nowait())
        self.assertTrue(any(m.startswith("event: state\ndata: ") for m in msgs))

    def test_history_window_is_the_last_seconds(self):
        hub = Hub()
        for i in range(100):
            hub.tick(i * 0.1)
        self.assertEqual(len(hub.history_columns(2.0)["t"]), 21)

    def test_a_slow_page_loses_its_oldest_messages_and_nobody_else_waits(self):
        hub = Hub()
        q = hub.subscribe()
        for i in range(1000):
            hub._publish("state", {"i": i})
        self.assertEqual(q.qsize(), 400)
        first = q.get_nowait()
        self.assertIn('"i":600', first)

    def test_nan_goes_out_as_null_not_as_a_broken_stream(self):
        self.assertEqual(json.loads(dumps({"a": float("nan"), "b": [1.0, float("inf")]})), {"a": None, "b": [1.0, None]})

    def test_console_lines_are_kept_per_source_and_coloured_text_is_cleaned(self):
        hub = Hub()
        hub.ingest_line("A", "\x1b[0;39m[frame 5] node B LATCHED OUT: vote disagreement\x1b[0m")
        hub.ingest_line("A", "\x1b[0;39m")
        self.assertEqual([r["text"] for r in hub.recent_lines()["A"]], ["[frame 5] node B LATCHED OUT: vote disagreement"])
        self.assertEqual(hub.model.events[-1].kind, "latched-out")

    def test_the_status_line_updates_the_node_and_is_not_an_event(self):
        hub = Hub()
        n0 = len(hub.model.events)
        hub.ingest_line("B", "[frame 100] TRIPLEX  A+ B+ C+  | crc=0 seq=0 wcet_frame=9100")
        self.assertEqual(hub.model.nodes[1].console["wcet_frame"], 9100)
        self.assertEqual(len(hub.model.events), n0)

    def test_frames_for_the_monitor_are_decoded_and_filterable(self):
        hub = Hub()
        hub.ingest_frame(1.0, P.pack_gyro(1, (1, 2, 3), 4))
        hub.ingest_frame(1.1, hb(0))
        hub.ingest_frame(1.2, P.Frame(0x101, bytes(8)))
        r = hub.recent_frames(0, 10, None)
        self.assertEqual([f["ok"] for f in r["frames"]], [True, True, False])
        self.assertIn("GYRO", r["frames"][0]["text"])
        self.assertEqual(len(hub.recent_frames(0, 10, {0x400})["frames"]), 1)
        self.assertEqual(len(hub.recent_frames(r["last"], 10)["frames"]), 0)

    def test_a_failing_recorder_is_dropped_with_a_note_and_the_caller_goes_on(self):
        hub = Hub()

        class Bad:
            def frame(self, *a):
                raise OSError("disk full")

        hub.recorder = Bad()
        hub.ingest_frame(1.0, hb(0))
        self.assertIsNone(hub.recorder)
        self.assertEqual(hub.model.events[-1].kind, "record")
        self.assertEqual(hub.model.events[-1].level, "crit")


class RecordThenReplay(unittest.TestCase):
    def test_what_is_recorded_is_what_a_replay_shows(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub(clock=lambda: 50.0)
            rec = Recorder(Path(d), 50.0, "vcan0", "t")
            hub.recorder = rec
            hub.ingest_frame(50.10, hb(0))
            hub.ingest_frame(50.11, act())
            hub.ingest_line("A", "[frame 4] node B LATCHED OUT: vote disagreement", 50.12)
            hub.ingest_truth({"alt": 12.0, "q": 3.0, "ft": 1.0, "clamped": 0}, 50.13)
            hub.note("info", "OP", "command", "operator command sent: noop (counter 1)")
            info = rec.close()
            self.assertEqual(info["frames"], 2)
            self.assertEqual(rec.path.suffix, ".log")
            self.assertEqual(sidecar_of(rec.path), rec.side_path)
            lines = rec.path.read_text().splitlines()
            self.assertEqual(len(lines), 2)
            self.assertRegex(lines[0], r"^\(0\.100000\) vcan0 400#")
            # the replay
            h2 = Hub()
            src = LogSource(h2, rec.path)
            self.assertTrue(src.side)
            src._feed_until(src.t_end + 1.0)
            kinds = [e.kind for e in h2.model.events]
            self.assertIn("latched-out", kinds)
            self.assertIn("command", kinds)
            self.assertEqual(h2.model.truth["alt"], 12.0)
            self.assertEqual(h2.model.nodes[0].hb.mode, 3)

    def test_a_log_without_a_sidecar_replays_the_bus_alone(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.log"
            p.write_text("(0.001000) vcan0 010#0100000000000000\n")
            p.write_text("".join(f"({i / 100:.6f}) vcan0 {f.id:03X}#{f.data.hex().upper()}\n" for i, f in enumerate([P.pack_sync(i, i & 255, 0) for i in range(20)])))
            src = LogSource(Hub(), p)
            self.assertEqual(src.side, [])
            self.assertEqual(len(src.frames), 20)

    def test_a_log_with_no_frames_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "empty.log"
            p.write_text("")
            with self.assertRaises(ValueError):
                LogSource(Hub(), p)

    def test_a_bad_line_names_its_line(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.log"
            p.write_text("(0.001000) vcan0 010#0100000000000000\nnot a frame\n")
            with self.assertRaisesRegex(ValueError, "line 2"):
                LogSource(Hub(), p)


class SeekIsReplayFromTheStart(unittest.TestCase):
    """The votes, the strikes and the mission clock are histories, not values: the only state that is true after a seek is the one a straight run reaches."""

    def test_seeking_to_a_time_gives_the_state_of_playing_to_it(self):
        straight = Hub()
        a = LogSource(straight, DEMO)
        a._feed_until(30.0)
        sought = Hub()
        b = LogSource(sought, DEMO)
        b._feed_until(70.0)
        b._rewind_to(30.0)

        def essence(h):
            s = h.snapshot()
            return (s["frame"], s["phase"], s["act"]["vote_status"], [(n["name"], n["health"], n["alive"]) for n in s["nodes"]], s["milestones"], s["truth"]["alt"] if s["truth"] else None, h.model.rx_total, len(h.model.events))
        self.assertEqual(essence(sought), essence(straight))

    def test_play_pause_speed_and_seek_from_another_thread(self):
        hub = Hub()
        src = LogSource(hub, DEMO, speed=32.0)
        hub.start()
        src.start()
        try:
            end = time.monotonic() + 15
            while src.playing and time.monotonic() < end:
                time.sleep(0.05)
            self.assertFalse(src.playing)
            self.assertAlmostEqual(src.pos, src.t_end, places=2)
            src.seek(20.0)          # the player forgets and feeds 20 s again from the start: how long that takes depends on the machine
            from tfc_console.server import wait_for
            self.assertTrue(wait_for(lambda: abs(src.pos - 20.0) < 0.5 and hub.status["pos"] > 19.0, 30.0), (src.pos, hub.status))
            self.assertEqual(hub.status["state"], "paused")
            src.set_speed(1000)
            self.assertEqual(src.speed, 32.0)                # clamped
            src.play()
            time.sleep(0.3)
            self.assertTrue(src.playing or src.pos >= src.t_end)
        finally:
            src.stop()
            hub.stop()


class TheRecordedSessionOpensAsAReplay(unittest.TestCase):
    def test_the_demo_has_its_sidecar_and_plays_to_the_end_state(self):
        hub = Hub()
        src = LogSource(hub, DEMO)
        self.assertTrue(src.side)
        src._feed_until(src.t_end)                 # not past the end: a second with no frames would make every node silent, which is true and not what this is about
        s = hub.snapshot()
        self.assertEqual(s["phase"]["name"], "flight")
        self.assertEqual(s["nodes"][1]["health"], "latched")
        self.assertIn("max-q", s["milestones"])
        self.assertTrue(any(e.src == "CON" and "killed" in e.text for e in hub.model.events))


if __name__ == "__main__":
    unittest.main()
