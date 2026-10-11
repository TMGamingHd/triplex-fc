# SPDX-License-Identifier: MIT
"""The 3D viewer's data path (console/tfc_console/viewer.py, docs/design/VIEWER.md): the poses and the spec from the simulator, the stream they go out on, a recording of them and a replay of the recording, the pose-file
player and its transport, the routes, and the page's own files (every import resolves, the vendored library is the one the README says)."""
import hashlib
import json
import queue
import re
import socket
import subprocess
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.app import SourceManager, register_core
from tfc_console.hub import Hub
from tfc_console.recorder import Recorder, open_text
from tfc_console.server import App, start_in_thread
from tfc_console.sources import LogSource
from tfc_console.vehicles import Vehicles
from tfc_console.viewer import PoseReplay, PoseSource, ViewerHub, ViewerService

TOKEN = "v" * 20
WEB = ROOT / "console" / "web"
FLY = ROOT / "build" / "host" / "tfc_fly"


def spec_line(name="t", engines=2):
    return json.dumps({"k": "spec", "v": 1, "name": name, "vehicle": {"engines": [{}] * engines, "stages": [{}]}, "air": {"step_m": 1000.0, "density": [1.2, 1.0]}}, separators=(",", ":"))


def pose_line(tt, t=None, **extra):
    d = {"k": "pose", "t": tt if t is None else t, "fr": int(tt * 100), "tt": tt, "r": [6378137.0 + tt, 0, 0], "alt": tt, **extra}
    return json.dumps(d, separators=(",", ":"))


def write_pose_file(path, n=10, step=0.5, t0=0.0):
    lines = [spec_line()] + [pose_line(t0 + i * step) for i in range(n)]
    path.write_text("\n".join(lines) + "\n")
    return path


def drain(q, timeout=0.0):
    out = []
    end = time.monotonic() + timeout
    while True:
        try:
            out.append(q.get(timeout=max(0.0, end - time.monotonic())) if timeout else q.get_nowait())
        except queue.Empty:
            return out


def kinds(msgs):
    return [m.split("\n", 1)[0].removeprefix("event: ") for m in msgs]


class ViewerHubTests(unittest.TestCase):
    def test_a_pose_and_a_spec_go_to_every_subscriber_as_they_came(self):
        v = ViewerHub()
        a, b = v.subscribe(), v.subscribe()
        v.ingest(spec_line())
        v.ingest(pose_line(1.0))
        for q in (a, b):
            msgs = drain(q)
            self.assertEqual(kinds(msgs), ["spec", "pose"])
            self.assertIn('"tt":1.0', msgs[1])
        v.unsubscribe(b)
        v.ingest(pose_line(2.0))
        self.assertEqual(len(drain(a)), 1)
        self.assertEqual(drain(b), [])

    def test_the_same_spec_again_is_not_sent_again_and_a_changed_one_is(self):
        """The simulator repeats its spec every two seconds (a datagram nobody listened to is lost): a page must not be told of it each time."""
        v = ViewerHub()
        q = v.subscribe()
        v.ingest(spec_line())
        v.ingest(spec_line())
        self.assertEqual(kinds(drain(q)), ["spec"])
        v.ingest(spec_line(name="other"))
        self.assertEqual(kinds(drain(q)), ["spec"])

    def test_the_hello_has_the_spec_and_the_latest_pose(self):
        v = ViewerHub()
        self.assertEqual(json.loads(v.hello()), {"spec": None, "pose": None, "source": {"kind": "none"}})
        v.ingest(spec_line())
        v.ingest(pose_line(3.5))
        h = json.loads(v.hello())
        self.assertEqual(h["spec"]["name"], "t")
        self.assertEqual(h["pose"]["tt"], 3.5)

    def test_something_that_is_neither_is_counted_and_dropped(self):
        v = ViewerHub()
        q = v.subscribe()
        v.ingest('{"k":"other"}')
        v.ingest("not json at all")
        self.assertEqual(v.bad, 2)
        self.assertEqual(drain(q), [])
        self.assertIsNone(v.pose_raw)

    def test_a_page_that_cannot_keep_up_loses_its_oldest_poses_not_the_newest(self):
        v = ViewerHub()
        q = v.subscribe()
        for i in range(1000):
            v.ingest(pose_line(i * 0.02))
        got = drain(q)
        self.assertLessEqual(len(got), 400)
        self.assertIn('"tt":19.98', got[-1])               # the newest is there
        self.assertNotIn('"tt":0.0,', got[0])              # the oldest is gone

    def test_a_seek_shows_only_where_it_ended(self):
        """A replay that seeks feeds the model from the start: a page must not be sent every pose on the way."""
        v = ViewerHub()
        q = v.subscribe()
        v.ingest(spec_line())
        drain(q)
        v.muted = True
        for i in range(200):
            v.ingest(pose_line(i * 0.1))
        self.assertEqual(drain(q), [])
        v.flush()
        msgs = drain(q)
        self.assertEqual(kinds(msgs), ["spec", "pose"])
        self.assertIn('"tt":19.9', msgs[1])
        self.assertFalse(v.muted)

    def test_a_reset_tells_the_pages_and_forgets_the_pose(self):
        v = ViewerHub()
        v.ingest(pose_line(1.0))
        q = v.subscribe()
        v.reset()
        self.assertEqual(kinds(drain(q)), ["reset"])
        self.assertIsNone(json.loads(v.hello())["pose"])

    def test_the_source_is_published_and_kept(self):
        v = ViewerHub()
        q = v.subscribe()
        v.set_source(kind="pose-file", file="x", playing=True)
        self.assertEqual(kinds(drain(q)), ["source"])
        self.assertEqual(json.loads(v.hello())["source"]["file"], "x")
        self.assertTrue(v.status()["subscribers"] == 1 and v.status()["have_spec"] is False)

    def test_nothing_is_built_when_nobody_listens(self):
        v = ViewerHub()
        v.ingest(pose_line(1.0))
        self.assertEqual(v.count, 1)                       # kept as the latest, so a page that opens later has something to show
        self.assertEqual(json.loads(v.hello())["pose"]["tt"], 1.0)


class PoseSourceTests(unittest.TestCase):
    def test_datagrams_from_the_simulator_arrive_in_the_hub(self):
        hub = Hub()
        src = PoseSource(hub, 0)
        src.start()
        try:
            q = hub.viewer.subscribe()
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(spec_line().encode(), ("127.0.0.1", src.port))
            s.sendto(pose_line(1.0).encode(), ("127.0.0.1", src.port))
            s.sendto(b"\xff\xfe not text", ("127.0.0.1", src.port))
            s.close()
            msgs = drain(q, timeout=2.0)
            end = time.monotonic() + 2.0
            while len(msgs) < 2 and time.monotonic() < end:
                msgs += drain(q, timeout=0.2)
            self.assertEqual(kinds(msgs), ["spec", "pose"])
            end = time.monotonic() + 2.0
            while hub.viewer.bad < 1 and time.monotonic() < end:
                time.sleep(0.02)
            self.assertEqual(hub.viewer.bad, 1)            # the datagram that was not text
        finally:
            src.stop()

    def test_a_port_that_is_taken_gives_a_free_one(self):
        a = PoseSource(Hub(), 0)
        try:
            b = PoseSource(Hub(), a.port)
            try:
                self.assertNotEqual(a.port, b.port)
            finally:
                b.sock.close()
        finally:
            a.sock.close()

    def test_while_a_pose_file_plays_the_live_poses_are_not_shown_but_are_recorded(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub()
            hub.recorder = Recorder(Path(d), 0.0, "vcan0")
            q = hub.viewer.subscribe()
            hub.viewer.set_source(kind="pose-file", file="x")
            drain(q)
            hub.ingest_pose(pose_line(1.0), 0.1)
            self.assertEqual(drain(q), [])                                         # the file's flight is not mixed with the live one
            hub.viewer.set_source(kind="none")
            drain(q)
            hub.ingest_pose(pose_line(2.0), 0.2)
            self.assertEqual(kinds(drain(q)), ["pose"])                            # closed: the live ones show again
            info = hub.recorder.close()
            with open(info["file"].replace(".log", ".side.jsonl")) as fh:
                self.assertEqual([json.loads(ln)["d"]["tt"] for ln in fh], [1.0, 2.0])

    def test_a_pose_with_a_recorder_running_is_recorded_as_it_came(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub()
            hub.recorder = Recorder(Path(d), 0.0, "vcan0")
            hub.ingest_pose(spec_line(), 0.5)
            hub.ingest_pose(pose_line(2.0), 1.25)
            info = hub.recorder.close()
            with open(info["file"].replace(".log", ".side.jsonl")) as fh:
                lines = [json.loads(ln) for ln in fh]
            self.assertEqual([x["k"] for x in lines], ["spec", "pose"])
            self.assertEqual(lines[1]["d"]["tt"], 2.0)
            self.assertAlmostEqual(lines[1]["t"], 1.25, 3)


class PoseReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.hub = Hub()

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_file_without_a_spec_or_poses_is_refused_with_a_reason(self):
        (self.dir / "a.pose.jsonl").write_text(pose_line(1.0) + "\n")
        with self.assertRaises(ValueError) as e:
            PoseReplay(self.hub, self.dir / "a.pose.jsonl")
        self.assertIn("no spec", str(e.exception))
        (self.dir / "b.pose.jsonl").write_text(spec_line() + "\n")
        with self.assertRaises(ValueError) as e:
            PoseReplay(self.hub, self.dir / "b.pose.jsonl")
        self.assertIn("no poses", str(e.exception))

    def test_it_plays_in_order_at_its_speed_and_stops_at_the_end(self):
        p = write_pose_file(self.dir / "f.pose.jsonl", n=6, step=0.1)
        q = self.hub.viewer.subscribe()
        r = PoseReplay(self.hub, p, speed=8.0)
        r.start()
        try:
            end = time.monotonic() + 5.0
            msgs = []
            while time.monotonic() < end and not any('"state":"paused"' in m for m in msgs):
                msgs += drain(q, timeout=0.1)
            poses = [json.loads(m.split("data: ", 1)[1])["tt"] for m in msgs if m.startswith("event: pose")]
            self.assertEqual(poses, sorted(poses))
            self.assertAlmostEqual(poses[0], 0.0)
            self.assertAlmostEqual(poses[-1], 0.5)
            self.assertFalse(r.playing)                    # at the end it stops (it does not loop unless it was asked)
            self.assertEqual(json.loads(self.hub.viewer.hello())["source"]["kind"], "pose-file")
        finally:
            r.stop()

    def test_a_seek_resets_the_pages_and_shows_the_pose_it_landed_on(self):
        p = write_pose_file(self.dir / "f.pose.jsonl", n=40, step=0.5)
        r = PoseReplay(self.hub, p, speed=1.0, autoplay=False)
        r.start()
        try:
            time.sleep(0.3)
            q = self.hub.viewer.subscribe()
            r.seek(7.3)
            end = time.monotonic() + 3.0
            msgs = []
            while time.monotonic() < end and not any(m.startswith("event: pose") for m in msgs):
                msgs += drain(q, timeout=0.1)
            ks = kinds(msgs)
            self.assertEqual(ks[0], "reset")
            self.assertIn("pose", ks)
            tt = [json.loads(m.split("data: ", 1)[1])["tt"] for m in msgs if m.startswith("event: pose")][0]
            self.assertEqual(tt, 7.0)                      # the last pose at or before the time asked for
            self.assertAlmostEqual(r.pos, 7.3, 2)
        finally:
            r.stop()

    def test_the_speed_is_kept_in_range_and_a_pause_holds_the_position(self):
        p = write_pose_file(self.dir / "f.pose.jsonl", n=100, step=0.5)
        r = PoseReplay(self.hub, p, speed=1.0, autoplay=False)
        r.start()
        try:
            r.set_speed(1000.0)
            self.assertEqual(r.speed, 32.0)
            r.set_speed(0.0)
            self.assertEqual(r.speed, 0.05)
            r.seek(10.0)
            time.sleep(0.4)
            pos = r.pos
            time.sleep(0.4)
            self.assertEqual(r.pos, pos)
        finally:
            r.stop()

    def test_a_loop_starts_again_at_the_end(self):
        p = write_pose_file(self.dir / "f.pose.jsonl", n=4, step=0.1)
        r = PoseReplay(self.hub, p, speed=32.0, loop=True)
        r.start()
        try:
            time.sleep(1.0)
            self.assertTrue(r.playing)
            self.assertGreater(self.hub.viewer.count, 8)   # more poses than the file has: it went round
        finally:
            r.stop()

    def test_a_gzip_file_plays_too(self):
        import gzip
        p = write_pose_file(self.dir / "f.pose.jsonl", n=3)
        gz = self.dir / "g.pose.jsonl.gz"
        gz.write_bytes(gzip.compress(p.read_bytes()))
        r = PoseReplay(self.hub, gz, autoplay=False)
        self.assertEqual(len(r.poses), 3)


class ViewerServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / "console" / "demo").mkdir(parents=True)
        (self.root / "logs").mkdir()
        (self.root / "console" / "models").mkdir()
        self.hub = Hub()
        self.vehicles = Vehicles(self.root, self.root / "state")
        self.svc = ViewerService(self.hub, self.root, self.root / "state", self.vehicles)

    def tearDown(self):
        self.svc.close()
        self.tmp.cleanup()

    def test_the_pose_files_are_listed_from_the_demo_the_logs_and_the_flown_ones_only(self):
        write_pose_file(self.root / "console" / "demo" / "d.pose.jsonl")
        write_pose_file(self.root / "logs" / "r.pose.jsonl")
        (self.root / "logs" / "other.txt").write_text("x")
        (self.root / "state" / "viewer").mkdir(parents=True)
        write_pose_file(self.root / "state" / "viewer" / "f.pose.jsonl")
        names = sorted((f["where"], f["name"]) for f in self.svc.pose_files())
        self.assertEqual(names, [("demo", "d.pose.jsonl"), ("flown", "f.pose.jsonl"), ("recorded", "r.pose.jsonl")])

    def test_a_name_that_is_a_path_is_never_opened(self):
        outside = self.root / "secret.pose.jsonl"
        write_pose_file(outside)
        for bad in (str(outside), "../secret.pose.jsonl", "nope.pose.jsonl", ""):
            with self.assertRaises(ValueError):
                self.svc.open(bad)

    def test_open_plays_and_a_second_open_replaces_the_first(self):
        write_pose_file(self.root / "console" / "demo" / "a.pose.jsonl", n=5)
        write_pose_file(self.root / "console" / "demo" / "b.pose.jsonl", n=5)
        self.svc.open("a.pose.jsonl")
        first = self.svc.player
        self.svc.open("b.pose.jsonl")
        self.assertIsNot(self.svc.player, first)
        self.assertFalse(first.is_alive())
        time.sleep(0.2)
        self.assertEqual(self.hub.viewer.source["file"], "b.pose.jsonl")
        self.svc.close()
        self.assertEqual(self.hub.viewer.source["kind"], "none")

    def test_control_needs_an_open_file_and_knows_its_actions(self):
        with self.assertRaises(ValueError):
            self.svc.control("play")
        write_pose_file(self.root / "console" / "demo" / "a.pose.jsonl", n=20)
        self.svc.open("a.pose.jsonl")
        self.svc.control("pause")
        self.svc.control("speed", 4)
        self.assertEqual(self.svc.player.speed, 4.0)
        with self.assertRaises(ValueError):
            self.svc.control("rewind")

    def test_a_model_is_served_by_name_from_the_models_folder_only(self):
        (self.root / "console" / "models" / "ship.glb").write_bytes(b"glTF")
        (self.root / "secret.glb").write_bytes(b"nope")
        body, ctype = self.svc.model_bytes("ship.glb")
        self.assertEqual((body, ctype), (b"glTF", "model/gltf-binary"))
        for bad in ("../secret.glb", "/etc/passwd", "ship.txt", "missing.glb"):
            with self.assertRaises(ValueError):
                self.svc.model_bytes(bad)
        self.assertEqual([m["name"] for m in self.svc.model_files()], ["ship.glb"])

    def test_flying_without_the_tool_says_what_to_build(self):
        self.vehicles.fly = self.root / "no-such-tfc_fly"
        with self.assertRaises(ValueError) as e:
            self.svc.fly("{}", "x")
        self.assertIn("not built", str(e.exception))


class Served(unittest.TestCase):
    """The viewer's routes on a real server, with a real (small) web directory."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        web = self.root / "web"
        (web / "viewer").mkdir(parents=True)
        (web / "index.html").write_text("<title>console</title>")
        (web / "viewer" / "index.html").write_text("<title>viewer</title>")
        (self.root / "console" / "demo").mkdir(parents=True)
        (self.root / "console" / "models").mkdir(parents=True)
        write_pose_file(self.root / "console" / "demo" / "a.pose.jsonl", n=30, step=0.5)
        (self.root / "console" / "models" / "m.glb").write_bytes(b"glTF-bytes")
        self.hub = Hub()
        self.app = App(self.hub, web, "127.0.0.1", 0, TOKEN)
        mgr = SourceManager(self.hub, self.root)
        register_core(self.app, mgr)
        from tfc_console import services
        truth = type("T", (), {"port": 1})()
        pose = type("P", (), {"port": 2})()
        self.svc = services.install(self.app, self.hub, mgr, truth, self.root, rig=False, state_dir=self.root / "state", pose=pose)
        start_in_thread(self.app)
        self.base = f"http://127.0.0.1:{self.app.port}"

    def tearDown(self):
        self.svc["viewer"].close()
        self.app.httpd.shutdown()
        self.app.httpd.server_close()
        self.tmp.cleanup()

    def call(self, path, body=None, token=TOKEN, method=None, follow=True):
        h = {"X-TFC-Token": token} if token else {}
        data = None if body is None else json.dumps(body).encode()
        if data:
            h["Content-Type"] = "application/json"

        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *a, **k):
                return None
        opener = urllib.request.build_opener() if follow else urllib.request.build_opener(NoRedirect)
        try:
            with opener.open(urllib.request.Request(self.base + path, data=data, headers=h, method=method), timeout=5) as r:
                return r.status, r.read(), dict(r.headers)
        except urllib.error.HTTPError as e:
            try:
                return e.code, e.read(), dict(e.headers)
            finally:
                e.close()

    def test_the_viewer_page_is_a_directory_with_its_own_policy(self):
        status, body, h = self.call("/viewer/", token=None)
        self.assertEqual((status, body), (200, b"<title>viewer</title>"))
        self.assertIn("blob:", h["Content-Security-Policy"])                  # a model the user chose is read from a blob: URL
        self.assertNotIn("unsafe-inline'; img", h["Content-Security-Policy"])
        self.assertNotIn("script-src", h["Content-Security-Policy"])          # no inline script: scripts are 'self' only, by default-src
        _s, _b, h2 = self.call("/", token=None)
        self.assertNotIn("blob:", h2["Content-Security-Policy"])               # the console's own page keeps the stricter one

    def test_without_the_slash_it_is_sent_to_the_page_keeping_the_token(self):
        status, _b, h = self.call("/viewer?token=abc&x=1", token=None, follow=False)
        self.assertEqual(status, 301)
        self.assertEqual(h["Location"], "/viewer/?token=abc&x=1")

    def test_the_pose_stream_needs_the_token_and_starts_with_what_there_is(self):
        self.assertEqual(self.call("/api/pose-stream", token=None)[0], 401)
        self.hub.viewer.ingest(spec_line("served"))
        self.hub.viewer.ingest(pose_line(4.0))
        req = urllib.request.Request(self.base + "/api/pose-stream?token=" + TOKEN)
        with urllib.request.urlopen(req, timeout=5) as r:
            self.assertIn("text/event-stream", r.headers["Content-Type"])
            head = b""
            while b"\n\n" not in head:
                head += r.read(1)
            text = head.decode()
        self.assertTrue(text.startswith("event: hello"))
        hello = json.loads(text.split("data: ", 1)[1])
        self.assertEqual((hello["spec"]["name"], hello["pose"]["tt"]), ("served", 4.0))

    def test_poses_that_come_after_are_streamed_one_by_one(self):
        req = urllib.request.Request(self.base + "/api/pose-stream?token=" + TOKEN)
        with urllib.request.urlopen(req, timeout=5) as r:
            def next_event():
                buf = b""
                while not buf.endswith(b"\n\n"):
                    buf += r.read(1)
                return buf.decode()
            self.assertTrue(next_event().startswith("event: hello"))
            for i in range(3):
                self.hub.ingest_pose(pose_line(float(i)))
                ev = next_event()
                self.assertTrue(ev.startswith("event: pose"))
                self.assertEqual(json.loads(ev.split("data: ", 1)[1])["tt"], float(i))

    def test_state_lists_what_can_be_opened_and_where_the_simulator_sends(self):
        status, body, _ = self.call("/api/viewer/state")
        d = json.loads(body)
        self.assertEqual(status, 200)
        self.assertEqual(d["pose_port"], 2)
        self.assertEqual([f["name"] for f in d["pose_files"]], ["a.pose.jsonl"])
        self.assertEqual([m["name"] for m in d["models"]], ["m.glb"])
        self.assertIn("reference", [v["name"] for v in d["vehicles"]])

    def test_open_and_the_transport_through_the_api(self):
        self.assertEqual(self.call("/api/viewer/open", {"name": "nope"})[0], 400)
        self.assertEqual(self.call("/api/viewer/control", {"action": "play"})[0], 400)        # nothing is open
        self.assertEqual(self.call("/api/viewer/open", {"name": "a.pose.jsonl", "autoplay": False})[0], 200)
        status, body, _ = self.call("/api/viewer/control", {"action": "speed", "value": 2})
        self.assertEqual(status, 200)
        self.call("/api/viewer/control", {"action": "seek", "value": 6.0})
        time.sleep(0.4)
        src = json.loads(self.call("/api/viewer/state")[1])["source"]
        self.assertEqual((src["kind"], src["speed"], src["playing"]), ("pose-file", 2.0, False))
        self.assertAlmostEqual(src["pos"], 6.0, 1)
        self.assertEqual(self.call("/api/viewer/close", {})[0], 200)
        self.assertEqual(self.hub.viewer.source["kind"], "none")

    def test_the_commands_of_the_viewer_need_the_token(self):
        for path in ("/api/viewer/open", "/api/viewer/control", "/api/viewer/close", "/api/viewer/fly", "/api/viewer/log"):
            self.assertEqual(self.call(path, {}, token=None)[0], 401, path)
        self.assertEqual(self.call("/api/viewer/state", token=None)[0], 401)
        self.assertEqual(self.call("/api/viewer/model?name=m.glb", token=None)[0], 401)

    def test_a_model_comes_back_as_bytes_with_its_type_and_only_by_name(self):
        status, body, h = self.call("/api/viewer/model?name=m.glb")
        self.assertEqual((status, body, h["Content-Type"]), (200, b"glTF-bytes", "model/gltf-binary"))
        self.assertEqual(self.call("/api/viewer/model?name=../../etc/passwd")[0], 404)
        self.assertEqual(self.call("/api/viewer/model?name=missing.glb")[0], 404)

    def test_the_pages_log_line_goes_to_the_servers_standard_error(self):
        import contextlib
        import io
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            self.assertEqual(self.call("/api/viewer/log", {"text": "dbg hello"})[0], 200)
        self.assertIn("viewer: dbg hello", err.getvalue())


class RecordedFlightTests(unittest.TestCase):
    """A bus recording whose sidecar carries poses: the replay plays them with the rest, and a seek shows the viewer only where it ended."""

    def make(self, d):
        d = Path(d)
        (d / "x.log").write_text("(0.000000) vcan0 7FF#0000000000000000\n(10.000000) vcan0 7FF#0000000000000000\n")
        side = [{"t": 0.0, "k": "spec", "d": json.loads(spec_line())}] + [{"t": i * 0.5, "k": "pose", "d": json.loads(pose_line(i * 0.5))} for i in range(20)]
        (d / "x.side.jsonl").write_text("\n".join(json.dumps(x) for x in side) + "\n")
        return d / "x.log"

    def test_the_replay_feeds_the_viewer_and_a_seek_is_shown_once(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub()
            src = LogSource(hub, self.make(d))
            q = hub.viewer.subscribe()
            src._feed_until(3.2)
            ks = kinds(drain(q))
            self.assertEqual(ks.count("pose"), 7)                                 # 0, 0.5 ... 3.0
            self.assertEqual(ks.count("spec"), 1)
            src._rewind_to(6.2)
            ks = kinds(drain(q))
            self.assertEqual(ks.count("pose"), 1)                                 # only where the seek ended
            self.assertEqual(ks[0], "reset")
            self.assertEqual(json.loads(hub.viewer.hello())["pose"]["tt"], 6.0)
            self.assertFalse(hub.viewer.muted)


class TheRealSimulator(unittest.TestCase):
    """`tfc_fly --pose` and the reader: the file the simulator writes is the file the viewer plays."""

    @unittest.skipUnless(FLY.exists(), "build/host/tfc_fly is not built")
    def test_a_flight_of_the_starship_class_vehicle_is_written_and_played(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "s.pose.jsonl"
            r = subprocess.run([str(FLY), str(ROOT / "vehicles" / "starship.json"), "--sensors", "vehicle", "--no-accel", "--pad", "20", "--frames", "400", "--pose", str(out), "--pose-hz", "25"], capture_output=True, text=True, timeout=120)
            self.assertIn("wrote the poses", r.stdout, r.stdout + r.stderr)
            lines = out.read_text().splitlines()
            spec = json.loads(lines[0])
            self.assertEqual((spec["k"], spec["name"]), ("spec", "starship-class"))
            self.assertEqual(len(spec["vehicle"]["engines"]), 39)                  # 33 + 6: the rings are expanded, an index in a pose is an index in this list
            self.assertEqual(len(spec["vehicle"]["stages"]), 2)
            self.assertGreater(len(spec["air"]["density"]), 100)
            poses = [json.loads(x) for x in lines[1:]]
            self.assertLess(poses[0]["t"], 0.0)                                    # the pad frames come first, with a negative time
            self.assertEqual(poses[0]["clamp"], 1)
            self.assertTrue(all(len(p["eng"]) == 39 for p in poses))
            self.assertTrue(all(abs(sum(x * x for x in p["q"]) - 1.0) < 1e-5 for p in poses))
            self.assertGreater(poses[-1]["alt"], 5.0)                             # it lifted off (four seconds of flight)
            self.assertEqual(sorted(p["tt"] for p in poses), [p["tt"] for p in poses])
            r = PoseReplay(Hub(), out, autoplay=False)
            self.assertEqual(len(r.poses), len(poses))
            self.assertLess(r.t_start, 0.0)


class FlyingAVehicle(unittest.TestCase):
    """The viewer's "fly a vehicle": the real flight software on a vehicle file, the poses written, and the file played."""

    @unittest.skipUnless(FLY.exists(), "build/host/tfc_fly is not built")
    def test_a_vehicle_is_flown_with_the_real_flight_software_and_the_flight_is_played(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub()
            vehicles = Vehicles(ROOT, Path(d) / "state")
            svc = ViewerService(hub, ROOT, Path(d) / "state", vehicles)
            try:
                text = vehicles.read("sounding_rocket")["text"]
                res = svc.fly(text, "sounding rocket/../x", "platform", 0, 25)               # a name with a path in it is made harmless
                self.assertTrue(res["ok"], res["output"])
                self.assertIn("verdict: flown", res["output"])
                self.assertNotIn("/", res["file"])
                self.assertTrue(res["file"].endswith(".pose.jsonl"))
                out = Path(d) / "state" / "viewer" / res["file"]
                self.assertTrue(out.is_file() and res["bytes"] == out.stat().st_size)
                self.assertTrue(any(f["name"] == res["file"] and f["where"] == "flown" for f in svc.pose_files()))
                svc.open(res["file"], speed=32.0)
                time.sleep(0.5)
                src = hub.viewer.source
                self.assertEqual((src["kind"], src["file"]), ("pose-file", res["file"]))
                self.assertGreater(hub.viewer.count, 0)
                spec = json.loads(hub.viewer.spec_raw)
                self.assertEqual(spec["name"], "sounding rocket")
                self.assertEqual(len(spec["vehicle"]["stages"]), 1)
            finally:
                svc.close()

    @unittest.skipUnless(FLY.exists(), "build/host/tfc_fly is not built")
    def test_a_vehicle_the_reader_refuses_is_reported_with_its_reasons_and_nothing_is_played(self):
        with tempfile.TemporaryDirectory() as d:
            hub = Hub()
            vehicles = Vehicles(ROOT, Path(d) / "state")
            svc = ViewerService(hub, ROOT, Path(d) / "state", vehicles)
            with self.assertRaises(ValueError) as e:
                svc.fly('{"name": "broken", "stages": [{"dry_mass": 1}]}', "broken", "vehicle", 0)
            self.assertIn("wrote no poses", str(e.exception))
            self.assertIn("dry_mass", str(e.exception))                                    # the reader's own message, with the nearest name it suggests
            self.assertIsNone(svc.player)
            self.assertEqual(hub.viewer.source["kind"], "none")


class ThePageFiles(unittest.TestCase):
    """There is no JavaScript engine on the bench to load the page with, so what can be checked without one is: every import resolves to a file, and the library is the one the README says it is."""

    IMPORT = re.compile(r"""(?:import|export)\s[^;]*?from\s+['"]([^'"]+)['"]|import\(\s*['"]([^'"]+)['"]\s*\)|import\s+['"]([^'"]+)['"]""", re.S)

    def test_every_import_of_every_module_of_the_page_resolves(self):
        mods = list((WEB / "viewer" / "js").rglob("*.js")) + list((WEB / "js").rglob("*.js"))
        self.assertGreater(len(mods), 20)
        for m in mods:
            text = m.read_text()
            for a, b, c in self.IMPORT.findall(text):
                spec = a or b or c
                if not spec.startswith("."):
                    self.fail(f"{m.relative_to(WEB)}: imports {spec!r}, which a browser cannot resolve without an import map")
                self.assertTrue((m.parent / spec).resolve().is_file(), f"{m.relative_to(WEB)}: {spec} is not a file")

    def test_the_html_pages_name_files_that_exist(self):
        for page in (WEB / "index.html", WEB / "viewer" / "index.html"):
            text = page.read_text()
            for ref in re.findall(r'(?:src|href)="([^"#?]+)"', text):
                if ref.startswith(("data:", "http")):
                    continue
                target = (WEB / ref.lstrip("/")) if ref.startswith("/") else (page.parent / ref)           # the server's root is console/web
                if target.is_dir():
                    target = target / "index.html"
                self.assertTrue(target.resolve().is_file(), f"{page.name}: {ref}")

    def test_the_vendored_library_is_the_release_the_readme_names(self):
        readme = (WEB / "viewer" / "vendor" / "README.md").read_text()
        want = dict(re.findall(r"^([0-9a-f]{64})  (\S+)$", readme, re.M))
        self.assertEqual(len(want), 3)
        for digest, name in want.items():
            path = WEB / "viewer" / "vendor" / name
            text = path.read_text()
            if "from '../../three.module.js'" in text:                              # the one line that is changed in the add-ons: undo it to compare with what was downloaded
                text = text.replace("from '../../three.module.js'", "from 'three'")
            self.assertEqual(hashlib.sha256(text.encode()).hexdigest(), digest, name)

    def test_no_page_script_is_inline(self):
        """The content-security policy has no 'unsafe-inline' for scripts: an inline one would not run."""
        for page in (WEB / "index.html", WEB / "viewer" / "index.html"):
            self.assertIsNone(re.search(r"<script(?![^>]*\bsrc=)[^>]*>\s*\S", page.read_text()), page.name)


if __name__ == "__main__":
    unittest.main()
