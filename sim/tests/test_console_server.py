# SPDX-License-Identifier: MIT
"""The web server: who may talk to it, what it serves, and the event stream. Everything here is the local security story of a console that can send commands."""
import json
import socket
import tempfile
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from .console_support import act, hb, ROOT  # first: puts console/ on the path
from tfc_console.app import SourceManager, register_core
from tfc_console.hub import Hub
from tfc_console.server import ApiError, App, Request, start_in_thread

TOKEN = "t" * 20


class Served(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        web = Path(self.tmp.name) / "web"
        web.mkdir()
        (web / "index.html").write_text("<!doctype html><title>x</title>")
        (web / "app.js").write_text("export const x = 1;")
        self.hub = Hub()
        self.hub.status.update(kind="live", iface="vcan0")
        self.app = App(self.hub, web, "127.0.0.1", 0, TOKEN)
        self.mgr = SourceManager(self.hub, Path(self.tmp.name))
        register_core(self.app, self.mgr)

        @self.app.route("POST", "/api/echo")
        def echo(req: Request):
            return 200, {"got": req.body}

        @self.app.route("GET", "/api/boom")
        def boom(req: Request):
            raise RuntimeError("a bug")

        @self.app.route("GET", "/api/refuse")
        def refuse(req: Request):
            raise ApiError(409, "not now")
        self.thread = start_in_thread(self.app)
        self.base = f"http://127.0.0.1:{self.app.port}"

    def tearDown(self):
        self.app.httpd.shutdown()
        self.app.httpd.server_close()
        self.tmp.cleanup()

    def call(self, path, body=None, token=TOKEN, headers=None, method=None):
        h = dict(headers or {})
        if token:
            h["X-TFC-Token"] = token
        data = None
        if body is not None:
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            h.setdefault("Content-Type", "application/json")
        req = urllib.request.Request(self.base + path, data=data, headers=h, method=method)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, r.read(), dict(r.headers)
        except urllib.error.HTTPError as e:
            try:
                return e.code, e.read(), dict(e.headers)
            finally:
                e.close()


class Access(Served):
    def test_static_files_need_no_token_but_carry_the_security_headers(self):
        status, body, h = self.call("/", token=None)
        self.assertEqual(status, 200)
        self.assertIn(b"<title>x</title>", body)
        self.assertIn("default-src 'self'", h["Content-Security-Policy"])
        self.assertEqual(h["Cache-Control"], "no-store")
        self.assertEqual(h["X-Content-Type-Options"], "nosniff")
        status, body, h = self.call("/app.js", token=None)
        self.assertEqual(status, 200)
        self.assertIn("javascript", h["Content-Type"])

    def test_the_api_needs_the_token(self):
        self.assertEqual(self.call("/api/config", token=None)[0], 401)
        self.assertEqual(self.call("/api/config", token="wrong")[0], 401)
        self.assertEqual(self.call("/api/config")[0], 200)
        self.assertEqual(self.call("/api/config?token=" + TOKEN, token=None)[0], 200)        # the event stream cannot send a header

    def test_a_post_needs_the_token_too(self):
        self.assertEqual(self.call("/api/echo", {"a": 1}, token=None)[0], 401)
        status, body, _ = self.call("/api/echo", {"a": 1})
        self.assertEqual((status, json.loads(body)), (200, {"got": {"a": 1}}))

    def test_a_request_with_another_host_is_refused(self):
        """DNS rebinding: a page on another name that resolves to 127.0.0.1 must not be able to read the console."""
        status, _b, _h = self.call("/api/config", headers={"Host": "evil.example:80"})
        self.assertEqual(status, 403)
        status, _b, _h = self.call("/", token=None, headers={"Host": "evil.example"})
        self.assertEqual(status, 403)
        self.assertEqual(self.call("/api/config", headers={"Host": f"localhost:{self.app.port}"})[0], 200)

    def test_a_cross_origin_post_is_refused(self):
        status, body, _ = self.call("/api/echo", {"a": 1}, headers={"Origin": "http://evil.example"})
        self.assertEqual(status, 403)
        self.assertEqual(self.call("/api/echo", {"a": 1}, headers={"Origin": f"http://127.0.0.1:{self.app.port}"})[0], 200)

    def test_the_static_server_does_not_leave_its_directory(self):
        (Path(self.tmp.name) / "secret.txt").write_text("TOPSECRET-CONTENT")
        for path in ("/../secret.txt", "/%2e%2e/secret.txt", "/..%2fsecret.txt"):
            status, body, _ = self.call(path, token=None)
            self.assertIn(status, (403, 404), path)
            self.assertNotIn(b"TOPSECRET", body)
        # a raw request, so that the client does not tidy the path
        s = socket.create_connection(("127.0.0.1", self.app.port))
        s.sendall(b"GET /../secret.txt HTTP/1.1\r\nHost: 127.0.0.1:%d\r\nConnection: close\r\n\r\n" % self.app.port)
        data = s.recv(4096)
        s.close()
        self.assertNotIn(b"TOPSECRET", data)

    def test_only_get_for_static_files_and_bad_bodies_are_refused(self):
        self.assertEqual(self.call("/index.html", {"x": 1}, token=None)[0], 405)
        self.assertEqual(self.call("/api/echo", b"not json")[0], 400)
        self.assertEqual(self.call("/api/echo", b"[1, 2]")[0], 400)
        self.assertEqual(self.call("/api/nothing")[0], 404)

    def test_a_bug_in_one_endpoint_is_a_500_with_its_message_and_the_server_goes_on(self):
        status, body, _ = self.call("/api/boom")
        self.assertEqual(status, 500)
        self.assertIn("a bug", json.loads(body)["error"])
        self.assertEqual(self.call("/api/config")[0], 200)
        status, body, _ = self.call("/api/refuse")
        self.assertEqual((status, json.loads(body)["error"]), (409, "not now"))


class CoreRoutes(Served):
    def test_config_carries_what_the_page_draws_from(self):
        _s, body, _h = self.call("/api/config")
        c = json.loads(body)
        self.assertEqual(len(c["channels"]), 8)
        self.assertEqual(c["phases"][3]["name"], "ascent")
        self.assertGreater(len(c["faults"]), 30)
        self.assertEqual(c["ground_ops"]["launch"], 5)

    def test_frames_endpoint_filters_by_id_and_rejects_nonsense(self):
        self.hub.ingest_frame(1.0, hb(0))
        self.hub.ingest_frame(1.1, act())
        _s, body, _h = self.call("/api/frames?ids=400")
        self.assertEqual([f["id"] for f in json.loads(body)["frames"]], [0x400])
        self.assertEqual(self.call("/api/frames?ids=zz")[0], 400)

    def test_bus_table_and_snapshot(self):
        self.hub.ingest_frame(1.0, hb(0))
        self.hub.tick(1.0)
        _s, body, _h = self.call("/api/bus")
        self.assertEqual(json.loads(body)["rows"][0]["name"], "HEARTBEAT A")
        _s, body, _h = self.call("/api/snapshot")
        self.assertEqual(json.loads(body)["nodes"][0]["name"], "A")

    def test_a_replay_can_be_opened_only_from_the_recordings_directories(self):
        status, body, _ = self.call("/api/source", {"replay": "../../etc/passwd"})
        self.assertEqual(status, 404)
        status, body, _ = self.call("/api/source", {"iface": "../x"})
        self.assertEqual(status, 400)
        status, body, _ = self.call("/api/source", {})
        self.assertEqual(status, 400)
        status, body, _ = self.call("/api/replay", {"play": True})
        self.assertEqual(status, 409)                      # no replay is open

    def test_a_missing_interface_is_a_409_that_says_how_to_make_it(self):
        status, body, _ = self.call("/api/source", {"iface": "vcan9"})
        self.assertEqual(status, 409)
        self.assertIn("setup_vcan", json.loads(body)["error"])


class TheStream(Served):
    def test_the_stream_says_hello_with_the_history_and_then_state_every_tenth_of_a_second(self):
        self.hub.ingest_frame(1.0, hb(0))
        self.hub.tick(1.0)
        self.hub.start()
        req = urllib.request.Request(self.base + "/api/stream?token=" + TOKEN)
        got = []
        with urllib.request.urlopen(req, timeout=5) as r:
            self.assertIn("text/event-stream", r.headers["Content-Type"])
            event = None
            for raw in r:
                line = raw.decode().rstrip("\n")
                if line.startswith("event:"):
                    event = line[7:]
                elif line.startswith("data:"):
                    got.append((event, json.loads(line[6:])))
                    if sum(1 for e, _ in got if e == "state") >= 2:
                        break
        self.assertEqual(got[0][0], "hello")
        self.assertIn("history", got[0][1])
        self.assertEqual(got[0][1]["snapshot"]["nodes"][0]["name"], "A")
        self.assertGreaterEqual(sum(1 for e, _ in got if e == "state"), 2)
        from tfc_console.server import wait_for
        self.assertTrue(wait_for(lambda: not self.hub.subscribers, 3.0), "the page went away: its queue must go with it")
        self.hub.stop()

    def test_the_stream_needs_the_token(self):
        req = urllib.request.Request(self.base + "/api/stream")
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(cm.exception.code, 401)


class TheRealPage(unittest.TestCase):
    """The shipped page: the files the server serves exist, and the page loads only its own scripts (the CSP allows nothing else)."""

    def test_the_web_directory_is_complete_and_self_contained(self):
        web = ROOT / "console" / "web"
        html = (web / "index.html").read_text()
        for needle in ("js/boot.js", "js/app.js", "style.css"):
            self.assertIn(needle, html)
        self.assertNotRegex(html, r"https?://(?!www\.w3\.org)")           # nothing is fetched from the network
        for p in (web / "js").rglob("*.js"):
            text = p.read_text()
            self.assertNotRegex(text, r"""from ['"]https?://""", p)
            self.assertNotIn("eval(", text, p)
            self.assertNotIn("new Function(", text, p)
        for t in ("mission", "launch", "voting", "flight", "commands", "faults", "vehicle", "rig", "bus", "events"):
            self.assertTrue((web / "js" / "tabs" / f"{t}.js").exists(), t)
            self.assertIn(f"import {t} from './tabs/{t}.js'", (web / "js" / "app.js").read_text())


if __name__ == "__main__":
    unittest.main()
