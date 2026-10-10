# SPDX-License-Identifier: MIT
"""The Earth imagery pack of the 3D viewer: the requests it makes (latitude first for WMS 1.3.0), what a pack holds, that a failed or empty answer is not kept as a picture, that only the files its manifest names
are served, and the viewer's routes. The network is a stand-in: nothing here asks NASA for anything."""
import json
import tempfile
import unittest
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from unittest import mock

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console import imagery
from tfc_console.app import SourceManager, register_core
from tfc_console.hub import Hub
from tfc_console.server import App, start_in_thread

JPEG = b"\xff\xd8\xff\xe0" + b"x" * 3000


class FakeResponse:
    def __init__(self, body, ctype="image/jpeg"):
        self.body, self.headers = body, {"Content-Type": ctype}

    def read(self):
        return self.body

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class Requests(unittest.TestCase):
    def test_a_box_is_sent_latitude_first_as_wms_1_3_0_with_epsg_4326_wants(self):
        url = imagery.wms_url("BlueMarble_NextGeneration", (-180, -90, 180, 90), 4096, 2048)
        q = urllib.parse.parse_qs(urllib.parse.urlparse(url).query)
        self.assertEqual(q["BBOX"], ["-90.000000,-180.000000,90.000000,180.000000"])
        self.assertEqual((q["VERSION"], q["CRS"], q["WIDTH"], q["HEIGHT"]), (["1.3.0"], ["EPSG:4326"], ["4096"], ["2048"]))
        self.assertNotIn("TIME", q)
        box = imagery.site_box("ksc", 0.5)
        q = urllib.parse.parse_qs(urllib.parse.urlparse(imagery.wms_url("L", box, 8, 8, "2000-12-01")).query)
        self.assertEqual(q["BBOX"], ["28.108300,-81.104100,29.108300,-80.104100"])           # latitude 28.6, longitude -80.6, a degree square
        self.assertEqual(q["TIME"], ["2000-12-01"])

    def test_the_pack_has_the_globe_three_ways_and_two_patches_for_every_site_in_the_united_states_only(self):
        ids = [p["id"] for p in imagery.layer_plan(["ksc", "kourou"])]
        self.assertEqual(ids, ["earth-4k", "earth-8k", "night-4k", "site-ksc-regional", "site-ksc-local"])      # Kourou has no Landsat WELD: the globe alone
        with self.assertRaises(ValueError):
            imagery.layer_plan(["nowhere"])
        local = [p for p in imagery.layer_plan(["ksc"]) if p["id"].endswith("local")][0]
        self.assertAlmostEqual(local["texel_km"], 111.0 / 4096, places=6)                  # 27 m a pixel: the sensor's own 30 m

    def test_the_nearest_site_by_latitude(self):
        self.assertEqual(imagery.site_for_latitude(28.5), "ksc")
        self.assertEqual(imagery.site_for_latitude(26.0), "starbase")
        self.assertEqual(imagery.site_for_latitude(5.0), "kourou")


class Fetching(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dest = Path(self.tmp.name) / "imagery"
        self.log = []

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_fetch_writes_the_files_and_a_manifest_that_says_where_each_is_and_where_it_came_from(self):
        with mock.patch.object(urllib.request, "urlopen", lambda req, timeout=0: FakeResponse(JPEG)):
            m = imagery.fetch(self.dest, ["ksc"], log=self.log.append)
        self.assertEqual(len(m["layers"]), 5)
        by = {l["id"]: l for l in m["layers"]}
        self.assertEqual(by["site-ksc-local"]["box"], [-81.1041, 28.1083, -80.1041, 29.1083])
        self.assertIn("2000-12-01", by["site-ksc-local"]["source"])
        self.assertIn("public domain", by["earth-4k"]["attribution"])
        self.assertTrue((self.dest / "earth_8k.jpg").is_file())
        self.assertEqual(json.loads((self.dest / "manifest.json").read_text())["sites"]["ksc"]["weld"], True)

    def test_what_is_already_there_is_not_fetched_again(self):
        calls = []
        def urlopen(req, timeout=0):
            calls.append(req.full_url)
            return FakeResponse(JPEG)
        with mock.patch.object(urllib.request, "urlopen", urlopen):
            imagery.fetch(self.dest, ["ksc"], log=self.log.append)
            first = len(calls)
            imagery.fetch(self.dest, ["ksc"], log=self.log.append)
        self.assertEqual((first, len(calls)), (5, 5))

    def test_a_service_that_answers_with_an_error_page_or_nothing_is_not_kept_as_a_picture(self):
        with mock.patch.object(urllib.request, "urlopen", lambda req, timeout=0: FakeResponse(b"<ServiceException>no</ServiceException>", "text/xml")):
            with self.assertRaises(RuntimeError) as cm:
                imagery.fetch(self.dest, ["ksc"], log=self.log.append)
        self.assertIn("text/xml", str(cm.exception))
        self.assertFalse(list(self.dest.glob("*.jpg")))
        def down(req, timeout=0):
            raise urllib.error.URLError("no route")
        with mock.patch.object(urllib.request, "urlopen", down):
            with self.assertRaises(RuntimeError):
                imagery.fetch(self.dest, ["ksc"], log=self.log.append)

    def test_the_pack_serves_only_what_its_manifest_names(self):
        with mock.patch.object(urllib.request, "urlopen", lambda req, timeout=0: FakeResponse(JPEG)):
            imagery.fetch(self.dest, ["ksc"], log=self.log.append, only={"earth-4k"})
        (self.dest / "stray.jpg").write_bytes(JPEG)
        (Path(self.tmp.name) / "secret.txt").write_text("no")
        pack = imagery.ImageryPack(self.dest)
        self.assertEqual([l["id"] for l in pack.manifest()["layers"]], ["earth-4k"])
        self.assertEqual(pack.file_bytes("earth_4k.jpg")[1], "image/jpeg")
        for bad in ("stray.jpg", "../secret.txt", "manifest.json", ""):
            with self.assertRaises(ValueError):
                pack.file_bytes(bad)

    def test_no_pack_is_a_manifest_that_says_so_and_a_file_that_was_deleted_is_dropped(self):
        pack = imagery.ImageryPack(self.dest)
        m = pack.manifest()
        self.assertEqual((m["present"], m["layers"]), (False, []))
        self.assertIn("ksc", m["sites"])
        with mock.patch.object(urllib.request, "urlopen", lambda req, timeout=0: FakeResponse(JPEG)):
            imagery.fetch(self.dest, ["ksc"], log=self.log.append, only={"earth-4k", "night-4k"})
        (self.dest / "night_4k.jpg").unlink()
        self.assertEqual([l["id"] for l in pack.manifest()["layers"]], ["earth-4k"])


class Routes(unittest.TestCase):
    def test_the_viewer_serves_the_manifest_and_its_files_behind_the_token(self):
        from tfc_console import services
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "web" / "viewer").mkdir(parents=True)
            (root / "web" / "index.html").write_text("<title>c</title>")
            (root / "console" / "imagery").mkdir(parents=True)
            (root / "console" / "imagery" / "earth_4k.jpg").write_bytes(JPEG)
            (root / "console" / "imagery" / "manifest.json").write_text(json.dumps({"version": 1, "layers": [{"id": "earth-4k", "file": "earth_4k.jpg"}], "sites": {}}))
            hub = Hub()
            app = App(hub, root / "web", "127.0.0.1", 0, "tok")
            mgr = SourceManager(hub, root)
            register_core(app, mgr)
            svc = services.install(app, hub, mgr, type("T", (), {"port": 1})(), root, rig=False, state_dir=root / "st", pose=type("P", (), {"port": 2})())
            start_in_thread(app)
            try:
                def get(path, token="tok"):
                    req = urllib.request.Request(f"http://127.0.0.1:{app.port}{path}", headers={"X-TFC-Token": token})
                    try:
                        with urllib.request.urlopen(req, timeout=5) as r:
                            return r.status, r.read()
                    except urllib.error.HTTPError as e:
                        try:
                            return e.code, e.read()
                        finally:
                            e.close()
                self.assertEqual(get("/api/viewer/imagery", "wrong")[0], 401)
                st, body = get("/api/viewer/imagery")
                self.assertEqual((st, json.loads(body)["present"]), (200, True))
                self.assertEqual(get("/api/viewer/imagery/file?name=earth_4k.jpg"), (200, JPEG))
                self.assertEqual(get("/api/viewer/imagery/file?name=manifest.json")[0], 404)
                self.assertEqual(get("/api/viewer/imagery/file?name=..%2Fweb%2Findex.html")[0], 404)
            finally:
                svc["viewer"].close()
                app.httpd.shutdown()
                app.httpd.server_close()
