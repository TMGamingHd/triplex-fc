# SPDX-License-Identifier: MIT
"""The flight computers built for the vehicle the rig flies: what is built, what is cached, what is refused, and what the rig is then given. The simulator's reader, the table generator and `west` are
stand-ins (shell scripts that leave the files the real ones leave and print what they print); the orchestration around them is the real one."""
import json
import stat
import tempfile
import time
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.hub import Hub
from tfc_console.rig import Rig
from tfc_console.rigbuild import RigBuilder
from tfc_console.vehicles import Vehicles


def script(path: Path, body: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/bin/sh\n" + body)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)
    return path


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self.tmp.name) / "repo"
        t = self.repo
        # the firmware, a source file the hash is taken over; the Zephyr environment (west on the path, as firmware/env.sh does)
        (t / "firmware" / "app" / "src").mkdir(parents=True)
        (t / "firmware" / "app" / "src" / "main.cpp").write_text("int main() { return 0; }\n")
        (t / "core" / "include").mkdir(parents=True)
        (t / "firmware" / "common").mkdir(parents=True)
        (t / "firmware" / "env.sh").write_text(f'export PATH="{t}/bin:$PATH"\n')
        self.west_log = t / "west.log"
        script(t / "bin" / "west", f"""echo "$@" >> {self.west_log}
d=""; prev=""; for a in "$@"; do [ "$prev" = "-d" ] && d="$a"; prev="$a"; done
[ -n "$FAIL_WEST" ] && {{ echo "CMake Error: nothing works" ; exit 1; }}
mkdir -p "$d/zephyr" && printf '#!/bin/sh\\nexit 0\\n' > "$d/zephyr/zephyr.exe" && chmod +x "$d/zephyr/zephyr.exe"
echo "-- Configuring done"
""")
        venv = Path(self.tmp.name) / "venv"
        script(venv / "bin" / "west", "exit 0\n")
        self.fly = script(t / "build" / "host" / "tfc_fly", 'grep -q BAD "$1" && { echo "line 3, column 4: a field it does not know"; exit 2; }; exit 0\n')
        self.gen = script(t / "build" / "host" / "tfc_gen_tables", 'echo "// tables for $2" > "$3"\n')
        script(t / "build" / "act_native" / "zephyr" / "zephyr.exe", "exit 0\n")
        import os
        self._env = {k: os.environ.get(k) for k in ("TFC_VENV", "TFC_GEN_TABLES_BIN", "FAIL_WEST")}
        os.environ["TFC_VENV"], os.environ["TFC_GEN_TABLES_BIN"] = str(venv), str(self.gen)
        os.environ.pop("FAIL_WEST", None)
        self.hub = Hub()
        self.veh = Vehicles(ROOT, Path(self.tmp.name) / "state")        # the committed vehicle files
        self.veh.fly = self.fly
        self.b = RigBuilder(self.hub, self.repo, self.veh)

    def tearDown(self):
        import os
        for k, v in self._env.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        self.tmp.cleanup()

    def wait(self, name="starship", knobs=None, until=("ready", "failed"), limit=30.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < limit:
            st = self.b.status(name, knobs or {})
            if st["state"] in until:
                return st
            time.sleep(0.05)
        self.fail(f"still {st['state']} after {limit} s")


class TheBuild(Fixture):
    def test_the_reference_vehicle_needs_nothing_built_and_a_design_knob_makes_it_another_vehicle(self):
        self.assertTrue(self.b.reference_untouched("reference", {}))
        self.assertTrue(self.b.reference_untouched("reference", {"vehicle.thrust_scale": 1.1}))            # a plant knob: the computers do not know it
        self.assertFalse(self.b.reference_untouched("reference", {"design.gains.wn": 3.0}))
        self.assertFalse(self.b.reference_untouched("starship", {}))

    def test_the_tables_are_made_from_the_design_knobs_alone(self):
        design = json.loads(self.b.design_text("reference", {"vehicle.thrust_scale": 0.8, "design.gains.wn": 2.5}))
        self.assertEqual(design["design"]["gains"]["wn"], 2.5)
        self.assertNotEqual(design.get("vehicle", {}).get("thrust_scale"), 0.8)

    def test_an_unbuilt_vehicle_is_built_in_steps_and_then_ready_with_three_images(self):
        self.assertEqual(self.b.status("starship", {})["state"], "needs-build")
        self.assertIsNone(self.b.images("starship", {}))
        self.b.prepare("starship", {})
        st = self.wait()
        self.assertEqual(st["state"], "ready", st)
        self.assertEqual([s["state"] for s in st["steps"]], ["done"] * 5)
        imgs = self.b.images("starship", {})
        self.assertEqual(len(imgs), 3)
        self.assertTrue(all(p.is_file() for p in imgs))
        self.assertTrue((self.b.dir_of("starship", {}) / "flight_tables.hpp").read_text().startswith("// tables for"))
        calls = self.west_log.read_text().splitlines()
        self.assertEqual(len(calls), 3)
        for n, c in enumerate(calls):
            self.assertIn(f"-DCONFIG_TFC_NODE_ID={n}", c)
            self.assertIn("-DCONFIG_TFC_ESTIMATOR_NO_ACCEL=y", c)             # a vehicle under thrust: the attitude does not use the accelerometer
            self.assertIn("-DTFC_TABLES_HEADER=" + str(self.b.dir_of("starship", {}) / "flight_tables.hpp"), c)

    def test_a_built_vehicle_is_not_built_twice_and_a_changed_firmware_is_noticed(self):
        self.b.prepare("starship", {})
        self.wait()
        before = self.west_log.read_text()
        self.b.prepare("starship", {})
        self.assertEqual(self.west_log.read_text(), before)
        (self.repo / "firmware" / "app" / "src" / "main.cpp").write_text("int main() { return 1; }\n")
        time.sleep(0.01)
        self.b._status_cache = None
        self.assertEqual(self.b.status("starship", {})["state"], "needs-build")           # other firmware: other key, other directory

    def test_a_design_knob_is_another_build_and_a_plant_knob_is_not(self):
        k1 = self.b.key("starship", {})
        self.assertEqual(self.b.key("starship", {"vehicle.thrust_scale": 0.9}), k1)
        self.assertNotEqual(self.b.key("starship", {"design.gains.wn": 3.0}), k1)

    def test_a_vehicle_the_simulator_refuses_is_refused_with_its_message_and_nothing_is_built(self):
        bad = self.veh.scratch
        bad.mkdir(parents=True)
        (bad / "broken.json").write_text('{"stages": [], "BAD": 1}')
        self.b.prepare("broken", {})
        st = self.wait("broken")
        self.assertEqual(st["state"], "failed")
        self.assertIn("line 3, column 4", st["error"])
        self.assertFalse(self.west_log.exists())

    def test_a_failed_build_says_so_with_the_tail_of_the_log_and_can_be_tried_again(self):
        import os
        os.environ["FAIL_WEST"] = "1"
        self.b.prepare("starship", {})
        st = self.wait()
        self.assertEqual(st["state"], "failed")
        self.assertIn("flight computer A failed", st["error"])
        self.assertIn("CMake Error", st["error"])
        self.assertIsNone(self.b.images("starship", {}))
        os.environ.pop("FAIL_WEST")
        self.b.forget_failure()
        self.b._status_cache = None
        self.b.prepare("starship", {})
        self.assertEqual(self.wait()["state"], "ready")

    def test_one_build_at_a_time(self):
        import os
        script(self.repo / "bin" / "west", "sleep 1\nexit 1\n")
        self.b.prepare("starship", {})
        with self.assertRaises(ValueError):
            self.b.prepare("two_stage_launcher", {})
        self.wait()

    def test_without_the_zephyr_environment_it_says_what_is_missing_instead_of_failing_later(self):
        import os
        os.environ["TFC_VENV"] = str(Path(self.tmp.name) / "nowhere")
        self.b._status_cache = None
        st = self.b.status("starship", {})
        self.assertEqual(st["state"], "cannot-build")
        self.assertIn("no west", st["error"])
        self.assertEqual(self.b.prepare("starship", {})["state"], "cannot-build")


class TheRigIsGivenTheVehicle(Fixture):
    def make_rig(self):
        for rel in ("build/host/tfc_simd", "build/launch_a/zephyr/zephyr.exe", "build/launch_b/zephyr/zephyr.exe", "build/launch_c/zephyr/zephyr.exe"):
            script(self.repo / rel, "exit 0\n")
        return Rig(self.hub, self.repo, lambda: "vcan0", lambda: 45679, lambda: self.veh.sim_args(), images=lambda: (self.b.images(self.veh.active["name"], self.veh.active["knobs"]), self.veh.active["name"]))

    def test_the_closed_loop_flies_the_reference_with_its_committed_images(self):
        p = self.make_rig().profiles()["closed-loop"]
        self.assertTrue(p["ready"], p["missing"])
        self.assertEqual(p["procs"][1].argv[0], str(self.repo / "build/launch_a/zephyr/zephyr.exe"))
        self.assertNotIn("--vehicle-true", p["procs"][0].argv)

    def test_another_vehicle_is_not_ready_until_its_flight_computers_are_built_and_then_it_is_given_its_own(self):
        rig = self.make_rig()
        self.veh.set_active("starship", {})
        p = rig.profiles()["closed-loop"]
        self.assertFalse(p["ready"])                                            # never the reference's computers on another vehicle
        self.assertIn("starship", p["doc"])
        self.b.prepare("starship", {})
        self.wait()
        p = rig.profiles()["closed-loop"]
        self.assertTrue(p["ready"], p["missing"])
        d = self.b.dir_of("starship", {})
        self.assertEqual([x.argv[0] for x in p["procs"][1:4]], [str(d / f"launch_{n}" / "zephyr" / "zephyr.exe") for n in "abc"])
        self.assertIn("--vehicle-true", p["procs"][0].argv)
        self.assertIn("--vehicle", p["procs"][0].argv)


class TheRoutes(Fixture):
    """The choice of vehicle as the Launch tab makes it, on a real server with the stand-in tools."""

    def setUp(self):
        super().setUp()
        import urllib.error
        import urllib.request
        from tfc_console import services
        from tfc_console.app import SourceManager, register_core
        from tfc_console.server import App, start_in_thread
        self.u = urllib
        (self.repo / "vehicles").symlink_to(ROOT / "vehicles")
        web = self.repo / "web"
        web.mkdir()
        (web / "index.html").write_text("<title>console</title>")
        self.hub = Hub()
        self.app = App(self.hub, web, "127.0.0.1", 0, "tok")
        mgr = SourceManager(self.hub, self.repo)
        register_core(self.app, mgr)
        self.svc = services.install(self.app, self.hub, mgr, type("T", (), {"port": 1})(), self.repo, rig=True, state_dir=Path(self.tmp.name) / "st")
        start_in_thread(self.app)
        self.base = f"http://127.0.0.1:{self.app.port}"

    def tearDown(self):
        self.app.httpd.shutdown()
        self.app.httpd.server_close()
        super().tearDown()

    def call(self, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = self.u.request.Request(self.base + path, data=data, headers={"X-TFC-Token": "tok", "Content-Type": "application/json"})
        try:
            with self.u.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read())
        except self.u.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read())
            finally:
                e.close()

    def test_choosing_a_vehicle_builds_it_and_the_snapshot_and_the_listing_follow(self):
        status, body = self.call("/api/rig/vehicle")
        self.assertEqual((status, body["active"]["name"]), (200, "reference"))
        self.assertIn("starship", [v["name"] for v in body["vehicles"]])
        self.assertFalse([v for v in body["vehicles"] if v["name"].startswith("_")])           # the console's own files are not vehicles
        status, body = self.call("/api/rig/vehicle", {"name": "starship"})
        self.assertEqual((status, body["active"]["name"], body["build"]["state"]), (200, "starship", "building"))
        st = self.wait()
        self.assertEqual(st["state"], "ready")
        self.assertEqual(self.hub.snapshot()["rigbuild"]["state"], "ready")                    # what the Launch tab reads, ten times a second
        self.assertEqual(self.call("/api/rig/vehicle")[1]["build"]["state"], "ready")

    def test_a_running_rig_keeps_its_vehicle_and_a_bad_choice_is_refused(self):
        self.svc["rig"].procs = {"A": type("P", (), {"state": "running"})()}
        status, body = self.call("/api/rig/vehicle", {"name": "starship"})
        self.assertEqual(status, 409)
        self.assertIn("stop it first", body["error"])
        self.assertEqual(self.svc["vehicles"].active["name"], "reference")
        self.svc["rig"].procs = {}
        self.assertEqual(self.call("/api/rig/vehicle", {"name": "../etc/passwd"})[0], 400)
        self.assertEqual(self.call("/api/rig/vehicle", {"name": "starship", "knobs": {"vehicle.thrust_scale": 99}})[0], 400)
