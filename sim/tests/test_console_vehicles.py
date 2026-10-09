# SPDX-License-Identifier: MIT
"""The vehicle tab's server side: the file reader's comments, the knobs and their limits, what is validated by the simulator's own reader, and what the rig is given."""
import json
import os
import tempfile
import time
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.vehicles import KNOBS, Jobs, Vehicles, get_path, set_path, strip_comments

FLY = Path(os.environ.get("TFC_FLY_BIN") or ROOT / "build" / "host" / "tfc_fly")


class Comments(unittest.TestCase):
    def test_comments_to_the_end_of_the_line_go_and_slashes_in_strings_stay(self):
        text = '{\n  "a": 1, // one\n  "url": "http://x//y", // two\n  "b": "// not a comment"\n}\n'
        self.assertEqual(json.loads(strip_comments(text)), {"a": 1, "url": "http://x//y", "b": "// not a comment"})

    def test_an_escaped_quote_does_not_end_the_string(self):
        self.assertEqual(json.loads(strip_comments('{"a": "q\\" // still the string"} // gone')), {"a": 'q" // still the string'})

    def test_every_committed_vehicle_file_reads(self):
        for p in sorted((ROOT / "vehicles").glob("*.json")):
            json.loads(strip_comments(p.read_text()))


class Paths(unittest.TestCase):
    def test_get_and_set_by_dotted_path(self):
        d = {"a": {"b": 1}}
        self.assertEqual(get_path(d, "a.b"), 1)
        self.assertIsNone(get_path(d, "a.c"))
        self.assertEqual(get_path(d, "x.y", 7), 7)
        set_path(d, "a.c.d", 5)
        self.assertEqual(d, {"a": {"b": 1, "c": {"d": 5}}})


class Knobs(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.v = Vehicles(ROOT, Path(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def test_a_knob_is_set_in_a_copy_and_the_original_is_untouched(self):
        data = {"vehicle": {"thrust_scale": 1.0}}
        out = self.v.apply_knobs(data, {"vehicle.thrust_scale": 0.9, "scenario.wind_scale": 2})
        self.assertEqual((out["vehicle"]["thrust_scale"], out["scenario"]["wind_scale"]), (0.9, 2.0))
        self.assertEqual(data, {"vehicle": {"thrust_scale": 1.0}})

    def test_a_value_outside_the_knobs_range_and_a_name_that_is_not_a_knob_are_refused(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            self.v.apply_knobs({}, {"vehicle.thrust_scale": 3.0})
        with self.assertRaisesRegex(ValueError, "not a knob"):
            self.v.apply_knobs({}, {"vehicle.name": 1})
        with self.assertRaises(ValueError):
            self.v.apply_knobs({}, {"vehicle.thrust_scale": "fast"})
        self.assertEqual(self.v.apply_knobs({"a": 1}, {"vehicle.thrust_scale": ""}), {"a": 1})        # an empty box is no knob

    def test_every_knob_names_a_field_the_reader_knows(self):
        """A knob that writes a field the simulator does not know would be refused by the reader at flight time; check each against the committed reference vehicle's keys or the documented defaults."""
        text = (ROOT / "docs" / "design" / "VEHICLE_SPEC.md").read_text()
        for path, *_rest in KNOBS:
            for key in path.split(".")[1:]:
                self.assertIn(f"`{key}`", text, f"{path}: {key} is not in VEHICLE_SPEC.md")

    def test_every_knobs_default_is_inside_its_range(self):
        for path, label, unit, step, lo, hi, doc, default in KNOBS:
            self.assertLessEqual(lo, default, path)
            self.assertGreaterEqual(hi, default, path)
            self.assertGreater(step, 0, path)


@unittest.skipUnless(FLY.exists(), "build/host/tfc_fly is not built")
class WithTheSimulatorsOwnReader(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.v = Vehicles(ROOT, Path(self.tmp.name))
        self.assertEqual(self.v.fly, FLY)

    def tearDown(self):
        self.tmp.cleanup()

    def test_the_listing_has_the_reference_once_and_not_the_dispersion_sets(self):
        names = [x["name"] for x in self.v.listing()]
        self.assertEqual(names.count("reference"), 1)
        self.assertIn("two_stage_launcher", names)
        self.assertNotIn("dispersions", names)

    def test_a_valid_vehicle_checks_and_a_wrong_one_names_its_line_and_column(self):
        ok = self.v.check(self.v.read("sounding_rocket")["text"])
        self.assertTrue(ok["ok"])
        self.assertIn("stage", ok["output"])
        bad = self.v.check('{\n  "name": "x",\n  "stages": [{"name": "s", "dry_mass": 5}]\n}\n')
        self.assertFalse(bad["ok"])
        self.assertRegex(bad["output"], r"line \d+, column \d+")
        self.assertIn("did you mean", bad["output"])
        self.assertNotIn(self.tmp.name, bad["output"])                          # the scratch path is not shown

    def test_the_reference_text_is_what_the_simulator_flies(self):
        r = self.v.read("reference")
        self.assertEqual(r["where"], "built in")
        self.assertEqual(r["data"]["name"], "reference")
        self.assertTrue(self.v.check(r["text"])["ok"])

    def test_the_reference_vehicle_with_a_knob_changed_flies_differently_and_the_untouched_one_flies_nominally(self):
        nominal = self.v.preview(self.v.read("reference")["text"])
        self.assertTrue(nominal["ok"])
        c = nominal["columns"]
        self.assertAlmostEqual(c["altitude_m"][-1] / 1000, 31.9, delta=0.3)                  # docs/design/VEHICLE_SIM.md, and tfc_fly
        self.assertGreater(c["range_m"][-1], 12000)
        data = self.v.read("reference")["data"]
        weak = self.v.preview(json.dumps(self.v.apply_knobs(data, {"vehicle.thrust_scale": 0.9})))
        self.assertLess(weak["columns"]["altitude_m"][-1], c["altitude_m"][-1] - 1000)

    def test_the_rig_gets_nothing_for_the_untouched_reference_and_a_scratch_file_otherwise(self):
        self.assertEqual(self.v.sim_args(), [])
        self.v.set_active("reference", {"scenario.wind_scale": 2.0, "vehicle.cd_scale": ""})
        args = self.v.sim_args()
        self.assertEqual(args[0], "--vehicle")
        self.assertTrue(args[1].startswith(self.tmp.name))
        written = json.loads(Path(args[1]).read_text())
        self.assertEqual(written["scenario"]["wind_scale"], 2.0)
        self.assertEqual(self.v.active["knobs"], {"scenario.wind_scale": 2.0})
        self.assertTrue(self.v.check(Path(args[1]).read_text())["ok"])            # what the simulator will be given is valid

    def test_a_bad_knob_is_refused_when_chosen_not_when_the_rig_starts(self):
        with self.assertRaises(ValueError):
            self.v.set_active("reference", {"vehicle.thrust_scale": 9})
        self.assertEqual(self.v.active, {"name": "reference", "knobs": {}})

    def test_saving_never_overwrites_an_example_and_keeps_valid_json(self):
        with self.assertRaisesRegex(ValueError, "examples"):
            self.v.save("sounding_rocket", "{}")
        with self.assertRaises(ValueError):
            self.v.save("../evil", "{}")
        with self.assertRaises(ValueError):
            self.v.save("reference", "{}")
        with self.assertRaisesRegex(ValueError, "not valid JSON"):
            self.v.save("mine", "{ nope")
        r = self.v.save("mine", self.v.read("sounding_rocket")["text"])
        self.assertTrue(Path(r["saved"]).is_file())
        self.assertIn("mine", [x["name"] for x in self.v.listing()])
        self.assertTrue(self.v.check(Path(r["saved"]).read_text())["ok"])

    def test_a_vehicle_name_is_never_a_path(self):
        for bad in ("../x", "a/b", "", "x" * 60):
            with self.assertRaises(ValueError):
                self.v.read(bad)


class TheJobs(unittest.TestCase):
    def test_a_job_runs_in_its_own_thread_and_reports_its_result_or_its_error(self):
        j = Jobs()
        ok = j.start("t", lambda: {"x": 1})
        bad = j.start("t", lambda: 1 / 0)
        for _ in range(100):
            if all(j.get(i)["state"] != "running" for i in (ok, bad)):
                break
            time.sleep(0.02)
        self.assertEqual((j.get(ok)["state"], j.get(ok)["result"]), ("done", {"x": 1}))
        self.assertEqual(j.get(bad)["state"], "failed")
        self.assertIn("ZeroDivisionError", j.get(bad)["error"])
        self.assertIsNone(j.get("nope"))


if __name__ == "__main__":
    unittest.main()
