# SPDX-License-Identifier: MIT
"""The behavioural-compatibility gate (tools/compat/compat_gate.py, ADR-021, TFC-ARCH-006): a release that behaves as the golden one passes, one whose commands move by more than the version tolerance fails,
and a golden release the harness cannot build against is reported as incomparable."""
import importlib.util
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load():
    spec = importlib.util.spec_from_file_location("compat_gate", ROOT / "tools" / "compat" / "compat_gate.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gate = load()
HAVE_GXX = shutil.which("g++") is not None


class CompareTests(unittest.TestCase):
    def test_commands_within_the_tolerance_agree_and_beyond_it_do_not(self):
        a = {"s": [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]}
        ok, lines = gate.compare(a, {"s": [(0.0, 0.0), (1.01, 1.0), (2.0, 1.99)]}, 0.015)
        self.assertTrue(ok and "ok" in lines[0])
        ok, lines = gate.compare(a, {"s": [(0.0, 0.0), (1.0, 1.0), (2.02, 2.0)]}, 0.015)
        self.assertFalse(ok)
        self.assertIn("FAILS", lines[0])
        self.assertIn("at frame 2", lines[0])
        ok, _ = gate.compare(a, {"s": [(0.0, 0.0), (1.015, 1.0), (2.0, 2.0)]}, 0.015)
        self.assertTrue(ok)  # exactly at the limit is inside it

    def test_a_different_length_a_missing_scenario_and_a_not_a_number_fail(self):
        a = {"s": [(0.0, 0.0), (1.0, 1.0)]}
        self.assertFalse(gate.compare(a, {"s": [(0.0, 0.0)]}, 0.5)[0])
        self.assertFalse(gate.compare(a, {}, 0.5)[0])
        self.assertFalse(gate.compare(a, {"s": [(0.0, 0.0), (float("nan"), 1.0)]}, 0.5)[0])


@unittest.skipUnless(HAVE_GXX, "needs g++")
class GateTests(unittest.TestCase):
    def run_gate(self, *args):
        return subprocess.run([sys.executable, str(ROOT / "tools" / "compat" / "compat_gate.py"), *args], capture_output=True, text=True, timeout=300)

    def test_the_current_release_is_compatible_with_itself(self):
        r = self.run_gate("--golden-include", str(ROOT / "core" / "include"))
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("COMPATIBLE", r.stdout)
        self.assertIn("largest command difference 0.0000 degree", r.stdout)

    def test_a_release_whose_controller_slews_slower_is_not_compatible(self):
        with tempfile.TemporaryDirectory() as d:
            inc = Path(d) / "include"
            shutil.copytree(ROOT / "core" / "include", inc)
            ctl = inc / "tfc" / "controller.hpp"
            text = ctl.read_text()
            self.assertIn("float slew_deg_per_frame = 0.6F;", text)
            ctl.write_text(text.replace("float slew_deg_per_frame = 0.6F;", "float slew_deg_per_frame = 0.004F;"))
            r = self.run_gate("--golden-include", str(inc))
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("NOT COMPATIBLE", r.stdout)
        self.assertIn("FAILS", r.stdout)

    def test_a_looser_factor_lets_a_small_difference_through(self):
        with tempfile.TemporaryDirectory() as d:
            inc = Path(d) / "include"
            shutil.copytree(ROOT / "core" / "include", inc)
            ctl = inc / "tfc" / "controller.hpp"
            ctl.write_text(ctl.read_text().replace("float slew_deg_per_frame = 0.6F;", "float slew_deg_per_frame = 0.004F;"))
            r = self.run_gate("--golden-include", str(inc), "--factor", "1000", "--tol-deg", "1")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_a_golden_release_that_does_not_build_against_the_harness_is_incomparable(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.run_gate("--golden-include", d)
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn("does not build against the harness", r.stderr)


if __name__ == "__main__":
    unittest.main()
