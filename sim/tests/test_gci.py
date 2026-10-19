# SPDX-License-Identifier: MIT
"""The grid-convergence index (tools/cfd/gci.py) against sequences whose answer is known."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("gci", ROOT / "tools" / "cfd" / "gci.py")
gci = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gci)


class GridConvergence(unittest.TestCase):
    def sequence(self, f0, c, p, h=(0.35, 0.7, 1.4)):
        return [f0 + c * x ** p for x in h]

    def test_a_second_order_sequence_gives_its_order_its_limit_and_an_index(self):
        f1, f2, f3 = self.sequence(1.0, 0.08, 2.0)
        g = gci.gci(f1, f2, f3, 2.0)
        self.assertTrue(g["asymptotic"])
        self.assertAlmostEqual(g["p"], 2.0, places=9)
        self.assertAlmostEqual(g["f_inf"], 1.0, places=9)
        # the error of the fine grid is c h^2 = 0.0098; the index is Fs times the relative change over (r^p - 1)
        self.assertAlmostEqual(g["gci"], 1.25 * abs((f2 - f1) / f1) / 3.0, places=12)

    def test_a_first_order_sequence_and_a_decreasing_one(self):
        f1, f2, f3 = self.sequence(5.0, -0.3, 1.0)
        g = gci.gci(f1, f2, f3, 2.0)
        self.assertAlmostEqual(g["p"], 1.0, places=9)
        self.assertAlmostEqual(g["f_inf"], 5.0, places=9)

    def test_results_that_oscillate_or_do_not_settle_are_not_in_the_asymptotic_range(self):
        self.assertFalse(gci.gci(1.0, 1.1, 1.0, 2.0)["asymptotic"])           # the differences change sign
        self.assertFalse(gci.gci(1.0, 1.1, 1.15, 2.0)["asymptotic"])          # the coarser step is smaller than the finer one: diverging
        self.assertFalse(gci.gci(1.0, 1.0, 1.2, 2.0)["asymptotic"])           # no change at all


if __name__ == "__main__":
    unittest.main()
