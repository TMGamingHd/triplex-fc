# SPDX-License-Identifier: MIT
"""The metrics tool (tools/docs/metrics.py): the documents quote the numbers the repository computes, and a stale or unknown number is refused."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "docs"))
import metrics as M  # noqa: E402


class Markers(unittest.TestCase):
    def test_a_marker_is_rewritten_with_the_current_value(self):
        problems = []
        text = "we run <!--m:cpp_tests-->1<!--/m--> tests and <!--m:cpp_tests:raw-->2<!--/m-->."
        out = M.render(text, {"cpp_tests": 1234}, problems, "x.md")
        self.assertEqual(out, "we run <!--m:cpp_tests-->1,234<!--/m--> tests and <!--m:cpp_tests:raw-->1234<!--/m-->.")
        self.assertEqual(problems, [])
        self.assertEqual(M.render(out, {"cpp_tests": 1234}, problems, "x.md"), out)   # idempotent

    def test_an_unknown_metric_is_a_problem_and_is_left_alone(self):
        problems = []
        text = "<!--m:no_such_thing-->7<!--/m-->"
        self.assertEqual(M.render(text, {}, problems, "x.md"), text)
        self.assertEqual(len(problems), 1)

    def test_formats(self):
        self.assertEqual(M.fmt(12362, None), "12,362")
        self.assertEqual(M.fmt(12362, "raw"), "12362")
        self.assertEqual(M.fmt(98.34, None), "98.3")
        self.assertEqual(M.fmt("ADR-058", None), "ADR-058")


class Computed(unittest.TestCase):
    def test_the_counts_are_the_repositorys(self):
        c = {k: v for k, (v, _how) in M.computed().items()}
        text = "".join(p.read_text() for p in sorted((ROOT / "tests").glob("*.cpp")))
        self.assertEqual(c["cpp_tests"], text.count("\nTFC_TEST(") + (1 if text.startswith("TFC_TEST(") else 0))
        self.assertEqual(c["fault_matrix_rows"], len([ln for ln in (ROOT / "docs/verification/FAULT_MATRIX.md").read_text().splitlines() if ln.startswith("| F")]))
        self.assertGreater(c["mutants_unit"], 390)
        self.assertLessEqual(c["mutants_campaign"], c["mutants_unit"])
        self.assertEqual(c["campaign_scenarios"] > 12000, True)
        self.assertTrue(str(c["adr_latest"]).startswith("ADR-"))


class Documents(unittest.TestCase):
    def test_every_marker_in_the_documents_is_current(self):
        self.assertEqual(M.cmd_apply(True), 0)


if __name__ == "__main__":
    unittest.main()
