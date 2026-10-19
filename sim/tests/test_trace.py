# SPDX-License-Identifier: MIT
"""The requirement trace (tools/trace/check_trace.py): the requirements marked as verified by an automated test that no test names may only get fewer."""
import importlib.util
import json
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("check_trace", ROOT / "tools" / "trace" / "check_trace.py")
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


class RequirementTrace(unittest.TestCase):
    def test_no_requirement_marked_t_loses_its_test_or_is_added_without_one(self):
        _reqs, _automated, orphans = trace.analyse()
        base = json.loads((ROOT / "tools" / "trace" / "baseline.json").read_text())["orphans"]
        new = [o for o in orphans if o not in base]
        self.assertEqual(new, [], "name a test with `// Verifies: ID` (or give the requirement a row in FAULT_MATRIX.md)")

    def test_the_baseline_lists_only_requirements_that_exist_and_are_still_orphans(self):
        reqs, _automated, orphans = trace.analyse()
        ids = {r["id"] for r in reqs}
        base = json.loads((ROOT / "tools" / "trace" / "baseline.json").read_text())["orphans"]
        self.assertTrue(set(base) <= ids)
        self.assertEqual([b for b in base if b not in orphans], [], "these now have a test: remove them from the baseline (python3 tools/trace/check_trace.py --write-baseline tools/trace/baseline.json)")

    def test_the_tool_finds_a_traced_requirement_and_an_untraced_one(self):
        reqs, automated, orphans = trace.analyse()
        self.assertGreater(len(reqs), 150)
        self.assertGreater(len(automated) - len(orphans), 30)
        self.assertGreater(len(orphans), 0)


if __name__ == "__main__":
    unittest.main()
