# SPDX-License-Identifier: MIT
"""The mutant tables are alive: every deliberate bug still finds its text exactly once in the code it injects into (tools/mutation/check_tables.py)."""
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "mutation"))
import check_tables as T  # noqa: E402
import mutations  # noqa: E402
import sim_mutations  # noqa: E402


class Tables(unittest.TestCase):
    def test_no_mutant_has_gone_stale(self):
        self.assertEqual(T.stale(), [])

    def test_a_pattern_that_has_moved_is_found_out(self):
        name = next(iter(mutations.MUTATIONS))
        f, old, new = mutations.MUTATIONS[name]
        mutations.MUTATIONS[name] = (f, old + " # text that is not in the file", new)
        try:
            self.assertTrue(any(name in line for line in T.stale()))
        finally:
            mutations.MUTATIONS[name] = (f, old, new)

    def test_a_pattern_that_occurs_twice_is_found_out(self):
        name = "dup_probe"
        sim_mutations.MUTATIONS[name] = ("vehicle6.hpp", "  }\n", "  }\n")   # a closing brace and a newline occur everywhere
        try:
            self.assertTrue(any(name in line for line in T.stale()))
        finally:
            del sim_mutations.MUTATIONS[name]

    def test_the_report_lists_every_header(self):
        rows = {h: (n, k) for h, n, k in T.report()}
        self.assertIn("redundancy.hpp", rows)
        self.assertGreater(rows["redundancy.hpp"][1], 100)
        self.assertGreater(rows["fault_monitor.hpp"][1], 0)   # the monitors had no mutant until the review of core/
        self.assertGreater(rows["flightlog.hpp"][1], 0)


class Copies(unittest.TestCase):
    """The copy a mutant is made in must itself build: a header that includes a sibling directory breaks every mutant at once (it did, when the flow solver arrived: all of them BUILD-FAILED)."""

    def compiles(self, include_dirs, source):
        with tempfile.TemporaryDirectory() as d:
            tu = Path(d) / "probe.cpp"
            tu.write_text(source)
            r = subprocess.run(["g++", "-std=c++17", "-fsyntax-only", "-w", *[f"-I{i}" for i in include_dirs], str(tu)], capture_output=True, text=True)
            return r.returncode, r.stderr[:400]

    def test_the_simulator_copy_builds_unmutated(self):
        with tempfile.TemporaryDirectory() as d:
            inc = sim_mutations.build_vehicle("BASELINE", Path(d))
            rc, err = self.compiles([ROOT / "core" / "include", inc], '#include "vehicle6.hpp"\nint main() { return 0; }\n')
            self.assertEqual(rc, 0, err)

    def test_the_core_copy_builds_unmutated(self):
        with tempfile.TemporaryDirectory() as d:
            name = next(iter(mutations.MUTATIONS))
            inc = mutations.build_include(name, Path(d))
            rc, err = self.compiles([inc, Path(d) / "include_sup"], '#include "tfc/redundancy.hpp"\n#include "tfc/act.hpp"\nint main() { return 0; }\n')
            self.assertEqual(rc, 0, err)


if __name__ == "__main__":
    unittest.main()
