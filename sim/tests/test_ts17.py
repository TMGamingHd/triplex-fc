# SPDX-License-Identifier: MIT
"""TS-17, the paper part (campaign/ts17.py): the arithmetic of the coverage model and the claims that docs/TRADE_STUDIES.md section 9c makes from it."""
import unittest

from campaign import ts17


class CoverageModel(unittest.TestCase):
    def test_every_cover_names_a_scenario_and_an_override_that_exist(self):
        self.assertEqual(set(ts17.COVERS), set(ts17.SCENARIOS))
        for scenario, covers in ts17.COVERS.items():
            for h, w in covers.items():
                self.assertIn(h, ts17.OVERRIDES, scenario)
                self.assertIn(w, (0.5, 1.0))
        for name, hs in ts17.OPTIONS.items():
            self.assertTrue(set(hs) <= set(ts17.OVERRIDES), name)
        self.assertEqual(set(ts17.HARM), set(ts17.OVERRIDES))
        self.assertEqual(set(ts17.COST), set(ts17.OVERRIDES))

    def test_the_options_are_cumulative_so_each_is_at_least_as_good_as_the_one_before(self):
        names = list(ts17.OPTIONS)
        for a, b in zip(names, names[1:]):
            self.assertTrue(set(ts17.OPTIONS[a]) <= set(ts17.OPTIONS[b]))
            self.assertLessEqual(ts17.cost(a), ts17.cost(b))
            for s in ts17.SCENARIOS:
                for r in (0.5, 0.95):
                    self.assertLessEqual(ts17.p_safe(a, s, r), ts17.p_safe(b, s, r) + 1e-12)

    def test_the_probability_is_the_complement_of_every_covering_override_failing(self):
        self.assertAlmostEqual(ts17.p_safe("O1", "SD1", 0.9), 0.9)              # H1 alone
        self.assertAlmostEqual(ts17.p_safe("O2", "SD1", 0.9), 1 - 0.1 * 0.1)    # H1 and H3
        self.assertAlmostEqual(ts17.p_safe("O2", "SD3", 0.8), 1 - 0.2 * 0.2 * (1 - 0.5 * 0.8))  # H2 only covers in part
        self.assertEqual(ts17.p_safe("O0", "SD1", 0.95), 0.0)
        self.assertEqual(ts17.p_safe("O4", "SD1", 0.0), 0.0)

    def test_the_smallest_option_with_a_program_free_cover_for_every_scenario_is_o3_and_o2_misses_the_two_that_can_cut_power(self):
        self.assertEqual(ts17.smallest_option_covering_every_scenario(), "O3")
        missing = {s for s in ts17.SCENARIOS if ts17.fully_covered("O2", s) == 0}
        self.assertEqual(missing, {"SD5", "SD6", "SD9"})
        self.assertEqual({s for s in ts17.SCENARIOS if ts17.fully_covered("O3", s) == 0}, {"SD9"})  # nothing but a better adapter removes a failing adapter

    def test_o4_adds_little_over_o3_at_the_assumed_reliability_and_only_in_scenarios_with_a_free_or_partial_cover(self):
        gain = {s: ts17.p_safe("O4", s, 0.95) - ts17.p_safe("O3", s, 0.95) for s in ts17.SCENARIOS}
        self.assertTrue(all(g >= 0 for g in gain.values()))
        self.assertEqual({s for s, g in gain.items() if g > 0.0}, {"SD5", "SD7", "SD9"})
        mean = lambda o, r: sum(ts17.p_safe(o, s, r) for s in ts17.SCENARIOS) / len(ts17.SCENARIOS)  # noqa: E731
        self.assertLess(mean("O4", 0.95) - mean("O3", 0.95), 0.05)

    def test_the_tables_print(self):
        text = ts17.tables()
        for needle in ("| O3 |", "SD9", "P(safe state reached)"):
            self.assertIn(needle, text)


if __name__ == "__main__":
    unittest.main()
