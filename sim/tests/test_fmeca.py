# SPDX-License-Identifier: MIT
"""The FMECA tool (tools/fmeca/fmeca.py): the repository passes its checks, and every check refuses what it says it refuses (a check that cannot fail proves nothing)."""
import dataclasses
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "fmeca"))
import data as D  # noqa: E402
import fmeca as F  # noqa: E402


def with_modes(modes):
    return mock.patch.object(D, "MODES", tuple(modes))


class Repository(unittest.TestCase):
    def test_the_data_passes_every_check_and_the_document_is_the_one_it_makes(self):
        self.assertEqual(F.check_data(), [])
        self.assertEqual(F.DOC.read_text(), F.render())

    def test_every_row_of_the_fault_matrix_and_every_fault_kind_is_in_some_failure_mode(self):
        rows = F.fault_rows()
        self.assertGreaterEqual(len(rows), 102)
        self.assertEqual(set(rows) - {f for m in D.MODES for f in m.faults}, set())
        self.assertEqual(F.fault_kinds() - {F.kind_of_group(k) for m in D.MODES for k in m.kinds}, set())

    def test_the_measured_figures_are_the_campaigns(self):
        camp, groups = F.campaign()
        self.assertIn("dropout", camp)
        self.assertIn("pairs", groups)
        m = next(x for x in D.MODES if x.id == "FM-S4")           # spike: one kind, so the row repeats the campaign's own line
        k = camp["spike"]
        self.assertIn(f"isolated {k['iso']}/{k['of']}", F.measured(m, camp))
        self.assertIn("falsely isolated 0/", F.measured(m, camp))


class Refusals(unittest.TestCase):
    def test_a_row_of_the_fault_matrix_that_no_mode_names_is_refused(self):
        keep = [dataclasses.replace(m, faults=tuple(f for f in m.faults if f != "F18")) for m in D.MODES]
        with with_modes(keep):
            self.assertTrue(any(e.startswith("F18:") for e in F.check_data()))

    def test_a_fault_kind_that_no_mode_names_is_refused(self):
        keep = [dataclasses.replace(m, kinds=tuple(k for k in m.kinds if k != "zero")) for m in D.MODES]
        with with_modes(keep):
            self.assertTrue(any("'zero'" in e for e in F.check_data()))

    def test_an_unknown_row_kind_or_recommendation_is_refused(self):
        bad = dataclasses.replace(D.MODES[0], faults=("F999",), kinds=("nonsense",), recs=("REC-99",))
        with with_modes([bad, *D.MODES[1:]]):
            errors = " ".join(F.check_data())
        self.assertIn("F999 is not a row", errors)
        self.assertIn("'nonsense' is neither", errors)
        self.assertIn("REC-99 is not a recommendation", errors)

    def test_a_redundant_mode_cannot_be_probable(self):
        bad = dataclasses.replace(next(m for m in D.MODES if m.suffix == "R"), pn=3)
        with with_modes([bad]):
            self.assertTrue(any("R mode" in e for e in F.check_data()))

    def test_a_critical_item_with_no_test_and_no_recommendation_is_refused(self):
        bad = dataclasses.replace(next(m for m in D.MODES if m.id == "FM-W1"), recs=())
        with with_modes([bad]):
            self.assertTrue(any("FM-W1" in e and "critical item" in e for e in F.check_data()))

    def test_a_single_point_that_limitations_names_must_be_in_the_analysis(self):
        without_act = [dataclasses.replace(m, suffix="R") if "ACT" in m.item else m for m in D.MODES]
        with with_modes(without_act):
            self.assertTrue(any("ACT" in e and "single point" in e for e in F.check_data()))

    def test_a_malformed_cell_of_the_software_fmea_is_refused(self):
        rows, kinds, groups, recs = F.fault_rows(), F.fault_kinds(), F.campaign()[1], {r.id for r in D.RECS}
        self.assertEqual(F.check_cell("n/a", rows, kinds, groups, recs), [])
        self.assertEqual(F.check_cell("host: anything", rows, kinds, groups, recs), [])
        self.assertEqual(F.check_cell("dropout", rows, kinds, groups, recs), [])
        self.assertEqual(F.check_cell("F15, F85", rows, kinds, groups, recs), [])
        self.assertTrue(F.check_cell("GAP REC-99", rows, kinds, groups, recs))
        self.assertTrue(F.check_cell("F999", rows, kinds, groups, recs))
        self.assertTrue(F.check_cell("nonsense", rows, kinds, groups, recs))


class Rules(unittest.TestCase):
    def test_the_criticality_number_and_the_critical_item_rule_are_ecss_5_2_and_5_3(self):
        base = D.MODES[0]
        for sev, pn, want_cn, want_critical in ((1, 1, 4, True), (2, 3, 9, True), (2, 2, 6, True), (3, 3, 6, True), (3, 2, 4, False), (4, 4, 4, False)):
            m = dataclasses.replace(base, sev=sev, pn=pn)
            self.assertEqual((F.cn(m), F.critical(m)), (want_cn, want_critical), (sev, pn))

    def test_the_evidence_of_a_mode_is_that_of_its_weakest_row(self):
        rows = {"F1": F.FaultRow("F1", "", [], "", "Passing (SIL)"), "F2": F.FaultRow("F2", "", [], "", "Rig pending"),
                "F3": F.FaultRow("F3", "", [], "", "Passing (host); rig pending"), "F4": F.FaultRow("F4", "", [], "", "Passing (host, live)")}
        self.assertEqual(F.evidence_of(("F1", "F4"), rows, ""), "verified on the host")
        self.assertEqual(F.evidence_of(("F1", "F3"), rows, ""), "host, rig pending")
        self.assertEqual(F.evidence_of(("F1", "F2"), rows, ""), "rig pending")
        self.assertEqual(F.evidence_of((), rows, ""), "none")
        self.assertEqual(F.evidence_of(("F2",), rows, "out of v1"), "out of v1")


if __name__ == "__main__":
    unittest.main()
