# SPDX-License-Identifier: MIT
"""The flow fields of the 3D viewer's CFD view (docs/design/CFD.md section 7): the library the console lists and serves, that a name outside it is refused, and that a file the solver (tfc_cfd --field) wrote has
the layout the viewer's decoder (console/web/viewer/js/cfd.js) reads: a JSON header padded to a multiple of four bytes, the skin's pressure coefficients as float32, three volumes of 16-bit numbers."""
import json
import os
import shutil
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.cfdlib import CfdLibrary

CFD = Path(os.environ.get("TFC_CFD_BIN") or ROOT / "build" / "host" / "tfc_cfd")


def fake(path: Path, **head) -> None:
    h = {"k": "cfd", "v": 1, "vehicle": "v", "stage_from": 0, "mach": 3.0, "alpha_deg": 4.0, "ca_skin": 0.2, "cn": 0.2, "x_cp_m": 100.0, "cells": 10, "iterations": 5, "residual_drop": 2.0, "h_m": 1.0, "d_ref_m": 9.0}
    h.update(head)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((json.dumps(h) + "\n").encode() + b"\0" * 16)


class Library(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / "cfd").mkdir()
        (self.tmp / "demo").mkdir()
        fake(self.tmp / "cfd" / "ship" / "m3_a4.cfd", mach=3.0)
        fake(self.tmp / "cfd" / "ship" / "m5_a0.cfd", mach=5.0, alpha_deg=0.0)
        fake(self.tmp / "demo" / "one.cfd", mach=2.0)
        (self.tmp / "cfd" / "ship" / "notes.txt").write_text("not a flow field")
        (self.tmp / "cfd" / "ship" / "broken.cfd").write_text("this is not json\n")
        (self.tmp / "cfd" / "README.md").write_text("x")
        self.lib = CfdLibrary(self.tmp / "cfd", self.tmp / "demo")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_the_cases_are_listed_from_their_headers_and_what_is_not_a_flow_field_is_left_out(self):
        m = self.lib.manifest()
        names = [c["name"] for c in m["cases"]]
        self.assertTrue(m["present"])
        self.assertEqual(sorted(names), ["cfd/ship/m3_a4.cfd", "cfd/ship/m5_a0.cfd", "demo/one.cfd"])
        c = next(c for c in m["cases"] if c["name"] == "cfd/ship/m5_a0.cfd")
        self.assertEqual((c["mach"], c["alpha_deg"], c["vehicle"], c["stage_from"]), (5.0, 0.0, "v", 0))
        self.assertGreater(c["bytes"], 0)

    def test_an_empty_library_says_so(self):
        self.assertEqual(CfdLibrary(self.tmp / "nowhere").manifest(), {"present": False, "cases": []})

    def test_only_a_name_in_the_manifest_is_served(self):
        body, ctype = self.lib.file_bytes("cfd/ship/m3_a4.cfd")
        self.assertTrue(body.startswith(b'{"k": "cfd"'))
        self.assertEqual(ctype, "application/octet-stream")
        for bad in ("../cfd/ship/m3_a4.cfd", "/etc/passwd", "cfd/ship/notes.txt", "cfd/ship/broken.cfd", "cfd/../demo/one.cfd", "", "m3_a4.cfd"):
            with self.assertRaises(ValueError, msg=bad):
                self.lib.file_bytes(bad)

    def test_a_file_that_changes_is_read_again(self):
        self.assertEqual(next(c for c in self.lib.manifest()["cases"] if c["name"] == "demo/one.cfd")["mach"], 2.0)
        fake(self.tmp / "demo" / "one.cfd", mach=9.0)
        os.utime(self.tmp / "demo" / "one.cfd", (1e9, 1e9 + 5))
        self.assertEqual(next(c for c in self.lib.manifest()["cases"] if c["name"] == "demo/one.cfd")["mach"], 9.0)

    def test_the_committed_demo_cases_are_flow_fields_the_decoder_can_read(self):
        lib = CfdLibrary(ROOT / "console" / "demo")
        for c in lib.manifest()["cases"]:
            body, _ = lib.file_bytes(c["name"])
            check_layout(self, body)


def check_layout(tc: unittest.TestCase, body: bytes) -> dict:
    """The layout cfd.js decodes: the header line ends at a newline at an offset one short of a multiple of four, then ns x na float32, then three volumes of nx x ny x nz uint16."""
    nl = body.index(b"\n")
    head = json.loads(body[:nl])
    tc.assertEqual((nl + 1) % 4, 0, "the numbers after the header must start on a multiple of four bytes (the decoder views them as float32 and uint16)")
    ns, na = head["surface"]["stations"], head["surface"]["angles"]
    v = head["volume"]
    n = v["nx"] * v["ny"] * v["nz"]
    tc.assertEqual(v["bits"], 16)
    tc.assertEqual(len(body), nl + 1 + (ns * na * 4) + (6 * n))
    tc.assertEqual(len(head["surface"]["x"]), ns)
    cp = struct.unpack_from(f"<{ns * na}f", body, nl + 1)
    tc.assertTrue(all(abs(x) < 50 for x in cp))
    vol = struct.unpack_from(f"<{3 * n}H", body, nl + 1 + ns * na * 4)
    tc.assertGreater(sum(1 for x in vol[:n] if x == 0), 0, "some cells are inside the body")
    tc.assertGreater(sum(1 for x in vol[:n] if x != 0), n // 4, "and most of the box is fluid")
    return head


@unittest.skipUnless(CFD.exists(), "tfc_cfd is not built")
class Solver(unittest.TestCase):
    def test_a_flow_field_the_solver_writes_has_the_layout_the_viewer_decodes(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "case.cfd"
            r = subprocess.run([str(CFD), str(ROOT / "vehicles" / "two_stage_launcher.json"), "--mach", "3", "--alpha", "4", "--h", "0.4", "--iters", "40", "--orders", "9", "--threads", "2", "--field", str(out)],
                               capture_output=True, text=True, timeout=300)
            self.assertEqual(r.returncode, 0, r.stdout[-600:] + r.stderr[-600:])
            head = check_layout(self, out.read_bytes())
            self.assertEqual((head["mach"], head["alpha_deg"], head["stage_from"], head["gamma"]), (3.0, 4.0, 0, 1.4))
            self.assertGreater(head["body"]["x1"], head["body"]["x0"])
            self.assertGreater(len(head["body"]["profile"]), 4)


if __name__ == "__main__":
    unittest.main()
