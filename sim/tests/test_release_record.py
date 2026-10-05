# SPDX-License-Identifier: MIT
"""tools/release/record_release.py (ADR-021, TFC-ARCH-005): the image hash, the campaign summary, and the refusals."""
import importlib.util
import json
import struct
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("record_release", ROOT / "tools" / "release" / "record_release.py")
rr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rr)


def tiny_elf32(payload: bytes, paddr: int = 0x08000000, path_noise: bytes = b"") -> bytes:
    """A 32-bit little-endian ELF with one PT_LOAD segment holding `payload`, followed by `path_noise` that no segment covers (debug information, with file paths in it)."""
    ehsize, phentsize = 52, 32
    hdr = bytearray(ehsize)
    hdr[0:4] = b"\x7fELF"
    hdr[4], hdr[5], hdr[6] = 1, 1, 1
    struct.pack_into("<HHIIIIIHHHHHH", hdr, 16, 2, 0x28, 1, paddr, ehsize, 0, 0, ehsize, phentsize, 1, 0, 0, 0)
    off = ehsize + phentsize
    ph = struct.pack("<IIIIIIII", 1, off, paddr, paddr, len(payload), len(payload), 5, 4)
    return bytes(hdr) + ph + payload + path_noise


class RecordTests(unittest.TestCase):
    def test_the_image_hash_covers_the_loadable_bytes_and_ignores_the_rest(self):
        with tempfile.TemporaryDirectory() as d:
            a = Path(d) / "a.elf"
            b = Path(d) / "b.elf"
            c = Path(d) / "c.elf"
            a.write_bytes(tiny_elf32(b"\x01\x02\x03\x04", path_noise=b"/home/alice/build/src/main.cpp"))
            b.write_bytes(tiny_elf32(b"\x01\x02\x03\x04", path_noise=b"/tmp/other/dir/main.cpp"))
            c.write_bytes(tiny_elf32(b"\x01\x02\x03\x05"))
            ha, na = rr.image_hash(a)
            hb, _ = rr.image_hash(b)
            hc, _ = rr.image_hash(c)
            self.assertEqual(ha, hb)
            self.assertNotEqual(ha, hc)
            self.assertEqual(na, 4)
            moved = Path(d) / "m.elf"
            moved.write_bytes(tiny_elf32(b"\x01\x02\x03\x04", paddr=0x08001000))
            self.assertNotEqual(rr.image_hash(moved)[0], ha)  # the same bytes at another address is another image
            notelf = Path(d) / "x"
            notelf.write_bytes(b"hello")
            with self.assertRaises(ValueError):
                rr.image_hash(notelf)

    def test_the_real_nucleo_image_hashes_the_same_twice(self):
        elf = ROOT / "build" / "nucleo_flight" / "zephyr" / "zephyr.elf"
        if not elf.exists():
            self.skipTest("the nucleo flight image is not built")
        h, n = rr.image_hash(elf)
        self.assertEqual((h, n), rr.image_hash(elf))
        self.assertGreater(n, 10000)

    def test_the_campaign_summary_counts_scenarios_and_anomalies(self):
        with tempfile.TemporaryDirectory() as d:
            f = Path(d) / "c.jsonl"
            f.write_text(json.dumps({"anomalies": []}) + "\n" + json.dumps({"anomalies": [["S1", "x"], ["S3", "y"]]}) + "\n\n" + json.dumps({"anomalies": [["S1", "z"]]}) + "\n")
            s = rr.campaign_summary(f)
            self.assertEqual((s["scenarios"], s["anomalies"], s["codes"]), (3, 3, {"S1": 2, "S3": 1}))

    def test_a_release_is_refused_with_anomalies_or_without_a_reproducible_rebuild_unless_allowed(self):
        with tempfile.TemporaryDirectory() as d:
            elf = Path(d) / "a.elf"
            elf.write_bytes(tiny_elf32(b"abcd"))
            same = Path(d) / "same.elf"
            same.write_bytes(tiny_elf32(b"abcd", path_noise=b"other path"))
            different = Path(d) / "different.elf"
            different.write_bytes(tiny_elf32(b"abce"))
            clean = Path(d) / "clean.jsonl"
            clean.write_text(json.dumps({"anomalies": []}) + "\n")
            dirty = Path(d) / "dirty.jsonl"
            dirty.write_text(json.dumps({"anomalies": [["S1", "x"]]}) + "\n")
            empty = Path(d) / "empty.jsonl"
            empty.write_text("")
            out = Path(d) / "out" / "r.json"
            _rec, problems = rr.build_record("t", elf, same, clean, False)
            self.assertTrue(all("tree is not clean" in p for p in problems) or problems == [])  # (the working tree of this checkout may or may not be clean)
            self.assertFalse(any("reproducible" in p for p in problems))
            _rec, problems = rr.build_record("t", elf, different, clean, False)
            self.assertTrue(any("not reproducible" in p for p in problems))
            _rec, problems = rr.build_record("t", elf, None, clean, False)
            self.assertTrue(any("reproducibility is not shown" in p for p in problems))
            _rec, problems = rr.build_record("t", elf, same, dirty, False)
            self.assertTrue(any("1 anomalies" in p for p in problems))
            _rec, problems = rr.build_record("t", elf, same, empty, False)
            self.assertTrue(any("no scenarios" in p for p in problems))
            self.assertEqual(rr.main(["--tag", "t", "--elf", str(elf), "--elf-rebuilt", str(different), "--campaign", str(clean), "--out", str(out)]), 1)
            self.assertFalse(out.exists())
            self.assertEqual(rr.main(["--tag", "t", "--elf", str(elf), "--elf-rebuilt", str(different), "--campaign", str(dirty), "--out", str(out), "--allow-anomalies"]), 0)
            rec = json.loads(out.read_text())
            self.assertTrue(rec["accepted_with_problems"] and rec["problems"] and rec["reproducible"] is False and rec["campaign"]["anomalies"] == 1)
            self.assertEqual(rr.main(["--tag", "t", "--elf", str(Path(d) / "missing.elf"), "--campaign", str(clean), "--out", str(out)]), 2)


if __name__ == "__main__":
    unittest.main()
