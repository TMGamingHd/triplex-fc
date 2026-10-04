# SPDX-License-Identifier: MIT
"""The bench tools (tools/bench): frame-timing analysis of logic-analyser edges, and the alignment of a live bus log for tfc_replay."""
import importlib.util
import os
import random
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BENCH = ROOT / "tools" / "bench"
REPLAY = os.environ.get("TFC_REPLAY_BIN", str(ROOT / "build" / "host" / "tfc_replay"))


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, BENCH / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


jitter = load("frame_jitter")
log_t0 = load("log_t0")
golden = load("check_golden")

from tfc_peers import peers as PE  # noqa: E402
from tfc_peers import protocol as P  # noqa: E402


def edges(n=6000, period=0.01, sigma_us=0.0, ppm=0.0, drop=(), seed=7):
    rnd = random.Random(seed)
    out = []
    for i in range(n):
        if i in drop:
            continue
        out.append(i * period * (1.0 + ppm * 1e-6) + rnd.gauss(0.0, sigma_us * 1e-6))
    return out


class FrameJitter(unittest.TestCase):
    def test_a_clean_grid_passes(self):
        r = jitter.analyse(edges(sigma_us=20.0), 0.01)
        self.assertEqual(r["missed_frames"], 0)
        self.assertLess(r["jitter_p99_us"], 100.0)
        self.assertGreater(r["jitter_p99_us"], 20.0)
        self.assertAlmostEqual(r["mean_period_us"], 10000.0, delta=0.5)

    def test_a_dropped_frame_is_counted_and_fails(self):
        r = jitter.analyse(edges(sigma_us=5.0, drop={1000}), 0.01)
        self.assertEqual(r["missed_frames"], 1)
        r = jitter.analyse(edges(sigma_us=5.0, drop={1000, 1001, 1002}), 0.01)
        self.assertEqual(r["missed_frames"], 3)

    def test_a_slow_clock_is_reported_as_ppm_not_as_jitter(self):
        r = jitter.analyse(edges(sigma_us=5.0, ppm=50.0), 0.01)
        self.assertAlmostEqual(r["clock_error_ppm"], 50.0, delta=1.0)
        self.assertLess(r["jitter_p99_us"], 20.0)

    def test_large_jitter_fails_the_threshold(self):
        r = jitter.analyse(edges(sigma_us=200.0), 0.01)
        self.assertGreater(r["jitter_p99_us"], 100.0)

    def test_file_formats_and_the_exit_status(self):
        good = edges(n=500, sigma_us=10.0)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "e.csv"
            p.write_text("Time [s],Channel 0\n" + "".join(f"{t:.9f},1\n{t + 0.001:.9f},0\n" for t in good))
            self.assertEqual(jitter.main([str(p)]), 0)  # a level export: only the high rows count
            p.write_text("".join(f"{t * 1e3:.6f}\n" for t in good))
            self.assertEqual(jitter.main([str(p), "--unit", "ms"]), 0)
            p.write_text("".join(f"{t:.9f}\n" for t in edges(n=500, sigma_us=10.0, drop={100})))
            self.assertEqual(jitter.main([str(p)]), 1)
            p.write_text("1.0\n2.0\n")
            self.assertEqual(jitter.main([str(p)]), 2)  # too few edges


@unittest.skipUnless(os.path.exists(REPLAY), "tfc_replay is not built")
class LiveLogAlignment(unittest.TestCase):
    """A log of a live bus starts mid-run and carries SYNC frame numbers; without alignment the replay sees thousands of sequence errors."""

    def live_log(self, first_kept=150, offset_s=1000.0):
        with tempfile.TemporaryDirectory() as d:
            ref = Path(d) / "ref.log"
            subprocess.run([sys.executable, "-m", "tfc_peers", "record", "--nodes", "A,B,C", "--frames", "600", "--fault",
                            "B:bias:start=300,mag=3", "--out", str(ref)], cwd=ROOT / "sim", check=True, capture_output=True)
            out = []
            for line in ref.read_text().splitlines():
                stamp, iface, body = line.split()
                sec, usec = stamp.strip("()").split(".")
                t_us = int(sec) * 1_000_000 + int(usec)
                if t_us >= first_kept * 10_000 - 3_000:
                    out.append((t_us, iface, body))
            for k in range(first_kept, 600):
                out.append((k * 10_000, "vcan0", "010#" + k.to_bytes(4, "little").hex().upper() + "0000" + "00"))
            out.sort(key=lambda x: x[0])
            return "\n".join(f"({int(offset_s) + t // 1_000_000}.{t % 1_000_000:06d}) {i} {b}" for t, i, b in out) + "\n"

    def replay(self, path, *extra):
        r = subprocess.run([REPLAY, str(path), *extra], capture_output=True, text=True)
        return dict(l.split("=", 1) for l in r.stdout.splitlines() if "=" in l and not l.startswith("frame"))

    def test_the_aligned_replay_finds_the_same_latch_as_the_original(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "live.log"
            p.write_text(self.live_log())
            a = log_t0.align(p.read_text().splitlines())
            self.assertIsNotNone(a)
            t0, first = a
            self.assertGreaterEqual(first, 140)
            self.assertLessEqual(first, 150)
            aligned = self.replay(p, "--t0", f"{t0:.6f}", "--first-frame", str(first))
            self.assertEqual(aligned["seq_bad"], "0", aligned)
            # the replay counts frames from the start of the log; the bus's number is that plus first_frame: B latched at 302 on the bus
            self.assertEqual(int(aligned["latch.B"]) + first, 302, aligned)

    def test_without_alignment_the_phase_check_fails(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "live.log"
            p.write_text(self.live_log())
            t0, _ = log_t0.align(p.read_text().splitlines())
            naive = self.replay(p, "--t0", f"{t0:.6f}")
            self.assertGreater(int(naive["seq_bad"]), 100, naive)

    def test_a_log_without_sync_cannot_be_aligned(self):
        self.assertIsNone(log_t0.align(["(1.000000) vcan0 100#0000000000000000"]))


class GoldenRun(unittest.TestCase):
    def stream(self, node, faults=(), n=200):
        vn = PE.VirtualNode(node, list(faults), 1)
        out = []
        for k in range(n):
            out.append(P.pack_sync(k, k & 0xFF))
            out += [tf.frame for tf in vn.step(k)]
        return out

    def test_a_healthy_replica_matches_the_golden_run_bit_for_bit(self):
        r = golden.compare(self.stream(0), 0)
        self.assertEqual(r["compared"], 200)
        self.assertEqual(r["mismatches"], [])

    def test_a_wrong_command_or_digest_is_reported(self):
        from tfc_peers.faults import parse_fault
        r = golden.compare(self.stream(1, [parse_fault("B:cmd_offset:start=50,mag=2")]), 1)
        self.assertEqual(r["compared"], 200)
        self.assertEqual(len(r["mismatches"]), 5)  # the first five are listed
        r = golden.compare(self.stream(1, [parse_fault("B:digest:start=10,end=11")]), 1)
        self.assertEqual(len(r["mismatches"]), 1)

    def test_commands_before_any_sync_or_of_another_cycle_are_skipped_not_guessed(self):
        frames = self.stream(0, n=20)
        no_sync = [f for f in frames if f.id != P.ID_SYNC]
        self.assertEqual(golden.compare(no_sync, 0)["compared"], 0)
        # a command whose sequence byte does not belong to the cycle of the last SYNC
        stale = [f for f in frames if f.id != P.ID_SYNC][:1] + [P.pack_sync(7, 7)]
        self.assertEqual(golden.compare(stale, 0)["compared"], 0)


if __name__ == "__main__":
    unittest.main()
