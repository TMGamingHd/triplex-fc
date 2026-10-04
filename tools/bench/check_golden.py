#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Does a flight computer's command stream match the PC's golden run? (S1 exit test: "digest matches the PC golden run".)

Until the estimator exists (docs/SOFTWARE_READINESS.md SW-04) every replica computes its command and digest as a fixed function of the
frame number (firmware/app/src/sim_imu.hpp, mirrored in sim/tfc_peers/peers.py). This reads a bus log (`python3 -m tfc_peers log`), takes each
command frame of the chosen node (CAN id 0x200 + node), recovers the frame number from the SYNC frame that opened its cycle and the frame's own
sequence byte, regenerates the frame the PC would have produced, and compares the pitch, yaw and digest bytes.

    python3 tools/bench/check_golden.py bus.log [--node A]

Exit status 0 if every command frame matches, 1 if any differs, 2 if there is nothing to compare.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim"))

from tfc_peers import bus as B  # noqa: E402
from tfc_peers import peers as PE  # noqa: E402
from tfc_peers import protocol as P  # noqa: E402


def expected_cmd(node: int, k: int) -> P.Frame:
    """The command frame a healthy replica sends in frame k (the same arithmetic as VirtualNode.step)."""
    g_true, _ = PE.truth(k * PE.FRAME_US / 1e6 + 0.0005)
    pitch = PE._clamp(0.1 * g_true[0], 30.0)
    yaw = PE._clamp(0.1 * g_true[1], 30.0)
    digest = (P.quantize(pitch, P.CMD_LSB_DEG) * 31 + P.quantize(yaw, P.CMD_LSB_DEG) * 17 + k * 40503) & 0xFFFF
    return P.pack_cmd(node, pitch, yaw, digest, k & 0xFF)


def compare(frames, node: int) -> dict:
    """frames: iterable of Frame in bus order. Returns counts and the first few mismatches."""
    frame_no = None
    compared = 0
    bad: list[str] = []
    skipped = 0
    for f in frames:
        s = P.unpack_sync(f)
        if s is not None:
            frame_no = s.frame_no
            continue
        if f.id != P.ID_CMD_BASE + node:
            continue
        if frame_no is None or not P.check(f):
            skipped += 1
            continue
        k = frame_no
        if (f.data[6] & 0xFF) != (k & 0xFF):  # the frame belongs to another cycle than the last SYNC: do not guess
            skipped += 1
            continue
        compared += 1
        want = expected_cmd(node, k)
        if f.data[:6] != want.data[:6]:
            if len(bad) < 5:
                got = P.unpack_cmd(f)
                exp = P.unpack_cmd(want)
                bad.append(f"frame {k}: got pitch {got.pitch_deg:.3f} yaw {got.yaw_deg:.3f} digest {got.digest:#06x}, "
                           f"expected pitch {exp.pitch_deg:.3f} yaw {exp.yaw_deg:.3f} digest {exp.digest:#06x}")
    return {"compared": compared, "skipped": skipped, "mismatches": bad}


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="bus log in candump -L format")
    ap.add_argument("--node", default="A", help="A, B or C (default A)")
    a = ap.parse_args(argv)
    node = "ABC".index(a.node.upper())
    frames = [f for _t, _i, f in B.read_log(a.log)]
    r = compare(frames, node)
    if r["compared"] == 0:
        print("nothing to compare: no command frames of that node after a SYNC (is the flight computer on the bus?)")
        return 2
    print(f"compared {r['compared']} command frames of node {a.node.upper()} (skipped {r['skipped']}): "
          f"{'all match the golden run' if not r['mismatches'] else 'MISMATCHES, first ' + str(len(r['mismatches']))}")
    for line in r["mismatches"]:
        print("  " + line)
    return 0 if not r["mismatches"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
