#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Print the `--t0` and `--first-frame` that line a live bus log up with the frame numbers on the bus, for `tfc_replay`.

`tfc_replay` groups frames into 10 ms frames by time from t0 and counts them from 0; a log written by `python3 -m tfc_peers record`
starts at frame 0, but a log of a live bus starts wherever the logger was started, and the frames carry the flight computer's SYNC
frame number. This reads the first SYNC (CAN id 0x010, payload = frame number, little endian) and returns the time at which frame 0
would have started, less half a millisecond so that a SYNC that reaches the PC a little late still lands in its own frame.

    build/host/tfc_replay "$LOG" --t0 "$(python3 tools/bench/log_t0.py "$LOG")" --verbose

Exit status 1 and no output if the log holds no SYNC (nothing to align to).
"""
from __future__ import annotations

import re
import sys

LINE = re.compile(r"\((\d+)\.(\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")
SYNC_ID = 0x010
FRAME_S = 0.010
EARLY_S = 0.0005
LEAD_S = 0.003


def align(lines) -> tuple[float, int] | None:
    """(t0, first_frame) for the log, or None if it has no SYNC."""
    t_first = None
    sync = None
    for line in lines:
        m = LINE.match(line.strip())
        if not m:
            continue
        t = int(m.group(1)) + int(m.group(2)) / 10 ** len(m.group(2))
        if t_first is None:
            t_first = t
        if sync is None and int(m.group(3), 16) == SYNC_ID:
            data = bytes.fromhex(m.group(4))
            if len(data) >= 4:
                sync = (t, int.from_bytes(data[:4], "little"))
    if sync is None or t_first is None:
        return None
    t_sync, n = sync
    t0 = t_sync - EARLY_S
    # A frame collects what arrives from 3 ms before its start (the vote is at 7 ms of the previous one): step back until the first line fits.
    steps = 0
    while t0 - LEAD_S > t_first and steps < n:
        t0 -= FRAME_S
        steps += 1
    return t0, n - steps


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    with open(argv[1], encoding="ascii") as fh:
        a = align(fh)
    if a is None:
        return 1
    print(f"{a[0]:.6f} {a[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
