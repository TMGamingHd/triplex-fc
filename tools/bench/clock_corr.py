#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Correlate the supervisor's counter with the PC's UTC, over a run of any length (docs/design/MISSION_CLOCK.md section 2, TFC-SUP-011 and SUP-012).

    python3 tools/bench/clock_corr.py --port /dev/ttyACM0 [--interval 10] [--duration 3600] [--out pairs.csv]     sample a live supervisor
    python3 tools/bench/clock_corr.py --pairs pairs.csv                                                           fit a recorded run

Every `interval` seconds the supervisor is sent `time`; its answer ("time ticks=N rtc=S met_us=M|-") is stamped with the PC's UTC (the PC's clock is disciplined by NTP) as it arrives, and the pair
(ticks, UTC microseconds) goes to `tfc_peers.timecorr.Correlator`, which fits the offset and the drift. It prints the oscillator's drift in ppm, the scatter, how far the converted time can be trusted now
and five years out, and the number of pairs rejected as stalls. `--out` keeps the pairs (CSV: ticks, utc_us) so a night of data can be fitted again later. The PC's stamp is late by the USB and operating-system latency;
that is the scatter, and a constant part of it is a bias the pairs cannot show (the correlator adds a stated allowance). Exit status 0 with a fit, 1 without enough pairs, 2 usage.
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim"))
from tfc_peers.timecorr import Correlator  # noqa: E402

TICK_HZ = 1_000_000  # sup::SupConfig::tick_hz
FIVE_YEARS_S = 5 * 365.25 * 86400.0
LINE = re.compile(r"time ticks=(\d+) rtc=(\d+) met_us=(\d+|-)")


def parse_time_line(line: str) -> tuple[int, int, int | None] | None:
    """(ticks, rtc_seconds, met_us or None before T-zero) from the supervisor's answer to `time`, or None if the line is anything else."""
    m = LINE.search(line)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), None if m.group(3) == "-" else int(m.group(3))


def sample(read_line: Callable[[float], str | None], write: Callable[[str], None], utc_us: Callable[[], float], sleep: Callable[[float], None],
           interval_s: float, duration_s: float, on_pair: Callable[[int, float], None]) -> int:
    """Ask for the time every `interval_s` for `duration_s` seconds; returns the number of answers understood."""
    got = 0
    t_end = utc_us() + duration_s * 1e6
    while True:
        write("time\n")
        deadline = utc_us() + min(interval_s, 2.0) * 1e6
        while utc_us() < deadline:
            line = read_line(0.2)
            if line is None:
                continue
            stamp = utc_us()  # stamped at arrival: the latency of the line is the scatter
            parsed = parse_time_line(line)
            if parsed is not None:
                on_pair(parsed[0], stamp)
                got += 1
                break
        if utc_us() >= t_end:
            return got
        sleep(interval_s)


def report(c: Correlator) -> str:
    lines = [f"pairs kept {len(c.pairs)}, rejected as stalls {c.rejected}, counter restarts {c.resets}, span {c.span_s() / 3600.0:.2f} h"]
    if not c.ok():
        lines.append("not enough pairs or span for a fit (at least 5 pairs over 10 s)")
        return "\n".join(lines)
    f = c.fit
    assert f is not None
    now = c.pairs[-1][0]
    lines.append(f"drift {c.drift_ppm():+.2f} ppm (the supervisor's oscillator against UTC), scatter {f.sigma_us:.0f} us")
    b_now = c.error_bound_us(now)
    b_5y = c.error_bound_us(now + int(FIVE_YEARS_S * c.tick_hz))
    lines.append(f"converted time good to +-{b_now / 1000.0:.1f} ms now (3 sigma plus the latency allowance), +-{b_5y / 1e6:.1f} s five years out (only as good as the baseline: extend it)")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="clock_corr", description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="the supervisor's USB serial port (needs pyserial)")
    ap.add_argument("--pairs", help="a CSV of (ticks, utc_us) pairs recorded earlier")
    ap.add_argument("--interval", type=float, default=10.0, help="seconds between `time` requests (default 10)")
    ap.add_argument("--duration", type=float, default=3600.0, help="seconds to sample (default one hour)")
    ap.add_argument("--out", help="write the pairs to this CSV")
    args = ap.parse_args(argv)
    if bool(args.port) == bool(args.pairs):
        print("give exactly one of --port and --pairs", file=sys.stderr)
        return 2
    c = Correlator(TICK_HZ)
    kept: list[tuple[int, float]] = []
    if args.pairs:
        for ln in Path(args.pairs).read_text().splitlines():
            if ln and not ln.startswith("#"):
                t, u = ln.split(",")
                c.add(int(t), float(u))
    else:
        import serial  # pyserial: only with a real port

        port = serial.Serial(args.port, 115200, timeout=0.2)

        def read_line(timeout: float) -> str | None:
            port.timeout = timeout
            raw = port.readline()
            return raw.decode(errors="replace").strip() if raw else None

        def on_pair(ticks: int, utc: float) -> None:
            kept.append((ticks, utc))
            c.add(ticks, utc)

        sample(read_line, lambda s: port.write(s.encode()), lambda: time.time() * 1e6, time.sleep, args.interval, args.duration, on_pair)
        if args.out:
            Path(args.out).write_text("# ticks, utc_us\n" + "".join(f"{t},{u:.0f}\n" for t, u in kept))
    print(report(c))
    return 0 if c.ok() else 1


if __name__ == "__main__":
    raise SystemExit(main())
