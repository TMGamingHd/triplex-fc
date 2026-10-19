#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The noise of a real IMU, from a record of it at rest: the Allan deviation, the noise density and the bias instability, against the datasheet.

The detection thresholds of the flight computers (a vote tolerance of 1 dps on a gyro, 0.02 g on an accelerometer) and the campaign's expectations were set against a *model* of the ISM330DHCX
(`docs/design/IMU_MODEL.md`: the datasheet's 5 mdps/rtHz and 3 deg/h). Whether the parts on the bench are like the datasheet, and by how much three of them disagree with each other, is a
measurement nobody has made. Record an hour or more of the IMUs at rest (P-S1-02) and run:

    python3 tools/bench/allan.py --log logs/rest.log --node A --sensor gyro            # from a candump -L log of the flight bus (the gyro and accelerometer frames at 100 Hz)
    python3 tools/bench/allan.py --csv rest.csv --column 1 --rate 100 --sensor gyro   # from a table of samples

The overlapping Allan deviation sigma(tau) is computed for tau = 1, 2, 4 ... samples. For a record at rest: the slope -1/2 at short tau is white noise (the angle or velocity random walk; its value at
tau = 1 s times sqrt(2) is the noise density in units/rtHz), the flat bottom is the bias instability (the minimum divided by 0.664), a slope +1/2 at long tau is the rate random walk (or temperature).
Limits of the bus as a source: a gyro count is 1/32 dps and an accelerometer count 1/2048 g, so a record taken from the bus is quantised; below about one count of noise the result is the quantisation's.

The datasheet figures are those recorded in docs/design/IMU_MODEL.md (DS13012 rev 7, Table 3): gyro 5 mdps/rtHz typical, 8 at most; accelerometer 60 ug/rtHz typical, 100 at most; gyro bias instability 3 deg/h.
The tool reports and, with --strict, exits 1 if the noise density is above the datasheet's limit.
"""
from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path

LINE = re.compile(r"^\((\d+)\.(\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$")
GYRO_LSB = 1.0 / 32.0
ACCEL_LSB = 1.0 / 2048.0
BASE = {"gyro": 0x100, "accel": 0x110}
LSB = {"gyro": GYRO_LSB, "accel": ACCEL_LSB}
# (typical, limit) noise density in the unit of the sensor per root hertz: dps for the gyro, g for the accelerometer
DATASHEET = {"gyro": (5.0e-3, 8.0e-3), "accel": (60.0e-6, 100.0e-6)}
UNIT = {"gyro": "dps", "accel": "g"}
BI_GYRO_TYPICAL_DPS = 3.0 / 3600.0   # 3 deg/h


def allan_deviation(x: list[float], rate: float, max_clusters: int = 9) -> list[tuple[float, float, int]]:
    """Overlapping Allan deviation of `x` (a rate, in units/s, sampled at `rate` Hz) at tau = m / rate for m = 1, 2, 4, ... while at least `max_clusters` averaging intervals fit.
    Returns (tau, sigma, number of overlapping differences)."""
    n = len(x)
    theta = [0.0] * (n + 1)
    acc = 0.0
    for i, v in enumerate(x):
        acc += v / rate
        theta[i + 1] = acc
    out = []
    m = 1
    while n // m >= max_clusters and n - 2 * m > 0:
        tau = m / rate
        terms = n + 1 - 2 * m
        s = 0.0
        a, b, c = iter(theta), iter(theta[m:]), iter(theta[2 * m:])
        for t0, t1, t2 in zip(a, b, c):
            d = t2 - 2.0 * t1 + t0
            s += d * d
        out.append((tau, math.sqrt(s / (2.0 * tau * tau * terms)), terms))
        m *= 2
    return out


def white_noise_density(curve: list[tuple[float, float, int]]) -> float | None:
    """The one-sided noise density (units per root hertz) from sigma at the tau nearest to 1 s on the slope -1/2: sigma(tau) = n / sqrt(2 tau)."""
    if not curve:
        return None
    tau, sigma, _n = min(curve, key=lambda p: abs(math.log(p[0])))
    return sigma * math.sqrt(2.0 * tau)


def bias_instability(curve: list[tuple[float, float, int]]) -> tuple[float, float] | None:
    """(bias instability, the tau of the minimum): the minimum of sigma divided by 0.664 (the flicker-noise floor), if the curve turns up after it."""
    if len(curve) < 4:
        return None
    k = min(range(len(curve)), key=lambda i: curve[i][1])
    if k == len(curve) - 1:
        return None   # still falling at the longest tau: the record is too short to see the floor
    return curve[k][1] / 0.664, curve[k][0]


def slope(curve: list[tuple[float, float, int]], i: int, j: int) -> float:
    return math.log(curve[j][1] / curve[i][1]) / math.log(curve[j][0] / curve[i][0])


def read_log(path: Path, node: int, sensor: str) -> tuple[list[list[float]], int, int]:
    """The three axes of a node's gyro or accelerometer frames, in order; the number of frames missing between them (by the frame number each carries), and the number read."""
    want = BASE[sensor] + node
    axes: list[list[float]] = [[], [], []]
    last = None
    gaps = 0
    for line in path.read_text().splitlines():
        m = LINE.match(line.strip())
        if not m or int(m.group(3), 16) != want:
            continue
        d = bytes.fromhex(m.group(4))
        if len(d) != 8:
            continue
        seq = d[6]
        if last is not None:
            step = (seq - last) & 0xFF
            if step == 0:
                continue   # a repeated frame is not a new sample
            if step > 1:   # frames missing: hold the last value for each, so that the record keeps its time base
                gaps += step - 1
                for _ in range(step - 1):
                    for a in range(3):
                        axes[a].append(axes[a][-1])
        last = seq
        for a in range(3):
            v = int.from_bytes(d[2 * a:2 * a + 2], "little", signed=True)
            axes[a].append(v * LSB[sensor])
    return axes, gaps, len(axes[0])


def read_csv(path: Path, column: int) -> list[float]:
    with path.open() as fh:
        rows = [r for r in csv.reader(fh) if r]
    out = []
    for r in rows:
        try:
            out.append(float(r[column]))
        except (ValueError, IndexError):
            continue   # a header
    return out


def report(name: str, x: list[float], rate: float, sensor: str) -> tuple[float | None, list[str]]:
    mean = sum(x) / len(x)
    rms = math.sqrt(sum((v - mean) ** 2 for v in x) / len(x))
    curve = allan_deviation(x, rate)
    lines = [f"{name}: {len(x)} samples at {rate:g} Hz ({len(x) / rate / 3600.0:.2f} h), mean {mean:+.5f} {UNIT[sensor]}, rms about the mean {rms:.5f} {UNIT[sensor]}"]
    lines.append("    tau (s)   sigma (" + UNIT[sensor] + ")   differences")
    for tau, sigma, n in curve:
        lines.append(f"    {tau:8.2f}  {sigma:.6g}  {n:>10}")
    nd = white_noise_density(curve)
    if nd is not None:
        typ, lim = DATASHEET[sensor]
        lines.append(f"    noise density {nd * 1e3:.3f} m{UNIT[sensor]}/rtHz (datasheet typical {typ * 1e3:.3f}, limit {lim * 1e3:.3f}): {nd / typ:.2f}x typical; " + ("within the limit" if nd <= lim else "ABOVE THE DATASHEET LIMIT"))
        quant = LSB[sensor] / math.sqrt(12.0)
        if rms < 2.0 * quant:
            lines.append(f"    note: the rms is within twice the quantisation of the bus ({quant:.5f} {UNIT[sensor]}, one count over root 12): the figure is the bus's, not the sensor's")
    bi = bias_instability(curve)
    if bi is not None:
        extra = f" (datasheet typical {BI_GYRO_TYPICAL_DPS * 3600.0:.0f} deg/h = {BI_GYRO_TYPICAL_DPS:.5f} dps)" if sensor == "gyro" else ""
        lines.append(f"    bias instability {bi[0]:.6g} {UNIT[sensor]} at tau {bi[1]:g} s{extra}")
    else:
        lines.append("    bias instability: not seen (the deviation is still falling at the longest tau; record for longer)")
    if len(curve) >= 6:
        lines.append(f"    slope over the first octaves {slope(curve, 0, 3):+.2f} (white noise is -0.50), over the last {slope(curve, len(curve) - 4, len(curve) - 1):+.2f} (a random walk is +0.50)")
    return nd, lines


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", type=Path, help="a candump -L log of the flight bus")
    ap.add_argument("--node", default="A", help="A, B or C: whose IMU frames to read from the log")
    ap.add_argument("--csv", type=Path, help="a table of samples instead")
    ap.add_argument("--column", type=int, default=0, help="the CSV column of the samples")
    ap.add_argument("--rate", type=float, default=100.0, help="the sample rate in Hz (the bus frames are 100 Hz)")
    ap.add_argument("--sensor", choices=["gyro", "accel"], default="gyro")
    ap.add_argument("--strict", action="store_true", help="exit 1 if a noise density is above the datasheet's limit")
    a = ap.parse_args(argv)
    if (a.log is None) == (a.csv is None):
        ap.error("give --log or --csv")
    worst = 0.0
    above = False
    if a.log is not None:
        node = "ABC".index(a.node.upper())
        axes, gaps, n = read_log(a.log, node, a.sensor)
        if n < 100:
            print(f"only {n} frames of node {a.node} {a.sensor} in {a.log}: not enough")
            return 2
        print(f"{a.log}: node {a.node} {a.sensor}, {n} frames, {gaps} missing (held)")
        for k, name in enumerate("xyz"):
            nd, lines = report(f"{a.sensor} {name}", axes[k], a.rate, a.sensor)
            print("\n".join(lines))
            if nd is not None:
                worst = max(worst, nd)
                above = above or nd > DATASHEET[a.sensor][1]
    else:
        x = read_csv(a.csv, a.column)
        if len(x) < 100:
            print(f"only {len(x)} samples: not enough")
            return 2
        nd, lines = report(f"{a.sensor} column {a.column}", x, a.rate, a.sensor)
        print("\n".join(lines))
        above = nd is not None and nd > DATASHEET[a.sensor][1]
    return 1 if a.strict and above else 0


if __name__ == "__main__":
    sys.exit(main())
