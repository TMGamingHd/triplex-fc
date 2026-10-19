#!/usr/bin/env python3
"""The grid-convergence index (Roache, 1994 and 1998; Celik et al., 2008, from memory) of a solver's result on three grids (docs/design/CFD.md section 15; docs/design/FUTURE_WORK.md 6a item 7).

A result f on three grids of cell sizes h1 < h2 < h3 with a constant ratio r = h2/h1 = h3/h2: the observed order of convergence p = ln((f3 - f2) / (f2 - f1)) / ln r, the Richardson extrapolation to a cell of
zero size f_inf = f1 + (f1 - f2) / (r^p - 1), and the index GCI = Fs |(f2 - f1) / f1| / (r^p - 1) with the safety factor Fs = 1.25 for three grids: an error band on the fine-grid result. When the
differences do not shrink (or change sign) the result is not in the asymptotic range, which the tool says instead of giving a number.

    python3 tools/cfd/gci.py FINE.json MEDIUM.json COARSE.json [--ratio 2] [--keys cn,ca_skin,cm,x_cp_m]
"""
import argparse
import json
import math
import sys


def gci(f1, f2, f3, r=2.0, fs=1.25):
    """The result on the fine (f1), medium (f2) and coarse (f3) grid -> a dict with p, f_inf, gci (a fraction of f1) and `asymptotic`: whether the three results converge monotonically."""
    d21 = f2 - f1
    d32 = f3 - f2
    if d21 == 0.0 or d32 == 0.0 or (d21 > 0) != (d32 > 0) or abs(d32) <= abs(d21):
        return {"asymptotic": False, "p": None, "f_inf": None, "gci": None, "d21": d21, "d32": d32}
    p = math.log(d32 / d21) / math.log(r)
    f_inf = f1 + (f1 - f2) / (r ** p - 1.0)
    index = fs * abs(d21 / f1) / (r ** p - 1.0) if f1 != 0.0 else None
    return {"asymptotic": True, "p": p, "f_inf": f_inf, "gci": index, "d21": d21, "d32": d32}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fine")
    ap.add_argument("medium")
    ap.add_argument("coarse")
    ap.add_argument("--ratio", type=float, default=2.0)
    ap.add_argument("--keys", default="cn,ca_skin,cm,x_cp_m")
    a = ap.parse_args(argv)
    runs = [json.load(open(p)) for p in (a.fine, a.medium, a.coarse)]
    print(f"cells {runs[0].get('cells')}, {runs[1].get('cells')}, {runs[2].get('cells')} (cell {runs[0].get('h_m')}, {runs[1].get('h_m')}, {runs[2].get('h_m')} m)")
    print(f"{'quantity':<10} {'fine':>10} {'medium':>10} {'coarse':>10} {'p':>6} {'extrapolated':>13} {'GCI of fine':>12}")
    for k in a.keys.split(","):
        v = [run.get(k) for run in runs]
        if any(x is None for x in v):
            continue
        g = gci(v[0], v[1], v[2], a.ratio)
        if g["asymptotic"]:
            print(f"{k:<10} {v[0]:10.4f} {v[1]:10.4f} {v[2]:10.4f} {g['p']:6.2f} {g['f_inf']:13.4f} {100 * g['gci']:11.2f} %")
        else:
            print(f"{k:<10} {v[0]:10.4f} {v[1]:10.4f} {v[2]:10.4f}   not in the asymptotic range (the differences do not shrink)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
