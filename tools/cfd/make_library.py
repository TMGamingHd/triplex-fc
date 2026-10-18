#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Runs the Euler solver (tfc_cfd) over a grid of Mach numbers and angles of attack on a vehicle's body, and keeps what the 3D viewer and the aero table need (docs/design/CFD.md section 7).

    python3 tools/cfd/make_library.py vehicles/starship.json [--machs 2,3,5,8,12] [--alphas 0,4,10] [--h 0.7] [--out console/cfd/starship] [--stage-from 0]

For each case it writes <out>/m<Mach>_a<alpha>.cfd (the flow field: the pressure on the skin and a box of the flow, in the form the viewer reads) and, when all of a Mach number's angles are done,
the line of the table: CA at no angle of attack, the normal-force slope and the centre of pressure from the smallest non-zero angle. <out>/library.json lists the cases with their coefficients.
A case that already has a file is not run again (--force to redo them). One case takes one to ten minutes on twelve cores at 0.7 m cells for a 9 m body (the solver takes every core).
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def header(path: Path) -> dict:
    with path.open("rb") as f:
        return json.loads(f.readline())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("vehicle")
    ap.add_argument("--machs", default="2,3,5,8,12")
    ap.add_argument("--alphas", default="0,4,10")
    ap.add_argument("--h", type=float, default=0.7, help="the core cell, m")
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--stage-from", type=int, default=0)
    ap.add_argument("--out", default=str(ROOT / "console" / "cfd" / "starship"))
    ap.add_argument("--bin", default=str(ROOT / "build" / "host" / "tfc_cfd"))
    ap.add_argument("--force", action="store_true", help="run the cases that already have a file again")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    machs = [float(x) for x in a.machs.split(",")]
    alphas = [float(x) for x in a.alphas.split(",")]
    lib = []
    for m in machs:
        for al in alphas:
            f = out / f"m{m:g}_a{al:g}.cfd"
            if a.force or not f.exists():
                t0 = time.time()
                cmd = [a.bin, a.vehicle, "--mach", str(m), "--alpha", str(al), "--h", str(a.h), "--iters", str(a.iters), "--stage-from", str(a.stage_from), "--field", str(f)]
                r = subprocess.run(cmd, capture_output=True, text=True)
                if r.returncode != 0 or not f.exists():
                    print(f"Mach {m:g} alpha {al:g}: failed\n{r.stdout[-400:]}{r.stderr[-400:]}", file=sys.stderr)
                    continue
                print(f"Mach {m:g} alpha {al:g}: {time.time() - t0:.0f} s", flush=True)
            h = header(f)
            lib.append({"file": f.name, "mach": h["mach"], "alpha_deg": h["alpha_deg"], "ca_skin": h["ca_skin"], "cn": h["cn"], "x_cp_m": h["x_cp_m"], "cells": h["cells"], "iterations": h["iterations"],
                        "residual_drop": h["residual_drop"], "h_m": h["h_m"], "d_ref_m": h["d_ref_m"], "stage_from": a.stage_from})
    table = []
    for m in machs:
        rows = {c["alpha_deg"]: c for c in lib if c["mach"] == m}
        if 0.0 in rows and any(al > 0 for al in rows):
            al = min(al for al in rows if al > 0)
            import math
            table.append({"mach": m, "ca": round(rows[0.0]["ca_skin"], 4), "cn_alpha": round(rows[al]["cn"] / math.radians(al), 4), "x_cp": round(rows[al]["x_cp_m"], 3)})
    (out / "library.json").write_text(json.dumps({"vehicle": a.vehicle, "stage_from": a.stage_from, "h_m": a.h, "cases": lib, "table": table}, indent=1) + "\n")
    print(f"{len(lib)} cases, {len(table)} table rows in {out / 'library.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
