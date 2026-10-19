#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Runs the Euler solver (tfc_cfd) over Mach number and angle of attack, from 0 to 180 degrees, on the body of one configuration of a vehicle, and writes the table the plant reads (`aero.cfd` of a vehicle file;
docs/design/CFD.md section 14).

    python3 tools/cfd/make_aerodb.py vehicles/starship.json --name stack --stages 0,1 --stage-from 0
    python3 tools/cfd/make_aerodb.py vehicles/starship.json --name ship --stages 1 --stage-from 1
    python3 tools/cfd/make_aerodb.py vehicles/starship.json --name booster --stages 0 --stage-from 0 --stage-to 0

The cases are cached one by one in --cache (default build/aerodb-cache), so that an interrupted run goes on where it stopped and a change of the table's grid costs only the new cases. From Mach 8 up the solver
runs in the real gas (vibrating, dissociating air in equilibrium) at the altitude a lifting entry has at that Mach number (REAL_GAS_ALTITUDE_KM, an assumption of this script and not a trajectory of the
plan); below it the perfect gas. Friction and the base drag are not in the table: the plant adds them. A case is about a minute on twelve cores at the default 0.7 m cells.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MACHS = [2.0, 3.0, 5.0, 8.0, 12.0, 20.0]
ALPHAS = [0.0, 4.0, 10.0, 20.0, 35.0, 50.0, 70.0, 90.0, 110.0, 135.0, 160.0, 180.0]
# the altitude, km, of a lifting entry at each Mach number from which the real gas applies (a representative corridor: dynamic pressure rising to a few kPa, then falling)
REAL_GAS_ALTITUDE_KM = {8.0: 42.0, 12.0: 52.0, 20.0: 68.0}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("vehicle")
    ap.add_argument("--name", required=True)
    ap.add_argument("--stage-from", type=int, default=0)
    ap.add_argument("--stage-to", type=int, default=1000)
    ap.add_argument("--stages", required=True, help="the stages that are on the vehicle in this configuration, e.g. 0,1 (the stack), 1 (the ship), 0 (the booster)")
    ap.add_argument("--machs", default=",".join(str(m) for m in MACHS))
    ap.add_argument("--alphas", default=",".join(str(a) for a in ALPHAS))
    ap.add_argument("--h", type=float, default=0.7)
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--out", help="the table (default vehicles/aerodb/<vehicle name>_<name>.json)")
    ap.add_argument("--cache", default=str(ROOT / "build" / "aerodb-cache"))
    ap.add_argument("--bin", default=str(ROOT / "build" / "host" / "tfc_cfd"))
    a = ap.parse_args(argv)
    machs = [float(x) for x in a.machs.split(",")]
    alphas = [float(x) for x in a.alphas.split(",")]
    cache = Path(a.cache) / f"{Path(a.vehicle).stem}_{a.name}"
    cache.mkdir(parents=True, exist_ok=True)
    cases: dict[tuple[float, float], dict] = {}
    t_start = time.time()
    for m in machs:
        for al in alphas:
            f = cache / f"m{m:g}_a{al:g}_h{a.h:g}.json"
            if not f.exists():
                cmd = [a.bin, a.vehicle, "--mach", str(m), "--alpha", str(al), "--h", str(a.h), "--iters", str(a.iters), "--stage-from", str(a.stage_from), "--stage-to", str(a.stage_to), "--json", str(f)]
                if m in REAL_GAS_ALTITUDE_KM:
                    cmd += ["--gas", "real", "--altitude", str(REAL_GAS_ALTITUDE_KM[m])]
                t0 = time.time()
                r = subprocess.run(cmd, capture_output=True, text=True)
                if r.returncode != 0 or not f.exists():
                    print(f"Mach {m:g} alpha {al:g}: failed\n{r.stdout[-300:]}{r.stderr[-300:]}", file=sys.stderr)
                    return 1
                print(f"Mach {m:g} alpha {al:g}: {time.time() - t0:.0f} s ({(time.time() - t_start) / 60:.0f} min in)", flush=True)
            cases[(m, al)] = json.loads(f.read_text())
    any_case = next(iter(cases.values()))

    def matrix(key: str) -> list[list[float]]:
        return [[round(cases[(m, al)][key], 5) for al in alphas] for m in machs]

    db = {"stages": [int(x) for x in a.stages.split(",")], "x_ref_m": any_case["x_ref_m"], "s_ref_m2": any_case["s_ref_m2"], "d_ref_m": any_case["d_ref_m"], "mach": machs,
          "alpha_deg": alphas, "ca": matrix("ca_skin"), "ca_base": matrix("ca_base_euler"), "cn": matrix("cn"), "cm": matrix("cm")}
    if any(abs(c.get("ca_fore", 0.0)) > 1e-9 or abs(c.get("cm_fore", 0.0)) > 1e-9 for c in cases.values()):   # a flat forward end (the booster without the ship): kept apart, a windward face at small angles
        db["ca_fore"] = matrix("ca_fore")
        db["cm_fore"] = matrix("cm_fore")
    import re
    name = json.loads(re.sub(r"(?m)^\s*//.*$", "", Path(a.vehicle).read_text())).get("name", Path(a.vehicle).stem)
    out = Path(a.out) if a.out else ROOT / "vehicles" / "aerodb" / f"{Path(a.vehicle).stem}_{a.name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"vehicle": name, "note": f"tfc_cfd, Euler, body only, {a.h} m cells; real gas from Mach 8; no friction and no base drag", "h_m": a.h, "cfd": db}) + "\n")
    print(f"wrote {out}: {len(machs)} Mach numbers x {len(alphas)} angles, {(time.time() - t_start) / 60:.0f} min")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
