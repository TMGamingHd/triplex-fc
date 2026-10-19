#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""A day-of-launch look at the loads: flies a mission on each real day's winds and says GO or NO-GO against the loads it met (docs/design/FUTURE_WORK.md 6b item 15, ADR-053).

    python3 tools/weather/launch_day.py vehicles/missions/starship_return.json vehicles/weather/ksc-2026-10-15T10.json [more days] [--limit-q-sin-alpha-kpa 4] [--limit-q-kpa 35] [--limit-normal-force-mn 1.5]

The large launchers decide on the day's upper winds: a launch commit criterion on the shear and the maximum wind aloft, and a day-of-launch change to the pitch program (the Shuttle's day-of-launch I-load update was one). The
simulator already flies a day's balloon sounding (`vehicles/weather/*.json`, tools/weather/fetch_sounding.py); this puts the mission's loads next to it: for each day, the mission is flown (`tfc_mission`, the design on the nominal
wind and the flight on the day's) and its envelope read back: the maximum q, q sin(alpha) and the aerodynamic normal force of the main body on its way up. A day is **GO** when none is over its limit, **NO-GO** when one is,
and **NO LIMIT** when no limit is given: the limits are from the mission file's `structure` block, if it has one, or the options. They are not made up here: with none the tool reports the loads and nothing else.
It judges the main body's ascent (the stage that returns has its own envelope, and its descent is another question: ADR-053).
"""
import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def read_json_with_comments(path):
    """The vehicle files may carry `//` comment lines (the simulator's reader accepts them): drop whole-line ones and read the rest."""
    text = "\n".join(l for l in Path(path).read_text().split("\n") if not l.lstrip().startswith("//"))
    return json.loads(text)


def parse_envelope(output):
    """The main body's line of the envelope that tfc_mission prints: {'q_kpa', 'q_sin_alpha_kpa', 'normal_force_kn'} (None when it is not there), and what became of the bodies."""
    main = re.search(r"^main: .*\n\s+loads: max q ([\d.]+) kPa at T\+\d+ s; max q sin\(alpha\) ([\d.]+) kPa at T\+\d+ s .*?aero normal force up to (\d+) kN", output, re.M)
    env = {"q_kpa": float(main.group(1)), "q_sin_alpha_kpa": float(main.group(2)), "normal_force_kn": float(main.group(3))} if main else None
    booster = re.search(r"^booster: (.*)$", output, re.M)
    orbit = re.search(r"^main: .*perigee ([\d.]+) km", output, re.M)
    return env, (booster.group(1) if booster else None), (float(orbit.group(1)) if orbit else None)


def verdict(env, limits):
    """('GO' | 'NO-GO' | 'NO LIMIT' | 'NO DATA', [the reasons])."""
    if env is None:
        return "NO DATA", ["the mission's envelope was not in the output"]
    if not any(v for v in limits.values()):
        return "NO LIMIT", []
    why = []
    for key, name, unit in (("q_kpa", "max q", "kPa"), ("q_sin_alpha_kpa", "max q sin(alpha)", "kPa"), ("normal_force_kn", "aero normal force", "kN")):
        if limits.get(key) and env[key] > limits[key]:
            why.append(f"{name} {env[key]:.1f} {unit} is over the limit of {limits[key]:.1f} {unit}")
    return ("NO-GO" if why else "GO"), why


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mission")
    ap.add_argument("days", nargs="+")
    ap.add_argument("--bin", default=str(ROOT / "build" / "host" / "tfc_mission"))
    ap.add_argument("--limit-q-kpa", type=float, default=0.0)
    ap.add_argument("--limit-q-sin-alpha-kpa", type=float, default=0.0)
    ap.add_argument("--limit-normal-force-mn", type=float, default=0.0)
    ap.add_argument("--pad", type=int, default=1500)
    a = ap.parse_args(argv)
    base = read_json_with_comments(a.mission)
    file_limits = (base.get("mission", {}).get("structure", {}) or {}).get("main", {}) or {}
    limits = {"q_kpa": a.limit_q_kpa or file_limits.get("max_q_pa", 0) / 1000.0, "q_sin_alpha_kpa": a.limit_q_sin_alpha_kpa or file_limits.get("max_q_sin_alpha_pa", 0) / 1000.0,
              "normal_force_kn": a.limit_normal_force_mn * 1000.0}
    print(f"{'day':<34} {'wind aloft':>10} {'max q':>9} {'q sin(a)':>9} {'normal F':>10}  {'verdict'}")
    worst = 0
    for day in a.days:
        w = json.loads(Path(day).read_text())
        data = json.loads(json.dumps(base))
        sc = data.setdefault("scenario", {})
        sc.pop("wind_profile", None)
        sc["wind_profile_enu"] = w["wind_profile_enu"]
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(data, f)
            tmp = f.name
        try:
            r = subprocess.run([a.bin, tmp, "--pad", str(a.pad)], capture_output=True, text=True, check=False)
        finally:
            Path(tmp).unlink(missing_ok=True)
        env, booster, perigee = parse_envelope(r.stdout)
        v, why = verdict(env, limits)
        peak = max((float(x[1]) ** 2 + float(x[2]) ** 2) ** 0.5 for x in w["wind_profile_enu"]) if w["wind_profile_enu"] else 0.0
        if env:
            print(f"{Path(day).stem:<34} {peak:8.1f} m/s {env['q_kpa']:7.1f} kPa {env['q_sin_alpha_kpa']:7.2f} kPa {env['normal_force_kn']:7.0f} kN  {v}" + (f"  ({'; '.join(why)})" if why else ""))
        else:
            print(f"{Path(day).stem:<34} {peak:8.1f} m/s  no envelope in the output (tfc_mission exit {r.returncode}): {v}")
        worst = max(worst, {"GO": 0, "NO LIMIT": 0, "NO-GO": 3, "NO DATA": 2}[v])
    return worst


if __name__ == "__main__":
    sys.exit(main())
