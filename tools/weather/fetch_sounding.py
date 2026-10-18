#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes a real day's wind at a launch site: a weather-balloon sounding, as a file the simulator can fly in (vehicles/weather/*.json).

    python3 tools/weather/fetch_sounding.py --site starbase --when 2026-10-14T00 [--out vehicles/weather/starbase-2026-10-14T00.json]
    python3 tools/weather/fetch_sounding.py --site ksc --latest
    python3 tools/weather/fetch_sounding.py --site vsfb --list           # the soundings the data holds, newest last

The data is NOAA NCEI's Integrated Global Radiosonde Archive (IGRA v2): the balloon's own measurements, one station per site (Starbase has no balloon of its own: the nearest is Brownsville, 35 km
away; the others are at or next to the pad). A balloon climbs to 20 to 35 km and gives the wind's speed and the direction it blows FROM at every pressure it reports. This script turns that into
the east and north components the wind blows TOWARD, at the altitude above the launch point, which is `scenario.wind_profile_enu` in a vehicle file (docs/design/ENVIRONMENT.md section 5).

What it does not do: invent. A level without a wind is dropped; a level with a wind but no height (most of them) gets its height from the levels that have one, by the logarithm of the pressure, and
the file says how many were; above the balloon's last level the file ends and the simulator holds the last wind and thins it with the air's density (`Vehicle6::air_presence`).

The network is needed once per station and year: the archive's file is cached in $XDG_CACHE_HOME/tfc-weather (default ~/.cache/tfc-weather) and read from there afterwards.
"""
from __future__ import annotations

import argparse
import datetime as dt
import io
import json
import math
import os
import re
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = "https://www.ncei.noaa.gov/data/integrated-global-radiosonde-archive/access"
ATTRIBUTION = "NOAA National Centers for Environmental Information, Integrated Global Radiosonde Archive version 2 (Durre et al. 2016); public data."
R_GEOPOTENTIAL = 6356766.0     # m, the radius that turns a geopotential height into a geometric one (the 1976 standard's)

# a launch site: id -> (name, IGRA station, how far the balloon is from the pad, said in the file)
SITES = {
    "ksc": ("Kennedy Space Center / Cape Canaveral", "USM00074794", "Cape Kennedy station, at the Cape"),
    "starbase": ("Starbase, Boca Chica", "USM00072250", "Brownsville, about 35 km from the pad"),
    "vsfb": ("Vandenberg SFB", "USM00072393", "Vandenberg AFB station, at the base"),
    "wallops": ("Wallops Island", "USM00072402", "Wallops Island station, at the base"),
}


def cache_dir() -> Path:
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    d = Path(base) / "tfc-weather"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _get(url: str, timeout: float = 120.0) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "tfc-weather/1 (flight computer simulator)"})
    with urllib.request.urlopen(req, timeout=timeout) as r:  # noqa: S310 - a fixed https host
        return r.read()


def station_file(station: str, year: int) -> bytes:
    """The text of the station's soundings for `year` and the one before it: the year-to-date file when it covers `year`, else the whole period of record (large: tens of megabytes)."""
    cache = cache_dir()
    listing = _get(f"{BASE}/data-y2d/").decode("utf-8", "replace")
    m = re.search(rf"{station}-data-beg(\d{{4}})\.txt\.zip", listing)
    if m and year >= int(m.group(1)):
        name, url = f"{station}-y2d-{m.group(1)}.zip", f"{BASE}/data-y2d/{m.group(0)}"
        # the year-to-date file changes daily: use today's copy
        stamp = cache / f"{name}.{dt.date.today().isoformat()}"
        if not stamp.exists():
            stamp.write_bytes(_get(url))
        raw = stamp.read_bytes()
    else:
        name, url = f"{station}-por.zip", f"{BASE}/data-por/{station}-data.txt.zip"
        path = cache / name
        if not path.exists():
            print(f"downloading the whole period of record for {station} (a large file, once)...", file=sys.stderr)
            path.write_bytes(_get(url, 900.0))
        raw = path.read_bytes()
    z = zipfile.ZipFile(io.BytesIO(raw))
    return z.read(z.namelist()[0]).decode("ascii", "replace")


def parse(text: str):
    """Yields (header dict, [level dicts]) for each sounding. A level: lvl1, lvl2, p_pa, gph_m, wdir_deg, wspd_ms (None for a missing or removed value)."""
    head, levels = None, []
    for line in text.splitlines():
        if line.startswith("#"):
            if head is not None:
                yield head, levels
            f = line.split()
            head = {"id": f[0][1:], "year": int(f[1]), "month": int(f[2]), "day": int(f[3]), "hour": int(f[4]), "reltime": int(f[5]), "numlev": int(f[6])}
            levels = []
        elif head is not None and len(line) >= 51:
            def num(a: int, b: int, scale: float = 1.0):
                s = line[a - 1:b].strip()
                try:
                    v = int(s)
                except ValueError:
                    return None
                return None if v in (-9999, -8888) else v * scale
            levels.append({"lvl1": int(line[0]), "lvl2": int(line[1]), "p_pa": num(10, 15), "gph_m": num(17, 21), "temp_c": num(23, 27, 0.1),
                           "wdir_deg": num(41, 45), "wspd_ms": num(47, 51, 0.1)})
    if head is not None:
        yield head, levels


def sounding_time(h: dict) -> dt.datetime:
    """The time of the sounding: the release time when it is given, else the nominal hour."""
    rt = h["reltime"]
    if 0 <= rt <= 2359 and rt % 100 < 60:
        return dt.datetime(h["year"], h["month"], h["day"], rt // 100, rt % 100, tzinfo=dt.timezone.utc)
    return dt.datetime(h["year"], h["month"], h["day"], min(h["hour"], 23) if h["hour"] != 99 else 0, 0, tzinfo=dt.timezone.utc)


def _geometric(h_gp: float) -> float:
    return R_GEOPOTENTIAL * h_gp / (R_GEOPOTENTIAL - h_gp)


def profile(levels: list[dict]):
    """The wind profile of one sounding: ([ [altitude above the surface m, east m/s, north m/s], ... ], notes). East and north are where the wind blows TOWARD."""
    ref = sorted(((l["p_pa"], l["gph_m"]) for l in levels if l["p_pa"] and l["gph_m"] is not None), reverse=True)   # (p, z) from the ground up
    surface = next((l["gph_m"] for l in levels if l["lvl2"] == 1 and l["gph_m"] is not None), None)
    if surface is None and ref:
        surface = ref[0][1]
    if surface is None:
        return [], {"reason": "no height at the surface"}
    interpolated = 0

    def height(l: dict):
        nonlocal interpolated
        if l["gph_m"] is not None:
            return l["gph_m"]
        p = l["p_pa"]
        if not p or len(ref) < 2 or p > ref[0][0] or p < ref[-1][0]:
            return None
        for (p0, z0), (p1, z1) in zip(ref, ref[1:]):
            if p0 >= p >= p1 and p0 > p1:
                interpolated += 1
                return z0 + (z1 - z0) * (math.log(p0) - math.log(p)) / (math.log(p0) - math.log(p1))
        return None

    pts = []
    for l in levels:
        if l["wspd_ms"] is None or l["wdir_deg"] is None or not (0 <= l["wdir_deg"] <= 360):
            continue
        z = height(l)
        if z is None:
            continue
        d = math.radians(l["wdir_deg"])
        s = l["wspd_ms"]
        pts.append((_geometric(z) - _geometric(surface), -s * math.sin(d), -s * math.cos(d)))
    pts.sort()
    out: list[list[float]] = []
    for z, e, n in pts:
        z = max(z, 0.0)
        if out and z - out[-1][0] < 1.0:
            out[-1] = [out[-1][0], round(e, 2), round(n, 2)]   # two reports within a metre: the later one
        else:
            out.append([round(z, 1), round(e, 2), round(n, 2)])
    return out, {"interpolated_heights": interpolated, "surface_gph_m": surface}


def pick(text: str, when: dt.datetime | None):
    """The sounding nearest `when` (the newest one with a wind profile when None), within 12 hours."""
    best = None
    for h, levels in parse(text):
        t = sounding_time(h)
        prof, notes = profile(levels)
        if len(prof) < 8:
            continue
        key = abs((t - when).total_seconds()) if when else -t.timestamp()
        if best is None or key < best[0]:
            best = (key, t, h, prof, notes)
    if best is None or (when and best[0] > 12 * 3600):
        return None
    return best[1:]


def build(site: str, t: dt.datetime, h: dict, prof: list, notes: dict) -> dict:
    name, station, where = SITES[site]
    top = prof[-1][0]
    speeds = [math.hypot(e, n) for _z, e, n in prof]
    return {
        "format": "tfc-weather-1",
        "site": site,
        "site_name": name,
        "station": station,
        "station_note": where,
        "time_utc": t.strftime("%Y-%m-%dT%H:%MZ"),
        "source": "NOAA NCEI IGRA v2 radiosonde (a balloon's own measurements)",
        "attribution": ATTRIBUTION,
        "levels": len(prof),
        "top_altitude_m": top,
        "max_wind_ms": round(max(speeds), 1),
        "heights_interpolated_from_pressure": notes.get("interpolated_heights", 0),
        "note": "wind_profile_enu is [altitude above the surface m, wind TOWARD the east m/s, TOWARD the north m/s]. Above the last level the simulator holds the last wind and thins it with the air's density.",
        "wind_profile_enu": prof,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--site", required=True, choices=sorted(SITES))
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--when", help="UTC time, YYYY-MM-DD[THH]: the sounding nearest to it (within 12 hours)")
    g.add_argument("--latest", action="store_true", help="the newest sounding")
    g.add_argument("--list", action="store_true", help="list the soundings in this year's data")
    ap.add_argument("--out", help="the file to write (default vehicles/weather/<site>-<time>.json)")
    a = ap.parse_args(argv)
    when = None
    if a.when:
        when = dt.datetime.fromisoformat(a.when if "T" in a.when else a.when + "T00").replace(tzinfo=dt.timezone.utc)
    year = (when or dt.datetime.now(dt.timezone.utc)).year
    try:
        text = station_file(SITES[a.site][1], year)
    except (OSError, ValueError, zipfile.BadZipFile) as e:      # no network, the archive down, a file that is not what it was
        print(f"cannot get the soundings of {SITES[a.site][1]} ({e.__class__.__name__}: {e}); a file already in {cache_dir()} is used only when it is today's copy", file=sys.stderr)
        return 1
    if a.list:
        for h, levels in parse(text):
            prof, _n = profile(levels)
            if len(prof) >= 8:
                print(f"{sounding_time(h):%Y-%m-%dT%H:%MZ}  {len(prof):4d} levels  to {prof[-1][0] / 1000:5.1f} km")
        return 0
    got = pick(text, when)
    if got is None:
        print("no sounding with a wind profile near that time", file=sys.stderr)
        return 1
    t, h, prof, notes = got
    doc = build(a.site, t, h, prof, notes)
    out = Path(a.out) if a.out else ROOT / "vehicles" / "weather" / f"{a.site}-{t:%Y-%m-%dT%H}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"{out}: {doc['site_name']}, {doc['time_utc']}, {doc['levels']} levels to {doc['top_altitude_m'] / 1000:.1f} km, strongest wind {doc['max_wind_ms']} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
