# SPDX-License-Identifier: MIT
"""Earth imagery for the 3D viewer: where it comes from, how it is fetched, and how it is served.

Nothing of it is in the repository. `console/tfc-imagery` downloads a pack into `console/imagery/` (git-ignored) from NASA's GIBS web map service (https://gibs.earthdata.nasa.gov, no key, NASA imagery is
public domain), and writes `manifest.json` beside it saying what each file is, where it is on the Earth, and where it came from. The viewer reads that manifest; with no pack it draws the procedural planet
it always drew.

The layers, and why these:

* **earth-4k, earth-8k**: *Blue Marble Next Generation*, a cloud-free monthly composite of MODIS at 500 m, as one equirectangular picture. 4,096 x 2,048 is 9.8 km a pixel at the equator, 8,192 x 4,096 is
  4.9 km; both are fine from orbit and blurred from a few hundred kilometres down. (The clouds the viewer draws are its own, so a cloud-free surface is the right one to put under them.)
* **night-4k**: *Black Marble*, the lights of the night side (VIIRS), at the same 4,096 x 2,048.
* **site-ID-regional, site-ID-local**: *Landsat WELD*, the annual true-colour composite of Landsat at 30 m, over the United States only (a launch site there): 4 x 4 degrees (about 440 km, 108 m a pixel) and
  1 x 1 degree (about 100 km, 27 m a pixel, which is the sensor's own resolution) around the site. The composite used is the one stamped 2000-12-01: the service's time dimension lists several years,
  and the two others tried over Florida (2008 and 2010) came back empty (black), so the picture is of the year 2000, and the manifest says so. A site outside the United States has the global layers alone.

A picture here is a *picture of the ground*: it is not terrain (there is no height), it carries the season and the sea state of the day it was taken, and the sea's colour in it is the satellite's, so the
viewer lights the sea with its own sun glint and only uses the picture's colour.
"""
from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

GIBS = "https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi"
WELD_TIME = "2000-12-01"           # the annual composite that has pixels over Florida (2008 and 2010 are empty)
ATTRIBUTION_NASA = "NASA Global Imagery Browse Services (GIBS), Earth Observing System Data and Information System (EOSDIS); NASA imagery is in the public domain."

# a launch site: id -> (name, latitude, longitude, has Landsat WELD)
SITES: dict[str, tuple[str, float, float, bool]] = {
    "ksc": ("Kennedy Space Center, pad 39A", 28.6083, -80.6041, True),
    "starbase": ("Starbase, Boca Chica", 25.9972, -97.1566, True),
    "vsfb": ("Vandenberg SFB, SLC-4E", 34.6321, -120.6107, True),
    "wallops": ("Wallops Island", 37.8340, -75.4880, True),
    "kourou": ("Kourou", 5.2360, -52.7750, False),
    "baikonur": ("Baikonur", 45.9650, 63.3050, False),
}


def site_for_latitude(lat_deg: float) -> str:
    """The site whose latitude is nearest (the simulator's launch point has a latitude and no longitude: the picture is placed at the nearest known site)."""
    return min(SITES, key=lambda k: abs(SITES[k][1] - lat_deg))


def wms_url(layer: str, box: tuple[float, float, float, float], width: int, height: int, time_: str | None = None, fmt: str = "image/jpeg") -> str:
    """A GetMap request for `layer`; box is (lon_min, lat_min, lon_max, lat_max) in degrees. WMS 1.3.0 with EPSG:4326 takes the box as latitude first."""
    lon0, lat0, lon1, lat1 = box
    q = {"SERVICE": "WMS", "REQUEST": "GetMap", "VERSION": "1.3.0", "STYLES": "", "FORMAT": fmt, "CRS": "EPSG:4326", "LAYERS": layer,
         "BBOX": f"{lat0:.6f},{lon0:.6f},{lat1:.6f},{lon1:.6f}", "WIDTH": str(width), "HEIGHT": str(height)}
    if time_:
        q["TIME"] = time_
    return GIBS + "?" + urllib.parse.urlencode(q, safe=":,")


def site_box(site: str, half_deg: float) -> tuple[float, float, float, float]:
    _n, lat, lon, _w = SITES[site]
    return (lon - half_deg, lat - half_deg, lon + half_deg, lat + half_deg)


def layer_plan(sites: list[str]) -> list[dict]:
    """What a pack holds: each layer with its request and its entry in the manifest."""
    plan = [
        {"id": "earth-4k", "kind": "day", "file": "earth_4k.jpg", "box": (-180, -90, 180, 90), "px": (4096, 2048), "layer": "BlueMarble_NextGeneration", "time": None,
         "what": "Blue Marble Next Generation, cloud-free, 500 m source", "texel_km": 40075.0 / 4096},
        {"id": "earth-8k", "kind": "day", "file": "earth_8k.jpg", "box": (-180, -90, 180, 90), "px": (8192, 4096), "layer": "BlueMarble_NextGeneration", "time": None,
         "what": "Blue Marble Next Generation, cloud-free, 500 m source", "texel_km": 40075.0 / 8192},
        {"id": "night-4k", "kind": "night", "file": "night_4k.jpg", "box": (-180, -90, 180, 90), "px": (4096, 2048), "layer": "VIIRS_Black_Marble", "time": None,
         "what": "Black Marble, the lights of the night side", "texel_km": 40075.0 / 4096},
    ]
    for s in sites:
        if s not in SITES:
            raise ValueError(f"unknown site {s!r}: one of {', '.join(SITES)}")
        if not SITES[s][3]:
            continue
        for tag, half, px in (("regional", 2.0, 4096), ("local", 0.5, 4096)):
            box = site_box(s, half)
            km = 2 * half * 111.0 / px
            plan.append({"id": f"site-{s}-{tag}", "kind": "patch", "site": s, "tier": tag, "file": f"{s}_{tag}.jpg", "box": box, "px": (px, px), "layer": "Landsat_WELD_CorrectedReflectance_TrueColor_Global_Annual",
                         "time": WELD_TIME, "what": f"Landsat WELD annual composite ({WELD_TIME[:4]}), 30 m source, around {SITES[s][0]}", "texel_km": km})
    return plan


def fetch(dest: Path, sites: list[str], log=print, timeout: float = 180.0, only: set[str] | None = None) -> dict:
    """Download a pack into `dest` and write its manifest. Layers already there are kept (delete the file to fetch it again)."""
    dest.mkdir(parents=True, exist_ok=True)
    mpath = dest / "manifest.json"
    have = {}
    if mpath.is_file():
        try:
            have = {l["id"]: l for l in json.loads(mpath.read_text()).get("layers", [])}
        except (OSError, ValueError):
            have = {}
    layers = dict(have)
    for item in layer_plan(sites):
        if only and item["id"] not in only:
            continue
        path = dest / item["file"]
        if not path.is_file() or path.stat().st_size < 2000:
            url = wms_url(item["layer"], item["box"], item["px"][0], item["px"][1], item["time"])
            log(f"  {item['id']}: {item['what']} ({item['px'][0]} x {item['px'][1]})")
            t0 = time.monotonic()
            try:
                with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "tfc-imagery/1"}), timeout=timeout) as r:
                    body = r.read()
                    ctype = r.headers.get("Content-Type", "")
            except (urllib.error.URLError, OSError) as e:
                raise RuntimeError(f"{item['id']}: the service did not answer: {e}") from e
            if not ctype.startswith("image/") or len(body) < 2000:
                raise RuntimeError(f"{item['id']}: the service answered with {ctype or 'nothing'}, {len(body)} bytes: {body[:200]!r}")
            path.write_bytes(body)
            log(f"      {len(body) / 1e6:.2f} MB in {time.monotonic() - t0:.1f} s")
        entry = {k: item[k] for k in ("id", "kind", "file", "what", "texel_km")}
        entry.update(box=list(item["box"]), px=list(item["px"]), bytes=path.stat().st_size, source=f"{item['layer']}" + (f", time {item['time']}" if item["time"] else ""), attribution=ATTRIBUTION_NASA)
        for k in ("site", "tier"):
            if k in item:
                entry[k] = item[k]
        layers[item["id"]] = entry
    manifest = {"version": 1, "fetched": time.strftime("%Y-%m-%d"), "sites": {k: {"name": v[0], "lat": v[1], "lon": v[2], "weld": v[3]} for k, v in SITES.items()}, "layers": list(layers.values())}
    mpath.write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


class ImageryPack:
    """The pack on disk, as the viewer's routes serve it: the manifest, and the files the manifest names (and no others)."""

    def __init__(self, directory: Path) -> None:
        self.dir = Path(directory)

    def manifest(self) -> dict:
        p = self.dir / "manifest.json"
        if not p.is_file():
            return {"version": 1, "layers": [], "sites": {k: {"name": v[0], "lat": v[1], "lon": v[2], "weld": v[3]} for k, v in SITES.items()}, "present": False}
        try:
            m = json.loads(p.read_text())
        except (OSError, ValueError):
            return {"version": 1, "layers": [], "sites": {}, "present": False, "error": "manifest.json cannot be read"}
        m["layers"] = [l for l in m.get("layers", []) if isinstance(l, dict) and (self.dir / str(l.get("file", ""))).is_file()]
        m["present"] = bool(m["layers"])
        return m

    def file_bytes(self, name: str) -> tuple[bytes, str]:
        for l in self.manifest()["layers"]:
            if l["file"] == name:
                return (self.dir / name).read_bytes(), "image/jpeg"
        raise ValueError(f"no imagery file named {name!r} in console/imagery/")


def main(argv: list[str] | None = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="tfc-imagery", description="Download the Earth imagery pack for the 3D viewer from NASA GIBS (public domain) into console/imagery/.")
    ap.add_argument("--site", action="append", help=f"a launch site to fetch Landsat patches for (repeatable; one of {', '.join(SITES)}; default ksc and starbase)")
    ap.add_argument("--dest", type=Path, default=Path(__file__).resolve().parent.parent / "imagery")
    ap.add_argument("--only", action="append", help="only this layer id (repeatable), e.g. earth-4k")
    args = ap.parse_args(argv)
    sites = args.site or ["ksc", "starbase"]
    print(f"fetching the imagery pack into {args.dest}\n{ATTRIBUTION_NASA}")
    try:
        m = fetch(args.dest, sites, only=set(args.only) if args.only else None)
    except (RuntimeError, ValueError) as e:
        print("error:", e, file=sys.stderr)
        return 1
    total = sum(l["bytes"] for l in m["layers"])
    print(f"done: {len(m['layers'])} layers, {total / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
