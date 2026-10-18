# SPDX-License-Identifier: MIT
"""A real day's wind (tools/weather/fetch_sounding.py, vehicles/weather/*.json): the IGRA reader against a sounding written out by hand (the network is a stand-in: nothing here asks NOAA for anything),
the turn from "direction the wind blows from" to east and north components, heights found from the pressure where the report has none, and what the console does with a day of wind."""
import importlib.util
import json
import math
import shutil
import tempfile
import unittest
import datetime as dt
from pathlib import Path

from .console_support import ROOT  # first: puts console/ on the path
from tfc_console.vehicles import Vehicles

_spec = importlib.util.spec_from_file_location("fetch_sounding", ROOT / "tools" / "weather" / "fetch_sounding.py")
fs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fs)


def level(lt1, lt2, press, gph, temp, wdir, wspd):
    """One IGRA data record, in the archive's columns (docs: LVLTYP1 1, LVLTYP2 2, ETIME 4-8, PRESS 10-15, PFLAG 16, GPH 17-21, ZFLAG 22, TEMP 23-27, TFLAG 28, RH 29-33, DPDP 35-39, WDIR 41-45, WSPD 47-51)."""
    return f"{lt1}{lt2} {-9999:>5} {press:>6} {gph:>5} {temp:>5} {-9999:>5} {-9999:>5} {wdir:>5} {wspd:>5}"


SOUNDING = "\n".join([
    "#USM00072250 2026 01 26 12 1102   10 ncdc-gts ncdc-gts  259167  -974192",
    level(2, 1, 101000, 7, 150, 90, 41),       # the surface: 4.1 m/s from the east
    level(1, 0, 85000, 1500, 100, 180, 100),    # a standard level: 10 m/s from the south
    level(2, 0, 80000, -9999, 80, 270, 200),    # no height, between the two above and below by pressure
    level(1, 0, 50000, 5800, -200, 0, 300),     # 30 m/s from the north
    level(2, 0, 40000, -9999, -9999, 225, -9999),   # no wind: dropped
    level(1, 0, 30000, 9600, -400, 270, 700),   # 70 m/s from the west
    level(1, 0, 20000, 12000, -500, 270, 650),
    level(1, 0, 10000, 16300, -600, 270, 400),
    level(1, 0, 5000, 20500, -600, 250, 300),
    level(1, 0, 2000, 26500, -600, 250, 200),
    "#USM00072250 2026 01 27 00 2315   10 ncdc-gts ncdc-gts  259167  -974192",
    level(2, 1, 101000, 7, 150, 0, 0),
]) + "\n"


class Reader(unittest.TestCase):
    def test_the_header_and_the_levels_are_read_with_their_missing_values(self):
        sounds = list(fs.parse(SOUNDING))
        self.assertEqual(len(sounds), 2)
        head, levels = sounds[0]
        self.assertEqual((head["year"], head["month"], head["day"], head["reltime"]), (2026, 1, 26, 1102))
        self.assertEqual(len(levels), 10)
        self.assertEqual((levels[0]["lvl2"], levels[0]["p_pa"], levels[0]["gph_m"], levels[0]["wdir_deg"]), (1, 101000, 7, 90))
        self.assertAlmostEqual(levels[0]["wspd_ms"], 4.1)
        self.assertIsNone(levels[2]["gph_m"])
        self.assertIsNone(levels[4]["wspd_ms"])
        self.assertEqual(fs.sounding_time(head), dt.datetime(2026, 1, 26, 11, 2, tzinfo=dt.timezone.utc))

    def test_the_wind_blows_toward_the_opposite_of_the_direction_it_comes_from(self):
        prof, _notes = fs.profile(list(fs.parse(SOUNDING))[0][1])
        by_alt = {round(z): (e, n) for z, e, n in prof}
        self.assertAlmostEqual(by_alt[0][0], -4.1, places=2)       # from the east: toward the west
        self.assertAlmostEqual(by_alt[0][1], 0.0, places=2)
        south = min(prof, key=lambda p: abs(p[0] - 1500))
        self.assertAlmostEqual(south[2], 10.0, places=1)           # from the south: toward the north
        north = min(prof, key=lambda p: abs(p[0] - 5800))
        self.assertAlmostEqual(north[2], -30.0, places=1)          # from the north: toward the south
        west = min(prof, key=lambda p: abs(p[0] - 9600))
        self.assertAlmostEqual(west[1], 70.0, places=1)            # from the west: toward the east: the jet stream
        for z, e, n in prof:
            self.assertTrue(math.isfinite(e) and math.isfinite(n))

    def test_altitudes_are_above_the_ground_increase_and_a_level_without_a_wind_is_dropped(self):
        prof, notes = fs.profile(list(fs.parse(SOUNDING))[0][1])
        alts = [p[0] for p in prof]
        self.assertEqual(alts[0], 0.0)
        self.assertEqual(alts, sorted(alts))
        self.assertEqual(len(set(alts)), len(alts))
        self.assertEqual(len(prof), 9)                  # ten levels less the one with no wind
        self.assertEqual(notes["surface_gph_m"], 7)

    def test_a_height_missing_from_a_report_comes_from_the_levels_around_it_by_the_pressure(self):
        prof, notes = fs.profile(list(fs.parse(SOUNDING))[0][1])
        self.assertEqual(notes["interpolated_heights"], 1)
        # 80 kPa lies between 85 kPa (1,500 m) and 50 kPa (5,800 m): ln(85/80) / ln(85/50) of the way
        f = math.log(85000 / 80000) / math.log(85000 / 50000)
        expect = fs._geometric(1500 + f * (5800 - 1500)) - fs._geometric(7)
        got = min(prof, key=lambda p: abs(p[0] - expect))
        self.assertLess(abs(got[0] - expect), 1.0)
        self.assertAlmostEqual(got[1], 20.0, places=1)    # 20 m/s from the west: toward the east, +20

    def test_the_sounding_nearest_the_time_is_chosen_and_one_too_far_is_refused(self):
        t = dt.datetime(2026, 1, 26, 12, tzinfo=dt.timezone.utc)
        got = fs.pick(SOUNDING, t)           # the second sounding has one level and no profile: only the first can be chosen
        self.assertEqual(got[0], dt.datetime(2026, 1, 26, 11, 2, tzinfo=dt.timezone.utc))
        self.assertIsNone(fs.pick(SOUNDING, dt.datetime(2026, 1, 27, 12, tzinfo=dt.timezone.utc)))     # 25 hours away
        self.assertEqual(fs.pick(SOUNDING, None)[0], got[0])                                          # the newest with a profile

    def test_the_file_says_where_the_wind_comes_from_and_how_much_was_guessed(self):
        t, head, prof, notes = fs.pick(SOUNDING, None)
        doc = fs.build("starbase", t, head, prof, notes)
        self.assertEqual((doc["format"], doc["site"], doc["station"], doc["levels"]), ("tfc-weather-1", "starbase", "USM00072250", 9))
        self.assertEqual(doc["heights_interpolated_from_pressure"], 1)
        self.assertEqual(doc["wind_profile_enu"], prof)
        self.assertAlmostEqual(doc["max_wind_ms"], 70.0, places=1)
        self.assertIn("balloon", doc["source"])


class Console(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / "vehicles" / "weather").mkdir(parents=True)
        t, head, prof, notes = fs.pick(SOUNDING, None)
        (self.tmp / "vehicles" / "weather" / "day-one.json").write_text(json.dumps(fs.build("starbase", t, head, prof, notes)))
        self.v = Vehicles(self.tmp, self.tmp / "state")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_the_days_of_wind_are_listed(self):
        days = self.v.weather_list()
        self.assertEqual([d["name"] for d in days], ["day-one"])
        self.assertEqual(days[0]["max_wind_ms"], 70.0)

    def test_a_day_replaces_the_vehicles_wind_profile_and_leaves_the_rest(self):
        data = {"stages": [], "scenario": {"wind_scale": 1.5, "wind_profile": [[0, 1], [1000, 2]], "site": {"latitude_deg": 26}}}
        out = self.v.apply_knobs(data, {"scenario.weather": "day-one", "scenario.wind_scale": 2.0})
        self.assertNotIn("wind_profile", out["scenario"])
        self.assertEqual(len(out["scenario"]["wind_profile_enu"]), 9)
        self.assertEqual((out["scenario"]["wind_scale"], out["scenario"]["site"]), (2.0, {"latitude_deg": 26}))
        self.assertIn("wind_profile", data["scenario"])      # the file it was read from is not touched
        self.assertEqual(self.v.apply_knobs(data, {"scenario.weather": ""}), data)       # none chosen: as it was

    def test_a_name_that_is_not_a_day_is_refused(self):
        for bad in ("../../etc/passwd", "no-such-day", "a b"):
            with self.assertRaises(ValueError):
                self.v.apply_knobs({"stages": []}, {"scenario.weather": bad})


class Committed(unittest.TestCase):
    def test_every_committed_day_is_a_profile_the_simulator_can_fly(self):
        files = sorted((ROOT / "vehicles" / "weather").glob("*.json"))
        self.assertGreaterEqual(len(files), 1)
        for p in files:
            w = json.loads(p.read_text())
            prof = w["wind_profile_enu"]
            self.assertEqual(w["format"], "tfc-weather-1")
            self.assertEqual(prof[0][0], 0.0, p.name)
            self.assertTrue(all(b[0] > a[0] for a, b in zip(prof, prof[1:])), p.name)
            self.assertGreater(prof[-1][0], 15000.0, p.name)
            self.assertLess(max(math.hypot(e, n) for _z, e, n in prof), 120.0, p.name)    # a balloon's wind, not a unit error


if __name__ == "__main__":
    unittest.main()
