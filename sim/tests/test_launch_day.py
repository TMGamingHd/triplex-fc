# SPDX-License-Identifier: MIT
"""The day-of-launch loads tool (tools/weather/launch_day.py): the reading of tfc_mission's envelope, the verdict against limits, and the reading of a vehicle file with comments."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("launch_day", ROOT / "tools" / "weather" / "launch_day.py")
ld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ld)

SAMPLE = """starship-v3-class-return: x
main: still flying, in orbit: perigee 231.4 km, apogee 255.9 km, speed 7752.2 m/s
  loads: max q 30.6 kPa at T+65 s; max q sin(alpha) 4.4 kPa at T+70 s (alpha up to 54.0 deg where q > 1 kPa); aero normal force up to 1171 kN
         axial 7.15 g, lateral 0.11 g; Mach 28.3; nose flux 49 kW/m2 at T+132 s, skin 742 K; aero moment up to 39.1 MN m
booster: caught by the tower, 0.7 m from the site, 364485 kg left
  loads: max q 183.0 kPa at T+320 s; max q sin(alpha) 18.4 kPa at T+329 s (alpha up to 179.2 deg where q > 1 kPa); aero normal force up to 2770 kN
"""


class LaunchDay(unittest.TestCase):
    def test_the_main_bodys_envelope_and_what_became_of_the_stage_and_the_ship_are_read(self):
        env, booster, perigee = ld.parse_envelope(SAMPLE)
        self.assertEqual(env, {"q_kpa": 30.6, "q_sin_alpha_kpa": 4.4, "normal_force_kn": 1171.0})
        self.assertTrue(booster.startswith("caught by the tower"))
        self.assertAlmostEqual(perigee, 231.4)
        self.assertEqual(ld.parse_envelope("nothing")[0], None)

    def test_a_day_is_go_under_its_limits_no_go_over_one_and_no_limit_without_any(self):
        env = {"q_kpa": 30.6, "q_sin_alpha_kpa": 4.4, "normal_force_kn": 1171.0}
        self.assertEqual(ld.verdict(env, {"q_kpa": 35.0, "q_sin_alpha_kpa": 5.0, "normal_force_kn": 1500.0}), ("GO", []))
        v, why = ld.verdict(env, {"q_kpa": 35.0, "q_sin_alpha_kpa": 4.0, "normal_force_kn": 1000.0})
        self.assertEqual(v, "NO-GO")
        self.assertEqual(len(why), 2)
        self.assertIn("max q sin(alpha) 4.4 kPa is over the limit of 4.0 kPa", why[0])
        self.assertEqual(ld.verdict(env, {"q_kpa": 0.0, "q_sin_alpha_kpa": 0.0, "normal_force_kn": 0.0}), ("NO LIMIT", []))
        self.assertEqual(ld.verdict(None, {"q_kpa": 1.0})[0], "NO DATA")

    def test_a_mission_file_with_comment_lines_is_read(self):
        data = ld.read_json_with_comments(ROOT / "vehicles" / "missions" / "starship_return.json")
        self.assertIn("mission", data)
        self.assertTrue(data["mission"]["main"])


if __name__ == "__main__":
    unittest.main()
