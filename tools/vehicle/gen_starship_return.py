#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes vehicles/missions/starship_return.json: the Starship V3 class stack with a MISSION in the file (docs/design/GNC.md): the flight computers fly the whole of it, the booster comes back and is caught by the tower.

    python3 tools/vehicle/gen_starship_return.py [--out vehicles/missions/starship_return.json]

The engines are listed one by one (a file with a hundred lines of engines is not a file anyone edits), so the file is generated; the numbers are in this script, with where each comes from. The file is a design
of mine from PUBLICLY REPORTED numbers, not SpaceX data (the header of the file says so; docs/design/RECOVERY.md section 2 lists every number and its source).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# ---- the vehicle ----
# Sources (read 9-10 Oct 2026): the SpaceX Starship page and Wikipedia (Super Heavy, Raptor, Starship flight 12); Flight 12 (22 May 2026): max-Q 0:45, MECO 2:22, hot staging 2:24, boost-back 2:30, ship cut-off
# 8:11 (planned), apogee about 195 km. Super Heavy: 72.3 m, 9 m, 33 Raptor 3 of 2.449 MN at sea level (80.8 MN), 3,650 t of propellant. The ship: three sea-level and three vacuum Raptor, 1,600 t.
# MY ESTIMATES (nothing public): the booster's dry mass 275 t (the Block 1/2 figure), the ship's dry mass 100 t, the vacuum Raptor's thrust 2.697 MN (Isp 380 s), the tanks' split and positions.
SEA_LEVEL = dict(thrust_vac=2_584_000, area=1.33, isp=350)   # 2.449 MN at sea level + 101325 Pa x 1.33 m^2
VACUUM = dict(thrust_vac=2_697_000, area=4.5, isp=380)


def ring(n: int, radius: float, start_deg: float):
    return [(radius * math.cos(math.radians(start_deg + 360.0 * i / n)), radius * math.sin(math.radians(start_deg + 360.0 * i / n))) for i in range(n)]


def engine(stage, yz, spec, group, gimbal=True, x=0.0):
    e = {"stage": stage, "position_m": [x, round(yz[0], 4), round(yz[1], 4)], "thrust_vac_n": spec["thrust_vac"], "exit_area_m2": spec["area"], "isp_vac_s": spec["isp"], "group": group,
         "min_throttle": 0.4, "rise_s": 0.8, "tail_s": 0.8}
    if not gimbal:
        e["gimbal"] = False
    return e


def thruster(stage, x, control, yz, direction, thrust):
    return {"stage": stage, "position_m": [x, yz[0], yz[1]], "thrust_vac_n": thrust, "exit_area_m2": 0.0, "isp_vac_s": 80, "gimbal": False, "direction": list(direction), "control": control, "full_cmd_deg": 2.0}


def plant():
    engines = []
    for k, yz in enumerate(ring(3, 0.95, 90)):   # the inner three: groups 0, 1, 2 (a triangle about the axis, so that they are balanced when they run alone)
        engines.append(engine(0, yz, SEA_LEVEL, k))
    for yz in ring(10, 2.25, 0):                 # the ring of ten: group 3 (with the inner three, the thirteen of the landing burn)
        engines.append(engine(0, yz, SEA_LEVEL, 3))
    for yz in ring(20, 3.85, 0):                 # the outer twenty are fixed: group 4
        engines.append(engine(0, yz, SEA_LEVEL, 4, gimbal=False))
    for yz in ring(3, 0.95, 90):                 # the ship: three sea-level (group 5) and three vacuum (group 6)
        engines.append(engine(1, yz, SEA_LEVEL, 5, x=72.3))
    for yz in ring(3, 2.55, 30):
        engines.append(engine(1, yz, VACUUM, 6, x=72.3))
    # attitude thrusters of the booster (cold-gas class, my sizes): pitch and yaw at the top, roll at the rim
    engines += [thruster(0, 66.0, "pitch+", (0, 0), (0, 1, 0), 100000), thruster(0, 66.0, "pitch-", (0, 0), (0, -1, 0), 100000),
                thruster(0, 66.0, "yaw+", (0, 0), (0, 0, -1), 100000), thruster(0, 66.0, "yaw-", (0, 0), (0, 0, 1), 100000),
                thruster(0, 66.0, "roll+", (4.5, 0), (0, 0, 1), 50000), thruster(0, 66.0, "roll-", (4.5, 0), (0, 0, -1), 50000)]
    engines += [thruster(1, 118.0, "pitch+", (0, 0), (0, 1, 0), 30000), thruster(1, 118.0, "pitch-", (0, 0), (0, -1, 0), 30000),   # and the ship's, at its nose
                thruster(1, 118.0, "yaw+", (0, 0), (0, 0, -1), 30000), thruster(1, 118.0, "yaw-", (0, 0), (0, 0, 1), 30000),
                thruster(1, 118.0, "roll+", (4.5, 0), (0, 0, 1), 15000), thruster(1, 118.0, "roll-", (4.5, 0), (0, 0, -1), 15000)]
    booster = {"name": "booster", "dry_mass_kg": 275000, "x_start_m": 0, "length_m": 72.3, "radius_m": 4.5, "inertia_factor": 0.55, "x_cg_dry_m": 28,
               "tanks": [{"propellant_kg": 802000, "x_bottom_m": 2.0, "radius_m": 4.5, "density_kg_m3": 424}, {"propellant_kg": 2848000, "x_bottom_m": 32.0, "radius_m": 4.5, "density_kg_m3": 1141}],
               "sections": [{"kind": "tube", "x_start_m": 0, "length_m": 72.3, "d_aft_m": 9.0, "d_fore_m": 9.0}],
               "guided": True, "separation_dv_ms": 1.5, "catch_pin_x_m": 60.0, "gimbal_limit_deg": 8, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08}
    ship = {"name": "ship", "dry_mass_kg": 100000, "x_start_m": 72.3, "length_m": 52.1, "radius_m": 4.5, "inertia_factor": 0.55, "x_cg_dry_m": 94,
            "tanks": [{"propellant_kg": 1252000, "x_bottom_m": 76.3, "radius_m": 4.5, "density_kg_m3": 1141}, {"propellant_kg": 348000, "x_bottom_m": 94.0, "radius_m": 4.5, "density_kg_m3": 424}],
            "sections": [{"kind": "tube", "x_start_m": 72.3, "length_m": 36.0, "d_aft_m": 9.0, "d_fore_m": 9.0}, {"kind": "nose", "x_start_m": 108.3, "length_m": 16.1, "d_aft_m": 9.0, "d_fore_m": 0, "shape": "ogive"}],
            "guided": True, "gimbal_limit_deg": 6, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08}
    surfaces = [{"name": "grid fin %d" % (i + 1), "stage": 0, "kind": "grid_fin", "x_hinge_m": 61.8, "azimuth_deg": az, "radius_m": 4.5, "area_m2": 11.1, "chord_m": 4.15, "span_m": 2.68,
                 "min_deg": -35, "max_deg": 35, "rate_dps": 60, "lag_s": 0.03, "channel": i, "deployed": False} for i, az in enumerate((0.0, 90.0, 180.0))]   # three, as on Flight 12
    return {"name": "starship-v3-class-return",
            "description": "Starship V3 class stack that flies its whole mission on the flight computers: ascent, hot staging, orbital insertion of the ship (PEG), and the booster's flip, boost-back, grid-fin entry, landing burn and catch.",
            "vehicle": {"gimbal_limit_deg": 8, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08, "ideal_roll_control": False, "landing_model": True, "max_substep_s": 0.004},
            "aero": {"diameter_m": 9.0, "roughness": 1.2, "full_regime": True, "nose_radius_m": 0.4},
            "stages": [booster, ship], "engines": engines, "payloads": [{"name": "payload", "mass_kg": 37500, "x_m": 112}], "surfaces": surfaces}


# ---- the mission ----
def mission():
    ship_wet = 100000 + 1600000 + 37500   # ship dry + propellant + payload
    booster_dry = 275000
    return {
        "site": {"offset_z_m": 45, "arm_height_m": 70, "capture_radius_m": 1.5},
        "ignition_margin": 1.2,
        "landing": {"sink_ms": 0.5, "tilt_max_deg": 40, "tilt_final_deg": 2.5, "final_height_m": 25, "engine_thrust_n": 2449000, "engine_min_throttle": 0.4, "propellant_kg": 25000, "decel_plan_ms2": 50},
        "air": {"ballistic_coefficient_kg_m2": 7000, "scale_height_m": 7200},
        "limits": {"command_deg": 8, "integrator_deg": 6, "slew_deg_per_frame": 0.6},
        "design": {"wn": 0.5, "zeta": 0.8, "ki_over_kp": 0.1, "kp_max": 80, "sample_s": 1, "max_time_s": 900, "tail_s": 30},
        "mixers": [{"name": "tvc", "gimbal": True, "roll_thrusters": True}, {"name": "rcs", "gimbal": True, "roll_thrusters": True},
                   {"name": "fins", "gimbal": False, "roll_thrusters": False, "surfaces": [0, 1, 2]}, {"name": "fins+rcs", "gimbal": True, "roll_thrusters": True, "surfaces": [0, 1, 2]}],
        "main": [
            {"name": "booster burn", "kind": "program", "end": {"mass_below_kg": ship_wet + booster_dry + 650000}, "groups": [0, 1, 2, 3, 4], "mixer": 0, "throttle": 1.0,
             "throttle_track": [[0, 1.0], [35, 1.0], [48, 0.78], [85, 0.78], [100, 1.0]], "slew_dps": 5, "program": [[0, 0], [10, 0], [30, 15], [60, 30], [100, 45], [130, 56]]},
            {"name": "MECO", "kind": "coast", "hold": "inertial", "end": {"time_s": 0.6}, "groups": [], "mixer": 0, "slew_dps": 5},
            {"name": "hot staging", "kind": "coast", "hold": "inertial", "end": {"time_s": 1.4}, "groups": [5, 6], "mixer": 0, "throttle": 1.0, "slew_dps": 5},
            {"name": "separation", "kind": "coast", "hold": "inertial", "end": {"time_s": 1.0}, "groups": [5, 6], "events": ["separate"], "mixer": 0, "throttle": 1.0, "mass_set_kg": ship_wet - 6000, "slew_dps": 5},
            {"name": "ship ascent", "kind": "peg", "end": {"cutoff": True}, "groups": [5, 6], "mixer": 0, "throttle": 1.0, "target": {"circular_km": 250}, "burnout_mass_kg": 137500, "slew_dps": 6},
            {"name": "coast", "kind": "coast", "hold": "prograde", "end": {"time_s": 40}, "groups": [], "mixer": 1, "slew_dps": 3},
            {"name": "payload release", "kind": "coast", "hold": "prograde", "end": {"time_s": 5}, "groups": [], "events": ["jettison"], "mixer": 1, "slew_dps": 3},
            {"name": "in orbit", "kind": "coast", "hold": "prograde", "end": {"time_s": 100000}, "groups": [], "mixer": 1, "slew_dps": 3}],
        "stage0": [
            {"name": "flip", "kind": "coast", "hold": "retro_horizontal", "end": {"aligned_deg": 8}, "groups": [], "mixer": 1, "slew_dps": 4.5},
            {"name": "boostback", "kind": "boostback", "end": {"cutoff": True}, "groups": [0, 1, 2, 3], "mixer": 0, "throttle": 1.0, "bias_m": 1000, "reserve_mass_kg": 330000, "slew_dps": 8, "events": ["deploy_surfaces"]},
            {"name": "coast", "kind": "coast", "hold": "radial", "end": {"altitude_below_m": 70000}, "groups": [], "mixer": 3, "slew_dps": 8},
            {"name": "entry", "kind": "glide", "end": {"ignition_below_m": 6000}, "groups": [], "mixer": 3, "alpha_max_deg": 12, "gain_deg_per_km": 2.0, "slew_dps": 10, "alpha_brake_deg": 0, "brake_max_deg": 0, "lift_area_per_deg_m2": 3.5},
            {"name": "landing burn", "kind": "landing", "end": {"touchdown": True}, "groups": [0, 1, 2], "mixer": 3, "landing_groups": {"one": [0, 1, 2], "two": [0, 1, 2], "three": [0, 1, 2], "many": [0, 1, 2, 3]}, "slew_dps": 25, "wn_rad_s": 1.0},
            {"name": "done", "kind": "done", "end": {"time_s": 100000}, "groups": [], "mixer": 3}],
    }


# ---- the text of the file ----
def dump(x, indent=0, width=170) -> str:
    """JSON with one line per small object, so that a file of a hundred engines is a hundred lines and not a thousand."""
    flat = json.dumps(x, separators=(", ", ": "))
    pad = "  " * indent
    if len(flat) + len(pad) <= width or not isinstance(x, (dict, list)):
        return flat
    if isinstance(x, dict):
        items = [f'{pad}  {json.dumps(k)}: {dump(v, indent + 1, width)}' for k, v in x.items()]
        return "{\n" + ",\n".join(items) + f"\n{pad}}}"
    items = [f"{pad}  {dump(v, indent + 1, width)}" for v in x]
    return "[\n" + ",\n".join(items) + f"\n{pad}]"


HEADER = """// The Starship V3 class stack WITH A MISSION IN THE FILE: the flight computers fly all of it (docs/design/GNC.md, docs/design/RECOVERY.md). The ascent follows a pitch program; the booster is let go at hot staging, flips,
// burns back, falls under three grid fins, lights for the landing and is caught by the tower's arms; the ship's computers steer it to a 250 km circular orbit with PEG (a predictor-corrector burn guidance).
// Built from PUBLICLY REPORTED numbers (9-10 Oct 2026: Super Heavy 72.3 m, 9 m, 33 Raptor 3, 80.8 MN at sea level, 3,650 t of propellant; the ship's six Raptor, three sea-level and three vacuum, 1,600 t; Flight 12 of
// 22 May 2026: max-Q 0:45, MECO 2:22, hot staging 2:24, boost-back 2:30). NOT SpaceX data and not a model of any flight article: the dry masses (275 t, 100 t), the vacuum Raptor's thrust, the tank split, the
// grid fins' size, the pitch program and every number of the mission below are my estimates or my tuning. Generated by tools/vehicle/gen_starship_return.py; edit the script, not this file.
//   build/rel/tfc_mission vehicles/missions/starship_return.json            # design the mission, fly it on the three flight computers, print what happened
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(ROOT / "vehicles" / "missions" / "starship_return.json"))
    args = ap.parse_args()
    v = plant()
    v["mission"] = mission()
    text = HEADER + dump(v) + "\n"
    Path(args.out).write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
