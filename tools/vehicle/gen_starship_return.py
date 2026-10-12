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

# ---- the ship's return: the numbers a scan chose (each is a flag too: --set KEY=VALUE, for scanning) ----
SHIP = {
    "with_return": 0,           # 0: the mission ends in orbit (starship_return.json); 1: the ship also deorbits and enters on its flaps (starship_ship_return.json)
    "orbit_s": 3640.0,          # the coast in orbit before the deorbit burn
    "burn_s": 6.0,              # the deorbit burn, on the three vacuum Raptor
    "burn_throttle": 0.4,
    "alpha_deg": 68.0,          # the belly entry's nominal angle between the body axis and the velocity
    "alpha_min_deg": 40.0,
    "alpha_max_deg": 89.0,
    "gain_deg_per_km": 0.1,
    "bank_gain_deg_per_km": 12.0,  # the bank that steers across the ground track
    "bank_max_deg": 45.0,
    "beta_kg_m2": 2000.0,       # the ballistic coefficient the entry's prediction uses
    "trim_deg": 60.0,           # the flaps' deflection at zero demand
    "ignition_below_m": 6000.0,
    "main_propellant_kg": 40000.0,
    "pin_x_m": 40.0,            # the ship's catch pins, from its aft end
    "max_time_s": 7200.0,
    "roll_deg": 12.0,           # the small roll of the climb: out to this many degrees and back (starting at 10 s, over 40 s); my estimate: launchers roll to their azimuth after the tower, nothing Starship-specific was found
}

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
    for k, yz in enumerate(ring(3, 0.95, 90)):   # the ship: three sea-level (one alone in group 5, the other two in group 7: a landing may light one or three) and three vacuum (group 6)
        engines.append(engine(1, yz, SEA_LEVEL, 5 if k == 0 else 7, x=72.3))
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
               "guided": True, "separation_dv_ms": 1.5, "catch_pin_x_m": 61.0, "gimbal_limit_deg": 8, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08}
    ship = {"name": "ship", "dry_mass_kg": 100000, "x_start_m": 72.3, "length_m": 52.1, "radius_m": 4.5, "inertia_factor": 0.55, "x_cg_dry_m": 94,
            "tanks": [{"propellant_kg": 1252000, "x_bottom_m": 76.3, "radius_m": 4.5, "density_kg_m3": 1141}, {"propellant_kg": 348000, "x_bottom_m": 94.0, "radius_m": 4.5, "density_kg_m3": 424}],
            "sections": [{"kind": "tube", "x_start_m": 72.3, "length_m": 36.0, "d_aft_m": 9.0, "d_fore_m": 9.0}, {"kind": "nose", "x_start_m": 108.3, "length_m": 16.1, "d_aft_m": 9.0, "d_fore_m": 0, "shape": "ogive"}],
            "guided": True, "catch_pin_x_m": SHIP["pin_x_m"], "gimbal_limit_deg": 6, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08}
    surfaces = [{"name": "grid fin %d" % (i + 1), "stage": 0, "kind": "grid_fin", "x_hinge_m": 62.5, "azimuth_deg": az, "radius_m": 4.5, "area_m2": 11.1, "chord_m": 4.15, "span_m": 2.68,
                 "min_deg": -35, "max_deg": 35, "rate_dps": 60, "lag_s": 0.03, "channel": i, "deployed": False} for i, az in enumerate((96.0, 264.0))]   # two (11 Oct 2026, the owner: one for each side the tower takes the stage by): one on each flank, about 96 degrees round from the spine
    # the ship's four flaps, as in vehicles/starship.json: hinged along the axis and standing straight out (90) for the climb, edge-on to the stream. They take no command until the entry deploys them (deployed false:
    # stowed at 90); then channels 0 to 3 move them about the mixer's trim (the ship's own mixers; the booster's grid fins use channels 0 to 2 of their own body)
    for ch, (nm, x, az, area, chord, span, cdir) in enumerate((("aft flap port", 73.6, 80, 36.9, 4.1, 9.0, 1), ("aft flap starboard", 73.6, 280, 36.9, 4.1, 9.0, -1), ("forward flap port", 107.0, 112, 16.2, 2.7, 6.0, 1), ("forward flap starboard", 107.0, 248, 16.2, 2.7, 6.0, -1))):
        surfaces.append({"name": nm, "stage": 1, "kind": "flap", "axial_hinge": True, "x_hinge_m": x, "azimuth_deg": az, "radius_m": 4.5, "area_m2": area, "chord_m": chord, "span_m": span, "chord_dir": cdir,
                         "stow_deg": 90, "min_deg": 0, "max_deg": 110, "rate_dps": 30, "lag_s": 0.05, "channel": ch, "deployed": False})
    return {"name": "starship-v3-class-return",
            "description": "Starship V3 class stack that flies its whole mission on the flight computers: ascent, hot staging, orbital insertion of the ship (PEG), and the booster's flip, boost-back, grid-fin entry, landing burn and catch.",
            "vehicle": {"gimbal_limit_deg": 8, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08, "ideal_roll_control": False, "landing_model": True, "max_substep_s": 0.004},
            "aero": {"diameter_m": 9.0, "roughness": 1.2, "full_regime": True, "nose_radius_m": 0.4},
            "stages": [booster, ship], "engines": engines, "payloads": [{"name": "payload", "mass_kg": 37500, "x_m": 112}], "surfaces": surfaces}


# ---- the rig's vehicle: the same plant, flown by firmware that has no mission ----
# The pitch the mission's own ship flew from hot staging to its cut-off (the angle of the long axis from the launch vertical, PEG's linear tangent steering), raised by 16 degrees and held under 89: the rig's
# firmware measures the tilt as a pitch angle, which is singular at 90 degrees from the vertical, so that it cannot follow the mission's 111; the ship then reaches 7.7 km/s but not a circular orbit (a perigee of
# about 64 km and an apogee of about 650 km, measured with tfc_fly).
MISSION_SHIP_PITCH = [(149, 21.4), (159, 26.5), (169, 28.1), (179, 30.1), (189, 32.6), (199, 35.1), (209, 37.7), (219, 40.4), (229, 43.1), (239, 45.8), (249, 48.5), (259, 51.3), (269, 54.0), (279, 56.7),
                      (289, 59.4), (299, 62.2), (309, 64.9), (319, 67.7), (329, 70.5), (339, 73.2), (349, 76.1), (359, 78.9), (369, 81.7), (379, 84.5), (389, 87.3), (399, 90.3), (409, 93.1), (419, 96.0),
                      (429, 99.0), (439, 102.0), (449, 104.9), (459, 108.0), (469, 111.4), (474, 112.5), (491, 104.0)]
RIG_PROGRAM = [[0, 0], [10, 0], [30, 15], [60, 30], [100, 45], [130, 56], [134, 56]] + [[t, round(min(a + 16.0, 89.0), 1)] for t, a in MISSION_SHIP_PITCH]
RIG_SHIP_CUTOFF_S = 470


RIG_ROLL_DEG = 30.0   # the roll of the climb in the rig: out to this many degrees and back between 10 s and 60 s (the firmware's two tilt angles follow a roll of 45 degrees with 4.4 degrees rms of error, 30 with 2.6, 12 with 1.5)


def roll_bump(peak_deg, t0, dur, step=2.0):
    """The roll-rate schedule (deg/s) of a roll that goes out to `peak_deg` and back as sin^2 between t0 and t0 + dur: the derivative of peak sin^2(pi (t - t0) / dur), sampled every `step` s."""
    pts = [[0, 0.0]]
    n = int(round(dur / step))
    for k in range(n + 1):
        t = t0 + k * step
        pts.append([round(t, 1), round(peak_deg * math.pi / dur * math.sin(2.0 * math.pi * (t - t0) / dur), 3)])
    return pts


def rig_vehicle():
    """vehicles/starship.json: the mission's plant, flown by the rig's firmware and by tfc_fly, which have no mission: the stages fly their schedules (no guided engines), the booster burns to the end of the booster
    burn of the mission (132 s: its engines cut off) and is let go at 134 s, the ship lights at hot staging and follows the mission's own ship pitch (less the part the firmware cannot fly: MISSION_SHIP_PITCH) to its cut-off at 470 s. The mission's own program and throttle are
    used for the booster so that the stack comes to the separation in the state the mission's stage plan was designed for (tfc_simd --mission and tfc_fly --mission fly that plan on the stage let go)."""
    v = plant()
    v["name"] = "starship-v3-class"
    v["description"] = "Starship V3 (Block 3) class: two stages, 9 m diameter, 124 m, about 5.4 kt at lift-off, 33 Raptor 3 + 6 Raptor 3, methane and liquid oxygen, hot staging. Public numbers and my estimates, not SpaceX data."
    v["vehicle"] = {"gimbal_limit_deg": 8, "gimbal_rate_dps": 15, "gimbal_lag_s": 0.08, "landing_model": True, "roll_rate_dps": roll_bump(RIG_ROLL_DEG, 10.0, 50.0)}
    v["engines"] = [e for e in v["engines"] if "control" not in e]               # (the attitude thrusters are the mission's: the rig's computers command the gimbal)
    for st in v["stages"]:
        st.pop("guided", None)
    for sf in v["surfaces"]:
        if sf["kind"] == "flap":
            sf["channel"] = -1          # nothing commands the flaps here: they stand out (stow_deg 90) as in the climb
            sf.pop("deployed", None)
    booster, ship = v["stages"]
    booster["separate_time_s"] = 134.0
    booster["throttle"] = [[0, 1.0], [35, 1.0], [48, 0.78], [85, 0.78], [100, 1.0]]
    ship["ignite_time_s"] = 132.6
    ship["throttle"] = [[0, 1.0]]
    for e in v["engines"]:
        e.pop("group", None)
        e.pop("min_throttle", None)
        e["cutoff_time_s"] = 132.0 if e["stage"] == 0 else RIG_SHIP_CUTOFF_S
    v["design"] = {"t_end_s": 491, "program": RIG_PROGRAM, "gains": {"every_s": 0, "wn": 0.5, "zeta": 0.8, "ki_over_kp": 0.15, "b_min": 0.02}}
    return v


# ---- the mission ----
def ship_return():
    """The ship's way home: a coast in orbit to the right place, a short burn against its motion, a belly entry on the flaps, the flip and the landing burn on the engines, and the arms."""
    if not SHIP["with_return"]:
        return [{"name": "in orbit", "kind": "coast", "hold": "prograde", "end": {"time_s": 100000}, "groups": [], "mixer": 1, "slew_dps": 3}]
    return [
        {"name": "orbit", "kind": "coast", "hold": "prograde", "end": {"time_s": SHIP["orbit_s"]}, "groups": [], "mixer": 1, "slew_dps": 3},
        {"name": "turn to retrograde", "kind": "coast", "hold": "retrograde", "end": {"aligned_deg": 5}, "groups": [], "mixer": 1, "slew_dps": 4},
        {"name": "deorbit burn", "kind": "coast", "hold": "retrograde", "end": {"time_s": SHIP["burn_s"]}, "groups": [6], "mixer": 1, "throttle": SHIP["burn_throttle"], "slew_dps": 3},
        {"name": "belly entry", "kind": "belly", "end": {"ignition_below_m": SHIP["ignition_below_m"]}, "groups": [], "mixer": 4, "events": ["deploy_surfaces"], "alpha_deg": SHIP["alpha_deg"],
         "alpha_min_deg": SHIP["alpha_min_deg"], "alpha_max_deg": SHIP["alpha_max_deg"], "gain_deg_per_km": SHIP["gain_deg_per_km"], "beta_kg_m2": SHIP["beta_kg_m2"], "bank_gain_deg_per_km": SHIP["bank_gain_deg_per_km"], "bank_max_deg": SHIP["bank_max_deg"], "slew_dps": 4},
        {"name": "landing burn", "kind": "landing", "end": {"touchdown": True}, "groups": [5], "mixer": 4, "landing_groups": {"one": [5], "three": [5, 7]}, "slew_dps": 25, "wn_rad_s": 1.0},
        {"name": "caught", "kind": "done", "end": {"time_s": 100000}, "groups": [], "mixer": 4}]


def mission():
    ship_wet = 100000 + 1600000 + 37500   # ship dry + propellant + payload
    booster_dry = 275000
    return {
        "site": {"offset_y_m": 0, "offset_z_m": 400, "arm_height_m": 70, "capture_radius_m": 1.5},
        "ignition_margin": 1.2,
        "landing": {"main_propellant_kg": SHIP["main_propellant_kg"] if SHIP["with_return"] else 0, "sink_ms": 0.5, "aim_below_m": 1.8, "tilt_max_deg": 40, "tilt_final_deg": 2.5, "final_height_m": 25, "engine_thrust_n": 2449000, "engine_min_throttle": 0.4, "propellant_kg": 25000, "decel_plan_ms2": 50},
        "air": {"ballistic_coefficient_kg_m2": 7000, "scale_height_m": 7200},
        "limits": {"command_deg": 8, "integrator_deg": 6, "slew_deg_per_frame": 0.6},
        "design": {"wn": 0.5, "zeta": 0.8, "ki_over_kp": 0.1, "kp_max": 80, "sample_s": 1, "max_time_s": SHIP["max_time_s"] if SHIP["with_return"] else 900, "tail_s": 30},
        "mixers": [{"name": "tvc", "gimbal": True, "roll_thrusters": True}, {"name": "rcs", "gimbal": True, "roll_thrusters": True},
                   {"name": "fins", "gimbal": False, "roll_thrusters": False, "surfaces": [0, 1]}, {"name": "fins+rcs", "gimbal": True, "roll_thrusters": True, "surfaces": [0, 1]},
                   ] + ([{"name": "flaps+rcs", "gimbal": True, "roll_thrusters": True, "surfaces": [0, 1, 2, 3], "trim_deg": [SHIP["trim_deg"]] * 4}] if SHIP["with_return"] else []),
        "main": [
            {"name": "booster burn", "kind": "program", "end": {"mass_below_kg": ship_wet + booster_dry + 650000}, "groups": [0, 1, 2, 3, 4], "mixer": 0, "throttle": 1.0,
             "throttle_track": [[0, 1.0], [35, 1.0], [48, 0.78], [85, 0.78], [100, 1.0]], "slew_dps": 5, "program": [[0, 0], [10, 0], [30, 15], [60, 30], [100, 45], [130, 56]],
             "roll_deg": SHIP["roll_deg"], "roll_start_s": 10, "roll_duration_s": 40},
            {"name": "MECO", "kind": "coast", "hold": "inertial", "end": {"time_s": 0.6}, "groups": [], "mixer": 0, "slew_dps": 5},
            {"name": "hot staging", "kind": "coast", "hold": "inertial", "end": {"time_s": 1.4}, "groups": [5, 6, 7], "mixer": 0, "throttle": 1.0, "slew_dps": 5},
            {"name": "separation", "kind": "coast", "hold": "inertial", "end": {"time_s": 1.0}, "groups": [5, 6, 7], "events": ["separate"], "mixer": 0, "throttle": 1.0, "mass_set_kg": ship_wet - 6000, "slew_dps": 5},
            {"name": "ship ascent", "kind": "peg", "end": {"cutoff": True}, "groups": [5, 6, 7], "mixer": 0, "throttle": 1.0, "target": {"circular_km": 250}, "burnout_mass_kg": 137500, "slew_dps": 6},
            {"name": "coast", "kind": "coast", "hold": "prograde", "end": {"time_s": 40}, "groups": [], "mixer": 1, "slew_dps": 3},
            {"name": "payload release", "kind": "coast", "hold": "prograde", "end": {"time_s": 5}, "groups": [], "events": ["jettison"], "mixer": 1, "slew_dps": 3},
            ] + ship_return(),
        "stage0": [
            {"name": "flip", "kind": "coast", "hold": "retro_horizontal", "end": {"aligned_deg": 8}, "groups": [], "mixer": 1, "slew_dps": 4.5},
            {"name": "boostback", "kind": "boostback", "end": {"cutoff": True}, "groups": [0, 1, 2, 3], "mixer": 0, "throttle": 1.0, "bias_m": 625, "reserve_mass_kg": 330000, "slew_dps": 8, "events": ["deploy_surfaces"]},
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
// burns back, falls under two grid fins (one for each side the tower takes it by), lights for the landing and is caught by the tower's arms; the ship's computers steer it to a 250 km circular orbit with PEG (a predictor-corrector burn guidance).
// Built from PUBLICLY REPORTED numbers (9-10 Oct 2026: Super Heavy 72.3 m, 9 m, 33 Raptor 3, 80.8 MN at sea level, 3,650 t of propellant; the ship's six Raptor, three sea-level and three vacuum, 1,600 t; Flight 12 of
// 22 May 2026: max-Q 0:45, MECO 2:22, hot staging 2:24, boost-back 2:30). NOT SpaceX data and not a model of any flight article: the dry masses (275 t, 100 t), the vacuum Raptor's thrust, the tank split, the
// grid fins' size, the pitch program and every number of the mission below are my estimates or my tuning. Generated by tools/vehicle/gen_starship_return.py; edit the script, not this file.
//   build/rel/tfc_mission vehicles/missions/starship_return.json            # design the mission, fly it on the three flight computers, print what happened
"""
HEADER_RIG = """// The Starship V3 (Block 3) class stack, flown by the rig's firmware and by tfc_fly, which have a pitch program and no mission: GENERATED with the mission files from the same plant by
// tools/vehicle/gen_starship_return.py (edit the script, not this file). Built from PUBLICLY REPORTED numbers (9-10 Oct 2026: Super Heavy 72.3 m with 33 Raptor 3, ship about 52 m with six Raptor 3, stack 124.4 m;
// hot staging at T+144 s on Flight 12, 22 May 2026). NOT SpaceX data and not a model of any flight article: the dry masses, the vacuum engine's thrust, the tank split, the grid fins and flaps and the pitch
// program are my estimates (docs/design/VIEWER.md section 7 lists each number and where it comes from). The booster burns for 132 s on the mission's program and throttle, is let go at 134 s with its return
// propellant, and the ship follows the pitch program to its cut-off at 470 s (7.7 km/s, a perigee of about 64 km and an apogee of about 650 km: the firmware cannot tilt past 90 degrees, so it is not a circular orbit). With `tfc_simd --mission vehicles/missions/starship_return.json` (the console passes it) or
// `tfc_fly --mission` the stage let go is flown on by that mission's plan for it, so that its boost-back, entry and catch can be followed. Fly it with a pad phase long enough for the flight computers to calibrate
// their gyros (10 s of rest; the examples use 1500 frames, 15 s):
//   build/host/tfc_fly vehicles/starship.json --sensors vehicle --no-accel --pad 1500 --pose starship.pose.jsonl [--mission vehicles/missions/starship_return.json]
"""


HEADER_SHIP = """// The Starship V3 class stack WITH A MISSION IN THE FILE: the flight computers fly all of it (docs/design/GNC.md, docs/design/RECOVERY.md). The ascent follows a pitch program; the booster is let go at hot staging, flips,
// burns back, falls under two grid fins (one for each side the tower takes it by), lights for the landing and is caught by the tower's arms; the ship's computers steer it to a 250 km circular orbit with PEG (a predictor-corrector burn guidance), then, in THIS file, bring it home: a coast to the right place, a short deorbit burn, a belly entry on the four flaps
// (held at their trim and steered about it, the angle of attack and a bank steering the point where it would come down) and, at 6 km, the landing burn. EXPERIMENTAL: the entry guidance predicts without lift, so the ship comes down
// tens of kilometres from the tower (docs/design/GNC.md section 11 says how far and why); the flip and the ship's catch are NOT finished. starship_return.json is the mission that is.
// Built from PUBLICLY REPORTED numbers (9-10 Oct 2026: Super Heavy 72.3 m, 9 m, 33 Raptor 3, 80.8 MN at sea level, 3,650 t of propellant; the ship's six Raptor, three sea-level and three vacuum, 1,600 t; Flight 12 of
// 22 May 2026: max-Q 0:45, MECO 2:22, hot staging 2:24, boost-back 2:30). NOT SpaceX data and not a model of any flight article: the dry masses (275 t, 100 t), the vacuum Raptor's thrust, the tank split, the
// grid fins' size, the pitch program and every number of the mission below are my estimates or my tuning. Generated by tools/vehicle/gen_starship_return.py; edit the script, not this file.
//   build/rel/tfc_mission vehicles/missions/starship_ship_return.json       # design the mission, fly it on the three flight computers, print what happened
"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(ROOT / "vehicles" / "missions" / "starship_return.json"))
    ap.add_argument("--ship-out", default=str(ROOT / "vehicles" / "missions" / "starship_ship_return.json"), help="the experimental mission in which the ship also deorbits and enters (empty: not written)")
    ap.add_argument("--rig-out", default=str(ROOT / "vehicles" / "starship.json"), help="the vehicle the rig and tfc_fly fly (empty: not written)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="override a number of the ship's return (SHIP in this script), for scanning")
    args = ap.parse_args()
    for kv in args.set:
        k, _, val = kv.partition("=")
        if k not in SHIP:
            ap.error(f"unknown key {k!r}: {', '.join(SHIP)}")
        SHIP[k] = float(val)
    for path, with_return in ((args.out, 0), (args.ship_out, 1)):
        if not path:
            continue
        SHIP["with_return"] = with_return or SHIP["with_return"]
        v = plant()
        v["mission"] = mission()
        Path(path).write_text((HEADER_SHIP if with_return else HEADER) + dump(v) + "\n")
    if args.rig_out:
        Path(args.rig_out).write_text(HEADER_RIG + dump(rig_vehicle()) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
