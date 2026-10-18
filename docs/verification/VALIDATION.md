# Validation: how far the models are from anything real

> Status: **written** (18 Oct 2026). Verification asks whether the code does what the design says (the rest of `verification/`); validation asks whether the design describes something real. Until this page, the project's answer was one sentence ("Flight 12's trajectory is matched in kind, not in numbers", `LIMITATIONS.md` 4). This page makes the comparison it can, says which side of each is **unchecked**, and lists the public data that would close the rest. The Starship-class vehicle is built from public round numbers and my estimates (`VIEWER.md` 7): validating it against the real vehicle is not the aim; the aim is to know how far it is.

## 1. The ascent against the public timeline

The left column is the simulator's, from `tfc_mission vehicles/missions/starship_return.json` (seed 0, the nominal flight). **The right column is from my memory of the public webcasts of Starship flights and is not checked against a source**: each figure has to be replaced by one that names its flight, its webcast and the time it was read.

| Event | The simulator | Public figure (from memory, unchecked) | Difference |
|---|---|---|---|
| maximum dynamic pressure | T+65 s, 10.5 km, 30.6 kPa | about T+1:00 to 1:10; the pressure is not announced | the time agrees to about 5 s |
| hot staging (the booster's engines cut, the ship's light) | T+134 s, 44 km, 1,527 m/s (Mach 4.7) | about T+2:35 to 2:40, about 65 to 70 km, about 1.6 to 1.8 km/s | **about 25 s early, 20 km low, 5 to 15 % slow** |
| ship engines cut | T+474 s, 250 km, 7,698 m/s, in orbit (perigee 231 km) | about T+8:20 on a suborbital path | an orbital target here, a suborbital one in the flights |
| the stage back at the tower | T+378 s, caught 0.7 m from the site | about T+6:30 to 7:00 | **the sim is some 15 to 30 s early** |

**What it says:** the vehicle in the simulator stages earlier, lower and slower than the public flights: its pitch program and its throttle are my estimates (`LIMITATIONS.md` 4). The comparison is coarse and the timeline is a poor judge; it is here to be improved, not to be quoted.

## 2. The solver against exact solutions and, so far, nothing real

Checked (`CFD.md`): Sod's tube on three grids (the density's L1 error 0.0062, 0.0033, 0.0018); a 10 degree wedge at Mach 2 (the pressure and the shock angle to 0.02 and 0.1 %); a 15 degree cone against Taylor-Maccoll (0.5 to 0.7 %); a real-gas shock behind a wall (pressure 0.00 %, density 1.5 and 0.7 %); the viscous layer against the exact laminar solution (0.02 to 1.8 %) and Van Driest II (1.4 to 10 %); the sphere's stagnation pressure at Mach 20 (8.9 % low); and the grid convergence of the tables (`CFD.md` 15: the axial force from the skin low by 7 to 15 %). **Not checked: any flow of a real body.**

Public data that would do it (from memory; each to be found and cited): the pressure on blunt bodies and cones in hypersonic wind tunnels (NASA technical notes of the 1960s on spheres, cones and the Apollo command module), the Apollo command module's and the Space Shuttle Orbiter's aerodynamic databases (NASA reports, with the Orbiter's flight-derived corrections), the standard model AGARD-B for the transonic and supersonic calibration of a solver, and the NASA/ONERA wind-tunnel cases used to verify Cart3D, the method this solver follows (`ADR-050`).

## 3. The weather against the balloons

The wind of a day is a balloon's own measurement (NOAA IGRA, `ADR-049`); nothing here is modelled except the gust and the turbulence on top (`ENVIRONMENT.md`). **Not compared:** the balloon's wind with the wind the vehicle would have met 30 km downrange (the Starbase balloon is at Brownsville, 35 km from the pad).

## 4. The sensors against the datasheet

The IMU model follows the ISM330DHCX datasheet (`IMU_MODEL.md`) and is tested against its own numbers (Allan deviation of the white noise falls as one over the root of the averaging time; the bias instability's variance and correlation time; the drift with temperature): **the model matches the datasheet's typical values, not a part on the bench**. The part is on the desk; `P-S1-01` measures it, and the model's numbers are then replaced by the part's.

## 5. What would make this page a validation

One real comparison for each of the four models above, each with its source: the timeline against a named webcast (an afternoon), the solver against one wind-tunnel case with published pressure and drag (a week), the wind against a second balloon (a day), the IMU against the part (the first day on the bench). The first is the cheapest and the one that changes what the simulator is asked to do: its staging time, altitude and speed are three numbers to tune to, and the tuning of the pitch program is the lever.
