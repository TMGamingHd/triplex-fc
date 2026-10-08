# Describing a vehicle: stages, tanks, engines, effectors, and the files that say so

> Status: **built** (accepted 6 Oct 2026, ADR-031): `sim/vehicle/spec.hpp`, `vehicle6.hpp`, `json.hpp`, `spec_io.hpp`, the tools `tfc_fly`, `tfc_sens --vehicle`, `tfc_simd --vehicle` and `tfc_gen_tables --vehicle`, and four example vehicles in `vehicles/`. Host-tested against the frozen model it replaced and against analytic answers; **nothing has run on the real platform**. The aerodynamics, the planet and the environment are still the reference vehicle's (section 8 says what is next).

## 1. What this is for

The simulator used to fly one vehicle, written into the code: five engines, 30 t, a 100 s ascent (`VEHICLE_SIM.md`). That vehicle is still the reference, but it is now **one instance of a general description**: any number of stages, tanks and engines at any position, payloads that leave, several ways of steering (gimbals, fins, reaction thrusters, reaction wheels), staging, throttling, engine start-up and shut-down, a vehicle that starts in orbit. A description is a JSON file; the real flight software flies it.

```bash
build/host/tfc_fly vehicles/two_stage_launcher.json --sensors vehicle --no-accel --csv flight.csv   # fly it with the real flight code; write the flight as a CSV
build/host/tfc_fly vehicles/spacecraft.json --check                                                 # only read and validate it, and say what is in it
build/host/tfc_fly reference --dump vehicles/reference.json                                         # write the built-in reference vehicle out as a file
build/host/tfc_sens --vehicle vehicles/sounding_rocket.json                                         # the sensitivity study of section 7 of SIM_FIDELITY, for that vehicle
build/host/tfc_gen_tables --vehicle vehicles/spacecraft.json tables.hpp                             # the pitch program and gains the firmware would carry for it
```

Nothing here changes what the flight computers are. They are still three computers, a voter and a controller that steers **two tilt angles** (the pitch plane and the yaw plane) with **one gain schedule of 16 points** and a **pitch program of 16 points** (`CONTROL_LOOP.md`); a vehicle is a different *plant* for them to control. What that means for the vehicles they can fly is section 6.

## 2. The model of a vehicle

**Coordinates.** A vehicle is a stack along its long axis. `x` is measured from the aft end of the first stage and is positive forward (nose); `y` and `z` are lateral. At lift-off the body axes coincide with the pad's: `x` up, `y` downrange, `z` crossrange. Positions in the file are in metres, masses in kilograms, forces in newtons, angles in degrees, times in seconds, all from T-zero.

**Stages** (`stages`). Each stage has a **structure** (a dry mass spread uniformly along `[x_start, x_start + length]`, with a radius for its roll inertia and an `inertia_factor` that turns a uniform rod's transverse inertia `m L^2 / 12` into this structure's: 0.6 for the reference vehicle, whose engines and interface are heavy and whose skin is light; `x_cg_dry` if the structure's centre of mass is not the middle), **tanks**, and the rules for when the stage ignites and when it is let go.
- A **tank** is a vertical cylinder (`x_bottom`, `radius`, `density`, `propellant` when full). The propellant sits at the bottom: its column is `m / (density pi r^2)` tall and its centroid is half-way up it. Tanks of a stage **drain together** (each in proportion to its size) or, with `sequential_drain`, **one after another** (the first listed empties first).
- A stage **ignites** at `ignite_time_s`, or `ignite_delay_s` after another stage separated (`ignite_after_sep_of`). It **separates** at `separate_time_s`, and/or `separate_delay_s` after its own burnout (`separate_on_burnout`). A stage that does neither stays with the vehicle.
- A **throttle** table (time since ignition, fraction of rated thrust) scales the thrust and the mass flow of every engine of the stage, linearly between points and held after the last.
- A stage can have its own **gimbal** limit, rate and lag (otherwise the vehicle-wide ones).

**Engines** (`engines`). Each belongs to a stage and has a position (the nozzle exit plane: where its thrust acts), a vacuum thrust, an exit area, a vacuum Isp. **Thrust** is `fraction x thrust_vac - p_ambient x exit_area` (never below zero), so it rises with altitude; the **mass flow** is `fraction x thrust_vac / (Isp_vac g0)` and does not depend on the ambient pressure, so the sea-level Isp is derived (269 s for the reference vehicle). The **fraction** is the stage's throttle while the engine is lit, 0 otherwise, passed through a first-order lag of `rise_s` (when it is going up) or `tail_s` (when it is coming down); an engine lights `start_offset_s` after its stage and can be cut at `cutoff_time_s`; it goes out at once if it fails (`scenario.engine_failures`) or its stage runs out of propellant or is let go. The **direction** of a gimbaled engine is the commanded gimbal of its stage plus its fixed `cant` (an engine mounted tilted) and the vehicle-wide misalignment dispersion; a fixed engine has only the cant. The **moment** about the centre of gravity is `r x F` for each engine: an engine off the axis, ahead of the centre of gravity, or canted turns the vehicle as it should.

**Payloads** (`payloads`): point masses (a fairing, a satellite) that can be jettisoned at a time.

**Mass properties** are the sum over the parts still on the vehicle (each stage's structure, each tank's propellant column, each payload) with the parallel-axis terms, recomputed every step as propellant burns and stages leave. The inertia changes with the propellant (`I w' = M - w x (I w) - I' w`, with `I'` from the mass flow).

**Effectors**: what the flight computers' two commands (a pitch angle and a yaw angle, in degrees) move. A vehicle can have several at once.
| Effector | What the command does | Described by | Sign |
|---|---|---|---|
| **Gimbal** | tilts the thrust of the engines with `gimbal: true` | `vehicle.gimbal_limit_deg`, `gimbal_rate_dps`, `gimbal_lag_s` (per stage: `stages[].gimbal_*`) | a positive pitch command turns the nose toward downrange; a positive yaw command, toward -Z (as on the platform) |
| **Fins** (`fins`) | deflect `gain x command` degrees, limited, rate-limited and lagged; a force `q x (2 x area_each) x lift_slope x deflection` at the hinge, along -Y for a positive pitch deflection and +Z for a positive yaw one | `area_each_m2`, `lift_slope`, `gain`, `limit_deg`, `rate_dps`, `lag_s`, `x_hinge_m` | a set **forward** of the centre of gravity (a canard) needs a **negative gain**: the validity check is that the vehicle's control effectiveness is positive |
| **Thrusters** (engines with a `control`) | an engine that fires in proportion to the command that asks for it: `pitch+` for a positive pitch command, `pitch-` for a negative one, `yaw+`, `yaw-`; fully on at `full_cmd_deg` | its own `direction` (a unit vector in the body frame), position, thrust, Isp, `rise_s`/`tail_s` (the valve) | pitch plus is a torque about +Z, yaw plus about +Y: an aft thruster firing along -Y and a forward one along +Y make a pitch-plus couple |
| **Wheels** (`wheels`) | a torque in proportion to the command, to `torque_max`; the wheel takes the opposite angular momentum, up to `momentum_max`, then gives no more in that direction | `torque_max_nm`, `momentum_max_nms`, `full_cmd_deg` | pitch about +Z, yaw about +Y; momentum is not unloaded |

The **control effectiveness** `b` (the angular acceleration per radian of command, which the gains are designed from) is the sum of what each effector gives: for the gimbal, thrust times the arm to the centre of gravity over the inertia; for fins, `q S C_L-alpha x gain x arm / I`; for thrusters, the torque of full thrust over `full_cmd`, per radian; for wheels, `torque_max / full_cmd / I`. A reaction thruster is a proportional actuator here (a valve with a rise and a tail, firing in proportion to the command): a real one is pulsed, with a minimum impulse and a dead band, which is not modelled.

**Aerodynamics** are the reference vehicle's for every vehicle in this version: one reference diameter, a normal-force slope `c_n_alpha` and a fixed centre of pressure `x_cp_m` (ahead of the centre of gravity makes the vehicle unstable, behind it stable), an axial coefficient that depends on Mach with a transonic rise, linear in the angle of attack, none while the vehicle flies tail first (`SIM_FIDELITY.md`). They do not change when a stage separates. The ones that do are in section 8.

**The ground** holds a vehicle with a thrust-to-weight under 1 on the pad until it is light enough to rise; a vehicle that comes down lands under 5 m/s and is destroyed above it (`VEHICLE_SIM.md` section 3).

**Everything else** is the reference model: a non-rotating spherical Earth, the 1976 standard atmosphere, a mean wind profile with scripted gusts, the platform and the IMU error models (`SIM_FIDELITY.md`).

## 3. The file

JSON, with `//` comments to the end of a line (a vehicle file explains its numbers). Every field is checked, every unknown field is refused with the nearest known name suggested, and **all** the problems are reported at once, each with its line and column:

```
vehicles/typo.json: stages[0].dry_mass: unknown field (did you mean "dry_mass_kg"?) (line 12, column 7)
vehicles/typo.json: engines[0].stage: must name a stage
```

The defaults in the tables are those of the reference vehicle where one exists. A field that is not listed as having a default is required in practice (a zero dry mass is refused).

**Top level**: `name`, `description` (text), and the objects below.

**`vehicle`**: `gimbal_limit_deg` 8, `gimbal_rate_dps` 60, `gimbal_lag_s` 0, `ideal_roll_control` true (the roll rate is held at zero: `VEHICLE_SIM.md` section 3), `ground_contact` true, `crash_speed_ms` 5, `max_substep_s` 0.002, and the dispersions `thrust_scale`, `cd_scale`, `cn_scale` (1) and `thrust_misalign_pitch_deg`, `thrust_misalign_yaw_deg` (0).

**`aero`**: `diameter_m` 1.8 (the reference area is its circle), `c_n_alpha` 2.5 (per radian), `x_cp_m` 12.5.

**`stages[]`**: `name`, `dry_mass_kg`, `x_start_m` 0, `length_m`, `radius_m` 0.9, `inertia_factor` 0.6, `x_cg_dry_m` -1 (the middle), `tanks[]`, `sequential_drain` false, `ignite_time_s` 0, `ignite_after_sep_of` -1, `ignite_delay_s` 0, `separate_time_s` -1 (never), `separate_on_burnout` false, `separate_delay_s` 0, `gimbal_limit_deg`, `gimbal_rate_dps`, `gimbal_lag_s` (-1: the vehicle's), `throttle` `[[t, fraction], ...]`.
**`stages[].tanks[]`**: `propellant_kg`, `x_bottom_m`, `radius_m` 0.9, `density_kg_m3` 900.

**`engines[]`**: `stage` 0, `position_m` `[x, y, z]`, `thrust_vac_n` 92000, `exit_area_m2` 0.12, `isp_vac_s` 310, `gimbal` true, `cant_pitch_deg`, `cant_yaw_deg` 0, `start_offset_s` 0, `cutoff_time_s` -1, `rise_s`, `tail_s` 0, `direction` `[1, 0, 0]`, `control` `"none"` (or `"pitch+"`, `"pitch-"`, `"yaw+"`, `"yaw-"`), `full_cmd_deg` 1. One entry can stand for several: **`count`** engines, all at `position_m`, or spread evenly on a circle of **`ring_radius_m`** around it, the first at **`ring_start_deg`** (from +Y toward +Z). A thruster has its own `direction` and `gimbal: false`; an engine with its own direction cannot gimbal or have a cant (checked).

**`payloads[]`**: `name`, `mass_kg`, `x_m`, `jettison_time_s` -1. **`fins[]`**: `name`, `stage` 0, `x_hinge_m`, `area_each_m2`, `lift_slope` 3, `gain` 1, `limit_deg` 15, `rate_dps` 100, `lag_s` 0. **`wheels`** (its presence turns them on): `stage` 0, `torque_max_nm`, `momentum_max_nms`, `full_cmd_deg` 1.

**`scenario`**: `wind_scale` 1, `wind_direction` `[0, 0, 1]`, `dry_cg_shift_m` 0 (moves every stage's dry centre of gravity), `gusts[]` (`t0_s`, `duration_s`, `peak_ms` `[x, y, z]`), `engine_failures[]` (`time_s`, `engine`: an index into the engine list as it is after rings are expanded), and **`start`** to begin in flight: `altitude_m` with `circular_orbit: true`, or `position_m`, `velocity_ms`, `rates_dps` (inertial frame, the vehicle's long axis along +X).

**`design`**: how the flight computers' tables are designed on the vehicle's nominal flight. `t_end_s` 100, `dt_s` 0.01, then either the gravity turn (`vertical_s` 8, `kick_ramp_s` 4, `kick_deg` 1, `follow_from_s` 16, `blend_s` 4: straight up, a smooth kick, then the attitude follows the velocity vector) or a **`program`** `[[t, deg], ...]` (the angle of the long axis from the pad vertical against time: a spacecraft that holds an attitude and slews). **`gains`**: `wn` 2.5, `zeta` 0.8, `ki_over_kp` 0.2 (the closed loop's natural frequency and damping from `s^2 + b kd s + (b kp - a) = 0`, and the integral gain), `kp_max`, **`b_min`** (the least control effectiveness the gains are designed for), and the spacing of the 16 points: **`every_s`** 6 for a gain every 6 s, or **0 for adaptive**, which puts the points where the gains change (`tolerance` 0.1 is the largest relative error accepted between two points).

## 4. The tools

| Tool | What it does with a vehicle file |
|---|---|
| `tfc_fly FILE\|reference` | Reads it, designs the tables on its nominal flight, flies it with the real flight software (three flight functions, the real ACT logic) and reports: the end state, max-Q, the attitude error against the program, whether it lifted off, was destroyed or went to Safe, when each stage ignited and separated. `--csv` writes the whole flight (every `--every` frames); `--sensors vehicle --no-accel` for a vehicle that leaves the platform's travel (section 6); `--pad N` adds a pad phase of N frames; `--check` only validates; `--dump FILE` writes the vehicle back out normalised. Exit code 0 only for a clean flight |
| `tfc_sens --vehicle FILE` | The sensitivity study (`SIM_FIDELITY.md` section 2) for that vehicle: how far each sensor error and each departure of the vehicle can go before the flight is lost, with its own design and flight time |
| `tfc_simd --vehicle FILE` | The bus simulator flies that vehicle. The flight computers carry the tables they were built with (the reference vehicle's): regenerate them for another vehicle |
| `tfc_gen_tables --vehicle FILE [OUT]` | Writes the pitch program and gain schedule for that vehicle in the form of `firmware/app/src/flight_tables.hpp`. With no `--vehicle` it writes the reference vehicle's, which is what is committed |

## 5. The examples (`vehicles/`)

| File | What it is | What it shows |
|---|---|---|
| `reference.json` | The reference vehicle written out: 5 engines, 30 t, a 100 s ascent. The committed file is checked against what the writer writes for the built-in vehicle, and it flies bit for bit as the built-in one does | The format; the reference is an instance of it |
| `two_stage_launcher.json` | A medium launcher invented for the simulator: 563 t, nine engines (one and a ring of eight) on the first stage, one vacuum engine on the second, kerosene and oxygen in separate tanks, a throttle bucket through max-Q, a coast and a delayed ignition between stages, a fairing, 15 t of payload | 400 s flight to 465 km and 3.3 km/s: max-Q 28.5 kPa at 60 s, Mach 12; the first stage separates at 165.7 s, the second lights at 169.7 s, the fairing goes at 210 s. **Flown with the vehicle's own sensors** (section 6). Its attitude error grows to about 8 degrees (7.8) over the flight: the flight computers' estimate drifts (`SIM_FIDELITY.md` section 3.4) |
| `sounding_rocket.json` | A 200 kg fin-stabilised rocket with a solid motor (a tank that cannot be throttled), no gimbal, four aerodynamic fins | Thrust-to-weight 7.5, Mach 4.5, max-Q 311 kPa at 17.6 s, 47 km in 60 s; the attitude error stays under 1.3 degrees (1.25). The fins have no authority at lift-off, which is why the example sets `b_min` and an adaptive schedule |
| `spacecraft.json` | An 800 kg satellite already in a circular 500 km orbit: eight cold-gas thrusters in four couples, two reaction wheels, no main engine | A 30-degree slew and back in 300 s tracked to 0.4 degrees at 7.61 km/s, with the orbit staying circular |

## 6. What the flight computers can and cannot do with a vehicle

These are limits of the flight software and of the rig, not of the description; the simulator shows them honestly.
1. **Two tilt planes.** The estimator and the controller work with the long axis's tilt from the pad vertical in a pitch plane and a yaw plane. They cannot steer a roll, and **one set of gains serves both planes**: a vehicle whose pitch and yaw effectiveness differ is designed from the pitch plane's.
2. **The platform travels ±45 degrees.** In platform mode (the rig) the IMUs sit on a servo platform that cannot tilt further: a vehicle that pitches past 45 degrees (every orbital launcher) saturates it and the flight computers go blind. Fly such a vehicle with **`--sensors vehicle`**, where the IMUs read the vehicle's own rates and specific force. The two-stage example does exactly this: flown in platform mode it is lost at 235 s, four seconds after its pitch passes 45 degrees (231 s).
3. **A vehicle under thrust cannot find gravity.** In vehicle sensor mode the accelerometer reads thrust, not gravity (`SIM_FIDELITY.md` section 3.2): pass **`--no-accel`**. Without gravity there is nothing to correct a drifting gyro, so the attitude estimate drifts (section 3.4 of the same page).
4. **Sixteen points.** The pitch program and the gain schedule hold 16 points each, so a long flight has coarse tables: use the adaptive schedule (`every_s: 0`) for the gains.
5. **A gain has to be designed for the effectiveness the vehicle really has.** A vehicle with almost no control at lift-off asks for enormous gains from `kp = (wn^2 + a) / b`; set `b_min` (and `kp_max`) so that the gain it is designed for is one the loop can live with. The sounding rocket rang at the actuator's limit for 3 s before it had `b_min`.
6. **The vehicle must be controllable at all.** Nothing here makes an unstable vehicle stable: a vehicle whose effectors cannot overcome its divergence `a` leaves the program, and the sweep says so.

## 7. How it is checked

- **The reference vehicle did not change.** The general model was written to replace a single-vehicle model; that model is frozen in `tests/oracle/vehicle6_legacy.hpp` and the new one is held to it over whole flights (four scenarios with dispersions, gimbal lag, engine-outs and gusts): the states agree **to the last bit**, and the sensitivity tables of `SIM_FIDELITY.md` reproduce to the digit. One decision made this possible: a vehicle of one stage keeps its propellant as the total mass minus its structure, exactly as before, because carrying a separate propellant number changed the rounding after 50 s and with it the quantised sensor values of the loop.
- **Each capability against an answer the code does not produce**: a two-stage vehicle in vacuum gains the speed of the rocket equation applied twice (to 5e-4); an engine off the axis makes `r x F`; a canted engine tilts the thrust by its angle; a ring of 33 engines has no moment and losing one has that engine's, reversed; the centre of gravity and the inertia of tanks and a payload equal the hand sum with the parallel-axis theorem; tanks drain together or one after another; a throttle table is linear; a rise and a tail follow `1 - e^(-t/tau)`; a fin makes `q S C_L-alpha delta`; a thruster makes its torque and burns `F / (Isp g0)`; a wheel gives the body the momentum it takes and stops when full; a vehicle in a circular orbit stays in it.
- **Bad descriptions are refused**, each kind with a test (tests/test_vehicle_general.cpp, test_vehicle_files.cpp), and so are the reader's errors (a trailing comma, a missing quote, an unknown field, a stage that does not exist).
- **Mutation testing** of the model (`tools/mutation/run_sim.py`) covers the general code as well as the reference's.
- **Every committed example** is loaded, validated and flown by the real flight software in the test suite.

## 8. Not modelled yet

The reference model's departures from real life are in `SIM_FIDELITY.md`; the next ones to go, in the order they are being built, are: aerodynamics that follow the shape (a nose, a body, fins, a centre of pressure that moves with Mach, a nonlinear force and drag at any angle of attack, the change when a stage separates); a rotating Earth, J2, other atmospheres and planets, random turbulence; propellant slosh, a bending mode, a second-order actuator, jet damping and a real roll controller; staging disturbances. Until then a vehicle's aerodynamics are a diameter, a normal-force slope and a centre of pressure.
