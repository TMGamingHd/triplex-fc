# Dynamics beyond a rigid body on a first-order servo: the servo, the exhaust, the separation, the roll controller, slosh and bending

> Status: **built** (accepted 7 Oct 2026, ADR-033): `ActuatorSpec`, `RollSpec`, `FlexSpec`, `SloshSpec`, `jet_damping` and the stage's separation fields in `sim/vehicle/spec.hpp`, `sim/vehicle/slosh.hpp`, and their use in `vehicle6.hpp`. Every setting defaults to what the reference model always had (a first-order servo, no play, no exhaust damping, a clean separation, a roll held ideally, a rigid vehicle with its liquid fixed), and **with the defaults every flight and every documented number is bit for bit what it was**. The formulas are the standard textbook ones, written from memory of the usual references (the slosh figures were checked against a web summary of Abramson and Dodge on 7 Oct 2026, not against the original reports) and are checked here against hand calculations (section 10), **not against a measured vehicle**: the numbers an example file carries are assumptions.

## 1. What it is for

A flight computer that steers a rigid body through an ideal servo is the easy case. A real vehicle's servo has a bandwidth, rings, has play in its linkage and stops at its travel; a turning vehicle's exhaust takes angular momentum away; a stage that separates pushes and twists what it leaves; the roll has to be held by something; the propellant sloshes and the structure bends, and both push on the vehicle at frequencies the controller may not like. Each of these changes what the controller sees, and the reference model had none of them. They are the settings of the vehicle file, so that the same flight software can be flown against a plant of any quality:

```json
"actuator": {"order": 2, "natural_hz": 8, "damping": 0.6, "backlash_deg": 0.25},
"roll_control": {"stage": 1, "torque_max_nm": 60000, "kp_nm_per_rad": 100000, "kd_nm_s_per_rad": 500000},
"jet_damping": true,
"flex": {"stage": 0, "frequency_hz": 2.5, "damping": 0.005, "generalized_mass_kg": 40000, "phi_engine": 1, "slope_engine_per_m": 0.05, "phi_imu": 0.3, "slope_imu_per_m": -0.04},
"stages": [{"tanks": [{"propellant_kg": 130000, "radius_m": 1.8, "slosh": {"damping": 0.03}}],
            "separation_dv_ms": 1.5, "tipoff_pitch_dps": 0.2, "tipoff_yaw_dps": -0.1, "tipoff_roll_dps": 0.3}]
```

## 2. The gimbal servo (`actuator`)

Every gimbaled stage has two servos (the pitch plane and the yaw plane), each following the flight computers' command, limited in travel (`gimbal_limit_deg`) and in rate (`gimbal_rate_dps`), stage by stage as before.

- **Order 1** (the default) is the first-order lag the model always had, `gimbal_lag_s`, then the rate limit.
- **Order 2** is `theta'' = wn^2 (target - theta) - 2 zeta wn theta'`, with `wn = 2 pi natural_hz`, the target being the command limited to the travel. It is advanced by RK4 inside the substep, then **the rate limit acts on its speed and on its step** and **the travel limit stops it, losing the speed it arrived with**. Limiting the speed matters: a servo that kept building up speed while it was rate limited would overshoot much more when it arrived (measured: a peak of 1.190 against 1.088 for the same servo, 20 Hz, damping 0.3, 20 degrees a second, a 1 degree step).
- **Backlash** (either order) is the play between the servo and the engine: the engine does not move until the servo has taken up half of it on that side, then follows it at that distance; when the command comes back, it stays where it is until the servo has crossed the play. A command that changes by less than the play does nothing; a reversal costs the play.

The servo's own position is not what the thrust uses: the **engine's** angle is (`gimbal_pitch_deg()`), after the play. The fins and the reaction wheels keep their own lag and limits (`VEHICLE_SPEC.md`).

## 3. Jet damping (`jet_damping`)

A vehicle that turns moves its nozzle, and the exhaust leaves with the velocity of the nozzle: it carries angular momentum away, `md r x (omega x r)` for an engine of mass flow `md` at `r` from the centre of gravity, and the vehicle gets the opposite: `-md r x (omega x r)`, a damping of the pitch and yaw rates (and of the roll for an engine off the axis). It is in addition to the `dI/dt omega` term the model has always had (the inertia that the propellant leaving takes with it) and not instead of it: without jet damping a vehicle that burns propellant **spins up** as it loses inertia (the model takes the exhaust to leave with no angular momentum), which is wrong by exactly this term. Measured on a 2000 kg vehicle with a 200 kN engine at the aft end, turning at 0.05 radian a second: the rate falls by `md l^2 omega / I` over the run, to 0.3 % of the hand figure (the rate changes a little in the run).

## 4. The separation (`separation_dv_ms`, `tipoff_*_dps`)

When a stage is let go, the vehicle that stays can be pushed and twisted by the release:
- `separation_dv_ms`: a speed added along the vehicle's own long axis (springs, a pressure), at the instant of separation, in whatever direction the axis points then;
- `tipoff_pitch_dps`, `tipoff_yaw_dps`, `tipoff_roll_dps`: the angular rates it is left with (pitch about body Z, yaw about Y, roll about X). A roll tip-off is dropped when the roll is held ideally (the reference model's `ideal_roll_control`, the default), since a roll held ideally has no rate to hand on.
All zero: a clean separation, as before. They are given in the file; a Monte Carlo of them is a later step.

## 5. The roll controller (`roll_control`)

The flight computers steer two tilt planes and have no roll axis; the reference model held the roll rate at exactly zero (`ideal_roll_control`). `roll_control` replaces that, for a vehicle that wants the roll to be real: a torque about the long axis, `-kp (roll angle turned through since T-zero) - kd (roll rate)`, limited to `torque_max`, from reaction-control jets or the like, while its `stage` is on the vehicle (afterwards the roll runs free). It is the vehicle's own autopilot, not the flight computers'. The roll angle is the integral of the roll rate (`State::roll`), not an attitude measurement: it holds the roll the vehicle had at T-zero and does not know what the roll should be.

With it, an engine-out's roll torque, a tip-off, a canted engine's torque turn the vehicle until the controller takes the roll back, and the gimbal planes turn away from the pad's by the roll the vehicle really has: the effect the ideal controller hid. **Give it to a stage that stays**: the example first put it on the first stage, which separates; the roll tip-off the separation leaves (0.3 degrees a second) then ran free on the second stage for 230 s, turned the vehicle through 70 degrees about its axis, and the two-plane controller, which steers planes fixed in the body, lost the flight (138 degrees of attitude error). On the second stage, which is on the vehicle from lift-off to the end, it holds.

## 6. Propellant slosh (`slosh` on a tank)

Each tank that sloshes has its first lateral mode: a mass `m1` on a spring and a damper that moves along the body's Y and Z, relative to the tank. The mass, the frequency and the place are the first-mode results of the linear theory of a liquid in an upright cylinder (Abramson; the pendulum form of Dodge), for a liquid of mass `m` and depth `h` in a tank of radius `R`, with `a = 1.84 h / R`:

| | |
|---|---|
| sloshing mass | `m1 = m (R / (2.2 h)) tanh(a)` (limited to 0.95 of the liquid) |
| frequency | `omega^2 = (1.84 g / R) tanh(a)`, `g` the acceleration along the axis (a 1 m tank filled 1 m deep, in 1 g: 0.66 Hz) |
| place | the pendulum's bob: the pivot, `(R / 3.68) / sinh(2a)` below the surface, less the pendulum length `(R / 1.84) coth(a)`; kept inside the liquid |

and a damping ratio, 0.02 unless the tank says (`damping`; **an assumption**: a baffled tank has a few percent). `mass_scale` and `frequency_scale` multiply the mass and the frequency, for dispersions. The spring has stiffness `m1 omega^2` and needs an axial acceleration: below 0.05 m/s^2 the model floors it (a liquid that is not settled is not a spring, and the model is only a stand-in there: coast between stages).

**How it is coupled.** The sloshing mass is taken out of the rigid body (its mass, its centre and its inertia, by the parallel-axis theorem, so that the rigid body and the mass add up to the liquid they replace), and put back as a force: the spring's, `k x + c x'`, on the vehicle at the mass, along Y and Z, with its moment about the centre of gravity. Each mass moves as `x'' = -F / m1 - a_point`, `a_point` the acceleration of the point it hangs at (the rigid body's specific force there and the turning of the vehicle: `alpha x r + omega x (omega x r)`). Sideways the vehicle accelerates as the rigid mass; **along the axis it accelerates as the whole mass**, since the sloshing mass rides on the tank, pressed on it by the liquid below (the first version of this model forgot that, and a launcher with slosh accelerated 26 % faster; the real flight showed it, the unit tests had not). Gravity acts on both alike and cancels. The IMUs of a vehicle read the thrust over the whole mass along the axis and the force on the rigid body over its mass sideways. The rigid mass properties the gains are designed from (`mass_props()`) keep **all** the liquid rigid, as a designer would; the dynamics use `rigid_mass_props()`.

Not modelled: the second and higher modes, the swirl (rotary slosh), the liquid's surface in low gravity (propellant management devices), non-cylindrical tanks (a spherical tank is a different mass and frequency), the lateral motion's effect on the centre of mass beyond the linear one (the second-order terms `omega x x'`, `omega x omega x x`: below a part in ten million at a 1 mm amplitude), a tank that is not full of one liquid. Up to 8 tanks on a vehicle slosh.

## 7. One bending mode (`flex`)

A lateral bending mode of the structure, one in each plane, belonging to a stage and the stages above it while that stage is on the vehicle. Its modal coordinate `eta` (m) has the frequency, the damping and the **generalised mass** given, and the mode shape is reduced to what couples it: its displacement `phi` and slope `sigma` (per metre) where the engines are and where the IMUs are:

- the lateral thrust drives it: `M eta'' + 2 zeta omega M eta' + omega^2 M eta = phi_engine F_lateral`;
- the thrust follows the structure: along the tangent of the bent axis at the engines, a lateral force `T sigma_engine eta` on top of the straight thrust (a feedback that softens the mode: its static stiffness is `M omega^2 - phi_engine sigma_engine T`);
- the gyros of the vehicle read the slope's rate on top of the body's: `(0, -sigma_imu eta'_z, sigma_imu eta'_y)` added to the body rates, and the accelerometers the displacement's acceleration, `phi_imu eta''`.
When the stage separates the mode ends (its state is zeroed). A vehicle on the platform (the rig) reads none of it: the IMUs are on the platform, not on the structure.

Not modelled: the reaction of the mode on the rigid body (the mode shape is assumed orthogonal to the rigid motions, so the rigid body's own equations are unchanged), the aerodynamic forces as a source of excitation, more than one mode, the modes of the payload, the dependence of the mode on the mass (its frequency and shape change as the tanks empty: the file carries one set; a Monte Carlo of `frequency_hz` is the way to look at it), and any structural nonlinearity.

## 8. What is not modelled in this whole page

Hydraulic and electric servo limits beyond the rate and the travel; deadband and the friction of a real linkage (backlash is the play); engine-gimbal inertia reaction on the structure; a moving centre of pressure from the bending; fin flutter; thrust-vector control by anything but a gimbal's angle; engine start and shutdown transients' impulses (the rise and tail of an engine are modelled, `VEHICLE_SPEC.md`); the sensors' own dynamics (an IMU's bandwidth is the calibrated IMU model's job, a later step).

## 9. The example, and what the bending mode did to the flight software

`vehicles/launcher_dynamics.json` is the two-stage launcher with all of it switched on, **every number an assumption**: a 6 Hz servo of damping 0.6 and a tenth of a degree of play; slosh at 3 % in all four tanks; a 2.5 Hz bending mode of the first stage of generalised mass 40 t; a separation that pushes the second stage at 1.5 m/s and leaves it turning at (0.2, -0.1, 0.3) degrees a second in (pitch, yaw, roll); a roll controller on the second stage; jet damping. The real flight software (three flight functions, ACT) flies it through the separation with the vehicle's own sensors: over 400 s the worst attitude error is 2.6 degrees (rms 0.28), against 2.4 (rms 0.37) for the launcher without any of it. Added one at a time to the plain launcher (400 s, `tfc_fly`, worst error in degrees), it was 2.4 (none), 2.6 (the second-order servo), 2.8 (backlash on the first-order one), 2.4 (jet damping), 2.4 (roll control, then on the first stage), 2.3 (slosh), 2.6 (bending), 2.8 (the separation): none of them moves the loop by itself at these numbers.

The bending mode is the one the flight software has no defence against: its attitude loop has a bandwidth of 0.2 Hz and no filter, and the gyros of the vehicle read the slope of the structure. Sweeping the slope at the IMU (`slope_imu_per_m`), everything else the example's, 400 s with the vehicle's own sensors (`tfc_fly`, worst attitude error after 3 s):

| slope at the IMU, per metre | worst error, degrees |
|---|---|
| -0.04 (the example's) | 2.6 |
| -0.2 | 2.5 |
| -0.5 | 1.0 |
| -0.7 | 1.1 |
| **-1.0** | **186: the flight is lost** |
| -2.0 / -4.0 | 179 / 178: lost |
| +0.5 | 24 |
| +1.0 | 26 |
| +2.0 | 99 |

The sign is the model's output, not a rule: it depends on where the engines and the IMUs are on the mode shape, which is the thing to know about the real structure. The frequency matters too: with a slope of -0.7 the loop tolerates 1, 2.5 and 5 Hz (worst 2.0, 1.1, 0.8 degrees) and loses the flight at 8 Hz (73 degrees). The slope of the thrust at the engines made no visible difference between 0.05, 0 and -0.05. These are measurements of a simulator with assumed numbers, **not of any vehicle**; they say that the simulator can find a bending-control interaction, and that the flight software as it is would need a notch filter or a gain-stabilised design for a vehicle whose IMU slope is of that size. A test holds three of them (`example_the_bending_mode_can_break_the_loop_...`).

## 10. How it is checked

Tests: `tests/test_dynamics.cpp` (16), `tests/test_slosh.cpp` (12), `tests/test_flex.cpp` (6), two flights in `tests/test_vehicle_files.cpp`; mutants: 54 (dynamics), 56 (slosh) and 32 (bending, one equivalent) in `tools/mutation/sim_mutations.py`.

- **The second-order servo**: its step response against the closed form `1 - exp(-zeta wn t)(cos wd t + (zeta wn / wd) sin wd t)` to a millionth of a degree at six instants, in both planes; its peak (`1 + exp(-pi zeta / sqrt(1 - zeta^2))`: 37.2 % for 0.3) and its time (`pi / wd`: 0.1048 s for 5 Hz); the rate limit (never exceeded in a step); a command beyond the travel the same as one at it, bit for bit; the stops (an under-damped servo arrives with speed and a command the other way moves it off at once); the overshoot of the rate-limited servo (measured: 1.088; a number, not a formula).
- **Backlash**, worked by hand: a command inside the half play does nothing, just at the edge nothing, 2.0 with a play of 1.0 puts the engine at 1.5, a return to 1.2 leaves it at 1.5, 0.0 puts it at 0.5; the yaw plane the same; with a second-order servo it settles at the command less half the play.
- **Jet damping**: the rate change against `md l^2 omega / I`; off by default, nothing to damp on a vehicle that does not turn.
- **Separation**: the push along the axis (and along an axis that points elsewhere), each tip-off rate on its own axis, the roll tip-off dropped when the roll is held, nothing handed on by a clean separation.
- **The roll controller**: an exponential decay at `I / kd` (to 0.01 %), a constant deceleration at the torque limit (both signs), a spring alone as an oscillator (the peak `w0 / wn` at a quarter period, and the angle against `(w0 / wn) sin(wn t)` to a nanoradian), the free roll running free, the controller gone with its stage.
- **Slosh**: the mass, frequency and place against three worked cases (a 1 m tank with 1 m of liquid: 0.4322 of the liquid, `omega^2 = 1.7495 g`, 0.4147 m up; a deep thin tank; a shallow wide one, where the mass is kept on the floor); the dispersions and the 0.95 limit; the rigid body and the sloshing mass adding up to the liquid (mass, centre, inertia about the common centre), for tanks that drain in turn and together; a free oscillation with a heavy vehicle at the frequency of **two bodies on a spring** (the reduced mass) and the damping given; **a vehicle free to turn, with no thrust, harmonic at `omega^2 = k (1/m1 + 1/M_r + a^2/I)` to 1e-5 of the amplitude at 20, 60 and 100 s** (which also pins the integration); the lateral momentum conserved to a millionth, including the turning of the point the mass hangs at; the force and the moment of a displaced mass (`F t / M_r` sideways and `arm F t / I`, both planes); the acceleration of the hanging point with two turning rates; the spring, the damper and the accelerometers on each plane; the thrust accelerating a vehicle with slosh as it does one without; the floor of the spring; a stage that leaves takes its slosh with it.
- **Bending**: a free mode against the damped-oscillator closed form to 1e-8 in both planes; the static deflection under a lateral thrust against `phi F / (M omega^2 - phi sigma T)` (2 %); the thrust's slope force and its moment; the gyros and the accelerometers, with and without thrust; the mode ending with its stage.
- **The files**: every field read, written and read back; the vehicle without any of it writes none of it (so `vehicles/reference.json` does not change); every refusal names its field.
- **The real flight software on the example** (above), and the three bending cases.
