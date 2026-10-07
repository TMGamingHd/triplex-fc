# Dynamics beyond a rigid body on a first-order servo: the servo, the exhaust, the separation, the roll controller

> Status: **built** (accepted 7 Oct 2026, ADR-031 continued): `ActuatorSpec`, `RollSpec`, `jet_damping` and the stage's separation fields in `sim/vehicle/spec.hpp`, and their use in `vehicle6.hpp`. Every setting defaults to what the reference model always had (a first-order servo, no play, no exhaust damping, a clean separation, a roll held ideally), and **with the defaults every flight and every documented number is bit for bit what it was**. The formulas are the standard textbook ones, written from memory of the usual references and **not checked against a source** here; each is checked against a hand calculation (section 6). No measured actuator, no measured vehicle: the numbers an example file carries are assumptions.

## 1. What it is for

A flight computer that steers a rigid body through an ideal servo is the easy case. A real vehicle's servo has a bandwidth, rings, has play in its linkage and stops at its travel; a turning vehicle's exhaust takes angular momentum away; a stage that separates pushes and twists what it leaves; the roll has to be held by something. Each of these changes what the controller sees, and the reference model had none of them. They are the settings of the vehicle file, so that the same flight software can be flown against a plant of any quality:

```json
"actuator": {"order": 2, "natural_hz": 8, "damping": 0.6, "backlash_deg": 0.25},
"roll_control": {"stage": 0, "torque_max_nm": 800, "kp_nm_per_rad": 120, "kd_nm_s_per_rad": 90},
"jet_damping": true,
"stages": [{"...": "...", "separation_dv_ms": 1.5, "tipoff_pitch_dps": 0.5, "tipoff_yaw_dps": -0.25, "tipoff_roll_dps": 1}]
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

With it, an engine-out's roll torque, a tip-off, a canted engine's torque turn the vehicle until the controller takes the roll back, and the gimbal planes turn away from the pad's by the roll the vehicle really has: the effect the ideal controller hid.

## 6. How it is checked (`tests/test_dynamics.cpp`, 15 tests; 54 mutants of this code)

- The second-order servo: its step response against the closed form `1 - exp(-zeta wn t)(cos wd t + (zeta wn / wd) sin wd t)` to a millionth of a degree at six instants, in both planes; its peak (1 + exp(-pi zeta / sqrt(1 - zeta^2)): 37.2 % for 0.3) and its time (pi / wd: 0.1048 s for 5 Hz); the rate limit (never exceeded in a step); a command beyond the travel the same as one at it, bit for bit; the stops (an under-damped servo arrives with speed and a command the other way moves it off at once); the overshoot of the rate-limited servo (measured: 1.088; a number, not a formula).
- Backlash, worked by hand: a command inside the half play does nothing, just at the edge nothing, 2.0 with a play of 1.0 puts the engine at 1.5, a return to 1.2 leaves it at 1.5, 0.0 puts it at 0.5; the yaw plane the same; with a second-order servo it settles at the command less half the play.
- Jet damping: the rate change against `md l^2 omega / I`; off by default, nothing to damp on a vehicle that does not turn.
- Separation: the push along the axis (and along an axis that points elsewhere), each tip-off rate on its own axis, the roll tip-off dropped when the roll is held, nothing handed on by a clean separation.
- The roll controller: an exponential decay at `I / kd` (to 0.01 %), a constant deceleration at the torque limit (both signs), a spring alone as an oscillator (the peak `w0 / wn` at a quarter period, and the angle against `(w0 / wn) sin(wn t)` to a nanoradian), the free roll running free, the controller gone with its stage.
- The files: every field read, written and read back; the vehicle without any of it writes none of it (so `vehicles/reference.json` does not change); every refusal names its field (order 3, a servo with no frequency or damping, a negative play, a roll controller on a stage that does not exist, with no torque or negative gains, a tip-off that is not a number, a misspelt field with a suggestion).
