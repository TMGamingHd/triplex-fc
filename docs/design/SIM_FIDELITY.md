# Simulator fidelity: how close to real life, and where it is not

> Written 4 Oct 2026 after an audit of the whole chain. It lists what each part of the simulator models, what the real thing does that it does not, how much that matters (measured by
> `tools/sim/tfc_sens.cpp`, which flies the whole software chain with one departure at a time scaled up until the flight is lost), and what to do about it. The vehicle is invented, so "real life" here means
> *the behaviour a real vehicle and a real IMU have*, not agreement with a particular rocket (VEHICLE_SIM.md section 10).

## 1. Model by model

| Part | What is modelled | What real life has that it does not | Matters? |
|---|---|---|---|
| Rigid-body dynamics | 6-DOF, quaternion attitude, RK4 at 2 ms, mass and inertia that move with the propellant; verified against a second implementation to 6e-14 m (VEHICLE_SIM section 9) | | Fine |
| Gravity and Earth | Inverse-square on a **non-rotating spherical** Earth | Earth's rotation (about 0.4 km/s of eastward launch velocity and a Coriolis term), the equatorial bulge (J2) | Small for a 100 s ascent; no effect on the attitude loop |
| Atmosphere | US Standard Atmosphere 1976 | Day-to-day density and temperature variation | Small |
| Wind | A mean profile (jet stream at 12 km), scripted 1-cosine gusts | Turbulence (a random, Dryden or von Karman, field) and shear layers | **Medium**: gusts are the load case that sets the gains; a random field would find different worst cases |
| Aerodynamics | A fixed centre of pressure ahead of the CG, normal force linear in the angle of attack at any angle, an axial coefficient with a Gaussian transonic rise | Centre of pressure that moves with Mach, a nonlinear normal force (stall), drag that grows with the angle of attack, base drag, plume effects | **Medium**: the unstable pitch mode is the hardest thing the controller faces and its strength is a free number (the loop tolerates 2.5 times the nominal normal-force slope) |
| Engines | Five identical engines, constant vacuum thrust, ambient-pressure loss, constant mass flow; engine-out | A start-up transient, thrust build-up and tail-off, throttling, engine-to-engine differences, thrust misalignment (now a parameter), propellant mixture shifts | **High at lift-off**: see 3.1 |
| Thrust-vector control | An ideal gimbal with an 8 degree limit and a 60 degrees per second rate limit; **now with an optional first-order lag** (`gimbal_lag_s`) | Actuator dynamics (bandwidth, delay, backlash), hydraulic or electric limits, the five engines moving together | Medium: the loop tolerates a 0.28 s lag, 5 to 10 times a real TVC's |
| Roll | **Ideal roll control**: the roll rate is held at zero and roll torques are ignored | A real roll torque, the roll controller and its interaction with pitch and yaw | Medium: a documented flattering assumption (VEHICLE_SIM section 3). Engine-out roll would otherwise end the flight |
| Propellant slosh | None | Sloshing masses in the tanks, an important mode for a real launch vehicle | **High for a real vehicle**, none for the rig (the rig has no tanks) |
| Structure | Rigid | Bending modes, which couple into the gyro and the gimbal | **High for a real vehicle**, none for the rig |
| Stage separation, second stage | None | Staging events | Out of scope |
| Platform (the rig) | A servo with a lag (60 ms), a rate limit (300 degrees per second), a travel limit (45 degrees); the platform **does not accelerate** | Backlash, deadband, calibration error, the platform's inertia and the servo's real bandwidth, vibration, and the platform's own accelerations that the IMUs feel | Medium; measured on the rig (PICO_TESTS E4 to E9) |
| Sensors, platform mode | The platform's rates and gravity in the platform frame, plus uniform noise of +-0.17 dps and +-3.5 mg | The IMU's real errors: bias and bias instability, scale-factor and cross-axis errors, mounting misalignment, temperature drift, quantisation, the output rate that is not synchronous with the frame (stale and repeated samples) | **Now modelled as options** (`SensorErrors`, `ImuModel`), swept below. The noise is **pessimistic**: by my recollection of the ISM330DHCX datasheet (not checked) its noise at this bandwidth is about 0.04 dps and 0.4 mg, a third to a fifth of the simulator's |
| Sensors, vehicle-true mode | The vehicle's body rates and specific force, in the sensor frame | The same errors; and a real vehicle's **vibration** | See 3.2 |
| Timing | The sensors are exactly synchronous with SYNC; the world is stepped on ACT's frame; the bus has no jitter | Frame jitter, receive latency, drift of each node's clock, a late frame | Medium: the fault campaign injects these on the flight bus; the closed loop does not |
| Flight computers, ACT, FDIR | The real code, not a model | | Fine: this is the part that is not simulated |
| Launch sequence | **None.** The vehicle is released at frame 0, the guidance and gain schedules start at frame 0, ACT is in Standby (a neutral gimbal) for the first second | A pad phase: hold-down, engine start, a go/no-go, a T-zero command, gyro calibration while stationary | **High**: see 3.1 and 3.2 |

## 2. How far each departure can go before the flight is lost

Method: `build/rel/tfc_sens --mode platform` and `--mode vehicle`; 80 s of ascent; a flight is lost if, after the first 3 s, the attitude is more than 5 degrees from the pitch program, or the platform saturates, or ACT enters Safe.
The nominal flight has a worst error of 0.34 degree (platform sensors) and 0.75 degree (vehicle sensors). The tables and gains were designed on the nominal vehicle; the flown one is dispersed. All sensors have the baseline noise
unless the row changes it. The figure in brackets is the worst error at that value and the lift-off transient (the first 3 s).

| Departure | Platform sensors (the rig): largest value that still flies | Vehicle sensors (a real vehicle) |
|---|---|---|
| gyro bias | 2.53125 dps (error 0.93 deg; lift-off 0.90) | 0.1875 dps (error 4.58 deg; lift-off 0.12) |
| gyro scale error | at least 0.5 fraction (error 0.35 deg; lift-off 0.02) | 0.363281 fraction (error 4.90 deg; lift-off 0.10) |
| accelerometer bias | at least 0.3 g (error 0.72 deg; lift-off 0.09) | at least 0.3 g (error 0.75 deg; lift-off 0.09) |
| accelerometer scale error | at least 0.5 fraction (error 0.72 deg; lift-off 0.09) | at least 0.5 fraction (error 0.75 deg; lift-off 0.09) |
| IMU misalignment | 6.48438 deg (error 4.98 deg; lift-off 5.00) | at least 20 deg (error 1.19 deg; lift-off 0.09) |
| gyro noise (times 0.17 dps) | 16.4062 x (error 2.29 deg; lift-off 0.47) | 15.625 x (error 4.28 deg; lift-off 0.33) |
| accelerometer noise (times 3.5 mg) | at least 200 x (error 0.72 deg; lift-off 0.09) | at least 200 x (error 0.75 deg; lift-off 0.09) |
| sensor latency (extra frames) | at least 7 frames (error 0.31 deg; lift-off 0.04) | at least 7 frames (error 0.59 deg; lift-off 0.07) |
| stale sample probability | at least 0.95 fraction (error 0.63 deg; lift-off 0.14) | at least 0.95 fraction (error 0.55 deg; lift-off 0.10) |
| gimbal actuator lag | 0.283203 s (error 4.66 deg; lift-off 0.04) | 0.386719 s (error 4.55 deg; lift-off 0.06) |
| thrust misalignment, pitch | at least 3 deg (error 4.45 deg; lift-off 22.37) | at least 3 deg (error 4.84 deg; lift-off 22.37) |
| thrust misalignment, yaw | at least 3 deg (error 4.46 deg; lift-off 22.37) | at least 3 deg (error 4.84 deg; lift-off 22.37) |
| thrust low (fraction lost) | at least 0.5 fraction (error 0.09 deg; lift-off 0.03) | 0.0820312 fraction (error 4.04 deg; lift-off 0.05) |
| thrust high (fraction gained) | at least 0.5 fraction (error 0.46 deg; lift-off 0.03) | at least 0.5 fraction (error 0.78 deg; lift-off 0.09) |
| normal-force slope high (fraction gained) | 1.52344 fraction (error 3.96 deg; lift-off 0.02) | 1.74609 fraction (error 3.37 deg; lift-off 0.10) |
| normal-force slope low (fraction lost) | at least 0.9 fraction (error 0.08 deg; lift-off 0.02) | at least 0.9 fraction (error 0.80 deg; lift-off 0.09) |
| axial force high (fraction gained) | at least 2 fraction (error 0.26 deg; lift-off 0.02) | at least 2 fraction (error 0.74 deg; lift-off 0.09) |
| mean wind (times the profile) | at least 8 x (error 2.44 deg; lift-off 0.02) | at least 8 x (error 2.53 deg; lift-off 0.10) |
| centre of gravity forward | at least 3 m (error 0.29 deg; lift-off 0.02) | at least 3 m (error 0.74 deg; lift-off 0.09) |
| centre of gravity aft | at least 3 m (error 0.44 deg; lift-off 0.02) | at least 3 m (error 0.71 deg; lift-off 0.09) |

Reading it:
- **The flight computers are robust to what a real IMU does to a flight on the platform**: 2.5 dps of gyro bias, a 50% gain error, 70 ms of extra latency, a 95% chance of a stale sample, 16 times the noise, a 0.3 g accelerometer bias.
- **The controller is robust to a real vehicle's dispersions**: 150% more normal-force slope (2.5 times the instability), 50% more or less thrust (on the platform), a gimbal lag of 0.28 s, 3 m of centre-of-gravity shift, 8 times the wind.
- **Where it is not**, and what each means, is in section 3.

## 3. What the audit found

### 3.1 There is no pad phase, and the lift-off is the weak point
> **Update (4 Oct 2026, the same day): the pad phase is built on the host** (`docs/design/LAUNCH_SEQUENCE.md`): the transient with 1 degree of thrust misalignment falls from 5.3 to 1.2 degrees and a vehicle now flies a gyro bias of at least 8 dps (from 0.19). The mission clock on the bus, the launch command and the firmware wiring are still to do.

The vehicle is released at frame 0 while ACT is still in Standby and holds the gimbal at neutral for the first second (100 trustworthy votes: Standby to Nominal at frame 103). With the engines 1 degree off the commanded
direction, the unopposed moment turns the vehicle **5.3 degrees in that second**, 22 degrees at 3 degrees of misalignment, and the loop then recovers: the lift-off column above. A real launch holds the vehicle on the pad
until the control is live and the go/no-go has been made (MISSION_PHASES P2 to P3). The simulator has no T-zero: the guidance and gain tables are indexed by the frame number since SYNC began, not by flight time.
The fix is the `phase` ground command (deferred, FDIR-043 and PHASE-004): a hold phase in which the vehicle is clamped and the flight computers align, and a launch command that sets T-zero for the schedule.

### 3.2 On a real vehicle the estimator cannot find the bias or the attitude reference under thrust
In **vehicle mode** the gyro bias tolerated is **0.19 dps**, against 2.5 dps on the platform. On the platform gravity is always visible, so the estimator learns the bias; on a vehicle under thrust the accelerometer reads the thrust, not gravity,
and a gyro error integrates into attitude. The ISM330DHCX's zero-rate offset is of the order of a degree per second before calibration (datasheet class; **not checked**), so a real vehicle needs the gyro calibrated on the pad, which needs the pad phase of 3.1.
A second, worse effect: the estimator's accelerometer correction accepts a specific force of 1 g plus or minus 0.3 g as gravity. With a thrust-to-weight near 1, **thrust looks like gravity** along the body axis and pulls the estimated tilt to zero;
that is why the vehicle-mode flight is lost with only 8% less thrust (the table's "thrust low" row). Checked by experiment: with 90% of the thrust the vehicle-mode flight reaches 20 degrees of error by 80 s; with the accelerometer correction
switched off (`EstimatorConfig::use_accel = false`, added in this audit, default unchanged) the same flight stays within 0.5 degree, and the vehicle tolerates **50%** less thrust (`tfc_sens --mode vehicle --no-accel`). The gyro-bias row does not change
(0.19 dps): with the accelerometer off nothing observes the bias, which is the point of 3.1. On the rig this cannot happen (the platform is not under thrust); on a real vehicle the correction must be switched off while the engines run, or the
expected thrust acceleration subtracted.

### 3.3 Edge cases checked in this audit
- A 300 s flight past burnout (about 158 s): finite, the engines off, the mass at its dry value.
- A total loss of the flight computers at 30 s: ACT enters Safe on lost votes within a few frames and stays; the simulator marks the run safed; the unstable vehicle leaves the platform's range (the rig would reach its stops).
- The simulator process with a SYNC frame number that is corrupt (4 billion), goes backwards, or jumps forward by more than a minute: it **used to be able to hang** (it stepped the world over every skipped frame); it now ignores a number beyond 30,000 frames (5 minutes) and treats a jump of more than 6,000 frames as a new run.
- ACT's frames absent: the simulator holds the last command and keeps publishing.
- Dispersions at their defaults change nothing: the nominal flight is bit for bit the same as before the model was extended.

## 4. What to do next, in order of what it would change

1. **A launch sequence**: the pad phase, the `phase` and launch commands, T-zero, gyro calibration on the pad, and flight computers that start the guidance schedule at T-zero (3.1, 3.2). The largest gap between the simulator and a real launch, and the largest effect on the results.
2. **An estimator that knows about thrust** (3.2): no accelerometer correction while the engines are running (or a correction that subtracts the expected thrust acceleration), and a bias estimate carried from the pad. Needed for any claim about a real vehicle.
3. **A random wind field** and a Monte Carlo over the dispersions (a hundred flights, each with random draws of every departure of section 2): the way a real programme shows a margin, and the way to find the worst case the one-at-a-time sweep does not.
4. **A calibrated IMU model**: the ISM330DHCX datasheet's noise density, bias and its instability, scale-factor and cross-axis figures, quantisation and the output-rate asynchrony, with the datasheet numbers checked and cited, and with the real parts measured on the bench (Allan variance on a still IMU) when they arrive.
5. **A servo and platform model from measurements**: the lag, rate limit and backlash from PICO_TESTS E4 to E9 replace the assumed ones.
6. **Timing in the closed loop**: jitter and a late frame in the live closed-loop test (the campaign has them on the bus; the loop does not).
7. **Fidelity items that matter for a real vehicle, not the rig**: slosh, a bending mode, a roll controller, a moving centre of pressure. Each is a modelling project of its own; worth doing only if the write-up claims the result carries over to a vehicle.
