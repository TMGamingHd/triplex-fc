# Vehicle simulator: a 6-DOF ascent, the platform, and the bus

> Status: **design, accepted 4 Oct 2026** (owner's choices: the servos are the platform driven by the Pico; the simulator is a C++ library
> plus a runner; the physics is a full 6-DOF ascent, not the two-plane test model of `CONTROL_LOOP.md`). Nothing here is built yet except
> that test model. Numbers are the parameters of a generic small launch vehicle, chosen to be plausible, not to copy any real one.

## 1. What it is for
The simulator is the world the flight computers fly in. It integrates a rocket from the pad through max-Q to the end of the first-stage
burn, takes the **voted gimbal command from ACT** every frame, and produces what the three IMUs will feel. On the rig it also drives the
platform that the IMUs sit on. It is also the engine of the trade studies (TS-4: how long the vehicle stays recoverable after a fault), so it
must be deterministic, fast, and usable without any hardware.

## 2. Who does what (corrects earlier docs)
- **ACT has no servo output.** It votes the three commands and puts the voted gimbal command on the bus (`0x300`). Its Safe action is the
  *value* of that command (freeze, then null).
- **The two D85MG servos are the platform.** They tilt the platform that carries the three IMUs. The **Pico** drives them from the attitude the
  simulator sends over USB, with rate and travel limits of its own and a watchdog on the PC link (PLAT-001 to 004).
- The simulator closes the loop: `command (CAN) -> vehicle dynamics -> platform attitude (USB, to the Pico) -> real IMUs -> flight computers`.
  On the bench without the platform the simulator feeds the nodes' sensors directly (section 6).

## 3. The model
**Frames.** A non-rotating inertial frame at the centre of a spherical Earth (radius 6,378,137 m, gravity `mu/r^2`); the launch point at the
pole of the pad's local vertical. Attitude is the quaternion body-to-inertial; body X is the long axis, nose forward. Earth's rotation, slosh,
flexibility and roll control are not modelled.

**Equations** (integrated with RK4 at 2 ms inside each 10 ms frame):
- Translation: `m a = F_thrust + F_aero + m g` (inertial).
- Rotation: `I w' = M - w x (I w) - I' w`, with the inertia `I(t)` and the centre of gravity moving as propellant burns; `q' = (1/2) q * (0, w)`.
- **Propulsion:** `N` engines on a gimbal a distance `L_g` aft of the CG; thrust `T = T_vac - p_a A_e` per engine (so it rises with altitude), mass flow
  `T / (Isp g0)`. The gimbal angles (pitch plane, yaw plane) come from ACT's command through an actuator model with a limit and a rate limit.
  Thrust acts along the gimbaled direction at the nozzle: force and moment about the CG.
- **Aerodynamics:** US Standard Atmosphere 1976 to 86 km (exponential above), relative wind = velocity - wind, angle of attack and sideslip from the
  body-axis relative velocity, axial force `C_A(Mach) q S` (with a transonic rise), normal force `C_Nalpha alpha q S` at a centre of pressure **ahead of the CG**,
  which makes the vehicle aerodynamically unstable and the TVC necessary.
- **Wind:** a mean profile (jet-stream peak) plus scripted 1-cosine gusts and shears.
- **Engine-out:** one engine's thrust goes to zero at a set time; its moment about the CG remains (a steady disturbance torque) and the control
  authority falls by `(N-1)/N`.

**Reference vehicle** (all parameters in one table in the code): liftoff mass 30,000 kg with 24,000 kg of propellant; 5 engines, sea-level thrust
500 kN in total (thrust-to-weight 1.7), Isp 280 s at sea level and 310 s in vacuum; diameter 1.8 m, length 20 m; `L_g` about 8 m; gimbal limit 8 degrees at 60 degrees/s.
At liftoff the control effectiveness `b = T L_g / I` is about 4 per second squared; around max-Q the aerodynamic divergence is about 1.4 per second squared.

## 4. The platform and what it can show
The platform has two tilt axes about horizontal axes. It shows the vehicle's **long axis relative to the pad vertical**, as two angles: the pitch
plane (rotation about Y) and the yaw plane (rotation about X), within +-45 degrees (D85MG travel is about 60 degrees; hard stops are fitted). A rotation about the long axis
(roll) is not a degree of freedom of the platform and is not reproduced; the ascent is flown without roll. If the vehicle's tilt leaves the platform's range the
platform saturates and the simulator marks the run (PLAT-004). The first 80 to 100 seconds of an ascent stay within about 30 degrees of the vertical.

## 5. What the sensors feel (two modes)
| Mode | Gyro | Accelerometer | Use |
|---|---|---|---|
| **Platform** (the rig; also the default on the bench) | the platform's body rates | gravity in the platform frame (`[-sin ty, cos ty sin tx, cos ty cos tx]` g) | the real IMUs on the platform feel exactly this; the platform does not accelerate |
| **Vehicle-true** (software only) | the vehicle's body rates | the vehicle's **specific force** (thrust and aerodynamics over mass, no gravity), several g along the axis in boost | tests the estimator's accelerometer gate: gravity is not observable under thrust, so the filter coasts on the gyro and keeps the bias it learned on the pad |

## 6. The simulator on the bus
Triggered by ACT, not by the PC's clock, so that its timing does not depend on the PC's jitter (SIM-004): when ACT's output frame for frame `k` arrives (about 6.5 to 7 ms
into the frame) the simulator advances the vehicle by one frame with that command, and publishes the sensor inputs for frame `k+1`. The nodes latch
the latest of them at 0.5 ms into the next frame. If ACT's frame does not arrive within 8 ms of the SYNC, the simulator advances with the last command held (and counts it).
The sensors therefore see the world one frame late, which is also true of the rig (command to platform to sensor).

| CAN id | Content | Rate |
|---|---|---|
| `0x300` | ACT: the voted gimbal command (pitch plane, yaw plane, 0.001 degree), mode and Safe flags (protocol v2) | every frame |
| `0x501` | Simulator: sensor-frame body rates, 0.125 dps per count (the same scale as the gyro frames) | every frame |
| `0x502` | Simulator: sensor-frame accelerometer input, 1/2048 g per count | every frame |
| `0x503` | Simulator telemetry: altitude, speed, mass, dynamic pressure, flags (safed, platform saturated, engine out) | every 10 frames |
| `0x504` | Simulator telemetry: attitude error and the commanded gimbal | every 10 frames |
USB, to the Pico, at 100 Hz: the platform angles (pitch plane, yaw plane) and a sequence number; the Pico returns its own status (position, saturation, watchdog state).
Each node's simulated IMU takes `0x501` and `0x502`, adds its own noise, bias and faults, and sends its own gyro and accelerometer frames as before.

## 7. Control for a changing plant
The plant's parameters change by a large factor through the ascent (thrust, mass, inertia, dynamic pressure), so fixed gains cannot serve. The simulator's nominal
run produces a **trajectory table**: the pitch program (the reference angle and rate of each plane, up to 16 points) and a **gain schedule** (`kp`, `kd`, `ki` against time) designed from the
local `a(t)` and `b(t)` for a chosen natural frequency and damping. Both go to the flight computers as configuration (`Guidance`, and a new `GainSchedule` in `core/`), and the
controller interpolates them each frame. The open-loop reference is the **gravity turn**: a short vertical rise, a small pitch kick, then attitude following the velocity vector (zero angle of attack).

## 8. Scenarios
Nominal ascent; wind shear and a gust at max-Q; engine-out at a chosen time (before, at and after max-Q); a mass or centre-of-gravity offset; a sensor fault is not a scenario of the vehicle but of the nodes (the fault
campaign). Every scenario is a small file with a seed, the events with their times, and the outputs it logs (a binary state history, byte-identical for the same scenario and seed).

## 9. Validation (written before the model is trusted)
Conservation and known values, each a unit test: **circular orbit** (zero thrust, no atmosphere: radius and specific energy constant to a tight tolerance over a quarter orbit); **the rocket equation**
(vacuum, no gravity: `dv = Isp g0 ln(m0/mf)`); **US Standard Atmosphere** values (sea level 101,325 Pa and 1.225 kg/m^3; 11 km: 22,632 Pa and 0.3639; 20 km: 5,475 Pa and 0.0880); **torque-free rotation**
(angular momentum and energy conserved for an asymmetric body); quaternion norm; the **sign of the aerodynamic moment** (unstable); thrust rising with altitude; a **nominal gravity-turn ascent** inside sanity bounds
(altitude, speed and max-Q time); and the closed loop with the scheduled controller and the real estimator holding the attitude through max-Q gusts and an engine-out.

## 10. What it does not model, and the caveats
Earth rotation and the shape of the geoid; propellant slosh; structural flexibility; roll; separation and a second stage; real atmospheric turbulence (gusts are scripted); the rig's accelerations (the platform does not
accelerate, which is why the vehicle-true mode exists). The model is a believable vehicle, not a flight-qualified one: say so in the write-up.
