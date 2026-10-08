# Vehicle simulator: a 6-DOF ascent, the platform, and the bus

> Status: **built** (accepted 4 Oct 2026; reviewed 5 Oct 2026): the model, the platform, the closed loop on the host and live on `vcan0`, the bus runner `tfc_simd` and the Pico link are built and tested; **nothing has run on the real platform**. Owner's choices: the servos are the platform driven by the Pico; the simulator is a C++
> library plus a runner; the physics is a full 6-DOF ascent, not the two-plane test model of `CONTROL_LOOP.md`. The numbers are those of a generic small launch
> vehicle, chosen to be plausible, not to copy any real one.

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
> **This page describes the reference vehicle.** The model itself takes any vehicle: any number of stages, tanks and engines at any position, payloads, fins, reaction thrusters and wheels, staging, throttling, a start in orbit, read from a file (`VEHICLE_SPEC.md`, ADR-031). The reference vehicle is one instance of that description and flies bit for bit as it did before it existed.

**Frames.** A non-rotating inertial frame at the centre of a spherical Earth (radius 6,378,137 m, gravity `mu/r^2`); the launch point at the
pole of the pad's local vertical. Attitude is the quaternion body-to-inertial; body X is the long axis, nose forward. Earth's rotation, slosh and
flexibility are not modelled, and **roll is held by an ideal roll controller** (the roll rate about the long axis is zero and roll torques are ignored; `Params::ideal_roll_control`).

**Equations** (integrated with RK4 at 2 ms inside each 10 ms frame):
- Translation: `m a = F_thrust + F_aero + m g` (inertial).
- Rotation: `I w' = M - w x (I w) - I' w`, with the inertia `I(t)` and the centre of gravity moving as propellant burns; `q' = (1/2) q * (0, w)`.
- **Propulsion:** `N` engines on a gimbal a distance `L_g` aft of the CG; thrust `T = T_vac - p_a A_e` per engine (so it rises with altitude), mass flow
  the constant `T_vac / (Isp_vac g0)` (it does not depend on the ambient pressure, so the sea-level Isp is derived: 269 s). The gimbal angles (pitch plane, yaw plane) come from ACT's command through an actuator model with a limit and a rate limit.
  Thrust acts along the gimbaled direction at the nozzle: force and moment about the CG.
- **Aerodynamics:** US Standard Atmosphere 1976 to 86 km (exponential above), relative wind = velocity - wind, angle of attack and sideslip from the
  body-axis relative velocity, axial force `C_A(Mach) q S` (with a transonic rise), normal force `C_Nalpha alpha q S` at a centre of pressure **ahead of the CG**,
  which makes the vehicle aerodynamically unstable and the TVC necessary.
  The force is linear in the angle of attack at any angle and **is zero while the vehicle flies tail-first** (relative velocity along the body's -X, an angle of attack past 90 degrees): there is no drag and no restoring moment then. This matters only for
  a vehicle that tumbles after a total loss of control, not for any flight that stays recoverable (`SIM_FIDELITY.md`).
- **Wind:** a mean profile (jet-stream peak) plus scripted 1-cosine gusts and shears.
- **Ground** (added 6 Oct 2026; `Params::ground_contact`, default on): the pad holds the vehicle up. A vehicle at the surface that is not moving away from it, with no net upward force along the local vertical, stays where it is (position and attitude held, no velocity or rotation) and burns
  propellant; it is released at once when the net force is upward. One that comes down is landed under `crash_speed_ms` (5 m/s) and destroyed above it (`crashed()`: the state freezes where it hit). It acts only at the surface, and a vehicle that lifts off at once goes through exactly the
  arithmetic it did before (a test compares the two with the ground on and off bit for bit). Without it a vehicle with thrust-to-weight under 1 sank through the pad.
- **Engine-out:** one engine's thrust goes to zero at a set time; its moment about the CG remains (a steady disturbance torque) and the control
  authority falls by `(N-1)/N`.

**Reference vehicle** (all parameters in one table in the code, `Params` in `vehicle6.hpp`): liftoff mass 30,000 kg with 24,000 kg of propellant (dry mass 6,000 kg); 5 engines of 92 kN vacuum thrust each, so **460 kN in vacuum and 399.2 kN at sea level** (the exit area of 0.12 m^2 per engine costs 12.2 kN of ambient pressure each),
a liftoff **thrust-to-weight of 1.357** (1.564 in vacuum); Isp 310 s in vacuum, which is **269 s at sea level** (it is derived: constant mass flow of 151.3 kg/s, 30.26 kg/s per engine, with the thrust falling as the ambient pressure rises), so the propellant lasts **158.6 s**;
diameter 1.8 m, length 20 m; the gimbal sits at the engines' exit plane, so the arm `L_g` from the centre of gravity is 7.8 m at liftoff and grows to 10.0 m as the tanks empty; gimbal limit 8 degrees at 60 degrees/s.
At liftoff the control effectiveness `b = T L_g / I` is **8.2 per second squared** and rises to 11.1 by 100 s; the aerodynamic divergence `a` peaks at **4.4 per second squared at 65 s** (max-Q). These figures are pinned by a test
(`reference_vehicle_figures_are_the_documented_ones` in `tests/test_vehicle.cpp`), so this paragraph cannot drift from the code again. (Until 6 Oct 2026 this paragraph quoted 500 kN, a thrust-to-weight of 1.7, 280 s, `b` of about 4 and `a` of about 1.4: they were the first
sketch of the vehicle and did not match the code, which every other number on this page and every test uses.)

## 4. The platform and what it can show
The platform has two tilt axes about horizontal axes. It shows the vehicle's **long axis relative to the pad vertical**, as two angles: the pitch
plane (rotation about Y) and the yaw plane (rotation about X), within +-45 degrees (D85MG travel is about 60 degrees; hard stops are fitted). A rotation about the long axis
(roll) is not a degree of freedom of the platform and is not reproduced; the ascent is flown with roll held (section 3). If the vehicle's tilt leaves the platform's range the
platform saturates and the simulator marks the run (PLAT-004). The first 80 to 100 seconds of an ascent stay within about 30 degrees of the vertical.

## 5. What the sensors feel (two modes)
| Mode | Gyro | Accelerometer | Use |
|---|---|---|---|
| **Platform** (the rig; also the default on the bench) | the platform's body rates | gravity in the platform frame (`[-sin ty, cos ty sin tx, cos ty cos tx]` g) | the real IMUs on the platform feel exactly this; the platform does not accelerate |
| **Vehicle-true** (software only) | the vehicle's body rates | the vehicle's **specific force** (thrust and aerodynamics over mass, no gravity), several g along the axis in boost | tests the estimator's accelerometer gate: gravity is not observable under thrust, so the filter coasts on the gyro and keeps the bias it learned on the pad |

## 6. The simulator on the bus
**Built** (`sim/vehicle/runner.hpp`, the library; `tools/sim/tfc_simd.cpp`, the SocketCAN executable). Triggered by ACT, not by the PC's clock, so that its timing does not depend on the PC's jitter
(SIM-004): when ACT's output frame for frame `k` arrives the simulator advances the vehicle over frame `k` with that command and **at once** publishes the sensor inputs for the start of frame `k+1`, long
before that frame's IMU latch (0.5 ms after its SYNC), so they are waiting in every node's queue. (A first version published after SYNC; the host's CAN driver polls its socket every millisecond, so a node
would have missed the sample.) The sensor frames carry the number of the frame they are for, and a node ignores a frame of another cycle. If ACT's frame does not arrive, the next SYNC steps the world with the
last command held (the `command held` flag); a SYNC that skips numbers (a sync-master takeover) steps over the skipped frames the same way, so sim time is always the frame number. A run starts at the first SYNC
heard (the world is brought to that frame with the gimbal neutral) and starts over if the frame number goes backwards. The sensors see the world one frame late, which is also true of the rig (command to platform to sensor).

| CAN id | Content | Rate |
|---|---|---|
| `0x300` | ACT: the voted gimbal command (pitch plane, yaw plane, 0.001 degree), mode and Safe flags (protocol v2) | every frame |
| `0x501` | Simulator: sensor-frame body rates, 1/32 dps per count (the same scale as the gyro frames) | every frame |
| `0x502` | Simulator: sensor-frame accelerometer input, 1/2048 g per count | every frame |
| `0x503` | Simulator state: altitude, speed, mass | every 10 frames |
| `0x504` | Simulator telemetry: dynamic pressure, attitude error in the two planes (against the pitch program) | every 10 frames |
| `0x505` | Simulator flags (safed, platform saturated, engine out, command held, aborted), engines on, time in frames | every 10 frames |
USB, to the Pico, at 100 Hz: the platform angles (pitch plane, yaw plane) and a sequence number; the Pico returns its own status (position, saturation, watchdog state). Built: `tfc_simd --pico PORT` streams it, and the Pico firmware and its PC client (`python3 -m tfc_peers pico`) are `PICO.md`; not run on a board.
Each node's simulated IMU (`CONFIG_TFC_SIM_BUS_IMU`) takes `0x501` and `0x502`, adds its own noise, and sends its own gyro and accelerometer frames as before.

**Results.** On the host with no sockets (`tests/test_runner.cpp`: the runner, three `FlightFunction`s, the real `ActLogic`, every frame through pack and unpack): the nominal 100 s ascent holds the attitude with an RMS error of
0.079 degree (worst 0.34), a 15 m/s gust at 60 s and an engine-out at max-Q stay under 2 degrees, a sensor fault on one computer is masked, a computer lost at 30 s is excluded by ACT and the flight is unchanged, and 0.2 s without
ACT's frame is held through. **Live, on vcan0** (`sim/tests/test_live_closed_loop.py`: three firmware flight computers, the ACT app and `tfc_simd`, six processes with Python as a monitor): 16 s of the ascent stay within 1 degree of the
program with the digests of the three replicas equal throughout, and a flight computer killed at 6 s is excluded and the vehicle flies on (a documented skip if the machine is too loaded or if the two remaining estimators diverge: TS-16).
Run it: `tools/bench/sil_triplex.sh --build --flight --sim-imu`, then `tools/bench/sil_triplex.sh --closed-loop 25`.

## 7. Control for a changing plant
The plant's parameters change by a large factor through the ascent (thrust, mass, inertia, dynamic pressure), so fixed gains cannot serve. The simulator's nominal
run produces a **trajectory table**: the pitch program (the reference angle and rate of each plane, up to 16 points) and a **gain schedule** (`kp`, `kd`, `ki` against time) designed from the
local `a(t)` and `b(t)` for a chosen natural frequency and damping. Both go to the flight computers as configuration (`Guidance`, and a new `GainSchedule` in `core/`), and the
controller interpolates them each frame.
The schedule holds at most 16 points (`GainSchedule::kMaxPoints`). Designing a gain every 6 s over 100 s gives 17, and **the last one (96 s) is dropped on purpose**: the 90 s gains are held to the end of the flight (kp 0.753 instead of 0.689, 9 % higher, in the last
4 s). `flight_tables()` does not report the drop; a test pins it (`reference_vehicle_figures_are_the_documented_ones`) so that a change of the design interval or the horizon that moves it is noticed. The open-loop reference is the **gravity turn**: a short vertical rise, a small pitch kick, then attitude following the velocity vector (zero angle of attack).

## 8. Scenarios
Nominal ascent; wind shear and a gust at max-Q; engine-out at a chosen time (before, at and after max-Q); a mass or centre-of-gravity offset; a sensor fault is not a scenario of the vehicle but of the nodes (the fault
campaign). Every scenario is a small file with a seed, the events with their times, and the outputs it logs (a binary state history, byte-identical for the same scenario and seed).

## 9. Validation (written before the model is trusted)
Conservation and known values, each a unit test: **circular orbit** (zero thrust, no atmosphere: radius and specific energy constant to a tight tolerance over a quarter orbit); **the rocket equation**
(vacuum, no gravity: `dv = Isp g0 ln(m0/mf)`); **US Standard Atmosphere** values (sea level 101,325 Pa and 1.225 kg/m^3; 11 km: 22,632 Pa and 0.3639; 20 km: 5,475 Pa and 0.0880); **torque-free rotation**
(angular momentum and energy conserved for an asymmetric body); quaternion norm; the **sign of the aerodynamic moment** (unstable); thrust rising with altitude; a **nominal gravity-turn ascent** inside sanity bounds
(altitude, speed and max-Q time); and the closed loop with the scheduled controller and the real estimator holding the attitude through max-Q gusts and an engine-out.

### Independent verification (what "accurate" means here)
No simulator of an invented vehicle can be "100% accurate": there is no real vehicle to be accurate against, and the aerodynamic coefficients, the centre of pressure, the engine and the mass numbers are chosen, not measured. What can be shown is
that the code solves the stated equations correctly, and that is what these tests do (`tests/test_vehicle.cpp`, the `verification_*` tests):
- **A second implementation.** The pitch plane is written again from scratch in two dimensions (no quaternions, no cross products, no gimbal vector; it shares only the atmosphere and the mass properties) and flown beside the 3-D model for 50 s in a
  closed pitching flight with an engine-out at 20 s. The two agree to **6e-14 m in position, 1e-14 degrees in attitude and exactly in mass**. This catches frame, sign, quaternion and rotation errors in the 3-D code.
- **Plane symmetry.** The same flight in the yaw plane is the mirror image of the pitch plane (tilt, position and rates equal to 1e-6 or better).
- **Step-size convergence.** Against a 0.5 ms reference after 40 s: 4 ms gives 6.5 mm, 2 ms (the simulator's step) **3.3 mm**, 1 ms 0.9 mm; attitude error under 1e-5 degrees. The convergence is first order, not RK4's fourth, because the gimbal is sampled and held over each substep.
- **The unstable mode.** A coasting vehicle in 300 m/s air with its axis half a degree off the velocity diverges at the rate `sqrt(a)` the aerodynamics predict (`a = q A CNalpha (xcp - xcg) / I`): 0.687 degrees of angle of attack after 0.5 s
  against 0.709 predicted by the linear model, the 3% difference being the sideways acceleration of the velocity that the linear model leaves out.
- **Conservation and known values** (above): orbit, rocket equation, torque-free rotation, atmosphere.

Still unverified against any outside data: the aerodynamic and engine numbers, the mean wind profile, and the real behaviour of the servo platform (that waits for the rig).

## 10. What it does not model, and the caveats
**The full account is `docs/design/SIM_FIDELITY.md`** (model by model, how far each departure can go before the flight is lost, what the audit found, what to do next). In short: dispersions of the vehicle and errors of the IMU are now options, swept by `tfc_sens`; the biggest gaps are the missing pad phase and launch command, an estimator that cannot tell thrust from gravity on a real vehicle, a pessimistic and uncalibrated IMU noise model, and no turbulence.

Earth rotation and the shape of the geoid; propellant slosh; structural flexibility; roll control (it is idealised: see section 3); separation and a second stage; real atmospheric turbulence (gusts are scripted); the rig's accelerations (the platform does not
accelerate, which is why the vehicle-true mode exists). The model is a believable vehicle, not a flight-qualified one: say so in the write-up.

## 11. What was built and what it shows (host, 4 Oct 2026)
**Code:** `sim/vehicle/math3.hpp`, `atmosphere.hpp` (US Standard Atmosphere 1976), `vehicle6.hpp` (the 6-DOF model, the gimbal actuator, wind, engine-out), `platform.hpp` (the platform model),
`design.hpp` (the nominal ascent and the gain schedule); in `core/`, `GainSchedule` and a 16-point `Guidance`. **Tests:** `tests/test_vehicle.cpp`: maths, the atmosphere at the layer bases and across layer
boundaries, mass properties, a circular orbit (radius, energy and angular momentum conserved), the rocket equation, torque-free rotation, thrust against altitude, the sign of the aerodynamic moment, the gimbal's signs and limits,
engine-out, wind and gusts, the tilt mapping and the platform, the nominal ascent, the gain schedule, and eight closed-loop flights in seven tests.

**The nominal ascent** (a 1 degree pitch kick at 8 to 12 s, then a gravity turn): max-Q **31.5 kPa at 65 s** (11 km, Mach 1.4); pitch **17 degrees at 60 s and 32 degrees at 100 s**; 32 km and 995 m/s at 100 s; the angle of attack stays under 3 degrees. The control
effectiveness `b` is 8 to 11 per second squared and the divergence `a` rises to 4.4 around max-Q. The centre of gravity moves **aft** as the tank empties (the propellant column drops) and forward again at the end, which the design handles
because the gains are computed from the local `a(t)` and `b(t)`.

**The loop** (three replicas, platform-mode sensors, consensus, estimator, the scheduled controller, the actuator node's vote, the platform lag):

| Flight | RMS error | Worst error | Platform saturated |
|---|---|---|---|
| Nominal | 0.07 deg | 0.31 deg | no |
| 15 m/s crosswind gust at max-Q | 0.14 deg | 1.25 deg | no |
| Mean wind doubled (56 m/s jet stream) | 0.14 deg | 0.57 deg | no |
| Engine-out at 30 s | 0.18 deg | 1.5 deg | no |
| Engine-out at 62 s (max-Q) | 0.20 deg | 1.95 deg | no |
| Dry centre of gravity 0.8 m aft | 0.08 deg | 0.33 deg | no |
| One IMU reading 15 dps too much on two axes | 0.08 deg | 0.31 deg | no |
| Vehicle-true sensors (gyro only, gravity unobservable under thrust) | 0.08 deg | 0.25 deg | no |

The replicas' commands and digests were bit-identical on every frame of every flight.

**What the flights found.** The engine-out flights first tumbled. The cause was not the controller: a failed off-axis engine, with the gimbal deflected, produces a **roll torque**, and the roll inertia is about 40 times smaller than the pitch inertia, so the vehicle rolled
until its body-fixed gimbal planes were no longer the pad's planes and control was lost. A real vehicle has roll control, which the flight computers' two-plane gimbal does not provide and which is not the subject here, so the model holds roll ideally (a test shows the roll rate stays zero, and
drifts without the option). It is stated plainly in section 3 because it is an assumption that flatters the result. Earlier, a fixed-point iteration for the gravity turn converged to the trivial vertical flight; the pitch program is now computed causally (the attitude follows the velocity vector as it flies).

**Limits.** The reference vehicle's parameters are mine; max-Q and the pitch history are plausible, not matched to any real vehicle. The aerodynamic model is a normal-force slope with a transonic axial-force bump, not a table from a wind tunnel. Closed-loop numbers are for one set of gains designed for `wn` 2.5 rad/s and damping 0.8; the margins
(how much the gains or the delay can change before the loop is lost) are measured by `tools/sim/tfc_sens.cpp`, one departure at a time scaled up until the flight is lost (`SIM_FIDELITY.md`).
