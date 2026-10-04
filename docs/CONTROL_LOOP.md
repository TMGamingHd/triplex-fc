# The control loop: consensus, estimator, controller, and the model it is tested against

> Status: **built and tested on the host** (P1, first increment). Not yet in the firmware, not yet on the bus: `core/` has the pieces and
> `tests/test_loop.cpp` closes them around a vehicle model. The numbers below are from those tests (`TFC_LOOP_VERBOSE=1 build/host/tfc_tests`).

## 1. The chain

```
 IMU A,B,C --(frames 0x100+n, 0x110+n)--> SensorConsensus --> AttitudeEstimator --> Controller --> command frame 0x200+n --> ACT vote
                                           (vote3 per axis)     (Mahony + bias)       (PID, limits)
                                                      ^                                   ^
                                       usable-node mask from the manager       Guidance (pitch program)
```
Every computer runs the whole chain on **the same consensus input**, so the three replicas stay bit-identical and their command digests agree
(ADR-006). This is what the architecture's "3.0 to 5.0 ms: consensus, estimator, controller, digest" slot is for.

| Piece | File | What it does |
|---|---|---|
| Sensor consensus | `core/include/tfc/consensus.hpp` | Takes the gyro and accelerometer frames of the three nodes, votes each axis (`vote3`, mid-value select with a tolerance), and says whether the result is trustworthy: a value from three, two that agree, or one; **not** a value from two that disagree, three that do not agree, or none. The caller says which nodes' sensors may be used (the manager's view); that mask is the "channel valid flag" interface of the sensor/compute split (ADR-020) |
| Estimator | `core/include/tfc/estimator.hpp` | A Mahony complementary filter: a quaternion corrected by the direction of gravity, with an integrator that learns the gyro bias. Aligns from the first good gravity reading, so there is no start-up transient. Holds its last rates and says so when the gyro is not trustworthy; coasts on the gyro while the accelerometer is outside a gate of 1 g +- 0.3 g (thrust would corrupt gravity) |
| Controller | `core/include/tfc/controller.hpp` | PID in each of two planes, with the rate from the estimator as the derivative, a gimbal limit, a slew limit per frame, and an integrator that stops while the command is saturated and the error would push it further. Holds the last command while the estimate is not trustworthy |
| Guidance | `core/include/tfc/controller.hpp` | A piecewise-linear reference (a pitch program) of up to eight points per plane, with its slope as the rate reference |
| Vehicle model | `tools/vehicle/vehicle_model.hpp` | Host only (double precision, libm): two unstable planes steered by thrust vectoring, a gimbal actuator with limits, and the platform that follows the vehicle through a lagged, rate-limited, travel-limited servo, with the IMU on the platform |

## 2. Conventions
- **Planes.** The rig is a platform that tilts about two horizontal axes, so both angles are observable from gravity: `tilt_y` is the rotation about the sensor's Y axis (the vehicle's **pitch** plane), `tilt_x` about X (the **yaw** plane). A rotation about the vertical is not a degree of freedom of the rig and is not estimated or controlled.
- **Gravity** in the sensor frame is `v = [-sin ty, cos ty sin tx, cos ty cos tx]`, which is what the accelerometer reads at rest, in g. **Body rates** of the platform are `p = tx', q = ty' cos tx, r = -ty' sin tx`.
- **Command frame** (`0x200+n`) carries the gimbal angles in degrees (pitch plane, yaw plane) and a 16-bit digest: the estimator's digest (the angles to 0.01 degree, the bias to 0.01 dps, the number of updates) XOR the controller's (the outputs and integrators to 0.001 degree).

## 3. Determinism (what keeps replicas, and host and target, identical)
- Only `+ - * /`, comparisons and `sqrt` (correctly rounded by IEEE 754) are used. The one transcendental the estimator needs, `atan2`, is its own polynomial (Abramowitz and Stegun 4.4.49, error at most 1e-5 rad, tested over the whole circle), not libm, whose last bit differs between libraries.
- The build must not fuse multiply-adds: **`-ffp-contract=off`** is now set for every host target and for the firmware app (GCC would otherwise contract on any target that has an FMA unit, such as the Cortex-M4F). `-ffast-math` stays off.
- A non-finite gyro is treated as untrustworthy, so the estimator's state can never become NaN; a NaN or out-of-gate accelerometer skips the correction.

## 4. Results (host, closed loop, three replicas, real frame protocol with CRC and quantisation)
Plant: open-loop divergence time constant about 1.5 s, control effectiveness 2 (rad/s^2 per rad), gimbal limit 8 degrees at 60 deg/s, platform lag 60 ms. Reference: a 20 degree nose-down pitch program over one minute. Sensor noise 0.1 dps and 0.002 g per IMU.

| Scenario | RMS pitch error | Max pitch error | Max yaw error | End state |
|---|---|---|---|---|
| Nominal | 0.17 deg | 0.27 deg | 0.02 deg | on the program (-20.005) |
| 0.08 rad/s^2 gust for 2 s | 0.29 deg | 0.85 deg | 0.02 deg | back on the program within eight seconds |
| Engine-out (authority x0.8, steady torque 0.1 and 0.05 rad/s^2) | 0.28 deg | 1.3 deg | 0.9 deg | on the program, yaw within 1 degree |
| One IMU with +12 dps on two gyro axes | as clean + 0.05 | under 1 deg | | masked by the median |
| One IMU frozen | | under 1 deg | | masked by the median |
| Two sensors only (a node excluded) | | under 1 deg | | flies on two |
| Two sensors that disagree (cannot be told apart) | | under 6 deg | | holds, then recovers when the third returns |

In every scenario the three replicas' commands and digests were bit-identical on every frame, and no vote was lost.

One thing the tests found: with the integrator limited to 3 degrees the vehicle sat 0.7 degree off the program, because holding 20 degrees against the divergence needs 4.4 degrees of gimbal. The limit is now 6 degrees. A steady offset like that is what a closed-loop test exists to find.

## 5. What this is not (yet)
- **Not wired in.** The firmware's command is still the fixed function of the frame number (`sim_imu.hpp`). Wiring needs: the consensus at about 3 ms in the frame, the estimator and controller at 3 to 5 ms, the command frame from their output, and the guidance table in configuration.
- **No actuator node.** The test's ACT is a `vote3` of the three commands, with a hold when they cannot be voted. The real ACT (SW-08, SW-09) adds the output latch, the step bound and the Safe sequence.
- **No vehicle-simulator gateway** between the host model and the nodes on the bus (SW-06, SW-07); the model here lives in the tests.
- **A linear-ish plant.** The model is two decoupled planes with a sine on the gimbal; it is the controller's test bench, not a 6-DOF ascent. The 6-DOF model comes with the simulator (SW-06), and the controller gains will be re-tuned there.
- **The accelerometer gate.** On the real vehicle in ascent, thrust makes the accelerometer unusable as a gravity reference for most of the flight; the estimator then coasts on the gyro and the bias estimate stops improving. That is a property of the idea (and an argument for the analytical-redundancy item in `DEFERRED.md`), and the rig, whose platform does not accelerate, does not show it.
- **Timing.** Execution time on the board is not measured (the cost of one step on the host is small; the figure that matters is the Cortex-M4's, measured at S1/S2 with the cycle counter).

## 6. Requirements and tests
TFC-LOOP-001 to 006 (`REQUIREMENTS.md`). Tests: `consensus_*`, `estimator_*`, `controller_*`, `guidance_*`, `atan2_*`, `loop_*` in `tests/test_loop.cpp`.
