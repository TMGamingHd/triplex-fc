# The IMU as the datasheet describes it: noise, bias, drift, cross-axis and jitter

> Status: **built** (7 Oct 2026): `SensorErrors`, `ism330dhcx_typical()`, `ism330dhcx_maximum()` and `ImuModel` in `sim/vehicle/imu_model.hpp`, `tests/test_imu_datasheet.cpp`. The numbers are **the ISM330DHCX datasheet's** (DS13012 rev 7, "Mechanical characteristics", Table 3, read on 7 Oct 2026 from ST's published datasheet), not measurements of the parts on the bench: the three IMUs that arrived on 6 Oct 2026 have not yet been characterised (`BENCH_LOG.md`), and an Allan-variance run on them will replace the datasheet's typical values here. Everything is **off by default**: with nothing switched on the model is the bench model of the first weeks, sample for sample (a test pins it).

## 1. What it is for

The bench model of the first weeks gave each of the three computers' IMUs uniform noise of +-0.17 dps and +-3.5 mg, and optional constant errors. That is enough for the rig (the platform's accelerometer corrects the gyros) and wrong for a vehicle in flight, where the accelerometer cannot see gravity and **a gyro's own errors are the attitude's errors** (`SIM_FIDELITY.md` 3.2, 3.4). So the errors a real part has, with the values its maker gives, are needed to say how long the flight computers can hold an attitude on the gyros alone.

## 2. The datasheet's figures, and what each does in the model

| Datasheet (Table 3) | Typical (limit) | In the model |
|---|---|---|
| Gyro zero-rate level | +-1 dps (+-3) | a constant bias per sensor and axis, uniform within it (`gyro_bias_dps`) |
| Gyro sensitivity | +-2 % (the limit; no typical given) | a constant gain error, uniform within it (`gyro_scale_err`) |
| Gyro rate noise density, high-performance mode | 5 mdps/rtHz (8) | white Gaussian noise of rms density x sqrt(bandwidth) per sample; the bandwidth is 416 Hz (half the 833 Hz output rate), so **0.102 dps rms** (`gyro_noise_density_dps`) |
| Gyro bias instability | 3 deg/h | a first-order Gauss-Markov bias of that standard deviation, correlation time 100 s unless set (`gyro_bias_instability_dps`) |
| Gyro zero-rate change with temperature | +-0.005 dps/degC (+-0.015) | up to this per degree, sign and size drawn once per sensor and axis, times the temperature above 25 degC |
| Gyro sensitivity change with temperature | +-0.007 %/degC (+-0.015) | a gain change of up to this fraction per degree |
| Gyro cross-axis sensitivity | +-1 % | every off-diagonal term of the sensor's 3x3 matrix up to this, drawn once |
| Accelerometer zero-g offset | +-10 mg (+-65) | a constant bias (`accel_bias_g`) |
| Accelerometer sensitivity | +-2 % | as the gyro's |
| Accelerometer noise density | 60 ug/rtHz (100) | white Gaussian noise: **1.22 mg rms** at 416 Hz |
| Accelerometer zero-g change with temperature | +-0.1 mg/degC (+-0.5) | as the gyro's |
| Accelerometer sensitivity change with temperature | +-0.005 %/degC (+-0.01) | as the gyro's |
| Accelerometer cross-axis sensitivity | +-0.5 % | as the gyro's |
| Gyro angular random walk | 0.21 deg/sqrt(h) (0.34) | not an input: it is what the noise density gives when it is integrated (checked below) |

The datasheet's limits are "based on characterization results at 3 sigma on a limited number of samples, not tested in production and not guaranteed" (its footnote 1). The model draws the constants **uniformly within** the figure given; a real population is not uniform, and a worst-case preset (`ism330dhcx_maximum`) is the 3-sigma figures.

**A consistency check that holds:** the bench model's uniform noise of +-0.17 dps has a standard deviation of 0.098 dps; the datasheet's 5 mdps per root hertz over 416 Hz is 0.102 dps. The first weeks' noise was, to 4 %, the datasheet's.

**One thing the datasheet model changes about the bench model, and why:** the bench model seeds the three computers' IMUs with neighbouring numbers (0x1234, 0x1235, 0x1236) and draws their constant errors from the first output of a linear congruential generator, so that the three have **almost the same** bias, scale error and misalignment (the outputs for seeds that differ by one differ by a thousandth of their range: a correlation above 0.99, pinned by a test). A real set of parts does not. The datasheet model lets its own generators run for a while and draws every constant from them, so the three are independent. The bench model is left as it was, so that every earlier flight and every documented number stays.

## 3. Sample jitter (timing in the closed loop)

`sample_jitter` takes the sample early by up to that fraction of a frame (uniform): the sensor is read at a random instant a little before the frame, by interpolating between this frame's true input and the previous frame's. It is the part of "timing in the closed loop" that reaches the gyros and accelerometers; a late or lost frame is `stale_prob` and `latency_frames` (older) and `Loop::frame_loss_prob` (lost at one receiver, `RESYNC.md`). What it does not model is the jitter of the flight computers' own tasks and of the bus: those are the live closed-loop test's (the fault campaign has them on the bus).

## 4. Presets and where to use them

`sim::ism330dhcx_typical()` and `sim::ism330dhcx_maximum()`; in a tool `--imu bench|ism330dhcx_typical|ism330dhcx_maximum` (`tfc_fly`, `tfc_mc`); in a Monte Carlo file `"sensor_preset"`, and the dispersions `gyro_noise_density_mdps`, `gyro_bias_instability_dph`, `sensor_temperature_offset_c` and `sample_jitter` (`MONTE_CARLO.md`). The shipped `vehicles/dispersions.json` uses the typical preset with a temperature drawn from -20 to +40 degC above 25 and a jitter of up to 0.3 frame (assumptions).

## 5. How it is checked (`tests/test_imu_datasheet.cpp`, 12 tests; 41 mutants of this code, one equivalent (with its reason))

- **Nothing on, the bench model sample for sample**: 200 samples equal to the formula with the generator replayed beside it (bit for bit).
- **White noise**: the rms of 300,000 samples against density x sqrt(bandwidth) to 1 % (0.102 dps and 1.22 mg), no mean, a kurtosis of 3 +-0.08, 0.27 % beyond three sigma.
- **The Allan deviation of white noise falls as one over the square root of the averaging time**: a million samples (10,000 s) at 0.1, 1, 10 and 100 s against sigma sqrt(dt/tau), each to 8 %. (The firmware reads at 100 Hz and the noise bandwidth is 416 Hz: the samples alias the noise, so the integrated gyro's random walk is about 2.9 times the datasheet's angular random walk, which is for the sensor's own output rate; this is a property of reading at 100 Hz, not of the model.)
- **The bias instability**: a Gauss-Markov process with the standard deviation given (to 10 %) and fallen to 1/e after one correlation time (to 0.06), over 1,500 correlation times; the axes wander independently.
- **Temperature**: the drift of the zero-rate level over 20 degrees within +-0.1 dps and reaching it, no preferred sign (400 sensors); the sensitivity change exactly 100 dps x 0.00007 x 20 = 0.14 dps, and 1 mg for 1 g.
- **Cross-axis**: within 1 % and 0.5 %, reaching them, no mean, the input's own axis untouched, and a rate about Y leaking into X and Z.
- **Jitter**: a ramp read early by 0 to 0.5 frame, mean 0.25, the accelerometer early by the same fraction, and the first frame on time.
- **The neighbouring seeds**: the bench model's correlation above 0.99, the datasheet model's below 0.15.
- **The presets** are the numbers of the table above, and the worst case is worse in every one that has one.
- **The closed loop** flies the reference vehicle with the typical and the maximum sensors, a temperature of 20 and 40 degrees and a jitter of 0.2, finite, without Safe, and the same every time.
- **The wandering bias starts from its stationary distribution** (the first samples of 600 sensors have the standard deviation given), and **each setting of the model alone switches it on** (a density, an instability, a temperature with a coefficient, a cross-axis term, a jitter); a channel without a density keeps the bench's uniform noise (bounded, rms amplitude / sqrt 3), whichever way round.
