# A Monte Carlo of the closed loop: many flights, each with a random draw of everything that can differ

> Status: **built** (7 Oct 2026): `sim/vehicle/montecarlo.hpp`, the tool `tools/sim/tfc_mc.cpp`, `tests/test_montecarlo.cpp`, and two starting sets of dispersions in `vehicles/`. It flies the same whole chain as `tfc_sens` (the simulated vehicle, three flight functions with their IMU models, the real ACT logic), many times. **Every number in the two dispersion files is an assumption** about a generic launcher, except the sensors' errors, which are the ISM330DHCX's datasheet values; nothing here is a statement about any real vehicle, and a probability that comes out of it is a probability **under those assumptions**.

## 1. What it is for

`tfc_sens` (`SIM_FIDELITY.md` section 2) moves one departure at a time until the flight is lost: how far can each go. A real programme asks the other question: with **all** of them at once, each drawn from the distribution it has, what fraction of the flights are good, how far do the bad ones stray, and which departure is behind them? That is a Monte Carlo, and it is the way to find the worst case that the one-at-a-time sweep does not (two departures that are each harmless and together are not). It also finds what no sweep was written to look for: the first run on the launcher with the dynamics found that every one of the flights that were **destroyed** had a bending slope at the IMU in the last fifth of its dispersion (section 5).

```
tfc_mc --config vehicles/dispersions.json --flights 300 --threads 8                    # the reference vehicle, the rig's platform sensors
tfc_mc --vehicle vehicles/launcher_dynamics.json --config vehicles/dispersions_dynamics.json \
       --sensors vehicle --no-accel --pad 1500 --flights 96 --limit 10                   # a launcher, with the vehicle's own sensors
tfc_mc ... --only 34                                                                    # fly flight 34 of that run again, alone, and print its draws
tfc_mc --list                                                                           # what can be dispersed
```
The report: the good flights with the 95 % Wilson interval of the probability, why the others failed, the distribution (mean, standard deviation, minimum, median, 95th and 99th percentiles, maximum) of the largest attitude error, the lift-off transient, the rms error, the final altitude and speed and the maximum dynamic pressure, the correlation of each draw with the error and with the final speed, and the five worst flights. `--csv` writes every flight's draws and results.

## 2. How a flight is drawn

- **A flight's draws depend on the seed and the flight's index and on nothing else.** The generator is xorshift64\* seeded through splitmix64 (the same on every machine, no library distributions), normal draws by Box-Muller; each flight has its own stream. So `--only N` flies flight N of a run alone and gets the same flight, and the results do not depend on the number of threads (both are tested).
- The dispersions act on the vehicle that is **flown**; the pitch program and the gains the flight computers carry are designed once, on the nominal vehicle (`RunnerConfig::tables`).
- Each flight has its own realisation of the sensor noise and of the constant sensor errors (`Loop::noise_seed`), of the turbulence and of the lost frames.
- Distributions: `normal` (mean, standard deviation, and a clip in standard deviations, 3 unless given: a draw farther is drawn again), `uniform` (min, max), `constant`. A constant uses no random numbers, so it does not change the other draws. The sensor errors (`gyro_bias_dps`, `gyro_scale_err`, `accel_bias_g`, `accel_scale_err`, `misalign_deg`) are **limits**: each of the three computers' IMUs draws its own error, uniform within them, on each axis, from the flight's noise seed.
- 34 things can be dispersed (`tfc_mc --list`): the engines' thrust and Isp, the drag and normal-force coefficients, the thrust misalignment, the servo's lag, the masses and the centre of gravity, the wind's strength and direction and the turbulence, the density and temperature, an engine failing and when, the sensors' errors and noise and latency, and (for a vehicle that has them) the slosh, the bending mode, the backlash and the separation's push and twist. A name the file does not know is refused with the nearest one; one the vehicle cannot use (slosh on a vehicle with none) does nothing.

## 3. What a good flight is

The conditions of `tfc_sens`: after the first 3 s the attitude stays within `limit_deg` (5 degrees unless given) of the pitch program; ACT is never in Safe; the platform never saturates (a condition of the rig only: with `--sensors vehicle` it is not applied); the vehicle is off the pad by 3 s and is not destroyed; every value is finite. The limit is a choice, and a result depends on it: the same runs below are given at 5, 10 and 20 degrees.

## 4. The statistics, and how they are checked

- **Percentiles** by linear interpolation between the order statistics (the median of 1, 2, 3, 4 is 2.5); the **sample** standard deviation (n - 1).
- **The Wilson interval** for k good flights in n, not the normal approximation: it is honest at the ends, where 300 good flights of 300 do not mean a probability of 1 (the interval is 98.7 to 100 %), and 0 of 10 does not mean 0 (it is 0 to 27.8 %). 90 of 100 is 82.6 to 94.5 %.
- **Pearson's correlation** of each draw with the error and the final speed, as a ranking of the suspects. It says which draw moves with the result, **not why**, and it sees nothing of an effect that appears only past a threshold or in a combination; it is the first thing to look at, and `--only` and the CSV are for the rest.
- `tests/test_montecarlo.cpp` (12 tests): the generator's mean, variance and skew; the independence of the flights' streams; the distributions' range and clip; every statistic against a worked number; every dispersion setting the field it names; the file and its refusals; a run with nothing dispersed being the nominal flight to a hair; a flight flown again alone, and the threads changing nothing; the known effect (thrust against final speed, correlation above 0.95 in 10 flights); the flights having their own noise, wind and lost frames; 72 mutants of the code (`tools/mutation/sim_mutations.py`), all killed, and two equivalent ones with their reasons.

## 5. What it found (assumed numbers; seed 1)

**The reference vehicle** (the rig's platform sensors, `dispersions.json`, 300 flights, 9 s): 300 of 300 good within 5 degrees (98.7 to 100 %); the largest error after 3 s averages 1.15 degrees (standard deviation 0.37, 99th percentile 2.07, maximum 2.23); the final speed is 998 m/s (standard deviation 40) and the maximum dynamic pressure 31.4 kPa (standard deviation 2.0). The final speed follows the thrust (correlation +0.81) and the propellant (-0.40) and the largest error follows the turbulence (+0.22) and the wind (+0.15): small effects, none of them decisive.

**The two-stage launcher** with the vehicle's own sensors, a 15 s pad phase to calibrate the gyros, no accelerometer correction, and the datasheet's sensor errors (a zero-rate level of up to 1 dps, a scale error of 2 %, a cross-axis error taken as 1 degree), 96 flights:

| | within 5 degrees | within 10 degrees |
|---|---|---|
| `two_stage_launcher.json`, `dispersions.json` | 62 (64.6 %; 54.6 to 73.4 %) | 88 (91.7 %; 84.4 to 95.7 %) |
| `launcher_dynamics.json`, `dispersions_dynamics.json` without the bending slope | 53 (55.2 %; 45.3 to 64.8 %) | 85 (88.5 %; 80.6 to 93.5 %) |
| the same, with the bending slope at the IMU drawn from -0.1 to 0.1 per metre | 25 (26.0 %; 18.3 to 35.6 %) | 44 (45.8 %; 36.2 to 55.8 %) |

- **The gyros' zero-rate error is the biggest single contributor to the large errors.** Alone (the other dispersions off), a 1 dps zero-rate level raises the mean of the largest error from 2.5 to 4.3 degrees and leaves 58 % of the flights within 5 degrees: what the 15 s pad calibration does not take out walks away over 400 s. A longer pad did not help in this model (a 60 s pad: 16.7 % within 5 degrees, against 26.0 %); that was not investigated. Nothing in flight can correct a gyro with no accelerometer to find gravity (`SIM_FIDELITY.md` 3.2): the way to the flight computers holding a launcher's attitude over minutes is not in the estimator but in an aiding source (a GPS or a star tracker), which the rig does not have.
- **The bending slope separates the destroyed from the rest.** With the slope drawn from -0.1 to 0.1 per metre, 11 of the 96 flights were destroyed, every one of them with a slope between -0.07 and -0.10 (and a servo with the example's 6 Hz and 0.1 degree of play). Alone, the example flies slopes of -0.5 and -0.7 (`DYNAMICS.md` section 9); with the sensors' errors, the dispersions and the servo's play together, the margin is a few percent of that. A one-at-a-time sweep would not have said so.
- The remaining failures (11 of 96 beyond 10 degrees, with no destroyed vehicle) are not explained by any one draw: the largest correlations are weak (a low thrust, -1.3 standard deviations in the failing flights; a high bending frequency, +0.4). They are the departures acting together on a vehicle that is steered by a gyro it cannot check, and were not examined further.

## 6. What it is not

It is not a statement about a vehicle: the distributions are assumed, and the model's own simplifications (`SIM_FIDELITY.md`, `AERODYNAMICS.md` section 6, `DYNAMICS.md` section 8) are in every flight. It does not draw correlated departures (a cold day with a dense one is two independent draws), does not model a failure of a flight computer or a sensor (the fault campaign does: `FAULT_CAMPAIGN.md`), and treats a flight as good or bad: it does not score margins. A programme's dispersion analysis starts from measured tolerances and a requirement for the probability (and from thousands of flights, which this tool can do: 300 flights of the reference vehicle take 9 s on 8 cores).
