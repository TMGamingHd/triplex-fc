# The world the vehicle flies in: planets, rotation, J2, atmospheres, wind and turbulence

> Status: **built** (accepted 6 Oct 2026, ADR-031): `PlanetSpec` in `sim/vehicle/spec.hpp`, and its use in `vehicle6.hpp`. The reference model's world is the default (a spherical, non-rotating Earth, the 1976 standard atmosphere, a mean wind with scripted gusts) and **every flight and every documented number still uses it, bit for bit**. The presets for other bodies are standard values written from memory of the usual references and **not checked against a source** here.

## 1. What it is for

The reference model puts the vehicle on a non-rotating Earth with no oblateness, in one atmosphere, in a wind with a fixed shape. That is a good first world and a poor general one: a launch from Cape Canaveral starts at 408 m/s eastward and flies in air that moves with the planet; an orbit precesses; a Mars lander meets a thin atmosphere with a different gas; a lunar ascent has no air and a sixth of the gravity; a day can be hot or dense. `planet` in the vehicle file, and `site`, `turbulence` and `wind_profile` in its scenario, give the vehicle those.

```json
"planet": {"preset": "mars", "j2": 0, "density_scale": 1.1, "temperature_offset_k": -10},
"scenario": {"site": {"latitude_deg": 28.5, "azimuth_deg": 90}, "turbulence": {"sigma_ms": 2.5, "scale_length_m": 300, "seed": 5},
             "wind_profile": [[0, 1], [5000, 9]], "start": {"altitude_m": 200000, "circular_orbit": true}}
```
A `preset` (`"reference"` (the default), `"earth"`, `"moon"`, `"mars"`) is applied first and any other field of `planet` overrides it.

## 2. The planet

| Field | What | Reference | `earth` | `moon` | `mars` |
|---|---|---|---|---|---|
| `radius_m` | the sphere the vehicle stands on and the gravity is measured from | 6,378,137 | 6,378,137 | 1,737,400 | 3,396,200 |
| `mu_m3_s2` | the gravitational parameter: `g = mu / r^2` | 3.986004418e14 | same | 4.9028e12 | 4.282837e13 |
| `rotation_rate_rad_s` | about the pole | **0** | 7.2921159e-5 | 2.6617e-6 | 7.0882e-5 |
| `j2` | the oblateness term of the gravity field | **0** | 1.08263e-3 | 2.034e-4 | 1.96045e-3 |
| `atmosphere` | `"us1976"`, `"exponential"` or `"none"` | us1976 | us1976 | none | exponential |

The shape is always a **sphere**: the oblateness acts on the *gravity* (J2), not on the surface, so a launch at latitude 28.5 degrees stands about 5 km further from the centre than the real surface does (the sphere has the equatorial radius), and the altitudes are the sphere's.

**Rotation.** The simulator's frame is inertial. The pole's direction in it comes from the launch site: with `latitude_deg` and the `azimuth_deg` of the launch direction (from north through east), the pole is `(sin lat, cos lat cos az, cos lat sin az)` in the frame whose X is the local vertical, Y downrange and Z crossrange. A vehicle on the pad therefore:
- **has the planet's velocity**, `omega x r`: `omega R cos(lat)`, 408 m/s at 28.5 degrees and 465 m/s at the equator (checked);
- **turns with it**: its gyros read the planet's rotation (15 degrees an hour for the Earth), which is what the pad calibration of `LAUNCH_SEQUENCE.md` exists to remove;
- **has no wind from it**: the air moves with the planet, so the relative velocity is `v - omega x r - wind` and the dynamic pressure on the pad is zero (checked);
- **stays on its pad** if it cannot lift off (a thrust-to-weight under 1): held, it goes round the pole with the planet, and the condition to rise includes the centripetal acceleration the pad has: the pad is carried round a circle and needs a force toward the centre of only `g - omega^2 R cos^2(lat)`, so a lift-off on the equator needs **less** than the weight, by 0.35 % for the Earth (checked: 0.994 of the weight stays down, 0.999 lifts). A pad that is held to the ground is also judged by its speed relative to the pad, not the inertial one: the pad's own velocity is perpendicular to the vertical, but its dot product with it is rounding noise, and a vehicle "moving away" by 1e-10 m/s was let go for good and flew off along the tangent (found by the mutation run of this code; fixed).

The pitch program and the tilts the flight computers use are measured **from the inertial X axis**, the local vertical at T-zero. The pad rotates away from it (0.004 degree a second for the Earth: about 2 degrees in 8 minutes), so on a long flight on a rotating planet the "vertical" drifts against the program: it is the vehicle that is simulated, and the program is designed on the same model, so the loop is consistent; but it is not what a real inertial guidance does, which flies to the local horizon.

**J2.** The acceleration is the point mass's plus `(3/2) J2 mu R^2 / r^4 [ (5 z^2 - 1) r_hat - 2 z p_hat ]`, with `z = r_hat . p_hat` and `p_hat` the pole. It is checked against the textbook: an orbit of 700 km at 60 degrees regresses its node at `-(3/2) J2 n (R/a)^2 cos i` over two orbits to within the test's 6 %, and holds its inclination to 0.0005 radian.

## 3. The atmosphere

- **`us1976`**: the standard atmosphere to 86 km, an exponential tail above (`VEHICLE_SIM.md`). Unchanged.
- **`exponential`**: `rho = rho_0 exp(-h / H)` at the constant temperature `temperature_k`, `p = rho R T`, `a = sqrt(gamma R T)`, with `surface_density_kg_m3`, `scale_height_m`, `gas_constant` and `gamma`. Mars: 0.020 kg/m^3, 11.1 km, 210 K, 188.9 J/(kg K), 1.29.
- **`none`**: no density, no pressure; the engines give their vacuum thrust at the surface (an engine in air loses `p_a A_e`); a speed of sound of 300 m/s is a placeholder so that a Mach number can be formed.
- **Dispersions** of any of them: `density_scale` multiplies the density and the pressure (a denser day), `temperature_offset_k` raises the temperature at the same pressure (a hot day: the density falls by `T / (T + dT)` and the speed of sound rises by `sqrt((T + dT) / T)`). Both are checked against the gas law.

## 4. The wind

The mean wind is a profile of speed against altitude along a fixed direction (`wind_direction`), multiplied by `wind_scale`; the built-in profile peaks at 28 m/s at 12 km. **`wind_profile`** `[[altitude, speed], ...]` replaces it (linear between points, held beyond them). The scripted 1-cosine gusts are unchanged.

**Turbulence** (`turbulence`: `sigma_ms`, `scale_length_m`, `seed`) is a random wind on top: each of the three components is a first-order Gauss-Markov process, `u' = a u + sigma sqrt(1 - a^2) N(0,1)` with `a = exp(-V dt / L)`, `V` the airspeed (at least 20 m/s) and `L` the length scale (533 m by default, 1750 ft, the value of Dryden's model above 2000 ft). It has the variance `sigma^2` and a correlation that falls by `1/e` over the distance `L` (both checked on a 600 s run), is the same on every machine for a seed (a xorshift generator and Box-Muller), and is zero when `sigma_ms` is 0. **This is Dryden's shape for the along-wind component used for all three**; the real model's lateral and vertical components are second-order filters with a different spectrum, a refinement not made.

## 5. What is not modelled

Gravity of other bodies (the Sun, the Moon): negligible for a launch, not for a long coast; solar radiation pressure; the geoid, tides, the atmosphere's rotation relative to the planet's (a jet stream is in the wind profile); day-to-night and seasonal variations beyond the two dispersions; wind shear layers as a random field; the non-spherical surface; aerodynamic heating; ionospheric and magnetic effects. A planet in the file with `rotation_rate_rad_s` and a site is as faithful as these simplifications allow.

## 6. How it is checked (`tests/test_environment.cpp`, 12 tests; 33 mutants of this code)

The speed of a point on a rotating sphere at three latitudes and the equator's 465 m/s; no wind at rest on the pad; a held vehicle at the position the planet's rotation has carried the pad to after ten minutes, with the planet's velocity and rate; the nodal regression of an inclined orbit against the textbook formula; the exponential atmosphere against its definition (Mars at one scale height is `0.020/e`), no atmosphere giving the vacuum thrust, a denser and a hotter day against the gas law; a wind profile interpolated and held; the turbulence's variance, its `1/e` correlation at a lag of `L/V`, determinism, and the seeds; the Moon's hover threshold (a thrust of 0.99 of the weight stays down and 1.01 lifts) and a lunar orbit keeping its radius and energy; the pole's direction at any azimuth; the lift-off threshold on the equator (0.994 of the weight stays down, 0.999 lifts: 0.35 % less than the weight, the pad's centripetal acceleration); a vehicle held at 88 sites never released by rounding noise; and the file's presets, overrides, refusals and the round trip. With every setting at its default the reference vehicle's flights are still bit for bit those of the frozen oracle.
