# Aerodynamics that follow the shape

> Status: **built** (accepted 6 Oct 2026, ADR-031; extended to the whole speed range, the full circle of angle of attack, control surfaces and parachutes on 10 Oct 2026, ADR-039: sections 7 to 9): `sim/vehicle/aero.hpp`, `newtonian.hpp`, `surfaces.hpp` and the parts of `vehicle6.hpp` that use them. A vehicle whose stages list their outer shape gets forces and a centre of pressure from it; one that does not keeps the reference vehicle's fixed numbers, exactly as before. The formulas are published, simple ones, **checked against hand calculations of the same formulas and against physical limits, not against any wind-tunnel or CFD data** (nothing of the kind has been done, and section 6 says what that means).

## 1. What it is

The reference vehicle's aerodynamics are four numbers: a diameter, a normal-force slope of 2.5 per radian, a fixed centre of pressure 12.5 m from the tail, and an axial coefficient that depends on Mach. They are right for one vehicle. A vehicle with a different nose, a longer body, fins, a flare or a payload fairing has a different centre of pressure, a different drag, and **a different answer to the question that matters most to its controller: is it stable?**

A stage can now list the pieces of its outer body, aft to forward, and its fixed fins; a payload can list its fairing:
```json
"sections": [{"kind": "tube", "x_start_m": 0, "length_m": 3.2, "d_aft_m": 0.3, "d_fore_m": 0.3},
             {"kind": "nose", "x_start_m": 3.2, "length_m": 0.8, "d_aft_m": 0.3, "d_fore_m": 0, "shape": "ogive"}],
"stabilizers": [{"count": 4, "x_le_root_m": 0.5, "root_chord_m": 0.4, "tip_chord_m": 0.15, "span_m": 0.25, "sweep_m": 0.25, "thickness_m": 0.005}]
```
The geometry of the vehicle is the pieces of the stages and payloads **still on it**: when a stage separates or a fairing is jettisoned, the aerodynamics are rebuilt without them (the two-stage example has a normal-force slope of 2 and its centre of pressure 66 m from the tail while its ogive fairing is on, and none at all once the fairing is gone: a bare tube has no slender-body lift). The alternative to a shape is a **table by Mach** of the axial coefficient, the normal-force slope and the centre of pressure (`aero.table`), for data from somewhere else; giving both is refused.

The pieces: a **nose** (a cone, tangent ogive, parabola or ellipse), a **tube** (one diameter), a **transition** (a different diameter at each end: a flare if the aft end is larger, a boat-tail if smaller). The reference area is the circle of the largest diameter (`aero.reference_diameter_m` overrides it). Coefficients are on that area.

## 2. The normal force and the centre of pressure

Barrowman's method (the one rocketry software uses), component by component, each a slope per radian on the reference area `S = pi d^2 / 4` and a place:
| Component | Slope | Centre of pressure |
|---|---|---|
| Nose | `2 (A_base - A_tip) / S` (2 for a point on a body of the reference diameter), independent of Mach | from the tip, **0.666** of its length for a cone, **0.466** a tangent ogive, **0.5** a parabola, **0.333** an ellipse |
| Transition | `2 (A_aft - A_fore) / S`: positive for a flare, negative for a boat-tail | from its forward end, `(L/3) (1 + (1 - r) / (1 - r^2))` with `r` = forward over aft diameter (a transition that narrows to a point is a cone: 2L/3) |
| Tube | none | |
| `N` fins | `K 4 N (s/d)^2 / (1 + sqrt(1 + (2 L_f / (C_r + C_t))^2))`, with `K = 1 + R / (s + R)` (the body's interference), `s` the span, `R` the body radius at the fins, `L_f` the length of the mid-chord line | from the leading edge of the root, back by `(m/3)(C_r + 2 C_t)/(C_r + C_t) + (1/6)(C_r + C_t - C_r C_t/(C_r + C_t))` with `m` the sweep (a rectangular fin: a quarter of its chord) |

The slopes add and the centre of pressure of the whole is their weighted mean, so fins pull it aft and a flare does too. **Compressibility** acts on the fins only: their slope follows the lift slope of their planform, `2 pi AR / (2 + sqrt(4 + AR^2 beta^2 (1 + tan^2(Lambda)/beta^2)))` below Mach 0.8 (Prandtl-Glauert makes it rise toward Mach 1), `(4/beta)(1 - 1/(2 AR beta))` above Mach 1.2 (falling as `1/beta`), and a straight line between, which hides the transonic peak. The slope of the nose and the flares does not depend on Mach.

## 3. The force at any angle of attack

Slender-body theory is good to a few degrees. A vehicle that tumbles, or one that turns through a large angle (a pitch-over, an abort, a loss of control), needs the force at every angle, so (unless `aero.full_angle` is false):
- the **normal force** is `q S [ sum(C_N-alpha_i) sin(a) cos(a) + eta C_dc (A_plan / S) sin^2(a) ]`: the slender-body part, odd about 90 degrees, plus the cross-flow force of the body as a cylinder in the flow across it (`C_dc` 1.2, `eta` 0.7, `A_plan` the plan area of the body and its fins). The two act at their own places (the slender part at the centres of pressure above, the cross-flow at the centroid of the plan area), so the moment is the sum, not the force times one arm;
- the **axial force** is `-q S C_A cos(a)`, with the nose-first coefficient for `cos(a) > 0` and the rear one (`aero.rear_axial`, 1.1: a blunt base facing the flow) for `cos(a) < 0`;
- the reference model had a force that was linear in the angle at any angle and **zero past 90 degrees**: that is what `full_angle: false` keeps, and what a vehicle without a shape has.

## 4. The axial force flying nose first

Skin friction, the waves of the nose, the flares and the fins, the front face, and the base:
- **skin friction**: `C_f = 0.455 / (log10 Re)^2.58` (Prandtl-Schlichting, turbulent, flat plate) times `(1 + 0.144 M^2)^-0.65` (compressibility), on the wetted area over `S`, with `Re` from the vehicle's length, the air's density and viscosity (Sutherland's law) and the speed; `aero.roughness` multiplies it;
- **wave drag**: Newtonian, `2 sin^2(half angle)` times the frontal area it covers over `S`, for the nose (times a shape factor: cone 1, tangent ogive 0.55, parabola 0.7, ellipse 0.4) and for a flare, and `4 (t/c)^2 / beta` for the fins; it is zero below Mach 0.85, builds to full by Mach 1.2, and carries a transonic peak of up to 1.6 times near Mach 1.15 (**an assumption, not data**);
- **front face**: `0.4` of its area subsonic, `0.85` supersonic, for the forward end of the forward-most piece: nothing for a pointed nose, a lot for a vehicle that ends in a flat face (the launcher after its fairing goes);
- **base**: `0.12 + 0.13 M^2` subsonic, `0.25 / M` supersonic, on the base area, **reduced by `power_on_base` (0.7) of the fraction of the main engines' thrust that is running** (the plume fills the wake: **an assumption**).

## 5. Where it is used, and what it feeds

`loads()` uses it for the force and the moment about the centre of gravity at every step. `divergence()` (the `a` the gains are designed from) is `q S sum(C_N-alpha_i (x_cp_i - x_cg)) / I`, which is positive for an unstable vehicle and **negative for a stable one**: the gain design uses `max(0, a)`, so a stable vehicle is designed for no divergence. `tfc_fly` prints, for the vehicle it flies, the slope, the axial coefficient and the centre of pressure at Mach 0.3 and 2 and the distance of the centre of pressure from the centre of gravity **in calibres** (diameters), with the verdict stable or unstable:
```
sounding rocket:  centre of pressure 1.12 m from the aft end, 1.20 calibres behind the centre of gravity (stable)
two-stage launcher: centre of pressure 66.41 m from the aft end, 9.60 calibres ahead of the centre of gravity (unstable)
```
The first is a rocket that flies itself; the second needs its controller every second of the flight.

## 6. What it is not, and what has been checked

**Not**: wind-tunnel or CFD data (the one place the model's numbers are assumed and not derived is the transonic peak, the front-face and base coefficients and `power_on_base`, each marked above); the transonic peak of the fins' lift; fin flutter, aeroelasticity, body bending; the roll the fins can induce; the interference of fins at large angles (the cross-flow term is the body's and the fins' plan area, nothing more); aerodynamic heating; laminar flow (the skin friction is turbulent at every Reynolds number, with a floor of 1e4); plume effects other than the base; ground effect and the pad.

**Checked** (`tests/test_aero.cpp`, 12 tests; mutation-tested with 44 mutants of this code): every component against a hand calculation of the same formula written separately in the test (a cone is 2 at two thirds of its length from the tip, an ogive 0.466, an ellipse 0.333; a flare is `2 (A_aft - A_fore)/S`; a transition that narrows to a point is a cone; a rectangular fin acts at a quarter of its chord; a worked swept, tapered, three-fin example); the whole against the slope-weighted mean of its parts, fins moving the centre of pressure aft; the fins' lift slope continuous at both ends of the transonic blend, rising with Mach to Mach 1 and falling above it, and its two-dimensional and slender limits; the axial force against a hand sum (skin, front face and base of a plain tube; skin, wave and base of a cone at Mach 2 and 0.5), a running engine taking away exactly 0.7 of the base drag, an ogive dragging less than a cone and a blunt cone more, the drag peaking near Mach 1.15, between 0.15 and 1.2 over Mach 0.2 to 5; the vehicle in the flow at a small angle (force `q S C_N-alpha sin(a) cos(a)`, a restoring moment for a body with its centre of pressure behind its centre of gravity, negative divergence), at 90 degrees (the cross-flow force and no axial force) and at 180 degrees (the rear coefficient), and continuous around the whole circle; the change when a stage separates (a bare cone: slope 2, two thirds from its tip); a table with the reference model's own numbers giving the reference model's own forces to 0.2 %; the file's reading, writing, and refusals.

## 7. The whole speed range and every angle of attack (`aero.full_regime`)

Sections 2 to 5 are good to a few degrees of angle of attack, flying nose first, below about Mach 3. A booster that falls engines first from 70 km, or a ship that goes belly first, is outside all of that. With `"full_regime": true` in the `aero` block the force and moment are built from three pieces and blended:

* **The slender-body build-up** of sections 2 to 4, with the cross-flow term's drag coefficient now a function of the **Mach number of the flow across the body** (`aero::cross_cd`): 1.20 in low-speed flow, rising through the transonic range to a peak of 1.60 near Mach 1.5, and falling to 1.23 (two thirds of the stagnation-pressure coefficient, the Newtonian cylinder) at Mach 25. The values are in the range of the published cylinder data (Hoerner, *Fluid-Dynamic Drag*, 1965; Jorgensen, NASA TN D-7228, 1973), not fitted to a vehicle.
* **Modified Newtonian impact theory** of the body in the stream (section 8), from a table of coefficients at 37 angles of attack.
* **What the body's own turning adds** (`AeroGeometry::rotation_increment`): the aerodynamic damping of a body that rotates, which a stability analysis needs and a model that takes the velocity of the centre of gravity alone does not have. Each slender-body part is evaluated at the local angle of attack of its centre of pressure (the velocity of that point through the air, including `w x r`) and each strip of the body (pieces of at most two metres, each with the diameter of its middle) takes the cross-flow drag of the local lateral velocity (Allen and Perkins). The increment is the difference from the load with the centre of gravity's velocity alone, so it is **zero for a vehicle that does not turn**: the steady load is whatever the other models say it is.

The blend: the slender-body model below Mach 3, the Newtonian one above Mach 6, a smoothstep between (`newtonian_from_mach`, `newtonian_to_mach`), continuous in the Mach number (`tests/test_landing_plant.cpp`). **The angle of attack runs the full circle**: flying tail first the base and the engines face the stream, the axial force changes sign, the axial coefficient is the `rear_axial` one. Skin friction (section 4) is added to the Newtonian pressure drag. With `full_regime` off, nothing changes (a test holds that).

## 8. Modified Newtonian impact theory (`newtonian.hpp`)

At Mach 5 and above the flow does not get out of the way of a body: the pressure on a face that looks into the stream is the stagnation pressure times the square of the cosine of the angle between the face's normal and the stream (Newton's sine-squared law, with Lees' modification: the maximum pressure coefficient is **1.84**, the value behind a normal shock at infinite Mach number for gamma 1.4, not the classical 2), and a face that looks away feels nothing. That one rule gives the force and the moment on any shape at any angle, which is what a blunt capsule, a stack that flies belly first and a booster that falls engines first need.

The body is the one the sections describe, turned into panels (stations along the axis of at most 48 segments a section, 36 steps of azimuth). The **flat faces** at its two ends and at every step in its radius are panels too, and **fixed fins are thin plates** under the same rule. The coefficients (axial `cx`, normal `cn`, and the moment arm sum `a0`) are summed **once**, at 37 angles from 0 to 180 degrees in steps of 5 (when the vehicle is built or loses a stage), and interpolated; `a0` is the arm sum about x = 0 so that about a centre of gravity it is `a0 - x_cg cn`, the same quantity as the slender-body model's, so the two blend.

Checked against the closed forms (`tests/test_newtonian.cpp`): a cone's drag is `Cp_max sin^2(half angle)` on its base, a hemisphere's is half of `Cp_max`, a flat face has the stagnation coefficient, a cylinder across the flow has four thirds of `Cp_max` per plan area, a cone's normal force acts a third of its length from the base, fins are flat plates, a step in the radius is a face, a shoulder faces forward.

**What it is not:** it knows nothing of the boundary layer, of shocks interacting with the body or each other, of the base pressure (the leeward side is a vacuum: zero), of real-gas effects, or of the rarefied flow above 90 km. It is the upper-bound-pressure estimate engineers use before there is data: within 10 to 20 percent for blunt shapes at Mach 6 and above and worse for slender ones at small angles, where the real flow carries more load than the Newtonian one.

## 9. Control surfaces, parachutes and heating (`surfaces.hpp`)

**A surface is a flat plate, or a lattice that acts as one, on a hinge.** A flap hinges on the tangent to the body; a grid fin turns on a shaft along the radius and swings its cells' axis from the body's axis toward the tangent. The force is that of a plate in a stream, at the surface's own place on the vehicle and with the dynamic pressure at that place (the velocity of that point through the air: the vehicle's plus its turning, so a vehicle that rotates feels the damping of its own surfaces):

* a **normal force** on the face that meets the air: `C_n(alpha) = CN_alpha sin(alpha) cos(alpha) stall(alpha) + k Cd90(M) sin^2(alpha)`, the linear lift of the planform at small angles and the cross-flow of a flat plate at large ones. For a **flap** the slope is the lift slope of its aspect ratio and sweep at the Mach number (`aero::wing_lift_slope`); for a **grid fin** it comes from a table by Mach in which the lift **drops through the transonic range** (the cells choke: 2.9 per radian below Mach 0.6, 1.5 near Mach 1, 2.2 at 1.5, falling to 1.2 at Mach 12) and the cross-flow term is multiplied by 0.35 (the lattice lets most of a cross-flow through). `Cd90` is 1.17 in low-speed flow rising to 1.84 hypersonic. A plate **stalls**: full lift to 25 degrees and none by 50;
* an **axial (chordwise) force**: 0.02 for a flap's friction and thickness, and for a grid fin the drag of its lattice from a table, 0.35 rising to **1.30 at Mach 1** and falling to 0.45;
* a **shadow factor**: a surface on the far side of a body flying belly first sees the wake, not the stream, and gets as little as 0.2 of the dynamic pressure (`stream_fraction`, a smoothstep over about 30 degrees of the side-on direction).

The numbers are design-level estimates in the range of the published data for flat plates and grid fins (Hoerner; Washington and Miller, "Grid fins: a new concept for missile stability and control", AIAA 93-0035; Theerthamalai and Nagarajan for the lattice through the transonic range): they show the right trends (the stall, the transonic drag, the loss of lift at high Mach) and are **not wind-tunnel or CFD data for any vehicle**. They are not the right size for a real grid fin; the Starship example's fins are 11.1 m^2 each by my estimate.

**A parachute** is a drag area `Cd A` that grows from zero as the canopy fills, with the square of the time over `inflation_s`, and then holds; the force is `-1/2 rho |v| Cd A v` along the velocity of the attachment point through the air. A vehicle under a canopy falls at `sqrt(2 m g / (rho Cd A))` (a test holds it to the formula).

**Heating** is reported, not fed back: the stagnation-point flux on the nose by **Sutton-Graves**, `q = 1.7415e-4 sqrt(rho / R_n) V^3` (SI units, air), the flux over the belly as a factor of it (`belly_heat_factor`) and the temperature a surface comes to when it radiates that flux from one side (`T = (q / (eps sigma))^(1/4)`, `emissivity`). It is a number for the viewer and the reader; there is no thermal protection model and no ablation.

**What the surfaces are not:** hinge moments and the loads on the actuators, the interference between a fin and the body or between fins, flutter, the plume, the grid fin's real unsteady stall. How the flight computers use them: `GNC.md` section 5; how they are checked: `RECOVERY.md` section 5.
