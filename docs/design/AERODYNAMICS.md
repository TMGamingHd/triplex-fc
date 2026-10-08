# Aerodynamics that follow the shape

> Status: **built** (accepted 6 Oct 2026, ADR-031): `sim/vehicle/aero.hpp` and the parts of `vehicle6.hpp` that use it. A vehicle whose stages list their outer shape gets forces and a centre of pressure from it; one that does not keeps the reference vehicle's fixed numbers, exactly as before. The formulas are published, simple ones, **checked against hand calculations of the same formulas and against physical limits, not against any wind-tunnel or CFD data** (nothing of the kind has been done, and section 6 says what that means).

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
