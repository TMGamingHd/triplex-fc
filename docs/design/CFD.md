# CFD: an Euler flow solver for the vehicle's body

> Status: **built, validated on exact solutions, and run on the Starship-class body** (accepted 17 Oct 2026, ADR-050): `sim/cfd/` (the solver, header only), `tools/sim/tfc_cfd.cpp` (the tool), `tools/cfd/make_library.py` (a library of cases), `tests/test_cfd.cpp` (9 tests), `sim/tests/test_console_cfd.py` (the viewer's side), and the viewer's CFD view (`console/web/viewer/js/cfd.js`, the Aerodynamics lens). The simulator's *flight* does not use it yet (section 8); it is a second, independent answer to "what does the air do to this body", next to the engineering build-up of `AERODYNAMICS.md`.

## 1. What it is, and what "full CFD" is not here

Computational fluid dynamics is the numerical solution of the equations of the moving air round a body. The most complete form (the Reynolds-averaged Navier-Stokes equations with a turbulence model and the chemistry of the hot air in a hypersonic shock) is a research code and a cluster. This is the first rung of that ladder, and a real one: **the Euler equations**, which are the Navier-Stokes equations without viscosity and heat conduction, solved in **three dimensions** on the **real outline of the vehicle** (the sections of its file), for a **calorically perfect gas** (gamma 1.4).

It is the method of NASA's Cart3D, which was used for the Space Shuttle's and the SLS's aerodynamic databases: a Cartesian grid, the body cut out of it, shocks captured, pressure forces integrated. It is good at exactly what an inviscid solution is good at, and the Mach number range where it matters (supersonic and hypersonic: a shock stands off the nose, expands round the shoulder, and the pressure on the skin follows) is the range the engineering build-up of `AERODYNAMICS.md` handles worst.

| It gives | It does not give |
|---|---|
| the shocks and expansions, where they are and how strong | skin friction (the part of the drag that comes from the boundary layer) and the heating |
| the pressure on every part of the skin | separation, the wake, and so the **base pressure** (an Euler solution has no wake: the number is reported, apart, and means little) |
| the force and moment from it: CA of the skin, CN, the centre of pressure | the vortices that roll up from a body at an angle of attack, and the forces they add |
| the flow field, for the viewer | real-gas effects in a hypersonic shock (the air dissociates; gamma is not 1.4), so a hot-gas temperature is not claimed |
| any Mach number from about 2 up | transonic flow (Mach 0.8 to 1.3: the boundary layer decides where the shock sits) |
| | flaps, grid fins and fixed fins: the geometry is the **body alone** for now (section 3) |

The next rungs, in order, are a viscous term and a turbulence model (the skin friction and the separation), flaps and fins as part of the geometry, and the real-gas equation of state. None is started.

## 2. The equations and the numerics

The unknowns in each cell of the grid are the density, the three components of the momentum and the total energy; `euler.hpp` and `solver.hpp` solve

    dU/dt + dF/dx + dG/dy + dH/dz = 0,    p = (gamma - 1) (E - rho |u|^2 / 2)

by the finite-volume method:

* **The grid** is a tensor product of three lines of cell faces (`graded_nodes`): cells of one size over the body and a little beyond it, growing by 10 % a cell up to four times that far from it, so that a body 124 m long and 9 m wide has about 350,000 cells at 0.6 m. The plane z = 0 is a plane of symmetry (the body is symmetric about it: the angle of attack is in the x-y plane), so the grid is half the vehicle.
* **The flux** across a face is HLLC (Toro): two waves and a contact, which keeps a contact and a shear sharp, with the wave speeds of Einfeldt from Roe averages; where the pressure differs by more than 1.8 times across the face (a shock lying along it) the flux is **HLL** instead, which has no contact and therefore cannot grow the carbuncle (a spurious bump on a bow shock that HLLC alone is known to make on a grid-aligned shock). Measured on the sphere (section 5) the choice moves the stagnation pressure by less than 1 %.
* **The reconstruction** to second order is MUSCL on the primitive variables (density, velocity, pressure) with the **van Albada** limiter (smooth, it does not flatten an extremum as the minmod does); a reconstruction that is not a physical state (a negative pressure or density) falls back to the cell's own value. Spacing-aware, so the grid may stretch.
* **The time** step is a two-stage Runge-Kutta (the strong-stability-preserving one of Heun) at a **local** step (each cell at its own CFL of 0.6) toward a steady state, or one global step for a time-accurate run (the shock tube). The density residual falls 2 to 4 orders in a few thousand iterations and stalls there (the limiter chatters in the shock); the forces stop moving long before, and the tool stops on that (the forces unchanged by 0.2 % over 500 iterations, after 1,000) or on a residual target.
* **The boundaries**: the free stream at the inflow, a characteristic (Riemann-invariant) far field where the flow can go either way (the sides, at an angle of attack), extrapolation at the outflow, and a mirror at the plane of symmetry.
* **Threads**: a persistent pool cuts each loop into equal planes, one part per thread; every plane writes only its own cells, so **the result is the same bit for bit whatever the number of threads** (a test: 1 and 5).

Speed, measured: 0.41 microseconds per cell per iteration on one core (an optimised build; the unit tests run 6 to 8 times slower, with the sanitisers), 6 ms an iteration for 51,000 cells on twelve threads. The Starship body at Mach 3 and 4 degrees: 95,000 cells 14 s, 177,000 cells 36 s, 353,000 cells 81 s, 735,000 cells 152 s.

## 3. The body

The body is **a signed distance** (`shape.hpp`): a function that is negative inside and positive outside, and the distance to the surface at either side. `Revolution` makes it from the outline of the vehicle (`vehicle_body.hpp`: the radius at stations along the sections of the stages, a nose with its stations crowded at the tip, the same outline the viewer draws); `Sphere`, `HalfSpace`, `Box`, `Union` and `Intersection` are there for the checks and for what comes next.

A cell whose centre is inside is **solid**. A solid cell within two cells of a fluid one along an axis is a **ghost cell** (the second-order stencil reaches two cells): at each stage it takes the state of the fluid at its **image point**, the point mirrored in the surface, with the velocity's component along the normal reversed, so that on average no air goes through the surface. That is the slip wall of the Euler equations (the **ghost-cell immersed boundary** of Tseng and Ferziger). It needs no mesh of the body, and a body of any shape is one function; the price is that the wall is not aligned with the grid, so a stagnation point is the weak case (section 5).

**The forces** are read at the surface elements of the shape (a patch of the skin with its place, its outward normal and its area): the pressure at 0.5 and 1 cell off the surface along the normal, extrapolated linearly to the surface (`probe_extrapolate`). The reason is in the data: behind a cone's shock the pressure falls with the distance from the wall (it is real: the flow is isentropic along the streamline and the Mach number rises toward the shock), so a single reading at 1.5 cells read 2 to 7 % low on the cone; read at 0.5 and 1 cell and extrapolated, the error is under 1 % at every resolution tried. The force is the gauge pressure (over the free stream's) times the normal and the area, over the half body and its mirror image.

**Frames**: the body's axis is x from the tail (the vehicle file's x) with the nose at the largest x; the air moves toward -x at the Mach number, tilted by the angle of attack in the x-y plane (the velocity of the vehicle through the air is +x and +y: the side it moves toward, y > 0, is the windward one). CA is positive for drag (`-F_x / q S`), CN positive toward the side the flow comes from (`-F_y / q S`), the reference area is that of the largest diameter (or `aero.reference_diameter_m`), and the **centre of pressure** is where the moment about a reference point says one force would have to act: `x_cp = x_ref + M_z / F_y`, which includes the moment of the axial force (the usual definition from a moment coefficient).

## 4. Running it

    build/host/tfc_cfd --validate [--quick]                                   # the solver against the exact solutions: section 5
    build/host/tfc_cfd vehicles/starship.json --mach 3 --alpha 4 --h 0.6      # one flow: CA, CN, CN-alpha, the centre of pressure
    build/host/tfc_cfd vehicles/starship.json --machs 2,3,5 --alpha 4 --table cfd_table.json   # a table in the form of aero.table
    build/host/tfc_cfd vehicles/starship.json --mach 3 --alpha 4 --field console/cfd/starship/m3_a4.cfd   # the flow for the viewer
    python3 tools/cfd/make_library.py vehicles/starship.json                  # Mach 2, 3, 5, 8, 12 at 0, 4 and 10 degrees

`--h` is the cell over the body (default a sixth of its radius: **use 8 or more cells across the radius for a number you will quote**, section 6), `--stage-from K` takes the body of the stages from K on (the ship alone: 1), `--probe` chooses how the surface pressure is read, `--xref` the point the moment is about. A case takes 30 to 150 seconds on twelve cores.

## 5. Checked against exact solutions

`tfc_cfd --validate` (589 s on twelve cores for the whole set; `--quick`, 85 s, leaves out the finer grids and the sphere) runs each case on several grids and prints the error against the exact answer, which comes from `gasdyn.hpp`, written without any of the solver's code: the Rankine-Hugoniot normal shock, the oblique shock (theta-beta-Mach), Rayleigh's pitot formula, the **Taylor-Maccoll** equation for the flow over a cone (integrated with a Runge-Kutta from the shock to the surface), and Toro's exact solution of the Riemann problem. Measured, 17 Oct 2026:

| Case | Quantity | Grid | Error |
|---|---|---|---|
| Sod's shock tube (time-accurate, 1D in the 3D code) | L1 error of the density at t = 0.2 | 100, 200, 400 cells | 0.0062, 0.0033, 0.0018 (first order at the discontinuities, as it should be) |
| Wedge of 10 degrees, Mach 2 | pressure behind the shock | 40 x 17, 80 x 34, 160 x 68 | 0.02 %, 0.00 %, 0.00 % |
| | shock angle (39.31 degrees) | the same | 0.96 %, 0.10 %, 0.10 % |
| Cone of 15 degrees, Mach 2.5 | surface pressure (1.805 x free stream) | 5.4, 7.7, 10.7 cells across the base radius | 0.76 %, 0.54 %, 0.60 % |
| | shock angle (28.45 degrees) | the same | 2.2 %, 1.6 %, 1.6 % |
| Cone of 10 degrees, Mach 5 | surface pressure (2.308 x free stream) | 9.9, 14.1 cells | 1.9 %, 1.5 % |
| | shock angle (15.61 degrees) | the same | 0.8 %, 0.5 % |
| Sphere, Mach 3 | pressure at the stagnation point, read within 6 degrees of the axis (12.06 x free stream, Rayleigh) | 10, 14 cells across the radius | **-8.9 %, -3.1 %** |
| | shock stand-off (Billig's correlation, 0.205 radii) | 10, 14 cells | +17 %, +12 % (one cell is 10 % and 7 % of a radius) |

What this says: a smooth, slender body (a cone, a wedge, the ogive of a launcher) is right to about 1 % in pressure at 8 or more cells across its radius; **a blunt nose is not**: an immersed boundary reflects the flow about a wall that does not lie along the grid, and at a stagnation point that costs 9 % at 10 cells and 3 % at 14 (it falls fast with the grid). The flux and the limiter hardly matter there (HLL against HLLC, a switch at 1.8 or never: 11.36 to 11.42 at 10 cells; the Van Leer limiter 11.6). The Mach 5 cone's error is larger than the Mach 2.5 one's and falls slowly with the grid because the shock is thin and close to the surface there.

The nine unit tests (`tests/test_cfd.cpp`, 10 s with the sanitisers) check the relations against their textbook numbers, a uniform stream staying uniform to 1e-12, Sod's tube, the wedge, a cone on a coarse grid (a smoke test: the number is in the table above), a body in still air leaving it still (1e-9), the surface elements adding up to the areas of the shapes (a sphere, a cone) and their normals pointing out, a constant pressure making no force on a closed body, and the result not depending on the number of threads.

## 6. On the vehicle: the Starship-class body, Mach 3, 4 degrees

Grid study (`tfc_cfd vehicles/starship.json --mach 3 --alpha 4`, the whole stack, 124.4 m long, 9 m wide, the moment about 40 % of the length):

| Cell over the body | Cells | Cells across the radius | CA (skin) | CN-alpha (per rad) | Centre of pressure from the tail | Time |
|---|---|---|---|---|---|---|
| 1.0 m | 95,000 | 4.5 | 0.204 | 3.05 | 111.2 m | 14 s |
| 0.8 m | 177,000 | 5.6 | 0.212 | 2.94 | 113.8 m | 36 s |
| 0.6 m | 353,000 | 7.5 | 0.206 | 2.90 | 113.8 m | 81 s |
| 0.45 m | 735,000 | 10.0 | 0.211 | 2.90 | 116.1 m | 152 s |

CN-alpha is settled to about 1 % from 7.5 cells across the radius; the centre of pressure moves by a metre or two (it is a small moment over a small force: 1.8 % of the length); CA varies by 2 % between grids, as the steps of the interstage (a forward-facing step is the weak case of an immersed boundary) are resolved better or worse. The simulator's engineering build-up, for the same body, has CN-alpha **2.0** and the centre of pressure at **116.9 m** at every Mach number (`aero_summary`; the full-regime model of section 7 of `AERODYNAMICS.md` changes the body's force above Mach 3 but is not on for this vehicle).

**Over Mach number and angle of attack** (`tools/cfd/make_library.py`, 0.7 m cells, 7 cells across the radius, 246,000 cells, 30 to 70 s a case; CN-alpha and the centre of pressure from the 4-degree case, CA from the case at no angle of attack; the build-up from `aero_summary`):

| Mach | CA skin, CFD | CA, build-up | CN-alpha, CFD (per rad) | CN-alpha, build-up | x_cp, CFD (m from the tail) | x_cp, build-up |
|---|---|---|---|---|---|---|
| 2 | 0.228 | 0.344 | 2.61 | 2.00 | 115.7 | 116.9 |
| 3 | 0.204 | 0.272 | 2.97 | 2.00 | 113.7 | 116.9 |
| 5 | 0.176 | 0.199 | 3.69 | 2.00 | 97.2 | 116.9 |
| 8 | 0.156 | 0.152 | 3.74 | 2.00 | 88.8 | 116.9 |
| 12 | 0.146 | 0.126 | 3.37 | 2.00 | 90.1 | 116.9 |

What the two models say, then. The **CFD's normal-force slope is 1.3 to 1.9 times the build-up's** (the build-up's 2.0 is slender-body theory for a body with a cylinder and a nose: it does not grow with the Mach number), and its **centre of pressure moves 20 to 30 m forward at hypersonic speed** (the build-up keeps 116.9 m at every Mach number): the centre of pressure is within a calibre of the nose at Mach 2 and 3 and well down the body from Mach 5. The CFD's **axial force is lower** at Mach 2 to 5 (skin only: the friction and the base, which an Euler solution does not have, are in the build-up's number, and at Mach 3 the base alone in this solution is +0.06 to +0.09 depending on the grid, which it should not be trusted for) and about the same or higher above Mach 8. Which is nearer the real vehicle cannot be said from here: neither is a measurement of the real vehicle, whose outline in the file is estimated, and the Euler solution has no viscosity (the flow separates from the real body where it does not from this one).

The force is **not linear in the angle**: CN at 10 degrees is 0.555 at Mach 2 but 1.02 at Mach 3 (CN/alpha 3.2 and 5.9 per radian), 1.14, 0.97 and 0.85 at Mach 5, 8 and 12, and the centre of pressure at 10 degrees is 72 to 83 m for Mach 3 and above (the lift is spread over the whole body: a crossflow). The cross-flow Mach number is M sin(alpha): 0.35 at Mach 2 and 0.52 at Mach 3, and a cylinder's flow in a cross-stream goes sonic on its shoulder at about 0.4, so what jumps between Mach 2 and 3 is probably the crossflow's wave drag (a physical effect of this inviscid flow), and not a defect; it is **not** a check, and a CN-alpha from the 4-degree case is a slope at 4 degrees.

## 7. The flow for the viewer (the CFD view)

`tfc_cfd --field FILE` writes one file for the 3D viewer: a JSON line (the case, the coefficients, the body's outline, the grids), the pressure coefficient on the skin on a grid of stations along the outline and angles round it (0 to 180 degrees, the mirror half is not stored), and a box round the body of three 16-bit volumes (pressure ratio and density ratio on logarithmic scales, the Mach number; 0 means inside the body), 0.4 body radii beyond the nose and the tail and 2.6 radii to each side. About 0.7 MB a case for the Starship (the two committed in `console/demo/` are Mach 3 and Mach 8 at 4 degrees). The layout is checked by `sim/tests/test_console_cfd.py` against what the solver wrote (the numbers start on a multiple of four bytes: the browser views them as `Float32Array` and `Uint16Array`).

The console serves them from `console/cfd/` (the library, not in the repository: `console/cfd/README.md`) and `console/demo/` (the few that are), as `/api/viewer/cfd` (the list, read from each file's header) and `/api/viewer/cfd/file?name=` (a name from the list only).

The **CFD** view of the Aerodynamics lens (key 2, then *CFD*) picks the case nearest the flight's Mach number (in its logarithm) and angle of attack, or the one you choose, and draws:

* **a section through the flow** in the plane of the vehicle's angle of attack (turned to the flight's own plane, as the angle of attack is in a plane that rolls), as the **Mach number**, the **pressure** over the free stream's (a tenth to a hundred, logarithmic) or a **schlieren** picture (the gradient of the density, which is what a shadowgraph shows: shocks light, expansions dark). The undisturbed air is left clear, so that what you see is the disturbance. A slider moves the section off the plane of symmetry (z in cells).
* **the solver's pressure on the skin**, in the colours of the Pressure view, in place of the Newtonian estimate (the shader looks it up by the body's length and the angle round it from the windward side).
* a card, **What the solver says**: the case's Mach number, angle of attack and grid; CA, CN, CN-alpha and the centre of pressure, beside the **build-up's** CN-alpha and centre of pressure at the present moment of the flight, so that the two models can be compared on the same screen.

![The CFD view at Mach 3 and 4 degrees: the live solver's section as the Mach number, with the library case's pressure on the skin](img/cfd-view-mach.jpg)

![The same as a schlieren picture: the bow shock off the ogive as a dark line, the density's gradient in grey](img/cfd-view-schlieren.jpg)

The section drawn here is the live solver's (section 10); a computed case is **not live**: it is at its own Mach number and angle of attack, labelled with both and with the flight's; nothing is interpolated between cases. The live solver (next section) is the part that moves.

## 8. What feeds the flight, and what does not

Nothing feeds it. The flight's forces are those of the engineering build-up (`AERODYNAMICS.md`), and the plant's `aero.table` (a table of CA, CN-alpha and the centre of pressure by Mach number) is **used only when a vehicle has no sections**: a table made by `tfc_cfd --table` is in the right form, but it cannot replace the build-up for a vehicle that flips and enters at large angles, since the table is linear in the angle and the build-up's cross-flow is what flies the booster's flip and the ship's belly entry.

What was done instead is a **measured question**: *does the flight survive the aerodynamics the solver gives?* The build-up's normal-force slope is scaled in the mission file (`"vehicle": {"cn_scale": X}`, the `vehicle.cn_scale` knob) and `tfc_mission vehicles/missions/starship_return.json --pad 1500 --seed N` flies the whole mission (the flight computers, the booster's boost-back, entry and catch, the ship to orbit) over 40 GNSS-noise seeds, 17 Oct 2026:

| CN-alpha of the plant | Ship's orbit (perigee over the seeds) | Booster caught by the tower |
|---|---|---|
| 1.0 x (the design) | 221 to 243 km | **40 of 40** (median miss 0.8 m) |
| 1.5 x (the solver at Mach 3 is 1.5 x) | 224 to 243 km | **4 of 40** (median miss 2.1 m, worst 4.5 m) |
| 1.9 x (the solver at Mach 5 to 8) | 224 to 244 km | **9 of 40** (median miss 3.0 m, worst 4.7 m) |

(and, seed 0 only, 1.3 x and 2.5 x: the orbit is 233 to 238 km perigee, the booster misses by 2.0 and 1.8 m.) So: **the climb and the orbit are robust to a plant whose normal force is 2.5 times what the design assumed** (the controller is already holding a vehicle that is 6 calibres unstable), **and the booster's catch is not**: the controller lands the stage 2 to 4 m from the tower's arms, which close at 1.5 m, once the air turns the stage more strongly than its plan thinks. Nothing here says the real stage does that: the solver is inviscid and the outline is an estimate. It says where the margin of this design is, which is the catch and not the ascent, and that a better model of the aerodynamics (this one) is worth a plan that tolerates one (`RECOVERY.md`'s owed list: a plan robust to the separation state is the other).

## 9. What is not done

The base pressure (a wake model), skin friction and heating, a viscous solver, flaps and grid fins in the geometry (the same `Box` shape and the surface table of the vehicle file), the real-gas equation of state, transonic flow, a grid that refines itself where a shock is (the 4 times coarser far field is fixed), mutation testing of the new code, the live solver's angle of attack (it is axisymmetric), the live solver's skin pressure, and a library for the ship alone and the booster alone (the library is of the whole stack: after separation the CFD view says there is no case).

## 10. The live solver (`console/web/viewer/js/gpuflow.js`)

The same scheme in two dimensions, on the graphics card, as WebGL2 fragment shaders: the **axisymmetric Euler equations** in the plane of the axis and the radius, `d(rU)/dt + d(rF)/dx + d(rG)/dr = (0, 0, p, 0)`, on a uniform grid (about 640 x 160 cells for the Starship: 0.23 m, 20 cells across the radius), with the state in four float textures (`EXT_color_buffer_float`). Each iteration is four passes: fill the ghost cells (the image point is found by four fluid-only bilinear weights; the velocity's normal component is reversed), advance one stage; again. The flux is HLLC with HLL on a strong shock, the reconstruction MUSCL with van Albada's limiter, the step local (a steady state), the axis a mirror, the inflow the free stream at the Mach number of the flight: a copy of `euler.hpp` and `solver.hpp` in GLSL, with the cylindrical source. The body is the outline of the stages on the vehicle (rebuilt when one leaves), cut out of the grid by a signed distance computed on the page.

* **It is checked on the page** (`viewer/gputest.html?token=...`, the cone of section 5, where an axisymmetric solver is exactly the Taylor-Maccoll problem; the numbers go to the console server's log). Measured on this machine (a GTX 1660 Super, headless Firefox), 233 x 100 cells, 3,000 iterations in 0.26 s (about 11,000 iterations a second at 23,000 cells): Mach 2.5, 15 degrees: surface pressure **1.797** against 1.805 (-0.4 %), shock angle 28.29 against 28.45 degrees; Mach 5, 10 degrees: **2.265** against 2.308 (-1.9 %), shock 15.62 against 15.61 degrees. The same errors as the 3D solver's on the same cones, from a different code.
* **It follows the flight**: the Mach number is set from the pose (above 1.25; below it, or where the air is thinner than the vehicle is wide, the case of the library is shown instead), a change of more than 50 % starts from the free stream, a smaller one continues from the flow it has. The flow settles in 1,000 to 3,000 iterations, a fraction of a second to a second at 60 frames a second (the iterations a frame are adjusted to the frame time, 8 to 120). It draws as the same section (Mach, pressure, schlieren), turned to the plane of the flight's angle of attack, with the radius both ways.
* **What it is not**: it has **no angle of attack** (the flow of a body of revolution at an angle is three-dimensional; the computed case has it, and the section says which it shows), it is inviscid, and it computes the section only: the pressure on the skin in the CFD view is the library's. It does not feed the flight.
* **Tests**: `selftest.js` goes through the four views of the Aerodynamics lens, and with a supersonic pose checks that the live solver has run, that its state is finite and its density positive everywhere, that a flow field loaded and that the solver card has numbers (72 of 72 checks on the recorded flight, 67 of 71 on a synthetic pose, the four that fail being those of a pose with no tanks and no booster).
