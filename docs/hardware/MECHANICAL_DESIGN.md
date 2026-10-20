# Mechanical design: everything to draw in NX, and everything to measure for it

> Status: **plan** (written 20 Oct 2026; nothing is drawn yet, no CAD file is in the repository). The complete list of the parts to design for the rig, what to measure before drawing each one, what to check after it is printed, and the decisions taken (section 7). It joins three sources: the owner's parts sheet (`Triplex_Flight_Computer_Parts_v4.xlsx`, tabs *Parts List* and *3D Prints*), the requirements and designs of this repository, and gaps found by reading the two together (marked **added**). **Every dimension quoted from the sheet is the vendor's listing or the owner's estimate, not a measurement**; a number here becomes a fact only when a `BENCH_LOG.md` entry exists for it.

The CAD tool is Siemens NX Student Edition; the printer is the Ultimaker 3 Extended (PLA and PETG, 2.85 mm filament). The sheet's weights and print times are the owner's rough estimates (0.2 mm layers, 15 to 20 % infill) and are to be replaced by the slicer's figures.

## 1. How to use this page

1. Section 2 is the list: **what must be designed**. Section 3 is the list of **what must be measured**, each with an id (`MEAS-nn`). Section 4 is one brief per part: its inputs (which measurements), what it must contain, what to check after the print. Section 5 holds the checks that cross several parts. Section 6 is the order of work against the schedule. Section 7 is the seven decisions, as taken on 20 Oct 2026.
2. **Measure, then draw.** The sheet itself says that the Nucleo's hole pattern and the IMU's hole size are not known (audit items 22 and 23). A rack drawn from a drawing and printed once is a rack printed twice.
3. **Record each measurement** as a `BENCH_LOG.md` entry (the block at the top of that page: item, measured value and how, the limit, the verdict; for a dimension the limit is "n/a" or the part's drawing). The procedure that asks for them is `P-M1-01` step 11 (section 3 of this page is its list).
4. **A part is done** when: the NX part and the STEP are made, the STL is sliced, the weight and the print time are recorded, a test fit passed (a coupon or a first print) and is in `BENCH_LOG.md`, and the part is photographed in place. The files live in `hardware/cad/` as STEP and STL (section 7, item 6).

## 2. What must be designed

Sheet numbers are the *3D Prints* tab's. `MECH-00` and `MECH-11` to `MECH-13` are **added by this page**; `MECH-05` is the sheet's part **redrawn**.

| Id | Part | Qty | Material | Sheet | Needed for | Why it exists |
|---|---|---|---|---|---|---|
| MECH-00 | **Print calibration coupons** (added) | 1 set | PLA, PETG | none | everything | Hole shrinkage, insert pilot holes, a 10 mm bearing pocket, a 30 mm servo slot, a screw-boss, in the two materials, on this printer. Fixes the tolerances of every other part before it is drawn |
| MECH-01 | **Board rack**: the four Nucleo slots A, B, C, ACT | 1 | PLA | 1 | S1, S3, S4 | Holds the flight computers and ACT, labelled, with their CAN Pals, the cable channel and the lines to the supervisor |
| MECH-02 | **IMU mounts** | 3 | PLA | 2 | S2 | Fix each ISM330DHCX to the platform in an identical orientation |
| MECH-03 | **Gimbal frame, 2-axis** | 1 | PETG | 3 | **S2** | The motion platform: two servo-driven tilts, hard stops, the IMU carrier |
| MECH-04 | **Servo brackets** | 2 | PETG | 4 | **S2** | Clamp the two D85MG servos to the frame |
| MECH-05 | **Fault-injector enclosure**, redrawn for **three** relay modules | 1 | PLA | 5 | S2b, S3 | Pico 1 and the injector's relay modules (HARDWARE_PARTS section 6, item 3) |
| MECH-06 | **Strain-relief and cable clips** | 10+ | PLA | 6 | S1 on | A wire pull must not tear out a connector; the wires over the moving gimbal |
| MECH-07 | **Base plate as tiles** (two or three) | 1 | PLA | 7 | S1 on | The tray everything mounts to |
| MECH-08 | **E-stop and fuse box** | 1 | PLA | 8 | S2 | The 22 mm E-stop, the 5 A fuse holder, the servo-rail terminals (TFC-PLAT-003, override H1) |
| MECH-09 | **CAN backbone board mount** | 1 | PLA | 9 | S1 on | The 5 x 7 cm backbone board with its 3-pin terminals |
| MECH-10 | **Power distribution plate** | 1 | PLA | 10 | S1 on | The WAGO levers, the two jack adapters, the node-rail capacitor |
| MECH-11 | **Supervisor mount** (added) | 1 | PLA | none | S2b | Pico 2 (the supervisor), the TCXO module, its wiring board and relay module 2 (the four `PWR` relays). Not in any printed part of the sheet |
| MECH-12 | **Override panel** (added) | 1 | PLA | none | S2b | The manual switches H2 to H5 (and the servo-tester board of H3), labelled and reachable. Not in the sheet. **The switches are not ordered** (`HARDWARE_PARTS.md` section 4) |
| MECH-13 | **Vehicle stand-in** (added, **optional**) | 0 or 1 | PLA | (in row 3's text) | demo | The sheet says the gimbal "holds the stand-in engine"; the repository needs only the IMU carrier. A rocket or engine silhouette on the tilting part is for the video and is a decision (section 7) |

Not printed and not planned: a mount for the USB hub (it has its own 12 V adapter and sits off the plate) and for the logic analyzer (it plugs into a PC port). The sheet's total for the ten printed parts is 625 g and 38 h (estimate); it does not include MECH-00 or MECH-11 to MECH-13.

## 3. What must be measured

All items are **open**: none has a value in `BENCH_LOG.md`. The 6 Oct entry for the IMUs says the hole values were *not recorded*; measure them again. A caliper (0.02 mm) and a steel rule are enough; photograph each part on graph paper or beside the caliper.

### 3.1 Printer and process
| Id | Measure | For |
|---|---|---|
| MEAS-01 | The printable area in Cura for this printer, single and dual extrusion (the sheet says 215 x 215 x 300 mm and budgets the base plate under 210 x 210; I recall the dual-extrusion area is smaller: check) | MECH-07 |
| MEAS-02 | The results of MECH-00: the diameter a 3.0, 3.4 and 2.2 mm hole prints at, the pilot hole for each insert, the slot clearance (the sheet says about +0.2 mm), the 10 mm bearing pocket's interference in PETG (the sheet's target is 0.05 to 0.1 mm), the thinnest wall that prints, the layer direction against strength at a boss | every part |
| MEAS-03 | The heat-set inserts actually received: outside diameter, length, knurled or tapping type (the sheet's own warning), and the kit's chart | every part with an insert |

### 3.2 Boards and modules
| Id | Measure | For | Source today |
|---|---|---|---|
| MEAS-04 | **Nucleo-G474RE**: outline, thickness, mounting holes (**count**, diameter, positions from one corner), the Micro-B connector (position, how far it overhangs), the tallest feature with the headers and Dupont plugs fitted, the positions of the **reset button**, **JP5** and the user LED, the morpho connectors CN7 and CN10 | MECH-01, 07 | none; UM2505 Fig. 6 is a drawing without the numbers (audit item 22); ST publishes a 3D model |
| MEAS-05 | **CAN Pal**: outline, holes (if any), the screw-terminal block's position and height, the **termination switch** position (it is checked OFF), the header side | MECH-01 | none |
| MEAS-06 | **ISM330DHCX board**: outline, thickness, mounting holes (**count, diameter, spacing**), the header height once soldered and which side it is on, the position of the sensor, **the silkscreen's axis arrows** (which edge is +X, +Y), the STEMMA QT connector position | MECH-02, 03 | none (audit item 23); the sheet says M2 with nylon washers fits a 2.5 mm hole |
| MEAS-07 | **Pico 2**: outline, the 2.1 mm holes' positions, the Micro-B position, header height | MECH-05, 11 | the sheet: holes 2.1 mm, use M2 |
| MEAS-08 | **Relay module** (ELEGOO 4-channel): outline, holes, board height with the relays on, the screw-terminal side and height, the VCC/JD-VCC jumper's access (it is removed), the LEDs | MECH-05, 11 | the sheet's listing: 72 x 51.7 mm, 3.1 mm holes 65.9 x 45 mm apart; confirm |
| MEAS-09 | **5 x 7 cm prototype board**: holes, thickness; **with the 3.5 mm terminal blocks soldered on**: height and how far they stand off the edge | MECH-09, 11 | the sheet: 50 x 70 mm, 2.54 mm grid |
| MEAS-10 | **TCXO / RTC module**: outline and holes, once the model is known | MECH-11 | **model not chosen** (`HARDWARE_PARTS.md` section 6, item 1) |

### 3.3 Actuators and bearings
| Id | Measure | For | Source today |
|---|---|---|---|
| MEAS-11 | **D85MG body**: length, width, height, the mounting ears (hole spacing, diameter, ear thickness), the **output shaft's position** along the body and from the ear plane, the spline's height above the case, where the cable leaves and its connector's size, the mass | MECH-03, 04 | the sheet: 29 x 13 x 30.0 mm; "size the bracket slots for 30.0 mm height" |
| MEAS-12 | **The horns supplied**: arm length, hole pattern and diameter, thickness, the screw | MECH-03 | none |
| MEAS-13 | **The servo's real travel and neutral**: the angle against the pulse from 850 to 2350 us, and where the horn sits at 1500 us. The sheet says 145 degrees; a comment in `sim/vehicle/platform.hpp` says "about 60 degrees". Settle it by measuring | MECH-03 (stop positions) | `P-M1-01` steps 9, 10; `PICO_TESTS.md` E9 |
| MEAS-14 | **623ZZ bearings**: outside diameter, bore, width of at least three, and the fit of the M3 x 20 axle | MECH-03 | the sheet: 3 x 10 x 4 mm |

### 3.4 Power, safety and the switches
| Id | Measure | For | Source today |
|---|---|---|---|
| MEAS-15 | **E-stop**: the panel hole it needs (the sheet says 22.3 mm), the mushroom's diameter (to twist it), the depth behind the panel with the contact blocks, the panel thickness it takes, any anti-rotation flat | MECH-08 | the sheet: 22 mm button |
| MEAS-16 | **The blade-fuse holder**: body length and width, lead direction, how the fuse is pulled | MECH-08 | none |
| MEAS-17 | **WAGO 221** 2-, 3- and 5-way: outside size and the lever's swing | MECH-10 | none |
| MEAS-18 | **The DC jack to screw-terminal boards**: outline, holes | MECH-10 | none |
| MEAS-19 | **The two 5 V wall adapters**: the body, the cable's exit, the length of the plug | MECH-10 | none |
| MEAS-20 | **The 1000 uF capacitors**: diameter, height, lead spacing | MECH-08, 10, 03 | none |
| MEAS-21 | **The override switches** (H2 guarded toggle, H3 two DPDT, H4 and H5 key or toggle) and the servo-tester board: panel hole diameter, depth, guard height, outline | MECH-12 | **parts not ordered** |
| MEAS-22 | **The rubber bumper**: diameter and thickness | MECH-07 | the sheet: 0.4 in round |

### 3.5 Wires and cables
| Id | Measure | For |
|---|---|---|
| MEAS-23 | The **insulation diameter** of the 22 AWG hook-up wire, the stripped Cat6 pair, the **Dupont female housing** and the **Micro-B plug body** (width x height x length), the tightest bend that does not stress them | MECH-06, 01, 03 |
| MEAS-24 | **Wires per node slot, counted on the real harness**: the sheet counts 5 jumpers for a CAN Pal and 7 for an IMU; the repository adds `FRAME`, `KICK`, `NRST`, `T0`, the reserved `ADOPT` and two node-id straps (`firmware/README.md` pins), the 5 V feed and the USB cable | MECH-01 (the channel's size) |

### 3.6 After the parts are printed and assembled
| Id | Measure | For | Where it connects |
|---|---|---|---|
| MEAS-25 | **Weight and print time** of each part from the slicer and from a scale, against the sheet's estimate | all | the sheet asks for it |
| MEAS-26 | **The gimbal's travel** on each axis with the stops fitted: it must reach plus and minus 45 degrees with margin and stop inside the servo's travel | MECH-03 | TFC-PLAT-001, 004 |
| MEAS-27 | **Slop and backlash** at the IMU plate (the angle the plate moves with the servo held), and the repeatability of **level** (command 0, read the IMUs, ten times) | MECH-03, 04 | `PICO_TESTS.md` E9; `SIM_FIDELITY.md` section 3 asks for the measured lag, backlash and deadband |
| MEAS-28 | **The mass of the moving assembly, its centre of mass against the two axes, and each IMU's distance from the two axes** | MECH-02, 03 | the simulator's platform model says the platform rotates but does **not accelerate** (`sim/vehicle/platform.hpp`); an IMU off the axes feels the rotation as acceleration |
| MEAS-29 | **Each IMU's alignment**: tilt the platform to known angles (a printed wedge or the servo itself against a bubble level) and compare each IMU's gravity vector with the tilt; and **the sign and axis of each IMU** against the model (section 5, item 3) | MECH-02 | `SensorErrors` models a mounting misalignment |
| MEAS-30 | **Current at idle and at the stops** (a free gimbal draws little; a binding one draws a lot) and the **shock when the E-stop is pressed at 20 degrees** | MECH-03, 08 | `PICO_TESTS.md` E3, E7 (HARDWARE_OVERRIDE G1) |
| MEAS-31 | **Each CAN stub's real length** (path, not distance) to the backbone, and **60 ohm across the bus** with the power off | MECH-01, 09 | the sheet: every stub under 30 cm |

## 4. The parts, one by one

For each: **measure first** (section 3), **must contain**, **check after**.

### 4.1 MECH-00 Print calibration coupons (added)
Measure first: MEAS-01, 03. Must contain: a row of holes 2.2 to 3.4 mm in 0.1 mm steps, the insert pilot holes the kit's chart gives (M2 and M3), a 10.0 mm pocket row at 9.9, 10.0 and 10.1 mm for the 623ZZ, a slot row at 30.0, 30.2 and 30.4 mm for the servo, a boss with an insert, a 2 mm wall. Print in both materials, in the orientation the real parts will use. Check after: MEAS-02, written down as the **tolerance table every later part uses**. Cost: minutes of print time; it replaces guessing on the five parts that need a fit.

### 4.2 MECH-03 Gimbal frame and MECH-04 Servo brackets (the critical path)
Measure first: MEAS-11, 12, 13, 14, 06, 20; MEAS-02 for the bearing pocket and the slot.
Must contain:
- **Two tilts, about X and about Y, of at least plus and minus 45 degrees each** (TFC-PLAT-004). On each axis the servo drives one side and a 623ZZ bearing carries the other (M3 x 20 axle, 10.0 mm pocket). **The nesting is fixed by the model, not free: the Y tilt is the outer axis and the X tilt the inner one**, so the X servo rides on the outer ring (section 5, item 1).
- **Hard stops** outside plus and minus 45 degrees and inside the servo's travel (start at plus and minus 50 degrees: **a screw into an insert with two positions**, retuned after MEAS-26; section 7, item 7), so that a runaway commanded to the end of the servo's range meets a stop and not the servo's gear train. Set the stop angles from MEAS-13, not from either of the two figures in the documents. The stops also keep the stall rare (the sheet's reason for the 5 A fuse).
- **The IMU carrier**: a flat plate with the three IMU mounts, at the axes' intersection or as near it as the geometry allows (MEAS-28), a flat reference face to put a level on, a keyed seat for each mount so it cannot be fitted rotated.
- **The cable path**: three IMUs of about 7 wires each (21 wires, the sheet's count) cross the moving joints. Give them a service loop that is not pulled at 45 degrees, a clip on each moving member (MECH-06) and no wire across a pinch point.
- **Servo clearance** at full travel: the 30 mm height, the horn, the cable; the servo's own cable and the 1000 uF capacitor at each servo connector (sheet row 22).
- **A fixed foot** to the base plate (four inserts) and clearance of the whole swept volume.
- Servo brackets: a slot for the measured body (the sheet: 29.0 x 13.0 x 30.0 mm, tolerance from MEAS-02), the ear holes at the measured spacing, a clamp that does not rely on the ears alone.
Check after: MEAS-26, 27, 28, 29, 30. In NX, put the assembly through plus and minus 45 degrees on both axes with the stops and look for interference before printing (use the assembly constraints and, if the edition has it, a motion or clearance check; otherwise step the angle by hand). Print the **servo bracket and one bearing pocket first** (the sheet's own advice for the bracket).

### 4.3 MECH-02 IMU mounts
Measure first: MEAS-06 (holes, header height, axis arrows), MEAS-02.
Must contain: the IMU's holes at the measured spacing (M2, nylon washers if the hole is 2.5 mm), standoffs high enough that the soldered header clears, a bottom face flat and parallel to the carrier, a key, a label A, B or C that matches the flight computer, a **reserved footprint for a small regulator board** (section 7, item 3: reserved, not built for v1; TFC-ARCH-010 plans each IMU on a rail that does not depend on its own computer, and `P-M1-01` step 12 decides how). Print **all three in one batch** with the same settings and orientation (the sheet).
Check after: MEAS-25, 28, 29. The mass of each is small against the servo's 3.6 kg-cm at 4.8 V (the sheet): an estimate, not a measurement; confirm it when the platform runs.

### 4.4 MECH-01 Board rack
Measure first: MEAS-04, 05, 23, 24.
Must contain:
- **Four identical slots** (A, B, C, ACT), the same pattern in each (STAGED_BUILD rule 6), labelled in the CAD; M3 inserts at the measured hole pattern.
- **Access** to what the operator touches: the Micro-B connector with room for the plug and a cable's bend, the **reset button** (an override path in `HARDWARE_OVERRIDE.md`), **JP5**, the user LED, the morpho connectors CN7 and CN10 (the supervisor lines and the node-id straps are on them), and the CAN Pal's **termination switch**.
- **A place for each CAN Pal** next to its Nucleo (not hanging on its wires).
- **A channel** sized from the real wire count (MEAS-24) and the bend radius (MEAS-23) for: the CAN stub, the IMU's SPI wires, the supervisor lines (`FRAME`, `KICK`, `NRST`, `T0` and the reserved `ADOPT`), the node-id straps, the 5 V feed.
- **The stub budget**: every CAN stub is under 30 cm (the sheet). Four Nucleos in a row with the backbone at one end may not meet it for the far slot once the path is counted (section 5, item 2).
Check after: MEAS-31, and that every board can be removed without moving the others.

### 4.5 MECH-07 Base plate (or tiles)
Measure first: MEAS-01, 22, and every outline of MEAS-04 to 09, 11, 15 to 19.
Must contain: first a **layout in NX**, one sketch with the outline of every board, box and plate (MECH-01, 03, 05, 08, 09, 10, 11, 12) and the swept volume of the gimbal; then **two or three tiles** (for instance the compute tile with the rack, the backbone and the power plate, and the platform tile with the gimbal and the E-stop box) bolted together with locating pins so that the parts stay aligned (section 7, item 4). Four to six rubber bumpers (the sheet), recessed to the bumper's thickness. Within hand's reach: the E-stop, the override panel, the reset buttons. Cable exits away from the platform's swing.
Check after: the whole rig on the plate at once, nothing in the gimbal's path, the plate flat on the desk.

### 4.6 MECH-05 Fault-injector enclosure (redrawn)
Measure first: MEAS-07, 08.
Must contain: Pico 1 (the platform driver and the injector) and **two** relay modules (module 1: the four node-power cuts; module 3: sensor-line and bus-stub faults), the carrier board for the servo headers if it is in the box, screw bosses at the measured holes, **vents** (the sheet), the exits for the four power cables to the nodes and the sensor and bus wires, the USB cable's exit. Label the eight channels. The sheet drew it for two modules and "relay/MOSFET module" in the singular; HARDWARE_PARTS section 6 item 3 records the redraw as open. **The disarm switch H4 is in series with these modules' coil supply**: it is on the override panel (section 7, item 5), with a wire to the box.
Check after: MEAS-25; every relay channel can be reached with a probe before a node is connected (`P-M1-01` step 7).

### 4.7 MECH-11 Supervisor mount (added)
Measure first: MEAS-07, 08, 09, 10.
Must contain: the second Pico 2, the TCXO module, the wiring board (about 17 lines: four `FRAME`, four `KICK`, four `NRST`, four `PWR`, `SAFE`, `SUPERVISOR.md`), and **relay module 2** (the four `PWR` relays, each in series with a node's feed, and so in series with the injector's cut). The pull-ups and pull-downs are soldered on the board, so keep it removable. The module is **separate from the injector's box on purpose** (the supervisor must be independent of the fault injector, ADR-022): different box, own USB cable.
Check after: the two relays in series on one node's feed can be traced by eye; each line is labelled.

### 4.8 MECH-12 Override panel (added; the parts are not ordered)
Measure first: MEAS-21 (after the switches are bought).
Must contain: H2 FORCE-SAFE (a guarded toggle), H3 PLATFORM-LEVEL (two DPDT switches, one per servo, with the servo-tester board), H4 INJECTOR-DISARM and H5 SUPERVISOR-DISARM (key or toggle), the five sense lines to the supervisor (4.7 kOhm pull-downs, `CONFIG_TFC_OVERRIDE_LINES_FITTED=0x1F`), a label under each switch, and the H1 E-stop within reach (it is in MECH-08). **Design the holes as parameters and print only once the switches are in hand**; a panel printed for a switch that was never bought is wasted.
Check after: `P-HWO-01` can be run on it step by step; each switch's engaged state is visible from the operator's seat.

### 4.9 MECH-08 E-stop and fuse box
Measure first: MEAS-15, 16, 20, 19.
Must contain: a **22.3 mm hole** (the sheet) with room to twist the mushroom head and the depth behind for the two NC contact blocks; the fuse holder with the fuse **replaceable without opening the box** (my suggestion); the servo-rail terminals; screw bosses with inserts; strain relief for the 18 AWG leads. The wiring is: adapter, fuse, the two NC contacts in series with the servo + line, the servos (TFC-PLAT-003; `HARDWARE_OVERRIDE.md`).
Check after: the E-stop cuts the servo rail with the box closed; the leads cannot be pulled out.

### 4.10 MECH-09 CAN backbone board mount
Measure first: MEAS-09.
Must contain: the 5 x 7 cm board on standoffs; the **eight 3-pin terminal blocks** with finger room to unplug each one (an unplugged block is a manual bus isolation, `HARDWARE_OVERRIDE.md`); the two 120 ohm resistors visible; every stub entry on the side nearest its node.
Check after: MEAS-31.

### 4.11 MECH-10 Power distribution plate
Measure first: MEAS-17, 18, 19, 20.
Must contain: the two jack boards (node rail and servo rail) with room for the wall adapters' plugs; the WAGO levers (the ground star, the node +5 V split, the servo +5 V split, spares), clipped in the way the sheet says (with zip ties); the node rail's 1000 uF capacitor with its polarity visible; **the two rails share one ground point** (audit item 26).
Check after: both rails read inside 4.75 to 5.25 V at a Nucleo's `E5V` pin under the real load (`BENCH_LOG.md` 6 Oct notes this is still to be measured).

### 4.12 MECH-06 Strain-relief and cable clips
Measure first: MEAS-23.
Must contain: clips for the wire gauge, for a Micro-B cable, for a Cat6 pair; a clip that holds the IMU wire loop on a moving member; **print spares** (the sheet).

### 4.13 MECH-13 Vehicle stand-in (optional)
No measurement but the gimbal's. Decided (section 7, item 2): **not built until the platform has passed MEAS-26 to 29.** A silhouette that mounts on the tilting member adds mass and moves the centre of mass off the axes: when it is built, make it light, removable with two screws, and repeat MEAS-27 and 28 with it fitted. It is for the video, not for the evidence.

## 5. Checks that cross several parts

1. **The platform's frame is the model's frame.** `sim/vehicle/platform.hpp` gives the gravity in the sensor frame as `[-sin(ty), cos(ty) sin(tx), cos(ty) cos(tx)]` (g): level, **+Z reads +1 g, that is up**; a positive tilt about Y makes X read negative. The mount must put each IMU's Z up and its X and Y along the two tilt axes; **the sign and the axis map are then checked on the bench** (MEAS-29) because the firmware's driver documentation does not say how the board's axes map to those. A wrong sign is a platform that tracks backwards. **The same formulas fix the nesting** (checked numerically on 20 Oct 2026 against rotation matrices, random angles within 45 degrees): the gravity and the body rates `p = tx', q = ty' cos(tx), r = -ty' sin(tx)` are exactly those of a platform turned first about Y (the outer ring) and then about the rotated X (the inner ring); with the opposite nesting the gravity differs by up to 0.19 g and the rates by up to 4.3 rad/s. A gimbal built the other way round would not match the simulator when both axes move at once.
2. **The stub budget** (MEAS-31). Four Nucleos are about 70 mm long and about 53 mm wide (my recollection of the Nucleo-64 outline: measure it, MEAS-04). A row of four, with the backbone at one end, puts the far node's stub at roughly 20 cm plus the routing. The sheet's limit is 30 cm. Put the backbone in the middle of the rack, or lay the slots in two rows, if the layout sketch says so.
3. **Lever-arm acceleration.** At the Pico's 300 degrees per second limit (5.2 rad/s), an IMU 50 mm from the axis sees about 1.4 m/s^2 (about 0.14 g) of centripetal acceleration, which the model leaves out. This is **my arithmetic**: it says to keep the three IMUs close to the intersection of the axes and to measure their distances (MEAS-28), not that 0.14 g matters until it is tested.
4. **Three IMUs on one rigid plate.** They share the platform's motion by design; a flexible plate would let them disagree for a mechanical reason and the voter would call it a sensor fault. Make the carrier stiff, and compare the three IMUs' readings at rest and in motion (the same test as the noise soak of `P-S1-02`).
5. **Two relays in series on every node's power.** The supervisor's `PWR` relay and the injector's cut are in series (P-HWO-01 step 4): the harness from the two boxes to the rack must keep the two paths distinguishable.
6. **Reach.** The E-stop, the override panel and every reset button are reached without crossing the platform's swing.
7. **Heat-set inserts in PETG and PLA.** The sheet says PLA is fine for the gimbal at these loads and prints it in PETG; use the process of MECH-00 for both and do not mix.

## 6. Order of work and the schedule

M2 (20 Oct to 3 Nov, `STATUS.md` section 5) holds S1 (one computer), S2 (ACT, the Pico and the platform) and S2b (the supervisor). The platform is the part with the most iterations and it is on the critical path of S2; the rack is needed from S1.

| Step | Do | Parts | Needs |
|---|---|---|---|
| 1 | Measure what is in hand and log it | MEAS-01 to 09, 11 to 20, 22, 23 | the parts (arrived 8 Oct, except the override parts and the TCXO) |
| 2 | Print the coupons; fix the tolerance table | MECH-00 | MEAS-01, 03 |
| 3 | Draw the layout sketch in NX | (for MECH-07) | step 1 |
| 4 | Servo bracket and a bearing pocket, test-printed; then the gimbal and the IMU mounts | MECH-04, 03, 02 | step 2; MEAS-11 to 14, 06 |
| 5 | The platform assembled and measured (it can run before the rack exists, on the desk) | MEAS-26 to 30 | `P-S2-01`, `PICO_TESTS.md` E1 to E9 |
| 6 | The rack, the backbone mount, the power plate, the clips | MECH-01, 09, 10, 06 | MEAS-04, 05, 24 |
| 7 | The base plate | MECH-07 | steps 3 to 6 |
| 8 | The E-stop box (before the platform runs unattended) | MECH-08 | MEAS-15, 16 |
| 9 | The injector box, the supervisor mount | MECH-05, 11 | MEAS-07 to 10 |
| 10 | The override panel, once the switches are bought | MECH-12 | MEAS-21 |

Steps 4, 5 and 8 are what S2 waits for. The sheet's estimate for all ten printed parts is 38 hours of printing; a failed print repeats its share.

## 7. Decisions that are the owner's

> **Decided 20 Oct 2026**: the owner accepted the recommendation on every row. The reasons are kept, because a decision whose reason is lost is reopened. "Reopen if" says what would change it. Items 1 and 6 are decided before the first part is drawn; item 4 is confirmed by the layout sketch (section 6, step 3).

| # | Decision | Decided | Why | Reopen if |
|---|---|---|---|---|
| 1 | **How the two axes are driven** (the nesting itself is fixed: Y outer, X inner, section 5 item 1) | The X servo rides on the outer ring and drives the inner ring **directly**, no push rods | The model fixes the nesting; push rods add play at the joints, and play at the IMU plate is what MEAS-27 is there to keep small; the outer servo carries about 22 g more, small against the D85MG's 3.6 kg-cm (the sheet) | MEAS-27 shows the direct drive's slop is larger than the servo's own, or the outer servo's current at the stops (MEAS-30) is high |
| 2 | **The stand-in** (MECH-13) | **None for now**: a plain IMU plate. A light silhouette is designed after the platform passes MEAS-26 to 29, removable with two screws, printed last | It adds mass off the axes, so MEAS-27 and 28 would be repeated with it; the video benefits, the evidence does not | S2 is done and the video needs it |
| 3 | **Power for each IMU** (TFC-ARCH-010, STAGED_BUILD rule 11) | **Reserve, do not build for v1**: leave a footprint in MECH-02 for a small regulator board and count about 3 more wires per IMU across the gimbal in MEAS-24 | The requirement is "planned" and the ring re-homing it serves is outside v1 (`STATUS.md` section 8); adding the space later costs a reprint and a re-harness | `P-M1-01` step 12 shows the IMU board is unsafe on a Nucleo's 3.3 V, or ARCH-010 is brought into v1 |
| 4 | **One base plate or tiles** | **Tiles, two or three** (for instance the compute tile: rack, backbone, power; and the platform tile: gimbal, E-stop box), bolted together with locating pins | Four Nucleos, three relay modules, two Picos and the rest will not fit under 210 x 210 mm; the sheet estimates the single plate at 10 h and a large flat print is the one that warps; a tile can be reprinted alone; the platform tile runs on the desk for S2 before the rack exists | The layout sketch fits everything under the printable area (MEAS-01), which I doubt |
| 5 | **Where H4 and H5 live** | **On the override panel** (MECH-12), with a wire to each box | One labelled place to reach in an emergency, and the five sense lines leave from one connector; the wires carry only relay coil current (about 70 mA a channel, the sheet), so their length is not an issue; a broken wire opens the coil supply, which disarms the injector and leaves every node powered, the safe direction (`HARDWARE_OVERRIDE.md`) | The coil-supply wire to a box is long enough to drop the coil voltage (measure it, MEAS-21) |
| 6 | **Where the CAD lives** | `hardware/cad/`: the **STEP and STL of every part committed**, with a README listing each part's key dimensions and the `BENCH_LOG.md` entries they come from; the **NX part files kept off the repository** and backed up in the owner's Drive | STEP outlives the student licence (which I believe is time-limited: check) and any NX version, and a reviewer can open it without NX; the files are small enough that Git LFS is not needed; `console/models/` is the precedent for not committing a tool's own binary files | The STEP files grow past a few megabytes each, or the licence terms allow the `.prt` files to be shared |
| 7 | **The stop angle** | **Start at plus and minus 50 degrees, adjustable**: the stop is a screw into an insert with two positions, retuned after MEAS-26 | The firmware clamps at 45 degrees (TFC-PLAT-001) and the servo's lag and backlash can carry it a little past; 5 degrees is about 11 % of the travel and sits well inside the servo's range if it is 145 degrees. **This is my judgement, not a calculation** | MEAS-13 shows the servo's travel is much less than 145 degrees (the comment in `platform.hpp` says about 60), or MEAS-27 shows more than a degree or two of play |

## 8. Where it connects to the rest

| Page | What it says about this |
|---|---|
| `HARDWARE_PARTS.md` | Section 2.5 (mechanical hardware), section 6 (open item 3: the injector box for three modules), section 7 (audit items 12, 21 to 26) |
| `STAGED_BUILD.md` | Rule 6: the rack has four slots; rule 11: the IMU power and the second SPI port |
| `../procedures/P-M1-01-parts-and-bench-checks.md` | Step 11 is the measurement of section 3 of this page |
| `../procedures/P-S2-01-actuator-and-platform.md`, `../verification/PICO_TESTS.md` | The platform's tests E1 to E9 that MEAS-26 to 30 feed |
| `../design/PICO.md`, `../design/HARDWARE_OVERRIDE.md`, `../design/SUPERVISOR.md` | What each box must carry |
| `../verification/REQUIREMENTS.md` | TFC-PLAT-001 to 004, TFC-ARCH-010 |
| `../design/SIM_FIDELITY.md` | Section 3: the model's platform is a lag, a rate limit and a travel limit; backlash and the platform's bandwidth are to be measured |
| `../verification/RISKS.md` | R24: the mechanical build as a schedule risk |
