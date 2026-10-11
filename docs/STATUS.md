# Status

> Status: **reference**, the page to read first. Reviewed 8 Oct 2026. The parts have arrived (8 Oct 2026); the bench work starts with [`procedures/FIRST_HARDWARE_DAYS.md`](procedures/FIRST_HARDWARE_DAYS.md). The numbers in section 3 are the output of the commands next to them (section 7 has the rest).

## 1. In one paragraph

Everything that can be built and checked without the hardware is built and checked: a three-computer fault-tolerant flight computer (the voter, the fault manager, the estimator and controller, the actuator node), its simulator and virtual peers, the
supervisor, the Pico platform driver and fault injector, the bench tools and the first-hardware-days procedures. All 31 design decisions (ADRs) are accepted and every one that can be built is built. What is left is **measurement**: timing, the real
bus, the real IMUs, the servos, the relays and the supervisor's lines. Nothing in this repository has run on a board yet, and nothing here claims it has.

## 2. What is built, and how it is checked

| Area | What exists | Checked by |
|---|---|---|
| **Flight core** (`core/`, 26 headers, about 6,200 lines, header-only, no heap, no exceptions, no RTTI) | Voter, `RedundancyManager` (FDIR: 3-of-5 and leaky count, probation, strikes, sticky Safe, authenticated ground commands, releases, phases and roles, state share), sensor/compute health split, state resynchronisation, sync clock, launch gate, estimator, controller, ACT logic | The C++ suite, the campaign, mutation testing, the coverage gate (section 3) |
| **Supervisor logic** (`supervisor/`, 5 headers) | KICK/FRAME watchdog and the reset, power-cycle and DEAD ladder, boot sequencing, T-zero and the `T0` line, mission clock and its record, the RTC conversion, the override sense lines | Same, and 100 % line coverage of the supervisor headers |
| **Simulator** (`sim/vehicle`, `tools/sim`) | 6-DOF flight of **any vehicle** read from a file (stages, tanks, engines at positions, staging, throttle, fins, thrusters, wheels, a start in orbit: `vehicles/`, `tfc_fly`), aerodynamics that follow the shape, any planet (rotation, J2, atmospheres, winds, turbulence), a second-order servo with backlash, jet damping, slosh, one bending mode, a roll controller, the push and twist of a separation, the ISM330DHCX datasheet's IMU errors, the platform model, the bus runner `tfc_simd`, the closed loop; sensitivity, resync and **Monte Carlo** (`tfc_mc`) studies | Host tests; a live closed loop on `vcan0` with six processes |
| **Virtual peers and campaign** (`sim/tfc_peers`, `sim/campaign`) | Fake FC-B and FC-C with 32 fault kinds, scripted commands, record and replay through the real `core/`; the campaign with oracles; the TS-15 and TS-17 studies | 297 Python tests; the campaign (section 3) |
| **Firmware** (`firmware/`: `app`, `act`, `pico`, `supervisor`) | Four Zephyr applications: the flight computer (node A, B or C from the build), the actuator node, the Pico (platform driver and injector), the supervisor (Pico 2). They build for `native_sim`, `nucleo_g474re` and `rpi_pico2`, and pass the ELF check (no heap, exceptions, RTTI or vtables) | CI builds all four; live tests with real firmware instances on `vcan0` (triplex, closed loop, resync, split, release, launch, T0, phases; run locally, CI runs the triplex test) |
| **Flight console** (`console/`, ADR-034, [`design/CONSOLE.md`](design/CONSOLE.md)) | One web page (Python standard library, no dependencies, local): the launch sequence and go/no-go, how the three computers vote, where the vehicle is, the faults and their **measured** detection, the bus and the events, the vehicle's parameters, and a button for every ground command (two-step where the protocol says so). Starts and breaks the virtual rig; records and replays a session; reads the nodes' serial consoles, the supervisor and the Pico for the hardware day | 184 tests, 5 of them live against the real firmware; an in-page smoke test that presses the real buttons (31, 37 and 43 steps); **not run on a board** |
| **3D viewer** (`console/web/viewer/`, ADR-035, [`design/VIEWER.md`](design/VIEWER.md)) | The vehicle drawn in a procedural sky and over a launch site, from a pose of the simulator's whole state sent 50 times a second (`tfc_simd --viewer`, or `tfc_fly --pose` to a file): six lenses (overview, aerodynamics with the centre of pressure and the centre of mass, atmosphere, propulsion, loads, attitude), a Starship V3-class model with adjustable details (ADR-037) and a vehicle file for it (reported numbers and stated estimates; flown through the real flight computers to 7.54 km/s at 227 km), a glTF import for a CAD model, optional NASA Earth imagery (ADR-038: `console/tfc-imagery`; Blue Marble, Black Marble and Landsat patches at the pad), live or from a recorded flight. Every quantity is labelled the simulator's, a formula's or an illustration | 10 C++ tests and 14 mutants (all killed), 39 Python tests; live on the real firmware: 50 poses a second, 59 frames a second; **the pictures are checked by eye, GPU time on one machine only** |
| **Guidance, navigation and a mission** (`core/` `dmath`, `nav`, `peg`, `attitude`, `descent`, `mission`; `sim/vehicle` `mission_*`, `avionics`; `tfc_mission`; ADR-039, [`design/GNC.md`](design/GNC.md), [`design/RECOVERY.md`](design/RECOVERY.md)) | The flight computers navigate (strapdown, GNSS-aided, attitude aided under thrust), steer a burn to a circular orbit (PEG), hold any attitude through any angle, bring a stage back (boost-back, entry on grid fins, landing burn, **caught by a tower**) and sequence a whole mission from tables a design run makes from the plant's own probes. The plant gained engines driven by group, surfaces, parachutes, separation that spawns a vehicle of its own, a ground, a tower and the aerodynamics of every speed. Two missions are committed: a Starship V3 class stack (ship to 234 x 257 km, booster caught) and a single-stage rocket that lands under parachutes | The C++ suite (a point-mass flight of every piece; the file, tables, design and closed loop of the small vehicles), and `tfc_mission` on the two missions in CI. **Host only: no timing on the board, no firmware path, no rig, console or viewer integration** |
| **Bench tools and procedures** (`tools/bench`, `docs/procedures`) | USB-CAN bring-up, logger, jitter and bus-loss measurement, golden-run check, the golden-release record and compatibility gate; ten procedures | Tested against `vcan0` and recorded logs; **not run on the adapter or a board** |

## 3. Evidence, in numbers

| Evidence | Result (5 to 8 Oct 2026) | Reproduce |
|---|---|---|
| C++ unit, fuzz and recovery tests under ASan and UBSan | 692 tests pass | `ctest --test-dir build` |
| Python tests (peers, replay, tools, docs, the flight console) | 488 tests; **41 are live tests that skip without `vcan0`**, so a run without it proves nothing about them. With `vcan0` and the built images all pass (the live ones were first run on 8 Oct 2026; one, `test_live_simd_edges`, had a 0.3 s start-up grace shorter than `tfc_simd`'s 0.36 s start, which it has had since ADR-031, and now waits 0.8 s) | `cd sim && python3 -m unittest discover -s tests -t .` |
| Structural coverage of `core/` and `supervisor/` | 100 % of lines, 98.4 % of branches (the gate: 100 and 98) | `python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98` |
| Fault campaign: every fault kind over its input range, safety properties on every frame | 12,362 scenarios, 4,900,623 frames, **no property violated, no anomaly** | `cd sim && python3 -m campaign.run --strict` |
| Mutation testing (deliberate bugs the tests must catch) | 390 mutants, every one killed (re-run on 7 Oct 2026 after the gyro-scale change of ADR-032: 390 of 390 in the fast pass, and the campaign's 58 of 58; the equivalent ones, changes that cannot alter behaviour, are not listed; `tools/mutation/mutations.py` records each with its reason). The runner stops a test binary at its first failure, and `--fast` (no optimisation, no sanitizers) and `--exclude` (the simulator's test files) make a pass take about an hour instead of most of a day, now that the suite includes the simulator's long flights | `python3 tools/mutation/run_unit.py --fast`; `python3 -m campaign.mutate` |
| Structural coverage and mutation testing of the vehicle simulator (`sim/vehicle`) | 99.4 % of lines, 91.0 % of branches (gate 99 and 90: the general model, the file reader and the aerodynamics bring many error branches); 481 simulator mutants, every one killed (five equivalent mutants are documented, with the reason). Its sweeps found 13 and then 22 gaps in the tests, and the dynamics' first runs 8, 7 and 3 more, all closed | `python3 tools/coverage/core_coverage.py --sim-min-line 99 --sim-min-branch 90`; `python3 tools/mutation/run_sim.py` |
| The flight console (`console/`) | 190 tests (`test_console_*.py`, `test_peers_control.py`, `test_live_console.py`): its go/no-go equals the launch checklist's verdict over 400 random states; its constants equal the C++ headers; its reading of the nodes' consoles covers every printk in the firmware; its model, run over the recorded launch, finds T-zero 1,000 frames after the countdown began, the loss of node B latched by both survivors in the same frame 2 frames after B's last frame, and **max-Q at 31.5 kPa, T+64.9 s, equal to `tfc_fly`'s nominal flight**; live, a 3 dps bias on B is detected 2 frames after it starts, with the node's own reason ("vote disagreement"). The page's own smoke test passes 31 of 31, 37 of 37 and 45 of 45 steps against the real firmware | `cd sim && python3 -m unittest discover -s tests -t .`; the page's test: `console/README.md` |
| The rig flies the vehicle you choose (ADR-036) | The Launch tab builds the three flight computers for the chosen vehicle (its own pitch program and gains, the estimator without the accelerometer) in about half a minute, and the rig flies it. **Measured, 9-10 Oct 2026, live on vcan0, every example vehicle** built from the page's route and flown through the real firmware with all three computers Triplex and healthy to the end: the sounding rocket, the reference with a design knob changed, the spacecraft, the two-stage launcher, the launcher with dynamics (worst pitch error 0.11° to 2.15°), and the Starship V3 class vehicle (`vehicles/starship.json`: 0.66° worst, 227 km and 7.54 km/s at T+495 s, the viewer receiving 50 poses a second). 13 tests (`test_console_rigbuild.py`) | `sim/tests/test_console_rigbuild.py`; `docs/design/CONSOLE.md` section 8 |
| Static analysis and the flight-code standard | clang-tidy, cppcheck and `tools/check_standard.py` clean; strict warnings as errors, 2 KB stack bound | CI; `docs/verification/CODING_STANDARD.md` |
| Live tests with real firmware on `vcan0` | triplex, closed loop through max-Q, resync, sensor split, mixed releases, launch, T0 line, phases, ACT's hardware Safe line (in flight and on the pad) | `tools/bench/sil_triplex.sh --test` |

## 3a. The decisions

All 34 ADRs are accepted (ten on 5 Oct 2026, three on 6 and 7 Oct 2026, one on 8 Oct 2026) and none is open. [`decisions/DECISIONS.md`](decisions/DECISIONS.md) opens with a register: for each one, whether it is built, what it still waits for, and where. In short: every ADR is built on the host and
in the firmware; the ones that need the hardware to be *confirmed* are listed below; two are stretch goals outside v1 (the ring that re-homes an orphaned IMU, ADR-020 case 2; a second ACT, ADR-023). Trade studies: TS-0, TS-15, TS-16, TS-17 (paper part) and
TS-23 are done; the rest are plans that the rig or the closed loop will feed (`decisions/TRADE_STUDIES.md` section 2 has the status of each). No ADR waits for one of those, except that the Safe hold time and the persistence constants are
*parameters* that TS-4 and TS-1 tune on the rig.

## 4. What waits for the hardware

Each row is a thing that cannot be settled without the parts. "Where" is the procedure or tool that will settle it; the order of the first days is in [`procedures/FIRST_HARDWARE_DAYS.md`](procedures/FIRST_HARDWARE_DAYS.md).

| # | What | Why it waits | Where |
|---|---|---|---|
| 1 | Frame jitter (p99 at most 100 us), WCET of the step and the vote, bus load (SYS-001 to 003) | Needs the target and a logic analyzer; the firmware's status line already prints `wcet_step`, `wcet_vote`, `wcet_frame` | `P-S1-01`, `tools/bench/frame_jitter.py` |
| 2 | **The real bus loss rate** | It sets the resync period, the digest persistence and the manager's command tolerance (TS-23, `RESYNC.md` 6b); nothing simulates real CAN | `tools/bench/bus_loss.py` |
| 3 | Sensor self-test limits (FDIR-036), arrival-margin telemetry (FDIR-037) | The part and the board | `FUTURE_WORK.md` 1.1, 1.2 |
| 4 | The supervisor on its board: `KICK`, `FRAME`, `NRST` and `PWR` against real nodes; the `T0` pin; the RTC and the TCXO's drift | The second Pico 2, the TCXO module and the third relay module should arrive with the order (check the confirmation) | `P-S2-03`, `MISSION_CLOCK.md` |
| 5 | The Pico: USB enumeration, servo pulse and jitter, relays at 3.3 V drive, what the servo does with no signal | The board and the servos | `P-M1-01`, `PICO_TESTS.md` |
| 6 | The platform: the shock at the E-stop, stall current, the platform's bandwidth against the time-scaled vehicle | Mechanics | `PICO_TESTS.md` E1 to E6, `SIM_FIDELITY.md` |
| 7 | **The hardware overrides** | TS-17 chose the set (O3: H1 to H5, about 12 USD of parts, none priced); **the parts are not in the order**: buy them before `P-HWO-01` | `HARDWARE_OVERRIDE.md`, `procedures/P-HWO-01-overrides.md` |
| 8 | CAN effects: bus-off recovery (FDIR-010), a babbler's real harm (FDIR-009), inconsistent omission, the sync-master takeover on hardware (F15) | Needs real transceivers | `P-S3-01`, `P-S4-01` |
| 9 | The closed loop on the real platform; TS-4 (Safe hold time) and TS-1 (persistence) re-tuned on measured data | The rig | `P-S2-01`, `TRADE_STUDIES.md` |
| 10 | The golden release (ADR-021) | A tagged release that has passed the hardware stage exits; the recorder and the gate exist | `procedures/P-REL-01-golden-release.md` |
| 11 | **The flight console's serial paths**: the Nucleos' consoles, the supervisor's commands and the Pico's relays and status, read and sent on the real ports | The boards; tested against pseudo-terminals and a fake Pico only | `design/CONSOLE.md` section 10 |

## 5. Schedule

All hardware is ordered; bring-up is in stages S1 to S4 (`hardware/STAGED_BUILD.md`), and only nodes that have passed their stage are powered and plugged into the bus. Each stage ends with a tagged commit and a short log or video.
Slack is intentionally in M4 and M5: if M2 slips, cut stretch goals, not the fault campaign; if M3 slips, ship a tested Duplex demo and say so.

| # | Dates | Goal | Done when |
|---|---|---|---|
| M0 | by 6 Oct | The order is placed; CI on GitHub; Zephyr on `native_sim` | **Done**: the order is placed and arrives 9 Oct. Check the confirmation for the second Pico 2, the TCXO module and the third relay module (decided 4 Oct) |
| M1 | 9 to 20 Oct | The parts arrive: inspect each, set JP5, measure the adapter (under 5.25 V), bench-check a relay at 3.3 V drive and a servo at 3.3 V signal; first board runs | The bench checks are logged in `hardware/BENCH_LOG.md`; Nucleo 1 blinks and prints |
| M2 | 20 Oct to 3 Nov | **S1** one flight computer on hardware (10 min at 100 Hz, timing measured); **S2** ACT, the Pico and the platform; **S2b** the supervisor | Zero frame errors; WCET and jitter recorded; the platform tracks commands |
| M3 | 3 to 17 Nov | **S3** FC-B (Duplex), **S4** FC-C (Triplex); takeover, consensus, the 2-of-3 vote; the platform tracks a simulated ascent | Unplug any one node: the output is unchanged; voting on against off is recorded |
| M4 | 17 Nov to 1 Dec | The hardware repeat of the fault campaign, detection-time histograms | The fault matrix is filled with measured data |
| M5 | 1 to 15 Dec | Write-up, video, README, tag v1.0 | Every box in `project/PROOF.md` is ticked |

## 6. Not in v1

A second ACT, the ring re-homing of an orphaned IMU, a second bus, dual-slot boot, the telemetry dictionary and event records, analytical redundancy for Duplex, AI-assisted tooling: designed or discussed, **not built**, and listed with their triggers in
[`design/FUTURE_WORK.md`](design/FUTURE_WORK.md). What the system does *not* claim is in [`LIMITATIONS.md`](LIMITATIONS.md).

## 7. How to reproduce everything

```bash
cmake -S . -B build/host -G Ninja -DTFC_SANITIZE=ON && cmake --build build/host && ctest --test-dir build/host   # the C++ suite and the strict gates
python3 tools/check_standard.py                                                                                  # flight-code rules
python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98                                           # coverage gate
cmake -S . -B build/rel -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/rel --target tfc_replay
(cd sim && TFC_REPLAY_BIN=$PWD/../build/rel/tfc_replay python3 -m campaign.run --strict)                         # the fault campaign
python3 tools/mutation/run_unit.py --jobs 12 --fast --exclude 'test_(environment|dynamics|slosh|flex|montecarlo|imu_datasheet|aero|vehicle_files|vehicle_general|vehicle_refusals|sim_model)'   # mutation testing: about an hour (without --fast, minutes per mutant)
sim/scripts/setup_vcan.sh && tools/bench/sil_triplex.sh --build --launch && tools/bench/sil_triplex.sh --test    # live tests (needs Zephyr: firmware/README.md)
console/tfc-console                                                                                              # the flight console on vcan0 (console/README.md); --replay console/demo/launch-and-node-loss.log.gz needs no bus
```
