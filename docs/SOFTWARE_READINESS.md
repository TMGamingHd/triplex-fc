# Software readiness: what is done, what is missing, and the order to do it before the parts arrive

> Status: **plan**, from an audit of the repository on 4 Oct 2026. The facts in section 2 were checked (tests run, firmware built);
> sizes in section 4 are relative (S about a day of focused work, M a few days, L a week or more) and are estimates, not promises.

## 1. Verdict

**The software is not ready, and it will not be complete before the parts arrive.** What exists is the *fault-management core* and the
test machinery around it, which is the hard and unusual part. What is missing is the *rest of a flight computer*: the estimator and
controller, the actuator node, the vehicle simulator, firmware for nodes B and C, the sensor driver, and the tools that run the rig. The
firmware's command today is a toy: it is computed from a synthetic function of the frame number that the Python peers repeat
(`firmware/app/src/sim_imu.hpp`), not from an estimate of anything.

The realistic goal for the parts' arrival is therefore: **everything that blocks stage S1 (one computer on real hardware) and S2
(the actuator node and the platform) is written, built and tested against fakes**, so the first day of hardware is spent on measuring, not
writing. The rest runs alongside the hardware work (section 5).

### Definition of "software-ready" (the gate)
| # | Gate | Evidence |
|---|---|---|
| G1 | A **software-only triplex**: three computers and the actuator node as processes on one virtual CAN bus, the vehicle simulator closing the loop, passing the SIL fault scenarios | one command runs it; CI runs a fast subset |
| G2 | Every driver-facing interface (IMU, CAN, watchdog, discrete lines, non-volatile store, clock) is behind a seam with a fake, and the real driver compiles for the board | the same app builds for `native_sim` and `nucleo_g474re` |
| G3 | The board support is prepared: overlays for the Nucleo (SPI, FDCAN, GPIO lines, node-id pins) checked against ST's user manual on paper, flash script, serial console | `west build` for the FC and ACT variants in CI |
| G4 | The PC side is ready: USB-CAN bring-up, logger with decoding, HIL runner that replays the SIL scenarios on `can0`, platform driver, Pico firmware (servo, relays) with host tests, supervisor logic with host tests | scripts and tests in the repository |
| G5 | The S1 and S2 exit tests are written as procedures (`VERIFICATION_PROCEDURE_TEMPLATE.md`) | files in `docs/procedures/` |
| G6 | CI builds and tests all of it within a cost the owner accepts | CI time under the agreed limit |

## 2. What exists (checked on 4 Oct 2026, after the audit)

| Area | State |
|---|---|
| `core/` | 21 headers, about 4,700 lines, header-only, no heap or exceptions: CRC-8, the protocol (v2: ACT, heartbeat, state share, simulator frames), the voter, `RedundancyManager` (1,445 lines), the fault monitor, integrity; the ISM330DHCX driver logic, progress and reset records; sensor consensus, the attitude estimator, the scheduled controller and guidance, **`FlightFunction`**; the ACT logic, its ground-command path and the heartbeat monitor; `SyncClock`; the Pico's link, platform driver, injector and frame loop |
| Firmware | Four Zephyr apps: **`firmware/app`** (flight computer A, B or C: SYNC following and takeover, the flight function and a simulator-fed IMU as options), **`firmware/act`** (the actuator node), **`firmware/pico`** (platform driver and fault injector), `firmware/common`. All build for their boards (`nucleo_g474re`: about 52 kB and 39 kB of 512 kB; `rpi_pico2`: 59 kB of 4 MB) and pass the ELF check. **None has run on a board.** A Nucleo overlay has every pin from ST's connector map |
| Simulator | `sim/vehicle`: the 6-DOF vehicle (verified against a second implementation), the platform model, the runner, the closed loop with IMU error models; `tools/sim/tfc_simd` on SocketCAN; `tools/sim/tfc_sens` (sensitivity). `docs/SIM_FIDELITY.md` says where it is and is not like real life |
| Tests | 365 C++ tests under ASan and UBSan (coverage 100% of lines, 98.5% of branches over `core/include/tfc`), 224 Python tests, live tests with real firmware instances on `vcan0`, the full fault campaign (11,263 scenarios, no anomaly), mutation testing (112 mutants) |
| Virtual peers | Python `tfc_peers`: B and C traffic, 32 fault kinds, commands, record, listen, decode, run with SYNC following, the Pico client |
| Replay and campaign | `tfc_replay`, the campaign runner with oracles M1 to M8 and S1 to S4, mutation testing |
| Not present | The supervisor's firmware and parts, the hardware overrides (proposed), a launch sequence (the `phase` command), a calibrated IMU model, the heartbeat-driven Safe request's live test, any hardware run |
| Tools on this PC | `west` and the Zephyr SDK 1.0.1; `openocd`; `can-utils`; `vcan0` (does not survive a reboot) |
| CI | Host build with sanitizers, tests, clang-tidy, cppcheck, coding standard, coverage gate, the campaign (a sample on pull requests); Zephyr builds for `native_sim` (A, the three-computer triplex, ACT), `nucleo_g474re` (both apps) and `rpi_pico2`, the live tests, the ELF check |

## 2a. Progress

| Date | Item | State |
|---|---|---|
| 4 Oct 2026 | SW-16 (logic), SW-19 (logic) | `ism330dhcx.hpp` (identity, reset, configuration with read-back, burst read, self-test mechanism with fail-closed limits; register facts from ST's own driver), `progress.hpp` and `resetlog.hpp`, host-tested against a model of the chip with a bus failure at every call (PR 18) |
| 4 Oct 2026 | SW-02, SW-16, SW-17, SW-19 (wiring) | The seams (IMU, lines, watchdog, reset record) in `firmware/app/src/hw.hpp`, the Nucleo overlay with every pin from ST's connector map, the board build verified (51.7 KB flash); native_sim and its 10 live tests unchanged (PR 19) |
| 4 Oct 2026 | SW-18 (partly), SW-22, SW-23 | CAN bus-state counters and receive timestamps enabled; bench tools, a live-bus logger, the golden-run and jitter checks, procedures P-M1-01 and P-S1-01 (PR 20). Not run on hardware |
| 4 Oct 2026 | **P1, first increment** (SW-04, SW-05 on the host) | Sensor consensus, attitude estimator, controller and guidance in `core/`, closed around a host vehicle model with three replicas: nominal 0.17 deg RMS, bit-identical replicas, a wild, biased or frozen IMU masked (`docs/CONTROL_LOOP.md`). **Not yet wired into the firmware**, no vehicle-simulator gateway, no actuator node |
| 4 Oct 2026 | SW-24 (CI) | Done: docs-only changes skip the build, pull requests run a sample of the campaign, `main` runs all of it (PR 22) |
| 4 Oct 2026 | **State of the repository** | PRs 17 to 23 are merged and the remote branches deleted; CI on `main` passes; `can-utils` installed on the bench PC. The parts order is not placed yet (it still needs the supervisor's second Pico 2, clock module and resistors) |
| 4 Oct 2026 | **SW-08 (ACT logic)** | `core/include/tfc/act.hpp`: the vote, node exclusion, the slew-bounded output, Safe (freeze, hold, ramp, neutral, no automatic exit), the reset behaviour; 19 tests including a 20,000-frame fuzz (`docs/ACT_LOGIC.md`). Not yet the app or the bus I/O |
| 4 Oct 2026 | **SW-01 (protocol v2)** | ACT out, heartbeat, state share and the simulator's frames in C++ and Python with the same golden bytes pinned in both; the manager knows the simulator's range; `docs/PROTOCOL.md`. Not yet: the `noop` and `phase` opcodes, and the firmware producing the heartbeat |
| 4 Oct 2026 | **SW-04 (flight function)** | `core/include/tfc/flight.hpp`: sensor consensus, estimator and scheduled controller as one object that the firmware, the tests and the closed-loop flights share (bit-identical to the stages composed by hand, replicas identical, a masked node's sensors unused); `to_act_frame()` for ACT; the gain schedule and pitch program generated from the vehicle design into `firmware/app/src/flight_tables.hpp` (`tools/vehicle/gen_tables.cpp`), with a test that fails if the committed file is stale. Not yet wired into `main.cpp` |
| 4 Oct 2026 | **SW-03 (nodes B and C, sync takeover)** | `tfc::SyncClock` and the firmware loop for all three nodes (`CONFIG_TFC_NODE_ID`): SYNC following, counting through a gap, takeover after two missed frames with staggered windows, silence until synced (ADR-025); `CONFIG_TFC_FLIGHT_FUNCTION` and `CONFIG_TFC_SIM_BUS_IMU` for the closed loop; `tools/bench/sil_triplex.sh`; a live test with three firmware instances in which A is killed. Not yet: a sync-master takeover on real hardware (HIL F15), the ACT app, the simulator's runner |
| 4 Oct 2026 | **SW-08 (ACT app)** | `firmware/act`: the actuator node as a Zephyr app (native_sim, Nucleo; 39 kB flash), `ActGround` (authenticated, ARMed clear-safe and reintegrate), `HeartbeatMonitor` (majority Safe request), flight-computer heartbeats; live test with four instances (Nominal, a computer dies and is excluded, ACT flies on one, Safe on lost votes). Not yet: the supervisor's SAFE line |
| 4 Oct 2026 | **SW-06 (the simulator on the bus, closed loop)** | `sim/vehicle/runner.hpp` and `tools/sim/tfc_simd.cpp`: the vehicle follows SYNC, publishes the sensor inputs one frame ahead, and flies with ACT's output; `tests/test_runner.cpp` closes the whole chain on the host (RMS 0.079 degree over the ascent, gust, engine-out, sensor fault, lost computer, lost ACT frame); `sim/tests/test_live_closed_loop.py` closes it live on vcan0 with six processes (16 s within 1 degree, digests equal; a computer killed in flight is excluded and the vehicle flies on). Found: TS-16 (replicated estimators on a lossy bus). Not yet: the Pico platform driver (P1-5) |
| 4 Oct 2026 | **Parts sheet v4 checked; P1-5 decisions** | `docs/PARTS_CHECK.md`: what matches, ten conflicts and gaps (the four spare relay channels claimed twice, the supervisor's parts still missing, an inconsistent CAN adapter model, servo stall current, the budget 77.81 USD over). ADR-026: the injector's four channels cut the power of A, B, C and ACT; the Pico firmware is Zephyr (a build check of blinky, USB CDC-ACM and PWM passed for `rpi_pico2/rp2350a/m33`; not run on a board). `docs/HARDWARE_OVERRIDE.md`, ADR-027, TS-17, TFC-HWO-001 to 008 and F75 to F80: the layer of manual hardware overrides (proposed) |
| 4 Oct 2026 | **SW-20 (Pico: platform driver and injector)** | `core/include/tfc/{pico_link,platform_driver,injector}.hpp` with host tests (golden frames pinned in C++ and Python; travel, rate, hold and level timeouts; relays that release by themselves); `firmware/pico` (Zephyr, builds to a UF2 of 117 kB with 59 kB of flash, no heap or vtables); `python3 -m tfc_peers pico`; `docs/PICO.md`. Not run on a board; the list of what to check is in PICO.md section 7 |
| 4 Oct 2026 | **Parts decisions; Pico test options** | The second Pico 2, the TCXO module and a third relay module are added; the resistors are owned; the CAN adapter is reported to support FD. `docs/HARDWARE_PARTS.md` is the full bill of parts. `docs/PICO_TESTS.md` lists every option for testing the Pico (host, desk, relay modules, nodes, servos, whole chain) with a recommended order; the next step needing no hardware is the real main loop under test with a fake board |
| 4 Oct 2026 | **Pico tests, first group (A3 to A5)** | The Pico's loop is now `tfc::PicoApp<Hal>` in `core/` and `main.cpp` is the glue; 15 loop tests against a fake board, fuzz and property tests (2 million bytes, 300,000 steps, eight properties against an independent relay model), 40 mutants (36 killed at first, three real gaps closed, one equivalent). Fixed a stale mutant of the manager. Trade studies TS-18 (testing without hardware), TS-19 (cut semantics), TS-20 (what the platform does when commands stop) |
| 4 Oct 2026 | **Audit of everything to date** | A clean build and all tests under sanitizers (365 C++, 224 Python), the full fault campaign (11,263 scenarios, no anomaly), all 112 mutants killed, clang-tidy, cppcheck, coding standard, coverage 100% lines, every live test on freshly built images, an 85 s live closed loop through max-Q (worst error 0.36 degree, replicas' digests equal). Added: vehicle dispersions, an IMU error model, `tools/sim/tfc_sens`, edge-case tests (past burnout, total loss, runner and `tfc_simd` with a corrupt, backward or far-forward frame number: **`tfc_simd` could hang on a corrupt SYNC; fixed**), `EstimatorConfig::use_accel`. Findings: no pad phase; the estimator mistakes thrust for gravity at a thrust-to-weight near 1 (`docs/SIM_FIDELITY.md`). Refreshed the stale status sections |
| 4 Oct 2026 | **Launch sequence, first increment (host)** | `docs/LAUNCH_SEQUENCE.md` (design, go/no-go, phases), ADR-028 and TS-21 (proposed: the supervisor decides T-zero, the flight computers carry the clock in SYNC; the owner confirms). Built: `ImuCalibrator` (per-IMU stationary gyro calibration, before the consensus), `FlightFunction::set_mission` and `sensors_ok`, the runner's pad clamp and release, the closed loop with a pad, `tfc_sens --pad`; 15 mutants killed. Measured: lift-off transient 5.3 to 1.2 degrees (1 degree of thrust misalignment), vehicle gyro-bias tolerance 0.19 to at least 8 dps. Found on the way: a consensus calibration cannot work because IMUs differ by more than the consensus tolerance. Not yet: mission time in SYNC, the `launch` and `scrub` commands, the firmware, `tfc_simd`, the checklist |
| 4 Oct 2026 | **Launch sequence, second increment (host) and the clock of record** | Mission frame in SYNC (C++ and Python, goldens pinned), carried by `SyncClock` (follow before T-zero, count and verify in flight, late joiner adopts, continuous through a takeover); `launch` (needs an ARM) and `scrub` ground operations; `launch_check` (go/no-go); the heartbeat ready bit; 19 more mutants, all killed. `docs/MISSION_CLOCK.md`, ADR-029, TS-22 and TFC-SUP-011 to 014: the supervisor keeps the independent clock of record (battery-backed, correlated against UTC), the owner's point about a five-year mission. Not yet: the firmware, `tfc_simd`, the live test, `tfc_peers launch`, the checklist |

## 3. What is missing, by area

IDs are for tracking. "Blocks" says which stage cannot start without it. "Done when" is the check.

### Foundations (everything depends on these)
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-01 | **Protocol v2.** Heartbeat payload (node, role and state summary, protocol version, release hash, reset count), ACT output frame and vote status (`0x300`), opcodes `noop` and `phase`, sensor frames keyed by IMU channel (ADR-020). C++ and Python together, with tests. Freeze it early: both sides and the golden release depend on it | M | S2, S3 | Both sides pass a shared test vector set |
| SW-02 | **Seams.** `ImuSource`, `CanBus`, `Watchdog`, `Lines` (kick, frame, reset, adopt), `NvStore`, `Clock` as small interfaces in the app, with fakes for `native_sim`; `core/` untouched | S to M | S1 | The app builds with fakes and with the real drivers |
| SW-03 | **Nodes B and C and the sync master.** Node id from the build option, SYNC following, takeover by the lowest healthy node, frame number continuous through SYNC loss (ADR-018's firmware requirements) | M | S3 | Three instances run together; killing A leaves B as master with no skipped frame |

### Control and the vehicle
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-04 | **Estimator** in `core/`: attitude from gyro and accelerometer, deterministic, no heap, bit-identical on host and target, with the quantised digest | L | S2 | Unit tests; a golden-run comparison; the digests of three replicas match |
| SW-05 | **Controller**: thrust-vector command from the attitude error, with saturation and rate limits | M | S2 | Unit tests; closed loop in SW-06 |
| SW-06 | **Vehicle simulator** (owner's choice 4 Oct 2026: a full 6-DOF ascent, a C++ library plus a SocketCAN runner; `docs/VEHICLE_SIM.md`): variable mass properties, thrust with altitude, TVC, atmosphere, wind, engine-out, validation tests, a trajectory table and gain schedule | L | S2 | The validation tests of section 9 pass; the nominal ascent and the closed loop run on the host |
| SW-07 | **Gateway**: the simulator feeds the computers' IMU source (replacing the toy `truth()` of `sim_imu.hpp`) and takes ACT's output back; on the rig the same data go to the table driver | M | S2 | The software-only loop of G1 |

### Actuator node and safety
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-08 | **ACT logic** in `core/`: command vote, output latch, step bound, loss-of-votes detection, the Safe sequence (freeze, then null; `SAFE_MODE.md`), start in Safe after a reset with the stored last output (RESP-003); oracle S5 for the step bound | M | S2 | Host tests; mutation-tested |
| SW-09 | **ACT app** (Zephyr): CAN, the voted command on the bus (there is no PWM: the servos are the platform's), watchdog, `SAFE` input, `KICK` | M | S2 | Builds for both targets; runs in G1 |
| SW-10 | **Software triplex harness**: A, B, C and ACT as `native_sim` processes on one `vcan`, a software fault injector in place of the Pico, scripted scenarios, a CI subset | M | S3 | `F01` to `F18` run in software |
| SW-11 | **Mixed releases**: a build from a tag as the golden node, the `common_mode` campaign group (F63), the compatibility-gate script (ARCH-006) | M | S4 | The measured weakness (F63) reproduces, then the hold-and-ask rule passes |

### Fault-management extensions
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-12 | **Sensor and compute health split** (ARCH-001; ADR-020 case 1): manager, campaign oracles, mutants; its own PR | L | none (improves S3 and S4) | F65 passes; coverage gate and mutants still at their levels |
| SW-13 | **`noop`, events and housekeeping telemetry, the command dictionary, reset counter and cause, state broadcast and restore** (FDIR-041 to 043, IF-004 to 006) | L | none | A test checks C++ and Python against the dictionary |
| SW-14 | **Parameters**: the Safe configuration and the phase table as validated tables with defaults | S | S2 (Safe) | Invalid values replaced and reported |
| SW-15 | **Phases and roles** (PHASE-001 to 004) | L | none | Optional; after S4 |

### Hardware-facing code (written and built now, verified on the rig)
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-16 | **ISM330DHCX driver**: SPI register set-up, ranges that match the protocol's quantisation, WHO_AM_I, self-test (FDIR-036), a timer-triggered read at 0.5 ms, calibration constants per channel; tested against an emulated register map on `native_sim` | M | S1 | The emulated test passes; the real read is checked on arrival |
| SW-17 | **Board overlays** for the Nucleo: SPI instance and pins (not the LED pin, compatibility audit 5), FDCAN pins (audit 4), GPIO lines, node-id pins, console; for the FC and ACT variants | M | S1 | Both variants build in CI; the pin table is in `firmware/README.md` |
| SW-18 | **FDCAN set-up**: bit timing, receive timestamps, filters, bus-off recovery (FDIR-010), transmit queue, error counters into the manager | M | S1 | Builds; checked on the rig |
| SW-19 | **Watchdog and lines**: the hardware watchdog serviced only from the end of a completed frame (FDIR-038), `FRAME` and `KICK` pulses, reset cause, memory that survives a reset | M | S1 | Host test with a task that never reports |
| SW-20 | **Pico firmware (fault injector)**: USB command protocol, servo PWM with a rate limiter and hard limits, relay outputs active-low with safe defaults; the PC-side client; host tests with mocks | L | S2 | Builds in CI; host tests pass |
| SW-21 | **Supervisor firmware**: the decision table of `SUPERVISOR.md` as portable logic with host tests, the Pico build, the hardware commands | M | S2b | Host tests; builds |

### PC tools and procedures
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-22 | **Bench tools**: USB-CAN bring-up (`gs_usb`, 1 Mbit/s) and its udev rule, a logger (SocketCAN to `candump -L` plus decoded), the flash script with the `openocd` runner, a HIL runner that replays the SIL scenarios on `can0`, the bench-check scripts of M1 | M | S1 | Run against `vcan0` with the software triplex |
| SW-23 | **Procedures** for the S1 and S2 exit tests and the M1 checks | S | S1, S2 | Files in `docs/procedures/` |
| SW-24 | **CI cost**: cancel stale runs, skip the campaign for docs-only changes, a fast subset on pull requests (about 17 min per push today) | S | none | The owner's limit met |

## 4. Order, and what fits before the parts arrive

The parts order closes 6 Oct; delivery time is not known here. If they arrive during M1 (to 20 Oct), about two weeks are left.

| Priority | Items | Why this order |
|---|---|---|
| **P0 (S1 on day one)** | SW-02, 16, 17, 18, 19, 22, 23, and the answers to the owner's questions that touch the protocol (SW-01 design) | Without these the first hardware day is spent writing |
| **P1 (S2: the loop)** | SW-01, 04, 05, 06, 07, 08, 09, 14, 20 | A platform that tracks a simulated ascent needs an estimator, a controller, a vehicle, an actuator node and a Pico |
| **P2 (S3 and S4)** | SW-03, 10, 11, 21 | Needed when the second and third computers arrive |
| **P3 (alongside)** | SW-12, 13, 15, 24 | Strengthen the fault management; not on the critical path to hardware |

**Honest expectation:** P0 is realistic before the parts arrive; P1 is a stretch; P2 and P3 run in parallel with the hardware, in simulation first.
Do not start P3 before P0 and the protocol freeze of SW-01 are done.

## 4a. P1 plan (after the owner's answers, 4 Oct 2026)

Independent pull requests wherever possible, each from `main`, so none waits on another:

| PR | Content | Depends on |
|---|---|---|
| P1-0 | This design: the ACT and servo role corrected, `VEHICLE_SIM.md`, ADR-024, requirements | none |
| P1-1 | Protocol v2: ACT output `0x300`, simulator frames `0x501` to `0x504`, heartbeat, `noop`; C++ and Python together | none |
| P1-2 | The 6-DOF vehicle library with its validation tests, the nominal trajectory, the gain schedule, and the closed loop with the real estimator and controller | none (the loop pieces are merged) |
| P1-3 | ACT logic in `core/`: the vote, the latch, the step bound, the Safe sequence, loss of votes (SW-08) | none |
| P1-4 | The simulator runner on the bus, the firmware wiring (consensus, estimator, controller in the frame, the sim-fed IMU), the ACT app, a software triplex on one `vcan0` | P1-1, P1-2, P1-3 |
| P1-5 | The Pico platform driver and its PC client, with the platform's own safety (SW-20) | P1-1 |

## 5. Things to do on this PC now (no code)

- `sim/scripts/setup_vcan.sh` after each reboot (or make it persistent).
- The ST-LINK udev rules, so `west flash` and `openocd` work without root; check with a USB device list when a board is plugged in.
- `can-utils` and the `gs_usb` module for the USB-CAN adapter; the script of SW-22 will do the `ip link` set-up; untested until the adapter arrives (`sim/README.md`).
- `pyserial` for the Pico client; the Pico SDK (or Zephyr's `rpi_pico2` board) for SW-20 and SW-21; a UF2 copy needs no tool.
- Logic-analyser software for the Kingst LA1010 (the parts sheet says Linux is supported; check the vendor program or sigrok before the device arrives).
- Confirm `west flash` is set to the `openocd` runner for the Nucleo.

## 6. What cannot be done before the hardware

Timing numbers (jitter, execution time, bus load), what the D85MG servo does with no signal, the relay at 3.3 V drive, the IMU board's logic level, the adapter's voltage, and every real bus effect. For each, the script or procedure should be ready (SW-22, SW-23) so that the measurement takes minutes.

## 7. Decisions (owner, 4 Oct 2026)

1. **Supervisor:** SUP-Lite (SW-21 and the lines of SW-17 to 19 are Lite).
2. **Release rule (ADR-021):** hold, request Safe, operator picks. Automatic takeover by the old release is decided after the TS-3 data.
3. **Safe mode:** the answers of `SAFE_MODE.md` section 10 and `FAULT_RESPONSE.md` section 6 (SW-08).
4. **Sensor and compute split (SW-12):** after the loop (P1) and before S3 (3 Nov); TS-15 chooses the degradation rule first.
5. **CI cost (SW-24):** trim (see `.github/workflows/ci.yml`).
