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

## 2. What exists (checked on 4 Oct 2026)

| Area | State |
|---|---|
| `core/` | 7 headers, about 2,200 lines: CRC-8, protocol (frames, SYNC, ground command), voter (`vote3`), `RedundancyManager` (1,445 lines: vote, FDIR, probation, strikes, Safe request, authentication, ARM and interlock, self-protection), fault monitor, integrity. **No estimator, controller, actuator logic, frame scheduler, telemetry, parameters, phases.** |
| Tests | `ctest` passes here: `tfc_tests` (15 s) and `peers_e2e` (70 s). The campaign (about 17 min) and the mutation job run in CI |
| Virtual peers | Python `tfc_peers`: B and C traffic, 32 fault kinds, commands, record, listen, decode, run with SYNC following |
| Replay and campaign | `tfc_replay`, the campaign runner with oracles M1 to M8 and S1 to S4, mutation testing |
| Firmware | `firmware/app/src/main.cpp` (304 lines): **node A only** (`BUILD_ASSERT(kNodeId == 0)`), the sync master, runs the frame loop against the virtual peers on `vcan0`. A simulated IMU. **Builds for `nucleo_g474re`** (checked now: 46.5 KB of 512 KB flash, 8.9%; 6.4 KB of 128 KB RAM, 4.9%), has not run on hardware |
| Not present | A board overlay for the Nucleo (the app takes the board file's FDCAN1 and nothing else), an ISM330DHCX driver, any SPI configuration, a watchdog, GPIO lines, an ACT app, B and C firmware, the Pico firmware, the vehicle simulator, the platform driver, a logger |
| Tools on this PC | `west` and the Zephyr SDK 1.0.1 present; `openocd` present; `picotool` and `dfu-util` not; `vcan0` does not exist now (it does not survive a reboot) |
| CI | Host build with sanitizers, tests, clang-tidy, cppcheck, coding standard, coverage gate, the campaign; Zephyr build for `native_sim` and `nucleo_g474re`, live test against the peers, ELF check |

## 2a. Progress

| Date | Item | State |
|---|---|---|
| 4 Oct 2026 | SW-16 (logic), SW-19 (logic) | `ism330dhcx.hpp` (identity, reset, configuration with read-back, burst read, self-test mechanism with fail-closed limits; register facts from ST's own driver), `progress.hpp` and `resetlog.hpp`, host-tested against a model of the chip with a bus failure at every call (PR 18) |
| 4 Oct 2026 | SW-02, SW-16, SW-17, SW-19 (wiring) | The seams (IMU, lines, watchdog, reset record) in `firmware/app/src/hw.hpp`, the Nucleo overlay with every pin from ST's connector map, the board build verified (51.7 KB flash); native_sim and its 10 live tests unchanged (PR 19) |
| 4 Oct 2026 | SW-18 (partly), SW-22, SW-23 | CAN bus-state counters and receive timestamps enabled; bench tools, a live-bus logger, the golden-run and jitter checks, procedures P-M1-01 and P-S1-01 (PR 20). Not run on hardware |

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
| SW-06 | **Vehicle simulator**: ascent dynamics (planar first, then 6-DOF), engine-out, wind, a platform model with rate and travel limits (the platform is bandwidth-limited), deterministic scenarios | L | S2 | A scenario runs end to end with the controller |
| SW-07 | **Gateway**: the simulator feeds the computers' IMU source (replacing the toy `truth()` of `sim_imu.hpp`) and takes ACT's output back; on the rig the same data go to the table driver | M | S2 | The software-only loop of G1 |

### Actuator node and safety
| ID | Item | Size | Blocks | Done when |
|---|---|---|---|---|
| SW-08 | **ACT logic** in `core/`: command vote, output latch, step bound, loss-of-votes detection, the Safe sequence (freeze, then null; `SAFE_MODE.md`), start in Safe after a reset with the stored last output (RESP-003); oracle S5 for the step bound | M | S2 | Host tests; mutation-tested |
| SW-09 | **ACT app** (Zephyr): CAN, servo PWM (a stub that publishes the output on `native_sim`), watchdog, `SAFE` input, `KICK` | M | S2 | Builds for both targets; runs in G1 |
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
