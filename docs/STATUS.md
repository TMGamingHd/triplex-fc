# Status

> Status: **reference**, the page to read first. Reviewed 19 Oct 2026. Every number that changes is written between markers and comes from `tools/docs/metrics.py`: **computed** from the repository (it cannot be stale) or **recorded** with the command, the date and the commit of the run that made it (section 2 says which). CI fails when one is out of date.

## 1. In five lines

* **What it is:** a three-computer fault-tolerant flight computer (voter, fault manager, estimator and controller, actuator node, supervisor), a simulator that flies any vehicle from a file, virtual peers that inject faults, and the tools that check them.
* **What is proven:** on the host and on the virtual rig, a fault in one computer, one IMU or one computer's link to the bus does not move the voted output, and the next fault degrades the system as designed. The evidence is in section 2 and the register of dated results ([`verification/RESULTS.md`](verification/RESULTS.md)).
* **What is not:** **nothing has run on a board.** The parts arrived on 8 Oct 2026; the bench log's last entry is 6 Oct 2026. Timing, the real bus, the real IMUs and detection time on hardware are all still to be measured (section 5), and every hardware claim is marked as a target until a log exists.
* **What the analyses say is missing** (section 6): the cycle time of the guidance on the target, the false-alarm rate on real sensors, the real bus, and an outside reader. The single points are ACT, the bus, the supervisor, the power rail and a bug common to every replica; the claim excludes them.
* **How far to trust it:** one person's design, code, tests and oracles. Mutation testing proves the tests catch the bugs that were injected, not all bugs. [`LIMITATIONS.md`](LIMITATIONS.md) is the honest list.

## 2. The numbers

| What | Now | Where it comes from |
|---|---|---|
| C++ tests (ASan and UBSan) | <!--m:cpp_tests-->887<!--/m--> | computed; `ctest --test-dir build` |
| Python tests (peers, replay, tools, console, docs) | <!--m:python_tests-->639<!--/m--> | computed; `cd sim && python3 -m unittest discover -s tests -t .` (some are live and skip without `vcan0`: [`RESULTS.md`](verification/RESULTS.md)) |
| Requirements | <!--m:requirements-->186<!--/m-->, of which <!--m:requirements_test-->152<!--/m--> are checked by a test and <!--m:requirements_hw-->45<!--/m--> by a hardware measurement | computed; [`REQUIREMENTS.md`](verification/REQUIREMENTS.md). <!--m:requirements_rig_pending-->41<!--/m--> say *rig pending* or *waits for* |
| Decisions | <!--m:adrs-->58<!--/m--> ADRs, the latest <!--m:adr_latest-->ADR-058<!--/m--> | computed; [`DECISIONS.md`](decisions/DECISIONS.md) |
| Fault matrix | <!--m:fault_matrix_rows-->102<!--/m--> rows, <!--m:fault_kinds-->32<!--/m--> fault kinds injected by the peers | computed; [`FAULT_MATRIX.md`](verification/FAULT_MATRIX.md) |
| Fault campaign | <!--m:campaign_scenarios-->12,362<!--/m--> scenarios, <!--m:campaign_frames-->4,900,623<!--/m--> frames, no property violated, no anomaly | computed from the generated page; `cd sim && python3 -m campaign.run --strict` |
| FMECA | <!--m:fmeca_rows-->102<!--/m--> failure modes | computed; [`FMECA.md`](verification/FMECA.md) |
| Structural coverage of `core/` and `supervisor/` | <!--m:coverage_core_line-->0<!--/m--> % of lines, <!--m:coverage_core_branch-->0<!--/m--> % of branches (gates 100 and 98) | **recorded** by `python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98`; CI enforces the gate |
| Structural coverage of `sim/vehicle` | <!--m:coverage_sim_line-->0<!--/m--> % of lines, <!--m:coverage_sim_branch-->0<!--/m--> % of branches (gates 99 and 90) | **recorded**, same command with `--sim-min-line 99 --sim-min-branch 90` |
| Mutants | <!--m:mutants_unit-->470<!--/m--> in the core and supervisor, <!--m:mutants_sim-->493<!--/m--> in the simulator; the fault campaign can reach <!--m:mutants_campaign-->130<!--/m--> of the first | computed; `tools/mutation/` |
| Mutation result | core: <!--m:mutation_unit_result-->not yet recorded<!--/m-->; simulator: <!--m:mutation_sim_result-->not yet recorded<!--/m--> | **recorded** from the last full run; `python3 tools/mutation/run_unit.py --fast --rerun-survivors` and `run_sim.py` |
| Headers that no mutant targets | <!--m:headers_without_mutants-->9<!--/m--> of <!--m:core_headers-->38<!--/m--> in core and supervisor (the guidance, navigation and attitude control, the sensor driver); <!--m:sim_headers_without_mutants-->12<!--/m--> in `sim/vehicle` | computed; `python3 tools/mutation/check_tables.py --report` |
| Flight code | <!--m:core_lines-->10,344<!--/m--> lines in <!--m:core_headers-->38<!--/m--> header-only files, no heap, no exceptions, no RTTI | computed; `python3 tools/check_standard.py`, `tools/check_elf.sh` |

Static analysis (clang-tidy, cppcheck), the coding standard and the 2 KB stack bound are CI gates; their dated results are in the register.

## 3. What is built

| Area | What exists | State |
|---|---|---|
| **Flight core** (`core/`, `supervisor/`) | voter; fault manager with 3-of-5 and leaky-count persistence, probation, strikes and a sticky Safe; sensor/compute split; state resynchronisation; launch gate; estimator and controller; actuator-node logic; supervisor logic | built, tested, mutation-tested (except the guidance); host only |
| **Firmware** (`firmware/`) | four Zephyr applications: flight computer, actuator node, Pico (platform driver and fault injector), supervisor. Builds for `native_sim`, `nucleo_g474re`, `rpi_pico2`; the ELF check finds no heap, exception, RTTI or vtable | builds in CI; live tests on `vcan0` with real instances; **not run on a board** |
| **Simulator and peers** (`sim/`, `tools/sim`) | 6-DOF flight of any vehicle read from a file; planets, rotation, atmospheres, real balloon winds; the IMU as its datasheet; virtual peers with <!--m:fault_kinds-->32<!--/m--> fault kinds; record and replay through the real core; the campaign and its oracles | built; validated against exact solutions and itself, not against a real vehicle ([`VALIDATION.md`](verification/VALIDATION.md)) |
| **Guidance, navigation, a whole mission** | PEG to orbit, a stage's return with a tower catch, recovery under parachutes, engine-out handling ([`GNC.md`](design/GNC.md), [`PROPULSION.md`](design/PROPULSION.md)) | host only: no timing on the board, no firmware path; **no mutants** |
| **Flow solver and weather** | an Euler solver for the body; the flight flies on its tables; five real launch-site wind days ([`CFD.md`](design/CFD.md), [`ENVIRONMENT.md`](design/ENVIRONMENT.md)) | built; the flaps and fins are not converged and not used |
| **Flight console and 3D viewer** | one local web page for the sequence, the votes, the faults, the bus and every ground command; a viewer with seven lenses ([`CONSOLE.md`](design/CONSOLE.md), [`VIEWER.md`](design/VIEWER.md)) | built; tested against `vcan0` and pseudo-terminals; **not run against a board** |
| **Analyses** | a hazard analysis (STPA), an FMEA gap analysis, **a FMECA** with a software FMEA, a safety case, a risk register, a validation page, a **review of the core by a second reader** ([`FMECA.md`](verification/FMECA.md), [`CODE_REVIEW_CORE.md`](verification/CODE_REVIEW_CORE.md)) | written; checked against the fault matrix and the requirements by tools |
| **Bench tools and procedures** | USB-CAN bring-up, logger, jitter and bus-loss measurement, golden-run check, ten procedures | tested against `vcan0` and recorded logs; **not run on the adapter or a board** |

## 4. The decisions

All <!--m:adrs-->58<!--/m--> ADRs are accepted and none is open. [`decisions/DECISIONS.md`](decisions/DECISIONS.md) opens with a register: for each one, whether it is built, what it still waits for, and where. Every ADR is built on the host and in the firmware; the ones that need the hardware to be *confirmed* are in section 5; two are stretch goals outside v1 (the ring that re-homes an orphaned IMU; a second ACT). The Safe hold time and the persistence constants are *parameters* that the rig tunes (TS-4, TS-1).

## 5. What waits for the hardware

Each row is a thing that cannot be settled without the parts. "Where" is the procedure or tool that settles it; the order of the first days is in [`procedures/FIRST_HARDWARE_DAYS.md`](procedures/FIRST_HARDWARE_DAYS.md).

| # | What | Why it waits | Where |
|---|---|---|---|
| 1 | Frame jitter (p99 at most 100 us), WCET of the step and the vote, bus load (SYS-001 to 003) | needs the target and a logic analyzer; the firmware's status line already prints `wcet_step`, `wcet_vote`, `wcet_frame` | `P-S1-01`, `tools/bench/frame_jitter.py` |
| 2 | **The real bus loss rate** | it sets the resync period, the digest persistence and the manager's command tolerance (TS-23, `RESYNC.md` 6b); nothing simulates real CAN | `tools/bench/bus_loss.py` |
| 3 | **The false-alarm rate on the real IMUs**, and their noise (Allan deviation) | the detection thresholds were tuned on a modelled IMU; a healthy computer's false isolation can only be bounded by a long fault-free run on the rig (REC-10, FMECA) | a soak procedure (to be written), `P-S1-01` |
| 4 | Sensor self-test limits (FDIR-036), arrival-margin telemetry (FDIR-037) | the part and the board | `FUTURE_WORK.md` 1.1, 1.2 |
| 5 | The supervisor on its board: `KICK`, `FRAME`, `NRST` and `PWR` against real nodes; the `T0` pin; the RTC and the TCXO's drift | the second Pico 2, the TCXO module and the third relay module | `P-S2-03`, `MISSION_CLOCK.md` |
| 6 | The Pico: USB enumeration, servo pulse and jitter, relays at 3.3 V drive, what the servo does with no signal | the board and the servos | `P-M1-01`, `PICO_TESTS.md` |
| 7 | The platform: the shock at the E-stop, stall current, the platform's bandwidth against the time-scaled vehicle | mechanics | `PICO_TESTS.md` E1 to E6, `SIM_FIDELITY.md` |
| 8 | **The hardware overrides** | TS-17 chose the set (H1 to H5, about 12 USD); **the parts are not in the order** | `HARDWARE_OVERRIDE.md`, `P-HWO-01` |
| 9 | CAN effects: bus-off recovery (FDIR-010), a babbler's real harm (FDIR-009), inconsistent omission, the sync-master takeover on hardware | real transceivers | `P-S3-01`, `P-S4-01` |
| 10 | The closed loop on the real platform; TS-4 and TS-1 re-tuned on measured data | the rig | `P-S2-01`, `TRADE_STUDIES.md` |
| 11 | **The guidance and navigation in the image**: the worst-case cycle on the G474 (double precision in software on a single-precision FPU) | the board | REC-03, `GNC.md` |
| 12 | The golden release (ADR-021) | a release that has passed the hardware stage exits | `P-REL-01` |
| 13 | **The flight console's serial paths**: the Nucleos' consoles, the supervisor's commands and the Pico's relays and status, on the real ports | the boards; tested against pseudo-terminals and a fake Pico only | `CONSOLE.md` section 10 |

## 6. What the analyses say is missing

From the FMECA ([`FMECA.md`](verification/FMECA.md)), the review of the core ([`CODE_REVIEW_CORE.md`](verification/CODE_REVIEW_CORE.md)) and the review against the field's practice (`FUTURE_WORK.md` 6a and 6b). In the order that would raise a reader's confidence most per week:

1. **The board.** Every row of section 5; the largest single gain.
2. **Critical items with nothing that exercises their compensation:** the guidance (host only, and the whole-mission Monte Carlo catches the stage in none of 100 flights under the assumed dispersions), the cycle time on the target, ACT silent or hung, inconsistent omission on the bus, the single power rail.
3. **An outside reader** for `core/`: the review here is one fresh reader of the same lineage as the author; the parts it did not read (the guidance) and the requirements themselves are for a human.
4. **Mutation cover of the guidance and the mission code**, which carry no mutants.
5. **The software FMEA's gaps:** ACT's output frame as the go/no-go reads it (the freshness rule is in the firmware, not the core, and untested), a SYNC whose counter stops, and a heartbeat that lies (the campaign's peers send none).
6. **The false-alarm rate** and the detectors' own parameters (an upset in one is not seen: REC-09).

## 7. Schedule

All hardware is ordered; bring-up is in stages S1 to S4 ([`hardware/STAGED_BUILD.md`](hardware/STAGED_BUILD.md)), and only nodes that have passed their stage are powered and plugged into the bus. Each stage ends with a tagged commit and a short log or video. Slack is intentionally in M4 and M5: if M2 slips, cut stretch goals, not the fault campaign; if M3 slips, ship a tested Duplex demo and say so.

| # | Dates | Goal | Done when |
|---|---|---|---|
| M0 | by 6 Oct | the order is placed; CI on GitHub; Zephyr on `native_sim` | **done** |
| M1 | 9 to 20 Oct | the parts arrive: inspect each, set JP5, measure the adapter (under 5.25 V), bench-check a relay at 3.3 V drive and a servo at 3.3 V signal; first board runs | the bench checks are in `hardware/BENCH_LOG.md`; a Nucleo blinks and prints |
| M2 | 20 Oct to 3 Nov | **S1** one flight computer on hardware (10 min at 100 Hz, timing measured); **S2** ACT, the Pico and the platform; **S2b** the supervisor | zero frame errors; WCET and jitter recorded; the platform tracks commands |
| M3 | 3 to 17 Nov | **S3** FC-B (Duplex), **S4** FC-C (Triplex); takeover, consensus, the 2-of-3 vote | unplug any one node: the output is unchanged; voting on against off is recorded |
| M4 | 17 Nov to 1 Dec | the hardware repeat of the fault campaign, detection-time histograms | the fault matrix is filled with measured data |
| M5 | 1 to 15 Dec | write-up, video, README, tag v1.0 | every box in `project/PROOF.md` is ticked |

## 8. Not in v1

A second ACT, the ring re-homing of an orphaned IMU, a second bus, dual-slot boot, the telemetry dictionary's records, analytical redundancy for Duplex: designed or discussed, **not built**, with their triggers in [`design/FUTURE_WORK.md`](design/FUTURE_WORK.md). What the system does not claim is in [`LIMITATIONS.md`](LIMITATIONS.md).

## 9. How to reproduce everything

```bash
cmake -S . -B build/host -G Ninja -DTFC_SANITIZE=ON && cmake --build build/host && ctest --test-dir build/host   # the C++ suite and the strict gates
python3 tools/check_standard.py                                                                                  # flight-code rules
python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98 --sim-min-line 99 --sim-min-branch 90    # coverage gates
cmake -S . -B build/rel -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/rel --target tfc_replay
(cd sim && TFC_REPLAY_BIN=$PWD/../build/rel/tfc_replay python3 -m campaign.run --strict)                         # the fault campaign
python3 tools/mutation/check_tables.py                                                                           # no mutant has gone stale
python3 tools/mutation/run_unit.py --fast --rerun-survivors --jobs 12                                            # mutation testing of the core, about two hours
python3 tools/mutation/run_sim.py --jobs 12                                                                      # and of the simulator
python3 tools/docs/metrics.py check && python3 tools/fmeca/fmeca.py check                                        # the numbers in the documents, and the FMECA
sim/scripts/setup_vcan.sh && tools/bench/sil_triplex.sh --build --launch && tools/bench/sil_triplex.sh --test    # live tests (needs Zephyr: firmware/README.md)
console/tfc-console                                                                                              # the flight console on vcan0; --replay console/demo/launch-and-node-loss.log.gz needs no bus
```
