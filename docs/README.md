# Documentation

> Start with **[STATUS](STATUS.md)** (what is built, what is proven, what waits for the hardware) and **[LIMITATIONS](LIMITATIONS.md)** (what is not claimed). Everything else is reference, grouped by what you want to do.
> Every page opens with a `> Status:` line that says whether it describes something built, a decision, a plan or a reference; a test (`sim/tests/test_docs_consistency.py`) keeps the links, this index and the section numbers honest.

## If you have five minutes
| Read | For |
|---|---|
| [STATUS.md](STATUS.md) | Where the project stands today, in numbers, and the schedule |
| [LIMITATIONS.md](LIMITATIONS.md) | What the system does not claim and why |
| [design/ARCHITECTURE.md](design/ARCHITECTURE.md) | What is built: nodes, the 100 Hz frame, voting, FDIR |
| [project/PROOF.md](project/PROOF.md) | What to show: the checklist and the evidence that already exists |

## Design: how it works (`design/`)
| Page | Covers |
|---|---|
| [ARCHITECTURE.md](design/ARCHITECTURE.md) | The system, the nodes, the frame schedule, agreement and voting, FDIR and the node life cycle |
| [PROTOCOL.md](design/PROTOCOL.md) | The flight-bus protocol v2: every id, payload and golden example |
| [CONTROL_LOOP.md](design/CONTROL_LOOP.md) | Consensus, estimator, controller and the model they are tested against |
| [ACT_LOGIC.md](design/ACT_LOGIC.md) | The actuator node: vote, latch, step bound, Safe |
| [SAFE_MODE.md](design/SAFE_MODE.md) | Safe: freeze, hold, null; who may end it |
| [FAULT_RESPONSE.md](design/FAULT_RESPONSE.md) | What the computers do and what the vehicle does, per fault and phase |
| [MISSION_PHASES.md](design/MISSION_PHASES.md) | Hot, warm and cold; the phase table; the `phase`, `warm` and `noop` commands |
| [LAUNCH_SEQUENCE.md](design/LAUNCH_SEQUENCE.md) | The pad, the countdown, T-zero, the `T0` line |
| [MISSION_CLOCK.md](design/MISSION_CLOCK.md) | The clock of record: the supervisor's independent time |
| [RESYNC.md](design/RESYNC.md) | State resynchronisation on a lossy bus, and the finding that limits it |
| [SUPERVISOR.md](design/SUPERVISOR.md) | The supervisor (SUP-Lite): lines, ladder, commands |
| [HARDWARE_OVERRIDE.md](design/HARDWARE_OVERRIDE.md) | The manual switches below the supervisor; their own failure modes |
| [PICO.md](design/PICO.md) | The Pico: platform driver and fault injector |
| [VIEWER.md](design/VIEWER.md) | The 3D viewer: the vehicle seen as a vehicle (a Starship-class stack or any vehicle file), the pose the simulator sends, the six lenses and what each number is (measured, derived or illustrative) |
| [CONSOLE.md](design/CONSOLE.md) | The flight console: one web page that shows the bus, the nodes' consoles and the simulator's truth, and sends the operator's commands |
| [VEHICLE_SIM.md](design/VEHICLE_SIM.md) | The 6-DOF ascent simulator, the platform and the bus runner |
| [ENVIRONMENT.md](design/ENVIRONMENT.md) | The world: planets, rotation, J2, atmospheres and their dispersions, wind profiles, turbulence |
| [IMU_MODEL.md](design/IMU_MODEL.md) | The IMU as the ISM330DHCX datasheet describes it: noise, wandering bias, drift with temperature, cross-axis, jitter; the presets, the checks |
| [MONTE_CARLO.md](design/MONTE_CARLO.md) | Many flights, each with a random draw of everything that can differ: the tool, the statistics, what it found |
| [DYNAMICS.md](design/DYNAMICS.md) | The servo (second order, backlash), jet damping, the push and tip-off of a separation, the roll controller: the formulas and their checks |
| [AERODYNAMICS.md](design/AERODYNAMICS.md) | The aerodynamics that follow a vehicle's shape: the formulas, their limits, how they are checked |
| [VEHICLE_SPEC.md](design/VEHICLE_SPEC.md) | Describing any vehicle (stages, tanks, engines, effectors) as a file; the tools; the examples; what the flight computers can do with it |
| [SIM_FIDELITY.md](design/SIM_FIDELITY.md) | How close the simulator is to real life, measured |
| [FUTURE_WORK.md](design/FUTURE_WORK.md) | Designs written down and not built, with their triggers |

## Decisions: why (`decisions/`)
| Page | Covers |
|---|---|
| [DECISIONS.md](decisions/DECISIONS.md) | The 34 ADRs, with a register of what is built and what each waits for |
| [TRADE_STUDIES.md](decisions/TRADE_STUDIES.md) | The studies behind them, with results where they are done |

## Verification: how we know (`verification/`)
| Page | Covers |
|---|---|
| [REQUIREMENTS.md](verification/REQUIREMENTS.md) | Every requirement, how it is verified and where it stands |
| [FAULT_MATRIX.md](verification/FAULT_MATRIX.md) | Every fault, how it is injected, the expected detection and response, and the test |
| [FMEA.md](verification/FMEA.md) | Failure-mode gap analysis: what is modelled, covered, or not |
| [FAULT_CAMPAIGN.md](verification/FAULT_CAMPAIGN.md) | The systematic fault campaign: the numbers (generated), the edge cases found and what was done |
| [CODING_STANDARD.md](verification/CODING_STANDARD.md) | The flight-code rules and the tool that enforces each |
| [PICO_TESTS.md](verification/PICO_TESTS.md) | Every way to test the Pico, and the recommended order |
| [VERIFICATION_PROCEDURE_TEMPLATE.md](verification/VERIFICATION_PROCEDURE_TEMPLATE.md) | The template every hardware procedure copies |

## Hardware: what to buy and build (`hardware/`)
| Page | Covers |
|---|---|
| [HARDWARE_PARTS.md](hardware/HARDWARE_PARTS.md) | The bill of parts, the audit of the parts sheet, what is ordered and what is not |
| [STAGED_BUILD.md](hardware/STAGED_BUILD.md) | One flight computer first, then two, then three: the rules and the exit tests |
| [BENCH_LOG.md](hardware/BENCH_LOG.md) | The as-run log: every bench check and measurement, dated |

## Procedures: what to do on the bench (`procedures/`)
Start with [FIRST_HARDWARE_DAYS.md](procedures/FIRST_HARDWARE_DAYS.md); the index of all of them is [procedures/README.md](procedures/README.md).

## Project (`project/`)
| Page | Covers |
|---|---|
| [PROOF.md](project/PROOF.md) | What to publish: the checklist, the evidence, the resume bullets |

Elsewhere in the repository: [`../firmware/README.md`](../firmware/README.md) (building, flashing, every Kconfig option and test knob), [`../sim/README.md`](../sim/README.md) (the virtual peers, the replay tool, the campaign),
[`../tools/bench/README.md`](../tools/bench/README.md) (the bench tools), [`../CONTRIBUTING.md`](../CONTRIBUTING.md).
