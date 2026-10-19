# What to publish (the application evidence)

> Status: **plan and checklist** (reviewed 19 Oct 2026). Recruiters and engineers will spend about a minute; this page says what to put in front of them and keeps the checklist honest. Everything marked done exists in the repository today; everything marked open needs the hardware (`../STATUS.md` section 5) or the time after it (M5, 1 to 15 Dec 2026). The numbers in the evidence table are computed or recorded by `tools/docs/metrics.py`.

## Checklist
**Done**
- [x] GitHub repo with a clear README, an architecture diagram and a CI badge (`../../README.md`, `../design/ARCHITECTURE.md`)
- [x] Requirements-to-test traceability: every requirement with its status and verification (`../verification/REQUIREMENTS.md`), every fault with its test (`../verification/FAULT_MATRIX.md`), every decision with its state (`../decisions/DECISIONS.md`)
- [x] A "known limitations / what is not claimed" section (`../LIMITATIONS.md`)
- [x] The evidence below, reproducible with one command each, and a status page that says what is built and what is not (`../STATUS.md`)
- [x] Procedures for the hardware days, written before the parts arrived (`../procedures/`)
- [x] A FMECA (ECSS-Q-ST-30-02C) with a software FMEA, checked by a tool against the fault matrix and the campaign (`../verification/FMECA.md`), a hazard analysis (`../verification/HAZARDS.md`) and a safety case (`../verification/SAFETY_CASE.md`)
- [x] A second-reader review of the flight core, with its findings fixed and pinned by tests and mutants (`../verification/CODE_REVIEW_CORE.md`); **an outside human reader is still open**

**Open (the rig, and a human reader)**
- [ ] An outside reader for `core/` (the review above is one fresh reader of the same lineage as the author; the owner or a named engineer reading `redundancy.hpp`, `act.hpp` and `supervisor.hpp` is worth more than another tool)
- [ ] Measured timing: frame jitter, WCET of the step and the vote, bus load (`P-S1-01`; the numbers go into `../hardware/BENCH_LOG.md`)
- [ ] The fault matrix with **measured** detection times (M4), not only the simulated ones
- [ ] 60 to 90 s video: the same ascent scenario twice, voting off then on, and a wire pulled mid-run
- [ ] A one-page write-up: the design in a paragraph, the fault matrix, the limitations, and what you would do differently
- [ ] The real bus loss rate and what it did to the resync settings (`../STATUS.md` section 5, item 2)
- [ ] Tag v1.0 (M5), every box above ticked

## Evidence that already exists (software only, reproducible)
| Claim you can make | Where it comes from |
|---|---|
| "<!--m:campaign_scenarios-->12,362<!--/m--> fault scenarios, <!--m:campaign_frames_million-->4.9<!--/m--> million frames, zero safety-property violations" | `cd sim && python3 -m campaign.run --strict` (`../verification/FAULT_CAMPAIGN.md`) |
| "Every one of <!--m:fault_kinds-->32<!--/m--> fault kinds characterised over its input range, with measured detection latency" | the generated tables in `../verification/FAULT_CAMPAIGN.md` |
| "A FMECA of <!--m:fmeca_rows-->102<!--/m--> failure modes that covers every one of the <!--m:fault_matrix_rows-->102<!--/m--> rows of the fault matrix and every fault kind, and finds the single points and what is missing" | `python3 tools/fmeca/fmeca.py check` (`../verification/FMECA.md`) |
| "Found and fixed N defects with the campaign" (a frozen command that got the healthy node isolated; an unrecoverable total loss; a replicated estimator that diverges for good on one lost frame; ...) | `../verification/FAULT_CAMPAIGN.md` section 6, ADR-014, 015, 017, 030 |
| "Found and fixed eight defects by reading the flight core a second time, among them the actuator node's mode byte, which one upset could turn from Safe into Standby" | `../verification/CODE_REVIEW_CORE.md` |
| "Tests proven able to fail: <!--m:mutants_unit-->470<!--/m--> injected bugs in the flight code and <!--m:mutants_sim-->493<!--/m--> in the simulator; last full run <!--m:mutation_unit_result-->not yet recorded<!--/m--> and <!--m:mutation_sim_result-->not yet recorded<!--/m-->" | `python3 tools/mutation/run_unit.py --fast --rerun-survivors`, `run_sim.py`, `cd sim && python3 -m campaign.mutate` |
| "<!--m:cpp_tests-->887<!--/m--> C++ tests under ASan and UBSan; <!--m:coverage_core_line-->0<!--/m--> % line and <!--m:coverage_core_branch-->0<!--/m--> % branch coverage of the flight core and the supervisor; JPL Power-of-Ten rules enforced by tools" | `tools/coverage/core_coverage.py`, `../verification/CODING_STANDARD.md` |
| "No heap, no exceptions, no RTTI in the flight binary, checked on the ARM ELF" | `tools/check_elf.sh` |
| "Real firmware instances on a virtual CAN bus: a triplex that survives the loss of its sync master, a closed loop through max-Q with a computer killed in flight, a launch sequence with a scrub, mixed releases, state resynchronisation, a hardware Safe line" | `tools/bench/sil_triplex.sh --test` (`../../sim/tests/test_live_*.py`) |
| "A decision log of <!--m:adrs-->58<!--/m--> ADRs, each with its state, and the trade studies behind them, with the data" | `../decisions/DECISIONS.md`, `../decisions/TRADE_STUDIES.md` |
| "Frame cost under a microsecond on the host, a 1.4 KB manager" (the *target* WCET stays open until S1) | `build/rel/tfc_bench`, `../verification/CODING_STANDARD.md` section 5 |

## Resume bullet templates (fill with real, measured numbers only)
- Built a triple-redundant STM32 flight computer (C++17, Zephyr, CAN) with 2-of-3 voting and time-triggered scheduling; isolates injected faults in <X> ms with no actuator excursion.
- Developed hardware-in-the-loop test rig with real IMUs on a servo motion platform driven by a 6-DOF ascent simulator; automated <N>-case fault campaign in CI.
- Achieved <X> us p99 control-loop jitter; <Y>% branch coverage on voter/FDIR under ASan/UBSan and static analysis.

## How to tie it to RPL
Pair this with one or two specific things you did on RPL avionics (bus, sensors, flight code, testing). The project shows initiative; the RPL work shows you can do it on a team.
