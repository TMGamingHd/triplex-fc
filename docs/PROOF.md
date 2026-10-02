# What to publish (the application evidence)

Recruiters and engineers will spend about a minute. Make that minute count.

## Checklist
- [ ] GitHub repo with a clear README, an architecture diagram, and CI badge
- [ ] 60-90 s video: same ascent scenario twice, voting off then on, pull a wire mid-run
- [ ] One-page write-up: design, fault matrix with **measured** detection times, limitations
- [ ] Requirements-to-test traceability (fault matrix is the spine)
- [ ] Measured timing: jitter, WCET, bus load
- [ ] A "what I would do differently / known limitations" section

## Evidence that already exists (software-only, reproducible with one command each)
| Claim you can make | Where it comes from |
|---|---|
| "11,263 fault scenarios, 4.7 million frames, zero safety-property violations" | `python3 -m campaign.run --strict` (`docs/FAULT_CAMPAIGN.md`) |
| "Every one of 32 fault kinds characterised over its input range, with measured detection latency" | the generated tables in `docs/FAULT_CAMPAIGN.md` |
| "Found and fixed N defects with the campaign" (a frozen command that got the healthy node isolated; unrecoverable total loss; ...) | `docs/FAULT_CAMPAIGN.md` section 6, ADR-014/015/017 |
| "Tests proven able to fail: 72 injected bugs, all caught" | `tools/mutation/run_unit.py`, `python3 -m campaign.mutate` |
| "100% line / 98.6% branch coverage of the flight core; JPL Power-of-Ten rules enforced by tools" | `tools/coverage/core_coverage.py`, `docs/CODING_STANDARD.md` |
| "No heap, no exceptions, no RTTI in the flight binary, checked on the ARM ELF" | `tools/check_elf.sh` |
| "Frame cost 0.5 us on the host, 1,080-byte manager, 120-byte worst stack" (the *target* WCET stays a TODO until M2) | `tools/bench`, `docs/CODING_STANDARD.md` section 5 |

## Resume bullet templates (fill with real, measured numbers only)
- Built a triple-redundant STM32 flight computer (C++17, Zephyr, CAN) with 2-of-3 voting and time-triggered scheduling; isolates injected faults in <X> ms with no actuator excursion.
- Developed hardware-in-the-loop test rig with real IMUs on a servo motion platform driven by a 6-DOF ascent simulator; automated <N>-case fault campaign in CI.
- Achieved <X> us p99 control-loop jitter; <Y>% branch coverage on voter/FDIR under ASan/UBSan and static analysis.

## How to tie it to RPL
Pair this with one or two specific things you did on RPL avionics (bus, sensors, flight code, testing). The project shows initiative; the RPL work shows you can do it on a team.
