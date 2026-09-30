# What to publish (the application evidence)

Recruiters and engineers will spend about a minute. Make that minute count.

## Checklist
- [ ] GitHub repo with a clear README, an architecture diagram, and CI badge
- [ ] 60-90 s video: same ascent scenario twice, voting off then on, pull a wire mid-run
- [ ] One-page write-up: design, fault matrix with **measured** detection times, limitations
- [ ] Requirements-to-test traceability (fault matrix is the spine)
- [ ] Measured timing: jitter, WCET, bus load
- [ ] A "what I would do differently / known limitations" section

## Resume bullet templates (fill with real, measured numbers only)
- Built a triple-redundant STM32 flight computer (C++17, Zephyr, CAN) with 2-of-3 voting and time-triggered scheduling; isolates injected faults in <X> ms with no actuator excursion.
- Developed hardware-in-the-loop test rig with real IMUs on a servo motion platform driven by a 6-DOF ascent simulator; automated <N>-case fault campaign in CI.
- Achieved <X> us p99 control-loop jitter; <Y>% branch coverage on voter/FDIR under ASan/UBSan and static analysis.

## How to tie it to RPL
Pair this with one or two specific things you did on RPL avionics (bus, sensors, flight code, testing). The project shows initiative; the RPL work shows you can do it on a team.
