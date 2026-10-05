# P-HWO-01: the hardware overrides, the pre-session check

> Status: **procedure**, waiting for the override parts (TS-17 chose H1 to H5: `../hardware/HARDWARE_PARTS.md` section 4; **not in the order placed for 9 Oct**). Build the supervisor with `CONFIG_TFC_OVERRIDE_LINES_FITTED=0x1F` (bit 0 H1, 1 H2, 2 H3, 3 H4, 4 H5) once the sense lines are wired.

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Design: `docs/design/HARDWARE_OVERRIDE.md` (ADR-027). Done before the first unattended run and at the start of every session; an override not operated in a session is reported at the start of the next run (TFC-HWO-007). **Skip a step whose switch is not fitted, and say so.**

| Field | Entry |
|---|---|
| Procedure ID | P-HWO-01 |
| Requirements verified | TFC-HWO-001 to 008 |
| Fault-matrix rows | F75 to F80 |
| Hardware configuration | the rig as built; every switch labelled with its name (H1 to H5) and its sense line wired to the supervisor (GP22, GP26, GP27, GP28, GP0 with a 4.7 kOhm pull-down each) |
| Tools | the multimeter, the supervisor's console (`status` shows each override's state), the logic analyser |

## 1. Description
Show that each override changes the state it should, in the way it should, with no program running; that its sense line reads right in each position; and that the rig refuses to start a run with an override engaged unless told.

## 3. Steps
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | **H1 E-stop:** press it; measure the servo rail | 0 V at the servo; the sense line shows engaged; release and reset by hand | | |
| 2 | Operator | **H3 platform level:** platform at 10 degrees; switch to level; switch back | The platform goes to level at the set rate; back, the Pico regains control | | |
| 3 | Operator | **H2 force-safe:** with ACT running, switch; release | ACT reports Safe, cause *hardware line*; it **stays** Safe until `clear-safe` under an ARM | | |
| 4 | Operator | **H4 injector-disarm:** open it; command the injector to cut a node; close it | The cut has no effect while open; the supervisor's own relays are separate (open H5 and check the same) | | |
| 5 | Operator | **H5 supervisor-disarm:** open it with a node made to hang | The supervisor's reset and power-cycle have no effect while open (the relays stay released); close it and they act | | |
| 6 | Operator | **A node cut by hand** (the free controls: unplug its stub, pull its adapter plug; H6 and H7 are not built): meter the cut node's signal pins | The supervisor reports it dead, tries its resets and gives up; 0 V on the cut node's pins, nothing back-fed through a signal line (HWO-006, G8, G11) | | |
| 7 | Operator | Engage H4 and type `launch` on the supervisor | `LAUNCH REFUSED: override 0x08 is engaged; `override-ok` accepts it`; after `override-ok` the launch goes ahead and `overrides not yet tested this session: 0x..` lists the others (G5, HWO-007) | | |
| 8 | Operator | Read `status` after each step | The line `overrides engaged=<mask> untested=<mask>` follows each switch; an override counts as tested once it has been seen engaged and released; each is logged with a date in `../hardware/BENCH_LOG.md` | | |

## 4. Records
As-run copy; the dates tested per override into `docs/hardware/BENCH_LOG.md`; the fault-matrix rows F75 to F80, F96 and F97.
