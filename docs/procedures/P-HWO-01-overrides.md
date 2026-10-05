# P-HWO-01: the hardware overrides, the pre-session check

Follows `docs/VERIFICATION_PROCEDURE_TEMPLATE.md`. Design: `docs/HARDWARE_OVERRIDE.md` (ADR-027). Done before the first unattended run and at the start of every session; an override not operated in a session is reported at the start of the next run (TFC-HWO-007). **Skip a step whose switch is not fitted, and say so.**

| Field | Entry |
|---|---|
| Procedure ID | P-HWO-01 |
| Requirements verified | TFC-HWO-001 to 008 |
| Fault-matrix rows | F75 to F80 |
| Hardware configuration | the rig as built; every switch labelled with its name (H1 to H7) and its sense line wired to the supervisor |
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
| 5 | Operator | **H6 per-node kills:** cut each node by hand | The supervisor reports it dead and tries its resets, then gives up | | |
| 6 | Operator | **H7 master power:** only with the rig unattended-safe | Everything off, nothing back-fed through a signal line (meter on the cut node's signal pins: 0 V, HWO-006) | | |
| 7 | Operator | Engage H4 and `launch` | The launch is refused with the override named; `override-ok` then lets it through (G5) | | |
| 8 | Operator | Read `status` after each step | Each sense line read right in each position; each override logged with a date as tested | | |

## 4. Records
As-run copy; the dates tested per override into `docs/HARDWARE_OVERRIDE.md` section 7; the fault-matrix rows F75 to F80.
