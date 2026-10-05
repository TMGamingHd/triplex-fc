# Verification procedure: template

For hardware and hardware-in-the-loop tests that no automated test replaces (the "M" entries of `REQUIREMENTS.md`). The outline follows
the verification-procedure format shown in the ASTE-331 lecture: description, what is verified, set-up, numbered steps that say who does
what and how, shutdown, and an *as-run* copy kept with the data. Copy this file to `docs/procedures/<id>-<name>.md` for each test.

## Header
| Field | Entry |
|---|---|
| Procedure ID | e.g. `P-F57-01` |
| Title | e.g. Partial hang: one task blocked, watchdog not serviced |
| Requirements verified | e.g. TFC-FDIR-038, TFC-SUP-003 |
| Fault-matrix rows | e.g. F57 |
| Firmware under test | git hash of every node, golden or current (ADR-021) |
| Hardware configuration | which nodes are fitted and powered; supervisor present or not |
| Prepared by, date | |

## 1. Description
What is being shown, in two or three sentences, and what a pass looks like in measured terms (a time, a count, an output value).

## 2. Test cases
One line each. For a requirement that covers a range (every allowed rate, every node, both polarities), list every case, so coverage is visible.

| Case | What is varied | Pass criterion |
|---|---|---|
| 1 | | |

## 3. Initial set-up
Wiring checked against the harness; power-up order (the supervisor first); tools running (logic analyser, `tfc_peers listen`, the log
file name); the state each node must be in before step 1 (mode, strikes, phase, role).

## 4. Steps
Each step names **who** (operator, PC script, supervisor), **what** (the action, as the exact command or switch), **how** (the tool and
port), and the **expected result**, with room for the actual one.

| Step | Who | Action (exact) | Expected | Actual (as run) | Pass/fail |
|---|---|---|---|---|---|
| 1 | | | | | |

## 5. Shutdown
The order in which things are turned off, and the state the rig is left in (relays released, E-stop reset, nodes' counters).

## 6. Data and records
| Item | Where it is kept |
|---|---|
| Signed procedure and the as-run copy | `docs/procedures/` or the release |
| Bus log (`candump -L`), decoded copy | |
| Supervisor log (its own time) | |
| Logic-analyser capture | |
| Fault-matrix row updated, with the test id | `docs/verification/FAULT_MATRIX.md` |

Data fields worth recording with each event (as the lecture's telemetry list suggests): the session id, the node, the record type, the
time received on the PC, the node's frame number (the spacecraft-clock analogue) and the supervisor's time, and for each channel both the
raw value and the converted engineering value.
