# Requirements (draft v0.1)

All numeric limits are **proposals**. Each will be confirmed, tightened or dropped after the first measurements. Verification: **T** = automated test, **M** = measurement on hardware, **I** = inspection/analysis.

## System
| ID | Requirement | Verif. |
|---|---|---|
| TFC-SYS-001 | The control loop shall run at 100 Hz with p99 frame-start jitter no greater than 100 us (target). | M |
| TFC-SYS-002 | Worst-case execution time of the estimator+controller step shall be no more than 50% of the frame's compute window. | M |
| TFC-SYS-003 | Worst-case CAN bus load shall not exceed 40% (target 20%) including simulator traffic. | M |
| TFC-SYS-004 | Loss of any single FC shall not move the voted actuator command by more than 0.5 deg in any frame. | T, M |
| TFC-SYS-005 | The vehicle simulation shall remain within its attitude-error envelope through any single injected fault. | T, M |

## Fault detection, isolation, recovery
| ID | Requirement | Verif. |
|---|---|---|
| TFC-FDIR-001 | A fail-silent FC shall be isolated within 3 frames (30 ms) of its last good frame. | T, M |
| TFC-FDIR-002 | A bias-step IMU fault greater than the vote tolerance shall be isolated within 3 frames. | T, M |
| TFC-FDIR-003 | A stuck-at IMU fault shall be isolated within 40 frames (400 ms) (bound to be tightened with the stuck detector). | T, M |
| TFC-FDIR-004 | A single-frame disagreement shall not isolate a healthy channel. | T |
| TFC-FDIR-005 | A latched channel shall be excluded from the vote starting the same frame it latches. | T |
| TFC-FDIR-006 | A latched node shall be readmitted to the vote only after (a) its minimum dwell, (b) an operator request or, under the opt-in automatic policy, a first transient-looking latch, and (c) a probation of N consecutive frames (100; 300 after a repeat latch) in which its data agree with the voted output of the healthy nodes and with their digest (ADR-010). | T |
| TFC-FDIR-007 | A node that latches `max_strikes` times in a run (3; 2 when the cause is physical, e.g. a stuck sensor) shall be Disabled for the run and shall refuse reintegration until a maintenance command clears it (ADR-010). | T |
| TFC-FDIR-008 | A duplex miscompare that cannot be attributed shall not be silently averaged; the system shall hold the last voted command and declare a Safe request that persists until cleared by an operator (ADR-008). | T |
| TFC-FDIR-009 | A node transmitting outside its schedule (babbling) shall not delay other schedule slots by more than 1 frame slot; ACT shall ignore out-of-schedule IDs. (Detection: FDIR-019. This requirement is the bus-timing effect, measured on hardware.) | M |
| TFC-FDIR-010 | CAN bus-off shall be recovered automatically within 100 ms without operator action. | M |
| TFC-FDIR-011 | An estimator-state digest mismatch shall be flagged within 1 frame. | T |
| TFC-FDIR-016 | A single corrupted or lost frame shall cost exactly one bad sample; two isolated corrupted or lost frames inside the persistence window shall not isolate a healthy node (ADR-007). | T |
| TFC-FDIR-017 | In duplex, a disagreement shall be blamed on a node only if that node jumped away from a fresh last-agreed value while the other stayed within tolerance of it; a stale reference shall never blame a node; a transient in duplex shall not isolate both survivors (ADR-008). | T |
| TFC-FDIR-018 | Whenever the vote cannot produce a trustworthy value (unresolved duplex disagreement, no majority, no data) the output shall hold the last good value and be flagged as held. | T |
| TFC-FDIR-019 | Out-of-schedule frames at or above the configured rate (default 3 per 10 ms frame) shall raise a bus alarm in the same frame, without blaming a node (ADR-009). | T |
| TFC-FDIR-020 | A node whose data still disagree with the healthy nodes' voted output shall never be readmitted; failing probation shall not count a new strike and shall name the reason. | T |
| TFC-FDIR-021 | At most one node shall be on probation at a time; a frame with no trustworthy reference shall neither advance nor fail a probation. | T |
| TFC-FDIR-022 | Operator actions (reintegrate, disable, clear-disabled, clear-safe) shall be accepted as CRC-checked ground-command frames and every command shall be answered with an accepted or refused outcome in the flight computer's report (ADR-011). | T |
| TFC-FDIR-023 | A node whose frames are bad at a sustained rate of about one in three (or two in five) shall be isolated within 40 frames of the fault starting, with the reason "intermittent fault"; a single glitch, a two-frame burst, or one bad frame in ten or fewer shall not isolate a healthy node (ADR-013). | T |

## Software quality
| ID | Requirement | Verif. |
|---|---|---|
| TFC-SW-001 | `core/` shall contain no dynamic allocation, exceptions, or RTTI. | I, T |
| TFC-SW-002 | `core/` shall build with warnings as errors and be clean under clang-tidy and cppcheck. | T |
| TFC-SW-003 | `core/` shall pass the full test suite under ASan and UBSan. | T |
| TFC-SW-004 | Branch coverage of the voter and FDIR code shall be 100%. | T |
| TFC-SW-005 | Every requirement shall trace to at least one test ID in the fault matrix or a measurement procedure. | I |
| TFC-SW-006 | Every merge to main shall run the SIL regression in CI. | T |

## Interfaces
| ID | Requirement | Verif. |
|---|---|---|
| TFC-IF-001 | All bus payloads shall be 8 bytes: 6 data, 1 sequence, 1 CRC-8. | T |
| TFC-IF-002 | Any single bit flip in a payload shall be detected. | T |
| TFC-IF-003 | CAN IDs shall order SYNC, sensors, commands, actuator output, heartbeat by priority. | T |
