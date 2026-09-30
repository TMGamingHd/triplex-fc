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
| TFC-FDIR-006 | Reintegration shall require an explicit request and at least 100 consecutive clean frames. | T |
| TFC-FDIR-007 | A channel that latches twice shall become permanently excluded. | T |
| TFC-FDIR-008 | A duplex miscompare shall not be silently averaged; the system shall hold the last voted command and declare Safe request. | T |
| TFC-FDIR-009 | A node transmitting outside its schedule (babbling) shall not delay other schedule slots by more than 1 frame slot; ACT shall ignore out-of-schedule IDs. | M |
| TFC-FDIR-010 | CAN bus-off shall be recovered automatically within 100 ms without operator action. | M |
| TFC-FDIR-011 | An estimator-state digest mismatch shall be flagged within 1 frame. | T |

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
