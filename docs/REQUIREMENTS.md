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
| TFC-SYS-006 | *(Proposed, ADR-023.)* After any single fault in a flight computer, an IMU, or a computer's link to the bus, the voted output shall continue within SYS-004 with no interruption longer than one frame. For a fault in the shared bus, the actuator node, the servo rail or the supervisor the system shall be fail-passive (SAFE-001), and the write-up shall state that fail-operational is not claimed there. | T, M |

## Fault detection, isolation, recovery
| ID | Requirement | Verif. |
|---|---|---|
| TFC-FDIR-001 | A fail-silent FC shall be isolated within 3 frames (30 ms) of its last good frame. | T, M |
| TFC-FDIR-002 | A bias-step IMU fault greater than the vote tolerance shall be isolated within 3 frames. | T, M |
| TFC-FDIR-003 | A stuck-at IMU fault shall be isolated within 40 frames (400 ms) (bound to be tightened with the stuck detector). | T, M |
| TFC-FDIR-004 | A single-frame disagreement shall not isolate a healthy channel. | T |
| TFC-FDIR-005 | A latched channel shall be excluded from the vote starting the same frame it latches. | T |
| TFC-FDIR-006 | A latched node shall be readmitted to the vote only after (a) its minimum dwell (50 frames after a first, transient-looking latch; 200 for a digest mismatch, a stuck or intermittent node and every repeat latch), (b) an operator request or, under the opt-in automatic policy, a first transient-looking latch, and (c) a probation of N consecutive frames (100; 300 after a repeat latch) in which its data agree with the voted output of the healthy nodes and with their digest (ADR-010). | T |
| TFC-FDIR-007 | A node that latches `max_strikes` times in a run (3; 2 when the cause is physical, e.g. a stuck sensor) shall be Disabled for the run and shall refuse reintegration until a maintenance command clears it (ADR-010). | T |
| TFC-FDIR-008 | A duplex miscompare that cannot be attributed shall not be silently averaged; the system shall hold the last voted command and declare a Safe request that persists until cleared by an operator (ADR-008). | T |
| TFC-FDIR-009 | A node transmitting outside its schedule (babbling) shall not delay other schedule slots by more than 1 frame slot; ACT shall ignore out-of-schedule IDs. (Detection: FDIR-019. This requirement is the bus-timing effect, measured on hardware.) | M |
| TFC-FDIR-010 | CAN bus-off shall be recovered automatically within 100 ms without operator action. | M |
| TFC-FDIR-011 | An estimator-state digest mismatch shall be flagged within 1 frame. | T |
| TFC-FDIR-016 | A single corrupted or lost frame shall cost exactly one bad sample; two isolated corrupted or lost frames inside the persistence window shall not isolate a healthy node (ADR-007). | T |
| TFC-FDIR-017 | In duplex, a disagreement shall be blamed on a node only if that node is farther than the arbitration factor times the tolerance from where the signal should be (the last agreed value, carried forward by the last agreed step where the signal moves more than a tolerance per frame, ADR-017) while the other stayed within tolerance of it; a stale reference shall never blame a node; a transient in duplex shall not isolate both survivors (ADR-008). | T |
| TFC-FDIR-018 | Whenever the vote cannot produce a trustworthy value (unresolved duplex disagreement, no majority, no data) the output shall hold the last good value and be flagged as held. | T |
| TFC-FDIR-019 | Out-of-schedule frames at or above the configured rate (default 3 per 10 ms frame) shall raise a bus alarm in the same frame, without blaming a node (ADR-009). | T |
| TFC-FDIR-020 | A node whose data still disagree with the healthy nodes' voted output shall never be readmitted; failing probation shall not count a new strike and shall name the reason. | T |
| TFC-FDIR-021 | At most one node shall be on probation at a time, except when no node is healthy (FDIR-024); a frame with no trustworthy reference shall neither advance nor fail a probation. | T |
| TFC-FDIR-022 | Operator actions (reintegrate, disable, clear-disabled, clear-safe) shall be accepted as ground-command frames that pass the CRC, the authentication tag and the replay check (FDIR-031), and every command that is accepted for processing shall be answered with an accepted or refused outcome in the flight computer's report (ADR-011, ADR-019). | T |
| TFC-FDIR-023 | A node whose frames are bad at a sustained rate of about one in three (or two in five) shall be isolated within 40 frames of the fault starting, with the reason "intermittent fault"; a single glitch, a two-frame burst, or one bad frame in ten or fewer shall not isolate a healthy node (ADR-013). | T |
| TFC-FDIR-024 | When no node is healthy, two or more nodes on probation shall be judged against the vote among themselves: the odd one out fails, nodes that cannot be told apart neither advance nor fail, and agreeing nodes are readmitted after the normal probation (ADR-014). A single candidate shall wait. | T |
| TFC-FDIR-025 | The manager shall detect a single-bit upset in its node states, its Safe flag or its configuration within one frame, repair it on the safe side (node excluded, Safe requested, configuration restored), and count and report it (ADR-015). | T |
| TFC-FDIR-026 | An invalid configuration value (a non-finite or non-positive tolerance, M-of-N outside 1 <= M <= N <= 32, a stuck limit below 2, out-of-range leaky-count constants, an arbitration factor between 0 and 1, contradictory probation or strike limits) shall be replaced by its default and reported, never used (ADR-015). | T |
| TFC-FDIR-027 | Every CAN id that is not part of the flight-bus schedule (including ids above the simulator id) shall be counted as out-of-schedule traffic. | T |
| TFC-FDIR-028 | In Duplex, a node whose output stopped following a moving signal (a frozen or stale command) shall never cause the healthy node to be isolated, and shall not put a wrong value on the output: it is isolated, or the output is held and Safe requested (ADR-017). | T |
| TFC-FDIR-029 | When no majority of estimator-state digests exists (three different values), no node shall be isolated; the disagreement shall be treated as unresolved (hold, Safe request). | T |
| TFC-FDIR-030 | Every sensor and command frame shall carry the number of the SYNC frame of its cycle; a receiver shall treat as a sequence error a frame whose number is not the current one, nor an earlier one whose own cycle delivered nothing, nor a number not yet seen; a node whose frames are out of phase (early, late, repeated, frozen or wrongly numbered) shall be isolated within 3 frames, and a single lost, damaged or late frame shall still cost exactly one sample (ADR-018). | T |
| TFC-FDIR-031 | A ground command shall act only if its 32-bit SipHash-2-4 tag verifies under the shared key and its counter is 1 to the configured window ahead of the last accepted one; a frame that fails either check shall be dropped, counted (`commands_unauthentic`, `commands_replayed`) and leave no trace on the bus (ADR-019). | T |
| TFC-FDIR-032 | `clear-disabled` and `clear-safe` shall act only under a matching ARM frame received within the arm window; one ARM shall cover one EXECUTE of one operation and node; an ARM that is not followed shall expire and be counted (ADR-019). | T |
| TFC-FDIR-033 | `disable` of a voting node shall be a plain command only while it leaves at least two healthy nodes; leaving one voter shall need an ARM; removing the last voter shall need an ARM and be reported as critical in the frame report, on the console and in `critical_commands` (ADR-019). | T |
| TFC-FDIR-034 | The manager's command state (counter record, ARM) shall be guarded and scrubbed like its other critical state (FDIR-025); a damaged ARM shall be cleared and a damaged counter record shall forget the history, never accept an unauthentic frame (ADR-015, ADR-019). | T |
| TFC-FDIR-035 | The arbitration reference shall follow the signal's motion only to the extent that the last agreed step exceeds one tolerance per frame (full from two), so that channels whose per-frame change is below the tolerance are arbitrated exactly as against the last agreed value (ADR-017 amended). | T |
| TFC-FDIR-036 | *(Deferred: the self-test mechanism exists and is host-tested (`ism330dhcx.hpp`); the datasheet's limits are not set and the test is not called until they are, and it needs the part.)* A node shall run the sensor's built-in self-test at power-up and shall not join the vote if it fails (docs/DEFERRED.md). | M |
| TFC-FDIR-037 | *(Deferred, until the board exists.)* The flight computer shall record, per node and stream, the arrival margin to the vote deadline, and shall raise a non-latching "timing degraded" flag when it stays under a threshold; a slot-window check against the arrival time shall be added telemetry-first (docs/DEFERRED.md; E3, E11). | M |
| TFC-FDIR-038 | *(The progress monitor exists and is host-tested (`progress.hpp`); wiring into the frame loop and the board's watchdog is next.)* A node's hardware watchdog shall be serviced only once per frame, at the end of a completed frame: after the vote has run and every monitored task has reported progress in that frame. A task that stops reporting shall stop the servicing, whatever the other tasks keep doing, and the node shall reset or fall silent within the watchdog timeout (proposal: 3 frames, to match FDIR-001) (docs/DEFERRED.md section 5.1). | T, M |
| TFC-FDIR-039 | *(Deferred, until the supervisor exists; ADR-022.)* The power or reset of each flight-computer node shall be controllable by lines that do not depend on the firmware of the node being controlled, driven by the supervisor (SUP-003), so that a node whose firmware is hung or misbehaving can be reset, power-cycled or held in reset (docs/SUPERVISOR.md). | M |
| TFC-FDIR-040 | *(Deferred, until the rig, with FDIR-009.)* The response to a bus alarm that persists for more than N frames (proposal: 10) shall be defined and tested. As a minimum it shall be reported; the operator shall be able to hold one node at a time in reset through the supervisor (`hold X`) to find the source by elimination. It shall never hold a node in reset automatically on a bus alarm alone, because an out-of-schedule id cannot be attributed to a node (ADR-009). | M |
| TFC-FDIR-041 | *(Deferred, until nodes B and C have firmware and the heartbeat payload is defined.)* Each node shall broadcast its view of the membership and command state (mode, per-node state, strike counts, last accepted command counter) at least every N frames (proposal: 10). A node that restarts shall rebuild its strike counts and command counter from what the others broadcast, using a value that at least two nodes agree on and, where they disagree, the more conservative one (the higher strike count and counter; a Disabled node stays Disabled); a restored state shall never make a node more trusted than the others hold it (docs/DEFERRED.md section 5.3). | T, M |
| TFC-FDIR-042 | *(The reset log exists and is host-tested (`resetlog.hpp`); wiring into the firmware and acting on a loop is next.)* Each node shall keep a reset counter and the cause of its last reset (power, watchdog, software) across resets, shall come up after a reset not voting and in a minimal state, and shall rejoin only through the normal join and probation path (FDIR-006). A node that resets more than R times in a window (proposal: 3 in 60 s) shall stay out of the vote, report the reboot loop, and rejoin only after a maintenance command (docs/DEFERRED.md section 5.5). | T, M |
| TFC-FDIR-043 | *(Planned, not implemented.)* A `noop` ground command shall pass the same CRC, tag and counter checks as the others, change no state, need no ARM, and be answered with an accepted outcome and its counter, so the operator can check the command path from end to end (docs/DEFERRED.md section 5.4). | T |

## Architecture: sensing and software diversity (ADR-020 accepted for case 1, ADR-021 proposed)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-ARCH-001 | *(Accepted, ADR-020 case 1; to be built after the loop and before stage S3 (3 Nov), with the degradation rule chosen by TS-15.)* Sensor-channel health shall be tracked separately from compute-channel health: a flight computer whose IMU is isolated shall remain a voter for commands for as long as its commands agree with the others, and only a disagreement of its commands shall remove it from the command vote. The numbers of healthy sensor channels and of healthy compute channels shall be reported separately. | T |
| TFC-ARCH-002 | Every flight computer shall obtain the IMU samples of the other computers only through the bus frames of the sensor exchange, so that all healthy computers form their consensus from identical inputs; no computer shall read another computer's IMU at the same time as its owner does. The one-owner-at-a-time exception is ARCH-003. | I |
| TFC-ARCH-003 | *(Deferred stretch, after S4; ADR-020 case 2.)* Each IMU shall be connectable to its home computer or to one backup host (the ring: IMU k to computer k-1) by a switch that connects it to exactly one at a time and defaults to home; the switch shall be driven only by the supervisor's `ADOPT` line, which also tells the backup host to read the orphaned IMU. The adopted channel shall start on probation and shall not count in the consensus until it has agreed with it for the probation length. | M |
| TFC-ARCH-009 | *(Planned, with the node firmware; ADR-020.)* A sensor frame shall identify the IMU channel it carries, not only the computer that sends it, and at any time there shall be exactly one publisher per channel. | I, T |
| TFC-ARCH-010 | *(Planned, with the harness; ADR-020.)* Each IMU shall be powered from a rail that does not depend on any one flight computer being powered. | I, M |
| TFC-ARCH-011 | *(Planned, with the estimator; ADR-020.)* The calibration constants of each IMU channel shall be known to every flight computer, so that any computer can use any channel. | I |
| TFC-ARCH-004 | *(Deferred, until FC-B and FC-C have firmware.)* Each node's heartbeat shall carry a protocol version and the source hash of its release, and the system shall run correctly with node C on the golden release and nodes A and B on the current one. | I, T |
| TFC-ARCH-005 | *(Deferred, with ARCH-004.)* The golden release shall be a tagged release that passed the full fault campaign and the hardware stage exit tests, built reproducibly with a recorded toolchain and hash, and changed afterwards only for a safety fix. | I |
| TFC-ARCH-006 | *(Deferred, with ARCH-004.)* A behavioural-compatibility gate in CI shall replay a defined scenario set through the current and the golden release and require their commands to agree within the version tolerance (proposal: 1.5 times the vote tolerance); a release that fails it shall not be fielded against that golden release. | T |
| TFC-ARCH-007 | *(Accepted 4 Oct 2026, ADR-021.)* When two nodes running one release agree with each other and disagree with the node running the other release beyond the version tolerance for M of N frames, no node shall be isolated, the output shall be held and a Safe request raised; the operator shall be able to resolve it by disabling one side under an ARM and clearing Safe (ADR-021, ADR-008). | T |
| TFC-ARCH-008 | *(Deferred, with ARCH-004.)* The estimator-state digest shall be computed over a version-stable, quantised state, so that nodes of different releases behaving correctly produce the same digest. | T |

## Supervisor (accepted 4 Oct 2026 as SUP-Lite, ADR-022; docs/SUPERVISOR.md)
All deferred until the supervisor hardware exists. Items marked Full need the variant with a listen-only CAN tap, which was **not** chosen: they are not built unless Full is taken later.

| ID | Requirement | Verif. |
|---|---|---|
| TFC-SUP-001 | The supervisor shall have its own processor of a different family from the flight computers, its own clock source (proposal: 5 ppm or better), its own firmware sharing no code with `core/`, and power taken upstream of every relay that switches a node. | I |
| TFC-SUP-002 | The supervisor shall act only through discrete lines (`NRST`, `PWR`, `SAFE`, `SEL`) and its USB link, and shall never transmit on the flight bus. | I, T |
| TFC-SUP-003 | Each flight computer and ACT shall pulse its `KICK` line only from the end-of-frame path of a completed frame (FDIR-038). The supervisor shall reset a node whose kicks are missing for 3 frames, power-cycle it after 3 resets in 60 s, and hold it in reset and report it dead after 2 power-cycles in 5 min. | T, M |
| TFC-SUP-004 | The supervisor shall measure the period of each node's `FRAME` line against its own clock and the phase between nodes, and shall report a period error above 200 ppm (10 s average) or a phase difference above 200 us. It shall not isolate a working node on these alone. | T, M |
| TFC-SUP-005 | *(Full only; not built with SUP-Lite.)* The supervisor shall record, per node and stream, the arrival margin to the vote deadline with its own clock (the telemetry of FDIR-037, taken outside the flight computers). | M |
| TFC-SUP-006 | The supervisor shall execute the hardware commands `reset`, `cycle`, `hold`, `release`, `safe-now` (and `sel` if a second ACT exists) received over USB without the participation of any flight computer, and shall answer and log each with its own time. | T, M |
| TFC-SUP-007 | A supervisor that is unpowered, in reset or hung shall leave every node running and every selection at its default: no relay energised by a floating line, no `SAFE` asserted. | M |
| TFC-SUP-008 | The supervisor shall have its own hardware watchdog and shall report its own reset count and the cause of its last reset. | M |
| TFC-SUP-009 | The supervisor shall keep mission elapsed time on its own clock. *(Full.)* It shall flag a node whose SYNC frame number, multiplied by the frame period, departs from that time by more than a set bound (the plausibility check on a time seed). | T, M |
| TFC-SUP-010 | The supervisor shall never decide whether a working node's data are good: its isolating actions are limited to a node that is silent, hung, looping through resets, or named by the operator. | I, T |

## Safe mode (proposed, ADR-023; docs/SAFE_MODE.md)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-SAFE-001 | On entering Safe the output shall freeze at the last good voted command at once, with no step; after `T_hold` (proposal: 50 frames) it shall move to the neutral command at no more than the rate limit (proposal: a quarter of full travel per second) and then hold neutral until Safe is cleared. | T, M |
| TFC-SAFE-002 | ACT shall carry out the sequence of SAFE-001 by itself, without the flight computers or the bus, when it has had no valid vote for 3 frames, when the vote status carries a Safe request, or when its `SAFE` line is asserted. | T, M |
| TFC-SAFE-003 | Safe shall be left only when at least two healthy flight computers have agreed for 100 frames, ACT sees valid votes, and an operator `clear-safe` under an ARM has been accepted (ADR-019). Nothing shall leave Safe by itself. | T |
| TFC-SAFE-004 | Every entry to and exit from Safe shall be recorded as an event with its cause and counted (IF-005, IF-006). | T |
| TFC-SAFE-005 | The neutral command, `T_hold`, the rate limit and the lost-vote count shall be parameters with defaults in non-volatile memory, validated like FDIR-026: an invalid value is replaced by its default and reported. | T |
| TFC-SAFE-006 | The Safe action shall be defined per mission phase (docs/MISSION_PHASES.md). | I |
| TFC-SAFE-007 | Responses shall follow the ladder: ignore, flag, isolate, degrade, hold and request Safe, Safe; the lowest adequate rung shall be used, and no monitor shall be allowed to trip on a single sample or be disabled by a parameter (docs/SAFE_MODE.md section 8). | I, T |
| TFC-SAFE-008 | *(Accepted 4 Oct 2026.)* A Safe event shall be reported to the vehicle simulator as a flag; the simulator shall keep the run going with the frozen or nulled output and mark it safed, and only an operator abort shall end it. | T |

## Mission phases (proposed, ADR-023; docs/MISSION_PHASES.md)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-PHASE-001 | *(Deferred, until the estimator and the simulator's scenario events exist.)* Each node shall have a role (HOT, WARM, COLD) independent of its health. HOT: powered, voting, output used. WARM: powered, running, judged by the shadow vote, output not used. COLD: not running. | T |
| TFC-PHASE-002 | *(Deferred, with PHASE-001.)* A phase table shall give, for each phase, the nominal and minimum number of HOT healthy nodes; a phase change that cannot meet the minimum shall be refused and the current phase held. | T |
| TFC-PHASE-003 | *(Deferred, with PHASE-001.)* Promotion of a WARM node to HOT shall use the probation criteria (FDIR-006, FDIR-020, FDIR-021); demotion shall follow the interlock tiers of FDIR-033 and shall be refused below the phase minimum. | T |
| TFC-PHASE-004 | *(Deferred, with PHASE-001.)* The phase shall be changed only by an authenticated `phase` command (ADR-019); a flight computer shall never change the phase itself. | T |

## Fault response (proposed; docs/FAULT_RESPONSE.md)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-RESP-001 | Every row of the fault matrix shall name a response class (R0 to R6 of `FAULT_RESPONSE.md`) and the behaviour of the vehicle's output, and the response shall be demonstrated by the test named in the row. | I |
| TFC-RESP-002 | The phase rules of `FAULT_RESPONSE.md` section 5 shall decide what loss of redundancy means in each phase: the pad holds the launch below Triplex, ascent continues in Duplex with an alert, and an unattributable disagreement is frozen in every phase. | T |
| TFC-RESP-003 | *(Deferred, with the actuator node.)* After any reset ACT shall start in Safe, not in nominal mode, and shall resume from its last output, kept in memory that survives a reset and protected by a check; if that memory is invalid it shall start from neutral. The output shall not step by more than the rate limit across a reset. | T, M |
| TFC-RESP-004 | *(Deferred, with the supervisor.)* The supervisor shall not reset or power-cycle a flight computer on its own while that computer is the only one whose output agrees with the vote; only the operator's `hold` may. | T |

## AI boundary (proposed; docs/DEFERRED.md section 7)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-AI-001 | No learned or statistical-inference component shall run in the flight binaries or take part in a vote; the flight binaries shall contain no inference runtime. | I, T |
| TFC-AI-002 | An AI-assisted tool shall not hold the ground-command key and shall not send a command; any suggestion shall be logged with its inputs, and a human shall issue the command. | I |
| TFC-AI-003 | The campaign output shall be byte-identical with every AI-assisted component disabled and enabled. | T |

## Control loop (docs/CONTROL_LOOP.md)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-LOOP-001 | Given the same frames, every replica shall compute a bit-identical command and digest (sensor consensus, estimator and controller are deterministic: only arithmetic, comparisons and `sqrt`; no fused multiply-add, no fast-math). | T |
| TFC-LOOP-002 | With three healthy sensors the loop shall hold a simulated vehicle on its pitch program to within 0.5 degree RMS and 1 degree at the worst, through a gust and an engine-out, and shall settle back within one degree after either. | T |
| TFC-LOOP-003 | One sensor that is wild, biased or frozen shall not change the loop's tracking by more than 0.05 degree RMS (the consensus masks it); with two sensors left the loop shall continue within one degree. | T |
| TFC-LOOP-004 | A consensus that is not a trustworthy value (two sensors that disagree, three with no majority, none) shall not be used: the estimator holds its rates and reports itself not valid, and the controller holds its last command and does not move its integrator. | T |
| TFC-LOOP-005 | A non-finite gyro value shall be treated as untrustworthy and a non-finite or out-of-gate accelerometer value shall not correct the attitude; the estimator's state shall stay finite. | T |
| TFC-LOOP-006 | The estimator shall align from the first trustworthy gravity reading, and the controller's gimbal command shall never exceed its angle limit or change by more than its slew limit in a frame. | T |

## Vehicle simulator and platform (ADR-024; docs/VEHICLE_SIM.md)
| ID | Requirement | Verif. |
|---|---|---|
| TFC-SIM-001 | The simulator shall be deterministic: the same scenario and seed shall give a byte-identical state history. | T |
| TFC-SIM-002 | The vehicle model shall be a 6-DOF rigid body with variable mass properties, thrust that depends on altitude, thrust-vector control, an atmosphere and a wind, aerodynamic instability and engine-out, and shall pass the conservation and known-value tests of `VEHICLE_SIM.md` section 9 (circular orbit, the rocket equation, standard atmosphere, torque-free rotation). | T |
| TFC-SIM-003 | Scenarios (nominal, wind shear and gust at max-Q, engine-out at a chosen time, a mass or centre-of-gravity offset) shall be files with a seed and timed events, and every one shall be logged. | T |
| TFC-SIM-004 | The simulator shall advance on ACT's output frame, or, if it does not arrive within 8 ms of SYNC, on the last command held and counted, and shall publish the next frame's sensor inputs, so that its timing does not depend on the host's jitter. | T |
| TFC-SIM-005 | The simulator shall provide a platform mode (the platform's rates and gravity in the platform frame) and a vehicle-true mode (the vehicle's rates and specific force). | T |
| TFC-SIM-006 | A Safe event shall not end the run; it shall be reported to the simulator, which marks the run safed and continues; only an operator abort ends it (SAFE-008). | T |
| TFC-PLAT-001 | The platform driver (the Pico) shall limit the platform's rate and travel in its own firmware, whatever the PC commands. | T, M |
| TFC-PLAT-002 | If no platform command arrives for 100 ms the driver shall hold the platform; after 1 s it shall bring it to level at a limited rate. | T, M |
| TFC-PLAT-003 | The E-stop shall cut the servo rail independently of every firmware. | I, M |
| TFC-PLAT-004 | The platform shall show the vehicle's long-axis tilt in the pitch and yaw planes within +-45 degrees; beyond that it shall saturate and the simulator shall mark the run. | T, M |

## Software quality
| ID | Requirement | Verif. |
|---|---|---|
| TFC-SW-001 | `core/` shall contain no dynamic allocation, exceptions, or RTTI. | I, T |
| TFC-SW-002 | `core/` shall build with warnings as errors and be clean under clang-tidy and cppcheck. | T |
| TFC-SW-003 | `core/` shall pass the full test suite under ASan and UBSan. | T |
| TFC-SW-004 | Branch coverage of the voter and FDIR code shall be as high as the structure allows (see SW-010 for the gate and `docs/CODING_STANDARD.md` for the deviation from 100%). | T |
| TFC-SW-005 | Every requirement shall trace to at least one test ID in the fault matrix or a measurement procedure. | I |
| TFC-SW-006 | Every merge to main shall run the SIL regression in CI. | T |
| TFC-SW-007 | `core/` shall compile with the strictest warning set as errors (GCC `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wold-style-cast -Wcast-qual -Wnull-dereference -Wfloat-equal -Wswitch-enum -Wswitch-default -Wmissing-declarations -Wuseless-cast -Wlogical-op -Wstack-usage=2048` among others), and each header shall compile on its own. | T |
| TFC-SW-008 | Every function in `core/` shall be at most 60 lines, 80 statements, 20 branches, nesting 4, 6 parameters and cognitive complexity 25 (clang-tidy), contain no recursion, no `goto`, no `while`/`do`, no macro, no namespace-scope variable (`tools/check_standard.py`). | T |
| TFC-SW-009 | The flight binary shall contain no heap, exception, RTTI or vtable symbol (`tools/check_elf.sh`), and the firmware shall be linked without the C library's `malloc`. | T |
| TFC-SW-010 | Line coverage of `core/` by the host tests shall be 100% and branch coverage at least 98% (`tools/coverage/core_coverage.py`); the gate may only be raised. | T |
| TFC-SW-011 | Every deliberate bug in `tools/mutation/mutations.py` shall be caught by the C++ tests (`run_unit.py`) and, unless it is a configuration or self-protection mutant, by the fault campaign (`campaign.mutate`). | T |
| TFC-SW-012 | The fault campaign (`campaign.run --strict`) shall raise no anomaly: every always-true property (M1-M8) and every single-fault property (S1-S4) shall hold on every frame of every scenario. | T |

## Interfaces
| ID | Requirement | Verif. |
|---|---|---|
| TFC-IF-001 | All bus payloads shall be 8 bytes: 6 data, 1 sequence, 1 CRC-8. | T |
| TFC-IF-002 | Any single bit flip in a payload shall be detected. | T |
| TFC-IF-003 | CAN IDs shall order SYNC, sensors, commands, actuator output, heartbeat by priority. | T |
| TFC-IF-004 | *(Deferred, until the first telemetry exists.)* One machine-readable command and telemetry dictionary shall define every ground command (name, opcode, arguments and their limits, whether an ARM is needed, the modes that refuse it), every telemetry channel and every event record. The firmware, the virtual peers and the documentation shall be checked against it (docs/DEFERRED.md section 5.4). | I, T |
| TFC-IF-005 | *(Deferred, until the first telemetry exists.)* Housekeeping telemetry shall be sent as numbered channels at a fixed rate (proposal: 1 Hz, and at once on a change of mode): mode, Safe flag, bus alarm, each node's state and strike count, the error counters, the command counters and the reset counters, each with a caution and a warning limit where one is meaningful (docs/DEFERRED.md section 5.4). | T, M |
| TFC-IF-006 | *(Deferred, until the first telemetry exists.)* Every event (latch, probation, readmission, disable, mode change, Safe request and clear, bus alarm, reset, watchdog service refused) shall be recorded with its frame number, kind, node, reason code and a per-kind sequence number, so a lost record is detectable; every ground command accepted for processing shall produce exactly one success or refused record (docs/DEFERRED.md section 5.4). | T |
