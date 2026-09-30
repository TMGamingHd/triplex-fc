# Fault matrix

SIL = software-in-the-loop (host, deterministic, runs in CI). HIL = on the hardware rig. "Test ID" names an existing automated test where one exists.
Detection times are in 10 ms frames. **Status is honest: only the rows marked Passing (SIL) have run.**

| # | Fault | How injected | Expected detection | Expected response | Requirement | Test ID | Status |
|---|---|---|---|---|---|---|---|
| F01 | FC power loss | SIL: stop node. HIL: relay cuts node power | Timeout, at most 3 frames | Isolate, Duplex, output unchanged | SYS-004, FDIR-001 | `dropout_fail_silent_node_is_isolated_and_output_unaffected` | Passing (SIL) |
| F02 | FC hang / missed deadline | HIL: infinite loop build; watchdog | Watchdog reset, timeouts | Fail-silent, then F01 | FDIR-001 | - | Not started |
| F03 | IMU stuck-at | SIL: freeze value. HIL: hold SPI line via relay | Miscompare or `StuckDetector` | Isolate the channel | FDIR-003 | `stuck_sensor_is_isolated_and_never_reaches_output` | Passing (SIL) |
| F04 | IMU bias step | SIL: add offset | Miscompare, exactly M frames | Isolate | FDIR-002 | `bias_step_isolated_in_exactly_m_frames` | Passing (SIL) |
| F05 | IMU slow drift | SIL: ramp | Miscompare once past tolerance | Isolate | FDIR-002 | - | Not started |
| F06 | IMU spike / noise burst | SIL: random outliers | Persistence filter rejects singles | No isolation on a single spike | FDIR-004 | `single_glitch_does_not_latch` | Passing (SIL, filter only) |
| F07 | Wild / non-finite values | SIL: +-1e6, NaN | Miscompare, non-finite check | Isolate | FDIR-002 | `wild_values_isolated_without_output_excursion`, `nan_and_inf_are_treated_as_invalid` | Passing (SIL) |
| F08 | Frame corruption | SIL/HIL: bit flip | CRC-8 | Frame dropped, counts as bad sample | IF-002 | `any_single_bit_flip_is_detected`, `seq_tracker_damaged_frame_costs_one_sample_not_two`; peers e2e `test_F08_*` | Passing (SIL) |
| F09 | Wrong-but-valid command (software bug) | SIL: replica outputs offset command | Vote miscompare, digest mismatch | Isolate | FDIR-011 | - | Not started |
| F10 | Silent state divergence | SIL: perturb one replica's estimator | Digest mismatch | Flag and isolate | FDIR-011 | - | Not started |
| F11 | Babbling node | HIL: node floods high-priority IDs | Rate monitor, schedule check | Ignore out-of-schedule IDs; bounded delay | FDIR-009 | - | Not started |
| F12 | One node cut from the bus | HIL: disconnect a CAN tap | Timeout | As F01 | FDIR-001 | - | Not started |
| F13 | Bus fully lost | HIL: cut trunk | ACT timeout on all channels | Safe hold, alarm | FDIR-008 | - | Not started |
| F14 | CAN bus-off | HIL: short CANH/CANL briefly | Controller error state | Auto-recover within 100 ms | FDIR-010 | - | Not started |
| F15 | Sync master loss | HIL: power off master | Missing SYNC for 2 frames | Next FC takes over sync | SYS-001 | - | Not started |
| F16 | Two simultaneous faults | SIL | Duplex miscompare | Hold last, alarm, Safe request | FDIR-008 | `two_simultaneous_faults_fall_back_to_safe_handling` | Partial (voter only) |
| F17 | ACT reset / brownout | HIL: power-cycle ACT | Watchdog, output state | Safe-hold servo output | - | - | Not started |
| F18 | Engine-out (vehicle-level) | SIL sim: one engine thrust to zero | Vehicle dynamics, not FDIR | Controller compensates | SYS-005 | - | Not started |
