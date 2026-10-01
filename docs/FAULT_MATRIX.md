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
| F11 | Babbling node | SIL/live: peers `babble`. HIL: node floods high-priority IDs | Rate monitor, schedule check (bus alarm at 3+ stray frames per 10 ms frame) | Alarm, no node blamed; ACT ignores out-of-schedule IDs; bounded delay (HIL) | FDIR-019, FDIR-009 | `manager_bus_alarm_follows_the_out_of_schedule_rate`, peers e2e `test_F11_*`, live `test_babble_on_B_*` | Alarm passing (SIL, live); bus-timing effect: not started (HIL) |
| F12 | One node cut from the bus | HIL: disconnect a CAN tap | Timeout | As F01 | FDIR-001 | - | Not started |
| F13 | Bus fully lost | HIL: cut trunk | ACT timeout on all channels | Safe hold, alarm | FDIR-008 | - | Not started |
| F14 | CAN bus-off | HIL: short CANH/CANL briefly | Controller error state | Auto-recover within 100 ms | FDIR-010 | - | Not started |
| F15 | Sync master loss | HIL: power off master | Missing SYNC for 2 frames | Next FC takes over sync | SYS-001 | - | Not started |
| F16 | Two simultaneous faults | SIL | Duplex disagreement | Attribute by continuity when clear-cut (continue in Simplex); otherwise hold last, Safe request (sticky) | FDIR-008, FDIR-017 | `manager_duplex_second_step_fault_is_attributed_by_continuity`, `manager_slow_drift_in_duplex_requests_safe_and_blames_nobody_afterwards`, `manager_duplex_three_spikes_in_five_frames_latch_only_the_culprit`, peers e2e `test_F16_*` | Passing (SIL) |
| F17 | ACT reset / brownout | HIL: power-cycle ACT | Watchdog, output state | Safe-hold servo output | - | - | Not started |
| F18 | Engine-out (vehicle-level) | SIL sim: one engine thrust to zero | Vehicle dynamics, not FDIR | Controller compensates | SYS-005 | - | Not started |
| F19 | Single lost frame | SIL: peers `dropout` with a 1-frame window | Missing frame | Costs one bad sample; two isolated losses do not isolate a healthy node | FDIR-016 | `manager_one_lost_frame_costs_one_sample_not_two`, `manager_two_isolated_lost_frames_in_the_window_do_not_latch`, peers e2e `test_one_lost_frame_costs_one_sample` | Passing (SIL) |
| F20 | Unattributable duplex disagreement | SIL: peers `digest` or a slow `drift`/small `bias` after one node is out | Duplex vote or digest mismatch with no clear culprit | Hold last good output, Safe request (sticky), nobody blamed | FDIR-008, FDIR-018 | `manager_duplex_digest_mismatch_is_unresolved_and_blames_nobody`, `manager_holds_the_last_good_output_when_nothing_can_be_voted`, peers e2e `test_duplex_digest_mismatch_*` | Passing (SIL) |
