# SPDX-License-Identifier: MIT
"""The mutants: small deliberate bugs injected into a COPY of core/include. A test suite that cannot tell a mutant from
the real code is not testing that behaviour. Each entry is name -> (file, text to find, text to put instead); the
text must occur in the file (the harness refuses to run otherwise, so a refactor that moves code cannot silently
turn a mutant into a no-op).

Used by tools/mutation/run_unit.py (the C++ unit, fuzz and recovery tests) and sim/campaign/mutate.py (the fault campaign).
"""
from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

MUTATIONS: dict[str, tuple[str, str, str]] = {
    # ---- voting and arbitration ----
    "voter_takes_first_value": ("voter.hpp", "r.value = median3(x[0], x[1], x[2]);", "r.value = x[0];"),
    "median_wrong": ("voter.hpp", "return lo_ab > mid ? lo_ab : mid;", "return lo_ab < mid ? lo_ab : mid;"),
    "voter_nan_leaks": ("voter.hpp", "if (((mask >> i) & 1U) != 0U && !std::isfinite(x[i])) {", "if (false) {"),
    "duplex_agrees_too_easily": ("voter.hpp", "if (std::fabs(x[first] - x[second]) <= tol) {", "if (std::fabs(x[first] - x[second]) <= tol * 1000.0F) {"),
    "tolerances_doubled": ("redundancy.hpp", "{{1.0F, 1.0F, 1.0F, 0.02F, 0.02F, 0.02F, 0.01F, 0.01F}}", "{{2.0F, 2.0F, 2.0F, 0.04F, 0.04F, 0.04F, 0.02F, 0.02F}}"),
    "arbitrate_stale_reference": ("redundancy.hpp", "if (!have_last_[ch] || !ref_fresh_[ch] ||", "if (!have_last_[ch] ||"),
    "arbitration_too_lenient": ("redundancy.hpp", "float duplex_arbitration_factor = 2.0F;", "float duplex_arbitration_factor = 0.3F;"),
    "arbitration_standstill_reference": ("redundancy.hpp", "return last_good_[ch] + weight * step;", "return last_good_[ch];"),
    "arbitration_extrapolates_backwards": ("redundancy.hpp", "return last_good_[ch] + weight * step;", "return last_good_[ch] - weight * step;"),
    "untrusted_updates_last_good": ("redundancy.hpp", "if (trusted && !safe_now) {", "if (!safe_now) {"),
    "hold_returns_zero": ("redundancy.hpp", "rep.output[ch] = have_last_[ch] ? last_good_[ch] : 0.0F;  // hold the last good value", "rep.output[ch] = 0.0F;"),
    "digest_duplex_blames_both": ("redundancy.hpp", "} else if (rx_[i].digest != first) {\n          d.unresolved = true;", "} else if (rx_[i].digest != first) {\n          d.unresolved = true; d.blame = valid;"),
    # ---- Safe request ----
    "safe_not_sticky": ("redundancy.hpp", "    rep.safe_request = safe_requested();\n", "    if (!unresolved) { safe_.set(0U); }\n    rep.safe_request = safe_requested();\n"),
    "mode_ignores_safe": ("redundancy.hpp", "rep.mode = rep.safe_request ? Mode::Safe : mode_from_healthy(rep.healthy);", "rep.mode = mode_from_healthy(rep.healthy);"),
    "output_not_held_in_safe": ("redundancy.hpp", "      rep.held_mask = 0xFFU;\n    }\n    counters_.unresolved_frames", "    }\n    counters_.unresolved_frames"),
    # ---- detection ----
    "latched_node_votes": ("redundancy.hpp", "if (good[n] && state_of(n) == NodeState::Healthy) {\n        valid", "if (good[n]) {\n        valid"),
    "no_alpha_count": ("redundancy.hpp", "const bool by_alpha = alpha_[n].update(bad);", "const bool by_alpha = false && alpha_[n].update(bad);"),
    "alpha_not_cleared_on_readmit": ("redundancy.hpp", "        alpha_[n].reset();\n", ""),
    "stuck_limit_doubled": ("redundancy.hpp", "uint16_t stuck_limit = 20;", "uint16_t stuck_limit = 40;"),
    "bus_alarm_off_by_one": ("redundancy.hpp", "rep.bus_alarm = oos_in_frame_ >= cfg_.bus_alarm_per_frame", "rep.bus_alarm = oos_in_frame_ > cfg_.bus_alarm_per_frame"),
    "unknown_ids_count_as_known": ("redundancy.hpp", "(f.id >= id::kSim && f.id <= id::kSimLast)", "(f.id >= id::kSim)"),
    # ---- sequence tracking ----
    "decoder_ignores_len": ("protocol.hpp", "return f.len == 8U && f.data[7] == crc8(f.data.data(), 7);", "return f.data[7] == crc8(f.data.data(), 7);"),
    "counter_decrements": ("redundancy.hpp", "++counters_.crc_bad;", "counters_.crc_bad = counters_.crc_bad > 5 ? counters_.crc_bad - 1 : counters_.crc_bad + 1;"),
    # ---- life cycle ----
    "no_shadow_vote": ("redundancy.hpp", "if (std::fabs(rx_[n].x[ch] - rep.output[ch]) > cfg_.tol[ch]) {", "if (false) {"),
    "strikes_never_disable": ("redundancy.hpp", "uint8_t max_strikes = 3;", "uint8_t max_strikes = 250;"),
    "one_probation_at_a_time_off": ("redundancy.hpp", "if ((!one_on_probation || no_healthy) && dwell_[n]", "if (dwell_[n]"),
    "probation_neutral_counts_clean": ("redundancy.hpp", "} else if (v == Verdict::Clean) {", "} else if (v != Verdict::Dirty) {"),
    "no_dwell": ("redundancy.hpp", "dwell_[n] >= dwell_needed(n) && wants_probation(n)", "wants_probation(n)"),
    "probation_too_short": ("redundancy.hpp", "uint16_t probation_frames = 100;", "uint16_t probation_frames = 10;"),
    "dwell_counts_command_frame": ("redundancy.hpp", "    dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << node));  // the frame of the command is not part of the dwell\n", ""),
    "dwell_counts_failure_frame": ("redundancy.hpp", "      dwell_[n] = 0U;\n      dwell_hold_ = static_cast<uint8_t>(dwell_hold_ | (1U << n));\n      req_[n] = false;\n      if (attempts_", "      dwell_[n] = 0U;\n      req_[n] = false;\n      if (attempts_"),
    # ---- commands ----
    "disable_command_ignored": ("redundancy.hpp", "    set_state(node, NodeState::Disabled);\n    req_[node] = false;\n    ++counters_.nodes_disabled;", "    req_[node] = false;\n    ++counters_.nodes_disabled;"),
    "clear_disabled_keeps_strikes": ("redundancy.hpp", "    strikes_[node] = 0U;\n    dwell_[node] = 0U;", "    dwell_[node] = 0U;"),
    "readmit_not_reported": ("redundancy.hpp", "        rep.newly_reintegrated = static_cast<uint8_t>(rep.newly_reintegrated | (1U << n));\n", "\n"),
    "disabled_not_reported": ("redundancy.hpp", "      rep.newly_disabled = static_cast<uint8_t>(rep.newly_disabled | (1U << n));\n      ++counters_.nodes_disabled;", "      ++counters_.nodes_disabled;"),
    # ---- total-loss recovery (ADR-014) ----
    "cohort_never_usable": ("redundancy.hpp", "if (count_in_state(NodeState::Healthy) != 0U || count_channels(c.members) < 2U) {", "if (true) {"),
    "cohort_keeps_the_odd_one": ("redundancy.hpp", "if (((v.disagree_mask >> n) & 1U) != 0U) {\n          add_reason(rep, n, reason::kVote);", "if (false) {\n          add_reason(rep, n, reason::kVote);"),
    "cohort_neutral_counts_clean": ("redundancy.hpp", "return neutral ? Verdict::Neutral : Verdict::Clean;", "return Verdict::Clean;"),
    "cohort_one_at_a_time": ("redundancy.hpp", "if ((!one_on_probation || no_healthy) && dwell_[n]", "if (!one_on_probation && dwell_[n]"),
    # ---- frame numbers (ADR-018) ----
    "phase_late_slot_not_checked_empty": ("protocol.hpp", "if (behind > 0U && (earlier_arrivals & bit) != 0U) {", "if (false) {"),
    "phase_duplicates_accepted": ("protocol.hpp", "    if ((seen_ & bit) != 0U) {\n      return FrameTiming::Bad;", "    if (false) {\n      return FrameTiming::Bad;"),
    "phase_future_numbers_look_late": ("protocol.hpp", "static_cast<uint8_t>(static_cast<uint8_t>(frame_no & 0xFFU) - seq);", "static_cast<uint8_t>(seq - static_cast<uint8_t>(frame_no & 0xFFU));"),
    "phase_damaged_frame_leaves_its_slot_empty": ("redundancy.hpp", "      phase_[node][stream].note_damaged();\n", ""),
    "phase_frame_counter_not_advanced": ("redundancy.hpp", "    ++frame_no_;\n    return rep;", "    return rep;"),
    "phase_begin_frame_ignores_the_number": ("redundancy.hpp", "    frame_no_ = frame_no;\n    begin_frame();", "    begin_frame();"),
    # ---- ground commands (ADR-019) ----
    "ground_tag_not_checked": ("redundancy.hpp", "if (!ground_authentic(f, cfg_.ground_key)) {", "if (false) {"),
    "ground_replay_window_ignored": ("redundancy.hpp", "if (ahead == 0U || ahead > cfg_.command_window) {", "if (false) {"),
    "ground_replay_window_off_by_one": ("redundancy.hpp", "if (ahead == 0U || ahead > cfg_.command_window) {", "if (ahead == 0U || ahead >= cfg_.command_window) {"),
    "ground_counter_not_remembered": ("redundancy.hpp", "      cmd_ctr_.set(d.counter);\n      cmd_have_.set(1U);", "      cmd_have_.set(1U);"),
    "ground_auth_off_by_default": ("redundancy.hpp", "bool ground_auth = true; ", "bool ground_auth = false;"),
    "arm_clear_safe_needs_none": ("redundancy.hpp", "if (op == GroundOp::ClearDisabled || op == GroundOp::ClearSafe || op == GroundOp::Launch) {", "if (op == GroundOp::ClearDisabled || op == GroundOp::Launch) {"),
    "arm_clear_disabled_needs_none": ("redundancy.hpp", "if (op == GroundOp::ClearDisabled || op == GroundOp::ClearSafe || op == GroundOp::Launch) {", "if (op == GroundOp::ClearSafe || op == GroundOp::Launch) {"),
    "interlock_off": ("redundancy.hpp", "n.arm = healthy <= 2U; ", "n.arm = false; "),
    "interlock_one_tier_late": ("redundancy.hpp", "n.arm = healthy <= 2U; ", "n.arm = healthy <= 1U; "),
    "interlock_last_voter_not_flagged": ("redundancy.hpp", "n.critical = healthy <= 1U;", "n.critical = false;"),
    "arm_not_consumed": ("redundancy.hpp", "      clear_arm();  // an ARM covers exactly one EXECUTE\n", ""),
    "arm_node_not_matched": ("redundancy.hpp", "(static_cast<unsigned>(op) << 2U) | (node & 3U)", "(static_cast<unsigned>(op) << 2U)"),
    "arm_never_ages": ("redundancy.hpp", "    arm_left_.set(static_cast<uint8_t>(arm_left_.get() - 1U));\n", ""),
    "arm_window_one_too_long": ("redundancy.hpp", "arm_left_.set(cfg_.arm_window_frames);", "arm_left_.set(static_cast<uint8_t>(cfg_.arm_window_frames + 1U));"),
    "scrub_ignores_command_state": ("redundancy.hpp", "    if (!cmd_ctr_.intact() || !cmd_have_.intact() || !arm_code_.intact() || !arm_left_.intact()) {", "    if (false) {"),
    # ---- fast reintegration and the arbitration reference (ADR-010, ADR-017 amended) ----
    "transient_dwell_ignored": ("redundancy.hpp", "return transient ? cfg_.min_dwell_frames_transient : cfg_.min_dwell_frames;", "return cfg_.min_dwell_frames;"),
    "transient_dwell_for_every_cause": ("redundancy.hpp", "return transient ? cfg_.min_dwell_frames_transient : cfg_.min_dwell_frames;", "return cfg_.min_dwell_frames_transient;"),
    "reference_always_predicts": ("redundancy.hpp", "const float weight = moving <= 1.0F ? 0.0F : (moving >= 2.0F ? 1.0F : moving - 1.0F);", "const float weight = 1.0F;"),
    "reference_never_predicts": ("redundancy.hpp", "const float weight = moving <= 1.0F ? 0.0F : (moving >= 2.0F ? 1.0F : moving - 1.0F);", "const float weight = 0.0F;"),
    # ---- configuration and self protection (ADR-015) ----
    "config_not_sanitised": ("redundancy.hpp", "  errors = validate_config(in);", "  errors = 0U;"),
    "tolerance_not_validated": ("redundancy.hpp", "    if (!detail::finite_positive(c.tol[i])) {\n      e |= cfgerr::kTolerance;", "    if (false) {\n      e |= cfgerr::kTolerance;"),
    "persistence_not_validated": ("redundancy.hpp", "if (c.persist_m < 1U || c.persist_m > c.persist_n || c.persist_n > 32U) {", "if (false) {"),
    "scrub_ignores_node_state": ("redundancy.hpp", "      if (!state_valid(n)) {\n        set_state(n, NodeState::Latched);", "      if (false) {\n        set_state(n, NodeState::Latched);"),
    "scrub_ignores_safe_flag": ("redundancy.hpp", "if (!safe_.intact() || safe_.get() > 1U) {", "if (false) {"),
    "scrub_ignores_config": ("redundancy.hpp", "    if (!repair_config()) {", "    if (false && !repair_config()) {"),
    "config_not_restored": ("redundancy.hpp", "      cfg_ = cfg_backup_;\n      cfg_digest_ = cfg_digest_backup_;", "      cfg_digest_ = cfg_digest_backup_;"),
    "queue_length_unchecked": ("redundancy.hpp", "if (!ensure(npending_ <= kMaxCommandsPerFrame, counters_.invariant_violations)) {", "if (false) {"),
    # ---- the Pico: platform driver, injector, link, frame loop (docs/PICO_TESTS.md A5) ----
    "pico_travel_not_limited": ("platform_driver.hpp", "return v > cfg_.limit_deg ? cfg_.limit_deg : (v < -cfg_.limit_deg ? -cfg_.limit_deg : v);", "return v;"),
    "pico_travel_asymmetric": ("platform_driver.hpp", "(v < -cfg_.limit_deg ? -cfg_.limit_deg : v)", "(v < -cfg_.limit_deg - 5.0F ? -cfg_.limit_deg : v)"),
    "pico_rate_not_limited": ("platform_driver.hpp", "return d > step ? cur + step : (d < -step ? cur - step : goal);", "return goal;"),
    "pico_rate_limit_one_way": ("platform_driver.hpp", "(d < -step ? cur - step : goal)", "goal"),
    "pico_never_holds": ("platform_driver.hpp", "} else if (age_ms_ >= cfg_.hold_after_ms) {", "} else if (false) {"),
    "pico_holds_one_tick_late": ("platform_driver.hpp", "} else if (age_ms_ >= cfg_.hold_after_ms) {", "} else if (age_ms_ > cfg_.hold_after_ms) {"),
    "pico_never_levels": ("platform_driver.hpp", "if (!have_ || age_ms_ >= cfg_.level_after_ms) {", "if (!have_) {"),
    "pico_levels_at_the_full_rate": ("platform_driver.hpp", "rate = cfg_.level_rate_dps;", "rate = cfg_.rate_limit_dps;"),
    "pico_nan_accepted": ("platform_driver.hpp", "if (!finite(x_deg) || !finite(y_deg)) {", "if (false) {"),
    "pico_rejected_command_restarts_timeout": ("platform_driver.hpp", "      ++rejected_;\n      return false;", "      ++rejected_;\n      age_ms_ = 0U;\n      return false;"),
    "pico_stale_command_still_saturated": ("platform_driver.hpp", "out_.saturated = have_ && age_ms_ < cfg_.hold_after_ms && (", "out_.saturated = have_ && ("),
    "pico_age_wraps": ("platform_driver.hpp", "age_ms_ = age_ms_ > 0xFFFFU - dt_ms ? 0xFFFFU : static_cast<uint16_t>(age_ms_ + dt_ms);", "age_ms_ = static_cast<uint16_t>(age_ms_ + dt_ms);"),
    "pico_servo_sign_ignored": ("platform_driver.hpp", "neutral_us + (sign * (angle_deg + trim_deg) * us_per_deg)", "neutral_us + ((angle_deg + trim_deg) * us_per_deg)"),
    "pico_servo_trim_ignored": ("platform_driver.hpp", "(sign * (angle_deg + trim_deg) * us_per_deg)", "(sign * angle_deg * us_per_deg)"),
    "pico_servo_pulse_not_clamped": ("platform_driver.hpp", "return p < min_us ? min_us : (p > max_us ? max_us : p);", "return p;"),
    "pico_cut_not_clamped": ("injector.hpp", "left_[channel] = cut_ms > kMaxCutMs ? kMaxCutMs : cut_ms;", "left_[channel] = cut_ms;"),
    "pico_cut_never_ends": ("injector.hpp", "l = l > dt_ms ? static_cast<uint16_t>(l - dt_ms) : 0U;", "l = l;"),
    "pico_cut_ends_one_tick_early": ("injector.hpp", "l = l > dt_ms ? static_cast<uint16_t>(l - dt_ms) : 0U;", "l = l > 2U * dt_ms ? static_cast<uint16_t>(l - dt_ms) : 0U;"),
    "pico_channel_not_checked": ("injector.hpp", "if (channel >= kInjectorChannels) {\n      return false;\n    }", "if (channel >= 8U) {\n      return false;\n    }"),
    "pico_release_all_does_nothing": ("injector.hpp", "void release_all() noexcept { left_ = {}; }", "void release_all() noexcept {}"),
    "pico_energised_bit_wrong": ("injector.hpp", "m = static_cast<uint8_t>(m | (left_[i] > 0U ? (1U << i) : 0U));", "m = static_cast<uint8_t>(m | (left_[i] > 0U ? (1U << (i ^ 1U)) : 0U));"),
    "pico_link_crc_not_checked": ("pico_link.hpp", "if (b != crc8(buf_.data(), 2U + static_cast<std::size_t>(buf_[1]))) {", "if (false) {"),
    "pico_link_length_not_checked": ("pico_link.hpp", "if (b > kMaxPayload) {  // not a frame", "if (false) {  // not a frame"),
    "pico_link_no_resync_after_bad_crc": ("pico_link.hpp", "      if (b == kSync) {  // the byte that failed may itself start a frame\n        state_ = State::Type;\n      }\n", ""),
    "pico_link_no_resync_after_bad_length": ("pico_link.hpp", "state_ = b == kSync ? State::Type : State::Sync;", "state_ = State::Sync;"),
    "pico_link_angle_truncates": ("pico_link.hpp", "return static_cast<int16_t>(q >= 0.0F ? q + 0.5F : q - 0.5F);", "return static_cast<int16_t>(q);"),
    "pico_link_angle_not_saturated": ("pico_link.hpp", "  if (q >= 32767.0F) {\n    return 32767;\n  }", "  if (q >= 40000.0F) {\n    return 32767;\n  }"),
    "pico_link_status_length_unchecked": ("pico_link.hpp", "if (m.type != Type::Status || m.length != 9U) {", "if (m.type != Type::Status) {"),
    "pico_app_boot_leaves_relays": ("pico_app.hpp", "    hal_.set_relays(0U);\n    hal_.set_servo_us(0U, cfg_.servo_x.pulse_us(0.0F));", "    hal_.set_servo_us(0U, cfg_.servo_x.pulse_us(0.0F));"),
    "pico_app_link_timeout_ignored": ("pico_app.hpp", "if (quiet_ms_ >= cfg_.link_timeout_ms) {", "if (false) {"),
    "pico_app_link_timeout_one_step_late": ("pico_app.hpp", "if (quiet_ms_ >= cfg_.link_timeout_ms) {", "if (quiet_ms_ > cfg_.link_timeout_ms) {"),
    "pico_app_quiet_not_reset_by_a_frame": ("pico_app.hpp", "quiet_ms_ = heard ? 0U : (quiet_ms_ + cfg_.period_ms);", "quiet_ms_ = quiet_ms_ + cfg_.period_ms;"),
    "pico_app_link_lost_never_clears": ("pico_app.hpp", "    } else if (heard) {\n      link_lost_ = false;\n    }", "    }"),
    "pico_app_relays_not_applied": ("pico_app.hpp", "    hal_.set_relays(injector_.energised());\n\n    // ---- the status", "    // ---- the status"),
    "pico_app_ping_not_answered": ("pico_app.hpp", "if (ping || (cfg_.status_every_steps != 0U", "if ((cfg_.status_every_steps != 0U"),
    "pico_app_watchdog_not_fed": ("pico_app.hpp", "    hal_.feed_watchdog();\n  }", "  }"),
    # (the app's sequence echo only follows commands the driver accepted; through the 16-bit link a command is always a finite number, so that path cannot be reached from outside and a mutant of it
    #  would be equivalent: the rejection itself is tested directly on the driver)
    "pico_app_y_servo_gets_x_map": ("pico_app.hpp", "hal_.set_servo_us(1U, cfg_.servo_y.pulse_us(platform_.output().y_deg));", "hal_.set_servo_us(1U, cfg_.servo_x.pulse_us(platform_.output().y_deg));"),
    "pico_app_status_flags_missing_link_lost": ("pico_app.hpp", "f = static_cast<uint8_t>(f | (link_lost_ ? pico::statusflag::kLinkLost : 0U));", "f = f;"),
    # ---- the pad phase: per-IMU calibration and the mission state (docs/LAUNCH_SEQUENCE.md) ----
    "cal_bias_not_frozen": ("imu_calibration.hpp", "      bias_ = mean_;  // frozen at the final mean\n", "\n"),
    "cal_applies_from_the_first_sample": ("imu_calibration.hpp", "if (n_ >= cfg_.apply_after) {", "if (true) {"),
    "cal_ready_ignores_spread": ("imu_calibration.hpp", "if (!(var <= cfg_.max_std_dps * cfg_.max_std_dps) || !(std::fabs(mean_.v[i]) <= cfg_.max_bias_dps)) {", "if (!(std::fabs(mean_.v[i]) <= cfg_.max_bias_dps)) {"),
    "cal_ready_ignores_bias_limit": ("imu_calibration.hpp", "if (!(var <= cfg_.max_std_dps * cfg_.max_std_dps) || !(std::fabs(mean_.v[i]) <= cfg_.max_bias_dps)) {", "if (!(var <= cfg_.max_std_dps * cfg_.max_std_dps)) {"),
    "cal_ready_with_too_few_samples": ("imu_calibration.hpp", "if (n_ < 2U || n_ < cfg_.samples) {", "if (n_ < 2U) {"),
    "cal_counts_non_numbers": ("imu_calibration.hpp", "if (pad_ && finite(raw)) {", "if (pad_) {"),
    "cal_pad_start_keeps_the_old_samples": ("imu_calibration.hpp", "      n_ = 0U;\n      mean_ = {};", "      mean_ = {};"),
    "cal_subtracts_with_the_wrong_sign": ("imu_calibration.hpp", "out.v[i] = raw.v[i] - bias_.v[i];", "out.v[i] = raw.v[i] + bias_.v[i];"),
    "cal_never_stops_counting": ("imu_calibration.hpp", "if (n_ >= 60000U) {", "if (false) {"),
    "mission_schedules_ignore_the_flight_frame": ("flight.hpp", "const uint32_t idx = !mission_set_ ? frame_ : (pad_ ? 0U : flight_frame_);", "const uint32_t idx = frame_;"),
    "mission_schedules_run_on_the_pad": ("flight.hpp", "(pad_ ? 0U : flight_frame_)", "flight_frame_"),
    "mission_sensors_ok_ignores_the_attitude": ("flight.hpp", "return c.gyro_ok && c.accel_ok && estimator_.attitude().valid;", "return c.gyro_ok && c.accel_ok;"),
    # ---- launch: mission time in SYNC, the launch commands, the gate (docs/LAUNCH_SEQUENCE.md) ----
    "mission_follower_ignores_sync_before_t_zero": ("sync_clock.hpp", "if (!mission::in_flight(mission_)) {", "if (false) {"),
    "mission_flying_follower_adopts_sync": ("sync_clock.hpp", "} else if (mission_ != heard_mission) {\n        t.mission_disagrees = true;", "} else if (mission_ != heard_mission) {\n        mission_ = heard_mission;\n        t.mission_disagrees = true;"),
    "mission_disagreement_not_reported": ("sync_clock.hpp", "        t.mission_disagrees = true;\n      }", "      }"),
    "mission_not_counted_through_a_gap": ("sync_clock.hpp", "      ++mission_;\n    }\n    ++next_;", "    }\n    ++next_;"),
    "mission_count_wraps_at_the_largest_value": ("sync_clock.hpp", "if (mission_ != mission::kNotLaunched && mission_ != mission::kMax) {", "if (mission_ != mission::kNotLaunched) {"),
    "launch_by_a_follower": ("sync_clock.hpp", "if (!master_ || mission_ != mission::kNotLaunched) {", "if (mission_ != mission::kNotLaunched) {"),
    "launch_twice": ("sync_clock.hpp", "if (!master_ || mission_ != mission::kNotLaunched) {", "if (!master_) {"),
    "scrub_after_t_zero": ("sync_clock.hpp", "if (!master_ || !mission::in_countdown(mission_)) {", "if (!master_) {"),
    "scrub_by_a_follower": ("sync_clock.hpp", "if (!master_ || !mission::in_countdown(mission_)) {", "if (!mission::in_countdown(mission_)) {"),
    "mission_countdown_boundary": ("protocol.hpp", "return m != kNotLaunched && m <= kCountdownFrames; }", "return m != kNotLaunched && m < kCountdownFrames; }"),
    "mission_flight_frames_off_by_one": ("protocol.hpp", "static_cast<uint32_t>(m) - kCountdownFrames - 1U : 0U; }", "static_cast<uint32_t>(m) - kCountdownFrames : 0U; }"),
    "gate_accepts_two_healthy_nodes": ("launch_gate.hpp", "if ((f.healthy_nodes & 0x07U) != 0x07U) {", "if ((f.healthy_nodes & 0x07U) == 0U) {"),
    "gate_accepts_a_node_not_ready": ("launch_gate.hpp", "if ((f.ready_nodes & 0x07U) != 0x07U) {", "if ((f.ready_nodes & 0x07U) == 0U) {"),
    "gate_ignores_the_safe_request": ("launch_gate.hpp", "  if (f.safe_requested) {", "  if (false) {"),
    "gate_ignores_act": ("launch_gate.hpp", "  if (!f.act_nominal) {", "  if (false) {"),
    "launch_needs_no_arm": ("redundancy.hpp", "if (op == GroundOp::ClearDisabled || op == GroundOp::ClearSafe || op == GroundOp::Launch) {", "if (op == GroundOp::ClearDisabled || op == GroundOp::ClearSafe) {"),
    "heartbeat_ready_not_packed": ("protocol.hpp", " | ((h.ready ? 1U : 0U) << 7U));", ");"),
    "estimator_ignores_use_accel": ("estimator.hpp", "const bool accel_usable = cfg_.use_accel && in.accel_ok;", "const bool accel_usable = in.accel_ok;"),
    "pico_app_status_relays_stale": ("pico_app.hpp", "st.relays = injector_.energised();", "st.relays = 0U;"),
    # ---- the supervisor's clock of record (supervisor/include/sup/mission_clock.hpp): file names start with "sup/" ----
    "sup_extender_ignores_the_wrap": ("sup/mission_clock.hpp", "if (started_ && hw < last_) {", "if (false) {"),
    "sup_extender_wraps_on_an_equal_reading": ("sup/mission_clock.hpp", "if (started_ && hw < last_) {", "if (started_ && hw <= last_) {"),
    "sup_crc_ignores_the_high_bytes": ("sup/mission_clock.hpp", "static_cast<uint8_t>(t0_rtc_s >> (8U * (i - 4U)))", "static_cast<uint8_t>(t0_rtc_s >> (8U * ((i - 4U) % 4U)))"),
    "sup_record_ignores_the_crc": ("sup/mission_clock.hpp", "r.magic == kMetMagic && r.crc == met_crc(r.magic, r.t0_rtc_s)", "r.magic == kMetMagic"),
    "sup_record_ignores_the_magic": ("sup/mission_clock.hpp", "r.magic == kMetMagic && r.crc == met_crc(r.magic, r.t0_rtc_s)", "r.crc == met_crc(r.magic, r.t0_rtc_s)"),
    "sup_second_t0_replaces_the_first": ("sup/mission_clock.hpp", "if (source_ == MetSource::None) {", "if (true) {"),
    "sup_met_us_overflows": ("sup/mission_clock.hpp", "return (s * 1000000U) + ((r * 1000000U) / cfg_.tick_hz);", "return (t * 1000000U) / cfg_.tick_hz;"),
    "sup_met_frames_wrong_divisor": ("sup/mission_clock.hpp", "return met_ticks(ticks_now) / (static_cast<uint64_t>(cfg_.tick_hz / 100U));", "return met_ticks(ticks_now) / (static_cast<uint64_t>(cfg_.tick_hz / 1000U));"),
    "sup_resume_accepts_a_future_t0": ("sup/mission_clock.hpp", "if (!record_valid(rec) || rec.t0_rtc_s > rtc_now_s) {", "if (!record_valid(rec)) {"),
    "sup_resume_without_an_offset": ("sup/mission_clock.hpp", "if (elapsed_ticks > ticks_now) {", "if (false) {"),
    "sup_met_before_t0_wraps": ("sup/mission_clock.hpp", "if (!launched() || ticks_now + offset_ticks_ < t0_ticks_) {", "if (!launched()) {"),
    "sup_recovered_t0_claims_exactness": ("sup/mission_clock.hpp", "source_ == MetSource::Recovered ? 1000000U : 0U", "0U"),
    "sup_watch_field_not_saturated": ("sup/mission_clock.hpp", "expect_wide > kMissionSaturated ? static_cast<uint64_t>(kMissionSaturated) : expect_wide", "expect_wide"),
    "sup_watch_no_drift_allowance": ("sup/mission_clock.hpp", "((met * cfg_.ppm) / 1000000U)", "0U"),
    "sup_watch_flags_at_once": ("sup/mission_clock.hpp", "return bad_run_ >= cfg_.persist;", "return bad_run_ >= 1U;"),
    "sup_watch_run_count_wraps": ("sup/mission_clock.hpp", "(bad_run_ < 255U ? static_cast<uint8_t>(bad_run_ + 1U) : bad_run_)", "static_cast<uint8_t>(bad_run_ + 1U)"),
    "sup_watch_ahead_not_detected": ("sup/mission_clock.hpp", "} else if (got > expect + allow) {", "} else if (false) {"),
    "sup_watch_behind_not_detected": ("sup/mission_clock.hpp", "if (got + allow < expect) {", "if (false) {"),
    "sup_watch_ignores_the_recovery_slack": ("sup/mission_clock.hpp", " + clock_slack(clock);", ";"),
    "sup_watch_run_survives_a_clock_reset": ("sup/mission_clock.hpp", "      bad_run_ = 0U;\n      return Verdict::NotLaunched;", "      return Verdict::NotLaunched;"),
}

# Mutants that only the C++ unit tests can see, with the reason: the campaign's peers cannot produce the input that
# distinguishes them from the real code.
CAMPAIGN_SKIP = {
    # the supervisor's code is not in the campaign's reach (the campaign drives the flight bus)
    *(n for n in MUTATIONS if n.startswith("sup_")),
    # the launch sequence is not in the campaign's reach (its peers send no mission frame and no launch command)
    "mission_follower_ignores_sync_before_t_zero", "mission_flying_follower_adopts_sync", "mission_disagreement_not_reported", "mission_not_counted_through_a_gap", "mission_count_wraps_at_the_largest_value", "launch_by_a_follower", "launch_twice", "scrub_after_t_zero", "scrub_by_a_follower", "mission_countdown_boundary", "mission_flight_frames_off_by_one", "gate_accepts_two_healthy_nodes", "gate_accepts_a_node_not_ready", "gate_ignores_the_safe_request", "gate_ignores_act", "launch_needs_no_arm", "heartbeat_ready_not_packed",
    # the pad phase is not in the campaign's reach (its peers do not run the flight function)
    "cal_bias_not_frozen", "cal_applies_from_the_first_sample", "cal_ready_ignores_spread", "cal_ready_ignores_bias_limit", "cal_ready_with_too_few_samples", "cal_counts_non_numbers", "cal_pad_start_keeps_the_old_samples", "cal_subtracts_with_the_wrong_sign", "cal_never_stops_counting", "mission_schedules_ignore_the_flight_frame", "mission_schedules_run_on_the_pad", "mission_sensors_ok_ignores_the_attitude",
    "estimator_ignores_use_accel",  # the campaign's peers do not run the estimator
    # the Pico's code is not in the campaign's reach (the campaign drives the flight bus, not the Pico's USB link)
    "pico_travel_not_limited", "pico_travel_asymmetric", "pico_rate_not_limited", "pico_rate_limit_one_way", "pico_never_holds", "pico_holds_one_tick_late", "pico_never_levels", "pico_levels_at_the_full_rate", "pico_nan_accepted", "pico_rejected_command_restarts_timeout", "pico_stale_command_still_saturated", "pico_age_wraps", "pico_servo_sign_ignored", "pico_servo_trim_ignored", "pico_servo_pulse_not_clamped", "pico_cut_not_clamped", "pico_cut_never_ends", "pico_cut_ends_one_tick_early", "pico_channel_not_checked", "pico_release_all_does_nothing", "pico_energised_bit_wrong", "pico_link_crc_not_checked", "pico_link_length_not_checked", "pico_link_no_resync_after_bad_crc", "pico_link_no_resync_after_bad_length", "pico_link_angle_truncates", "pico_link_angle_not_saturated", "pico_link_status_length_unchecked", "pico_app_boot_leaves_relays", "pico_app_link_timeout_ignored", "pico_app_link_timeout_one_step_late", "pico_app_quiet_not_reset_by_a_frame", "pico_app_link_lost_never_clears", "pico_app_relays_not_applied", "pico_app_ping_not_answered", "pico_app_watchdog_not_fed", "pico_app_y_servo_gets_x_map", "pico_app_status_flags_missing_link_lost", "pico_app_status_relays_stale",
    "config_not_sanitised", "tolerance_not_validated", "persistence_not_validated",  # the peers never present a bad configuration
    "scrub_ignores_node_state", "scrub_ignores_safe_flag", "scrub_ignores_config",   # nor flip a bit of the manager's memory
    "config_not_restored", "queue_length_unchecked",                                  # nor overflow its command queue
    "voter_nan_leaks",          # an int16 sample cannot be NaN or infinite
    "decoder_ignores_len",      # the peers never send a frame of the wrong length
    "counter_decrements",       # a statistics counter is not an input to any decision
    "ground_replay_window_off_by_one",  # the campaign's command counters advance by one; the window edge needs a gap of exactly 32
    "scrub_ignores_command_state",      # the peers never flip a bit of the manager's memory
    "phase_begin_frame_ignores_the_number",  # the replay tool counts frames itself and never passes a number
}


def build_include(name: str, dest: Path) -> Path:
    """Copy core/include into `dest` with mutation `name` applied; returns the include directory."""
    inc = dest / "include"
    shutil.copytree(ROOT / "core/include", inc)
    shutil.copytree(ROOT / "supervisor/include", dest / "include_sup")
    fn, old, new = MUTATIONS[name]
    p = dest / "include_sup" / fn if fn.startswith("sup/") else inc / "tfc" / fn
    text = p.read_text()
    if old not in text:
        raise RuntimeError(f"mutation {name}: pattern not found in {fn} (the code moved: update tools/mutation/mutations.py)")
    p.write_text(text.replace(old, new, 1))
    return inc


def check_patterns() -> list[str]:
    """Names of mutations whose pattern no longer occurs in the current core."""
    return [n for n, (fn, old, _new) in MUTATIONS.items() if old not in ((ROOT / "supervisor/include" / fn) if fn.startswith("sup/") else (ROOT / "core/include/tfc" / fn)).read_text()]
