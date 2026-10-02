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
    "arbitration_standstill_reference": ("redundancy.hpp", "const float ref = have_prev_[ch] ? last_good_[ch] + (last_good_[ch] - prev_good_[ch]) : last_good_[ch];", "const float ref = last_good_[ch];"),
    "arbitration_extrapolates_backwards": ("redundancy.hpp", "const float ref = have_prev_[ch] ? last_good_[ch] + (last_good_[ch] - prev_good_[ch]) : last_good_[ch];", "const float ref = have_prev_[ch] ? last_good_[ch] - (last_good_[ch] - prev_good_[ch]) : last_good_[ch];"),
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
    "unknown_ids_count_as_known": ("redundancy.hpp", "f.id == id::kActOut || f.id == id::kSim ||", "f.id == id::kActOut || f.id >= id::kSim ||"),
    # ---- sequence tracking ----
    "lost_frame_costs_two": ("redundancy.hpp", "seq_[n][st].note_missing();", ""),
    "seq_late_accepts_one_too_many": ("protocol.hpp", "behind < pending_", "behind <= pending_"),
    "decoder_ignores_len": ("protocol.hpp", "return f.len == 8U && f.data[7] == crc8(f.data.data(), 7);", "return f.data[7] == crc8(f.data.data(), 7);"),
    "counter_decrements": ("redundancy.hpp", "++counters_.crc_bad;", "counters_.crc_bad = counters_.crc_bad > 5 ? counters_.crc_bad - 1 : counters_.crc_bad + 1;"),
    # ---- life cycle ----
    "no_shadow_vote": ("redundancy.hpp", "if (std::fabs(rx_[n].x[ch] - rep.output[ch]) > cfg_.tol[ch]) {", "if (false) {"),
    "strikes_never_disable": ("redundancy.hpp", "uint8_t max_strikes = 3;", "uint8_t max_strikes = 250;"),
    "one_probation_at_a_time_off": ("redundancy.hpp", "if ((!one_on_probation || no_healthy) && dwell_[n]", "if (dwell_[n]"),
    "probation_neutral_counts_clean": ("redundancy.hpp", "} else if (v == Verdict::Clean) {", "} else if (v != Verdict::Dirty) {"),
    "no_dwell": ("redundancy.hpp", "dwell_[n] >= cfg_.min_dwell_frames && wants_probation(n)", "wants_probation(n)"),
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
    # ---- configuration and self protection (ADR-015) ----
    "config_not_sanitised": ("redundancy.hpp", "  errors = validate_config(in);", "  errors = 0U;"),
    "tolerance_not_validated": ("redundancy.hpp", "    if (!detail::finite_positive(c.tol[i])) {\n      e |= cfgerr::kTolerance;", "    if (false) {\n      e |= cfgerr::kTolerance;"),
    "persistence_not_validated": ("redundancy.hpp", "if (c.persist_m < 1U || c.persist_m > c.persist_n || c.persist_n > 32U) {", "if (false) {"),
    "scrub_ignores_node_state": ("redundancy.hpp", "      if (!state_valid(n)) {\n        set_state(n, NodeState::Latched);", "      if (false) {\n        set_state(n, NodeState::Latched);"),
    "scrub_ignores_safe_flag": ("redundancy.hpp", "if (!safe_.intact() || safe_.get() > 1U) {", "if (false) {"),
    "scrub_ignores_config": ("redundancy.hpp", "    if (!repair_config()) {", "    if (false && !repair_config()) {"),
    "config_not_restored": ("redundancy.hpp", "      cfg_ = cfg_backup_;\n      cfg_digest_ = cfg_digest_backup_;", "      cfg_digest_ = cfg_digest_backup_;"),
    "queue_length_unchecked": ("redundancy.hpp", "if (!ensure(npending_ <= kMaxCommandsPerFrame, counters_.invariant_violations)) {", "if (false) {"),
}

# Mutants that only the C++ unit tests can see, with the reason: the campaign's peers cannot produce the input that
# distinguishes them from the real code.
CAMPAIGN_SKIP = {
    "config_not_sanitised", "tolerance_not_validated", "persistence_not_validated",  # the peers never present a bad configuration
    "scrub_ignores_node_state", "scrub_ignores_safe_flag", "scrub_ignores_config",   # nor flip a bit of the manager's memory
    "config_not_restored", "queue_length_unchecked",                                  # nor overflow its command queue
    "voter_nan_leaks",          # an int16 sample cannot be NaN or infinite
    "decoder_ignores_len",      # the peers never send a frame of the wrong length
    "counter_decrements",       # a statistics counter is not an input to any decision
    "seq_late_accepts_one_too_many",  # needs a lost frame followed by a duplicate of the last frame (unit test boundary_a_duplicate_*)
}


def build_include(name: str, dest: Path) -> Path:
    """Copy core/include into `dest` with mutation `name` applied; returns the include directory."""
    inc = dest / "include"
    shutil.copytree(ROOT / "core/include", inc)
    fn, old, new = MUTATIONS[name]
    p = inc / "tfc" / fn
    text = p.read_text()
    if old not in text:
        raise RuntimeError(f"mutation {name}: pattern not found in {fn} (the code moved: update tools/mutation/mutations.py)")
    p.write_text(text.replace(old, new, 1))
    return inc


def check_patterns() -> list[str]:
    """Names of mutations whose pattern no longer occurs in the current core."""
    return [n for n, (fn, old, _new) in MUTATIONS.items() if old not in (ROOT / "core/include/tfc" / fn).read_text()]
