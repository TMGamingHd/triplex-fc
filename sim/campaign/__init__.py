# SPDX-License-Identifier: MIT
"""Systematic fault campaign: sweep every fault kind over parameters, target nodes, start times and redundancy
contexts, run each scenario through the real C++ core (tfc_replay) and check safety properties on the result.
See docs/FAULT_CAMPAIGN.md."""
