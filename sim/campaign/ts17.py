# SPDX-License-Identifier: MIT
"""TS-17, the paper part: which hardware overrides are worth building (docs/TRADE_STUDIES.md section 9c, docs/HARDWARE_OVERRIDE.md).

Nine software-down scenarios (SD1 to SD9) against the candidate overrides, as a coverage matrix (does the override reach the safe state without any program: fully, partly, not at all); a
model of the probability that the safe state is reached when each override acts, when demanded, with probability r (the chance that it is not dead from a latent fault and is operated
correctly); and the same at a tenth of the reliability. The judgments in the matrix are stated here so that they can be argued with; the arithmetic is mechanical.

    python3 -m campaign.ts17          (prints the three tables)

The model is deliberately simple: overrides are independent and each covers a scenario with a fixed weight (full 1.0, partly 0.5); P(safe) = 1 - product of (1 - weight * r).
"""
from __future__ import annotations

import argparse

SCENARIOS = {
    "SD1": "the PC or simulator hangs",
    "SD2": "the Pico platform driver hangs or runs away",
    "SD3": "ACT hangs or votes wrong",
    "SD4": "all flight computers wrong the same way (a common bug)",
    "SD5": "the supervisor resets a healthy node again and again",
    "SD6": "the injector's relay stays on (a crashed Pico, a failed driver)",
    "SD7": "a node babbles on the bus or hangs it",
    "SD8": "a servo stalls or runs to its stop",
    "SD9": "the node rail droops (relay coils, a failing adapter)",
}

# What each override is (HARDWARE_OVERRIDE.md section 3). "F" is the set of free manual controls the parts list already has (the Nucleos' reset buttons, the unpluggable CAN stubs, the wall
# adapters' plugs).
OVERRIDES = {
    "H1": "servo-rail E-stop (exists)",
    "F": "free manual controls (reset buttons, unpluggable stubs, adapter plugs; exist)",
    "H2": "FORCE-SAFE toggle to ACT",
    "H3": "PLATFORM-LEVEL switch (neutral servo signal from a no-code source)",
    "H4": "INJECTOR-DISARM in the injector relays' coil supply",
    "H5": "SUPERVISOR-DISARM in the supervisor relays' coil supply",
    "H6": "per-node POWER-KILL (four)",
    "H7": "MASTER-POWER in the node rail feed",
}

# Coverage weight: 1.0 reaches the safe state without a program, 0.5 reaches it in part (or only if another part works), absent: no.
COVERS: dict[str, dict[str, float]] = {
    "SD1": {"H1": 1.0, "H3": 1.0},
    "SD2": {"H1": 1.0, "H3": 1.0},
    "SD3": {"H1": 1.0, "H3": 1.0, "H2": 0.5},          # H2 needs ACT to be running
    "SD4": {"H1": 1.0, "H3": 1.0, "H2": 1.0},
    "SD5": {"H5": 1.0, "H7": 0.5},
    "SD6": {"H4": 1.0},
    "SD7": {"F": 1.0, "H6": 1.0},
    "SD8": {"H1": 1.0, "H3": 0.5},
    "SD9": {"H4": 0.5, "H5": 0.5, "H7": 0.5},          # remove the coil load; nothing removes a failing adapter
}

# The candidate sets (TS-17 options), cumulative.
OPTIONS = {
    "O0": [],
    "O1": ["H1", "F"],
    "O2": ["H1", "F", "H2", "H3"],
    "O3": ["H1", "F", "H2", "H3", "H4", "H5"],
    "O4": ["H1", "F", "H2", "H3", "H4", "H5", "H6", "H7"],
}

# The harm each override can do by itself (HARDWARE_OVERRIDE.md section 6): 1 = a lost run at worst, 2 = a shock or a cut node, 3 = a hard mechanical stop of the platform.
HARM = {"H1": 3, "F": 1, "H2": 1, "H3": 2, "H4": 1, "H5": 1, "H6": 2, "H7": 2}
# The cost in USD (HARDWARE_OVERRIDE.md section 8; H1 and F exist; none of the new parts is priced on a listing).
COST = {"H1": 0, "F": 0, "H2": 2, "H3": 8, "H4": 1, "H5": 1, "H6": 4, "H7": 1}


def p_safe(option: str, scenario: str, r: float) -> float:
    miss = 1.0
    for h in OPTIONS[option]:
        w = COVERS[scenario].get(h, 0.0)
        miss *= 1.0 - w * r
    return 1.0 - miss


def fully_covered(option: str, scenario: str) -> int:
    """How many independent overrides of the option reach the safe state in full."""
    return sum(1 for h in OPTIONS[option] if COVERS[scenario].get(h, 0.0) >= 1.0)


def smallest_option_covering_every_scenario() -> str | None:
    """The decision rule's first clause: the smallest option in which every scenario has at least one override that needs no program (full cover). SD9 has none in any option."""
    for name in OPTIONS:
        if all(fully_covered(name, s) >= 1 for s in SCENARIOS if s != "SD9"):
            return name
    return None


def cost(option: str) -> int:
    return sum(COST[h] for h in OPTIONS[option])


def tables() -> str:
    out = []
    out.append("| Scenario | " + " | ".join(OVERRIDES) + " |")
    out.append("|---|" + "---|" * len(OVERRIDES))
    for s, text in SCENARIOS.items():
        cells = []
        for h in OVERRIDES:
            w = COVERS[s].get(h, 0.0)
            cells.append("full" if w >= 1.0 else ("part" if w > 0.0 else ""))
        out.append(f"| {s} {text} | " + " | ".join(cells) + " |")
    out.append("")
    out.append("| Option | overrides | new cost (USD, unpriced) | full covers per scenario (SD1..SD9) | scenarios with no full cover |")
    out.append("|---|---|---|---|---|")
    for name, hs in OPTIONS.items():
        counts = [fully_covered(name, s) for s in SCENARIOS]
        none = [s for s, c in zip(SCENARIOS, counts) if c == 0]
        out.append(f"| {name} | {', '.join(hs) or 'none'} | {cost(name)} | {' '.join(str(c) for c in counts)} | {', '.join(none) or '-'} |")
    out.append("")
    for r in (0.95, 0.5):
        out.append(f"P(safe state reached), each override acting with probability r = {r}:")
        out.append("")
        out.append("| Option | " + " | ".join(SCENARIOS) + " | mean |")
        out.append("|---|" + "---|" * (len(SCENARIOS) + 1))
        for name in OPTIONS:
            ps = [p_safe(name, s, r) for s in SCENARIOS]
            out.append(f"| {name} | " + " | ".join(f"{p:.2f}" for p in ps) + f" | {sum(ps) / len(ps):.2f} |")
        out.append("")
    return "\n".join(out)


def main(argv=None) -> int:
    argparse.ArgumentParser(prog="ts17", description=__doc__.split("\n")[0]).parse_args(argv)
    print(tables())
    print(f"smallest option with a full cover for every scenario but SD9: {smallest_option_covering_every_scenario()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
