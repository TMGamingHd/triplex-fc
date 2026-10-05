# SPDX-License-Identifier: MIT
"""Scenario and result model, plus the expected outcome of a fault derived from its parameters."""
from __future__ import annotations

from dataclasses import dataclass, field

from tfc_peers.faults import Fault, parse_fault

TOL = [1.0, 1.0, 1.0, 0.02, 0.02, 0.02, 0.01, 0.01]  # vote tolerance per channel (core RedundancyConfig::tol)
INF = 10**9


@dataclass
class Scenario:
    group: str                      # e.g. "bias"
    faults: list[str]               # fault specs, as on the CLI
    commands: list[str] = field(default_factory=list)
    nodes: str = "A,B,C"
    frames: int = 500
    seed: int = 1
    policy: str = "manual"
    release: str = ""              # the releases the computers report, `--release` (e.g. "A=1,B=1,C=2"; empty: not reported)
    split: bool = False             # the manager judges each IMU channel apart from its computer (ADR-020 case 1, tfc_replay --sensor-split)
    context: str = "triplex"        # triplex | duplex-X | simplex-XY (the named nodes are dropped at frame 5)
    rest: bool = False              # vehicle at rest (constant truth)
    expect: str = "any"             # detect | ignore | gray | any   (for the *single* fault under test)
    latency_max: int | None = None  # for detect: latest acceptable latch frame offset from fault start
    tag: dict = field(default_factory=dict)

    def all_fault_specs(self) -> list[str]:
        specs = list(self.faults)
        if self.context.startswith("duplex-"):
            specs.append(f"{self.context.split('-')[1]}:dropout:start=5")
        elif self.context.startswith("simplex-"):
            for ch in self.context.split("-")[1]:
                specs.append(f"{ch}:dropout:start=5")
        return specs

    def key(self) -> str:
        return f"{self.group}|{','.join(self.faults)}|{','.join(self.commands)}|{self.context}|s{self.seed}|f{self.frames}|{self.policy}|rest={self.rest}" + ("|split" if self.split else "") + (f"|rel={self.release}" if self.release else "")


@dataclass
class Result:
    scenario: Scenario
    ok: bool = True
    error: str = ""
    latch: dict = field(default_factory=dict)       # node -> first latch frame
    reason_at_latch: dict = field(default_factory=dict)
    final_state: dict = field(default_factory=dict)
    strikes: dict = field(default_factory=dict)
    final_mode: str = ""
    counters: dict = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    anomalies: list = field(default_factory=list)   # (code, detail)


def parsed_faults(sc: Scenario) -> list[Fault]:
    return [parse_fault(f) for f in sc.faults]


def faulty_nodes(sc: Scenario) -> set[int]:
    return {f.node for f in parsed_faults(sc)}
