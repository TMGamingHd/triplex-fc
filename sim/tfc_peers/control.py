# SPDX-License-Identifier: MIT
"""Changing the faults of a running scenario: `tfc_peers run --control` reads one command per line from standard input and answers one line on standard output.

    add SPEC [for N]    add a fault (SPEC as for --fault, e.g. B:bias:mag=3). Without `start=` it starts in the frame the command is applied in (not at frame 0: an intermittent pattern then
                        counts from now); `for N` ends it N frames later. Answers `ok add ID SPEC`, ID being the number `clear` takes.
    clear ID | all      remove one fault or every fault. Answers `ok clear ID` or `ok clear all N`. What the fault already did stays done: a node it got latched out stays out until the operator readmits it.
    list                answers `ok list` and then one line `fault ID SPEC` for each fault now in the scenario
    frame               answers `ok frame N`: the frame number the next command will be applied in (a console uses it to time `end=` and `for`)

An error answers `error: <why>` and changes nothing. Commands are applied between frames, in the run's own thread, so a command never meets a frame half-built.
This is the interface the flight console (docs/design/CONSOLE.md) injects faults through; it is a plain text protocol on purpose, so it can be typed by hand.
"""
from __future__ import annotations

from .faults import Fault, FaultSpecError, parse_fault
from .peers import Scenario


def _rebase(fault: Fault, spec: str, k: int, length: int | None) -> Fault:
    """A fault given without `start=` starts now; `for N` ends it N frames after it starts."""
    if "start=" not in spec:
        fault.start = k
    if length is not None:
        if length < 1:
            raise FaultSpecError("`for` needs a whole number of frames of at least 1")
        fault.end = fault.start + length
    if fault.end is not None and fault.end <= fault.start:
        raise FaultSpecError(f"the fault would end (frame {fault.end}) before it starts (frame {fault.start})")
    return fault


def handle(scenario: Scenario, line: str, k: int) -> list[str]:
    """Apply one control line in frame `k`; returns the lines to answer (always at least one)."""
    words = line.split()
    if not words:
        return []
    op = words[0].lower()
    try:
        if op == "add":
            if len(words) not in (2, 4) or (len(words) == 4 and words[2].lower() != "for"):
                raise FaultSpecError("usage: add SPEC [for FRAMES]")
            length = int(words[3]) if len(words) == 4 else None
            fault = _rebase(parse_fault(words[1]), words[1], k, length)
            number = scenario.add_fault(fault)
            return [f"ok add {number} {fault}"]
        if op == "clear":
            if len(words) != 2:
                raise FaultSpecError("usage: clear ID | clear all")
            if words[1].lower() == "all":
                return [f"ok clear all {scenario.clear_faults()}"]
            if not scenario.clear_fault(int(words[1])):
                raise FaultSpecError(f"there is no fault {words[1]}")
            return [f"ok clear {int(words[1])}"]
        if op == "list":
            return ["ok list", *(f"fault {i} {f}" for i, f in scenario.fault_table())]
        if op == "frame":
            return [f"ok frame {k}"]
        raise FaultSpecError(f"unknown command {op!r}; known: add, clear, list, frame")
    except (FaultSpecError, ValueError) as e:
        return [f"error: {e}"]
