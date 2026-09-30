# Design decisions (ADR log)

## ADR-001: C++17 subset for flight logic
**Decision:** Portable `core/` in C++17, header-only, no heap, no exceptions, no RTTI, warnings as errors.
**Why:** SpaceX's public Starship software postings ask for C++, Rust or another systems language and real-time embedded experience; public writing about its flight software describes C and C++. A restricted C++ subset is the industry pattern for flight code (MISRA/JSF-style rules). **Alternatives:** C (simpler to certify, less expressive), Rust (attractive, but a weaker toolchain fit for Zephyr today). **Revisit:** Rust for a non-critical tool.

## ADR-002: Zephyr RTOS on the target
**Decision:** Zephyr, with the portable core tested on `native_sim`.
**Why:** Modern, upstream, actively maintained, first-class support for the Nucleo-G474RE (FDCAN, SPI, PWM, watchdogs) and an ISM330DHCX sensor binding, plus real space-industry interest (for example NASA cFS being ported to Zephyr by a space-electronics company). Its `native_sim` target lets the same firmware run on a PC for CI. **Honest note:** SpaceX's own stack is not public and what has been described publicly is dated (Linux and VxWorks have been mentioned). No one will hire on RTOS brand; what transfers is deterministic scheduling, priority discipline, timing measurement and testing. **Alternatives:** FreeRTOS (ubiquitous, lighter), bare metal (max determinism, fewer marketable skills). **Fallback:** if Zephyr bring-up stalls beyond milestone M1 by 2 weeks, drop to FreeRTOS; `core/` is unaffected.

## ADR-003: Time-triggered schedule at 100 Hz
**Why:** Predictable, measurable, easy to reason about for voting, and it mirrors how safety-critical buses are scheduled. **Alternative:** event-driven; simpler but jitter is harder to bound.

## ADR-004: Classic CAN 1 Mbit/s for v1
**Why:** The USB-CAN adapter's FD support with default firmware is unconfirmed; 8-byte frames fit our messages. CAN FD is a stretch goal that does not change `core/`.

## ADR-005: Real IMUs on a servo motion platform driven by a simulator
**Why:** Gives real sensor noise and latency in the loop (strong hardware-in-the-loop story) while keeping vehicle physics repeatable. **Cost:** servo bandwidth limits platform rates, so the simulated vehicle is time-scaled. **Alternative:** pure PC-injected sensor data (easier, weaker as a portfolio piece). Keep the injection mode as well, since CI needs it.

## ADR-006: Deterministic identical replicas with an estimator-state digest
**Why:** Bit-identical replicas make exact cross-checking possible and make silent divergence detectable. **Limit:** no protection against a shared software bug (see limitations).
