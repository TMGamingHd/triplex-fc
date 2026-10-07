# Coding standard for the flight core

> Status: **in force** (reviewed 5 Oct 2026) for everything under `core/` (the portable flight logic) and the firmware's use of it. Sources: JPL
> "The Power of Ten: Rules for Developing Safety-Critical Code" (G. Holzmann, 2006, used on JPL flight software),
> NASA-STD-8719.13 (software safety) and NPR 7150.2 (software engineering requirements), the MISRA C++ and CERT C++
> principles that the clang-tidy `cert-*` and `bugprone-*` families implement, and DO-178C structural-coverage practice.
> "Enforced by" names the tool that fails the build; "by inspection" means a person has to check it. Where a rule is not
> met in full the deviation is listed in section 4 with the reason, rather than claimed.

## 1. The ten rules and how the core meets them

| # | Rule (Power of Ten) | How `core/` meets it | Enforced by |
|---|---|---|---|
| 1 | Simple control flow: no `goto`, `setjmp`/`longjmp`, no recursion | None used. The deepest call chain is `end_frame` -> `advance_life_cycle` -> `judge_probation` -> `cohort_verdict`; every function is non-recursive and stack use is static | clang-tidy `misc-no-recursion`, `cppcoreguidelines-avoid-goto`; `tools/check_standard.py`; GCC `-Wstack-usage=2048` (every function) and `-fstack-usage` (reports `static`) |
| 2 | Every loop has a fixed upper bound | Every loop is a `for` over a compile-time constant: nodes (3), streams (3), vote channels (8), command queue (4), bits (16/32), or the caller's buffer size. There is no `while` and no `do` in the core | `tools/check_standard.py` rejects `while`/`do`; `bugprone-infinite-loop` |
| 3 | No dynamic memory after initialisation | No heap at all: no `new`, `malloc`, containers or strings. All state lives inside the `RedundancyManager` object (1,080 bytes) | `tools/check_standard.py`; `tools/check_elf.sh` scans the ARM binary for `malloc`, `operator new`, exception, RTTI and vtable symbols; the firmware sets `CONFIG_COMMON_LIBC_MALLOC=n`, so an accidental use is a *link error* |
| 4 | No function longer than about 60 lines | The longest core function is under 60 lines. `vote3` and `judge_nodes`, which were not, were split | clang-tidy `readability-function-size` (60 lines, 80 statements, 20 branches, 4 nesting, 6 parameters) and `readability-function-cognitive-complexity` (25), as errors |
| 5 | At least two assertions per function on average | **Partly met, by design**: see section 4. Runtime invariants are checked where a corrupted value or a coding error would change a *safety decision*: configuration on entry (`validate_config`), node states, the Safe flag, the configuration copy and the command-queue length every frame (`scrub`), every external frame (`decode`: length, CRC), every public node index. Compile-time: four `static_assert`s on the table sizes. None aborts: a failure is counted, repaired on the safe side, and reported (`ensure`, `integrity_faults`) | unit tests that inject each upset (`seu_*`, `config_*`); mutation testing removes each check and the tests notice |
| 6 | Declare data at the smallest scope | No namespace-scope variables: the compiled core has no writable data or bss symbol of its own | `tools/check_standard.py` compiles the core and fails on any `b`/`d` symbol; `-Wshadow` |
| 7 | Check every return value and every parameter | Pure functions and accessors are `[[nodiscard]]`; commands return a `CommandResult` that the report records; a node index is range-checked before use; configuration is validated | `-Wunused-result` / `[[nodiscard]]`, `bugprone-unused-return-value`, `cert-err33-c`; unit tests `coverage_queries_about_a_node_that_does_not_exist_*`, `config_*` |
| 8 | Limit the preprocessor | `#include` and `#pragma once` only: no macro, no conditional compilation in the core | `tools/check_standard.py` |
| 9 | Restrict pointers | No pointer arithmetic, no function pointers, no `reinterpret_cast`/`const_cast`. Raw pointers appear only where a buffer is passed (`crc8`, `fnv1a`, `format_reasons`, `memcpy` of a 6-byte payload), each with its length | `tools/check_standard.py`, `-Wold-style-cast -Wcast-qual -Wcast-align` |
| 10 | Compile with all warnings, zero warnings, analyse every build | The strict gate below is part of the default build; warnings are errors; three static analysers and two sanitizers run in CI | CMake target `tfc_strict_check`; clang-tidy; cppcheck; ASan + UBSan |

## 2. Beyond the ten rules: what a flight-computer core also needs

| Concern | What the core does | Evidence |
|---|---|---|
| **Determinism** | Same inputs give the same outputs, bit for bit, on any platform: no `-ffast-math`, no uninitialised reads, no undefined behaviour, no float equality (`-Wfloat-equal`), the consensus-critical leaky count is Q16 fixed point, the CRC and digest are integer | per-frame decision hash of every scenario (12,362) identical before and after a refactor (`campaign.run`, `docs/verification/FAULT_CAMPAIGN.md`) |
| **Fail to the safe side** | Every unresolved doubt resolves to "exclude the node", "hold the last good value" or "request Safe", never to "trust it". A Safe flag or node state that fails its integrity check reads as the restrictive value | `seu_*` tests; fuzz invariants I1-I8 |
| **Validate at the boundary** | Frames: length, CRC, sequence. Configuration: every field range-checked and replaced by its default if invalid, and the fact reported. Faults for the test rig: every parameter validated | `decoders_reject_every_malformed_frame`, `config_every_invalid_field_*`, `tests/test_faults_fmea.py` |
| **Protect critical state against upsets (SEU)** | Node states and the Safe flag are stored with their bitwise complement; the configuration is stored twice with a checksum; all are scrubbed at the start of every frame. An upset is repaired on the safe side, counted and reported (ADR-015) | `seu_*` tests; mutants `scrub_ignores_*` |
| **Bounded, measured cost** | One frame of the manager costs about 0.5 us on a desktop CPU (section 5); worst-case on the target is to be measured with the cycle counter (milestone M2) | `tools/bench/bench_end_frame.cpp` |
| **Single thread, no shared state** | The core is a plain object driven from one task; there are no atomics, locks or callbacks | by construction |
| **Traceability** | Every requirement maps to a test or a measurement (`docs/verification/REQUIREMENTS.md`); every fault to a matrix row (`docs/verification/FAULT_MATRIX.md`); every design decision to an ADR (`docs/decisions/DECISIONS.md`) | `TFC-SW-005` |

## 3. The verification stack

| Layer | What it proves | Command | In CI |
|---|---|---|---|
| Strict build | The whole core, and each header on its own, compile with the strictest warning set, as errors | `cmake --build build` (target `tfc_strict_check`) | every PR |
| Unit, property and fuzz tests (C++) | Functional behaviour, boundaries, the properties of the voter and monitors under random input, the manager's invariants under random fault processes, SEU repair | `ctest` / `build/host/tfc_tests` (ASan + UBSan) | every PR |
| clang-tidy / cppcheck | Bug patterns, CERT rules, analyzer paths, the size and complexity limits | `clang-tidy -p build tests/*.cpp tools/replay/*.cpp`, `cppcheck ...` | every PR |
| Mechanical rules | No `goto`/`while`/heap/macros/exceptions/RTTI/globals in the core | `python3 tools/check_standard.py` | every PR |
| Target binary scan | The ARM binary has no heap, exception, RTTI or vtable symbols | `tools/check_elf.sh build/nucleo_g474re/zephyr/zephyr.elf` | every PR |
| Structural coverage | Which lines and branches of the core the tests execute | `python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98` | every PR |
| Fault campaign | Safety properties on every frame of 12,362 scenarios covering all 32 fault kinds, the sensor split and mixed releases | `python3 -m campaign.run --strict` | every PR |
| Mutation: unit tests | 390 deliberate bugs in the core and the supervisor; every one must be caught by the C++ tests (the equivalent ones are listed in the file with the reason) | `python3 tools/mutation/run_unit.py` | weekly |
| Mutation: campaign | The same bugs against the campaign's oracles | `python3 -m campaign.mutate` | weekly |
| Structural coverage: simulator | Which lines and branches of `sim/vehicle` the tests execute (a model, so the gate is lower than the flight code's: it holds guards that no test can reach, such as a non-finite state) | `python3 tools/coverage/core_coverage.py --sim-min-line 99 --sim-min-branch 90` | every PR |
| Mutation: simulator | 257 simulator mutants: deliberate bugs in the vehicle dynamics, atmosphere, ground, platform, IMU model, runner and the design of the flight tables; each must be killed by the C++ tests (the one equivalent mutant is listed with the reason) | `python3 tools/mutation/run_sim.py` | weekly |

Mutation testing is the check on the checks: a test suite that cannot tell a deliberately broken core from the real one is not
testing that behaviour. It found real gaps in this project (a Safe flag that was not sticky, a probation that could not fail, a
bus-alarm boundary nobody had tested), each now closed.

## 4. Deviations, and why

| Rule | Deviation | Reason | Mitigation |
|---|---|---|---|
| 5 (two assertions per function) | Assertions are placed at safety-decision points, not in every function | A check that cannot fail in any reachable state is dead code, and dead code cannot be covered or mutation-tested. Many functions here are pure and total (every input maps to an output) | checks at every external boundary; a periodic scrub of the state that matters; `static_assert` for the tables |
| 1 / 3 (stack, heap) | `std::array`, `<cmath>` and `<cstring>` come from the C++ library | They are header-only use of fixed-size types and `memcpy`/`fabs`, with no allocation | the ELF scan proves no heap or exception machinery is linked |
| 9 (pointers) | `crc8`, `fnv1a`, `format_reasons` and the payload `memcpy` take pointers | They are the interfaces of byte-oriented operations | each takes its length; covered by the fuzz and decoder tests |
| Branch coverage 100% | Line coverage is 100 % (3,490 lines of the core and the supervisor); branch coverage is 98.4 % (2,707 of 2,752 compiler branches) | The untaken branches are sub-conditions of compound tests that cannot be separated (for example `i < npending_ && i < kMaxCommandsPerFrame`, whose second half is a guard that `scrub` makes redundant) and the compiler's own short-circuit edges; each is listed by `core_coverage.py --list` | the gate is 100 % of lines and 98 % of branches and may only rise |
| Worst-case execution time | Measured on the host only | The target is not here yet | `tools/bench` runs on the board in milestone M2 using the DWT cycle counter |

## 5. Measured cost

Host (desktop CPU, `-O2`, 400,000 frames each; 9 `on_frame` calls plus `end_frame`):

| Case | median | p99 | p99.9 |
|---|---|---|---|
| healthy triplex | 0.50 us | 0.80 us | 1.02 us |
| node biased, latched (duplex) | 0.47 us | 0.58 us | 0.88 us |
| node silent (duplex) | 0.43 us | 0.45 us | 0.72 us |

The worst single sample in each run is 17-47 us and is the operating system preempting the benchmark, not the code: there is
no allocation, no loop over unbounded data and no I/O in the path. A frame is 10,000,000 ns.

Object sizes: `RedundancyManager` 1,432 bytes, `FrameReport` 176, `RedundancyConfig` 112, `Counters` 120, the supervisor's `Supervisor` 1,104.
Stack: the strict build bounds every function of the core at 2 KB (`-Wstack-usage=2048`, which is why the check harness splits its exercise into functions); the deepest library function on the host (x86-64, `-Os`, `-fstack-usage`) is `RedundancyManager::scrub` at 320 bytes, all statically known. The Cortex-M4F figure is read
from the target build at S1. Firmware sizes (`nucleo_g474re`, 5 Oct 2026): the flight computer 59.1 KB of its 512 KB flash and 10.0 KB of 128 KB RAM (60.5 KB and 11.1 KB with the flight function), the actuator node 39.5 KB and 8.9 KB; the Pico 58.8 KB and the supervisor 69.1 KB of 4 MB.

These are *host* and *size* figures. A desktop executes this code perhaps a hundred times faster than a 170 MHz Cortex-M4,
so the estimate on the target is in the tens of microseconds, a fraction of a percent of the 10 ms frame, to be confirmed
by measurement on the board (TFC-SYS-002).

## 6. Rules for changes

1. A change to `core/` comes with a test that fails without it, written first.
2. `cmake --build build && ctest --test-dir build` must pass with zero warnings.
3. A refactor must leave the campaign's per-frame decision hashes unchanged (`campaign.run`, then compare); a change that
   alters behaviour says so in an ADR and in `docs/verification/FAULT_CAMPAIGN.md`.
4. A new fault kind is a new row in `docs/verification/FAULT_MATRIX.md`, a unit test of what it puts on the wire, a replay test of what the
   core decides, and a group in the campaign.
5. A new check (`ensure`, `static_assert`, validation) is accompanied by a mutant in `tools/mutation/mutations.py` that removes
   it, and the suite must kill the mutant.
