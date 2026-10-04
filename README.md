# Triplex Flight Computer

A desk-scale fault-tolerant flight computer: three redundant STM32 flight computers vote through a CAN bus to an actuator node, with real IMUs on a servo motion platform driven by a vehicle simulator. Faults are injected on purpose and measured.

**Status:** design and portable core done and tested; firmware, simulator and rig are not started. See [docs/MILESTONES.md](docs/MILESTONES.md).

| Area | Status |
|---|---|
| Architecture / requirements / fault matrix | Draft v0.1, in `docs/` |
| `core/` voter, FDIR, protocol, `RedundancyManager` (C++17, header-only, no heap) | Done: 178 host tests under ASan+UBSan, 100% line / 98.6% branch coverage, 72 deliberate bugs all caught, strict warning gate and the flight-code standard enforced ([CODING_STANDARD](docs/CODING_STANDARD.md)) |
| Firmware (Zephyr, Nucleo-G474RE) | FC-A (sync master) runs the 100 Hz loop on `native_sim` against virtual peers on `vcan0`; builds for `nucleo_g474re` (not run on hardware); see [firmware/README.md](firmware/README.md) |
| Virtual peers (S1b): fake FC-B/C with 32 kinds of fault injection, replayed through `core/` | Done (host and vcan); see [sim/README.md](sim/README.md) |
| Fault campaign: every fault kind over its input range, safety properties checked on every frame | 11,263 scenarios, 4.7 million frames, no property violated; edge cases found and fixed: [FAULT_CAMPAIGN](docs/FAULT_CAMPAIGN.md), failure-mode analysis: [FMEA](docs/FMEA.md) |
| Simulator + motion platform | Not started |
| Hardware fault campaign | Not started |

## Build and test (host)
```bash
cmake -S . -B build -G Ninja -DTFC_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## CI
The workflow is `.github/workflows/ci.yml` (runs on pull requests and on `main`): strict build, ASan/UBSan tests, clang-tidy, cppcheck, the coding-standard and coverage gates, and the fault campaign. `.github/workflows/mutation.yml` runs the mutation checks weekly. The whole workflow ran green on GitHub for PR #12 (the `sil` job takes about 16 minutes, most of it the fault campaign on four cores).

## Contributing
This repo uses GitHub Flow: `main` is always green, all work happens on short-lived branches and lands by squash-merged PR. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Docs
[Architecture](docs/ARCHITECTURE.md) - [Requirements](docs/REQUIREMENTS.md) - [Fault matrix](docs/FAULT_MATRIX.md) - [Fault campaign](docs/FAULT_CAMPAIGN.md) - [FMEA](docs/FMEA.md) - [Coding standard](docs/CODING_STANDARD.md) - [Decisions](docs/DECISIONS.md) - [Supervisor](docs/SUPERVISOR.md) - [Fault response](docs/FAULT_RESPONSE.md) - [Control loop](docs/CONTROL_LOOP.md) - [Vehicle simulator](docs/VEHICLE_SIM.md) - [Software readiness](docs/SOFTWARE_READINESS.md) - [Trade studies](docs/TRADE_STUDIES.md) - [Safe mode](docs/SAFE_MODE.md) - [Mission phases](docs/MISSION_PHASES.md) - [Deferred work](docs/DEFERRED.md) - [Milestones](docs/MILESTONES.md) - [What to publish](docs/PROOF.md)

SpaceX-related statements are from public material or inference, not insider knowledge. This is an educational project, not flight-qualified hardware.
