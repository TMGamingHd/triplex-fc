# Triplex Flight Computer

A desk-scale fault-tolerant flight computer: three redundant STM32 flight computers vote through a CAN bus to an actuator node, with real IMUs on a servo motion platform driven by a vehicle simulator. Faults are injected on purpose and measured.

**Status:** design and portable core done and tested; firmware, simulator and rig are not started. See [docs/MILESTONES.md](docs/MILESTONES.md).

| Area | Status |
|---|---|
| Architecture / requirements / fault matrix | Draft v0.1, in `docs/` |
| `core/` voter, FDIR, protocol (C++17, header-only, no heap) | Done, 31 host tests passing under ASan+UBSan |
| Firmware (Zephyr, Nucleo-G474RE) | M0 hello world builds on `native_sim` and `nucleo_g474re`; see [firmware/README.md](firmware/README.md) |
| Virtual peers (S1b): fake FC-B/C with fault injection, replayed through `core/` | Done (host and vcan); see [sim/README.md](sim/README.md) |
| Simulator + motion platform | Not started |
| Hardware fault campaign | Not started |

## Build and test (host)
```bash
cmake -S . -B build -G Ninja -DTFC_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## CI
The workflow is `.github/workflows/ci.yml` (runs on pull requests and on `main`). It has not been run on GitHub yet; the cppcheck step in particular is untested.

## Contributing
This repo uses GitHub Flow: `main` is always green, all work happens on short-lived branches and lands by squash-merged PR. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Docs
[Architecture](docs/ARCHITECTURE.md) - [Requirements](docs/REQUIREMENTS.md) - [Fault matrix](docs/FAULT_MATRIX.md) - [Decisions](docs/DECISIONS.md) - [Milestones](docs/MILESTONES.md) - [What to publish](docs/PROOF.md)

SpaceX-related statements are from public material or inference, not insider knowledge. This is an educational project, not flight-qualified hardware.
