# Triplex Flight Computer

A desk-scale fault-tolerant flight computer: three redundant STM32 flight computers vote through a CAN bus to an actuator node, with real IMUs on a servo motion platform driven by a vehicle simulator. Faults are injected on purpose and measured.

**Status:** design and portable core done and tested; firmware, simulator and rig are not started. See [docs/MILESTONES.md](docs/MILESTONES.md).

| Area | Status |
|---|---|
| Architecture / requirements / fault matrix | Draft v0.1, in `docs/` |
| `core/` voter, FDIR, protocol (C++17, header-only, no heap) | Done, 31 host tests passing under ASan+UBSan |
| Firmware (Zephyr, Nucleo-G474RE) | Not started |
| Simulator + motion platform | Not started |
| Hardware fault campaign | Not started |

## Build and test (host)
```bash
cmake -S . -B build -G Ninja -DTFC_SANITIZE=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## CI
The workflow is in `ci/ci.yml` (the tooling that copied this repo cannot write into `.github/`). When you create the GitHub repo, move it to `.github/workflows/ci.yml`. It has not been run on GitHub yet; the cppcheck step in particular is untested.

## Docs
[Architecture](docs/ARCHITECTURE.md) - [Requirements](docs/REQUIREMENTS.md) - [Fault matrix](docs/FAULT_MATRIX.md) - [Decisions](docs/DECISIONS.md) - [Milestones](docs/MILESTONES.md) - [What to publish](docs/PROOF.md)

SpaceX-related statements are from public material or inference, not insider knowledge. This is an educational project, not flight-qualified hardware.
