# Milestones (target: v1.0 by mid-December 2026)

All hardware is ordered up front; bring-up happens in stages S1-S4 (see `STAGED_BUILD.md`).
Only nodes that have passed their stage are powered and plugged into the bus.

| # | Dates | Goal | Done when |
|---|---|---|---|
| M0 | by Oct 6 | Order all parts. Push repo to GitHub with CI (move `ci/ci.yml` to `.github/workflows/`). Install Zephyr SDK and run hello world on `native_sim`. | CI green; `west build -b native_sim` runs |
| M1 | Oct 6 - 20 | While parts ship: virtual-peer tools on the PC (S1b), `native_sim` frame loop. On arrival: inspect every part, set JP5 on all Nucleos, measure adapter output (< 5.25 V), bench-check relay at 3.3 V drive and servo at 3.3 V signal, measure Nucleo/IMU mounting holes. | Bench checks logged in `docs/BENCH_CHECKS.md`; Nucleo #1 blinks, prints over UART |
| M2 | Oct 20 - Nov 3 | **S1**: single FC-A on real hardware, 100 Hz frame, USB-CAN adapter as peer, IMU over SPI, timing measured. **S2**: ACT node votes (one input) and drives servos. | 10 min at 100 Hz, zero frame errors; WCET/jitter recorded; platform moves from commands |
| M3 | Nov 3 - 17 | **S3** add FC-B (Duplex), **S4** add FC-C (Triplex). Sync master takeover, consensus, ACT 2-of-3 vote. Estimator + controller in `core/`; sim + platform closed loop; Pico fault injector. | Unplug any one node, output unchanged; platform tracks simulated ascent; A/B run (voting on vs off) recorded |
| M4 | Nov 17 - Dec 1 | Automated fault campaign (F01-F18), detection-time histograms, SIL regression in CI, static analysis clean. **The software-in-the-loop part of this already exists** (`docs/FAULT_CAMPAIGN.md`: 32 fault kinds, 10,713 scenarios, in CI); M4 is the hardware-in-the-loop repeat of it on the rig, plus timing. | Fault matrix fully filled with measured data |
| M5 | Dec 1 - 15 | Write-up, video, README, resume bullets, tag v1.0. | Everything in `PROOF.md` checklist ticked |

**Decision due before the order closes (6 Oct, M0): the supervisor hardware.** `docs/SUPERVISOR.md` proposes one more Pico 2 (6.00 USD on the
parts sheet) and a TCXO clock module (price not checked) for the Lite build; the Full build (a Nucleo and a CAN Pal in place of the Pico)
adds 18.08 USD. The relay channels, wire, headers and USB cables already in the list cover the rest; the 1 k to 10 k resistors need
checking. The parts sheet's own budget page reads 677.81 USD against a 600 USD ceiling before this change. A second ACT (stretch,
`docs/DEFERRED.md` 6.6) would add about 24 USD plus a selector chip and need not be bought now.

## Next up (as of 4 Oct 2026, after the owner's decisions)

1. **The order (not placed yet; the owner will place it soon).** Add to it: a second Pico 2 and a TCXO clock module for the supervisor (SUP-Lite), 1 k to 10 k resistors for pull-ups and pull-downs, and the per-IMU power parts of ADR-020; confirm the total against the budget (the sheet is over its ceiling).
2. **Done and merged:** the decision records, P0 (the driver logic, the firmware seams and overlay, the bench tools and procedures), the CI trim, and the first increment of P1 (the estimator, controller and consensus on the host).
3. **P0 on this PC:** `can-utils` is installed; `tools/bench/check_pc.sh` should report nothing missing.
4. **P1, the loop, continues:** wire it into the firmware's frame, the vehicle-simulator gateway, the actuator node, protocol v2: the critical path to S2; see `docs/SOFTWARE_READINESS.md`.
5. **After the loop and before S3 (3 Nov):** the sensor and compute health split (ARCH-001), with TS-15 choosing its degradation rule.
6. **On the simulator in parallel:** the `common_mode` campaign group (F63), the `noop` command (FDIR-043), TS-1 and TS-3 (TS-3 also decides whether the old release may take over automatically).
7. **M1 hardware checks** when the parts arrive (P-M1-01), then S1 (P-S1-01); S2 to M4 as planned, with the supervisor (Lite) at S2b and the studies that need the rig after S3.

Each stage ends with a tagged commit and a short log or video.

Slack is intentionally in M4/M5. If M2 slips, cut stretch goals, not the fault campaign.
If M3 slips, ship a tested Duplex demo and say so honestly.
