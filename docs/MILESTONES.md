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

## Next up (as of 4 Oct 2026)

In order. The first three have dates; the rest can overlap.

0. **Software readiness first** (`docs/SOFTWARE_READINESS.md`): the audit shows the software is not complete; P0 (S1 on day one) is the target before the parts arrive, P1 (the loop for S2) a stretch.
1. **By 5 Oct: the order.** Supervisor Lite or Full (TS-0 in `TRADE_STUDIES.md`), the clock module, the per-IMU power parts of ADR-020, and the budget cuts that make the total fit. The order closes 6 Oct.
2. **Merge PR 15** (ADR-020 rework) and answer the open questions: `SAFE_MODE.md` section 10, `FAULT_RESPONSE.md` section 6, and the hold-and-ask rule of ADR-021.
3. **M1 hardware checks** after the parts arrive (JP5, adapter voltage, relay at 3.3 V, servo at 3.3 V, mounting holes), plus two new ones: what the servo does with no signal, and the logic level and independent power of the IMU board.
4. **On the simulator, in parallel:** the `common_mode` campaign group (F63) that shows the voter's weakness; the `noop` command (FDIR-043); the sensor and compute health split (ARCH-001) as its own PR; a three-process `native_sim` triplex on one `vcan0` for mixed releases.
5. **Trade studies on the simulator:** TS-1, TS-3, TS-6 first, then TS-2 and TS-4.
6. **Decide the CI cost** (cancel stale runs, skip the campaign for docs-only changes, fast subset on pull requests): about 17 minutes per push today.
7. **M2 to M4** as planned, with the supervisor at S2b and the studies that need the rig (TS-5, TS-8, TS-9) after S3.

Each stage ends with a tagged commit and a short log or video.

Slack is intentionally in M4/M5. If M2 slips, cut stretch goals, not the fault campaign.
If M3 slips, ship a tested Duplex demo and say so honestly.
