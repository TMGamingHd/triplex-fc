# Milestones (target: v1.0 by mid-December 2026)

All hardware is ordered up front; bring-up happens in stages S1-S4 (see `STAGED_BUILD.md`).
Only nodes that have passed their stage are powered and plugged into the bus.

| # | Dates | Goal | Done when |
|---|---|---|---|
| M0 | by Oct 6 | Order all parts. Push repo to GitHub with CI (move `ci/ci.yml` to `.github/workflows/`). Install Zephyr SDK and run hello world on `native_sim`. | CI green; `west build -b native_sim` runs |
| M1 | Oct 6 - 20 | While parts ship: virtual-peer tools on the PC (S1b), `native_sim` frame loop. On arrival: inspect every part, set JP5 on all Nucleos, measure adapter output (< 5.25 V), bench-check relay at 3.3 V drive and servo at 3.3 V signal, measure Nucleo/IMU mounting holes. | Bench checks logged in `docs/BENCH_CHECKS.md`; Nucleo #1 blinks, prints over UART |
| M2 | Oct 20 - Nov 3 | **S1**: single FC-A on real hardware, 100 Hz frame, USB-CAN adapter as peer, IMU over SPI, timing measured. **S2**: ACT node votes (one input) and drives servos. | 10 min at 100 Hz, zero frame errors; WCET/jitter recorded; platform moves from commands |
| M3 | Nov 3 - 17 | **S3** add FC-B (Duplex), **S4** add FC-C (Triplex). Sync master takeover, consensus, ACT 2-of-3 vote. Estimator + controller in `core/`; sim + platform closed loop; Pico fault injector. | Unplug any one node, output unchanged; platform tracks simulated ascent; A/B run (voting on vs off) recorded |
| M4 | Nov 17 - Dec 1 | Automated fault campaign (F01-F18), detection-time histograms, SIL regression in CI, static analysis clean. | Fault matrix fully filled with measured data |
| M5 | Dec 1 - 15 | Write-up, video, README, resume bullets, tag v1.0. | Everything in `PROOF.md` checklist ticked |

Each stage ends with a tagged commit and a short log or video.

Slack is intentionally in M4/M5. If M2 slips, cut stretch goals, not the fault campaign.
If M3 slips, ship a tested Duplex demo and say so honestly.
