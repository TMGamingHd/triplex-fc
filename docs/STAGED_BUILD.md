# Staged build: one flight computer first, then two, then three

Goal: get one node fully working, then grow to Triplex without rewriting anything.
The rule that makes this possible: **the software always assumes three flight-computer
slots (A, B, C) plus ACT. A missing node is just a node that never sends frames.**

## Why this already works in `core/`
- `vote3(x, valid_mask, tol)` takes a validity mask. One valid channel returns `Simplex`,
  two return `Duplex` or `DuplexMiscompare`, three return `Triplex` or `NoMajority`.
- `mode_from_healthy()` maps 1/2/3 healthy computers to Simplex/Duplex/Triplex.
- Frame IDs are `base + node`, so adding node B or C adds no new message types.

## Design rules (follow from day one)
1. **Same firmware image on every FC.** The node ID (0-2) comes from a build option, later
   from two strap pins. Never hard-code "I am node A" in logic; only in board config.
2. **Absent means invalid.** No frame in a slot this cycle = valid bit cleared. No special
   "single-node mode" code path anywhere.
3. **Sync master = lowest healthy FC.** With one node that is node A, always.
4. **Full backbone from the start.** Build the CAN backbone with all five taps (A, B, C, ACT,
   USB-CAN) and both 120 ohm terminations on day one. Adding a node only means plugging in a stub.
5. **Identical wiring per node.** Same pins for CAN Pal, IMU (SPI), power. Copy the harness.
6. **Printed rack has four slots** even if three are empty.
7. **Digest from the start.** The estimator-state digest is computed and logged even with one
   node; it is checked against a golden run on the PC. Later it is compared across nodes.
8. **CAN needs a second node.** A lone CAN node gets no ACK and goes bus-off. Stage 1 uses the
   USB-CAN adapter as the second node, so it is required from the first day.

## Stages

| Stage | Hardware | What runs | Exit test |
|---|---|---|---|
| S1 Single FC | 1 Nucleo + CAN Pal + IMU, USB-CAN adapter, PC | FC-A runs the full 100 Hz frame in Simplex. PC plays ACT and ground station, logs frames, feeds sim data. | 10 min at 100 Hz, zero frame errors; WCET and jitter measured; digest matches PC golden run |
| S1b Virtual peers | same | PC publishes frames as FC-B and FC-C (replayed or generated data, injected faults) through the USB-CAN adapter | FC-A sees Duplex/Triplex behaviour and FDIR transitions against fake peers; runs in CI on `native_sim` too |
| S2 Add ACT | +1 Nucleo + CAN Pal, servo rail | Real ACT node votes the commands and drives the servos. Motion platform closes the loop. | Platform tracks simulated ascent with one FC |
| S3 Duplex | +FC-B (Nucleo, CAN Pal, IMU) | Two FCs; miscompare is detectable but cannot say who is wrong | Unplug B: back to Simplex without a glitch. Inject a bad value: miscompare flagged |
| S4 Triplex | +FC-C | Full 2-of-3, fault campaign F01-F18 | Fault matrix filled with measured data |

Each stage ends with a tagged commit and a short log/video so the repo shows progress.

## What to buy first (about $200)
2 Nucleos, 2 CAN Pals, 1 IMU, USB-CAN adapter, Cat6 + terminal blocks + resistors, wiring
group, 1 x 5 V adapter with jack, Micro-USB cables, M2/M3 hardware. Later: nodes B/C
(2 Nucleos, 2 CAN Pals, 2 IMUs), servos and servo rail, Pico + relays, hub, logic analyzer.
Cost of staging: extra shipping and some risk of stock or price changes (servos especially).

## Honest limits
- One node cannot show voting. The resume value of the project appears at S3/S4, so protect
  that schedule. S1 alone is still a real deliverable (deterministic 100 Hz loop, measured
  timing, logged data).
- Virtual peers test the logic but not real bus timing or wiring faults; the real nodes still
  have to prove those.
- If time runs short, a working Duplex demo with the fault campaign that applies is better than
  an untested Triplex.
