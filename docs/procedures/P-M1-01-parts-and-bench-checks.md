# P-M1-01: parts inspection and bench checks

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. Done once, when the parts arrive (milestone M1), before any node is powered from the rig.

| Field | Entry |
|---|---|
| Procedure ID | P-M1-01 |
| Requirements verified | none directly; closes the "to confirm on the bench" items of the parts sheet (notes 11) and its compatibility audit (items 1, 2, 7, 9, 11, 14, 15, 16, 22, 23) |
| Fault-matrix rows | none |
| Firmware under test | none (no firmware is needed except step 4, which uses the ST-LINK only) |
| Hardware configuration | parts on the bench, nothing wired together yet |
| Tools | multimeter, `tools/bench/check_pc.sh`, a ruler or calipers, a phone for photos |

## 1. Description
Establish, before wiring, that each part is the part ordered, that nothing exceeds a rating (above all the 5.25 V limit on the Nucleo's E5V input), and that
five open questions have a measured answer: how the relay behaves at 3.3 V drive, whether the servo follows a 3.3 V signal and what it does with no signal,
the mounting-hole dimensions, and the IMU board's logic level when it is powered on its own. **Pass** = every step below records a value inside its limit.

## 2. Initial set-up
`tools/bench/check_pc.sh` reports no `[MISS]` (install `can-utils` and anything else it names). Nothing powered. A sheet of paper to write the values on.

## 3. Steps

| Step | Who | Action (exact) | Expected | Actual (as run) | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | Unpack every part and tick it against `Triplex_Flight_Computer_Parts_v4.xlsx` (4 Nucleos, 4 CAN Pals, 3 IMUs, 1 Pico 2, USB-CAN adapter, 2 relay modules, 2 servos, hub, cables); photograph any that differ | Every row present and as listed | | |
| 2 | Operator | On each Nucleo, move jumper JP5 to select **E5V** (pins 5-6 per UM2505, audit item 1); photograph | 4 of 4 moved | | |
| 3 | Operator | Plug each 5 V adapter in with nothing connected, measure the output with the multimeter; then with a 100 ohm load (50 mA) | Both below 5.25 V, above 4.75 V (the E5V window), at no load and at 50 mA | | |
| 4 | Operator | Plug each USB cable and each device (4 Nucleos, Pico, adapter) into the hub in turn and run `lsusb` | Each device appears with its own cable (a charge-only cable shows nothing) | | |
| 5 | Operator | Plug in the USB-CAN adapter, then `tools/bench/can_up.sh`; then `python3 -m tfc_peers listen --iface can0 --duration 3` with nothing else on the bus | `can0` is up at 1 Mbit/s; no frames, no error. A warning that the adapter has no automatic bus-off restart is expected on the SH-C31A (record it) | | |
| 6 | Operator | Measure the adapter's own supply output (if it exposes one) | Below 5.25 V (audit item 16) | | |
| 7 | Operator | Relay module, jumper VCC/JD-VCC removed, VCC from a 3.3 V source (a Pico 3V3 pin), JD-VCC from 5 V; for each of the 8 channels pull its input to ground through the Pico and listen | Each coil pulls in at 3.3 V drive (audit item 7); contacts measure closed with the meter | | |
| 8 | Operator | Same module with an input **left floating** (nothing driving it) | The relay stays released. If it pulls in, the supervisor's `PWR` pins need the external pull-ups of `docs/design/SUPERVISOR.md` section 4 | | |
| 9 | Operator | Servo: Pico PWM at 3.3 V, 50 Hz, 1.5 ms pulse, servo on its own 5 V rail (never from a Nucleo or the Pico) | The servo holds position and follows 1.0 and 2.0 ms pulses without twitching (audit item 11); if it twitches, order the level-shifter row | | |
| 10 | Operator | Same servo, then **stop the signal** (disconnect the signal wire) with the servo loaded lightly | Record: holds its last position, or goes limp. This answers `FAULT_RESPONSE.md` open question 2 (what ACT's output does on a reset) | | |
| 11 | Operator | Measure one Nucleo's mounting holes (count, diameter, spacing) and one IMU board's | Recorded for the rack and mounts (audit items 22, 23) | | |
| 12 | Operator | Power an IMU board from 3.3 V (a Pico 3V3 pin), then separately from 5 V; measure the voltage on its data-out (MISO) pin when it is driven high | Record the high level in each case. If it is 5 V when powered from 5 V, power the IMU boards from 3.3 V only. This sets the independent-power plan (TFC-ARCH-010) | | |
| 13 | Operator | Check each pin of the table in `firmware/README.md` against the Nucleo's user manual UM2505 and the board itself: free, not tied to a solder bridge, not on the ST-LINK | Every pin confirmed, or the table corrected | | |

## 4. Shutdown
Everything unplugged. Values copied into `docs/procedures/` as the as-run copy of this file, and into the parts sheet's audit table.

## 5. Records
This file, as run; the photographs; any change to `firmware/app/boards/nucleo_g474re.overlay` or `firmware/README.md` that step 13 requires.
