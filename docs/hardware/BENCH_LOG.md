# Bench log

> Status: **in use** since 5 Oct 2026 (the first parts arrived early; the main shipment is due 9 Oct 2026). The as-run record of every bench check and measurement: one dated entry each, with what was measured, the number, the limit it is compared with and a verdict. Procedures say *what to do*
> (`docs/procedures/`); this page is *what happened*. An entry that fails is as important as one that passes: keep it, and say what was done about it.

Copy this block for each entry (newest at the bottom).

```
### YYYY-MM-DD  <procedure id and step, e.g. P-M1-01 step 4>   <who>
Item: <part or node, with its serial or label>
Measured: <value and unit, how (meter, analyzer channel, console line)>
Limit: <the requirement or the datasheet figure>   Verdict: pass | fail | n/a
Notes: <set-up, deviations from the procedure, what was done about a fail; the file name of any capture>
```

## Entries
### 2026-10-05  P-M1-01 step 5 (first attempt)   T. Nardelli
Item: USB-CAN adapter SH-C31A (DSD TECH, Cannable 2.0), serial 005A00214546500220383145
Measured: enumerated as 1d50:606f "SH-C31x" (gs_usb), then re-enumerated as 0483:df11 "DFU in FS Mode" (STMicroelectronics bootloader); no can0 in DFU mode
Limit: appears as a gs_usb device with can0   Verdict: fail (then pass: see the next entry)
Notes: the adapter's BOOT setting was on. With BOOT off it enumerated as 1d50:606f. Nothing was flashed. The green-then-red light on plugging in is its normal start-up with BOOT off.

### 2026-10-05  P-M1-01 step 5 (can_up.sh)   T. Nardelli
Item: USB-CAN adapter SH-C31A on can0
Measured: `tools/bench/can_up.sh` (old version) failed with "Device doesn't support restart from Bus Off": the driver refuses `restart-ms`, so the whole command failed and can0 was not configured. Manual `ip link set can0 down`, `... type can bitrate 1000000`, `... up` worked. `ip -details link show can0`: state UP, ERROR-ACTIVE, bitrate 1000000, sample point 0.747, restart-ms 0, controller clock 170 MHz, data-phase timing limits (dtseg1 1..32, dbrp 1..32) reported
Limit: can0 up at 1 Mbit/s   Verdict: pass (bitrate); the automatic bus-off restart is not available on this adapter
Notes: the 170 MHz clock and the data-phase limits show the adapter is CAN FD capable (the owner was right; the repository runs classic CAN). `can_up.sh` fixed on branch fix/can-up-restart-ms (tries `restart-ms 100`, warns and carries on without it). After a bus-off on the PC side: `ip link set can0 down; ip link set can0 up`. TFC-FDIR-010 on the PC side cannot be met by this adapter alone; the nodes' own recovery is measured later.

### 2026-10-05  P-M1-01 step 5 (listen, loopback)   T. Nardelli
Item: USB-CAN adapter SH-C31A on can0
Measured: `python3 -m tfc_peers listen --iface can0 --duration 3` with nothing on the bus; loopback test (`loopback on`, candump, cansend), then `loopback off`
Limit: no frames, no error; loopback echoes the frame sent   Verdict: pass (reported by the operator)
Notes: the frame contents and counts were not recorded; repeat with the output kept when the first node is on the bus.

### 2026-10-05  P-M1-01 step 4 (hub, partly)   T. Nardelli
Item: 7-port USB 3.0 hub with per-port switches, with the adapter on one port
Measured: the adapter appears in lsusb through the hub; turning its port off removes it and on restores it
Limit: each device appears; the per-port switch cuts and restores it   Verdict: pass (reported by the operator)
Notes: the five micro-USB cables are not tested yet (they need a Nucleo or a Pico on the end; a charge-only cable only shows up then).

### 2026-10-05  P-M1-01 (bus terminations, before the backbone is built)   T. Nardelli
Item: 120 ohm resistors (100-piece pack): a sample, then two in parallel
Measured: with the multimeter on ohms, a sample of the resistors and two in parallel; the operator reports them good (values not recorded)
Limit: about 120 ohm each (within 5 %), about 60 ohm for two in parallel   Verdict: pass (reported by the operator)
Notes: the numbers were not written down. The 60 ohm check of the finished bus (power off, between CANH and CANL) is repeated when the backbone is built.

### 2026-10-06  P-M1-01 step 3 (5 V adapter, jacks A and B)   T. Nardelli
Item: 5 V 4 A switching supply (node-rail adapter), with each of the two 2.1 mm DC jack to screw terminal adapters
Measured: multimeter on DC volts, probes on the heads of the two terminal screws. No load: **5.3 V**. Load of five 120 ohm resistors in parallel (measured 23 ohm with the supply unplugged, about 0.22 A): **5.06 V**. The same on jack A and jack B
Limit: 4.75 to 5.25 V (the `E5V` window), at no load and at 50 mA   Verdict: **fail at no load (5.3 V); pass under load (5.06 V)**
Notes: the step's load is 100 ohm (50 mA); five 120 ohm resistors were used, which is a heavier load (about 0.22 A) and so a stricter test of the sag. A Nucleo is never on an unloaded rail (the Picos and relay coils are always on it, and the Nucleo itself draws current), so the supply is judged usable without a series diode. A diode drops 0.3 to 0.4 V and would take the loaded 5.06 V to about 4.7 V, under the 4.75 V floor. Not settled: the voltage at a Nucleo's `E5V` pin with the whole rail loaded and the coils switching. That is measured on 9 Oct; add a diode or a buck module only if it reads above 5.25 V. The second adapter gets this test before use. Measuring tip: read the voltage on the screw heads, with the meter on DC volts (red lead in the V socket); a load clamped badly in the terminal gives a falsely low reading.

### 2026-10-06  P-M1-01 step 3 (jack polarity)   T. Nardelli
Item: both DC jack to screw terminal adapters, with the supply's plug
Measured: with the supply on, the sign of the meter reading across the two screws, to find the centre-pin terminal of each adapter
Limit: centre positive (the usual barrel-plug convention; the polarity of this supply's plug was not recorded)   Verdict: pass (reported by the operator; the sign was not recorded)
Notes: the polarity matters because the 1000 uF capacitors are polarised. Record which screw is the centre pin on each adapter (and mark it) before the first node is wired.

### 2026-10-06  P-M1-01 step 1 (6 Oct parts: terminal blocks, headers, IMUs)   T. Nardelli
Item: ten 3-pin 3.5 mm terminal blocks (two packs of five); ten 36-pin 0.1 in header strips; three ISM330DHCX IMU boards
Measured: terminal blocks: count, plug and unplug, a stripped Cat6 conductor clamped (continuity through the plugged block, open when unplugged), fit on the prototype board. Headers: count, clean break, fit in an IMU's holes. IMUs: identity and silkscreen against the sheet, visual inspection, 3V3 to GND with the meter on ohms (unpowered), board size and mounting holes (step 11)
Limit: every row present and as listed; no short; the parts fit   Verdict: pass (reported by the operator)
Notes: the values (the ohm reading, the hole spacing, the IMU maker and pin labels) were not recorded; write them in when the IMU is wired to a Nucleo. Soldering is expected to be needed for the IMU, CAN Pal and Pico headers, and will be done at those hardware stages. The IMU's logic level (step 12) waits for a Pico.
