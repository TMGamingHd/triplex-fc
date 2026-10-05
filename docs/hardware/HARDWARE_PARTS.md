# Hardware parts: everything the setup needs

> Status: **reference** (reviewed 5 Oct 2026). Written from the owner's parts sheet (Triplex_Flight_Computer_Parts_v4, prices checked 2026-09-29) plus the decisions of 4 and 5 Oct, and merged with the audit of that sheet. **The order has been placed and arrives on 9 Oct 2026.**
> Prices are the sheet's unless marked **n/c** (not checked). **Status:** *ordered* = in the order the owner placed; *check* = decided on 4 Oct and to be confirmed against the order confirmation (the second Pico 2, the TCXO module, the third relay module);
> *owned* = the owner has it; *not ordered* = decided, not in the order (the override parts, section 4). Nothing was bought or measured by this document.

## 1. The system in parts

```
PC (Ubuntu) --USB hub--+-- Nucleo A, B, C (flight computers)  --- CAN Pal --+
                       +-- Nucleo ACT (actuator node)          --- CAN Pal --+-- CAN backbone (2 x 120 ohm) -- USB-CAN adapter -- PC
                       +-- Pico 1: platform driver + fault injector --> 2 servos (platform) ; 4 relays (node power cuts) + 4 relays (sensor and bus faults)
                       +-- Pico 2: supervisor (SUP-Lite) + TCXO clock --> FRAME/KICK/NRST per node, PWR relays, SAFE to ACT
                       +-- USB-CAN adapter
5 V node rail (adapter 1): Nucleos (E5V), relay coils, Picos          5 V servo rail (adapter 2): servos, through a fuse and the E-stop
```

## 2. Bill of parts

### 2.1 Core electronics
| # | Part | Qty | Status | Role in the software |
|---|---|---|---|---|
| 1 | STM32 NUCLEO-G474RE | 4 | ordered | Flight computers A, B, C and ACT (`firmware/app`, `firmware/act`) |
| 2 | Adafruit CAN Pal (TJA1051T/3) | 4 | ordered | The CAN transceiver of each Nucleo (classic CAN, 1 Mbit/s) |
| 3 | Adafruit ISM330DHCX IMU | 3 | ordered | One per flight computer, SPI2 (`ism330dhcx.hpp`) |
| 4 | Raspberry Pi Pico 2, with headers | **1 -> 2** | ordered; **second: check** | Pico 1: platform driver and fault injector (`firmware/pico`, `PICO.md`). Pico 2: the supervisor (`SUPERVISOR.md`) |
| 5 | USB-to-CAN adapter (DSD TECH, gs_usb) | 1 | ordered | SocketCAN `can0` for the bench tools. **The owner says the model supports CAN FD**; the repository runs classic CAN at 1 Mbit/s, which it also does. Not isolated |
| 6 | Kingst LA1010 logic analyzer | 1 | ordered | SPI and CAN timing, frame jitter, servo pulse width and jitter, relay timing |
| 7 | 4-channel opto-isolated 5 V relay module (ELEGOO) | 2 -> **3** | ordered; **third: check** | Module 1: injector power cuts A, B, C, ACT. Module 2: the supervisor's four `PWR` relays. **Module 3: sensor-line and bus-stub faults** (ADR-026) |
| 8 | **TCXO real-time-clock module** (DS3231-class) | **1** | **check**; model and price **n/c** | The supervisor's clock (`SUPERVISOR.md` section 3a: about 2 ppm, a 32.768 kHz reference). Confirm the 32K output is brought out on the module before ordering |
| 9 | **Resistors** (4.7 k, 10 k, 100 k assortment) | 1 kit | **owned** | Pull-ups to 3.3 V on every relay input (12) and the supervisor's `PWR` lines, 4.7 k pull-downs on the Pico inputs (RP2350 erratum E9: 8.2 k or less) |

### 2.2 Wiring, connectors and cabling
| Part | Qty | Status | Notes |
|---|---|---|---|
| Dupont jumper kit (40 M-F, 40 M-M, 40 F-F) | 2 | ordered | Nucleo, CAN Pal, IMU, Pico, relays. The supervisor adds about 17 lines (4 x `FRAME`, 4 x `KICK`, 4 x `NRST`, 4 x `PWR`, `SAFE`): two kits are enough |
| 22 AWG hook-up wire, 6 colours | 1 | ordered | Rails, grounds, relay wiring, the override switches |
| Break-away header strips | 1 | ordered | Pico 1 and 2 (2 x 20 pins each: **80**, the sheet counted 40 for one Pico), CAN Pals, IMUs |
| 5 x 7 cm prototype boards | 1 pack | ordered | CAN backbone, Pico carrier, spare; the supervisor's wiring board is the spare |
| Heat shrink, zip ties, solder (optional) | 1 each | ordered | |
| Cat6 patch cable, 3-pin 3.5 mm terminals (2 packs), 120 ohm resistors | 1, 2, 1 | ordered | Twisted-pair stubs, the taps, the two terminations. Unplugging a terminal block is also a manual bus isolation (`HARDWARE_OVERRIDE.md`) |

### 2.3 Power and servo safety
| Part | Qty | Status | Notes |
|---|---|---|---|
| 5 V 4 A adapter, node rail | 1 | ordered | Nucleos through `E5V` (JP5 to E5V; 4.75 to 5.25 V: read it under load), the relay coils, the Picos. Load about 1.2 A, plus up to 0.84 A if all twelve coils are on |
| 5 V 4 A adapter, servo rail | 1 | ordered | The two servos; stall could reach 4.3 A (section 7, item 5) |
| DC jack to screw terminal | 2 | ordered | |
| WAGO 221 levers | 1 | ordered | Star grounds (the two rails share one ground point), rail splits |
| 1000 uF 16 V capacitors | 1 pack | ordered | One on the node rail (relay inrush, G7), one at each servo |
| In-line blade fuse holder, 5 A fuses | 1, 1 | ordered | Servo rail |
| 22 mm latching E-stop, 2 NC contacts | 1 | ordered | Cuts the servo rail (TFC-PLAT-003): override H1 |

### 2.4 USB, servos and the platform
| Part | Qty | Status | Notes |
|---|---|---|---|
| 7-port powered USB 3 hub, per-port switches | 1 | ordered | **Seven of seven ports are now used**: four Nucleos, Pico 1, Pico 2, the USB-CAN adapter. The logic analyzer stays on a PC port. A second hub, or a Nucleo on the PC directly, is needed to add anything |
| Micro-USB data cables, 1 ft | 5 -> **6** | ordered; **one more: check** | The supervisor's Pico 2 is Micro-B too |
| Hitec D85MG servo | 2 | ordered | The platform's two axes; PWM from Pico 1 (GP2, GP3) |
| 74AHCT125 level shifter | 0 | optional | Only if a servo ignores 3.3 V |
| Aluminium servo horns | 0 | optional | |

### 2.5 Mechanical (3D prints and hardware)
M3 and M2 screw kits, brass heat-set inserts, M3 standoffs (the third relay module needs **4 more M3 screws and standoffs**), 623ZZ bearings, rubber feet, PLA filament (owned printer): all on the sheet, unchanged. The third relay module needs a place in the injector box print (`3D Prints` sheet, 72 x 51.7 mm, holes 65.9 x 45 mm apart).

## 3. Added by the decisions of 4 Oct 2026 (to confirm against the order)
| Part | Qty | Price | Why |
|---|---|---|---|
| Raspberry Pi Pico 2 | 1 | 6.00 (sheet) | The supervisor must be separate from the fault injector (ADR-022) |
| TCXO RTC module | 1 | **n/c** | The supervisor's independent clock; its 32 kHz output must be brought out |
| ELEGOO 4-channel relay module | 1 | 6.99 (sheet) | Sensor-line and bus-stub faults need channels the supervisor does not use (ADR-026) |
| Micro-USB cable | 1 | about 1.50 (the pack of five is 7.19) | Pico 2 |
| Header strip pins (80 for two Picos) | covered | in the 36-pin pack | The sheet's count was for one Pico |
| **Subtotal added, priced** | | **about 14.50 + the TCXO module** | |

The sheet's budget was already 77.81 USD over its 600 USD ceiling (627.60 parts; 677.81 with the sheet's 8 % allowance for shipping and tax). These additions and the override parts come on top; the sheet's own advice is to cut Phase 2 and the optional rows first.

## 4. Hardware overrides: decided by TS-17, **not ordered**
TS-17's paper part chose option **O3** (`TRADE_STUDIES.md` section 9c, `HARDWARE_OVERRIDE.md`): the E-stop that the sheet has (H1) and four more. Buy these separately this week; they are cheap, common parts, and none has been priced on a listing. `P-HWO-01` waits for them.
| Part | Qty | Rough cost (USD) | Override |
|---|---|---|---|
| Guarded toggle switch, SPST, and two diodes | 1 + 2 | 2 | H2 FORCE-SAFE (the diode-OR with the supervisor's `SAFE`) |
| DPDT switch per servo, plus a servo-tester board (or a 555 circuit) as the no-code neutral source | 2 + 1 | 8 | H3 PLATFORM-LEVEL |
| Key or toggle switch | 2 | 2 | H4 INJECTOR-DISARM, H5 SUPERVISOR-DISARM (in each relay module's coil supply; the third module needs one too) |
| Switch-state sense lines | 5 | (resistors, owned) | The supervisor's sense inputs: 4.7 kOhm pull-downs; set `CONFIG_TFC_OVERRIDE_LINES_FITTED=0x1F` |
**Not built:** the per-node power-kill switches and the master-power switch (H6, H7): they add no scenario that the Nucleos' reset buttons, the unpluggable CAN stubs and H4 do not already cover.

## 5. Test equipment
| Item | Status | Used for |
|---|---|---|
| Multimeter | owned | Every relay channel before a node is connected; rail voltage; continuity of the overrides |
| Logic analyzer (LA1010) | ordered | Servo pulse width and jitter, relay and `NRST` timing, USB-to-platform latency |
| **Oscilloscope** | **not on the list** (borrow from the lab) | The node rail while twelve relay coils switch (G7), a cut node's pins (G8), the servo current. The Pico's own ADC on a divider is a cheap partial substitute (`PICO_TESTS.md`) |
| Spare IMU and CAN Pal | on sheet (optional) | Fault injection can kill boards |
| 3D printer, PC, soldering | owned | |

## 6. Open items
1. **The order confirmation:** the second Pico 2, the TCXO module (model, price, and that its 32 kHz output is on the board's pins) and the third relay module.
2. **The override parts** (section 4): buy before `P-HWO-01`.
3. Whether the injector box print is redrawn for three relay modules.
4. The USB hub is full (section 2.4): decide whether a second hub is wanted for later.
5. The CAN adapter's exact model on arrival (the owner reports FD support; the sheet's own text is inconsistent), and whether to take the isolated model because the bus-stub relays are for shorting the bus.
6. An oscilloscope to borrow for the rail and servo-current measurements (section 5).

## 7. Audit of the parts sheet (4 Oct 2026)
The sheet has six tabs (Parts List, 3D Prints, Budget, Compatibility, Compatibility Audit, Notes). **What matches the plan:** three flight computers and ACT on Nucleo-G474RE (4 boards); one CAN transceiver per node (TJA1051T/3, terminations off, two 120 ohm on the
backbone; check 60 ohm with the power off); one ISM330DHCX per flight computer on SPI2 (PB13, PB14, PB15, CS PB12, which avoids SPI1's clock shared with the user LED, audit item 5); FDCAN1 on the board file's pins; node power through `E5V` (read the adapter under load: above 5.2 V
needs a diode or a buck module); the injector Pico with 8 relay channels; the servo-rail E-stop with a 5 A fuse; 4.7 kOhm pull-downs on the Pico inputs (the RP2350 erratum E9; the supervisor needs 8.2 kOhm or less); a 7-port hub (the supervisor's Pico is the seventh). **Conflicts and gaps:**

| # | Finding | Why it matters | State |
|---|---|---|---|
| 1 | The four spare relay channels were claimed twice: "sensor-line and bus-stub faults" on the sheet, the supervisor's four `PWR` relays in `SUPERVISOR.md` | With both on four channels, none is left for a sensor-line or bus-stub fault (F03, the HIL half of the bus faults) | **Resolved 4 Oct:** a third relay module (check the order) |
| 2 | The supervisor's parts were not on the sheet: a second Pico 2, a TCXO module, resistors (pull-ups to 3.3 V on the relay inputs and the supervisor's `PWR` lines, pull-downs on the Pico inputs) | `SUPERVISOR.md` needs them | **Resolved 4 Oct:** the Pico and the TCXO added (check the order, and the TCXO's 32 kHz output); the resistors are owned |
| 3 | The hardware overrides were not on the sheet | The E-stop is the only manual override in the list | **Decided 5 Oct (TS-17, O3): section 4. Not ordered** |
| 4 | The CAN adapter's model is inconsistent on the sheet (SH-C31A "the FD version" in one cell, SH-C30A with FD unconfirmed in another) and it is **not isolated** | The tools assume `gs_usb` at 1 Mbit/s classic, which either does; a non-isolated adapter is a risk when a fault test shorts the bus | The owner reports FD support. Check the model on arrival; consider the isolated SH-C30G (19.99) |
| 5 | Servo stall current disagrees between sources (2.15 A each on one Hitec page, 1.2 to 1.4 A on another; two servos stalled could reach 4.3 A against a 4 A adapter) | The platform's rate and travel limit (TFC-PLAT-001) and the hard stops inside the servo travel keep the servos out of stall | Measure on the bench; the Pico limits the platform to 45 degrees where the servo can do 145; keep the 5 A fuse |
| 6 | The servo's signal level is not stated (3.3 V from the Pico) | The 74AHCT125 level shifter is a zero-quantity option | Test with the real servo; buy the shifter only if it twitches |
| 7 | The relay module at 3.3 V drive (audit item 7): with the `VCC`/`JD-VCC` jumper left on, a high from the Pico may not turn the opto LED off | A relay that does not release is a node that stays cut | Remove the jumper, feed `VCC` from the Pico's 3V3 and `JD-VCC` from the 5 V node rail, use the normally-closed contacts, test every channel with a meter before a node is connected (`P-M1-01`). The firmware's relay outputs are active low with external pull-ups |
| 8 | A power cut leaves the ST-LINK alive (audit item 1) | Logging survives a cut; the cut node's other pins can be fed through protection diodes by powered neighbours (G8, G11) | Meter the pins of a cut node; compare with a real power loss (`P-HWO-01`) |
| 9 | Eight (now twelve) relay coils on the node rail, about 0.56 A, against `E5V`'s 4.75 to 5.25 V window | A coil switching on can pull the rail down and reset a healthy node (G7) | The 1000 uF capacitor is on the sheet (row 22); scope the rail with all coils switching |
| 10 | The budget is 77.81 USD over its ceiling | Everything above adds to it | Cut Phase 2 and the optional rows first; the third module, the supervisor parts and the overrides come to roughly 45 USD |
