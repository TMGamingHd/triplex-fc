# Hardware parts: everything the setup needs

> Written 4 Oct 2026 from the owner's parts sheet (Triplex_Flight_Computer_Parts_v4, prices checked 2026-09-29) plus the decisions of that day. Prices are the sheet's
> unless marked **n/c** (not checked). **Status:** *on sheet* = in the sheet; *add* = decided, to add to the order; *owned* = the owner has it; *proposed* = waits for a study.
> What each part is for in the software: the last column. Nothing here was bought or measured by this document.

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
| 1 | STM32 NUCLEO-G474RE | 4 | on sheet | Flight computers A, B, C and ACT (`firmware/app`, `firmware/act`) |
| 2 | Adafruit CAN Pal (TJA1051T/3) | 4 | on sheet | The CAN transceiver of each Nucleo (classic CAN, 1 Mbit/s) |
| 3 | Adafruit ISM330DHCX IMU | 3 | on sheet | One per flight computer, SPI2 (`ism330dhcx.hpp`) |
| 4 | Raspberry Pi Pico 2, with headers | **1 -> 2** | on sheet; **second added** | Pico 1: platform driver and fault injector (`firmware/pico`, `PICO.md`). Pico 2: the supervisor (`SUPERVISOR.md`; firmware not written) |
| 5 | USB-to-CAN adapter (DSD TECH, gs_usb) | 1 | on sheet | SocketCAN `can0` for the bench tools. **The owner says the model supports CAN FD**; the repository runs classic CAN at 1 Mbit/s, which it also does. Not isolated |
| 6 | Kingst LA1010 logic analyzer | 1 | on sheet | SPI and CAN timing, frame jitter, servo pulse width and jitter, relay timing |
| 7 | 4-channel opto-isolated 5 V relay module (ELEGOO) | 2 -> **3** | on sheet; **third added** | Module 1: injector power cuts A, B, C, ACT. Module 2: the supervisor's four `PWR` relays. **Module 3: sensor-line and bus-stub faults** (ADR-026) |
| 8 | **TCXO real-time-clock module** (DS3231-class) | **1** | **add**; model and price **n/c** | The supervisor's clock (`SUPERVISOR.md` section 3a: about 2 ppm, a 32.768 kHz reference). Confirm the 32K output is brought out on the module before ordering |
| 9 | **Resistors** (4.7 k, 10 k, 100 k assortment) | 1 kit | **owned** | Pull-ups to 3.3 V on every relay input (12) and the supervisor's `PWR` lines, 4.7 k pull-downs on the Pico inputs (RP2350 erratum E9: 8.2 k or less) |

### 2.2 Wiring, connectors and cabling
| Part | Qty | Status | Notes |
|---|---|---|---|
| Dupont jumper kit (40 M-F, 40 M-M, 40 F-F) | 2 | on sheet | Nucleo, CAN Pal, IMU, Pico, relays. The supervisor adds about 17 lines (4 x `FRAME`, 4 x `KICK`, 4 x `NRST`, 4 x `PWR`, `SAFE`): two kits are enough |
| 22 AWG hook-up wire, 6 colours | 1 | on sheet | Rails, grounds, relay wiring, the override switches |
| Break-away header strips | 1 | on sheet | Pico 1 and 2 (2 x 20 pins each: **80**, the sheet counted 40 for one Pico), CAN Pals, IMUs |
| 5 x 7 cm prototype boards | 1 pack | on sheet | CAN backbone, Pico carrier, spare; the supervisor's wiring board is the spare |
| Heat shrink, zip ties, solder (optional) | 1 each | on sheet | |
| Cat6 patch cable, 3-pin 3.5 mm terminals (2 packs), 120 ohm resistors | 1, 2, 1 | on sheet | Twisted-pair stubs, the taps, the two terminations. Unplugging a terminal block is also a manual bus isolation (`HARDWARE_OVERRIDE.md`) |

### 2.3 Power and servo safety
| Part | Qty | Status | Notes |
|---|---|---|---|
| 5 V 4 A adapter, node rail | 1 | on sheet | Nucleos through `E5V` (JP5 to E5V; 4.75 to 5.25 V: read it under load), the relay coils, the Picos. Load about 1.2 A, plus up to 0.84 A if all twelve coils are on |
| 5 V 4 A adapter, servo rail | 1 | on sheet | The two servos; stall could reach 4.3 A (PARTS_CHECK item 5) |
| DC jack to screw terminal | 2 | on sheet | |
| WAGO 221 levers | 1 | on sheet | Star grounds (the two rails share one ground point), rail splits |
| 1000 uF 16 V capacitors | 1 pack | on sheet | One on the node rail (relay inrush, G7), one at each servo |
| In-line blade fuse holder, 5 A fuses | 1, 1 | on sheet | Servo rail |
| 22 mm latching E-stop, 2 NC contacts | 1 | on sheet | Cuts the servo rail (TFC-PLAT-003): override H1 |

### 2.4 USB, servos and the platform
| Part | Qty | Status | Notes |
|---|---|---|---|
| 7-port powered USB 3 hub, per-port switches | 1 | on sheet | **Seven of seven ports are now used**: four Nucleos, Pico 1, Pico 2, the USB-CAN adapter. The logic analyzer stays on a PC port. A second hub, or a Nucleo on the PC directly, is needed to add anything |
| Micro-USB data cables, 1 ft | 5 -> **6** | on sheet; **one more** | The supervisor's Pico 2 is Micro-B too |
| Hitec D85MG servo | 2 | on sheet | The platform's two axes; PWM from Pico 1 (GP2, GP3) |
| 74AHCT125 level shifter | 0 | optional | Only if a servo ignores 3.3 V |
| Aluminium servo horns | 0 | optional | |

### 2.5 Mechanical (3D prints and hardware)
M3 and M2 screw kits, brass heat-set inserts, M3 standoffs (the third relay module needs **4 more M3 screws and standoffs**), 623ZZ bearings, rubber feet, PLA filament (owned printer): all on the sheet, unchanged. The third relay module needs a place in the injector box print (`3D Prints` sheet, 72 x 51.7 mm, holes 65.9 x 45 mm apart).

## 3. Added by the decisions of 4 Oct 2026
| Part | Qty | Price | Why |
|---|---|---|---|
| Raspberry Pi Pico 2 | 1 | 6.00 (sheet) | The supervisor must be separate from the fault injector (ADR-022) |
| TCXO RTC module | 1 | **n/c** | The supervisor's independent clock |
| ELEGOO 4-channel relay module | 1 | 6.99 (sheet) | Sensor-line and bus-stub faults need channels the supervisor does not use (ADR-026) |
| Micro-USB cable | 1 | about 1.50 (the pack of five is 7.19) | Pico 2 |
| Header strip pins (80 for two Picos) | covered | in the 36-pin pack | the sheet's count was for one Pico |
| **Subtotal added, priced** | | **about 14.50 + the TCXO module** | |

The sheet's budget was already 77.81 USD over its 600 USD ceiling (627.60 parts; 677.81 with the sheet's 8% allowance for shipping and tax). These additions come on top.

## 4. Proposed, waiting for TS-17 (hardware overrides, `HARDWARE_OVERRIDE.md`)
| Part | Qty | Rough cost | Override |
|---|---|---|---|
| Guarded toggle switch, SPST | 1 | 2 | H2 force-safe (with two diodes) |
| DPDT switch per servo, plus a servo-tester board or a 555 circuit as the neutral source | 2 + 1 | 8 | H3 platform-level |
| Key or toggle switch | 2 | 2 | H4 injector-disarm, H5 supervisor-disarm (in each relay module's coil supply; the third module needs one too) |
| Rocker switches | 4 + 1 | 5 | H6 per-node power-kill, H7 master power |
| Switch-state sense lines | | (resistors, owned) | HWO-005 |
None of these is on the order until the paper part of TS-17 has decided the count.

## 5. Test equipment
| Item | Status | Used for |
|---|---|---|
| Multimeter | owned | Every relay channel before a node is connected; rail voltage; continuity of the overrides |
| Logic analyzer (LA1010) | on sheet | Servo pulse width and jitter, relay and `NRST` timing, USB-to-platform latency |
| **Oscilloscope** | **not on the list** (borrow from the lab) | The node rail while twelve relay coils switch (G7), a cut node's pins (G8), the servo current. The Pico's own ADC on a divider is a cheap partial substitute (`PICO_TESTS.md`) |
| Spare IMU and CAN Pal | on sheet (optional) | Fault injection can kill boards |
| 3D printer, PC, soldering | owned | |

## 6. Open items before the order closes
1. The TCXO module: model, price, and that its 32 kHz output is on the board's pins.
2. Whether the injector box print is redrawn for three relay modules.
3. The USB hub is full (section 2.4): decide whether a second hub is wanted for later.
4. The override parts (section 4) after TS-17.
5. The CAN adapter's exact model on arrival (the owner reports FD support; the sheet's own text is inconsistent), and whether to take the isolated model because the bus-stub relays are for shorting the bus.
