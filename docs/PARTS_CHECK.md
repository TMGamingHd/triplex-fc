# Parts sheet check (Triplex_Flight_Computer_Parts_v4, prices of 2026-09-29)

> Written 4 Oct 2026 against the owner's parts sheet (six sheets: Parts List, 3D Prints, Budget, Compatibility, Compatibility Audit, Notes). It records what the
> sheet already settles, what conflicts with the repository's design documents, and what is still missing. Nothing was bought or measured by this check.

## 1. What matches the plan

| Plan | Parts sheet | Result |
|---|---|---|
| Three flight computers and ACT on Nucleo-G474RE | 4 boards (row 1) | Matches |
| One CAN transceiver per node | 4 CAN Pal TJA1051T/3 (row 2), terminations off, 2 x 120 ohm on the backbone | Matches; check 60 ohm with the power off |
| One ISM330DHCX per flight computer, on SPI | 3 boards (row 3), header pins, level-shifted | Matches. The firmware's overlay uses SPI2 (PB13, PB14, PB15, CS PB12) and avoids SPI1's clock, which is shared with the user LED (audit item 5) |
| FDCAN1 on the board file's pins | Audit item 4: either pair works, take it from the board file | Matches: the app takes the board file's FDCAN1 (PA11, PA12) and nothing else |
| Node power through `E5V` | Row 18, audit items 1 and 2 | Matches. Read the adapter under load: above 5.2 V needs a diode or a buck module |
| Fault injector Pico 2 with relays | 1 Pico 2 (row 4), 2 relay modules = 8 channels (row 7) | Matches the count (decision of 4 Oct 2026: four channels cut the power of A, B, C and ACT) |
| Servo rail E-stop (TFC-PLAT-003) | 22 mm latching E-stop, 2 NC contacts (row 26), fuse 5 A | Matches |
| Pico input pull-downs (erratum E9) | 4.7 kOhm, audit item 9 | Matches `SUPERVISOR.md` (8.2 kOhm or less) |
| USB for every board | 7-port hub with manual port switches (row 27), 6 ports used | Matches; the supervisor's Pico is the seventh |

## 2. Conflicts and gaps (action in the last column)

| # | Finding | Why it matters | Action |
|---|---|---|---|
| 1 | **Resolved 4 Oct 2026: the owner chose a third relay module.** The four spare relay channels were claimed twice. The sheet (row 7) says they are for "sensor-line and bus-stub faults"; `SUPERVISOR.md` uses them as the supervisor's four `PWR` relays | With the injector on four power cuts and the supervisor on the other four, **no channel is left for a sensor-line or bus-stub fault** (F03, the HIL half of the bus faults) | Buy a **third relay module** (6.99 USD on the sheet) so the sensor and bus faults have four channels of their own. Done in `HARDWARE_PARTS.md` |
| 2 | **Partly resolved 4 Oct 2026:** the second Pico 2 and the TCXO module are added to the order list (`HARDWARE_PARTS.md`; the TCXO's model and price are not checked); the resistors are owned. The supervisor's parts are still not on the sheet: a **second Pico 2** (6.00), a **TCXO clock module** (price not checked), and **resistors** (the sheet's only resistors are 120 ohm: pull-ups to 3.3 V for the relay inputs and the supervisor's `PWR` lines, 4.7 kOhm pull-downs for the Pico inputs) | `SUPERVISOR.md` section 9 says the order must include them; it is not placed yet | Confirm the TCXO module's model and its 32 kHz output |
| 3 | **Hardware overrides** (`HARDWARE_OVERRIDE.md`: a force-safe toggle, a platform-level switch, two disarm switches, node kill switches) are not on the sheet | The E-stop is the only manual override in the list | Wait for TS-17 to decide the count; about 25 USD at most |
| 4 | **The owner reports that the adapter model supports CAN FD (4 Oct 2026); the sheet's text is still inconsistent.** The CAN adapter's model is inconsistent. Row 5 names the DSD TECH SH-C31A and says in the same cell that "the SH-C31A at 17.99 is the FD version"; the audit and the compatibility sheet say SH-C30A and that FD support is unconfirmed. It is **not isolated** | The tools assume `gs_usb` at 1 Mbit/s classic (`tools/bench/can_up.sh`), which either does. A non-isolated adapter is a risk if the bus is shorted by a fault test, and the bus-stub relays are for that | Check the model when it arrives. Consider the isolated SH-C30G (19.99), as row 5 itself suggests |
| 5 | **Servo stall current disagrees between sources** (2.15 A each on one Hitec page, 1.2 to 1.4 A on another; two servos stalled could reach 4.3 A against a 4 A adapter) | The platform's rate and travel limit (TFC-PLAT-001) and the hard stops inside the servo travel are what keep the servos out of stall | Measure on the bench; the Pico firmware limits the platform to 45 degrees where the servo can do 145; keep the 5 A fuse |
| 6 | **The servo's signal level is not stated** (3.3 V from the Pico) | The 74AHCT125 level shifter (row 30) is a zero-quantity option | Test with the real servo; buy the shifter only if it twitches |
| 7 | **The relay module at 3.3 V drive** (audit item 7): with the `VCC`/`JD-VCC` jumper left on, a high from the Pico may not turn the opto LED off | A relay that does not release is a node that stays cut | Remove the jumper, feed `VCC` from the Pico's 3V3 and `JD-VCC` from the 5 V node rail, use the normally-closed contacts, and test every channel with a meter before a node is connected. The firmware's relay outputs are active low with external pull-ups, so a floating pin means "off" |
| 8 | **A power cut leaves the ST-LINK alive** (audit item 1) | Logging survives a cut, which is good. The cut node's other pins (CAN transceiver, IMU, the ST-LINK's own lines) can be fed through protection diodes by powered neighbours (G8, G11 in `HARDWARE_OVERRIDE.md`) | Meter the pins of a cut node on the bench; compare with a real power loss |
| 9 | **Eight relay coils on the node rail** (about 0.56 A) and `E5V`'s 4.75 to 5.25 V window | A coil switching on can pull the rail down and reset a healthy node (G7) | The 1000 uF capacitor on the node rail is in the sheet (row 22); scope the rail with all eight switching |
| 10 | **The budget is 77.81 USD over its ceiling** (627.60 parts, 677.81 with the 8% allowance, against 600) | Everything in 2 and 3 adds to it | The sheet's own advice: cut Phase 2 and the optional rows first. The third relay module, the supervisor parts and the overrides come to roughly 45 USD |

## 3. Software readiness against the parts (4 Oct 2026)

| Part | Software state |
|---|---|
| Nucleo boards, CAN Pals | The firmware builds for `nucleo_g474re` (flight computer and ACT, about 52 kB and 39 kB of 512 kB of flash), runs on `native_sim`, and the closed loop runs live on `vcan0`. **Not run on a board.** |
| IMUs | The ISM330DHCX driver, written against the datasheet's register map, is host-tested with a fake bus. **Not run on a part.** |
| USB-CAN adapter | `tools/bench/*` scripts and the live logger are written and tested against `vcan0`. **Not run on the adapter.** |
| Pico 2 (injector and platform) | A Zephyr build check passed on 4 Oct 2026: `rpi_pico2/rp2350a/m33` builds the Zephyr blinky, the USB CDC-ACM sample and the PWM fade sample, each producing a UF2 file (17 kB, 57 kB and 27 kB of 4 MB). The board file lists `gpio`, `pwm`, `usbd`, `watchdog`, `hwinfo`. **The run-time behaviour (USB enumeration, the 50 Hz servo pulse and its jitter, the watchdog) is not checked until the board arrives.** Decision: Zephyr (ADR-026) |
| Servos, platform | The platform model and the platform driver's safety logic are the next work (P1-5). The mechanical side and the servos' real behaviour with no signal wait for the hardware |
| Supervisor | Not started; waits for its parts (item 2) |
