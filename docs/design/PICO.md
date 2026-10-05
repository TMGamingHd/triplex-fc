# The Pico: platform driver and fault injector

> Status: **built and tested on the host, builds for the board, not run on a board** (P1-5, ADR-026). The logic is in `core/` (`pico_link.hpp`, `platform_driver.hpp`,
> `injector.hpp`) with host tests; the application is `firmware/pico` (Zephyr, `rpi_pico2/rp2350a/m33`, 59 kB of 4 MB flash, UF2 output); the PC side is
> `sim/tfc_peers/pico_link.py` and `python3 -m tfc_peers pico`. This is the Pico of the parts sheet (row 4), **not** the supervisor's (a second Pico, `SUPERVISOR.md`).

## 1. Two jobs, one board

1. **The platform driver.** The PC sends the platform's two tilts at 100 Hz (from `tfc_simd`'s platform model); the Pico makes the two servo pulses, with its own limits whatever the
   PC sends (TFC-PLAT-001, 002, 004).
2. **The fault injector.** Four relays, one power cut each for flight computers A, B, C and the actuator node (decision of 4 Oct 2026). A cut is a **time**, named by the PC, that ends by
   itself.

## 2. The link (USB serial, `core/include/tfc/pico_link.hpp`)

`0xA5 | type | length | payload (0 to 12 bytes) | CRC-8` with the flight bus's CRC. Little endian. A parser finds frames in the stream and drops what does not check.

| Type | Direction | Payload |
|---|---|---|
| `0x01` platform | PC to Pico | sequence u8, tilt about X i16, tilt about Y i16 (0.01 degree). Sent at 100 Hz |
| `0x02` relay | PC to Pico | channel u8 (0 A, 1 B, 2 C, 3 ACT), cut time u16 ms (0 releases; at most 30,000; send again before it ends to keep it) |
| `0x03` ping | PC to Pico | none; asks for a status now |
| `0x81` status | Pico to PC | sequence echo u8, output X i16, output Y i16, flags u8, relays u8 (bit n set: channel n energised, node n cut), command age u16 ms. Every 20 ms |

Flags: saturated, holding, levelling, link lost, watchdog reset, rejected command. The golden frames are pinned in `tests/test_pico.cpp` and `sim/tests/test_pico_link.py`.

## 3. The platform's safety (what the Pico does whatever the PC says)

| Rule | Behaviour | Requirement |
|---|---|---|
| Travel | a tilt beyond 45 degrees is clamped to it, and the command is reported as saturated (the simulator marks the run) | PLAT-001, 004 |
| Rate | the output moves toward the target by at most 300 degrees per second, so a step becomes a ramp (a full reversal from 30 to -30 degrees takes 0.2 s) | PLAT-001 |
| Timeout | no command for 100 ms: **hold** where it is; for 1 s: bring to **level** at 30 degrees per second | PLAT-002 |
| Validity | a command that is not a number, or is wildly out of range, is rejected and counted; it does not restart the timeout | PLAT-001 |
| Servo pulse | `neutral + sign x (angle + trim) x 10.34 us per degree`, never outside 900 to 2100 us (the D85MG: 850 to 2350 us over 145 degrees; +-45 degrees is +-466 us) | PLAT-001 |
| At boot | the servos are told **level** before the first command | |

The numbers are Kconfig options (`firmware/pico/Kconfig`) so a bench measurement changes a number, not code.

## 4. The injector's safety (a cut ends by itself)

- **Boot, reset, a crash:** every relay input has an external pull-up to 3.3 V and the application configures every output inactive first, so a pin that is not driven is a relay **off**, which is a
  **powered node** (normally-closed contacts).
- **A cut is a time.** The PC names how long (at most 30 s) and sends again to keep it; a PC that crashes, a cable that is pulled or a Pico that is reset leaves every node powered by the end of the time.
- **The link timeout:** nothing from the PC for 2 s (`CONFIG_TFC_LINK_TIMEOUT_MS`) releases every relay, whatever time was left, and sets the link-lost flag.
- **The hardware watchdog** (100 ms) resets the Pico if the loop hangs. After a watchdog reset the relays are off and the servo pulse stops; the status says so. **What the D85MG does with no pulse is not known** (parts sheet note 11): measure it.
- **A disarm switch** in the relay modules' coil supply (`HARDWARE_OVERRIDE.md`, H4) makes any cut impossible whatever the Pico does.

## 5. Wiring (devicetree overlay `firmware/pico/boards/rpi_pico2_rp2350a_m33.overlay`)

| Pico pin | Function | Notes |
|---|---|---|
| GP2 | servo, tilt about X (PWM slice 1, channel A) | 50 Hz, 20 ms period, 0.43 us resolution (slice clock divided by 64) |
| GP3 | servo, tilt about Y (slice 1, channel B) | shares the period with GP2 |
| GP10, GP11, GP12, GP13 | relay inputs for nodes A, B, C, ACT | active low; **external pull-up to 3.3 V on each**; relay module `VCC` from the Pico's 3V3, `JD-VCC` from the 5 V node rail, jumper removed (parts check item 7) |
| USB | serial to the PC (CDC ACM, vendor id 0x1209 and product id 0x0001: the open pid.codes ids, for a private bench) | |

Not yet wired: the read-only sense lines of the hardware overrides (HWO-005), which need 4.7 kOhm pull-downs (erratum E9).

## 6. Build, flash, first use

```
. firmware/env.sh
west build -p auto -b rpi_pico2/rp2350a/m33 firmware/pico -d build/pico
```
Hold the BOOTSEL button while plugging the Pico in, and copy `build/pico/zephyr/zephyr.uf2` onto the drive that appears. Then `python3 -m tfc_peers pico status`.

## 7. What to check when the board arrives (none of it is known yet)

1. The board enumerates as a serial port, and `pico status` answers at 50 Hz without losing frames while the platform stream runs at 100 Hz.
2. The servo pulse: a scope or the logic analyzer on GP2 shows 20 ms and the commanded width, with jitter well under 10 us; both servos follow the 3.3 V signal (else the 74AHCT125 of parts row 30).
3. The servo with **no** pulse (after a watchdog reset or with the Pico unplugged): does it hold, drift or go limp?
4. Every relay channel with a meter before a node is connected (parts check item 7); a node cut and restored; the rail while all four switch (G7 in `HARDWARE_OVERRIDE.md`).
5. The watchdog: stall the loop on purpose and see the relays release and the status report it.
6. The link timeout: kill the PC tool during a cut and see the node come back.
