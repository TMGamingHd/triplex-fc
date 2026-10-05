# Tests for the Pico: the options

> Status: **a catalogue and a recommendation** (written 4 Oct 2026; reviewed 5 Oct 2026). Part of it is done (section 2); the rest needs the board, the relay modules or the rig. The Pico is
> `firmware/pico` (`PICO.md`): a platform driver (two servos) and a fault injector (relays). The question is what to test, with what, in which order, and what counts as a pass.

## 1. What can go wrong, which is what the tests are for

| Area | What has to be true | Requirement or finding |
|---|---|---|
| Platform limits | The servos never go beyond 45 degrees or faster than 300 degrees per second, whatever the PC sends | TFC-PLAT-001 |
| Timeout | No command for 100 ms holds; 1 s levels, at a limited rate | TFC-PLAT-002 |
| Saturation | A tilt beyond the travel is clamped and reported | TFC-PLAT-004 |
| Servo pulse | The right width, 20 ms period, little jitter, never outside 900 to 2100 us | PLAT-001 |
| Relay safety | Boot, reset and a crash leave every relay off (nodes powered); a cut ends by itself; a quiet PC releases everything | ADR-026, F75 |
| Link | Garbage, a split frame, a flood, a pulled cable never wedge the board or move the platform | `pico_link.hpp` |
| Watchdog | A hung loop resets the Pico and the relays release | PICO.md section 4 |
| The relay module | Each channel really switches at 3.3 V drive with the jumper removed; the contacts are the normally-closed ones | HARDWARE_PARTS.md section 7, item 7 |
| The rail | Twelve coils switching do not reset a healthy node | G7, F80 |
| A cut node | A cut node's pins sit at zero, not half-powered through a signal line | G8, F79 |
| The servo | What it does with no pulse; its stall current; the platform's shock when levelled or limp | HARDWARE_PARTS.md section 7, items 5, 6; G1, G2; F78 |
| The whole chain | The platform follows the simulated ascent with a latency and an error that are small enough for the IMUs to see a true attitude | VEHICLE_SIM section 6 |

## 2. Options that need no hardware (A)

| ID | Test | What it shows | Status |
|---|---|---|---|
| A1 | **Unit tests of the portable logic** (`tests/test_pico.cpp`): link, parser resynchronisation, driver limits, rate, hold, level, NaN, servo map, injector auto-release | The rules are right | **Done**, 100% lines, 98% branches |
| A2 | **Cross-language golden frames** (C++ and Python, an independent CRC) | The PC tool and the board speak the same bytes | **Done** |
| A3 | **The real main loop on the host (done: `tests/test_pico_app.cpp`, `core/include/tfc/pico_app.hpp`).** Move the loop's logic out of `main.cpp` into a class that takes the hardware as a small interface (read a byte, write bytes, set a servo pulse, set the relay mask, the time, feed the watchdog), and drive it in a unit test with a fake | The things only the loop does: the order of the steps, the 2 s link timeout, the status contents, the relays released on link loss, the flags | **Done** (4 Oct 2026): the loop moved out of `main.cpp` into `tfc::PicoApp<Hal>`; 15 tests against a fake board. `main.cpp` is now the glue only (the `Board` class), which no host test sees |
| A4 | **Property and fuzz tests**: random byte streams into the parser; random command sequences into the driver, checking the invariants (output inside travel, never faster than the rate, relays off after the longest time) | Cases nobody wrote | **Done** (`tests/test_pico_fuzz.cpp`): 2 million hostile bytes into the parser; every single-bit error in a frame rejected; 300,000 steps of the real loop under a random mix of good, extreme, damaged and garbage input, with eight properties checked every step against an independent model of the relays |
| A5 | **Mutation testing** of `pico_link.hpp`, `platform_driver.hpp`, `injector.hpp` (`tools/mutation/`) | The tests would notice a changed limit or timeout | **Done** (`tools/mutation`): 40 Pico mutants; 36 killed by the first tests, the other four showed three real gaps and one equivalent mutant; all 39 non-equivalent now killed (TS-18) |
| A6 | **The `native_sim` build of the Pico app** with Zephyr's emulated GPIO, a fake PWM and the UART on a pseudo-terminal | The real application, not a copy, on the host. Needs a way to read the emulated pins and pulses from outside, which means a small test-only debug message | Option; A3 gets most of the value at less cost |
| A7 | **An emulator of the RP2350** (QEMU or Renode) | The machine code | **Not checked.** I have not verified that either runs an RP2350 Zephyr image with USB and PWM; treat as unknown |
| A8 | **Live pseudo-terminal test** (`tfc_simd --pico` into a pty) | The stream from the simulator is well formed and rolls its sequence | **Done** (`test_live_pico_stream.py`) |
| A9 | **Build gates**: the ELF check (no heap, vtables), warnings as errors, size | Nothing forbidden crept in | **Done**, in CI |
| A10 | **A fake board in Python** (a serial port object that behaves like the Pico: runs the same rules in Python and answers status) | The PC tools and scripts without the board | Option; it would duplicate the C++ rules, so A3 is better |

## 3. Options with the Pico alone, on the desk (B)

Needs the board, a USB cable, the logic analyzer and the multimeter.

| ID | Test | Measure | Pass (proposal) |
|---|---|---|---|
| B1 | **Enumeration**: plug in, `tfc_peers pico status` | A serial port appears; a status comes back | Answers within 1 s, every time over 20 plug-ins |
| B2 | **Status rate and loss**: stream platform commands at 100 Hz for 10 minutes | Status frames per second, parser bad frames, drops | 50 Hz status, zero bad frames |
| B3 | **Servo pulse**: logic analyzer on GP2 and GP3 | Period, width against the commanded tilt, jitter | 20 ms period, width within 2 us of the map, jitter under 10 us peak to peak |
| B4 | **Relay pins at boot, reset and crash**: logic analyzer or meter on GP10 to GP13, with the external pull-ups | The pins are high (relay off) from power-on, through USB reset, through a watchdog reset, with the app crashed (a deliberate hang) | Never low except by command |
| B5 | **Link timeout**: start a cut with `pico cut B 10000`, then kill the tool | The pin goes high within 2 s of the last byte | Within 2.1 s |
| B6 | **A cut ends by itself**: `pico cut A 500` | The pin low for 500 ms and no longer | 500 ms plus or minus 20 ms |
| B7 | **Watchdog**: build a variant that stalls the loop (a test-only command) | The Pico resets, the relays are off, the status shows the watchdog flag | Reset within 100 ms plus the boot time |
| B8 | **USB unplug and replug** during a stream | The board keeps its outputs sane, comes back, no stuck state | Level and relays off during; recovers on replug |
| B9 | **Garbage and flood**: random bytes, 0xA5 storms, a 10 Mbyte stream | No wedge, no output change, parser counters rise | The outputs never leave the commanded values |
| B10 | **Soak**: 8 hours with the stream running | Memory, counters, timer wrap, thermal | No reset, no drift in the status |

## 4. Options with the relay modules (C)

Needs the relay modules, 5 V rail and the meter. No node connected yet.

| ID | Test | Measure | Pass |
|---|---|---|---|
| C1 | **Each channel at 3.3 V drive** with the `VCC`/`JD-VCC` jumper removed, `VCC` from the Pico's 3V3 | The relay pulls in on a low, releases on a high, 20 times each; the contact state with a meter | All channels, all times; normally-closed contact closed when off |
| C2 | **A floating pin**: unplug the Pico | Every relay off | Off |
| C3 | **Switching time**: logic analyzer on the input and on a contact | Latency from command to contact, bounce | Recorded: sets the minimum meaningful cut time |
| C4 | **The rail**: oscilloscope on the node rail while all twelve coils switch together and one by one | Minimum voltage and recovery | Inside 4.75 to 5.25 V (G7) |
| C5 | **Coil inrush on the Pico's 3V3** (the opto LEDs are fed from it) | The Pico's 3V3 rail under all twelve | No brown-out of the Pico |
| C6 | **Contact life**: 10,000 cycles on one channel with a dummy load | Contact resistance after | Within spec of the module |

## 5. Options with a node connected (D)

Needs Nucleos on the bench: the Pico cuts a node and the system reacts.

| ID | Test | Measure | Pass |
|---|---|---|---|
| D1 | **Cut each node** (A, B, C, ACT), 2 s each | The others latch it within 3 frames, keep flying; the cut node comes back by itself and rejoins through probation | F01, F17 on hardware |
| D2 | **Cut A (the sync master)** | B takes over SYNC, frame numbers continuous (the live triplex test, on hardware) | F15 on hardware |
| D3 | **A cut node's pins**: meter and scope on the CAN transceiver, IMU, SPI lines, ST-LINK | Zero volts, no current into a dead node | G8, F79 |
| D4 | **Cut latency**: the first frame missed after the relay moves | Time from the relay command to the first missing frame | Recorded |
| D5 | **Cut during a vote**, at each phase of the frame | The system degrades as designed, no bad data reaches ACT | Safety properties of the campaign hold |
| D6 | **Injector and supervisor together** (after the supervisor exists) | Priority and conflict (G9) | The supervisor gives up after its limits and reports `DEAD` |
| D7 | **The disarm switch** (H4, if built) | Open it during a cut: the node returns at once | Powered |

## 6. Options with the servos and the platform (E)

Needs the servos, their rail and the platform. Do these with the E-stop in reach.

| ID | Test | Measure | Pass |
|---|---|---|---|
| E1 | **The servo with no pulse** (Pico unplugged, or a watchdog reset) | Holds, drifts or goes limp | Recorded: sets whether PLAT-002's "hold" needs a pulse |
| E2 | **Does the servo follow a 3.3 V signal** | Motion against command | Else the 74AHCT125 |
| E3 | **Stall current** at the hard stops, both axes | Peak and sustained, against the 4 A adapter and 5 A fuse | Under the adapter's limit (section 7, item 5) |
| E4 | **Step and ramp**: command 0 to 30 degrees | Measured angle against the Pico's output, with a camera or the IMU | Follows the rate limit; no overshoot beyond the servo's own |
| E5 | **Saturation**: command 60 degrees | The platform stops at 45 and the status flags it | PLAT-004 |
| E6 | **Timeout on the real platform**: stop the stream | Holds at 100 ms, goes level in 1 s at 30 degrees per second | PLAT-002 |
| E7 | **The E-stop**: press it at 20 degrees | The platform's motion, shock, the IMU reading | G1: recorded, with the hard stops |
| E8 | **The level override** (H3, if built) at 5, 20, 40 degrees | The snap and the current | G2: recorded |
| E9 | **Backlash and calibration**: command a sweep, read the IMU | The platform's true angle against the command; the trim and sign of each axis | The servo map's trim and scale |

## 7. Options for the whole chain (F)

| ID | Test | Measure | Pass |
|---|---|---|---|
| F1 | **Latency** from `tfc_simd`'s frame to the platform's motion: logic analyzer on a CAN frame and on the servo pulse | Time | Recorded; decides how much of the control loop's phase lag the platform adds |
| F2 | **Tracking**: the simulated ascent through the platform, IMUs on the platform, three flight computers and ACT live | The attitude the IMUs measure against the vehicle model's tilt | Within a small fraction of the loop's own error |
| F3 | **Closed loop with the real IMUs** (the milestone S2 test) | The vehicle holds the pitch program through the platform | The rig's version of `test_live_closed_loop.py` |
| F4 | **Injected faults with the real platform**: cut a node mid-flight, an engine-out, a lost ACT frame | The platform follows the changed flight; nothing leaves its limits | The campaign's safety properties |
| F5 | **A PC crash mid-flight**: kill `tfc_simd` | The platform holds, then levels (PLAT-002); the injector releases | As the rules say |

## 8. Instruments, and what can be done without them

| Need | Best | Cheaper |
|---|---|---|
| Pulse width and jitter, relay timing | The logic analyzer (owned) | |
| A rail's dip and a cut node's pins | An oscilloscope (not owned; borrow) | **The Pico's own ADC** on a divider (5 V to 3.3 V full scale) samples a rail at tens of kHz: enough to see a droop, not a spike; **the second Pico** (the supervisor's, before its firmware exists) as a **test jig** that senses each node's feed through the relay contacts with a divider and timestamps every cut: automated, repeatable, and it measures D4 and C3 |
| The platform's real angle | An IMU on the platform (they are there) or a phone camera | A protractor and a fixed camera |
| Servo current | A bench supply with a current display (the KORAD, optional) or a USB power meter in the 5 V line | A multimeter in series |

## 9. Automation

The B, C, D and E tests are to be scripted in `sim/tests/bench_pico.py` (not written yet; one function per row, an identifier in each, a log line with the result and the date), skipped unless the port is given
(`TFC_PICO_PORT`). A pre-session check runs the quick ones (B1, B4, B6, C1 with a meter prompt, the override checks of `HARDWARE_OVERRIDE.md` section 7) and writes a report, in the style of the
procedures in `docs/procedures/`. The scripted ones need the board; the report is the evidence for the fault matrix rows.

## 10. Recommended order

1. **A3, A4, A5: done.** What no host test sees now is the glue in `firmware/pico/src/main.cpp` (pin numbers, the devicetree overlay, the PWM period, the USB descriptors): that is what B1 to B6 and C1 to C3 are for, and the option A6 would reach some of it (TS-18).
2. **B1 to B6 and C1 to C3** the day the Pico and the relay modules arrive, with the meter and the logic analyzer. These are cheap, short, and decide whether the design's assumptions (3.3 V drive, pull-ups, enumeration) hold.
3. **E1 to E3** next, before the platform is ever driven with the Pico's stream: the servo's behaviour with no signal and its stall current decide the mechanical safety.
4. **D1 to D4** with the Nucleos; **C4, C5, D3** need a scope or the jig.
5. **E4 to E9, then F1 to F5** with the platform: the measurements that tell what the chain adds.
6. **B7 to B10** (watchdog, replug, flood, soak) in the background while other work goes on.

What I would **not** spend time on yet: an RP2350 emulator (unverified, and A3 covers the logic), and C6 (contact life) until the injector is used enough to matter.
