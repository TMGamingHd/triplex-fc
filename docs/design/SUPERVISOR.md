# Supervisor ("lizard brain"): proposal

> Status: **accepted 4 Oct 2026 as SUP-Lite** (ADR-022); SUP-Full stays an upgrade. Nothing here is built. Every number is a proposal to be confirmed on the rig.
> The order must include a second Pico 2, a TCXO clock module and resistors before it closes (M0, 6 Oct 2026); see section 9.

## 1. What it is and why

A small, separate computer whose only job is to watch the flight computers and the actuator node, to hold the watchdog that does not
depend on their software, to keep an accurate clock of its own, and to be able to reset or power-cycle any of them. In the ASTE-331
avionics lecture this is the "lizard brain": the brainstem that keeps working when the brain is hung. Its three properties there are the
ones designed for here:

1. **It runs on its own hardware**, so a hung or misbehaving flight computer cannot stop it.
2. **It has hardware commands that bypass the flight software** (the lecture's example is a hardware system reset on the same uplink).
3. **It keeps an independent copy of time**, to seed or check the computers' clocks after a reset.

Why the flight computers cannot do this for themselves: the per-node watchdog and the frame-deadline monitor (`ARCHITECTURE.md`
section 5) live in the software they protect. On the Mars rover's Sol 200 anomaly enough tasks stayed alive to keep servicing the
watchdog while others were hung, and the recovery was a hardware command sent from outside the flight software. The supervisor is the
rig's version of that outside.

## 2. Principles

- **Independent.** A different processor family from the flight computers (RP2350 against STM32G4), its own oscillator, its own firmware
  with nothing shared with `core/`, and power taken from the rail upstream of every relay the supervisor or the fault injector controls.
- **Simple.** One loop, no RTOS, no voting, no estimator. Small enough to be read in an afternoon and tested to 100% branch coverage.
- **Acts only through discrete lines and USB.** Never through the flight bus: a bus fault must not blind or disable it, and it must not
  be able to cause one. (The full variant, section 3, listens to the bus but still never transmits.)
- **Fail-passive.** A supervisor that is unpowered, in reset or hung leaves every node running and the actuator selection at its default.
  It loses protection, never function.
- **Never takes a voting decision.** It acts on a node that is silent, hung, looping through resets, or that the operator names. Whether a
  working node's data are good stays the flight computers' job (ADR-010).

## 3. Two builds

| | **SUP-Lite** (recommended for v1) | **SUP-Full** (upgrade) |
|---|---|---|
| Hardware | 1 Raspberry Pi Pico 2 (6.00 USD on the parts list) + a TCXO clock module | 1 Nucleo-G474RE (20.13) + CAN Pal (3.95) + TCXO module |
| Sees the nodes through | Two discrete lines per node (frame start, end-of-frame kick) | The same lines, plus a listen-only tap on the CAN bus |
| Can do | Watchdog, resets, power-cycles, hardware commands, independent clock, clock-drift and phase check of every node, boot sequencing | All of Lite, plus arrival-time margins per stream (the telemetry of TFC-FDIR-037, from outside the flight computers), SYNC frame-number plausibility, ACT output presence |
| Cannot do | Anything that needs the contents of bus frames | Nothing more is needed |
| Independence from the bus | Complete | Partial: it can see the bus, but acts through lines |

Lite is enough for everything the requirements ask of the supervisor except TFC-SUP-005 and the frame-number half of TFC-SUP-009, which
are marked as Full-only. The Pico 2 has no CAN controller, which is why Lite has no bus tap; the RP2350 erratum E9 already noted in the
parts sheet means every input line needs an external pull-down of 8.2 kOhm or less.

## 3a. Clock
> Extended 4 Oct 2026 (ADR-029, `docs/design/MISSION_CLOCK.md`): the supervisor keeps the **clock of record**, mission elapsed time across resets and years, on a battery-backed oscillator, correlated against the PC's UTC, and gives time to the nodes by a pulse per second and a one-way serial line. The paragraph below is the original frame-clock check.

The supervisor's reference needs to be better than the nodes' (the Nucleo's crystal is in the tens of ppm). A TCXO real-time-clock
module such as a DS3231 (specified at about 2 ppm at room temperature, with a 32.768 kHz output; **price and exact figures not checked**,
verify before ordering) feeds the Pico's counter. With a 32.768 kHz reference, 100 frame periods (1 s) give a resolution of about 30 ppm,
10 s about 3 ppm. That is enough to see a node whose frame clock is off by a few hundred ppm, which is what the `clockdrift` fault does
(F45), and to see a node drift relative to the others.

## 4. Interfaces

Per flight computer (A, B, C) and for ACT:

| Line | Direction | What it is |
|---|---|---|
| `FRAME` | node -> SUP | A pulse at the start of every frame, driven from the node's frame timer (aligned to SYNC): it rises at the start of the frame and falls when the IMU sample is latched, 0.5 ms later (implemented in `firmware/app/src/main.cpp`). The supervisor measures period against its clock and phase against the other nodes |
| `KICK` | node -> SUP | A pulse at the end of a **completed** frame (it rises there and falls at the start of the next frame; no rising edge means a refused frame), driven only from the end-of-frame path after the vote has run and every monitored task has reported progress (TFC-FDIR-038). This is the watchdog |
| `NRST` | SUP -> node | Open-drain on the board's reset pin; pulled low to reset the node |
| `PWR` | SUP -> relay channel | A relay in series with the node's 5 V feed, in addition to the fault injector's own relay. De-energised means powered |

For each IMU, in the stretch of ADR-020 (ring re-homing, after S4): one `ADOPT` line (SUP -> the IMU's bus switch and its backup host). Not built in v1; the pins are reserved.

For the launch sequence (ADR-028, proposed): one `T0` line from the supervisor to every flight computer, asserted when the supervisor decides T-zero; the sync master latches it. The supervisor also keeps a mission clock from that edge and checks the system's mission time against it. It is not the runtime source of mission time (`LAUNCH_SEQUENCE.md` section 3).

For ACT, two more: `SAFE` (SUP -> ACT, a hardware "enter Safe now") and, only if a second ACT is built (ADR-023), `SEL` (SUP -> the
output selector). To the PC: one USB serial port (the hardware commands, section 6, and telemetry).

About 17 of the Pico's 26 GPIO. The 4 spare channels of the second relay module in the parts list (8 channels bought, 4 used for the
injector) can carry the `PWR` lines, so no relay parts are added. The ELEGOO module's inputs are active-low and a floating pin must not
pull a relay in: the supervisor's `PWR` pins need external pull-ups to 3.3 V (and the check at 3.3 V drive in the compatibility audit,
item 7, applies).

**Channel allocation (decided 4 Oct 2026, ADR-026; `HARDWARE_PARTS.md` item 1).** The two relay modules give 8 channels: 4 for the injector's power cuts (A, B, C, ACT) and 4 for these `PWR`
lines. The parts sheet also counts those four as the injector's sensor-line and bus-stub faults; both uses cannot have them, so a **third relay module** (6.99 USD) is proposed for the sensor
and bus faults. Each relay bank's coil supply gets a disarm switch (`HARDWARE_OVERRIDE.md`, H4 and H5), so no program can cut a node when the switch is open.

The fault injector keeps its own relays. Injected power cuts therefore stay a test action, and the supervisor's actions stay a protection
action; the two are in series so neither can undo the other. When the injector has cut a node's power the supervisor will see it dead,
will try its resets and power-cycles, and will give up after the limit in section 5 and report the node as `DEAD`.

## 5. What it watches and what it does (all numbers proposals)

| Condition | Action | Then |
|---|---|---|
| `KICK` missing for 3 frames (30 ms, matching FDIR-001) | Pulse `NRST` for 100 ms | The node boots; it rejoins through the normal path (FDIR-042, probation) |
| 3 resets of one node inside 60 s | Power-cycle it (relay open 500 ms) | As above |
| 2 power-cycles inside 5 min | Hold the node in reset and report it `DEAD` | Rejoins only after an operator `release` |
| `FRAME` period of a node off by more than 200 ppm from the supervisor's clock, averaged over 10 s | Report only | The flight computers' own checks decide; the supervisor never isolates a working node |
| `FRAME` phase of two nodes differing by more than 200 us | Report only | As above |
| No node has produced `FRAME` or `KICK` for 3 frames (total loss of the flight computers) | Report; staggered resets as above | ACT, seeing no votes, enters Safe by itself (SAFE-002) |
| ACT `KICK` missing for 3 frames | Pulse ACT's `NRST` (then as for a flight computer); if a second ACT exists, switch `SEL` | |
| An operator hardware command | Execute it (section 6) | |

## 6. Hardware commands (USB serial, executed without any flight computer)

`reset X`, `cycle X`, `hold X`, `release X` (X is A, B, C or ACT), `safe-now` (asserts the `SAFE` line), `sel 1|2` (only with a second ACT), `adopt X` / `unadopt X` (only with the ring re-homing: hold the computer that hosts IMU X in reset, then assert or release its `ADOPT` line),
`status`. Each is answered and logged with the supervisor's time. There is no authentication: the supervisor's USB port is a physical
port on the bench, and the PC is a trusted peer (ADR-019's key protects the bus, not this). This is the rig's equivalent of the hardware
commands in the lecture that bypass flight software.

## 7. Boot sequencing

The supervisor powers up first and holds every node in reset. It then releases them in a configurable order and spacing (default ACT,
then A, B, C at 0.5 s intervals). This makes the boot-order and startup-grace behaviour (`ARCHITECTURE.md` section 4) reproducible instead of
depending on who was plugged in first, and it is how COLD and WARM nodes are brought up in `MISSION_PHASES.md`.

## 8. When the supervisor itself fails

| Failure | Result |
|---|---|
| Unpowered or in reset | Relays de-energised (pull-ups), `NRST` released, `SAFE` low: every node runs, nothing is protected. Detectable: the PC loses its USB port and the supervisor's heartbeat line to ACT (if built) stops |
| Hung | Its own RP2350 watchdog resets it, and it comes up holding nothing (default outputs) |
| Wrong action (a spurious reset or `SAFE`) | One node resets and rejoins, or the output goes to Safe and waits for an operator. Both are the safe direction. The supervisor is therefore the one device that can cost availability, which is why it is kept this simple |

It cannot take the system below what the flight computers do alone, because it has no vote and no bus transmit.

## 9. Hardware changes and cost

All prices are from the parts sheet (checked 2026-09-29) unless marked.

| Change | Cost | Note |
|---|---|---|
| Second Pico 2 (SUP-Lite) | 6.00 | **Added to the order 4 Oct 2026.** The existing Pico is the fault injector and must stay separate |
| TCXO RTC module | not checked, small | **Added to the order 4 Oct 2026**; DS3231 or similar; confirm the output and the price |
| Resistors (pull-ups, pull-downs) | owned (owner, 4 Oct 2026) | |
| Relay channels | 6.99 | the 4 spare channels of module 2; **a third module is added** for the injector's sensor and bus faults (ADR-026) |
| **SUP-Full instead of Lite** | +18.08 (Nucleo 20.13 + CAN Pal 3.95, in place of the 6.00 Pico) | Needs a CAN tap on the backbone board |

The sheet's budget page already reads 677.81 USD (with the 8% allowance) against a 600 USD ceiling, over by 77.81. Anything added widens
that gap; the decision on what to cut is yours. The cost of this item is small, and it is the one that makes the other items testable.

## 10. Build and test plan

Stage S2b (after FC-A and ACT work): the supervisor on a perfboard with the lines to FC-A and ACT. Then each node as it joins. Tests: F57
(partial hang), F58 (a hung node), F66 (the supervisor failing), F67 (a slowly drifting node clock), F68 (a loop of resets). The
supervisor firmware is separately built, separately tested on the host (its decision table is plain logic and is tested like `core/`),
and its source is kept out of `core/`.

## 11. What was deliberately left out

- **The supervisor as the time master (a central bus guardian).** It would remove the sync-master takeover but make the supervisor a
  single point of failure for time. SYNC stays with the flight computers; the supervisor only measures it.
- **Transmitting on the bus** (a TIME frame, health frames). Would make it able to babble. Revisit once it has run for a while.
- **Authenticated hardware commands.** Not needed on a bench.
- **Redundant supervisors.** The supervisor is fail-passive by design.
