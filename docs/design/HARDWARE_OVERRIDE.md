# Hardware override: switches that work when every program is down

> Status: **decided (TS-17: option O3), the software side built, the hardware not ordered or built** (ADR-027; reviewed 5 Oct 2026). The set is H1 (the E-stop, on the parts sheet) and H2 to H5; H6 and H7 are not built (`../decisions/TRADE_STUDIES.md` section 9c). The supervisor's sense inputs and the launch guard are built (`SUPERVISOR.md`); the parts are about 12 USD and are **not in the order placed for 9 Oct**. Every number is a proposal to be confirmed on the rig.
> Written after the owner's question of 4 Oct 2026: *"How do we have a hardware relay that can override the software and make a command to the hardware itself, and how many of those are needed?"*

## 1. What it is, and how it differs from the supervisor

The system has three layers of authority, from the most capable to the most basic:

| Layer | Made of | Can fail by | Lives in |
|---|---|---|---|
| 3. Flight software | the flight computers, ACT, the simulator, the Pico platform driver | bugs, hangs, wrong values | `core/`, `firmware/` |
| 2. Supervisor (ADR-022) | a second Pico 2 with its own small program, discrete lines to the nodes | a bug in its program, a hang, a spurious action | `docs/design/SUPERVISOR.md` |
| **1. Hardware override** | **switches, relays driven by switches, fuses, diodes: nothing that runs code** | a contact that does not make, a wire that breaks, a person pressing the wrong one | this page |

The supervisor is software too, only smaller. The lecture's "lizard brain" survives a hung flight computer, but a hung or wrong supervisor
is still possible, and so is a bug that is the same in every program the project wrote. Layer 1 is what remains **when all of them are down or
wrong**. Its properties:

1. **No code.** A person moves a switch; contacts move; the hardware changes state. Nothing depends on a processor running.
2. **It has priority.** Manual overrides beat the supervisor, which beats the software (section 5). A program cannot undo a switch.
3. **It is fail-safe in its wiring.** A broken wire, a lost supply or an unplugged cable must leave the system in the state that is safe for
   *that* function (section 4), not in a state that silently disables the protection.
4. **It is tested.** An override nobody has operated since it was wired is a switch that may not work. Section 7.

"A hardware relay that makes a command to the hardware itself" is two things in practice, and they should not be confused:

- **A switch in the power or signal path** (a cut, a disconnect). The simplest and the most reliable: the switch *is* the override.
- **A relay whose coil is driven by a switch, not by a program.** Its contacts select the *source* of a signal (the Pico's, or a fixed value) or open
  a line. It is worth its cost only where a plain switch cannot do the job: several poles at once, a remote panel, isolation between a
  switch and a logic input, or a latching state with an indicator. For a desk rig a plain switch does most jobs; a relay is a switch with more
  poles and a coil that can fail.

## 2. What exists already, at no cost

The parts list (v4) and the boards already carry manual controls. They count as overrides:

| Control | What it does | Limit |
|---|---|---|
| **E-stop, 22 mm latching, 2 NC contacts** (parts row 26; TFC-PLAT-003) | Cuts the servo rail, after the fuse, independently of every firmware | The platform goes limp (section 6, G1). It does not stop the simulator or the flight computers |
| **Reset button on each Nucleo** | Resets that one node by hand (a manual `NRST`) | Resets only; the node comes back and rejoins through probation |
| **Pluggable CAN terminal block per node** (parts row 16) | Unplugging one disconnects a node's stub from the bus: a manual bus isolation of a babbling or hung node | Needs the bus to stay terminated (the terminations are on the backbone board, not on the stubs) |
| **The wall adapters' plugs** (rows 18, 19) | Cut the whole node rail or the whole servo rail | Cuts everything on the rail |
| **USB hub port switches** (row 27) | Cut a Nucleo's USB (debug and logging) | **Do not** cut a node powered from E5V (compatibility audit item 1); manual, not scriptable |

## 3. Candidate overrides (what could be added)

Costs are rough. None of these has been priced on a listing; each is a common part.

| ID | Override | How it is wired | Covers | Rough cost |
|---|---|---|---|---|
| **H1** | Servo-rail E-stop | Exists (row 26) | Platform runaway, a hung Pico, anything that must stop moving | in the list |
| **H2** | **FORCE-SAFE** toggle (guarded) | Asserts ACT's `SAFE` input; OR-ed with the supervisor's `SAFE` by two diodes, so either can assert it and neither can pull the other's line down | The flight computers' commands are wrong but ACT is running: ACT freezes, holds, ramps to neutral (`SAFE_MODE.md`) | 2 USD |
| **H3** | **PLATFORM-LEVEL** switch | A double-throw switch on each servo signal: the Pico's PWM, or a fixed neutral pulse from a no-code source (a servo-tester board, or a 555 timer) | A hung or runaway Pico or simulator, ACT or flight computers driving the platform to its stops, **with the servos still powered and holding** | 8 USD |
| **H4** | **INJECTOR-DISARM** key or toggle | In series with the injector relay module's coil supply (`JD-VCC`). Open: every coil is off, every normally-closed contact is closed, every node is powered, and **no program can cut anything** | A Pico that crashed with a relay output held on; a welded driver; any bug in the injector that cuts a node when it should not | 1 USD |
| **H5** | **SUPERVISOR-DISARM** toggle | The same, for the supervisor's four `PWR` relays | A supervisor that resets or power-cycles a healthy node in a loop | 1 USD |
| **H6** | **Per-node POWER-KILL** (four, one per node) | A switch in series with each node's feed, after the relays | A manual node loss, independent of the Pico and the supervisor; the hardware way to cause F01 and F15 | 4 USD |
| **H7** | **MASTER-POWER** | A switch in the node rail feed | Everything on the node rail at once | 1 USD |

**Decided (TS-17, 5 Oct 2026): build H1 to H5, not H6 and H7.** H1 is on the parts sheet; H2 to H5 are about 12 USD. The per-node kills and the master switch cover no scenario that the Nucleos' reset buttons, the unpluggable CAN stubs and H4 do not already cover (the coverage matrix and the model are in `../decisions/TRADE_STUDIES.md` section 9c, `python3 -m campaign.ts17`). The supervisor's sense input bit 5 ("any node power-kill") is reserved for them and unfitted.

Two ideas that were considered and not proposed:
- **A bypass switch that closes a relay's contacts by hand** to force a node on. It defeats the protection it bypasses and can power a node
  the supervisor is holding in reset on purpose. H4 and H5 reach the same goal (everything powered) in the safe direction, by removing the cause.
- **A switch that forces the vehicle to level without the servos** (a mechanical prop). Not a switch; a hard stop inside the servo travel
  (already in the parts audit, item 12) does that job.

## 4. Fail-safe wiring: what each function does when its wire breaks

The rule is that **a break must leave the function in the state that is safe for it**, and the safe state differs:

| Function | Safe state | Wiring that makes a break go there |
|---|---|---|
| Servo rail | unpowered | E-stop contacts normally closed, in series: a broken wire opens the rail (already so) |
| Node power | **powered** | Normally-closed relay contacts; coil energised = cut. A lost coil supply restores power. The cut is the deliberate action, the default is "on" |
| `SAFE` to ACT | not asserted (a floating line must not safe the vehicle by itself) | An external pull-down; the switch pulls it high through a resistor. A broken wire means "no override", **which is the loss of protection**, so H2 needs a periodic test (section 7) |
| Platform signal source | **the neutral source** if the switch fails | Wire the switch so that its *unpowered or broken* position is the Pico (the normal one); the neutral position needs a positive action. A broken wire then loses the override and not the platform, which is why it is tested, not assumed |

One case has no clean answer: a function where both states are unsafe. A node cut during the vote costs availability (Triplex becomes Duplex);
that is the designed degradation, not a hazard to the rig. The servo case matters more because it is mechanical (section 6).

## 5. Priority and conflicts

**Manual beats supervisor beats software.** The wiring must make that true, not merely intended:

- H4 and H5 are in the *coil supply*, so no output pin can energise a relay whatever the program does.
- H6 is in the node feed *after* both relay banks, so a closed relay cannot restore power that a switch has cut.
- H2 and the supervisor's `SAFE` are diode-OR-ed: either asserts, neither can release the other.
- H3 selects the servo signal at the servo; the Pico cannot reach it.

The reverse holds for restoring: a manual cut is lifted only by hand. The supervisor sees a node it did not cut go dead, tries its resets and
power-cycles, gives up after its limit, and reports `DEAD` (`SUPERVISOR.md` section 5): the report is correct, the node *is* dead, and the
operator knows why. To avoid a confusing report, each override has a **read-only sense line** to the supervisor (an input with an external 4.7 kOhm
pull-down, the RP2350 erratum E9; six inputs on GP22, GP26, GP27, GP28, GP0 and GP1), which reports its changes to the PC, refuses a `launch` while one is engaged until the operator types `override-ok`, and lists the overrides not yet seen engaged and released this session when a countdown starts. The sense line is never an input to any control decision (TFC-HWO-005; a test runs two supervisors with and without every override engaged and compares their outputs step by step).

## 6. What could go wrong

The overrides are there to prevent harm, and each of them can also cause some. The table is the first pass at their failure analysis
(it feeds TS-17 and the fault matrix, F75 to F80).

| # | Failure | Effect | Mitigation | How to test |
|---|---|---|---|---|
| G1 | **E-stop makes the platform limp.** The servo rail is cut; the platform drops or swings to a stop | A mechanical shock; the IMUs see a large, fast motion that the flight computers read as a fault | Hard stops inside the servo travel (audit item 12); a platform that is balanced about both axes; H3 as the gentler override | Cut the rail with the platform at 20 degrees and watch it |
| G2 | **H3 snaps the platform to level** with no rate limit (the Pico's limit, TFC-PLAT-001, is bypassed) | The servo moves at its own speed (about 60 degrees in 0.17 s): a shock, a stall current spike (audit item 12) | Use it only near level, or add an RC-filtered neutral; measure the snap on the bench first | Engage H3 at 5, 20 and 40 degrees and measure the current |
| G3 | **An override fails to act** (a contact that does not make, a cold joint, a broken wire) | The protection is not there when needed: the dangerous failure, because it is latent | A test before every session (section 7); NC wiring where a break goes to the safe side (E-stop); a sense line that shows the switch state | The pre-session check |
| G4 | **A spurious activation** (an accidental touch, bounce, vibration) | A lost run; a node cut for no reason | A guard or key on H2, H4, H5; a latching switch with a visible state; no override within reach of the platform's motion | Operate each with a deliberate brush |
| G5 | **Mode confusion**: an override left engaged | The rig seems dead, or a test seems to pass because the fault was never injected (H4 open means the injector cannot cut) | The sense lines mark the run; the PC refuses to start a run with an override engaged unless told | Start a run with H4 open and check the refusal |
| G6 | **The injector's relay stays on** after the Pico crashed with an output held, or a driver failed | A node stays unpowered and the test looks like a node fault | External pull-ups so a floating or high-impedance pin means "off" (as the supervisor's `PWR`); the Pico's watchdog; H4 | Reset the Pico with a relay on and check the node comes back |
| G7 | **Rail droop from the relay coils.** Eight coils at 70 mA are 0.56 A on the node rail that also feeds the Nucleos through `E5V` (limit 4.75 to 5.25 V) | A coil switching on makes a healthy node brown out and reset: a fault the test did not intend | The 1000 uF capacitor on the node rail (parts row 22); coils from the same 4 A adapter; measure the rail while all eight switch | Scope the rail on a switch of all eight |
| G8 | **A cut node is not dead.** The `ST-LINK` stays powered over USB (audit item 1), the CAN transceiver and the IMU share lines with powered neighbours | A half-powered node: its pins pushed through protection diodes by a neighbour, an odd brown-out state, a node that comes back wrongly | Cut the transceiver and the IMU with the node; check a cut node's pins at zero volts; compare with a real power loss | Meter every signal of a cut node |
| G9 | **Two authorities fight.** The supervisor power-cycles a node an operator switch has cut; or the injector cuts a node the supervisor holds in reset | Confusing logs, wasted cycles, `DEAD` reports | The priority rule of section 5; the sense lines; the supervisor's limits | Cut by hand and watch the supervisor give up |
| G10 | **A common cause.** One 5 V adapter feeds every node, every relay coil and the Pico; one star ground ties the rails | An override on that rail cannot help when the rail fails; a ground fault disturbs every signal | A separate switch and fuse per rail; the two rails stay separate (the parts list already does this for the servos); a common ground at one point (audit item 26) | Unplug the node adapter and check what is left alive |
| G11 | **A sneak path through a signal line.** A relay or switch that opens a power line leaves a signal line connected, and the cut node is fed from it | A node that does not turn off | Open signal lines as well where it matters; the check of G8 | As G8 |
| G12 | **The override works and hides a problem.** A person gets used to pressing H2 | The software's own Safe logic is never exercised | Log every use; the run is marked; review the log | Count uses per session |

Not a failure but a limit: H2 is a command **to ACT**, so it needs ACT's program to be running. If ACT itself is down, the simulator holds the
last command (`command held`) and the platform goes where the vehicle goes. That case is H1 or H3, which act on the platform and need nothing
of ACT.

## 7. Testing the overrides

A pre-session check (a procedure, `docs/procedures/`, to be written with the rig):

1. Servo rail: press the E-stop; measure 0 V at the servo; release.
2. H3: switch to neutral with the platform at 10 degrees; check it goes to level; switch back; check the Pico regains control.
3. H2: with ACT running, switch; check ACT reports `SAFE` with cause "hardware line"; release; check it stays Safe until an operator `clear-safe` (the design, `SAFE_MODE.md`).
4. H4: open; command the injector to cut a node (it must have no effect); close.
5. H5 likewise; H6: cut each node and check the supervisor reports it.
6. The sense lines: each one reads right in each position.

Each result is a log line with a date. An override that has not been tested in a session is reported at the start of the next run.

## 8. Cost and parts

H4 and H5 need a toggle or a key switch each (1 USD); H2 two diodes and a guarded toggle (2 USD); H3 a double-pole double-throw switch per servo and a no-code neutral source (a servo-tester board costs a few dollars; 8 USD in all, **not priced on a listing**). The decided set (O3) is about 12 USD of new parts,
against a budget that is already 77.81 USD over its ceiling (parts sheet, Budget). H6 and H7 (about 5 USD) are not built. **None of these parts is in the order placed for 9 Oct**: see `../hardware/HARDWARE_PARTS.md` section 4.

## 9. Relation to other documents

`SUPERVISOR.md` (layer 2, and the `PWR`, `NRST`, `SAFE` lines this layer works beside); `SAFE_MODE.md` (what ACT does when `SAFE` is asserted);
`VEHICLE_SIM.md` (the platform and PLAT-001 to 004); `FAULT_MATRIX.md` (F75 to F80); `TRADE_STUDIES.md` (TS-17); `HARDWARE_PARTS.md` (what the parts list has).
