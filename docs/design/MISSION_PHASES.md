# Mission phases: which computers run, and how (hot, warm, cold)

> Status: **built on the host and in the firmware, not run on a board** (ADR-023; reviewed 5 Oct 2026). The phase table, the `phase`, `warm` and `noop` commands, the WARM role and the phase-aware interlock are in the fault manager (`RedundancyConfig::phases`; `CONFIG_TFC_PHASES` in the
> firmware); the Safe action per phase and automatic cold standby are not built (section 5). Numbers are proposals.

## 1. Why phases

Redundancy has a price: a computer that is powered and voting is exposed to wear, to radiation and to a shared bug for as long as it runs,
and costs power. A real spacecraft keeps all its computers running only when a fault cannot be waited out, and runs fewer, with the rest
cold or warm, the rest of the time. The lecture's criterion for "cannot be waited out" is the list that asks for fail-operational:
a pointing constraint that protects the hardware, crewed flight, and an event that cannot be repeated (orbit insertion, landing). For the
rig the equivalent is a burn or an ascent. Everywhere else it is cheaper and, for a demonstration, more instructive to run fewer nodes and
show the system bringing the others in.

## 2. The three roles

| Role | Powered | Software | Votes | Output used | Time to become hot | Reference |
|---|---|---|---|---|---|---|
| **HOT** | yes | running | yes | yes | n/a | hot standby: same inputs and status, any one can take over at once |
| **WARM** | yes | running, current inputs, shadow-voted | no | no | the probation length (100 frames, 1 s) | warm standby: ready, its state not necessarily the one in use |
| **COLD** | no (held in reset or unpowered) | not running | no | no | boot, startup grace and probation (a few seconds) | cold standby: off until needed, zero wear |

**Role is separate from health.** Health (Healthy, Latched, Probation, Disabled; ADR-010) is what the fault manager decides. Role is what
the mission phase decides. A WARM node is not faulty. The existing probation, a node judged every frame by a shadow vote against the
healthy nodes, **is** the WARM state, entered for a different reason (the `warm` command) and *held* there until promoted; promoting WARM to HOT is the existing probation criterion (100
agreeing frames, 300 after a repeat latch), so a promoted node has proved itself on live data. A WARM computer that fails the shadow vote is a faulty one: it is latched like any other. The supervisor (`SUPERVISOR.md`) is what makes COLD real: it holds a
node in reset or opens its power relay.

The **mode** (Triplex, Duplex, Simplex) stays the number of healthy HOT nodes. A system with two HOT nodes and one WARM node is Duplex by
mode but is a *planned* Duplex, which the phase table marks as allowed; the same Duplex after a fault is not.

## 3. Phase table for the rig scenario

| # | Phase | Nominal HOT | Minimum HOT | Other nodes | Reintegration | Safe action | Why |
|---|---|---|---|---|---|---|---|
| P0 | Off | 0 | 0 | COLD (supervisor only) | n/a | n/a | |
| P1 | Power-up and checkout | up to 3, one at a time | 1 | joining through startup grace | by the manager | none | Boot order, key check, sensor self-test (FDIR-036), SYNC lock |
| P2 | Pre-launch hold | 3 | 3 | none | operator | command neutral; platform level | **Go/no-go: fewer than 3 healthy HOT is a hold**, not a launch |
| P3 | Ascent (boost) | 3 | 2 | none; a lost node is not replaced while a burn is under way | automatic only for a first transient latch; otherwise after the phase | freeze, then null (`SAFE_MODE.md`) | an event that cannot be repeated: fail-operational with no spare to wake |
| P4 | Coast | 2 | 2 | 1 WARM | operator | freeze | no steering is needed, so the third node can rest; this is where the exposure is saved |
| P5 | Pre-burn | 3 | 3 | the WARM node promoted at least N s before the burn | by the phase | as P3 | the promotion must be finished and proved before it is needed |
| P6 | Burn (second ascent, landing) | 3 | 2 | none | as P3 | as P3 | as P3 |
| P7 | Safed / recovery | as found | 1 | operator's choice | operator | already in Safe | the operator brings the system back deliberately |

Notes. (a) P3 and P6 are the only phases that need Triplex; the others are shown so that the mechanism, not the saving, is what is being
demonstrated, because a short desk ascent saves no power. (b) A WARM node receives the same inputs as the HOT ones, so it shares any bug
that depends only on the inputs; it protects against hardware faults and wear, not against a software bug. Only the diverse node of ADR-021
protects against that. (c) Which node rests in P4 should be chosen so that the golden node of ADR-021 stays HOT.

## 4. Rules for changing roles

- **Promoting WARM to HOT** (`reintegrate` on a WARM computer) uses the probation criteria (FDIR-006, FDIR-020, FDIR-021). One at a time. A computer that has just been rested and is promoted again must prove itself again from the start.
- **Demoting HOT to WARM** (`warm`) or **disabling** follows the interlock tiers (ADR-019) *of the phase*, with `n` the number of healthy voters after the command: plain while `n` is at least the phase's nominal count; an ARM when `n` is below the nominal count but at least the minimum;
  **refused** (`RefusedPhase`) below the minimum. Without phases the tiers of ADR-019 apply (plain while two or more voters are left) and a WARM rest never takes the last voter. Commands that take no voter away (a computer that is already latched) are not held to the tiers.
- **A phase change that cannot meet its minimum is refused** and the system holds in the current phase (P2 to P3 with two healthy nodes is fine, P2 to P5 with two is not).
- **Who changes phase:** an authenticated `phase` command (ground opcode 7; its node field is the phase number 0 to 7; no ARM), sent by the operator or by the simulator at a scripted event. A flight computer never changes phase on its own; it can only refuse.
- **Phase and Safe:** entering Safe does not change the phase; the phase decides what Safe does (`SAFE_MODE.md` section 5; not built, section 5 below).
- **Below the minimum is an alert, never an action.** With fewer healthy voters than the phase's minimum the manager sets `below_minimum` in the report and counts the frames; it does not request Safe or abort (owner's decision, 4 Oct 2026: Simplex in ascent alerts only).
- **`noop`** (opcode 8) changes nothing, needs no ARM and is answered like any command, so the operator can test the command path end to end (TFC-FDIR-043).

## 5. What is built, and what is not

| Built | Where |
|---|---|
| The phase table (nominal and minimum voters per phase) and `RedundancyConfig::phases` (off by default: the manager then knows no phase and nothing changes) | `core/include/tfc/redundancy_types.hpp` |
| `phase`, `warm`, `noop` ground operations, the WARM role (the probation held until promoted), the phase-aware tiers, `below_minimum`, a guarded phase byte that fails to the safest phase and is reported, a scrubbed WARM mask | `core/include/tfc/redundancy.hpp`, `protocol.hpp` |
| Console and status line (`w` for a resting computer; `phase=` and `warm=` at the end), `CONFIG_TFC_PHASES` | `firmware/app` |
| Replay and scripted commands (`tfc_replay --phases`; `--command 100:phase:coast`, `150:warm:C`), a live test on `vcan0` | `tools/replay`, `sim/tfc_peers`, `sim/tests/test_live_phases.py` |
| Tests: `tests/test_phases.cpp` (the tiers per phase, the minimums' edges, WARM, promotion, noop), 32 mutants | |

**Not built:** the **Safe action per manager phase**: ACT tells the pad from flight by the mission frame in SYNC (Safe on the pad goes to neutral at once, in flight it freezes, holds and ramps: `SAFE_MODE.md` section 5), but it does not know the manager's phase, which the heartbeat does not carry, so the coast phase's "freeze only" is not built; **COLD as an automatic standby**: a cold node is one the operator told the supervisor to
`hold` (`SUPERVISOR.md`); events and telemetry for every role change beyond the console lines (IF-005, IF-006, `FUTURE_WORK.md` 2.2).

## 6. Sources

The hot, warm and cold definitions are the usual ones in the redundancy literature, found through search summaries (not read in full):
a cold unit is off with a zero failure rate until used; a warm unit is partly powered, quicker to bring in than cold but with state not
up to date; a hot unit runs concurrently with the same inputs and can take over at once, at the price of exposure and power. The ASTE-331
lecture's fail-operational triggers are the rule for when to keep all three hot.
