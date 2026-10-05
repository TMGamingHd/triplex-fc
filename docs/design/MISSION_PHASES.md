# Mission phases: which computers run, and how (hot, warm, cold)

> Status: **proposed** (ADR-023). Nothing here is built; the phase table is a design for the manager and the simulator to share. Numbers
> are proposals.

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
healthy nodes, **is** the WARM state, entered for a different reason; promoting WARM to HOT is the existing probation criterion (100
agreeing frames), so a promoted node has proved itself on live data. The supervisor (`SUPERVISOR.md`) is what makes COLD real: it holds a
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

- **Promoting WARM to HOT** uses the probation criteria (FDIR-006, FDIR-020, FDIR-021). One at a time.
- **Demoting HOT to WARM or COLD** follows the `disable` interlock tiers (ADR-019): plain while the phase minimum is still met, an ARM
  below the phase's nominal count, and refused below its minimum.
- **A phase change that cannot meet its minimum is refused** and the system holds in the current phase (P2 to P3 with two healthy nodes,
  for example).
- **Who changes phase:** an authenticated `phase` command (a new ground-command opcode, ADR-019's rules), sent by the operator or by the
  simulator at a scripted event. A flight computer never changes phase on its own; it can only refuse.
- **Phase and Safe:** entering Safe does not change the phase; the phase decides what Safe does (`SAFE_MODE.md` section 5).

## 5. What this needs

- A phase and role field in the manager (`core/`), a phase table as parameters, and the `phase` command. Needs the estimator and the
  simulator's scenario events to be useful, hence deferred (`FUTURE_WORK.md`).
- Supervisor `hold`, `release` and boot sequencing for COLD (`SUPERVISOR.md`).
- Events and telemetry for every role change (IF-005, IF-006).
- Tests: F71 (a phase change refused for lack of nodes) and F72 (a promotion that fails probation).

## 6. Sources

The hot, warm and cold definitions are the usual ones in the redundancy literature, found through search summaries (not read in full):
a cold unit is off with a zero failure rate until used; a warm unit is partly powered, quicker to bring in than cold but with state not
up to date; a hot unit runs concurrently with the same inputs and can take over at once, at the price of exposure and power. The ASTE-331
lecture's fail-operational triggers are the rule for when to keep all three hot.
