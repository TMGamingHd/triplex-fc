# Abort and range safety: what a real vehicle has, and what this project does and does not do

> Status: **written** (18 Oct 2026). A page that says why the project has no flight-termination function, what the real one is, and what the flight computer is and is not asked to do about a vehicle that leaves its corridor. The statements about real systems are from my memory of public material and are not checked against a source; nothing here is insider knowledge. The decision itself is the owner's (4 Oct 2026: "an abort is a launch rule, not a flight-computer rule", `FAULT_RESPONSE.md` 6).

## 1. What a real launch has

A launch range protects the public by a **flight-termination system** (FTS): a command-destruct receiver and ordnance on the vehicle, fired by the range's safety officer when the vehicle leaves its approved corridor or loses control; and, on current launchers, an **autonomous flight-termination system** (AFTS): software on its own computers and sensors that decides the same thing on board, so that termination does not depend on a ground link. The defining properties (from memory) are that it is **independent of the flight computer** (its own sensors, power, logic and, in the older form, receiver), **simple** (a set of rules that can be argued complete), and **fail-safe to the safe side of the public's risk** (a failure of the system terminates the flight and does not let it continue). The flight computer's part is to fly the vehicle and to report; the decision to destroy it is not the flight computer's, so that a fault in the flight software cannot also disable the thing that protects the public from the flight software.

## 2. What this project does

* **No automatic abort in the flight computers** (decision of 4 Oct 2026). In ascent, Simplex *alerts*; an unresolved disagreement *freezes, then nulls* the actuator (`SAFE_MODE.md`): that is a response to a loss of redundancy, not a decision to end the flight.
* **An abort on the rig is an operator command to the simulator** (`TFC-SAFE-008`): the simulator marks the run safed and continues; only an operator abort ends it.
* **The supervisor and the hardware overrides are the independent layers**: a supervisor that never takes a voting decision (`TFC-SUP-010`) and switches that work with no program (`HARDWARE_OVERRIDE.md`). They protect the *rig* (a relay cuts a node's power, the E-stop stops the platform). They are not a termination system and make no claim to be one.

## 3. What a termination function on this architecture would be

If the demonstrator were to carry one, the design that follows from the rules above:

1. **A corridor** defined on the ground: positions and velocities as functions of time with bounds (the nominal trajectory of the design and a margin from the Monte Carlo of `MONTE_CARLO.md`, not a hand-drawn number). The mission design already flies the nominal; the corridor is its envelope.
2. **A monitor that is not the flight computer**: its own IMU and receiver, its own estimate of position and velocity, a rule set (outside the corridor for N seconds; attitude beyond a limit; loss of the flight computers' heartbeats), on the supervisor's Pico or a third unit, with **no connection to the flight bus** (the supervisor never transmits on it).
3. **An action that is a discrete line**: `SAFE` to ACT already exists (freeze then null); a termination would be a separate line to separate hardware (on the rig: the E-stop relay).
4. **Fail-safe**: the monitor's loss, hang or disagreement with the flight computers' state asserts the line (the opposite of the supervisor's fail-passive rule, which is why it is a different unit).

**Built:** none of it. **Not decided:** whether the project needs it: on a desk rig, the corridor is the platform's travel and the E-stop is the termination; for a flight vehicle the first sentence of section 1 applies and the project would not be the one to decide.

## 4. What the flight computer reports that a monitor would use

The poses and the heartbeat carry the voted attitude and command, the number of voters and the Safe flag; `tfc_mission` reports the **envelope** of each body (maximum q, q sin(alpha), the accelerations, the heating: ADR-053), the pins' state at the arms and the clearance of a separation. A corridor monitor would read the same numbers. **A report, not a rule:** nothing in the flight acts on them.
