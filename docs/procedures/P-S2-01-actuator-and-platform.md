# P-S2-01: stage S2, the actuator node and the platform

**Procedure, to be run when the parts arrive.** The actuator node's firmware, the Pico firmware and the vehicle simulator it needs are built and tested against fakes and on the virtual rig; the steps below fix what is to be shown on the real platform. Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`.

| Field | Entry |
|---|---|
| Procedure ID | P-S2-01 |
| Requirements verified | TFC-SYS-005 (envelope with one computer), TFC-SAFE-001 and 002 (Safe by ACT alone), TFC-RESP-003 (ACT after a reset) |
| Fault-matrix rows | F17 (ACT reset), F69 (ACT alone), F70 (Safe exit refused) |
| Hardware configuration | FC-A, ACT, the Pico (servo and relays), the platform with two servos and the IMU on it, the E-stop, the servo rail with its fuse |

## Steps to be written out (who, exact action, expected, as-run)
1. **Platform without a load path:** the Pico drives the servos from a scripted ramp with the hard stops fitted; rate and travel limits confirmed.
2. **Open loop:** the simulator's attitude drives the platform; the IMU on it reads the motion; compare against the commanded rate.
3. **Closed loop with one computer:** FC-A's command to ACT to the platform; the platform tracks the simulated ascent within the envelope (SYS-005).
4. **Safe by ACT alone:** stop FC-A; ACT must freeze, then null at the rate limit, with no step (SAFE-001, SAFE-002; F69).
5. **ACT reset:** reset ACT mid-run; it must start in Safe and resume from its stored output with no step larger than the rate limit (RESP-003; F17), and the supervisor's `NRST` path if built.
6. **Safe exit:** `clear-safe` under an ARM with and without the exit conditions (SAFE-003; F70).
7. **E-stop:** the servo rail cut; the platform stops; recovery by hand.
