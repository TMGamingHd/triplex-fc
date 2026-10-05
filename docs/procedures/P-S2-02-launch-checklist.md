# P-S2-02: the launch checklist (pad, go/no-go, countdown, T-zero)

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. The human checklist for a run of the closed loop, and the verification of the launch sequence on the virtual rig (`vcan0`) now and on the real rig later.
Design: `docs/design/LAUNCH_SEQUENCE.md`. The automated half is `python3 -m tfc_peers launch`.

| Field | Entry |
|---|---|
| Procedure ID | P-S2-02 |
| Requirements verified | TFC-LAUN-001 to 007, TFC-PLAT-002 (the platform holds and levels if the stream stops, on the rig), TFC-SAFE-008 |
| Fault-matrix rows | F81 to F87 |
| Firmware under test | git hash of every node, built with `CONFIG_TFC_LAUNCH_SEQUENCE=y` (`tools/bench/sil_triplex.sh --build --launch`); the supervisor, if present, is not part of the launch yet |
| Hardware configuration | Virtual rig: three flight-computer processes, ACT, `tfc_simd --hold` on `vcan0`. Real rig (later): three Nucleos with IMUs on the platform, ACT, the Pico, the PC with `tfc_simd --hold --pico` |
| Tools | `tools/bench/sil_triplex.sh`, `python3 -m tfc_peers launch`, `tfc_peers command`, `tfc_peers listen` |

## 1. Description
Show that a launch only happens when every item of the go/no-go holds, that the pad calibration happens, that the countdown is 10 s and can be scrubbed, that T-zero releases the vehicle on every computer in the same frame, and that the
flight follows the program from T-zero. **Pass** = every step as expected; a no-go or a scrub is a pass when it is the expected outcome of the step.

## 2. Initial set-up
Images built and the ELF check passed; `vcan0` up (`sim/scripts/setup_vcan.sh`); on the real rig, the rig checks of P-S2-01 done, the platform level, the E-stop released, **nobody near the platform**, the `INJECTOR-DISARM` switch (if fitted) in the disarmed position.
Start order: the simulator (`tfc_simd --hold`), then the three flight computers, then ACT. Each flight computer prints its console; keep all four visible.

## 3. The checklist (before the launch command)
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 1 | Operator | Start `tfc_simd --iface vcan0 --hold` | `tfc_simd on vcan0: waiting for SYNC` | | |
| 2 | Operator | Start the three flight computers and ACT | A prints `listens for a master, then claims SYNC`, then B and C `joined the bus`; `MODE ... -> TRIPLEX` | | |
| 3 | Operator | Wait about 12 s for the pad calibration | The status lines show `ready=1` on all three (calibration needs 10 s of rest); ACT prints `STANDBY -> NOMINAL` | | |
| 4 | Operator | `python3 -m tfc_peers launch --check` | `GO: every item holds.` and exit status 0. Anything else lists the reasons: **do not launch** | | |
| 5 | Operator | Read each console | No `LATCHED OUT`, no `SAFE REQUESTED`, `crc=0 seq=0 vote=0 digest=0` | | |
| 6 | Operator | Read the simulator | The vehicle is clamped: altitude 0, flight time 0 (`tfc_peers listen` shows `0x503`/`0x505`) | | |
| 7 | Everyone | **Clear the platform** (real rig) | Nobody within reach of the platform; the E-stop within reach of the operator | | |

## 4. The launch
| Step | Who | Action | Expected | Actual | Pass/fail |
|---|---|---|---|---|---|
| 8 | Operator | `python3 -m tfc_peers launch` and type `LAUNCH` | `GO. Sending the launch command: ARM, then EXECUTE.` then `COUNTDOWN started.` | | |
| 9 | Operator | Watch the master's console | `[frame N] LAUNCH: go; the countdown starts`, then `T-9 s` ... `T-1 s`, one a second, on every computer | | |
| 10 | Operator | At T-zero | `T-ZERO: lift-off; the schedules start` on all three computers in the same frame number; `tfc_peers launch` exits 0 | | |
| 11 | Operator | Watch the simulator for 15 s | Altitude rises, the attitude error stays within 1.5 degrees of the program; flags 0; ACT stays Nominal | | |

## 5. Cases of the countdown and the pad (run each as its own procedure after step 3)
| Case | Action | Expected |
|---|---|---|
| No-go before the calibration | Send `launch` 4 s after start (`tfc_peers command launch --arm`) | The master prints `LAUNCH REFUSED, no-go: a flight computer is not ready ...`; no countdown; the vehicle stays clamped |
| Scrub | During the countdown at about T-5 s: `tfc_peers command scrub` | `SCRUB: back to the pad` on all; the mission frame returns to 0; the vehicle stays clamped; after the pad is ready again a new launch works |
| A computer lost in the countdown | At about T-5 s kill a flight computer (or power it off through the injector) | `COUNTDOWN SCRUBBED, no-go: not three healthy flight computers` on the master (the new master, if A was the one lost); the vehicle never leaves the pad |
| A forged or replayed launch | `tfc_peers run ... --command 400:forged-launch`, or a repeated frame | Refused without a trace; no countdown |
| A scrub after T-zero | `tfc_peers command scrub` in flight | `SCRUB REFUSED: not in the countdown`; the flight goes on |
| A computer lost in flight | Kill B at T+30 s | Duplex: the vehicle flies on, the digests of A and C stay equal (or the documented skip, TS-16) |
| The PC link on the real rig | Stop `tfc_simd` in flight | The platform holds at 100 ms and levels after 1 s (PLAT-002) |

## 6. Shutdown
Stop `tfc_simd`, then the computers; on the real rig, level the platform, press the E-stop, power down the servo rail, then the node rail. Record the console logs with the as-run copy.

## 7. After a no-go
Read the reason `tfc_peers launch --check` prints; fix it (a moving platform: wait for it to rest; a latched node: `tfc_peers command reintegrate B`, and wait for probation; a Safe request: `command clear-safe --arm`); run step 4 again. **Never** launch with a reason showing.

## 8. As-run
Copy this file to `docs/procedures/as-run/` with the date, the commit, the actual column and the logs.
