# P-REL-01: make and use a golden release (ADR-021)

Follows `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`. The golden release is the previous known-good release that runs on node C, so that a regression in the current release cannot take all three computers with it at once; a
disagreement between the two releases isolates nobody and asks the operator (ADR-021). Everything the procedure needs is built; what waits is a release that has passed the hardware stage exits.

| Field | Entry |
|---|---|
| Procedure ID | P-REL-01 |
| Requirements verified | TFC-ARCH-004 to 007 (TFC-ARCH-008 is realised as "digests are compared only within a release", ADR-021) |
| Fault-matrix rows | F63 (a common-mode fault), F64 (a release that differs beyond the tolerance) |
| Tools | `tools/release/record_release.py`, `tools/compat/compat_gate.py`, `tools/bench/sil_triplex.sh --build-mixed`, `tfc_replay --release A=ID,B=ID,C=ID` |
| When | After S4 and the campaign on the hardware (M4); repeat for every release that is to be fielded beside a golden one |

## 1. Before: what must be true
1. The release is a **git tag** on a clean tree (`golden-N`), built from that tag, and has passed (a) the full fault campaign with no anomaly (`cd sim && python3 -m campaign.run --strict --out logs/campaign.jsonl`) and (b) the exit tests of S1 to S4 (the as-run copies of `P-S1-01` to `P-S4-01` in `hardware/BENCH_LOG.md`).
2. The firmware reports its release in the heartbeat: the default is the first four hex digits of the git hash (`CONFIG_TFC_RELEASE_ID=0`), printed in the boot line `release 0x....`. Two releases built from different commits are different releases without any further step.
3. The bus protocol and the manager's API are the ones the golden release was built with (TFC-ARCH-004). If the golden release's headers do not build against the compatibility harness, the gate says so (exit 3) and that is a finding, not a pass.

## 2. Record the release (TFC-ARCH-005)
```
. firmware/env.sh
git checkout golden-1 && west build -p always -b nucleo_g474re firmware/app -d build/golden_a -- -DCONFIG_TFC_FLIGHT_FUNCTION=y
# a second, independent build of the same commit, in another directory: reproducibility
west build -p always -b nucleo_g474re firmware/app -d build/golden_b -- -DCONFIG_TFC_FLIGHT_FUNCTION=y
python3 tools/release/record_release.py --tag golden-1 --elf build/golden_a/zephyr/zephyr.elf --elf-rebuilt build/golden_b/zephyr/zephyr.elf \
    --campaign logs/campaign.jsonl --out releases/golden-1.json
```
The record names the commit and whether the tree was clean, the release id, the compiler, Zephyr and SDK versions, the hash of the flash image (the loadable segments, so that paths in the debug information do not matter), whether the second build gave the same image, and the
campaign's scenario and anomaly counts. It **refuses** (exit 1) a release whose campaign has anomalies unless `--allow-anomalies` is given, and says so in the record when it is. **Pass:** the record is written, the tree is clean, the two builds hash alike, the campaign has no anomaly. Keep `releases/golden-1.json`.

## 3. Before a new release is fielded beside it: the compatibility gate (TFC-ARCH-006)
```
python3 tools/compat/compat_gate.py --golden golden-1          # or --golden-include DIR for a checked-out copy
```
It builds the same harness against the current and the golden `core/include`, flies the fixed scenario set through both (three flight functions with their IMU models over the 6-DOF vehicle) and requires the voted commands to agree in every frame within the version tolerance (1.5 times the command vote
tolerance). Exit 0: compatible. Exit 1: a command differs by more than the tolerance: **do not field this release beside that golden one** (the golden computer would be the odd one out and the manager would hold and ask). Exit 3: the golden release cannot be built against the harness.

## 4. Run it mixed on the bench (TFC-ARCH-007)
With the three images flashed (A and B the current release, C the golden; or any pair and a lone computer):
1. Quiet run for a minute: nothing is isolated, no Safe request, the digests of the pair agree (the digest of the lone computer is not compared with theirs). The status line shows all three `+`.
2. Make the golden computer's command differ by **less** than the version tolerance (a test knob: `CONFIG_TFC_TEST_CMD_OFFSET_MDEG`, 12 for 0.012 degree): nothing happens at all.
3. Make it differ by **more** (50 for 0.05 degree): no computer is latched out, the outputs are held, **Safe is requested** (`SAFE REQUESTED` on every console, the status line says `SAFE`, `release_split_frames` counts).
4. The operator names the side to trust: `disable C` (Triplex to Duplex is plain), then `clear-safe` under an ARM; the system flies on in Duplex. Record the frame numbers and the console lines.
The same four steps run live on `vcan0` as `sim/tests/test_live_release.py` (build the images with `tools/bench/sil_triplex.sh --build-mixed`); on the rig they are the pass criteria.

## 5. After
A safety fix to the golden release is the only reason to change it; make it a new tag, record it again, and run section 4 once more. An **automatic** takeover by the golden release (instead of hold and ask) stays open until the data of TS-3 exist; it is not part of v1.
