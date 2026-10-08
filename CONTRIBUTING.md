# Workflow: GitHub Flow

One rule: **`main` always builds, passes CI, and is safe to tag.** Everything else follows from it.

```
main ──●────────●──────────────●─────────●──▶
        \      /  \           /
         feat/x    fix/y ─────           (short-lived branches, one PR each)
```

## The loop
1. **Branch from an up-to-date `main`.** `git switch main && git pull && git switch -c <type>/<short-name>`
2. **Commit small and often** on the branch. Push it early so the work is visible.
3. **Open a pull request** (draft is fine) as soon as there is something to discuss. Fill in the PR template.
4. **CI must be green** (strict build with warnings as errors, tests under ASan/UBSan, clang-tidy, cppcheck, the coding-standard and coverage gates, the fault campaign).
5. **Review** - even solo, read your own diff on GitHub before merging, and tick the checklist.
6. **Squash-merge** into `main`, then **delete the branch**. One PR = one commit on `main`.
7. `main` is never committed to directly.

## Branch names
`<type>/<short-kebab-description>`

| Type | Use for | Example |
|---|---|---|
| `feat/` | new capability | `feat/native-sim-frame-loop` |
| `fix/` | bug fix | `fix/stuck-detector-overflow` |
| `hw/` | bench checks, wiring, mechanical/CAD, parts list changes | `hw/m1-bench-checks` |
| `docs/` | documentation only | `docs/adr-007-can-pins` |
| `test/` | tests or fault-campaign additions | `test/f05-slow-drift` |
| `chore/` | tooling, CI, repo housekeeping | `chore/github-flow` |

Keep branches short-lived (days, not weeks) and scoped to one concern. If a branch grows, split it.

## Commit and PR titles
Imperative, specific, under ~70 characters, optionally prefixed with the area: `core: add slow-drift detector`, `docs: record CAN pin choice`. Because we squash-merge, the **PR title becomes the commit on `main`**, so make it read well.

## Project-specific rules
These come from `docs/verification/REQUIREMENTS.md` and are what reviewers check:
- **`core/` stays portable and follows `docs/verification/CODING_STANDARD.md`:** no heap, no exceptions, no RTTI, no `while`/`goto`/recursion/macros/globals, functions of at most 60 lines, warnings as errors, no `-ffast-math` (TFC-SW-001/002/007/008).
- **New behaviour needs a test, written first;** a fault-matrix row only moves to *Passing* when its test exists and passes (`docs/verification/FAULT_MATRIX.md`). A change to `core/` must leave the campaign's per-frame decision hashes unchanged unless it is meant to change behaviour (then an ADR says so).
- **Measured numbers only.** Anything marked *target* stays a target until a measurement is logged. Never paste a number into the README or write-up that has no log behind it.
- **Design changes get an ADR** in `docs/decisions/DECISIONS.md`; requirement changes update `docs/verification/REQUIREMENTS.md` in the same PR.
- **Hardware findings** (measured voltages, mounting holes, relay and servo checks) go in `docs/hardware/BENCH_LOG.md` with the date and the part's lot/source.

## Before you push a change to `core/`
```bash
cmake -S . -B build -G Ninja -DTFC_SANITIZE=ON && cmake --build build && ctest --test-dir build --output-on-failure
python3 tools/check_standard.py                                   # mechanical coding-standard rules
python3 tools/coverage/core_coverage.py --min-line 100 --min-branch 98 --sim-min-line 99 --sim-min-branch 90
cmake -S . -B build/rel -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/rel --target tfc_replay
(cd sim && TFC_REPLAY_BIN=$PWD/../build/rel/tfc_replay python3 -m campaign.run --strict)   # about 2-4 minutes
python3 tools/mutation/run_unit.py                                # deliberate bugs must still be caught (weekly in CI)
```
A new check or fallback in `core/` comes with a mutant in `tools/mutation/mutations.py` that removes it.

## Before you push a change to the vehicle simulator (`sim/vehicle/`)
```bash
ctest --test-dir build --output-on-failure                         # the whole suite, with -Werror
python3 tools/coverage/core_coverage.py --sim-min-line 99 --sim-min-branch 90
python3 tools/mutation/run_sim.py                                  # deliberate bugs in the simulator must still be caught (weekly in CI; about 15 minutes)
```
`a_controlled_flight_of_the_reference_vehicle_is_unchanged_by_changes_to_the_model` (`tests/test_vehicle.cpp`) and `general_the_reference_vehicle_flies_as_the_frozen_single_vehicle_model_did` (`tests/test_vehicle_general.cpp`, against `tests/oracle/`) must still pass without edits: the reference vehicle's flight is the contract of every change. A change that moves it by even a rounding moves the quantised sensor values of the closed loop, and with them the documented sensitivity numbers. A new
model, option or guard comes with a test that fails without it, and a mutant in `tools/mutation/sim_mutations.py` that removes it.

## Milestones, stages and tags
- Each staged-build stage (S1-S4) and each milestone (M0-M5) ends with a **tag on `main`** and a short log or video: `git tag -a s1-single-fc -m "..." && git push origin s1-single-fc`.
- Tags: `s1-single-fc`, `s2-act`, `s3-duplex`, `s4-triplex`, and `v1.0` at M5.
- Because `main` is always releasable, tagging is just pointing at a commit. No release branches.

## One-time repository settings (maintainer)
Already applied: squash-merge only, merge commits and rebase merges off, head branches auto-deleted.

**Branch protection is not enabled yet:** GitHub blocks branch protection and rulesets on private repos under the
free plan (API: "Upgrade to GitHub Pro or make this repository public"). Until the repo is public or the account is
on Pro (the GitHub Student Developer Pack includes Pro), the rules below are followed by discipline, not enforced.

In GitHub: Settings -> Rules (or Branches) -> protect `main`:
- Require a pull request before merging
- Require status checks **`sil`** and **`firmware`** to pass, branch up to date before merge
- Block force pushes and deletions
- (Optional, solo) leave "required approvals" at 0 so you can merge your own PRs; bring it to 1 once there is a second contributor

Settings -> General -> Pull Requests: allow **squash merging** only, and enable **Automatically delete head branches**.
