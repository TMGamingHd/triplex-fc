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
4. **CI must be green** (build with warnings as errors, tests under ASan/UBSan, clang-tidy, cppcheck).
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
These come from `docs/REQUIREMENTS.md` and are what reviewers check:
- **`core/` stays portable:** no heap, no exceptions, no RTTI, warnings as errors, no `-ffast-math` (TFC-SW-001/002).
- **New behaviour needs a test;** a fault-matrix row only moves to *Passing* when its test exists and passes (`docs/FAULT_MATRIX.md`).
- **Measured numbers only.** Anything marked *target* stays a target until a measurement is logged. Never paste a number into the README or write-up that has no log behind it.
- **Design changes get an ADR** in `docs/DECISIONS.md`; requirement changes update `docs/REQUIREMENTS.md` in the same PR.
- **Hardware findings** (measured voltages, mounting holes, relay and servo checks) go in `docs/BENCH_CHECKS.md` with the date and the part's lot/source.

## Milestones, stages and tags
- Each staged-build stage (S1-S4) and each milestone (M0-M5) ends with a **tag on `main`** and a short log or video: `git tag -a s1-single-fc -m "..." && git push origin s1-single-fc`.
- Tags: `s1-single-fc`, `s2-act`, `s3-duplex`, `s4-triplex`, and `v1.0` at M5.
- Because `main` is always releasable, tagging is just pointing at a commit. No release branches.

## One-time repository settings (maintainer)
In GitHub: Settings -> Rules (or Branches) -> protect `main`:
- Require a pull request before merging
- Require status check **`sil`** to pass (appears after the first CI run), branch up to date before merge
- Block force pushes and deletions
- (Optional, solo) leave "required approvals" at 0 so you can merge your own PRs; bring it to 1 once there is a second contributor

Settings -> General -> Pull Requests: allow **squash merging** only, and enable **Automatically delete head branches**.
