## What and why
<!-- One or two sentences. Link the milestone/stage (e.g. M1, S1) or fault-matrix row (e.g. F05) if relevant. -->

## Changes
-

## Verification
<!-- How do you know it works? Paste the command and result, or link the bench log. Say "not run" if not run. -->
- [ ] `ctest` passes locally (or CI is green)
- [ ] Hardware check logged in `docs/hardware/BENCH_LOG.md` (hardware PRs only)

## Checklist
- [ ] Branch is named `<type>/<short-name>` and is scoped to one concern
- [ ] `core/` changes: no heap / exceptions / RTTI, new behaviour has a test
- [ ] Docs updated if behaviour, a requirement, or a fault-matrix row changed (`docs/`)
- [ ] Design decision recorded as an ADR if one was made
- [ ] Any number I added is measured (logged), or clearly labelled **target**
