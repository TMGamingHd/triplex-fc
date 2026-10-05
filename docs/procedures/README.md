# Procedures

> Status: **procedure** index. Every hardware and hardware-in-the-loop check that no automated test replaces has a procedure here, written from `docs/verification/VERIFICATION_PROCEDURE_TEMPLATE.md`: set-up, numbered steps, pass criteria, shutdown, and an as-run
> copy that is kept with the data. Results are logged, dated, in [`../hardware/BENCH_LOG.md`](../hardware/BENCH_LOG.md).

Start with **[FIRST_HARDWARE_DAYS.md](FIRST_HARDWARE_DAYS.md)**: the order of work for the days after the parts arrive, what each number decides, and where to stop.

| Procedure | Stage | What it settles |
|---|---|---|
| [P-M1-01-parts-and-bench-checks.md](P-M1-01-parts-and-bench-checks.md) | M1 | Every part against the sheet; JP5, the adapter's voltage, the relay at 3.3 V, the servo with and without a signal |
| [P-S1-01-single-flight-computer.md](P-S1-01-single-flight-computer.md) | S1 | One flight computer: 10 min at 100 Hz, jitter, WCET, the digest against the golden run |
| [P-S2-01-actuator-and-platform.md](P-S2-01-actuator-and-platform.md) | S2 | ACT, the Pico and the platform; Safe by ACT alone; an ACT reset |
| [P-S2-02-launch-checklist.md](P-S2-02-launch-checklist.md) | S2 | The launch checklist on the rig: calibration, go/no-go, countdown, scrub, T-zero |
| [P-S2-03-supervisor.md](P-S2-03-supervisor.md) | S2b | The supervisor: a hung node reset in three frames, the ladder, unplugged, the `T0` line, the clock |
| [P-HWO-01-overrides.md](P-HWO-01-overrides.md) | S2b | Operate every hardware override and log it; the sense lines |
| [P-S3-01-duplex.md](P-S3-01-duplex.md) | S3 | Two computers: loss of one, a bad value, an IMU fault with the sensor split |
| [P-S4-01-triplex.md](P-S4-01-triplex.md) | S4 | Three computers: the fault campaign on the real bus, the real loss rate |
| [P-REL-01-golden-release.md](P-REL-01-golden-release.md) | after S4 | Make, record and field a golden release; the compatibility gate; the mixed-release run |
