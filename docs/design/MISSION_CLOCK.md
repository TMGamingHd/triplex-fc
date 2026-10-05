# The clock of record: one independent truth of what time it is

> Status: **built on the host, in the supervisor's application and on the PC; not run on the hardware** (ADR-029 as amended, TS-22; reviewed 5 Oct 2026). Written after the owner's point of 4 Oct 2026: *if a mission runs for five years we still need one independent truth of what time it is; on a spacecraft that is usually an
> atomic clock.* The distribution of time to the nodes (section 3) is outside v1. Drift figures are datasheet-class numbers from memory and **not checked**; verify before relying on them.

## 1. Two different clocks, and why they must not be confused

| | **Schedule time** (what the launch sequence needs) | **Clock of record** (what this page is about) |
|---|---|---|
| What it is | Frames since T-zero, carried in SYNC (`LAUNCH_SEQUENCE.md` section 3) | Mission elapsed time (MET) since T-zero, and the epoch behind it, on an independent oscillator |
| Lifetime | One ascent: minutes (16 bits of frames cover 655 s) | The whole mission: hours to years, across resets, power cuts and safe modes |
| Accuracy needed | The same on every computer, to the frame | Traceable to a reference (UTC), with a known, bounded error |
| Source | The replicated computers (the sync master counts, the rest follow and verify) | **The supervisor**, on its own oscillator, with battery-backed memory of the time |
| Must survive | The loss of the supervisor (TFC-SUP-007) | The loss of every flight computer, and a power cut |
| Used for | Guidance and gain schedules, the countdown | Time-tagging telemetry and events, long-term scheduled actions, the plausibility check of everything else, correlating with ground time |

The owner's clarification fits the second row exactly: the supervisor is the right home for the **clock of record**, because it is independent of the flight computers and must not be impacted by them. It stays out of the schedule path for the reason in ADR-022.

## 2. How good an oscillator has to be: drift over five years

| Oscillator class | Fractional error | Drift per day | Drift in 5 years |
|---|---|---|---|
| The Nucleo's crystal (tens of ppm; 20 ppm taken) | 2e-5 | 1.7 s | **3,160 s (53 minutes)** |
| A TCXO real-time-clock module (DS3231 class, about 2 ppm) | 2e-6 | 0.17 s | **316 s** |
| An oven-controlled crystal (about 0.1 ppm) | 1e-7 | 9 ms | 16 s |
| A rubidium or chip-scale atomic clock (about 1e-10) | 1e-10 | 9 us | 16 ms |
| A deep-space ultra-stable oscillator (about 1e-12) | 1e-12 | 0.09 us | 0.16 ms |

So the TCXO the project has chosen is **not** a five-year clock: it is the clock for a desk demonstration, and for a real mission its error would be five minutes. Two things make it adequate, and both are designed in rather than assumed:
1. **Time correlation.** Real spacecraft do not trust their oscillator for absolute time; they record pairs (spacecraft clock count, ground UTC) and fit the drift and aging, so the count is converted to UTC with a known error and the correction is applied on the ground. The rig does the same with the PC, whose time is disciplined by NTP: the supervisor reports its counter over USB, the PC stamps UTC on arrival, and a drift and offset are fitted. The supervisor's clock is then an *independent truth with a known error*, which is what is asked of it.
2. **A better reference where one exists.** On the desk a **GPS receiver with a pulse-per-second output** is a UTC-traceable, atomic-clock-disciplined reference for a few tens of dollars (it needs a sky view; an indoor bench may not get a fix). It disciplines the supervisor's oscillator or checks it. In deep space there is no GPS, and the answer is a better oscillator plus ground correlation.

## 3. What the supervisor keeps and how it gives time to the others (the second half is outside v1)

- **Keeps:** MET (a 64-bit count of its oscillator's ticks since the epoch), the epoch (T-zero, or boot time before launch), the correlation pairs, and the oscillator's measured drift. A **battery-backed RTC** (the DS3231-class module has a coin-cell input) keeps counting through a power cut of the whole rig, so MET survives a total loss; the supervisor reads it on start-up and flags any gap.
- **(Outside v1: ADR-029 as amended.) Gives time without transmitting on the bus** (ADR-022 keeps it off the bus): a **1 Hz pulse line** (`PPS`) to each node, from its oscillator, and a **one-way serial line** (`TIME`, one transmitter, four receivers) carrying MET and a status once a second. A node uses the pulse to align its own frame counter's drift measurement and the message to seed its time-tag after a reset. A node that hears nothing keeps its own count and flags the age of its time. These are two more lines on the supervisor (about 19 of its 26 GPIO), none of them able to make a node stop.
- **Checks, never overrides:** it compares each node's frame count (from the `FRAME` pulses) with its own clock and flags a node whose time departs by more than a bound (TFC-SUP-009); it does not decide whether a working node is right.
- **Fails passive:** if it resets, MET resumes from the battery-backed RTC; the nodes keep their last time with a growing age until it returns.

## 4. What the flight computers do with it
Time-tag every event and telemetry frame with MET when they have it (age flagged when not), use the schedule time (frames since T-zero) for guidance exactly as before, and never let MET into a control decision. If the two clocks disagree beyond a bound, report, do not act.

## 5. Requirements (statuses in `../verification/REQUIREMENTS.md`)
| ID | Requirement |
|---|---|
| TFC-SUP-011 | The supervisor shall keep mission elapsed time on an oscillator of its own, backed by a battery so that it survives a power cut of the rig, and shall report it, with the oscillator's measured drift, over USB. |
| TFC-SUP-012 | The supervisor shall record correlation pairs (its counter against the PC's UTC) and the fitted drift and offset, so that its time converts to UTC with a stated error. |
| TFC-SUP-013 | The supervisor shall distribute time to the flight computers and ACT by a pulse per second and a one-way serial message, not by the flight bus; a node that loses either shall flag the age of its time and carry on. |
| TFC-SUP-014 | No control decision of any node shall depend on the supervisor's time (TFC-SUP-007). |

## 6. What is built
The logic is written and tested on the PC, the supervisor's Pico application uses it (the USB `status`, the DS3231 on I2C, the flash record of T-zero), and the PC side correlates; nothing runs on the real parts yet.

| Piece | Where | What it does | Tests |
|---|---|---|---|
| MET on the supervisor's ticks | `supervisor/include/sup/mission_clock.hpp` (`MissionClock`, `TickExtender`) | Integer-only: 64-bit ticks from a 32-bit timer, T-zero latched once (a repeat changes nothing), MET in ticks, microseconds and frames with no overflow over five years | `tests/test_supervisor_clock.cpp` |
| The battery-backed record | same (`MetRecord`, CRC-16) | The RTC seconds at T-zero, with a magic number and a CRC; after a reset `resume` rebuilds T-zero's tick from the RTC, **to a second** (`MetSource::Recovered`), and refuses a corrupt record or a T-zero in the RTC's future | same, plus a test that every single-bit corruption of the seconds is caught |
| The plausibility check (TFC-SUP-009) | same (`MissionWatch`) | Compares the SYNC mission field a node reports with MET: allows a few frames plus 200 ppm of the elapsed time plus the second of uncertainty of a recovered T-zero, saturates at 65 535 like SYNC, flags after 3 bad checks in a row, reports only | same |
| Reporting the counter (TFC-SUP-011): the `time` command answers `time ticks=N rtc=S met_us=M` | `supervisor/include/sup/supervisor.hpp` (`time_text`), `firmware/supervisor` | The supervisor's own tick counter, the RTC's seconds and MET in microseconds (`-` before T-zero) | `time_*` tests in `tests/test_supervisor.cpp` |
| Correlation with the PC (TFC-SUP-012) | `sim/tfc_peers/timecorr.py` (`Correlator`); the sampler `tools/bench/clock_corr.py` asks `time` every few seconds, stamps each answer with UTC and prints the drift and the bound now and five years out | Fits the supervisor's counter against PC UTC; gives the oscillator's drift in ppm, a conversion to UTC and an error bound that grows with the distance from the data (an hour of pairs says little about year five) plus a stated latency allowance; throws out a stalled stamp; starts a new run when the counter goes backwards (a supervisor restart); keeps a bounded record that still covers the whole baseline | `sim/tests/test_timecorr.py` |

The supervisor shares no code with `core/` (TFC-SUP-001): `sup::` restates the frame period, the first flight value of the mission field and its saturation value, and
`sup_clock_restated_protocol_facts_match_the_flight_protocol` is the only place that includes both and checks that they still agree. The mechanical coding-standard check
(`tools/check_standard.py`) now covers `supervisor/include` too, and 20 mutants (`sup_*`) check the tests.

What the measurements say, from the correlator's own tests (a simulated 20 ppm oscillator, 2 ms of Gaussian scatter on the PC's stamps, a pair every minute): an **hour of pairs
measures the drift to well under 1 ppm** (the test's limit; the standard error is about 0.25 ppm), and the converted time stays **inside its own error bound** at 5 years
out, where the bound is tens of seconds because the line was fitted on one hour; ten days of pairs tighten it by a factor of more than ten. That is the honest way to read the
table of section 2: the oscillator alone is minutes off in five years, and the correlation turns it into a time with a *stated* error, which is only as good as the baseline.

**Not done, and why.** (1) **Distributing time to the nodes** (the pulse per second and the one-way serial line, TFC-SUP-013) is outside v1: nothing controls on it (TFC-SUP-014), the Lite supervisor's pins are all used (`SUPERVISOR.md` section 4), and the PC reads the clock over USB. (2) The check compares the **count of a
node's `FRAME` pulses since T-zero** with the clock (the Lite supervisor cannot read the SYNC mission field, which needs the bus: that is the Full supervisor's half of TFC-SUP-009); the same function serves with the count converted to the field's encoding. (3) The oscillator's *aging and temperature* behaviour is not modelled: the fit assumes
a constant rate over the record, and a real crystal's rate changes, so the error bound understates the far extrapolation for a real part; a windowed fit and a measured temperature coefficient are the fix (on the desk a hair dryer and a logged temperature are enough for a first look). (4) The real RTC, the TCXO's drift and the first correlation data wait for the parts (`P-S2-03`, `FIRST_HARDWARE_DAYS.md` day 3).
