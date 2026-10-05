# SPDX-License-Identifier: MIT
"""Time correlation for the clock of record (docs/design/MISSION_CLOCK.md section 2, TFC-SUP-012).

The supervisor reports its tick counter over USB; the PC stamps UTC on arrival. A pair is (counter ticks, UTC microseconds). Real spacecraft convert a clock count to UTC
the same way: fit offset and drift over many pairs, then say how far the converted time can be trusted. `Correlator` does that, with floating point on the PC (the supervisor's small
processor does integers only and keeps just the counter).

What it does not remove: the PC's stamp is late by the USB and operating-system latency, and that delay is not the same each time. The scatter of it is in the fit's error bound;
a constant part of it is a bias nobody can see from the pairs alone, so `latency_floor_us` is added to every bound as a stated allowance.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# A pair that departs from the fit by more than this is thrown out as a stall of the PC or the link, not counted as scatter.
DEFAULT_GATE_US = 20_000.0
MIN_PAIRS = 5
MIN_SPAN_S = 10.0


@dataclass
class Fit:
    """utc_us = utc0_us + rate * (ticks - ticks0), the least-squares line through the pairs, with how well it describes them."""
    ticks0: float
    utc0_us: float
    rate: float           # microseconds of UTC per supervisor tick (1e6 / tick_hz if the oscillator were exact)
    sigma_us: float       # the scatter of the pairs about the line (one standard deviation)
    n: int
    x_mean_s: float       # the centre of the data, in seconds of counter time from ticks0
    sxx_s2: float         # the spread of the data in time (sum of squared distances from the centre)


@dataclass
class Correlator:
    tick_hz: int
    max_pairs: int = 512
    gate_us: float = DEFAULT_GATE_US
    latency_floor_us: float = 1_000.0
    pairs: list[tuple[int, float]] = field(default_factory=list)
    rejected: int = 0
    resets: int = 0                 # times the counter went backwards: the supervisor restarted, its old pairs no longer apply
    _fit: Fit | None = None
    _stride: int = 1
    _skip: int = 0

    # ---------------------------------------------------------------- input
    def add(self, ticks: int, utc_us: float) -> bool:
        """Record a pair. Returns False if it was rejected (the counter went backwards and started a new run, or the pair is an outlier)."""
        if self.pairs and ticks < self.pairs[-1][0]:
            self.pairs.clear()
            self._fit = None
            self._stride, self._skip = 1, 0
            self.resets += 1
        if self._fit is not None and abs(self._residual_us(ticks, utc_us)) > self.gate_us:
            self.rejected += 1
            return False
        if self._skip > 0:
            self._skip -= 1                 # the thinned-out record keeps one pair in `_stride`, so a long baseline survives
            return True
        self.pairs.append((ticks, utc_us))
        self._skip = self._stride - 1
        if len(self.pairs) > self.max_pairs:
            self.pairs = self.pairs[::2]    # halve the density, keep both ends of the baseline
            self._stride *= 2
            self._skip = self._stride - 1
        self._refit()
        return True

    # ---------------------------------------------------------------- fit
    def _refit(self) -> None:
        n = len(self.pairs)
        if n < 2:
            self._fit = None
            return
        t0 = self.pairs[0][0]
        xs = [(t - t0) / self.tick_hz for t, _ in self.pairs]
        ys = [u for _, u in self.pairs]
        xm = sum(xs) / n
        ym = sum(ys) / n
        sxx = sum((x - xm) ** 2 for x in xs)
        if sxx <= 0.0:
            self._fit = None
            return
        sxy = sum((x - xm) * (y - ym) for x, y in zip(xs, ys))
        slope_us_per_s = sxy / sxx
        resid = [y - (ym + slope_us_per_s * (x - xm)) for x, y in zip(xs, ys)]
        sigma = math.sqrt(sum(r * r for r in resid) / (n - 2)) if n > 2 else 0.0
        self._fit = Fit(ticks0=float(t0), utc0_us=ym - slope_us_per_s * xm, rate=slope_us_per_s / self.tick_hz, sigma_us=sigma, n=n, x_mean_s=xm, sxx_s2=sxx)

    def _residual_us(self, ticks: int, utc_us: float) -> float:
        assert self._fit is not None
        return utc_us - (self._fit.utc0_us + self._fit.rate * (ticks - self._fit.ticks0))

    # ---------------------------------------------------------------- output
    @property
    def fit(self) -> Fit | None:
        return self._fit

    def span_s(self) -> float:
        return (self.pairs[-1][0] - self.pairs[0][0]) / self.tick_hz if len(self.pairs) >= 2 else 0.0

    def ok(self) -> bool:
        """Enough pairs over enough time to say anything about the drift."""
        return self._fit is not None and self._fit.n >= MIN_PAIRS and self.span_s() >= MIN_SPAN_S

    def drift_ppm(self) -> float | None:
        """How much faster than UTC the supervisor's oscillator runs, in parts per million (negative: slower)."""
        if not self.ok():
            return None
        assert self._fit is not None
        return (1.0e6 / (self.tick_hz * self._fit.rate) - 1.0) * 1.0e6  # ticks counted per true second, against the nominal rate

    def to_utc_us(self, ticks: int) -> float | None:
        if not self.ok():
            return None
        assert self._fit is not None
        return self._fit.utc0_us + self._fit.rate * (ticks - self._fit.ticks0)

    def error_bound_us(self, ticks: int, sigmas: float = 3.0) -> float | None:
        """A bound on the error of `to_utc_us(ticks)`: `sigmas` standard deviations of the prediction (the scatter, plus the uncertainty of the fitted line,
        which grows with the distance from the data: an hour of pairs says little about year five) plus the stated latency allowance."""
        if not self.ok():
            return None
        assert self._fit is not None
        f = self._fit
        x = (ticks - f.ticks0) / self.tick_hz
        var = f.sigma_us**2 * (1.0 / f.n + (x - f.x_mean_s) ** 2 / f.sxx_s2)
        return sigmas * math.sqrt(var) + self.latency_floor_us
