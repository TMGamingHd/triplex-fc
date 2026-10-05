# SPDX-License-Identifier: MIT
"""Time correlation (tfc_peers/timecorr.py): the fit recovers a known oscillator error from noisy pairs, says how far the converted time can be trusted, and survives a stall and a restart."""
import random
import unittest

from tfc_peers.timecorr import Correlator

HZ = 1_000_000
YEAR_S = 365 * 86400


class Oscillator:
    """A supervisor counter that runs `ppm` fast, started at `epoch_utc_us`, read through a PC with `jitter_us` of Gaussian latency scatter and a constant `latency_us`."""

    def __init__(self, ppm=20.0, epoch_utc_us=1_800_000_000_000_000.0, jitter_us=2000.0, latency_us=500.0, seed=1):
        self.ppm, self.epoch, self.jitter, self.latency = ppm, epoch_utc_us, jitter_us, latency_us
        self.rng = random.Random(seed)

    def ticks_at(self, true_s):
        return int(round(true_s * HZ * (1.0 + self.ppm * 1e-6)))

    def true_utc_us(self, ticks):
        return self.epoch + ticks / (HZ * (1.0 + self.ppm * 1e-6)) * 1e6

    def pair(self, true_s):
        t = self.ticks_at(true_s)
        return t, self.true_utc_us(t) + self.latency + self.rng.gauss(0.0, self.jitter)


class TestCorrelator(unittest.TestCase):
    def feed(self, c, osc, seconds, step=60.0):
        t = 0.0
        while t <= seconds:
            c.add(*osc.pair(t))
            t += step

    def test_nothing_is_said_without_enough_pairs_over_enough_time(self):
        c = Correlator(HZ)
        osc = Oscillator()
        self.assertFalse(c.ok())
        self.assertIsNone(c.drift_ppm())
        self.assertIsNone(c.to_utc_us(1))
        self.assertIsNone(c.error_bound_us(1))
        for t in range(4):
            c.add(*osc.pair(float(t) * 5.0))
        self.assertFalse(c.ok())            # four pairs
        c.add(*osc.pair(21.0))
        self.assertTrue(c.ok())             # five pairs over 21 s
        c2 = Correlator(HZ)
        for t in range(8):
            c2.add(*osc.pair(t * 0.5))      # eight pairs in 3.5 s: too short a baseline to fit a drift
        self.assertFalse(c2.ok())

    def test_a_20_ppm_oscillator_is_measured_to_under_a_ppm_from_an_hour_of_noisy_pairs(self):
        for seed in range(5):
            osc = Oscillator(ppm=20.0, seed=seed)
            c = Correlator(HZ)
            self.feed(c, osc, 3600.0)
            self.assertAlmostEqual(c.drift_ppm(), 20.0, delta=1.0)

    def test_negative_drift_and_an_exact_clock(self):
        c = Correlator(HZ)
        self.feed(c, Oscillator(ppm=-35.0, jitter_us=500.0), 1800.0)
        self.assertAlmostEqual(c.drift_ppm(), -35.0, delta=1.0)
        c = Correlator(HZ)
        self.feed(c, Oscillator(ppm=0.0, jitter_us=500.0), 1800.0)
        self.assertAlmostEqual(c.drift_ppm(), 0.0, delta=1.0)

    def test_conversion_to_utc_is_inside_its_own_error_bound_now_and_five_years_out(self):
        for seed in range(8):
            osc = Oscillator(ppm=20.0, seed=seed)
            c = Correlator(HZ)
            self.feed(c, osc, 3600.0)
            for true_s in (1800.0, 3600.0, 86400.0, YEAR_S, 5 * YEAR_S):
                t = osc.ticks_at(true_s)
                err = abs(c.to_utc_us(t) - osc.true_utc_us(t))
                self.assertLessEqual(err, c.error_bound_us(t), f"seed {seed} at {true_s} s")
            # and the bound is honest about distance: five years from an hour of data is seconds, not milliseconds
            far = osc.ticks_at(5 * YEAR_S)
            near = osc.ticks_at(1800.0)
            self.assertGreater(c.error_bound_us(far), 1000.0 * c.error_bound_us(near))

    def test_a_longer_baseline_tightens_the_far_bound(self):
        osc = Oscillator(ppm=20.0, seed=3)
        short, long_ = Correlator(HZ), Correlator(HZ)
        self.feed(short, osc, 3600.0)
        osc = Oscillator(ppm=20.0, seed=3)
        self.feed(long_, osc, 30 * 86400.0, step=3600.0)
        far = osc.ticks_at(5 * YEAR_S)
        self.assertLess(long_.error_bound_us(far), short.error_bound_us(far) / 10.0)

    def test_the_latency_allowance_is_in_every_bound_and_a_constant_latency_is_not_hidden_by_the_fit(self):
        osc = Oscillator(jitter_us=0.0, latency_us=3000.0)
        c = Correlator(HZ, latency_floor_us=5000.0)
        self.feed(c, osc, 600.0, step=30.0)
        t = osc.ticks_at(300.0)
        self.assertAlmostEqual(c.error_bound_us(t), 5000.0, delta=1.0)         # no scatter: the bound is the allowance
        self.assertAlmostEqual(c.to_utc_us(t) - osc.true_utc_us(t), 3000.0, delta=1.0)   # the fixed latency is a bias the pairs cannot show: inside the allowance

    def test_a_stall_of_the_pc_is_rejected_and_does_not_spoil_the_fit(self):
        osc = Oscillator(ppm=20.0, jitter_us=300.0)
        c = Correlator(HZ)
        self.feed(c, osc, 1200.0, step=30.0)
        before = c.drift_ppm()
        t = osc.ticks_at(1230.0)
        self.assertFalse(c.add(t, osc.true_utc_us(t) + 400_000.0))      # the stamp was 0.4 s late
        self.assertEqual(c.rejected, 1)
        self.assertAlmostEqual(c.drift_ppm(), before, places=9)
        self.assertTrue(c.add(*osc.pair(1260.0)))                       # and the next good one is taken

    def test_a_restart_of_the_supervisor_starts_a_new_run(self):
        osc = Oscillator()
        c = Correlator(HZ)
        self.feed(c, osc, 600.0, step=30.0)
        self.assertTrue(c.ok())
        # the supervisor resets: its counter restarts from zero (a pair is far from the old line, but must not count as an outlier)
        self.assertTrue(c.add(5_000_000, osc.epoch + 700e6))
        self.assertEqual(c.resets, 1)
        self.assertEqual(len(c.pairs), 1)
        self.assertFalse(c.ok())

    def test_the_record_is_bounded_and_keeps_the_whole_baseline(self):
        osc = Oscillator(ppm=20.0, jitter_us=1000.0, seed=4)
        c = Correlator(HZ, max_pairs=64)
        self.feed(c, osc, 10 * 86400.0, step=600.0)      # 1 440 pairs into a record of 64
        self.assertLessEqual(len(c.pairs), 64)
        self.assertGreater(c.span_s(), 9.5 * 86400.0)    # the baseline is still the whole ten days
        self.assertAlmostEqual(c.drift_ppm(), 20.0, delta=0.1)

    def test_identical_counter_values_do_not_divide_by_zero(self):
        c = Correlator(HZ)
        for _ in range(10):
            c.add(1000, 1.8e15)
        self.assertFalse(c.ok())
        self.assertIsNone(c.fit)


if __name__ == "__main__":
    unittest.main()
