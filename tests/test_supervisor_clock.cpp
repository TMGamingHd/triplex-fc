// SPDX-License-Identifier: MIT
// The supervisor's clock of record (supervisor/include/sup/mission_clock.hpp, docs/MISSION_CLOCK.md): MET across years and resets, the battery-backed
// record, and the plausibility check of a flight computer's mission time. The header shares no code with core/; this file is the one place that includes both,
// to pin the few protocol facts the supervisor restates.
#include <cmath>
#include <cstdint>

#include "sup/mission_clock.hpp"
#include "tfc/flight.hpp"
#include "tfc/protocol.hpp"
#include "tfc_test.hpp"

namespace {

constexpr uint64_t kMhz = 1000000U;
constexpr uint64_t kFiveYearsS = 5ULL * 365ULL * 86400ULL;

TFC_TEST(sup_clock_restated_protocol_facts_match_the_flight_protocol) {
  CHECK(sup::kFrameUs == static_cast<uint32_t>(std::lround(static_cast<double>(tfc::kFramePeriodS) * 1.0e6)));
  CHECK(sup::kMissionFirstFlight == tfc::mission::kCountdownFrames + 1U);
  CHECK(sup::kMissionSaturated == tfc::mission::kMax);
  CHECK(tfc::mission::flight_frames(sup::kMissionFirstFlight) == 0U && tfc::mission::flight_frames(static_cast<uint16_t>(sup::kMissionFirstFlight + 7U)) == 7U);
}

TFC_TEST(sup_clock_extends_a_32_bit_timer_to_64_bits_across_wraps) {
  sup::TickExtender ext;
  CHECK(ext.extend(4000000000U) == 4000000000ULL);  // the first reading is taken as it is
  CHECK(ext.extend(4200000000U) == 4200000000ULL);
  CHECK(ext.extend(100U) == (1ULL << 32U) + 100U);  // wrapped once
  CHECK(ext.extend(100U) == (1ULL << 32U) + 100U);  // the same reading again is not a wrap
  CHECK(ext.extend(3000000000U) == (1ULL << 32U) + 3000000000ULL);
  CHECK(ext.extend(5U) == (2ULL << 32U) + 5U);
  sup::TickExtender fresh;
  CHECK(fresh.extend(7U) == 7U && fresh.extend(9U) == 9U);
}

TFC_TEST(sup_clock_is_not_launched_until_t0_and_counts_exactly_after_it) {
  sup::MissionClock c;
  CHECK(c.config_ok() && !c.launched() && c.source() == sup::MetSource::None);
  CHECK(c.met_ticks(123456U) == 0U && c.met_us(123456U) == 0U && c.met_frames(123456U) == 0U);
  const sup::MetRecord r = c.latch_t0(5U * kMhz, 777U);
  CHECK(sup::record_valid(r) && r.t0_rtc_s == 777U);
  CHECK(c.launched() && c.source() == sup::MetSource::Counted && c.recovery_error_us() == 0U);
  CHECK(c.met_ticks(5U * kMhz) == 0U && c.met_ticks(5U * kMhz + 1U) == 1U);
  CHECK(c.met_us(5U * kMhz + 1500000U) == 1500000U);
  CHECK(c.met_frames(5U * kMhz + 9999U) == 0U && c.met_frames(5U * kMhz + 10000U) == 1U && c.met_frames(5U * kMhz + 1000000U) == 100U);
  CHECK(c.met_ticks(4U * kMhz) == 0U);  // a tick before T-zero (a stale reading) is 0, not a huge wrapped number
}

TFC_TEST(sup_clock_the_first_t0_stands_and_a_repeat_changes_nothing) {
  sup::MissionClock c;
  const sup::MetRecord first = c.latch_t0(1000U, 50U);
  const sup::MetRecord again = c.latch_t0(900000U, 60U);
  CHECK(again.t0_rtc_s == 50U && again.crc == first.crc);
  CHECK(c.met_ticks(2000U) == 1000U);
}

TFC_TEST(sup_clock_met_does_not_overflow_in_five_years_and_a_non_megahertz_timer_works) {
  sup::MissionClock c;
  (void)c.latch_t0(0U, 0U);
  const uint64_t end = kFiveYearsS * kMhz + 250000U;  // five years and a quarter second
  CHECK(c.met_us(end) == kFiveYearsS * 1000000U + 250000U);
  CHECK(c.met_frames(end) == kFiveYearsS * 100U + 25U);
  sup::MissionClock slow(sup::ClockConfig{3276800U});  // 3 276 800 Hz: still a multiple of 100
  CHECK(slow.config_ok());
  (void)slow.latch_t0(0U, 0U);
  CHECK(slow.met_frames(3276800U) == 100U && slow.met_us(3276800U) == 1000000U);
  CHECK(!sup::MissionClock(sup::ClockConfig{32768U}).config_ok() && !sup::MissionClock(sup::ClockConfig{50U}).config_ok());
}

TFC_TEST(sup_clock_the_record_detects_corruption) {
  const sup::MetRecord r = sup::make_record(123456789012ULL);
  CHECK(sup::record_valid(r));
  for (unsigned bit = 0; bit < 64U; ++bit) {
    sup::MetRecord bad = r;
    bad.t0_rtc_s ^= (1ULL << bit);
    CHECK(!sup::record_valid(bad));
  }
  sup::MetRecord bad_magic = r;
  bad_magic.magic ^= 1U;
  sup::MetRecord bad_crc = r;
  bad_crc.crc = static_cast<uint16_t>(bad_crc.crc ^ 0x8000U);
  CHECK(!sup::record_valid(bad_magic) && !sup::record_valid(bad_crc) && !sup::record_valid(sup::MetRecord{}));
  CHECK(!sup::record_valid(sup::MetRecord{0x12345678U, 5U, sup::met_crc(0x12345678U, 5U)}));  // a record with a good CRC but not ours (another format, or leftover memory)
  CHECK(sup::met_crc(sup::kMetMagic, 0U) != sup::met_crc(sup::kMetMagic, 1U));
  CHECK(sup::met_crc(sup::kMetMagic, 1ULL << 32U) != sup::met_crc(sup::kMetMagic, 1ULL << 40U));  // the high bytes count
}

TFC_TEST(sup_clock_a_reset_recovers_met_from_the_rtc_to_a_second) {
  sup::MissionClock before;
  const sup::MetRecord rec = before.latch_t0(2U * kMhz, 1000U);
  // 90 s of mission, then the supervisor resets: its timer restarts at 0; 12 s later (the reboot took that long) it reads the RTC.
  sup::MissionClock after;
  CHECK(after.resume(rec, 3U * kMhz, 1000U + 90U + 12U));  // the timer says 3 s since this boot; the RTC says 102 s since T-zero
  CHECK(after.source() == sup::MetSource::Recovered && after.recovery_error_us() == 1000000U);
  CHECK(after.met_us(3U * kMhz) == 102ULL * 1000000U);
  CHECK(after.met_us(4U * kMhz) == 103ULL * 1000000U);  // and it counts on from there
  // T-zero before this run's tick zero (the timer has run 1 s, the mission 102 s): represented with an offset, and still exact afterwards
  sup::MissionClock offset;
  CHECK(offset.resume(rec, 1U * kMhz, 1000U + 102U));
  CHECK(offset.met_us(1U * kMhz) == 102ULL * 1000000U && offset.met_us(3U * kMhz) == 104ULL * 1000000U);
  // the same instant: no mission time has passed
  sup::MissionClock zero;
  CHECK(zero.resume(rec, 5U, 1000U) && zero.met_ticks(5U) == 0U && zero.met_ticks(10U) == 5U);
}

TFC_TEST(sup_clock_a_bad_or_impossible_record_is_not_used) {
  const sup::MetRecord rec = sup::make_record(5000U);
  sup::MissionClock c;
  CHECK(!c.resume(rec, 0U, 4999U) && !c.launched());  // T-zero in the RTC's future: the RTC was set back, or the record is from another run
  sup::MetRecord corrupt = rec;
  corrupt.t0_rtc_s += 1U;
  CHECK(!c.resume(corrupt, 0U, 9000U) && !c.launched() && c.met_us(1000U) == 0U);
  CHECK(!c.resume(sup::MetRecord{}, 0U, 9000U) && !c.launched());  // never written
  CHECK(c.resume(rec, 0U, 5000U) && c.launched());
}

// The plausibility check: a node's SYNC mission field against MET.
TFC_TEST(sup_watch_agrees_with_an_exact_node_and_says_nothing_before_t0) {
  sup::MissionClock c;
  sup::MissionWatch w;
  CHECK(w.check(c, 12345U, 0U) == sup::Verdict::NotLaunched && w.last() == sup::Verdict::NotLaunched && !w.flagged());
  (void)c.latch_t0(0U, 0U);
  for (uint64_t s = 0; s < 600U; s += 7U) {
    const uint64_t frames = s * 100U;
    CHECK(w.check(c, s * kMhz, static_cast<uint16_t>(sup::kMissionFirstFlight + frames)) == sup::Verdict::Ok);
  }
  CHECK(!w.flagged() && w.last() == sup::Verdict::Ok);
}

TFC_TEST(sup_watch_flags_a_node_that_is_behind_or_ahead_only_after_it_persists) {
  sup::MissionClock c;
  (void)c.latch_t0(0U, 0U);
  sup::MissionWatch w;  // base 3 frames, 200 ppm, persist 3
  const uint64_t t = 100U * kMhz;  // 10 000 frames of MET: the allowance is 3 + 2 = 5 frames
  const uint16_t exact = static_cast<uint16_t>(sup::kMissionFirstFlight + 10000U);
  CHECK(w.check(c, t, static_cast<uint16_t>(exact - 5U)) == sup::Verdict::Ok);   // at the edge
  CHECK(w.check(c, t, static_cast<uint16_t>(exact + 5U)) == sup::Verdict::Ok);
  CHECK(w.check(c, t, static_cast<uint16_t>(exact - 6U)) == sup::Verdict::Behind && !w.flagged());
  CHECK(w.check(c, t, static_cast<uint16_t>(exact - 6U)) == sup::Verdict::Behind && !w.flagged());
  CHECK(w.check(c, t, static_cast<uint16_t>(exact - 6U)) == sup::Verdict::Behind && w.flagged());   // the third in a row
  CHECK(w.check(c, t, exact) == sup::Verdict::Ok && !w.flagged());                                    // one good check clears it
  CHECK(w.check(c, t, static_cast<uint16_t>(exact + 6U)) == sup::Verdict::Ahead);
  CHECK(w.check(c, t, static_cast<uint16_t>(exact - 6U)) == sup::Verdict::Behind);                    // alternating still counts as bad
  CHECK(w.check(c, t, static_cast<uint16_t>(exact + 6U)) == sup::Verdict::Ahead && w.flagged());
  sup::MissionWatch stuck;  // a node that never launched while MET is 100 s
  for (unsigned i = 0; i < 258U; ++i) {  // 258 mod 256 = 2: a run counter that wrapped would have forgotten it
    (void)stuck.check(c, t, 0U);
  }
  CHECK(stuck.flagged() && stuck.last() == sup::Verdict::Behind);  // the bad run saturates, it does not wrap to "good"
  const sup::MissionClock unlaunched;
  CHECK(stuck.check(unlaunched, t, 0U) == sup::Verdict::NotLaunched && !stuck.flagged());  // nothing is flagged against a clock that has no T-zero (a supervisor that restarted unlaunched)
}

TFC_TEST(sup_watch_allowance_grows_with_the_drift_of_the_node_and_the_field_saturates_like_sync) {
  sup::MissionClock c;
  (void)c.latch_t0(0U, 0U);
  sup::MissionWatch w;
  // 600 s: 60 000 frames of MET but the field stops at 65 535 = 654 s: at 600 s the field is 61 001; allowance 3 + 12 = 15 frames
  const uint64_t t = 600U * kMhz;
  CHECK(w.check(c, t, 61001U + 15U) == sup::Verdict::Ok && w.check(c, t, 61001U + 16U) == sup::Verdict::Ahead);
  CHECK(w.check(c, t, 61001U - 15U) == sup::Verdict::Ok && w.check(c, t, 61001U - 16U) == sup::Verdict::Behind);
  // an hour in: the expected field is saturated, and a node whose field is saturated agrees; one that is not, does not
  const uint64_t hour = 3600U * kMhz;
  CHECK(w.check(c, hour, sup::kMissionSaturated) == sup::Verdict::Ok);
  CHECK(w.check(c, hour, 40000U) == sup::Verdict::Behind);
}

TFC_TEST(sup_watch_a_recovered_t0_is_allowed_its_second_of_uncertainty) {
  const sup::MetRecord rec = sup::make_record(1000U);
  sup::MissionClock c;
  CHECK(c.resume(rec, 10U * kMhz, 1100U));  // MET 100 s, known to a second
  sup::MissionWatch w;
  const uint16_t exact = static_cast<uint16_t>(sup::kMissionFirstFlight + 10000U);
  CHECK(w.check(c, 10U * kMhz, static_cast<uint16_t>(exact - 100U)) == sup::Verdict::Ok);   // a whole second of frames, plus the 5 above: 105 is the edge
  CHECK(w.check(c, 10U * kMhz, static_cast<uint16_t>(exact - 105U)) == sup::Verdict::Ok);
  CHECK(w.check(c, 10U * kMhz, static_cast<uint16_t>(exact - 106U)) == sup::Verdict::Behind);
  sup::MissionClock counted;
  (void)counted.latch_t0(0U, 0U);
  CHECK(w.check(counted, 100U * kMhz, static_cast<uint16_t>(exact - 100U)) == sup::Verdict::Behind);  // the same departure from an exact clock is a fault
}

}  // namespace
