// SPDX-License-Identifier: MIT
// The flight log (core/include/tfc/flightlog.hpp, docs/design/FUTURE_WORK.md 6b item 12): the ring of the last frames and the record of a fatal fault, kept in memory that a reset does not clear. A store that
// was never written (garbage) starts empty, one that was written survives a boot, a record that is damaged is found out, and the numbers are kept in hundredths with saturation.
#include <cstdint>
#include <cstring>

#include "tfc/flightlog.hpp"
#include "tfc_test.hpp"

using namespace tfc;

namespace {

constexpr std::size_t kN = 8U;

void fill(FlightLog<kN>& log, uint32_t from, uint32_t count) {
  for (uint32_t i = 0; i < count; ++i) {
    log.push(from + i, 2U, 0x07U, 0x00U, static_cast<uint8_t>(i & 3U), 5.0F + (0.01F * static_cast<float>(i)), -1.25F, 0.5F);
  }
}

}  // namespace

TFC_TEST(flightlog_a_store_that_was_never_written_starts_empty_and_a_written_one_survives_a_boot) {
  FlightLogStore<kN> store;
  std::memset(&store, 0xA5, sizeof store);   // a power-on: whatever the RAM held
  FlightLog<kN> log(store);
  CHECK(!log.boot());
  CHECK(log.size() == 0U && !log.has_fault());
  fill(log, 100U, 3U);
  CHECK(log.size() == 3U);
  FlightLog<kN> again(store);                // the next boot sees the same memory
  CHECK(again.boot());
  CHECK(again.size() == 3U);
  bool ok = false;
  const FlightRecord newest = again.get(0, ok);
  CHECK(ok && newest.frame == 102U && newest.mode == 2U && newest.valid_mask == 7U && newest.output == 502 && newest.cmd_pitch == -125 && newest.cmd_yaw == 50);
}

TFC_TEST(flightlog_the_ring_keeps_the_newest_in_order_and_overwrites_the_oldest) {
  FlightLogStore<kN> store{};
  FlightLog<kN> log(store);
  CHECK(!log.boot());                         // (zeroed: no magic)
  fill(log, 0U, 20U);                         // two and a half times round
  CHECK(log.size() == kN);
  for (std::size_t age = 0; age < kN; ++age) {
    bool ok = false;
    const FlightRecord r = log.get(age, ok);
    CHECK(ok && r.frame == 19U - age);
  }
  bool ok = true;
  (void)log.get(kN, ok);                      // older than the ring holds: none
  CHECK(!ok);
}

TFC_TEST(flightlog_a_damaged_record_is_found_out_and_a_damaged_header_empties_the_ring) {
  FlightLogStore<kN> store{};
  FlightLog<kN> log(store);
  (void)log.boot();
  fill(log, 10U, 5U);
  store.rec[2].output = static_cast<int16_t>(store.rec[2].output ^ 0x40);       // a stray write into the record of frame 12
  bool ok = true;
  (void)log.get(2, ok);                       // age 2 of frames 10..14 is frame 12
  CHECK(!ok);
  (void)log.get(1, ok);
  CHECK(ok);
  store.head = 3U;                            // the header no longer matches its own CRC
  FlightLog<kN> after(store);
  CHECK(!after.boot() && after.size() == 0U);
  store.magic = 0U;                           // and a store with another magic number is not a log either
  FlightLog<kN> third(store);
  CHECK(!third.boot());
  FlightLogStore<kN> stray{};
  FlightLog<kN> g(stray);
  (void)g.boot();
  stray.count = static_cast<uint32_t>(kN) + 1U;   // a count larger than the ring: refused whatever the CRC says
  FlightLog<kN> h(stray);
  CHECK(!h.boot());
}

TFC_TEST(flightlog_the_fault_record_keeps_what_the_fault_knew_until_it_is_cleared_and_notices_damage) {
  FlightLogStore<kN> store{};
  FlightLog<kN> log(store);
  (void)log.boot();
  CHECK(!log.has_fault());
  log.record_fault(4U, 0x08001234U, 0x08005678U, 77U);
  CHECK(log.has_fault());
  const FaultRecord f = log.fault();
  CHECK(f.reason == 4U && f.pc == 0x08001234U && f.lr == 0x08005678U && f.frame == 77U);
  FlightLog<kN> again(store);
  CHECK(again.boot() && again.has_fault());   // it survives a boot with the ring
  store.fault.pc ^= 1U;                       // damaged
  CHECK(!again.has_fault());
  log.clear_fault();
  CHECK(!log.has_fault());
}

TFC_TEST(flightlog_numbers_are_kept_in_hundredths_rounded_saturated_and_a_nan_is_zero) {
  using logdetail::hundredths;
  CHECK(hundredths(0.0F) == 0 && hundredths(1.234F) == 123 && hundredths(-1.236F) == -124 && hundredths(0.005F) == 1);
  CHECK(hundredths(400.0F) == 32767 && hundredths(-400.0F) == -32768 && hundredths(327.0F) == 32700);
  CHECK(hundredths(std::nanf("")) == 0);
}
