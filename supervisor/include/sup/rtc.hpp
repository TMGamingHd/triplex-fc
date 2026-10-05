// SPDX-License-Identifier: MIT
// The supervisor's battery-backed real-time clock: seconds since 2000-01-01 00:00:00 from the seven time registers of a DS3231 (the TCXO RTC module, docs/design/MISSION_CLOCK.md section 3) and its
// status register. A reading is valid only if every field is a legal BCD value of a real date and the oscillator-stop flag is clear (the flag says the time may have stopped: after a
// total power loss of the module's battery too). Portable and integer-only; the I2C read itself is the hardware layer's.
#pragma once
#include <array>
#include <cstdint>

namespace sup {

struct RtcReading {
  uint64_t seconds = 0U;  // since 2000-01-01 00:00:00
  bool valid = false;
};

namespace detail {

constexpr bool bcd_ok(uint8_t v) noexcept { return (v & 0x0FU) <= 9U && (v >> 4U) <= 9U; }
constexpr unsigned bcd(uint8_t v) noexcept { return static_cast<unsigned>((v >> 4U) * 10U + (v & 0x0FU)); }
constexpr bool leap(unsigned y) noexcept { return (y % 4U == 0U && y % 100U != 0U) || y % 400U == 0U; }

constexpr unsigned days_in_month(unsigned y, unsigned m) noexcept {
  constexpr unsigned days[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
  return (m == 2U && leap(y)) ? 29U : days[m - 1U];
}

// Days from 2000-01-01 to year-month-day (y >= 2000), counted year by year and month by month: at most 200 + 12 steps, so no closed form to get wrong.
constexpr uint64_t days_since_2000(unsigned y, unsigned m, unsigned d) noexcept {
  uint64_t days = 0U;
  for (unsigned yy = 2000U; yy < y && yy < 2200U; ++yy) {
    days += leap(yy) ? 366U : 365U;
  }
  for (unsigned mm = 1U; mm < m; ++mm) {
    days += days_in_month(y, mm);
  }
  return days + (d - 1U);
}

}  // namespace detail

// `r` is registers 0x00 to 0x06 (seconds, minutes, hours, day of week, date, month with the century bit, year), `status` register 0x0F.
[[nodiscard]] constexpr RtcReading ds3231_seconds(const std::array<uint8_t, 7>& r, uint8_t status) noexcept {
  RtcReading out;
  const uint8_t month_reg = static_cast<uint8_t>(r[5] & 0x7FU);
  if ((status & 0x80U) != 0U) {  // oscillator stopped: the time cannot be trusted
    return out;
  }
  if (!detail::bcd_ok(r[0]) || !detail::bcd_ok(r[1]) || !detail::bcd_ok(r[2] & 0x3FU) || !detail::bcd_ok(r[4]) || !detail::bcd_ok(month_reg) || !detail::bcd_ok(r[6]) ||
      (r[0] & 0x80U) != 0U || (r[1] & 0x80U) != 0U) {
    return out;
  }
  if ((r[2] & 0x40U) != 0U) {  // 12-hour mode: the module is set to 24-hour by the supervisor, so anything else is not ours
    return out;
  }
  const unsigned sec = detail::bcd(r[0]);
  const unsigned min = detail::bcd(r[1]);
  const unsigned hour = detail::bcd(static_cast<uint8_t>(r[2] & 0x3FU));
  const unsigned date = detail::bcd(r[4]);
  const unsigned month = detail::bcd(month_reg);
  const unsigned year = 2000U + detail::bcd(r[6]) + ((r[5] & 0x80U) != 0U ? 100U : 0U);
  if (sec > 59U || min > 59U || hour > 23U || month < 1U || month > 12U || date < 1U || date > detail::days_in_month(year, month)) {
    return out;
  }
  out.seconds = (detail::days_since_2000(year, month, date) * 86400U) + (static_cast<uint64_t>(hour) * 3600U) + (static_cast<uint64_t>(min) * 60U) + sec;
  out.valid = true;
  return out;
}

}  // namespace sup
