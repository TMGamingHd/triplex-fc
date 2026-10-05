// SPDX-License-Identifier: MIT
// The Pico's portable logic: the USB link framing and parser (pico_link.hpp), the platform driver's safety (platform_driver.hpp: PLAT-001, 002, 004), the servo map,
// and the injector's relay logic (injector.hpp). The golden frames below are pinned in the Python client's tests too (sim/tests/test_pico_link.py), made with an
// independent CRC, so a change on one side fails a test until both agree.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "tfc/injector.hpp"
#include "tfc/pico_link.hpp"
#include "tfc/platform_driver.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;
using namespace tfc::pico;

std::string hex(const Message& m) {
  std::array<uint8_t, kMaxFrame> b{};
  const std::size_t n = encode(m, b.data());
  std::string s;
  for (std::size_t i = 0; i < n; ++i) {
    std::array<char, 3> c{};
    (void)std::snprintf(c.data(), c.size(), "%02x", b[i]);
    s += c.data();
  }
  return s;
}

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

}  // namespace

TFC_TEST(pico_link_golden_frames_are_pinned) {
  PlatformCommand pc;
  pc.seq = 7U;
  pc.tilt_x_deg = 1.234F;
  pc.tilt_y_deg = -5.678F;
  CHECK(hex(pack_platform(pc)) == "a50105077b00c8fd45");
  CHECK(hex(pack_relay(RelayCommand{2U, 1500U})) == "a5020302dc05f4");
  CHECK(hex(pack_ping()) == "a503006a");
  Status st;
  st.seq_echo = 9U;
  st.out_x_deg = 12.34F;
  st.out_y_deg = -45.0F;
  st.flags = statusflag::kSaturated | statusflag::kLevelling;
  st.relays = 6U;
  st.command_age_ms = 250U;
  CHECK(hex(pack_status(st)) == "a5810909d2046cee0506fa0094");
  CHECK(hex(pack_status(Status{})) == "a58109000000000000000000b8");
  PlatformCommand sat;
  sat.seq = 255U;
  sat.tilt_x_deg = 1.0e9F;   // saturates in the 16-bit field
  sat.tilt_y_deg = -1.0e9F;
  CHECK(hex(pack_platform(sat)) == "a50105ffff7f0080b5");
  PlatformCommand wide;  // 330 degrees is 33,000 counts: beyond the 16-bit field, so it saturates (it must not wrap)
  wide.tilt_x_deg = 330.0F;
  wide.tilt_y_deg = -330.0F;
  const Message wm = pack_platform(wide);
  CHECK(wm.payload[1] == 0xFFU && wm.payload[2] == 0x7FU && wm.payload[3] == 0x00U && wm.payload[4] == 0x80U);
  PlatformCommand not_a_number;
  not_a_number.tilt_x_deg = std::nanf("");  // no angle: zero, not a random value
  not_a_number.tilt_y_deg = 0.0F;
  CHECK(pack_platform(not_a_number).payload[1] == 0U && pack_platform(not_a_number).payload[2] == 0U);
}

TFC_TEST(pico_link_round_trip_and_the_parser_finds_frames_in_a_stream) {
  PlatformCommand pc;
  pc.seq = 200U;
  pc.tilt_x_deg = -33.33F;
  pc.tilt_y_deg = 44.44F;
  std::array<uint8_t, kMaxFrame> b{};
  Parser p;
  Message m;
  std::size_t n = encode(pack_platform(pc), b.data());
  bool got = false;
  for (std::size_t i = 0; i < n; ++i) {
    got = p.feed(b[i], m);
  }
  PlatformCommand back;
  CHECK(got && unpack_platform(m, back) && back.seq == 200U && near(back.tilt_x_deg, -33.33F, 0.006F) && near(back.tilt_y_deg, 44.44F, 0.006F));
  RelayCommand rc;
  CHECK(unpack_relay(pack_relay(RelayCommand{3U, 65535U}), rc) && rc.channel == 3U && rc.cut_ms == 65535U);
  Status s;
  CHECK(unpack_status(pack_status(Status{4U, -1.5F, 2.5F, 0x20U, 0x0FU, 12345U}), s) && s.seq_echo == 4U && near(s.out_x_deg, -1.5F, 0.006F) && s.relays == 0x0FU &&
        s.command_age_ms == 12345U);
  CHECK(!unpack_platform(pack_ping(), back) && !unpack_relay(pack_ping(), rc) && !unpack_status(pack_ping(), s));  // wrong type
  Message short_status = pack_status(Status{});
  short_status.length = 8U;
  Message long_status = pack_status(Status{});
  long_status.length = 10U;
  CHECK(!unpack_status(short_status, s) && !unpack_status(long_status, s));  // wrong length
  Message bad_relay = pack_relay(RelayCommand{1U, 5U});
  bad_relay.length = 2U;
  CHECK(!unpack_relay(bad_relay, rc));
  Message wrong = pack_platform(pc);
  wrong.length = 4U;
  CHECK(!unpack_platform(wrong, back));  // wrong length
}

TFC_TEST(pico_link_the_parser_resynchronises_after_garbage_a_bad_crc_and_a_false_start) {
  std::array<uint8_t, kMaxFrame> good{};
  const std::size_t gn = encode(pack_relay(RelayCommand{1U, 100U}), good.data());
  Parser p;
  Message m;
  unsigned frames = 0U;
  auto feed = [&](const uint8_t* d, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
      frames += p.feed(d[i], m) ? 1U : 0U;
    }
  };
  const std::array<uint8_t, 5> junk{0x00U, 0x13U, 0xFFU, 0x42U, 0x99U};
  feed(junk.data(), junk.size());
  CHECK(frames == 0U && p.skipped_bytes() == 5U);
  feed(good.data(), gn);
  CHECK(frames == 1U);
  std::array<uint8_t, kMaxFrame> bad = good;
  bad[gn - 1U] = static_cast<uint8_t>(bad[gn - 1U] ^ 0x55U);  // a damaged CRC
  feed(bad.data(), gn);
  CHECK(frames == 1U && p.bad_frames() == 1U);
  feed(good.data(), gn);
  CHECK(frames == 2U);
  const std::array<uint8_t, 3> false_start{kSync, 0xFFU, 0xA5U};  // an 0xA5 whose length byte is impossible, and the next 0xA5 starts again
  feed(false_start.data(), false_start.size());
  feed(good.data() + 1, gn - 1U);  // the rest of a good frame after that 0xA5
  CHECK(frames == 3U && p.bad_frames() == 2U);
  // a bad CRC byte that is itself 0xA5 starts the next frame
  std::array<uint8_t, kMaxFrame> tail = good;
  tail[gn - 1U] = kSync;
  feed(tail.data(), gn);  // fails its check, but the last byte is a sync
  feed(good.data() + 1, gn - 1U);
  CHECK(frames == 4U && p.bad_frames() == 3U);
  // an empty payload and a full one both parse
  std::array<uint8_t, kMaxFrame> png{};
  const std::size_t pn = encode(pack_ping(), png.data());
  feed(png.data(), pn);
  CHECK(frames == 5U && m.type == Type::Ping && m.length == 0U);
  Message full;
  full.type = Type::Status;
  full.length = static_cast<uint8_t>(kMaxPayload);
  for (std::size_t i = 0; i < kMaxPayload; ++i) {
    full.payload[i] = static_cast<uint8_t>(i + 1U);
  }
  std::array<uint8_t, kMaxFrame> fb{};
  feed(fb.data(), encode(full, fb.data()));
  CHECK(frames == 6U && m.length == kMaxPayload && m.payload[11] == 12U);
  Message too_long = full;
  too_long.length = 200U;  // encode clamps it
  CHECK(encode(too_long, fb.data()) == kMaxFrame);
}

TFC_TEST(platform_driver_limits_the_travel_and_reports_saturation) {
  PlatformDriver e;
  for (int i = 0; i < 100; ++i) {
    (void)e.command(60.0F, -80.0F);
    e.tick(10U);
  }
  CHECK(near(e.output().x_deg, 45.0F, 1e-4F) && near(e.output().y_deg, -45.0F, 1e-4F) && e.output().saturated);
  for (int i = 0; i < 10; ++i) {
    e.tick(10U);  // the PC stops: the stale command is no longer reported as saturated
  }
  CHECK(!e.output().saturated && e.output().holding);
  PlatformDriver f;
  for (int i = 0; i < 100; ++i) {
    (void)f.command(30.0F, -20.0F);
    f.tick(10U);
  }
  CHECK(near(f.output().x_deg, 30.0F, 1e-4F) && near(f.output().y_deg, -20.0F, 1e-4F) && !f.output().saturated);
}

TFC_TEST(platform_driver_limits_the_rate_a_step_becomes_a_ramp) {
  PlatformDriver d;  // 300 degrees per second: 3 degrees per 10 ms tick
  (void)d.command(30.0F, 0.0F);
  d.tick(10U);
  CHECK(near(d.output().x_deg, 3.0F, 1e-4F));
  (void)d.command(30.0F, 0.0F);
  d.tick(10U);
  CHECK(near(d.output().x_deg, 6.0F, 1e-4F));
  for (int i = 0; i < 20; ++i) {
    (void)d.command(30.0F, 0.0F);
    d.tick(10U);
  }
  CHECK(near(d.output().x_deg, 30.0F, 1e-4F));  // arrives and stays
  (void)d.command(-30.0F, 0.0F);                 // a full reversal is a 200 ms ramp, never a jump
  d.tick(10U);
  CHECK(near(d.output().x_deg, 27.0F, 1e-4F));
}

TFC_TEST(platform_driver_holds_after_100_ms_without_a_command_and_levels_after_a_second) {
  PlatformDriver d;
  for (int i = 0; i < 20; ++i) {
    (void)d.command(20.0F, 10.0F);
    d.tick(10U);
  }
  CHECK(near(d.output().x_deg, 20.0F, 1e-4F) && !d.output().holding && !d.output().levelling);
  CHECK(d.command_age_ms() == 10U);  // the last command, and one tick since
  for (int i = 0; i < 8; ++i) {
    d.tick(10U);  // 90 ms: still tracking
  }
  CHECK(!d.output().holding && d.command_age_ms() == 90U);
  d.tick(10U);  // 100 ms
  CHECK(d.output().holding && near(d.output().x_deg, 20.0F, 1e-4F));
  for (int i = 0; i < 89; ++i) {
    d.tick(10U);  // to 990 ms: holding the whole time
  }
  CHECK(d.output().holding && near(d.output().x_deg, 20.0F, 1e-4F) && near(d.output().y_deg, 10.0F, 1e-4F));
  d.tick(10U);  // 1000 ms
  CHECK(d.output().levelling && !d.output().holding && near(d.output().x_deg, 19.7F, 1e-3F));  // 30 deg/s: 0.3 degree a tick
  for (int i = 0; i < 100; ++i) {
    d.tick(10U);
  }
  CHECK(near(d.output().x_deg, 0.0F, 1e-4F) && near(d.output().y_deg, 0.0F, 1e-4F) && d.output().levelling);
  (void)d.command(5.0F, 5.0F);  // a command after levelling resumes tracking, rate-limited
  d.tick(10U);
  CHECK(!d.output().levelling && near(d.output().x_deg, 3.0F, 1e-4F));
}

TFC_TEST(platform_driver_rejects_commands_that_are_not_numbers_and_does_not_let_them_keep_it_alive) {
  PlatformDriver d;
  CHECK(d.command(10.0F, 0.0F));
  for (int i = 0; i < 20; ++i) {
    d.tick(10U);
  }
  const float nan = std::nanf("");
  CHECK(!d.command(nan, 0.0F) && !d.command(0.0F, nan) && !d.command(1.0e9F, 0.0F) && d.rejected() == 3U);
  CHECK(d.command_age_ms() == 200U && d.output().holding);  // the timeout was not restarted
  PlatformDriver never;
  never.tick(10U);  // never commanded: it is at level and says so
  CHECK(never.output().levelling && !never.output().saturated);
  PlatformDriver big;
  big.tick(60000U);
  big.tick(60000U);
  CHECK(big.command_age_ms() == 0xFFFFU);  // the age saturates
}

TFC_TEST(servo_map_is_linear_signed_trimmed_and_always_inside_its_range) {
  ServoMap m;
  CHECK(near(m.pulse_us(0.0F), 1500.0F, 1e-3F) && near(m.pulse_us(10.0F), 1603.4F, 0.01F) && near(m.pulse_us(-45.0F), 1034.7F, 0.1F));
  CHECK(near(m.pulse_us(1000.0F), 2100.0F, 1e-3F) && near(m.pulse_us(-1000.0F), 900.0F, 1e-3F));  // never outside the safe range
  ServoMap r;
  r.sign = -1.0F;
  r.trim_deg = 2.0F;
  CHECK(near(r.pulse_us(0.0F), 1500.0F - (2.0F * 10.34F), 0.01F) && near(r.pulse_us(10.0F), 1500.0F - (12.0F * 10.34F), 0.01F));
}

TFC_TEST(injector_a_cut_ends_by_itself_and_a_refresh_restarts_it) {
  InjectorLogic inj;
  CHECK(inj.energised() == 0U);  // boots with every node powered
  CHECK(inj.cut(1U, 100U) && inj.cut(3U, 50U));
  CHECK(inj.energised() == 0x0AU && inj.remaining_ms(1U) == 100U);
  inj.tick(50U);
  CHECK(inj.energised() == 0x02U);  // the ACT cut ended
  CHECK(inj.cut(1U, 100U));         // a refresh
  inj.tick(99U);
  CHECK(inj.energised() == 0x02U);
  inj.tick(1U);
  CHECK(inj.energised() == 0U);
  inj.tick(500U);  // nothing to expire
  CHECK(inj.energised() == 0U);
}

TFC_TEST(injector_releases_on_command_on_link_loss_and_never_cuts_for_longer_than_the_maximum) {
  InjectorLogic inj;
  CHECK(inj.cut(0U, 1000U) && inj.cut(2U, 1000U));
  CHECK(inj.cut(0U, 0U) && inj.energised() == 0x04U);  // a zero releases
  inj.release_all();
  CHECK(inj.energised() == 0U);
  CHECK(inj.cut(2U, 65535U) && inj.remaining_ms(2U) == InjectorLogic::kMaxCutMs);  // clamped: a lost PC cannot leave a node cut for long
  CHECK(!inj.cut(4U, 100U) && !inj.cut(255U, 100U) && inj.remaining_ms(9U) == 0U);  // no such channel
  CHECK(inj.energised() == 0x04U);
}
