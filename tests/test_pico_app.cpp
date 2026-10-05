// SPDX-License-Identifier: MIT
// The Pico's real frame loop (core/include/tfc/pico_app.hpp) driven against a fake board: what it reads, what the servos and relays are told after each step, what it writes back,
// whether the watchdog is fed. This is the loop that runs on the board; only the hardware behind `Hal` is fake.
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "tfc/pico_app.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;

// The fake board: bytes in, bytes out, the last outputs, and a history.
struct FakeHal {
  std::vector<uint8_t> rx;
  std::size_t rx_pos = 0U;
  std::vector<uint8_t> tx;
  std::array<float, 2> servo_us{0.0F, 0.0F};
  uint8_t relays = 0xFFU;  // not the safe value until boot() has set it
  std::vector<uint8_t> relay_history;
  std::vector<std::array<float, 2>> servo_history;
  unsigned watchdog_feeds = 0U;
  unsigned servo_sets = 0U;

  bool read_byte(uint8_t& b) {
    if (rx_pos >= rx.size()) {
      return false;
    }
    b = rx[rx_pos++];
    return true;
  }
  void write(const uint8_t* d, std::size_t n) { tx.insert(tx.end(), d, d + n); }
  void set_servo_us(unsigned axis, float us) {
    servo_us[axis] = us;
    ++servo_sets;
  }
  void set_relays(uint8_t m) { relays = m; }
  void feed_watchdog() {
    ++watchdog_feeds;
    relay_history.push_back(relays);
    servo_history.push_back(servo_us);
  }

  void send(const pico::Message& m) {
    std::array<uint8_t, pico::kMaxFrame> b{};
    const std::size_t n = pico::encode(m, b.data());
    rx.insert(rx.end(), b.data(), b.data() + n);
  }
  void send_bytes(const std::vector<uint8_t>& v) { rx.insert(rx.end(), v.begin(), v.end()); }

  // The status frames written so far.
  [[nodiscard]] std::vector<pico::Status> statuses() const {
    std::vector<pico::Status> out;
    pico::Parser p;
    pico::Message m;
    for (const uint8_t b : tx) {
      pico::Status s;
      if (p.feed(b, m) && pico::unpack_status(m, s)) {
        out.push_back(s);
      }
    }
    return out;
  }
};

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

pico::Message platform(uint8_t seq, float x, float y) {
  pico::PlatformCommand c;
  c.seq = seq;
  c.tilt_x_deg = x;
  c.tilt_y_deg = y;
  return pico::pack_platform(c);
}

}  // namespace

TFC_TEST(pico_app_boots_to_the_safe_state_before_anything_is_read) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  CHECK(hal.relays == 0U);  // every node powered
  CHECK(near(hal.servo_us[0], 1500.0F, 1e-3F) && near(hal.servo_us[1], 1500.0F, 1e-3F));  // level
  CHECK(hal.tx.empty() && hal.watchdog_feeds == 0U);
}

TFC_TEST(pico_app_feeds_the_watchdog_once_per_step_and_sets_both_servos_every_step) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  hal.servo_sets = 0U;
  for (int i = 0; i < 25; ++i) {
    app.step();
  }
  CHECK(hal.watchdog_feeds == 25U && hal.servo_sets == 50U);
}

TFC_TEST(pico_app_a_platform_stream_drives_the_servos_through_the_limits_and_the_map) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  for (int i = 0; i < 40; ++i) {
    hal.send(platform(static_cast<uint8_t>(i), 10.0F, -5.0F));
    app.step();
  }
  CHECK(near(hal.servo_us[0], 1500.0F + (10.0F * 10.34F), 0.01F));
  CHECK(near(hal.servo_us[1], 1500.0F - (5.0F * 10.34F), 0.01F));
  const std::vector<pico::Status> st = hal.statuses();
  CHECK(st.size() == 20U);  // every second step
  CHECK(st.back().seq_echo == 39U && near(st.back().out_x_deg, 10.0F, 0.011F) && near(st.back().out_y_deg, -5.0F, 0.011F));
  CHECK(st.back().flags == 0U && st.back().relays == 0U && st.back().command_age_ms == 10U);
  // the first step already moved by at most 3 degrees (300 degrees per second at 10 ms)
  CHECK(near(hal.servo_history[0][0], 1500.0F + (3.0F * 10.34F), 0.01F));
}

TFC_TEST(pico_app_a_tilt_beyond_the_travel_is_clamped_and_reported) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  for (int i = 0; i < 40; ++i) {
    hal.send(platform(1U, 60.0F, -80.0F));
    app.step();
  }
  CHECK(near(hal.servo_us[0], 1500.0F + (45.0F * 10.34F), 0.01F) && near(hal.servo_us[1], 1500.0F - (45.0F * 10.34F), 0.01F));
  CHECK((hal.statuses().back().flags & pico::statusflag::kSaturated) != 0U);
}

TFC_TEST(pico_app_holds_then_levels_when_the_stream_stops) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  for (int i = 0; i < 30; ++i) {
    hal.send(platform(1U, 20.0F, 0.0F));
    app.step();
  }
  const float held = hal.servo_us[0];
  for (int i = 0; i < 50; ++i) {
    app.step();  // 500 ms of silence: holding
  }
  CHECK(near(hal.servo_us[0], held, 0.01F));
  CHECK((hal.statuses().back().flags & pico::statusflag::kHolding) != 0U);
  for (int i = 0; i < 100; ++i) {
    app.step();  // past a second: levelling
  }
  CHECK(hal.servo_us[0] < held && (hal.statuses().back().flags & pico::statusflag::kLevelling) != 0U);
  for (int i = 0; i < 100; ++i) {
    app.step();
  }
  CHECK(near(hal.servo_us[0], 1500.0F, 0.01F));
}

TFC_TEST(pico_app_a_relay_cut_runs_for_its_time_and_ends_by_itself) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  hal.send(pico::pack_relay(pico::RelayCommand{1U, 50U}));
  hal.send(pico::pack_ping());  // keeps the link alive for the test's length
  for (int i = 0; i < 8; ++i) {
    app.step();
    if (i == 2) {
      hal.send(pico::pack_ping());
    }
  }
  // the cut is applied in the step that read it: 40 ms remain after that step's tick, so it shows in steps 1 to 4 and is gone in the fifth
  CHECK(hal.relay_history.size() == 8U);
  for (int i = 0; i < 4; ++i) {
    CHECK(hal.relay_history[static_cast<std::size_t>(i)] == 0x02U);
  }
  for (int i = 4; i < 8; ++i) {
    CHECK(hal.relay_history[static_cast<std::size_t>(i)] == 0U);
  }
}

TFC_TEST(pico_app_a_refresh_keeps_a_cut_and_a_zero_releases_it_at_once) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  for (int i = 0; i < 30; ++i) {
    hal.send(pico::pack_relay(pico::RelayCommand{3U, 100U}));  // refreshed every step: stays cut
    app.step();
  }
  CHECK(hal.relays == 0x08U);
  hal.send(pico::pack_relay(pico::RelayCommand{3U, 0U}));
  app.step();
  CHECK(hal.relays == 0U);
  hal.send(pico::pack_relay(pico::RelayCommand{9U, 100U}));  // no such channel
  app.step();
  CHECK(hal.relays == 0U);
}

TFC_TEST(pico_app_a_quiet_pc_releases_every_relay_after_the_link_timeout_and_the_flag_clears_when_it_returns) {
  FakeHal hal;
  PicoAppConfig cfg;
  cfg.link_timeout_ms = 500U;
  PicoApp<FakeHal> app(hal, cfg);
  app.boot();
  hal.send(pico::pack_relay(pico::RelayCommand{0U, 30000U}));
  hal.send(pico::pack_relay(pico::RelayCommand{2U, 30000U}));
  app.step();
  CHECK(hal.relays == 0x05U && !app.link_lost());
  for (int i = 0; i < 49; ++i) {
    app.step();  // 490 ms of silence so far
  }
  CHECK(hal.relays == 0x05U && !app.link_lost());
  app.step();  // the 500 ms
  CHECK(hal.relays == 0U && app.link_lost());
  app.step();  // (the regular status comes on even steps)
  CHECK((hal.statuses().back().flags & pico::statusflag::kLinkLost) != 0U);
  hal.send(pico::pack_ping());  // the PC returns: the ping is answered at once, with the flag cleared
  app.step();
  CHECK(!app.link_lost() && (hal.statuses().back().flags & pico::statusflag::kLinkLost) == 0U);
  CHECK(hal.relays == 0U);  // and a release is not undone by the PC coming back
}

TFC_TEST(pico_app_a_ping_is_answered_in_the_same_step_even_between_the_regular_statuses) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  app.step();  // step 1: no regular status (every second step)
  CHECK(hal.statuses().empty());
  hal.send(pico::pack_ping());
  app.step();  // step 2: a regular status, and the ping must not double it
  CHECK(hal.statuses().size() == 1U);
  hal.send(pico::pack_ping());
  app.step();  // step 3: only the ping
  CHECK(hal.statuses().size() == 2U);
}

TFC_TEST(pico_app_garbage_and_split_frames_change_nothing_and_a_split_frame_is_assembled_across_steps) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  hal.send_bytes({0x00U, 0x13U, 0xFFU, 0xA5U, 0xFFU, 0xA5U, 0xA5U, 0x42U, 0x99U});
  for (int i = 0; i < 5; ++i) {
    app.step();
  }
  CHECK(near(hal.servo_us[0], 1500.0F, 1e-3F) && hal.relays == 0U);
  CHECK(app.parser().skipped_bytes() > 0U);
  std::array<uint8_t, pico::kMaxFrame> b{};
  const std::size_t n = pico::encode(platform(7U, 12.0F, 0.0F), b.data());
  hal.rx.assign(b.data(), b.data() + 3);  // the first three bytes arrive in one step ...
  hal.rx_pos = 0U;
  app.step();
  CHECK(near(hal.statuses().empty() ? 0.0F : hal.statuses().back().out_x_deg, 0.0F, 0.011F));
  hal.rx.assign(b.data() + 3, b.data() + n);  // ... the rest in the next
  hal.rx_pos = 0U;
  app.step();
  CHECK(app.platform().output().x_deg > 0.0F);  // the command took effect
}

TFC_TEST(pico_app_a_command_that_is_not_a_number_is_rejected_and_reported_and_changes_nothing) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  for (int i = 0; i < 10; ++i) {
    hal.send(platform(static_cast<uint8_t>(i), 10.0F, 10.0F));
    app.step();
  }
  const float before = hal.servo_us[0];
  // a NaN cannot be sent through the 16-bit field (it becomes zero), so poke the driver's own rejection through a frame it cannot decode:
  pico::Message bad = platform(99U, 5.0F, 5.0F);
  bad.length = 4U;  // wrong length: not a platform command at all
  hal.send(bad);
  app.step();
  CHECK(near(hal.servo_us[0], before, 3.0F * 10.34F + 0.1F));  // no jump; at most the tracking ramp
  CHECK(hal.statuses().back().seq_echo != 99U);
}

TFC_TEST(pico_app_reports_why_it_booted_and_the_command_age) {
  FakeHal hal;
  PicoAppConfig cfg;
  cfg.after_watchdog_reset = true;
  PicoApp<FakeHal> app(hal, cfg);
  app.boot();
  hal.send(platform(1U, 1.0F, 1.0F));
  app.step();
  for (int i = 0; i < 5; ++i) {
    app.step();  // six steps in all: the status of the sixth carries an age of 60 ms
  }
  const pico::Status s = hal.statuses().back();
  CHECK((s.flags & pico::statusflag::kWatchdogReset) != 0U && s.command_age_ms == 60U);
}

TFC_TEST(pico_app_with_the_status_rate_set_to_zero_only_answers_pings) {
  FakeHal hal;
  PicoAppConfig cfg;
  cfg.status_every_steps = 0U;
  PicoApp<FakeHal> app(hal, cfg);
  app.boot();
  for (int i = 0; i < 10; ++i) {
    app.step();
  }
  CHECK(hal.statuses().empty());
  hal.send(pico::pack_ping());
  app.step();
  CHECK(hal.statuses().size() == 1U);
}

TFC_TEST(pico_app_status_relays_field_matches_what_the_relays_were_told) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  hal.send(pico::pack_relay(pico::RelayCommand{0U, 1000U}));
  hal.send(pico::pack_relay(pico::RelayCommand{3U, 1000U}));
  hal.send(pico::pack_ping());
  app.step();
  CHECK(hal.relays == 0x09U && hal.statuses().back().relays == 0x09U);
}

TFC_TEST(pico_app_each_servo_uses_its_own_map) {
  FakeHal hal;
  PicoAppConfig cfg;
  cfg.servo_y.sign = -1.0F;
  cfg.servo_y.trim_deg = 2.0F;
  cfg.servo_x.us_per_deg = 10.0F;
  cfg.servo_y.us_per_deg = 12.0F;
  PicoApp<FakeHal> app(hal, cfg);
  app.boot();
  CHECK(near(hal.servo_us[0], 1500.0F, 1e-3F) && near(hal.servo_us[1], 1500.0F - 24.0F, 1e-3F));  // level is the trim on y only
  for (int i = 0; i < 40; ++i) {
    hal.send(platform(1U, 10.0F, 10.0F));
    app.step();
  }
  CHECK(near(hal.servo_us[0], 1500.0F + 100.0F, 0.01F));                 // x: +10 degrees at 10 us per degree
  CHECK(near(hal.servo_us[1], 1500.0F - ((10.0F + 2.0F) * 12.0F), 0.01F));  // y: reversed, trimmed, its own scale
}

TFC_TEST(pico_app_reads_a_bounded_number_of_bytes_per_step_so_a_flood_cannot_stall_the_loop) {
  FakeHal hal;
  PicoApp<FakeHal> app(hal, PicoAppConfig{});
  app.boot();
  hal.send_bytes(std::vector<uint8_t>(1000U, 0x00U));  // a flood of zeros
  hal.send(pico::pack_relay(pico::RelayCommand{2U, 5000U}));  // and then a command behind it
  app.step();
  CHECK(hal.rx_pos == kMaxBytesPerStep && hal.relays == 0U);  // only the bound was read; the command is still waiting
  for (int i = 0; i < 2; ++i) {
    app.step();  // steps 2 and 3: 768 bytes read in all
  }
  CHECK(hal.relays == 0U);
  app.step();  // step 4 reads the end of the flood and the command behind it
  CHECK(hal.relays == 0x04U && hal.watchdog_feeds == 4U);
}
