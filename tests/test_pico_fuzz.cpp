// SPDX-License-Identifier: MIT
// Property and fuzz tests of the Pico (seeded, deterministic, run under ASan and UBSan in CI): hostile bytes into the link parser, and a random mix of good, damaged, extreme and
// non-numeric commands into the real frame loop against a fake board. After EVERY step these must hold:
//   P1  every servo pulse is inside 900 to 2100 us                       P5  the relays equal an independent model of the cuts (including the link timeout)
//   P2  every platform tilt is inside the travel                          P6  the watchdog was fed exactly once
//   P3  no tilt moves faster than the rate limit                          P7  everything written to the PC is a status frame that decodes
//   P4  the relay mask uses only the four channels                        P8  a cut never outlasts the longest cut time since the last relay command
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "tfc/pico_app.hpp"
#include "tfc/pico_link.hpp"
#include "tfc_test.hpp"

namespace {

using namespace tfc;

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s((seed * 0x9E3779B97F4A7C15ULL) + 0xBEEF1234ULL) {
    for (int i = 0; i < 4; ++i) {
      next();
    }
  }
  uint64_t next() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545F4914F6CDD1DULL;
  }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>((next() >> 33) % n); }
  float uniform(float lo, float hi) { return lo + ((hi - lo) * (static_cast<float>(next() >> 40) / 16777216.0F)); }
};

struct FakeHal {
  std::vector<uint8_t> rx;
  std::size_t pos = 0U;
  std::vector<uint8_t> tx;
  std::array<float, 2> us{1500.0F, 1500.0F};
  uint8_t relays = 0U;
  unsigned feeds = 0U;
  bool read_byte(uint8_t& b) {
    if (pos >= rx.size()) {
      return false;
    }
    b = rx[pos++];
    return true;
  }
  void write(const uint8_t* d, std::size_t n) { tx.insert(tx.end(), d, d + n); }
  void set_servo_us(unsigned a, float v) { us[a] = v; }
  void set_relays(uint8_t m) { relays = m; }
  void feed_watchdog() { ++feeds; }
  void send(const pico::Message& m) {
    std::array<uint8_t, pico::kMaxFrame> b{};
    const std::size_t n = pico::encode(m, b.data());
    rx.insert(rx.end(), b.data(), b.data() + n);
  }
};

}  // namespace

TFC_TEST(pico_fuzz_the_parser_never_misbehaves_on_hostile_bytes_and_only_returns_frames_that_check) {
  Rng rng(1U);
  pico::Parser p;
  pico::Message m;
  uint64_t frames = 0U;
  for (int i = 0; i < 2000000; ++i) {
    const uint8_t b = static_cast<uint8_t>(rng.below(8U) == 0U ? pico::kSync : rng.below(256U));  // a lot of sync bytes, to stress resynchronisation
    if (p.feed(b, m)) {
      ++frames;
      CHECK(m.length <= pico::kMaxPayload);
      std::array<uint8_t, pico::kMaxFrame> re{};
      const std::size_t n = pico::encode(m, re.data());
      CHECK(n == 4U + m.length && crc8(re.data() + 1, 2U + m.length) == re[n - 1U]);  // what came out is a frame that checks
    }
  }
  CHECK(frames > 0U && p.bad_frames() > 0U && p.skipped_bytes() > 0U);  // random bytes do make some accidental frames (1 in 256 of the candidates)
}

TFC_TEST(pico_fuzz_valid_frames_among_garbage_without_sync_bytes_are_all_found) {
  Rng rng(2U);
  pico::Parser p;
  pico::Message m;
  unsigned sent = 0U;
  unsigned found = 0U;
  for (int i = 0; i < 20000; ++i) {
    for (uint32_t g = rng.below(6U); g > 0U; --g) {  // garbage with no 0xA5 in it
      uint8_t b = static_cast<uint8_t>(rng.below(256U));
      b = b == pico::kSync ? 0x11U : b;
      (void)p.feed(b, m);  // garbage with no sync byte: it can only complete a frame if one is open, and a valid frame is never open here
    }
    pico::Message msg = rng.below(2U) == 0U ? pico::pack_platform(pico::PlatformCommand{static_cast<uint8_t>(i), rng.uniform(-60.0F, 60.0F), rng.uniform(-60.0F, 60.0F)})
                                           : pico::pack_relay(pico::RelayCommand{static_cast<uint8_t>(rng.below(4U)), static_cast<uint16_t>(rng.below(65536U))});
    std::array<uint8_t, pico::kMaxFrame> b{};
    const std::size_t n = pico::encode(msg, b.data());
    ++sent;
    bool got = false;
    for (std::size_t k = 0; k < n; ++k) {
      got = p.feed(b[k], m) || got;
    }
    found += got ? 1U : 0U;
  }
  CHECK(found == sent);
}

TFC_TEST(pico_fuzz_single_bit_errors_in_a_frame_are_never_accepted) {
  pico::PlatformCommand pc;
  pc.seq = 5U;
  pc.tilt_x_deg = 12.34F;
  pc.tilt_y_deg = -5.0F;
  std::array<uint8_t, pico::kMaxFrame> good{};
  const std::size_t n = pico::encode(pico::pack_platform(pc), good.data());
  for (std::size_t byte = 1; byte < n; ++byte) {   // not the sync byte: damage there is a missing frame, not a wrong one
    for (unsigned bit = 0; bit < 8U; ++bit) {
      if (byte == 2U) {
        continue;  // the length byte changes where the frame ends: a different question (the next test)
      }
      pico::Parser p;
      pico::Message m;
      bool got = false;
      for (std::size_t k = 0; k < n; ++k) {
        const uint8_t v = k == byte ? static_cast<uint8_t>(good[k] ^ (1U << bit)) : good[k];
        got = p.feed(v, m) || got;
      }
      CHECK(!got);  // CRC-8 sees every single-bit error
    }
  }
}

TFC_TEST(pico_fuzz_the_frame_loop_keeps_every_invariant_under_a_random_mix_of_commands) {
  Rng rng(3U);
  FakeHal hal;
  PicoAppConfig cfg;
  cfg.link_timeout_ms = 300U;  // short, so the fuzz meets it often
  PicoApp<FakeHal> app(hal, cfg);
  app.boot();
  // The reference: an independent parser (it keeps its state across steps, as the app's does) sees the same bytes and keeps the cuts in milliseconds by its own arithmetic.
  pico::Parser mirror;
  std::array<int32_t, 4> model_left{};
  uint32_t model_quiet = 0U;
  float prev_x = 0.0F;
  float prev_y = 0.0F;
  const float max_step = (cfg.platform.rate_limit_dps * 0.01F) + 1e-4F;
  for (uint32_t step = 0; step < 300000U; ++step) {
    hal.rx.clear();
    hal.pos = 0U;
    hal.tx.clear();
    const unsigned feeds_before = hal.feeds;
    // a random burst of 0 to 3 things this step
    for (uint32_t k = rng.below(4U); k > 0U; --k) {
      switch (rng.below(8U)) {
        case 0:
        case 1: {  // a good platform command, sometimes extreme
          const float r = rng.below(4U) == 0U ? 1.0e6F : 80.0F;
          hal.send(pico::pack_platform(pico::PlatformCommand{static_cast<uint8_t>(step), rng.uniform(-r, r), rng.uniform(-r, r)}));
          break;
        }
        case 2: {  // a relay command, any channel (including nonexistent), any time
          const uint8_t ch = static_cast<uint8_t>(rng.below(7U));
          const uint16_t ms = rng.below(3U) == 0U ? static_cast<uint16_t>(rng.below(65536U)) : static_cast<uint16_t>(rng.below(400U));
          hal.send(pico::pack_relay(pico::RelayCommand{ch, ms}));
          break;
        }
        case 3:
          hal.send(pico::pack_ping());
          break;
        case 4: {  // garbage bytes
          for (uint32_t g = rng.below(12U); g > 0U; --g) {
            hal.rx.push_back(static_cast<uint8_t>(rng.below(256U)));
          }
          break;
        }
        case 5: {  // a good frame with one bit damaged
          std::array<uint8_t, pico::kMaxFrame> b{};
          const std::size_t n = pico::encode(pico::pack_relay(pico::RelayCommand{static_cast<uint8_t>(rng.below(4U)), 5000U}), b.data());
          b[1U + rng.below(static_cast<uint32_t>(n - 1U))] ^= static_cast<uint8_t>(1U << rng.below(8U));
          hal.rx.insert(hal.rx.end(), b.data(), b.data() + n);
          break;
        }
        default:
          break;  // silence
      }
    }
    // ---- the reference sees what the app will see ----
    bool heard = false;
    for (const uint8_t b : hal.rx) {
      pico::Message mm;
      pico::RelayCommand rc;
      if (mirror.feed(b, mm)) {
        heard = true;
        if (pico::unpack_relay(mm, rc) && rc.channel < 4U) {
          model_left[rc.channel] = rc.cut_ms > 30000U ? 30000 : static_cast<int32_t>(rc.cut_ms);
        }
      }
    }
    app.step();
    model_quiet = heard ? 0U : model_quiet + 10U;
    if (model_quiet >= cfg.link_timeout_ms) {
      model_left = {};
    }
    uint8_t model_mask = 0U;
    for (unsigned c = 0; c < 4U; ++c) {
      model_left[c] = model_left[c] > 10 ? model_left[c] - 10 : 0;
      model_mask = static_cast<uint8_t>(model_mask | (model_left[c] > 0 ? (1U << c) : 0U));
    }

    // ---- the properties ----
    CHECK(hal.us[0] >= 900.0F && hal.us[0] <= 2100.0F && hal.us[1] >= 900.0F && hal.us[1] <= 2100.0F);                                       // P1
    const PlatformOutput& o = app.platform().output();
    CHECK(std::isfinite(o.x_deg) && std::isfinite(o.y_deg) && std::fabs(o.x_deg) <= 45.0F + 1e-4F && std::fabs(o.y_deg) <= 45.0F + 1e-4F);  // P2
    CHECK(std::fabs(o.x_deg - prev_x) <= max_step && std::fabs(o.y_deg - prev_y) <= max_step);                                             // P3
    prev_x = o.x_deg;
    prev_y = o.y_deg;
    CHECK((hal.relays & 0xF0U) == 0U);                                                                                                      // P4
    CHECK(hal.relays == model_mask);                                                                                                        // P5, P8
    CHECK(hal.feeds == feeds_before + 1U);                                                                                                  // P6
    pico::Parser tp;
    pico::Message tm;
    std::size_t statuses = 0U;
    for (const uint8_t b : hal.tx) {
      if (tp.feed(b, tm)) {
        pico::Status st;
        CHECK(pico::unpack_status(tm, st) && st.relays == hal.relays);
        ++statuses;
      }
    }
    CHECK(tp.bad_frames() == 0U && tp.skipped_bytes() == 0U && statuses <= 1U);                                                             // P7
  }
}
