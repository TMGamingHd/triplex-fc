// SPDX-License-Identifier: MIT
// The USB serial link between the PC and the Pico that drives the platform and injects power faults (docs/design/PICO.md). A frame is
//   0xA5 | type | length | payload (0 to 12 bytes) | CRC-8 over type, length and payload
// with the same CRC as the flight bus (crc8.hpp). A byte-at-a-time parser finds frames in a stream, drops what does not check and
// resynchronises on the next 0xA5, so garbage or a half frame never costs more than the frame it damages.
//   PC to Pico  0x01 platform   seq u8, tilt about X i16, tilt about Y i16 (0.01 degree)         the platform's target, 100 Hz
//               0x02 relay      channel u8, cut time u16 ms (0 releases)                          a power cut that ends by itself
//               0x03 ping       nothing                                                           keeps the link alive and asks for a status
//   Pico to PC  0x81 status     seq echo u8, out X i16, out Y i16 (0.01 degree), flags u8, relays u8, command age u16 ms
// Multi-byte fields are little endian. No heap, no exceptions, no RTTI. Deterministic.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "tfc/crc8.hpp"

namespace tfc::pico {

constexpr uint8_t kSync = 0xA5U;
constexpr std::size_t kMaxPayload = 12U;
constexpr std::size_t kMaxFrame = 3U + kMaxPayload + 1U;
constexpr float kAngleLsbDeg = 0.01F;

enum class Type : uint8_t { Platform = 0x01, Relay = 0x02, Ping = 0x03, Status = 0x81 };

namespace statusflag {
constexpr uint8_t kSaturated = 0x01U;   // the commanded tilt is beyond the platform's travel
constexpr uint8_t kHolding = 0x02U;     // no command for the hold time: the platform holds
constexpr uint8_t kLevelling = 0x04U;   // no command for the level time: the platform is going to level
constexpr uint8_t kLinkLost = 0x08U;    // nothing at all from the PC for the link timeout: the relays have been released
constexpr uint8_t kWatchdogReset = 0x10U;  // the last reset of this Pico was its watchdog
constexpr uint8_t kRejected = 0x20U;    // a command with a value that is not a number or not in range has been rejected since boot
}  // namespace statusflag

struct Message {
  Type type = Type::Ping;
  uint8_t length = 0U;
  std::array<uint8_t, kMaxPayload> payload{};
};

struct PlatformCommand {
  uint8_t seq = 0U;
  float tilt_x_deg = 0.0F;
  float tilt_y_deg = 0.0F;
};

struct RelayCommand {
  uint8_t channel = 0U;
  uint16_t cut_ms = 0U;
};

struct Status {
  uint8_t seq_echo = 0U;
  float out_x_deg = 0.0F;
  float out_y_deg = 0.0F;
  uint8_t flags = 0U;
  uint8_t relays = 0U;  // bit n set: channel n is energised (the node is cut)
  uint16_t command_age_ms = 0U;
};

namespace detail {
inline int16_t quantize_angle(float deg) noexcept {
  const float q = deg / kAngleLsbDeg;
  if (q >= 32767.0F) {
    return 32767;
  }
  if (q <= -32768.0F) {
    return -32768;
  }
  if (!(q > -32768.0F && q < 32767.0F)) {  // NaN: no angle
    return 0;
  }
  return static_cast<int16_t>(q >= 0.0F ? q + 0.5F : q - 0.5F);
}
constexpr uint16_t le16(const uint8_t* p) noexcept { return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8U)); }
inline void put16(uint8_t* p, uint16_t v) noexcept {
  p[0] = static_cast<uint8_t>(v & 0xFFU);
  p[1] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
}
}  // namespace detail

// Bytes of a frame for `m`, into `out` (at least kMaxFrame long); returns the count.
inline std::size_t encode(const Message& m, uint8_t* out) noexcept {
  const std::size_t n = m.length > kMaxPayload ? kMaxPayload : m.length;
  out[0] = kSync;
  out[1] = static_cast<uint8_t>(m.type);
  out[2] = static_cast<uint8_t>(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[3U + i] = m.payload[i];
  }
  out[3U + n] = crc8(out + 1, 2U + n);
  return 4U + n;
}

inline Message pack_platform(const PlatformCommand& c) noexcept {
  Message m;
  m.type = Type::Platform;
  m.length = 5U;
  m.payload[0] = c.seq;
  detail::put16(&m.payload[1], static_cast<uint16_t>(detail::quantize_angle(c.tilt_x_deg)));
  detail::put16(&m.payload[3], static_cast<uint16_t>(detail::quantize_angle(c.tilt_y_deg)));
  return m;
}

inline Message pack_relay(const RelayCommand& c) noexcept {
  Message m;
  m.type = Type::Relay;
  m.length = 3U;
  m.payload[0] = c.channel;
  detail::put16(&m.payload[1], c.cut_ms);
  return m;
}

inline Message pack_ping() noexcept { return Message{}; }

inline Message pack_status(const Status& s) noexcept {
  Message m;
  m.type = Type::Status;
  m.length = 9U;
  m.payload[0] = s.seq_echo;
  detail::put16(&m.payload[1], static_cast<uint16_t>(detail::quantize_angle(s.out_x_deg)));
  detail::put16(&m.payload[3], static_cast<uint16_t>(detail::quantize_angle(s.out_y_deg)));
  m.payload[5] = s.flags;
  m.payload[6] = s.relays;
  detail::put16(&m.payload[7], s.command_age_ms);
  return m;
}

// The decoders return false if the message is not of that type or has the wrong length.
inline bool unpack_platform(const Message& m, PlatformCommand& c) noexcept {
  if (m.type != Type::Platform || m.length != 5U) {
    return false;
  }
  c.seq = m.payload[0];
  c.tilt_x_deg = static_cast<float>(static_cast<int16_t>(detail::le16(&m.payload[1]))) * kAngleLsbDeg;
  c.tilt_y_deg = static_cast<float>(static_cast<int16_t>(detail::le16(&m.payload[3]))) * kAngleLsbDeg;
  return true;
}

inline bool unpack_relay(const Message& m, RelayCommand& c) noexcept {
  if (m.type != Type::Relay || m.length != 3U) {
    return false;
  }
  c.channel = m.payload[0];
  c.cut_ms = detail::le16(&m.payload[1]);
  return true;
}

inline bool unpack_status(const Message& m, Status& s) noexcept {
  if (m.type != Type::Status || m.length != 9U) {
    return false;
  }
  s.seq_echo = m.payload[0];
  s.out_x_deg = static_cast<float>(static_cast<int16_t>(detail::le16(&m.payload[1]))) * kAngleLsbDeg;
  s.out_y_deg = static_cast<float>(static_cast<int16_t>(detail::le16(&m.payload[3]))) * kAngleLsbDeg;
  s.flags = m.payload[5];
  s.relays = m.payload[6];
  s.command_age_ms = detail::le16(&m.payload[7]);
  return true;
}

// Finds frames in a byte stream.
class Parser {
 public:
  // Feed one byte; true when it completed a good frame, which is then in `out`.
  bool feed(uint8_t b, Message& out) noexcept {
    if (state_ == State::Sync) {
      if (b == kSync) {
        state_ = State::Type;
      } else {
        ++skipped_;
      }
      return false;
    }
    if (state_ == State::Type) {
      buf_[0] = b;
      state_ = State::Length;
      return false;
    }
    if (state_ == State::Length) {
      if (b > kMaxPayload) {  // not a frame: this 0xA5 was data; look again from this byte
        ++bad_;
        state_ = b == kSync ? State::Type : State::Sync;
        return false;
      }
      buf_[1] = b;
      have_ = 0U;
      state_ = b == 0U ? State::Crc : State::Payload;
      return false;
    }
    if (state_ == State::Payload) {
      buf_[2U + have_] = b;
      ++have_;
      if (have_ == buf_[1]) {
        state_ = State::Crc;
      }
      return false;
    }
    // the check byte
    state_ = State::Sync;
    if (b != crc8(buf_.data(), 2U + static_cast<std::size_t>(buf_[1]))) {
      ++bad_;
      if (b == kSync) {  // the byte that failed may itself start a frame
        state_ = State::Type;
      }
      return false;
    }
    out.type = static_cast<Type>(buf_[0]);
    out.length = buf_[1];
    for (std::size_t i = 0; i < kMaxPayload; ++i) {
      out.payload[i] = i < buf_[1] ? buf_[2U + i] : 0U;
    }
    return true;
  }

  [[nodiscard]] uint32_t bad_frames() const noexcept { return bad_; }
  [[nodiscard]] uint32_t skipped_bytes() const noexcept { return skipped_; }

 private:
  enum class State : uint8_t { Sync, Type, Length, Payload, Crc };
  State state_ = State::Sync;
  std::array<uint8_t, 2U + kMaxPayload> buf_{};
  std::size_t have_ = 0U;
  uint32_t bad_ = 0U;
  uint32_t skipped_ = 0U;
};

}  // namespace tfc::pico
