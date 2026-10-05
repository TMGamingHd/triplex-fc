// SPDX-License-Identifier: MIT
// The supervisor's hardware commands, typed over USB serial (docs/design/SUPERVISOR.md section 6): `reset X`, `cycle X`, `hold X`, `release X`, `safe-now`, `safe-clear`, `launch`, `scrub`,
// `t0`, `override-ok`, `time`, `status`. X is A, B, C or ACT (any case). A line is parsed without allocating; a bad line gives `Kind::Unknown` and says why. Integer-only and portable; by TFC-SUP-001 it
// shares no code with `core/`.
#pragma once
#include <cstddef>
#include <cstdint>

namespace sup {

enum class Unit : uint8_t { A = 0, B = 1, C = 2, Act = 3 };
constexpr unsigned kUnits = 4U;  // the three flight computers and the actuator node

enum class Kind : uint8_t {
  Unknown = 0,
  Reset,      // pulse the unit's NRST
  Cycle,      // open the unit's power relay for a moment
  Hold,       // hold the unit in reset until released
  Release,    // let a held unit boot
  SafeNow,    // assert the SAFE line to ACT
  SafeClear,  // release it
  Launch,     // start the supervisor's countdown to T-zero
  Scrub,      // cancel the countdown
  T0,         // T-zero now (a bench shortcut)
  Time,       // report the supervisor's own counter, the RTC and the mission elapsed time (the PC correlates them with UTC, docs/design/MISSION_CLOCK.md)
  OverrideOk, // accept the hardware overrides that are engaged now: a launch may go ahead with them (docs/design/HARDWARE_OVERRIDE.md G5)
  Status
};

enum class Parse : uint8_t { Ok = 0, Empty, UnknownWord, MissingUnit, BadUnit, ExtraWords, TooLong };

struct Command {
  Kind kind = Kind::Unknown;
  Unit unit = Unit::A;
  Parse parse = Parse::Empty;
};

namespace detail {

constexpr char lower(char c) noexcept { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

// Is word [b, e) of `s` equal to `lit` (case-insensitive)?
constexpr bool word_is(const char* s, std::size_t b, std::size_t e, const char* lit) noexcept {
  std::size_t i = 0;
  for (; lit[i] != '\0'; ++i) {
    if (b + i >= e || lower(s[b + i]) != lit[i]) {
      return false;
    }
  }
  return b + i == e;
}

}  // namespace detail

// Parse one line of `len` characters (no terminator needed). Words are separated by spaces or tabs; at most two words; a line longer than 40 characters is refused.
[[nodiscard]] constexpr Command parse_command(const char* line, std::size_t len) noexcept {
  Command c;
  if (len > 40U) {
    c.parse = Parse::TooLong;
    return c;
  }
  std::size_t begin[2] = {0U, 0U};
  std::size_t end[2] = {0U, 0U};
  unsigned words = 0U;
  bool inside = false;
  std::size_t start = 0U;
  for (std::size_t i = 0; i <= len; ++i) {
    const bool sep = i == len || line[i] == ' ' || line[i] == '\t' || line[i] == '\r' || line[i] == '\n';
    if (!sep && !inside) {
      inside = true;
      start = i;
    } else if (sep && inside) {
      inside = false;
      if (words < 2U) {
        begin[words] = start;
        end[words] = i;
      }
      ++words;
    }
  }
  if (words == 0U) {
    return c;  // Parse::Empty
  }
  struct Entry {
    const char* word;
    Kind kind;
    bool unit;
  };
  constexpr Entry table[] = {{"reset", Kind::Reset, true},   {"cycle", Kind::Cycle, true},       {"hold", Kind::Hold, true},   {"release", Kind::Release, true},
                             {"safe-now", Kind::SafeNow, false}, {"safe-clear", Kind::SafeClear, false}, {"launch", Kind::Launch, false}, {"scrub", Kind::Scrub, false},
                             {"t0", Kind::T0, false},         {"override-ok", Kind::OverrideOk, false}, {"time", Kind::Time, false}, {"status", Kind::Status, false}};
  const Entry* found = nullptr;
  for (const Entry& e : table) {
    if (detail::word_is(line, begin[0], end[0], e.word)) {
      found = &e;
    }
  }
  if (found == nullptr) {
    c.parse = Parse::UnknownWord;
    return c;
  }
  c.kind = found->kind;
  if (!found->unit) {
    c.parse = words == 1U ? Parse::Ok : Parse::ExtraWords;
    if (c.parse != Parse::Ok) {
      c.kind = Kind::Unknown;
    }
    return c;
  }
  if (words == 1U) {
    c.kind = Kind::Unknown;
    c.parse = Parse::MissingUnit;
    return c;
  }
  if (words > 2U) {
    c.kind = Kind::Unknown;
    c.parse = Parse::ExtraWords;
    return c;
  }
  if (detail::word_is(line, begin[1], end[1], "a")) {
    c.unit = Unit::A;
  } else if (detail::word_is(line, begin[1], end[1], "b")) {
    c.unit = Unit::B;
  } else if (detail::word_is(line, begin[1], end[1], "c")) {
    c.unit = Unit::C;
  } else if (detail::word_is(line, begin[1], end[1], "act")) {
    c.unit = Unit::Act;
  } else {
    c.kind = Kind::Unknown;
    c.parse = Parse::BadUnit;
    return c;
  }
  c.parse = Parse::Ok;
  return c;
}

}  // namespace sup
