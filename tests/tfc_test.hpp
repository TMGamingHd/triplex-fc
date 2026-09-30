// SPDX-License-Identifier: MIT
// Minimal dependency-free unit-test harness (host only; never linked into firmware).
#pragma once
#include <cstdio>
#include <functional>
#include <vector>

namespace tfct {
struct Case {
  const char* name;
  void (*fn)();
};
inline std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}
inline int& failures() {
  static int f = 0;
  return f;
}
struct Registrar {
  Registrar(const char* n, void (*f)()) { registry().push_back({n, f}); }
};
}  // namespace tfct

#define TFC_TEST(name)                                   \
  static void name();                                    \
  static tfct::Registrar reg_##name(#name, &name);       \
  static void name()

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::printf("    FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__, #cond);      \
      ++tfct::failures();                                                        \
    }                                                                            \
  } while (0)
