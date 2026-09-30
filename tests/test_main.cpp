// SPDX-License-Identifier: MIT
#include "tfc_test.hpp"

int main() {
  int failed_cases = 0;
  for (const auto& c : tfct::registry()) {
    const int before = tfct::failures();
    c.fn();
    const bool ok = tfct::failures() == before;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    if (!ok) ++failed_cases;
  }
  std::printf("\n%zu tests, %d failed\n", tfct::registry().size(), failed_cases);
  return failed_cases == 0 ? 0 : 1;
}
