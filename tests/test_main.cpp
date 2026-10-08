// SPDX-License-Identifier: MIT
#include <cstdlib>
#include <cstring>

#include "tfc_test.hpp"

// With an argument, only the tests whose name contains it are run (tfc_tests slosh); with none, all of them.
int main(int argc, char** argv) {
  int failed_cases = 0;
  const bool stop_at_first = std::getenv("TFC_STOP_AT_FIRST_FAIL") != nullptr;
  for (const auto& c : tfct::registry()) {
    if (argc > 1 && std::strstr(c.name, argv[1]) == nullptr) {
      continue;
    }
    const int before = tfct::failures();
    c.fn();
    const bool ok = tfct::failures() == before;
    std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    if (!ok) {
      ++failed_cases;
      if (stop_at_first) {
        break;  // a mutation run needs one failing test, not all of them
      }
    }
  }
  std::printf("\n%zu tests, %d failed\n", tfct::registry().size(), failed_cases);
  return failed_cases == 0 ? 0 : 1;
}
