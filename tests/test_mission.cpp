// SPDX-License-Identifier: MIT
// placeholder, filled in below
#include "tfc_test.hpp"
#include "tfc/mission.hpp"
TFC_TEST(mission_compiles) { tfc::gnc::Mission m; CHECK(!m.configured()); }
