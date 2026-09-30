// SPDX-License-Identifier: MIT
#include <limits>

#include "tfc/voter.hpp"
#include "tfc_test.hpp"

using namespace tfc;

TFC_TEST(median3_all_orderings) {
  const float p[6][3] = {{1, 2, 3}, {1, 3, 2}, {2, 1, 3}, {2, 3, 1}, {3, 1, 2}, {3, 2, 1}};
  for (const auto& t : p) CHECK(median3(t[0], t[1], t[2]) == 2.0F);
  CHECK(median3(5, 5, 1) == 5.0F);
  CHECK(median3(1, 1, 1) == 1.0F);
}

TFC_TEST(triplex_all_agree) {
  const VoteResult r = vote3({1.00F, 1.01F, 0.99F}, kAllChannels, 0.1F);
  CHECK(r.status == VoteStatus::Triplex);
  CHECK(r.disagree_mask == 0);
  CHECK(r.value == 1.0F);
}

TFC_TEST(triplex_outvotes_one_wild_channel) {
  for (unsigned bad = 0; bad < 3; ++bad) {
    std::array<float, 3> x{1.0F, 1.0F, 1.0F};
    x[bad] = 500.0F;
    const VoteResult r = vote3(x, kAllChannels, 0.1F);
    CHECK(r.status == VoteStatus::Triplex);
    CHECK(r.value == 1.0F);
    CHECK(r.disagree_mask == (1U << bad));
  }
}

TFC_TEST(three_way_split_is_no_majority) {
  const VoteResult r = vote3({0.0F, 10.0F, 20.0F}, kAllChannels, 0.1F);
  CHECK(r.status == VoteStatus::NoMajority);
}

TFC_TEST(duplex_agree_and_miscompare) {
  const uint8_t m = 0b101;  // channels 0 and 2 valid
  const VoteResult ok = vote3({1.0F, 99.0F, 1.02F}, m, 0.1F);
  CHECK(ok.status == VoteStatus::Duplex);
  CHECK(std::fabs(ok.value - 1.01F) < 1e-5F);
  const VoteResult bad = vote3({1.0F, 99.0F, 2.0F}, m, 0.1F);
  CHECK(bad.status == VoteStatus::DuplexMiscompare);
  CHECK(bad.disagree_mask == m);  // cannot tell which one is wrong
}

TFC_TEST(simplex_and_nodata) {
  const VoteResult s = vote3({0.0F, 4.0F, 0.0F}, 0b010, 0.1F);
  CHECK(s.status == VoteStatus::Simplex);
  CHECK(s.value == 4.0F);
  CHECK(vote3({1, 2, 3}, 0, 0.1F).status == VoteStatus::NoData);
}

TFC_TEST(nan_and_inf_are_treated_as_invalid) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const VoteResult r = vote3({1.0F, nan, 1.0F}, kAllChannels, 0.1F);
  CHECK(r.status == VoteStatus::Duplex);
  CHECK((r.disagree_mask & 0b010) != 0);
  CHECK(r.value == 1.0F);
  const VoteResult r2 = vote3({inf, nan, 3.0F}, kAllChannels, 0.1F);
  CHECK(r2.status == VoteStatus::Simplex);
  CHECK(r2.value == 3.0F);
}

TFC_TEST(vote_is_deterministic_and_order_insensitive) {
  const float a = 0.3F, b = 0.31F, c = 7.0F;
  const float v1 = vote3({a, b, c}, kAllChannels, 0.1F).value;
  const float v2 = vote3({c, a, b}, kAllChannels, 0.1F).value;
  const float v3 = vote3({b, c, a}, kAllChannels, 0.1F).value;
  CHECK(v1 == v2);
  CHECK(v2 == v3);
}

TFC_TEST(count_channels_matches_popcount) {
  for (unsigned m = 0; m < 8; ++m) {
    unsigned n = 0;
    for (unsigned i = 0; i < 3; ++i) n += (m >> i) & 1U;
    CHECK(count_channels(static_cast<uint8_t>(m)) == n);
  }
}
