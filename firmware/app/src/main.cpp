// SPDX-License-Identifier: MIT
// M0 hello world: proves the portable core/ library builds and runs under Zephyr
// (native_sim on the host, nucleo_g474re on the target).
// Include core/ (and so the C++ standard library) BEFORE Zephyr headers: Zephyr defines an
// `__unused` macro that breaks a glibc header (struct_mutex.h) on the native_sim host build.
#include "tfc/voter.hpp"

#include <zephyr/kernel.h>

int main() {
  // Channel 2 disagrees; a healthy 2-of-3 vote must ignore it.
  const tfc::VoteResult r = tfc::vote3({1.00F, 1.01F, 9.00F}, tfc::kAllChannels, 0.5F);

  // printk has no float support, so print milli-units.
  printk("triplex-fc hello: vote_milli=%d status=%d disagree_mask=0x%02x\n",
         static_cast<int>(r.value * 1000.0F), static_cast<int>(r.status),
         static_cast<unsigned>(r.disagree_mask));
  return 0;
}
