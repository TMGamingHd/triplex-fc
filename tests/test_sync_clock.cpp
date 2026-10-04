// SPDX-License-Identifier: MIT
// SYNC following and sync-master takeover (core/include/tfc/sync_clock.hpp): the frame number stays continuous through loss and takeover, a lone node
// that merely lost a frame does not take over, B and C never claim an empty bus, a returning A follows, and two masters resolve to one.
#include <array>
#include <cstdint>

#include "tfc/sync_clock.hpp"
#include "tfc_test.hpp"

namespace {

using tfc::SyncClock;
using tfc::SyncStart;

// Three clocks on a perfect bus. A node that is `down` does not run. Each frame, the nodes decide in order of their wait windows (A first): the first
// master sends SYNC and every node still waiting hears it. `drop[n]` loses the SYNC for node n in that frame.
struct Net {
  std::array<SyncClock, 3> clk{SyncClock(SyncStart::Master), SyncClock(SyncStart::FollowOnly), SyncClock(SyncStart::FollowOnly)};
  std::array<bool, 3> down{false, false, false};
  std::array<bool, 3> drop{false, false, false};
  std::array<tfc::SyncTick, 3> last{};
  uint32_t sync_number = 0U;
  bool sync_sent = false;
  unsigned masters_this_frame = 0U;

  void frame() {
    masters_this_frame = 0U;
    sync_sent = false;
    std::array<bool, 3> was_master{};
    for (unsigned n = 0; n < 3U; ++n) {
      was_master[n] = clk[n].master();
    }
    // the nodes that are masters at the start of the frame send at its nominal start; the followers then wait out their windows, lowest node first, and
    // a follower that takes over sends when its window ends (the nodes still waiting hear it)
    for (const bool masters_pass : {true, false}) {
      for (unsigned n = 0; n < 3U; ++n) {
        if (down[n] || was_master[n] != masters_pass) {
          continue;
        }
        const bool heard = sync_sent && !drop[n];
        last[n] = clk[n].cycle(heard, sync_number);
        if (last[n].master) {
          ++masters_this_frame;
          sync_sent = true;
          sync_number = last[n].frame;
        }
      }
    }
  }
};

}  // namespace

TFC_TEST(sync_a_master_counts_from_zero_and_followers_take_its_number) {
  Net net;
  for (uint32_t k = 0; k < 20U; ++k) {
    net.frame();
    CHECK(net.last[0].master && net.last[0].frame == k && net.masters_this_frame == 1U);
    CHECK(!net.last[1].master && net.last[1].frame == k && net.last[1].locked && net.last[1].missed == 0U);
    CHECK(net.last[2].frame == k);
  }
}

TFC_TEST(sync_followers_keep_counting_through_a_gap_and_do_not_take_over_after_one_lost_frame) {
  Net net;
  for (int i = 0; i < 10; ++i) {
    net.frame();
  }
  net.drop[1] = true;  // B misses one SYNC
  net.frame();
  CHECK(net.last[1].frame == 10U && !net.last[1].master && net.last[1].missed == 1U && !net.last[1].locked);
  net.drop[1] = false;
  net.frame();
  CHECK(net.last[1].frame == 11U && net.last[1].missed == 0U && net.last[1].locked && net.masters_this_frame == 1U);
}

TFC_TEST(sync_when_the_master_stops_the_lowest_follower_takes_over_after_two_frames_with_the_number_continuous) {
  Net net;
  for (int i = 0; i < 10; ++i) {
    net.frame();
  }
  net.down[0] = true;  // A dies after frame 9
  net.frame();         // frame 10: nobody sends SYNC
  CHECK(!net.last[1].master && net.last[1].frame == 10U && net.last[1].missed == 1U && net.last[2].missed == 1U);
  net.frame();  // frame 11: B's second miss: it takes over and sends; C, whose window is longer, hears it
  CHECK(net.last[1].master && net.last[1].took_over && net.last[1].frame == 11U);
  CHECK(!net.last[2].master && net.last[2].locked && net.last[2].frame == 11U && net.masters_this_frame == 1U);
  for (uint32_t k = 12; k < 40U; ++k) {  // and it carries on, one master, no skipped or repeated number
    net.frame();
    CHECK(net.last[1].master && net.last[1].frame == k && net.last[2].frame == k && net.masters_this_frame == 1U);
  }
}

TFC_TEST(sync_the_third_node_takes_over_when_both_others_are_gone) {
  Net net;
  for (int i = 0; i < 5; ++i) {
    net.frame();
  }
  net.down[0] = true;
  net.down[1] = true;
  net.frame();
  CHECK(!net.last[2].master && net.last[2].missed == 1U);
  net.frame();
  CHECK(net.last[2].master && net.last[2].took_over && net.last[2].frame == 6U);
  net.frame();
  CHECK(net.last[2].master && net.last[2].frame == 7U);
}

TFC_TEST(sync_a_follower_that_never_heard_sync_never_claims_the_bus) {
  SyncClock b(SyncStart::FollowOnly);
  for (uint32_t k = 0; k < 1000U; ++k) {
    const tfc::SyncTick t = b.cycle(false, 0U);
    CHECK(!t.master && t.frame == k && !t.took_over);
  }
  CHECK(b.cycle(false, 0U).missed == 255U);  // the miss count saturates
  CHECK(!b.cycle(false, 0U).synced);         // and it is not synced: no frame number to stamp frames with
  const tfc::SyncTick t = b.cycle(true, 77U);  // the first SYNC it hears: it follows
  CHECK(t.frame == 77U && t.locked && t.missed == 0U && t.synced);
  CHECK(b.cycle(false, 0U).synced);  // and stays synced through a gap (it keeps counting)
}

TFC_TEST(sync_a_restarted_a_follows_the_master_that_took_over) {
  Net net;
  for (int i = 0; i < 10; ++i) {
    net.frame();
  }
  net.down[0] = true;
  for (int i = 0; i < 20; ++i) {
    net.frame();  // B is the master from frame 11 on
  }
  CHECK(net.last[1].master);
  net.down[0] = false;
  net.clk[0] = SyncClock(SyncStart::Listen);  // A restarts after a reset
  for (int i = 0; i < 30; ++i) {
    net.frame();
    CHECK(net.masters_this_frame == 1U && net.last[1].master);  // never two masters, B stays the master
    CHECK(net.last[0].frame == net.last[1].frame);              // and A is in step with it from the first frame it hears
  }
  CHECK(!net.last[0].master && net.last[0].locked);
}

TFC_TEST(sync_a_node_listening_after_a_reset_claims_an_empty_bus_after_the_listen_time) {
  SyncClock a(SyncStart::Listen);
  for (uint32_t k = 0; k < 4U; ++k) {
    const tfc::SyncTick t = a.cycle(false, 0U);
    CHECK(!t.master && t.frame == k && !t.synced);  // listening: silent
  }
  const tfc::SyncTick t = a.cycle(false, 0U);  // the fifth silent frame
  CHECK(t.master && t.took_over && t.frame == 4U && t.synced);
  CHECK(a.master() && a.next_frame() == 5U);
}

TFC_TEST(sync_a_master_that_hears_another_sync_yields_and_takes_its_number) {
  SyncClock a(SyncStart::Master);
  CHECK(a.cycle(false, 0U).frame == 0U);
  const tfc::SyncTick t = a.cycle(true, 500U);  // someone else is sending
  CHECK(t.yielded && !t.master && t.frame == 500U && t.locked);
  CHECK(!a.master());
  CHECK(a.cycle(true, 501U).frame == 501U && !a.cycle(true, 502U).yielded);
}

TFC_TEST(sync_windows_grow_with_the_node_number_so_the_lowest_takeover_is_heard_first) {
  CHECK(tfc::sync_window_us(0U) < tfc::sync_window_us(1U) && tfc::sync_window_us(1U) < tfc::sync_window_us(2U));
  CHECK(tfc::sync_window_us(1U) - tfc::sync_window_us(0U) >= 500);  // enough for a SYNC frame (about 0.1 ms) to cross the bus and be heard
  CHECK(tfc::sync_window_us(2U, 1500) == 4000 && tfc::sync_window_us(0U, 1500) == 1000);  // a longer stagger for the host
}
