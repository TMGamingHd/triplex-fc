// SPDX-License-Identifier: MIT
// SYNC following and sync-master takeover (core/include/tfc/sync_clock.hpp): the frame number stays continuous through loss and takeover, a lone node
// that merely lost a frame does not take over, B and C never claim an empty bus, a returning A follows, and two masters resolve to one.
#include <array>
#include <cstdio>
#include <string>
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
  uint16_t sync_mission = 0U;
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
        last[n] = clk[n].cycle(heard, sync_number, sync_mission);
        if (last[n].master) {
          ++masters_this_frame;
          sync_sent = true;
          sync_number = last[n].frame;
          sync_mission = last[n].mission;
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
  CHECK(tfc::sync_window_us(2U, 2500) == 6000 && tfc::sync_window_us(0U, 2500) == 1000);  // a longer stagger for the host
}

TFC_TEST(sync_an_observer_follows_and_counts_but_never_becomes_the_master) {
  SyncClock act(SyncStart::Observer);
  CHECK(act.cycle(true, 10U).frame == 10U && act.cycle(true, 11U).synced);
  for (uint32_t k = 12; k < 400U; ++k) {  // SYNC is gone for good
    const tfc::SyncTick t = act.cycle(false, 0U);
    CHECK(!t.master && !t.took_over && t.frame == k && t.synced);
  }
  SyncClock fresh(SyncStart::Observer);
  for (int i = 0; i < 50; ++i) {
    CHECK(!fresh.cycle(false, 0U).master);
  }
}

TFC_TEST(mission_the_sync_frame_carries_the_mission_frame_and_the_helpers_agree_on_the_boundaries) {
  const tfc::Frame f = tfc::pack_sync(123456U, 9U, 0x1234U);
  const tfc::DecodedSync d = tfc::unpack_sync(f);
  CHECK(d.ok && d.frame_no == 123456U && d.mission == 0x1234U && d.seq == 9U);
  CHECK(tfc::unpack_sync(tfc::pack_sync(7U, 1U)).mission == tfc::mission::kNotLaunched);  // a sender that knows nothing of the launch sequence says "not launched"
  namespace m = tfc::mission;
  CHECK(!m::counting(0U) && m::counting(1U) && m::in_countdown(1U) && m::in_countdown(1000U) && !m::in_countdown(1001U) && !m::in_countdown(0U));
  CHECK(!m::in_flight(1000U) && m::in_flight(1001U) && m::flight_frames(1001U) == 0U && m::flight_frames(1002U) == 1U && m::flight_frames(500U) == 0U);
  CHECK(m::frames_to_zero(1U) == 1000U && m::frames_to_zero(1000U) == 1U && m::frames_to_zero(1001U) == 0U && m::frames_to_zero(0U) == 0U);
}

TFC_TEST(mission_only_the_master_launches_once_and_followers_take_the_countdown_from_sync) {
  Net net;
  for (int i = 0; i < 5; ++i) {
    net.frame();
  }
  CHECK(!net.clk[1].launch() && !net.clk[2].launch());  // followers cannot
  CHECK(net.last[0].mission == 0U && net.last[1].mission == 0U);
  CHECK(net.clk[0].launch() && !net.clk[0].launch());   // the master can, once
  net.frame();
  CHECK(net.last[0].mission == 1U && net.last[1].mission == 1U && net.last[2].mission == 1U);
  for (int i = 0; i < 1000; ++i) {
    net.frame();
  }
  CHECK(net.last[0].mission == 1001U && net.last[1].mission == 1001U && net.last[2].mission == 1001U);  // T-zero, on every computer in the same frame
  CHECK(tfc::mission::flight_frames(net.last[2].mission) == 0U);
  net.frame();
  CHECK(tfc::mission::flight_frames(net.last[1].mission) == 1U && !net.last[1].mission_disagrees);
}

TFC_TEST(mission_a_scrub_returns_everyone_to_the_pad_and_only_before_t_zero) {
  Net net;
  net.frame();
  CHECK(!net.clk[0].scrub());  // nothing to scrub
  (void)net.clk[0].launch();
  for (int i = 0; i < 300; ++i) {
    net.frame();
  }
  CHECK(!net.clk[1].scrub());  // not the master
  CHECK(net.clk[0].scrub());
  net.frame();
  CHECK(net.last[0].mission == 0U && net.last[1].mission == 0U && net.last[2].mission == 0U);
  (void)net.clk[0].launch();  // and a new countdown can start
  for (int i = 0; i < 1100; ++i) {
    net.frame();
  }
  CHECK(tfc::mission::in_flight(net.last[1].mission) && !net.clk[0].scrub());  // too late: after T-zero there is no scrub
}

TFC_TEST(mission_a_flying_follower_counts_for_itself_and_reports_a_sync_that_disagrees) {
  tfc::SyncClock f(SyncStart::FollowOnly);
  (void)f.cycle(true, 10U, 1500U);  // hears a flight in progress: adopts it (it was not in flight)
  tfc::SyncTick t = f.cycle(true, 11U, 1501U);
  CHECK(t.mission == 1501U && !t.mission_disagrees);
  t = f.cycle(true, 12U, 0U);  // a damaged or false SYNC says "not launched": ignored, reported
  CHECK(t.mission == 1502U && t.mission_disagrees);
  t = f.cycle(true, 13U, 9000U);  // or jumps ahead: ignored, reported
  CHECK(t.mission == 1503U && t.mission_disagrees);
  t = f.cycle(false, 0U);  // and a missed SYNC is counted through
  CHECK(t.mission == 1504U && !t.mission_disagrees);
  t = f.cycle(true, 15U, 1505U);
  CHECK(t.mission == 1505U && !t.mission_disagrees);  // back in agreement
}

TFC_TEST(mission_a_late_joiner_adopts_the_flight_and_a_pad_node_follows_a_scrub_or_a_launch) {
  tfc::SyncClock late(SyncStart::FollowOnly);
  CHECK(late.cycle(true, 500U, 3000U).mission == 3000U);  // joins long after T-zero
  tfc::SyncClock pad(SyncStart::FollowOnly);
  CHECK(pad.cycle(true, 1U, 0U).mission == 0U);
  CHECK(pad.cycle(true, 2U, 1U).mission == 1U);     // a launch it did not see the command for
  CHECK(pad.cycle(true, 3U, 2U).mission == 2U);
  CHECK(pad.cycle(true, 4U, 0U).mission == 0U);     // a scrub: still in the countdown, so it follows SYNC back to zero
}

TFC_TEST(mission_survives_a_sync_master_takeover_without_a_jump) {
  Net net;
  for (int i = 0; i < 3; ++i) {
    net.frame();
  }
  (void)net.clk[0].launch();
  for (int i = 0; i < 1500; ++i) {
    net.frame();  // well into flight
  }
  const uint16_t before = net.last[1].mission;
  net.down[0] = true;  // A dies
  net.frame();
  net.frame();  // B takes over on its second missed frame
  CHECK(net.last[1].master && net.last[1].took_over);
  CHECK(net.last[1].mission == before + 2U && net.last[2].mission == before + 2U);  // the count went on through the gap: no jump, no repeat
  net.frame();
  CHECK(net.last[1].mission == before + 3U && net.last[2].mission == before + 3U && !net.last[2].mission_disagrees);
  net.down[0] = false;  // A returns, listening first, and adopts the flight
  net.clk[0] = SyncClock(SyncStart::Listen);
  for (int i = 0; i < 5; ++i) {
    net.frame();
  }
  CHECK(net.last[0].mission == net.last[1].mission && !net.last[0].master);
}

TFC_TEST(mission_the_count_stops_at_its_largest_value) {
  tfc::SyncClock m(SyncStart::Master);
  (void)m.cycle(false, 0U);
  CHECK(m.launch());
  tfc::SyncTick t;
  for (uint32_t i = 0; i < 70000U; ++i) {
    t = m.cycle(false, 0U);
  }
  CHECK(t.mission == tfc::mission::kMax && m.mission_frame() == tfc::mission::kMax);
}

TFC_TEST(mission_golden_sync_frames_are_pinned_with_the_python_mirror) {
  auto hex = [](const tfc::Frame& f) {
    std::string s;
    for (unsigned i = 0; i < 8U; ++i) {
      std::array<char, 3> c{};
      (void)std::snprintf(c.data(), c.size(), "%02x", f.data[i]);
      s += c.data();
    }
    return s;
  };
  CHECK(hex(tfc::pack_sync(0x01020304U, 9U, 1001U)) == "04030201e90309d2");  // T-zero
  CHECK(hex(tfc::pack_sync(0U, 0U, 0xFFFFU)) == "00000000ffff0045");
  CHECK(hex(tfc::pack_sync(0x01020304U, 9U)) == "0403020100000915");  // no mission frame: the bytes of the old frame
}

namespace {

using T0 = tfc::SyncClock::T0;
using namespace tfc;

// A lone master that counts its own frames (nobody else on the bus) and is fed the T0 line one level per frame.
struct Pad {
  SyncClock c{SyncStart::Master};
  uint32_t n = 0U;
  uint16_t mission() const { return c.mission_frame(); }
  // One frame: the line is sampled at the end of the frame, after the cycle.
  T0 frame(bool line) {
    (void)c.cycle(false, n++);
    return c.t0_line(line);
  }
  void low(unsigned frames) {
    for (unsigned i = 0; i < frames; ++i) {
      (void)frame(false);
    }
  }
};

}  // namespace

TFC_TEST(t0_line_an_edge_in_the_last_second_of_the_countdown_is_t_zero_and_the_next_frame_is_the_t_zero_frame) {
  Pad p;
  p.low(5U);
  CHECK(p.c.launch());
  p.low(850U);  // 5 ... 855: more than a second to go
  CHECK(mission::frames_to_zero(p.mission()) > SyncClock::kT0Window);
  while (mission::frames_to_zero(p.mission()) > SyncClock::kT0Window) {
    (void)p.frame(false);
  }
  CHECK(p.frame(true) == T0::None);  // the first high sample is not yet an edge
  CHECK(p.frame(true) == T0::Latched);
  CHECK(p.mission() == mission::kCountdownFrames + 1U);
  CHECK(mission::in_flight(p.mission()));
  CHECK(p.frame(true) == T0::None && p.frame(false) == T0::None);  // the line may stay high or fall: nothing more happens
}

TFC_TEST(t0_line_without_an_edge_the_countdown_runs_out_on_its_own_and_t_zero_comes_when_it_always_did) {
  Pad p;
  p.low(3U);
  CHECK(p.c.launch());
  unsigned frames = 0U;
  while (!mission::in_flight(p.mission())) {
    (void)p.frame(false);
    ++frames;
    CHECK(frames < 1100U);
  }
  CHECK(frames == mission::kCountdownFrames);
}

TFC_TEST(t0_line_an_early_edge_is_refused_once_and_does_not_shorten_the_countdown) {
  Pad p;
  p.low(3U);
  CHECK(p.c.launch());
  CHECK(p.frame(false) == T0::None);
  CHECK(p.frame(true) == T0::None);
  CHECK(p.frame(true) == T0::TooEarly);  // 10 s before T-zero
  for (unsigned i = 0; i < 20U; ++i) {
    CHECK(p.frame(true) == T0::None);  // it stays high: it is not a new edge, and it never turns into T-zero when the window opens
  }
  CHECK(mission::in_countdown(p.mission()) && mission::frames_to_zero(p.mission()) > 900U);
  while (mission::frames_to_zero(p.mission()) > SyncClock::kT0Window) {
    CHECK(p.frame(true) == T0::None);
  }
  CHECK(p.frame(true) == T0::None && mission::in_countdown(p.mission()));
}

TFC_TEST(t0_line_stuck_high_since_the_start_is_never_an_edge) {
  Pad p;
  CHECK(p.frame(true) == T0::None && p.frame(true) == T0::None && p.frame(true) == T0::None);
  CHECK(p.c.launch());
  unsigned frames = 0U;
  while (!mission::in_flight(p.mission())) {
    CHECK(p.frame(true) == T0::None);
    ++frames;
    CHECK(frames < 1100U);
  }
  CHECK(frames == mission::kCountdownFrames);  // the countdown ran its full length
}

TFC_TEST(t0_line_a_node_that_first_looks_at_the_line_in_the_window_and_finds_it_high_does_not_take_it_for_an_edge) {
  SyncClock c(SyncStart::Master);  // a master that took over late: it has never seen the line low
  uint32_t n = 0U;
  (void)c.cycle(false, n++);
  CHECK(c.launch());
  while (mission::frames_to_zero(c.mission_frame()) > SyncClock::kT0Window) {
    (void)c.cycle(false, n++);
  }
  (void)c.cycle(false, n++);
  CHECK(c.t0_line(true) == T0::None);
  (void)c.cycle(false, n++);
  CHECK(c.t0_line(true) == T0::None);  // the second high sample: no edge was seen, only a level
  CHECK(mission::in_countdown(c.mission_frame()));
  (void)c.cycle(false, n++);
  CHECK(c.t0_line(false) == T0::None);
  (void)c.cycle(false, n++);
  CHECK(c.t0_line(true) == T0::None);
  (void)c.cycle(false, n++);
  CHECK(c.t0_line(true) == T0::Latched);  // after a low, an edge counts
}

TFC_TEST(t0_line_one_noisy_sample_is_not_an_edge_and_the_edge_must_be_two_samples_in_a_row) {
  Pad p;
  p.low(3U);
  CHECK(p.c.launch());
  while (mission::frames_to_zero(p.mission()) > SyncClock::kT0Window) {
    (void)p.frame(false);
  }
  CHECK(p.frame(true) == T0::None);   // a spike of one frame
  CHECK(p.frame(false) == T0::None);
  CHECK(p.frame(true) == T0::None);   // another, not adjacent
  CHECK(p.frame(false) == T0::None);
  CHECK(mission::in_countdown(p.mission()));
  CHECK(p.frame(true) == T0::None && p.frame(true) == T0::Latched);
}

TFC_TEST(t0_line_the_window_edge_is_exact) {
  for (const unsigned to_go : std::array<unsigned, 2>{{SyncClock::kT0Window, SyncClock::kT0Window + 1U}}) {  // frames to T-zero at the moment of the decision
    Pad p;
    p.low(3U);
    CHECK(p.c.launch());
    while (mission::frames_to_zero(p.mission()) > to_go + 2U) {
      (void)p.frame(false);
    }
    CHECK(p.frame(true) == T0::None);
    const T0 r = p.frame(true);  // the cycle of this frame has counted the mission frame on: `to_go` frames remain
    CHECK(r == (to_go <= SyncClock::kT0Window ? T0::Latched : T0::TooEarly));
  }
}

TFC_TEST(t0_line_only_the_sync_master_acts_and_only_in_a_countdown) {
  Pad master;
  master.low(3U);
  CHECK(master.frame(true) == T0::None && master.frame(true) == T0::None);  // not launched: a supervisor's T0 means nothing
  CHECK(master.mission() == mission::kNotLaunched);
  SyncClock follower(SyncStart::FollowOnly);
  (void)follower.cycle(true, 0U, 500U);  // in a countdown that the master runs
  CHECK(follower.t0_line(false) == T0::None);
  for (unsigned i = 0; i < 4U; ++i) {
    CHECK(follower.t0_line(true) == T0::None);
  }
  CHECK(follower.mission_frame() == 501U);  // it only counts on; T-zero reaches it in SYNC
}
