#!/usr/bin/env python3
"""Worst-case response times of the flight bus's frames under CAN arbitration (docs/design/PROTOCOL.md; docs/design/FUTURE_WORK.md 6b item 14).

The bus-load figure of the design (111 bits a frame, no stuffing) is arithmetic. A time-triggered schedule has a stronger question: does every frame reach the other end before the step that needs it,
with the arbitration, the blocking by a frame already on the wire, the bit stuffing and the release jitter all at their worst? Two answers are given.

  1. A bound (Davis, Burns, Bril and Lukkien, "Controller Area Network schedulability analysis: refuted, revisited and revised", 2007, from memory): the frames as sporadic tasks at the critical instant
     (every frame released together, which the slot schedule prevents, so the bound is pessimistic), non-preemptive fixed priority by identifier, the worst-case stuffed frame (135 bits for eight data
     bytes in the standard format) and blocking by the longest lower-priority frame. If the bound meets the deadline, the schedule meets it under any release times.
  2. A run of the schedule itself: every frame released at its slot's start plus a random jitter, a bus that sends the lowest identifier pending when it goes idle, every frame at its worst-case length,
     over many frames of 10 ms. The largest response seen. It is evidence about the schedule and not a bound.

The slot times are those of docs/design/ARCHITECTURE.md section 3 (the ones the design gives; the offsets of the propulsion and surface frames, which the design only says follow the command, are
assumed). The jitter is a parameter (default 100 us: the design's jitter target). Nothing here is measured on a bus: `tools/bench/bus_loss.py` and a logic analyser are, and wait for the board.

    python3 tools/bench/can_rta.py [--bitrate 1000000] [--jitter-us 100] [--cycles 20000] [--no-sim] [--with-gnc-frames] [--strict]
"""
import argparse
import math
import random
import sys

TAU_US = 1e6  # replaced by 1e6 / bitrate


def frame_bits(dlc=8):
    """The longest a standard-format frame can be on the wire, with stuffing: (34 + 8n) + 13 + floor((34 + 8n - 1) / 4) bits (Davis et al.)."""
    return 34 + 8 * dlc + 13 + (34 + 8 * dlc - 1) // 4


def schedule(with_sim=True, with_gnc=False):
    """The periodic traffic of one 10 ms frame: (name, id, release offset us, deadline offset us, period in frames). The deadline is the time by which the receiver needs the frame.

    What each computer does is firmware/app/src/main.cpp's: the three computers lock to SYNC and each sends at the same instants (not staggered): at 1.5 ms its gyro and accel frames back to back
    (`kGyroUs`), at 5.0 ms its command (`kCmdUs`), in a resync frame at 5.5 ms its four state frames (`kResyncTxUs`), and after the vote at `kVoteUs` = 7.0 ms its heartbeat; ACT publishes its output at
    the vote. The propulsion and surface frames (0x210, 0x220) are not sent by the firmware yet (LIMITATIONS 10), so they are not here. The simulator's traffic (0x501 to 0x508) is the design's: in the slack
    after 7 ms (the offsets within it are assumed); `with_sim=False` leaves it out (the rig on the bench has it, the vehicle has not)."""
    f = [("SYNC", 0x010, 0, 1500, 1)]
    for n in range(3):
        f.append((f"gyro {n}", 0x100 + n, 1500, 3000, 1))
        f.append((f"accel {n}", 0x110 + n, 1500, 3000, 1))
    for n in range(3):
        f.append((f"command {n}", 0x200 + n, 5000, 7000, 1))
    for n in range(3):
        for k in range(4):
            f.append((f"resync {n}.{k}", 0x420 + 4 * n + k, 5500, 8000, 100))      # a resync frame only: the last of every 100; the others' are collected at vote time + 1 ms
    if with_gnc:                                    # the propulsion and surface commands a flight computer that flies a mission sends after its control command (not in the firmware yet)
        for n in range(3):
            f.append((f"propulsion {n}", 0x210 + n, 5000, 7000, 1))
            f.append((f"surface {n}", 0x220 + n, 5000, 7000, 1))
    f.append(("ACT output", 0x300, 7000, 10000, 1))
    for n in range(3):
        f.append((f"heartbeat {n}", 0x400 + n, 7000, 10000, 1))
    if with_sim:
        for i, ident in enumerate(range(0x501, 0x509)):
            f.append((f"sim {ident:#x}", ident, 7800 + 100 * i, 10000, 1))
    return f


def bound(frames, tau, jitter_us):
    """The Davis et al. worst-case response time of each frame, in us, at the critical instant with every frame at its period of one cycle (a once-per-period frame is taken at its short period of a cycle too:
    the analysis has to hold in the cycle where it is sent)."""
    c = {ident: frame_bits() * tau for (_n, ident, _o, _d, _p) in frames}
    out = {}
    for (name, ident, _o, _d, _p) in frames:
        hp = [i for (_n, i, _o2, _d2, _p2) in frames if i < ident]
        lp = [i for (_n, i, _o2, _d2, _p2) in frames if i > ident]
        blocking = max([c[i] for i in lp] + [0.0])
        w = blocking
        for _ in range(1000):
            new = blocking + sum(math.ceil((w + jitter_us + tau) / 10000.0) * c[i] for i in hp)
            if new == w:
                break
            w = new
        out[ident] = jitter_us + w + c[ident]
    return out


def simulate(frames, tau, jitter_us, cycles, seed=1):
    """The schedule on a bus that sends the lowest identifier pending when it goes idle; returns the largest response (release to the end of the frame) of each frame, in us."""
    rnd = random.Random(seed)
    c = frame_bits() * tau
    worst = {ident: 0.0 for (_n, ident, _o, _d, _p) in frames}
    for cyc in range(cycles):
        rel = []
        for (_n, ident, off, _dl, per) in frames:
            if per > 1 and (cyc % per) != per - 1:
                continue            # a once-per-period frame goes in the last frame of the period
            rel.append((off + rnd.uniform(0.0, jitter_us), ident))
        rel.sort()
        t = 0.0
        pending = []
        i = 0
        done = 0
        while done < len(rel):
            while i < len(rel) and rel[i][0] <= t:
                pending.append(rel[i])
                i += 1
            if not pending:
                t = rel[i][0]
                continue
            pending.sort(key=lambda r: r[1])
            r, ident = pending.pop(0)
            t += c
            worst[ident] = max(worst[ident], t - r)
            done += 1
    return worst


def group_of(name, ident, off):
    """Frames that one computer (or the one simulator host) releases in one burst share one timer, so they share one jitter."""
    if name.startswith("sim"):
        return ("sim",)
    for kind in ("gyro", "accel", "command", "heartbeat", "resync"):
        if name.startswith(kind):
            return (off, int(name.split()[1].split(".")[0]))
    return (name,)


def corners(frames, tau, jitter_us, resync_cycle):
    """The largest response of each frame over every assignment of the jitter, 0 or its largest, to each burst (a computer's frames of one slot, the simulator's frames): the schedule's worst corner. Exact for a
    schedule whose responses grow with the release times of the others, which fixed-priority arbitration is except for the one case of a frame that arrives just after a lower-priority one has started
    (the corner where it is released earliest and the blockers latest is among them)."""
    c = frame_bits() * tau
    use = [f for f in frames if resync_cycle or f[4] == 1]
    keys = sorted({group_of(n, i, o) for (n, i, o, _d, _p) in use}, key=str)
    worst = {ident: 0.0 for (_n, ident, _o, _d, _p) in use}
    for mask in range(1 << len(keys)):
        jit = {k: (jitter_us if (mask >> j) & 1 else 0.0) for j, k in enumerate(keys)}
        rel = sorted((off + jit[group_of(n, ident, off)], ident) for (n, ident, off, _d, _p) in use)
        t = 0.0
        pending = []
        i = 0
        done = 0
        while done < len(rel):
            while i < len(rel) and rel[i][0] <= t:
                pending.append(rel[i])
                i += 1
            if not pending:
                t = rel[i][0]
                continue
            pending.sort(key=lambda r: r[1])
            r, ident = pending.pop(0)
            t += c
            worst[ident] = max(worst[ident], t - r)
            done += 1
    return worst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bitrate", type=float, default=1e6)
    ap.add_argument("--jitter-us", type=float, default=100.0)
    ap.add_argument("--cycles", type=int, default=20000)
    ap.add_argument("--no-sim", action="store_true", help="leave out the simulator's traffic (0x501 to 0x508)")
    ap.add_argument("--with-gnc-frames", action="store_true", help="add the propulsion and surface frames (0x210, 0x220) of a computer that flies a mission: not sent by the firmware yet")
    ap.add_argument("--strict", action="store_true")
    a = ap.parse_args(argv)
    tau = 1e6 / a.bitrate
    frames = schedule(not a.no_sim, a.with_gnc_frames)
    b = bound(frames, tau, a.jitter_us)
    s = simulate(frames, tau, a.jitter_us, a.cycles)
    cr = corners(frames, tau, a.jitter_us, True)
    print(f"{len(frames)} frames in a 10 ms frame (the periodic ones), {frame_bits()} bits each at worst, {a.bitrate / 1e6:g} Mbit/s, jitter {a.jitter_us:g} us")
    load = sum(frame_bits() * tau / (10000.0 * (1 if p == 1 else p)) for (_n, _i, _o, _d, p) in frames)
    print(f"worst-case bus load {100 * load:.1f} % (stuffed frames, the once-per-period ones spread over the period)")
    print(f"{'frame':<16} {'id':>6} {'release':>8} {'deadline':>9} {'bound us':>9} {'run max us':>11} {'worst corner us':>16} {'slack (corner) us':>18}")
    bad = 0
    for (name, ident, off, dl, per) in frames:
        slack = (dl - off) - cr[ident]
        flag = "" if slack >= 0 else "  DEADLINE MISSED"
        if slack < 0:
            bad += 1
        print(f"{name:<16} {ident:#06x} {off:8.0f} {dl:9.0f} {b[ident]:9.0f} {s[ident]:11.0f} {cr[ident]:16.0f} {slack:18.0f}{flag}")
    worst_slack = min((dl - off) - cr[ident] for (_n, ident, off, dl, _p) in frames)
    nb = sum(1 for (_n, ident, off, dl, _p) in frames if (dl - off) - b[ident] < 0)
    print(f"smallest slack over the jitter corners: {worst_slack:.0f} us; frames that miss their deadline in a corner: {bad}; frames whose critical-instant bound (pessimistic: it releases every frame together) exceeds their window: {nb}")
    return 3 if (bad and a.strict) else 0


if __name__ == "__main__":
    sys.exit(main())
