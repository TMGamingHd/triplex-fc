# SPDX-License-Identifier: MIT
"""Command line: python3 -m tfc_peers {run,record,listen,decode,faults} ..."""
from __future__ import annotations

import argparse
import sys
import time

from . import bus as B
from .faults import KINDS, FaultSpecError, parse_fault, parse_node
from .peers import Scenario
from .protocol import describe


def _scenario(args: argparse.Namespace) -> Scenario:
    nodes = [parse_node(n) for n in args.nodes.split(",") if n.strip()]
    if not nodes:
        raise FaultSpecError("--nodes is empty: give at least one of A, B, C (e.g. --nodes B,C)")
    faults = [parse_fault(f) for f in args.fault]
    for f in faults:
        if f.node not in nodes:
            raise FaultSpecError(f"fault {f} targets a node that is not simulated (--nodes {args.nodes})")
    return Scenario(nodes, faults, args.seed)


def _add_scenario_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--nodes", default="B,C",
                   help="virtual flight computers to simulate, comma separated, from A,B,C (or 0,1,2); "
                        "default B,C. Use A,B,C for a fully virtual triplex; with a real FC-A on the bus use B,C")
    p.add_argument("--fault", action="append", default=[], metavar="SPEC",
                   help="NODE:KIND[:key=value,...], repeatable; see `faults` (e.g. B:bias:start=100,mag=3)")
    p.add_argument("--seed", type=int, default=1,
                   help="seed for sensor noise and random faults; the same seed gives byte-identical traffic (default 1)")
    p.add_argument("--frames", type=int, default=1000,
                   help="number of 10 ms major frames (default 1000 = 10 s); 100 frames = 1 s")


def cmd_run(args: argparse.Namespace) -> int:
    sc = _scenario(args)
    bus = B.SocketCanBus(args.iface)
    names = ",".join("ABC"[n] for n in sc.nodes)
    mode = ("following SYNC from the flight computer" if args.follow_sync else "free-running at 100 Hz")
    print(f"virtual {names} on {args.iface}, {mode}, {args.frames} frames ({args.frames / 100:.1f} s); "
          f"faults: {[str(f) for f in sc.faults] or 'none'}", flush=True)
    try:
        if args.follow_sync:
            st = B.run_synced(sc, bus, args.frames, on_note=lambda m: print(f"note: {m}", flush=True))
        else:
            st = B.run_realtime(sc, bus, args.frames)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except TimeoutError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    finally:
        bus.close()
    print(f"sent {st['sent']:.0f} frames; send lateness p50 {st['late_p50_us']:.0f} us, "
          f"p99 {st['late_p99_us']:.0f} us, max {st['late_max_us']:.0f} us (Python, not real-time)")
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    sc = _scenario(args)
    bus = B.LogBus(args.out, args.iface)
    try:
        n = B.record(sc, bus, args.frames)
    finally:
        bus.close()
    print(f"wrote {n} frames ({args.frames} major frames) to {args.out}")
    return 0


def cmd_listen(args: argparse.Namespace) -> int:
    bus = B.SocketCanBus(args.iface)
    t0 = time.monotonic()
    shown = 0
    try:
        while args.duration is None or time.monotonic() - t0 < args.duration:
            frame = bus.recv(0.2)
            if frame is None:
                continue
            print(f"{time.monotonic() - t0:9.4f}  {frame.id:03X}  {describe(frame)}", flush=True)
            shown += 1
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
    print(f"{shown} frames")
    return 0


def cmd_decode(args: argparse.Namespace) -> int:
    limit = args.limit
    for i, (t_us, _iface, frame) in enumerate(B.read_log(args.log)):
        if limit is not None and i >= limit:
            break
        print(f"{t_us / 1e6:9.4f}  {frame.id:03X}  {describe(frame)}")
    return 0


def cmd_faults(_args: argparse.Namespace) -> int:
    print("Fault kinds (NODE:KIND[:start=N,end=N,key=value,...]; start/end are 10 ms frame numbers)\n")
    for kind, (row, desc, params) in KINDS.items():
        opts = ", ".join(f"{k}={v}" for k, v in params.items()) or "-"
        print(f"  {kind:<11} {row:<4} {desc}\n  {'':<11} {'':<4} options: {opts}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="tfc_peers", description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("run", help="send virtual peers' traffic on a SocketCAN interface in real time")
    _add_scenario_args(p)
    p.add_argument("--iface", default="vcan0",
                   help="SocketCAN interface to send on (default vcan0; can0 for the USB-CAN adapter)")
    p.add_argument("--follow-sync", action="store_true",
                   help="phase-lock to the flight computer's SYNC frames (frame numbers come from SYNC); "
                        "--frames then counts SYNC frames. Without it the peers free-run on their own 100 Hz clock")
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("record", help="generate traffic offline into a candump-format log (no sleeping)")
    _add_scenario_args(p)
    p.add_argument("--out", required=True, help="log file to write (candump -L format); overwritten if it exists")
    p.add_argument("--iface", default="vcan0", help="interface name written into each log line (default vcan0)")
    p.set_defaults(fn=cmd_record)

    p = sub.add_parser("listen", help="print decoded frames seen on a SocketCAN interface")
    p.add_argument("--iface", default="vcan0", help="SocketCAN interface to monitor (default vcan0)")
    p.add_argument("--duration", type=float, default=None, help="stop after this many seconds (default: run until Ctrl+C)")
    p.set_defaults(fn=cmd_listen)

    p = sub.add_parser("decode", help="print a candump-format log in decoded form")
    p.add_argument("log", help="candump -L format log, e.g. one written by `record`")
    p.add_argument("--limit", type=int, default=None, help="print only the first N frames")
    p.set_defaults(fn=cmd_decode)

    p = sub.add_parser("faults", help="list fault kinds and their options")
    p.set_defaults(fn=cmd_faults)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.fn(args)
    except (FaultSpecError, OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
