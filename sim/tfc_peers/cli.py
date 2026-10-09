# SPDX-License-Identifier: MIT
"""Command line: python3 -m tfc_peers {run,record,listen,log,decode,command,launch,pico,faults} ..."""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time
from pathlib import Path

from . import bus as B
from . import control as CTL
from . import protocol as P
from .commands import GroundCommand, parse_commands, parse_phase
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
    commands = [c for spec in args.command for c in parse_commands(spec)]
    return Scenario(nodes, faults, args.seed, commands)


def _add_scenario_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--nodes", default="B,C",
                   help="virtual flight computers to simulate, comma separated, from A,B,C (or 0,1,2); "
                        "default B,C. Use A,B,C for a fully virtual triplex; with a real FC-A on the bus use B,C")
    p.add_argument("--fault", action="append", default=[], metavar="SPEC",
                   help="NODE:KIND[:key=value,...], repeatable; see `faults` (e.g. B:bias:start=100,mag=3)")
    p.add_argument("--command", action="append", default=[], metavar="SPEC",
                   help="scripted operator command FRAME:OP[:NODE], repeatable; OP is reintegrate, disable, "
                        "clear-disabled, clear-safe, warm, phase (its NODE is a phase: 0..7 or a name) or noop, optionally prefixed arm-, armed- or forged- (e.g. 450:reintegrate:B, "
                        "600:armed-clear-safe), or FRAME:replay. Sent as an authenticated ground-command frame "
                        "in that frame number")
    p.add_argument("--seed", type=int, default=1,
                   help="seed for sensor noise and random faults; the same seed gives byte-identical traffic (default 1)")
    p.add_argument("--frames", type=int, default=1000,
                   help="number of 10 ms major frames (default 1000 = 10 s); 100 frames = 1 s. "
                        "For `run`, 0 means keep going until Ctrl+C")


def _control_poll(sc: Scenario):
    """`--control`: a thread reads lines from standard input; the returned function, called once per frame, applies them and answers on standard output."""
    lines: queue.SimpleQueue[str | None] = queue.SimpleQueue()

    def reader() -> None:
        for line in sys.stdin:
            lines.put(line)
        lines.put(None)  # end of input: the controller has gone; the run goes on with the faults it has

    threading.Thread(target=reader, daemon=True).start()

    def poll(k: int) -> None:
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                return
            if line is None:
                return
            for reply in CTL.handle(sc, line, k):
                print(reply, flush=True)

    return poll


def cmd_run(args: argparse.Namespace) -> int:
    sc = _scenario(args)
    if args.control and not args.follow_sync:
        print("error: --control needs --follow-sync (commands are applied between SYNC frames)", file=sys.stderr)
        return 2
    bus = B.SocketCanBus(args.iface)
    names = ",".join("ABC"[n] for n in sc.nodes)
    mode = "following SYNC from the flight computer" if args.follow_sync else "free-running at 100 Hz"
    length = (f"{args.frames} frames ({args.frames / 100:.1f} s)" if args.frames > 0
              else "until Ctrl+C")
    print(f"virtual {names} on {args.iface}, {mode}, {length}; "
          f"faults: {[str(f) for f in sc.faults] or 'none'}", flush=True)
    try:
        if args.follow_sync:
            st = B.run_synced(sc, bus, args.frames, on_note=lambda m: print(f"note: {m}", flush=True), poll=_control_poll(sc) if args.control else None)
        else:
            st = B.run_realtime(sc, bus, args.frames)
    except TimeoutError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    finally:
        bus.close()
    print(f"{'interrupted; ' if st.get('interrupted') else ''}sent {st['sent']:.0f} frames in "
          f"{st['frames']:.0f} frames' time; send lateness p50 {st['late_p50_us']:.0f} us, "
          f"p99 {st['late_p99_us']:.0f} us, max {st['late_max_us']:.0f} us (Python, not real-time)")
    if args.frames > 0 and not st.get("interrupted"):
        print("The peers have stopped sending. A flight computer that is still running will now report "
              "them as 'frame missing' within 3 frames; that is expected, not a fault. "
              "Use --frames 0 to keep the peers running until Ctrl+C.")
    if st.get("interrupted"):
        return 0 if args.frames <= 0 else 130
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


def cmd_log(args: argparse.Namespace) -> int:
    """Record what is on a CAN interface into a candump -L log that `decode` and `tfc_replay` read (timestamps from 0)."""
    bus = B.SocketCanBus(args.iface)
    out = B.LogBus(args.out, args.iface)
    t0 = time.monotonic()
    n = 0
    try:
        while args.duration is None or time.monotonic() - t0 < args.duration:
            frame = bus.recv(0.2)
            if frame is None:
                continue
            out.send(int((time.monotonic() - t0) * 1e6), frame)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
        out.close()
    print(f"wrote {n} frames from {args.iface} to {args.out}")
    return 0


def cmd_decode(args: argparse.Namespace) -> int:
    limit = args.limit
    for i, (t_us, _iface, frame) in enumerate(B.read_log(args.log)):
        if limit is not None and i >= limit:
            break
        print(f"{t_us / 1e6:9.4f}  {frame.id:03X}  {describe(frame)}")
    return 0


def _counter_file() -> Path:
    base = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    return base / "tfc_peers" / "ground_counter"


def _next_counter(count: int, explicit: int | None) -> int:
    """The first of `count` consecutive command counters, remembered between runs (the flight computer wants them to increase)."""
    path = _counter_file()
    if explicit is not None:
        first = explicit
    else:
        try:
            first = (int(path.read_text()) + 1) & 0xFF
        except (OSError, ValueError):
            first = 1
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(str((first + count - 1) & 0xFF))
    except OSError:
        pass  # the counter is only a convenience; --counter overrides it
    return first


def cmd_command(args: argparse.Namespace) -> int:
    op = args.op.lower()
    if op not in P.GROUND_OPS:
        raise FaultSpecError(f"unknown command {op!r}; known: {', '.join(P.GROUND_OPS)}")
    if op not in P.NODELESS_OPS and args.node is None:
        raise FaultSpecError(f"command {op!r} needs a phase (0..7 or a name)" if op in P.PHASE_OPS else f"command {op!r} needs a node (A, B or C)")
    node = (parse_phase(args.node) if op in P.PHASE_OPS else parse_node(args.node)) if args.node is not None else 0
    steps = [GroundCommand(0, op, node, arm=True), GroundCommand(0, op, node)] if args.arm else [GroundCommand(0, op, node)]
    counter = _next_counter(len(steps), args.counter)
    bus = B.SocketCanBus(args.iface)
    try:
        for i, cmd in enumerate(steps):
            bus.send(0, cmd.frame_for((counter + i) & 0xFF))
            time.sleep(0.05)  # five frames between the ARM and the EXECUTE
    finally:
        bus.close()
    who = "" if op in P.NODELESS_OPS else (f" {P.PHASE_NAMES[node]}" if op in P.PHASE_OPS else f" {P.NODE_NAMES[node]}")
    print(f"sent ground command: {'ARM + ' if args.arm else ''}{op}{who} (counter {counter}"
          f"{'-' + str((counter + 1) & 0xFF) if args.arm else ''}) on {args.iface}; the flight computer prints the outcome "
          f"(accepted or why it was refused) on its console")
    return 0


def cmd_launch(args: argparse.Namespace) -> int:
    from . import launch as LA

    bus = B.SocketCanBus(args.iface)
    bus.set_filter([(P.ID_HEARTBEAT, 0x7FC), (P.ID_ACT_OUT, 0x7FF), (P.ID_SYNC, 0x7FF)])
    try:
        if not args.check and not args.yes:
            answer = input("Launch checklist complete and every person clear? Type LAUNCH to continue: ")
            if answer.strip() != "LAUNCH":
                print("Not launched.")
                return 1
        counter = _next_counter(2, args.counter) if not args.check else 0
        return LA.run(lambda t: bus.recv(t), lambda f: bus.send(0, f), time.monotonic, time.sleep, counter, wait_s=args.wait, check_only=args.check)
    finally:
        bus.close()


def cmd_pico(args: argparse.Namespace) -> int:
    from . import pico_link as L

    port = L.open_serial(args.port)
    client = L.PicoClient(port)
    op = args.op.lower()
    if op == "status":
        pass
    elif op == "platform":
        if len(args.operands) != 2:
            raise ValueError("platform needs two tilts: platform X_DEG Y_DEG [--for SECONDS]")
        x, y = float(args.operands[0]), float(args.operands[1])
        end = time.monotonic() + max(args.seconds, 0.0)
        client.platform(x, y)
        while time.monotonic() < end:  # the driver holds after 100 ms without a command: keep sending at 100 Hz
            time.sleep(0.01)
            client.platform(x, y)
            client.poll()
    elif op == "cut":
        if len(args.operands) != 2:
            raise ValueError("cut needs a node and a time: cut B 3000 (A, B, C or ACT; milliseconds, at most 30000; it ends by itself)")
        client.cut(args.operands[0], int(args.operands[1]))
    elif op == "restore":
        if len(args.operands) != 1:
            raise ValueError("restore needs a node or 'all'")
        if args.operands[0].lower() == "all":
            client.restore_all()
        else:
            client.restore(args.operands[0])
    else:
        raise ValueError(f"unknown pico operation {args.op!r}: status, platform, cut or restore")
    st = client.wait_status(1.0)
    print(st.describe() if st is not None else "no status from the board (is the port right, and the firmware running?)")
    return 0 if st is not None else 1


def cmd_faults(_args: argparse.Namespace) -> int:
    print("Fault kinds (NODE:KIND[:start=N,end=N,period=N,duty=N,key=value,...]; frame numbers are 10 ms frames)\n")
    print("Any fault also accepts period=N,duty=K: intermittent, active K frames out of every N from `start`.\n")
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
    p.add_argument("--control", action="store_true",
                   help="read fault commands from standard input while running (needs --follow-sync): `add SPEC [for N]`, `clear ID|all`, `list`, `frame`; "
                        "one answer line each (`ok ...` or `error: ...`). See tfc_peers/control.py")
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

    p = sub.add_parser("log", help="record the frames on a SocketCAN interface into a candump-format log (live)")
    p.add_argument("--iface", default="vcan0", help="SocketCAN interface to record (default vcan0; can0 for the USB-CAN adapter)")
    p.add_argument("--out", required=True, help="log file to write (candump -L format, timestamps from 0); overwritten if it exists")
    p.add_argument("--duration", type=float, default=None, help="stop after this many seconds (default: run until Ctrl+C)")
    p.set_defaults(fn=cmd_log)

    p = sub.add_parser("decode", help="print a candump-format log in decoded form")
    p.add_argument("log", help="candump -L format log, e.g. one written by `record`")
    p.add_argument("--limit", type=int, default=None, help="print only the first N frames")
    p.set_defaults(fn=cmd_decode)

    p = sub.add_parser("command", help="send one operator command to the flight computers now (live)")
    p.add_argument("op", help="reintegrate, disable, clear-disabled, clear-safe, launch, scrub, warm, phase or noop")
    p.add_argument("node", nargs="?", help="A, B or C (not needed for clear-safe, launch or scrub)")
    p.add_argument("--iface", default="vcan0", help="SocketCAN interface to send on (default vcan0)")
    p.add_argument("--arm", action="store_true", help="send the ARM frame, then the EXECUTE frame 50 ms later (clear-safe, "
                   "clear-disabled and a disable that would leave fewer than two healthy nodes need this)")
    p.add_argument("--counter", type=int, default=None, metavar="N", help="command counter of the first frame (default: the next one "
                   "after the last this tool sent, remembered in ~/.cache/tfc_peers/ground_counter)")
    p.set_defaults(fn=cmd_command)

    p = sub.add_parser("launch", help="the launch checklist, automated: watch the go/no-go, send the launch (ARM then EXECUTE), run the countdown; exit 0 at T-zero")
    p.add_argument("--iface", default="vcan0", help="SocketCAN interface (default vcan0)")
    p.add_argument("--wait", type=float, default=60.0, help="seconds to wait for a go (default 60)")
    p.add_argument("--check", action="store_true", help="only report the go/no-go and exit (0 go, 1 no-go); sends nothing")
    p.add_argument("--yes", action="store_true", help="do not ask for the typed confirmation")
    p.add_argument("--counter", type=int, default=None, metavar="N", help="command counter of the ARM frame (default: the next one, as for `command`)")
    p.set_defaults(fn=cmd_launch)

    p = sub.add_parser("pico", help="talk to the Pico (platform driver and fault injector) over USB serial: status, platform X Y, cut NODE MS, restore NODE|all")
    p.add_argument("op", help="status, platform, cut or restore")
    p.add_argument("operands", nargs="*", help="platform: X_DEG Y_DEG; cut: NODE MS (A, B, C or ACT); restore: NODE or all")
    p.add_argument("--port", default="/dev/ttyACM0", help="the Pico's serial port (default /dev/ttyACM0)")
    p.add_argument("--for", dest="seconds", type=float, default=0.0, help="platform: keep sending the tilt at 100 Hz for this many seconds")
    p.set_defaults(fn=cmd_pico)

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
