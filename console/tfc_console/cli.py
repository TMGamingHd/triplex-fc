# SPDX-License-Identifier: MIT
"""`tfc-console`: start the flight console.

    tfc-console                         attach to vcan0 and open the page
    tfc-console --iface can0            the USB-CAN adapter
    tfc-console --replay FILE.log       play a recording (a console recording, or any candump -L log)
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import webbrowser
from pathlib import Path

from .app import ROOT, WEB, SourceManager, register_core
from .hub import Hub
from .server import App, start_in_thread
from .sources import TruthSource


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="tfc-console", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group()
    src.add_argument("--iface", default=None, metavar="IFACE", help="SocketCAN interface to attach to (default vcan0; can0 for the USB-CAN adapter)")
    src.add_argument("--replay", metavar="FILE", help="play a recorded log (candump -L format, .log or .log.gz; a sidecar .side.jsonl beside it is played too) instead of a live bus")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed (default 1)")
    p.add_argument("--host", default="127.0.0.1", help="address to listen on (default 127.0.0.1: this machine only). 0.0.0.0 makes the console, and its commands, reachable from the network: the session token is then the only protection")
    p.add_argument("--port", type=int, default=8765, help="port to listen on (default 8765; 0 picks a free one)")
    p.add_argument("--token", default=None, help="session token (default: a random one, printed in the URL)")
    p.add_argument("--truth-port", type=int, default=TruthSource.DEFAULT_PORT, help=f"UDP port the simulator's telemetry is received on (default {TruthSource.DEFAULT_PORT}; give `tfc_simd --telemetry` the same). The rig the console starts is told it")
    p.add_argument("--no-open", action="store_true", help="do not open the page in the browser")
    p.add_argument("--no-rig", action="store_true", help="do not offer to start and stop the rig's processes (the page can only watch and command)")
    p.add_argument("--repo", default=str(ROOT), help="the repository the binaries, vehicle files and logs are in (default: this one)")
    return p


def yield_cpu() -> None:
    """Run this thread, and every thread it starts after this, a little behind the rig's processes.

    The virtual rig is five processes with 10 ms deadlines on an ordinary desktop; the console, and the browser that watches it, are not what the rig's timing should depend on. On Linux a nice value belongs to
    a *thread*, so the thread that starts the rig's processes (`rig.Rig`, created before this call) keeps the default and the children inherit it, while the server's threads, started after this, yield.
    """
    try:
        os.setpriority(os.PRIO_PROCESS, 0, 5)
    except (OSError, AttributeError):
        pass


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    hub = Hub()
    repo = Path(args.repo).resolve()
    mgr = SourceManager(hub, repo)
    truth = TruthSource(hub, args.truth_port)
    truth.start()
    app = App(hub, WEB, args.host, args.port, args.token)
    register_core(app, mgr)
    from . import services                       # the parts that act on the rig; imported here so a read-only console (`--replay`) does not need them
    services.install(app, hub, mgr, truth, repo, rig=not args.no_rig)
    yield_cpu()
    hub.start()
    app.on_close.append(lambda: (mgr.disconnect(), truth.stop(), hub.stop()))
    try:
        if args.replay:
            mgr.open_replay(Path(args.replay), args.speed)
        else:
            iface = args.iface or "vcan0"
            try:
                mgr.connect_live(iface)
            except OSError as e:
                print(f"note: no data source yet: {e}\n      the page lets you connect an interface or open a recording", file=sys.stderr)
                hub.status.update(kind="none", state="no source", message=str(e))
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    start_in_thread(app)
    url = app.url()
    print(f"tfc-console: {url}", flush=True)
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        print("warning: listening beyond this machine; anyone who has the token can send commands to the rig", file=sys.stderr)
    if not args.no_open:
        threading.Thread(target=lambda: webbrowser.open(url), daemon=True).start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    stop.wait()
    print("tfc-console: stopping", flush=True)
    app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
