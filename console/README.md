# The flight console

One web page that shows the triplex flight computer and the vehicle it flies, and sends the operator's commands: the launch sequence, how the three computers vote, where the rocket is, the faults being injected and what the system does about them, the vehicle's parameters, the bus, the events, and a button for every ground command. It runs on the bench PC and reads the same CAN bus the nodes talk on. Design, principles, what is measured and what is not: [`docs/design/CONSOLE.md`](../docs/design/CONSOLE.md) (ADR-034).

**It is a bench tool, not flight software.** It decides nothing about the flight, sends only authenticated ground commands, and has not met a board yet.

![The Voting tab replaying the recorded run: ACT's vote with node B lost](../docs/design/img/console-voting.png)

## Quick start
```bash
sim/scripts/setup_vcan.sh            # once per boot: creates vcan0 (needs sudo)
console/tfc-console                  # opens the page; the URL carries this session's token
```
In the page: **Rig** tab, *Start* on "Closed loop with the launch sequence" (needs the launch images: `tools/bench/sil_triplex.sh --build --launch`, and `cmake --build build/host` for `tfc_simd` and `tfc_fly`). About 10 s later the **Launch** tab says GO; *ARM and LAUNCH…* asks you to type `LAUNCH`. Then watch **Flight** and **Voting**; on the **Faults** tab, start the *Fault lab* profile instead and inject a fault on B or C.

**The 3D viewer** is a second page of the same server, for a window of its own (a second monitor): the **3D** button in the console's header, or the `3D viewer:` address printed at start (`/viewer/` with the same token). It draws the vehicle (a Starship-class stack, any vehicle file, or a CAD model you put in `console/models/`) in a physically drawn sky, in six lenses, from the live simulator or a recorded flight: [`docs/design/VIEWER.md`](../docs/design/VIEWER.md). `console/tfc-console --viewer console/demo/starship-ascent.pose.jsonl.gz` needs no bus; the page's source menu flies any vehicle in `vehicles/` with the real flight software and plays it.

No bus yet? `console/tfc-console --replay console/demo/launch-and-node-loss.log.gz`: a real run (a launch, then node B killed at T+23 s), with the nodes' consoles and the simulator's telemetry; play, pause, seek and speed are in the bar at the top of the page.

Standard library only (written and tested on Python 3.14; `pyserial` only if you attach serial devices): nothing to install, nothing fetched from the network.

## Flags
| Flag | Meaning |
|---|---|
| `--iface IFACE` | The SocketCAN interface to attach to (default `vcan0`; `can0` for the USB-CAN adapter). If it does not exist the console still starts and the Rig tab lets you connect later |
| `--replay FILE` | Play a recording (`.log` or `.log.gz`, `candump -L` format; a `.side.jsonl` beside it is played too) instead of a live bus |
| `--speed X` | Replay speed (default 1) |
| `--host ADDR`, `--port N` | Where to listen (default `127.0.0.1:8765`). `--host 0.0.0.0` puts the console, and its commands, on the network: the session token is then the only protection, and the console says so |
| `--token T` | A session token of your own (default: random, printed in the URL) |
| `--pose-port N` | The UDP port the 3D viewer's poses arrive on (default 45680; the rig the console starts is told it; give an externally started `tfc_simd --viewer` the same) |
| `--viewer FILE` | Open the 3D viewer on a pose file (`tfc_fly VEHICLE --pose FILE`, `.pose.jsonl` or `.gz`): no bus and no rig needed; the URL printed is the viewer's |
| `--truth-port N` | The UDP port the simulator's telemetry arrives on (default 45679; the rig the console starts is told it; give an externally started `tfc_simd --telemetry` the same) |
| `--no-open` | Do not open the page in the browser |
| `--no-rig` | Do not offer to start and stop the rig's processes: watch and command only |
| `--repo DIR` | The repository the binaries, vehicle files and logs are in (default: this one) |

`console/tfc-console` works from any directory (it puts `console/` and `sim/` on the path); `python3 -m tfc_console` works from inside `console/`.

## The tabs
| Tab | Key | What it is for |
|---|---|---|
| Mission | 1 | The whole system at a glance: three computers, ACT, sync, alerts, the vehicle, the latest events |
| Launch | 2 | Countdown, GO / NO-GO, the sequence, the checklist, the launch and scrub buttons |
| Voting | 3 | How the agreement works: ACT's vote, the node life cycle, each channel's deviation from the median against the tolerance, the nodes' views of each other |
| Flight | 4 | Trajectory against the nominal flight, the vehicle and its stages, the numbers and eight charts |
| Commands | 5 | Every ground command, with its ARM rule; what to do now; the log with the nodes' answers |
| Faults | 6 | Any of the 32 fault kinds on the virtual B and C while they run, with the detection **measured**; kill, freeze, restart a node; the injector's relays |
| Vehicle | 7 | Choose a vehicle, change its parameters, validate it with the simulator's own reader, fly a preview, give it to the rig |
| Rig | 8 | The data source and the replay, the rig's processes, the serial hardware |
| Bus | 9 | Every id with its rate and failures, and a live decoded monitor |
| Events | 0 | Every event, filterable and searchable, and the raw console of each node |

The **REC** button in the header records the bus (and the nodes' consoles and the simulator's telemetry) to `logs/`; **the theme button** switches dark and light.

## Troubleshooting
* **"no CAN interface found"** or *cannot open CAN interface 'vcan0'*: run `sim/scripts/setup_vcan.sh` (needs sudo; it does not survive a reboot).
* **A profile says "not built"**: the Rig tab names the missing binary. The launch images are `tools/bench/sil_triplex.sh --build --launch` (needs Zephyr: `firmware/README.md`); `tfc_simd` and `tfc_fly` are `cmake --build build/host`.
* **The page says "No session token"**: open the address the console printed (it ends in `?token=...`). The console keeps the token for the tab; a new tab needs the full address.
* **Commands say "no answer"**: no node console is attached, so the console cannot see the answer. Start the rig from the Rig tab (its processes' consoles are attached automatically), or attach the Nucleos' serial consoles there.
* **A node latched out on its own**: the virtual rig is five real-time processes with 10 ms deadlines; a compile, a browser start-up or a video call can cost a frame. The Voting tab shows it and the Events tab has the reasons. See `docs/design/CONSOLE.md` section 8.
* **Commands are refused for a counter**: the console shares `~/.cache/tfc_peers/ground_counter` with `tfc_peers command`. A flight computer accepts a counter 1 to 32 ahead of the last it accepted; after restarting the rig the first command is accepted whatever it is.

## Layout
```
console/
  tfc-console              launcher
  tfc_console/             the server (Python standard library)
    model.py               frames -> state, events, the vote monitor, the go/no-go
    lines.py               a node's console lines -> events and counters
    hub.py                 the one place sources deliver to and pages read from
    sources.py             the bus, a recorded log, the simulator's telemetry
    recorder.py            a session as the bus plus a sidecar
    commands.py            the operator's signed ground frames
    rig.py                 start, watch and break the rig's processes
    faultlab.py            faults through tfc_peers run --control
    vehicles.py            vehicle files, knobs, validate, preview
    hardware.py            serial: the consoles, the supervisor, the Pico
    viewer.py              the 3D viewer's data: poses and spec, the pose-file player, the models
    server.py, app.py, services.py, cli.py
  web/                     the page: index.html, style.css, js/ (ES modules), js/tabs/ (one per tab)
  web/viewer/              the 3D viewer: index.html, js/ (world, sky, models, effects, six lenses), vendor/ (three.js, MIT)
  models/                  your own glTF models for the viewer (not committed)
  demo/                    one recorded run (a launch and a node loss) and one flight of the Starship-class vehicle for the viewer, gzip
```
Tests are `sim/tests/test_console_*.py` (the viewer's: `test_console_viewer.py`), `test_peers_control.py` and `test_live_console.py` (they run with the rest: `cd sim && python3 -m unittest discover -s tests -t .`). A smoke test that clicks the real buttons in a headless Firefox against a live rig: open the page with `?selftest=base` (any rig) or `?selftest=faults` (the fault lab running).
