# Bench tools

Scripts for the day the hardware arrives, all usable now against the virtual bus (`vcan0`). See `docs/procedures/` for where each is used.

| Tool | What it does |
|---|---|
| `check_pc.sh` | Is this PC ready? Read-only checklist (west, SDK, openocd, ST-LINK rule, groups, `vcan0`, `gs_usb`, `can-utils`, pyserial, built binaries); exit status = number missing |
| `can_up.sh [iface] [bitrate]` | Bring up the USB-CAN adapter at 1 Mbit/s with a 100 ms bus-off restart (needs sudo; untested until the adapter arrives) |
| `flash.sh [build_dir]` | Flash a built Nucleo image with openocd |
| `hil_run.sh IFACE SECONDS [run args]` | Record the bus into `logs/` while the virtual peers run, then replay the log through the real core and print what a flight computer would have decided |
| `log_t0.py LOG` | Print the `--t0` and `--first-frame` that align a **live** log with the frame numbers on the bus, for `tfc_replay` |
| `frame_jitter.py EDGES.csv` | Frame timing from logic-analyser edges: jitter about the fitted grid, clock error in ppm, missed frames; pass at p99 of 100 us (SYS-001) |
| `bus_loss.py LOG [--resync-period N] [--console A=FILE ...]` | How much of the schedule arrives and when: per frame id the share missing at the capture point, the delay after SYNC (mean, 99th percentile, maximum), the share after the vote time, and each node's own `missing`/`crc`/`seq` counters from its console; ends with what the figure means for the resync period, the digest persistence and the command tolerance (TS-23). The real bus loss rate, the number those settings are waiting for |
| `check_golden.py LOG [--node A]` | Compare a node's command and digest stream with the PC's golden run, frame by frame |

The log recorder itself is `python3 -m tfc_peers log` (see `sim/README.md`).
