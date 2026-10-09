# SPDX-License-Identifier: MIT
"""What the console's tests share: the path to the package, the committed real recording, and builders of the frames a test needs."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "console"))

from tfc_peers import protocol as P  # noqa: E402

DEMO = ROOT / "console" / "demo" / "launch-and-node-loss.log.gz"


def hb(node, mode=3, ready=True, view=(0, 0, 0), role=0, safe=False, alarm=False, quarantined=False, resets=1, release=0xCFE1, seq=0):
    return P.pack_heartbeat(node, P.Heartbeat(mode=mode, ready=ready, node_state=tuple(view), role=role, safe_requested=safe, bus_alarm=alarm, quarantined=quarantined, reset_count=resets, release_hash=release), seq)


def act(state=1, vote_status=0, voted=7, excluded=0, cause=0, pitch=0.0, yaw=0.0, held=False, seq=0):
    return P.pack_act_out(P.ActOut(pitch, yaw, state, held, vote_status, voted, excluded, cause), seq)


def feed_frame(tm, t, seq, gyro=((0.0, 0.0, 0.0),) * 3, accel=((0.0, 0.0, 1.0),) * 3, cmd=((0.0, 0.0),) * 3, nodes=(0, 1, 2), digest=0x1234):
    """One 10 ms frame of the three nodes' samples into a model, in the bus order (gyro, accel, command)."""
    for n in nodes:
        tm.feed(t + 0.0015 + 0.0002 * n, P.pack_gyro(n, gyro[n], seq))
    for n in nodes:
        tm.feed(t + 0.0023 + 0.0002 * n, P.pack_accel(n, accel[n], seq))
    for n in nodes:
        tm.feed(t + 0.0050 + 0.0003 * n, P.pack_cmd(n, cmd[n][0], cmd[n][1], digest, seq))
