# SPDX-License-Identifier: MIT
"""Virtual flight computers: generate the bus traffic a real FC-A/B/C would send.

Each 10 ms major frame a healthy node sends (offsets follow docs/ARCHITECTURE.md section 3):
  gyro  (0x100+n)  ~1.5 ms      accel (0x110+n)  ~2.3 ms      cmd (0x200+n)  ~5.0 ms
Everything is a pure function of (seed, frame index), so runs are exactly repeatable and the
same scenario can be recorded offline or played in real time.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import protocol as P
from .faults import Fault

FRAME_US = 10_000  # 100 Hz major frame
GYRO_NOISE_DPS = 0.1  # 1-sigma; ISM330DHCX-class noise, about one LSB
ACCEL_NOISE_G = 0.002
BABBLE_ID_BASE = 0x020  # out of schedule, and higher priority than every sensor id

Vec = tuple[float, float, float]


@dataclass(frozen=True)
class TimedFrame:
    t_us: int  # absolute time since scenario start
    frame: P.Frame


def truth(t: float) -> tuple[Vec, Vec]:
    """The motion every healthy IMU feels: slow sinusoidal rates, gravity on Z."""
    tau = 2.0 * math.pi
    gyro = (10.0 * math.sin(tau * 0.8 * t), 6.0 * math.cos(tau * 0.5 * t),
            3.0 * math.sin(tau * 0.3 * t))
    accel = (0.05 * math.sin(tau * 0.4 * t), 0.03 * math.cos(tau * 0.6 * t), 1.0)
    return gyro, accel


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


class VirtualNode:
    """One virtual flight computer. Call `step(k)` for k = 0, 1, 2, ... in order."""

    def __init__(self, node: int, faults: list[Fault], seed: int) -> None:
        self.node = node
        self.faults = [f for f in faults if f.node == node]
        self._noise = random.Random(seed * 1009 + node)   # sensor noise
        self._fault_rng = random.Random(seed * 7919 + node)  # spike / corrupt / babble choices
        self._prev_clean: tuple[Vec, Vec] | None = None
        self._frozen: tuple[Vec, Vec] | None = None
        self._seq_offset = 0

    def _active(self, kind: str, k: int) -> list[Fault]:
        return [f for f in self.faults if f.kind == kind and f.active(k)]

    def step(self, k: int) -> list[TimedFrame]:
        t0 = k * FRAME_US
        g_true, a_true = truth(k * FRAME_US / 1e6 + 0.0005)  # IMU latched at 0.5 ms
        gyro = tuple(v + self._noise.gauss(0.0, GYRO_NOISE_DPS) for v in g_true)
        accel = tuple(v + self._noise.gauss(0.0, ACCEL_NOISE_G) for v in a_true)
        clean: tuple[Vec, Vec] = (gyro, accel)  # type: ignore[assignment]

        # Commands come from the (noise-free) truth so healthy replicas are bit-identical.
        pitch = _clamp(0.1 * g_true[0], 30.0)
        yaw = _clamp(0.1 * g_true[1], 30.0)
        digest = (P.quantize(pitch, P.CMD_LSB_DEG) * 31 + P.quantize(yaw, P.CMD_LSB_DEG) * 17
                  + k * 40503) & 0xFFFF

        # ---- sample-level faults ----
        if self._active("stuck", k):
            if self._frozen is None:
                self._frozen = self._prev_clean or clean
            gyro, accel = self._frozen
        else:
            self._frozen = None
        self._prev_clean = clean

        gyro_l, accel_l = list(gyro), list(accel)
        sensors = {"gyro": gyro_l, "accel": accel_l}
        for f in self._active("bias", k):
            sensors[str(f.params["sensor"])][int(f.params["axis"])] += float(f.params["mag"])
        for f in self._active("drift", k):
            sensors[str(f.params["sensor"])][int(f.params["axis"])] += (
                float(f.params["rate"]) * (k - f.start + 1))
        for f in self._active("spike", k):
            if self._fault_rng.random() < float(f.params["p"]):
                axis = self._fault_rng.randrange(3)
                sign = 1.0 if self._fault_rng.random() < 0.5 else -1.0
                sensors[str(f.params["sensor"])][axis] += sign * float(f.params["mag"])
        if self._active("saturate", k):
            sign = 1.0 if k % 2 == 0 else -1.0
            gyro_l, accel_l = [sign * 1e6] * 3, [sign * 1e6] * 3
        for f in self._active("cmd_offset", k):
            pitch += float(f.params["mag"])
        for f in self._active("digest", k):
            digest ^= int(f.params["xor"]) & 0xFFFF

        # ---- frame-level faults ----
        if self._active("dropout", k):
            return self._babble(k, t0)  # drops the scheduled frames; babble is an independent fault
        for f in self._active("seqgap", k):
            if k == f.start:
                self._seq_offset += int(f.params["gap"])
        seq = (k + self._seq_offset) & 0xFF

        n = self.node
        frames = [
            TimedFrame(t0 + 1500 + 200 * n, P.pack_gyro(n, tuple(gyro_l), seq)),  # type: ignore[arg-type]
            TimedFrame(t0 + 2300 + 200 * n, P.pack_accel(n, tuple(accel_l), seq)),  # type: ignore[arg-type]
            TimedFrame(t0 + 5000 + 300 * n, P.pack_cmd(n, pitch, yaw, digest, seq)),
        ]
        for f in self._active("corrupt", k):
            frames = [self._corrupt(tf, float(f.params["p"])) for tf in frames]
        return sorted(frames + self._babble(k, t0), key=lambda tf: tf.t_us)

    def _corrupt(self, tf: TimedFrame, p: float) -> TimedFrame:
        if self._fault_rng.random() >= p:
            return tf
        bit = self._fault_rng.randrange(64)
        data = bytearray(tf.frame.data)
        data[bit // 8] ^= 1 << (bit % 8)
        return TimedFrame(tf.t_us, P.Frame(tf.frame.id, bytes(data)))

    def _babble(self, k: int, t0: int) -> list[TimedFrame]:
        out: list[TimedFrame] = []
        for f in self._active("babble", k):
            for i in range(int(f.params["n"])):
                payload = P.seal(bytes(self._fault_rng.randrange(256) for _ in range(6)), k)
                out.append(TimedFrame(t0 + 8000 + 50 * i,
                                      P.Frame(BABBLE_ID_BASE + (i % 16), payload)))
        return out


class Scenario:
    """A set of virtual nodes plus their faults. `frames(k)` returns one major frame of traffic.

    Frame numbers are absolute (they are the SYNC frame numbers when following a live flight
    computer). Requesting frame k after frame j < k fast-forwards through j+1..k-1 (so noise and
    fault state stay exactly what a run from frame 0 would give); going backwards needs `reset()`.
    """

    def __init__(self, nodes: list[int], faults: list[Fault] | None = None, seed: int = 1) -> None:
        self.nodes = sorted(set(nodes))
        self.faults = list(faults or [])
        self.seed = seed
        self.reset()

    def reset(self) -> None:
        self._virtual = {n: VirtualNode(n, self.faults, self.seed) for n in self.nodes}
        self._next_k = 0

    @property
    def next_frame(self) -> int:
        return self._next_k

    def _generate(self, k: int) -> list[TimedFrame]:
        out = [tf for n in self.nodes for tf in self._virtual[n].step(k)]
        self._next_k = k + 1
        return sorted(out, key=lambda tf: tf.t_us)

    def frames(self, k: int) -> list[TimedFrame]:
        if k < self._next_k:
            raise ValueError(f"frame {k} was already generated (next is {self._next_k}); call reset() to rewind")
        while self._next_k < k:
            self._generate(self._next_k)  # fast-forward, discarding
        return self._generate(k)
