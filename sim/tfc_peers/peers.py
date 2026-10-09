# SPDX-License-Identifier: MIT
"""Virtual flight computers: generate the bus traffic a real FC-A/B/C would send.

Each 10 ms major frame a healthy node sends (offsets follow docs/design/ARCHITECTURE.md section 3):
  gyro  (0x100+n)  ~1.5 ms      accel (0x110+n)  ~2.3 ms      cmd (0x200+n)  ~5.0 ms
Everything is a pure function of (seed, frame index), so runs are exactly repeatable and the
same scenario can be recorded offline or played in real time.

Fault application order inside one frame (docs/verification/FMEA.md): sensor sample (stuck, repeat, bias, drift, spike, scale,
invert, swap, clip, oscillate, noise, saturate, zero) -> quantised counts (bitflip, stuckbit) -> command (stuck, offset,
invert) -> digest -> frame level (dropout, reboot, sequence, partial) -> corruption on the wire -> timing (late, early,
jitter, clock drift) -> duplicates and replays. The kinds added after the first release draw from their own random
stream, so adding them never changed the traffic of the older kinds.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import protocol as P
from .commands import COMMAND_SEND_US, GroundCommand
from .faults import Fault

FRAME_US = 10_000  # 100 Hz major frame
GYRO_NOISE_DPS = 0.1  # 1-sigma; ISM330DHCX-class noise, about one LSB
ACCEL_NOISE_G = 0.002
BABBLE_ID_BASE = 0x020  # out of schedule, and higher priority than every sensor id
HISTORY_FRAMES = 300  # how far back `replay` can reach

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


def _to_i16(u: int) -> int:
    u &= 0xFFFF
    return u - 0x10000 if u & 0x8000 else u


class VirtualNode:
    """One virtual flight computer. Call `step(k)` for k = 0, 1, 2, ... in order."""

    def __init__(self, node: int, faults: list[Fault], seed: int) -> None:
        self.node = node
        self.faults = [f for f in faults if f.node == node]
        self._noise = random.Random(seed * 1009 + node)   # sensor noise
        self._fault_rng = random.Random(seed * 7919 + node)  # spike / corrupt / babble choices (first-release kinds)
        self._rng2 = random.Random(seed * 104729 + node)  # every kind added later: keeps the older streams unchanged
        self._prev_clean: tuple[Vec, Vec] | None = None
        self._frozen: tuple[Vec, Vec] | None = None
        self._held: tuple[Vec, Vec] | None = None          # repeat
        self._cmd_prev: tuple[float, float] | None = None
        self._cmd_frozen: tuple[float, float] | None = None  # cmdstuck
        self._seq_frozen: int | None = None                 # seqstuck
        self._history: dict[int, list[TimedFrame]] = {}     # replay

    def set_faults(self, faults: list[Fault]) -> None:
        """Replace this node's faults between frames (the list is swapped whole, never edited in place, so a frame in progress sees one or the other)."""
        self.faults = [f for f in faults if f.node == self.node]

    def _active(self, kind: str, k: int) -> list[Fault]:
        return [f for f in self.faults if f.kind == kind and f.active(k)]

    # ------------------------------------------------------------------ sample-level faults
    def _sensor_faults(self, k: int, gyro: Vec, accel: Vec) -> tuple[list[float], list[float]]:
        clean: tuple[Vec, Vec] = (gyro, accel)
        if self._active("stuck", k):
            if self._frozen is None:
                self._frozen = self._prev_clean or clean
            gyro, accel = self._frozen
        else:
            self._frozen = None
        self._prev_clean = clean

        rep = self._active("repeat", k)
        if rep:
            n = int(rep[0].params["n"])
            if self._held is None or (k - rep[0].start) % n == 0:
                self._held = (gyro, accel)
            else:
                gyro, accel = self._held
        else:
            self._held = None

        gyro_l, accel_l = list(gyro), list(accel)
        sensors = {"gyro": gyro_l, "accel": accel_l}
        sigma = {"gyro": GYRO_NOISE_DPS, "accel": ACCEL_NOISE_G}
        t = k * FRAME_US / 1e6 + 0.0005
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
        for f in self._active("scale", k):
            sensors[str(f.params["sensor"])][int(f.params["axis"])] *= float(f.params["factor"])
        for f in self._active("invert", k):
            ax = int(f.params["axis"])
            sensors[str(f.params["sensor"])][ax] = -sensors[str(f.params["sensor"])][ax]
        for f in self._active("swap", k):
            s = sensors[str(f.params["sensor"])]
            a, b = int(f.params["axis"]), int(f.params["other"])
            s[a], s[b] = s[b], s[a]
        for f in self._active("clip", k):
            lim = float(f.params["limit"])
            sensors[str(f.params["sensor"])][:] = [_clamp(v, lim) for v in sensors[str(f.params["sensor"])]]
        for f in self._active("oscillate", k):
            sensors[str(f.params["sensor"])][int(f.params["axis"])] += (
                float(f.params["amp"]) * math.sin(2.0 * math.pi * float(f.params["hz"]) * t))
        for f in self._active("noise", k):
            name = str(f.params["sensor"])
            extra = (float(f.params["mult"]) - 1.0) * sigma[name]
            sensors[name][:] = [v + self._rng2.gauss(0.0, extra) for v in sensors[name]]
        if self._active("saturate", k):
            sign = 1.0 if k % 2 == 0 else -1.0
            gyro_l, accel_l = [sign * 1e6] * 3, [sign * 1e6] * 3
        if self._active("zero", k):
            gyro_l, accel_l = [0.0] * 3, [0.0] * 3
        return gyro_l, accel_l

    def _apply_bits(self, k: int, vals: list[float], lsb: float, sensor: str) -> tuple[float, float, float]:
        """Faults in the 16-bit sample itself, before the CRC is computed (a single-event upset in a register)."""
        flips = [f for f in self._active("bitflip", k) if str(f.params["sensor"]) == sensor]
        stucks = [f for f in self._active("stuckbit", k) if str(f.params["sensor"]) == sensor]
        if not flips and not stucks:
            return tuple(vals)  # type: ignore[return-value]
        counts = [P.quantize(v, lsb) for v in vals]
        for f in flips:
            if self._rng2.random() < float(f.params["p"]):
                ax = self._rng2.randrange(3)
                counts[ax] = _to_i16((counts[ax] & 0xFFFF) ^ (1 << int(f.params["bit"])))
        for f in stucks:
            b, val = int(f.params["bit"]), int(f.params["value"])
            for ax in range(3):
                u = counts[ax] & 0xFFFF
                counts[ax] = _to_i16((u | (1 << b)) if val else (u & ~(1 << b)))
        return tuple(c * lsb for c in counts)  # type: ignore[return-value]

    # ------------------------------------------------------------------ one frame
    def step(self, k: int) -> list[TimedFrame]:
        t0 = k * FRAME_US
        g_true, a_true = truth(k * FRAME_US / 1e6 + 0.0005)  # IMU latched at 0.5 ms
        gyro = tuple(v + self._noise.gauss(0.0, GYRO_NOISE_DPS) for v in g_true)
        accel = tuple(v + self._noise.gauss(0.0, ACCEL_NOISE_G) for v in a_true)

        # Commands come from the (noise-free) truth so healthy replicas are bit-identical.
        pitch = _clamp(0.1 * g_true[0], 30.0)
        yaw = _clamp(0.1 * g_true[1], 30.0)
        digest = (P.quantize(pitch, P.CMD_LSB_DEG) * 31 + P.quantize(yaw, P.CMD_LSB_DEG) * 17
                  + k * 40503) & 0xFFFF

        gyro_l, accel_l = self._sensor_faults(k, gyro, accel)  # type: ignore[arg-type]
        clean_cmd = (pitch, yaw)
        if self._active("cmdstuck", k):
            if self._cmd_frozen is None:
                self._cmd_frozen = self._cmd_prev or (pitch, yaw)
            pitch, yaw = self._cmd_frozen
        else:
            self._cmd_frozen = None
        self._cmd_prev = clean_cmd
        for f in self._active("cmd_offset", k):
            pitch += float(f.params["mag"])
        if self._active("cmdinvert", k):
            pitch, yaw = -pitch, -yaw
        for f in self._active("digest", k):
            digest ^= int(f.params["xor"]) & 0xFFFF

        # ---- frame-level faults ----
        if self._active("dropout", k):
            return self._babble(k, t0)  # drops the scheduled frames; babble is an independent fault
        resume = 0
        for f in self.faults:  # reboot: silent for `down` frames, then back (in phase, unless it does not resync)
            if f.kind != "reboot":
                continue
            back = f.start + int(f.params["down"])
            if f.start <= k < back:
                return self._babble(k, t0)
            if k >= back and int(f.params.get("resync", 1)) == 0:
                resume = max(resume, back)  # the frame number restarts at 0 instead of following SYNC
        offset = sum(int(f.params["gap"]) for f in self._active("seqgap", k))  # frames are numbered from SYNC: a wrong number is a wrong frame
        seq = ((k - resume) if resume else (k + offset)) & 0xFF
        if self._active("seqstuck", k):
            if self._seq_frozen is None:
                self._seq_frozen = seq
            seq = self._seq_frozen
        else:
            self._seq_frozen = None

        n = self.node
        frames = [
            TimedFrame(t0 + 1500 + 200 * n, P.pack_gyro(n, self._apply_bits(k, gyro_l, P.GYRO_LSB_DPS, "gyro"), seq)),
            TimedFrame(t0 + 2300 + 200 * n, P.pack_accel(n, self._apply_bits(k, accel_l, P.ACCEL_LSB_G, "accel"), seq)),
            TimedFrame(t0 + 5000 + 300 * n, P.pack_cmd(n, pitch, yaw, digest, seq)),
        ]
        self._history[k] = list(frames)
        self._history.pop(k - HISTORY_FRAMES - 1, None)  # keeps frames k-300 .. k
        for f in self._active("partial", k):  # only some frame types are sent
            mask = int(f.params["mask"])
            frames = [tf for tf in frames if not ((mask & 1 and tf.frame.id == P.ID_GYRO_BASE + n) or
                                                   (mask & 2 and tf.frame.id == P.ID_ACCEL_BASE + n) or
                                                   (mask & 4 and tf.frame.id == P.ID_CMD_BASE + n))]
        for f in self._active("replay", k):  # stale frames re-sent: old data, old sequence number, valid CRC
            old = self._history.get(k - int(f.params["age"]))
            if old:
                slot = {tf.frame.id: tf.t_us for tf in frames}
                frames = [TimedFrame(slot[of.frame.id], of.frame) for of in old if of.frame.id in slot]
        for f in self._active("corrupt", k):
            frames = [self._corrupt(tf, float(f.params["p"])) for tf in frames]
        shift = sum(int(f.params["us"]) for f in self._active("late", k))
        shift -= sum(int(f.params["us"]) for f in self._active("early", k))
        for f in self._active("clockdrift", k):
            shift += int(f.params["us_per_frame"]) * (k - f.start + 1)
        jit = [int(f.params["us"]) for f in self._active("jitter", k)]
        if shift or jit:  # timing faults: every scheduled frame leaves at a shifted time (never before time 0)
            frames = [TimedFrame(max(0, tf.t_us + shift + sum(self._rng2.randint(-u, u) for u in jit)), tf.frame)
                      for tf in frames]
        for f in self._active("duplicate", k):  # every scheduled frame sent twice
            frames = frames + [TimedFrame(tf.t_us + int(f.params["gap_us"]), tf.frame) for tf in frames]
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
                                      P.Frame(int(f.params["id"]) + (i % 16), payload)))
        return out


class Scenario:
    """A set of virtual nodes plus their faults. `frames(k)` returns one major frame of traffic.

    Frame numbers are absolute (they are the SYNC frame numbers when following a live flight
    computer). Requesting frame k after frame j < k fast-forwards through j+1..k-1 (so noise and
    fault state stay exactly what a run from frame 0 would give); going backwards needs `reset()`.
    """

    def __init__(self, nodes: list[int], faults: list[Fault] | None = None, seed: int = 1,
                 commands: list[GroundCommand] | None = None) -> None:
        self.nodes = sorted(set(nodes))
        self.faults = list(faults or [])
        self._fault_ids = {i + 1: f for i, f in enumerate(self.faults)}  # the faults it started with are 1, 2, 3...; add_fault goes on from there
        self._next_fault_id = len(self.faults) + 1
        self.seed = seed
        self.commands = list(commands or [])
        self._ground = self._build_ground_frames()
        self.reset()

    def _build_ground_frames(self) -> dict[int, list[P.Frame]]:
        """Counters 1, 2, 3... in time order (the ground station's strictly increasing command counter)."""
        out: dict[int, list[P.Frame]] = {}
        counter = 0
        last: P.Frame | None = None
        for _i, c in sorted(enumerate(self.commands), key=lambda t: (t[1].frame, t[0])):
            if c.replay:
                frame = last if last is not None else c.frame_for(0)  # the previous frame again, unchanged: a stale counter
            else:
                counter += 1
                frame = c.frame_for(counter)
                last = frame
            out.setdefault(c.frame, []).append(frame)
        return out

    def reset(self) -> None:
        self._virtual = {n: VirtualNode(n, self.faults, self.seed) for n in self.nodes}
        self._next_k = 0

    @property
    def next_frame(self) -> int:
        return self._next_k

    # ---- faults changed while the scenario runs (`tfc_peers run --control`; the flight console) ----
    def fault_table(self) -> list[tuple[int, Fault]]:
        """Every fault now in the scenario with the number that names it (`clear_fault`)."""
        return sorted(self._fault_ids.items())

    def _apply_faults(self) -> None:
        self.faults = [f for _i, f in sorted(self._fault_ids.items())]
        for node in self._virtual.values():
            node.set_faults(self.faults)

    def add_fault(self, fault: Fault) -> int:
        """Add a fault from the next frame on; returns its number. The node must be one the scenario simulates."""
        if fault.node not in self.nodes:
            raise ValueError(f"fault {fault} targets a node that is not simulated ({','.join(P.NODE_NAMES[n] for n in self.nodes)})")
        number = self._next_fault_id
        self._next_fault_id += 1
        self._fault_ids[number] = fault
        self._apply_faults()
        return number

    def clear_fault(self, number: int) -> bool:
        """Remove one fault by its number; False if there is none. What it already did to the traffic stays done (a node it latched out stays out until the operator readmits it)."""
        if self._fault_ids.pop(number, None) is None:
            return False
        self._apply_faults()
        return True

    def clear_faults(self) -> int:
        n = len(self._fault_ids)
        self._fault_ids.clear()
        self._apply_faults()
        return n

    def _generate(self, k: int) -> list[TimedFrame]:
        out = [tf for n in self.nodes for tf in self._virtual[n].step(k)]
        for i, g in enumerate(self._ground.get(k, [])):
            out.append(TimedFrame(k * FRAME_US + COMMAND_SEND_US + 50 * i, g))
        self._next_k = k + 1
        return sorted(out, key=lambda tf: tf.t_us)

    def frames(self, k: int) -> list[TimedFrame]:
        if k < self._next_k:
            raise ValueError(f"frame {k} was already generated (next is {self._next_k}); call reset() to rewind")
        while self._next_k < k:
            self._generate(self._next_k)  # fast-forward, discarding
        return self._generate(k)
