"""The node's line protocol (firmware/gello_node/gello_node.ino) and the raw-count -> joint-angle mapping."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

COUNTS_PER_TURN = 4096
MISSING = 65535
JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6", "trigger")


@dataclass
class Sample:
    seq: int
    t_ms: int
    counts: np.ndarray        # 7 raw positions, NaN where the servo did not answer
    ok: int


def parse_line(line: str) -> Sample | None:
    """'G,seq,t,p1..p7,ok' -> Sample; anything else -> None."""
    parts = line.strip().split(",")
    if len(parts) != 11 or parts[0] != "G":
        return None
    try:
        seq, t = int(parts[1]), int(parts[2])
        raw = [int(p) for p in parts[3:10]]
        ok = int(parts[10])
    except ValueError:
        return None
    counts = np.array([float("nan") if r == MISSING else float(r) for r in raw])
    return Sample(seq, t, counts, ok)


@dataclass
class Calibration:
    """q_arm[i] = sign[i] * unwrap(counts[i] - zero[i]) * 360 / 4096 + offset[i], in degrees, the arm's conventions.

    zero: the counts read with the leader in its calibration pose (hanging straight, wrist centred, trigger released).
    offset: the arm's joint angles in that pose (all 0 except what the desk stop forces on J2).
    sign: +1/-1 so the leader and the arm turn the same way (found once by hand; see docs/design-2026-09-05.md).
    """
    zero: list[float] = field(default_factory=lambda: [2048.0] * 7)
    sign: list[int] = field(default_factory=lambda: [1] * 7)
    offset: list[float] = field(default_factory=lambda: [0.0] * 7)
    limits: dict[str, tuple[float, float]] = field(default_factory=lambda: {
        "j1": (-160, 160), "j2": (-8, 178), "j3": (-160, 160), "j4": (-180, 180), "j5": (-100, 100), "j6": (-180, 180),
        "trigger": (0, 40)})

    def to_degrees(self, counts: np.ndarray) -> np.ndarray:
        d = counts - np.array(self.zero)
        d = (d + COUNTS_PER_TURN / 2) % COUNTS_PER_TURN - COUNTS_PER_TURN / 2       # unwrap into +/- half a turn
        q = np.array(self.sign) * d * (360.0 / COUNTS_PER_TURN) + np.array(self.offset)
        lo = np.array([self.limits[j][0] for j in JOINTS]); hi = np.array([self.limits[j][1] for j in JOINTS])
        return np.clip(q, lo, hi)

    def save(self, path: Path) -> None:
        path.write_text(json.dumps({"zero": self.zero, "sign": self.sign, "offset": self.offset, "limits": self.limits}, indent=1))

    @classmethod
    def load(cls, path: Path) -> "Calibration":
        d = json.loads(path.read_text())
        return cls(zero=d["zero"], sign=d["sign"], offset=d["offset"], limits={k: tuple(v) for k, v in d["limits"].items()})


def trigger_to_grip(deg: float, travel: float = 40.0) -> float:
    """0 (released, open) .. 1 (pulled, closed)."""
    return float(np.clip(deg / travel, 0.0, 1.0))
