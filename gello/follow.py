"""Make the RobotDoctor arm follow the leader.

    uv run python -m gello.follow --sim            # against the arm's MuJoCo simulator (RobotDoctor's robot/control)
    uv run python -m gello.follow --dry            # print the targets only

The leader's joint angles become the arm's joint targets through a rate limiter (deg/s per joint) and the arm's
own supervisor; the trigger drives the gripper's close fraction over the hand's CAN node.  Nothing here talks to a
puck directly: it hands targets to RobotDoctor's runner, which owns the PD, the gravity term and the safety gate.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from gello.protocol import Calibration, JOINTS, trigger_to_grip
from gello.reader import CAL, Leader

ROBOTDOCTOR = Path.home() / "Desktop" / "RobotDoctor"
RATE_DEG_S = np.array([60.0, 45.0, 60.0, 120.0, 90.0, 120.0])      # the arm's joint speed caps while following
LOOP_HZ = 50.0


def limited(target: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    step = RATE_DEG_S * dt
    return current + np.clip(target - current, -step, step)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("--sim", action="store_true"); ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    cal = Calibration.load(CAL)
    L = Leader(a.port, cal)
    q_cmd = None
    arm = None
    if a.sim:
        sys.path.insert(0, str(ROBOTDOCTOR))
        from robot.control.kinematics import Arm            # noqa: E402  (the arm's simulator + kinematics)
        arm = Arm()
    dt = 1 / LOOP_HZ
    while True:
        t0 = time.time()
        q = L.q()
        if q is not None:
            target = q[:6]
            grip = trigger_to_grip(q[6])
            q_cmd = target.copy() if q_cmd is None else limited(target, q_cmd, dt)
            if a.dry or arm is None:
                print(" ".join(f"{n}={v:6.1f}" for n, v in zip(JOINTS, q_cmd)), f"grip={grip:.2f}", end="\r")
            else:
                pos, _ = arm.fk(np.radians(q_cmd))
                print(" ".join(f"{n}={v:6.1f}" for n, v in zip(JOINTS, q_cmd)), f"tip=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) grip={grip:.2f}", end="\r")
        time.sleep(max(0.0, dt - (time.time() - t0)))


if __name__ == "__main__":
    main()
