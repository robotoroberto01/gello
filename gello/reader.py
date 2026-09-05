"""Read the leader arm over USB serial.

    uv run python -m gello.reader                 # print joint angles at 10 Hz (needs a calibration.json, or shows raw)
    uv run python -m gello.reader --port /dev/tty.usbmodem1101 --calibrate   # capture the zero pose into calibration.json
"""
from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path

import numpy as np

from gello.protocol import Calibration, JOINTS, Sample, parse_line

CAL = Path(__file__).resolve().parents[1] / "calibration.json"


def find_port() -> str:
    ports = glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*")
    if not ports:
        sys.exit("no ESP32-S3 on USB (looked for /dev/tty.usbmodem* and /dev/ttyACM*)")
    return ports[0]


class Leader:
    def __init__(self, port: str | None = None, cal: Calibration | None = None, baud: int = 921600):
        import serial
        self.ser = serial.Serial(port or find_port(), baud, timeout=0.05)
        self.cal = cal
        self.last: Sample | None = None

    def read(self) -> Sample | None:
        """The newest complete sample, draining the buffer; None if nothing new."""
        newest = None
        while self.ser.in_waiting:
            s = parse_line(self.ser.readline().decode(errors="replace"))
            if s:
                newest = s
        if newest:
            self.last = newest
        return newest

    def q(self) -> np.ndarray | None:
        s = self.read() or self.last
        if s is None or self.cal is None:
            return None
        return self.cal.to_degrees(s.counts)

    def hold(self, on: bool) -> None:
        if on:
            raise RuntimeError("Parking torque is unavailable: this leader is a passive encoder")
        self.ser.write(b"t")  # the passive node interprets this as an explicit torque-off request


def calibrate(port: str | None) -> Calibration:
    L = Leader(port)
    print("put the leader in its zero pose (hanging straight, wrist centred, trigger released) and hold it; 3 s")
    t0 = time.time(); samples = []
    while time.time() - t0 < 3.0:
        s = L.read()
        if s is not None and s.ok == 0x7F:
            samples.append(s.counts)
        time.sleep(0.01)
    if len(samples) < 20:
        sys.exit(f"only {len(samples)} complete samples; check the bus (ok mask must be 127)")
    cal = Calibration(zero=[float(v) for v in np.median(np.array(samples), axis=0)])
    cal.save(CAL)
    print(f"zero = {[round(z) for z in cal.zero]} -> {CAL}; set sign[] by hand for any joint that turns the wrong way")
    return cal


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--calibrate", action="store_true")
    a = ap.parse_args()
    if a.calibrate:
        calibrate(a.port); return
    cal = Calibration.load(CAL) if CAL.exists() else None
    L = Leader(a.port, cal)
    print("raw counts" if cal is None else "degrees (the arm's joints)")
    while True:
        s = L.read()
        if s:
            v = s.counts if cal is None else cal.to_degrees(s.counts)
            print(" ".join(f"{n}={x:7.1f}" for n, x in zip(JOINTS, v)), f"ok={s.ok:07b}", end="\r")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
