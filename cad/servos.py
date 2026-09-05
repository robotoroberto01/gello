"""The small bus servos the leader arm is built from: ROBOTIS DYNAMIXEL X330 family (XL330-M288-T planned, not ordered; seven needed for
this; the XC330-M288-T ordered for the arm's five-finger hand is the same case).

Case frame: the horn axis is +Z at the origin, the horn face at z = 0, the body extends -Z and along -Y (the
long side).  Numbers from the ROBOTIS e-manual drawing of the XL330-M288-T; VERIFY every one with calipers on the
first servo in hand (the XC330s arrive first).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Servo:
    key: str
    name: str
    w: float            # across the horn face, X
    l: float            # along the body, Y (horn end to the far end)
    d: float            # depth, Z (horn face to the back)
    horn_off: float     # horn axis from the horn-end edge, along Y
    horn_d: float       # horn disc diameter
    horn_h: float       # horn standing proud of the case face
    horn_pcd: float     # the horn's 4 x M2 on this pitch circle
    idler: bool         # a free idler bearing on the back face, coaxial with the horn
    side_holes: tuple[tuple[float, float], ...]   # (y, z) of the M2 case holes on each side face, from the horn axis / horn face
    mass_g: float
    stall_nm: float
    volts: str
    price_usd: float


XL330_M288 = Servo(
    key="xl330_m288", name="DYNAMIXEL XL330-M288-T",
    w=20.0, l=34.0, d=26.0, horn_off=8.0, horn_d=13.0, horn_h=2.5, horn_pcd=8.0, idler=True,
    side_holes=((-4.0, -8.0), (-4.0, -20.0), (-22.0, -8.0), (-22.0, -20.0)),
    mass_g=18.0, stall_nm=0.52, volts="5 V", price_usd=27.49,
)

XC330_M288 = Servo(
    key="xc330_m288", name="DYNAMIXEL XC330-M288-T",
    w=20.0, l=34.0, d=26.0, horn_off=8.0, horn_d=13.0, horn_h=2.5, horn_pcd=8.0, idler=True,
    side_holes=XL330_M288.side_holes,
    mass_g=23.0, stall_nm=0.92, volts="5 V", price_usd=99.90,
)

SERVOS = {s.key: s for s in (XL330_M288, XC330_M288)}
