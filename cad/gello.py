"""GELLO-style leader arm: the RobotDoctor arm at half scale, one XL330 per joint plus a trigger, printed links.

    uv run python -m cad.gello                    # exports every printed part to out/ and the assembly at the home pose
    uv run python -m cad.gello --q 0,110,-70,0,20,0,0
    uv run python tools/view.py                   # live in the OCP viewer

Frames.  World: Z up, the base on the desk at z = 0.  Joint frames follow the big arm's: J1 yaw about Z at the base,
J2 and J3 pitch about X, J4 roll about the forearm's axis, J5 pitch about X, J6 roll about the tool axis.  q = 0 is
the arm hanging straight down from J2 (the arm's own convention: J2 is measured from hanging), so on a desk the
leader's usable J2 starts near 40 deg; every other joint is centred.

Each joint is one servo whose BODY is carried by the upstream link and whose HORN drives the downstream link.  Pitch
joints (J2, J3, J5) get a yoke on the driven link that straddles the servo: an ear on the horn and one on the idler,
so the joint is held on both sides.  Roll joints (J4, J6) are a plate on the horn.  Servo frame is cad/servos.py's:
horn axis +Z, horn face at z = 0, the case behind it (-Z) and along -Y.
"""
from __future__ import annotations

import math
import sys

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, Part, Plane, Pos, Vector

from cad import params as P
from cad.lib import OUT, save
from cad.servos import Servo

S = P.SERVO
M2_CLEAR = 2.4
EAR_T = 4.0                         # the ears riding on horn and idler
FIT = 0.4                           # pocket clearance round a servo case
BASE_T = 6.0
Z1 = BASE_T + 8.0 + S.d             # J1 servo horn face above the desk: the case lies flat on an 8 mm cradle floor
W5 = P.ARM["W5_Z"] * P.SCALE        # J4 horn face -> J5 axis (27.5)
J4_BACK = 30.0                      # J4 servo horn face sits this far up the forearm from the wrist centre's frame
# a pitch joint's cradle protrudes 8.4 mm past the axis (horn end) and 12.4 mm sideways; that corner sweeps a 15 mm
# radius as the joint turns, so the driven link's crossbar sits outside it at every angle
YOKE_DROP = 18.5
ARM_TOP = S.horn_pcd / 2 + M2_CLEAR / 2 + 1.5   # the ear arms stop this far below the axis: clear of the horn's 4 x M2
SWEEP_R = math.hypot(YOKE_DROP + 8.0, P.LINK_W / 2)   # the yoke's crossbar corner: nothing of the carrying link inside this radius (29)
Y_POST = -(S.l - S.horn_off + FIT + P.WALL)     # the shoulder's post stands behind the J2 case's back wall (-30.4 < -29)
COLOURS = {"printed": "#5c99a8", "servo": "#2e3438", "handle": "#c9a86a", "desk": "#a99e8b"}


def _cyl(r, h):
    return Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def frame(origin, z_dir, y_dir) -> Location:
    """A Location whose local +Z is z_dir and local +Y is y_dir (both world vectors)."""
    z = Vector(*z_dir).normalized(); y = Vector(*y_dir).normalized()
    x = y.cross(z)
    return Plane(origin=Vector(*origin), x_dir=x, z_dir=z).location


def servo_body(s: Servo = S, grow: float = 0.0) -> Part:
    """The servo in its own frame; grow > 0 makes the pocket that swallows it."""
    g = grow
    case = Pos(-s.w / 2 - g, -(s.l - s.horn_off) - g, -s.d - g) * Box(s.w + 2 * g, s.l + 2 * g, s.d + 2 * g, align=(Align.MIN,) * 3)
    horn = _cyl(s.horn_d / 2 + g, s.horn_h + g)
    idler = Pos(0, 0, -s.d - 1.5 - g) * _cyl(s.horn_d / 2 + g, 1.5 + g) if s.idler else Part()
    return case + horn + idler


def ear(s: Servo = S, on_idler: bool = False) -> Part:
    """A disc on the horn (or the idler) with the horn's 4 x M2 and a centre bore; in the servo frame."""
    r = s.horn_d / 2 + 5.0
    z0 = s.horn_h if not on_idler else -s.d - 1.5 - EAR_T
    e = Pos(0, 0, z0) * _cyl(r, EAR_T)
    e -= Pos(0, 0, z0 - 1) * _cyl(3.0 / 2 if on_idler else 4.0 / 2, EAR_T + 2)        # idler: an M3 pivot screw; horn: the horn's boss
    if not on_idler:
        for k in range(4):
            a = math.radians(45 + 90 * k)
            e -= Pos(s.horn_pcd / 2 * math.cos(a), s.horn_pcd / 2 * math.sin(a), z0 - 1) * _cyl(M2_CLEAR / 2, EAR_T + 2)
    return e


def cradle(s: Servo = S, wall: float = P.WALL, floor: float = 0.0) -> Part:
    """A U that carries the servo's case by its side holes, open on the horn and idler faces.  Servo frame."""
    g = FIT
    outer = Pos(-s.w / 2 - g - wall, -(s.l - s.horn_off) - g - wall - floor, -s.d - g) * \
        Box(s.w + 2 * (g + wall), s.l + 2 * g + wall + floor, s.d + 2 * g, align=(Align.MIN,) * 3)   # floor: extra past the case's far end
    c = outer - servo_body(s, grow=g)
    c -= Pos(-s.w, -(s.l - s.horn_off) + 4.0, -s.d + 3.0) * Box(2 * s.w, s.l - 8.0, s.d - 6.0, align=(Align.MIN,) * 3)   # window in the side walls
    for y, z in s.side_holes:                                                 # the case's M2 side holes, both walls
        c -= Location((0, y, z), (0, 90, 0)) * (Pos(0, 0, -s.w) * _cyl(M2_CLEAR / 2, 2 * s.w))
    return c


# --- kinematics ------------------------------------------------------------------------------------
def _rx(a):
    c, s_ = math.cos(a), math.sin(a); return np.array([[1, 0, 0, 0], [0, c, -s_, 0], [0, s_, c, 0], [0, 0, 0, 1]])


def _rz(a):
    c, s_ = math.cos(a), math.sin(a); return np.array([[c, -s_, 0, 0], [s_, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def _tz(d):
    T = np.eye(4); T[2, 3] = d; return T


def fk(q_deg) -> list[np.ndarray]:
    """Frames [J1, J2, J3, J4, J5, J6, tip] in world for joint angles in degrees (the arm's conventions)."""
    q = [math.radians(v) for v in q_deg[:6]]
    T1 = _tz(Z1 + S.horn_h) @ _rz(q[0])
    T2 = T1 @ _tz(P.J2_H) @ _rx(q[1])
    T3 = T2 @ _tz(-P.L_UPPER) @ _rx(q[2])
    T4 = T3 @ _tz(-(P.L_FORE - W5)) @ _rz(q[3])
    T5 = T4 @ _tz(-W5) @ _rx(q[4])
    T6 = T5 @ _rz(q[5])
    tip = T6 @ _tz(GRIP_TOP - P.HANDLE_L)
    return [T1, T2, T3, T4, T5, T6, tip]


def loc(T: np.ndarray) -> Location:
    return Plane(origin=Vector(*T[:3, 3]), x_dir=Vector(*T[:3, 0]), z_dir=Vector(*T[:3, 2])).location


# --- the servo frames inside each link frame -------------------------------------------------------------
# J1: on the base, horn up.  J2: in the shoulder, horn +X, case backward (-Y).  J3: at the upper arm's end, horn +X,
# case up the link (+Z).  J4: at the forearm's end, horn down the link (-Z), case across (-Y).  J5: in the wrist, horn +X,
# case up the wrist (+Z).  J6: its CASE is in the handle, its horn points up into a plate under the tool link's crossbar
# (a servo's case is behind its horn, so a horn-down J6 would put the case in the wrist).  Trigger: in the grip.
F_J1 = frame((0, 0, Z1), (0, 0, 1), (0, -1, 0))
F_J2 = frame((0, 0, P.J2_H), (1, 0, 0), (0, 1, 0))
F_J3 = frame((0, 0, -P.L_UPPER), (1, 0, 0), (0, 0, -1))
F_J4 = frame((0, 0, -(P.L_FORE - W5) + J4_BACK), (0, 0, -1), (0, 1, 0))
F_J5 = frame((0, 0, -W5), (1, 0, 0), (0, 0, -1))      # case up the wrist link, toward J4
HF6 = -(YOKE_DROP + 6.0 + EAR_T + S.horn_h) + 0.5   # J6's horn face, under the tool yoke's crossbar; the horn points UP into
F_J6 = frame((0, 0, HF6), (0, 0, 1), (0, 1, 0))      # a plate on the tool link, and the HANDLE carries the case (inverted mount)
GRIP_TOP = HF6 - (S.d + FIT) - 0.5                   # the grip cylinder starts under the J6 case
F_TRIG = frame((0, -14.0, GRIP_TOP - 22.0), (1, 0, 0), (0, 0, 1))
# the same joints seen from the DRIVEN link, whose origin is on the joint axis: the yokes' ears use these
IN_PITCH = frame((0, 0, 0), (1, 0, 0), (0, 1, 0))       # J2 and J5 as the upper arm / tool see them
IN_J3 = frame((0, 0, 0), (1, 0, 0), (0, 0, -1))          # J3 as the forearm sees it (the servo's case runs up the upper arm)


def base() -> Part:
    b = _cyl(P.BASE_D / 2, BASE_T)
    b += F_J1 * (Pos(0, 0, 0) * cradle(floor=0.0))
    b += Pos(0, (S.l - S.horn_off - S.horn_off) / 2, BASE_T) * Box(40, S.l + 6, Z1 - S.d - BASE_T - FIT, align=(Align.CENTER, Align.CENTER, Align.MIN))     # the cradle floor under the J1 case
    for k in range(4):                                                        # desk screws / clamp slots
        a = math.radians(45 + 90 * k)
        b -= Pos(P.BASE_D / 2 * 0.8 * math.cos(a), P.BASE_D / 2 * 0.8 * math.sin(a), -1) * _cyl(2.5, BASE_T + 2)
    if P.COUNTERBALANCE:
        b += Pos(0, -45, BASE_T) * _cyl(4.0, 60.0)                            # the rubber band's post
    return b


def shoulder() -> Part:
    """Link 1: on J1's horn; carries the J2 servo at J2_H with its horn along +X."""
    plate = ear()                                                             # link 1's origin is J1's horn face, horn axis +Z
    bar = Pos(0, (Y_POST - 12) / 2, S.horn_h) * Box(P.LINK_W, -(Y_POST - 12), EAR_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    post = Pos(0, Y_POST - 6 + 0.5, S.horn_h + EAR_T - 0.01) * Box(P.LINK_W, 12, P.J2_H + 8 - S.horn_h - EAR_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plate + bar + post + F_J2 * cradle()


def upper_arm() -> Part:
    """Link 2: yoke on J2 (ears on horn +X and idler -X), bar down -Z to J3, whose servo it carries."""
    yoke = IN_PITCH * ear() + IN_PITCH * ear(on_idler=True)
    x_h = S.horn_h + EAR_T                                                    # the horn ear's outer face, +X
    x_i = -(S.d + 1.5 + EAR_T)                                                # the idler ear's outer face, -X
    z_top = -YOKE_DROP                                                        # the crossbar clears the cradle's swept corner
    cross = Pos((x_h + x_i) / 2, 0, z_top) * Box(x_h - x_i, P.LINK_W, 8.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for x in (x_h - EAR_T / 2, x_i + EAR_T / 2):                              # the ears' arms down to the crossbar
        yoke += Pos(x, 0, z_top) * Box(EAR_T, P.LINK_W, -z_top - ARM_TOP, align=(Align.CENTER, Align.CENTER, Align.MIN))
    bar = Pos(0, 0, z_top - 8.0) * Box(P.LINK_W, P.LINK_T, (z_top - 8.0) - (-P.L_UPPER + S.l - S.horn_off + P.WALL + FIT), align=(Align.CENTER, Align.CENTER, Align.MAX))
    return yoke + cross + bar + F_J3 * cradle()


def forearm() -> Part:
    """Link 3: yoke on J3, bar down to the J4 servo, whose horn points down the link (the roll)."""
    yoke = IN_J3 * ear() + IN_J3 * ear(on_idler=True)
    x_h, x_i = S.horn_h + EAR_T, -(S.d + 1.5 + EAR_T)
    z_top = -YOKE_DROP
    cross = Pos((x_h + x_i) / 2, 0, z_top) * Box(x_h - x_i, P.LINK_W, 8.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for x in (x_h - EAR_T / 2, x_i + EAR_T / 2):
        yoke += Pos(x, 0, z_top) * Box(EAR_T, P.LINK_W, -z_top - ARM_TOP, align=(Align.CENTER, Align.CENTER, Align.MIN))
    z_j4 = -(P.L_FORE - W5) + J4_BACK
    bar = Pos(0, 0, z_top - 8.0) * Box(P.LINK_W, P.LINK_T, (z_top - 8.0) - (z_j4 + S.w / 2 + FIT + P.WALL), align=(Align.CENTER, Align.CENTER, Align.MAX))
    return yoke + cross + bar + F_J4 * cradle()


def wrist() -> Part:
    """Link 4: a plate on J4's horn, dropping J4_BACK to the wrist centre where it carries the J5 servo (horn +X)."""
    # link 4's origin is the J4 axis point (L_FORE - W5 down the forearm); J4's horn face is J4_BACK above it, facing down
    plate = Pos(0, 0, J4_BACK) * (frame((0, 0, 0), (0, 0, -1), (0, 1, 0)) * ear())
    z_post = -W5 + SWEEP_R + 1.0                                              # outside the tool yoke's sweep
    post = Pos(0, 0, z_post) * Box(P.LINK_W, 12, J4_BACK - S.horn_h - EAR_T - z_post, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return plate + post + F_J5 * cradle(floor=6.0)                          # the floor reaches up to meet the post


def tool() -> Part:
    """Link 5: yoke on J5 (ears +X horn, -X idler), carrying the J6 servo with its horn down the tool axis."""
    yoke = IN_J3 * ear() + IN_J3 * ear(on_idler=True)
    x_h, x_i = S.horn_h + EAR_T, -(S.d + 1.5 + EAR_T)
    z_top = -YOKE_DROP
    cross = Pos((x_h + x_i) / 2, 0, z_top) * Box(x_h - x_i, P.LINK_W, 6.0, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for x in (x_h - EAR_T / 2, x_i + EAR_T / 2):
        yoke += Pos(x, 0, z_top) * Box(EAR_T, P.LINK_W, -z_top - ARM_TOP, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return yoke + cross + F_J6 * ear()                                        # the J6 horn plate, under the crossbar


def handle() -> tuple[Part, Part]:
    """Link 6: the grip, carrying the J6 servo's case (horn up into the tool link), and the trigger lever."""
    grip = Pos(0, 0, GRIP_TOP) * Cylinder(14.0, P.HANDLE_L, align=(Align.CENTER, Align.CENTER, Align.MAX))
    h = F_J6 * cradle() + grip
    h += F_TRIG * cradle()
    h -= F_TRIG * servo_body(grow=FIT)
    lever = F_TRIG * (Pos(0, 0, S.horn_h) * ear() + Pos(0, 0, S.horn_h) * Box(6, 40, EAR_T, align=(Align.CENTER, Align.MIN, Align.MIN)))
    return h, lever


def overlaps(G) -> list[str]:
    printed = {s.label: s for s in G["printed"]}
    chain = ["base", "shoulder", "upper arm", "forearm", "wrist", "tool"]
    return [f"{a} x {b}: {(printed[a] & printed[b]).volume:.0f} mm^3" for a, b in zip(chain, chain[1:]) if (printed[a] & printed[b]).volume > 1.0]


def fold_limit(joint: int, step: float = 5.0, base=(0, 90, 0, 0, 0, 0, 0)) -> tuple[float, float]:
    """(negative, positive) travel from `base` at which the driven link first meets the carrying link."""
    out = []
    for sgn in (-1, 1):
        q = list(base); a = 0.0
        while abs(a) < 180:
            a += sgn * step; q[joint] = base[joint] + a
            if overlaps(assembly(q)[0]):
                break
        out.append(a - sgn * step)
    return out[0], out[1]


LINKS = {"base": base, "shoulder": shoulder, "upper_arm": upper_arm, "forearm": forearm, "wrist": wrist, "tool": tool}
HOME = (0.0, 110.0, -70.0, 0.0, 20.0, 0.0, 0.0)


def assembly(q=HOME):
    T = fk(q)
    G: dict[str, list] = {"printed": [], "servo": [], "handle": [], "desk": []}
    add = lambda g, s, name: (setattr(s, "label", name), G[g].append(s))
    add("printed", base(), "base")
    add("servo", F_J1 * servo_body(), "J1 servo")
    L1 = loc(T[0]); add("printed", L1 * shoulder(), "shoulder"); add("servo", L1 * F_J2 * servo_body(), "J2 servo")
    L2 = loc(T[1]); add("printed", L2 * upper_arm(), "upper arm"); add("servo", L2 * F_J3 * servo_body(), "J3 servo")
    L3 = loc(T[2]); add("printed", L3 * forearm(), "forearm"); add("servo", L3 * F_J4 * servo_body(), "J4 servo")
    L4 = loc(T[3]); add("printed", L4 * wrist(), "wrist"); add("servo", L4 * F_J5 * servo_body(), "J5 servo")
    L5 = loc(T[4]); add("printed", L5 * tool(), "tool")
    L6 = loc(T[5]); h, lever = handle(); add("servo", L6 * F_J6 * servo_body(), "J6 servo")
    add("handle", L6 * h, "handle"); add("handle", L6 * lever, "trigger lever"); add("servo", L6 * F_TRIG * servo_body(), "trigger servo")
    add("desk", Pos(0, 0, -5) * Box(500, 500, 10), "desk")
    return G, T


def main(argv):
    q = tuple(float(v) for v in argv[argv.index("--q") + 1].split(",")) if "--q" in argv else HOME
    OUT.mkdir(exist_ok=True)
    for name, fn in LINKS.items():
        save(fn(), name, "PETG", 0.4)
    h, lever = handle()
    save(h, "handle", "PETG", 0.4); save(lever, "trigger_lever", "PETG", 0.4)
    G, T = assembly(q)
    tip = T[6][:3, 3]
    print(f"pose {q}: tip at ({tip[0]:.0f}, {tip[1]:.0f}, {tip[2]:.0f}) mm; reach J2 -> wrist {P.L_UPPER + P.L_FORE:.0f} mm")
    from build123d import export_stl
    for grp in ("printed", "servo", "handle"):
        export_stl(Compound(G[grp]), str(OUT / f"assembly_{grp}.stl"), tolerance=0.05, angular_tolerance=0.3)
    mass = sum(s.volume for s in G["printed"] + G["handle"]) / 1000 * 1.27 * 0.4
    print(f"printed mass ~{mass:.0f} g at 40 % PETG; 7 servos {7 * S.mass_g:.0f} g")
    bad = overlaps(G)
    print("overlaps at this pose:", "none" if not bad else "; ".join(bad))
    if "--limits" in argv:
        for j, name in ((1, "J2"), (2, "J3"), (4, "J5")):
            print(f"{name} folds to {fold_limit(j)} deg before the links meet")
    return G


if __name__ == "__main__":
    main(sys.argv[1:])
