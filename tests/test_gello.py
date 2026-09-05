import math

import numpy as np
import pytest

from cad import gello as G
from cad import params as P
from gello.protocol import Calibration, MISSING, parse_line, trigger_to_grip


def test_fk_reach_and_scale():
    T = G.fk((0, 0, 0, 0, 0, 0, 0))
    wrist = T[4][:3, 3]
    assert abs(wrist[0]) < 1e-9 and abs(wrist[1]) < 1e-9
    assert abs((T[1][2, 3] - wrist[2]) - (P.L_UPPER + P.L_FORE)) < 1e-9          # hanging: wrist straight below J2
    assert P.L_UPPER / P.L_FORE == pytest.approx(P.ARM["L_UPPER"] / P.ARM["L_FORE"])
    # J2 = 90: the upper arm points +Y (forward), J3 = -90 brings the forearm down
    T = G.fk((0, 90, -90, 0, 0, 0, 0))
    j3 = T[2][:3, 3]
    assert j3[1] == pytest.approx(P.L_UPPER) and j3[2] == pytest.approx(T[1][2, 3])
    assert T[4][:3, 3][2] == pytest.approx(T[1][2, 3] - P.L_FORE)


def test_j1_turns_about_z():
    a = G.fk((0, 90, 0, 0, 0, 0, 0))[4][:3, 3]
    b = G.fk((90, 90, 0, 0, 0, 0, 0))[4][:3, 3]
    assert np.hypot(*a[:2]) == pytest.approx(np.hypot(*b[:2]))
    assert a[2] == pytest.approx(b[2])
    assert b[0] == pytest.approx(-a[1]) and b[1] == pytest.approx(a[0])


def test_parts_build_and_pockets_open():
    from build123d import Vector
    for name, fn in G.LINKS.items():
        p = fn()
        assert p.volume > 100, name
    # the J2 cradle in the shoulder must have room for the servo's case centre
    sh = G.shoulder()
    from build123d import Pos
    centre = (G.F_J2 * Pos(0, -(G.S.l - G.S.horn_off) / 2, -G.S.d / 2)).position
    assert not sh.is_inside(centre)
    # and the horn ear on the upper arm has its 4 x M2 open
    ua = G.upper_arm()
    for k in range(4):
        a = math.radians(45 + 90 * k)
        pt = (G.IN_PITCH * Pos(G.S.horn_pcd / 2 * math.cos(a), G.S.horn_pcd / 2 * math.sin(a), G.S.horn_h + 2)).position
        assert not ua.is_inside(pt)


@pytest.mark.parametrize("q", [G.HOME, (0, 40, 0, 0, 0, 0, 0), (45, 150, -100, 90, 90, 90, 40), (0, 90, 90, 0, -90, 0, 0)])
def test_assembly_has_no_link_overlaps(q):
    Gr, _ = G.assembly(q)
    printed = {s.label: s for s in Gr["printed"]}
    assert G.overlaps(Gr) == []


def test_fold_limits_cover_the_arm():
    lo, hi = G.fold_limit(2, step=10.0)                        # J3 from straight: the arm's working band is within +/-110
    assert lo <= -110 and hi >= 110, (lo, hi)
    lo, hi = G.fold_limit(4, step=10.0, base=(0, 90, -90, 0, 0, 0, 0))
    assert lo <= -90 and hi >= 90, (lo, hi)


def test_protocol():
    s = parse_line("G,12,3456,2048,1024,3072,2048,2048,2048,65535,63\n")
    assert s and s.seq == 12 and s.ok == 63 and np.isnan(s.counts[6]) and s.counts[1] == 1024
    assert parse_line("junk") is None and parse_line("G,1,2,3") is None
    cal = Calibration(zero=[2048] * 7, sign=[1, -1, 1, 1, 1, 1, 1])
    q = cal.to_degrees(np.array([2048 + 1024, 2048 + 1024, 2048, 2048, 2048, 2048, 2048 + 200]))
    assert q[0] == pytest.approx(90) and q[1] == pytest.approx(-8)          # sign flipped, then clamped to J2's -8 floor
    assert q[6] == pytest.approx(200 * 360 / 4096)
    # unwrap: a count just under the zero reads slightly negative, not +359
    q = cal.to_degrees(np.array([2040, 2048, 2048, 2048, 2048, 2048, 2048]))
    assert -1 < q[0] < 0
    assert trigger_to_grip(20) == pytest.approx(0.5) and trigger_to_grip(80) == 1.0
