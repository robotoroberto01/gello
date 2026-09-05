"""Shared constants and helpers for FDM-printed parts (Bambu, 0.4 mm nozzle).

All dimensions in millimetres. Tune the tolerances after the first test print;
they are the only numbers here that are printer-specific.
"""
from __future__ import annotations

import math
from pathlib import Path

from build123d import *  # noqa: F401,F403  (re-exported for part scripts)
from build123d import Part, export_step, export_stl

OUT = Path(__file__).resolve().parent.parent / "out"

# --- FDM tolerances -----------------------------------------------------------
LAYER = 0.2
PERIMETER = 0.45          # extrusion width at 0.4 mm nozzle
WALL_4 = 4 * PERIMETER    # a 4-perimeter wall, ~1.8 mm
BOLT_CLEARANCE = 0.3      # added to nominal bolt diameter for a free fit
PRESS_FIT = -0.15         # added to nominal diameter for a light press fit
HORIZONTAL_HOLE_SAG = 0.2 # horizontal holes print undersize at the top; add this

# heat-set inserts (standard brass, e.g. M3 x 5.7 mm, OD 4.0 mm)
INSERT = {
    "M3": {"hole": 4.0, "depth": 6.0, "boss": 7.0},
    "M4": {"hole": 5.6, "depth": 8.0, "boss": 9.0},
    "M5": {"hole": 6.4, "depth": 9.5, "boss": 10.5},
}

DENSITY = {"PETG": 1.27, "ASA": 1.07, "PLA": 1.24, "PET-CF": 1.30, "PAHT-CF": 1.20, "6061": 2.70}  # g/cm^3 (6061: the J1 adapter, machined)


def teardrop_points(r: float, n: int = 24) -> list[tuple[float, float]]:
    """Outline of a teardrop with a 90-degree apex pointing +Y, for horizontal holes
    that must print without support. The apex sits at y = r*sqrt(2)."""
    pts = []
    # arc from 45 deg round the bottom to 135 deg (going the long way through -Y)
    start, end = math.radians(45), math.radians(-225)
    for i in range(n + 1):
        a = start + (end - start) * i / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    pts.append((0.0, r * math.sqrt(2)))
    return pts


def save(part, name: str, material: str = "PETG", infill: float = 1.0, tolerance: float = 0.02) -> None:
    """Export STL + STEP into out/ and print the numbers worth checking."""
    OUT.mkdir(exist_ok=True)
    export_stl(part, str(OUT / f"{name}.stl"), tolerance=tolerance, angular_tolerance=0.1 if tolerance < 0.1 else 0.3)
    export_step(part, str(OUT / f"{name}.step"))
    bb = part.bounding_box()
    size = bb.size
    vol_cm3 = part.volume / 1000.0
    grams = vol_cm3 * DENSITY[material] * infill
    print(f"{name}: {size.X:.1f} x {size.Y:.1f} x {size.Z:.1f} mm, "
          f"{vol_cm3:.1f} cm^3, ~{grams:.0f} g of {material} at {infill:.0%} of solid")
    print(f"  wrote {OUT / (name + '.stl')} and .step")
