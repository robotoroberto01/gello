"""Quantity-aware material budget from current CAD; no slicer or stock assumptions.

Run: python -m tools.materials -> out/material-plan.json and out/material-plan.md.
Solid-equivalent + 20% is a purchasing allowance, not a slicer result or proof of stock.
"""
import json
from collections import defaultdict
from cad.lib import OUT, DENSITY
from cad.gello import printed_parts


def report():
    rows = []
    totals = defaultdict(float)
    for name, (part, material, infill, quantity) in printed_parts().items():
        grams = part.volume / 1000 * DENSITY[material] * quantity
        rows.append(dict(part=name, quantity=quantity, material=material,
                         volume_each_cm3=round(part.volume / 1000, 2),
                         solid_equivalent_g=round(grams, 1)))
        totals[material] += grams
    data = dict(parts=rows, material_totals={m: dict(solid_equivalent_g=round(g, 1),
                allowance_g=round(g * 1.2, 1), confirmed_free_stock_g=None) for m, g in totals.items()})
    OUT.mkdir(exist_ok=True)
    (OUT / "material-plan.json").write_text(json.dumps(data, indent=2) + "\n")
    lines = ["# CAD material allowance", "", "Generated with `python -m tools.materials`. Quantities include both sides.",
             "", "These are full-solid polymer equivalents, not infill multiplied by volume. The 20% allowance", 
             "is provisional for support, test fits and waste; replace it with actual slicer plate totals.",
             "Free stock is unknown, so no purchase deficit is computed.", "",
             "| Part | Qty | Material | Solid equivalent, all copies (g) |", "|---|---:|---|---:|"]
    lines += [f"| {r['part']} | {r['quantity']} | {r['material']} | {r['solid_equivalent_g']:.1f} |" for r in rows]
    lines += ["", "| Material | Solid equivalent (g) | With 20% allowance (g) |", "|---|---:|---:|"]
    lines += [f"| {m} | {g:.1f} | {g*1.2:.1f} |" for m, g in totals.items()]
    (OUT / "material-plan.md").write_text("\n".join(lines) + "\n")
    print(json.dumps(data['material_totals'], indent=2))
    return data


if __name__ == "__main__":
    report()
