# Shared stock review — 5 September 2026

Snapshot from `/Users/tyler/Desktop/RobotDoctor/docs/inventory.md`, its `docs/cross-build-procurement.md`, and Fable's `/Users/tyler/Desktop/claude-openai-bridge/claude-to-openai.md` reply 001. Fable's newer allocation/order corrections take precedence where the earlier draft conflicts. This is a review of records, not a physical count or an order-account inspection. No purchases or reservations with suppliers were made.

## Material records

| Material | Recorded stock/order | Availability for these builds |
|---|---|---|
| Carbon, 30 mm OD × 27 mm ID × 500 mm | Two ordered 09-01; scheduled 09-04 | Tube 1 reserved for RobotDoctor, tube 2 for Wheelbot's 200 + 200 mm cuts. Arrival and actual length need confirmation; include kerf |
| PET-CF17 | Ordered; count and remaining weight unrecorded | Shared shortage cannot be calculated. The arm already has substantial demand; no free "margin" is established |
| PAHT-CF | Ordered; count and remaining weight unrecorded | No new allocation in these revisions; keep reserved for the arm |
| PETG | Ordered; count and remaining weight unrecorded | Intended for Gello and Wheelbot lid after fit checks; subtract other builds first |
| PLA | Ordered; count and remaining weight unrecorded | First-fit prototypes; actual free stock unknown |
| TPU 95A, Overture | One 1 kg spool ordered 09-04 | Recorded quantity covers Wheelbot tyre allowance (~404 g) plus the earlier arm's ~25 g estimate if delivered and otherwise unused |
| ASA white, Polymaker | One 1 kg spool ordered 09-04 | No new allocation for Wheelbot or Gello; arm covers remain the intended use |
| Aluminium stock / McMaster fasteners | Several items were placed in a cart | Not confirmed purchased or available; no new metal stock is required for these CAD changes |

The revised CAD allowances are **1,311 g PET-CF + 126 g PETG + 404 g TPU for Wheelbot**, and **551 g PETG for Gello**. They use each part's full-solid volume, actual build quantity and a separate 20% allowance. This deliberately avoids the old volume × infill calculation, which ignores solid walls. Allowances are not slicer outputs; supports or additional test prints can exceed the extra 20%. The combined PETG allowance is about 677 g before RobotDoctor's demand.

## Installed component allocation

| Component | Recorded total | RobotDoctor allocation | Wheelbot / Gello consequence |
|---|---:|---:|---|
| RS03 | 3: original recorded delivered, two additional ordered 09-04, expected 09-06 per Fable | 2, shoulder and elbow | 1 remains toward Wheelbot's matching pair; one more needed |
| J8009P | 1, ordered | 1, base joint | None available for Wheelbot; mixed-hip design retired |
| J4310, ordered 48 V V4 drives | 3, ordered | 3, wrist | Wheelbot needs two additional matching units with boards/leads |
| XC330-M288-T | 6, ordered | 6, five-finger hand | None available for Gello |
| XL330-M288-T | 0 orders recorded | 0 | Gello needs seven |
| ESP32-S3 | Three ordered | Three hand nodes per Fable | A fourth board needed for Gello unless a separate spare is confirmed |
| SN74AHCT125 | Ten ordered | Hand nodes and spares | Potentially one for Gello; delivery and electrical design still to confirm |
| X3P 180 mm cables | One ten-pack ordered | Hand cabling | Gello needs its own route-based allocation; check reach and service loops before choosing lengths |
| 4700 µF capacitors / fuses / stops / power converters | Orders recorded | Existing units allocated to arm per Fable | No complete spare Wheelbot power system established |
| Pi 5, Pi 4, USB-CAN adapters | Existing/ordered mix | Allocation differs across old docs | Confirm a dedicated Wheelbot controller; do not assign one installed device to both running robots |

Fable records no U2D2/OpenRB purchase and no identified RC transmitter, receiver or charger. Old LiPos exist but have unknown identity and condition and are not counted as usable Wheelbot power. GH leads, 120 Ω resistors, wire, Wagos, epoxy and threadlocker are recorded; exact remaining consumable quantities still need a bench check. The Glarks fastener kit was expected 09-06, and the McMaster cart is not a delivery record.

## Information requested from Tyler

- Purchased and remaining quantities of PET-CF, PAHT-CF, PETG and PLA.
- Whether the 1 kg TPU, 1 kg ASA and both carbon tubes have arrived.
- Any RC transmitter/receiver and balance charger model numbers.

Before full printing, measure the real servo/motor interfaces and slice the final plate quantities. Before deciding Wheelbot power, obtain the exact J4310 V4 voltage specification, controller allocation and battery identity. Those missing facts do not prevent the CAD repairs or fit prototypes, but they prevent a reliable final purchase shortfall and a powered build release.
