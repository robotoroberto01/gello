# Gello parts and allocation — 5 September 2026

Source: Fable bridge reply 001 and RobotDoctor's recorded orders. No purchasing occurred. [Stock review](stock-review.md) separates order records from available material.

| Item | Qty | Current status |
|---|---:|---|
| XL330-M288-T | 7 | No order recorded; six joints plus trigger. Six ordered XC330s remain committed to the hand |
| Dedicated X3P cable set | Route-dependent | A second cable allocation is needed. Measure connector-to-connector routes with service loops; a 180 mm cable must not be assumed to span a 200 mm forearm |
| ESP32-S3 board | 1 | Fourth board needed if all three incoming boards remain assigned to the three hand nodes |
| SN74AHCT125 | 1 | Ten ordered; reserve one only after delivery and allocation check |
| RX level-shifting components, decoupling and prototyping board | Circuit-dependent | Missing from old BOM; final schematic and exact parts unresolved |
| USB data cable and qualified encoder power | 1 set | Existing cable/port may suffice only after current-path, voltage and backfeed checks |
| Servo horn/case and rear support hardware | Drawing-dependent | Fit actual ROBOTIS hardware before buying the old generic M2/M3 quantities |
| Base screws or clamps | 4 screws or suitable clamps | Reuse workshop stock if available; model has four 5 mm through holes |
| Counterbalance parts | Undesigned | Modeled peg is not a completed mechanism; no powered parking mode |

Each of base, shoulder, upper_arm, forearm, wrist, tool, handle and trigger_lever prints once. Use existing PLA for fit coupons; PETG is the prototype material. No CF filament, new metal stock or new carbon tube is allocated to Gello.

[Generated material allowance](material-plan.md): **0.551 kg PETG**, based on full-solid equivalent plus 20%. The previous 150 g figure multiplied volume by infill and did not include the raised pedestal or account for solid walls. Slice all eight parts and supports before subtracting measured stock. This is an allowance, not an order quantity.
