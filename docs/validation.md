# Validation — 5 September 2026

- Python test suite: **20 passed**. The bundled lib3mf dependency emitted deprecation warnings.
- All individual print STLs are watertight and consistently wound.
- CAD export requires one connected valid solid per named print and produced STL plus STEP files.
- Updated assembly PNG inspected visually; no missing parts or detached grip solids.
- Material reports include all copies and distinguish planning allowance from actual sliced mass.
- `git diff --check` passed.

 The Gello suite also compiles the sketch with a fake UART under AddressSanitizer and UndefinedBehaviorSanitizer; it does not compile an ESP32 target.

No motor, servo, battery or printer was operated. Actual component dimensions, tolerances, load capacity, wiring, complete workspace and power behaviour remain to be qualified as described in the design brief. The finite CAD poses are not a continuous interference proof.
