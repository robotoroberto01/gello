# Gello

A passive seven-encoder leader prototype for RobotDoctor with a raised desk base, connected grip and moving finger trigger. The arm links retain half-scale nominal geometry; the actual follower integration remains to be built.

[Design and validation](docs/design-2026-09-05.md) · [Parts and allocation](docs/bom.md) · [Stock review](docs/stock-review.md) · [Electrical review](docs/electronics.md)

![Current design](docs/design-review.png)

```sh
uv sync
uv run python -m cad.gello
uv run python -m tools.materials
uv run pytest
# After node/interface commissioning and measured calibration:
uv run python -m gello.reader
uv run python -m gello.follow --dry
```

STL and STEP files go to `out/`. Each of the eight named parts prints once. Geometry is checked against nominal servo envelopes and representative poses; actual horn, case and rear supports need a physical fit check. All seven XL330s remain unpurchased according to the recorded inventory.

The sketch remains torque-off. `t` requests torque-off; parking is unavailable. `z` directs calibration to the host. `gello.follow --sim` prints forward kinematics through RobotDoctor; it does not drive MuJoCo or a physical arm. The native firmware test uses a fake UART under sanitizers, not an ESP32 board build.
