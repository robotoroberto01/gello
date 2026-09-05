# Gello

A GELLO-style leader arm for the RobotDoctor arm: its kinematics at half scale, a DYNAMIXEL XL330 in every joint
as an encoder, a trigger in the grip. Move the small arm and the big one follows, joint for joint.

Design brief: `docs/design-2026-09-05.md`. Parts: `docs/bom.md`.

    uv sync
    uv run python -m cad.gello                       # every printed part to out/ + the assembly at the home pose
    uv run python -m cad.gello --q 0,90,-90,0,0,0,0  # any pose (degrees, the arm's joint conventions)
    uv run python tools/render.py --scene out/gello.png out/assembly_printed.stl=#5c99a8 out/assembly_servo.stl=#2e3438 out/assembly_handle.stl=#c9a86a
    uv run python tools/view.py                      # live in the OCP viewer (uv run python -m ocp_vscode --host 127.0.0.1 --port 3939 first)
    uv run pytest

    # with the node flashed (firmware/gello_node) and the servos on the bus:
    uv run python -m gello.reader --calibrate        # capture the zero pose -> calibration.json
    uv run python -m gello.reader                    # joint angles, live
    uv run python -m gello.follow --sim              # the arm's simulator follows the leader (needs ~/Desktop/RobotDoctor)

`cad/servos.py` holds the servo's dimensions (VERIFY with calipers on the first one), `cad/params.py` the arm's
link lengths and the scale, `cad/gello.py` the links, the forward kinematics and the assembly.
