"""Show the leader arm in the OCP CAD Viewer (port 3939).

    uv run python tools/view.py                        # assembly at the home pose
    uv run python tools/view.py --q 0,90,-90,0,0,0,0
    uv run python tools/view.py upper_arm forearm      # single parts side by side
"""
from __future__ import annotations

import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build123d import Pos  # noqa: E402
from cad import gello as G  # noqa: E402


def viewer_show(items):
    try:
        from ocp_vscode import Camera, show
    except ImportError:
        sys.exit("ocp_vscode is not installed: uv sync")
    try:
        socket.create_connection(("127.0.0.1", 3939), timeout=0.5).close()
    except OSError:
        sys.exit("no OCP CAD Viewer on port 3939: uv run python -m ocp_vscode --host 127.0.0.1 --port 3939")
    show(*[s for _, s, _ in items], names=[n for n, _, _ in items], colors=[c for _, _, c in items],
         axes=True, axes0=True, grid=(True, False, False), default_edgecolor="#333333", reset_camera=Camera.RESET)
    print("shown:", ", ".join(n for n, _, _ in items))


def main(argv):
    names = [a for a in argv if a in G.LINKS or a == "handle"]
    if names:
        x, items = 0.0, []
        for n in names:
            part = G.handle()[0] if n == "handle" else G.LINKS[n]()
            w = part.bounding_box().size.X
            items.append((n, Pos(x + w / 2, 0, 0) * part, G.COLOURS["printed"])); x += w + 30
        viewer_show(items); return
    q = tuple(float(v) for v in argv[argv.index("--q") + 1].split(",")) if "--q" in argv else G.HOME
    Gr, _ = G.assembly(q)
    viewer_show([(f"{g}: {s.label}", s, G.COLOURS[g]) for g, shapes in Gr.items() for s in shapes])


if __name__ == "__main__":
    main(sys.argv[1:])
