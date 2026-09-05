"""Render STL files to shaded PNG views with a numpy z-buffer (no display, no OpenGL).

    uv run python tools/render.py out/part.stl [more.stl ...]          # one PNG per file, 3 views
    uv run python tools/render.py --scene out/name.png a.stl=#5f9aa6 b.stl=#2e3438 [--views iso,side,front]

Crease edges are drawn where visible, so holes, chamfers and text read clearly.
"""
import sys
from pathlib import Path

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

W, H = 900, 640
INK = (0.10, 0.13, 0.15)
BODY = "#5c99a8"
VIEWS = {
    "iso": (1.2, -1.6, 1.0), "iso2": (-1.4, -1.2, 0.9), "end": (-1, 0.12, 0.25), "top": (0.02, -0.01, 1),
    "side": (1, 0.0, 0.12), "front": (0.0, 1, 0.15), "back": (0.0, -1, 0.15),
}


def _hex(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) / 255 for i in (0, 2, 4)])


def _basis(cam_dir, up=(0, 0, 1)):
    f = np.asarray(cam_dir, float); f /= np.linalg.norm(f)
    up = np.asarray(up, float)
    if abs(np.dot(f, up)) > 0.98:
        up = np.array([0, 1, 0], float)
    right = np.cross(up, f); right /= np.linalg.norm(right)
    return right, np.cross(f, right), f


def rasterize(meshes, cam_dir, margin=0.06, fit=None):
    """fit: optional (points Nx3) whose projection fixes the framing, e.g. across animation frames."""
    right, up, fwd = _basis(cam_dir)
    allv = np.vstack([m.vertices for m, _ in meshes]) if fit is None else np.asarray(fit, float)
    pts = np.stack([allv @ right, allv @ up], axis=1)
    lo, hi = pts.min(0), pts.max(0)
    span = (hi - lo).max() * (1 + 2 * margin)
    scale = min(W, H) / span
    centre = (lo + hi) / 2
    zbuf = np.full((H, W), -np.inf)
    img = np.ones((H, W, 3))
    light = fwd * 0.75 + up * 0.55 + right * 0.35
    light /= np.linalg.norm(light)
    segs = []
    t = np.linspace(0, 1, 14)
    for mesh, color in meshes:
        v = mesh.vertices
        px = (v @ right - centre[0]) * scale + W / 2
        py = H / 2 - (v @ up - centre[1]) * scale
        pz = v @ fwd
        for tri, n in zip(mesh.faces, mesh.face_normals):
            if np.dot(n, fwd) <= 0:
                continue
            xs, ys, zs = px[tri], py[tri], pz[tri]
            x0, x1 = int(max(np.floor(xs.min()), 0)), int(min(np.ceil(xs.max()), W - 1))
            y0, y1 = int(max(np.floor(ys.min()), 0)), int(min(np.ceil(ys.max()), H - 1))
            if x1 < x0 or y1 < y0:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            d = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0])
            if abs(d) < 1e-9:
                continue
            l1 = ((xs[1] - gx) * (ys[2] - gy) - (xs[2] - gx) * (ys[1] - gy)) / d
            l2 = ((xs[2] - gx) * (ys[0] - gy) - (xs[0] - gx) * (ys[2] - gy)) / d
            l3 = 1 - l1 - l2
            inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
            if not inside.any():
                continue
            depth = l1 * zs[0] + l2 * zs[1] + l3 * zs[2]
            sub = zbuf[y0:y1 + 1, x0:x1 + 1]
            upd = inside & (depth > sub)
            sub[upd] = depth[upd]
            shade = 0.30 + 0.70 * max(float(np.dot(n, light)), 0.0)
            img[y0:y1 + 1, x0:x1 + 1][upd] = np.clip(color * shade + 0.10 * (1 - shade), 0, 1)
        ang = mesh.face_adjacency_angles
        creases = mesh.face_adjacency_edges[ang > np.radians(25)]
        for a, b in creases:
            p = (px[a] + (px[b] - px[a]) * t, py[a] + (py[b] - py[a]) * t, pz[a] + (pz[b] - pz[a]) * t)
            segs.append(p)
    # visible crease segments, tested against the finished z-buffer
    lines = []
    for p in segs:
        xi, yi = np.clip(p[0].astype(int), 0, W - 1), np.clip(p[1].astype(int), 0, H - 1)
        vis = p[2] >= zbuf[yi, xi] - span * 0.004
        run = []
        for k in range(len(t)):
            if vis[k]:
                run.append((p[0][k], p[1][k]))
            elif run:
                if len(run) > 1: lines.append(run)
                run = []
        if len(run) > 1:
            lines.append(run)
    def proj(p):
        p = np.asarray(p, float)
        return ((p @ right - centre[0]) * scale + W / 2, H / 2 - (p @ up - centre[1]) * scale)
    return img, lines, proj


def _load(path):
    m = trimesh.load(path, force="mesh")
    m.update_faces(m.nondegenerate_faces())
    return m


def render_scene(meshes, out: Path, views, title="", labels=()) -> Path:
    """labels: (text, xyz, (dx, dy)) — a 3D anchor plus a pixel offset for the tag."""
    fig, axes = plt.subplots(1, len(views), figsize=(5.4 * len(views), 4.6), dpi=120)
    for ax, name in zip(np.atleast_1d(axes), views):
        img, lines, proj = rasterize(meshes, VIEWS[name])
        ax.imshow(img)
        for run in lines:
            r = np.array(run)
            ax.plot(r[:, 0], r[:, 1], color=INK, linewidth=0.5, solid_capstyle="round")
        for text, xyz, (dx, dy) in labels:
            x, y = proj(xyz)
            ax.annotate(text, (x, y), xytext=(x + dx, y + dy), fontsize=7.5, color=INK, ha="center",
                        arrowprops=dict(arrowstyle="-", color=INK, lw=0.7, shrinkA=0, shrinkB=2),
                        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#8a9aa0", lw=0.6))
        ax.set_axis_off(); ax.set_title(name, fontsize=9, color="#444")
    if title:
        fig.suptitle(title, fontsize=10)
    fig.tight_layout(); fig.savefig(out, facecolor="white"); plt.close(fig)
    print(out)
    return out


def render(stl: Path, views=("iso", "end", "top")) -> Path:
    mesh = _load(stl)
    ext = mesh.extents
    return render_scene([(mesh, _hex(BODY))], stl.with_suffix(".png"), views,
                        f"{stl.stem}   {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm")


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--scene":
        out = Path(args[1]); views = ("iso", "side", "front"); items = []
        rest = args[2:]
        if "--views" in rest:
            i = rest.index("--views"); views = tuple(rest[i + 1].split(",")); rest = rest[:i] + rest[i + 2:]
        for item in rest:
            path, _, color = item.partition("=")
            items.append((_load(path), _hex(color or BODY)))
        render_scene(items, out, views, out.stem)
    else:
        for arg in args:
            render(Path(arg))
