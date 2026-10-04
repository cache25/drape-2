#!/usr/bin/env python3
"""Preview renderer for drape_dump output: python3 tools/render_obj.py <in.obj> <out.png>

Draws a 1600x1000 px figure with two panels, a front view from +Z and a side view from +X. Objects named
body_* are light grey and every other object (the garment) is blue, Lambert-shaded from the light direction
(0.3, 0.6, 1.0). All faces share one Poly3DCollection so matplotlib depth-sorts body and garment together;
in each orthographic view the body is pushed 1 cm away from the camera, which leaves the image unchanged but
stops body faces lying a few millimetres under the cloth from sorting in front of it.
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

BODY_RGB = np.array([0.82, 0.82, 0.82])
GARMENT_RGB = np.array([0.20, 0.42, 0.85])
LIGHT = np.array([0.3, 0.6, 1.0]) / np.linalg.norm([0.3, 0.6, 1.0])


def load_obj(path):
    """Returns (vertices Nx3, faces Mx3 zero-based, is_body per face)."""
    vertices, faces, body = [], [], []
    current = ""
    with open(path) as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "o":
                current = parts[1] if len(parts) > 1 else ""
            elif parts[0] == "v":
                vertices.append([float(c) for c in parts[1:4]])
            elif parts[0] == "f":
                faces.append([int(p.split("/")[0]) - 1 for p in parts[1:4]])
                body.append(current.startswith("body"))
    return np.array(vertices), np.array(faces, dtype=int), np.array(body, dtype=bool)


def face_colors(vertices, faces, is_body):
    tri = vertices[faces]
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.maximum(lengths, 1e-20)
    lambert = np.abs(normals @ LIGHT)  # cloth is two-sided; body faces are lit the same way
    shade = (0.25 + 0.75 * lambert)[:, None]
    base = np.where(is_body[:, None], BODY_RGB, GARMENT_RGB)
    return np.clip(base * shade, 0, 1)


def main():
    if len(sys.argv) != 3:
        print("usage: render_obj.py <in.obj> <out.png>", file=sys.stderr)
        return 2
    vertices, faces, is_body = load_obj(sys.argv[1])
    colors = face_colors(vertices, faces, is_body)
    # World is Y-up; matplotlib's 3D axes are Z-up. (x, y, z) -> (x, -z, y) is a proper rotation.
    plot = np.stack([vertices[:, 0], -vertices[:, 2], vertices[:, 1]], axis=1)
    lo, hi = plot.min(axis=0), plot.max(axis=0)
    centre, half = (lo + hi) / 2, (hi - lo).max() / 2 * 1.05

    fig = plt.figure(figsize=(16, 10), dpi=100)
    for k, (azim, title) in enumerate([(-90, "front (+Z)"), (0, "side (+X)")]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d", proj_type="ortho")
        camera = np.array([np.cos(np.radians(azim)), np.sin(np.radians(azim)), 0.0])
        shifted = plot[faces] - np.where(is_body[:, None, None], 0.01 * camera, 0.0)
        ax.add_collection3d(Poly3DCollection(shifted, facecolors=colors, edgecolors="none"))
        for axis, c in zip("xyz", centre):
            getattr(ax, f"set_{axis}lim")(c - half, c + half)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=0, azim=azim)
        ax.set_axis_off()
        ax.set_title(title)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.95, wspace=0)
    fig.savefig(sys.argv[2], dpi=100)
    return 0


if __name__ == "__main__":
    sys.exit(main())
