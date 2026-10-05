"""Mannequin styling: a smooth egg for the head and mitten-like feet.

MakeHuman's body has a face, ears and separate toes. A white display mannequin has none
of those, so this module reshapes two skinning-weight regions of the built body:

* `make_egg_head` blends every head vertex toward an egg fitted to the skull
  (`fit_egg`) and relaxes the region so the nose, ears and eye sockets melt away;
* `smooth_toes` Laplacian-smooths the toe region while holding the sole plane.

Vertices are float64 (N, 3) in metres, Y up. Quads are int arrays (M, 4), counter-clockwise
seen from outside. Nothing here is random, so equal inputs give byte-identical outputs.
Python 3.13 and numpy only.
"""
from dataclasses import dataclass

import numpy as np

from makehuman import Rig, descendants

_PIN_WEIGHT = 0.999          # vertices at least this weighted sit on the egg
_HEAD_ROUNDS = 30
_RELAX_STEP = 0.5            # Laplacian step is _RELAX_STEP * weight
_SOLE_BAND = 0.003           # metres above the lowest toe vertex that count as the sole
_SECTION_ABOVE_EYES = 0.03   # the rig has no ear bones; above the ears is where the skull is bare
_SECTION_HALF_THICKNESS = 0.005
_CHIN_HALF_WIDTH = 0.02
_RAY_ITERATIONS = 60


# --------------------------------------------------------------------------- regions

def region_weights(rig: Rig, roots: list[str], n: int,
                   exclude: frozenset[str] = frozenset()) -> np.ndarray:
    """(n,) skinning weight of a body region: the weights of every bone under `roots`
    (each root included) minus the bones in `exclude`, summed per vertex and clipped to [0, 1].
    Bones without skin weights contribute nothing; `exclude` removes only the named bones, not
    their descendants. `n` is the body's vertex count: MakeHuman's rig also weights the helper
    geometry (eyes, teeth, tongue), which comes after the body's vertices in base.obj, so
    weights on vertices >= n are ignored."""
    bones: dict[str, None] = {}             # an ordered set: descendants() order keeps the float sums reproducible
    for root in roots:
        for bone in descendants(rig, root):
            if bone not in exclude:
                bones[bone] = None
    total = np.zeros(n, dtype=np.float64)
    for bone in bones:
        if bone not in rig.weights:
            continue
        indices, weights = rig.weights[bone]
        inside = indices < n
        np.add.at(total, indices[inside], weights[inside])
    return np.clip(total, 0.0, 1.0)


# --------------------------------------------------------------------------- egg

@dataclass
class Egg:
    """An ellipsoid with semi-axes (ax, ay, az) whose lower half narrows toward the chin:
    below the centre the horizontal radii shrink linearly to `taper` times their value at the
    bottom pole."""
    centre: np.ndarray                       # (3,)
    radii: np.ndarray                        # (3,) = (ax, ay, az)
    taper: float = 0.85


def egg_value(egg: Egg, p: np.ndarray) -> np.ndarray:
    """Implicit egg function for a point or an (..., 3) array of points: 1 on the surface,
    below 1 inside, above 1 outside. With q = p - centre and k = 1 for q_y >= 0, else
    1 - (1 - taper) * min(1, -q_y / ay), it is (q_x/(ax k))^2 + (q_y/ay)^2 + (q_z/(az k))^2."""
    q = np.asarray(p, dtype=np.float64) - egg.centre
    ax, ay, az = egg.radii
    qy = q[..., 1]
    k = np.where(qy >= 0.0, 1.0, 1.0 - (1.0 - egg.taper) * np.minimum(1.0, -qy / ay))
    return (q[..., 0] / (ax * k)) ** 2 + (qy / ay) ** 2 + (q[..., 2] / (az * k)) ** 2


def project_to_egg(egg: Egg, p: np.ndarray) -> np.ndarray:
    """The point on the ray from the egg's centre through each point of `p` (shape (3,) or
    (..., 3)) where `egg_value` is 1, found by bisection on the distance along the ray.
    A point exactly at the centre has no direction and is sent to the top of the egg."""
    q = np.asarray(p, dtype=np.float64) - egg.centre
    length = np.linalg.norm(q, axis=-1, keepdims=True)
    safe = np.where(length > 0.0, length, 1.0)
    direction = np.where(length > 0.0, q / safe, np.array([0.0, 1.0, 0.0]))
    lo = np.zeros_like(length)
    hi = np.full_like(length, 4.0 * float(np.max(egg.radii)))
    for _ in range(_RAY_ITERATIONS):
        mid = 0.5 * (lo + hi)
        inside = egg_value(egg, egg.centre + mid * direction)[..., np.newaxis] < 1.0
        lo = np.where(inside, mid, lo)
        hi = np.where(inside, hi, mid)
    return egg.centre + 0.5 * (lo + hi) * direction


def fit_egg(vertices: np.ndarray, head_w: np.ndarray, eye_y: float) -> Egg:
    """Fit an egg to the skull: the vertices with head weight >= 0.999.

    Height and centre height come from the chin (lowest point within 2 cm of the midline) to
    the top of the head. Width and depth are measured on a thin band 3 cm above eye level, not
    at it: the rig has no ear bones, so the ears sit in the head region and measuring above
    them is how they are left out. That section is smaller than the egg's equator, so it is
    scaled back by 1 / sqrt(1 - t^2), t being the band's height as a fraction of the half-height.
    """
    skull = np.asarray(vertices, dtype=np.float64)[np.asarray(head_w) >= _PIN_WEIGHT]
    if len(skull) == 0:
        raise ValueError("fit_egg: no vertex has head weight >= 0.999")
    y_top = skull[:, 1].max()
    midline = skull[np.abs(skull[:, 0]) <= _CHIN_HALF_WIDTH]
    if len(midline) == 0:
        raise ValueError("fit_egg: no head vertex within 2 cm of the midline")
    y_chin = midline[:, 1].min()
    ay = (y_top - y_chin) / 2.0
    c_y = (y_top + y_chin) / 2.0

    y_band = eye_y + _SECTION_ABOVE_EYES
    band = skull[np.abs(skull[:, 1] - y_band) <= _SECTION_HALF_THICKNESS]
    if len(band) == 0:
        raise ValueError(f"fit_egg: no head vertex within 5 mm of y = {y_band:.4f} (eye level + 3 cm)")
    half_width = (band[:, 0].max() - band[:, 0].min()) / 2.0
    half_depth = (band[:, 2].max() - band[:, 2].min()) / 2.0
    c_z = (band[:, 2].max() + band[:, 2].min()) / 2.0

    t = float(np.clip((y_band - c_y) / ay, -0.9, 0.9))
    scale = 1.0 / np.sqrt(1.0 - t * t)
    return Egg(centre=np.array([0.0, c_y, c_z]),
               radii=np.array([half_width * scale, ay, half_depth * scale]))


# --------------------------------------------------------------------------- relaxation

def _quad_normals(vertices: np.ndarray, quads: np.ndarray) -> np.ndarray:
    """(M, 3) quad normals (unnormalised): the cross product of the diagonals."""
    p = vertices[quads]
    return np.cross(p[:, 2] - p[:, 0], p[:, 3] - p[:, 1])


def _revert_flips(before: np.ndarray, after: np.ndarray, quads: np.ndarray) -> np.ndarray:
    """`after`, except that every vertex of a quad whose normal no longer agrees with its
    `before` normal (dot product <= 0) is put back where it was. Reverting can flip a
    neighbouring quad that shares a moved vertex, so this repeats until nothing flips; it
    ends because a reverted vertex never moves again."""
    reference = _quad_normals(before, quads)
    result = after.copy()
    while True:
        flipped = (reference * _quad_normals(result, quads)).sum(axis=1) <= 0.0
        if not flipped.any():
            return result
        ids = np.unique(quads[flipped])
        moved = ids[np.any(result[ids] != before[ids], axis=1)]
        if len(moved) == 0:
            return result            # flipped already before the round; nothing left to undo
        result[moved] = before[moved]


class _Rings:
    """1-ring neighbourhoods: two vertices are neighbours when they share a quad edge."""

    def __init__(self, quads: np.ndarray, n: int):
        sides = np.concatenate([quads[:, [0, 1]], quads[:, [1, 2]], quads[:, [2, 3]], quads[:, [3, 0]]])
        self.edges = np.unique(np.sort(sides, axis=1), axis=0)
        self.degree = np.bincount(self.edges.ravel(), minlength=n)

    def mean(self, v: np.ndarray) -> np.ndarray:
        """Per-vertex mean of its neighbours' positions (a vertex with none keeps its own)."""
        total = np.zeros_like(v)
        np.add.at(total, self.edges[:, 0], v[self.edges[:, 1]])
        np.add.at(total, self.edges[:, 1], v[self.edges[:, 0]])
        linked = self.degree > 0
        mean = v.copy()
        mean[linked] = total[linked] / self.degree[linked, np.newaxis]
        return mean


def make_egg_head(vertices: np.ndarray, quads: np.ndarray, head_w: np.ndarray, egg: Egg) -> np.ndarray:
    """New vertex positions with the head turned into the egg; the input is not modified.

    Each vertex first moves to x + w (P(x) - x), P being the projection onto the egg. Then 30
    rounds of uniform Laplacian relaxation (step 0.5 w toward the mean of the 1-ring) run over
    the region, each followed by projecting the w >= 0.999 vertices back onto the egg, so the
    core stays on the egg while the mesh spreads evenly over it and the neck blend (0 < w < 1)
    settles. A round that would flip a quad's normal against its pre-round normal is undone
    for the vertices involved. Vertices with w = 0 are never touched."""
    w = np.clip(np.asarray(head_w, dtype=np.float64), 0.0, 1.0)
    v = np.array(vertices, dtype=np.float64)
    moving = w > 0.0
    pinned = w >= _PIN_WEIGHT
    v[moving] += w[moving, np.newaxis] * (project_to_egg(egg, v[moving]) - v[moving])

    rings = _Rings(quads, len(v))
    step = (_RELAX_STEP * w)[moving, np.newaxis]
    for _ in range(_HEAD_ROUNDS):
        relaxed = v.copy()
        relaxed[moving] += step * (rings.mean(v)[moving] - v[moving])
        relaxed[pinned] = project_to_egg(egg, relaxed[pinned])
        v = _revert_flips(v, relaxed, quads)
    return v


def smooth_toes(vertices: np.ndarray, quads: np.ndarray, toe_w: np.ndarray, rounds: int = 40) -> np.ndarray:
    """New vertex positions with the toes smoothed into one mitten; the input is not modified.

    Uniform Laplacian relaxation, step 0.5 w toward the mean of the 1-ring, `rounds` times.
    Vertices that start within 3 mm of the lowest vertex of the toe region (w > 0) are the
    sole: they slide in x and z only, so the foot stays flat on the floor. Vertices with
    w = 0 are never touched."""
    w = np.clip(np.asarray(toe_w, dtype=np.float64), 0.0, 1.0)
    v = np.array(vertices, dtype=np.float64)
    moving = w > 0.0
    if not moving.any():
        return v
    sole = moving & (v[:, 1] <= v[moving, 1].min() + _SOLE_BAND)
    rings = _Rings(quads, len(v))
    step = (_RELAX_STEP * w)[moving, np.newaxis]
    y_sole = v[sole, 1].copy()
    for _ in range(rounds):
        v[moving] += step * (rings.mean(v)[moving] - v[moving])
        v[sole, 1] = y_sole
    return v
