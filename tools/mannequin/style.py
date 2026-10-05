"""Mannequin styling: a smooth egg for the head and mitten-like feet.

MakeHuman's body has a face, ears and separate toes. A white display mannequin has none
of those, so this module reshapes two skinning-weight regions of the built body:

* `make_egg_head` blends every head vertex toward an egg fitted to the skull
  (`fit_egg`) and relaxes the region so the nose and ears melt away; the eye sockets and
  mouth, which are cavities in the body mesh, are tucked just under the egg surface
  (`cavity_vertices`);
* `smooth_toes` Laplacian-smooths the toe region while holding the sole plane.

Vertices are float64 (N, 3) in metres, Y up. Quads are int arrays (M, 4), counter-clockwise
seen from outside. Nothing here is random, so equal inputs give byte-identical outputs.
Python 3.13 and numpy only.
"""
from dataclasses import dataclass

import numpy as np

from makehuman import Rig, descendants

_PIN_WEIGHT = 0.999          # vertices at least this weighted sit on the egg (or just under it)
_CAVITY_DEPTH = 0.002        # metres: cavity vertices sit this far inside the egg surface
_HEAD_ROUNDS = 30
_RELAX_STEP = 0.5            # Laplacian step is _RELAX_STEP * weight
_SOLE_BAND = 0.0022          # metres above the lowest toe vertex that count as the sole (held at their height)
_SECTION_ABOVE_EYES = 0.03   # the rig has no ear bones; above the ears is where the skull is bare
_SECTION_HALF_THICKNESS = 0.005
_CHIN_HALF_WIDTH = 0.02
_RAY_ITERATIONS = 60
_AREA_FLOOR = 3e-9           # m^2: no corner triangle of a head quad ends up smaller than this
_UNTANGLE_TARGET = 2.0 * _AREA_FLOOR   # the untangler pushes corners up to this, so none ends on the floor
_UNTANGLE_ROUNDS = 300       # descent steps before a head that is still folded is an error
_UNTANGLE_STEP = 2e-4        # metres: the largest vertex move of one full descent step


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


def _facing(vertices: np.ndarray, quads: np.ndarray, centre: np.ndarray) -> np.ndarray:
    """(M,) quad normal . (quad centroid - centre): positive when a quad faces away from `centre`."""
    centroids = vertices[quads].mean(axis=1)
    return (_quad_normals(vertices, quads) * (centroids - centre)).sum(axis=1)


def _corners(vertices: np.ndarray, quads: np.ndarray, centre: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The four corner triangles of every quad: corner j is (p[j-1], p[j], p[j+1]), which is also
    what `triangulate` cuts a quad into, whichever diagonal it takes. Returns (M, 4) arrays:
    the triangle's normal . (centroid - centre), positive when it faces away from `centre`, and its area."""
    p = vertices[quads]
    prev, nxt = np.roll(p, 1, axis=1), np.roll(p, -1, axis=1)
    normal = np.cross(p - prev, nxt - p)
    centroid = (prev + p + nxt) / 3.0
    return (normal * (centroid - centre)).sum(axis=2), 0.5 * np.linalg.norm(normal, axis=2)


def _revert_flips(before: np.ndarray, after: np.ndarray, quads: np.ndarray,
                  centre: np.ndarray) -> np.ndarray:
    """`after`, except that every moved vertex of a quad that went wrong in the move is put back
    where it was. A quad goes wrong when

    * it faced away from `centre` in `before` and faces it in `after`, as a quad or through any
      one of its four corner triangles (a concave quad can face outward as a whole and still have
      one corner that faces in, which is the triangle `triangulate` may then cut out of it); or
    * a corner triangle's area is below `_AREA_FLOOR` and smaller than it was (the Laplacian step
      contracts two neighbouring vertices with the same 1-ring onto each other).

    Quads and corners that were already inward, or already tiny, may change freely as long as they
    do not get worse. Reverting can turn a neighbouring quad bad (it shares a moved vertex), so
    this repeats until none is; it ends because a reverted vertex never moves again. Only quads
    with a moved vertex can go wrong, so only those are looked at."""
    result = after.copy()
    moved = np.any(after != before, axis=1)
    quads = quads[moved[quads].any(axis=1)]
    if len(quads) == 0:
        return result
    quad_out = _facing(before, quads, centre) > 0.0
    corner_out_before, area_before = _corners(before, quads, centre)
    corner_out = corner_out_before > 0.0
    while True:
        corner_out_now, area_now = _corners(result, quads, centre)
        turned = quad_out & (_facing(result, quads, centre) <= 0.0)
        turned |= (corner_out & (corner_out_now <= 0.0)).any(axis=1)
        turned |= ((area_now < _AREA_FLOOR) & (area_now < area_before)).any(axis=1)
        if not turned.any():
            return result
        ids = np.unique(quads[turned])
        back = ids[np.any(result[ids] != before[ids], axis=1)]
        if len(back) == 0:
            return result            # unreachable for protected quads; stops a loop on bad input
        result[back] = before[back]


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


def cavity_vertices(vertices: np.ndarray, quads: np.ndarray, head_w: np.ndarray, egg: Egg) -> np.ndarray:
    """(N,) bool: the head vertices (w >= 0.999) that face into the head, such as the eye sockets
    and the mouth, which MakeHuman models as cavities in the body mesh. A vertex is one when its
    normal (the normalised sum of its quads' normals) points toward the egg centre:
    n . (p - c) < 0. Call it on the vertices before they are moved onto the egg."""
    p = np.asarray(vertices, dtype=np.float64)
    normal = np.zeros_like(p)
    face_normals = _quad_normals(p, quads)
    for corner in range(4):
        np.add.at(normal, quads[:, corner], face_normals)
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    normal = np.divide(normal, length, out=np.zeros_like(normal), where=length > 0.0)
    return (np.asarray(head_w) >= _PIN_WEIGHT) & ((normal * (p - egg.centre)).sum(axis=1) < 0.0)


def _pin_targets(egg: Egg, points: np.ndarray, cavity: np.ndarray) -> np.ndarray:
    """Where pinned vertices live: on the egg surface along their ray from the centre, or for
    cavity vertices `_CAVITY_DEPTH` inside it."""
    surface = project_to_egg(egg, points)
    out = surface - egg.centre
    tucked = surface - _CAVITY_DEPTH * out / np.linalg.norm(out, axis=1, keepdims=True)
    return np.where(cavity[:, np.newaxis], tucked, surface)


def _corner_sizes(vertices: np.ndarray, quads: np.ndarray, centre: np.ndarray,
                  on_egg: np.ndarray) -> tuple[np.ndarray, tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """How big each corner triangle of `quads` is, and how that changes when its vertices move.

    Corner j of a quad is (a, p, b) = (p[j-1], p[j], p[j+1]) with normal n = (p - a) x (b - p).
    Where `on_egg` is true (all three vertices on the egg surface) the size is the signed area
    seen from outside, d . n / 2 with d the unit direction from `centre` to the centroid: negative
    when the triangle is folded over. Elsewhere it is the plain area, d being the triangle's own
    unit normal. Returns the (M, 4) sizes and their gradients with respect to a, p and b (each
    (M, 4, 3), with d held fixed)."""
    p = vertices[quads]
    a, b = np.roll(p, 1, axis=1), np.roll(p, -1, axis=1)
    normal = np.cross(p - a, b - p)
    direction = np.where(on_egg[..., np.newaxis], (a + p + b) / 3.0 - centre, normal)
    length = np.linalg.norm(direction, axis=2, keepdims=True)
    direction = np.divide(direction, length, out=np.zeros_like(direction), where=length > 0.0)
    size = 0.5 * (normal * direction).sum(axis=2)
    return size, (0.5 * np.cross(p - b, direction),
                  0.5 * np.cross(b - a, direction),
                  0.5 * np.cross(direction, p - a))


def _untangle(v: np.ndarray, quads: np.ndarray, rings: _Rings, egg: Egg, moving: np.ndarray,
              pinned: np.ndarray, cavity: np.ndarray) -> np.ndarray:
    """`v` with every folded or collapsed corner triangle of the head turned outward and made at
    least `_AREA_FLOOR` big, or RuntimeError if the descent cannot manage it.

    A corner is bad when its three vertices are on the egg surface (pinned, not cavity) and it
    faces inward or is smaller than the floor, or when it is any other corner of a quad of the
    head and is smaller than the floor. Relaxation cannot cure these: a rim of the nostril that
    the projection folded has a 1-ring that winds twice around its vertex, and the guard, which
    refuses any move that turns another quad inward, holds the vertices where they are. So the
    pinned vertices of the bad quads and their 1-rings go down the gradient of
    sum(max(0, target - size)^2) over the corners they touch, re-projected onto the egg after
    every step, with the step halved until the penalty falls. Every vertex moves at once, so a
    mirrored fold is cured identically."""
    surface = pinned & ~cavity
    on_egg = surface[np.roll(quads, 1, axis=1)] & surface[quads] & surface[np.roll(quads, -1, axis=1)]
    head = np.flatnonzero(moving[quads].any(axis=1))

    def folded(points: np.ndarray, which: np.ndarray) -> np.ndarray:
        size, _ = _corner_sizes(points, quads[which], egg.centre, on_egg[which])
        return (size < _AREA_FLOOR).any(axis=1)

    bad = head[folded(v, head)]
    if len(bad) == 0:
        return v
    free = np.zeros(len(v), dtype=bool)
    free[quads[bad]] = True
    near = free.copy()
    near[rings.edges[free[rings.edges[:, 0]], 1]] = True
    near[rings.edges[free[rings.edges[:, 1]], 0]] = True
    free = near & pinned
    touched = np.flatnonzero(free[quads].any(axis=1))      # every quad the descent can change
    local, local_on_egg = quads[touched], on_egg[touched]

    def penalty(points: np.ndarray) -> float:
        size, _ = _corner_sizes(points, local, egg.centre, local_on_egg)
        return float((np.maximum(0.0, _UNTANGLE_TARGET - size) ** 2).sum())

    def gradient(points: np.ndarray) -> np.ndarray:
        size, pieces = _corner_sizes(points, local, egg.centre, local_on_egg)
        lack = np.maximum(0.0, _UNTANGLE_TARGET - size)[..., np.newaxis]
        grad = np.zeros_like(points)
        for ids, piece in zip((np.roll(local, 1, axis=1), local, np.roll(local, -1, axis=1)), pieces):
            np.add.at(grad, ids.ravel(), (-2.0 * lack * piece).reshape(-1, 3))
        grad[~free] = 0.0
        return grad

    current = penalty(v)
    scale = 1.0
    for _ in range(_UNTANGLE_ROUNDS):
        if not folded(v, touched).any():
            break
        grad = gradient(v)
        largest = np.abs(grad).max()
        if largest == 0.0:
            break
        for _ in range(40):
            moved = v - (scale * _UNTANGLE_STEP / largest) * grad
            moved[free] = _pin_targets(egg, moved[free], cavity[free])
            trial = penalty(moved)
            if trial < current:
                break
            scale *= 0.5
        else:
            break                                          # no step lowers the penalty: a local minimum
        v, current, scale = moved, trial, min(1.0, 2.0 * scale)
    left = int(folded(v, head).sum())
    if left:
        raise RuntimeError(f"make_egg_head: {left} head quads are still folded or collapsed after "
                           f"{_UNTANGLE_ROUNDS} untangling steps")
    return v


def make_egg_head(vertices: np.ndarray, quads: np.ndarray, head_w: np.ndarray, egg: Egg) -> np.ndarray:
    """New vertex positions with the head turned into the egg; the input is not modified.

    Each vertex first moves to x + w (P(x) - x), P being the projection onto the egg; the
    pinned vertices (w >= 0.999) go all the way. Cavity vertices (`cavity_vertices` of the
    original mesh: eye sockets, mouth) are pinned 2 mm inside the egg instead of on it, so they
    do not fold over the face as a second skin. Then 30 rounds of uniform Laplacian relaxation
    (step 0.5 w toward the mean of the 1-ring) run over the region, each followed by putting the
    pinned vertices back on their ray's target, so the core stays on the egg while the mesh
    spreads evenly over it and the neck blend (0 < w < 1) settles. `_revert_flips` undoes the
    part of a round that would turn a quad that faced outward (normal . (centroid - egg centre)
    > 0) inward, through the quad or any of its corner triangles, or shrink a corner triangle
    below `_AREA_FLOOR`; quads that face inward, such as those folded by the first projection,
    may turn outward. What that leaves folded or collapsed on the egg surface (the rims of the
    nostrils) `_untangle` fixes, or it raises. Vertices with w = 0 are never touched."""
    w = np.clip(np.asarray(head_w, dtype=np.float64), 0.0, 1.0)
    v = np.array(vertices, dtype=np.float64)
    cavity = cavity_vertices(v, quads, w, egg)
    moving = w > 0.0
    pinned = w >= _PIN_WEIGHT
    blend = np.where(pinned, 1.0, w)[moving, np.newaxis]
    target = np.where(pinned[moving, np.newaxis], _pin_targets(egg, v[moving], cavity[moving]),
                      project_to_egg(egg, v[moving]))
    v[moving] += blend * (target - v[moving])

    rings = _Rings(quads, len(v))
    step = (_RELAX_STEP * w)[moving, np.newaxis]
    for _ in range(_HEAD_ROUNDS):
        relaxed = v.copy()
        relaxed[moving] += step * (rings.mean(v)[moving] - v[moving])
        relaxed[pinned] = _pin_targets(egg, relaxed[pinned], cavity[pinned])
        v = _revert_flips(v, relaxed, quads, egg.centre)
    return _untangle(v, quads, rings, egg, moving, pinned, cavity)


def smooth_toes(vertices: np.ndarray, quads: np.ndarray, toe_w: np.ndarray, rounds: int = 40) -> np.ndarray:
    """New vertex positions with the toes smoothed into one mitten; the input is not modified.

    Uniform Laplacian relaxation, step 0.5 w toward the mean of the 1-ring, `rounds` times.
    Vertices that start within 2.2 mm of the lowest vertex of the toe region (w > 0) are the
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
