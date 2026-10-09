"""Mannequin styling: a smooth egg for the head and one rounded forefoot per foot.

MakeHuman's body has a face, ears and separate toes. A white display mannequin has none of those,
so this module reshapes two regions of the built body:

* `make_egg_head` puts the head on an egg fitted to the skull (`fit_egg`), the neck blending into
  it. The face's cavities and overhangs (eye sockets, mouth, nostrils, the backs of the ears) cannot
  simply be projected onto the egg: their sheets would land on top of one another, back to front.
  So the head is laid out afresh (`_lay_out`): every vertex gets a new direction from the egg's
  centre such that the directions form a fold-free disk, and is then put on its ray. A surface whose
  vertices sit on the rays of a fold-free layout of directions cannot pass through itself and faces
  away from the centre everywhere.
* `round_forefeet` does the same for the front of each foot, on a smooth convex shell fitted round
  the toes with a flat sole, so the toes and the webbing between them become one rounded forefoot.

Vertices are float64 (N, 3) in metres, Y up. Quads are int arrays (M, 4), counter-clockwise seen
from outside. Nothing here is random, so equal inputs give byte-identical outputs. Python 3.13 and
numpy only.
"""
from dataclasses import dataclass

import numpy as np

from makehuman import Rig, descendants

_PIN_WEIGHT = 0.999          # head vertices at least this weighted sit on the egg
_CORE_WEIGHT = 0.1           # head vertices at least this weighted are laid out afresh (below: the shoulders)
_HEAD_ROUNDS = 100           # guarded relaxation rounds on the egg after the layout
_RELAX_STEP = 0.5            # Laplacian step is _RELAX_STEP * weight
_SECTION_ABOVE_EYES = 0.03   # the rig has no ear bones; above the ears is where the skull is bare
_SECTION_HALF_THICKNESS = 0.005
_CHIN_HALF_WIDTH = 0.02
_RAY_ITERATIONS = 60
_AREA_FLOOR = 3e-9           # m^2: the relaxation never squashes a corner triangle below this
_STRETCH_ROUNDS = 12         # layout re-weightings that even out the area the vertices get on the surface
_SOLVE_TOLERANCE = 1e-13     # relative residual of the layout's linear solves
_SOLVE_ITERATIONS = 5000
_UNFOLD_ROUNDS = 2000        # descent steps of the chart untangler
_MIRROR_DIGITS = 7           # vertices are mirror partners when they match to 0.1 micron

_FOREFOOT_RINGS = 4          # quad rings from the forefoot's edge loop to the first toe vertex, at least
_FOREFOOT_ROUNDS = 30        # guarded relaxation rounds on the forefoot shell
_FOREFOOT_BEHIND = 0.05      # (stretched) metres behind the edge loop where the forefoot layout is seen from
_FOREFOOT_SINK = 0.02        # (stretched) metres the shell's footprint is sunk below the floor before the floor cuts it
_FOREFOOT_SMOOTHING = 0.15   # radians: the angular blur that rounds the shell's edges
_SHELL_NORMALS = 3000        # directions of the support planes that describe a convex shell
_SHELL_SAMPLES = 6000
_SHELL_OUTPUT_NORMALS = 10000
_WEIGHT_ITERATIONS = 400     # Jacobi sweeps of the forefoot's harmonic blend weight


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
    below the centre the horizontal radii shrink, with the square of the depth, to `taper` times
    their value at the bottom pole (so the side has no kink at the centre)."""
    centre: np.ndarray                       # (3,)
    radii: np.ndarray                        # (3,) = (ax, ay, az)
    taper: float = 0.85


def egg_value(egg: Egg, p: np.ndarray) -> np.ndarray:
    """Implicit egg function for a point or an (..., 3) array of points: 1 on the surface,
    below 1 inside, above 1 outside. With q = p - centre, s = min(1, max(0, -q_y / ay)) and
    k = 1 - (1 - taper) s^2, it is (q_x/(ax k))^2 + (q_y/ay)^2 + (q_z/(az k))^2: above the centre k is 1,
    at the bottom pole it is `taper`, and its slope is 0 at the centre."""
    q = np.asarray(p, dtype=np.float64) - egg.centre
    ax, ay, az = egg.radii
    qy = q[..., 1]
    s = np.minimum(1.0, np.maximum(0.0, -qy / ay))
    k = 1.0 - (1.0 - egg.taper) * s * s
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




# --------------------------------------------------------------------------- mirror symmetry

def _mirror_partners(vertices: np.ndarray, which: np.ndarray) -> np.ndarray | None:
    """(N,) index of each vertex's mirror image (x -> -x), matched to _MIRROR_DIGITS decimals, -1 for
    vertices outside `which`; None when a vertex in `which` has no partner (the input is not symmetric)."""
    keys = np.round(np.asarray(vertices, dtype=np.float64), _MIRROR_DIGITS) + 0.0
    index = {key: i for i, key in enumerate(map(tuple, keys.tolist()))}
    if len(index) != len(keys):
        return None                                   # coincident vertices: no unique partners
    mirrored = (keys * np.array([-1.0, 1.0, 1.0])) + 0.0
    partner = np.full(len(keys), -1, dtype=np.int64)
    for i in np.flatnonzero(which).tolist():
        partner[i] = index.get(tuple(mirrored[i].tolist()), -1)
    if not (partner[which] >= 0).all() or not (partner[partner[which]] == np.flatnonzero(which)).all():
        return None
    return partner


def _symmetrize(v: np.ndarray, partner: np.ndarray, which: np.ndarray) -> np.ndarray:
    """`v` with each vertex in `which` (and so its partner) moved to the mean of itself and its partner's
    mirror image: the pair ends exactly mirrored, and a vertex that is its own partner ends at x = 0."""
    ids = np.flatnonzero(which)
    result = v.copy()
    result[ids] = 0.5 * (v[ids] + v[partner[ids]] * np.array([-1.0, 1.0, 1.0]))
    return result


# --------------------------------------------------------------------------- fold-free layout of directions

def _cross2(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _boundary_loop(region: np.ndarray) -> np.ndarray | None:
    """The vertices round the boundary of a disk of quads, in order (the quads on its left), None
    when the quads close up with no boundary, RuntimeError when the boundary is not one simple loop."""
    sides = np.concatenate([region[:, [0, 1]], region[:, [1, 2]], region[:, [2, 3]], region[:, [3, 0]]])
    _, inverse, count = np.unique(np.sort(sides, axis=1), axis=0, return_inverse=True, return_counts=True)
    outer = sides[count[inverse.ravel()] == 1]
    if len(outer) == 0:
        return None
    following = {int(a): int(b) for a, b in outer}
    if len(following) != len(outer):
        raise RuntimeError("layout: the region's boundary touches itself")
    start = min(following)
    loop = [start]
    while following[loop[-1]] != start:
        loop.append(following[loop[-1]])
    if len(loop) != len(outer):
        raise RuntimeError(f"layout: the region's boundary is more than one loop ({len(outer) - len(loop)} "
                           f"of its {len(outer)} edges are off the first): it is not a disk")
    return np.array(loop, dtype=np.int64)


def _ordered_rings(quads: np.ndarray, ids: np.ndarray) -> list[list[int]]:
    """For each vertex in `ids`, its edge neighbours in order round it (counter-clockwise from outside)."""
    turn: dict[int, dict[int, int]] = {}
    for q in quads.tolist():
        for j in range(4):
            turn.setdefault(q[j], {})[q[(j + 1) % 4]] = q[j - 1]
    rings = []
    for v in ids.tolist():
        step = turn[v]
        start = min(step)
        ring = [start]
        while step.get(ring[-1], start) != start and len(ring) <= len(step):
            ring.append(step[ring[-1]])
        if len(ring) != len(step) or step.get(ring[-1]) != start:
            raise RuntimeError(f"layout: the quads round vertex {v} do not close round it")
        rings.append(ring)
    return rings


def _kernel_point(polygon: np.ndarray, on_axis: bool) -> tuple[np.ndarray, float]:
    """A point deep inside the kernel of a counter-clockwise polygon (from where it sees every edge
    from the inside) and its distance to the nearest edge line, negative when the kernel is empty: a
    refined grid search for the largest smallest edge-line distance. With `on_axis` only x = 0 is searched."""
    a = polygon
    d = np.roll(polygon, -1, axis=0) - polygon
    length = np.linalg.norm(d, axis=1)

    def margin(c: np.ndarray) -> np.ndarray:
        return (_cross2(d[np.newaxis], c[:, np.newaxis, :] - a[np.newaxis]) / length).min(axis=1)

    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    if on_axis:
        lo[0] = hi[0] = 0.0
    grid = np.linspace(0.0, 1.0, 33)
    best, score = (lo + hi) / 2.0, -np.inf
    for _ in range(40):
        gx, gy = np.meshgrid(lo[0] + grid * (hi[0] - lo[0]), lo[1] + grid * (hi[1] - lo[1]), indexing="ij")
        candidates = np.stack([gx.ravel(), gy.ravel()], axis=1)
        scores = margin(candidates)
        k = int(np.argmax(scores))
        best, score = candidates[k], float(scores[k])
        span = (hi - lo) / 8.0
        lo, hi = best - span, best + span
    return best, score


def _bicgstab(matvec, b: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Solve matvec(x) = b (b, x of shape (n, 2)) by BiCGSTAB, with numpy reductions only, so the
    result does not depend on BLAS threading."""
    r = b - matvec(x)
    r_hat = r.copy()
    rho = alpha = omega = 1.0
    v = np.zeros_like(b)
    p = np.zeros_like(b)
    norm_b = np.sqrt((b * b).sum())
    for _ in range(_SOLVE_ITERATIONS):
        rho_next = (r_hat * r).sum()
        p = r + (rho_next / rho) * (alpha / omega) * (p - omega * v)
        v = matvec(p)
        alpha = rho_next / (r_hat * v).sum()
        s = r - alpha * v
        t = matvec(s)
        omega = (t * s).sum() / (t * t).sum()
        x = x + alpha * p + omega * s
        r = s - omega * t
        rho = rho_next
        if np.sqrt((r * r).sum()) <= _SOLVE_TOLERANCE * norm_b:
            return x
    raise RuntimeError(f"layout: the linear solve did not converge in {_SOLVE_ITERATIONS} iterations")


def _chart_corners(z: np.ndarray, quads: np.ndarray):
    """Signed areas of the corner triangles (a, p, b) = (p[j-1], p[j], p[j+1]) of every quad in the plane,
    and their gradients with respect to a, p and b."""
    p = z[quads]
    a, b = np.roll(p, 1, axis=1), np.roll(p, -1, axis=1)
    area = 0.5 * _cross2(p - a, b - p)
    return area, (0.5 * np.stack([p[..., 1] - b[..., 1], b[..., 0] - p[..., 0]], axis=-1),
                  0.5 * np.stack([b[..., 1] - a[..., 1], a[..., 0] - b[..., 0]], axis=-1),
                  0.5 * np.stack([a[..., 1] - p[..., 1], p[..., 0] - a[..., 0]], axis=-1))


def _unfold(z: np.ndarray, quads: np.ndarray, movable: np.ndarray) -> np.ndarray:
    """Chart points with every corner triangle of `quads` at a `movable` vertex turned positive and at
    least 1/1000 of the median corner area (a corner at a fixed boundary vertex is the boundary's own
    shape, which may be concave), moving only `movable` points near the bad ones (3 rings round them): a gradient descent on
    sum(max(0, floor - area)^2), the step halved until the penalty falls. RuntimeError if that is not
    reached."""
    mobile = movable[quads]
    area, _ = _chart_corners(z, quads)
    floor = 1e-3 * float(np.median(np.abs(area)))
    bad = ((area < floor) & mobile).any(axis=1)
    if not bad.any():
        return z
    free = np.zeros(len(z), dtype=bool)
    free[quads[bad]] = True
    for _ in range(3):
        free[quads[free[quads].any(axis=1)]] = True
    free &= movable
    touched = free[quads].any(axis=1)
    local, local_mobile = quads[touched], mobile[touched]
    scale = np.sqrt(float(np.median(np.abs(area))))

    def penalty(points: np.ndarray) -> float:
        a, _ = _chart_corners(points, local)
        return float((np.where(local_mobile, np.maximum(0.0, floor - a), 0.0) ** 2).sum())

    current = penalty(z)
    step = 1.0
    for _ in range(_UNFOLD_ROUNDS):
        if current == 0.0:
            break
        a, pieces = _chart_corners(z, local)
        lack = np.where(local_mobile, np.maximum(0.0, floor - a), 0.0)[..., np.newaxis]
        grad = np.zeros_like(z)
        for ids, piece in zip((np.roll(local, 1, axis=1), local, np.roll(local, -1, axis=1)), pieces):
            np.add.at(grad, ids.ravel(), (-2.0 * lack * piece).reshape(-1, 2))
        grad[~free] = 0.0
        largest = np.abs(grad).max()
        if largest == 0.0:
            break
        for _ in range(50):
            trial = z - (step * 0.1 * scale / largest) * grad
            value = penalty(trial)
            if value < current:
                break
            step *= 0.5
        else:
            break
        z, current, step = trial, value, min(1.0, 2.0 * step)
    area, _ = _chart_corners(z, quads)
    folded = ((area <= 0.0) & mobile).any(axis=1)
    if folded.any():
        raise RuntimeError(f"layout: {int(folded.sum())} quads are still folded in the chart")
    return z


def _lay_out(vertices: np.ndarray, quads: np.ndarray, inner: np.ndarray, centre: np.ndarray, *,
             symmetric: bool, surface, stretch_rows: np.ndarray, axis: np.ndarray | None = None) -> np.ndarray:
    """New unit directions from `centre` for the `inner` vertices, laid out without folds; every other
    vertex keeps its own direction.

    The region is the quads with an inner vertex; its boundary loop (outer vertices) stays where it is.
    Directions are charted in the plane, stereographically from the boundary's mean direction or, with
    `axis`, gnomonically toward it (then every region vertex must lie in front of the centre). An affine
    change makes the boundary round, and a radial stretch from a point in its kernel makes it the unit
    circle; there a Tutte embedding (each inner vertex an average of its neighbours: mean-value weights
    where the original chart is a proper star round the vertex, so the layout keeps its shape, equal
    weights where it is folded) cannot fold. `_STRETCH_ROUNDS` times, the weights of `stretch_rows`
    are divided by how much area each neighbour got on `surface` (a function from directions to points)
    against an even share, which spreads the squeezed-up folds out. Back in the chart, any corner the
    radial stretch turned over is unfolded (`_unfold`). The boundary must be star-shaped in the chart.
    A region with no boundary keeps its directions."""
    v = np.asarray(vertices, dtype=np.float64)
    region = quads[inner[quads].any(axis=1)]
    loop = _boundary_loop(region)
    u = v - centre
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    if loop is None:
        return u
    if inner[loop].any():
        raise RuntimeError("layout: a vertex to lay out lies on the region's boundary")
    ids = np.flatnonzero(inner)
    used = np.unique(region)

    if axis is None:
        pole = u[loop].mean(axis=0)
    else:
        pole = -np.asarray(axis, dtype=np.float64)
    if symmetric:
        pole[0] = 0.0
    pole /= np.linalg.norm(pole)
    side = np.array([1.0, 0.0, 0.0]) if abs(pole[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
    e1 = np.cross(pole, side)
    e1 /= np.linalg.norm(e1)
    e0 = np.cross(pole, e1)
    if axis is None:
        def chart(d: np.ndarray) -> np.ndarray:
            s = 1.0 - d @ pole
            return np.stack([(d @ e0) / s, (d @ e1) / s], axis=1)

        def unchart_plane(z: np.ndarray) -> np.ndarray:
            r2 = (z * z).sum(axis=1, keepdims=True)
            t = z[:, :1] * e0 + z[:, 1:] * e1
            return (2.0 * t + (r2 - 1.0) * pole) / (r2 + 1.0)
    else:
        if (u[used] @ pole >= 0.0).any():
            raise RuntimeError("layout: part of the region is not in front of the centre")

        def chart(d: np.ndarray) -> np.ndarray:
            s = -(d @ pole)
            return np.stack([(d @ e0) / s, (d @ e1) / s], axis=1)

        def unchart_plane(z: np.ndarray) -> np.ndarray:
            t = z[:, :1] * e0 + z[:, 1:] * e1 - pole
            return t / np.linalg.norm(t, axis=1, keepdims=True)

    z = np.zeros((len(v), 2))
    z[used] = chart(u[used])
    mean = z[loop].mean(axis=0)
    spread = np.cov((z[loop] - mean).T)
    if symmetric:
        mean[0] = 0.0
        white = np.diag(1.0 / np.sqrt(np.diag(spread)))
    else:
        evals, evecs = np.linalg.eigh(spread)
        white = evecs / np.sqrt(evals)
        if np.linalg.det(white) < 0.0:
            white[:, 0] = -white[:, 0]
    unwhite = np.linalg.inv(white)
    z[used] = (z[used] - mean) @ white

    def unchart(q: np.ndarray) -> np.ndarray:
        return unchart_plane(q @ unwhite + mean)

    polygon = z[loop]
    if 0.5 * _cross2(polygon, np.roll(polygon, -1, axis=0)).sum() <= 0.0:
        raise RuntimeError("layout: the region's boundary runs clockwise in the chart")
    hub, margin = _kernel_point(polygon, symmetric)
    if margin <= 0.0:
        raise RuntimeError("layout: the region's boundary is not star-shaped in the chart")
    angle = np.arctan2(polygon[:, 1] - hub[1], polygon[:, 0] - hub[0])
    radius = np.linalg.norm(polygon - hub, axis=1)
    order = np.argsort(angle)
    sorted_angle = np.r_[angle[order][-1] - 2.0 * np.pi, angle[order], angle[order][0] + 2.0 * np.pi]
    sorted_radius = np.r_[radius[order][-1], radius[order], radius[order][0]]
    base = min(float(radius.mean()), 1.9 * float(radius.min()))

    def reach(theta: np.ndarray) -> np.ndarray:
        return np.interp(np.mod(theta - sorted_angle[0], 2.0 * np.pi) + sorted_angle[0], sorted_angle, sorted_radius)

    # Disk coordinates: a point at angle theta and fraction r of the way out lies base + (reach - base) r
    # times r from the hub, so the stretch is the identity at the hub and exact on the boundary.
    def to_disk(p: np.ndarray) -> np.ndarray:
        d = p - hub
        dist = np.linalg.norm(d, axis=1)
        extra = reach(np.arctan2(d[:, 1], d[:, 0])) - base
        flat = np.abs(extra) <= 1e-12
        safe = np.where(flat, 1.0, extra)
        r = np.where(flat, dist / base,
                     (np.sqrt(np.maximum(base * base + 4.0 * safe * dist, 0.0)) - base) / (2.0 * safe))
        return np.divide(d, dist[:, np.newaxis], out=np.zeros_like(d), where=dist[:, np.newaxis] > 0) * r[:, np.newaxis]

    def from_disk(q: np.ndarray) -> np.ndarray:
        r = np.linalg.norm(q, axis=1)
        return hub + q * (base + (reach(np.arctan2(q[:, 1], q[:, 0])) - base) * r)[:, np.newaxis]

    zeta = np.zeros((len(v), 2))
    zeta[used] = to_disk(z[used])

    rows, cols, values = [], [], []
    for k, ring in enumerate(_ordered_rings(region, ids)):
        p = zeta[ring] - zeta[ids[k]]
        q = np.roll(p, -1, axis=0)
        angles = np.arctan2(_cross2(p, q), (p * q).sum(axis=1))
        if (angles > 0.0).all() and abs(angles.sum() - 2.0 * np.pi) < 1e-9:
            half = np.tan(angles / 2.0)
            weight = (half + np.roll(half, 1)) / np.linalg.norm(p, axis=1)       # mean-value coordinates
        else:
            weight = np.ones(len(ring))
        rows += [k] * len(ring)
        cols += ring
        values += (weight / weight.sum()).tolist()
    rows, cols, base_weights = np.array(rows), np.array(cols), np.array(values)
    slot = np.full(len(v), -1)
    slot[ids] = np.arange(len(ids))
    free = slot[cols] >= 0
    count = len(ids)

    def solve(weights: np.ndarray) -> np.ndarray:
        def matvec(x: np.ndarray) -> np.ndarray:
            y = x.copy()
            for c in range(2):
                y[:, c] -= np.bincount(rows[free], weights=weights[free] * x[slot[cols[free]], c], minlength=count)
            return y
        b = np.stack([np.bincount(rows[~free], weights=weights[~free] * zeta[cols[~free], c], minlength=count)
                      for c in range(2)], axis=1)
        return _bicgstab(matvec, b, zeta[ids])

    def quad_area(points: np.ndarray) -> np.ndarray:
        return 0.5 * np.linalg.norm(np.cross(points[:, 2] - points[:, 0], points[:, 3] - points[:, 1]), axis=1)

    weights = base_weights.copy()
    zeta[ids] = solve(weights)
    share = np.full(len(region), 1.0 / len(region))
    reweighted = stretch_rows[ids[rows]]
    for _ in range(_STRETCH_ROUNDS):
        d = u.copy()
        d[ids] = unchart(from_disk(zeta[ids]))
        area = quad_area(surface(d[region].reshape(-1, 3)).reshape(-1, 4, 3))
        stretch = share / (area / area.sum())
        total, mass = np.zeros(len(v)), np.zeros(len(v))
        for c in range(4):
            np.add.at(total, region[:, c], share * stretch)
            np.add.at(mass, region[:, c], share)
        weights = np.where(reweighted, weights / np.sqrt(total[cols] / mass[cols]), weights)
        weights = weights / np.bincount(rows, weights=weights, minlength=count)[rows]
        zeta[ids] = solve(weights)

    z_final = z.copy()
    z_final[ids] = from_disk(zeta[ids])
    movable = np.zeros(len(v), dtype=bool)
    movable[ids] = True
    z_final = _unfold(z_final, region, movable)
    d = u.copy()
    d[ids] = unchart(z_final[ids])
    return d


# --------------------------------------------------------------------------- head

def head_vertices(head_w: np.ndarray) -> np.ndarray:
    """Ascending indices of the head vertices that end on the egg: head weight >= 0.999."""
    return np.flatnonzero(np.clip(np.asarray(head_w, dtype=np.float64), 0.0, 1.0) >= _PIN_WEIGHT)


def _relax(v: np.ndarray, quads: np.ndarray, w: np.ndarray, centre: np.ndarray, rounds: int, put) -> np.ndarray:
    """`rounds` rounds of uniform Laplacian relaxation (step _RELAX_STEP * w toward the 1-ring mean) of
    the w > 0 vertices, each followed by `put` (which puts vertices back where they belong) and the
    flip guard `_revert_flips` against `centre`."""
    moving = w > 0.0
    rings = _Rings(quads, len(v))
    step = (_RELAX_STEP * w)[moving, np.newaxis]
    for _ in range(rounds):
        relaxed = v.copy()
        relaxed[moving] += step * (rings.mean(v)[moving] - v[moving])
        v = _revert_flips(v, put(relaxed), quads, centre)
    return v


def make_egg_head(vertices: np.ndarray, quads: np.ndarray, head_w: np.ndarray, egg: Egg) -> np.ndarray:
    """New vertex positions with the head turned into the egg; the input is not modified.

    1. Every head vertex moves to x + w (P(x) - x), P the projection onto the egg along the ray from its
       centre; vertices with w >= 0.999 go all the way.
    2. The vertices with w >= 0.1 (the head and neck; below that the region runs out over the
       shoulders, which face the head) get new directions from `_lay_out`, the area spread evenly over
       the egg, and are put back on their new rays: on the egg for w >= 0.999, at
       (1 - w) |x - c| + w |P - c| from the centre otherwise. The eye sockets, mouth, nostrils and the
       ears' backs are part of that one fold-free sheet, so they lie flat on the egg with the rest.
    3. `_HEAD_ROUNDS` rounds of Laplacian relaxation (step 0.5 w) even the vertices out, the w >= 0.999
       ones put back on the egg each round, every round guarded by `_revert_flips` against the egg centre
       (no quad that faces away from it turns toward it, no corner is squashed below `_AREA_FLOOR`).
    4. A mirror-symmetric head is made exactly symmetric again (the solves round differently on the two
       sides). Vertices with w = 0 are never touched."""
    w = np.clip(np.asarray(head_w, dtype=np.float64), 0.0, 1.0)
    v0 = np.array(vertices, dtype=np.float64)
    v = v0.copy()
    used = np.zeros(len(v), dtype=bool)
    used[quads] = True
    w = np.where(used, w, 0.0)
    moving = w > 0.0
    pinned = w >= _PIN_WEIGHT
    blend = np.where(pinned, 1.0, w)
    v[moving] += blend[moving, np.newaxis] * (project_to_egg(egg, v0[moving]) - v0[moving])

    core = w >= _CORE_WEIGHT

    def on_egg(d: np.ndarray) -> np.ndarray:
        return project_to_egg(egg, egg.centre + d)

    d = _lay_out(v, quads, core, egg.centre, symmetric=True, surface=on_egg, stretch_rows=pinned)
    ids = np.flatnonzero(core)
    egg_radius = np.linalg.norm(on_egg(d[ids]) - egg.centre, axis=1)
    radius = (1.0 - blend[ids]) * np.linalg.norm(v0[ids] - egg.centre, axis=1) + blend[ids] * egg_radius
    v[ids] = egg.centre + radius[:, np.newaxis] * d[ids]

    def put(points: np.ndarray) -> np.ndarray:
        points[pinned] = project_to_egg(egg, points[pinned])
        return points

    v = _relax(v, quads, w, egg.centre, _HEAD_ROUNDS, put)
    partner = _mirror_partners(v0, moving)
    if partner is not None:
        v = _symmetrize(v, partner, moving)
        v[pinned] = project_to_egg(egg, v[pinned])
    return v


# --------------------------------------------------------------------------- feet

class _Topology:
    """Quad-mesh adjacency: each vertex's neighbours and each edge's quads."""

    def __init__(self, quads: np.ndarray):
        self.neighbours: dict[int, list[int]] = {}
        self.edge_quads: dict[tuple[int, int], list[int]] = {}
        for k, q in enumerate(quads.tolist()):
            for j in range(4):
                a, b = q[j], q[(j + 1) % 4]
                self.edge_quads.setdefault((min(a, b), max(a, b)), []).append(k)
        for a, b in sorted(self.edge_quads):
            self.neighbours.setdefault(a, []).append(b)
            self.neighbours.setdefault(b, []).append(a)

    def straight_on(self, previous: int, current: int) -> int | None:
        """The edge that leaves `current` opposite the edge from `previous` (sharing no quad with it), when
        `current` has four neighbours; None otherwise."""
        if len(self.neighbours[current]) != 4:
            return None
        beside = set(self.edge_quads[(min(previous, current), max(previous, current))])
        for x in self.neighbours[current]:
            if x != previous and not beside & set(self.edge_quads[(min(current, x), max(current, x))]):
                return x
        return None

    def edge_loop(self, a: int, b: int, limit: int = 200) -> list[int] | None:
        """The closed edge loop that starts a -> b and goes straight on at every vertex, or None."""
        loop = [a, b]
        while len(loop) < limit:
            nxt = self.straight_on(loop[-2], loop[-1])
            if nxt is None or (nxt in loop and nxt != a):
                return None
            if nxt == a:
                return loop
            loop.append(nxt)
        return None

    def side(self, seed: int, wall: set[int]) -> set[int]:
        """The vertices reached from `seed` without crossing `wall`."""
        seen, todo = {seed}, [seed]
        while todo:
            for b in self.neighbours[todo.pop()]:
                if b not in seen and b not in wall:
                    seen.add(b)
                    todo.append(b)
        return seen

    def depth(self, start: list[int], within: set[int]) -> dict[int, int]:
        """Rings (edge hops) from `start` to every vertex of `within` reachable through `within`."""
        depth = {x: 0 for x in start}
        frontier = list(start)
        while frontier:
            nxt = []
            for x in frontier:
                for y in self.neighbours[x]:
                    if y in within and y not in depth:
                        depth[y] = depth[x] + 1
                        nxt.append(y)
            frontier = nxt
        return depth


def _forefoot_loop(vertices: np.ndarray, toes: np.ndarray, side: np.ndarray, topology: _Topology,
                   used_count: int) -> tuple[np.ndarray, np.ndarray, dict[int, int]]:
    """The edge loop that closes round the foot nearest behind the toes (at least _FOREFOOT_RINGS rings
    behind every toe vertex), the vertices in front of it and their ring depths from it."""
    toe_set = set(toes.tolist())
    centre = vertices[toes].mean(axis=0)
    candidates = np.flatnonzero(side & (np.linalg.norm(vertices - centre, axis=1) < 0.15))
    found: list[tuple[int, int, list[int], set[int]]] = []
    seen: set[frozenset[int]] = set()
    for a in candidates.tolist():
        if a in toe_set:
            continue
        for b in topology.neighbours.get(a, []):
            loop = topology.edge_loop(a, b)
            if loop is None:
                continue
            key = frozenset(loop)
            if key in seen or key & toe_set:
                continue
            seen.add(key)
            front = topology.side(int(toes[0]), set(loop))
            if toe_set <= front and 2 * len(front) < used_count:
                found.append((len(front), min(loop), loop, front))
    for _, _, loop, front in sorted(found, key=lambda item: (item[0], item[1])):
        depth = topology.depth(loop, front)
        if min(depth.get(t, 0) for t in toe_set) >= _FOREFOOT_RINGS:
            return np.array(loop, dtype=np.int64), np.array(sorted(front), dtype=np.int64), depth
    raise RuntimeError(f"round_forefeet: no edge loop closes round the foot {_FOREFOOT_RINGS} or more rings "
                       f"behind the toes")


def _fibonacci(n: int) -> np.ndarray:
    """(n, 3) unit vectors spread evenly over the sphere."""
    i = np.arange(n) + 0.5
    polar = np.arccos(1.0 - 2.0 * i / n)
    turn = np.pi * (1.0 + 5.0 ** 0.5) * i
    return np.stack([np.cos(turn) * np.sin(polar), np.cos(polar), np.sin(turn) * np.sin(polar)], axis=1)


class _Shell:
    """A smooth convex shell with a flat sole, seen from an outside point.

    The convex hull of `points` has its gauge (1 / distance to the surface, per direction) about `inner`
    averaged over directions within about `_FOREFOOT_SMOOTHING` radians: an average of convex gauges is
    convex, and the averaging rounds the hull's edges. That surface is sampled and stored as its support
    planes, and the floor plane y = `floor` cuts it. `reach(d)` is how far along each unit direction d from
    `centre` the ray leaves the shell; rays must hit it."""

    def __init__(self, points: np.ndarray, inner: np.ndarray, centre: np.ndarray, floor: float):
        normals = _fibonacci(_SHELL_NORMALS)
        support = ((points - inner) @ normals.T).max(axis=0)
        gauge = 1.0 / self._exit(normals, normals, support)
        samples = _fibonacci(_SHELL_SAMPLES)
        smooth = np.empty(len(samples))
        for start in range(0, len(samples), 500):
            kernel = np.exp((samples[start:start + 500] @ normals.T - 1.0) / _FOREFOOT_SMOOTHING ** 2)
            smooth[start:start + 500] = (kernel * gauge).sum(axis=1) / kernel.sum(axis=1)
        surface = inner + samples / smooth[:, np.newaxis]
        self.normals = _fibonacci(_SHELL_OUTPUT_NORMALS)
        self.support = ((surface - centre) @ self.normals.T).max(axis=0)
        self.centre, self.floor = centre, floor

    @staticmethod
    def _exit(d: np.ndarray, normals: np.ndarray, support: np.ndarray) -> np.ndarray:
        out = np.empty(len(d))
        for start in range(0, len(d), 500):
            dn = d[start:start + 500] @ normals.T
            ahead = dn > 1e-12
            leave = np.where(ahead, support / np.where(ahead, dn, 1.0), np.inf).min(axis=1)
            behind = dn < -1e-12
            enter = np.where(behind, support / np.where(behind, dn, -1.0), -np.inf).max(axis=1)
            if (enter > leave).any():
                raise RuntimeError("round_forefeet: a ray from the layout centre misses the forefoot shell")
            out[start:start + 500] = leave
        return out

    def floor_distance(self, d: np.ndarray) -> np.ndarray:
        down = d[:, 1] < 0.0
        return np.where(down, (self.floor - self.centre[1]) / np.where(down, d[:, 1], -1.0), np.inf)

    def reach(self, d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(distance along each ray to the shell, whether that point is on the floor)."""
        wall = self._exit(d, self.normals, self.support)
        floor = self.floor_distance(d)
        return np.minimum(wall, floor), floor <= wall


def _surface_distance(centre: np.ndarray, d: np.ndarray, corners: np.ndarray) -> np.ndarray:
    """How far along each unit direction d from `centre` the ray last crosses the triangles `corners`
    ((T, 3, 3)); NaN where it crosses none."""
    a, e1, e2 = corners[:, 0], corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]
    best = np.full(len(d), np.nan)
    for start in range(0, len(d), 200):
        ray = d[start:start + 200, np.newaxis, :]
        h = np.cross(ray, e2[np.newaxis])
        det = (e1[np.newaxis] * h).sum(axis=2)
        ok = np.abs(det) > 1e-18
        inv = np.divide(1.0, det, out=np.zeros_like(det), where=ok)
        s = centre - a
        u = inv * (s[np.newaxis] * h).sum(axis=2)
        q = np.cross(s, e1)
        v = inv * (ray * q[np.newaxis]).sum(axis=2)
        t = inv * (e2 * q).sum(axis=1)[np.newaxis]
        hit = ok & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > 0.0)
        far = np.where(hit, t, -np.inf).max(axis=1)
        best[start:start + 200] = np.where(np.isfinite(far), far, np.nan)
    return best


def _on_ray(points: np.ndarray, centre: np.ndarray, distance) -> np.ndarray:
    """Each point moved along its ray from `centre` to `distance(direction)` from it."""
    d = points - centre
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    return centre + distance(d)[:, np.newaxis] * d


def _round_forefoot(vertices: np.ndarray, quads: np.ndarray, toes: np.ndarray, side: np.ndarray,
                    topology: _Topology, used_count: int) -> tuple[np.ndarray, np.ndarray]:
    """One foot of `round_forefeet`: (new vertices, indices of the forefoot)."""
    loop, front, depth = _forefoot_loop(vertices, toes, side, topology, used_count)
    n = len(vertices)
    # blend weight: 0 on the loop, 1 from the toes' depth on, harmonic in between, then smoothstepped
    full = np.array([x for x in front.tolist() if depth[x] >= _FOREFOOT_RINGS], dtype=np.int64)
    h = np.zeros(n)
    h[full] = 1.0
    between = np.zeros(n, dtype=bool)
    between[front] = True
    between[full] = False
    rings = _Rings(quads, n)
    for _ in range(_WEIGHT_ITERATIONS):
        h[between] = rings.mean(h[:, np.newaxis])[between, 0]
    w = np.zeros(n)
    t = np.clip(h[front], 0.0, 1.0)
    w[front] = t * t * (3.0 - 2.0 * t)
    w[full] = 1.0

    # Work in a space stretched vertically until the forefoot is about as tall as it is wide, so the
    # rays from the layout centre meet the sole and the instep at a fair angle.
    extent = np.ptp(vertices[front], axis=0)
    stretch = np.array([1.0, min(extent[0], extent[2]) / extent[1], 1.0])
    p0 = vertices * stretch
    floor = float(p0[front, 1].min())
    behind = set(loop.tolist())
    frontier, front_set = list(loop.tolist()), set(front.tolist())
    for _ in range(3):                    # the hull runs on past the loop, so its back edge is not rounded in
        frontier = [y for x in frontier for y in topology.neighbours[x] if y not in front_set and y not in behind
                    and not behind.add(y)]
    hull_points = p0[sorted(front_set | behind)]
    footprint = hull_points * np.array([1.0, 0.0, 1.0]) + np.array([0.0, floor - _FOREFOOT_SINK, 0.0])
    axis = p0[toes].mean(axis=0) - p0[loop].mean(axis=0)
    axis[1] = 0.0
    axis /= np.linalg.norm(axis)
    middle = floor + 0.5 * (p0[front, 1].max() - floor)
    loop_centre = p0[loop].mean(axis=0)
    centre = loop_centre + axis * (float(((p0[loop] - loop_centre) @ axis).min()) - _FOREFOOT_BEHIND)
    centre[1] = middle
    inner = p0[full].mean(axis=0)
    inner[1] = middle
    shell = _Shell(np.vstack([hull_points, footprint]), inner, centre, floor)

    core = np.zeros(n, dtype=bool)
    core[front] = True
    region = quads[core[quads].any(axis=1)]
    pinned = w >= _PIN_WEIGHT
    d = _lay_out(p0, quads, core, centre, symmetric=False, stretch_rows=pinned, axis=axis,
                 surface=lambda dd: centre + shell.reach(dd)[0][:, np.newaxis] * dd)
    ids = front
    wall, _ = shell.reach(d[ids])
    # Below weight 1 the vertex blends the shell with the original surface on its new ray (with its own
    # old distance where the ray misses that surface), so the sole stays on the floor through the blend.
    tris = np.concatenate([region[:, [0, 1, 2]], region[:, [0, 2, 3]]])
    original = _surface_distance(centre, d[ids], p0[tris])
    original = np.where(np.isnan(original), np.linalg.norm(p0[ids] - centre, axis=1), original)
    radius = (1.0 - w[ids]) * original + w[ids] * wall
    p = p0.copy()
    p[ids] = centre + np.minimum(radius, shell.floor_distance(d[ids]))[:, np.newaxis] * d[ids]

    sole = core & (np.abs(p[:, 1] - floor) < 1e-12)

    def put(points: np.ndarray) -> np.ndarray:
        points[pinned] = _on_ray(points[pinned], centre, lambda dd: shell.reach(dd)[0])
        low = core & (points[:, 1] < floor)
        points[low] = _on_ray(points[low], centre, shell.floor_distance)
        d = points[sole] - centre
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        reach, on_floor = shell.reach(d)                             # back down to the sole where the ray
        points[np.flatnonzero(sole)[on_floor]] = centre + reach[on_floor, np.newaxis] * d[on_floor]   # meets it
        return points

    p = _relax(p, region, w * core, centre, _FOREFOOT_ROUNDS, put)
    # A vertex lower than all its neighbours would stand up out of the floor plane's sections on its own:
    # it goes down its ray to the floor (moving along its ray keeps the layout fold-free).
    for x in front.tolist():
        if p[x, 1] > floor and all(p[y, 1] > p[x, 1] for y in topology.neighbours[x]):
            ray = (p[x] - centre)[np.newaxis] / np.linalg.norm(p[x] - centre)
            reach, on_floor = shell.reach(ray)
            if on_floor[0]:
                p[x] = centre + reach[0] * ray[0]
    on_floor = core & (np.abs(p[:, 1] - floor) < 1e-9)
    result = vertices.copy()
    result[front] = p[front] / stretch
    result[on_floor, 1] = vertices[front, 1].min()
    # Everything stays within the box round the old forefoot and the rings behind it: the shell is inside
    # it, and a vertex outside it is a ray that ran on along the floor.
    box = np.array(sorted(front_set | behind))
    lo, hi = vertices[box].min(axis=0) - 1e-3, vertices[box].max(axis=0) + 1e-3
    outside = ((result[front] < lo) | (result[front] > hi)).any(axis=1)
    if outside.any():
        raise RuntimeError(f"round_forefeet: {int(outside.sum())} forefoot vertices end outside the box round the "
                           f"old forefoot")
    return result, front


def round_forefeet(vertices: np.ndarray, quads: np.ndarray, toe_w: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(new vertex positions, ascending indices of the forefoot vertices) with the toes of each foot
    turned into one rounded forefoot with a flat sole; the input is not modified.

    Run on the posed body (so the floor is level). Per foot (x > 0, then x < 0):

    1. The forefoot is everything in front of the nearest edge loop that closes round the foot at least
       `_FOREFOOT_RINGS` rings behind every toe vertex (toe weight >= 0.5). Its blend weight is 0 on the
       loop and 1 from those rings on (harmonic in between, smoothstepped).
    2. The shell is a smoothed convex hull of the forefoot (and 3 rings behind it) whose footprint
       reaches below the floor, cut flat by the floor plane y = the forefoot's lowest point: it spans
       the gaps between the toes and rounds the tips.
    3. The forefoot is laid out afresh (`_lay_out`) as directions from a point behind the loop, in a
       space stretched vertically to make the forefoot about as tall as wide, put on its rays (on the
       shell for weight 1, blended with the original distance below that, never under the floor),
       relaxed `_FOREFOOT_ROUNDS` rounds under the flip guard, and a vertex lower than all its neighbours
       is lowered to the floor along its ray.

    A mirror-symmetric body gets the right foot as the exact mirror image of the left. Nothing outside
    the forefoot moves."""
    v = np.array(vertices, dtype=np.float64)
    w_toe = np.clip(np.asarray(toe_w, dtype=np.float64), 0.0, 1.0)
    used = np.zeros(len(v), dtype=bool)
    used[quads] = True
    topology = _Topology(quads)
    partner = _mirror_partners(v, used)
    result = v.copy()
    forefeet = []
    for sign in (1.0, -1.0):
        side = used & (sign * v[:, 0] > 0.0)
        toes = np.flatnonzero(side & (w_toe >= 0.5))
        if len(toes) == 0:
            continue
        if sign < 0.0 and partner is not None and forefeet:
            left = forefeet[0]
            result[partner[left]] = result[left] * np.array([-1.0, 1.0, 1.0])
            forefeet.append(partner[left])
            continue
        moved, front = _round_forefoot(v, quads, toes, side, topology, int(used.sum()))
        result[front] = moved[front]
        forefeet.append(front)
    return result, np.sort(np.concatenate(forefeet)) if forefeet else np.zeros(0, dtype=np.int64)
