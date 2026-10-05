"""Measurements of the posed mannequin: girths, landmarks and region cylinders.

Everything here works on the finished body in Drape's frame (metres, Y up, facing +Z, the
character's left is +x, x = 0 on the symmetry plane). `section_points` and
`convex_hull_perimeter` give the girths (convex-hull perimeters of horizontal sections, as in
the milestone spec 8.2); `dominant_bones` sorts the vertices into body parts by their strongest
skinning weight; `landmarks_regions_girths` turns those into the numbers written to
`<body>.json`.

Nothing is random, so equal inputs give byte-identical outputs. Python 3.13 and numpy only.
"""
import math
from dataclasses import dataclass

import numpy as np

from makehuman import Rig, descendants
from style import Egg

_STEP = 0.01                  # metres between the heights tried for the waist and the hip
_WAIST_ABOVE_CROTCH = 0.10    # the waist is searched from here up to the chest
_CHEST_BAND = 0.01            # torso vertices this close to the breast joint's height are "at the chest"
_CROTCH_HALF_WIDTH = 0.01
_NECK_GIRTH_ABOVE_BASE = 0.02
_HPS_BAND = 0.02              # width of the strip beside the neck that the high point of shoulder is in
_HPS_HALF_DEPTH = 0.03
_SHOULDER_DISC = 0.015
_TIE = 1e-9                   # extremes closer than this are ties


# --------------------------------------------------------------------------- sections

def convex_hull_perimeter(points: np.ndarray) -> float:
    """Perimeter of the convex hull of (n, 2) points (Andrew's monotone chain).

    Duplicates, input order and points on a hull edge do not matter. Fewer than two distinct
    points have perimeter 0; points on a line are a polygon walked there and back, so twice the
    segment's length."""
    pts = np.unique(np.asarray(points, dtype=np.float64).reshape(-1, 2), axis=0)    # sorted by (x, z)
    if len(pts) < 2:
        return 0.0

    def cross(o: list[float], a: list[float], b: list[float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def chain(sequence: list[list[float]]) -> list[list[float]]:
        out: list[list[float]] = []
        for p in sequence:
            while len(out) >= 2 and cross(out[-2], out[-1], p) <= 0.0:
                out.pop()
            out.append(p)
        return out

    ordered = pts.tolist()
    hull = chain(ordered)[:-1] + chain(ordered[::-1])[:-1]
    return sum(math.dist(hull[i], hull[(i + 1) % len(hull)]) for i in range(len(hull)))


def section_points(vertices: np.ndarray, tris: np.ndarray, y: float, vertex_mask: np.ndarray) -> np.ndarray:
    """(n, 2) points (x, z) where the horizontal plane at height `y` meets the triangles whose
    three vertices are all in `vertex_mask`.

    Every triangle edge that crosses the plane contributes its crossing point, and every vertex
    that lies exactly on the plane contributes itself, so a plane through vertices, or through a
    whole face, still gives the section. Points are not de-duplicated (the hull does not need it)."""
    vertices = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(tris).reshape(-1, 3)
    corners = vertices[tris[np.asarray(vertex_mask, dtype=bool)[tris].all(axis=1)]]      # (K, 3, 3)
    found = [np.zeros((0, 2))]
    for a, b in ((0, 1), (1, 2), (2, 0)):
        pa, pb = corners[:, a], corners[:, b]
        da, db = pa[:, 1] - y, pb[:, 1] - y
        crossing = da * db < 0.0
        t = (da[crossing] / (da[crossing] - db[crossing]))[:, np.newaxis]
        found.append((pa[crossing] + t * (pb[crossing] - pa[crossing]))[:, [0, 2]])
        found.append(pa[da == 0.0][:, [0, 2]])
    return np.concatenate(found)


# --------------------------------------------------------------------------- body parts

def dominant_bones(rig: Rig, n: int) -> list[str]:
    """For each of the first `n` vertices, the bone with its largest skinning weight, or "" when
    no bone weights it. A tie goes to the bone listed first in `rig.weights`; weights on
    vertices >= n (MakeHuman's helper geometry) are ignored."""
    best = np.zeros(n, dtype=np.float64)
    owner = np.full(n, -1, dtype=np.int64)
    names = list(rig.weights)
    for number, bone in enumerate(names):
        indices, weights = rig.weights[bone]
        inside = indices < n
        indices, weights = indices[inside], weights[inside]
        stronger = weights > best[indices]
        best[indices[stronger]] = weights[stronger]
        owner[indices[stronger]] = number
    return ["" if o < 0 else names[o] for o in owner]


# --------------------------------------------------------------------------- landmarks and regions

@dataclass
class Measurement:
    landmarks: dict[str, np.ndarray]        # name -> (3,) position
    regions: dict[str, dict]                # BodyRegion name -> {"origin", "axis", "e0": (3,), "radius": float}
    girths: dict[str, float]                # chest, waist, hip, neck
    region_vertices: dict[str, np.ndarray]  # BodyRegion name -> ascending vertex indices that set its radius


def _round(x):
    """x rounded to 6 decimals (the precision the assets are written with), -0.0 folded into 0.0."""
    return np.round(np.asarray(x, dtype=np.float64), 6) + 0.0


def _ceil6(x: float) -> float:
    """The smallest multiple of 1e-6 that is not below x (up to 1e-15 of float noise)."""
    return math.ceil(x * 1e6 - 1e-9) / 1e6


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _pick(vertices: np.ndarray, candidates: np.ndarray, key: np.ndarray, highest: bool = True,
          side: float = 1.0) -> int:
    """Index of the candidate (a boolean mask) with the largest, or smallest, `key`. Ties within
    a nanometre go to the vertex with the largest `side * x`, so mirror-image vertices resolve the
    same way on every run and every machine; a remaining tie goes to the lowest index."""
    ids = np.flatnonzero(candidates)
    values = key[ids] if highest else -key[ids]
    tied = ids[values >= values.max() - _TIE]
    return int(tied[np.argmax(side * vertices[tied, 0])])


def _steps(low: float, high: float, what: str) -> np.ndarray:
    """Heights low, low + 1 cm, ... up to `high` (inclusive)."""
    count = int(math.floor((high - low) / _STEP + 1e-9)) + 1
    if count < 1:
        raise ValueError(f"no height to try for the {what}: the search runs from {low:.4f} to {high:.4f}")
    return low + _STEP * np.arange(count)


def _classes(rig: Rig, dominant: list[str]) -> dict[str, np.ndarray]:
    """Boolean masks of the vertex classes, by dominant bone (see the module docs)."""
    def under(*roots: str) -> set[str]:
        return {bone for root in roots for bone in descendants(rig, root)}

    def mask(bones: set[str]) -> np.ndarray:
        return np.fromiter((bone in bones for bone in dominant), dtype=bool, count=len(dominant))

    arm = {side: mask(under(f"shoulder01.{side}")) for side in "LR"}
    leg = {side: mask(under(f"upperleg01.{side}")) for side in "LR"}
    head, neck = mask(under("head")), mask({"neck01", "neck02", "neck03"})
    torso = ~(arm["L"] | arm["R"] | head | neck | leg["L"] | leg["R"])
    return {"arm_L": arm["L"], "arm_R": arm["R"], "arm": arm["L"] | arm["R"], "head": head, "neck": neck,
            "leg_L": leg["L"], "leg_R": leg["R"], "torso": torso}


def measure_body(vertices: np.ndarray, tris: np.ndarray, dominant: list[str], rig: Rig,
                 joints: dict[str, np.ndarray], egg: Egg,
                 head_vertices: np.ndarray | None = None) -> Measurement:
    """Landmarks, region cylinders and girths of the finished body.

    `vertices` (N, 3) are the body's vertices only, in Drape's frame, and `tris` (K, 3) index
    them; `dominant` is `dominant_bones(rig, N)`. `joints` holds the posed `upperarm01`, `wrist`,
    `upperleg01` and `foot` joints (`<bone>.<L|R>____head`, as `pose.Posed.joints` names them)
    and the unposed `breast.L____head` and `neck01____head`, all shifted into the same frame
    (`eye.L____head` may be present; nothing here needs it). `head_vertices` are the vertex
    indices the Head cylinder is meant to contain (the egg-surface vertices); they only fill
    `region_vertices["Head"]`, because the head's radius comes from the egg.

    Vertex classes, by dominant bone: *arm* (a descendant of `shoulder01.L` or `.R`), *head*
    (of `head`), *neck* (`neck01` to `neck03`), *leg* (of `upperleg01.L` or `.R`), *torso* (the
    rest); *non-arm* is every class but arm.

    Everything returned is rounded to 6 decimals, the precision the assets are written with, and
    the radii are then measured from the rounded cylinders and rounded up, so a vertex that lies
    inside its cylinder here still does when the files are read back.

    Where this departs from the brief (both forced by the real MakeHuman bodies; see the task 5
    report):

    * `hip`: the 1 cm steps start at the hip joints' height, not at the crotch. Section hulls only
      get smaller going up from the crotch (the widest section is the pair of thigh tops), so
      searching from the crotch put the hip at the crotch (male) or 3 cm above it (female).
    * `hps_*`: the strip beside the neck starts at the neck's own radius at the girth height (the
      largest distance from the body axis of the neck section), not at the Neck cylinder's radius,
      which the neck01-weighted trapezius inflates to about 0.11 m; and head vertices are not
      candidates, because the egg is as wide as the strip.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(tris).reshape(-1, 3)
    if len(dominant) != len(vertices):
        raise ValueError(f"dominant has {len(dominant)} entries for {len(vertices)} vertices")
    cls = _classes(rig, dominant)
    x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    non_arm = ~cls["arm"]

    def girth(height: float, mask: np.ndarray) -> float:
        return convex_hull_perimeter(section_points(vertices, tris, height, mask))

    # --- the body axis and the landmarks that fix the heights
    chest_y = float(joints["breast.L____head"][1])
    at_chest = cls["torso"] & (np.abs(y - chest_y) <= _CHEST_BAND)
    if not at_chest.any():
        raise ValueError(f"no torso vertex within {_CHEST_BAND} m of the chest height {chest_y:.4f}")
    z_axis = float(_round((z[at_chest].min() + z[at_chest].max()) / 2.0))
    neck_base_y = float(_round(joints["neck01____head"][1]))
    neck_base = np.array([0.0, neck_base_y, z_axis])

    chest = _pick(vertices, at_chest, z)

    hip_joint_y = float((joints["upperleg01.L____head"][1] + joints["upperleg01.R____head"][1]) / 2.0)
    midline_below_hip = (np.abs(x) <= _CROTCH_HALF_WIDTH) & (y < hip_joint_y)
    if not midline_below_hip.any():
        raise ValueError("no vertex within 1 cm of the midline below the hip joints (crotch)")
    crotch = _pick(vertices, midline_below_hip, y, highest=False)
    crotch_y = float(y[crotch])

    waist_heights = _steps(crotch_y + _WAIST_ABOVE_CROTCH, chest_y, "waist")
    waist_y = float(_round(waist_heights[int(np.argmin([girth(h, non_arm) for h in waist_heights]))]))
    hip_heights = _steps(hip_joint_y, waist_y, "hip")
    hip_y = float(_round(hip_heights[int(np.argmax([girth(h, non_arm) for h in hip_heights]))]))

    neck_section = section_points(vertices, tris, neck_base_y + _NECK_GIRTH_ABOVE_BASE, cls["neck"])
    if len(neck_section) == 0:
        raise ValueError("the neck has no section 2 cm above the neck base")
    girths = {"chest": girth(chest_y, non_arm), "waist": girth(waist_y, non_arm),
              "hip": girth(hip_y, non_arm), "neck": convex_hull_perimeter(neck_section)}
    neck_radius = float(np.hypot(neck_section[:, 0], neck_section[:, 1] - z_axis).max())

    # --- the other landmarks
    landmarks: dict[str, np.ndarray] = {
        "neck_base": neck_base, "chest": vertices[chest], "crotch": vertices[crotch],
        "waist": np.array([0.0, waist_y, z_axis]), "hip": np.array([0.0, hip_y, z_axis])}
    shoulder_candidates = non_arm & ~cls["head"]
    for side, name, sign in (("L", "left", 1.0), ("R", "right", -1.0)):
        strip = (shoulder_candidates & (sign * x >= neck_radius) & (sign * x <= neck_radius + _HPS_BAND)
                 & (np.abs(z - z_axis) <= _HPS_HALF_DEPTH))
        if not strip.any():
            raise ValueError(f"no vertex for the {name} high point of shoulder, beside the neck "
                             f"(x from {neck_radius:.4f} to {neck_radius + _HPS_BAND:.4f})")
        landmarks[f"hps_{name}"] = vertices[_pick(vertices, strip, y, side=sign)]

        joint = joints[f"upperarm01.{side}____head"]
        near_joint = np.hypot(x - joint[0], z - joint[2]) <= _SHOULDER_DISC
        if not near_joint.any():
            raise ValueError(f"no vertex within 1.5 cm of the {name} shoulder joint")
        landmarks[f"shoulder_{name}"] = vertices[_pick(vertices, near_joint, y, side=sign)]
        landmarks[f"shoulder_joint_{name}"] = joint
        landmarks[f"wrist_{name}"] = joints[f"wrist.{side}____head"]
        landmarks[f"hip_joint_{name}"] = joints[f"upperleg01.{side}____head"]
        landmarks[f"ankle_{name}"] = joints[f"foot.{side}____head"]
    landmarks = {name: _round(p) for name, p in landmarks.items()}

    # --- region cylinders
    regions: dict[str, dict] = {}
    region_vertices: dict[str, np.ndarray] = {}

    def vertical(name: str, origin: np.ndarray, mask: np.ndarray) -> None:
        origin = _round(origin)
        ids = np.flatnonzero(mask)
        if len(ids) == 0:
            raise ValueError(f"no vertices for the {name} region")
        reach = np.hypot(x[ids] - origin[0], z[ids] - origin[2]).max()
        regions[name] = {"origin": origin, "axis": np.array([0.0, 1.0, 0.0]), "e0": np.array([0.0, 0.0, 1.0]),
                         "radius": _ceil6(float(reach))}
        region_vertices[name] = ids

    def limb(name: str, root: np.ndarray, tip: np.ndarray, reference: np.ndarray, mask: np.ndarray) -> None:
        """Cylinder along root -> tip; e0 is `reference` less its part along the axis."""
        origin = _round(root)
        axis = _unit(tip - root)
        e0 = _unit(reference - (reference @ axis) * axis)
        axis, e0 = _round(axis), _round(e0)
        along = _unit(axis)
        offset = vertices - origin
        axial = offset @ along
        ids = np.flatnonzero(mask & (axial >= 0.0) & (axial <= np.linalg.norm(tip - root)))
        if len(ids) == 0:
            raise ValueError(f"no vertices for the {name} region")
        reach = np.linalg.norm(offset[ids] - axial[ids, None] * along, axis=1).max()
        regions[name] = {"origin": origin, "axis": axis, "e0": e0, "radius": _ceil6(float(reach))}
        region_vertices[name] = ids

    below_neck = (y >= crotch_y) & (y <= neck_base_y)
    vertical("Torso", neck_base, cls["torso"] & below_neck)
    vertical("Neck", neck_base, cls["neck"])
    egg_radius = max(float(egg.radii[0]), float(egg.radii[2]))
    regions["Head"] = {"origin": _round(egg.centre), "axis": np.array([0.0, 1.0, 0.0]),
                       "e0": np.array([0.0, 0.0, 1.0]), "radius": _ceil6(egg_radius)}
    region_vertices["Head"] = (np.zeros(0, dtype=np.int64) if head_vertices is None
                               else np.asarray(head_vertices, dtype=np.int64))
    vertical("Waist", np.array([0.0, waist_y, z_axis]), non_arm & (y >= crotch_y) & (y <= waist_y))
    up, forward = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    for side, name in (("L", "Left"), ("R", "Right")):
        limb(f"Arm{name}", joints[f"upperarm01.{side}____head"], joints[f"wrist.{side}____head"], up,
             cls[f"arm_{side}"])
        limb(f"Leg{name}", joints[f"upperleg01.{side}____head"], joints[f"foot.{side}____head"], forward,
             cls[f"leg_{side}"])

    return Measurement(landmarks=landmarks, regions=regions,
                       girths={name: float(_round(g)) for name, g in girths.items()},
                       region_vertices=region_vertices)


def landmarks_regions_girths(vertices: np.ndarray, tris: np.ndarray, dominant: list[str], rig: Rig,
                             joints: dict[str, np.ndarray], egg: Egg) -> tuple[dict, dict, dict]:
    """(landmarks, regions, girths) of the body, as `measure_body` defines them."""
    m = measure_body(vertices, tris, dominant, rig, joints, egg)
    return m.landmarks, m.regions, m.girths
