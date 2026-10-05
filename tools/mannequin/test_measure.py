"""Offline unit tests for measure.py: synthetic geometry only, never the network."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

import makehuman
import measure
import style


def box_mesh(dx: float, dy: float, dz: float) -> tuple[np.ndarray, np.ndarray]:
    """An axis-aligned box with a corner at the origin: 8 vertices (index = ix + 2 iy + 4 iz,
    i* in {0, 1}) and 12 triangles, two per face."""
    vertices = np.array([[ix * dx, iy * dy, iz * dz]
                         for iz in (0, 1) for iy in (0, 1) for ix in (0, 1)], dtype=np.float64)
    faces = [(0, 2, 6, 4), (1, 3, 7, 5),         # x = 0, x = dx
             (0, 1, 5, 4), (2, 3, 7, 6),         # y = 0, y = dy
             (0, 1, 3, 2), (4, 5, 7, 6)]         # z = 0, z = dz
    tris = [t for a, b, c, d in faces for t in ((a, b, c), (a, c, d))]
    return vertices, np.array(tris, dtype=np.int64)


def rig_with(parent: dict[str, str | None],
             weights: dict[str, tuple[list[int], list[float]]] | None = None) -> makehuman.Rig:
    return makehuman.Rig(parent=parent, head_joint={b: b + "____head" for b in parent}, joints={},
                         weights={b: (np.array(i, dtype=np.int64), np.array(w))
                                  for b, (i, w) in (weights or {}).items()})


class ConvexHullPerimeter(unittest.TestCase):
    def test_square_perimeter(self):
        corners = [[0, 0], [1, 0], [1, 1], [0, 1]]
        points = np.array(corners + [[0.5, 0.5]], dtype=np.float64)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 4.0, delta=1e-12)

    def test_order_and_duplicates_do_not_matter(self):
        points = np.array([[1, 1], [0, 0], [1, 0], [0.5, 0.5], [0, 1], [1, 0], [0, 0]], dtype=np.float64)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 4.0, delta=1e-12)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points[::-1]), 4.0, delta=1e-12)

    def test_right_triangle(self):
        points = np.array([[0, 0], [3, 0], [0, 4], [1, 1]], dtype=np.float64)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 12.0, delta=1e-12)

    def test_points_on_an_edge_are_not_counted_twice(self):
        points = np.array([[0, 0], [0.5, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 4.0, delta=1e-12)

    def test_degenerate_inputs(self):
        self.assertEqual(measure.convex_hull_perimeter(np.zeros((0, 2))), 0.0)
        self.assertEqual(measure.convex_hull_perimeter(np.array([[2.0, 3.0]])), 0.0)
        # a segment is a polygon walked there and back
        self.assertAlmostEqual(measure.convex_hull_perimeter(np.array([[0, 0], [1, 0], [3, 0]], dtype=float)),
                               6.0, delta=1e-12)

    def test_regular_polygon(self):
        n, r = 36, 0.07
        t = 2.0 * math.pi * np.arange(n) / n
        points = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 2 * n * r * math.sin(math.pi / n),
                               delta=1e-12)


class SectionPoints(unittest.TestCase):
    def test_section_of_a_box(self):
        vertices, tris = box_mesh(0.2, 0.3, 0.1)
        points = measure.section_points(vertices, tris, 0.15, np.ones(8, dtype=bool))
        self.assertEqual(points.shape[1], 2)
        np.testing.assert_allclose(np.ptp(points, axis=0), [0.2, 0.1], atol=1e-12)    # (x, z) extent
        self.assertAlmostEqual(measure.convex_hull_perimeter(points), 0.6, delta=1e-9)

    def test_mask_excludes_triangles(self):
        vertices, tris = box_mesh(0.2, 0.3, 0.1)
        on_the_x0_face = vertices[:, 0] == 0.0
        self.assertEqual(int(on_the_x0_face.sum()), 4)
        points = measure.section_points(vertices, tris, 0.15, on_the_x0_face)
        # Only the two triangles of the x = 0 face have all three vertices in the mask; the
        # other faces' triangles each reach a vertex at x = 0.2 and are skipped.
        self.assertGreaterEqual(len(points), 2)
        np.testing.assert_allclose(points[:, 0], 0.0, atol=1e-12)                      # nothing at x = 0.2
        np.testing.assert_allclose([points[:, 1].min(), points[:, 1].max()], [0.0, 0.1], atol=1e-12)
        self.assertEqual(measure.section_points(vertices, tris, 0.15, np.zeros(8, dtype=bool)).shape, (0, 2))

    def test_plane_outside_the_mesh_is_empty(self):
        vertices, tris = box_mesh(0.2, 0.3, 0.1)
        for y in (-0.01, 0.31):
            self.assertEqual(measure.section_points(vertices, tris, y, np.ones(8, dtype=bool)).shape, (0, 2))

    def test_a_plane_through_vertices_still_cuts(self):
        vertices, tris = box_mesh(0.2, 0.3, 0.1)
        for y in (0.0, 0.3):          # the plane holds a whole face
            points = measure.section_points(vertices, tris, y, np.ones(8, dtype=bool))
            self.assertAlmostEqual(measure.convex_hull_perimeter(points), 0.6, delta=1e-9)

    def test_empty_mesh(self):
        self.assertEqual(measure.section_points(np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), 0.0,
                                                np.zeros(0, dtype=bool)).shape, (0, 2))


def sheet_mesh(heights: np.ndarray, cell: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """An open height field: vertex (i, j) is at (i * cell, heights[i, j], j * cell), two triangles per cell."""
    n, m = heights.shape
    vertices = np.array([[i * cell, heights[i, j], j * cell] for i in range(n) for j in range(m)])
    tris = []
    for i in range(n - 1):
        for j in range(m - 1):
            a, b, c, d = i * m + j, (i + 1) * m + j, (i + 1) * m + j + 1, i * m + j + 1
            tris += [(a, b, c), (a, c, d)]
    return vertices, np.array(tris, dtype=np.int64)


class SectionLoops(unittest.TestCase):
    def test_a_box_is_cut_in_one_loop(self):
        vertices, tris = box_mesh(1.0, 1.0, 1.0)
        self.assertEqual(measure.section_loops(vertices, tris, 0.5, np.ones(8, dtype=bool)), 1)

    def test_two_boxes_are_two_loops_and_the_mask_chooses(self):
        a, ta = box_mesh(1.0, 1.0, 1.0)
        b, tb = box_mesh(1.0, 1.0, 1.0)
        b[:, 0] += 3.0
        vertices, tris = np.vstack([a, b]), np.vstack([ta, tb + 8])
        everything = np.ones(16, dtype=bool)
        self.assertEqual(measure.section_loops(vertices, tris, 0.5, everything), 2)
        self.assertEqual(measure.section_loops(vertices, tris, 0.5, np.arange(16) < 8), 1)
        self.assertEqual(measure.section_loops(vertices, tris, 0.5, np.zeros(16, dtype=bool)), 0)

    def test_a_plane_above_or_below_everything_cuts_nothing(self):
        vertices, tris = box_mesh(1.0, 1.0, 1.0)
        for y in (-0.5, 2.0):
            self.assertEqual(measure.section_loops(vertices, tris, y, np.ones(8, dtype=bool)), 0)

    def test_a_plane_through_vertices_counts_each_loop_once(self):
        # The top face is on the plane: its vertices count as above it, so the box is cut along
        # the side faces in one loop; the bottom face's plane has everything above it: no loop.
        vertices, tris = box_mesh(1.0, 1.0, 1.0)
        self.assertEqual(measure.section_loops(vertices, tris, 1.0, np.ones(8, dtype=bool)), 1)
        self.assertEqual(measure.section_loops(vertices, tris, 0.0, np.ones(8, dtype=bool)), 0)

    def test_a_ridge_between_two_pits_splits_the_section_until_it_is_lower_than_the_cut(self):
        heights = np.full((9, 9), 3.0)
        heights[2:4, 2:7] = 0.0          # a pit
        heights[5:7, 2:7] = 0.0          # another pit, one row of ridge between them (row 4)
        heights[4, 2:7] = 2.0            # the ridge
        vertices, tris = sheet_mesh(heights)
        everything = np.ones(len(vertices), dtype=bool)
        self.assertEqual(measure.section_loops(vertices, tris, 1.0, everything), 2)     # below the ridge
        self.assertEqual(measure.section_loops(vertices, tris, 2.5, everything), 1)     # over the ridge: one pit
        self.assertEqual(measure.section_loops(vertices, tris, 3.5, everything), 0)


class DominantBones(unittest.TestCase):
    PARENT = {"root": None, "a": "root", "b": "root", "c": "a"}

    def test_largest_weight_wins_and_unweighted_is_empty(self):
        rig = rig_with(self.PARENT, {"a": ([0, 1, 2], [0.6, 0.2, 0.5]),
                                     "b": ([1, 2], [0.8, 0.5]),
                                     "c": ([2, 3], [0.1, 1.0])})
        self.assertEqual(measure.dominant_bones(rig, 6), ["a", "b", "a", "c", "", ""])

    def test_weights_on_vertices_beyond_n_are_ignored(self):
        rig = rig_with(self.PARENT, {"a": ([0, 5], [0.3, 1.0]), "b": ([1, 5], [0.4, 0.9])})
        self.assertEqual(measure.dominant_bones(rig, 2), ["a", "b"])

    def test_ties_go_to_the_bone_listed_first(self):
        rig = rig_with(self.PARENT, {"b": ([0], [0.5]), "a": ([0], [0.5])})
        self.assertEqual(measure.dominant_bones(rig, 1), ["b"])

    def test_zero_weights_do_not_count(self):
        rig = rig_with(self.PARENT, {"a": ([0], [0.0])})
        self.assertEqual(measure.dominant_bones(rig, 1), [""])


# --------------------------------------------------------------------------- a synthetic body

SEGMENTS = 24
SHOULDER = {"L": np.array([0.20, 1.40, 0.03]), "R": np.array([-0.20, 1.40, 0.03])}
HIP = {"L": np.array([0.09, 0.90, 0.03]), "R": np.array([-0.09, 0.90, 0.03])}
ARM_LENGTH, LEG_LENGTH = 0.60, 0.85
BREAST_Y, NECK_BASE_Y = 1.30, 1.45
ARM_RADIUS, LEG_RADIUS = 0.04, 0.075
SYNTHETIC_PARENT = {
    "root": None, "spine02": "root", "neck01": "spine02", "neck02": "neck01", "neck03": "neck02",
    "head": "neck03",
    **{f"{bone}.{s}": parent
       for s in "LR"
       for bone, parent in (("clavicle", "spine02"), ("shoulder01", f"clavicle.{s}"),
                            ("upperarm01", f"shoulder01.{s}"), ("wrist", f"upperarm01.{s}"),
                            ("pelvis", "root"), ("upperleg01", f"pelvis.{s}"),
                            ("lowerleg01", f"upperleg01.{s}"))}}


def tube(centres, u, v, ru, rv) -> tuple[np.ndarray, np.ndarray]:
    """A closed-ended-less tube: one ring of SEGMENTS vertices per centre (vertex k at angle
    2 pi k / SEGMENTS in the ring's (u, v) plane, semi-axes ru and rv), and two triangles per
    ring-to-ring quad."""
    centres = np.asarray(centres, dtype=np.float64)
    rings = len(centres)
    theta = 2.0 * np.pi * np.arange(SEGMENTS) / SEGMENTS
    vertices = (centres[:, None, :]
                + (np.asarray(ru)[:, None, None] * np.cos(theta)[None, :, None]) * np.asarray(u)[:, None, :]
                + (np.asarray(rv)[:, None, None] * np.sin(theta)[None, :, None]) * np.asarray(v)[:, None, :])
    i, k = np.arange(rings - 1)[:, None], np.arange(SEGMENTS)[None, :]
    a, b = i * SEGMENTS + k, i * SEGMENTS + (k + 1) % SEGMENTS
    c, d = (i + 1) * SEGMENTS + (k + 1) % SEGMENTS, (i + 1) * SEGMENTS + k
    tris = np.concatenate([np.stack([a, b, c], -1).reshape(-1, 3), np.stack([a, c, d], -1).reshape(-1, 3)])
    return vertices.reshape(-1, 3), tris


def ellipse_polygon_perimeter(a: float, b: float) -> float:
    theta = 2.0 * np.pi * np.arange(SEGMENTS + 1) / SEGMENTS
    return float(np.hypot(np.diff(a * np.cos(theta)), np.diff(b * np.sin(theta))).sum())


def synthetic_body() -> SimpleNamespace:
    """A stick-figure mannequin built from tubes, in Drape's frame, with the features the
    definitions react to:

    * torso: an elliptical tube centred at z = 0.03, y 0.80 to 1.45, widest at y = 0.95 (hips,
      a = 0.17), narrowest at 1.10 (waist, a = 0.12), wide again at 1.30 (chest, a = 0.16), plus
      free vertices: one "pec" on each side at y = 1.30, z = 0.16 (so the chest has a mirror-image
      tie) and one "buttock" at y = 0.95, z = -0.12 (so the body axis is not the torso's mid z);
    * neck: a tube that flares at its base (radius 0.09 at y = 1.45 down to 0.05 at 1.50), the
      way the trapezius does on the real body, off-centre at z = 0.04;
    * head: a tube at z = 0.06, y 1.47 to 1.80: radius 0.10 up to y = 1.60 (a skirt of head-weighted
      vertices around the neck, as the levator bones give the real body), narrowing to 0.08;
    * arms at 45 degrees from the shoulder joints, with a fat cap behind the joint (radius 0.08)
      and a fat hand past the wrist (0.07) that the cylinder radius must not count;
    * legs at 5 degrees from the hip joints (y = 0.90), starting at y = 0.80 with a fat foot
      ring at the far end; at y = 0.80 the thigh tops make the widest section below the waist.
    """
    parts: list[tuple[str, np.ndarray, np.ndarray, str]] = []

    ys = np.linspace(0.80, NECK_BASE_Y, 66)
    half_width = np.interp(ys, [0.80, 0.95, 1.10, 1.30, 1.45], [0.14, 0.17, 0.12, 0.16, 0.12])
    centres = np.stack([np.zeros(66), ys, np.full(66, 0.03)], axis=1)
    x_axis, z_axis = np.tile([1.0, 0.0, 0.0], (66, 1)), np.tile([0.0, 0.0, 1.0], (66, 1))
    v, t = tube(centres, x_axis, z_axis, half_width, np.full(66, 0.10))
    parts.append(("torso", v, t, "spine02"))

    ys = np.linspace(NECK_BASE_Y, 1.60, 16)
    radius = np.interp(ys, [NECK_BASE_Y, 1.50, 1.60], [0.09, 0.05, 0.05])
    v, t = tube(np.stack([np.zeros(16), ys, np.full(16, 0.04)], axis=1), x_axis[:16], z_axis[:16],
                radius, radius)
    parts.append(("neck", v, t, "neck02"))

    ys = np.linspace(1.47, 1.80, 34)
    radius = np.interp(ys, [1.47, 1.60, 1.62], [0.10, 0.10, 0.08])
    v, t = tube(np.stack([np.zeros(34), ys, np.full(34, 0.06)], axis=1), x_axis[:34], z_axis[:34], radius, radius)
    parts.append(("head", v, t, "head"))

    joints = {}
    for side, sx in (("L", 1.0), ("R", -1.0)):
        arm_dir = np.array([sx * math.sin(math.radians(45)), -math.cos(math.radians(45)), 0.0])
        ts = np.arange(-0.05, 0.7501, 0.05)
        radius = np.where(ts < -1e-9, 0.08, np.where(ts > ARM_LENGTH + 1e-9, 0.07, ARM_RADIUS))
        u = np.array([sx * math.cos(math.radians(45)), math.sin(math.radians(45)), 0.0])
        v, t = tube(SHOULDER[side] + ts[:, None] * arm_dir, np.tile(u, (len(ts), 1)),
                    np.tile([0.0, 0.0, 1.0], (len(ts), 1)), radius, radius)
        parts.append((f"arm.{side}", v, t, f"upperarm01.{side}"))

        leg_dir = np.array([sx * math.sin(math.radians(5)), -math.cos(math.radians(5)), 0.0])
        ts = np.arange(0.10, 0.9001, 0.05)
        radius = np.where(ts > LEG_LENGTH + 1e-9, 0.10, LEG_RADIUS)
        u = np.array([sx * math.cos(math.radians(5)), math.sin(math.radians(5)), 0.0])
        v, t = tube(HIP[side] + ts[:, None] * leg_dir, np.tile(u, (len(ts), 1)),
                    np.tile([0.0, 0.0, 1.0], (len(ts), 1)), radius, radius)
        parts.append((f"leg.{side}", v, t, f"lowerleg01.{side}"))

        joints[f"upperarm01.{side}____head"] = SHOULDER[side].copy()
        joints[f"wrist.{side}____head"] = SHOULDER[side] + ARM_LENGTH * arm_dir
        joints[f"upperleg01.{side}____head"] = HIP[side].copy()
        joints[f"foot.{side}____head"] = HIP[side] + LEG_LENGTH * leg_dir
    joints["breast.L____head"] = np.array([0.07, BREAST_Y, 0.05])
    joints["neck01____head"] = np.array([0.0, NECK_BASE_Y, 0.03])
    joints["eye.L____head"] = np.array([0.03, 1.70, 0.12])

    vertices, tris, dominant, ranges = [], [], [], {}
    for name, v, t, bone in parts:
        ranges[name] = np.arange(sum(map(len, vertices)), sum(map(len, vertices)) + len(v))
        tris.append(t + sum(map(len, vertices)))
        vertices.append(v)
        dominant += [bone] * len(v)
    ranges["egg_ring"] = ranges["head"][23 * SEGMENTS:24 * SEGMENTS]       # the ring at y = 1.70, on the egg
    free = np.array([[0.07, BREAST_Y, 0.16], [-0.07, BREAST_Y, 0.16], [0.0, 0.95, -0.12]])    # in no triangle
    ranges["free"] = np.arange(sum(map(len, vertices)), sum(map(len, vertices)) + 3)
    vertices.append(free)
    dominant += ["spine02"] * 3
    return SimpleNamespace(vertices=np.concatenate(vertices), tris=np.concatenate(tris), dominant=dominant,
                           rig=rig_with(SYNTHETIC_PARENT), joints=joints, ranges=ranges,
                           egg=style.Egg(np.array([0.0, 1.70, 0.06]), np.array([0.08, 0.10, 0.08])))


Z_AXIS = 0.045         # mid z-extent of the torso vertices within 1 cm of y = 1.30: (-0.07 + 0.16) / 2
NECK_SECTION_RADIUS = 0.079      # the flared neck at y = 1.47 is a circle of radius 0.074 at z = 0.04


class LandmarksRegionsGirths(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.body = synthetic_body()
        b = cls.body
        cls.result = measure.measure_body(b.vertices, b.tris, b.dominant, b.rig, b.joints, b.egg,
                                          head_vertices=b.ranges["egg_ring"])
        cls.lm, cls.regions, cls.girths = (cls.result.landmarks, cls.result.regions, cls.result.girths)

    def test_the_brief_interface_returns_the_same_three_dicts(self):
        b = self.body
        landmarks, regions, girths = measure.landmarks_regions_girths(b.vertices, b.tris, b.dominant, b.rig,
                                                                      b.joints, b.egg)
        self.assertEqual(set(landmarks), set(self.lm))
        self.assertEqual(girths, self.girths)
        self.assertEqual(set(regions), set(self.regions))

    def test_landmark_names(self):
        self.assertEqual(set(self.lm), {
            "hps_left", "hps_right", "shoulder_left", "shoulder_right", "neck_base", "chest", "waist", "hip",
            "crotch", "ankle_left", "ankle_right", "shoulder_joint_left", "shoulder_joint_right",
            "wrist_left", "wrist_right", "hip_joint_left", "hip_joint_right"})
        for name, p in self.lm.items():
            self.assertEqual(p.shape, (3,), name)

    def test_neck_base_is_on_the_body_axis_at_the_neck_joint_height(self):
        np.testing.assert_allclose(self.lm["neck_base"], [0.0, NECK_BASE_Y, Z_AXIS], atol=1e-9)

    def test_chest_is_the_most_forward_torso_vertex_and_a_tie_goes_to_the_left(self):
        np.testing.assert_allclose(self.lm["chest"], [0.07, BREAST_Y, 0.16], atol=1e-9)
        self.assertGreater(self.lm["chest"][2], self.regions["Torso"]["origin"][2] + 0.05)

    def test_crotch_is_the_lowest_vertex_near_the_midline_below_the_hip_joint(self):
        self.assertLessEqual(abs(self.lm["crotch"][0]), 0.01)
        self.assertAlmostEqual(self.lm["crotch"][1], 0.80, delta=1e-9)

    def test_waist_is_the_narrowest_level_and_hip_the_widest_above_the_hip_joint(self):
        np.testing.assert_allclose(self.lm["waist"], [0.0, 1.10, Z_AXIS], atol=1e-9)
        # The thigh tops at y = 0.80 are wider (hull 0.880) than the hips at y = 0.95 (0.860), so a
        # search from the crotch would put the hip on the crotch. The search starts at the hip joint.
        np.testing.assert_allclose(self.lm["hip"], [0.0, 0.95, Z_AXIS], atol=1e-9)

    def test_shoulder_point_is_the_highest_vertex_within_1_5_cm_of_the_joint(self):
        b = self.body
        for side, sign in (("left", 1), ("right", -1)):
            joint = SHOULDER["L" if sign > 0 else "R"]
            near = np.hypot(b.vertices[:, 0] - joint[0], b.vertices[:, 2] - joint[2]) <= 0.015
            top = b.vertices[near][np.argmax(b.vertices[near, 1])]
            np.testing.assert_allclose(self.lm[f"shoulder_{side}"], top, atol=1e-6)
            self.assertLessEqual(np.hypot(*(self.lm[f"shoulder_{side}"][[0, 2]] - joint[[0, 2]])), 0.015 + 1e-6)

    def test_hps_sits_at_the_side_of_the_neck_base_not_on_the_head_or_the_neck_column(self):
        # The band starts at the neck's own radius (0.079 at y = 1.47). Head vertices at x = 0.08
        # are inside the band and higher, so the head must be left out. The flare at y = 1.46
        # (radius 0.082) is the highest non-head vertex in it; of the three vertices tied there, the
        # one farthest from the midline wins.
        np.testing.assert_allclose(self.lm["hps_left"], [0.082, 1.46, 0.04], atol=1e-6)
        np.testing.assert_allclose(self.lm["hps_right"], [-0.082, 1.46, 0.04], atol=1e-6)

    def test_ankles_and_joint_landmarks_are_the_posed_joints(self):
        for side, key in (("left", "L"), ("right", "R")):
            np.testing.assert_allclose(self.lm[f"ankle_{side}"], self.body.joints[f"foot.{key}____head"], atol=1e-6)
            np.testing.assert_allclose(self.lm[f"shoulder_joint_{side}"],
                                       self.body.joints[f"upperarm01.{key}____head"], atol=1e-6)
            np.testing.assert_allclose(self.lm[f"wrist_{side}"], self.body.joints[f"wrist.{key}____head"], atol=1e-6)
            np.testing.assert_allclose(self.lm[f"hip_joint_{side}"],
                                       self.body.joints[f"upperleg01.{key}____head"], atol=1e-6)

    def test_girths_are_hull_perimeters_of_the_non_arm_sections(self):
        self.assertEqual(set(self.girths), {"chest", "waist", "hip", "neck"})
        self.assertAlmostEqual(self.girths["chest"], ellipse_polygon_perimeter(0.16, 0.10), delta=1e-5)
        self.assertAlmostEqual(self.girths["waist"], ellipse_polygon_perimeter(0.12, 0.10), delta=1e-5)
        self.assertAlmostEqual(self.girths["hip"], ellipse_polygon_perimeter(0.17, 0.10), delta=1e-5)
        # the neck girth is taken 2 cm above the neck base over neck vertices only: a circle of 0.074
        # (the head skirt there is 0.10 wide)
        self.assertAlmostEqual(self.girths["neck"], ellipse_polygon_perimeter(0.074, 0.074), delta=1e-5)

    def test_all_eight_regions_use_the_body_region_names(self):
        self.assertEqual(set(self.regions), {"Torso", "Neck", "Head", "ArmLeft", "ArmRight", "LegLeft", "LegRight",
                                             "Waist"})
        for name, cyl in self.regions.items():
            self.assertEqual(set(cyl), {"origin", "axis", "e0", "radius"}, name)

    def test_vertical_regions(self):
        up, forward = [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]
        for name, origin in (("Torso", [0.0, NECK_BASE_Y, Z_AXIS]), ("Neck", [0.0, NECK_BASE_Y, Z_AXIS]),
                             ("Head", [0.0, 1.70, 0.06]), ("Waist", [0.0, 1.10, Z_AXIS])):
            cyl = self.regions[name]
            np.testing.assert_allclose(cyl["origin"], origin, atol=1e-9, err_msg=name)
            np.testing.assert_array_equal(cyl["axis"], up, err_msg=name)
            np.testing.assert_array_equal(cyl["e0"], forward, err_msg=name)

    def test_radii(self):
        r = {name: cyl["radius"] for name, cyl in self.regions.items()}
        self.assertAlmostEqual(r["Torso"], math.hypot(0.17, 0.015), delta=2e-6)    # hips, from the body axis
        self.assertAlmostEqual(r["Neck"], 0.095, delta=2e-6)         # the flare's back, 0.09 + 0.005 off axis
        self.assertAlmostEqual(r["Head"], 0.08, delta=1e-9)          # max(ax, az) of the egg
        for side in ("Left", "Right"):
            self.assertAlmostEqual(r[f"Arm{side}"], ARM_RADIUS, delta=2e-6)    # not the cap or the hand
            self.assertAlmostEqual(r[f"Leg{side}"], LEG_RADIUS, delta=2e-6)    # not the foot ring

    def test_limb_axes_run_from_the_joint_to_the_wrist_or_ankle(self):
        root2 = math.sqrt(0.5)
        for name, sx in (("ArmLeft", 1.0), ("ArmRight", -1.0)):
            cyl = self.regions[name]
            np.testing.assert_allclose(cyl["origin"], SHOULDER["L" if sx > 0 else "R"], atol=1e-9)
            np.testing.assert_allclose(cyl["axis"], [sx * root2, -root2, 0.0], atol=2e-6)
            np.testing.assert_allclose(cyl["e0"], [sx * root2, root2, 0.0], atol=2e-6)     # +Y less its axis part
        for name, sx in (("LegLeft", 1.0), ("LegRight", -1.0)):
            cyl = self.regions[name]
            np.testing.assert_allclose(cyl["origin"], HIP["L" if sx > 0 else "R"], atol=1e-9)
            np.testing.assert_allclose(cyl["axis"], [sx * math.sin(math.radians(5)), -math.cos(math.radians(5)), 0.0],
                                       atol=2e-6)
            np.testing.assert_allclose(cyl["e0"], [0.0, 0.0, 1.0], atol=2e-6)              # +Z is already across it

    def test_waist_radius_counts_every_non_arm_vertex_between_crotch_and_waist(self):
        b = self.body
        non_arm = np.array([not bone.startswith("upperarm01") for bone in b.dominant])
        inside = non_arm & (b.vertices[:, 1] >= 0.80 - 1e-12) & (b.vertices[:, 1] <= 1.10 + 1e-12)
        np.testing.assert_array_equal(self.result.region_vertices["Waist"], np.flatnonzero(inside))

    def test_region_vertices_lie_inside_their_cylinders_and_the_radius_is_tight(self):
        v = self.body.vertices
        self.assertEqual(set(self.result.region_vertices), set(self.regions))
        for name, cyl in self.regions.items():
            indices = self.result.region_vertices[name]
            self.assertGreater(len(indices), 0, name)
            self.assertTrue(np.all(np.diff(indices) > 0), f"{name}: ascending, no repeats")
            axis = cyl["axis"] / np.linalg.norm(cyl["axis"])
            offset = v[indices] - cyl["origin"]
            distance = np.linalg.norm(offset - (offset @ axis)[:, None] * axis, axis=1)
            self.assertLessEqual(distance.max(), cyl["radius"] + 1e-12, name)
            if name != "Head":
                self.assertGreater(distance.max(), cyl["radius"] - 1.1e-6, name)

    def test_head_region_vertices_are_the_ones_the_caller_gave(self):
        np.testing.assert_array_equal(self.result.region_vertices["Head"], self.body.ranges["egg_ring"])

    def test_limb_vertices_are_limited_to_the_axial_range(self):
        v = self.body.vertices
        for name, length in (("ArmLeft", ARM_LENGTH), ("ArmRight", ARM_LENGTH), ("LegLeft", LEG_LENGTH),
                             ("LegRight", LEG_LENGTH)):
            cyl = self.regions[name]
            axial = (v[self.result.region_vertices[name]] - cyl["origin"]) @ cyl["axis"]
            self.assertGreaterEqual(axial.min(), -1e-6, name)
            self.assertLessEqual(axial.max(), length + 1e-6, name)

    def test_everything_written_is_on_the_microns_grid(self):
        def on_grid(x):
            x = np.atleast_1d(np.asarray(x, dtype=float))
            return np.all(np.abs(x * 1e6 - np.round(x * 1e6)) < 1e-6)
        for name, p in self.lm.items():
            self.assertTrue(on_grid(p), name)
        for name, cyl in self.regions.items():
            for key, value in cyl.items():
                self.assertTrue(on_grid(value), f"{name}.{key}")
        for name, g in self.girths.items():
            self.assertTrue(on_grid(g), name)

    def test_mirror_image_body_has_mirror_image_landmarks(self):
        for name in ("hps", "shoulder", "ankle", "shoulder_joint", "wrist", "hip_joint"):
            left, right = self.lm[f"{name}_left"], self.lm[f"{name}_right"]
            np.testing.assert_allclose(right, left * [-1, 1, 1], atol=2e-6, err_msg=name)
        for left, right in (("ArmLeft", "ArmRight"), ("LegLeft", "LegRight")):
            self.assertAlmostEqual(self.regions[left]["radius"], self.regions[right]["radius"], delta=2e-6)

    def test_no_torso_vertex_at_chest_height_is_an_error(self):
        b = self.body
        joints = dict(b.joints)
        joints["breast.L____head"] = np.array([0.07, 3.0, 0.05])
        with self.assertRaisesRegex(ValueError, "chest"):
            measure.measure_body(b.vertices, b.tris, b.dominant, b.rig, joints, b.egg,
                                 head_vertices=b.ranges["egg_ring"])


if __name__ == "__main__":
    unittest.main()
