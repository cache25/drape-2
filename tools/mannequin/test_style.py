"""Offline unit tests for style.py: synthetic meshes only, fixed seeds."""
import unittest
from unittest import mock

import numpy as np

import makehuman
import measure
import pose
import style

EGG = style.Egg(np.array([0.0, 1.62, 0.02]), np.array([0.075, 0.115, 0.095]))


def cube_sphere(n: int) -> tuple[np.ndarray, np.ndarray]:
    """Unit sphere from a subdivided cube: n x n quads per face, closed, no poles, no
    degenerate quads (like MakeHuman's all-quad body). Even n puts a vertex on each axis
    and a ring on every equator. Quads are counter-clockwise seen from outside."""
    t = np.linspace(-1.0, 1.0, n + 1)
    ids: dict[tuple[float, ...], int] = {}
    cube: list[tuple[float, ...]] = []

    def vertex(axis: int, sign: float, a: float, b: float) -> int:
        point = [0.0, 0.0, 0.0]
        point[axis] = sign
        point[(axis + 1) % 3], point[(axis + 2) % 3] = a, b
        key = tuple(round(c, 9) + 0.0 for c in point)       # + 0.0 folds -0.0 into 0.0
        if key not in ids:
            ids[key] = len(cube)
            cube.append(key)
        return ids[key]

    quads = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            for i in range(n):
                for j in range(n):
                    quads.append([vertex(axis, sign, t[a], t[b])
                                  for a, b in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1))])
    points = np.array(cube)
    points /= np.linalg.norm(points, axis=1, keepdims=True)
    quads = np.array(quads, dtype=np.int64)
    inward = (style._quad_normals(points, quads) * points[quads].mean(axis=1)).sum(axis=1) < 0
    quads[inward] = quads[inward][:, ::-1]
    return points, quads


def outward(vertices: np.ndarray, quads: np.ndarray, centre: np.ndarray) -> np.ndarray:
    """Each quad's normal . (quad centroid - centre): positive when the quad faces outward."""
    centroids = vertices[quads].mean(axis=1)
    return (style._quad_normals(vertices, quads) * (centroids - centre)).sum(axis=1)


def nosed_sphere() -> tuple[np.ndarray, np.ndarray]:
    """The unit sphere scaled to r = 0.1 at the egg's centre, with a 2 cm "nose": the front pushed
    forward along +z (not radially)."""
    points, quads = cube_sphere(16)
    vertices = EGG.centre + 0.1 * points
    d2 = vertices[:, 0] ** 2 + (vertices[:, 1] - (EGG.centre[1] + 0.01)) ** 2
    vertices[:, 2] += 0.02 * np.exp(-d2 / (2 * 0.02 ** 2)) * (points[:, 2] > 0)
    return vertices, quads


def rig_with(parent: dict[str, str | None], weights: dict[str, tuple[list[int], list[float]]]) -> makehuman.Rig:
    return makehuman.Rig(parent=parent, head_joint={b: b + "_h" for b in parent}, joints={},
                         weights={b: (np.array(i, dtype=np.int64), np.array(w)) for b, (i, w) in weights.items()})


class RegionWeights(unittest.TestCase):
    PARENT = {"root": None, "spine": "root", "head": "spine", "jaw": "head", "eye.L": "head",
              "foot.L": "root", "toe.L": "foot.L", "foot.R": "root", "toe.R": "foot.R"}
    WEIGHTS = {"head": ([0, 1, 2], [1.0, 0.7, 0.4]), "jaw": ([1, 2, 3], [0.6, 0.2, 0.5]),
               "eye.L": ([4], [0.5]), "spine": ([0, 5], [0.3, 1.0]),
               "foot.L": ([6, 7], [1.0, 0.4]), "toe.L": ([7, 8], [0.6, 1.0])}

    def setUp(self):
        self.rig = rig_with(self.PARENT, self.WEIGHTS)

    def test_head_sums_the_head_bone_and_its_descendants_clipped_to_one(self):
        w = style.region_weights(self.rig, ["head"], 9)
        self.assertEqual(w.shape, (9,))
        # v0: head only (spine ignored); v1: 0.7 + 0.6 clips; v2: 0.4 + 0.2; v3: jaw; v4: eye
        np.testing.assert_allclose(w, [1.0, 1.0, 0.6, 0.5, 0.5, 0, 0, 0, 0], atol=1e-12)

    def test_exclude_drops_those_bones_but_keeps_their_descendants(self):
        w = style.region_weights(self.rig, ["foot.L", "foot.R"], 9, exclude=frozenset({"foot.L", "foot.R"}))
        np.testing.assert_allclose(w, [0, 0, 0, 0, 0, 0, 0, 0.6, 1.0], atol=1e-12)
        everything = style.region_weights(self.rig, ["foot.L", "foot.R"], 9)
        np.testing.assert_allclose(everything[6:], [1.0, 1.0, 1.0], atol=1e-12)    # 0.4 + 0.6 clips

    def test_weights_on_vertices_beyond_n_are_ignored(self):
        # the rig also weights helper geometry that follows the body's vertices in base.obj
        w = style.region_weights(self.rig, ["head"], 3)
        np.testing.assert_allclose(w, [1.0, 1.0, 0.6], atol=1e-12)

    def test_unknown_root_is_an_error(self):
        with self.assertRaises(KeyError):
            style.region_weights(self.rig, ["tail"], 9)


class EggShape(unittest.TestCase):
    def test_project_lands_on_surface(self):
        rng = np.random.default_rng(7)
        d = rng.normal(size=(500, 3))
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        for scale in (0.02, 0.3):                      # starting inside and far outside the egg
            landed = style.project_to_egg(EGG, EGG.centre + scale * d)
            np.testing.assert_allclose(style.egg_value(EGG, landed), 1.0, atol=1e-9)
            # on the same ray from the centre
            along = np.cross(landed - EGG.centre, d)
            np.testing.assert_allclose(along, 0.0, atol=1e-12)
            self.assertTrue(np.all(((landed - EGG.centre) * d).sum(axis=1) > 0))

    def test_project_accepts_a_single_point(self):
        p = style.project_to_egg(EGG, EGG.centre + np.array([0.3, 0.1, -0.2]))
        self.assertEqual(p.shape, (3,))
        self.assertAlmostEqual(float(style.egg_value(EGG, p)), 1.0, places=9)

    def test_project_survives_a_point_at_the_centre(self):
        p = style.project_to_egg(EGG, EGG.centre.copy())
        self.assertTrue(np.all(np.isfinite(p)))
        self.assertAlmostEqual(float(style.egg_value(EGG, p)), 1.0, places=9)

    @staticmethod
    def half_width(axis: int, q_y: float) -> float:
        """Surface half-extent of EGG along x (axis 0) or z (axis 2) at height centre_y + q_y."""
        lo, hi = 0.0, 1.0
        for _ in range(100):
            mid = (lo + hi) / 2
            offset = np.zeros(3)
            offset[axis], offset[1] = mid, q_y
            lo, hi = (mid, hi) if style.egg_value(EGG, EGG.centre + offset) < 1 else (lo, mid)
        return lo

    def test_taper_halfway_down(self):
        ax, ay, az = EGG.radii
        qy = -ay / 2
        k = 1 - 0.15 * 0.5 ** 2            # k = 1 - (1 - taper) (-q_y / a_y)^2
        # The horizontal ellipse at height q_y has radii (ax, az) * k * sqrt(1 - (q_y/ay)^2);
        # the factor k * (ax, az) is the "horizontal radius" the taper scales.
        s = np.sqrt(1 - (qy / ay) ** 2)
        self.assertAlmostEqual(self.half_width(0, qy) / s, ax * k, places=9)
        self.assertAlmostEqual(self.half_width(2, qy) / s, az * k, places=9)
        # at the same elliptical fraction above the centre the radius is the full ax
        self.assertAlmostEqual(self.half_width(0, -qy) / s, ax, places=9)
        # and at the bottom pole the taper is the full 0.85: no extra shrink
        self.assertAlmostEqual(float(style.egg_value(EGG, EGG.centre + [ax * 0.85, -ay, 0.0])), 2.0, places=9)

    def test_the_taper_has_no_kink_at_the_centre(self):
        # The side of the egg is smooth through the equator: the half-width a little below the centre
        # matches the one the same distance above to second order (a linear taper is off to first order).
        ax, ay, _ = EGG.radii
        for delta in (1e-3, 1e-4):
            above, below = self.half_width(0, delta * ay), self.half_width(0, -delta * ay)
            self.assertLess(abs(above - below), 0.2 * ax * delta * delta)

    def test_egg_value_is_one_on_the_axis_poles(self):
        ax, ay, az = EGG.radii
        for offset in ([0, ay, 0], [0, -ay, 0], [ax, 0, 0], [-ax, 0, 0], [0, 0, az], [0, 0, -az]):
            self.assertAlmostEqual(float(style.egg_value(EGG, EGG.centre + offset)), 1.0, places=12)


class FitEgg(unittest.TestCase):
    CENTRE = np.array([0.0, 1.6, 0.01])
    RADII = np.array([0.07, 0.11, 0.09])

    def ellipsoid(self):
        points, quads = cube_sphere(16)
        return self.CENTRE + points * self.RADII, quads

    def test_fit_recovers_ellipsoid(self):
        vertices, _ = self.ellipsoid()
        egg = style.fit_egg(vertices, np.ones(len(vertices)), eye_y=1.6 - 0.03)
        np.testing.assert_allclose(egg.centre, self.CENTRE, atol=1e-3)
        np.testing.assert_allclose(egg.radii, self.RADII, atol=1e-3)
        self.assertEqual(egg.taper, 0.85)

    def test_ears_below_the_measuring_band_are_ignored(self):
        vertices, _ = self.ellipsoid()
        eye_y = 1.6 - 0.03
        ears = (np.abs(vertices[:, 1] - eye_y) <= 0.015) & (np.abs(vertices[:, 0]) > 0.04)
        self.assertGreater(ears.sum(), 0)
        vertices[ears, 0] *= 1.4
        egg = style.fit_egg(vertices, np.ones(len(vertices)), eye_y=eye_y)
        np.testing.assert_allclose(egg.radii, self.RADII, atol=1e-3)

    def test_only_vertices_with_full_head_weight_count(self):
        vertices, _ = self.ellipsoid()
        w = np.ones(len(vertices))
        shoulders = np.array([[0.2, 1.3, 0.0], [-0.2, 1.3, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0],
                              [0.2, 1.6, 0.3]])
        vertices = np.vstack([vertices, shoulders])
        w = np.concatenate([w, [0.998, 0.5, 0.0, 0.9, 0.0]])
        egg = style.fit_egg(vertices, w, eye_y=1.6 - 0.03)
        np.testing.assert_allclose(egg.centre, self.CENTRE, atol=1e-3)
        np.testing.assert_allclose(egg.radii, self.RADII, atol=1e-3)

    def test_measured_section_is_corrected_to_the_equator(self):
        # A point cloud with exact rings: poles, and a ring half-way up at t = 0.5.
        c, r = np.array([0.0, 1.7, -0.02]), np.array([0.08, 0.12, 0.1])
        t = 0.5
        phi = np.linspace(0, 2 * np.pi, 72, endpoint=False)
        ring = np.stack([r[0] * np.sqrt(1 - t * t) * np.cos(phi),
                         np.full_like(phi, r[1] * t),
                         r[2] * np.sqrt(1 - t * t) * np.sin(phi)], axis=1)
        points = c + np.vstack([ring, [[0, r[1], 0], [0, -r[1], 0]]])
        eye_y = c[1] + r[1] * t - 0.03
        egg = style.fit_egg(points, np.ones(len(points)), eye_y=eye_y)
        np.testing.assert_allclose(egg.centre, c, atol=1e-9)
        np.testing.assert_allclose(egg.radii, r, atol=1e-9)

    def test_no_vertices_in_the_band_is_an_error(self):
        vertices, _ = self.ellipsoid()
        with self.assertRaises(ValueError):
            style.fit_egg(vertices, np.ones(len(vertices)), eye_y=3.0)
        with self.assertRaises(ValueError):
            style.fit_egg(vertices, np.zeros(len(vertices)), eye_y=1.6)


def mirror_of(vertices: np.ndarray) -> np.ndarray:
    """Index of each vertex's mirror image (x -> -x), matched to 1e-9 m; -1 where there is none."""
    key = {tuple(np.round(p, 9) + 0.0): i for i, p in enumerate(vertices)}
    return np.array([key.get(tuple(np.round([-p[0], p[1], p[2]], 9) + 0.0), -1) for p in vertices])


def crossings(vertices: np.ndarray, quads: np.ndarray) -> int:
    """Triangle pairs of the quads' (shorter-diagonal) triangulation that pass through each other."""
    return len(measure.intersecting_pairs(vertices, pose.triangulate(vertices, quads)))


def neck_weights(vertices: np.ndarray) -> np.ndarray:
    """A head region like the real one: everything above 6 cm under the egg centre is head (w = 1),
    blending to 0 over the 2 cm below it, so the region has a single boundary loop round the neck."""
    t = np.clip((vertices[:, 1] - (EGG.centre[1] - 0.08)) / 0.02, 0.0, 1.0)
    return t


class EggHead(unittest.TestCase):
    def test_egg_head_removes_bumps(self):
        vertices, quads = nosed_sphere()
        self.assertGreater(vertices[:, 2].max() - EGG.centre[2], 0.115)      # the bump is there
        out = style.make_egg_head(vertices, quads, np.ones(len(vertices)), EGG)
        self.assertEqual(out.shape, vertices.shape)
        err = np.linalg.norm(out - style.project_to_egg(EGG, out), axis=1)
        self.assertLess(err.max(), 1e-6)
        self.assertGreater(outward(out, quads, EGG.centre).min(), 0.0)

    def test_pinned_vertices_go_all_the_way_in_the_blend(self):
        # w = 0.9995 is "pinned": it lands on the egg at once, not 99.95 % of the way there.
        vertices, quads = nosed_sphere()
        w = np.full(len(vertices), 0.9995)
        with mock.patch.object(style, "_HEAD_ROUNDS", 0):
            out = style.make_egg_head(vertices, quads, w, EGG)
        np.testing.assert_allclose(style.egg_value(EGG, out), 1.0, atol=1e-9)

    def test_egg_head_relaxes_the_mesh_not_just_the_projection(self):
        vertices, quads = nosed_sphere()
        projected = style.project_to_egg(EGG, vertices)
        out = style.make_egg_head(vertices, quads, np.ones(len(vertices)), EGG)
        self.assertGreater(np.linalg.norm(out - projected, axis=1).max(), 1e-3)

    def test_zero_weight_vertices_never_move(self):
        vertices, quads = nosed_sphere()
        w = neck_weights(vertices)
        out = style.make_egg_head(vertices, quads, w, EGG)
        still = w == 0
        self.assertGreater(still.sum(), 0)
        self.assertEqual(out[still].tobytes(), vertices[still].tobytes())
        self.assertGreater(np.linalg.norm(out[w > 0] - vertices[w > 0], axis=1).max(), 1e-4)
        top = w >= 0.999
        np.testing.assert_allclose(style.egg_value(EGG, out[top]), 1.0, atol=1e-9)

    def test_input_is_not_modified_and_output_is_deterministic(self):
        vertices, quads = nosed_sphere()
        before = vertices.copy()
        w = neck_weights(vertices)
        first = style.make_egg_head(vertices, quads, w, EGG)
        second = style.make_egg_head(vertices, quads, w, EGG)
        np.testing.assert_array_equal(vertices, before)
        self.assertEqual(first.tobytes(), second.tobytes())

    def test_a_mirror_symmetric_head_stays_exactly_mirror_symmetric(self):
        vertices, quads = nosed_sphere()
        partner = mirror_of(vertices)
        self.assertTrue((partner >= 0).all())
        out = style.make_egg_head(vertices, quads, neck_weights(vertices), EGG)
        np.testing.assert_array_equal(out[partner] * [-1.0, 1.0, 1.0], out)

    def test_flip_guard_reverts_every_vertex_of_a_quad_that_turns_inward(self):
        c = np.array([0.0, 0.0, -1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)    # faces +z, away from c
        after = before.copy()
        after[2] = [-0.5, -0.5, 0.0]                  # drags the far corner through the quad: it now faces -z
        after[0] += [0.05, 0.0, 0.0]                  # a harmless nudge that must also be undone with it
        quads = np.array([[0, 1, 2, 3]])
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), before)

    def test_flip_guard_keeps_moves_that_leave_quads_facing_outward(self):
        c = np.array([0.0, 0.0, -1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)
        after = before + np.array([[0.1, 0, 0], [0.1, 0.05, 0], [0.1, 0.1, 0], [0.1, 0.0, 0.2]])
        quads = np.array([[0, 1, 2, 3]])
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), after)

    def test_flip_guard_lets_an_inward_quad_turn_outward(self):
        c = np.array([0.0, 0.0, 1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)    # faces +z, toward c
        after = before.copy()
        after[2] = [-0.5, -0.5, 0.0]                  # faces -z, away from c
        quads = np.array([[0, 1, 2, 3]])
        self.assertLess(outward(before, quads, c)[0], 0)
        self.assertGreater(outward(after, quads, c)[0], 0)
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), after)

    def test_flip_guard_ignores_quads_that_were_already_inward(self):
        c = np.array([0.0, 0.0, 1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)    # faces +z, toward c
        after = before + np.array([[0.1, 0, 0], [0.1, 0.05, 0], [0.1, 0.1, 0], [0.1, 0.0, -0.2]])
        quads = np.array([[0, 1, 2, 3]])
        self.assertLess(outward(after, quads, c)[0], 0)
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), after)

    def test_flip_guard_cascades_to_neighbouring_quads(self):
        c = np.array([1.0, 0.5, -1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [2, 0, 0], [2, 1, 0]], dtype=float)
        quads = np.array([[0, 1, 2, 3], [1, 4, 5, 2]])
        after = before.copy()
        after[1] = [3.0, 0.0, 0.0]
        after[3] = [2.0, 0.5, 0.0]
        after[5] = [0.0, -1.0, 0.0]
        faced_out = outward(before, quads, c) > 0
        turned_in = faced_out & (outward(after, quads, c) <= 0)
        self.assertEqual(turned_in.tolist(), [False, True])
        out = style._revert_flips(before, after, quads, c)
        self.assertTrue(np.all(outward(out, quads, c) > 0))
        np.testing.assert_array_equal(out, before)

    def test_flip_guard_looks_at_every_corner_triangle(self):
        c = np.array([0.0, 0.0, -1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)
        after = before.copy()
        after[2] = [0.3, 0.3, 0.0]
        quads = np.array([[0, 1, 2, 3]])
        self.assertGreater(outward(after, quads, c)[0], 0.0)
        facing, _ = style._corners(after, quads, c)
        self.assertEqual((facing[0] <= 0.0).tolist(), [False, False, True, False])
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), before)

    def test_flip_guard_will_not_squash_a_corner_below_the_area_floor(self):
        c = np.array([0.0, 0.0, -1.0])
        before = np.array([[0, 0, 0], [0.01, 0, 0], [0.01, 0.01, 0], [0, 0.01, 0]], dtype=float)
        quads = np.array([[0, 1, 2, 3]])
        squashed = before.copy()
        squashed[2:, 1] = 1e-7
        _, area = style._corners(squashed, quads, c)
        self.assertLess(area.max(), style._AREA_FLOOR)
        np.testing.assert_array_equal(style._revert_flips(before, squashed, quads, c), before)
        thin = before.copy()
        thin[2:, 1] = 1e-3
        np.testing.assert_array_equal(style._revert_flips(before, thin, quads, c), thin)

    def test_flip_guard_lets_a_tiny_corner_grow_or_stay(self):
        c = np.array([0.0, 0.0, -1.0])
        quads = np.array([[0, 1, 2, 3]])
        sliver = np.array([[0, 0, 0], [0.01, 0, 0], [0.01, 1e-7, 0], [0, 1e-7, 0]], dtype=float)
        wider = sliver.copy()
        wider[2:, 1] = 2e-7
        np.testing.assert_array_equal(style._revert_flips(sliver, wider, quads, c), wider)


class FacelessHead(unittest.TestCase):
    """The head's cavities and overhangs (eye sockets, mouth, nostrils, the ears' backs) have to end up
    as one smooth sheet on the egg: no layer may pass through another, none may face inward."""

    @staticmethod
    def pocket_sphere():
        """The sphere with a socket wider inside than its mouth, like an eye under its lids: the front
        patch within 3 cm sunk up to 2 cm and spread sideways (a point at fraction r of the radius moves
        out to r (1 + 2 (1 - r)^2)), so its inner walls lie behind the rim as seen from the centre and
        a radial projection lays them over the face."""
        points, quads = cube_sphere(16)
        vertices = EGG.centre + 0.1 * points
        front = int(np.argmax(points @ np.array([0.0, 0.1, 1.0])))
        normal = points[front]
        d = vertices - vertices[front]
        across = d - np.outer(d @ normal, normal)
        r = np.linalg.norm(across, axis=1) / 0.03
        near = r < 1.0
        vertices[near] = (vertices[front] + across[near] * (1.0 + 2.0 * (1.0 - r[near]) ** 2)[:, np.newaxis]
                          - 0.02 * np.outer(1.0 - r[near] ** 2, normal))
        return vertices, quads, near

    @staticmethod
    def undercut_sphere():
        """A socket that is wider inside than its mouth, like an eye under its lids: a patch of the
        front turned half a turn about its centre and sunk 1 cm, which folds the surface over itself."""
        points, quads = cube_sphere(16)
        vertices = EGG.centre + 0.1 * points
        front = int(np.argmax(points @ np.array([0.0, 0.05, 1.0])))
        normal = points[front]
        t1 = np.cross(normal, [0.0, 1.0, 0.0])
        t1 /= np.linalg.norm(t1)
        t2 = np.cross(normal, t1)
        d = vertices - vertices[front]
        near = np.linalg.norm(d, axis=1) < 0.03
        x, y = d[near] @ t1, d[near] @ t2
        vertices[near] = (vertices[front] - np.outer(x, t1) - np.outer(y, t2)
                          - 0.01 * normal * (1 - (np.hypot(x, y) / 0.03) ** 2)[:, np.newaxis])
        return vertices, quads

    def check_faceless(self, vertices, quads, w):
        out = style.make_egg_head(vertices, quads, w, EGG)
        on_egg = w >= 0.999
        np.testing.assert_allclose(style.egg_value(EGG, out[on_egg]), 1.0, atol=1e-9)
        facing, area = style._corners(out, quads[(w[quads] > 0).any(axis=1)], EGG.centre)
        self.assertGreater(facing.min(), 0.0)
        self.assertGreater(area.min(), 0.0)
        self.assertEqual(crossings(out, quads), 0)
        return out

    def test_a_socket_is_laid_flat_on_the_egg_not_tucked_under_it(self):
        vertices, quads, in_pocket = self.pocket_sphere()
        w = neck_weights(vertices)
        self.assertTrue((w[in_pocket] == 1.0).all())
        projected = style.project_to_egg(EGG, vertices)
        self.assertGreater((outward(projected, quads, EGG.centre) <= 0).sum(), 0)      # projection folds it
        self.check_faceless(vertices, quads, w)

    def test_an_undercut_socket_ends_as_one_sheet_on_the_egg(self):
        vertices, quads = self.undercut_sphere()
        w = neck_weights(vertices)
        projected = style.project_to_egg(EGG, vertices)
        self.assertGreater(crossings(projected, quads) + (outward(projected, quads, EGG.centre) <= 0).sum(), 0)
        self.check_faceless(vertices, quads, w)

    def test_head_vertices_lists_every_fully_weighted_vertex(self):
        w = np.array([0.0, 0.5, 0.999, 1.0, 0.9989])
        np.testing.assert_array_equal(style.head_vertices(w), [2, 3])

    def test_a_head_region_with_two_boundary_loops_is_an_error(self):
        # a band round the middle of the sphere has two boundaries: no single neck to lay it out against
        vertices, quads = nosed_sphere()
        w = (np.abs(vertices[:, 1] - EGG.centre[1]) < 0.05).astype(float)
        with self.assertRaisesRegex(RuntimeError, "boundary"):
            style.make_egg_head(vertices, quads, w, EGG)


# --------------------------------------------------------------------------- feet

def tube_foot(x0: float, rings: int = 14, around: int = 16, toes: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A synthetic left foot of quads at x = x0 running along +z: a flat-bottomed tube (`rings` rings
    of `around` vertices, 1 cm apart, the first one open) closed at the front by a 4 x 4 grid. With
    `toes`, the grid's columns 1 and 3 rise 2 cm in their top two rows as two toes with a gap between
    them, so a section above the tube's top cuts them apart. Returns
    the vertices, the quads (counter-clockwise from outside) and toe weights (1 on the grid)."""
    vertices, quads = [], []
    theta = 2.0 * np.pi * (np.arange(around) + 0.5) / around
    for k in range(rings):
        for t in theta:
            vertices.append([x0 + 0.04 * np.cos(t), max(0.0, 0.02 + 0.022 * np.sin(t)), 0.01 * k])
    for k in range(rings - 1):
        for j in range(around):
            a, b = k * around + j, k * around + (j + 1) % around
            quads.append([a, a + around, b + around, b])
    # the front cap: a 5 x 5 vertex grid whose boundary is the last ring (16 vertices)
    last = (rings - 1) * around
    border = [(i, 0) for i in range(4)] + [(4, j) for j in range(4)] + [(i, 4) for i in range(4, 0, -1)] + \
             [(0, j) for j in range(4, 0, -1)]
    # border cells and ring vertices both go once round the tube: pair them in angular order
    border.sort(key=lambda cell: np.arctan2(cell[1] - 2.0, cell[0] - 2.0) % (2.0 * np.pi))
    index = {cell: last + m for m, cell in enumerate(border)}
    front = 0.01 * (rings - 1)
    for i in range(1, 4):
        for j in range(1, 4):
            index[(i, j)] = len(vertices)
            rises = toes and i in (1, 3) and j >= 2
            vertices.append([x0 + 0.02 * (i - 2), 0.005 + 0.01 * j + (0.02 if rises else 0.0), front + 0.015])
    for i in range(4):
        for j in range(4):
            quads.append([index[(i, j)], index[(i, j + 1)], index[(i + 1, j + 1)], index[(i + 1, j)]])
    vertices, quads = np.array(vertices), np.array(quads, dtype=np.int64)
    tube, cap = slice(0, (rings - 1) * around), slice((rings - 1) * around, None)
    # each part was built with one winding: turn it round as a whole if it faces in
    if outward(vertices, quads[tube][:1], np.array([x0, 0.02, 0.01 * quads[tube][0, 0] // around]))[0] < 0:
        quads[tube] = quads[tube][:, ::-1]
    if outward(vertices, quads[cap], np.array([x0, 0.02, front - 0.05])).sum() < 0:
        quads[cap] = quads[cap][:, ::-1]
    toe_w = np.zeros(len(vertices))
    toe_w[last + around:] = 1.0
    return vertices, quads, toe_w


def two_tube_feet(toes: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`tube_foot` at x = 0.1 and its mirror image at x = -0.1 (vertex i of the right foot mirrors vertex i
    of the left)."""
    left, quads, toe_w = tube_foot(0.1, toes=toes)
    right = left * [-1.0, 1.0, 1.0]
    return np.vstack([left, right]), np.vstack([quads, quads[:, ::-1] + len(left)]), np.concatenate([toe_w, toe_w])


class RoundForefeet(unittest.TestCase):
    def setUp(self):
        self.vertices, self.quads, self.toe_w = two_tube_feet()
        self.tris = pose.triangulate(self.vertices, self.quads)

    def pieces(self, vertices, mask, side):
        """Pieces of the forefoot in each 2 mm section from 5 mm up to the forefoot's top."""
        part = mask & (side * vertices[:, 0] > 0)
        top = vertices[part, 1].max()
        return [measure.section_loops(vertices, self.tris, y, part) for y in np.arange(0.005, top, 0.002)]

    def test_the_synthetic_toes_are_separate(self):
        forefoot = self.vertices[:, 2] > 0.06
        self.assertIn(2, self.pieces(self.vertices, forefoot, 1.0))

    def test_the_forefoot_is_one_piece_at_every_height(self):
        out, forefoot = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        mask = np.zeros(len(out), dtype=bool)
        mask[forefoot] = True
        self.assertTrue((mask[self.toe_w >= 0.5]).all())
        for side in (1.0, -1.0):
            pieces = self.pieces(out, mask, side)
            self.assertGreater(len(pieces), 10)
            self.assertEqual(set(pieces), {1})

    def test_no_triangles_pass_through_each_other_and_none_collapse(self):
        out, _ = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        self.assertEqual(crossings(out, self.quads), 0)
        a, b, c = (out[self.tris[:, k]] for k in range(3))
        self.assertGreater((0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)).min(), 1e-9)

    def test_the_sole_stays_flat_on_the_floor(self):
        out, forefoot = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        self.assertEqual(out[:, 1].min(), 0.0)
        self.assertGreater((out[forefoot, 1] == 0.0).sum(), 4)

    def test_only_the_forefoot_moves(self):
        out, forefoot = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        still = np.ones(len(out), dtype=bool)
        still[forefoot] = False
        self.assertEqual(out[still].tobytes(), self.vertices[still].tobytes())
        self.assertGreater(np.abs(out[forefoot] - self.vertices[forefoot]).max(), 0.005)

    def test_the_feet_are_exact_mirror_images(self):
        out, forefoot = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        half = len(out) // 2
        np.testing.assert_array_equal(out[half:], out[:half] * [-1.0, 1.0, 1.0])
        self.assertEqual(len(forefoot) % 2, 0)

    def test_input_is_not_modified_and_output_is_deterministic(self):
        before = self.vertices.copy()
        first, ff1 = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        second, ff2 = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        np.testing.assert_array_equal(self.vertices, before)
        self.assertEqual(first.tobytes(), second.tobytes())
        np.testing.assert_array_equal(ff1, ff2)

    def test_the_forefoot_stays_inside_the_box_round_the_old_one(self):
        # it may not grow forward, up or out to the sides (backward is where it meets the rest of the foot)
        out, forefoot = style.round_forefeet(self.vertices, self.quads, self.toe_w)
        for side in (1.0, -1.0):
            part = forefoot[side * self.vertices[forefoot, 0] > 0]
            lo, hi = self.vertices[part].min(axis=0), self.vertices[part].max(axis=0)
            self.assertTrue(np.all(out[part] <= hi + 1e-3))
            self.assertTrue(np.all(out[part][:, :2] >= lo[:2] - 1e-3))

    def test_a_forefoot_that_runs_out_of_its_box_is_an_error(self):
        # a shell that never stops a ray at the floor: what a ray grazing the floor would do
        with mock.patch.object(style._Shell, "floor_distance", lambda shell, d: np.full(len(d), 10.0)), \
                self.assertRaisesRegex(RuntimeError, "outside the box"):
            style.round_forefeet(self.vertices, self.quads, self.toe_w)

    def test_a_foot_without_an_edge_loop_behind_the_toes_is_an_error(self):
        # weights on the whole tube: no loop is left behind the toes
        with self.assertRaisesRegex(RuntimeError, "edge loop"):
            style.round_forefeet(self.vertices, self.quads, np.ones(len(self.vertices)))


if __name__ == "__main__":
    unittest.main()
