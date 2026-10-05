"""Offline unit tests for style.py: synthetic meshes only, fixed seeds."""
import unittest
from unittest import mock

import numpy as np

import makehuman
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

    def test_taper_halfway_down(self):
        ax, ay, az = EGG.radii
        qy = -ay / 2
        k = 1 - 0.15 * 0.5

        def half_width(axis: int, q_y: float) -> float:
            """Surface half-extent along x (axis 0) or z (axis 2) at height centre_y + q_y."""
            lo, hi = 0.0, 1.0
            for _ in range(100):
                mid = (lo + hi) / 2
                offset = np.zeros(3)
                offset[axis], offset[1] = mid, q_y
                lo, hi = (mid, hi) if style.egg_value(EGG, EGG.centre + offset) < 1 else (lo, mid)
            return lo

        # The horizontal ellipse at height q_y has radii (ax, az) * k * sqrt(1 - (q_y/ay)^2);
        # the factor k * (ax, az) is the "horizontal radius" the taper scales.
        s = np.sqrt(1 - (qy / ay) ** 2)
        self.assertAlmostEqual(half_width(0, qy) / s, ax * k, places=9)
        self.assertAlmostEqual(half_width(2, qy) / s, az * k, places=9)
        # at the same elliptical fraction above the centre the radius is the full ax
        self.assertAlmostEqual(half_width(0, -qy) / s, ax, places=9)
        # and the body is the same egg at the very bottom: taper only, no extra shrink
        self.assertAlmostEqual(float(style.egg_value(EGG, EGG.centre + [ax * 0.85, -ay, 0.0])), 2.0, places=9)

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
        w = np.ones(len(vertices))
        projected = style.project_to_egg(EGG, vertices)
        out = style.make_egg_head(vertices, quads, w, EGG)
        self.assertGreater(np.linalg.norm(out - projected, axis=1).max(), 1e-3)

    def test_zero_weight_vertices_never_move(self):
        vertices, quads = nosed_sphere()
        w = np.where(vertices[:, 1] > EGG.centre[1], 1.0, 0.0)
        w[(vertices[:, 1] <= EGG.centre[1]) & (vertices[:, 1] > EGG.centre[1] - 0.03)] = 0.5
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
        w = np.linspace(0.0, 1.0, len(vertices))
        first = style.make_egg_head(vertices, quads, w, EGG)
        second = style.make_egg_head(vertices, quads, w, EGG)
        np.testing.assert_array_equal(vertices, before)
        self.assertEqual(first.tobytes(), second.tobytes())

    @staticmethod
    def rough_sphere(jitter: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
        """Noise on the sphere radially and sideways: projecting it onto the egg folds many quads
        and relaxing it wants to turn many more over."""
        rng = np.random.default_rng(seed)
        points, quads = cube_sphere(8)
        vertices = EGG.centre + 0.1 * points * (1 + jitter * rng.uniform(-1, 1, (len(points), 1)))
        vertices += rng.normal(0.0, 0.03 * jitter, vertices.shape)
        return vertices, quads

    @staticmethod
    def after_the_blend(vertices: np.ndarray, quads: np.ndarray, w: np.ndarray) -> np.ndarray:
        """make_egg_head with no relaxation rounds: just the blend onto the egg."""
        with mock.patch.object(style, "_HEAD_ROUNDS", 0):
            return style.make_egg_head(vertices, quads, w, EGG)

    def test_quads_that_face_outward_never_turn_inward(self):
        vertices, quads = self.rough_sphere(jitter=0.6, seed=0)
        w = np.clip((vertices[:, 1] - EGG.centre[1] + 0.05) / 0.1, 0.0, 1.0)     # a soft lower edge, w = 0 below
        faced_out = outward(self.after_the_blend(vertices, quads, w), quads, EGG.centre) > 0

        def turned_in(result: np.ndarray) -> int:
            return int((faced_out & (outward(result, quads, EGG.centre) <= 0)).sum())

        self.assertEqual(turned_in(style.make_egg_head(vertices, quads, w, EGG)), 0)
        with mock.patch.object(style, "_revert_flips", lambda before, after, quads, centre: after):
            self.assertGreater(turned_in(style.make_egg_head(vertices, quads, w, EGG)), 5)   # the guard is what does it

    def test_quads_folded_by_the_blend_can_unfold(self):
        vertices, quads = self.rough_sphere(jitter=0.3, seed=1)
        w = np.ones(len(vertices))
        folded = outward(self.after_the_blend(vertices, quads, w), quads, EGG.centre) <= 0
        self.assertGreater(folded.sum(), 20)
        out = style.make_egg_head(vertices, quads, w, EGG)
        still = folded & (outward(out, quads, EGG.centre) <= 0)
        self.assertLess(still.sum() * 4, folded.sum())

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
        # The same square seen from the other side of it: it faces toward c to begin with, and the
        # move that turns it over (what the old normal-against-normal rule forbade) is the cure.
        c = np.array([0.0, 0.0, 1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)    # faces +z, toward c
        after = before.copy()
        after[2] = [-0.5, -0.5, 0.0]                  # faces -z, away from c
        quads = np.array([[0, 1, 2, 3]])
        self.assertLess(outward(before, quads, c)[0], 0)
        self.assertGreater(outward(after, quads, c)[0], 0)
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), after)

    def test_flip_guard_ignores_quads_that_were_already_inward(self):
        # An inward quad that stays inward is nobody's regression: moving it is allowed.
        c = np.array([0.0, 0.0, 1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], dtype=float)    # faces +z, toward c
        after = before + np.array([[0.1, 0, 0], [0.1, 0.05, 0], [0.1, 0.1, 0], [0.1, 0.0, -0.2]])
        quads = np.array([[0, 1, 2, 3]])
        self.assertLess(outward(after, quads, c)[0], 0)
        np.testing.assert_array_equal(style._revert_flips(before, after, quads, c), after)

    def test_flip_guard_cascades_to_neighbouring_quads(self):
        # Two quads sharing the edge 1-2. Only the right quad turns inward in the round, but
        # reverting its vertices (1, 2, 4, 5) leaves the left quad with a moved vertex 3 beside
        # an unmoved vertex 1, and that mix turns it inward. The guard must keep going.
        c = np.array([1.0, 0.5, -1.0])
        before = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [2, 0, 0], [2, 1, 0]], dtype=float)
        quads = np.array([[0, 1, 2, 3], [1, 4, 5, 2]])
        after = before.copy()
        after[1] = [3.0, 0.0, 0.0]
        after[3] = [2.0, 0.5, 0.0]
        after[5] = [0.0, -1.0, 0.0]
        faced_out = outward(before, quads, c) > 0
        turned_in = faced_out & (outward(after, quads, c) <= 0)
        self.assertEqual(turned_in.tolist(), [False, True])             # the setup is what we think it is
        out = style._revert_flips(before, after, quads, c)
        self.assertTrue(np.all(outward(out, quads, c) > 0))
        np.testing.assert_array_equal(out, before)


class CavityTuck(unittest.TestCase):
    """A pocket in the sphere, like an eye socket: the quads around four vertices are turned
    inside out and those vertices pushed 2 cm in, so their vertex normals face the centre."""
    DEPTH = 0.002

    def pocket_sphere(self):
        points, quads = cube_sphere(16)
        vertices = EGG.centre + 0.1 * points
        aim = np.array([0.2, 0.1, 1.0]) / np.linalg.norm([0.2, 0.1, 1.0])
        centroids = points[quads].mean(axis=1)
        pocket = np.unique(quads[np.argmax(centroids @ aim)])               # the four corners of one quad
        in_pocket = np.zeros(len(points), dtype=bool)
        in_pocket[pocket] = True
        touching = in_pocket[quads].any(axis=1)                              # 9 quads
        quads = quads.copy()
        quads[touching] = quads[touching][:, ::-1]
        vertices[pocket] -= 0.02 * points[pocket]
        return vertices, quads, in_pocket, touching

    def test_cavity_vertices_flags_exactly_the_pocket(self):
        vertices, quads, in_pocket, _ = self.pocket_sphere()
        self.assertEqual(in_pocket.sum(), 4)
        flagged = style.cavity_vertices(vertices, quads, np.ones(len(vertices)), EGG)
        self.assertEqual(flagged.dtype, bool)
        np.testing.assert_array_equal(flagged, in_pocket)

    def test_only_fully_weighted_vertices_can_be_cavity(self):
        vertices, quads, in_pocket, _ = self.pocket_sphere()
        w = np.ones(len(vertices))
        w[np.flatnonzero(in_pocket)[0]] = 0.998
        flagged = style.cavity_vertices(vertices, quads, w, EGG)
        self.assertEqual(flagged.sum(), 3)
        self.assertFalse(flagged[np.flatnonzero(in_pocket)[0]])

    def test_a_clean_sphere_has_no_cavity(self):
        points, quads = cube_sphere(16)
        vertices = EGG.centre + 0.1 * points
        self.assertFalse(style.cavity_vertices(vertices, quads, np.ones(len(vertices)), EGG).any())
        vertices, quads = nosed_sphere()
        self.assertFalse(style.cavity_vertices(vertices, quads, np.ones(len(vertices)), EGG).any())

    def test_pocket_ends_two_millimetres_inside_and_everything_else_on_the_egg(self):
        vertices, quads, in_pocket, _ = self.pocket_sphere()
        out = style.make_egg_head(vertices, quads, np.ones(len(vertices)), EGG)
        surface = style.project_to_egg(EGG, out)
        depth = np.linalg.norm(surface - out, axis=1)
        np.testing.assert_allclose(depth[in_pocket], self.DEPTH, atol=1e-9)
        self.assertTrue(np.all(style.egg_value(EGG, out[in_pocket]) < 1.0))
        np.testing.assert_allclose(depth[~in_pocket], 0.0, atol=1e-9)

    def test_no_outward_facing_surface_quad_turns_inward(self):
        vertices, quads, in_pocket, touching = self.pocket_sphere()
        out = style.make_egg_head(vertices, quads, np.ones(len(vertices)), EGG)
        surface_quads = ~touching
        self.assertGreater(outward(out, quads, EGG.centre)[surface_quads].min(), 0.0)

    def test_the_tuck_is_part_of_the_blend_for_partial_weights(self):
        # w < 0.999 vertices are never cavity and never tucked, whatever their normals say.
        vertices, quads, in_pocket, _ = self.pocket_sphere()
        w = np.where(in_pocket, 0.5, 1.0)
        out = style.make_egg_head(vertices, quads, w, EGG)
        depth = np.linalg.norm(style.project_to_egg(EGG, out) - out, axis=1)
        np.testing.assert_allclose(depth[w >= 0.999], 0.0, atol=1e-9)


class SmoothToes(unittest.TestCase):
    @staticmethod
    def bumpy_grid():
        rng = np.random.default_rng(11)
        n = 12
        gx, gz = np.meshgrid(np.linspace(0, 0.1, n), np.linspace(0, 0.1, n), indexing="ij")
        v = np.stack([gx + rng.uniform(-0.003, 0.003, gx.shape),
                      rng.uniform(0.0, 0.02, gx.shape),
                      gz + rng.uniform(-0.003, 0.003, gx.shape)], axis=-1).reshape(-1, 3)
        v[::7, 1] = 0.0                                    # the sole: several vertices at the lowest y
        quads = [[i * n + j, (i + 1) * n + j, (i + 1) * n + j + 1, i * n + j + 1]
                 for i in range(n - 1) for j in range(n - 1)]
        return v, np.array(quads, dtype=np.int64)

    def test_toe_smoothing_keeps_the_sole(self):
        v, quads = self.bumpy_grid()
        sole = v[:, 1] <= v[:, 1].min() + 0.003
        self.assertGreater(sole.sum(), 10)
        self.assertLess(sole.sum(), len(v))
        out = style.smooth_toes(v, quads, np.ones(len(v)))
        self.assertEqual(out[sole, 1].tobytes(), v[sole, 1].tobytes())
        self.assertGreater(np.abs(out[sole][:, [0, 2]] - v[sole][:, [0, 2]]).max(), 1e-4)   # sole slides
        self.assertLess(np.ptp(out[~sole, 1]), 0.5 * np.ptp(v[~sole, 1]))                   # bumps relax

    def test_zero_weight_vertices_never_move(self):
        v, quads = self.bumpy_grid()
        w = (v[:, 0] > 0.05).astype(float)
        out = style.smooth_toes(v, quads, w)
        self.assertEqual(out[w == 0].tobytes(), v[w == 0].tobytes())
        self.assertGreater(np.linalg.norm(out[w == 1] - v[w == 1], axis=1).max(), 1e-4)

    def test_sole_is_measured_where_the_toe_weight_is(self):
        # Vertices outside the region do not set the sole height: a low heel (w = 0) must not
        # turn the toes' own lowest vertices into free movers.
        v, quads = self.bumpy_grid()
        v[0, 1] = -0.05
        w = np.ones(len(v))
        w[0] = 0.0
        sole = (v[:, 1] <= v[1:, 1].min() + 0.003) & (w > 0)
        out = style.smooth_toes(v, quads, w)
        self.assertEqual(out[sole, 1].tobytes(), v[sole, 1].tobytes())

    def test_rounds_and_determinism(self):
        v, quads = self.bumpy_grid()
        w = np.ones(len(v))
        before = v.copy()
        once = style.smooth_toes(v, quads, w, rounds=1)
        forty = style.smooth_toes(v, quads, w)
        np.testing.assert_array_equal(v, before)
        np.testing.assert_array_equal(forty, style.smooth_toes(v, quads, w, rounds=40))
        self.assertEqual(forty.tobytes(), style.smooth_toes(v, quads, w).tobytes())
        self.assertGreater(np.abs(forty - once).max(), 1e-4)
        np.testing.assert_array_equal(style.smooth_toes(v, quads, w, rounds=0), v)


if __name__ == "__main__":
    unittest.main()
