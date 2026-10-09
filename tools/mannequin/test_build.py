"""Offline unit tests for build_mannequins.py's self-check (synthetic meshes only)."""
import unittest

import numpy as np

import build_mannequins
import style

EGG = style.Egg(np.array([0.0, 1.6, 0.0]), np.array([0.08, 0.11, 0.1]))


def box(x0: float, size=(0.1, 0.05, 0.25)) -> tuple[np.ndarray, np.ndarray]:
    """A closed box standing on y = 0 with its low-x corner at x0: 8 vertices, 12 triangles."""
    dx, dy, dz = size
    vertices = np.array([[x0 + ix * dx, iy * dy, iz * dz] for iz in (0, 1) for iy in (0, 1) for ix in (0, 1)])
    faces = [(0, 2, 6, 4), (1, 3, 7, 5), (0, 1, 5, 4), (2, 3, 7, 6), (0, 1, 3, 2), (4, 5, 7, 6)]
    return vertices, np.array([t for a, b, c, d in faces for t in ((a, b, c), (a, c, d))], dtype=np.int64)


def two_feet() -> tuple[np.ndarray, np.ndarray]:
    left, tl = box(0.1)
    right, tr = box(-0.2)
    return np.vstack([left, right]), np.vstack([tl, tr + 8])


def with_triangle(vertices: np.ndarray, tris: np.ndarray, corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The mesh plus one more triangle of its own: three new vertices, so the rest is untouched."""
    n = len(vertices)
    return np.vstack([vertices, corners]), np.vstack([tris, [[n, n + 1, n + 2]]])


class CheckMesh(unittest.TestCase):
    def check(self, vertices, tris, head=(), forefoot=None):
        if forefoot is None:
            forefoot = np.flatnonzero(vertices[:, 1] < 0.1)             # the feet, not the test's head triangles
        build_mannequins.check_mesh("test", vertices, tris, np.array(head, dtype=np.int64), EGG,
                                    np.asarray(forefoot, dtype=np.int64))

    def test_a_good_mesh_passes(self):
        vertices, tris = two_feet()
        self.check(vertices, tris)

    def test_a_triangle_under_the_minimum_area_fails(self):
        vertices, tris = with_triangle(*two_feet(), np.array([[0.5, 1.0, 0.0], [0.6, 1.0, 0.0], [0.6, 1.0, 1e-9]]))
        with self.assertRaisesRegex(RuntimeError, "1 triangles are smaller than"):
            self.check(vertices, tris)

    def test_a_head_triangle_facing_the_egg_centre_fails_and_one_facing_out_passes(self):
        # three vertices on the egg's front, one counter-clockwise seen from in front (outward), one clockwise
        front = EGG.centre + np.array([[0.0, 0.0, 0.1], [0.01, 0.0, 0.1], [0.0, 0.01, 0.1]])
        base, base_tris = two_feet()
        for corners, ok in ((front, True), (front[[0, 2, 1]], False)):
            vertices, tris = with_triangle(base, base_tris, corners)
            head = [len(base), len(base) + 1, len(base) + 2]
            if ok:
                self.check(vertices, tris, head)
            else:
                with self.assertRaisesRegex(RuntimeError, "1 head triangles face into the egg"):
                    self.check(vertices, tris, head)
        self.check(vertices, tris)       # the same inward triangle, not marked as head, is just a triangle

    def test_a_foot_with_two_loops_at_1_cm_fails(self):
        # a second box under the left foot's side of the body: the 1 cm plane cuts it in its own loop
        vertices, tris = two_feet()
        extra, extra_tris = box(0.4)
        vertices, tris = np.vstack([vertices, extra]), np.vstack([tris, extra_tris + 16])
        with self.assertRaisesRegex(RuntimeError, "2 loops under the left foot"):
            self.check(vertices, tris)

    def test_triangles_that_pass_through_each_other_fail(self):
        # a separate triangle stabbing through the left box's top face
        vertices, tris = with_triangle(*two_feet(), np.array([[0.15, 0.03, 0.1], [0.15, 0.08, 0.1], [0.16, 0.08, 0.12]]))
        with self.assertRaisesRegex(RuntimeError, "1 triangle pairs pass through each other"):
            self.check(vertices, tris)

    def test_a_low_separate_toe_fails_at_every_height_it_reaches(self):
        # a toe box from 4 to 8 mm in front of the left foot: the 1 cm section misses it, the
        # forefoot sections at 5 and 7 mm cut it in its own loop
        vertices, tris = two_feet()
        toe, toe_tris = box(0.12, size=(0.02, 0.004, 0.02))
        toe += [0.0, 0.004, 0.26]
        vertices, tris = np.vstack([vertices, toe]), np.vstack([tris, toe_tris + 16])
        with self.assertRaisesRegex(RuntimeError, "forefoot section at y = 0.005 m has 2 pieces on the left"):
            self.check(vertices, tris)
        # outside the forefoot it is only an extra loop under 1 cm, which the 1 cm check cannot see
        self.check(vertices, tris, forefoot=np.arange(16))

    def test_forefoot_sections_go_up_to_the_top_of_the_forefoot_in_2_mm_steps(self):
        heights = build_mannequins.forefoot_heights(np.array([[0.0, 0.0, 0.0], [0.0, 0.0125, 0.0]]))
        np.testing.assert_allclose(heights, [0.005, 0.007, 0.009, 0.011])

    def test_a_foot_that_does_not_reach_1_cm_fails(self):
        left, tl = box(0.1, size=(0.1, 0.005, 0.25))           # a 5 mm slab: the 1 cm plane misses it
        right, tr = box(-0.2)
        with self.assertRaisesRegex(RuntimeError, "0 loops under the left foot"):
            self.check(np.vstack([left, right]), np.vstack([tl, tr + 8]))


class LicenseText(unittest.TestCase):
    def test_states_the_source_terms_and_provenance(self):
        text = build_mannequins.LICENSE_TEXT
        self.assertIn("CC0 1.0 Universal", text)                  # the MakeHuman source assets' licence
        self.assertIn(build_mannequins.makehuman.PINNED_COMMIT, text)
        self.assertIn(f"Builder version: {build_mannequins.BUILDER_VERSION}", text)
        self.assertIn(build_mannequins.VERTEX_ORDER, text)

    def test_does_not_release_the_derived_assets(self):
        # Drape's mannequins are not dedicated to the public domain; whether to release them is Ian's call.
        text = build_mannequins.LICENSE_TEXT.lower()
        self.assertNotIn("released", text)
        self.assertNotIn("same terms", text)


if __name__ == "__main__":
    unittest.main()
