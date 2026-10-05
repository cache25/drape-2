"""Offline unit tests for pose.py: a synthetic body of vertex strips, never the network.

Each limb is a strip of vertices fully weighted to one bone, so the posed strip shows exactly
which transform that bone received. Left is +x; the right side mirrors it.
"""
import math
import unittest

import numpy as np

import makehuman
import pose

ARM_REST_DEG = 40.0       # frontal angle of the synthetic arms before posing
LEG_REST_DEG = 6.0        # ... and legs
SHOULDER = np.array([0.20, 1.40, 0.05])
HIP = np.array([0.10, 0.90, 0.00])
ARM_LENGTH = 0.60
LEG_LENGTH = 0.85
ARM_FORWARD_Z = 0.10      # MakeHuman's arms swing forward; rotation about z must keep it
SIDES = (("L", 1.0), ("R", -1.0))


def swap_sides(name: str) -> str:
    """'foot.L____head' <-> 'foot.R____head'; names without a side are returned as they are."""
    return name.replace(".L", ".\0").replace(".R", ".L").replace(".\0", ".R")


def mirrored(rig: makehuman.Rig) -> makehuman.Rig:
    """The same rig seen in a mirror: every bone and joint name changes sides."""
    return makehuman.Rig(
        parent={swap_sides(b): None if p is None else swap_sides(p) for b, p in rig.parent.items()},
        head_joint={swap_sides(b): swap_sides(j) for b, j in rig.head_joint.items()},
        joints={swap_sides(j): list(ids) for j, ids in rig.joints.items()},
        weights={swap_sides(b): (idx.copy(), w.copy()) for b, (idx, w) in rig.weights.items()})


class Body:
    """Vertices plus a rig. `strips[name]` is the vertex index range of each strip; `rig.joints`
    puts one helper vertex at each joint. Bones: root > upperarm01 > wrist and
    root > upperleg01 > lowerleg01 > foot, per side (foot is a descendant of the upper leg, as
    in MakeHuman)."""

    def __init__(self, arm_deg: float = ARM_REST_DEG, leg_deg: float = LEG_REST_DEG):
        points: list[np.ndarray] = []
        self.strips: dict[str, np.ndarray] = {}
        joints: dict[str, list[int]] = {}
        parent: dict[str, str | None] = {"root": None}
        head_joint: dict[str, str] = {}
        weights: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        def add_strip(name: str, bone: str, strip: np.ndarray) -> None:
            first = len(points)
            points.extend(strip)
            self.strips[name] = np.arange(first, len(points))
            weights[bone] = (self.strips[name], np.ones(len(strip)))

        def add_joint(name: str, position: np.ndarray) -> None:
            joints[name] = [len(points)]
            points.append(position)

        t = np.linspace(0.0, 1.0, 5)[:, None]
        add_strip("torso", "root", np.array([0.0, 1.2, 0.0]) + t * np.array([0.0, 0.3, 0.1]))
        for side, sx in SIDES:
            shoulder = SHOULDER * [sx, 1, 1]
            arm_dir = np.array([sx * math.sin(math.radians(arm_deg)), -math.cos(math.radians(arm_deg)),
                                ARM_FORWARD_Z])
            hip = HIP * [sx, 1, 1]
            leg_dir = np.array([sx * math.sin(math.radians(leg_deg)), -math.cos(math.radians(leg_deg)), 0.0])
            ankle = hip + LEG_LENGTH * leg_dir
            knee = hip + 0.5 * LEG_LENGTH * leg_dir
            wrist = shoulder + ARM_LENGTH * arm_dir
            add_strip("arm." + side, "upperarm01." + side, shoulder + t * ARM_LENGTH * arm_dir)
            add_strip("leg." + side, "upperleg01." + side, hip + t * 0.5 * LEG_LENGTH * leg_dir)
            add_strip("shin." + side, "lowerleg01." + side, knee + t * 0.5 * LEG_LENGTH * leg_dir)
            add_strip("hand." + side, "wrist." + side, wrist + t * 0.2 * arm_dir)
            # A horizontal sole strip across the foot, heel to toe, at ankle height. It must have
            # an x extent: a strip along z would not show a rotation about z.
            add_strip("sole." + side, "foot." + side, ankle + t * np.array([sx * 0.1, 0.0, 0.2]))
            add_joint(f"upperarm01.{side}____head", shoulder)
            add_joint(f"wrist.{side}____head", wrist)
            add_joint(f"upperleg01.{side}____head", hip)
            add_joint(f"foot.{side}____head", ankle)
            for bone, up in (("upperarm01", "root"), ("upperleg01", "root"),
                             ("lowerleg01", "upperleg01"), ("foot", "lowerleg01")):
                parent[f"{bone}.{side}"] = up if up == "root" else f"{up}.{side}"
                head_joint[f"{bone}.{side}"] = f"{bone}.{side}____head"
        parent["wrist.L"], parent["wrist.R"] = "upperarm01.L", "upperarm01.R"
        head_joint["wrist.L"], head_joint["wrist.R"] = "wrist.L____head", "wrist.R____head"

        # One vertex that no bone mentions, to show it stays where it is.
        self.orphan = len(points)
        points.append(np.array([0.3, 0.3, 0.3]))
        self.vertices = np.array(points)
        self.rig = makehuman.Rig(parent=parent, head_joint=head_joint, joints=joints, weights=weights)

    def joint(self, name: str) -> np.ndarray:
        return makehuman.joint_pos(self.vertices, self.rig, name)


def frontal(posed: pose.Posed, tip: str, root: str) -> float:
    return pose.frontal_angle(posed.joints[tip + "____head"] - posed.joints[root + "____head"])


class FrontalAngle(unittest.TestCase):
    def test_measures_from_straight_down_in_the_x_y_plane(self):
        self.assertAlmostEqual(pose.frontal_angle(np.array([0.0, -1.0, 0.0])), 0.0, places=12)
        self.assertAlmostEqual(pose.frontal_angle(np.array([1.0, -1.0, 0.0])), 45.0, places=12)
        self.assertAlmostEqual(pose.frontal_angle(np.array([2.0, 0.0, 0.0])), 90.0, places=12)

    def test_ignores_sign_of_x_and_depth(self):
        self.assertAlmostEqual(pose.frontal_angle(np.array([-1.0, -1.0, 7.0])), 45.0, places=12)


class Pose(unittest.TestCase):
    def test_arms_reach_45_degrees(self):
        body = Body()
        for side, _ in SIDES:           # start away from the target so the test can fail
            reach = body.joint(f"wrist.{side}____head") - body.joint(f"upperarm01.{side}____head")
            self.assertAlmostEqual(pose.frontal_angle(reach), ARM_REST_DEG, places=9)
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            self.assertAlmostEqual(frontal(posed, f"wrist.{side}", f"upperarm01.{side}"), 45.0, delta=1e-9)

    def test_legs_reach_5_degrees(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            self.assertAlmostEqual(frontal(posed, f"foot.{side}", f"upperleg01.{side}"), 5.0, delta=1e-9)

    def test_targets_are_arguments(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig, arm_deg=30.0, leg_deg=10.0)
        for side, _ in SIDES:
            self.assertAlmostEqual(frontal(posed, f"wrist.{side}", f"upperarm01.{side}"), 30.0, delta=1e-9)
            self.assertAlmostEqual(frontal(posed, f"foot.{side}", f"upperleg01.{side}"), 10.0, delta=1e-9)

    def test_skinned_strips_follow_their_joints(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            arm = posed.vertices[body.strips["arm." + side]]
            self.assertAlmostEqual(pose.frontal_angle(arm[-1] - arm[0]), 45.0, delta=1e-9)
            self.assertTrue(np.allclose(arm[0], posed.joints[f"upperarm01.{side}____head"], atol=1e-12))
            self.assertTrue(np.allclose(arm[-1], posed.joints[f"wrist.{side}____head"], atol=1e-12))
            hand = posed.vertices[body.strips["hand." + side]]        # wrist is a child of the upper arm
            self.assertTrue(np.allclose(hand[0], posed.joints[f"wrist.{side}____head"], atol=1e-12))
            self.assertAlmostEqual(pose.frontal_angle(hand[-1] - hand[0]), 45.0, delta=1e-9)
            leg = posed.vertices[body.strips["leg." + side]]
            self.assertAlmostEqual(pose.frontal_angle(leg[-1] - leg[0]), 5.0, delta=1e-9)

    def test_pivots_stay_and_rotation_keeps_z(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            for joint in (f"upperarm01.{side}____head", f"upperleg01.{side}____head"):
                np.testing.assert_array_equal(posed.joints[joint], body.joint(joint))
            wrist = body.joint(f"wrist.{side}____head")
            self.assertAlmostEqual(posed.joints[f"wrist.{side}____head"][2], wrist[2], places=12)
            arm = body.vertices[body.strips["arm." + side]]
            arm_after = posed.vertices[body.strips["arm." + side]]
            np.testing.assert_allclose(arm_after[:, 2], arm[:, 2], atol=1e-12)

    def test_whole_leg_chain_moves_with_the_hip(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            shin = posed.vertices[body.strips["shin." + side]]       # lowerleg01 is below upperleg01
            self.assertAlmostEqual(pose.frontal_angle(shin[-1] - shin[0]), 5.0, delta=1e-9)
            self.assertFalse(np.allclose(shin, body.vertices[body.strips["shin." + side]], atol=1e-6))

    def test_soles_stay_level(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        for side, _ in SIDES:
            sole = posed.vertices[body.strips["sole." + side]]
            self.assertLess(np.ptp(sole[:, 1]), 1e-12)
            # The counter-rotation pivots on the ankle, so the sole just rides along with it.
            ankle_shift = posed.joints[f"foot.{side}____head"] - body.joint(f"foot.{side}____head")
            np.testing.assert_allclose(sole - body.vertices[body.strips["sole." + side]],
                                       np.tile(ankle_shift, (len(sole), 1)), atol=1e-12)
            self.assertFalse(np.allclose(ankle_shift, 0.0, atol=1e-6))    # the legs did move

    def test_unweighted_vertices_stay(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        torso = body.strips["torso"]
        self.assertTrue(np.array_equal(posed.vertices[torso], body.vertices[torso]))
        self.assertTrue(np.array_equal(posed.vertices[body.orphan], body.vertices[body.orphan]))

    def test_partial_weights_blend_linearly(self):
        body = Body()
        arm = body.strips["arm.L"]
        full = pose.pose(body.vertices, body.rig).vertices[arm]
        body.rig.weights["upperarm01.L"] = (arm, np.full(len(arm), 0.25))
        body.rig.weights["root"] = (arm, np.full(len(arm), 0.75))
        blended = pose.pose(body.vertices, body.rig).vertices[arm]
        rest = body.vertices[arm]
        np.testing.assert_allclose(blended - rest, 0.25 * (full - rest), atol=1e-12)

    def test_input_is_not_modified(self):
        body = Body()
        before = body.vertices.copy()
        posed = pose.pose(body.vertices, body.rig)
        self.assertTrue(np.array_equal(body.vertices, before))
        self.assertIsNot(posed.vertices, body.vertices)

    def test_posed_holds_exactly_the_eight_joints(self):
        body = Body()
        posed = pose.pose(body.vertices, body.rig)
        self.assertEqual(set(posed.joints),
                         {f"{bone}.{side}____head" for bone in ("upperarm01", "wrist", "upperleg01", "foot")
                          for side in "LR"})

    def test_deterministic(self):
        body = Body()
        a, b = pose.pose(body.vertices, body.rig), pose.pose(body.vertices, body.rig)
        self.assertTrue(np.array_equal(a.vertices, b.vertices))

    def test_mirror_symmetry(self):
        body = Body(arm_deg=38.0, leg_deg=7.5)
        flip = np.array([-1.0, 1.0, 1.0])
        posed = pose.pose(body.vertices, body.rig)
        in_mirror = pose.pose(body.vertices * flip, mirrored(body.rig))
        np.testing.assert_allclose(in_mirror.vertices, posed.vertices * flip, atol=1e-12)
        for name, position in posed.joints.items():
            np.testing.assert_allclose(in_mirror.joints[swap_sides(name)], position * flip, atol=1e-12)
        # And the body itself is left-right symmetric, so its posed form must be too.
        for strip in ("arm", "hand", "leg", "shin", "sole"):
            np.testing.assert_allclose(posed.vertices[body.strips[strip + ".R"]],
                                       posed.vertices[body.strips[strip + ".L"]] * flip, atol=1e-12)


class DrapeFrame(unittest.TestCase):
    VERTICES = np.array([[0.1, 0.30, 0.20],      # body: the lowest body vertex
                         [-0.2, 1.50, 0.40],     # body
                         [0.0, 0.10, 5.00]])     # not in body_ids (a helper): must not set the floor
    BODY = np.array([0, 1])

    def test_frame_puts_floor_at_zero(self):
        ankle_l, ankle_r = np.array([0.1, 0.1, 0.30]), np.array([-0.1, 0.1, 0.10])
        framed, offset = pose.to_drape_frame(self.VERTICES, self.BODY, ankle_l, ankle_r)
        self.assertEqual(framed[self.BODY, 1].min(), 0.0)
        self.assertAlmostEqual(offset[2], 0.20, places=12)                    # mean ankle z
        np.testing.assert_allclose(offset, [0.0, 0.30, 0.20], atol=1e-12)     # x is not shifted
        np.testing.assert_allclose(framed, self.VERTICES - offset, atol=1e-12)
        self.assertAlmostEqual(framed[2, 1], -0.20, places=12)                # helpers may go below

    def test_offset_applies_to_joints(self):
        ankle_l, ankle_r = np.array([0.1, 0.1, 0.30]), np.array([-0.1, 0.1, 0.10])
        _, offset = pose.to_drape_frame(self.VERTICES, self.BODY, ankle_l, ankle_r)
        self.assertEqual(offset.shape, (3,))
        np.testing.assert_allclose(ankle_l - offset, [0.1, -0.2, 0.10], atol=1e-12)
        np.testing.assert_allclose(ankle_r - offset, [-0.1, -0.2, -0.10], atol=1e-12)
        self.assertAlmostEqual(((ankle_l - offset)[2] + (ankle_r - offset)[2]) / 2, 0.0, places=12)

    def test_input_is_not_modified(self):
        before = self.VERTICES.copy()
        pose.to_drape_frame(self.VERTICES, self.BODY, np.zeros(3), np.zeros(3))
        self.assertTrue(np.array_equal(self.VERTICES, before))


class Triangulate(unittest.TestCase):
    @staticmethod
    def normals(vertices: np.ndarray, tris: np.ndarray) -> np.ndarray:
        n = np.cross(vertices[tris[:, 1]] - vertices[tris[:, 0]], vertices[tris[:, 2]] - vertices[tris[:, 0]])
        return n / np.linalg.norm(n, axis=1, keepdims=True)

    def test_triangulate_short_diagonal(self):
        # a-c is sqrt(1.1^2 + 1) = 1.487, b-d is sqrt(2) = 1.414, so the quad splits along b-d.
        v = np.array([[0, 0, 0], [1, 0, 0], [1.1, 1, 0], [0, 1, 0]], dtype=float)
        tris = pose.triangulate(v, np.array([[0, 1, 2, 3]]))
        self.assertEqual(tris.shape, (2, 3))
        np.testing.assert_array_equal(tris, [[0, 1, 3], [1, 2, 3]])
        np.testing.assert_allclose(self.normals(v, tris), [[0, 0, 1], [0, 0, 1]], atol=1e-12)

    def test_short_a_c_diagonal_and_ties_split_along_a_c(self):
        v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],           # a square: equal diagonals
                      [0, 0, 1], [1.2, 0, 1], [0.9, 1, 1], [0, 1, 1]], dtype=float)
        quads = np.array([[0, 1, 2, 3], [4, 5, 6, 7]])
        tris = pose.triangulate(v, quads)
        np.testing.assert_array_equal(tris, [[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]])
        np.testing.assert_allclose(self.normals(v, tris), [[0, 0, 1]] * 4, atol=1e-12)

    def test_empty_input(self):
        tris = pose.triangulate(np.zeros((0, 3)), np.zeros((0, 4), dtype=np.int64))
        self.assertEqual(tris.shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
