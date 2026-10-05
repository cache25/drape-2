"""Mannequin posing, coordinate frame and triangulation.

`pose` swings the arms and legs of MakeHuman's rest body to the mannequin's stance:
rotations about +z through the shoulder or hip, so the frontal (x-y) angle of each limb hits a
target while MakeHuman's forward swing of the arms (the z coordinate) is kept. The feet are
counter-rotated about the ankle so the soles stay level. `to_drape_frame` moves the result into
Drape's frame and `triangulate` turns the body's quads into triangles.

Vertices are float64 (N, 3) in metres, Y up, left is +x. Quads are int arrays (M, 4),
counter-clockwise seen from outside. Nothing here is random, so equal inputs give
byte-identical outputs. Python 3.13 and numpy only.
"""
import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from makehuman import Rig, descendants, joint_pos

Transform = Callable[[np.ndarray], np.ndarray]     # (K, 3) points -> (K, 3) points


@dataclass
class Posed:
    vertices: np.ndarray                  # (N, 3) skinned vertices, same indexing as the input
    joints: dict[str, np.ndarray]         # posed (3,) positions of the limb joints, keyed by joint name


def frontal_angle(d: np.ndarray) -> float:
    """Angle in degrees between `d` and straight down, in the frontal (x-y) projection:
    atan2(|d_x|, -d_y). 0 is hanging down, 90 is held out sideways; z is ignored."""
    return math.degrees(math.atan2(abs(d[0]), -d[1]))


def _head(bone: str) -> str:
    return f"{bone}____head"


def _rotation_about_z(theta: float, pivot: np.ndarray) -> Transform:
    """Rotation by `theta` radians about the +z axis through `pivot`; z is untouched."""
    c, s = math.cos(theta), math.sin(theta)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return lambda points: pivot + (points - pivot) @ rotation.T


def _then(first: Transform, second: Transform) -> Transform:
    """The transform that applies `first`, then `second`."""
    return lambda points: second(first(points))


def _swing(vertices: np.ndarray, rig: Rig, root: str, tip: str, target_deg: float,
           mirror: float) -> float:
    """Radians about +z that bring the frontal angle of the limb root->tip to `target_deg`.
    +x (left, `mirror` = 1) opens counter-clockwise; the right side runs the other way."""
    reach = joint_pos(vertices, rig, _head(tip)) - joint_pos(vertices, rig, _head(root))
    return mirror * math.radians(target_deg - frontal_angle(reach))


def pose(vertices: np.ndarray, rig: Rig, arm_deg: float = 45.0, leg_deg: float = 5.0) -> Posed:
    """Pose the arms at `arm_deg` and the legs at `leg_deg` from vertical (frontal projection).

    `vertices` is the whole base-mesh array, helper vertices included: joints are the means of
    helper vertices and the rig's weights index into it. Per side, every bone under the upper
    arm gets the shoulder rotation, every bone under the upper leg gets the hip rotation, and
    every bone under the foot additionally gets the opposite rotation about the posed ankle, which
    keeps the sole level. Skinning is v' = v + sum_b w_b (T_b v - v), so bones that are not posed
    contribute nothing and vertices they alone hold are returned bit for bit.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    posed = vertices.copy()
    joints: dict[str, np.ndarray] = {}
    transforms: dict[str, Transform] = {}

    for side, mirror in (("L", 1.0), ("R", -1.0)):
        shoulder, wrist = f"upperarm01.{side}", f"wrist.{side}"
        hip, foot = f"upperleg01.{side}", f"foot.{side}"
        shoulder_pos = joint_pos(vertices, rig, _head(shoulder))
        hip_pos = joint_pos(vertices, rig, _head(hip))
        ankle_pos = joint_pos(vertices, rig, _head(foot))

        arm_swing = _rotation_about_z(_swing(vertices, rig, shoulder, wrist, arm_deg, mirror), shoulder_pos)
        leg_theta = _swing(vertices, rig, hip, foot, leg_deg, mirror)
        leg_swing = _rotation_about_z(leg_theta, hip_pos)
        ankle_posed = leg_swing(ankle_pos[None])[0]
        foot_swing = _then(leg_swing, _rotation_about_z(-leg_theta, ankle_posed))

        for bone in descendants(rig, shoulder):
            transforms[bone] = arm_swing
        for bone in descendants(rig, hip):
            transforms[bone] = leg_swing
        for bone in descendants(rig, foot):         # after the leg, so the foot's transform wins
            transforms[bone] = foot_swing

        joints[_head(shoulder)] = shoulder_pos      # pivots: unchanged
        joints[_head(hip)] = hip_pos
        joints[_head(wrist)] = arm_swing(joint_pos(vertices, rig, _head(wrist))[None])[0]
        joints[_head(foot)] = foot_swing(ankle_pos[None])[0]

    for bone, transform in transforms.items():
        if bone not in rig.weights:
            continue
        indices, weights = rig.weights[bone]
        at_rest = vertices[indices]
        np.add.at(posed, indices, weights[:, None] * (transform(at_rest) - at_rest))
    return Posed(vertices=posed, joints=joints)


def to_drape_frame(vertices: np.ndarray, body_ids: np.ndarray, ankle_l: np.ndarray,
                   ankle_r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Move vertices into Drape's frame: the lowest body vertex at y = 0 and z = 0 midway
    between the ankles (x already is symmetric about 0 in MakeHuman, so it is not shifted).

    Returns (shifted copy of `vertices`, offset (3,)), where shifted = vertices - offset.
    Apply `joint - offset` to joints. Only `body_ids` set the floor: helper vertices (eyes,
    joint markers) may lie below it.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    offset = np.array([0.0, vertices[body_ids, 1].min(), (ankle_l[2] + ankle_r[2]) / 2.0])
    return vertices - offset, offset


def triangulate(vertices: np.ndarray, quads: np.ndarray) -> np.ndarray:
    """(2M, 3) triangle indices from (M, 4) quads (a, b, c, d), two rows per quad in quad order.

    Each quad is split along its shorter diagonal, which gives the better-shaped triangles:
    (a, b, c), (a, c, d) along a-c, or (a, b, d), (b, c, d) along b-d; a tie takes a-c. Both
    splits keep the quad's winding.
    """
    quads = np.asarray(quads).reshape(-1, 4)
    a, b, c, d = quads.T
    along_ac = (((vertices[a] - vertices[c]) ** 2).sum(axis=1)
                <= ((vertices[b] - vertices[d]) ** 2).sum(axis=1))[:, None]
    triangles = np.empty((len(quads), 2, 3), dtype=quads.dtype)
    triangles[:, 0] = np.where(along_ac, np.stack([a, b, c], axis=1), np.stack([a, b, d], axis=1))
    triangles[:, 1] = np.where(along_ac, np.stack([a, c, d], axis=1), np.stack([b, c, d], axis=1))
    return triangles.reshape(-1, 3)
