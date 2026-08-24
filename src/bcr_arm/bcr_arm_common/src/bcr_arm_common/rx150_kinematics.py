#!/usr/bin/env python3
"""Shared RX-150 kinematics and whole-body collision model.

This module is the single source of truth for the RX-150's forward-kinematics
chain. Used across ``rx150_dls_ik_executor.py``, ``rx150_point_cloud_path_planner.py``
and ``rx150_path_waypoint_executor.py`` allows synchronization across them. If not synchronized
the planner and the solver would silently disagree about where the arm
actually is.

On top of the FK chain this module adds a simplified whole-arm collision model:
each physical link is approximated as a capsule (the line segment between two
consecutive joint origins, plus a radius). ``link_positions`` returns every joint
position (not just the end effector), and ``check_arm_collision`` tests those
capsules against a set of obstacle points. This is what lets the planner reason
about the arm's *body* instead of only the tracked end-effector point.

Pure numpy, no ROS dependencies, so it can be unit-tested in isolation.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np


JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']

# Joint origins (translation applied before each joint's rotation) and rotation
# axes, taken from interbotix_xsarm_descriptions/urdf/rx150.urdf.xacro. These
# values match the real robot's URDF joint offsets exactly.
JOINT_ORIGINS: List[np.ndarray] = [
    np.array([0.0, 0.0, 0.06566], dtype=float),
    np.array([0.0, 0.0, 0.03891], dtype=float),
    np.array([0.05, 0.0, 0.15], dtype=float),
    np.array([0.15, 0.0, 0.0], dtype=float),
    np.array([0.065, 0.0, 0.0], dtype=float),
]
JOINT_AXES: List[np.ndarray] = [
    np.array([0.0, 0.0, 1.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([1.0, 0.0, 0.0], dtype=float),
]
# Default tool offset: wrist_rotate origin to the point the arm actually aims.
# Matches the ``tool_offset_*`` parameter defaults in rx150_dls_ik_executor.py.
#
# Aiming at a point puts the grasped object's AXIS there, so this value trades
# pad contact against clearance for the parts of the hand behind the pads.
# Geometry from the URDF collision meshes, in the gripper_link frame (the same
# description the real arm loads, so none of this is sim-specific):
#
#     palm / finger pivots (fingers_link)   0.0660
#     pad surfaces span                     0.0539 .. 0.1092  (centre 0.0816)
#     gripper bar (bracket, 103 mm wide)    0.0430 .. 0.0710
#     ee_gripper_link (vendor tool point)   0.0936
#
# The GRIPPER BAR is the binding constraint, not the pads -- it stands 35 mm
# proud of the gripper axis just behind the fingers, so
#
#     bar clearance = TOOL_OFFSET - 0.071 - object_radius
#
# Aim too short (the pad centre, or even ee_gripper_link) and the bar comes down
# inside the object and shunts it aside during the descent; aim too long and only
# the fingertips touch, which closes off centre and shoves the object. 0.1000
# leaves 8 mm of bar clearance on a 21 mm-radius object -- enough to survive the
# vision error -- with 30 mm of pad still on it.
#
# The same formula caps the object size: much past a 29 mm radius and the bracket
# reaches the object before the fingers close. Worth knowing before picking a
# target object for the real arm.
TOOL_OFFSET = np.array([0.1000, 0.0, 0.0], dtype=float)

# How far the gripper physically REACHES, for collision checking only: the
# wrist_rotate origin to the fingertips. This is deliberately longer than
# TOOL_OFFSET, because the two answer different questions:
#
#     TOOL_OFFSET           where the arm AIMS      (pad centre, 0.0936)
#     COLLISION_TIP_OFFSET  how far the arm REACHES (fingertips, 0.121)
#
# Conflating them under-covers the gripper: the last segment's capsule would stop
# at the aim point, leaving the fingertips outside the collision model, and the
# planner would sweep them through objects on a path it had certified clear.
COLLISION_TIP_OFFSET = np.array([0.121, 0.0, 0.0], dtype=float)

# Per-link capsule radii for the whole-body collision model. Index i is the
# segment from joint i to joint i+1 (segment 4 runs from wrist_rotate to the tool
# tip). Base radii are the lateral cross-section half-widths measured directly
# from the vendor STL meshes in
# interbotix_xsarm_descriptions/meshes/rx150_meshes/*.stl, rounded up:
#   segment 0  waist -> shoulder        shoulder.stl    ~0.0635
#   segment 1  shoulder -> elbow        upper_arm.stl   ~0.038
#   segment 2  elbow -> wrist_angle     forearm.stl     ~0.023
#   segment 3  wrist_angle -> wrist_rot wrist.stl       ~0.034
#   segment 4  wrist_rotate -> tool tip gripper_bar.stl ~0.052 (widest gripper part)
# A safety margin is added on top of every radius; all values are tunable.
_BASE_LINK_RADII = np.array([0.065, 0.040, 0.030, 0.040, 0.055], dtype=float)
CAPSULE_SAFETY_MARGIN = 0.015
LINK_CAPSULE_RADII = _BASE_LINK_RADII + CAPSULE_SAFETY_MARGIN


def translation(offset: np.ndarray) -> np.ndarray:
    """4x4 homogeneous translation matrix."""
    transform = np.eye(4)
    transform[:3, 3] = offset
    return transform


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """4x4 homogeneous rotation of ``angle`` radians about ``axis`` (Rodrigues)."""
    axis = axis / np.linalg.norm(axis)
    x_axis, y_axis, z_axis = axis
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    one_minus_cos = 1.0 - cos_theta
    return np.array(
        [
            [
                cos_theta + x_axis * x_axis * one_minus_cos,
                x_axis * y_axis * one_minus_cos - z_axis * sin_theta,
                x_axis * z_axis * one_minus_cos + y_axis * sin_theta,
                0.0,
            ],
            [
                y_axis * x_axis * one_minus_cos + z_axis * sin_theta,
                cos_theta + y_axis * y_axis * one_minus_cos,
                y_axis * z_axis * one_minus_cos - x_axis * sin_theta,
                0.0,
            ],
            [
                z_axis * x_axis * one_minus_cos - y_axis * sin_theta,
                z_axis * y_axis * one_minus_cos + x_axis * sin_theta,
                cos_theta + z_axis * z_axis * one_minus_cos,
                0.0,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _fk_core(
    q: np.ndarray, tool_offset: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray]]:
    """Walk the kinematic chain once, returning everything downstream FK needs.

    Returns (end_effector_xyz, end_effector_rotation, joint_positions,
    joint_axes_world) where ``joint_positions`` holds the world position of each
    of the 5 joint origins in order.
    """
    transform = np.eye(4)
    joint_positions: List[np.ndarray] = []
    joint_axes_world: List[np.ndarray] = []

    for origin, axis, joint_angle in zip(JOINT_ORIGINS, JOINT_AXES, q):
        transform = transform @ translation(origin)
        joint_positions.append(transform[:3, 3].copy())
        joint_axes_world.append(transform[:3, :3] @ axis)
        transform = transform @ axis_rotation(axis, joint_angle)

    transform = transform @ translation(tool_offset)
    end_effector_xyz = transform[:3, 3].copy()
    end_effector_rotation = transform[:3, :3].copy()
    return end_effector_xyz, end_effector_rotation, joint_positions, joint_axes_world


def forward_kinematics(
    q: np.ndarray, tool_offset: np.ndarray = TOOL_OFFSET
) -> Tuple[np.ndarray, np.ndarray]:
    """End-effector position and rotation for joint configuration ``q``."""
    ee_xyz, ee_rotation, _, _ = _fk_core(q, tool_offset)
    return ee_xyz, ee_rotation


def forward_kinematics_with_jacobian(
    q: np.ndarray, tool_offset: np.ndarray = TOOL_OFFSET
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """End-effector pose plus the 6xN geometric Jacobian for ``q``."""
    ee_xyz, ee_rotation, joint_positions, joint_axes_world = _fk_core(q, tool_offset)
    jacobian = np.zeros((6, len(JOINT_NAMES)))
    for index, (joint_origin, axis_world) in enumerate(
        zip(joint_positions, joint_axes_world)
    ):
        jacobian[:3, index] = np.cross(axis_world, ee_xyz - joint_origin)
        jacobian[3:, index] = axis_world
    return ee_xyz, ee_rotation, jacobian


def link_positions(
    q: np.ndarray, tool_offset: np.ndarray = COLLISION_TIP_OFFSET
) -> np.ndarray:
    """World positions of every joint plus the gripper tip, shape (6, 3).

    The 5 rows of joint origins followed by the tip define the 5 capsule
    segments used by :func:`check_arm_collision` (segment i spans rows i, i+1).

    Defaults to COLLISION_TIP_OFFSET, not TOOL_OFFSET: this describes the space
    the arm OCCUPIES, which runs to the fingertips, not the point it aims at.
    """
    ee_xyz, _, joint_positions, _ = _fk_core(q, tool_offset)
    return np.vstack(joint_positions + [ee_xyz])


def _point_segment_distances(
    points: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray
) -> np.ndarray:
    """Distance from each row of ``points`` (N,3) to the segment [seg_a, seg_b]."""
    segment = seg_b - seg_a
    denom = float(segment @ segment)
    if denom < 1e-12:
        return np.linalg.norm(points - seg_a, axis=1)
    t = np.clip((points - seg_a) @ segment / denom, 0.0, 1.0)
    projection = seg_a + t[:, None] * segment
    return np.linalg.norm(points - projection, axis=1)


def check_arm_collision(
    joint_positions: np.ndarray,
    obstacle_points: Optional[np.ndarray],
    radii: np.ndarray = LINK_CAPSULE_RADII,
    extra_margin: float = 0.0,
) -> Tuple[bool, int]:
    """Test the whole-arm capsule model against a set of obstacle points.

    ``joint_positions`` is the (6,3) array from :func:`link_positions`.
    ``obstacle_points`` is an (N,3) array of obstacle points in the same frame.
    Returns ``(collided, segment_index)``; ``segment_index`` is the first
    colliding link segment, or -1 if the arm is clear.
    """
    if obstacle_points is None or len(obstacle_points) == 0:
        return False, -1

    obstacle_points = np.asarray(obstacle_points, dtype=float)
    for segment_index in range(len(joint_positions) - 1):
        radius = float(radii[segment_index]) + extra_margin
        distances = _point_segment_distances(
            obstacle_points,
            joint_positions[segment_index],
            joint_positions[segment_index + 1],
        )
        if np.any(distances <= radius):
            return True, segment_index
    return False, -1
