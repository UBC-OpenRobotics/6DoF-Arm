#!/usr/bin/env python3
"""Shared RX-150 forward kinematics.

This module is the single source of truth for the RX-150's forward-kinematics
chain, used by the DLS IK solver/executor and its tests so they never
silently disagree about where the arm actually is.

Pure numpy, no ROS dependencies, so it can be unit-tested in isolation.
"""

from __future__ import annotations

from typing import List, Tuple

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
