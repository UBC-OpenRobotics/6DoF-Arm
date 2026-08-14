#!/usr/bin/env python3
"""Tests for the ROS-free RRT fallback glue (goal IK + collision_fn + plan)."""

import numpy as np
import pytest

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150.rx150_dls_solver import DlsSolver
from bcr_arm_rx150.rx150_rrt_fallback import (
    generate_goal_configs,
    make_collision_fn,
    plan_joint_path,
)


def _box_points(center, half_extents, spacing=0.02):
    cx, cy, cz = center
    hx, hy, hz = half_extents
    xs = np.arange(cx - hx, cx + hx + spacing, spacing)
    ys = np.arange(cy - hy, cy + hy + spacing, spacing)
    zs = np.arange(cz - hz, cz + hz + spacing, spacing)
    return np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=float)


def test_make_collision_fn_none_obstacles_never_collides():
    collision_fn = make_collision_fn(None)
    assert collision_fn(np.zeros(5)) is False
    collision_fn_empty = make_collision_fn(np.empty((0, 3)))
    assert collision_fn_empty(np.zeros(5)) is False


def test_generate_goal_configs_reachable_clear_target():
    solver = DlsSolver()
    collision_fn = make_collision_fn(None)  # no obstacles
    q_start = np.zeros(5)
    target = np.array([0.22, 0.0, 0.16])
    goals = generate_goal_configs(solver, q_start, target, collision_fn)
    assert len(goals) >= 1
    for q in goals:
        ee = rx150_kinematics.forward_kinematics(q)[0]
        assert np.linalg.norm(ee - target) < 0.02


def test_generate_goal_configs_drops_colliding_goals():
    solver = DlsSolver()
    q_start = np.zeros(5)
    target = np.array([0.22, 0.0, 0.16])
    # A box wrapped tightly around the target point: any config reaching it must
    # put the gripper into the box, so every goal branch should be dropped.
    obstacle = _box_points(center=(0.22, 0.0, 0.16),
                           half_extents=(0.05, 0.05, 0.05))
    collision_fn = make_collision_fn(obstacle)
    goals = generate_goal_configs(solver, q_start, target, collision_fn)
    assert goals == []


def test_plan_joint_path_routes_around_front_box():
    solver = DlsSolver()
    # Tall box in front; target on the +y side of it, start facing -y. A direct
    # joint interpolation would sweep the arm through the box.
    obstacle = _box_points(center=(0.28, 0.0, 0.12),
                           half_extents=(0.03, 0.03, 0.12))
    q_start = np.array([-0.8, -0.3, 0.3, 0.0, 0.0])
    target = np.array([0.20, 0.26, 0.20])  # reachable, +y side, clear of the box

    result = plan_joint_path(
        q_start, target, obstacle, solver,
        max_iters=4000, time_budget_sec=5.0,
        rng=np.random.default_rng(0),
    )
    assert result.succeeded, result.stats
    assert result.goal_count >= 1

    # Start matches, end reaches the target, and no config collides.
    np.testing.assert_allclose(result.path[0], q_start, atol=1e-9)
    ee_end = rx150_kinematics.forward_kinematics(result.path[-1])[0]
    assert np.linalg.norm(ee_end - target) < 0.02
    for q in result.path:
        joints = rx150_kinematics.link_positions(q)
        collided, _ = rx150_kinematics.check_arm_collision(joints, obstacle)
        assert not collided


def test_plan_joint_path_no_goal_config_reports_cleanly():
    solver = DlsSolver()
    q_start = np.zeros(5)
    target = np.array([0.22, 0.0, 0.16])
    obstacle = _box_points(center=(0.22, 0.0, 0.16),
                           half_extents=(0.05, 0.05, 0.05))
    result = plan_joint_path(q_start, target, obstacle, solver,
                             rng=np.random.default_rng(0))
    assert not result.succeeded
    assert result.stats['result'] == 'no_goal_config'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
