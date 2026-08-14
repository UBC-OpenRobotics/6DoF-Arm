#!/usr/bin/env python3
"""Unit tests for the ROS-free joint-space RRT-Connect module.

Covers the two cases called out in WHOLE_BODY_MOTION_PLANNING_PLAN.md: a solvable
case must return a path that is genuinely collision-free (re-verified here), and a
boxed-in case must return None within budget. Plus the trivial early-outs and an
integration check against the real RX-150 capsule collision model.
"""

import numpy as np
import pytest

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150.rx150_rrt_connect import RrtConnectPlanner


# ---------------------------------------------------------------------------
# Abstract 2D configuration-space tests (algorithm only, no kinematics)
# ---------------------------------------------------------------------------
LOWER_2D = np.array([0.0, 0.0])
UPPER_2D = np.array([1.0, 1.0])


def _wall_with_gap(q):
    """A vertical wall at x in [0.4, 0.6] with a gap at y in [0.45, 0.55]."""
    x, y = q
    return 0.4 <= x <= 0.6 and not (0.45 <= y <= 0.55)


def _solid_wall(q):
    """A vertical wall at x in [0.4, 0.6] with no gap -- fully blocking."""
    x, _ = q
    return 0.4 <= x <= 0.6


def _assert_path_valid(planner, path, q_start, q_goal):
    assert path is not None
    assert len(path) >= 2
    np.testing.assert_allclose(path[0], q_start, atol=1e-9)
    np.testing.assert_allclose(path[-1], q_goal, atol=1e-9)
    for config in path:
        assert not planner._invalid(config), 'path config is in collision'
    for a, b in zip(path[:-1], path[1:]):
        assert planner._edge_valid(a, b), 'path edge sweeps through an obstacle'


def test_finds_path_through_gap():
    q_start = np.array([0.1, 0.5])
    q_goal = np.array([0.9, 0.5])
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _wall_with_gap,
        step=0.05, check_step=0.02, max_iters=5000, time_budget_sec=5.0,
        rng=np.random.default_rng(0),
    )
    path = planner.plan(q_start, [q_goal])
    _assert_path_valid(planner, path, q_start, q_goal)
    assert planner.last_stats['result'] == 'found'


def test_returns_none_when_walled_off():
    q_start = np.array([0.1, 0.5])
    q_goal = np.array([0.9, 0.5])  # itself collision-free, but unreachable
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _solid_wall,
        step=0.05, check_step=0.02, max_iters=2000, time_budget_sec=1.0,
        rng=np.random.default_rng(0),
    )
    path = planner.plan(q_start, [q_goal])
    assert path is None
    assert planner.last_stats['result'] in ('max_iters', 'time_budget')


def test_start_in_collision_returns_none_immediately():
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _solid_wall,
        rng=np.random.default_rng(0),
    )
    path = planner.plan(np.array([0.5, 0.5]), [np.array([0.9, 0.5])])
    assert path is None
    assert planner.last_stats['result'] == 'start_in_collision'


def test_all_goals_in_collision_returns_none_immediately():
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _solid_wall,
        rng=np.random.default_rng(0),
    )
    path = planner.plan(np.array([0.1, 0.5]), [np.array([0.5, 0.5])])
    assert path is None
    assert planner.last_stats['result'] == 'goals_in_collision'


def test_out_of_limits_config_is_invalid():
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, lambda q: False,
        rng=np.random.default_rng(0),
    )
    assert planner._invalid(np.array([1.5, 0.5]))
    assert planner._invalid(np.array([0.5, -0.1]))
    assert not planner._invalid(np.array([0.5, 0.5]))


def test_multiple_goals_uses_reachable_one():
    # One goal is walled off, the other is on the start's side and directly
    # reachable; the planner should still succeed via the reachable one.
    q_start = np.array([0.1, 0.5])
    blocked_goal = np.array([0.9, 0.5])
    reachable_goal = np.array([0.3, 0.5])
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _solid_wall,
        step=0.05, check_step=0.02, max_iters=5000, time_budget_sec=5.0,
        rng=np.random.default_rng(1),
    )
    path = planner.plan(q_start, [blocked_goal, reachable_goal])
    assert path is not None
    np.testing.assert_allclose(path[-1], reachable_goal, atol=1e-9)


def test_densified_path_has_small_steps():
    q_start = np.array([0.1, 0.5])
    q_goal = np.array([0.9, 0.5])
    step = 0.05
    planner = RrtConnectPlanner(
        LOWER_2D, UPPER_2D, _wall_with_gap,
        step=step, check_step=0.02, max_iters=5000, time_budget_sec=5.0,
        rng=np.random.default_rng(0),
    )
    path = planner.plan(q_start, [q_goal])
    assert path is not None
    for a, b in zip(path[:-1], path[1:]):
        assert np.linalg.norm(b - a) <= step + 1e-9


# ---------------------------------------------------------------------------
# Integration test against the real RX-150 whole-body capsule model
# ---------------------------------------------------------------------------
def _rx150_limits():
    pi_epsilon = 1e-5
    lower = np.array([-np.pi + pi_epsilon, np.deg2rad(-106.0), np.deg2rad(-102.0),
                      np.deg2rad(-100.0), -np.pi + pi_epsilon])
    upper = np.array([np.pi - pi_epsilon, np.deg2rad(100.0), np.deg2rad(95.0),
                      np.deg2rad(123.0), np.pi - pi_epsilon])
    return lower, upper


def _box_points(center, half_extents, spacing=0.02):
    """A dense grid of points filling an axis-aligned box (a solid obstacle)."""
    cx, cy, cz = center
    hx, hy, hz = half_extents
    xs = np.arange(cx - hx, cx + hx + spacing, spacing)
    ys = np.arange(cy - hy, cy + hy + spacing, spacing)
    zs = np.arange(cz - hz, cz + hz + spacing, spacing)
    grid = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=float)
    return grid


def test_rx150_capsule_collision_fn_plans_around_box():
    lower, upper = _rx150_limits()
    # A tall box directly in front (y=0). Start and goal face just left/right of
    # it (waist -/+0.8), both clear -- but the direct waist interpolation between
    # them passes through waist=0, which aims the arm straight into the box. RRT
    # must route the waist the *long* way around (through the back) to connect.
    obstacle = _box_points(center=(0.30, 0.0, 0.12),
                            half_extents=(0.03, 0.03, 0.12))

    def collision_fn(q):
        joints = rx150_kinematics.link_positions(q)
        collided, _ = rx150_kinematics.check_arm_collision(joints, obstacle)
        return collided

    q_start = np.array([-0.8, -0.3, 0.3, 0.0, 0.0])
    q_goal = np.array([0.8, -0.3, 0.3, 0.0, 0.0])

    planner = RrtConnectPlanner(
        lower, upper, collision_fn,
        step=0.10, check_step=0.05, max_iters=4000, time_budget_sec=5.0,
        rng=np.random.default_rng(0),
    )
    # Guard the test's own premise: start and goal must be collision-free.
    assert not collision_fn(q_start)
    assert not collision_fn(q_goal)

    path = planner.plan(q_start, [q_goal])
    assert path is not None, planner.last_stats
    for config in path:
        joints = rx150_kinematics.link_positions(config)
        collided, _ = rx150_kinematics.check_arm_collision(joints, obstacle)
        assert not collided, 'RRT path puts the arm body into the box'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
