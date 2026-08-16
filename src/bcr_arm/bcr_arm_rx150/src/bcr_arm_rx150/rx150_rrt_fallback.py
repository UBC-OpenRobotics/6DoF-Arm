#!/usr/bin/env python3
"""Turns a Cartesian target into an RRT-Connect joint path (ROS-free).

The planner node (`rx150_point_cloud_path_planner.py`) invokes this when its 2D
grid search cannot find a whole-body-clear Cartesian path. Everything here is pure
numpy so it can be unit-tested without ROS:

  * ``make_collision_fn`` builds the ``collision_fn(q)`` RRT needs from the same
    whole-arm capsule model + obstacle points the A* body check already uses.
  * ``generate_goal_configs`` runs the shared DLS solver from several seeds to get
    multiple IK branches for the target, dropping any that collide.
  * ``plan_joint_path`` wires those together with :class:`RrtConnectPlanner` and
    returns a joint-space path (or None), plus stats for the caller to log.

Joint limits come straight from the ``DlsSolver`` config, so planning and
execution never disagree about the reachable range.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

import numpy as np

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150.rx150_dls_solver import DlsSolver
from bcr_arm_rx150.rx150_rrt_connect import RrtConnectPlanner


CollisionFn = Callable[[np.ndarray], bool]


def make_collision_fn(
    obstacle_points: Optional[np.ndarray],
    extra_margin: float = 0.0,
) -> CollisionFn:
    """Return ``collision_fn(q)`` testing the whole arm against obstacle points.

    Uses exactly the same FK + capsule model as the A* body check
    (``rx150_kinematics.link_positions`` + ``check_arm_collision``), so both
    planners agree on what "collision" means. Joint-limit checking is handled by
    the RRT planner itself, so this only reports geometric collisions.
    """
    points = None
    if obstacle_points is not None and len(obstacle_points) > 0:
        points = np.asarray(obstacle_points, dtype=float)

    def collision_fn(q: np.ndarray) -> bool:
        joints = rx150_kinematics.link_positions(q)
        collided, _ = rx150_kinematics.check_arm_collision(
            joints, points, extra_margin=extra_margin
        )
        return collided

    return collision_fn


def _seed_configs(q_start: np.ndarray, target_position: np.ndarray) -> List[np.ndarray]:
    """A handful of IK seeds giving different branches to converge from.

    The far-side goal is often reachable on only one branch, so trying several
    matters: current pose, neutral, the waist aimed at the target's azimuth (and
    the opposite side), and an elbow-flipped variant.
    """
    q_start = np.asarray(q_start, dtype=float)
    azimuth = float(np.arctan2(target_position[1], target_position[0]))

    neutral = np.zeros_like(q_start)

    waist_aimed = q_start.copy()
    waist_aimed[0] = azimuth

    waist_opposite = q_start.copy()
    waist_opposite[0] = _wrap_angle(azimuth + np.pi)

    elbow_flipped = q_start.copy()
    elbow_flipped[2] = -elbow_flipped[2]

    return [q_start.copy(), neutral, waist_aimed, waist_opposite, elbow_flipped]


def _wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def generate_goal_configs(
    solver: DlsSolver,
    q_start: np.ndarray,
    target_position: np.ndarray,
    collision_fn: CollisionFn,
    target_rotation: Optional[np.ndarray] = None,
    dedupe_tol: float = 0.05,
) -> List[np.ndarray]:
    """Solve IK from several seeds; keep converged, collision-free, distinct goals.

    Position-only by default (``target_rotation=None``), matching the planner's
    own per-waypoint solve -- for routing feasibility we care that the tool tip
    reaches the point, not its final orientation.
    """
    q_start = np.asarray(q_start, dtype=float)
    target_position = np.asarray(target_position, dtype=float)
    lower = solver.config.joint_limits_lower
    upper = solver.config.joint_limits_upper

    goals: List[np.ndarray] = []
    for seed in _seed_configs(q_start, target_position):
        seed = np.clip(seed, lower, upper)
        result = solver.solve(seed, target_position, target_rotation)
        if result is None:
            continue
        q_solution, _, _, converged = result
        if not converged:
            continue
        if np.any(q_solution < lower) or np.any(q_solution > upper):
            continue
        if collision_fn(q_solution):
            continue
        if any(np.linalg.norm(q_solution - existing) < dedupe_tol for existing in goals):
            continue
        goals.append(q_solution)

    return goals


@dataclass
class RrtFallbackResult:
    """Outcome of a fallback attempt."""

    path: Optional[List[np.ndarray]]
    goal_count: int
    stats: dict

    @property
    def succeeded(self) -> bool:
        return self.path is not None


def plan_joint_path(
    q_start: np.ndarray,
    target_position: np.ndarray,
    obstacle_points: Optional[np.ndarray],
    solver: DlsSolver,
    target_rotation: Optional[np.ndarray] = None,
    extra_margin: float = 0.0,
    step: float = 0.10,
    check_step: float = 0.05,
    max_iters: int = 3000,
    time_budget_sec: float = 1.5,
    smoothing_iters: int = 100,
    rng: Optional[np.random.Generator] = None,
) -> RrtFallbackResult:
    """Full fallback: goal IK branches -> RRT-Connect -> joint path (or None)."""
    collision_fn = make_collision_fn(obstacle_points, extra_margin=extra_margin)
    goals = generate_goal_configs(
        solver, q_start, target_position, collision_fn, target_rotation
    )
    if not goals:
        return RrtFallbackResult(path=None, goal_count=0, stats={'result': 'no_goal_config'})

    planner = RrtConnectPlanner(
        solver.config.joint_limits_lower,
        solver.config.joint_limits_upper,
        collision_fn,
        step=step,
        check_step=check_step,
        max_iters=max_iters,
        time_budget_sec=time_budget_sec,
        smoothing_iters=smoothing_iters,
        rng=rng,
    )
    path = planner.plan(np.asarray(q_start, dtype=float), goals)
    return RrtFallbackResult(path=path, goal_count=len(goals), stats=planner.last_stats)
