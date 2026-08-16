#!/usr/bin/env python3
"""Joint-space RRT-Connect planner for the RX-150 (pure algorithm, no ROS).

This is the whole-body motion-planning fallback the primary 2D grid planner
(``rx150_point_cloud_path_planner.py``) hands off to when it cannot find a body-
clear Cartesian path -- typically start and goal on opposite sides of a tall
obstacle, where "up and over" or "retract, rotate the waist, re-approach" has no
tool-tip representation.

RRT-Connect searches over the arm's joint angles instead of the end-effector
position: a configuration ``q`` fully determines where every link is, so the
collision check is exact for the *whole* arm body. It grows two trees -- one
rooted at the start config, one at the goal config(s) -- and tries to join them.
s
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional, Sequence

import numpy as np


# Extend outcomes.
_TRAPPED = 0   # the step toward the target was blocked (or invalid)
_ADVANCED = 1  # a new node was added, but the target was not reached
_REACHED = 2   # the target config was reached (within one step)


class _Tree:
    """A tree of joint configurations with parent back-pointers.

    Supports multiple roots (a forest) so the goal tree can be seeded with
    several IK branches at once; ``path_to_root`` walks up to whichever root a
    node descends from.
    """

    def __init__(self, root: np.ndarray) -> None:
        self.nodes: List[np.ndarray] = [np.asarray(root, dtype=float)]
        self.parents: List[int] = [-1]
        self._array: Optional[np.ndarray] = None  # cache for nearest-neighbour

    def add_root(self, q: np.ndarray) -> int:
        return self._append(np.asarray(q, dtype=float), -1)

    def add(self, q: np.ndarray, parent: int) -> int:
        return self._append(np.asarray(q, dtype=float), parent)

    def _append(self, q: np.ndarray, parent: int) -> int:
        self.nodes.append(q)
        self.parents.append(parent)
        self._array = None
        return len(self.nodes) - 1

    def nearest(self, q: np.ndarray) -> int:
        if self._array is None:
            self._array = np.asarray(self.nodes, dtype=float)
        delta = self._array - q
        distances_sq = np.einsum('ij,ij->i', delta, delta)
        return int(np.argmin(distances_sq))

    def path_to_root(self, index: int) -> List[np.ndarray]:
        """Nodes from ``index`` up to (and including) its root, meeting-first."""
        path: List[np.ndarray] = []
        while index != -1:
            path.append(self.nodes[index])
            index = self.parents[index]
        return path


class RrtConnectPlanner:
    """Bidirectional RRT-Connect over the arm's joint angles.

    Parameters
    ----------
    joint_lower, joint_upper:
        Per-joint limits (arrays of the same length as a config). Samples are
        drawn uniformly within these bounds; sourced from the same place the DLS
        solver/executor use so planning and execution agree on the reachable
        range.
    collision_fn:
        ``collision_fn(q) -> bool`` returning True if config ``q`` is in
        collision or otherwise invalid. Injected so this module stays ROS-free;
        in the stack it wraps ``rx150_kinematics`` FK + capsule check against the
        point-cloud-derived obstacle points.
    step:
        Tree growth increment (rad) per extend toward a sample.
    check_step:
        Edge collision-check resolution (rad). An edge between two configs is
        validated by testing intermediate configs at this spacing (endpoints
        included), so a single step cannot sweep the arm through an obstacle.
    max_iters, time_budget_sec:
        Search caps; ``plan`` returns None when either is hit first.
    smoothing_iters:
        Shortcut attempts applied to a found path.
    rng:
        Optional ``numpy.random.Generator`` for reproducible tests.
    """

    def __init__(
        self,
        joint_lower: Sequence[float],
        joint_upper: Sequence[float],
        collision_fn: Callable[[np.ndarray], bool],
        step: float = 0.10,
        check_step: float = 0.05,
        max_iters: int = 3000,
        time_budget_sec: float = 1.5,
        smoothing_iters: int = 100,
        rng: Optional[np.random.Generator] = None,
    ) -> None:
        self.joint_lower = np.asarray(joint_lower, dtype=float)
        self.joint_upper = np.asarray(joint_upper, dtype=float)
        if self.joint_lower.shape != self.joint_upper.shape:
            raise ValueError('joint_lower and joint_upper must have the same shape')
        self.collision_fn = collision_fn
        self.step = float(step)
        self.check_step = float(check_step)
        self.max_iters = int(max_iters)
        self.time_budget_sec = float(time_budget_sec)
        self.smoothing_iters = int(smoothing_iters)
        self.rng = rng if rng is not None else np.random.default_rng()

        # Populated by the most recent plan() call, for the caller to log.
        self.last_stats: dict = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan(
        self,
        q_start: Sequence[float],
        q_goals: Sequence[Sequence[float]],
    ) -> Optional[List[np.ndarray]]:
        """Plan a joint-space path from ``q_start`` to the nearest reachable goal.

        ``q_goals`` is one or more candidate goal configurations (e.g. multiple
        IK branches). Returns a smoothed list of configs from start to a goal
        (endpoints included), or None if the start/goals are invalid or the
        search budget is exhausted.
        """
        start_time = time.monotonic()
        q_start = np.asarray(q_start, dtype=float)
        goals = self._as_goal_list(q_goals)

        self.last_stats = {
            'iterations': 0,
            'nodes': 0,
            'elapsed_sec': 0.0,
            'result': 'none',
        }

        # A colliding start or no collision-free goal means no search is needed.
        if self._invalid(q_start):
            self.last_stats['result'] = 'start_in_collision'
            self.last_stats['elapsed_sec'] = time.monotonic() - start_time
            return None
        valid_goals = [g for g in goals if not self._invalid(g)]
        if not valid_goals:
            self.last_stats['result'] = 'goals_in_collision'
            self.last_stats['elapsed_sec'] = time.monotonic() - start_time
            return None

        start_tree = _Tree(q_start)
        goal_tree = _Tree(valid_goals[0])
        for extra_goal in valid_goals[1:]:
            goal_tree.add_root(extra_goal)

        tree_a, tree_b = start_tree, goal_tree
        a_is_start = True

        for iteration in range(self.max_iters):
            if time.monotonic() - start_time > self.time_budget_sec:
                self.last_stats['result'] = 'time_budget'
                break

            q_rand = self._sample()
            status, new_index = self._extend(tree_a, q_rand)
            if status != _TRAPPED:
                q_new = tree_a.nodes[new_index]
                connect_status, connect_index = self._connect(tree_b, q_new)
                if connect_status == _REACHED:
                    self.last_stats.update(
                        iterations=iteration + 1,
                        nodes=len(start_tree.nodes) + len(goal_tree.nodes),
                        elapsed_sec=time.monotonic() - start_time,
                        result='found',
                    )
                    raw_path = self._assemble(
                        tree_a.path_to_root(new_index),
                        tree_b.path_to_root(connect_index),
                        a_is_start,
                    )
                    return self._postprocess(raw_path)

            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start

        if self.last_stats['result'] == 'none':
            self.last_stats['result'] = 'max_iters'
        self.last_stats.update(
            iterations=self.max_iters,
            nodes=len(start_tree.nodes) + len(goal_tree.nodes),
            elapsed_sec=time.monotonic() - start_time,
        )
        return None

    # ------------------------------------------------------------------
    # Core RRT-Connect steps
    # ------------------------------------------------------------------
    def _extend(self, tree: _Tree, q_target: np.ndarray) -> tuple:
        """Grow ``tree`` one step toward ``q_target``.

        Returns ``(status, new_index)``; ``new_index`` is None when trapped.
        """
        nearest_index = tree.nearest(q_target)
        q_near = tree.nodes[nearest_index]
        q_new, reached = self._steer(q_near, q_target)
        if not self._edge_valid(q_near, q_new):
            return _TRAPPED, None
        new_index = tree.add(q_new, nearest_index)
        return (_REACHED if reached else _ADVANCED), new_index

    def _connect(self, tree: _Tree, q_target: np.ndarray) -> tuple:
        """Repeatedly extend ``tree`` toward ``q_target`` until reached/blocked."""
        last_index = None
        status = _ADVANCED
        while status == _ADVANCED:
            status, index = self._extend(tree, q_target)
            if index is not None:
                last_index = index
        return status, last_index

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> tuple:
        """Config one ``step`` from ``q_from`` toward ``q_to`` (clamped at q_to)."""
        delta = q_to - q_from
        distance = float(np.linalg.norm(delta))
        if distance <= self.step or distance < 1e-12:
            return q_to.copy(), True
        return q_from + (self.step / distance) * delta, False

    def _edge_valid(self, q_from: np.ndarray, q_to: np.ndarray) -> bool:
        """True if every config along [q_from, q_to] is collision-free.

        Samples the straight configuration-space segment at ``check_step``
        resolution, endpoints included.
        """
        delta = q_to - q_from
        distance = float(np.linalg.norm(delta))
        segment_count = max(1, int(np.ceil(distance / self.check_step)))
        for step_index in range(segment_count + 1):
            q = q_from + (step_index / segment_count) * delta
            if self._invalid(q):
                return False
        return True

    # ------------------------------------------------------------------
    # Path assembly / post-processing
    # ------------------------------------------------------------------
    @staticmethod
    def _assemble(
        path_a: List[np.ndarray],
        path_b: List[np.ndarray],
        a_is_start: bool,
    ) -> List[np.ndarray]:
        """Stitch the two meeting-first tree paths into one start->goal path.

        Each ``path_*`` runs meeting-node -> ... -> root. The trees meet at the
        same config (connect reached it exactly), so the duplicate is dropped.
        """
        if a_is_start:
            start_side = path_a[::-1]  # root_start ... q_meet
            goal_side = path_b         # q_meet ... root_goal
        else:
            start_side = path_b[::-1]  # root_start ... q_meet
            goal_side = path_a         # q_meet ... root_goal
        return start_side + goal_side[1:]

    def _postprocess(self, path: List[np.ndarray]) -> List[np.ndarray]:
        smoothed = self._smooth(path)
        return self._densify(smoothed)

    def _smooth(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Shortcut smoothing: replace detours with straight C-space segments."""
        if len(path) <= 2:
            return list(path)
        path = list(path)
        for _ in range(self.smoothing_iters):
            if len(path) <= 2:
                break
            i = int(self.rng.integers(0, len(path)))
            j = int(self.rng.integers(0, len(path)))
            if abs(i - j) < 2:
                continue
            low, high = (i, j) if i < j else (j, i)
            if self._edge_valid(path[low], path[high]):
                path = path[: low + 1] + path[high:]
        return path

    def _densify(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Re-space the path at ``step`` so executed segments stay small.

        Smoothing produces straight but possibly long segments; densifying to the
        same increment the search used keeps each commanded waypoint within the
        resolution the edges were validated at.
        """
        if len(path) <= 1:
            return [np.asarray(p, dtype=float) for p in path]
        dense: List[np.ndarray] = [np.asarray(path[0], dtype=float)]
        for start_config, end_config in zip(path[:-1], path[1:]):
            delta = end_config - start_config
            distance = float(np.linalg.norm(delta))
            segment_count = max(1, int(np.ceil(distance / self.step)))
            for step_index in range(1, segment_count + 1):
                dense.append(start_config + (step_index / segment_count) * delta)
        return dense

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _sample(self) -> np.ndarray:
        return self.rng.uniform(self.joint_lower, self.joint_upper)

    def _invalid(self, q: np.ndarray) -> bool:
        if np.any(q < self.joint_lower) or np.any(q > self.joint_upper):
            return True
        return bool(self.collision_fn(q))

    @staticmethod
    def _as_goal_list(q_goals: Sequence[Sequence[float]]) -> List[np.ndarray]:
        array = np.asarray(q_goals, dtype=float)
        if array.ndim == 1:
            return [array]
        return [np.asarray(row, dtype=float) for row in array]
