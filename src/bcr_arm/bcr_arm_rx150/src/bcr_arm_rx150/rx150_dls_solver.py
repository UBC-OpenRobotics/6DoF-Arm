#!/usr/bin/env python3
"""Damped-least-squares Cartesian IK solver for the RX-150, as a reusable class.

This is the exact solver that ``rx150_dls_ik_executor.py`` runs to turn a
Cartesian target into a joint configuration, extracted so the path planner can
run the *same* solve to predict where the arm's body will be at each candidate
waypoint. If the planner used a different IK than the executor, its whole-body
collision check would be checking the wrong arm pose -- so sharing this solver is
what makes the collision check sound.

Pure numpy, no ROS dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from bcr_arm_common import rx150_kinematics


def _default_lower_limits() -> np.ndarray:
    pi_epsilon = 1e-5
    return np.array(
        [
            -np.pi + pi_epsilon,
            np.deg2rad(-106.0),
            np.deg2rad(-102.0),
            np.deg2rad(-100.0),
            -np.pi + pi_epsilon,
        ],
        dtype=float,
    )


def _default_upper_limits() -> np.ndarray:
    pi_epsilon = 1e-5
    return np.array(
        [
            np.pi - pi_epsilon,
            np.deg2rad(100.0),
            np.deg2rad(95.0),
            np.deg2rad(123.0),
            np.pi - pi_epsilon,
        ],
        dtype=float,
    )


@dataclass
class DlsSolverConfig:
    """Tuning for :class:`DlsSolver`. Defaults match rx150_dls_ik_executor.py."""

    position_weight: float = 3.0
    orientation_weight: float = 0.6
    damping: float = 0.10
    step_scale: float = 1.0
    max_joint_step: float = 0.12
    max_joint_velocity: float = 1.5
    servo_rate_hz: float = 15.0
    position_tolerance: float = 0.01
    orientation_tolerance: float = 0.12
    solver_max_iterations: int = 120
    orientation_mode: str = 'upright_free_yaw'  # or 'exact'
    tool_axis: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0], dtype=float)
    )
    tool_offset: np.ndarray = field(
        default_factory=lambda: rx150_kinematics.TOOL_OFFSET.copy()
    )
    joint_limits_lower: np.ndarray = field(default_factory=_default_lower_limits)
    joint_limits_upper: np.ndarray = field(default_factory=_default_upper_limits)


class DlsSolver:
    """Iterative damped-least-squares Cartesian IK solver."""

    def __init__(self, config: Optional[DlsSolverConfig] = None) -> None:
        self.config = config if config is not None else DlsSolverConfig()

    def solve(
        self,
        q_seed: np.ndarray,
        target_position: np.ndarray,
        target_rotation: Optional[np.ndarray] = None,
    ) -> Optional[Tuple[np.ndarray, float, float, bool]]:
        """Solve for a joint configuration reaching ``target_position``.

        Returns ``(q, position_error_norm, orientation_error_norm, converged)``.
        On non-convergence returns the best configuration found with
        ``converged=False``; returns ``None`` only if no finite solution exists.
        """
        cfg = self.config
        q_trial = np.asarray(q_seed, dtype=float).copy()
        best_q = q_trial.copy()
        best_cost = float('inf')
        best_position_error_norm = float('inf')
        best_orientation_error_norm = float('inf')

        for _ in range(cfg.solver_max_iterations):
            current_xyz, current_rotation, jacobian = (
                rx150_kinematics.forward_kinematics_with_jacobian(
                    q_trial, cfg.tool_offset
                )
            )
            position_error, orientation_error = self._compute_task_errors(
                current_xyz, current_rotation, target_position, target_rotation
            )
            position_error_norm = float(np.linalg.norm(position_error))
            orientation_error_norm = float(np.linalg.norm(orientation_error))
            cost = (cfg.position_weight * position_error_norm) + (
                cfg.orientation_weight * orientation_error_norm
            )

            if cost < best_cost:
                best_cost = cost
                best_q = q_trial.copy()
                best_position_error_norm = position_error_norm
                best_orientation_error_norm = orientation_error_norm

            if (
                position_error_norm <= cfg.position_tolerance
                and orientation_error_norm <= cfg.orientation_tolerance
            ):
                return q_trial.copy(), position_error_norm, orientation_error_norm, True

            dq = self._solve_dq(jacobian, position_error, orientation_error)
            if np.linalg.norm(dq) < 1e-8:
                break

            q_trial = np.clip(
                q_trial + dq,
                cfg.joint_limits_lower,
                cfg.joint_limits_upper,
            )

        if np.isfinite(best_cost):
            return best_q, best_position_error_norm, best_orientation_error_norm, False
        return None

    def _compute_task_errors(
        self,
        current_xyz: np.ndarray,
        current_rotation: np.ndarray,
        target_position: np.ndarray,
        target_rotation: Optional[np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray]:
        position_error = target_position - current_xyz
        if target_rotation is None:
            return position_error, np.zeros(3, dtype=float)

        if self.config.orientation_mode == 'exact':
            orientation_error = self._rotation_error(current_rotation, target_rotation)
        else:
            target_axis_world = target_rotation @ self.config.tool_axis
            orientation_error = self._axis_alignment_error(
                current_rotation, self.config.tool_axis, target_axis_world
            )
        return position_error, orientation_error

    def _solve_dq(
        self, jacobian: np.ndarray, position_error: np.ndarray, orientation_error: np.ndarray
    ) -> np.ndarray:
        cfg = self.config
        weighted_jacobian = np.vstack(
            [
                cfg.position_weight * jacobian[:3, :],
                cfg.orientation_weight * jacobian[3:, :],
            ]
        )
        weighted_error = np.concatenate(
            [
                cfg.position_weight * position_error,
                cfg.orientation_weight * orientation_error,
            ]
        )

        jj_t = weighted_jacobian @ weighted_jacobian.T
        damping_matrix = (cfg.damping ** 2) * np.eye(weighted_jacobian.shape[0])
        dq = weighted_jacobian.T @ np.linalg.solve(jj_t + damping_matrix, weighted_error)
        dq *= cfg.step_scale

        velocity_limited_step = cfg.max_joint_velocity / max(1.0, cfg.servo_rate_hz)
        dq = np.clip(dq, -velocity_limited_step, velocity_limited_step)
        dq = np.clip(dq, -cfg.max_joint_step, cfg.max_joint_step)
        return dq

    @staticmethod
    def _rotation_error(
        current_rotation: np.ndarray, target_rotation: np.ndarray
    ) -> np.ndarray:
        return 0.5 * (
            np.cross(current_rotation[:, 0], target_rotation[:, 0])
            + np.cross(current_rotation[:, 1], target_rotation[:, 1])
            + np.cross(current_rotation[:, 2], target_rotation[:, 2])
        )

    @staticmethod
    def _axis_alignment_error(
        current_rotation: np.ndarray,
        tool_axis: np.ndarray,
        target_axis_world: np.ndarray,
    ) -> np.ndarray:
        current_axis_world = current_rotation @ tool_axis
        current_axis_world = current_axis_world / np.linalg.norm(current_axis_world)
        target_axis_world = target_axis_world / np.linalg.norm(target_axis_world)
        return np.cross(current_axis_world, target_axis_world)
