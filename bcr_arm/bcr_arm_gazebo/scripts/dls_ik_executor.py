#!/usr/bin/env python3

import argparse
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DlsIkExecutor(Node):
    """Solve Cartesian IK targets for the BCR arm and publish joint trajectories."""

    def __init__(
        self,
        oneshot_point: Optional[PointStamped] = None,
        oneshot_pose: Optional[PoseStamped] = None,
    ):
        super().__init__("dls_ik_executor")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("target_topic", "/cartesian_target")
        self.declare_parameter("target_pose_topic", "/cartesian_target_pose")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "command_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("goal_time_sec", 3.0)
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("orientation_tolerance", 0.06)
        self.declare_parameter("damping_lambda", 0.08)
        self.declare_parameter("step_scale", 1.15)
        self.declare_parameter("max_joint_step", 0.18)
        self.declare_parameter("max_joint_velocity", 3.0)
        self.declare_parameter("deadband", 0.005)
        self.declare_parameter("servo_rate_hz", 25.0)
        self.declare_parameter("position_weight", 3.0)
        self.declare_parameter("orientation_weight", 0.9)
        self.declare_parameter("orientation_mode", "exact")
        self.declare_parameter("point_target_orientation_policy", "current")
        self.declare_parameter("prealign_orientation_first", False)
        self.declare_parameter("prealign_orientation_tolerance", 0.20)
        self.declare_parameter("neutral_qx", 0.0)
        self.declare_parameter("neutral_qy", -0.70710678)
        self.declare_parameter("neutral_qz", 0.0)
        self.declare_parameter("neutral_qw", 0.70710678)
        self.declare_parameter("tool_up_axis_x", 0.0)
        self.declare_parameter("tool_up_axis_y", 0.0)
        self.declare_parameter("tool_up_axis_z", 1.0)
        self.declare_parameter("world_up_axis_x", 0.0)
        self.declare_parameter("world_up_axis_y", 0.0)
        self.declare_parameter("world_up_axis_z", 1.0)
        self.declare_parameter("l1_len", 0.200)
        self.declare_parameter("l2_offset", 0.065)
        self.declare_parameter("l3_len", 0.225)
        self.declare_parameter("l4_offset", -0.065)
        self.declare_parameter("l5_len", 0.150)
        self.declare_parameter("l6_offset", 0.060)
        self.declare_parameter("l7_len", 0.125)
        self.declare_parameter("grasp_offset_x", 0.0)
        self.declare_parameter("grasp_offset_y", 0.0)
        self.declare_parameter("grasp_offset_z", 0.143)
        self.declare_parameter("solver_max_iterations", 120)
        self.declare_parameter("fallback_to_neutral_on_failure", True)
        self.declare_parameter("retry_after_neutral_attempts", 1)
        self.declare_parameter("neutral_retry_joint_tolerance", 0.08)

        self._world_frame = self.get_parameter("world_frame").value
        self._target_topic = self.get_parameter("target_topic").value
        self._target_pose_topic = self.get_parameter("target_pose_topic").value
        self._joint_state_topic = self.get_parameter("joint_state_topic").value
        self._command_topic = self.get_parameter("command_topic").value
        self._goal_time_sec = float(self.get_parameter("goal_time_sec").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance").value)
        self._orientation_tolerance = float(
            self.get_parameter("orientation_tolerance").value
        )
        self._damping = float(self.get_parameter("damping_lambda").value)
        self._step_scale = float(self.get_parameter("step_scale").value)
        self._max_joint_step = float(self.get_parameter("max_joint_step").value)
        self._max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._servo_rate_hz = max(1.0, float(self.get_parameter("servo_rate_hz").value))
        self._position_weight = float(self.get_parameter("position_weight").value)
        self._orientation_weight = float(self.get_parameter("orientation_weight").value)
        self._prealign_orientation_first = bool(
            self.get_parameter("prealign_orientation_first").value
        )
        self._prealign_orientation_tolerance = float(
            self.get_parameter("prealign_orientation_tolerance").value
        )

        point_target_orientation_policy = str(
            self.get_parameter("point_target_orientation_policy").value
        ).strip().lower()
        if point_target_orientation_policy not in {"current", "neutral", "none"}:
            self.get_logger().warning(
                "Unknown point_target_orientation_policy '%s'. Falling back to 'current'."
                % point_target_orientation_policy
            )
            point_target_orientation_policy = "current"
        self._point_target_orientation_policy = point_target_orientation_policy

        neutral_rotation = self._quaternion_to_rotation_matrix(
            np.array(
                [
                    float(self.get_parameter("neutral_qx").value),
                    float(self.get_parameter("neutral_qy").value),
                    float(self.get_parameter("neutral_qz").value),
                    float(self.get_parameter("neutral_qw").value),
                ],
                dtype=float,
            )
        )
        if neutral_rotation is None:
            self.get_logger().warning(
                "Neutral quaternion is invalid. Falling back to identity rotation."
            )
            neutral_rotation = np.eye(3)
        self._neutral_rotation = neutral_rotation

        orientation_mode = str(self.get_parameter("orientation_mode").value).strip().lower()
        if orientation_mode not in {"upright_free_yaw", "exact"}:
            self.get_logger().warning(
                "Unknown orientation_mode '%s'. Falling back to 'upright_free_yaw'."
                % orientation_mode
            )
            orientation_mode = "upright_free_yaw"
        self._orientation_mode = orientation_mode

        self._joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self._joint_limits_lower = np.array(
            [-6.28, -2.0, -6.28, -2.0, -6.28, -2.0, -6.28], dtype=float
        )
        self._joint_limits_upper = np.array(
            [6.28, 2.0, 6.28, 2.0, 6.28, 2.0, 6.28], dtype=float
        )

        self._l1_len = float(self.get_parameter("l1_len").value)
        self._l2_offset = float(self.get_parameter("l2_offset").value)
        self._l3_len = float(self.get_parameter("l3_len").value)
        self._l4_offset = float(self.get_parameter("l4_offset").value)
        self._l5_len = float(self.get_parameter("l5_len").value)
        self._l6_offset = float(self.get_parameter("l6_offset").value)
        self._l7_len = float(self.get_parameter("l7_len").value)
        self._grasp_offset = np.array(
            [
                float(self.get_parameter("grasp_offset_x").value),
                float(self.get_parameter("grasp_offset_y").value),
                float(self.get_parameter("grasp_offset_z").value),
            ],
            dtype=float,
        )
        self._solver_max_iterations = max(1, int(self.get_parameter("solver_max_iterations").value))
        self._fallback_to_neutral_on_failure = bool(
            self.get_parameter("fallback_to_neutral_on_failure").value
        )
        self._retry_after_neutral_attempts = max(
            0, int(self.get_parameter("retry_after_neutral_attempts").value)
        )
        self._neutral_retry_joint_tolerance = float(
            self.get_parameter("neutral_retry_joint_tolerance").value
        )
        self._neutral_carry_joint_positions = np.array(
            [0.0, -1.10, 0.0, 0.90, 0.0, -1.37, 0.0], dtype=float
        )

        self._origins = [
            np.array([0.0, 0.0, 0.025]),
            np.array([0.0, 0.0, self._l1_len]),
            np.array([self._l2_offset, 0.0, 0.0]),
            np.array([0.0, 0.0, self._l3_len]),
            np.array([self._l4_offset, 0.0, 0.0]),
            np.array([0.0, 0.0, self._l5_len]),
            np.array([self._l6_offset, 0.0, 0.0]),
        ]
        self._axes = [
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
        ]
        self._tool_offset = self._grasp_offset.copy()
        self._tool_up_axis = self._normalized_vector(
            np.array(
                [
                    float(self.get_parameter("tool_up_axis_x").value),
                    float(self.get_parameter("tool_up_axis_y").value),
                    float(self.get_parameter("tool_up_axis_z").value),
                ],
                dtype=float,
            ),
            np.array([0.0, 0.0, 1.0], dtype=float),
        )
        self._world_up_axis = self._normalized_vector(
            np.array(
                [
                    float(self.get_parameter("world_up_axis_x").value),
                    float(self.get_parameter("world_up_axis_y").value),
                    float(self.get_parameter("world_up_axis_z").value),
                ],
                dtype=float,
            ),
            np.array([0.0, 0.0, 1.0], dtype=float),
        )

        self._current_q: Optional[np.ndarray] = None
        self._target_position: Optional[np.ndarray] = None
        self._target_rotation: Optional[np.ndarray] = None
        self._target_kind: Optional[str] = None
        self._pending_solve = False
        self._retry_attempts_remaining = 0
        self._retry_after_neutral_pending = False

        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_callback, 10
        )
        self._trajectory_pub = self.create_publisher(JointTrajectory, self._command_topic, 10)
        self._servo_timer = self.create_timer(1.0 / self._servo_rate_hz, self._servo_timer_callback)

        self._oneshot = oneshot_point is not None or oneshot_pose is not None
        if not self._oneshot:
            self._target_sub = self.create_subscription(
                PointStamped, self._target_topic, self._target_callback, 10
            )
            self._target_pose_sub = self.create_subscription(
                PoseStamped, self._target_pose_topic, self._target_pose_callback, 10
            )
            self.get_logger().info(
                "Servoing on %s (PointStamped) and %s (PoseStamped); publishing to %s"
                % (self._target_topic, self._target_pose_topic, self._command_topic)
            )
        elif oneshot_pose is not None:
            self._set_pose_target(oneshot_pose, source="oneshot")
        elif oneshot_point is not None:
            self._set_point_target(oneshot_point, source="oneshot")

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(joint_name in positions for joint_name in self._joint_names):
            return
        self._current_q = np.array([positions[j] for j in self._joint_names], dtype=float)

    def _target_callback(self, msg: PointStamped) -> None:
        self._set_point_target(msg, source="topic")

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        self._set_pose_target(msg, source="topic")

    def _set_point_target(self, target: PointStamped, source: str) -> None:
        frame = target.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().error(
                "Target frame '%s' is not supported. Expected '%s'." % (frame, self._world_frame)
            )
            if self._oneshot:
                rclpy.shutdown()
            return

        self._target_position = np.array(
            [target.point.x, target.point.y, target.point.z], dtype=float
        )
        self._target_rotation = self._point_target_rotation_from_policy()
        self._target_kind = "point"
        self._pending_solve = True
        self._retry_attempts_remaining = self._retry_after_neutral_attempts
        self._retry_after_neutral_pending = False

        self.get_logger().info(
            "Accepted %s point target x=%.3f y=%.3f z=%.3f (orientation policy=%s)"
            % (
                source,
                self._target_position[0],
                self._target_position[1],
                self._target_position[2],
                self._point_target_orientation_policy,
            )
        )

    def _set_pose_target(self, target: PoseStamped, source: str) -> None:
        frame = target.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().error(
                "Target frame '%s' is not supported. Expected '%s'." % (frame, self._world_frame)
            )
            if self._oneshot:
                rclpy.shutdown()
            return

        rotation = self._quaternion_to_rotation_matrix(
            np.array(
                [
                    target.pose.orientation.x,
                    target.pose.orientation.y,
                    target.pose.orientation.z,
                    target.pose.orientation.w,
                ],
                dtype=float,
            )
        )
        if rotation is None:
            self.get_logger().error("Received invalid quaternion in pose target.")
            if self._oneshot:
                rclpy.shutdown()
            return

        self._target_position = np.array(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z], dtype=float
        )
        self._target_rotation = rotation
        self._target_kind = "pose"
        self._pending_solve = True
        self._retry_attempts_remaining = self._retry_after_neutral_attempts
        self._retry_after_neutral_pending = False
        self.get_logger().info(
            "Accepted %s pose target x=%.3f y=%.3f z=%.3f"
            % (source, self._target_position[0], self._target_position[1], self._target_position[2])
        )

    def _servo_timer_callback(self) -> None:
        if self._current_q is None or self._target_position is None:
            return

        if self._retry_after_neutral_pending:
            if self._is_near_joint_target(
                self._neutral_carry_joint_positions, self._neutral_retry_joint_tolerance
            ):
                self._retry_after_neutral_pending = False
                self._pending_solve = True
                self.get_logger().info("Neutral carry pose reached. Retrying previous target.")
            else:
                return

        if not self._pending_solve:
            if self._oneshot:
                current_xyz, current_rotation, _ = self._forward_kinematics(self._current_q)
                position_error, orientation_error = self._compute_task_errors(
                    current_xyz, current_rotation
                )
                if (
                    float(np.linalg.norm(position_error)) <= self._position_tolerance
                    and float(np.linalg.norm(orientation_error)) <= self._orientation_tolerance
                ):
                    rclpy.shutdown()
            return

        if self._target_rotation is None and self._target_kind == "point":
            _, current_rotation, _ = self._forward_kinematics(self._current_q)
            self._target_rotation = self._point_target_rotation_from_policy(current_rotation)

        solution = self._solve_target_configuration(self._current_q.copy())
        self._pending_solve = False
        if solution is None:
            self.get_logger().warning("IK solve did not converge for the requested target.")
            if self._oneshot:
                rclpy.shutdown()
            return

        q_command, position_error_norm, orientation_error_norm, converged = solution
        if (
            not converged
            and self._fallback_to_neutral_on_failure
            and self._target_kind == "point"
            and self._point_target_orientation_policy == "neutral"
        ):
            retry_message = ""
            if self._retry_attempts_remaining > 0:
                self._retry_attempts_remaining -= 1
                self._retry_after_neutral_pending = True
                retry_message = " Will retry the same target after resetting to neutral."
            else:
                retry_message = " No retry attempts remain; clearing the target after reset."
            self.get_logger().warning(
                "Locked neutral pose could not be satisfied within tolerance. "
                "Returning to neutral carry pose instead. Best position error: %.4f m, "
                "orientation error: %.4f rad.%s"
                % (position_error_norm, orientation_error_norm, retry_message)
            )
            self._publish_trajectory(self._neutral_carry_joint_positions, self._goal_time_sec)
            if not self._retry_after_neutral_pending:
                self._clear_target()
            if self._oneshot and not self._retry_after_neutral_pending:
                rclpy.shutdown()
            return

        self._publish_trajectory(q_command, self._goal_time_sec)
        self.get_logger().info(
            "Published solved joint target. Expected final position error: %.4f m, orientation error: %.4f rad"
            % (position_error_norm, orientation_error_norm)
        )

    def _compute_task_errors(
        self, current_xyz: np.ndarray, current_rotation: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        position_error = self._target_position - current_xyz

        if self._target_rotation is None and self._target_kind == "point":
            self._target_rotation = self._point_target_rotation_from_policy(current_rotation)

        if self._target_rotation is not None:
            orientation_error = self._rotation_error(current_rotation, self._target_rotation)
        elif self._orientation_mode == "exact":
            self._target_rotation = current_rotation.copy()
            orientation_error = np.zeros(3, dtype=float)
        else:
            orientation_error = self._upright_orientation_error(current_rotation)

        if (
            self._prealign_orientation_first
            and self._target_kind == "point"
            and self._target_rotation is not None
            and float(np.linalg.norm(orientation_error)) > self._prealign_orientation_tolerance
        ):
            position_error = np.zeros(3, dtype=float)

        return position_error, orientation_error

    def _solve_dq(
        self, jacobian: np.ndarray, position_error: np.ndarray, orientation_error: np.ndarray
    ) -> np.ndarray:
        weighted_jacobian = np.vstack(
            [
                self._position_weight * jacobian[:3, :],
                self._orientation_weight * jacobian[3:, :],
            ]
        )
        weighted_error = np.concatenate(
            [
                self._position_weight * position_error,
                self._orientation_weight * orientation_error,
            ]
        )

        jj_t = weighted_jacobian @ weighted_jacobian.T
        damping_matrix = (self._damping ** 2) * np.eye(weighted_jacobian.shape[0])
        dq = weighted_jacobian.T @ np.linalg.solve(jj_t + damping_matrix, weighted_error)
        dq *= self._step_scale

        velocity_limited_step = self._max_joint_velocity / self._servo_rate_hz
        dq = np.clip(dq, -velocity_limited_step, velocity_limited_step)
        dq = np.clip(dq, -self._max_joint_step, self._max_joint_step)
        return dq

    def _solve_target_configuration(
        self, q_seed: np.ndarray
    ) -> Optional[tuple[np.ndarray, float, float, bool]]:
        q_trial = q_seed.copy()
        best_q = q_trial.copy()
        best_cost = float("inf")
        best_position_error_norm = float("inf")
        best_orientation_error_norm = float("inf")

        for _ in range(self._solver_max_iterations):
            current_xyz, current_rotation, jacobian = self._forward_kinematics(
                q_trial, with_jacobian=True
            )
            position_error, orientation_error = self._compute_task_errors(current_xyz, current_rotation)
            position_error_norm = float(np.linalg.norm(position_error))
            orientation_error_norm = float(np.linalg.norm(orientation_error))
            cost = (self._position_weight * position_error_norm) + (
                self._orientation_weight * orientation_error_norm
            )

            if cost < best_cost:
                best_cost = cost
                best_q = q_trial.copy()
                best_position_error_norm = position_error_norm
                best_orientation_error_norm = orientation_error_norm

            if (
                position_error_norm <= self._position_tolerance
                and orientation_error_norm <= self._orientation_tolerance
            ):
                return q_trial.copy(), position_error_norm, orientation_error_norm, True

            dq = self._solve_dq(jacobian, position_error, orientation_error)
            if np.linalg.norm(dq) < 1e-8:
                break

            q_trial = np.clip(
                q_trial + dq, self._joint_limits_lower, self._joint_limits_upper
            )

        if np.isfinite(best_cost):
            return best_q, best_position_error_norm, best_orientation_error_norm, False
        return None

    def _clear_target(self) -> None:
        self._target_position = None
        self._target_rotation = None
        self._target_kind = None
        self._pending_solve = False
        self._retry_attempts_remaining = 0
        self._retry_after_neutral_pending = False

    def _is_near_joint_target(self, joint_target: np.ndarray, tolerance: float) -> bool:
        return bool(np.max(np.abs(self._current_q - joint_target)) <= tolerance)

    def _forward_kinematics(self, q: np.ndarray, with_jacobian: bool = False):
        transform = np.eye(4)
        joint_positions = []
        joint_axes_world = []

        for origin, axis, joint_angle in zip(self._origins, self._axes, q):
            transform = transform @ self._translation(origin)
            joint_positions.append(transform[:3, 3].copy())
            joint_axes_world.append(transform[:3, :3] @ axis)
            transform = transform @ self._rotation(axis, joint_angle)

        transform = transform @ self._translation(self._tool_offset)
        end_effector_xyz = transform[:3, 3].copy()
        end_effector_rotation = transform[:3, :3].copy()

        if not with_jacobian:
            return end_effector_xyz, end_effector_rotation, None

        jacobian = np.zeros((6, len(self._joint_names)))
        for index, (joint_origin, axis_world) in enumerate(zip(joint_positions, joint_axes_world)):
            jacobian[:3, index] = np.cross(axis_world, end_effector_xyz - joint_origin)
            jacobian[3:, index] = axis_world

        return end_effector_xyz, end_effector_rotation, jacobian

    def _upright_orientation_error(self, current_rotation: np.ndarray) -> np.ndarray:
        current_up = current_rotation @ self._tool_up_axis
        return np.cross(current_up, self._world_up_axis)

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

    def _point_target_rotation_from_policy(
        self, current_rotation: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        if self._point_target_orientation_policy == "none":
            return None
        if self._point_target_orientation_policy == "neutral":
            return self._neutral_rotation.copy()
        if current_rotation is not None:
            return current_rotation.copy()
        if self._current_q is not None:
            _, current_rotation, _ = self._forward_kinematics(self._current_q)
            return current_rotation.copy()
        return None

    @staticmethod
    def _rotation_error(current_rotation: np.ndarray, target_rotation: np.ndarray) -> np.ndarray:
        return 0.5 * (
            np.cross(current_rotation[:, 0], target_rotation[:, 0])
            + np.cross(current_rotation[:, 1], target_rotation[:, 1])
            + np.cross(current_rotation[:, 2], target_rotation[:, 2])
        )

    def _publish_trajectory(self, joint_positions: np.ndarray, time_from_start_sec: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = Duration(
            sec=int(time_from_start_sec),
            nanosec=int((time_from_start_sec % 1.0) * 1e9),
        )
        msg.points = [point]
        self._trajectory_pub.publish(msg)

    @staticmethod
    def _translation(offset: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, 3] = offset
        return transform

    @staticmethod
    def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
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

    @staticmethod
    def _normalized_vector(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9:
            return fallback.copy()
        return vector / norm

    @staticmethod
    def _quaternion_to_rotation_matrix(quaternion: np.ndarray) -> Optional[np.ndarray]:
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-9:
            return None
        x_axis, y_axis, z_axis, w_axis = quaternion / norm
        return np.array(
            [
                [
                    1.0 - 2.0 * (y_axis * y_axis + z_axis * z_axis),
                    2.0 * (x_axis * y_axis - z_axis * w_axis),
                    2.0 * (x_axis * z_axis + y_axis * w_axis),
                ],
                [
                    2.0 * (x_axis * y_axis + z_axis * w_axis),
                    1.0 - 2.0 * (x_axis * x_axis + z_axis * z_axis),
                    2.0 * (y_axis * z_axis - x_axis * w_axis),
                ],
                [
                    2.0 * (x_axis * z_axis - y_axis * w_axis),
                    2.0 * (y_axis * z_axis + x_axis * w_axis),
                    1.0 - 2.0 * (x_axis * x_axis + y_axis * y_axis),
                ],
            ],
            dtype=float,
        )


def _build_oneshot_point_from_args(args) -> Optional[PointStamped]:
    if args.x is None or args.y is None or args.z is None:
        return None
    if None not in (args.qx, args.qy, args.qz, args.qw):
        return None

    point = PointStamped()
    point.header.frame_id = args.frame
    point.point.x = args.x
    point.point.y = args.y
    point.point.z = args.z
    return point


def _build_oneshot_pose_from_args(args) -> Optional[PoseStamped]:
    if args.x is None or args.y is None or args.z is None:
        return None
    if None in (args.qx, args.qy, args.qz, args.qw):
        return None

    pose = PoseStamped()
    pose.header.frame_id = args.frame
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    pose.pose.orientation.x = args.qx
    pose.pose.orientation.y = args.qy
    pose.pose.orientation.z = args.qz
    pose.pose.orientation.w = args.qw
    return pose


def main(argv=None):
    parser = argparse.ArgumentParser(description="DLS Cartesian IK executor for the BCR arm.")
    parser.add_argument("--x", type=float, default=None, help="Target X in meters")
    parser.add_argument("--y", type=float, default=None, help="Target Y in meters")
    parser.add_argument("--z", type=float, default=None, help="Target Z in meters")
    parser.add_argument("--qx", type=float, default=None, help="Target orientation X quaternion")
    parser.add_argument("--qy", type=float, default=None, help="Target orientation Y quaternion")
    parser.add_argument("--qz", type=float, default=None, help="Target orientation Z quaternion")
    parser.add_argument("--qw", type=float, default=None, help="Target orientation W quaternion")
    parser.add_argument("--frame", type=str, default="world", help="Target frame")
    args, ros_args = parser.parse_known_args(argv)

    quaternion_fields = [args.qx, args.qy, args.qz, args.qw]
    if any(value is not None for value in quaternion_fields) and not all(
        value is not None for value in quaternion_fields
    ):
        parser.error("Provide all quaternion fields (--qx --qy --qz --qw) or none of them.")

    rclpy.init(args=ros_args)
    oneshot_pose = _build_oneshot_pose_from_args(args)
    oneshot_point = _build_oneshot_point_from_args(args)
    node = DlsIkExecutor(oneshot_point=oneshot_point, oneshot_pose=oneshot_pose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
