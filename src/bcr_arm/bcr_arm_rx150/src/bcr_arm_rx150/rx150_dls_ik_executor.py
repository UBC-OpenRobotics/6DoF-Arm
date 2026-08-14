#!/usr/bin/env python3

import argparse
from typing import Optional

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150.rx150_dls_solver import DlsSolver, DlsSolverConfig
from geometry_msgs.msg import PointStamped, PoseStamped
from interbotix_xs_msgs.msg import JointGroupCommand
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Rx150DlsIkExecutor(Node):
    """Solve Cartesian targets for the physical RX-150 and publish joint trajectories."""

    def __init__(
        self,
        oneshot_point: Optional[PointStamped] = None,
        oneshot_pose: Optional[PoseStamped] = None,
    ):
        super().__init__('rx150_dls_ik_executor')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('target_topic', '/cartesian_target')
        self.declare_parameter('target_pose_topic', '/cartesian_target_pose')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('command_topic', '/rx150/commands/joint_group')
        self.declare_parameter('command_mode', 'group')
        self.declare_parameter('goal_time_sec', 2.5)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.12)
        self.declare_parameter('damping_lambda', 0.10)
        self.declare_parameter('step_scale', 1.0)
        self.declare_parameter('max_joint_step', 0.12)
        self.declare_parameter('max_joint_velocity', 1.5)
        self.declare_parameter('servo_rate_hz', 15.0)
        self.declare_parameter('position_weight', 3.0)
        self.declare_parameter('orientation_weight', 0.6)
        self.declare_parameter('orientation_mode', 'upright_free_yaw')
        self.declare_parameter('point_target_orientation_policy', 'current')
        self.declare_parameter('neutral_qx', 0.0)
        self.declare_parameter('neutral_qy', 0.0)
        self.declare_parameter('neutral_qz', 0.0)
        self.declare_parameter('neutral_qw', 1.0)
        self.declare_parameter('tool_axis_x', 1.0)
        self.declare_parameter('tool_axis_y', 0.0)
        self.declare_parameter('tool_axis_z', 0.0)
        self.declare_parameter('tool_offset_x', 0.108)
        self.declare_parameter('tool_offset_y', 0.0)
        self.declare_parameter('tool_offset_z', 0.0)
        self.declare_parameter('solver_max_iterations', 120)
        self.declare_parameter('fallback_to_neutral_on_failure', True)
        self.declare_parameter('retry_after_neutral_attempts', 1)
        self.declare_parameter('neutral_retry_joint_tolerance', 0.08)
        self.declare_parameter('joint_command_topic', '/rx150/joint_command')
        self.declare_parameter('joint_command_time_sec', 0.6)

        self._world_frame = self.get_parameter('world_frame').value
        self._target_topic = self.get_parameter('target_topic').value
        self._target_pose_topic = self.get_parameter('target_pose_topic').value
        self._joint_state_topic = self.get_parameter('joint_state_topic').value
        self._command_topic = self.get_parameter('command_topic').value
        command_mode = str(self.get_parameter('command_mode').value).strip().lower()
        if command_mode not in {'group', 'trajectory'}:
            self.get_logger().warning(
                "Unknown command_mode '%s'. Falling back to 'group'." % command_mode
            )
            command_mode = 'group'
        self._command_mode = command_mode
        self._goal_time_sec = float(self.get_parameter('goal_time_sec').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._orientation_tolerance = float(
            self.get_parameter('orientation_tolerance').value
        )
        self._damping = float(self.get_parameter('damping_lambda').value)
        self._step_scale = float(self.get_parameter('step_scale').value)
        self._max_joint_step = float(self.get_parameter('max_joint_step').value)
        self._max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)
        self._servo_rate_hz = max(1.0, float(self.get_parameter('servo_rate_hz').value))
        self._position_weight = float(self.get_parameter('position_weight').value)
        self._orientation_weight = float(self.get_parameter('orientation_weight').value)

        orientation_mode = str(self.get_parameter('orientation_mode').value).strip().lower()
        if orientation_mode not in {'exact', 'upright_free_yaw'}:
            self.get_logger().warning(
                "Unknown orientation_mode '%s'. Falling back to 'upright_free_yaw'."
                % orientation_mode
            )
            orientation_mode = 'upright_free_yaw'
        self._orientation_mode = orientation_mode

        point_target_orientation_policy = str(
            self.get_parameter('point_target_orientation_policy').value
        ).strip().lower()
        if point_target_orientation_policy not in {'current', 'neutral', 'none'}:
            self.get_logger().warning(
                "Unknown point_target_orientation_policy '%s'. Falling back to 'current'."
                % point_target_orientation_policy
            )
            point_target_orientation_policy = 'current'
        self._point_target_orientation_policy = point_target_orientation_policy

        neutral_rotation = self._quaternion_to_rotation_matrix(
            np.array(
                [
                    float(self.get_parameter('neutral_qx').value),
                    float(self.get_parameter('neutral_qy').value),
                    float(self.get_parameter('neutral_qz').value),
                    float(self.get_parameter('neutral_qw').value),
                ],
                dtype=float,
            )
        )
        self._neutral_rotation = np.eye(3) if neutral_rotation is None else neutral_rotation
        self._tool_axis = self._normalized_vector(
            np.array(
                [
                    float(self.get_parameter('tool_axis_x').value),
                    float(self.get_parameter('tool_axis_y').value),
                    float(self.get_parameter('tool_axis_z').value),
                ],
                dtype=float,
            ),
            np.array([1.0, 0.0, 0.0], dtype=float),
        )

        self._joint_names = list(rx150_kinematics.JOINT_NAMES)
        self._neutral_carry_joint_positions = np.array(
            [0.0, -0.35, 0.75, -0.40, 0.0],
            dtype=float,
        )
        self._solver_max_iterations = max(
            1, int(self.get_parameter('solver_max_iterations').value)
        )
        self._fallback_to_neutral_on_failure = bool(
            self.get_parameter('fallback_to_neutral_on_failure').value
        )
        self._retry_after_neutral_attempts = max(
            0, int(self.get_parameter('retry_after_neutral_attempts').value)
        )
        self._neutral_retry_joint_tolerance = float(
            self.get_parameter('neutral_retry_joint_tolerance').value
        )
        self._joint_command_topic = str(self.get_parameter('joint_command_topic').value)
        self._joint_command_time_sec = max(
            0.0, float(self.get_parameter('joint_command_time_sec').value)
        )
        .
        self._tool_offset = np.array(
            [
                float(self.get_parameter('tool_offset_x').value),
                float(self.get_parameter('tool_offset_y').value),
                float(self.get_parameter('tool_offset_z').value),
            ],
            dtype=float,
        )

        # Single source of truth for the DLS solve: the same DlsSolver the path
        # planner and RRT fallback use, built from this node's ROS params. Joint
        # limits come from the solver config so planning and execution never
        # disagree on the reachable range.
        self._solver = DlsSolver(DlsSolverConfig(
            position_weight=self._position_weight,
            orientation_weight=self._orientation_weight,
            damping=self._damping,
            step_scale=self._step_scale,
            max_joint_step=self._max_joint_step,
            max_joint_velocity=self._max_joint_velocity,
            servo_rate_hz=self._servo_rate_hz,
            position_tolerance=self._position_tolerance,
            orientation_tolerance=self._orientation_tolerance,
            solver_max_iterations=self._solver_max_iterations,
            orientation_mode=self._orientation_mode,
            tool_axis=self._tool_axis,
            tool_offset=self._tool_offset,
        ))
        self._joint_limits_lower = self._solver.config.joint_limits_lower
        self._joint_limits_upper = self._solver.config.joint_limits_upper

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
        # Direct joint-command channel (no IK); used by the RRT whole-body fallback.
        self._joint_command_sub = self.create_subscription(
            JointState, self._joint_command_topic, self._joint_command_callback, 10
        )
        self._group_pub = None
        self._trajectory_pub = None
        if self._command_mode == 'group':
            self._group_pub = self.create_publisher(
                JointGroupCommand, self._command_topic, 10
            )
        else:
            self._trajectory_pub = self.create_publisher(
                JointTrajectory, self._command_topic, 10
            )
        self._servo_timer = self.create_timer(
            1.0 / self._servo_rate_hz, self._servo_timer_callback
        )

        self._oneshot = oneshot_point is not None or oneshot_pose is not None
        if not self._oneshot:
            self.create_subscription(
                PointStamped, self._target_topic, self._target_callback, 10
            )
            self.create_subscription(
                PoseStamped, self._target_pose_topic, self._target_pose_callback, 10
            )
            self.get_logger().info(
                'RX-150 DLS solver listening on %s and %s; publishing %s commands to %s'
                % (
                    self._target_topic,
                    self._target_pose_topic,
                    self._command_mode,
                    self._command_topic,
                )
            )
        elif oneshot_pose is not None:
            self._set_pose_target(oneshot_pose, source='oneshot')
        elif oneshot_point is not None:
            self._set_point_target(oneshot_point, source='oneshot')

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(joint_name in positions for joint_name in self._joint_names):
            return
        self._current_q = np.array([positions[name] for name in self._joint_names], dtype=float)

    def _joint_command_callback(self, msg: JointState) -> None:
        """Command a pre-computed joint configuration directly, bypassing IK.

        Used by the RRT-Connect whole-body fallback: the config is already
        collision-checked in joint space, so re-solving it through Cartesian IK
        (which could land in a different, unchecked posture) is exactly what we
        must not do. The config is published through the same mode-aware channel
        (`group`/`trajectory`) the solved targets use, so it is portable to the
        physical arm unchanged.
        """
        positions = dict(zip(msg.name, msg.position))
        base = self._current_q if self._current_q is not None else np.zeros(
            len(self._joint_names), dtype=float
        )
        q_command = np.array(
            [positions.get(name, base[index]) for index, name in enumerate(self._joint_names)],
            dtype=float,
        )
        q_command = np.clip(q_command, self._joint_limits_lower, self._joint_limits_upper)
        # A direct joint command supersedes any pending Cartesian solve so the
        # servo loop does not fight it (the two channels are mutually exclusive).
        self._clear_target()
        self._publish_trajectory(q_command, self._joint_command_time_sec)

    def _target_callback(self, msg: PointStamped) -> None:
        self._set_point_target(msg, source='topic')

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        self._set_pose_target(msg, source='topic')

    def _set_point_target(self, target: PointStamped, source: str) -> None:
        frame = target.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().error(
                "Target frame '%s' is not supported. Expected '%s'."
                % (frame, self._world_frame)
            )
            if self._oneshot:
                rclpy.shutdown()
            return

        self._target_position = np.array(
            [target.point.x, target.point.y, target.point.z], dtype=float
        )
        self._target_rotation = self._point_target_rotation_from_policy()
        self._target_kind = 'point'
        self._pending_solve = True
        self._retry_attempts_remaining = self._retry_after_neutral_attempts
        self._retry_after_neutral_pending = False
        self.get_logger().info(
            'Accepted %s point target x=%.3f y=%.3f z=%.3f'
            % (
                source,
                self._target_position[0],
                self._target_position[1],
                self._target_position[2],
            )
        )

    def _set_pose_target(self, target: PoseStamped, source: str) -> None:
        frame = target.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().error(
                "Target frame '%s' is not supported. Expected '%s'."
                % (frame, self._world_frame)
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
            self.get_logger().error('Received invalid quaternion in pose target.')
            if self._oneshot:
                rclpy.shutdown()
            return

        self._target_position = np.array(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z],
            dtype=float,
        )
        self._target_rotation = rotation
        self._target_kind = 'pose'
        self._pending_solve = True
        self._retry_attempts_remaining = self._retry_after_neutral_attempts
        self._retry_after_neutral_pending = False
        self.get_logger().info(
            'Accepted %s pose target x=%.3f y=%.3f z=%.3f'
            % (
                source,
                self._target_position[0],
                self._target_position[1],
                self._target_position[2],
            )
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
                self.get_logger().info('Neutral carry pose reached. Retrying previous target.')
            else:
                return

        if not self._pending_solve:
            return

        solution = self._solve_target_configuration(self._current_q.copy())
        self._pending_solve = False
        if solution is None:
            self.get_logger().warning('IK solve did not converge for the requested target.')
            if self._oneshot:
                rclpy.shutdown()
            return

        q_command, position_error_norm, orientation_error_norm, converged = solution
        if not converged and self._fallback_to_neutral_on_failure:
            retry_message = ''
            if self._retry_attempts_remaining > 0:
                self._retry_attempts_remaining -= 1
                self._retry_after_neutral_pending = True
                retry_message = ' Will retry the same target after resetting to neutral.'
            else:
                retry_message = ' No retry attempts remain; clearing the target after reset.'
            self.get_logger().warning(
                'RX-150 DLS solve did not converge within tolerance. '
                'Returning to neutral carry pose instead. Best position error: %.4f m, '
                'orientation error: %.4f rad.%s'
                % (position_error_norm, orientation_error_norm, retry_message)
            )
            self._publish_trajectory(self._neutral_carry_joint_positions, self._goal_time_sec)
            if not self._retry_after_neutral_pending:
                self._clear_target()
            return

        self._publish_trajectory(q_command, self._goal_time_sec)
        self.get_logger().info(
            'Published solved RX-150 joint target. Expected final position error: %.4f m, '
            'orientation error: %.4f rad'
            % (position_error_norm, orientation_error_norm)
        )

    def _solve_target_configuration(
        self, q_seed: np.ndarray
    ) -> Optional[tuple[np.ndarray, float, float, bool]]:
        """Solve the current target from ``q_seed`` via the shared DlsSolver.

        Returns ``(q, position_error, orientation_error, converged)`` or None --
        the same contract the servo loop already consumes.
        """
        return self._solver.solve(
            q_seed, self._target_position, self._target_rotation
        )

    def _forward_kinematics(self, q: np.ndarray, with_jacobian: bool = False):
        if with_jacobian:
            return rx150_kinematics.forward_kinematics_with_jacobian(
                q, self._tool_offset
            )
        end_effector_xyz, end_effector_rotation = rx150_kinematics.forward_kinematics(
            q, self._tool_offset
        )
        return end_effector_xyz, end_effector_rotation, None

    def _point_target_rotation_from_policy(
        self, current_rotation: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        if self._point_target_orientation_policy == 'none':
            return None
        if self._point_target_orientation_policy == 'neutral':
            return self._neutral_rotation.copy()
        if current_rotation is not None:
            return current_rotation.copy()
        if self._current_q is not None:
            _, current_rotation, _ = self._forward_kinematics(self._current_q)
            return current_rotation.copy()
        return None

    def _publish_trajectory(self, joint_positions: np.ndarray, time_from_start_sec: float) -> None:
        if self._command_mode == 'group':
            msg = JointGroupCommand()
            msg.name = 'arm'
            msg.cmd = joint_positions.tolist()
            self._group_pub.publish(msg)
            return

        msg = JointTrajectory()
        msg.joint_names = self._joint_names

        if self._current_q is not None:
            current_point = JointTrajectoryPoint()
            current_point.positions = self._current_q.tolist()
            current_point.time_from_start.sec = 0
            current_point.time_from_start.nanosec = 0
            msg.points.append(current_point)

        target_point = JointTrajectoryPoint()
        target_point.positions = joint_positions.tolist()
        whole_sec = max(0.0, float(time_from_start_sec))
        sec = int(whole_sec)
        nanosec = int(round((whole_sec - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        target_point.time_from_start.sec = sec
        target_point.time_from_start.nanosec = nanosec
        msg.points.append(target_point)
        self._trajectory_pub.publish(msg)

    def _clear_target(self) -> None:
        self._target_position = None
        self._target_rotation = None
        self._target_kind = None
        self._pending_solve = False
        self._retry_attempts_remaining = 0
        self._retry_after_neutral_pending = False

    def _is_near_joint_target(self, joint_target: np.ndarray, tolerance: float) -> bool:
        return bool(np.max(np.abs(self._current_q - joint_target)) <= tolerance)

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


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Custom DLS Cartesian IK executor for the physical RX-150.'
    )
    parser.add_argument('--x', type=float, default=None, help='Target X in meters')
    parser.add_argument('--y', type=float, default=None, help='Target Y in meters')
    parser.add_argument('--z', type=float, default=None, help='Target Z in meters')
    parser.add_argument('--qx', type=float, default=None, help='Target orientation X quaternion')
    parser.add_argument('--qy', type=float, default=None, help='Target orientation Y quaternion')
    parser.add_argument('--qz', type=float, default=None, help='Target orientation Z quaternion')
    parser.add_argument('--qw', type=float, default=None, help='Target orientation W quaternion')
    parser.add_argument('--frame', type=str, default='world', help='Target frame')
    args, ros_args = parser.parse_known_args(argv)

    quaternion_fields = [args.qx, args.qy, args.qz, args.qw]
    if any(value is not None for value in quaternion_fields) and not all(
        value is not None for value in quaternion_fields
    ):
        parser.error('Provide all quaternion fields (--qx --qy --qz --qw) or none of them.')

    rclpy.init(args=ros_args)
    node = Rx150DlsIkExecutor(
        oneshot_point=_build_oneshot_point_from_args(args),
        oneshot_pose=_build_oneshot_pose_from_args(args),
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
