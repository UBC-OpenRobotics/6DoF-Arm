#!/usr/bin/env python3

import argparse
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DlsIkExecutor(Node):
    """Solve position-only IK with damped least squares and publish joint trajectories."""

    def __init__(self, oneshot_point: Optional[PointStamped] = None):
        super().__init__("dls_ik_executor")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("target_topic", "/cartesian_target")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "command_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("goal_time_sec", 3.0)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("damping_lambda", 0.1)
        self.declare_parameter("max_iterations", 150)
        self.declare_parameter("step_scale", 0.6)
        self.declare_parameter("max_joint_step", 0.15)
        self.declare_parameter("deadband", 0.005)
        self.declare_parameter("l1_len", 0.200)
        self.declare_parameter("l2_offset", 0.065)
        self.declare_parameter("l3_len", 0.225)
        self.declare_parameter("l4_offset", -0.065)
        self.declare_parameter("l5_len", 0.150)
        self.declare_parameter("l6_offset", 0.060)
        self.declare_parameter("l7_len", 0.125)

        self._world_frame = self.get_parameter("world_frame").value
        self._target_topic = self.get_parameter("target_topic").value
        self._joint_state_topic = self.get_parameter("joint_state_topic").value
        self._command_topic = self.get_parameter("command_topic").value
        self._goal_time_sec = float(self.get_parameter("goal_time_sec").value)
        self._position_tolerance = float(self.get_parameter("position_tolerance").value)
        self._damping = float(self.get_parameter("damping_lambda").value)
        self._max_iterations = int(self.get_parameter("max_iterations").value)
        self._step_scale = float(self.get_parameter("step_scale").value)
        self._max_joint_step = float(self.get_parameter("max_joint_step").value)
        self._deadband = float(self.get_parameter("deadband").value)

        self._joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self._joint_limits_lower = np.array([-6.28, -2.0, -6.28, -2.0, -6.28, -2.0, -6.28])
        self._joint_limits_upper = np.array([6.28, 2.0, 6.28, 2.0, 6.28, 2.0, 6.28])

        self._l1_len = float(self.get_parameter("l1_len").value)
        self._l2_offset = float(self.get_parameter("l2_offset").value)
        self._l3_len = float(self.get_parameter("l3_len").value)
        self._l4_offset = float(self.get_parameter("l4_offset").value)
        self._l5_len = float(self.get_parameter("l5_len").value)
        self._l6_offset = float(self.get_parameter("l6_offset").value)
        self._l7_len = float(self.get_parameter("l7_len").value)

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
        self._tool_offset = np.array([0.0, 0.0, self._l7_len])

        self._current_q: Optional[np.ndarray] = None
        self._busy = False

        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_callback, 10
        )
        self._trajectory_pub = self.create_publisher(JointTrajectory, self._command_topic, 10)

        self._oneshot = oneshot_point is not None
        self._oneshot_timer = None
        if oneshot_point is None:
            self._target_sub = self.create_subscription(
                PointStamped, self._target_topic, self._target_callback, 10
            )
            self.get_logger().info(
                "Listening on %s and publishing trajectories to %s"
                % (self._target_topic, self._command_topic)
            )
        else:
            self._oneshot_target = oneshot_point
            self._oneshot_timer = self.create_timer(0.25, self._oneshot_timer_callback)

    def _oneshot_timer_callback(self) -> None:
        if self._current_q is None or self._busy:
            return
        self.destroy_timer(self._oneshot_timer)
        self._oneshot_timer = None
        self._solve_and_publish(self._oneshot_target)

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(joint_name in positions for joint_name in self._joint_names):
            return
        self._current_q = np.array([positions[j] for j in self._joint_names], dtype=float)

    def _target_callback(self, msg: PointStamped) -> None:
        if self._busy:
            self.get_logger().warning("Ignoring target while previous solve is in progress.")
            return
        if self._current_q is None:
            self.get_logger().warning("No joint state received yet. Cannot solve IK.")
            return
        self._solve_and_publish(msg)

    def _solve_and_publish(self, target: PointStamped) -> None:
        frame = target.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().error(
                "Target frame '%s' is not supported. Expected '%s'." % (frame, self._world_frame)
            )
            if self._oneshot:
                rclpy.shutdown()
            return

        target_xyz = np.array([target.point.x, target.point.y, target.point.z], dtype=float)
        current_xyz, _ = self._forward_kinematics(self._current_q)
        current_error = np.linalg.norm(target_xyz - current_xyz)
        if current_error <= self._deadband:
            self.get_logger().info(
                "Target within deadband (%.4f m). Skipping motion." % current_error
            )
            if self._oneshot:
                rclpy.shutdown()
            return

        self._busy = True
        self.get_logger().info(
            "Solving DLS IK for x=%.3f y=%.3f z=%.3f" % tuple(target_xyz.tolist())
        )

        success, solution_q, final_error, iterations = self._solve_dls(target_xyz)
        if not success:
            self.get_logger().error(
                "DLS IK failed after %d iterations. Final position error: %.4f m"
                % (iterations, final_error)
            )
            self._busy = False
            if self._oneshot:
                rclpy.shutdown()
            return

        self._publish_trajectory(solution_q)
        self._current_q = solution_q
        self.get_logger().info(
            "Published joint target after %d iterations. Final error: %.4f m"
            % (iterations, final_error)
        )
        self._busy = False
        if self._oneshot:
            rclpy.shutdown()

    def _solve_dls(self, target_xyz: np.ndarray):
        q = np.array(self._current_q, dtype=float)

        for iteration in range(1, self._max_iterations + 1):
            current_xyz, jacobian = self._forward_kinematics(q, with_jacobian=True)
            error = target_xyz - current_xyz
            error_norm = float(np.linalg.norm(error))
            if error_norm <= self._position_tolerance:
                return True, q, error_norm, iteration

            jj_t = jacobian @ jacobian.T
            damping_matrix = (self._damping ** 2) * np.eye(3)
            dq = jacobian.T @ np.linalg.solve(jj_t + damping_matrix, error)
            dq *= self._step_scale
            dq = np.clip(dq, -self._max_joint_step, self._max_joint_step)

            q = np.clip(q + dq, self._joint_limits_lower, self._joint_limits_upper)

        final_xyz, _ = self._forward_kinematics(q)
        final_error = float(np.linalg.norm(target_xyz - final_xyz))
        return False, q, final_error, self._max_iterations

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

        if not with_jacobian:
            return end_effector_xyz, None

        jacobian = np.zeros((3, len(self._joint_names)))
        for index, (joint_origin, axis_world) in enumerate(zip(joint_positions, joint_axes_world)):
            jacobian[:, index] = np.cross(axis_world, end_effector_xyz - joint_origin)

        return end_effector_xyz, jacobian

    def _publish_trajectory(self, joint_positions: np.ndarray) -> None:
        msg = JointTrajectory()
        msg.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = Duration(
            sec=int(self._goal_time_sec),
            nanosec=int((self._goal_time_sec % 1.0) * 1e9),
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

        rotation = np.array(
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
            ]
        )
        return rotation


def _build_oneshot_from_args(args) -> Optional[PointStamped]:
    if args.x is None or args.y is None or args.z is None:
        return None

    point = PointStamped()
    point.header.frame_id = args.frame
    point.point.x = args.x
    point.point.y = args.y
    point.point.z = args.z
    return point


def main(argv=None):
    parser = argparse.ArgumentParser(description="DLS IK executor for BCR arm.")
    parser.add_argument("--x", type=float, default=None, help="Target X in meters")
    parser.add_argument("--y", type=float, default=None, help="Target Y in meters")
    parser.add_argument("--z", type=float, default=None, help="Target Z in meters")
    parser.add_argument("--frame", type=str, default="world", help="Target frame")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    oneshot = _build_oneshot_from_args(args)
    node = DlsIkExecutor(oneshot_point=oneshot)

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
