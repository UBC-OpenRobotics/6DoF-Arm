#!/usr/bin/env python3

import argparse
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive


class CartesianTargetExecutor(Node):
    """Accept Cartesian targets and delegate planning/execution to MoveIt's move_group."""

    def __init__(self, oneshot_point: Optional[PointStamped] = None):
        super().__init__("cartesian_target_executor")

        self.declare_parameter("group_name", "bcr_arm")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("end_effector_link", "flange")
        self.declare_parameter("target_topic", "/cartesian_target")
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_x", 0.0)
        self.declare_parameter("orientation_y", 0.0)
        self.declare_parameter("orientation_z", 0.0)
        self.declare_parameter("orientation_w", 1.0)
        self.declare_parameter("orientation_tol_x", 0.1)
        self.declare_parameter("orientation_tol_y", 0.1)
        self.declare_parameter("orientation_tol_z", 0.1)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 5)
        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)
        self.declare_parameter("execute", True)
        self.declare_parameter("replan", True)

        self._group_name = self.get_parameter("group_name").get_parameter_value().string_value
        self._world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self._ee_link = self.get_parameter("end_effector_link").get_parameter_value().string_value
        self._target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        self._position_tolerance = (
            self.get_parameter("position_tolerance").get_parameter_value().double_value
        )
        self._planning_time = self.get_parameter("planning_time").get_parameter_value().double_value
        self._planning_attempts = (
            self.get_parameter("planning_attempts").get_parameter_value().integer_value
        )
        self._velocity_scaling = (
            self.get_parameter("velocity_scaling").get_parameter_value().double_value
        )
        self._acceleration_scaling = (
            self.get_parameter("acceleration_scaling").get_parameter_value().double_value
        )
        self._execute = self.get_parameter("execute").get_parameter_value().bool_value
        self._replan = self.get_parameter("replan").get_parameter_value().bool_value

        self._orientation_pose = Pose().orientation
        self._orientation_pose.x = self.get_parameter("orientation_x").get_parameter_value().double_value
        self._orientation_pose.y = self.get_parameter("orientation_y").get_parameter_value().double_value
        self._orientation_pose.z = self.get_parameter("orientation_z").get_parameter_value().double_value
        self._orientation_pose.w = self.get_parameter("orientation_w").get_parameter_value().double_value

        self._orientation_tol_x = (
            self.get_parameter("orientation_tol_x").get_parameter_value().double_value
        )
        self._orientation_tol_y = (
            self.get_parameter("orientation_tol_y").get_parameter_value().double_value
        )
        self._orientation_tol_z = (
            self.get_parameter("orientation_tol_z").get_parameter_value().double_value
        )

        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self._busy = False

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action server '/move_action' not available.")

        self.get_logger().info(
            "Connected to /move_action (group=%s, ee_link=%s, execute=%s)"
            % (self._group_name, self._ee_link, self._execute)
        )

        self._oneshot = oneshot_point is not None
        if oneshot_point is None:
            self._sub = self.create_subscription(
                PointStamped,
                self._target_topic,
                self._target_callback,
                10,
            )
            self.get_logger().info("Listening on %s (geometry_msgs/PointStamped)" % self._target_topic)
        else:
            self._send_target(oneshot_point)

    def _target_callback(self, msg: PointStamped) -> None:
        if self._busy:
            self.get_logger().warning("Ignoring target while previous goal is in progress.")
            return
        self._send_target(msg)

    def _send_target(self, target: PointStamped) -> None:
        frame = target.header.frame_id if target.header.frame_id else self._world_frame

        goal = MoveGroup.Goal()
        goal.request.group_name = self._group_name
        goal.request.num_planning_attempts = int(self._planning_attempts)
        goal.request.allowed_planning_time = float(self._planning_time)
        goal.request.max_velocity_scaling_factor = float(self._velocity_scaling)
        goal.request.max_acceleration_scaling_factor = float(self._acceleration_scaling)

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame
        pos_constraint.link_name = self._ee_link
        pos_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(self._position_tolerance, 1e-4)]
        pos_constraint.constraint_region.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position.x = float(target.point.x)
        sphere_pose.position.y = float(target.point.y)
        sphere_pose.position.z = float(target.point.z)
        sphere_pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(sphere_pose)

        constraints.position_constraints.append(pos_constraint)

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = frame
        ori_constraint.link_name = self._ee_link
        ori_constraint.orientation = self._orientation_pose
        ori_constraint.absolute_x_axis_tolerance = float(self._orientation_tol_x)
        ori_constraint.absolute_y_axis_tolerance = float(self._orientation_tol_y)
        ori_constraint.absolute_z_axis_tolerance = float(self._orientation_tol_z)
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints = [constraints]

        goal.planning_options.plan_only = not self._execute
        goal.planning_options.replan = self._replan
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.1

        self._busy = True
        self.get_logger().info(
            "Sending target x=%.3f y=%.3f z=%.3f frame=%s"
            % (target.point.x, target.point.y, target.point.z, frame)
        )

        send_future = self._move_group_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._busy = False
            self.get_logger().error("MoveGroup goal rejected.")
            if self._oneshot:
                rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        self._busy = False
        wrapped_result = future.result()
        result = wrapped_result.result

        error_code = result.error_code.val
        if error_code == 1:
            self.get_logger().info("Target completed successfully.")
        else:
            self.get_logger().error(f"Target failed. MoveIt error code: {error_code}")

        if self._oneshot:
            rclpy.shutdown()


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
    parser = argparse.ArgumentParser(description="Cartesian target executor for BCR arm.")
    parser.add_argument("--x", type=float, default=None, help="Target X in meters")
    parser.add_argument("--y", type=float, default=None, help="Target Y in meters")
    parser.add_argument("--z", type=float, default=None, help="Target Z in meters")
    parser.add_argument("--frame", type=str, default="world", help="Target frame")
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)

    oneshot = _build_oneshot_from_args(args)

    node = CartesianTargetExecutor(oneshot_point=oneshot)

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
