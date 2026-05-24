#!/usr/bin/env python3

import argparse

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class SetupArmPose(Node):
    """Send a one-shot named joint pose to the joint trajectory controller."""

    def __init__(self, pose_name: str, duration_sec: float):
        super().__init__("setup_arm_pose")
        self._pose_name = pose_name
        self._duration_sec = duration_sec
        self._joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self._poses = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "neutral_carry": [0.0, -1.10, 0.0, 0.90, 0.0, -1.37, 0.0],
            "neutral_carry_yaw_left": [0.30, -1.10, 0.0, 0.90, 0.0, -1.37, 0.0],
            "neutral_carry_yaw_right": [-0.30, -1.10, 0.0, 0.90, 0.0, -1.37, 0.0],
        }
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def run(self) -> int:
        target = self._poses.get(self._pose_name)
        if target is None:
            self.get_logger().error(
                "Unknown pose '%s'. Available poses: %s"
                % (self._pose_name, ", ".join(sorted(self._poses.keys())))
            )
            return 1

        self.get_logger().info(
            "Waiting for trajectory action server to send pose '%s'" % self._pose_name
        )
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Trajectory action server is not available. Is joint_trajectory_controller running?"
            )
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in target]
        point.time_from_start = Duration(
            sec=int(self._duration_sec),
            nanosec=int((self._duration_sec % 1.0) * 1e9),
        )
        goal.trajectory.points = [point]

        self.get_logger().info(
            "Sending setup pose '%s' over %.2fs: %s"
            % (self._pose_name, self._duration_sec, target)
        )
        goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Setup pose goal was rejected.")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_msg = result_future.result()
        if result_msg is None:
            self.get_logger().error("No result returned from trajectory action server.")
            return 1

        result = result_msg.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "Setup pose execution failed with error code %s" % result.error_code
            )
            return 1

        self.get_logger().info("Setup pose reached successfully.")
        return 0


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Send a named setup pose to the BCR arm.")
    parser.add_argument(
        "--pose",
        type=str,
        default="neutral_carry",
        help="Named pose to send: home, neutral_carry, neutral_carry_yaw_left, neutral_carry_yaw_right",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Time in seconds to reach the pose",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = SetupArmPose(args.pose, args.duration)
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
