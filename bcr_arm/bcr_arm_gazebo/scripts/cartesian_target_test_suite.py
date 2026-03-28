#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node


@dataclass(frozen=True)
class TargetCase:
    case_id: str
    label: str
    x_pos: float
    y_pos: float
    z_pos: float
    quaternion: Optional[tuple[float, float, float, float]] = None


class CartesianTargetTestSuite(Node):
    """Publish a staged Cartesian target scenario and wait for user input between steps."""

    def __init__(self):
        super().__init__("cartesian_target_test_suite")

        self.declare_parameter("target_topic", "/cartesian_target")
        self.declare_parameter("target_pose_topic", "/cartesian_target_pose")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_count", 3)

        self._target_topic = self.get_parameter("target_topic").value
        self._target_pose_topic = self.get_parameter("target_pose_topic").value
        self._world_frame = self.get_parameter("world_frame").value
        self._publish_count = int(self.get_parameter("publish_count").value)

        self._point_publisher = self.create_publisher(PointStamped, self._target_topic, 10)
        self._pose_publisher = self.create_publisher(PoseStamped, self._target_pose_topic, 10)

    def publish_case(self, target_case: TargetCase) -> None:
        if target_case.quaternion is None:
            msg = PointStamped()
            msg.header.frame_id = self._world_frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(target_case.x_pos)
            msg.point.y = float(target_case.y_pos)
            msg.point.z = float(target_case.z_pos)
            for _ in range(max(1, self._publish_count)):
                self._point_publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)
            target_kind = "point"
        else:
            msg = PoseStamped()
            msg.header.frame_id = self._world_frame
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(target_case.x_pos)
            msg.pose.position.y = float(target_case.y_pos)
            msg.pose.position.z = float(target_case.z_pos)
            msg.pose.orientation.x = float(target_case.quaternion[0])
            msg.pose.orientation.y = float(target_case.quaternion[1])
            msg.pose.orientation.z = float(target_case.quaternion[2])
            msg.pose.orientation.w = float(target_case.quaternion[3])
            for _ in range(max(1, self._publish_count)):
                self._pose_publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)
            target_kind = "pose"

        self.get_logger().info(
            "Published %s (%s, %s): x=%.3f y=%.3f z=%.3f"
            % (
                target_case.case_id,
                target_case.label,
                target_kind,
                target_case.x_pos,
                target_case.y_pos,
                target_case.z_pos,
            )
        )


def _scenario_cases() -> list[TargetCase]:
    top_down_quaternion = (1.0, 0.0, 0.0, 0.0)
    horizontal_quaternion = (0.0, -0.70710678, 0.0, 0.70710678)
    return [
        TargetCase(
            "1-1",
            "top_pick_initial_cup",
            -0.08,
            0.38,
            0.18,
            top_down_quaternion,
        ),
        TargetCase(
            "1-2",
            "top_lift_initial_cup",
            -0.08,
            0.38,
            0.30,
            top_down_quaternion,
        ),
        TargetCase(
            "1-3",
            "top_translate_to_staging_xy",
            -0.22,
            0.37,
            0.30,
            top_down_quaternion,
        ),
        TargetCase(
            "1-4",
            "top_lower_to_horizontal_pick_height",
            -0.22,
            0.37,
            0.24,
            top_down_quaternion,
        ),
        TargetCase(
            "1-5",
            "side_pick_staged_cup_horizontal",
            -0.22,
            0.37,
            0.24,
            horizontal_quaternion,
        ),
        TargetCase(
            "1-6",
            "side_lift_staged_cup_horizontal",
            -0.26,
            0.42,
            0.32,
        ),
        TargetCase(
            "1-7",
            "side_transfer_to_final_destination",
            -0.04,
            0.56,
            0.32,
        ),
    ]


def _build_groups() -> list[tuple[str, list[TargetCase]]]:
    return [("Cup Transfer Scenario", _scenario_cases())]


def _interactive_prompt() -> str:
    while True:
        response = input("Press Enter for next target, 'r' to resend, or 'q' to quit: ").strip().lower()
        if response in {"", "r", "q"}:
            return response
        print("Use Enter, 'r', or 'q'.")


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Publish a staged cup-handling target sequence for IK testing."
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = CartesianTargetTestSuite()

    groups = _build_groups()
    total_cases = sum(len(group_cases) for _, group_cases in groups)
    case_counter = 0

    try:
        for group_name, group_cases in groups:
            print(f"\n=== {group_name} ===")
            for target_case in group_cases:
                case_counter += 1
                target_kind = "pose" if target_case.quaternion is not None else "point"
                print(
                    "[%d/%d] %s %s (%s) -> x=%.3f y=%.3f z=%.3f"
                    % (
                        case_counter,
                        total_cases,
                        target_case.case_id,
                        target_case.label,
                        target_kind,
                        target_case.x_pos,
                        target_case.y_pos,
                        target_case.z_pos,
                    )
                )
                node.publish_case(target_case)

                response = _interactive_prompt()
                while response == "r":
                    node.publish_case(target_case)
                    response = _interactive_prompt()
                if response == "q":
                    print("Stopping test suite.")
                    return
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
