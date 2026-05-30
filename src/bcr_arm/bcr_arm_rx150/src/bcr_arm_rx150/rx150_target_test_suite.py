#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
import time
from typing import Optional

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class TargetCase:
    case_id: str
    label: str
    x_pos: float
    y_pos: float
    z_pos: float
    quaternion: Optional[tuple[float, float, float, float]] = None
    note: str = ''


def _suite_catalog() -> dict[str, list[TargetCase]]:
    forward_quaternion = (0.0, 0.0, 0.0, 1.0)
    return {
        'smoke': [
            TargetCase('S1', 'front_low_point', 0.18, 0.00, 0.12, note='Small front reach'),
            TargetCase('S2', 'front_mid_point', 0.22, 0.00, 0.16, note='Known-good point'),
            TargetCase('S3', 'front_high_point', 0.22, 0.00, 0.26, note='Higher vertical reach'),
        ],
        'lateral': [
            TargetCase('L1', 'center_point', 0.22, 0.00, 0.16, note='Center reference'),
            TargetCase('L2', 'left_point', 0.18, 0.08, 0.14, note='Moderate left offset'),
            TargetCase('L3', 'right_point', 0.18, -0.08, 0.14, note='Moderate right offset'),
        ],
        'pose': [
            TargetCase('P1', 'forward_pose', 0.20, 0.00, 0.16, forward_quaternion, 'Exact pose test'),
            TargetCase('P2', 'forward_pose_high', 0.22, 0.00, 0.20, forward_quaternion, 'Higher exact pose'),
        ],
        'full': [
            TargetCase('F1', 'front_low_point', 0.18, 0.00, 0.12, note='Small front reach'),
            TargetCase('F2', 'front_mid_point', 0.22, 0.00, 0.16, note='Known-good point'),
            TargetCase('F3', 'front_high_point', 0.22, 0.00, 0.26, note='Higher vertical reach'),
            TargetCase('F4', 'left_point', 0.18, 0.08, 0.14, note='Moderate left offset'),
            TargetCase('F5', 'right_point', 0.18, -0.08, 0.14, note='Moderate right offset'),
            TargetCase('F6', 'forward_pose', 0.20, 0.00, 0.16, forward_quaternion, 'Exact pose test'),
        ],
    }


class Rx150TargetTestSuite(Node):
    """Publish staged Cartesian targets for the physical RX-150."""

    def __init__(self) -> None:
        super().__init__('rx150_target_test_suite')
        self.declare_parameter('target_topic', '/cartesian_target')
        self.declare_parameter('target_pose_topic', '/cartesian_target_pose')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('publish_count', 3)

        self._target_topic = self.get_parameter('target_topic').value
        self._target_pose_topic = self.get_parameter('target_pose_topic').value
        self._world_frame = self.get_parameter('world_frame').value
        self._publish_count = int(self.get_parameter('publish_count').value)

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


def _interactive_prompt() -> str:
    while True:
        response = input("Press Enter for next target, 'r' to resend, 's' to skip, or 'q' to quit: ")
        response = response.strip().lower()
        if response in {'', 'r', 's', 'q'}:
            return response
        print("Use Enter, 'r', 's', or 'q'.")


def _print_case(index: int, total: int, target_case: TargetCase, frame: str) -> None:
    target_kind = 'pose' if target_case.quaternion is not None else 'point'
    print(
        "[%d/%d] %s %s (%s, frame=%s) -> x=%.3f y=%.3f z=%.3f"
        % (
            index,
            total,
            target_case.case_id,
            target_case.label,
            target_kind,
            frame,
            target_case.x_pos,
            target_case.y_pos,
            target_case.z_pos,
        )
    )
    if target_case.note:
        print(f"note: {target_case.note}")


def main(argv=None) -> None:
    suite_catalog = _suite_catalog()
    parser = argparse.ArgumentParser(
        description='Publish a staged Cartesian target sequence for RX-150 DLS testing.'
    )
    parser.add_argument(
        '--suite',
        choices=sorted(suite_catalog.keys()),
        default='smoke',
        help='Named target suite to run',
    )
    parser.add_argument(
        '--auto',
        action='store_true',
        help='Advance automatically without waiting for keyboard input',
    )
    parser.add_argument(
        '--pause-sec',
        type=float,
        default=4.0,
        help='Pause between targets when running with --auto',
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Rx150TargetTestSuite()
    cases = suite_catalog[args.suite]

    print(f"Running RX-150 suite '{args.suite}' with {len(cases)} targets.")
    print(f"Target frame: {node._world_frame}")
    if args.auto:
        print(f"Auto-advance enabled with {args.pause_sec:.1f}s pauses.")
    else:
        print("Interactive mode enabled.")

    try:
        for index, target_case in enumerate(cases, start=1):
            _print_case(index, len(cases), target_case, node._world_frame)
            node.publish_case(target_case)

            if args.auto:
                time.sleep(max(0.0, args.pause_sec))
                continue

            response = _interactive_prompt()
            while response == 'r':
                node.publish_case(target_case)
                response = _interactive_prompt()
            if response == 's':
                continue
            if response == 'q':
                print('Stopping RX-150 target test suite.')
                return
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
