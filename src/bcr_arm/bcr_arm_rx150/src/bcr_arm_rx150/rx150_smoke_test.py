#!/usr/bin/env python3

import argparse
import time

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node


SMOKE_SEQUENCE = [
    ('prelift', [0.0, -0.20, 0.40, -0.20, 0.0], 2.0),
    ('lift_ready', [0.0, -0.35, 0.75, -0.40, 0.0], 2.5),
    ('home', [0.0, 0.0, 0.0, 0.0, 0.0], 2.5),
]


class Rx150SmokeTest(Node):
    def __init__(self, robot_name: str, publish_count: int):
        super().__init__('rx150_smoke_test')
        self._publish_count = max(1, publish_count)
        self._publisher = self.create_publisher(
            JointGroupCommand,
            f'/{robot_name}/commands/joint_group',
            10,
        )

    def send_pose(self, label: str, target: list[float], settle_time: float) -> None:
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [float(value) for value in target]
        for _ in range(self._publish_count):
            self._publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info("Published smoke-test step '%s': %s" % (label, target))
        time.sleep(settle_time)

    def run(self) -> int:
        for label, target, settle_time in SMOKE_SEQUENCE:
            self.send_pose(label, target, settle_time)
        return 0


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Run a small RX-150 lift-and-return smoke test.'
    )
    parser.add_argument('--robot-name', default='rx150')
    parser.add_argument('--publish-count', type=int, default=5)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Rx150SmokeTest(args.robot_name, args.publish_count)
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


if __name__ == '__main__':
    main()
