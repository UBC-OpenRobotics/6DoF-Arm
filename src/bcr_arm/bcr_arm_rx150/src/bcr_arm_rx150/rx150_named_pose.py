#!/usr/bin/env python3

import argparse

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node


POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0],
    'neutral_carry': [0.0, -0.35, 0.75, -0.40, 0.0],
    'neutral_carry_yaw_left': [0.30, -0.35, 0.75, -0.40, 0.0],
    'neutral_carry_yaw_right': [-0.30, -0.35, 0.75, -0.40, 0.0],
    'lift_ready': [0.0, -0.50, 1.00, -0.50, 0.0],
}


class Rx150NamedPose(Node):
    def __init__(self, robot_name: str, pose_name: str, publish_count: int):
        super().__init__('rx150_named_pose')
        self._pose_name = pose_name
        self._publish_count = max(1, publish_count)
        self._publisher = self.create_publisher(
            JointGroupCommand,
            f'/{robot_name}/commands/joint_group',
            10,
        )

    def run(self) -> int:
        target = POSES.get(self._pose_name)
        if target is None:
            self.get_logger().error(
                "Unknown pose '%s'. Available poses: %s"
                % (self._pose_name, ', '.join(sorted(POSES.keys())))
            )
            return 1

        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = [float(value) for value in target]
        for _ in range(self._publish_count):
            self._publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            "Published pose '%s' as arm group command: %s" % (self._pose_name, target)
        )
        return 0


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Publish a named RX-150 joint pose.')
    parser.add_argument('--robot-name', default='rx150')
    parser.add_argument('--pose', default='neutral_carry')
    parser.add_argument('--publish-count', type=int, default=5)
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Rx150NamedPose(args.robot_name, args.pose, args.publish_count)
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
