#!/usr/bin/env python3

"""Placeholder vision node -- stands in for real vision.

Speaks the exact VISION CONTRACT the orchestrator expects, so the full mission
runs end to end before real vision is connected:

  REQUEST  in : /vision/find_request   std_msgs/String   ("cup" | "goal")
  RESPONSE out: /vision/object_point   geometry_msgs/PointStamped  (base frame)

On each request it replies with a canned point (params ``cup_xyz`` / ``goal_xyz``)
in ``planning_frame``. Delete/replace this node once real vision publishes the
same response topic -- the orchestrator does not care who produces the point.
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class VisionPlaceholder(Node):
    def __init__(self):
        super().__init__('vision_placeholder')
        self.declare_parameter('planning_frame', 'rx150/base_link')
        self.declare_parameter('request_topic', '/vision/find_request')
        self.declare_parameter('response_topic', '/vision/object_point')
        # Canned answers -- tune to a reachable spot in front of the arm.
        self.declare_parameter('cup_xyz', [0.22, 0.00, 0.16])
        self.declare_parameter('goal_xyz', [0.20, 0.18, 0.16])

        self._frame = str(self.get_parameter('planning_frame').value)
        self._cup = [float(v) for v in self.get_parameter('cup_xyz').value]
        self._goal = [float(v) for v in self.get_parameter('goal_xyz').value]

        self._pub = self.create_publisher(
            PointStamped, str(self.get_parameter('response_topic').value), 10
        )
        self.create_subscription(
            String, str(self.get_parameter('request_topic').value),
            self._on_request, 10
        )
        self.get_logger().info(
            'vision_placeholder up: cup=%s goal=%s (frame %s). '
            'Replace with real vision on the same response topic.'
            % (self._cup, self._goal, self._frame)
        )

    def _on_request(self, msg: String) -> None:
        kind = msg.data.strip().lower()
        if kind == 'cup':
            xyz = self._cup
        elif kind == 'goal':
            xyz = self._goal
        else:
            self.get_logger().warning("Unknown vision request '%s'; ignoring." % msg.data)
            return
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame
        out.point.x, out.point.y, out.point.z = xyz[0], xyz[1], xyz[2]
        self._pub.publish(out)
        self.get_logger().info("Answered '%s' -> [%.3f, %.3f, %.3f]" % (kind, *xyz))


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = VisionPlaceholder()
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
