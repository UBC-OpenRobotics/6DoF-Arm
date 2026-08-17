#!/usr/bin/env python3

"""Placeholder sweep node -- stands in for the real RealSense sweep.

The real sweep (tuck -> rotate waist -> accumulate depth -> merge -> publish) is
blocked on the physical camera port (see TASK_realsense_sweep_pipeline.md). This
stub just latches a **canned obstacle cloud** onto ``/planning/point_cloud`` so
the planner has a map to check against and the mission can run today.

Trigger contract (matches the orchestrator):
  IN  : /sweep/start           std_msgs/Empty   (orchestrator asks for a sweep)
  OUT : /planning/point_cloud  sensor_msgs/PointCloud2  (base frame, latched)

It publishes once on startup and again on every ``/sweep/start``, then keeps
republishing (latched-style) so late subscribers still receive it. Replace with
the real sweep on the same output topic -- the orchestrator only needs a
non-empty cloud there.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty, Header


def _box_points(center, size, n_per_axis=8):
    """Dense-ish surface points of an axis-aligned box (a stand-in obstacle)."""
    cx, cy, cz = center
    sx, sy, sz = size
    xs = np.linspace(cx - sx / 2, cx + sx / 2, n_per_axis)
    ys = np.linspace(cy - sy / 2, cy + sy / 2, n_per_axis)
    zs = np.linspace(cz - sz / 2, cz + sz / 2, n_per_axis)
    grid = np.array([[x, y, z] for x in xs for y in ys for z in zs], dtype=np.float32)
    return grid


class SweepPlaceholder(Node):
    def __init__(self):
        super().__init__('sweep_placeholder')
        self.declare_parameter('planning_frame', 'rx150/base_link')
        self.declare_parameter('output_topic', '/planning/point_cloud')
        self.declare_parameter('trigger_topic', '/sweep/start')
        # One canned box obstacle in front of the arm (matches the sim world's
        # front box). Tune / add more for your test scene.
        self.declare_parameter('box_center', [0.34, 0.14, 0.12])
        self.declare_parameter('box_size', [0.06, 0.06, 0.24])
        self.declare_parameter('republish_sec', 2.0)

        self._frame = str(self.get_parameter('planning_frame').value)
        center = [float(v) for v in self.get_parameter('box_center').value]
        size = [float(v) for v in self.get_parameter('box_size').value]
        self._points = _box_points(center, size)

        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), latched
        )
        self.create_subscription(
            Empty, str(self.get_parameter('trigger_topic').value),
            self._on_trigger, 10
        )
        self._publish()  # latch one immediately
        self.create_timer(
            float(self.get_parameter('republish_sec').value), self._publish
        )
        self.get_logger().info(
            'sweep_placeholder up: canned box at %s size %s (%d pts, frame %s).'
            % (center, size, self._points.shape[0], self._frame)
        )

    def _on_trigger(self, _msg: Empty) -> None:
        self.get_logger().info('Sweep requested; publishing canned map.')
        self._publish()

    def _publish(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = self._points.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * self._points.shape[0]
        cloud.data = np.ascontiguousarray(self._points, dtype=np.float32).tobytes()
        cloud.is_dense = False
        self._pub.publish(cloud)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SweepPlaceholder()
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
