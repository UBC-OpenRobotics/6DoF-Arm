"""RealSense camera health-check and convenience wrapper.

The upstream realsense2_camera package provides the actual driver node.
This node monitors camera topics and provides a health-check service.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('timeout_sec', 5.0)

        color_topic = self.get_parameter('color_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self._color_received = False
        self._depth_received = False
        self._camera_info = None

        self.create_subscription(Image, color_topic, self._color_callback, 1)
        self.create_subscription(Image, depth_topic, self._depth_callback, 1)
        self.create_subscription(CameraInfo, info_topic, self._info_callback, 1)

        timeout = self.get_parameter('timeout_sec').value
        self.create_timer(timeout, self._check_health)

        self.get_logger().info('RealSense health monitor started')

    def _color_callback(self, msg: Image):
        if not self._color_received:
            self.get_logger().info(
                f'Color stream active: {msg.width}x{msg.height} ({msg.encoding})'
            )
            self._color_received = True

    def _depth_callback(self, msg: Image):
        if not self._depth_received:
            self.get_logger().info(
                f'Depth stream active: {msg.width}x{msg.height} ({msg.encoding})'
            )
            self._depth_received = True

    def _info_callback(self, msg: CameraInfo):
        if self._camera_info is None:
            self._camera_info = msg
            self.get_logger().info(
                f'Camera info received: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}'
            )

    def _check_health(self):
        if not self._color_received:
            self.get_logger().warn('No color frames received - is camera connected?')
        if not self._depth_received:
            self.get_logger().warn('No depth frames received - is camera connected?')


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
