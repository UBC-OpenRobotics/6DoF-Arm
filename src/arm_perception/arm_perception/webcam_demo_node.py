"""Webcam demo node: publishes laptop webcam frames as ROS 2 Image messages.

Drop-in replacement for the RealSense camera node during development.
Publishes on the same topics so downstream nodes (YOLO, etc.) work unchanged.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class WebcamDemoNode(Node):
    def __init__(self):
        super().__init__('webcam_demo_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        color_topic = self.get_parameter('color_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self._bridge = CvBridge()
        self._cap = cv2.VideoCapture(device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open webcam (device {device_id})')
            raise RuntimeError('Cannot open webcam')

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Webcam opened: {actual_w}x{actual_h}')

        self._color_pub = self.create_publisher(Image, color_topic, 10)
        self._info_pub = self.create_publisher(CameraInfo, info_topic, 10)

        # Approximate camera intrinsics for a typical laptop webcam
        self._camera_info = CameraInfo()
        self._camera_info.width = actual_w
        self._camera_info.height = actual_h
        fx = float(actual_w)  # rough approximation
        fy = float(actual_w)
        cx = float(actual_w) / 2.0
        cy = float(actual_h) / 2.0
        self._camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._camera_info.distortion_model = 'plumb_bob'
        self._camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        period = 1.0 / fps
        self.create_timer(period, self._publish_frame)

    def _publish_frame(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Failed to read webcam frame')
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_color_optical_frame'
        self._color_pub.publish(img_msg)

        self._camera_info.header.stamp = stamp
        self._camera_info.header.frame_id = 'camera_color_optical_frame'
        self._info_pub.publish(self._camera_info)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
