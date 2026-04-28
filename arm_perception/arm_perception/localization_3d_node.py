"""3D localization node.

Takes 2D detections and aligned depth image, projects detection centers
into 3D points in the camera frame using the pinhole camera model.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
import message_filters

from arm_interfaces.msg import DetectedObject, DetectedObjectArray


class Localization3DNode(Node):
    def __init__(self):
        super().__init__('localization_3d_node')

        self.declare_parameter('detection_topic', '/perception/detections')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_topic', '/perception/detections_3d')
        self.declare_parameter('marker_topic', '/perception/markers')
        self.declare_parameter('depth_scale', 0.001)  # RealSense default: mm to meters

        det_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        output_topic = self.get_parameter('output_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self._depth_scale = self.get_parameter('depth_scale').value

        # Camera intrinsics (populated from CameraInfo)
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._intrinsics_received = False

        # Subscribe to camera info
        self.create_subscription(CameraInfo, info_topic, self._info_callback, 1)

        # Time-synchronized subscribers for detections + depth
        det_sub = message_filters.Subscriber(self, DetectedObjectArray, det_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self._sync.registerCallback(self._sync_callback)

        # Publishers
        self._det3d_pub = self.create_publisher(DetectedObjectArray, output_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.get_logger().info('3D localization node started')

    def _info_callback(self, msg: CameraInfo):
        if not self._intrinsics_received:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]
            self._intrinsics_received = True
            self.get_logger().info(
                f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx:.1f} cy={self._cy:.1f}'
            )

    def _sync_callback(self, det_msg: DetectedObjectArray, depth_msg: Image):
        if not self._intrinsics_received:
            return

        # Convert depth image to numpy array
        if depth_msg.encoding == '16UC1':
            dtype = np.uint16
        elif depth_msg.encoding == '32FC1':
            dtype = np.float32
        else:
            self.get_logger().warn(f'Unexpected depth encoding: {depth_msg.encoding}')
            return

        depth_array = np.frombuffer(depth_msg.data, dtype=dtype).reshape(
            depth_msg.height, depth_msg.width
        )

        # Project each detection to 3D
        output = DetectedObjectArray()
        output.header = det_msg.header
        markers = MarkerArray()

        for i, det in enumerate(det_msg.objects):
            # Center of bounding box
            cx_px = (det.bbox_2d[0] + det.bbox_2d[2]) // 2
            cy_px = (det.bbox_2d[1] + det.bbox_2d[3]) // 2

            # Clamp to image bounds
            cx_px = max(0, min(cx_px, depth_msg.width - 1))
            cy_px = max(0, min(cy_px, depth_msg.height - 1))

            # Read depth at center (average a small patch for robustness)
            patch_size = 5
            y_min = max(0, cy_px - patch_size)
            y_max = min(depth_msg.height, cy_px + patch_size + 1)
            x_min = max(0, cx_px - patch_size)
            x_max = min(depth_msg.width, cx_px + patch_size + 1)

            depth_patch = depth_array[y_min:y_max, x_min:x_max]
            valid_depths = depth_patch[depth_patch > 0]

            if len(valid_depths) == 0:
                continue

            depth_val = float(np.median(valid_depths))

            # Convert to meters
            if dtype == np.uint16:
                z = depth_val * self._depth_scale
            else:
                z = depth_val

            if z <= 0.1 or z > 3.0:  # ignore invalid depths
                continue

            # Deproject to 3D using pinhole model
            x = (cx_px - self._cx) * z / self._fx
            y = (cy_px - self._cy) * z / self._fy

            # Build enriched detection
            det_3d = DetectedObject()
            det_3d.header = det.header
            det_3d.class_name = det.class_name
            det_3d.confidence = det.confidence
            det_3d.bbox_2d = det.bbox_2d
            det_3d.position_3d = Point(x=x, y=y, z=z)

            # Estimate dimensions from bbox + depth
            bbox_w = det.bbox_2d[2] - det.bbox_2d[0]
            bbox_h = det.bbox_2d[3] - det.bbox_2d[1]
            dim_x = bbox_w * z / self._fx
            dim_y = bbox_h * z / self._fy
            det_3d.dimensions_3d = Vector3(x=dim_x, y=dim_y, z=0.05)

            output.objects.append(det_3d)

            # Create RViz marker
            marker = Marker()
            marker.header = det_msg.header
            marker.ns = 'detections'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=x, y=y, z=z)
            marker.scale = Vector3(x=dim_x, y=dim_y, z=0.05)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime.sec = 1
            markers.markers.append(marker)

        self._det3d_pub.publish(output)
        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Localization3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
