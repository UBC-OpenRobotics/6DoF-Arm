"""3D localization node.

Takes 2D detections and aligned depth image, projects detection centers
into 3D points in base_link.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, Vector3
import message_filters

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from arm_interfaces.msg import DetectedObject, DetectedObjectArray


class Localization3DNode(Node):
    def __init__(self):
        super().__init__('localization_3d_node')

        self.declare_parameter('detection_topic', '/perception/detections')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw') #TODO: CHECK TOPIC
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info') #TODO: CHECK TOPIC
        self.declare_parameter('output_topic', '/perception/detections_3d')
        self.declare_parameter('marker_topic', '/perception/markers')
        self.declare_parameter('depth_scale', 0.001)  # RealSense default: mm to meters
        self.declare_parameter('target_frame', 'rx150/base_link')
        # Valid depth window.
        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('allow_latest_tf', True)
        # Push the deprojected point AWAY from the camera along the view ray
        # by this many metres, to convert a surface hit into an object centre.
        self.declare_parameter('surface_to_centre_m', 0.0)
        # Where in the bounding box to sample depth and deproject from.
        # 'centre' or 'bottom'
        self.declare_parameter('bbox_anchor', 'centre')
        self.declare_parameter('bottom_inset_frac', 0.12)

        #input topics
        det_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        output_topic = self.get_parameter('output_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self._depth_scale = self.get_parameter('depth_scale').value
        self._target_frame = self.get_parameter('target_frame').value
        self._min_depth = float(self.get_parameter('min_depth_m').value)
        self._max_depth = float(self.get_parameter('max_depth_m').value)
        self._allow_latest_tf = bool(self.get_parameter('allow_latest_tf').value)
        self._surface_to_centre = float(
            self.get_parameter('surface_to_centre_m').value)
        self._bbox_anchor = str(self.get_parameter('bbox_anchor').value).strip().lower()
        self._bottom_inset = float(self.get_parameter('bottom_inset_frac').value)
        if self._bbox_anchor not in ('centre', 'center', 'bottom'):
            self.get_logger().warning(
                "Unknown bbox_anchor '%s'; using 'centre'." % self._bbox_anchor)
            self._bbox_anchor = 'centre'
        self._depth_rejects = 0
        self._depth_accepts = 0
        self._warned_latest_tf = False
        
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Camera intrinsics (populated from CameraInfo)
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._intrinsics_received = False

        # Subscribe
        # Subscribe to camera info
        self.create_subscription(CameraInfo, info_topic, self._info_callback, 1)

        # Time-synchronized subscribers for detections + depth
        det_sub = message_filters.Subscriber(self, DetectedObjectArray, det_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)

        # synchronize detection and depth messages based on their timestamps (allowing a small time difference of slop seconds)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self._sync.registerCallback(self._sync_callback) # register the callback to be called once synchronized messages are received

        # Publishers
        self._det3d_pub = self.create_publisher(DetectedObjectArray, output_topic, 10) #publish 3d detections (rgb detection + depth)
        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, 10) #publish RViz markers for visualization

        self.get_logger().info('3D localization node started')

        # Catch the clock mismatch at startup instead of at 3am. Discovery is
        # not instant, so this runs once a few seconds in.
        self._clock_check_timer = self.create_timer(4.0, self._check_clock_agreement)

    def _check_clock_agreement(self) -> None:
        """Warn if this node's clock disagrees with whether /clock exists."""
        self._clock_check_timer.cancel()
        using_sim_time = bool(self.get_parameter('use_sim_time').value)
        clock_present = self.count_publishers('/clock') > 0

        if using_sim_time and not clock_present:
            self.get_logger().error(
                'use_sim_time is TRUE but nothing publishes /clock. This node '
                'is waiting for a clock that will never tick; it will process '
                'nothing. Start Gazebo first, or pass use_sim_time:=false.'
            )
        elif clock_present and not using_sim_time:
            self.get_logger().error(
                'A simulator is publishing /clock but this node is on the WALL '
                'clock. Image stamps are in sim time, so every stamped TF '
                'lookup will fail and EVERY DETECTION WILL BE DROPPED -- '
                'silently. Relaunch with use_sim_time:=true.'
            )
        else:
            self.get_logger().info(
                'Clock check OK: use_sim_time=%s, /clock %s.'
                % (using_sim_time, 'present' if clock_present else 'absent')
            )

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

    def _surface_to_axis(self, point, transform):
        if not self._surface_to_centre:
            return point
        cam = transform.transform.translation
        dx, dy = point.x - cam.x, point.y - cam.y
        horizontal = float(np.hypot(dx, dy))
        if horizontal < 1e-6:
            # Camera directly overhead: no horizontal bearing to push along, and
            # in that view the near face is the top, not the side.
            return point
        scale = self._surface_to_centre / horizontal
        point.x += dx * scale
        point.y += dy * scale
        return point

    def _lookup_transform(self, header):
        """camera frame -> target frame, preferring the transform at capture time.

        Returns None if no usable transform exists, in which case the caller
        skips the detection rather than placing it wrongly.
        """
        try:
            return self._tf_buffer.lookup_transform(
                self._target_frame, header.frame_id, header.stamp
            )
        except TransformException as exact_error:
            if not self._allow_latest_tf:
                self.get_logger().warning(
                    f"TF lookup failed for '{header.frame_id}' -> "
                    f"'{self._target_frame}': {exact_error}. Skipping this detection."
                )
                return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, header.frame_id, RclpyTime()
            )
        except TransformException as latest_error:
            self.get_logger().warning(
                f"TF lookup failed for '{header.frame_id}' -> "
                f"'{self._target_frame}' at both the capture stamp and latest: "
                f"{latest_error}. Skipping this detection."
            )
            return None

        if not self._warned_latest_tf:
            self._warned_latest_tf = True
            self.get_logger().warning(
                f"No transform at the capture stamp for '{header.frame_id}' -> "
                f"'{self._target_frame}'; using the latest available instead. "
                'Accurate while the arm is stationary (as it is when the sweep '
                'samples), but detections taken mid-motion may be misplaced. '
                'Logged once.'
            )
        return transform

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
        output.header = det_msg.header #already synchronized with depth_msg
        markers = MarkerArray()

        # Loop through detections (det) and compute 3D position for each
        for i, det in enumerate(det_msg.objects):
            # Where in the box to sample. See bbox_anchor.
            cx_px = (det.bbox_2d[0] + det.bbox_2d[2]) // 2
            if self._bbox_anchor == 'bottom':
                box_h = det.bbox_2d[3] - det.bbox_2d[1]
                cy_px = int(det.bbox_2d[3] - self._bottom_inset * box_h)
            else:
                cy_px = (det.bbox_2d[1] + det.bbox_2d[3]) // 2

            # Clamp to image bounds
            cx_px = max(0, min(cx_px, depth_msg.width - 1))
            cy_px = max(0, min(cy_px, depth_msg.height - 1))

            # Read depth at center (take the median of a small patch for robustness)
            patch_size = 5 # in pixels
            y_min = max(0, cy_px - patch_size)
            y_max = min(depth_msg.height, cy_px + patch_size + 1)
            x_min = max(0, cx_px - patch_size)
            x_max = min(depth_msg.width, cx_px + patch_size + 1)

            depth_patch = depth_array[y_min:y_max, x_min:x_max]
            valid_depths = depth_patch[depth_patch > 0]

            # check if we have any valid depth measurements in the patch
            if len(valid_depths) == 0:
                continue

            depth_val = float(np.median(valid_depths))

            # Convert to meters, depth_val is in millimeters if dtype is uint16, in meters if dtype is float32
            if dtype == np.uint16:
                z = depth_val * self._depth_scale
            else:
                z = depth_val

            if z < self._min_depth or z > self._max_depth:

                self._depth_rejects += 1
                total = self._depth_rejects + self._depth_accepts
                if total >= 50:
                    pct = 100.0 * self._depth_rejects / total
                    if pct > 50.0:
                        self.get_logger().warning(
                            '%.0f%% of detections (%d/%d) rejected by the depth '
                            'gate [%.3f, %.3f] m. This depth was %.3f m. If most '
                            'are rejected the gate is probably wrong for this '
                            'camera, not the detector.'
                            % (pct, self._depth_rejects, total,
                               self._min_depth, self._max_depth, z)
                        )
                    self._depth_rejects = 0
                    self._depth_accepts = 0
                continue
            self._depth_accepts += 1

            # Deproject to 3D using pinhole model
            x = (cx_px - self._cx) * z / self._fx
            y = (cy_px - self._cy) * z / self._fy


            # Build enriched detection with 3D position and dimensions

            det_3d = DetectedObject()
            det_3d.header.stamp = det_msg.header.stamp
            det_3d.header.frame_id = self._target_frame
            det_3d.class_name = det.class_name
            det_3d.confidence = det.confidence
            det_3d.bbox_2d = det.bbox_2d

            #det_3d.position_3d = Point(x=x, y=y, z=z)  -> still in camera frame

            #transform position point to base_link frame. #TODO: make this a helper function?
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header = det_msg.header
            point_in_camera_frame.point = Point(x=x, y=y, z=z)

            transform = self._lookup_transform(depth_msg.header)
            if transform is None:
                continue
            # Apply the transform to the 3D point
            point_in_base_link_frame = do_transform_point(point_in_camera_frame, transform)
            det_3d.position_3d = self._surface_to_axis(
                point_in_base_link_frame.point, transform)


            # Estimate dimensions from bbox + depth
            bbox_w = det.bbox_2d[2] - det.bbox_2d[0]
            bbox_h = det.bbox_2d[3] - det.bbox_2d[1]
            dim_x = bbox_w * z / self._fx
            dim_y = bbox_h * z / self._fy
            det_3d.dimensions_3d = Vector3(x=dim_x, y=dim_y, z=0.05) #Vector is still in camera frame TODO: does motion needs this?

            output.objects.append(det_3d)

            # Create RViz marker
            marker = Marker()
            marker.header = det_msg.header
            marker.ns = 'detections'
            marker.id = i

            #marker.type = Marker.CUBE
            marker.type = marker.TEXT_VIEW_FACING #TODO: configure detection labels
            marker.action = Marker.ADD
            marker.text = det.class_name
            marker.pose.position = Point(x=x, y=y, z=z)
            #marker.scale = Vector3(x=dim_x, y=dim_y, z=0.05)
            marker.scale.z = 0.1 # 10 cm letters?
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
