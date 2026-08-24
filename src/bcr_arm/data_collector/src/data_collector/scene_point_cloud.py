#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


class ScenePointCloudRelay(Node):
    """Relay a Gazebo depth-camera point cloud into the planner frame."""

    def __init__(self) -> None:
        super().__init__('scene_point_cloud')

        self.declare_parameter('input_topic', '/camera_sensor/points')
        self.declare_parameter('output_topic', '/planning/point_cloud')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', '')
        self.declare_parameter('camera_x', 0.28)
        self.declare_parameter('camera_y', 0.34)
        self.declare_parameter('camera_z', 0.28)
        self.declare_parameter('camera_roll', 3.8)
        self.declare_parameter('camera_pitch', 0.0)
        self.declare_parameter('camera_yaw', 3.14)
        self.declare_parameter('broadcast_static_tf', True)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('stall_timeout_sec', 2.0)
        # Fall back to the newest transform when the one matching the cloud's
        # own stamp is unavailable. OFF: a wrongly placed obstacle is worse than
        # a missing one, because it makes the planner refuse clear paths. Turn
        # it on only to keep a stack limping while a clock mismatch is fixed.
        self.declare_parameter('allow_latest_tf', False)
        # How long to wait for the transform matching a cloud's stamp. A cloud
        # routinely arrives a millisecond or two BEFORE the TF sample covering
        # it -- the camera and the joint publisher are not synchronised, so the
        # newest transform can be marginally older than the newest image, and
        # tf2 refuses to extrapolate forward. Without a short wait every cloud is
        # dropped for the sake of a millisecond. Small enough not to stack up
        # behind a 5 Hz camera.
        self.declare_parameter('tf_timeout_sec', 0.15)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._source_frame = str(self.get_parameter('source_frame').value)
        self._translation = np.array(
            [
                float(self.get_parameter('camera_x').value),
                float(self.get_parameter('camera_y').value),
                float(self.get_parameter('camera_z').value),
            ],
            dtype=float,
        )
        self._camera_roll = float(self.get_parameter('camera_roll').value)
        self._camera_pitch = float(self.get_parameter('camera_pitch').value)
        self._camera_yaw = float(self.get_parameter('camera_yaw').value)
        self._broadcast_static_tf = bool(
            self.get_parameter('broadcast_static_tf').value
        )
        self._max_range = max(0.0, float(self.get_parameter('max_range').value))
        self._stall_timeout_sec = max(0.0, float(self.get_parameter('stall_timeout_sec').value))
        self._allow_latest_tf = bool(self.get_parameter('allow_latest_tf').value)
        self._tf_timeout = RclpyDuration(
            seconds=max(0.0, float(self.get_parameter('tf_timeout_sec').value)))
        self._tf_miss_count = 0

        self._rotation = self._euler_to_matrix(
            self._camera_roll,
            self._camera_pitch,
            self._camera_yaw,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cloud_cb_group = MutuallyExclusiveCallbackGroup()
        self._publisher = self.create_publisher(PointCloud2, self._output_topic, 10)

        depth1 = QoSProfile(
            depth=1,
            reliability=qos_profile_sensor_data.reliability,
            durability=qos_profile_sensor_data.durability,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._point_cloud_cb,
            depth1,
            callback_group=self._cloud_cb_group,
        )

        self._tf_broadcaster: Optional[StaticTransformBroadcaster] = None
        if self._broadcast_static_tf:
            self._tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        self._cb_count = 0
        self._pub_count = 0
        self._first_published = False
        self._stalled = False
        self._last_publish_monotonic: Optional[float] = None

        # Watchdog: only speaks up when publishing stalls or recovers
        if self._stall_timeout_sec > 0.0:
            self.create_timer(self._stall_timeout_sec / 2.0, self._watchdog_cb)

        self.get_logger().info(
            'Relaying Gazebo scene point cloud from %s to %s in frame %s'
            % (self._input_topic, self._output_topic, self._target_frame)
        )

    def _point_cloud_cb(self, msg: PointCloud2) -> None:
        self._cb_count += 1
        try:
            points = point_cloud2.read_points_numpy(
                msg,
                field_names=['x', 'y', 'z'],
                skip_nans=True,
            )
            if points.dtype.names:
                points = np.column_stack(
                    [points['x'], points['y'], points['z']]
                ).astype(np.float32)
            else:
                points = np.asarray(points, dtype=np.float32)
            if points.ndim == 1:
                points = points.reshape(-1, 3)
        except Exception as exc:
            self.get_logger().error(
                'read_points_numpy failed (cb=%d): %s' % (self._cb_count, exc)
            )
            return

        if points.size == 0:
            return

        if self._max_range > 0.0:
            norms = np.linalg.norm(points, axis=1)
            points = points[norms <= self._max_range]
            if points.size == 0:
                return

        source_frame = msg.header.frame_id or self._source_frame
        if source_frame:
            transform = self._lookup_transform(source_frame, msg.header.stamp)
            if transform is None:
                return
            transformed_points = self._transform_points(points, transform)
        else:
            transformed_points = (points @ self._rotation.T) + self._translation

        cloud_arr = np.ascontiguousarray(transformed_points, dtype=np.float32)
        cloud = PointCloud2()
        # Carry the CAPTURE stamp through, so downstream can tell when this
        # geometry was true rather than seeing every cloud as fresh.
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self._target_frame
        cloud.height = 1
        cloud.width = cloud_arr.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud_arr.shape[0]
        cloud.data = cloud_arr.tobytes()
        cloud.is_dense = True
        self._publisher.publish(cloud)
        self._pub_count += 1
        self._last_publish_monotonic = time.monotonic()

        if not self._first_published:
            self._first_published = True
            self.get_logger().info(
                'Publishing cloud to %s (%d points).'
                % (self._output_topic, cloud_arr.shape[0])
            )
        elif self._stalled:
            self._stalled = False
            self.get_logger().info(
                'Cloud publishing resumed (cb=%d pub=%d pts=%d).'
                % (self._cb_count, self._pub_count, cloud_arr.shape[0])
            )

    def _warn_tf(self, source_frame: str, exc: Exception) -> None:
        """Explain a TF miss in terms of the thing that usually causes it."""
        self.get_logger().warning(
            "No transform '%s' -> '%s' at the cloud's own stamp (%d missed, "
            'cb=%d pub=%d); dropping it. %s'
            % (source_frame, self._target_frame, self._tf_miss_count,
               self._cb_count, self._pub_count, exc),
            throttle_duration_sec=5.0,
        )
        if self._tf_miss_count == 20 and not self.get_parameter(
                'use_sim_time').value:
            self.get_logger().error(
                'Every cloud is being dropped and use_sim_time is FALSE. In '
                'Gazebo the camera stamps in sim time while this node reads '
                'the wall clock, so no stamp will ever match. Launch this node '
                'with use_sim_time:=true.'
            )

    def _watchdog_cb(self) -> None:
        """Warn once if the relay was publishing and has since gone quiet."""
        if not self._first_published or self._stalled:
            return
        if self._last_publish_monotonic is None:
            return
        elapsed = time.monotonic() - self._last_publish_monotonic
        if elapsed >= self._stall_timeout_sec:
            self._stalled = True
            self.get_logger().warning(
                'Cloud publishing stalled: no output for %.1f s '
                '(cb=%d pub=%d). Is the camera still streaming?'
                % (elapsed, self._cb_count, self._pub_count)
            )

    def _lookup_transform(self, source_frame: str,
                          stamp) -> Optional[np.ndarray]:
        """Transform for the moment this cloud was CAPTURED, not for now.

        The camera rides on the arm, so the transform is only meaningful paired
        with the pose the arm held when the shutter opened. Asking for the newest
        transform instead pairs a cloud captured mid-slew with the pose the arm
        ended up in, and the geometry lands in the map rotated by however far the
        waist travelled in between.

        A stamped lookup only works if this node and the TF publisher agree on
        what time it is. Gazebo stamps in sim time and publishes /clock, and the
        interbotix bringup runs robot_state_publisher with use_sim_time, so this
        node must run with use_sim_time too -- the sim launches set it. Without
        it the lookup fails on every cloud and the relay goes silent, which is
        why the failure below is loud.

        Returns None to DROP the cloud rather than transform it wrongly. A
        missing obstacle is recoverable; a phantom one sitting over the
        workspace makes the planner refuse paths that are actually clear.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                stamp,
                timeout=self._tf_timeout,
            )
        except Exception as exc:  # noqa: BLE001
            self._tf_miss_count += 1
            if self._allow_latest_tf:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._target_frame, source_frame, rclpy.time.Time())
                except Exception as fallback_exc:  # noqa: BLE001
                    self._warn_tf(source_frame, fallback_exc)
                    return None
                self.get_logger().warning(
                    'Using the LATEST transform for a cloud stamped %d.%09d -- '
                    'geometry captured while the arm moved will be placed '
                    'wrongly. Set allow_latest_tf false once the clocks agree.'
                    % (stamp.sec, stamp.nanosec),
                    throttle_duration_sec=10.0,
                )
            else:
                self._warn_tf(source_frame, exc)
                return None

        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        quaternion = np.array(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float32,
        )
        matrix = np.eye(4, dtype=np.float32)
        matrix[:3, :3] = self._quaternion_to_matrix(quaternion)
        matrix[:3, 3] = translation
        return matrix

    @staticmethod
    def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        homogeneous = np.hstack((points, np.ones((points.shape[0], 1), dtype=np.float32)))
        return (homogeneous @ transform.T)[:, :3]

    def _publish_static_tf(self) -> None:
        if self._tf_broadcaster is None:
            return

        qx, qy, qz, qw = self._euler_to_quaternion(
            self._camera_roll,
            self._camera_pitch,
            self._camera_yaw,
        )
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._target_frame
        transform.child_frame_id = self._source_frame
        transform.transform.translation.x = float(self._translation[0])
        transform.transform.translation.y = float(self._translation[1])
        transform.transform.translation.z = float(self._translation[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        qx = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        qy = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
        qz = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
        qw = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        return qx, qy, qz, qw

    @staticmethod
    def _euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)

        rotation_x = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=np.float32,
        )
        rotation_y = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=np.float32,
        )
        rotation_z = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        return rotation_z @ rotation_y @ rotation_x

    @staticmethod
    def _quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
        x_axis, y_axis, z_axis, w_axis = quaternion
        return np.array(
            [
                [
                    1.0 - 2.0 * (y_axis * y_axis + z_axis * z_axis),
                    2.0 * (x_axis * y_axis - z_axis * w_axis),
                    2.0 * (x_axis * z_axis + y_axis * w_axis),
                ],
                [
                    2.0 * (x_axis * y_axis + z_axis * w_axis),
                    1.0 - 2.0 * (x_axis * x_axis + z_axis * z_axis),
                    2.0 * (y_axis * z_axis - x_axis * w_axis),
                ],
                [
                    2.0 * (x_axis * z_axis - y_axis * w_axis),
                    2.0 * (y_axis * z_axis + x_axis * w_axis),
                    1.0 - 2.0 * (x_axis * x_axis + y_axis * y_axis),
                ],
            ],
            dtype=np.float32,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenePointCloudRelay()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
