#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
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

        self._rotation = self._euler_to_matrix(
            self._camera_roll,
            self._camera_pitch,
            self._camera_yaw,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(PointCloud2, self._output_topic, 10)
        self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._point_cloud_cb,
            qos_profile_sensor_data,
        )

        self._tf_broadcaster: Optional[StaticTransformBroadcaster] = None
        if self._broadcast_static_tf:
            self._tf_broadcaster = StaticTransformBroadcaster(self)
            self._publish_static_tf()

        self._cb_count = 0
        self._pub_count = 0

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
            transform = self._lookup_transform(source_frame)
            if transform is None:
                if self._cb_count % 5 == 1:
                    self.get_logger().warning(
                        'TF lookup failed (cb=%d pub=%d) frame=%s'
                        % (self._cb_count, self._pub_count, source_frame)
                    )
                return
            transformed_points = self._transform_points(points, transform)
        else:
            transformed_points = (points @ self._rotation.T) + self._translation

        cloud_arr = np.ascontiguousarray(transformed_points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
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
        if self._pub_count % 5 == 1:
            self.get_logger().info(
                'Published cloud (cb=%d pub=%d pts=%d)'
                % (self._cb_count, self._pub_count, cloud_arr.shape[0])
            )

    def _lookup_transform(self, source_frame: str) -> Optional[np.ndarray]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Could not transform scene point cloud from '%s' to '%s': %s"
                % (source_frame, self._target_frame, exc)
            )
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
