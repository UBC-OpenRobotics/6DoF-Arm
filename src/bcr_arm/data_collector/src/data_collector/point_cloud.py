#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class PointCloudCollector(Node):
    def __init__(self) -> None:
        super().__init__('point_cloud')

        self._tf_broadcasted = False
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._subscriber = None

        self.create_timer(0.1, self.broadcast_tf)
        self.create_timer(0.5, self.setup_subscriber)
        self._points_publisher = self.create_publisher(PointCloud2, 'filtered_point_cloud', 10)

        self.get_logger().info('PointCloudCollector has started')

    def setup_subscriber(self) -> None:
        if self._subscriber is not None or not self._tf_broadcasted:
            return
        self._subscriber = self.create_subscription(
            PointCloud2,
            '/camera_sensor/points',
            self.point_subscriber,
            10,
        )
        self.get_logger().info('PointCloud subscription started')

    def broadcast_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'camera'
        transform.transform.translation.x = 0.28
        transform.transform.translation.y = 0.34
        transform.transform.translation.z = 0.28

        qx, qy, qz, qw = self.euler_to_quaternion(3.8, 0.0, 3.14)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(transform)
        self._tf_broadcasted = True

    def point_subscriber(self, msg: PointCloud2) -> None:
        points = self.pointcloud2_to_numpy(msg)
        transform = self._tf_buffer.lookup_transform('world', 'camera', rclpy.time.Time())

        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = self.quaternion_to_matrix(quaternion)
        transform_matrix[:3, 3] = translation

        transformed_points = self.transform_point_cloud(points, transform_matrix)
        self._points_publisher.publish(self.numpy_to_pointcloud2(transformed_points))

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def quaternion_to_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        x, y, z, w = quaternion
        return np.array([
            [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x**2 - 2 * y**2],
        ])

    def numpy_to_pointcloud2(self, np_array: np.ndarray) -> PointCloud2:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, np_array.tolist())

    def pointcloud2_to_numpy(self, points_msg: PointCloud2) -> np.ndarray:
        return point_cloud2.read_points_numpy(
            points_msg,
            field_names=['x', 'y', 'z', 'rgb'],
            skip_nans=True,
        )

    def transform_point_cloud(self, points: np.ndarray, transformation: np.ndarray) -> np.ndarray:
        if points.shape[1] != 4:
            raise ValueError('Input points must have shape (N, 4) with [x, y, z, rgb]')

        xyz = points[:, :3]
        xyz_homogeneous = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
        transformed_xyz = (xyz_homogeneous @ transformation.T)[:, :3]
        return np.hstack((transformed_xyz, points[:, 3:4]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
