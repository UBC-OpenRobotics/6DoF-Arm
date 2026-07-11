#!/usr/bin/env python3

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


# Waist angles for the sweep (radians). ±135° covers the full workspace arc.
# Shoulder/elbow/wrist are sampled from the arm's starting joint state at runtime
# so the scan uses whatever orientation the arm is already in.
SCAN_WAIST_ANGLES = [-2.356, -1.571, -0.785, 0.0, 0.785, 1.571, 2.356]

JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']

# RX-150 kinematic parameters (matches rx150_point_cloud_path_planner)
_FK_ORIGINS = [
    np.array([0.0,   0.0, 0.06566], dtype=float),
    np.array([0.0,   0.0, 0.03891], dtype=float),
    np.array([0.05,  0.0, 0.15],    dtype=float),
    np.array([0.15,  0.0, 0.0],     dtype=float),
    np.array([0.065, 0.0, 0.0],     dtype=float),
]
_FK_AXES = [
    np.array([0.0, 0.0, 1.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([0.0, 1.0, 0.0], dtype=float),
    np.array([1.0, 0.0, 0.0], dtype=float),
]
_FK_TOOL_OFFSET = np.array([0.108, 0.0, 0.0], dtype=float)


def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    t = 1.0 - c
    return np.array([
        [t*x*x + c,   t*x*y - z*s, t*x*z + y*s, 0.0],
        [t*x*y + z*s, t*y*y + c,   t*y*z - x*s, 0.0],
        [t*x*z - y*s, t*y*z + x*s, t*z*z + c,   0.0],
        [0.0,         0.0,         0.0,          1.0],
    ], dtype=float)


def _translation_matrix(offset: np.ndarray) -> np.ndarray:
    m = np.eye(4, dtype=float)
    m[:3, 3] = offset
    return m


def forward_kinematics(q: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """Return (xyz, rotation_matrix 3x3) of the end-effector in base frame."""
    transform = np.eye(4, dtype=float)
    for origin, axis, angle in zip(_FK_ORIGINS, _FK_AXES, q):
        transform = transform @ _translation_matrix(origin)
        transform = transform @ _rotation_matrix(axis, angle)
    transform = transform @ _translation_matrix(_FK_TOOL_OFFSET)
    return transform[:3, 3].copy(), transform[:3, :3].copy()


def rotation_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to (qx, qy, qz, qw)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class SceneSweepMapper(Node):
    def __init__(self) -> None:
        super().__init__('scene_sweep_mapper')

        self.declare_parameter('input_topic',      '/planning/live_point_cloud')
        self.declare_parameter('output_topic',     '/planning/point_cloud')
        self.declare_parameter('ik_target_topic',  '/ik_waypoint_target_pose')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('world_frame',      'rx150/base_link')
        self.declare_parameter('frame_id',         'rx150/base_link')
        self.declare_parameter('settle_sec',       5.0)
        self.declare_parameter('sample_sec',       2.0)
        self.declare_parameter('command_publish_count', 5)
        self.declare_parameter('voxel_size',       0.01)
        self.declare_parameter('x_min',  -1.60)
        self.declare_parameter('x_max',   1.60)
        self.declare_parameter('y_min',  -1.60)
        self.declare_parameter('y_max',   1.60)
        self.declare_parameter('z_min',  -0.05)
        self.declare_parameter('z_max',   0.50)
        self.declare_parameter('max_input_range', 1.50)
        self.declare_parameter('return_to_home',  False)

        self._input_topic    = str(self.get_parameter('input_topic').value)
        self._output_topic   = str(self.get_parameter('output_topic').value)
        self._ik_topic       = str(self.get_parameter('ik_target_topic').value)
        self._js_topic       = str(self.get_parameter('joint_state_topic').value)
        self._world_frame    = str(self.get_parameter('world_frame').value)
        self._frame_id       = str(self.get_parameter('frame_id').value)
        self._settle_sec     = float(self.get_parameter('settle_sec').value)
        self._sample_sec     = float(self.get_parameter('sample_sec').value)
        self._publish_count  = max(1, int(self.get_parameter('command_publish_count').value))
        self._voxel_size     = max(1e-4, float(self.get_parameter('voxel_size').value))
        self._bounds = np.array([
            [float(self.get_parameter('x_min').value), float(self.get_parameter('x_max').value)],
            [float(self.get_parameter('y_min').value), float(self.get_parameter('y_max').value)],
            [float(self.get_parameter('z_min').value), float(self.get_parameter('z_max').value)],
        ], dtype=np.float32)
        self._max_input_range = max(0.0, float(self.get_parameter('max_input_range').value))
        self._return_to_home  = bool(self.get_parameter('return_to_home').value)

        self._latest_points: np.ndarray | None = None   # raw, bounds applied only in _build_map
        self._accumulated_points: list[np.ndarray] = []
        self._cloud_received: bool = False
        self._current_q: np.ndarray | None = None
        self._cb_count: int = 0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._map_pub = self.create_publisher(PointCloud2, self._output_topic, latched_qos)
        self._ik_pub  = self.create_publisher(PoseStamped, self._ik_topic, 10)

        self.create_subscription(
            PointCloud2, self._input_topic, self._point_cloud_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, self._js_topic, self._joint_state_cb, 10
        )

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self._current_q = np.array(
                [positions[name] for name in JOINT_NAMES], dtype=np.float64
            )

    def _point_cloud_cb(self, msg: PointCloud2) -> None:
        self._cloud_received = True
        self._cb_count += 1
        points = point_cloud2.read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True
        )
        if points.dtype.names:
            points = np.column_stack(
                [points['x'], points['y'], points['z']]
            ).astype(np.float32)
        else:
            points = np.asarray(points, dtype=np.float32)
        if points.ndim == 1:
            points = points.reshape(-1, 3)

        if points.size == 0:
            self._latest_points = None
            return
        if self._max_input_range > 0.0:
            mask = np.linalg.norm(points, axis=1) <= self._max_input_range
            points = points[mask]
            if points.size == 0:
                self._latest_points = None
                return
        # Store raw range-filtered points; bounds cropping happens in _build_map
        self._latest_points = points
        if self._cb_count % 20 == 1:
            self.get_logger().info(
                'Cloud cb=%d  pts=%d  x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]'
                % (self._cb_count, points.shape[0],
                   points[:, 0].min(), points[:, 0].max(),
                   points[:, 1].min(), points[:, 1].max(),
                   points[:, 2].min(), points[:, 2].max())
            )

    # ------------------------------------------------------------------
    def run(self) -> int:
        self.get_logger().info(
            'Starting sweep: %s → %s (frame %s)'
            % (self._input_topic, self._output_topic, self._frame_id)
        )

        if not self._wait_for_cloud(timeout_sec=90.0):
            self.get_logger().error(
                'No cloud on %s after 90 s. Is use_scene_point_cloud:=true?'
                % self._input_topic
            )
            return 1

        # Lock the arm's current shoulder/elbow/wrist as the fixed scan orientation.
        # Only the waist (joint 0) changes per pose.
        q0 = self._current_q
        scan_poses = [
            ('scan_%+.0fdeg' % np.degrees(w),
             [float(w), float(q0[1]), float(q0[2]), float(q0[3]), float(q0[4])])
            for w in SCAN_WAIST_ANGLES
        ]
        self.get_logger().info(
            'Sweep orientation locked from starting joints: '
            'shoulder=%.3f elbow=%.3f wrist=%.3f' % (q0[1], q0[2], q0[3])
        )

        for pose_name, joints in scan_poses:
            self.get_logger().info("Moving to sweep pose '%s'" % pose_name)
            self._send_pose(joints)
            if not self._spin_for(self._settle_sec):
                return 130

            before = len(self._accumulated_points)
            if not self._collect_for_pose(pose_name):
                return 130
            frames = len(self._accumulated_points) - before
            self.get_logger().info("Collected %d frames at '%s'" % (frames, pose_name))

        if self._return_to_home:
            self.get_logger().info('Returning to home')
            self._send_pose([0.0, 0.0, 0.0, 0.0, 0.0])
            self._spin_for(self._settle_sec)

        merged = self._build_map()
        if merged.size == 0:
            self.get_logger().error('Sweep complete but accumulated map is empty.')
            return 1

        self._publish_map(merged)
        self.get_logger().info(
            'Published map with %d points on %s — node staying alive, Ctrl+C when done.'
            % (merged.shape[0], self._output_topic)
        )
        # Keep the node alive so the latched publisher can deliver to RViz subscribers
        # that connect after the map is published.
        try:
            while rclpy.ok():
                self._executor.spin_once(timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        return 0

    # ------------------------------------------------------------------
    def _send_pose(self, joints: list[float]) -> None:
        """Compute full FK pose and send to the IK executor as PoseStamped."""
        xyz, rot = forward_kinematics(joints)
        qx, qy, qz, qw = rotation_to_quaternion(rot)
        self.get_logger().info(
            'FK → xyz=[%.3f, %.3f, %.3f], publishing to %s'
            % (xyz[0], xyz[1], xyz[2], self._ik_topic)
        )
        msg = PoseStamped()
        msg.header.frame_id = self._world_frame
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        for _ in range(self._publish_count):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._ik_pub.publish(msg)
            self._executor.spin_once(timeout_sec=0.05)

    def _collect_for_pose(self, pose_name: str) -> bool:
        deadline = time.monotonic() + self._sample_sec
        frames = 0
        last_pts = None
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            self._executor.spin_once(timeout_sec=0.1)
            if self._latest_points is not None and self._latest_points.size > 0:
                self._accumulated_points.append(self._latest_points.copy())
                last_pts = self._latest_points
                frames += 1
        if frames == 0:
            self.get_logger().warning("No cloud frames at '%s'" % pose_name)
        else:
            cropped = self._crop_points(last_pts) if last_pts is not None else np.empty((0, 3))
            self.get_logger().info(
                "'%s': %d raw pts  →  %d in-bounds  (x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f])"
                % (pose_name, last_pts.shape[0] if last_pts is not None else 0, cropped.shape[0],
                   self._bounds[0, 0], self._bounds[0, 1],
                   self._bounds[1, 0], self._bounds[1, 1],
                   self._bounds[2, 0], self._bounds[2, 1])
            )
        return True

    def _build_map(self) -> np.ndarray:
        if not self._accumulated_points:
            return np.empty((0, 3), dtype=np.float32)
        merged = np.vstack(self._accumulated_points).astype(np.float32, copy=False)
        merged = self._crop_points(merged)
        if merged.size == 0:
            return merged
        voxel_index = np.floor(merged / self._voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_index, axis=0, return_index=True)
        return merged[np.sort(unique_idx)]

    def _publish_map(self, points: np.ndarray) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        cloud_arr = np.ascontiguousarray(points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = cloud_arr.shape[0]
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud_arr.shape[0]
        cloud.data = cloud_arr.tobytes()
        cloud.is_dense = False
        self._map_pub.publish(cloud)

    def _wait_for_cloud(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            self._executor.spin_once(timeout_sec=0.1)
            if self._cloud_received and self._current_q is not None:
                return True
        return False

    def _spin_for(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            self._executor.spin_once(timeout_sec=0.1)
        return True

    def _crop_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points.reshape((-1, 3))
        mask = np.logical_and.reduce([
            points[:, 0] >= self._bounds[0, 0],
            points[:, 0] <= self._bounds[0, 1],
            points[:, 1] >= self._bounds[1, 0],
            points[:, 1] <= self._bounds[1, 1],
            points[:, 2] >= self._bounds[2, 0],
            points[:, 2] <= self._bounds[2, 1],
        ])
        return points[mask]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneSweepMapper()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node._executor = executor
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
