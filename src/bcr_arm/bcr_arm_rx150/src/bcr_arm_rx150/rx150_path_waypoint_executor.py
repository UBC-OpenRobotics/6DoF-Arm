#!/usr/bin/env python3

from __future__ import annotations

from typing import List, Optional

from bcr_arm_common import rx150_kinematics
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Rx150PathWaypointExecutor(Node):
    """Publish planned Cartesian path waypoints to the IK solver one at a time."""

    def __init__(self) -> None:
        super().__init__('rx150_path_waypoint_executor')

        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('path_topic', '/planned_cartesian_path')
        self.declare_parameter('ik_target_topic', '/ik_waypoint_target')
        self.declare_parameter('ik_target_pose_topic', '/ik_waypoint_target_pose')
        self.declare_parameter('executing_path_topic', '/ik_waypoint_path')
        self.declare_parameter('waypoint_target_mode', 'point')
        self.declare_parameter('waypoint_reached_tolerance', 0.015)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('waypoint_height_offset', 0.0)
        self.declare_parameter('verbose_waypoint_logging', True)
        self.declare_parameter('stuck_waypoint_warn_sec', 2.0)
        self.declare_parameter('stuck_waypoint_log_period_sec', 2.0)
        self.declare_parameter('stuck_waypoint_abort_sec', 12.0)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._path_topic = str(self.get_parameter('path_topic').value)
        self._ik_target_topic = str(self.get_parameter('ik_target_topic').value)
        self._ik_target_pose_topic = str(self.get_parameter('ik_target_pose_topic').value)
        self._executing_path_topic = str(self.get_parameter('executing_path_topic').value)
        waypoint_target_mode = str(self.get_parameter('waypoint_target_mode').value).strip().lower()
        if waypoint_target_mode not in {'point', 'pose_locked_current'}:
            self.get_logger().warning(
                "Unknown waypoint_target_mode '%s'. Falling back to 'point'."
                % waypoint_target_mode
            )
            waypoint_target_mode = 'point'
        self._waypoint_target_mode = waypoint_target_mode
        self._waypoint_reached_tolerance = float(
            self.get_parameter('waypoint_reached_tolerance').value
        )
        self._publish_period_sec = max(
            0.05, float(self.get_parameter('publish_period_sec').value)
        )
        self._waypoint_height_offset = float(
            self.get_parameter('waypoint_height_offset').value
        )
        self._verbose_waypoint_logging = bool(
            self.get_parameter('verbose_waypoint_logging').value
        )
        self._stuck_waypoint_warn_sec = max(
            0.0, float(self.get_parameter('stuck_waypoint_warn_sec').value)
        )
        self._stuck_waypoint_log_period_sec = max(
            0.1, float(self.get_parameter('stuck_waypoint_log_period_sec').value)
        )
        self._stuck_waypoint_abort_sec = max(
            0.0, float(self.get_parameter('stuck_waypoint_abort_sec').value)
        )
        self._joint_names = list(rx150_kinematics.JOINT_NAMES)

        self._current_q: Optional[np.ndarray] = None
        self._active_waypoints: List[np.ndarray] = []
        self._current_waypoint_index = 0
        self._last_published_index = -1
        self._last_logged_waypoint_index = -1
        self._locked_target_quaternion: Optional[np.ndarray] = None
        self._active_waypoint_publish_time_sec: Optional[float] = None
        self._last_stuck_log_time_sec: Optional[float] = None

        self.create_subscription(JointState, self._joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(Path, self._path_topic, self._path_cb, 10)
        self._point_pub = self.create_publisher(PointStamped, self._ik_target_topic, 10)
        self._pose_pub = self.create_publisher(PoseStamped, self._ik_target_pose_topic, 10)
        self._executing_path_pub = self.create_publisher(Path, self._executing_path_topic, 10)
        self.create_timer(self._publish_period_sec, self._timer_cb)

        self.get_logger().info(
            'RX-150 waypoint executor listening on %s and publishing IK waypoints to %s'
            % (self._path_topic, self._ik_target_topic)
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self._joint_names):
            self._current_q = np.array([positions[name] for name in self._joint_names], dtype=float)

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self.get_logger().warning('Received empty path; ignoring it.')
            return

        frame = msg.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().warning(
                "Ignoring path in frame '%s'; expected '%s'." % (frame, self._world_frame)
            )
            return

        xyz_points = []
        for pose_stamped in msg.poses:
            xyz_points.append(
                np.array(
                    [
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z + self._waypoint_height_offset,
                    ],
                    dtype=float,
                )
            )

        if len(xyz_points) >= 2:
            xyz_points = xyz_points[1:]

        if not xyz_points:
            self.get_logger().warning('Path only contained a start pose; nothing to execute.')
            return

        if self._waypoint_target_mode == 'pose_locked_current':
            if self._current_q is None:
                self.get_logger().warning(
                    'No current joint state yet; cannot lock carry orientation for path execution.'
                )
                return
            _, current_rotation = self._forward_kinematics_pose(self._current_q)
            self._locked_target_quaternion = self._rotation_matrix_to_quaternion(
                current_rotation
            )
            if self._locked_target_quaternion is None:
                self.get_logger().warning(
                    'Could not compute carry orientation from current end-effector pose.'
                )
                return
            self.get_logger().info(
                'Locked current end-effector orientation for carry path execution.'
            )
        else:
            self._locked_target_quaternion = None

        self._active_waypoints = xyz_points
        self._current_waypoint_index = 0
        self._last_published_index = -1
        self._last_logged_waypoint_index = -1
        self._active_waypoint_publish_time_sec = None
        self._last_stuck_log_time_sec = None
        self._publish_remaining_path()
        self.get_logger().info(
            'Loaded planned path with %d executable waypoint(s).' % len(self._active_waypoints)
        )

    def _timer_cb(self) -> None:
        if self._current_q is None or not self._active_waypoints:
            return

        if self._current_waypoint_index >= len(self._active_waypoints):
            return

        target_xyz = self._active_waypoints[self._current_waypoint_index]
        current_xyz = self._forward_kinematics(self._current_q)
        distance_to_target = float(np.linalg.norm(target_xyz - current_xyz))

        self._maybe_log_waypoint_status(target_xyz)
        self._maybe_log_stuck_waypoint(target_xyz, distance_to_target)

        if self._waypoint_abort_due(distance_to_target):
            self.get_logger().error(
                'Aborting path: waypoint %d/%d [%.3f, %.3f, %.3f] not reached '
                'after %.1f s (best distance %.4f m). It is likely beyond the '
                "arm's reach; discarding the rest of the path."
                % (
                    self._current_waypoint_index + 1,
                    len(self._active_waypoints),
                    target_xyz[0],
                    target_xyz[1],
                    target_xyz[2],
                    self._clock_now_sec() - self._active_waypoint_publish_time_sec,
                    distance_to_target,
                )
            )
            self._active_waypoints = []
            self._last_published_index = -1
            self._last_logged_waypoint_index = -1
            self._active_waypoint_publish_time_sec = None
            self._last_stuck_log_time_sec = None
            self._publish_remaining_path()
            return

        if distance_to_target <= self._waypoint_reached_tolerance:
            self._current_waypoint_index += 1
            if self._current_waypoint_index >= len(self._active_waypoints):
                self._active_waypoints = []
                self._last_published_index = -1
                self._last_logged_waypoint_index = -1
                self._active_waypoint_publish_time_sec = None
                self._last_stuck_log_time_sec = None
                self._publish_remaining_path()
                return
            target_xyz = self._active_waypoints[self._current_waypoint_index]
            self._active_waypoint_publish_time_sec = None
            self._last_stuck_log_time_sec = None
            self._publish_remaining_path()

        if self._last_published_index == self._current_waypoint_index:
            return

        if (
            self._waypoint_target_mode == 'pose_locked_current'
            and self._locked_target_quaternion is not None
        ):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self._world_frame
            pose_msg.pose.position.x = float(target_xyz[0])
            pose_msg.pose.position.y = float(target_xyz[1])
            pose_msg.pose.position.z = float(target_xyz[2])
            pose_msg.pose.orientation.x = float(self._locked_target_quaternion[0])
            pose_msg.pose.orientation.y = float(self._locked_target_quaternion[1])
            pose_msg.pose.orientation.z = float(self._locked_target_quaternion[2])
            pose_msg.pose.orientation.w = float(self._locked_target_quaternion[3])
            self._pose_pub.publish(pose_msg)
        else:
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = self._world_frame
            point_msg.point.x = float(target_xyz[0])
            point_msg.point.y = float(target_xyz[1])
            point_msg.point.z = float(target_xyz[2])
            self._point_pub.publish(point_msg)
        self._last_published_index = self._current_waypoint_index
        self._active_waypoint_publish_time_sec = self._clock_now_sec()
        self._last_stuck_log_time_sec = None
        self.get_logger().info(
            'Sent waypoint %d/%d to IK | target=[%.3f, %.3f, %.3f] | mode=%s'
            % (
                self._current_waypoint_index + 1,
                len(self._active_waypoints),
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
                self._waypoint_target_mode,
            )
        )

    def _maybe_log_waypoint_status(
        self,
        target_xyz: np.ndarray,
    ) -> None:
        if not self._verbose_waypoint_logging:
            return

        if self._last_logged_waypoint_index == self._current_waypoint_index:
            return

        self._last_logged_waypoint_index = self._current_waypoint_index
        self.get_logger().info(
            'Active waypoint %d/%d | target=[%.3f, %.3f, %.3f]'
            % (
                self._current_waypoint_index + 1,
                len(self._active_waypoints),
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
            )
        )

    def _waypoint_abort_due(self, distance_to_target: float) -> bool:
        if self._stuck_waypoint_abort_sec <= 0.0:
            return False
        if self._active_waypoint_publish_time_sec is None:
            return False
        if distance_to_target <= self._waypoint_reached_tolerance:
            return False
        elapsed_sec = self._clock_now_sec() - self._active_waypoint_publish_time_sec
        return elapsed_sec >= self._stuck_waypoint_abort_sec

    def _maybe_log_stuck_waypoint(
        self,
        target_xyz: np.ndarray,
        distance_to_target: float,
    ) -> None:
        if self._active_waypoint_publish_time_sec is None:
            return

        elapsed_sec = self._clock_now_sec() - self._active_waypoint_publish_time_sec
        if elapsed_sec < self._stuck_waypoint_warn_sec:
            return

        if (
            self._last_stuck_log_time_sec is not None
            and (self._clock_now_sec() - self._last_stuck_log_time_sec)
            < self._stuck_waypoint_log_period_sec
        ):
            return

        self._last_stuck_log_time_sec = self._clock_now_sec()
        self.get_logger().warning(
            'Still waiting on waypoint %d/%d | target=[%.3f, %.3f, %.3f] | '
            'distance=%.4f m | elapsed=%.1f s'
            % (
                self._current_waypoint_index + 1,
                len(self._active_waypoints),
                target_xyz[0],
                target_xyz[1],
                target_xyz[2],
                distance_to_target,
                elapsed_sec,
            )
        )

    def _publish_remaining_path(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._world_frame

        for xyz in self._active_waypoints[self._current_waypoint_index:]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self._executing_path_pub.publish(path)

    def _forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        xyz, _ = self._forward_kinematics_pose(q)
        return xyz

    def _forward_kinematics_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return rx150_kinematics.forward_kinematics(q)

    def _clock_now_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + (float(now_msg.nanosec) * 1e-9)

    @staticmethod
    def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> Optional[np.ndarray]:
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = 2.0 * np.sqrt(trace + 1.0)
            if scale <= 1e-9:
                return None
            qw = 0.25 * scale
            qx = (rotation[2, 1] - rotation[1, 2]) / scale
            qy = (rotation[0, 2] - rotation[2, 0]) / scale
            qz = (rotation[1, 0] - rotation[0, 1]) / scale
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            if scale <= 1e-9:
                return None
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif rotation[1, 1] > rotation[2, 2]:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            if scale <= 1e-9:
                return None
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            if scale <= 1e-9:
                return None
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale

        quaternion = np.array([qx, qy, qz, qw], dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1e-9:
            return None
        return quaternion / norm


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rx150PathWaypointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
