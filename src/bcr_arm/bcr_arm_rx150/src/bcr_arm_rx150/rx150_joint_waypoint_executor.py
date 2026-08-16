#!/usr/bin/env python3
"""Step a planned joint-space path through the IK executor's direct joint channel.

The RRT-Connect whole-body fallback produces a path in *joint* space (a list of
configurations, each collision-checked as a whole-arm posture). That guarantee is
only preserved if the path is executed as joint angles -- never re-solved through
Cartesian IK, which could re-converge to a different, unchecked posture.

This node is the joint-space analogue of ``rx150_path_waypoint_executor`` (which
is Cartesian): it subscribes to a ``trajectory_msgs/JointTrajectory`` path, and
commands each configuration one at a time on the IK executor's direct
``/rx150/joint_command`` input (a ``sensor_msgs/JointState``), advancing when the
measured joints from ``/rx150/joint_states`` are within a joint-space tolerance.
Because the actual arm command is emitted by the IK executor's mode-aware
publisher, this stepper is identical on sim (``trajectory``) and hardware
(``group``) -- it never touches a controller topic directly.
"""

from __future__ import annotations

from typing import List, Optional

from bcr_arm_common import rx150_kinematics
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class Rx150JointWaypointExecutor(Node):
    """Play a joint-space path waypoint-by-waypoint via the direct joint channel."""

    def __init__(self) -> None:
        super().__init__('rx150_joint_waypoint_executor')

        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('joint_path_topic', '/planned_joint_path')
        self.declare_parameter('joint_command_topic', '/rx150/joint_command')
        self.declare_parameter('waypoint_joint_tolerance', 0.05)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('stuck_waypoint_warn_sec', 3.0)
        self.declare_parameter('stuck_waypoint_log_period_sec', 2.0)

        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._joint_path_topic = str(self.get_parameter('joint_path_topic').value)
        self._joint_command_topic = str(self.get_parameter('joint_command_topic').value)
        self._waypoint_joint_tolerance = float(
            self.get_parameter('waypoint_joint_tolerance').value
        )
        self._publish_period_sec = max(
            0.05, float(self.get_parameter('publish_period_sec').value)
        )
        self._stuck_waypoint_warn_sec = max(
            0.0, float(self.get_parameter('stuck_waypoint_warn_sec').value)
        )
        self._stuck_waypoint_log_period_sec = max(
            0.1, float(self.get_parameter('stuck_waypoint_log_period_sec').value)
        )

        self._joint_names = list(rx150_kinematics.JOINT_NAMES)

        self._current_q: Optional[np.ndarray] = None
        self._active_waypoints: List[np.ndarray] = []
        self._current_waypoint_index = 0
        self._last_published_index = -1
        self._active_waypoint_publish_time_sec: Optional[float] = None
        self._last_stuck_log_time_sec: Optional[float] = None

        self.create_subscription(JointState, self._joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(JointTrajectory, self._joint_path_topic, self._path_cb, 10)
        self._command_pub = self.create_publisher(
            JointState, self._joint_command_topic, 10
        )
        self.create_timer(self._publish_period_sec, self._timer_cb)

        self.get_logger().info(
            'RX-150 joint waypoint executor listening on %s; commanding %s'
            % (self._joint_path_topic, self._joint_command_topic)
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self._joint_names):
            self._current_q = np.array(
                [positions[name] for name in self._joint_names], dtype=float
            )

    def _path_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning('Received empty joint path; ignoring it.')
            return

        # Map incoming joint order to our canonical order; missing joints hold at
        # the current value (or zero if unknown).
        name_to_slot = {name: index for index, name in enumerate(msg.joint_names)}
        base = self._current_q if self._current_q is not None else np.zeros(
            len(self._joint_names), dtype=float
        )
        waypoints: List[np.ndarray] = []
        for point in msg.points:
            config = base.copy()
            for slot, joint_name in enumerate(self._joint_names):
                if joint_name in name_to_slot:
                    source_index = name_to_slot[joint_name]
                    if source_index < len(point.positions):
                        config[slot] = float(point.positions[source_index])
            waypoints.append(config)

        # Drop a leading waypoint that equals the current pose (nothing to do).
        if len(waypoints) >= 2 and self._current_q is not None:
            if np.max(np.abs(waypoints[0] - self._current_q)) <= self._waypoint_joint_tolerance:
                waypoints = waypoints[1:]

        self._active_waypoints = waypoints
        self._current_waypoint_index = 0
        self._last_published_index = -1
        self._active_waypoint_publish_time_sec = None
        self._last_stuck_log_time_sec = None
        self.get_logger().info(
            'Loaded planned joint path with %d waypoint(s).' % len(self._active_waypoints)
        )

    def _timer_cb(self) -> None:
        if self._current_q is None or not self._active_waypoints:
            return
        if self._current_waypoint_index >= len(self._active_waypoints):
            return

        target_q = self._active_waypoints[self._current_waypoint_index]
        joint_error = float(np.max(np.abs(target_q - self._current_q)))

        self._maybe_log_stuck_waypoint(joint_error)

        if joint_error <= self._waypoint_joint_tolerance:
            self._current_waypoint_index += 1
            if self._current_waypoint_index >= len(self._active_waypoints):
                self.get_logger().info('Joint path execution complete.')
                self._active_waypoints = []
                self._last_published_index = -1
                self._active_waypoint_publish_time_sec = None
                self._last_stuck_log_time_sec = None
                return
            target_q = self._active_waypoints[self._current_waypoint_index]
            self._active_waypoint_publish_time_sec = None
            self._last_stuck_log_time_sec = None

        if self._last_published_index == self._current_waypoint_index:
            return

        self._publish_joint_command(target_q)
        self._last_published_index = self._current_waypoint_index
        self._active_waypoint_publish_time_sec = self._clock_now_sec()
        self._last_stuck_log_time_sec = None
        self.get_logger().info(
            'Commanded joint waypoint %d/%d | q=[%s]'
            % (
                self._current_waypoint_index + 1,
                len(self._active_waypoints),
                ', '.join('%.3f' % value for value in target_q),
            )
        )

    def _publish_joint_command(self, target_q: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [float(value) for value in target_q]
        self._command_pub.publish(msg)

    def _maybe_log_stuck_waypoint(self, joint_error: float) -> None:
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
            'Still waiting on joint waypoint %d/%d | max joint error=%.4f rad | elapsed=%.1f s'
            % (
                self._current_waypoint_index + 1,
                len(self._active_waypoints),
                joint_error,
                elapsed_sec,
            )
        )

    def _clock_now_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + (float(now_msg.nanosec) * 1e-9)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rx150JointWaypointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
