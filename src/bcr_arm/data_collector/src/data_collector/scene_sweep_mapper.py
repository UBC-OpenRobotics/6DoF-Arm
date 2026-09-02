#!/usr/bin/env python3

from __future__ import annotations

import time

from bcr_arm_common import rx150_kinematics
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Header, String

# Open3D backs the optional outlier filters. Imported softly because they are
# off by default, so a workspace without Open3D still runs the sweep.
try:
    import open3d as o3d
except ImportError:  # pragma: no cover - depends on the environment
    o3d = None


# Waist angles for the sweep (radians), 45 deg apart from -177 deg to +135 deg,
# leaving a 47 deg gap through the back where nothing is placed. Eight stations
# cover the full circle against the D435i's 54.5 deg horizontal FOV; respacing
# them evenly opens a hole rather than closing one.
SCAN_WAIST_ANGLES = [-3.10, -2.356, -1.571, -0.785, 0.0, 0.785, 1.571, 2.356]

# waist travel limits from interbotix_xsarm_descriptions rx150.urdf.xacro
# (-180 deg .. +180 deg). Stations are clamped to these, for the same reason the
# wrist offsets are: a station past the stop never reports arrival and the sweep
# hangs on settle instead of failing.
WAIST_ANGLE_LIMITS = (-3.142, 3.142)

# Wrist-angle offsets (radians) applied to the scan posture at EVERY waist
# station -- the arm takes one look per offset before moving on.
SCAN_WRIST_OFFSETS = [0.0, 0.15]

# wrist_angle travel limits from interbotix_xsarm_descriptions rx150.urdf.xacro
# (-100 deg .. +123 deg). Offsets are clamped to these so a hand-tuned posture
# plus an offset can never command past the servo's stop.
WRIST_ANGLE_LIMITS = (-1.745, 2.147)

# Scan postures (shoulder, elbow, wrist_angle, wrist_rotate), selected by the
# `scan_posture` parameter. 
SCAN_POSTURES = {
    'level':  [-1.00,  1.60, -0.60, 0.0],
    'tilted': [-1.00, -0.43,  1.95, 0.0],
}
SCAN_TUCKED_JOINTS = SCAN_POSTURES['tilted']  # back-compat for external importers

# The FK chain, joint names, and capsule collision model live in the shared
# bcr_arm_common.rx150_kinematics module (single source of truth). The sweep uses
# rx150_kinematics.forward_kinematics + JOINT_NAMES below.


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
        self.declare_parameter('settle_sec',       3.0)
        self.declare_parameter('sample_sec',       1.0)
        self.declare_parameter('min_frames_per_pose', 2)
        self.declare_parameter('sample_timeout_sec',  5.0)
        self.declare_parameter('drain_sec', 0.0)
        self.declare_parameter('command_publish_count', 5)
        self.declare_parameter('voxel_size',       0.01)
        self.declare_parameter('x_min',  -1.60)
        self.declare_parameter('x_max',   1.60)
        self.declare_parameter('y_min',  -1.60)
        self.declare_parameter('y_max',   1.60)
        self.declare_parameter('z_min',  -0.05)
        self.declare_parameter('z_max',   0.50)
        self.declare_parameter('max_input_range', 1.50)
        self.declare_parameter('enable_statistical_outlier_removal', False)
        self.declare_parameter('statistical_nb_neighbors', 20)
        self.declare_parameter('statistical_std_ratio', 2.0)
        self.declare_parameter('enable_radius_outlier_removal', False)
        self.declare_parameter('radius_outlier_radius', 0.02)
        self.declare_parameter('radius_outlier_min_neighbors', 10)
        self.declare_parameter('return_to_home',  False)
        self.declare_parameter('home_joints', [0.0, -0.65, -0.20, -1.00, 0.0])
        self.declare_parameter('scan_posture',    'tilted')  # 'tilted' | 'level'
        self.declare_parameter('scan_tucked_joints', [float('nan')] * 4)
        self.declare_parameter('scan_waist_angles', list(SCAN_WAIST_ANGLES))
        self.declare_parameter('scan_wrist_offsets', list(SCAN_WRIST_OFFSETS))
        self.declare_parameter('scan_wrist_settle_sec', 1.2)
        self.declare_parameter('scan_return_to_neutral', True)
        self.declare_parameter('trigger_topic',    '/sweep/start')
        self.declare_parameter('status_topic',     '/motion/status')
        self.declare_parameter('stop_topic',       '/sweep/stop')
        self.declare_parameter('wait_for_trigger', False)

        self._input_topic    = str(self.get_parameter('input_topic').value)
        self._output_topic   = str(self.get_parameter('output_topic').value)
        self._ik_topic       = str(self.get_parameter('ik_target_topic').value)
        self._js_topic       = str(self.get_parameter('joint_state_topic').value)
        self._world_frame    = str(self.get_parameter('world_frame').value)
        self._frame_id       = str(self.get_parameter('frame_id').value)
        self._settle_sec     = float(self.get_parameter('settle_sec').value)
        self._sample_sec     = float(self.get_parameter('sample_sec').value)
        self._min_frames     = max(0, int(self.get_parameter('min_frames_per_pose').value))
        self._sample_timeout_sec = float(self.get_parameter('sample_timeout_sec').value)
        self._drain_sec      = max(0.0, float(self.get_parameter('drain_sec').value))
        self._publish_count  = max(1, int(self.get_parameter('command_publish_count').value))
        self._voxel_size     = max(1e-4, float(self.get_parameter('voxel_size').value))
        self._bounds = np.array([
            [float(self.get_parameter('x_min').value), float(self.get_parameter('x_max').value)],
            [float(self.get_parameter('y_min').value), float(self.get_parameter('y_max').value)],
            [float(self.get_parameter('z_min').value), float(self.get_parameter('z_max').value)],
        ], dtype=np.float32)
        self._max_input_range = max(0.0, float(self.get_parameter('max_input_range').value))
        self._stat_enabled  = bool(self.get_parameter('enable_statistical_outlier_removal').value)
        self._stat_neighbors = int(self.get_parameter('statistical_nb_neighbors').value)
        self._stat_std_ratio = float(self.get_parameter('statistical_std_ratio').value)
        self._radius_enabled = bool(self.get_parameter('enable_radius_outlier_removal').value)
        self._radius_size    = float(self.get_parameter('radius_outlier_radius').value)
        self._radius_min_neighbors = int(self.get_parameter('radius_outlier_min_neighbors').value)
        if (self._stat_enabled or self._radius_enabled) and o3d is None:
            self.get_logger().error(
                'Outlier filtering was requested but Open3D is not importable; '
                'continuing with crop + voxel only. Install open3d or set '
                'enable_statistical_outlier_removal / '
                'enable_radius_outlier_removal to false to silence this.'
            )
        self._return_to_home  = bool(self.get_parameter('return_to_home').value)
        self._home_joints = [float(v) for v in
                             self.get_parameter('home_joints').value]
        self._scan_joints = self._resolve_scan_posture()
        self._wrist_offsets = self._resolve_wrist_offsets()
        self._waist_angles = self._resolve_waist_angles()
        self._wrist_settle_sec = max(
            0.0, float(self.get_parameter('scan_wrist_settle_sec').value))
        self._return_to_neutral = bool(
            self.get_parameter('scan_return_to_neutral').value)
        self._wait_for_trigger = bool(self.get_parameter('wait_for_trigger').value)
        self._trigger_pending = False
        self._abort_requested = False
        self._sweeping = False

        self._latest_points: np.ndarray | None = None   # raw, bounds applied only in _build_map
        self._accumulated_points: list[np.ndarray] = []
        self._cloud_received: bool = False
        self._new_frame: bool = False
        self._current_q: np.ndarray | None = None
        self._cb_count: int = 0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._map_pub = self.create_publisher(PointCloud2, self._output_topic, latched_qos)
        self._ik_pub  = self.create_publisher(PoseStamped, self._ik_topic, 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )

        self.create_subscription(
            PointCloud2, self._input_topic, self._point_cloud_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, self._js_topic, self._joint_state_cb, 10
        )
        if self._wait_for_trigger:
            self.create_subscription(
                Empty,
                str(self.get_parameter('trigger_topic').value),
                self._trigger_cb,
                10,
            )
            self.create_subscription(
                Empty,
                str(self.get_parameter('stop_topic').value),
                self._stop_cb,
                10,
            )

    def _stop_cb(self, _msg: Empty) -> None:
        """Ask a running sweep to give up at the next pose boundary."""
        self._trigger_pending = False
        if self._sweeping:
            self.get_logger().warning('Sweep stop requested; aborting the scan.')
            self._abort_requested = True

    def _resolve_scan_posture(self):
        """Pick the tucked scan posture: explicit joints, else a named preset."""
        override = [float(v) for v in
                    self.get_parameter('scan_tucked_joints').value]
        if len(override) == 4 and all(v == v for v in override):   # v==v -> not NaN
            self.get_logger().info(
                'Scan posture: explicit scan_tucked_joints %s' % override)
            return override

        name = str(self.get_parameter('scan_posture').value).strip().lower()
        if name not in SCAN_POSTURES:
            self.get_logger().warning(
                "Unknown scan_posture '%s' (known: %s); using 'tilted'."
                % (name, ', '.join(sorted(SCAN_POSTURES))))
            name = 'tilted'
        joints = list(SCAN_POSTURES[name])
        self.get_logger().info("Scan posture: '%s' %s" % (name, joints))
        return joints

    def _resolve_wrist_offsets(self):
        """Wrist-angle offsets for the per-station looks, clamped to the servo.

        Clamping rather than rejecting: an offset that would drive wrist_angle
        past its stop is still a useful look at the stop itself, and silently
        commanding past the limit is how you get a joint that never reports
        arrival and a sweep that hangs on settle.
        """
        raw = [float(v) for v in self.get_parameter('scan_wrist_offsets').value]
        raw = [v for v in raw if v == v]                 # v==v -> not NaN
        if not raw:
            self.get_logger().warning(
                'scan_wrist_offsets is empty; falling back to a single look.')
            raw = [0.0]

        base = float(self._scan_joints[2])
        low, high = WRIST_ANGLE_LIMITS
        offsets, clamped = [], []
        for off in raw:
            target = min(max(base + off, low), high)
            if abs(target - (base + off)) > 1e-9:

                clamped.append((off, target - base))
                off = target - base
            if not any(abs(off - kept) < 1e-9 for kept in offsets):
                offsets.append(off)
        if clamped:
            self.get_logger().warning(
                'wrist_angle offsets clamped to the joint limit [%.3f, %.3f] '
                'from base %.3f: %s'
                % (low, high, base,
                   ', '.join('%+.3f->%+.3f' % pair for pair in clamped)))

        self.get_logger().info(
            '%d look(s) per waist station, wrist_angle %s'
            % (len(offsets), [round(base + o, 3) for o in offsets]))
        return offsets

    def _resolve_waist_angles(self):
        """Waist stations to scan, clamped to the servo and de-duplicated.

        Empty means "use the default full circle", so a launch file can pass
        [] to mean "unchanged" without restating the default and letting the two
        drift apart.

        Nothing here checks that the stations actually COVER the scene. Against
        the D435i's 54.5 deg horizontal FOV, stations closer than ~45 deg apart
        overlap and anything wider leaves a blind wedge between them; the sweep
        will happily scan a sparse ring and report success.
        """
        # An empty list arrives as UNINITIALIZED, not as an empty array: neither
        # `-p scan_waist_angles:="[]"` nor a launch ParameterValue typed
        # List[float] can tell rclpy what an empty [] holds, so the declared
        # default is discarded rather than kept. Treat that as "not specified".
        try:
            value = self.get_parameter('scan_waist_angles').value or []
        except ParameterUninitializedException:
            value = []
        raw = [float(v) for v in value]
        raw = [v for v in raw if v == v]                 # v==v -> not NaN
        if not raw:
            raw = list(SCAN_WAIST_ANGLES)

        low, high = WAIST_ANGLE_LIMITS
        angles, clamped = [], []
        for angle in raw:
            target = min(max(angle, low), high)
            if abs(target - angle) > 1e-9:
                clamped.append((angle, target))
                angle = target
            if not any(abs(angle - kept) < 1e-9 for kept in angles):
                angles.append(angle)
        if clamped:
            self.get_logger().warning(
                'waist stations clamped to the joint limit [%.3f, %.3f]: %s'
                % (low, high,
                   ', '.join('%+.3f->%+.3f' % pair for pair in clamped)))

        span = max(angles) - min(angles) if len(angles) > 1 else 0.0
        self.get_logger().info(
            '%d waist station(s) spanning %.0f deg: %s'
            % (len(angles), np.degrees(span),
               [round(np.degrees(a)) for a in angles]))
        return angles

    def _station_joints(self, waist: float, offset: float) -> list[float]:
        """Full 5-joint pose for one look: a waist station plus a wrist offset."""
        shoulder, elbow, wrist_angle, wrist_rotate = self._scan_joints
        return [float(waist), float(shoulder), float(elbow),
                float(wrist_angle) + float(offset), float(wrist_rotate)]

    def _emit(self, event: str) -> None:
        """Publish a sweep lifecycle event (sweep:complete / sweep:failed)."""
        self._status_pub.publish(String(data=event))

    def _trigger_cb(self, _msg: Empty) -> None:
        """Queue a sweep. The actual scan runs on the main loop, never here --
        it drives the arm and spins the executor itself, which cannot be done
        from inside a callback that executor is already dispatching."""
        if self._trigger_pending:
            return
        self.get_logger().info('Sweep requested.')
        self._trigger_pending = True

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in rx150_kinematics.JOINT_NAMES):
            self._current_q = np.array(
                [positions[name] for name in rx150_kinematics.JOINT_NAMES], dtype=np.float64
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
        self._new_frame = True
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
        """Sweep once on startup, or idle and sweep on each trigger."""
        if not self._wait_for_trigger:
            merged = self._do_sweep()
            if merged is None:
                return 1
            self._publish_map(merged)
            self.get_logger().info(
                'Published map with %d points on %s — republishing every 2 s, '
                'Ctrl+C when done.' % (merged.shape[0], self._output_topic)
            )
            return self._republish_forever(merged)
        return self._run_triggered()

    def _run_triggered(self) -> int:
        """Idle until triggered, sweep, publish, then idle again.

        Keeps republishing the most recent map between sweeps so a late
        subscriber (RViz, the planner) still receives it -- the same reason the
        standalone path republishes.
        """
        self.get_logger().info(
            'Sweep mapper ready (triggered mode). Waiting for %s ...'
            % str(self.get_parameter('trigger_topic').value)
        )
        merged = None
        next_republish = time.monotonic() + 2.0
        while rclpy.ok():
            self._spin_for(0.2)
            if self._trigger_pending:
                self._trigger_pending = False
                result = self._do_sweep()
                self._sweeping = False

                superseded = self._trigger_pending

                if result is None:
                    aborted = self._abort_requested
                    self.get_logger().error(
                        'Sweep %s; keeping the previous map (if any).'
                        % ('aborted' if aborted else 'failed')
                    )
                    if superseded:
                        self.get_logger().warning(
                            'Not emitting sweep:%s -- a new sweep was requested '
                            'while this one was ending, and that request will '
                            'get its own result.'
                            % ('aborted' if aborted else 'failed')
                        )
                    else:
                        self._emit('sweep:aborted' if aborted else 'sweep:failed')
                else:
                    merged = result
                    self._publish_map(merged)
                    self.get_logger().info(
                        'Published map with %d points on %s.'
                        % (merged.shape[0], self._output_topic)
                    )
                    if superseded:
                        self.get_logger().warning(
                            'Not emitting sweep:complete -- a new sweep was '
                            'requested while this one was finishing, and that '
                            'request will get its own result. The map above is '
                            'still published and usable.'
                        )
                    else:
                        # Emit only after the map is on the wire, so a waiter
                        # that advances on this event is guaranteed to have it.
                        self._emit('sweep:complete')
                next_republish = time.monotonic() + 2.0
            elif merged is not None and time.monotonic() >= next_republish:
                self._publish_map(merged)
                next_republish = time.monotonic() + 2.0
        return 0

    def _republish_forever(self, merged: np.ndarray) -> int:
        # Republish periodically so any subscriber (RViz, planner) receives the
        # map regardless of its QoS durability setting.
        try:
            while rclpy.ok():
                self._spin_for(2.0)
                self._publish_map(merged)
        except KeyboardInterrupt:
            pass
        return 0

    def _do_sweep(self):
        """Run one full scan. Returns the merged cloud, or None on failure."""
        self.get_logger().info(
            'Starting sweep: %s → %s (frame %s)'
            % (self._input_topic, self._output_topic, self._frame_id)
        )
        # Start from empty, or a re-triggered sweep would merge this scan into
        # the previous one and the map would never forget moved obstacles.
        self._accumulated_points = []
        self._abort_requested = False
        self._sweeping = True

        if not self._wait_for_cloud(timeout_sec=90.0):
            self.get_logger().error(
                'No cloud on %s after 90 s. Is use_scene_point_cloud:=true?'
                % self._input_topic
            )
            return None

        # Fold into the tucked scan posture at the current waist angle first,
        # then sweep only the waist. Doing the fold as its own move keeps the
        # DLS solver on the tucked solution branch for the whole sweep.
        tucked = self._scan_joints
        self.get_logger().info(
            'Tucking arm for scan: shoulder=%.2f elbow=%.2f wrist=%.2f'
            % (tucked[0], tucked[1], tucked[2])
        )
        current_waist = float(self._current_q[0])
        self._send_pose(self._station_joints(current_waist, self._wrist_offsets[0]))
        if not self._spin_for(self._settle_sec):
            return None

        neutral = self._wrist_offsets[0]
        multi_look = len(self._wrist_offsets) > 1
        for waist in self._waist_angles:
            station = 'scan_%+.0fdeg' % np.degrees(waist)
            for index, offset in enumerate(self._wrist_offsets):
                if self._abort_requested:
                    self.get_logger().warning('Sweep aborted at %s.' % station)
                    return None
                pose_name = station if not multi_look else (
                    '%s_look%d' % (station, index + 1))
                # A full settle is only owed to the waist slew that opens the
                # station. Later looks move one light joint a few degrees.
                settle = self._settle_sec if index == 0 else self._wrist_settle_sec
                self.get_logger().info(
                    "Moving to sweep pose '%s' (wrist_angle %+.3f)"
                    % (pose_name, self._scan_joints[2] + offset))
                self._send_pose(self._station_joints(waist, offset))
                if not self._spin_for(settle):
                    return None

                before = len(self._accumulated_points)
                if not self._collect_for_pose(pose_name):
                    return None
                frames = len(self._accumulated_points) - before
                self.get_logger().info(
                    "Collected %d frames at '%s'" % (frames, pose_name))

            if multi_look and self._return_to_neutral:
                if self._abort_requested:
                    self.get_logger().warning('Sweep aborted at %s.' % station)
                    return None
                self._send_pose(self._station_joints(waist, neutral))
                if not self._spin_for(self._wrist_settle_sec):
                    return None

        if self._return_to_home:
            self.get_logger().info('Returning to home %s' % self._home_joints)
            self._send_pose(self._home_joints)
            self._spin_for(self._settle_sec)

        merged = self._build_map()
        if merged.size == 0:
            self.get_logger().error('Sweep complete but accumulated map is empty.')
            return None
        return merged

    # ------------------------------------------------------------------
    def _send_pose(self, joints: list[float]) -> None:
        """Compute full FK pose and send to the IK executor as PoseStamped."""
        xyz, rot = rx150_kinematics.forward_kinematics(np.asarray(joints, dtype=float))
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
        """Dwell at one look until it has genuinely sampled the cloud.

        A fixed wall-clock window is not enough. The camera nominally runs at
        5 Hz, but Gazebo, the depth render and YOLO all compete for the CPU and
        the cloud publisher can stall for seconds, so a fixed window may catch
        no clouds at all and leave a hole in the map.

        The window is therefore a floor, not a ceiling -- sample for at least
        sample_sec, then keep waiting up to sample_timeout_sec until
        min_frames_per_pose clouds have actually ARRIVED.

        Arrival is counted separately from what survives the crop, so a look
        pointed at genuinely empty space returns as soon as its frames land
        instead of burning the full timeout.
        """
        # Throw away whatever was already in flight before counting anything.
        # Only does something when drain_sec is non-zero; see that parameter.
        drain_deadline = time.monotonic() + self._drain_sec
        discarded = 0
        self._new_frame = False
        while time.monotonic() < drain_deadline:
            if not rclpy.ok():
                return False
            self._executor.spin_once(timeout_sec=0.05)
            if self._new_frame:
                self._new_frame = False
                discarded += 1

        start = time.monotonic()
        floor_deadline = start + self._sample_sec
        hard_deadline = start + max(self._sample_timeout_sec, self._sample_sec)
        received = 0
        frames = 0
        last_pts = None
        self._new_frame = False
        while time.monotonic() < hard_deadline:
            if not rclpy.ok():
                return False
            if time.monotonic() >= floor_deadline and received >= self._min_frames:
                break
            self._executor.spin_once(timeout_sec=0.1)
            if not self._new_frame:
                continue
            self._new_frame = False
            received += 1
            if self._latest_points is not None and self._latest_points.size > 0:
                # Crop per-frame so accumulation stays small
                cropped = self._crop_points(self._latest_points)
                if cropped.size == 0:
                    continue
                self._accumulated_points.append(cropped.copy())
                last_pts = self._latest_points
                frames += 1
        if received == 0:
            self.get_logger().warning(
                "No cloud arrived at '%s' in %.1f s -- the camera or the cloud "
                'pipeline is stalled, not the scene being empty.'
                % (pose_name, time.monotonic() - start))
        elif frames == 0:
            self.get_logger().info(
                "'%s': %d cloud(s), nothing inside the bounds (empty direction)."
                % (pose_name, received))
        else:
            cropped = self._crop_points(last_pts) if last_pts is not None else np.empty((0, 3))
            self.get_logger().info(
                "'%s': %d/%d frames kept (%d drained), %d raw pts  →  %d in-bounds  "
                "(x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f])"
                % (pose_name, frames, received, discarded,
                   last_pts.shape[0] if last_pts is not None else 0, cropped.shape[0],
                   self._bounds[0, 0], self._bounds[0, 1],
                   self._bounds[1, 0], self._bounds[1, 1],
                   self._bounds[2, 0], self._bounds[2, 1])
            )
        return True

    def _build_map(self) -> np.ndarray:
        """merge -> crop -> voxel -> statistical outliers -> radius outliers.

        Crop runs BEFORE the outlier filters, unlike the vision-side node this
        filtering came from. Both Open3D filters do nearest-neighbour searches,
        so throwing away the floor and everything outside the workspace first
        makes them operate on a small fraction of the points. On a real sweep
        that is the difference between a fraction of a second and several.
        """
        if not self._accumulated_points:
            return np.empty((0, 3), dtype=np.float32)
        merged = np.vstack(self._accumulated_points).astype(np.float32, copy=False)

        cropped = self._crop_points(merged)
        if cropped.size == 0:
            return cropped

        voxel_index = np.floor(cropped / self._voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_index, axis=0, return_index=True)
        points = cropped[np.sort(unique_idx)]
        self.get_logger().info(
            'Map: %d merged -> %d in bounds -> %d after voxel (%.3f m).'
            % (merged.shape[0], cropped.shape[0], points.shape[0], self._voxel_size)
        )
        return self._remove_outliers(points)

    def _remove_outliers(self, points: np.ndarray) -> np.ndarray:
        """Statistical + radius outlier removal (Open3D).

        Ported from arm_perception/mapping_node.py so this node handles real
        RealSense noise -- flying pixels at depth discontinuities and speckle,
        which a voxel grid happily preserves as solid-looking obstacles. A
        handful of those in the workspace is enough to make the planner refuse
        an otherwise clear path.

        Both stages are no-ops unless enabled, and the node degrades to
        crop+voxel if Open3D is missing rather than failing the sweep.
        """
        if points.shape[0] == 0 or o3d is None:
            return points
        if not (self._stat_enabled or self._radius_enabled):
            return points

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))

        if self._stat_enabled:
            before = len(cloud.points)
            cloud, _ = cloud.remove_statistical_outlier(
                nb_neighbors=self._stat_neighbors, std_ratio=self._stat_std_ratio
            )
            self.get_logger().info(
                'Statistical outlier removal (nb_neighbors=%d, std_ratio=%.2f): '
                '%d -> %d points (%d removed).'
                % (self._stat_neighbors, self._stat_std_ratio,
                   before, len(cloud.points), before - len(cloud.points))
            )

        if self._radius_enabled:
            before = len(cloud.points)
            cloud, _ = cloud.remove_radius_outlier(
                nb_points=self._radius_min_neighbors, radius=self._radius_size
            )
            self.get_logger().info(
                'Radius outlier removal (radius=%.3f, min_neighbors=%d): '
                '%d -> %d points (%d removed).'
                % (self._radius_size, self._radius_min_neighbors,
                   before, len(cloud.points), before - len(cloud.points))
            )

        filtered = np.asarray(cloud.points, dtype=np.float32)
        if filtered.shape[0] == 0:
            # Publishing an empty map would silently disarm every obstacle check.
            self.get_logger().error(
                'Outlier filtering removed EVERY point; keeping the unfiltered '
                'map instead. Loosen statistical_std_ratio / '
                'radius_outlier_min_neighbors.'
            )
            return points
        return filtered

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
