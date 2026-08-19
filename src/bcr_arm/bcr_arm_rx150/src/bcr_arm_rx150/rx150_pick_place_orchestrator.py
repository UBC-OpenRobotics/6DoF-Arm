#!/usr/bin/env python3

"""RX-150 pick-and-place orchestrator (the "conductor").

Runs the whole cup pick-and-place mission end to end by issuing the *same*
commands you send by hand -- a ``PointStamped`` on ``/cartesian_target``, a
``String`` on ``/rx150/gripper_command``, a home pose -- just sequenced
automatically, each step waiting for the previous one to actually finish.


The orchestrator acts as a simplified planner, only decides *what* to send and *when*.

Sequencing (the crux): the motion nodes emit lifecycle **events** on
``/motion/status`` (``std_msgs/String``), and the orchestrator waits for a
terminal one before advancing:

  success : ``path:complete``   (Cartesian executor finished)
            ``joint:complete``  (joint/RRT executor finished; also used for home)
  failure : ``path:aborted`` / ``joint:aborted``  (executor gave up)
            ``planner:no_path`` (planner's fail-safe -- fires immediately, so we
                                 fail fast instead of waiting on a timeout)

``move_timeout_sec`` remains only as a **backstop** in case an event is ever
lost, so the mission can never hang. This is what makes the sequence safe: if the
planner cannot find a whole-body path it emits ``planner:no_path``, the
orchestrator aborts, and we never, say, close the gripper on empty air. Home is
routed through the joint executor (a one-waypoint joint path) so it, too, reports
completion the same way rather than needing a separate arrival check.

``/rx150/joint_states`` is still consumed, but only to compute the relative
"lift" targets (current end-effector + dz), not for arrival.

TODO: Connect Vision: see the VISION CONTRACT block below and
``vision_placeholder`` for a stub that lets the whole mission run today.
"""

import argparse
import time

import numpy as np
import rclpy
from bcr_arm_common import rx150_kinematics
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── VISION CONTRACT (orchestrator ⇄ vision node) ─────────────────────────────
#
# REQUEST  (orchestrator → vision)
#   topic : /vision/find_request        std_msgs/String
#   data  : "cup"   -> locate the cup to pick
#           "goal"  -> locate the drop-off / goal location
#
# RESPONSE (vision → orchestrator)
#   topic : /vision/object_point        geometry_msgs/PointStamped
#   header.frame_id : "rx150/base_link"   ← MUST be the arm base frame; this is
#                     the SAME vector you would type into /cartesian_target
#   header.stamp    : capture time (used to reject stale/previous detections)
#   point.{x,y,z}   : object position in metres, base frame
#
# One request → one fresh response. Vision owns *how* it finds things; we only
# depend on the returned point being reachable-ish and in metres/base_link.
# Until vision is ready, `vision_placeholder` answers with a canned point.
# ─────────────────────────────────────────────────────────────────────────────


# Terminal /motion/status events the orchestrator waits on. Success advances the
# mission; failure aborts it. Any other (non-terminal) token is ignored.
_MOTION_SUCCESS = {'path:complete', 'joint:complete'}
_MOTION_FAILURE = {
    'path:aborted',
    'joint:aborted',
    'planner:no_path',
    'ik:tilt_exceeded',
}


class Rx150PickPlaceOrchestrator(Node):
    def __init__(self):
        super().__init__('rx150_pick_place_orchestrator')

        # Topics -- all match what the manual workflow / existing stack use.
        self.declare_parameter('planning_frame', 'rx150/base_link')
        self.declare_parameter('cartesian_target_topic', '/cartesian_target')
        self.declare_parameter('gripper_command_topic', '/rx150/gripper_command')
        self.declare_parameter('planned_joint_path_topic', '/planned_joint_path')
        self.declare_parameter('status_topic', '/motion/status')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('point_cloud_topic', '/planning/point_cloud')
        self.declare_parameter('sweep_request_topic', '/sweep/start')
        self.declare_parameter('vision_request_topic', '/vision/find_request')
        self.declare_parameter('carry_level_topic', '/motion/carry_level')
        # Gripper actions wait for the arm to actually stop first: grabbing or
        # releasing mid-deceleration knocks the cup. Settled = every joint below
        # settle_speed_rad_s for settle_dwell_sec.
        self.declare_parameter('settle_speed_rad_s', 0.02)
        self.declare_parameter('settle_dwell_sec', 0.3)
        self.declare_parameter('settle_timeout_sec', 4.0)
        # Hold the gripper level on the carrying moves (phases 5 and 7). Off by
        # default -- the path executor's own launch param decides otherwise.
        self.declare_parameter('carry_level', False)
        self.declare_parameter('vision_response_topic', '/vision/object_point')
        self.declare_parameter('start_topic', '/mission/start')
        self.declare_parameter('stop_topic', '/mission/stop')
        self.declare_parameter('restart_topic', '/mission/restart')
        # Broadcast to every executor to drop whatever it is running, so a stop
        # halts the arm now rather than at the end of the current move.
        self.declare_parameter('cancel_topic', '/motion/cancel')

        # Behaviour.
        self.declare_parameter('autostart', False)
        self.declare_parameter('home_joints', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('home_on_abort', True)

        # Backstop timeout: how long to wait for a /motion/status event before
        # giving up on a move (only fires if an event is lost -- events are the
        # primary signal). Generous, since slow hardware moves are fine.
        self.declare_parameter('move_timeout_sec', 60.0)

        # Gripper. Both shipped launches run the gripper controller with
        # command_units:=normalized, so a numeric token here is 0.0 (fully
        # closed) .. 1.0 (fully open) and means the same openness in sim and on
        # hardware. 'open'/'close' are still accepted as the named extremes.
        #
        # pregrasp_value is sent at the hover point *before* descending onto the
        # cup: the fingers spawn at their closed limit (the rx150 URDF gives
        # left_finger a 0.015..0.037 m range and nothing sets an initial
        # position, so Gazebo settles it at 0.015 = shut). Without this the
        # phase-4 'close' is a no-op onto an already-shut gripper.
        self.declare_parameter('pregrasp_value', 'open')
        # PLACEHOLDER: how far to close on the cup. Partial, not fully shut --
        # squeezing to 0.0 on a real cup either crushes it or stalls the servo.
        # Tune to the actual cup before trusting it (there is no cup in the sim
        # world to calibrate against, so this number is a guess by construction).
        self.declare_parameter('grasp_value', '0.3')
        self.declare_parameter('release_value', 'open')
        self.declare_parameter('grasp_settle_sec', 1.5)

        # Geometry.
        self.declare_parameter('lift_dz', 0.06)              # m, raise carried cup
        self.declare_parameter('clearance_dz', 0.03)         # m, back off after release
        self.declare_parameter('place_height', float('nan'))  # override goal z if set
        # Approach shape: hover this far directly above a grasp/place point and
        # descend onto it, instead of driving straight in. 0.0 restores the old
        # straight-in behaviour. See _descend_onto for why overhead and not a
        # standoff toward the base.
        self.declare_parameter('approach_height', 0.10)

        # Timeouts.
        self.declare_parameter('sweep_timeout_sec', 90.0)
        self.declare_parameter('vision_timeout_sec', 10.0)

        # Debugging aid: pause between phases so each one is easy to watch and
        # read in the log. Set to 0.0 for normal (continuous) running.
        self.declare_parameter('phase_delay_sec', 2.0)

        gp = self.get_parameter
        self._frame = str(gp('planning_frame').value)
        self._autostart = bool(gp('autostart').value)
        self._home_joints = [float(v) for v in gp('home_joints').value]
        self._home_on_abort = bool(gp('home_on_abort').value)
        self._move_timeout_sec = float(gp('move_timeout_sec').value)
        self._pregrasp_value = str(gp('pregrasp_value').value)
        self._grasp_value = str(gp('grasp_value').value)
        self._release_value = str(gp('release_value').value)
        self._grasp_settle_sec = float(gp('grasp_settle_sec').value)
        self._lift_dz = float(gp('lift_dz').value)
        self._clearance_dz = float(gp('clearance_dz').value)
        self._place_height = float(gp('place_height').value)
        self._approach_height = max(0.0, float(gp('approach_height').value))
        self._sweep_timeout_sec = float(gp('sweep_timeout_sec').value)
        self._vision_timeout_sec = float(gp('vision_timeout_sec').value)
        self._carry_level = bool(gp('carry_level').value)
        self._settle_speed_rad_s = float(gp('settle_speed_rad_s').value)
        self._settle_dwell_sec = float(gp('settle_dwell_sec').value)
        self._settle_timeout_sec = float(gp('settle_timeout_sec').value)
        self._phase_delay_sec = max(0.0, float(gp('phase_delay_sec').value))

        # Observed state.
        self._current_q = None
        self._last_joint_time = None
        self._joint_speed = None
        self._cloud_points = 0
        self._vision_result = None
        self._start_received = False
        self._stop_requested = False
        self._restart_requested = False
        self._last_status = None  # latest terminal /motion/status token

        # Publishers (command channels -- same as manual use).
        self._target_pub = self.create_publisher(
            PointStamped, str(gp('cartesian_target_topic').value), 10
        )
        self._gripper_pub = self.create_publisher(
            String, str(gp('gripper_command_topic').value), 10
        )
        # Home is sent as a one-waypoint joint path so the joint executor drives
        # it and reports joint:complete -- same event channel as every other move.
        self._joint_path_pub = self.create_publisher(
            JointTrajectory, str(gp('planned_joint_path_topic').value), 10
        )
        self._sweep_pub = self.create_publisher(
            Empty, str(gp('sweep_request_topic').value), 10
        )
        self._vision_req_pub = self.create_publisher(
            String, str(gp('vision_request_topic').value), 10
        )
        # Level-carry is enabled only while the cup is actually in the gripper
        # (phases 5 and 7). The empty-gripper approach and retreat stay
        # unconstrained so they keep full reach on this 5-DOF arm.
        self._carry_level_pub = self.create_publisher(
            Bool, str(gp('carry_level_topic').value), 10
        )
        self._cancel_pub = self.create_publisher(
            Empty, str(gp('cancel_topic').value), 10
        )

        # Subscriptions (the only feedback we have).
        self.create_subscription(
            JointState, str(gp('joint_state_topic').value), self._joint_state_cb, 10
        )
        self.create_subscription(
            PointCloud2, str(gp('point_cloud_topic').value), self._cloud_cb, 10
        )
        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10
        )
        self.create_subscription(
            PointStamped, str(gp('vision_response_topic').value),
            self._vision_cb, 10
        )
        self._start_topic = str(gp('start_topic').value)
        self._stop_topic = str(gp('stop_topic').value)
        self._restart_topic = str(gp('restart_topic').value)
        self.create_subscription(
            Empty, self._start_topic, self._start_cb, 10
        )
        self.create_subscription(
            Empty, self._stop_topic, self._stop_cb, 10
        )
        self.create_subscription(
            Empty, self._restart_topic, self._restart_cb, 10
        )

    # -- callbacks ----------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in rx150_kinematics.JOINT_NAMES):
            return
        q = np.array(
            [positions[name] for name in rx150_kinematics.JOINT_NAMES], dtype=float
        )
        # Track how fast the arm is actually moving. path:complete only means the
        # tool tip entered the tolerance sphere -- the arm is usually still
        # decelerating at that moment -- so gripper actions wait on this instead.
        # Finite-differenced from position so it works whether or not the driver
        # populates JointState.velocity.
        now = time.monotonic()
        if self._current_q is not None and self._last_joint_time is not None:
            dt = now - self._last_joint_time
            if dt > 1e-3:
                self._joint_speed = float(np.max(np.abs(q - self._current_q)) / dt)
        self._last_joint_time = now
        self._current_q = q

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._cloud_points = int(msg.width) * int(msg.height)

    def _status_cb(self, msg: String) -> None:
        token = msg.data.strip()
        if token in _MOTION_SUCCESS or token in _MOTION_FAILURE:
            self._last_status = token

    def _vision_cb(self, msg: PointStamped) -> None:
        self._vision_result = msg

    def _start_cb(self, _msg: Empty) -> None:
        self._start_received = True

    def _stop_cb(self, _msg: Empty) -> None:
        """Stop the running mission and halt the arm immediately.

        Cancelling here rather than at the next phase boundary is the whole
        point: the interruptible waits below unwind the mission, but only the
        cancel broadcast actually stops an arm that is mid-path.
        """
        if self._stop_requested:
            return
        self.get_logger().warning('STOP requested.')
        self._stop_requested = True
        self._cancel_pub.publish(Empty())

    def _restart_cb(self, _msg: Empty) -> None:
        """Stop the running mission, then start a fresh one straight away.

        Also arms the start flag, so restart works from *idle* too -- otherwise
        the idle wait (which is deliberately not interruptible) would never wake
        and the request would be silently swallowed.
        """
        self.get_logger().warning('RESTART requested.')
        self._restart_requested = True
        self._start_received = True
        if not self._stop_requested:
            self._stop_requested = True
            self._cancel_pub.publish(Empty())

    # -- low-level helpers --------------------------------------------------
    def _spin_for(self, seconds: float, interruptible: bool = True) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if interruptible and self._stop_requested:
                return

    def _wait_until(
        self,
        predicate,
        timeout_sec: float,
        dwell_sec: float = 0.0,
        interruptible: bool = True,
    ):
        """Spin until predicate() holds continuously for dwell_sec, or timeout.

        Returns False early when a stop is requested, which is what unwinds an
        in-flight mission: every blocking wait in a phase gives up, the phase
        reports failure, and run_once() falls through to _abort(). Pass
        interruptible=False for waits that must survive a stop (the idle wait
        for the next start, and the recovery move home).
        """
        start = time.monotonic()
        held_since = None
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if interruptible and self._stop_requested:
                return False
            if predicate():
                if held_since is None:
                    held_since = time.monotonic()
                if time.monotonic() - held_since >= dwell_sec:
                    return True
            else:
                held_since = None
        return False

    def _current_ee(self):
        if self._current_q is None:
            return None
        xyz, _rot = rx150_kinematics.forward_kinematics(self._current_q)
        return np.asarray(xyz, dtype=float)

    # -- command primitives (same topics a human uses) ----------------------
    def _send_cartesian(self, xyz) -> None:
        if self._stopping():
            return
        self._last_status = None
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.point.x, msg.point.y, msg.point.z = (
            float(xyz[0]), float(xyz[1]), float(xyz[2])
        )
        self._target_pub.publish(msg)
        self.get_logger().info(
            'Sent target [%.3f, %.3f, %.3f] on %s'
            % (xyz[0], xyz[1], xyz[2], self._target_pub.topic_name)
        )

    def _stopping(self) -> bool:
        """True while a stop is unwinding the mission.

        Every command primitive checks this. Bailing out of the *waits* alone is
        not enough: the phases after a stop would still publish their targets,
        those would be planned and queued downstream, and the recovery move home
        would then execute a stale target instead.
        """
        return self._stop_requested

    def _move_to(self, xyz, label: str) -> bool:
        """Send one Cartesian target and wait for its terminal motion event."""
        self._send_cartesian(xyz)
        return self._wait_for_motion(label)

    def _actuate_gripper(self, token: str, label: str) -> None:
        """Stop the arm, command the gripper, and let the fingers finish moving.

        Always goes through the settle check: acting mid-deceleration knocks the
        cup, and the fingers need grasp_settle_sec to actually travel (the
        command is a position setpoint, not a completion event -- the gripper
        reports nothing back on /motion/status).
        """
        self._wait_until_stationary(label)
        self._send_gripper(token)
        self._spin_for(self._grasp_settle_sec)

    def _descend_onto(self, target, label: str, hover_gripper=None) -> bool:
        """Reach `target` from directly above it instead of driving straight in.

        The descent needs no special planner support: hover and target share an
        XY grid cell, so A* returns a single cell and the planner emits a
        straight interpolated segment between them.
        """
        target = np.asarray(target, dtype=float)
        if self._approach_height <= 0.0:
            return self._move_to(target, label)

        hover = target + np.array([0.0, 0.0, self._approach_height])
        self.get_logger().info(
            '%s: approaching via hover %.3f m above the target.'
            % (label, self._approach_height)
        )
        if not self._move_to(hover, '%s-hover' % label):
            return False

        if hover_gripper:
            # Open here, not at the bottom: the fingers have to already be clear
            # of the cup before the descent puts them around it.
            self.get_logger().info(
                "%s: setting gripper to '%s' at hover, before descending."
                % (label, hover_gripper)
            )
            self._actuate_gripper(hover_gripper, '%s pre-grasp open' % label)
        else:
            # Settle anyway so the descent starts from the hover point rather
            # than from wherever the arm is mid-deceleration.
            self._wait_until_stationary('%s descent' % label)

        self.get_logger().info('%s: descending straight down onto the target.' % label)
        return self._move_to(target, label)

    def _send_gripper(self, token: str) -> None:
        if self._stopping():
            return
        self._gripper_pub.publish(String(data=str(token)))
        self.get_logger().info("Sent gripper command '%s'" % token)

    def _phase(self, text: str) -> None:
        """Announce a mission phase: pause, rule off, then the header.

        The pause (phase_delay_sec) is a debugging aid -- it separates phases in
        time so the arm visibly finishes one before starting the next. Set it to
        0 for normal running.
        """
        if self._phase_delay_sec > 0.0:
            self._spin_for(self._phase_delay_sec)
        self.get_logger().info('-' * 64)
        self.get_logger().info(text)
        self.get_logger().info('-' * 64)

    def _wait_until_stationary(self, label: str) -> None:
        self._joint_speed = None
        settled = self._wait_until(
            lambda: self._joint_speed is not None
            and self._joint_speed <= self._settle_speed_rad_s,
            timeout_sec=self._settle_timeout_sec,
            dwell_sec=self._settle_dwell_sec,
        )
        if settled:
            self.get_logger().info('Arm settled before %s.' % label)
        else:
            self.get_logger().warning(
                'Arm still moving after %.1f s; proceeding with %s anyway '
                '(last speed %s rad/s).'
                % (
                    self._settle_timeout_sec,
                    label,
                    'unknown' if self._joint_speed is None
                    else '%.3f' % self._joint_speed,
                )
            )

    def _set_carry_level(self, enabled: bool) -> None:
        """Turn the path executor's level-carry constraint on/off.

        No-op unless the mission was launched with carry_level, so the default
        behaviour is unchanged. Takes effect on the next planned path.
        """
        if not self._carry_level:
            return
        self._carry_level_pub.publish(Bool(data=bool(enabled)))
        self.get_logger().info(
            'Level-carry %s for the next move(s).'
            % ('ON (cup in gripper)' if enabled else 'OFF (gripper empty)')
        )

    def _send_home(self) -> None:
        # Route home as a one-waypoint joint path through the joint executor, so
        # it drives the move and reports joint:complete -- same event channel as
        # every other motion (no separate arrival check needed).
        if self._stopping():
            return
        self._last_status = None
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(rx150_kinematics.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self._home_joints]
        traj.points = [point]
        self._joint_path_pub.publish(traj)
        self.get_logger().info('Sent home joint path %s' % self._home_joints)

    # -- phase waits --------------------------------------------------------
    def _wait_for_motion(self, label: str) -> bool:
        """Wait for a terminal /motion/status event for the move just sent.

        Success token -> True, failure token -> False. ``move_timeout_sec`` is a
        backstop only (a lost event should never hang the mission); it is not the
        primary signal. Call *after* the command that starts the move, which
        cleared ``_last_status``.
        """
        ok = self._wait_until(
            lambda: self._last_status is not None, self._move_timeout_sec
        )
        if self._stop_requested:
            self.get_logger().warning("Move '%s' abandoned: stop requested." % label)
            return False
        if not ok:
            self.get_logger().error(
                "No motion event for '%s' within %.0f s (backstop timeout)."
                % (label, self._move_timeout_sec)
            )
            return False
        success = self._last_status in _MOTION_SUCCESS
        self.get_logger().info(
            "Motion '%s' -> %s" % (label, self._last_status)
        )
        return success

    def _wait_for_cloud(self) -> bool:
        return self._wait_until(
            lambda: self._cloud_points > 0, self._sweep_timeout_sec
        )

    def _request_vision(self, kind: str):
        """Ask vision for 'cup'/'goal'; return xyz np.array or None."""
        self._vision_result = None  # clear so we only accept a fresh reply
        self._vision_req_pub.publish(String(data=kind))
        self.get_logger().info("Requested vision target '%s'" % kind)
        ok = self._wait_until(
            lambda: self._vision_result is not None, self._vision_timeout_sec
        )
        if not ok or self._vision_result is None:
            return None
        pt = self._vision_result.point
        frame = self._vision_result.header.frame_id
        if frame and frame != self._frame:
            # Vision should publish in base_link; warn but trust the numbers so a
            # frame-name mismatch doesn't silently misplace the arm.
            self.get_logger().warning(
                "Vision point frame '%s' != planning frame '%s'; using values "
                'as-is. Agree on the frame with the vision owner.'
                % (frame, self._frame)
            )
        result = np.array([pt.x, pt.y, pt.z], dtype=float)
        self.get_logger().info(
            "Vision '%s' -> [%.3f, %.3f, %.3f]" % (kind, result[0], result[1], result[2])
        )
        return result

    def _wait_for_start(self) -> bool:
        if self._autostart:
            self._autostart = False
            return True
        if not self._start_received:
            self.get_logger().info(
                'Idle. Waiting for mission start on %s (or %s to run again, %s to '
                'stop).' % (self._start_topic, self._restart_topic, self._stop_topic)
            )
        return self._wait_until(
            lambda: self._start_received, timeout_sec=1e9, interruptible=False
        )

    def _abort(self, reason: str) -> int:
        """Unwind the mission, park the arm, and return an exit code.

        A stop request lands here too (every wait in the phase bailed out), but
        it is an operator action, not a failure -- so it is reported as such.
        """
        stopped = self._stop_requested
        # Clear before doing anything else: _stopping() guards every command
        # primitive, so the recovery move below would be silently dropped.
        self._stop_requested = False
        log = self.get_logger().warning if stopped else self.get_logger().error
        log('=' * 64)
        log('MISSION %s' % ('STOPPED by request.' if stopped else 'ABORTED: %s' % reason))
        log('=' * 64)
        # Clear the constraint even if we aborted mid-carry, so it cannot leak
        # into the recovery move or a later re-run of the mission.
        self._set_carry_level(False)
        if self._home_on_abort:
            # Second cancel: planning is asynchronous, so a target sent just
            # before the stop can still land a path on the executors after the
            # first cancel. Flush it, or home would queue behind it.
            self._cancel_pub.publish(Empty())
            self._spin_for(0.2, interruptible=False)
            self._last_status = None
            self.get_logger().info('Sending arm home.')
            self._send_home()
            self._wait_for_motion('home')
        return 2 if stopped else 1

    # -- the mission --------------------------------------------------------
    def run(self) -> int:
        """Idle -> run one mission -> idle, repeating until the node is killed.

        The mission used to be one-shot: run() returned and the process exited,
        so re-running meant relaunching the whole stack. Now each cycle resets
        its own state and returns to the idle wait, and a restart request feeds
        straight back into the next cycle without passing through idle.
        """
        while rclpy.ok():
            if not self._wait_for_start():
                return 130
            self._reset_for_new_mission()
            self._run_once()
            if self._restart_requested:
                self._restart_requested = False
                self.get_logger().info('Restarting mission immediately.')
                self._start_received = True
            else:
                self._start_received = False
        return 0

    def _reset_for_new_mission(self) -> None:
        """Clear every per-mission scrap of state so cycle N+1 is not cycle N.

        _cloud_points in particular: zeroing it makes phase 1 wait for a cloud
        published *after* this cycle's sweep request, so the map is genuinely
        regenerated each run rather than satisfied instantly by the latched one
        left over from last time.
        """
        self._stop_requested = False
        # Consumed here, at the moment the mission actually begins. Leaving it
        # set would make run()'s post-mission check fire on a restart that this
        # cycle already satisfied, looping missions forever.
        self._restart_requested = False
        self._last_status = None
        self._vision_result = None
        self._cloud_points = 0
        self._joint_speed = None

    def _run_once(self) -> int:
        self.get_logger().info('=' * 64)
        self.get_logger().info(
            'MISSION START (level-carry %s, %.1fs between phases)'
            % ('ON' if self._carry_level else 'off', self._phase_delay_sec)
        )
        self.get_logger().info('=' * 64)

        # Phase 1: sweep -> obstacle map on /planning/point_cloud.
        self._phase('[1/10] Sweep: requesting obstacle map.')
        self._sweep_pub.publish(Empty())
        if not self._wait_for_cloud():
            return self._abort('no obstacle map on point-cloud topic after sweep')
        self.get_logger().info('Obstacle map present (%d points).' % self._cloud_points)

        # Phase 2: find the cup.
        self._phase('[2/10] Vision: locate cup.')
        cup = self._request_vision('cup')
        if cup is None:
            return self._abort('vision returned no cup')

        # Phase 3: move to the cup.
        # Level goes on BEFORE the approach, not after the grasp: arriving at the
        # cup already level means the gripper closes in its final orientation and
        # nothing has to rotate afterwards. Enabling it only at phase 4 would grasp
        # the cup at whatever pitch the approach happened to end on, then twist it
        # upright while holding it.
        self._phase('[3/10] Move to cup (hover, open gripper, then descend).')
        self._set_carry_level(True)
        if not self._descend_onto(cup, 'cup', hover_gripper=self._pregrasp_value):
            return self._abort('could not reach cup (planner refused or timed out)')

        # Phase 4: grasp. Level is already on from the approach, so the gripper
        # closes in the same orientation it will carry.
        self._phase('[4/10] Grasp (close gripper to %s).' % self._grasp_value)
        self._actuate_gripper(self._grasp_value, 'grasp')

        # Phase 5: lift the cup.
        self._phase('[5/10] Lift cup.')
        ee = self._current_ee()
        if ee is None:
            return self._abort('no joint state; cannot compute lift')
        lift_target = ee + np.array([0.0, 0.0, self._lift_dz])
        self._send_cartesian(lift_target)
        if not self._wait_for_motion('lift'):
            return self._abort('could not lift cup')

        # Phase 6: find the goal.
        self._phase('[6/10] Vision: locate goal.')
        goal = self._request_vision('goal')
        if goal is None:
            return self._abort('vision returned no goal')

        # Phase 7: move to the goal (optionally at a fixed place height).
        self._phase('[7/10] Move to goal (hover above, then descend).')
        goal_target = goal.copy()
        if not np.isnan(self._place_height):
            goal_target[2] = self._place_height
        # Same shape as the pick: the cup is set down by lowering onto the spot,
        # not by sliding into it across the table.
        if not self._descend_onto(goal_target, 'goal'):
            return self._abort('could not reach goal')

        # Phase 8: release. Cup is out of the gripper, so drop the level
        # constraint -- the retreat and home moves get full reach back.
        self._phase('[8/10] Release (gripper open).')
        self._actuate_gripper(self._release_value, 'release')
        self._set_carry_level(False)

        # Phase 9: lift clear of the placed cup (best-effort).
        self._phase('[9/10] Lift clear of cup.')
        ee = self._current_ee()
        if ee is not None:
            clear_target = ee + np.array([0.0, 0.0, self._clearance_dz])
            self._send_cartesian(clear_target)
            self._wait_for_motion('lift-clear')

        # Phase 10: home.
        self._phase('[10/10] Return home.')
        self._send_home()
        if not self._wait_for_motion('home'):
            self.get_logger().warning('Home not confirmed within timeout.')

        self.get_logger().info('=' * 64)
        self.get_logger().info('MISSION COMPLETE.')
        self.get_logger().info('=' * 64)
        return 0


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='RX-150 pick-and-place mission.')
    parser.add_argument(
        '--autostart', action='store_true',
        help='start immediately instead of waiting for /mission/start',
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Rx150PickPlaceOrchestrator()
    if args.autostart:
        node._autostart = True
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
