#!/usr/bin/env python3
"""Play a fixed list of recorded joint poses, opening and closing the gripper.

This is the no-vision, no-planner path: the poses were captured by hand off
/rx150/joint_states, so nothing here solves IK, looks at a camera, or checks
for obstacles. It replays exactly what was recorded, which makes it the right
tool for proving the arm and gripper work end to end before trusting anything
that computes a target.

    THE ARM MOVES THE INSTANT THIS STARTS (unless autostart is false).
    It replays recorded poses with NO collision checking. Clear the workspace.

Each move is published as a one-waypoint JointTrajectory on
/planned_joint_path, so rx150_joint_waypoint_executor drives it and reports
joint:complete on /motion/status -- the same channel every other move in this
stack uses. Gripper steps go to rx150_gripper_controller as plain tokens.

Recorded poses carry all 8 names (5 arm joints + gripper + both fingers). Only
the 5 arm joints are commanded here; the gripper is driven by its own
controller, because the finger joints are a mimic of the gripper servo and
commanding them as trajectory points fights that controller.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150.rx150_dls_solver import DlsSolverConfig


# The rest pose the arm folds into, from interbotix_xsarm_control/config/
# rx150.yaml `sleep_positions`. This is the "flopped over itself" posture the
# sequence starts and ends at.
#
# The elbow value is NOT reachable on this arm. Folded, the forearm rests on the
# arm and the elbow stops around 1.61, about 3.5 deg short of the 1.55 below,
# and stays there stalled against the contact. That is why this launch runs the
# waypoint executor at a looser tolerance (see waypoint_joint_tolerance in
# rx150_joint_sequence.launch.py) -- at the executor's default 0.05 the
# sequence aborts on step 1 having never moved past home.
#
# Commanding an angle the arm cannot reach leaves the elbow servo pushing
# against a hard stop for as long as it holds the pose, which draws current and
# heats it. If this pose is going to be used often, re-record it off the real
# arm the way the cup poses were, rather than taking it from the config file.
SLEEP = [0.0, -1.80, 1.55, 0.8, 0.0]

# The 'at cup' pose is a RE-RECORDING. The first one put the tool 0.14 m from
# the base -- the folded rest posture with the waist turned, captured while the
# arm was resting rather than reaching -- so the arm drove backwards to reach
# it. The replacement sits 0.272 m out, next to the pre-grasp at 0.277 m, which
# is the small descent it was meant to be.
#
# It commands wrist_angle -1.8024, which is 3.3 deg past the -1.7453 (-100 deg)
# lower limit in both the URDF and the solver, so the relay clips it. That clip
# costs about 9 mm of tool height -- more than the 9 mm the move itself covers
# -- so the grasp sits roughly a centimetre high. Re-record this pose with the
# arm under power to get rid of that.
#
# 'forward 30 mm' is the only pose here that was NOT recorded off the arm. It
# is the 'at cup' tool pose moved 30 mm straight out from the base, level, with
# the wrist orientation held: solved with the shared DLS solver, converged to
# 0.1 mm and 0.0006 rad, every joint inside its limit. Only shoulder, elbow and
# wrist_angle move; waist and wrist_rotate are untouched, so the approach
# direction is unchanged.
#
# The gripper now closes HERE rather than at 'at cup', and the cup is set back
# down here too -- releasing 30 mm short would drop it in the wrong place. The
# 'back off 30 mm' step afterwards retreats to 'at cup' before folding home, so
# the gripper withdraws from the cup instead of sweeping through it.
#
# Recorded poses, arm joints only, in the order they are visited.
#   (label, [waist, shoulder, elbow, wrist_angle, wrist_rotate], gripper_after)
# gripper_after runs AFTER the arm reports arrival, so the fingers never move
# while the arm is still travelling.
SEQUENCE = [
    ('home (folded)',   SLEEP,                                                           'open'),
    ('pre-grasp',       [0.5967185497283936, 0.7531846165657043, 0.9986215233802795,
                         -1.7057867050170898, -0.11965050548315048],                     None),
    ('at cup',          [0.6043884754180908, 0.7992039918899536, 1.0078253746032715,
                         -1.8024275302886963, -0.02147573232650757],                     None),
    ('forward 30 mm',   [0.6043884754, 0.7710721298, 0.8214471397,
                         -1.5314454611, -0.0214757323],                                 'close'),
    ('cup in air',      [0.5399612784385681, -0.1533980816602707, 0.552233099937439,
                         -0.38196122646331787, -0.003067961661145091],                   None),
    ('cup back down',   [0.6043884754, 0.7710721298, 0.8214471397,
                         -1.5314454611, -0.0214757323],                                 'open'),
    ('back off 30 mm',  [0.6043884754180908, 0.7992039918899536, 1.0078253746032715,
                         -1.8024275302886963, -0.02147573232650757],                     None),
    ('home (folded)',   SLEEP,                                                           None),
]

_SUCCESS = {'joint:complete'}
_FAILURE = {'joint:aborted'}


class Rx150JointSequence(Node):

    def __init__(self):
        super().__init__('rx150_joint_sequence')

        self.declare_parameter('joint_path_topic', '/planned_joint_path')
        self.declare_parameter('status_topic', '/motion/status')
        self.declare_parameter('gripper_command_topic', '/rx150/gripper_command')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('autostart', True)
        # With autostart false the node waits here instead. Without a trigger
        # that flag would just be a way to make the node do nothing forever.
        self.declare_parameter('start_topic', '/sequence/start')
        # Per-move deadline. The executor reports joint:aborted on its own if a
        # waypoint stalls; this only catches a move that never reports at all.
        self.declare_parameter('move_timeout_sec', 20.0)
        # Let the fingers finish before the next arm move starts. The gripper
        # controller commands and returns -- it does not report completion.
        self.declare_parameter('gripper_settle_sec', 1.5)
        # Pause between steps. This is settling time, not padding: the
        # waypoint executor reports arrival on joint POSITION being within
        # tolerance, which happens while the arm is still decelerating and
        # ringing. Starting the next move at that instant stacks the new
        # command onto residual motion. Waiting lets each pose actually come to
        # rest, which also makes a misplaced pose obvious to watch.
        self.declare_parameter('step_pause_sec', 5.0)
        self.declare_parameter('loop', False)

        gp = self.get_parameter
        self._move_timeout = float(gp('move_timeout_sec').value)
        self._gripper_settle = float(gp('gripper_settle_sec').value)
        self._step_pause = float(gp('step_pause_sec').value)
        self._loop = bool(gp('loop').value)

        self._path_pub = self.create_publisher(
            JointTrajectory, str(gp('joint_path_topic').value), 10)
        self._gripper_pub = self.create_publisher(
            String, str(gp('gripper_command_topic').value), 10)
        self.create_subscription(
            String, str(gp('status_topic').value), self._status_cb, 10)
        self.create_subscription(
            JointState, str(gp('joint_state_topic').value), self._joint_cb, 10)

        self._status = None
        self._have_joints = False
        self._start_requested = bool(gp('autostart').value)

        self.create_subscription(
            Empty, str(gp('start_topic').value), self._start_cb, 10)
        if not self._start_requested:
            self.get_logger().info(
                "Holding. Start with:  ros2 topic pub --once %s "
                "std_msgs/msg/Empty '{}'" % str(gp('start_topic').value))

    def _status_cb(self, msg: String) -> None:
        token = msg.data.strip()
        if token in _SUCCESS or token in _FAILURE:
            self._status = token

    def _joint_cb(self, _msg: JointState) -> None:
        self._have_joints = True

    def _start_cb(self, _msg: Empty) -> None:
        if self._start_requested:
            return
        self.get_logger().info('Start received.')
        self._start_requested = True

    def wait_for_start(self) -> bool:
        """Block until triggered. Returns False if shut down while waiting."""
        while rclpy.ok() and not self._start_requested:
            rclpy.spin_once(self, timeout_sec=0.1)
        return rclpy.ok()

    def _spin(self, seconds: float) -> None:
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for(self, predicate, timeout_sec: float) -> bool:
        end = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _move_to(self, label: str, joints) -> bool:
        self._status = None
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(rx150_kinematics.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joints]
        traj.points = [point]
        self._path_pub.publish(traj)
        self.get_logger().info(
            "-> '%s'  q=[%s]" % (label, ', '.join('%.3f' % v for v in joints)))

        if not self._wait_for(lambda: self._status is not None, self._move_timeout):
            self.get_logger().error(
                "'%s': no completion within %.1f s. Is "
                'rx150_joint_waypoint_executor running?' % (label, self._move_timeout))
            return False
        if self._status in _FAILURE:
            self.get_logger().error("'%s' -> %s" % (label, self._status))
            return False
        self.get_logger().info("'%s' -> %s" % (label, self._status))
        return True

    def _gripper(self, token: str) -> None:
        self.get_logger().info("gripper: '%s'" % token)
        self._gripper_pub.publish(String(data=token))
        self._spin(self._gripper_settle)

    def _check_sequence(self) -> None:
        """Report poses that cannot be commanded as recorded, before moving.

        Two failure modes are easy to record by accident and impossible to spot
        in a list of radians:

        OUT OF LIMITS -- a pose captured while the arm was hand-posed with
        torque off can sit past the URDF limits. The relay in
        rx150_dls_ik_executor clips to those limits, so the arm quietly goes
        somewhere other than what was recorded.

        FOLDED -- a pose captured while the arm was resting rather than at the
        object. It looks like a normal set of joint angles, but forward
        kinematics puts the tool almost on top of the base, so the arm drives
        backwards to reach it.
        """
        cfg = DlsSolverConfig()
        lower, upper = cfg.joint_limits_lower, cfg.joint_limits_upper
        for index, (label, joints, _) in enumerate(SEQUENCE, 1):
            q = np.array(joints, dtype=float)
            for name, value, lo, hi in zip(rx150_kinematics.JOINT_NAMES, q, lower, upper):
                if value < lo or value > hi:
                    self.get_logger().warning(
                        "step %d '%s': %s = %+.4f is outside [%+.4f, %+.4f] and "
                        'will be CLIPPED to %+.4f. A pose recorded with the arm '
                        'hand-posed can exceed the limits; the arm cannot go '
                        'there under power.'
                        % (index, label, name, value, lo, hi, min(max(value, lo), hi)))
            xyz, _, _ = rx150_kinematics.forward_kinematics_with_jacobian(
                np.clip(q, lower, upper), rx150_kinematics.TOOL_OFFSET)
            radial = float(np.hypot(xyz[0], xyz[1]))
            self.get_logger().info(
                "step %d '%s': tool [%+.3f, %+.3f, %+.3f], %.3f m out from the base"
                % (index, label, xyz[0], xyz[1], xyz[2], radial))
            if radial < 0.16:
                self.get_logger().warning(
                    "step %d '%s': the tool is only %.3f m from the base -- this "
                    'is close to the folded rest posture. If it was meant to be '
                    'at an object, it was probably recorded while the arm was '
                    'resting rather than reaching.' % (index, label, radial))

    def run(self) -> None:
        # Without joint states the executor cannot tell when a waypoint is
        # reached, so every move would time out with a far less obvious message.
        if not self._wait_for(lambda: self._have_joints, 10.0):
            self.get_logger().error(
                'No joint states. The arm driver (xs_sdk) is not publishing -- '
                'nothing can move. Not starting the sequence.')
            return

        self._check_sequence()

        while rclpy.ok():
            for index, (label, joints, gripper_after) in enumerate(SEQUENCE, 1):
                self.get_logger().info('=' * 60)
                self.get_logger().info('[%d/%d] %s' % (index, len(SEQUENCE), label))
                if not self._move_to(label, joints):
                    self.get_logger().error('Sequence ABORTED at step %d.' % index)
                    return
                if gripper_after:
                    self._gripper(gripper_after)
                self._spin(self._step_pause)
            self.get_logger().info('=' * 60)
            self.get_logger().info('Sequence complete.')
            if not self._loop:
                return


def main():
    rclpy.init()
    node = Rx150JointSequence()
    try:
        if node.wait_for_start():
            node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
