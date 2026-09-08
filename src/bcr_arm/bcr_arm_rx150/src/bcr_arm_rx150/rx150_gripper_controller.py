#!/usr/bin/env python3

"""RX-150 gripper controller.

Single node that opens/closes the gripper on either target, mirroring the
`command_mode` split the arm's DLS IK executor uses:

- **Physical** (`command_mode='single'`): the gripper is one servo named
  ``gripper`` (motor ID 6). It is driven with an ``interbotix_xs_msgs/
  JointSingleCommand`` (``name='gripper'``) on ``/rx150/commands/joint_single``.
  The value you feed is interpreted by whatever operating mode that motor is
  configured for in the xs_sdk (``position`` -> radians of servo angle,
  ``pwm`` -> raw PWM). This node just passes the number through; make sure the
  gripper motor's operating mode matches the units of your open/closed values.

- **Sim** (`command_mode='trajectory'`): the gripper is the ros2_control
  ``gripper_controller`` (a JointTrajectoryController) driving the two prismatic
  finger joints ``left_finger``/``right_finger``. In Gazebo the mimic is
  disabled, so both are commanded explicitly with ``right = -left``. The value
  is the ``left_finger`` prismatic position in metres (finger travel limits are
  ``0.015`` closed .. ``0.037`` open).

Three ways to command it (your "3 values"):
  1. a raw variable position (native units of the active mode), clamped to
     [``position_min``, ``position_max``];
  2. the named state ``open``  -> the ``open_position`` param;
  3. the named state ``close`` -> the ``closed_position`` param.

Interfaces:
  - topic ``/rx150/gripper_command`` (``std_msgs/String``): send ``"open"``,
    ``"close"`` (or ``"closed"``), or a numeric string like ``"0.025"``;
  - topic ``/rx150/gripper_position`` (``std_msgs/Float64``): send a raw value
    directly (handy for other nodes / a future grasp step);
  - CLI one-shot: ``--state open|close`` or ``--position <value>``.

Because the named-state values mean different things per mode (finger metres vs.
servo radians), when ``open_position``/``closed_position``/``position_min``/
``position_max`` are left at their sentinel default the node fills in
mode-appropriate defaults automatically (finger-space for ``trajectory``,
servo-space for ``single``). Override them explicitly to tune for real hardware.

The raw variable's *units* are selected by ``command_units``:
  - ``native`` (node default): the value is in the active mode's own units
    (finger metres / servo radians), so the same number means different openings
    in sim vs. hardware;
  - ``normalized``: the value is ``0.0`` (closed) .. ``1.0`` (open), linearly
    mapped into each mode's range, so the same number is the same openness on
    both. Both shipped launches use ``normalized`` for sim/hardware parity.
Named states (``open``/``close``) are unaffected.
"""

import argparse
import math

import rclpy
from interbotix_xs_msgs.msg import JointSingleCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Sim finger prismatic travel (metres), from the base rx150 URDF finger limits.
_SIM_FINGER_CLOSED = 0.015
_SIM_FINGER_OPEN = 0.037

# TODO
#Physical gripper servo angle (radians). Placeholders for the gripper motor in
# `position` operating mode -- verify/tune on the real arm before trusting them.
_HW_SERVO_CLOSED = 0.6
_HW_SERVO_OPEN = 1.7

# Physical gripper PWM effort, for the motor in `pwm` operating mode -- which is
# what interbotix_xsarm_control/config/modes.yaml actually sets:
#
#   singles:
#     gripper:
#       operating_mode: pwm
#
# In that mode JointSingleCommand.cmd is a PWM effort (roughly +/-885), NOT a
# joint angle. Sending the radian values above puts ~0.6 of 885 on the motor,
# which is indistinguishable from no command: the gripper never moves and
# nothing errors. Sign is direction; magnitude is how hard it squeezes, so the
# grip force self-limits instead of the servo fighting to a setpoint and
# stalling on the object. That is why Interbotix ships the gripper this way.
#
# OPEN and CLOSE are deliberately asymmetric, because they are doing different
# jobs.
#
# CLOSING squeezes an object. Effort is grip force, and it must be HELD for as
# long as the object is carried. ~1/3 of full scale holds a light object
# without straining the motor. Raise it if the cup slips, but watch the servo
# temperature.
#
# OPENING has nothing to squeeze -- it has to drive the fingers the whole way
# to their mechanical stop, against stiction. In PWM mode the fingers stop
# wherever the applied effort balances friction, NOT at the stop, so too small
# a value leaves the gripper part-open with no error anywhere. Opening
# therefore uses the larger magnitude even though it is the gentler operation.
# It is held indefinitely once commanded -- dropping it to zero (or easing it)
# lets the fingers sag back off the stop, since a light pwm command leaves the
# gripper closer to limp than held.
#
# SIGN: POSITIVE pwm OPENS this gripper, confirmed on the arm. Read left_finger
# in /rx150/joint_states WHILE a command is applied to re-check either value --
# not after, since a following command (or the effort easing) changes it back.
_HW_PWM_CLOSED = -300.0
_HW_PWM_OPEN = 800.0

_SENTINEL = float('nan')


class Rx150GripperController(Node):
    def __init__(self, oneshot_command=None):
        super().__init__('rx150_gripper_controller')

        self.declare_parameter('command_mode', 'single')
        self.declare_parameter('command_topic', '')
        self.declare_parameter('command_input_topic', '/rx150/gripper_command')
        self.declare_parameter('position_input_topic', '/rx150/gripper_position')
        self.declare_parameter('joint_names', ['left_finger', 'right_finger'])
        self.declare_parameter('single_joint_name', 'gripper')
        self.declare_parameter('command_time_sec', 1.0)
        self.declare_parameter('command_units', 'native')
        # What the servo reads JointSingleCommand.cmd AS -- 'position' (joint
        # angle) or 'pwm' (effort). It must match the gripper's operating_mode
        # in the mode config xs_sdk loaded, or commands are silently ignored.
        # Separate from command_units, which scales the INPUT: 'normalized'
        # plus 'pwm' means 0.0..1.0 maps across the PWM range.
        self.declare_parameter('single_command_kind', 'position')
        # Sentinel defaults -> filled in per-mode below.
        self.declare_parameter('open_position', _SENTINEL)
        self.declare_parameter('closed_position', _SENTINEL)
        self.declare_parameter('position_min', _SENTINEL)
        self.declare_parameter('position_max', _SENTINEL)
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('state_output_topic', '/rx150/gripper_state')

        mode = str(self.get_parameter('command_mode').value).strip().lower()
        if mode not in {'single', 'trajectory'}:
            self.get_logger().warning(
                "Unknown command_mode '%s'. Falling back to 'single'." % mode
            )
            mode = 'single'
        self._command_mode = mode

        # Mode-appropriate defaults when the user left params at the sentinel.
        command_kind = str(
            self.get_parameter('single_command_kind').value
        ).strip().lower()
        if command_kind not in {'position', 'pwm'}:
            self.get_logger().warning(
                "Unknown single_command_kind '%s'. Falling back to 'position'."
                % command_kind
            )
            command_kind = 'position'
        self._single_command_kind = command_kind

        if mode == 'trajectory':
            default_open, default_closed = _SIM_FINGER_OPEN, _SIM_FINGER_CLOSED
            default_topic = '/rx150/gripper_controller/joint_trajectory'
        elif command_kind == 'pwm':
            default_open, default_closed = _HW_PWM_OPEN, _HW_PWM_CLOSED
            default_topic = '/rx150/commands/joint_single'
        else:
            default_open, default_closed = _HW_SERVO_OPEN, _HW_SERVO_CLOSED
            default_topic = '/rx150/commands/joint_single'

        self._open_position = self._param_or(
            'open_position', default_open
        )
        self._closed_position = self._param_or(
            'closed_position', default_closed
        )
        # Travel bounds default to the span of the two named states.
        lo, hi = sorted((self._open_position, self._closed_position))
        self._position_min = self._param_or('position_min', lo)
        self._position_max = self._param_or('position_max', hi)

        command_topic = str(self.get_parameter('command_topic').value).strip()
        self._command_topic = command_topic if command_topic else default_topic
        self._single_joint_name = str(self.get_parameter('single_joint_name').value)
        self._joint_names = [
            str(name) for name in self.get_parameter('joint_names').value
        ]
        self._command_time_sec = max(
            0.0, float(self.get_parameter('command_time_sec').value)
        )
        units = str(self.get_parameter('command_units').value).strip().lower()
        if units not in {'native', 'normalized'}:
            self.get_logger().warning(
                "Unknown command_units '%s'. Falling back to 'native'." % units
            )
            units = 'native'
        self._command_units = units

        if self._command_mode == 'single':
            self._publisher = self.create_publisher(
                JointSingleCommand, self._command_topic, 10
            )
        else:
            self._publisher = self.create_publisher(
                JointTrajectory, self._command_topic, 10
            )

        self._state_pub = self.create_publisher(
            Float64, str(self.get_parameter('state_output_topic').value), 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._joint_state_callback, 10,
        )

        self._oneshot = oneshot_command is not None
        if not self._oneshot:
            self.create_subscription(
                String,
                str(self.get_parameter('command_input_topic').value),
                self._string_callback,
                10,
            )
            self.create_subscription(
                Float64,
                str(self.get_parameter('position_input_topic').value),
                self._float_callback,
                10,
            )
            self.get_logger().info(
                'RX-150 gripper controller (%s mode) listening on %s / %s; '
                'publishing to %s. open=%.4f close=%.4f bounds=[%.4f, %.4f]'
                % (
                    self._command_mode,
                    self.get_parameter('command_input_topic').value,
                    self.get_parameter('position_input_topic').value,
                    self._command_topic,
                    self._open_position,
                    self._closed_position,
                    self._position_min,
                    self._position_max,
                )
            )

    def _feedback_joint(self) -> str:
        """The joint whose position reports actual gripper openness."""
        if self._command_mode == 'single':
            return self._single_joint_name
        # Trajectory mode drives two mirrored fingers; either reports the travel,
        # and left_finger is the one the URDF gives a positive range.
        return self._joint_names[0] if self._joint_names else 'left_finger'

    def _joint_state_callback(self, msg: JointState) -> None:
        """Publish actual openness, normalized 0 (closed) .. 1 (open).

        This is what makes grasp detection possible. A position *command* is not
        a grasp: the fingers reach the commanded value when they close on empty
        air, and stall short of it when an object is between them. Comparing
        commanded against actual therefore distinguishes the two, but only if
        both are in the same units, which is what this conversion provides.
        """
        name = self._feedback_joint()
        if name not in msg.name:
            return
        raw = float(msg.position[msg.name.index(name)])
        span = self._open_position - self._closed_position
        if abs(span) < 1e-9:
            return
        # Deliberately NOT clamped to [0, 1]: a value outside that range means
        # the gripper is past an endpoint, which is exactly the signal that the
        # endpoints are wrong. Clamping would hide it.
        self._state_pub.publish(Float64(data=(raw - self._closed_position) / span))

    def _param_or(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        return default if math.isnan(value) else value

    def _string_callback(self, msg: String) -> None:
        self.command(msg.data)

    def _float_callback(self, msg: Float64) -> None:
        self._publish_position(
            self._to_native(float(msg.data)), source='%.4f' % msg.data
        )

    def _to_native(self, value: float) -> float:
        """Map a raw variable command into native units.

        In 'native' mode this is a pass-through. In 'normalized' mode the input
        is 0.0 (closed) .. 1.0 (open), linearly interpolated between
        closed_position and open_position -- so the same number is the same
        openness regardless of sim vs. hardware.
        """
        if self._command_units != 'normalized':
            return value
        fraction = min(max(value, 0.0), 1.0)
        return self._closed_position + fraction * (
            self._open_position - self._closed_position
        )

    def command(self, token) -> bool:
        """Resolve a string token ('open'/'close'/'<number>') and publish it."""
        position = self._resolve(token)
        if position is None:
            self.get_logger().error(
                "Unrecognized gripper command '%s'. Use 'open', 'close', or a "
                'number.' % token
            )
            return False
        self._publish_position(position, source=str(token))
        return True

    def _resolve(self, token):
        text = str(token).strip().lower()
        if text in {'open', 'opened'}:
            return self._open_position
        if text in {'close', 'closed'}:
            return self._closed_position
        try:
            return self._to_native(float(text))
        except ValueError:
            return None

    def _publish_position(self, position: float, source: str) -> None:
        clamped = min(max(position, self._position_min), self._position_max)
        if abs(clamped - position) > 1e-9:
            self.get_logger().warning(
                'Gripper command %.4f out of [%.4f, %.4f]; clamped to %.4f.'
                % (position, self._position_min, self._position_max, clamped)
            )
        if self._command_mode == 'single':
            msg = JointSingleCommand()
            msg.name = self._single_joint_name
            msg.cmd = float(clamped)
            self._publisher.publish(msg)
        else:
            msg = JointTrajectory()
            msg.joint_names = list(self._joint_names)
            point = JointTrajectoryPoint()
            # left_finger = +position, right_finger mirrors it.
            point.positions = [float(clamped), float(-clamped)]
            point.time_from_start.sec = int(self._command_time_sec)
            point.time_from_start.nanosec = int(
                (self._command_time_sec - int(self._command_time_sec)) * 1e9
            )
            msg.points = [point]
            self._publisher.publish(msg)
        self.get_logger().info(
            "Gripper command '%s' -> %.4f (%s mode)"
            % (source, clamped, self._command_mode)
        )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description='Open/close the RX-150 gripper.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--state', choices=['open', 'close'], help='named state')
    group.add_argument('--position', type=float, help='raw position (native units)')
    args, ros_args = parser.parse_known_args(argv)

    oneshot = args.state if args.state is not None else args.position
    oneshot = None if oneshot is None else oneshot

    rclpy.init(args=ros_args)
    node = Rx150GripperController(oneshot_command=oneshot)
    exit_code = 0
    try:
        if node._oneshot:
            ok = node.command(
                args.state if args.state is not None else args.position
            )
            # Give the publisher a moment to deliver before shutdown.
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)
            exit_code = 0 if ok else 1
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
