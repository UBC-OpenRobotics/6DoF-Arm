#!/usr/bin/env python3

"""Keyboard control for the pick-and-place mission.

Turns single key presses into the mission control events the orchestrator
already listens on, so a running stack can be driven by hand without publishing
topics from the shell:

  s  START    -- run the mission (ignored if one is already running)
  x  STOP     -- cancel motion now, park the arm home, go back to idle
  r  RESTART  -- stop, then immediately begin a fresh mission
  q  QUIT     -- exit this node only; the mission stack keeps running

Run it in its **own terminal**, not from the launch file. A node started by
``ros2 launch`` shares one multiplexed stdin with every other node in the
launch, so it cannot reliably read key presses; this needs a real TTY of its
own::

    docker compose exec -it rx150-sim bash -lc \\
      "source /workspaces/bcr_arm/install/setup.bash && \\
       ros2 run bcr_arm_rx150 mission_keyboard"

The ``-it`` matters: without it docker gives the exec no TTY and this node
exits with an explanation rather than silently ignoring every key.

Keys are read raw (cbreak mode, no Enter needed). The terminal is always
restored on exit, including on Ctrl-C.
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


_BANNER = """
================================================================
 RX-150 mission keyboard
================================================================
   s   start the mission
   x   stop  (cancels motion, arm returns home)
   r   restart (stop, then run again immediately)
   q   quit this node (mission stack keeps running)
================================================================
"""


class MissionKeyboard(Node):
    def __init__(self) -> None:
        super().__init__('mission_keyboard')

        self.declare_parameter('start_topic', '/mission/start')
        self.declare_parameter('stop_topic', '/mission/stop')
        self.declare_parameter('restart_topic', '/mission/restart')

        gp = self.get_parameter
        self._pubs = {
            's': (
                'START',
                self.create_publisher(Empty, str(gp('start_topic').value), 10),
            ),
            'x': (
                'STOP',
                self.create_publisher(Empty, str(gp('stop_topic').value), 10),
            ),
            'r': (
                'RESTART',
                self.create_publisher(Empty, str(gp('restart_topic').value), 10),
            ),
        }

    def handle_key(self, key: str) -> bool:
        """Act on one key. Returns False when the user asked to quit."""
        key = key.lower()
        if key in ('q', '\x03'):  # q or Ctrl-C
            return False
        entry = self._pubs.get(key)
        if entry is None:
            return True
        label, publisher = entry

        listeners = publisher.get_subscription_count()
        publisher.publish(Empty())
        if listeners == 0:
            self.get_logger().error(
                '%s -> %s but NOTHING IS SUBSCRIBED -- the message went nowhere. '
                'Is the mission stack up ("Idle. Waiting for mission start")? Is '
                'this terminal in the same container / ROS_DOMAIN_ID? Check with: '
                'ros2 topic info %s'
                % (label, publisher.topic_name, publisher.topic_name)
            )
        else:
            self.get_logger().info(
                '%s -> %s (%d listener%s)'
                % (label, publisher.topic_name, listeners,
                   '' if listeners == 1 else 's')
            )
        return True


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = MissionKeyboard()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'stdin is not a terminal, so no key press can ever be read. Run this '
            'node in its own terminal with a TTY attached, e.g. '
            '"docker compose exec -it rx150-sim bash -lc \'...\'" -- note the -it. '
            'Meanwhile you can still drive the mission by publishing '
            'std_msgs/Empty on /mission/start, /mission/stop or /mission/restart.'
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    print(_BANNER, flush=True)

    stdin_fd = sys.stdin.fileno()
    original_term = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            # select() so we poll ROS callbacks instead of blocking on a read.
            if select.select([sys.stdin], [], [], 0.0)[0]:
                if not node.handle_key(sys.stdin.read(1)):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_term)
        print('\nmission_keyboard exiting.', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
