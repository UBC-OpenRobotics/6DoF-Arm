#!/usr/bin/env python3

from pathlib import Path
import pickle
import signal
import time
from typing import List, Optional

import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Collector(Node):
    def __init__(self) -> None:
        super().__init__('collector')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter(
            'output_dir',
            str(Path.home() / 'openRobotics' / 'bcr_arm' / 'data' / 'positions'),
        )
        self.declare_parameter('log_every_n_samples', 50)

        joint_state_topic = self.get_parameter('joint_state_topic').value
        output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self._log_every_n_samples = max(1, int(self.get_parameter('log_every_n_samples').value))

        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime('%Y_%m_%d_%H_%M_%S', time.localtime())
        self._pickle_path = output_dir / f'{timestamp}.pkl'
        self._csv_path = output_dir / f'{timestamp}.csv'

        self._joint_names: Optional[List[str]] = None
        self._positions: List[List[float]] = []

        self.create_subscription(JointState, joint_state_topic, self.joint_callback, 10)
        self.get_logger().info(f'Collecting joint states from {joint_state_topic}')
        self.get_logger().info(f'Saving captures under {output_dir}')

    def joint_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        if self._joint_names is None and msg.name:
            self._joint_names = list(msg.name)

        self._positions.append(list(msg.position))
        if len(self._positions) % self._log_every_n_samples == 0:
            self.get_logger().info(f'Captured {len(self._positions)} joint state samples')

    def save_and_exit(self, signum, frame) -> None:
        del signum, frame

        self.get_logger().info(
            f'Saving {len(self._positions)} joint state samples to {self._pickle_path}'
        )
        try:
            with self._pickle_path.open('wb') as file_obj:
                pickle.dump(self._positions, file_obj)

            if self._positions:
                width = len(self._positions[0])
                if self._joint_names and len(self._joint_names) == width:
                    columns = self._joint_names
                else:
                    columns = [f'joint_{index + 1}' for index in range(width)]
                data_frame = pd.DataFrame(self._positions, columns=columns)
                data_frame.to_csv(self._csv_path, index=False)
                self.get_logger().info(f'Wrote {self._csv_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save collected data: {exc}')
        finally:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Collector()
    signal.signal(signal.SIGINT, node.save_and_exit)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_exit(None, None)

    node.destroy_node()


if __name__ == '__main__':
    main()
