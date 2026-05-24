"""Demo launch: webcam replaces RealSense, YOLO uses pre-trained COCO model.

Run natively on Mac (no Docker needed):
    ros2 launch arm_perception webcam_demo.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_perception')

    return LaunchDescription([
        # Webcam publisher (replaces RealSense)
        Node(
            package='arm_perception',
            executable='webcam_demo_node',
            name='webcam_demo_node',
            parameters=[{
                'device_id': 0,
                'width': 640,
                'height': 480,
                'fps': 30.0,
            }],
            output='screen',
        ),

        # YOLO detector (pre-trained COCO model)
        Node(
            package='arm_perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.4,
                'device': 'cpu',  # use 'mps' for M1 Metal acceleration
            }],
            output='screen',
        ),
    ])
