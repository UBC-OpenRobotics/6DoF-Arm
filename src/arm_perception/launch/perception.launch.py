import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_perception')
    realsense_config = os.path.join(pkg_dir, 'config', 'realsense_params.yaml')
    yolo_config = os.path.join(pkg_dir, 'config', 'yolo_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Launch RealSense camera driver',
        ),
        DeclareLaunchArgument(
            'camera_serial_no',
            default_value='',
            description='RealSense camera serial number',
        ),

        # RealSense camera driver (from upstream package)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py',
            ]),
            launch_arguments={
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'rosbag_filename': '/ros2_ws/bag_files/test1_uncompressed.bag',
                'serial_no': LaunchConfiguration('camera_serial_no'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),

        # # RealSense health monitor --> commented out for now it's causing conflict with the realsense driver
        # Node(
        #     package='arm_perception',
        #     executable='realsense_node',
        #     name='realsense_node',
        #     parameters=[realsense_config],
        #     output='screen',
        # ),

        # # Color preprocessing
        # Node(
        #     package='arm_perception',
        #     executable='color_preprocessing_node',
        #     name='color_preprocessing_node',
        #     parameters=[{
        #         'filter': 'median', # options: none, bilateral, median, gaussian
        #         'light_processing': 'none', # options: none, clahe
        #     }],
        #     output='screen',
        # ),

        # YOLO detector
        Node(
            package='arm_perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            parameters=[yolo_config],
            output='screen',
        ),

        # 3D localization
        Node(
            package='arm_perception',
            executable='localization_3d_node',
            name='localization_3d_node',
            parameters=[{
                'depth_scale': 0.001,
            }],
            output='screen',
        ),

        #RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'perception.rviz')],
            output='screen',
        ),

        #Get 3D point action server
        Node(
            package='arm_perception',
            executable='get_3d_point_action_node',
            name='get_3d_point_action_node',
            output='screen',
        )
    ])
