from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('hardware_type', default_value='actual'),
        DeclareLaunchArgument('use_moveit_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xsarm_moveit_interface'),
                    'launch',
                    'xsarm_moveit_interface.launch.py',
                ])
            ]),
            launch_arguments={
                'robot_model': 'rx150',
                'robot_name': LaunchConfiguration('robot_name'),
                'hardware_type': LaunchConfiguration('hardware_type'),
                'use_moveit_rviz': LaunchConfiguration('use_moveit_rviz'),
            }.items(),
        ),
    ])
