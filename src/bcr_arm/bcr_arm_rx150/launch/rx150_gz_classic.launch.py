from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
        ),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('recording', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xsarm_sim'),
                    'launch',
                    'xsarm_gz_classic.launch.py',
                ])
            ]),
            launch_arguments={
                'robot_model': 'rx150',
                'robot_name': LaunchConfiguration('robot_name'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
                'world_filepath': LaunchConfiguration('world_filepath'),
                'paused': LaunchConfiguration('paused'),
                'verbose': LaunchConfiguration('verbose'),
                'debug': LaunchConfiguration('debug'),
                'recording': LaunchConfiguration('recording'),
            }.items(),
        ),
    ])
