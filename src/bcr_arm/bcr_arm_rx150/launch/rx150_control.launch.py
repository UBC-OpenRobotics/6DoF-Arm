from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_configs', default_value='true'),
        # Extra links to graft onto the RX-150 description -- on hardware this is
        # how the RealSense mounting link gets into the URDF. Without it
        # robot_state_publisher publishes no camera frame at all, the RealSense's
        # own TF tree has no path to rx150/base_link, and every camera->base
        # lookup fails: no obstacle map from the sweep and no 3D detections.
        # Declared and forwarded here because the upstream
        # xsarm_control.launch.py already accepts it (via
        # declare_interbotix_xsarm_robot_description_launch_arguments).
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('interbotix_xsarm_control'),
                    'launch',
                    'xsarm_control.launch.py',
                ])
            ]),
            launch_arguments={
                'robot_model': 'rx150',
                'robot_name': LaunchConfiguration('robot_name'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'load_configs': LaunchConfiguration('load_configs'),
                'external_urdf_loc': LaunchConfiguration('external_urdf_loc'),
            }.items(),
        ),
    ])
