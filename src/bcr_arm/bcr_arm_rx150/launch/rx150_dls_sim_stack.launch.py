from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Every node in this stack runs on Gazebo's clock. The interbotix bringup
    # already publishes TF with use_sim_time, so a node left on the wall clock
    # cannot line up against it.
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Read time from Gazebo's /clock. Must stay true in sim.",
        ),
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bcr_arm_rx150'),
                    'launch',
                    'rx150_gz_classic.launch.py',
                ])
            ]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_gazebo_gui': LaunchConfiguration('use_gazebo_gui'),
            }.items(),
        ),
        # IK executor listens directly on /cartesian_target (and
        # /cartesian_target_pose) -- drive it straight from teleop, no planner.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'command_mode': 'trajectory',
                'command_topic': '/rx150/arm_controller/joint_trajectory',
                'joint_state_topic': '/rx150/joint_states',
                'world_frame': LaunchConfiguration('planning_frame'),
                'point_target_orientation_policy': 'none',
                'orientation_mode': 'exact',
                'fallback_to_neutral_on_failure': False,
                'position_tolerance': 0.015,
            }],
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_gripper_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'command_mode': 'trajectory',
                'command_topic': '/rx150/gripper_controller/joint_trajectory',
                # 0.0 = closed, 1.0 = fully open
                'command_units': 'normalized',
            }],
        ),
    ])
