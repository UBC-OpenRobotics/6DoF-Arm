from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('bcr_arm_rx150'),
                'worlds',
                'rx150_obstacles.world',
            ]),
        ),
        DeclareLaunchArgument(
            'external_urdf_loc',
            default_value=PathJoinSubstitution([
                FindPackageShare('bcr_arm_rx150'),
                'urdf',
                'rx150_gripper_depth_camera.urdf.xacro',
            ]),
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('bcr_arm_rx150'),
                'rviz',
                'rx150_dls_sim_stack.rviz',
            ]),
        ),
        DeclareLaunchArgument('use_scene_point_cloud', default_value='true'),
        DeclareLaunchArgument('scene_point_cloud_output_topic', default_value='/planning/live_point_cloud'),
        DeclareLaunchArgument('use_path_planner', default_value='true'),
        DeclareLaunchArgument('use_waypoint_executor', default_value='true'),
        DeclareLaunchArgument('use_joint_waypoint_executor', default_value='true'),
        DeclareLaunchArgument('waypoint_target_mode', default_value='point'),
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
                'external_urdf_loc': LaunchConfiguration('external_urdf_loc'),
                'rvizconfig': LaunchConfiguration('rvizconfig'),
                'world_filepath': LaunchConfiguration('world_filepath'),
            }.items(),
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            parameters=[{
                'command_mode': 'trajectory',
                'command_topic': '/rx150/arm_controller/joint_trajectory',
                'joint_state_topic': '/rx150/joint_states',
                'world_frame': LaunchConfiguration('planning_frame'),
                'target_topic': '/ik_waypoint_target',
                'target_pose_topic': '/ik_waypoint_target_pose',
                'point_target_orientation_policy': 'none',
                'orientation_mode': 'exact',
                'fallback_to_neutral_on_failure': False,
                'position_tolerance': 0.015,
                'joint_command_topic': '/rx150/joint_command',
            }],
        ),
        Node(
            package='data_collector',
            executable='scene_point_cloud',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_scene_point_cloud')),
            parameters=[{
                'input_topic': '/gripper_camera/points',
                'output_topic': LaunchConfiguration('scene_point_cloud_output_topic'),
                'target_frame': LaunchConfiguration('planning_frame'),
                'source_frame': '',
                'broadcast_static_tf': False,
            }],
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_point_cloud_path_planner',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_path_planner')),
            parameters=[{
                'world_frame': LaunchConfiguration('planning_frame'),
                'point_cloud_topic': '/planning/point_cloud',
                'joint_state_topic': '/rx150/joint_states',
                'target_topic': '/cartesian_target',
                'path_topic': '/planned_cartesian_path',
                'grid_x_min': -0.45,
                'grid_x_max': 0.45,
                'grid_y_min': -0.45,
                'grid_y_max': 0.45,
                'obstacle_inflation_cells': 3,
            }],
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_path_waypoint_executor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_waypoint_executor')),
            parameters=[{
                'world_frame': LaunchConfiguration('planning_frame'),
                'joint_state_topic': '/rx150/joint_states',
                'path_topic': '/planned_cartesian_path',
                'ik_target_topic': '/ik_waypoint_target',
                'ik_target_pose_topic': '/ik_waypoint_target_pose',
                'waypoint_target_mode': LaunchConfiguration('waypoint_target_mode'),
            }],
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_joint_waypoint_executor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joint_waypoint_executor')),
            parameters=[{
                'joint_state_topic': '/rx150/joint_states',
                'joint_path_topic': '/planned_joint_path',
                'joint_command_topic': '/rx150/joint_command',
            }],
        ),
    ])
