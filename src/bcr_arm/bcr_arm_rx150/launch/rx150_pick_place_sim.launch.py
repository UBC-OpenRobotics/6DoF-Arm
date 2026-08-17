"""One-command pick-and-place mission in Gazebo sim (no hardware needed).

Sim analogue of rx150_pick_place.launch.py: brings up the Gazebo sim stack
(arm + planner + RRT + executors + gripper + cloud relay) plus the placeholders
and the orchestrator, so the full mission + event pipeline can be exercised end to
end on your machine.

  - rx150_dls_sim_stack.launch.py  (Gazebo + planner + executors + gripper)
  - vision_placeholder             (canned cup/goal points)
  - sweep_placeholder              (canned obstacle cloud on /planning/point_cloud)
  - rx150_pick_place_orchestrator  (the conductor)

Nothing moves until you start the mission (unless autostart:=true):

    ros2 topic pub --once /mission/start std_msgs/msg/Empty '{}'

Args:
  autostart        (false) start the mission immediately instead of waiting.
  use_vision_stub  (true)  run vision_placeholder.
  use_sweep_stub   (true)  run sweep_placeholder. Set false to instead run the
                           real sim sweep by hand (ros2 run data_collector
                           scene_sweep_mapper) for a more faithful map.
  planning_frame   (rx150/base_link)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('use_vision_stub', default_value='true'),
        DeclareLaunchArgument('use_sweep_stub', default_value='true'),
        DeclareLaunchArgument(
            'carry_level', default_value='false',
            description='Keep the gripper level so a grasped cup cannot tip.',
        ),
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),
        DeclareLaunchArgument(
            'pregrasp_value', default_value='open',
            description='Gripper opening set at the hover point, before descending '
                        'onto the cup. 0.0 (closed) .. 1.0 (open), or open/close.',
        ),
        DeclareLaunchArgument(
            'grasp_value', default_value='0.3',
            description='How far to close on the cup. 0.0 (closed) .. 1.0 (open). '
                        'PLACEHOLDER -- tune to the real cup.',
        ),

        # Full Gazebo sim stack: arm + planner + RRT + executors + gripper + relay.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bcr_arm_rx150'),
                    'launch',
                    'rx150_dls_sim_stack.launch.py',
                ])
            ]),
            launch_arguments={
                'planning_frame': planning_frame,
                'carry_level': LaunchConfiguration('carry_level'),
            }.items(),
        ),

        # Vision stub (swap for real vision on the same response topic).
        Node(
            package='bcr_arm_rx150',
            executable='vision_placeholder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_vision_stub')),
            parameters=[{'planning_frame': planning_frame}],
        ),

        # Sweep stub (canned obstacle map -- matches the world's front box).
        Node(
            package='bcr_arm_rx150',
            executable='sweep_placeholder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sweep_stub')),
            parameters=[{'planning_frame': planning_frame}],
        ),

        # The conductor.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_pick_place_orchestrator',
            output='screen',
            parameters=[{
                'planning_frame': planning_frame,
                'autostart': LaunchConfiguration('autostart'),
                # Orchestrator toggles the level constraint on/off around the
                # grasp so only the carrying moves are constrained.
                'carry_level': LaunchConfiguration('carry_level'),
                'pregrasp_value': ParameterValue(
                    LaunchConfiguration('pregrasp_value'), value_type=str),
                'grasp_value': ParameterValue(
                    LaunchConfiguration('grasp_value'), value_type=str),
            }],
        ),
    ])
