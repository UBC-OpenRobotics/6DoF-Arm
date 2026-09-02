from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Every node in this stack runs on Gazebo's clock. The camera stamps its
    # clouds in sim time and the interbotix bringup already publishes TF with
    # use_sim_time, so a node left on the wall clock cannot line either of them
    # up with anything -- which is what let clouds captured mid-slew be
    # transformed with the pose the arm finished in. See scene_point_cloud.
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Read time from Gazebo's /clock. Must stay true in sim: "
                        'stamped TF lookups fail against a wall clock.',
        ),
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
        # carry_level:=true keeps the gripper LEVEL while executing planned paths
        # (pose_level + exact IK orientation) so a grasped cup stays upright.
        # Default off -- position-only, freer to reach.
        DeclareLaunchArgument(
            'carry_level', default_value='false',
            description='Keep the gripper LEVEL while moving (approach horizontal, '
                        'gripper-up = world-up) so a grasped cup stays upright '
                        '(pose_level + exact IK).',
        ),
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),
        DeclareLaunchArgument(
            'grasp_clearance_radius', default_value='0.08',
            description='Cloud points within this radius of the grasp anchor are '
                        'treated as the target object and excluded from collision '
                        'checks -- without it the cup blocks the reach to itself. '
                        'Raise it when the cup sits among clutter the arm must '
                        'push past; it also blinds the planner to anything real '
                        'inside the sphere, so keep it just over the object.',
        ),
        DeclareLaunchArgument(
            'obstacle_height_threshold', default_value='0.01',
            description='Map points at or below this height are ground, not '
                        'obstacles. Must sit ABOVE the sweep z_min crop or it '
                        'drops nothing and the work surface is planned around as '
                        'a wall; the planner warns when that happens.',
        ),
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
                'use_sim_time': use_sim_time,
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
        Node(
            package='data_collector',
            executable='scene_point_cloud',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_scene_point_cloud')),
            parameters=[{
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
                'world_frame': LaunchConfiguration('planning_frame'),
                'point_cloud_topic': '/planning/point_cloud',
                'joint_state_topic': '/rx150/joint_states',
                'target_topic': '/cartesian_target',
                'path_topic': '/planned_cartesian_path',
                'grid_x_min': -0.45,
                'grid_x_max': 0.45,
                'grid_y_min': -0.45,
                'grid_y_max': 0.45,
                # 1 cell (2 cm) of inflation, not 3. The grid is 45x45 = 2025
                # cells; at 3 cells every obstacle point blocks a 14x14 cm
                # square, so a few hundred scattered points sever the grid and
                # A* reports "no route" on a scene that is mostly clear.
                'obstacle_inflation_cells': 1,
                # The fallback is collision-check bound: ~3 ms per check against
                # an 18k-point map, ~3 checks per tree extension. The old 1.5 s
                # budget bought roughly 30 nodes, which is not enough for a
                # 5-DOF search to route around anything.
                'rrt_time_budget_sec': 8.0,
                'grasp_clearance_radius': ParameterValue(
                    LaunchConfiguration('grasp_clearance_radius'),
                    value_type=float),
                'obstacle_height_threshold': ParameterValue(
                    LaunchConfiguration('obstacle_height_threshold'),
                    value_type=float),
            }],
        ),
        Node(
            package='bcr_arm_rx150',
            executable='rx150_path_waypoint_executor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_waypoint_executor')),
            parameters=[{
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
                'joint_state_topic': '/rx150/joint_states',
                'joint_path_topic': '/planned_joint_path',
                'joint_command_topic': '/rx150/joint_command',
            }],
        ),
    ])
