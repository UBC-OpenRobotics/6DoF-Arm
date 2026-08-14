from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame')
    minimal = LaunchConfiguration('minimal')

    # Conservative motion params shared by both IK-executor variants. The real
    # arm moves slowly / gently for safe bring-up; raise once trusted. All the
    # planning nodes use the SAME frame as sim (rx150/base_link) -- that is the
    # actual TF base frame the Interbotix description publishes on hardware, so
    # no frame divergence between sim and real.
    hardware_ik_params = {
        'command_mode': 'group',
        'command_topic': '/rx150/commands/joint_group',
        'world_frame': planning_frame,
        'damping_lambda': 0.18,
        'step_scale': 0.35,
        'max_joint_step': 0.04,
        'max_joint_velocity': 0.25,
        'goal_time_sec': 4.0,
        'point_target_orientation_policy': 'none',
        'orientation_mode': 'upright_free_yaw',
    }

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_configs', default_value='true'),
        # minimal:=false (default) brings up the FULL stack: the obstacle
        #   planner, the A*/RRT-Connect executors, and the cloud relay -- same as
        #   the sim stack, on the real arm.
        # minimal:=true brings up ONLY the arm driver + DLS IK executor (drive it
        #   directly with Cartesian targets, no camera/planner). Use this until
        #   the RealSense port is done, or for a bare bring-up.
        DeclareLaunchArgument('minimal', default_value='false'),
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),
        # Where the RealSense driver publishes its cloud (the relay's input). The
        # driver itself is the in-progress hardware port -- start it separately
        # (see HARDWARE_COMMANDS.md); until then the planner idles with no cloud.
        DeclareLaunchArgument(
            'camera_points_topic', default_value='/camera/depth/color/points'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bcr_arm_rx150'),
                    'launch',
                    'rx150_control.launch.py',
                ])
            ]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'load_configs': LaunchConfiguration('load_configs'),
            }.items(),
        ),

        # --- MINIMAL mode: IK executor listens directly on /cartesian_target ---
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            condition=IfCondition(minimal),
            parameters=[dict(hardware_ik_params, target_topic='/cartesian_target')],
        ),

        # --- FULL mode: IK executor driven by the waypoint executors ------------
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            condition=UnlessCondition(minimal),
            parameters=[dict(
                hardware_ik_params,
                target_topic='/ik_waypoint_target',
                target_pose_topic='/ik_waypoint_target_pose',
            )],
        ),

        # --- FULL mode: cloud relay (RealSense frame -> base_link) --------------
        # Applies the camera->base_link transform via TF. Needs the RealSense
        # driver publishing camera_points_topic; add/launch that when ready.
        Node(
            package='data_collector',
            executable='scene_point_cloud',
            output='screen',
            condition=UnlessCondition(minimal),
            parameters=[{
                'input_topic': LaunchConfiguration('camera_points_topic'),
                'output_topic': '/planning/live_point_cloud',
                'target_frame': planning_frame,
                'source_frame': '',
                'broadcast_static_tf': False,
            }],
        ),

        # --- FULL mode: obstacle planner (A* + RRT-Connect fallback) ------------
        Node(
            package='bcr_arm_rx150',
            executable='rx150_point_cloud_path_planner',
            output='screen',
            condition=UnlessCondition(minimal),
            parameters=[{
                'world_frame': planning_frame,
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

        # --- FULL mode: Cartesian waypoint executor ----------------------------
        Node(
            package='bcr_arm_rx150',
            executable='rx150_path_waypoint_executor',
            output='screen',
            condition=UnlessCondition(minimal),
            parameters=[{
                'world_frame': planning_frame,
                'joint_state_topic': '/rx150/joint_states',
                'path_topic': '/planned_cartesian_path',
                'ik_target_topic': '/ik_waypoint_target',
                'ik_target_pose_topic': '/ik_waypoint_target_pose',
            }],
        ),

        # --- FULL mode: joint waypoint executor (RRT-Connect fallback path) -----
        Node(
            package='bcr_arm_rx150',
            executable='rx150_joint_waypoint_executor',
            output='screen',
            condition=UnlessCondition(minimal),
            parameters=[{
                'joint_state_topic': '/rx150/joint_states',
                'joint_path_topic': '/planned_joint_path',
                'joint_command_topic': '/rx150/joint_command',
            }],
        ),
    ])
