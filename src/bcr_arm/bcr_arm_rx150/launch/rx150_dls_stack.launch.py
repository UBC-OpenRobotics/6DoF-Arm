from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame')
    minimal = LaunchConfiguration('minimal')
    carry_level = LaunchConfiguration('carry_level')

    carry_on = ["'", carry_level, "'.lower() == 'true'"]
    orientation_mode = PythonExpression(
        ["'exact' if "] + carry_on + [" else 'upright_free_yaw'"]
    )

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
        'orientation_mode': orientation_mode,
    }

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        # RViz is launched HERE, not by the Interbotix chain: upstream
        # xsarm_control.launch.py accepts no `rvizconfig` and forwards none, so
        # a config passed down it is silently dropped and RViz comes up on the
        # bare xsarm_description default (robot model + TF, no point cloud).
        # We therefore hand `use_rviz:=false` to the include below and run our
        # own node, which is the only way to preselect displays without
        # patching the vendored interbotix subtree.
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('bcr_arm_rx150'),
                'rviz',
                'rx150_dls_stack.rviz',
            ]),
            description='RViz display config. The default preloads the swept '
                        'obstacle map, the planned path and the planning '
                        'markers, plus the raw RealSense cloud as a display '
                        'that is off until you tick it.',
        ),
        DeclareLaunchArgument('load_configs', default_value='true'),
        # minimal:=false (default) brings up the FULL stack: the obstacle
        #   planner, the A*/RRT-Connect executors, and the cloud relay -- same as
        #   the sim stack, on the real arm.
        # minimal:=true brings up ONLY the arm driver + DLS IK executor (drive it
        #   directly with Cartesian targets, no camera/planner). Use this until
        #   the RealSense port is done, or for a bare bring-up.
        DeclareLaunchArgument('minimal', default_value='false'),
        # bypass_planner:=true keeps the FULL stack (sweep, vision, the mission
        # orchestrator) but takes the obstacle planner out of the motion path:
        # the IK executor reports its own terminal motion events so whatever
        # drives it can wait on a move the way it waits on the planner's.
        # Retargeting the orchestrator onto it is done in
        # rx150_pick_place.launch.py; on its own this flag only makes the IK
        # executor speak.
        #
        # THERE IS NO COLLISION CHECKING ON THAT PATH. Use it to test reach and
        # camera extrinsics, not to work around a cluttered scene.
        DeclareLaunchArgument('bypass_planner', default_value='false'),
        # carry_level:=true keeps the gripper level (orientation-locked) while
        # moving so a grasped cup can't tip. Default off (position-only, freer to
        # reach). Applies to the FULL-stack Cartesian path; the RRT fallback joint
        # path does NOT hold orientation (it commands raw configs).
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
        # Where the RealSense driver publishes its cloud (the relay's input). The
        # driver itself is the in-progress hardware port. Start it separately
        # (see HARDWARE_COMMANDS.md); until then the planner idles with no cloud.
        # depth topic from the real realsense camera driver is camera/camera/depth/color/points
        DeclareLaunchArgument(
            'external_urdf_loc',
            default_value=PathJoinSubstitution([
                FindPackageShare('bcr_arm_rx150'),
                'urdf',
                'rx150_realsense_camera.urdf.xacro',
            ]),
        ),
        DeclareLaunchArgument(
            'camera_points_topic', default_value='camera/camera/depth/color/points'
        ), 

        # SCOPED ON PURPOSE. IncludeLaunchDescription is NOT scoped by itself:
        # its launch_arguments run as SetLaunchConfiguration in THIS context, so
        # the `use_rviz: 'false'` below would leak straight back out and switch
        # off our own RViz node further down (which reads the same name and is
        # visited after this). Wrapping in GroupAction keeps the override inside
        # the include, where it belongs.
        GroupAction([
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
                    # Always false: our own RViz node below replaces it.
                    'use_rviz': 'false',
                    'load_configs': LaunchConfiguration('load_configs'),
                    'external_urdf_loc': LaunchConfiguration('external_urdf_loc'),
                }.items(),
            ),
        ], scoped=True),

        # Namespaced under robot_name to match what xsarm_description would
        # have done: the RobotModel display subscribes to a RELATIVE
        # `robot_description`, which only resolves to /rx150/robot_description
        # from inside that namespace. Every other topic in the config is
        # absolute and so is unaffected.
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=LaunchConfiguration('robot_name'),
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': False}],
            output={'both': 'log'},
        ),

        # --- Gripper controller (both modes; independent of the camera) --------
        # One servo named `gripper` on the physical arm, driven via
        # JointSingleCommand. open/closed default to servo-angle placeholders --
        # verify on hardware (see rx150_gripper_controller docstring).
        Node(
            package='bcr_arm_rx150',
            executable='rx150_gripper_controller',
            output='screen',
            parameters=[{
                'command_mode': 'single',
                'command_topic': '/rx150/commands/joint_single',
                # 0.0 = closed, 1.0 = open -- same command works in sim.
                'command_units': 'normalized',
                # The gripper motor runs in PWM mode (see modes.yaml in
                # interbotix_xsarm_control), so cmd is an effort, not an angle.
                # Sending angles here is silently ignored by the servo, which
                # looks exactly like a gripper that never opens or closes.
                'single_command_kind': 'pwm',
            }],
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
                emit_motion_status=ParameterValue(
                    LaunchConfiguration('bypass_planner'), value_type=bool),
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
                # Level-carry is toggled at runtime on /motion/carry_level by the
                # orchestrator; the executor starts unconstrained.
                'waypoint_target_mode': 'point',
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
