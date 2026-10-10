from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame')
    carry_level = LaunchConfiguration('carry_level')

    carry_on = ["'", carry_level, "'.lower() == 'true'"]
    orientation_mode = PythonExpression(
        ["'exact' if "] + carry_on + [" else 'upright_free_yaw'"]
    )

    # Conservative motion params for the real arm: it moves slowly / gently
    # for safe bring-up, raise once trusted. Same frame as sim
    # (rx150/base_link) -- that is the actual TF base frame the Interbotix
    # description publishes on hardware, so no frame divergence between sim
    # and real.
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
        'target_topic': '/cartesian_target',
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
        ),
        DeclareLaunchArgument('load_configs', default_value='true'),
        # carry_level:=true keeps the gripper level (orientation-locked) while
        # moving so a grasped cup can't tip. Default off (position-only, freer
        # to reach).
        DeclareLaunchArgument(
            'carry_level', default_value='false',
            description='Keep the gripper LEVEL while moving (approach horizontal, '
                        'gripper-up = world-up) so a grasped cup stays upright '
                        '(pose_level + exact IK).',
        ),
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),

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
                # The gripper motor runs in position mode (see modes.yaml in
                # interbotix_xsarm_control), so cmd is a servo angle in radians,
                # not an effort. open_position/closed_position default to the
                # measured endpoints (_HW_SERVO_OPEN/_HW_SERVO_CLOSED). No
                # current limit is set, so closing on an object will stall the
                # servo at full torque rather than backing off -- watch it.
                'single_command_kind': 'position',
            }],
        ),

        # IK executor listens directly on /cartesian_target (and
        # /cartesian_target_pose) -- drive it straight from teleop, no planner.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            parameters=[hardware_ik_params],
        ),
    ])
