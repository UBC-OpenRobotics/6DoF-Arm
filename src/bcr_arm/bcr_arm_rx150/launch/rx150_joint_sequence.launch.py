"""Replay recorded joint poses on the physical RX-150. No vision, no planner.

    ros2 launch bcr_arm_rx150 rx150_joint_sequence.launch.py

Brings up only what a recorded playback needs:

    rx150_control.launch.py        the arm driver (xs_sdk) + robot_state_publisher
    rx150_joint_waypoint_executor  drives each pose, reports joint:complete
    rx150_dls_ik_executor          relays joint commands to the servos
    rx150_gripper_controller       open/close, PWM effort (see modes.yaml)
    rx150_joint_sequence           the pose list itself

The IK executor is here for its OTHER job. It solves Cartesian targets, which
this launch never sends -- but it also subscribes to /rx150/joint_command and
relays raw joint configs to /rx150/commands/joint_group, which is the only
channel xs_sdk actually listens on. The waypoint executor publishes a plain
sensor_msgs/JointState there and nothing else converts it, so without this node
every pose is commanded into a topic with no subscriber: the arm sits still and
the waypoint executor reports "Still waiting ... max joint error" forever.

Deliberately absent: the planner, the sweep, the RealSense, the orchestrator.
Nothing here computes a target, so nothing here can be wrong about where the
cup is -- which is the point. Use it to prove the arm and gripper work before
debugging anything that does compute a target.

    THE ARM MOVES ON LAUNCH and does NOT check for obstacles. It replays poses
    recorded by hand. Clear the workspace before starting.

Args:
  autostart   (true)  start moving on launch; false to launch and hold.
  loop        (false) repeat the sequence until stopped.
  robot_name  (rx150)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rx150'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Move on launch. false brings the stack up and waits, '
                        'so you can watch the arm before anything commands it.',
        ),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument(
            'step_pause_sec', default_value='5.0',
            description='Settling pause after each pose. The waypoint executor '
                        'reports arrival on position tolerance, which is met '
                        'while the arm is still decelerating, so without this '
                        'the next move stacks onto residual motion.',
        ),
        DeclareLaunchArgument(
            'waypoint_joint_tolerance', default_value='0.09',
            description='How close a joint must get before a pose counts as '
                        'reached. Looser than the executor default 0.05 because '
                        'the folded home rests the forearm ON the arm: the '
                        'elbow stops about 3.5 deg short of the nominal 1.55 '
                        'from rx150.yaml and stays there, stalled against '
                        'contact, so 0.05 can never be met and the sequence '
                        'aborts on step 1. This is a PASS THRESHOLD, not a '
                        'target -- a pose the arm can actually reach still '
                        'converges to near zero error and passes immediately. '
                        'The cost is that a pose blocked by something (the cup, '
                        'say) can now report success while up to 0.09 rad off.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),

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
            }.items(),
        ),

        # Drives each pose and reports joint:complete. The sequence node waits
        # on that rather than on a timer, so a slow move is waited out and a
        # stalled one aborts instead of the next pose piling in on top of it.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_joint_waypoint_executor',
            output='screen',
            parameters=[{
                'waypoint_joint_tolerance': ParameterValue(
                    LaunchConfiguration('waypoint_joint_tolerance'),
                    value_type=float),
            }],
        ),

        # Joint-command relay: /rx150/joint_command (JointState) ->
        # /rx150/commands/joint_group (JointGroupCommand). Same tuning as
        # rx150_dls_stack.launch.py so playback moves at the same rate as the
        # rest of the stack. target_topic is pointed at an unused name because
        # nothing here sends Cartesian goals and we do not want a stray
        # /cartesian_target to drive the arm mid-sequence.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_dls_ik_executor',
            output='screen',
            parameters=[{
                'command_mode': 'group',
                'command_topic': '/rx150/commands/joint_group',
                'joint_command_topic': '/rx150/joint_command',
                'target_topic': '/sequence/unused_cartesian_target',
                'world_frame': 'rx150/base_link',
                'damping_lambda': 0.18,
                'step_scale': 0.35,
                'max_joint_step': 0.04,
                'max_joint_velocity': 0.25,
                'goal_time_sec': 4.0,
                'point_target_orientation_policy': 'none',
            }],
        ),

        # Same configuration as the full stack: the gripper servo runs in PWM
        # mode, so cmd is an effort, not an angle.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_gripper_controller',
            output='screen',
            parameters=[{
                'command_mode': 'single',
                'command_topic': '/rx150/commands/joint_single',
                'command_units': 'normalized',
                'single_command_kind': 'pwm',
            }],
        ),

        Node(
            package='bcr_arm_rx150',
            executable='rx150_joint_sequence',
            output='screen',
            parameters=[{
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'loop': ParameterValue(
                    LaunchConfiguration('loop'), value_type=bool),
                'step_pause_sec': ParameterValue(
                    LaunchConfiguration('step_pause_sec'), value_type=float),
            }],
        ),
    ])
