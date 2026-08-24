"""One-command pick-and-place mission bring-up (physical RX-150).

Brings up the whole mission in a single launch instead of the manual multi-terminal
workflow:

  - the full hardware stack (driver + gripper + planner + RRT + executors + relay)
    via rx150_dls_stack.launch.py;
  - `perception.launch.py` (RealSense driver + YOLO + 3D localization +
    vision_bridge) -- the real vision, unless `use_vision_stub:=true`;
  - `scene_sweep_mapper` (the real arm-driven RealSense sweep) -- unless
    `use_sweep_stub:=true`, which feeds a canned obstacle cloud instead;
  - `rx150_pick_place_orchestrator` (the conductor / state machine).

Both stubs now default to **false**, and both real sources are wired, so the
default launch is the real pipeline end to end:

    RealSense -> scene_point_cloud (relay, camera frame -> base_link)
              -> scene_sweep_mapper (8-pose scan) -> /planning/point_cloud
    RealSense -> YOLO -> localization_3d_node -> vision_bridge
              -> /vision/object_point

The cup is DETECTED. The goal is still a PLACEHOLDER point
(`goal_fallback_xyz`): the detector has no drop-off class yet.

Nothing moves on launch: the orchestrator waits for a start trigger unless
`autostart:=true`. Start it with:

    ros2 topic pub --once /mission/start std_msgs/msg/Empty '{}'

Args:
  autostart        (false) start the mission immediately instead of waiting.
  use_vision_stub  (false) run vision_placeholder (canned cup/goal points)
                   instead of the real perception pipeline.
  use_sweep_stub   (false) latch a canned obstacle cloud instead of physically
                   sweeping. Useful for testing mission logic without moving the
                   arm through the scan poses.
  cup_classes      ([cup]) detector class names accepted as "the cup".
  goal_fallback_xyz  PLACEHOLDER drop-off point.
  min_depth_m      (0.05) raise to ~0.2 for a D435, which cannot focus closer.
  planning_frame   (rx150/base_link) base frame for targets (matches the stack).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from typing import List

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planning_frame = LaunchConfiguration('planning_frame')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument(
            'check_grasp', default_value='true',
            description='Abort if the fingers reach the commanded position, which '
                        'means they closed on empty air. Needs grasp_value tighter '
                        'than the object.'),
        DeclareLaunchArgument(
            'yolo_confidence', default_value='0.5',
            description='Detector confidence floor. Sim runs 0.15 because a COCO '
                        'model scores an untextured Gazebo primitive poorly; a real '
                        'cup photographs like a cup.'),
        DeclareLaunchArgument(
            'phase_delay_sec', default_value='2.0',
            description='Pause between mission phases. 2.0 is a debugging aid that '
                        'makes each cycle ~20 s longer; set 0.0 for normal running.'),
        DeclareLaunchArgument(
            'observe_joints', default_value='[]',
            description='OPTIONAL fixed look-down pose struck before the vision '
                        'request. Empty means "no pose", which is the intended '
                        'setting -- the SWEEP is what locates the cup. Anything '
                        'that is not exactly 5 joints is treated as off. '
                        'Example: [0.0,-1.65,1.07,1.25,0.0]'),
        DeclareLaunchArgument(
            'sweep_timeout_sec', default_value='240.0',
            description='Deadline for a hung sweep, not a schedule. A two-look '
                        'scan runs ~2 min in sim and slower on real servos.'),
        DeclareLaunchArgument('use_vision_stub', default_value='false'),
        DeclareLaunchArgument('use_sweep_stub', default_value='false'),
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
            'scan_posture', default_value='tilted',
            description="Tucked posture the sweep scans from. 'tilted' (default) "
                        "aims the camera ~30 deg down from higher up; 'level' "
                        "aims it parallel to the ground (the original). Against a "
                        "correctly modelled D435i, tilted maps several times "
                        "more of the scene. See SCAN_POSTURES in "
                        "scene_sweep_mapper.py. Override the joints outright "
                        "with the node's scan_tucked_joints parameter.",
        ),
        DeclareLaunchArgument(
            'approach_back_off', default_value='0.045',
            description='How far behind the target the hover sits, so the final '
                        'approach comes in diagonally. The gripper bar is a '
                        'bracket standing 35 mm proud of the gripper axis and '
                        '103 mm wide, just behind the fingers: descending '
                        'straight down lowers it onto the object and shunts it '
                        'aside. 0.0 restores the vertical descent.',
        ),
        DeclareLaunchArgument(
            'approach_height', default_value='0.15',
            description='Hover this far above a grasp/place point, then descend. '
                        'Must exceed 0.085 + half the object height: the gripper '
                        'capsule is 0.085 m and the object is a hard obstacle '
                        'while flying to the hover point. Too low aborts with '
                        '"Body collision on link segment 4".',
        ),
        DeclareLaunchArgument(
            'grasp_value', default_value='0.3',
            description='How far to close on the cup. 0.0 (closed) .. 1.0 (open).\n'
                        'MUST BE RECOMPUTED once the servo endpoints are measured. '
                        'The value maps to the gap between the PAD SURFACES, not '
                        'the finger separation the joint states report -- each pad '
                        'is inset 11 mm in the URDF, so the pad gap is 22 mm '
                        'narrower. In sim that is a usable grip range of 8..52 mm '
                        'of object width.\n'
                        'Pick the value that gives a pad gap a few mm under the '
                        'object width, leaving enough squeeze to absorb the vision '
                        'error or one finger touches and the other misses.',
        ),
        DeclareLaunchArgument(
            'cup_classes', default_value='[cup]',
            description='Detector class names accepted as "the cup".',
        ),
        DeclareLaunchArgument(
            'goal_fallback_xyz', default_value='[0.20, 0.22, 0.06]',
            description='PLACEHOLDER drop-off point, used because the detector '
                        'has no goal class yet. There is deliberately no cup '
                        'fallback: an undetected cup aborts the mission rather '
                        'than sending the real arm to a guessed position.',
        ),
        DeclareLaunchArgument(
            'min_depth_m', default_value='0.2',
            description='Depths below this are invalid. 0.2 is the minimum for '
                        'the D435i named in arm_perception/package.xml; anything '
                        'closer is noise that would be localized as the object. '
                        'Change it only if a different camera is mounted (a D405 '
                        'focuses much closer).',
        ),

        # --- Object geometry -------------------------------------------------
        # These describe the OBJECT and the observation pose, not the pipeline.
        # Same argument names as the sim launch so the two stay in step -- they
        # must, since the localization scheme is only correct for the object it
        # was tuned against. Defaults below mirror the sim object.
        # RE-MEASURE them for the real one.
        DeclareLaunchArgument(
            'bbox_anchor', default_value='bottom',
            description='Where in the detection box to sample depth. "bottom" '
                        'anchors where the object meets its surface, which does '
                        'not wander with viewing angle the way the box centre '
                        'does.',
        ),
        DeclareLaunchArgument(
            'surface_to_centre_m', default_value='0.021',
            description="The object's RADIUS. Pushes the point from the near "
                        'face the camera sees to the object axis, horizontally '
                        'in the base frame. 0.021 is the sim shot glass.',
        ),
        DeclareLaunchArgument(
            'grasp_z_offset', default_value='0.018',
            description='Raises the anchored point to the grasp height. MUST BE '
                        'MEASURED, not derived. The bottom anchor does not land '
                        'on the true base, and the error changes sign with the '
                        'viewing angle (measured 20.5 mm BELOW the base for an '
                        'object on the floor, 9.5 mm ABOVE it for one on a '
                        'stand). Read the detected z off the vision_bridge '
                        '"Answered" log and set this to reach mid-body.',
        ),

        # Full hardware stack: driver + gripper + planner + RRT + executors + relay.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bcr_arm_rx150'),
                    'launch',
                    'rx150_dls_stack.launch.py',
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

        # Sweep stub (canned obstacle map until the RealSense sweep exists).
        Node(
            package='bcr_arm_rx150',
            executable='sweep_placeholder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sweep_stub')),
            parameters=[{'planning_frame': planning_frame}],
        ),

        # The real sweep: tucks the arm, scans 8 waist angles, accumulates the
        # relayed RealSense cloud into a merged map on /planning/point_cloud, and
        # emits sweep:complete when done. Triggered mode, so it idles until the
        # orchestrator's /sweep/start rather than scanning on startup.
        #
        # Its input, /planning/live_point_cloud, is produced by the scene_point_cloud
        # relay already running in rx150_dls_stack.launch.py -- that relay is what
        # applies the camera->base_link transform.
        #
        # WARNING: this physically moves the arm through the scan poses. Clear the
        # workspace before starting a mission.
        Node(
            package='data_collector',
            executable='scene_sweep_mapper',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_sweep_stub')),
            parameters=[{
                'wait_for_trigger': True,
                'world_frame': planning_frame,
                'frame_id': planning_frame,
                'scan_posture': LaunchConfiguration('scan_posture'),
                # Crop the map to the arm's actual workspace, above the floor.
                #
                # The node defaults (x/y +-1.60, z_min -0.05) keep the whole
                # ground plane and everything out to 1.5 m. That makes the FLOOR
                # an obstacle -- the base capsule contains ground points at any
                # posture, so the RRT fallback's start check fails every time and
                # only A* ever runs -- and it inflates the cloud with tens of
                # thousands of points beyond the 0.45 m reach.
                #
                # z_min 0.03 clears the floor while keeping anything tall enough
                # for the arm to hit; +-0.60 comfortably covers the reach.
                'x_min': -0.60, 'x_max': 0.60,
                'y_min': -0.60, 'y_max': 0.60,
                'z_min': 0.03,  'z_max': 0.50,
                # Outlier filtering ON for hardware, OFF in sim (the node's own
                # default). Real RealSense depth carries flying pixels and speckle
                # that a voxel grid preserves as solid-looking obstacles, and a
                # few of those in the workspace are enough to make the planner
                # refuse a clear path. Gazebo's cloud is clean, so filtering there
                # only costs time.
                'enable_statistical_outlier_removal': True,
                'statistical_nb_neighbors': 20,
                'statistical_std_ratio': 2.0,
                'enable_radius_outlier_removal': True,
                'radius_outlier_radius': 0.02,
                'radius_outlier_min_neighbors': 10,
            }],
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
                'approach_height': ParameterValue(
                    LaunchConfiguration('approach_height'), value_type=float),
                'approach_back_off': ParameterValue(
                    LaunchConfiguration('approach_back_off'), value_type=float),
                'check_grasp': ParameterValue(
                    LaunchConfiguration('check_grasp'), value_type=bool),
                'phase_delay_sec': ParameterValue(
                    LaunchConfiguration('phase_delay_sec'), value_type=float),
                'sweep_timeout_sec': ParameterValue(
                    LaunchConfiguration('sweep_timeout_sec'), value_type=float),
                'observe_joints': ParameterValue(
                    LaunchConfiguration('observe_joints'), value_type=List[float]),
            }],
        ),

        # Full vision/perception stack: RealSense driver + YOLO + 3D
        # localization + vision_bridge. Only when the stub is off.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('arm_perception'),
                    'launch',
                    'perception.launch.py',
                ])
            ]),
            condition=UnlessCondition(LaunchConfiguration('use_vision_stub')),
            launch_arguments={
                'planning_frame': planning_frame,
                # This launch owns the RealSense driver, which also feeds the
                # sweep: its cloud goes to the scene_point_cloud relay in
                # rx150_dls_stack.launch.py, then to scene_sweep_mapper above.
                'enable_camera': 'true',
                # The arm stack runs its own RViz, and scene_sweep_mapper owns
                # /planning/point_cloud -- running arm_perception's mapping_node
                # too would put two publishers on it, and the planner's obstacle
                # map would alternate between whichever arrived last.
                'enable_rviz': 'false',
                'enable_mapping': 'false',
                'min_depth_m': LaunchConfiguration('min_depth_m'),
                'bbox_anchor': LaunchConfiguration('bbox_anchor'),
                'surface_to_centre_m': LaunchConfiguration('surface_to_centre_m'),
                'grasp_z_offset': LaunchConfiguration('grasp_z_offset'),
                'cup_classes': LaunchConfiguration('cup_classes'),
                'goal_fallback_xyz': LaunchConfiguration('goal_fallback_xyz'),
                'yolo_confidence': LaunchConfiguration('yolo_confidence'),
            }.items(),
        ),
    ])
