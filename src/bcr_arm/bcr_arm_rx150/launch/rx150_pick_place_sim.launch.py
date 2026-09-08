"""One-command pick-and-place mission in Gazebo sim (no hardware needed).

Sim analogue of rx150_pick_place.launch.py: brings up the Gazebo sim stack
(arm + planner + RRT + executors + gripper + cloud relay) plus the placeholders
and the orchestrator, so the full mission + event pipeline can be exercised end to
end on your machine.

  - rx150_dls_sim_stack.launch.py  (Gazebo + planner + executors + gripper)
  - perception.launch.py           (REAL detection: YOLO on the gripper camera)
  - scene_sweep_mapper             (real camera sweep, triggered by /sweep/start)
  - rx150_pick_place_orchestrator  (the conductor)

Nothing moves until you start the mission (unless autostart:=true):

    ros2 topic pub --once /mission/start std_msgs/msg/Empty '{}'

Args:
  autostart        (false) start the mission immediately instead of waiting.
  use_vision_stub  (false) run vision_placeholder (canned points) instead of
                   the real perception pipeline. The cup is DETECTED by default:
                   YOLO runs on the Gazebo gripper camera, localization_3d_node
                   projects the detection into rx150/base_link, and vision_bridge
                   answers the orchestrator. Flip this to true to fall back to
                   canned points if detection misbehaves.
                   The GOAL is still a placeholder point (goal_fallback_xyz) --
                   there is nothing in the world for the detector to call a
                   drop-off target.
  (The sweep is always the REAL camera sweep in sim -- data_collector
   scene_sweep_mapper in triggered mode. There is no canned-cloud option here;
   phase 1 scans for real and waits for sweep:complete before planning.)
  scan_waist_angles  ([]) waist angles in RADIANS the sweep stops at, in order.
                   Empty keeps the node default: a 90 deg front sector in 3
                   stations, '[-0.785,0.0,0.785]'. Pass a list to scan somewhere
                   else; the full circle is 8 stations,
                   '[-3.10,-2.356,-1.571,-0.785,0.0,0.785,1.571,2.356]'.
                   Keep stations <=45 deg apart or the camera's 54.5 deg FOV
                   leaves a blind wedge. ONLY THE SWEPT SECTOR IS MAPPED; the
                   planner reads the rest as empty space.
  planning_frame   (rx150/base_link)
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
    # Gazebo's clock, not the wall clock. The camera stamps clouds in sim time
    # and TF is published in sim time, so a node on the wall clock cannot match
    # a cloud to the pose the arm held when it was captured.
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false'),
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
            'approach_back_off', default_value='0.045',
            description='How far BEHIND the object the hover sits, so the gripper '
                        'descends diagonally instead of dropping its rear bracket '
                        'onto the cup.'),
        DeclareLaunchArgument(
            'sweep_timeout_sec', default_value='240.0',
            description='Deadline for a hung sweep, not a schedule. A two-look '
                        'scan runs ~2 min in sim and slower on real servos.'),
        DeclareLaunchArgument('use_vision_stub', default_value='false'),
        DeclareLaunchArgument(
            'cup_classes', default_value='[cup, frisbee, bowl, toilet]',
            description='PRIORITY ORDER of detector class names accepted as "the '
                        'cup" -- the first class with a fresh detection wins.\n'
                        'The odd aliases are artefacts of running a COCO-trained '
                        'yolov8n on flat-shaded Gazebo primitives. It localises the '
                        'cup correctly every time but labels it by silhouette, so '
                        'from above the cup reads as "frisbee" and "cup" may never '
                        'fire at all.\n'
                        'Only add classes the CUP itself produces -- "umbrella" and '
                        '"vase" belong to the obstacles, and adding either sends the '
                        'arm to grasp one. Narrow to [cup] once a task-trained model '
                        'replaces yolov8n.pt.',
        ),
        DeclareLaunchArgument(
            'yolo_confidence', default_value='0.15',
            description='Low on purpose -- see cup_classes. A COCO model scores '
                        'an untextured Gazebo primitive poorly, so this catches the '
                        'sim cup while still suppressing noise. Raise to the usual '
                        '0.5 once the detector is trained on the task objects.',
        ),
        DeclareLaunchArgument(
            'goal_fallback_xyz', default_value='[0.20, 0.22, 0.06]',
            description='PLACEHOLDER drop-off point, on the floor beside the cup '
                        '(the cup now stands on the ground, not on a table). '
                        'Nothing in the world is a detectable goal, so this '
                        'stays a fixed vector.',
        ),
        DeclareLaunchArgument(
            'carry_level', default_value='false',
            description='Keep the gripper level so a grasped cup cannot tip.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Read time from Gazebo's /clock. Must stay true in "
                        'sim: stamped TF lookups fail against a wall clock.',
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
            'scan_waist_angles', default_value='[]',
            description='Waist angles (radians) the sweep stops at, in order. '
                        'Empty keeps the node default: a 90 deg front sector in '
                        "3 stations, '[-0.785, 0.0, 0.785]'. Pass a list to scan "
                        'somewhere else; the full circle is 8 stations, '
                        '\'[-3.10,-2.356,-1.571,-0.785,0.0,0.785,1.571,2.356]\'. Stations '
                        "must be no more than ~45 deg apart or the D435i's "
                        '54.5 deg FOV leaves a blind wedge between them. ONLY '
                        'THE SWEPT SECTOR IS MAPPED; the planner reads '
                        'everything else as empty space.',
        ),
        DeclareLaunchArgument(
            'grasp_clearance_radius', default_value='0.08',
            description='Radius of the exclusion bubble around the detected '
                        'object: cloud points inside it are the object itself, '
                        'not obstacles. Raise it when the cup or nearby clutter '
                        'blocks the approach to the cup.',
        ),
        DeclareLaunchArgument(
            'obstacle_height_threshold', default_value='0.01',
            description='Map points at or below this are ground. Keep it above '
                        'the sweep z_min or it drops nothing.',
        ),
        DeclareLaunchArgument(
            'enable_ground_plane_removal', default_value='false',
            description='Fit and delete the work surface from the swept map, at '
                        'whatever height and tilt it actually came out at. The '
                        'z_min crop only removes a surface that landed exactly '
                        'where the model says; this removes one that did not. '
                        'Guarded: the fit is ignored unless it is near-level, '
                        'low, and a large share of the cloud.',
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
            'check_grasp', default_value='false',
            description='Verify the grasp by checking the fingers stalled short '
                        'of their commanded opening, and abort if they did not.\n'
                        'OFF in sim. The check infers a grasp from finger travel, '
                        'and a compliant Gazebo contact lets the fingers reach their '
                        'target while still holding, so a good grasp reads as a '
                        'failure. Sim has a better test anyway -- ask Gazebo whether '
                        'the object moved.\n'
                        'The node default stays TRUE so hardware still verifies, '
                        'where there is no ground truth to check instead.',
        ),
        # --- Object geometry -------------------------------------------------
        # These describe the OBJECT and the observation pose, not the pipeline.
        # Launch arguments rather than inline values so they can be retuned per
        # run without editing the file:
        #   ros2 launch ... rx150_pick_place_sim.launch.py grasp_z_offset:=0.025
        # Same argument names as the hardware launch, so the procedure is identical.
        DeclareLaunchArgument(
            'bbox_anchor', default_value='bottom',
            description='Where in the detection box to sample depth. "bottom" '
                        'anchors where the object meets its surface, which does '
                        'not wander with viewing angle the way the box centre does.',
        ),
        DeclareLaunchArgument(
            'surface_to_centre_m', default_value='0.021',
            description="The object's RADIUS, pushing the point from the near face "
                        'the camera sees to the object axis. 0.021 is the shot glass.',
        ),
        DeclareLaunchArgument(
            'grasp_z_offset', default_value='0.018',
            description='Raises the anchored point to the grasp height. MEASURED, '
                        'not derived -- the bottom anchor misses the true base by an '
                        'amount that changes SIGN with viewing angle, so it cannot '
                        'be predicted from the geometry. Re-measure after any change '
                        'to observe_joints, the object height, or what it stands on.',
        ),
        DeclareLaunchArgument(
            'grasp_value', default_value='0.60',
            description='How far to close on the cup. 0.0 (closed) .. 1.0 (open).\n'
                        'This maps to the gap between the PAD SURFACES, which is '
                        '22 mm narrower than the finger-link separation the joint '
                        'states report, because each pad is inset 11 mm from its '
                        'link origin in the URDF collision geometry:\n'
                        '    pad gap = 0.008 + value * 0.044\n'
                        'so the gripper holds objects 8..52 mm wide -- NOT the '
                        '30..74 mm the link separation suggests.\n'
                        'Aim for a pad gap ~2 mm per side under the object width. '
                        'Less squeeze than that and Gazebo contact min_depth eats '
                        'most of it, leaving the object sitting in an open hand.',
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
                'grasp_clearance_radius': LaunchConfiguration('grasp_clearance_radius'),
                'obstacle_height_threshold': LaunchConfiguration('obstacle_height_threshold'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # Vision stub -- canned points, only when detection is turned off.
        Node(
            package='bcr_arm_rx150',
            executable='vision_placeholder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_vision_stub')),
            parameters=[{'planning_frame': planning_frame,
                         'use_sim_time': use_sim_time}],
        ),

        # REAL perception, pointed at the Gazebo gripper camera instead of a
        # RealSense. Same pipeline and same nodes as hardware -- only the three
        # camera topics differ, which is the whole point of parameterising them.
        #
        # The Gazebo depth image is 32FC1 in metres (depth_scale is unused for
        # float depth), and it is inherently aligned to colour because both come
        # from the same sensor at the same resolution.
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
                # Gazebo is the camera here.
                'enable_camera': 'false',
                'color_topic': '/gripper_camera/image_raw',
                'depth_topic': '/gripper_camera/depth/image_raw',
                'camera_info_topic': '/gripper_camera/camera_info',
                'depth_scale': '1.0',
                # Matches the D435i modelled in the URDF (min-Z ~0.2 m).
                'min_depth_m': '0.2',
                'bbox_anchor': LaunchConfiguration('bbox_anchor'),
                'surface_to_centre_m': LaunchConfiguration('surface_to_centre_m'),
                'grasp_z_offset': LaunchConfiguration('grasp_z_offset'),
                # The URDF already publishes the camera optical link, so there is
                # no second TF tree to bridge onto the arm's.
                'enable_camera_tf': 'false',
                # The sim stack runs its own RViz, and scene_sweep_mapper owns
                # /planning/point_cloud -- a second publisher on it would make
                # the planner's obstacle map alternate between two maps.
                'enable_rviz': 'false',
                'enable_mapping': 'false',
                'yolo_confidence': LaunchConfiguration('yolo_confidence'),
                'cup_classes': LaunchConfiguration('cup_classes'),
                'goal_fallback_xyz': LaunchConfiguration('goal_fallback_xyz'),
                # localization_3d_node transforms detections with TF too, so it
                # needs the same clock as everything else.
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # The real camera sweep -- always, in triggered mode. Sim has a depth
        # camera, so there is no reason to feed the planner a canned cloud here;
        # phase 1 fires /sweep/start, this node physically scans, and the mission
        # waits for its sweep:complete before planning anything.
        Node(
            package='data_collector',
            executable='scene_sweep_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wait_for_trigger': True,
                # The orchestrator owns the arm the instant the scan ends, so a
                # "return home" here would only unfold the arm across the scene
                # and fold it back. Standalone sweeps still go home.
                'return_to_home': False,
                'world_frame': planning_frame,
                'frame_id': planning_frame,
                'scan_posture': LaunchConfiguration('scan_posture'),
                'enable_ground_plane_removal': ParameterValue(
                    LaunchConfiguration('enable_ground_plane_removal'),
                    value_type=bool),
                'scan_waist_angles': ParameterValue(
                    LaunchConfiguration('scan_waist_angles'),
                    value_type=List[float]),
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
            }],
        ),

        # The conductor.
        Node(
            package='bcr_arm_rx150',
            executable='rx150_pick_place_orchestrator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'planning_frame': planning_frame,
                'autostart': LaunchConfiguration('autostart'),
                # Orchestrator toggles the level constraint on/off around the
                # grasp so only the carrying moves are constrained.
                'carry_level': LaunchConfiguration('carry_level'),
                'check_grasp': ParameterValue(
                    LaunchConfiguration('check_grasp'), value_type=bool),
                'pregrasp_value': ParameterValue(
                    LaunchConfiguration('pregrasp_value'), value_type=str),
                'grasp_value': ParameterValue(
                    LaunchConfiguration('grasp_value'), value_type=str),
                'approach_height': ParameterValue(
                    LaunchConfiguration('approach_height'), value_type=float),
                'approach_back_off': ParameterValue(
                    LaunchConfiguration('approach_back_off'), value_type=float),
                'phase_delay_sec': ParameterValue(
                    LaunchConfiguration('phase_delay_sec'), value_type=float),
                'sweep_timeout_sec': ParameterValue(
                    LaunchConfiguration('sweep_timeout_sec'), value_type=float),
                'observe_joints': ParameterValue(
                    LaunchConfiguration('observe_joints'), value_type=List[float]),
            }],
        ),
    ])
