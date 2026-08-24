"""Perception pipeline: camera -> YOLO -> 3D localization -> vision bridge.

One launch file for both targets. The pipeline is identical; only the *front*
differs, so every camera-specific topic is an argument:

    hardware : RealSense driver     -> /camera/camera/...        (defaults)
    sim      : Gazebo depth camera  -> /gripper_camera/...       (see the
               `sim` preset in rx150_pick_place_sim.launch.py)

The tail of the pipeline -- `vision_bridge` -- answers the orchestrator's
VISION CONTRACT (`/vision/find_request` -> `/vision/object_point`), so bringing
this up with `use_vision_stub:=false` is all that is needed to replace
`vision_placeholder` with real detection.

Embedding this in a mission launch: pass `enable_rviz:=false` (the mission stack
runs its own RViz) and `enable_mapping:=false` (the mission's scene_sweep_mapper
owns /planning/point_cloud -- two publishers on it make the planner's obstacle
map alternate between them).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _floats(text, expected=3):
    """Parse a launch-argument string like '[0.2, 0.18, 0.16]' into floats.

    Launch arguments are strings, but `goal_fallback_xyz` has to reach the node
    as a real double array -- hence the OpaqueFunction wrapping this file.
    """
    cleaned = str(text).strip().strip('[]')
    if not cleaned:
        return []
    values = [float(v) for v in cleaned.replace(',', ' ').split()]
    if len(values) != expected:
        raise RuntimeError(
            'Expected %d comma-separated numbers, got %r' % (expected, text)
        )
    return values


def _strings(text):
    """Parse '[cup, mug]' or 'cup,mug' into a list of class names."""
    cleaned = str(text).strip().strip('[]')
    return [v.strip().strip('"\'') for v in cleaned.split(',') if v.strip()]


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg_dir = get_package_share_directory('arm_perception')
    yolo_config = os.path.join(pkg_dir, 'config', 'yolo_params.yaml')
    planning_frame = arg('planning_frame')

    color_topic = arg('color_topic')
    preprocess = arg('enable_preprocessing').lower() in ('true', '1')
    # YOLO reads the preprocessed stream when preprocessing is on, and the raw
    # camera stream when it is off -- so turning preprocessing off cannot leave
    # the detector subscribed to a topic nobody publishes.
    detector_input = arg('preprocessed_topic') if preprocess else color_topic

    use_sim_time = arg('use_sim_time').lower() in ('true', '1')

    nodes = []

    if preprocess:
        nodes.append(Node(
            package='arm_perception',
            executable='color_preprocessing_node',
            name='color_preprocessing_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'filter': arg('preprocessing_filter'),
                'light_processing': arg('light_processing'),
                'input_topic': color_topic,
                'output_topic': arg('preprocessed_topic'),
            }],
            output='screen',
        ))

    nodes.append(Node(
        package='arm_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        parameters=[yolo_config, {
            'use_sim_time': use_sim_time,
            'input_topic': detector_input,
            'confidence_threshold': float(arg('yolo_confidence')),
        }],
        output='screen',
    ))

    nodes.append(Node(
        package='arm_perception',
        executable='localization_3d_node',
        name='localization_3d_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'depth_scale': float(arg('depth_scale')),
            'target_frame': planning_frame,
            'depth_topic': arg('depth_topic'),
            'camera_info_topic': arg('camera_info_topic'),
            'min_depth_m': float(arg('min_depth_m')),
            'max_depth_m': float(arg('max_depth_m')),
            'allow_latest_tf': arg('allow_latest_tf').lower() in ('true', '1'),
            'surface_to_centre_m': float(arg('surface_to_centre_m')),
            'bbox_anchor': arg('bbox_anchor'),
            'bottom_inset_frac': float(arg('bottom_inset_frac')),
        }],
        output='screen',
    ))

    # this node is the only thing the orchestrator talks to.
    nodes.append(Node(
        package='arm_perception',
        executable='get_3d_point_node',
        name='vision_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planning_frame': planning_frame,
            # An empty list has no inferable parameter type in rclpy, so the
            # 'none' case is spelled as a sentinel the node filters out:
            # [''] for class lists, [nan] for a disabled fallback point.
            'cup_classes': _strings(arg('cup_classes')) or [''],
            'goal_classes': _strings(arg('goal_classes')) or [''],
            'goal_fallback_xyz': (
                _floats(arg('goal_fallback_xyz')) or [float('nan')]),
            'detection_ttl_sec': float(arg('detection_ttl_sec')),
            'response_wait_sec': float(arg('response_wait_sec')),
            'z_offset': float(arg('grasp_z_offset')),
        }],
        output='screen',
    ))

    # Bridges the RealSense driver's own TF tree onto the arm's. Not needed in
    # sim, where the URDF already publishes the camera optical link.
    if arg('enable_camera_tf').lower() in ('true', '1'):
        # Deliberately IDENTITY. The real mounting pose lives in the URDF joint
        # (rx150_realsense_camera.urdf.xacro) so the extrinsics have exactly one
        # home; splitting them across a URDF offset and a static transform is how
        # calibrations end up double-counted.
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_bridge',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', arg('camera_mount_frame'),
                '--child-frame-id', arg('camera_optical_frame'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ))

    if arg('enable_mapping').lower() in ('true', '1'):
        nodes.append(Node(
            package='arm_perception',
            executable='mapping_node',
            name='mapping_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_scale': float(arg('depth_scale')),
                'enable_radius_outlier_removal': True,
                'target_frame': planning_frame,
                'voxel_size': 0.005,
            }],
            output='screen',
        ))

    if arg('enable_rviz').lower() in ('true', '1'):
        rviz_config = os.path.join(pkg_dir, 'config', 'perception.rviz')
        # Only pass -d if the config actually shipped; otherwise RViz fails to
        # load and comes up blank with an error.
        rviz_args = ['-d', rviz_config] if os.path.isfile(rviz_config) else []
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=rviz_args, output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planning_frame', default_value='rx150/base_link'),

        # --- Camera front end -------------------------------------------------
        DeclareLaunchArgument(
            'enable_camera', default_value='true',
            description='Launch the upstream RealSense driver. false in sim, '
                        'where Gazebo is the camera.',
        ),
        DeclareLaunchArgument('camera_serial_no', default_value=''),
        DeclareLaunchArgument(
            'color_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument(
            'depth_scale', default_value='0.001',
            description='Depth units -> metres. 0.001 for a RealSense 16UC1 '
                        'image; ignored for a 32FC1 image, which is already in '
                        'metres (Gazebo).',
        ),
        DeclareLaunchArgument(
            'min_depth_m', default_value='0.05',
            description='Depths below this are treated as invalid. Raise to ~0.2 '
                        'for a D435, which cannot focus closer.',
        ),
        DeclareLaunchArgument('max_depth_m', default_value='3.0'),
        DeclareLaunchArgument(
            'bbox_anchor', default_value='centre',
            description="Where in the detection box to sample depth. 'centre' is "
                        "simple but wanders with viewing angle; 'bottom' samples "
                        "where the object meets the surface it stands on, which "
                        "is stable across angles and needs no per-pose "
                        "correction. Pair 'bottom' with grasp_z_offset = +half "
                        "the object height.",
        ),
        DeclareLaunchArgument(
            'bottom_inset_frac', default_value='0.12',
            description='For bbox_anchor=bottom, how far up from the bottom edge '
                        'to sample, as a fraction of box height. The very last '
                        'row often catches the surface behind the object.',
        ),
        DeclareLaunchArgument(
            'surface_to_centre_m', default_value='0.0',
            description='Push the detected point away from the camera by this '
                        'much, turning a near-face hit into an object centre. '
                        'Set to roughly half the target depth (e.g. a cup '
                        'radius); 0.0 keeps the raw surface point.',
        ),
        DeclareLaunchArgument(
            'allow_latest_tf', default_value='false',
            description='Fall back to the latest transform when none exists at '
                        "the capture stamp. OFF: the camera rides on the arm, "
                        'so the newest transform describes where the arm ended '
                        'up, not where it was looking -- a detection localized '
                        'that way lands in the wrong place. Dropping it is '
                        'safer. This defaulted true while the perception nodes '
                        'ran on the wall clock and every stamped lookup failed; '
                        'use_sim_time fixes that properly.',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description="Read time from Gazebo's /clock. True in sim, where "
                        'images and TF are both stamped in sim time; false on '
                        'hardware, where everything is on the wall clock.',
        ),

        # --- Preprocessing ----------------------------------------------------
        DeclareLaunchArgument(
            'yolo_confidence', default_value='0.5',
            description='Detector confidence threshold. Lower it in sim: an '
                        'untextured Gazebo primitive scores far below a photo '
                        'of the same object, even when the box is placed '
                        'correctly on it.',
        ),
        DeclareLaunchArgument('enable_preprocessing', default_value='true'),
        DeclareLaunchArgument(
            'preprocessed_topic', default_value='/perception/color_preprocessed'),
        DeclareLaunchArgument(
            'preprocessing_filter', default_value='median',
            description='none | bilateral | median | gaussian'),
        DeclareLaunchArgument(
            'light_processing', default_value='none', description='none | clahe'),

        # --- Vision bridge (the motion-stack seam) ----------------------------
        DeclareLaunchArgument(
            'cup_classes', default_value='[cup]',
            description='Detector class names that satisfy a "cup" request.',
        ),
        DeclareLaunchArgument(
            'goal_classes', default_value='[]',
            description='Detector class names that satisfy a "goal" request. '
                        'Empty means the goal is always the fallback point below.',
        ),
        DeclareLaunchArgument(
            'goal_fallback_xyz', default_value='[0.20, 0.18, 0.16]',
            description='PLACEHOLDER drop-off point, used when goal_classes is '
                        'empty or nothing matching is seen. There is deliberately '
                        'no cup fallback: an unseen cup must fail the mission, '
                        'not send the arm to a guess.',
        ),
        DeclareLaunchArgument(
            'detection_ttl_sec', default_value='300.0',
            description='A cached detection older than this is not an answer. '
                        'MUST exceed one full sweep: the sweep is what finds '
                        'the cup, and a sighting from the first waist station '
                        'has to still be valid when the mission asks, a whole '
                        'sweep later. Measured sweep 123 s in sim and slower on '
                        'real servos, so 45.0 (the old value) expired every '
                        'sighting from the first two thirds of the sweep.'),
        DeclareLaunchArgument('response_wait_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'grasp_z_offset', default_value='0.0',
            description='Added to the detected z. The detector reports the '
                        'bounding-box centre; nudge the grasp point here.',
        ),

        # --- Optional extras --------------------------------------------------
        DeclareLaunchArgument(
            'enable_rviz', default_value='true',
            description='false when embedded in a mission launch that runs its own.'),
        DeclareLaunchArgument(
            'enable_mapping', default_value='true',
            description='false when scene_sweep_mapper owns /planning/point_cloud.'),
        DeclareLaunchArgument(
            'enable_camera_tf', default_value='true',
            description='Publish the static transform that grafts the RealSense '
                        'driver TF tree onto the arm. false in sim, where the '
                        'URDF already provides the camera frame.',
        ),
        DeclareLaunchArgument(
            'camera_mount_frame', default_value='rx150/camera_mount_link',
            description='PARENT: the camera link in the robot description, from '
                        'rx150_realsense_camera.urdf.xacro. This must exist, or '
                        'the transform below parents onto nothing and the camera '
                        'stays a disconnected TF island.',
        ),
        DeclareLaunchArgument(
            'camera_optical_frame', default_value='camera_link',
            description='CHILD: the ROOT frame of the RealSense driver tree, not '
                        'an optical frame. The driver already publishes '
                        'camera_link -> camera_depth_optical_frame and friends, '
                        'so bridging at the root gives every one of its frames a '
                        'path to rx150/base_link. Bridging straight to an optical '
                        'frame instead would duplicate part of the driver tree '
                        'and create two parents for it.',
        ),

        # Upstream RealSense driver.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py',
            ]),
            launch_arguments={
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'serial_no': LaunchConfiguration('camera_serial_no'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_camera')),
        ),

        OpaqueFunction(function=_setup),
    ])
