# arm_perception

This package provides the perception/vision pipeline for a 6-DoF robotic arm. It includes camera input (upstream realsense driver), RGB image preprocessing, YOLO-based object detection, 3D point extraction, localization, and PC generation nodes.

Currently only runs on Linux.

## Sim time — read this before launching

**On hardware:** nothing to do. `use_sim_time` defaults to `false`, everything shares
the system clock.

**In sim:** every node must run on Gazebo's clock. `rx150_pick_place_sim.launch.py`
passes `use_sim_time:=true` down automatically, so the normal path is already correct.

**The trap** is launching this package **standalone** against a running sim.
`perception.launch.py` defaults `use_sim_time` to `false` because its default target is
real hardware. Launched that way in sim, image stamps are in sim time while the node is
on the wall clock, so every stamped TF lookup fails and — with `allow_latest_tf` false,
which is correct — **every detection is dropped**. Nothing is published, and no error
says why.

Pass it explicitly:

```bash
ros2 launch arm_perception perception.launch.py use_sim_time:=true enable_camera:=false \
  color_topic:=/gripper_camera/image_raw \
  depth_topic:=/gripper_camera/depth/image_raw \
  camera_info_topic:=/gripper_camera/camera_info
```

`localization_3d_node` checks this a few seconds after startup and says so if it is
wrong:

```
[ERROR] A simulator is publishing /clock but this node is on the WALL clock.
        ... EVERY DETECTION WILL BE DROPPED -- silently. Relaunch with use_sim_time:=true.
[ERROR] use_sim_time is TRUE but nothing publishes /clock. ... Start Gazebo first.
```

A healthy start logs `Clock check OK: use_sim_time=True, /clock present.`

# Node overview

## `webcam_demo_node.py`
Provides a lightweight webcam input node for testing and demonstration without requiring a depth camera.

Typical responsibilities:
- open a local camera device
- run YOLO inference on each frame
- display bounding boxes
- act as a simple prototype pipeline input


## `realsense_node.py`
Monitors camera topics and provides a health-check service. **Currently commented out** in the launch file because it conflicts with the upstream realsense camera driver.

Responsibilities:
- Logs the first valid frame metadata from each stream (image size, encoding, camera intrinsics such as fx, fy, cx, cy)
- Uses a timer to periodically check whether the camera is still alive and warns if no frames have arrived.


## `color_preprocessing_node.py`
This node preprocesses incoming RGB camera frames before they are sent to downstream perception stages such as object detection. 

Typical responsibilities:
- Optional OpenCV-based filtering
- Optional lighting enhancement, can be especially useful for outdoor workspaces

All of the RGB stream (from `/camera/camera/color/image_raw`) goes through this node first before it is republished on `/perception/color_preprocessed`

The node supports the following parameters:
- `filter`
  - Options: `none`, `bilateral`, `median`, `gaussian`
  - Controls spatial smoothing or denoising
- `light_processing`
  - Options: `none`, `clahe`
  - Improves local contrast in uneven lighting conditions


## `yolo_detector_node.py`
Runs object detection using a YOLO model.

Typical responsibilities:
- load YOLO weights and configuration (current YOLO version is YOLOv8)
- process incoming RGB frames
- detect object classes and bounding boxes
- publish detection results to downstream nodes, each stamped with the source
  image's header (needed downstream to tell a live detection from a stale one)


## `localization_3d_node.py`

This node converts 2D object detections from YOLO node into 3D positions in a robot frame using aligned depth image from the camera. It publishes the resulting 3D detections and RViz markers. This is the part of the pipeline that turns “object seen in the image” into “object located in 3D space relative to the robot.”

### Inputs
The node subscribes to the following topics:

- `detection_topic`  
  Default: `/perception/detections`  
  Type: `DetectedObjectArray`

- `depth_topic`  
  Default: `/camera/camera/aligned_depth_to_color/image_raw`  
  Type: `sensor_msgs/Image`

- `camera_info_topic`  
  Default: `/camera/camera/color/camera_info`  
  Type: `sensor_msgs/CameraInfo`

The code uses `ApproximateTimeSynchronizer` to align detection and depth messages by timestamp, with a `slop` of `0.1` seconds.

### Outputs
The node publishes:

- `DetectedObjectArray` on `/perception/detections_3d`
  - each entry includes the original 2D detection and a 3D position
  - the position is transformed into the configured robot frame

- `MarkerArray` on `/perception/markers`
  - used for RViz visualization
  - each marker is placed at the deprojected object location

### Important notes
- It depends on a valid TF transform between the camera frame and `target_frame`.
- It looks up the transform **at the image capture stamp**. If that fails and
  `allow_latest_tf` is true it falls back to the latest available transform, logging
  once. **Leave `allow_latest_tf` false** — `perception.launch.py` already does; the
  node default is true only for standalone use.
  - That fallback used to be required because nothing in the stack set `use_sim_time`,
    so image stamps were sim time while the TF buffer ran on wall time and every
    exact-stamp lookup failed. That is fixed (see *Sim time* above), and the fallback
    is now a hazard rather than a workaround: **the sweep is what locates the cup**, and
    "latest" places a detection using wherever the arm is *now* rather than where it was
    when the shutter fired. Mid-slew that is a large, silent error — the same class of
    mistake that smeared the obstacle cloud by 26°.
  - Dropping a detection is the safe failure. Placing it wrongly is not.
- If both lookups fail, the detection is skipped rather than placed wrongly.
- `min_depth_m` / `max_depth_m` bound valid depths. The floor matters: the scan
  posture holds the camera close to the scene, and a D435 cannot focus closer than
  ~0.2 m while a D405 can.
- The depth sampling is intentionally local and robust, but it is still a simplified estimate of object center depth.


## `get_3d_point_node.py` (node name: `vision_bridge`)

**This node is the seam between perception and the motion stack.** It is the only
thing the pick-and-place orchestrator talks to; everything upstream of it is
perception's business. It caches the 3D detections from the localization stage and,
on request, publishes the position of one requested object in the arm's planning
frame.

### Inputs
- `localization_3d_topic` — default `/perception/detections_3d`, `DetectedObjectArray`
- `object_request_topic` — default `/vision/find_request`, `std_msgs/String`
  (`"cup"` or `"goal"`)

### Outputs
- `object_point_topic` — default `/vision/object_point`, `geometry_msgs/PointStamped`

### Two rules that make this safe to drive a real arm with

1. **Silence means "not found".** If nothing matches, the node publishes **nothing**.
   The orchestrator then times out and aborts the mission cleanly. The earlier
   version published a default-constructed `PointStamped` on failure — that is
   `(0, 0, 0)` with an empty `frame_id`, which the orchestrator accepts as a valid
   target and drives the arm into **its own base**. Never reintroduce an
   unconditional publish at the end of the request callback.
2. **Detections expire.** A cached hit older than `detection_ttl_sec` is not an
   answer, it is a memory. Without it the arm confidently reaches for a cup that was
   carried off two minutes ago. The TTL is measured against message *receipt* time
   (`time.monotonic`), not the header stamp, so it is immune to the sim-time /
   wall-time split between Gazebo's stamps and the node's clock.

Caching is deliberate, not laziness: on this arm the camera is on the gripper, so the
cup is normally seen **during the scan sweep** and the mission asks for it a phase
later, when the arm has already returned home and is no longer looking at it.

### Requests are served asynchronously
If nothing fresh is cached when a request lands, the node holds the request open for
`response_wait_sec` and answers the moment a matching detection arrives, rather than
blocking. On expiry it either answers with the configured fallback or stays silent.

### Parameters

| param | default | meaning |
| --- | --- | --- |
| `planning_frame` | `rx150/base_link` | frame stamped on the response |
| `cup_classes` | `[cup]` | detector class names that satisfy a `"cup"` request |
| `goal_classes` | *(empty)* | class names for a `"goal"` request |
| `cup_fallback_xyz` | *(none)* | deliberately unset — an unseen cup must abort, not guess |
| `goal_fallback_xyz` | `[0.20, 0.18, 0.16]` | **placeholder** drop-off point |
| `detection_ttl_sec` | `45.0` | older cached detections are not answers |
| `response_wait_sec` | `3.0` | how long to hold an unanswerable request open |
| `min_confidence` | `0.0` | extra gate above the detector's own threshold |
| `cache_policy` | `best` | `best` keeps the most confident hit per class within the TTL; `latest` keeps the newest |
| `z_offset` | `0.0` | added to the detected z (the detector reports the bbox centre) |

The class lists decouple the two vocabularies: the orchestrator asks in task terms
(`"cup"`, `"goal"`), the detector answers in COCO/custom class names, and this mapping
is a parameter rather than code. Adding a request kind is a two-line change.

### `cup_classes` is a priority order, not a set

The list is tried **in order**; the first class with a fresh detection wins, and
confidence only breaks ties *within* a class. Ranking by confidence across the whole
list is actively dangerous: in sim a blue obstacle cylinder scores `vase` **0.49**
while the real cup scores `cup` **0.06**, so a max-confidence rule answered a cup
request with the *obstacle's* position and sent the arm to grasp it. Only add aliases
the target object itself produces — never ones other scene objects produce.

### The cache keeps the *best* hit per class, not the latest

`cache_policy: best`. The camera is on the gripper and pans across the whole scene
during the sweep: it sees the cup, then everything else. With `latest`, one spurious
low-confidence hit on an obstacle later in the sweep overwrites the genuine cup
detection, and the arm is sent to the obstacle — observed exactly that in sim. A stale
entry always loses regardless of how confident it was. Use `latest` only if objects
actually move between observations.

> **The goal is a placeholder.** `goal_classes` is empty by default because the model
> has no drop-off class, so a `"goal"` request always answers `goal_fallback_xyz` and
> logs a warning saying so. That warning is not noise — read it as "the arm is about
> to place the cup at a coordinate nobody looked at."



## `mapping_node.py`

This node builds a cleaned obstacle map from incoming 3D point clouds gathered during a robot scan sweep. It transforms each incoming cloud into the robot base frame, accumulates the point clouds over time, filters out noise, and, once the sweep is finished, publishes a final merged map for planning or motion execution.

### Inputs
The node subscribes to:
- `PointCloud2` on `/camera/camera/depth/color/points`
- `Empty` on `/sweep/start` but is not used in current implementation
- `Empty` on `/sweep/stop` to trigger map building

### Outputs
The node publishes:
- `PointCloud2` on `/planning/point_cloud`

### Parameters
The node declares the following configurable parameters:

- `target_frame`
  - Default: `rx150/base_link`
  - Frame used to transform and publish the final map

- `voxel_size`
  - Default: `0.01`
  - Controls downsampling resolution

- `statistical_nb_neighbors`
  - Default: `20`
  - Number of neighbors used for statistical outlier removal

- `statistical_std_ratio`
  - Default: `2.0`
  - Threshold for removing points with unusually large neighbor distances

- `radius_outlier_radius`
  - Default: `0.02`
  - Radius used in radius-based outlier filtering

- `radius_outlier_min_neighbors`
  - Default: `10`
  - Minimum neighbors required within the radius

- `enable_radius_outlier_removal`
  - Default: `True`
  - Enables or disables the radius filter stage

- Workspace bounds:
  - `workspace_x_min`, `workspace_x_max`
  - `workspace_y_min`, `workspace_y_max`
  - `workspace_z_min`, `workspace_z_max`
  - Useful if the enclosure does not have walls
  - Currently commented out from the crop step in the actual implementation.

### Processing pipeline
The node performs the following steps:

1. Convert each incoming `PointCloud2` to an `(N, 3)` NumPy array.
2. Remove NaN or infinite points.
3. Look up the TF transform from the cloud source frame to the target frame.
4. Transform the cloud from the source frame into the target frame.
5. Store each transformed cloud in an internal list.
6. When the stop signal is received on `/sweep/stop`, call `build_map()`.

The `build_map()` function then:
1. merges all stored clouds into one large array
2. voxel-downsamples the cloud using Open3D
3. removes statistical outliers
4. optionally removes radius outliers
5. publishes the cleaned cloud as a `PointCloud2`

### Notes
- The node is designed to accumulate clouds over a scan or sweep and only build the map once the sweep ends.
- TF availability is required for successful transformation into the target frame.
- The node is built around a scan-and-merge workflow, which is useful for map generation from static scene coverage.


## Regression tests

`arm_perception/test/test_vision_bridge.py` pins the contract's safety properties
(silence on not-found, TTL expiry, goal-is-not-cup, priority ordering, best-not-latest
caching). Each check corresponds to a bug that actually shipped or was caught in a sim
run. Run it with the workspace sourced:

```bash
python3 arm_perception/test/test_vision_bridge.py
```

It exits non-zero on failure.

# Run it

## Running vision with the arm
Add dependencies to the Dockerfile and include ```perception.launch.py`` as a launch description in the robot's launch file. 

```python
IncludeLaunchDescription(
  PythonLaunchDescriptionSource([
    PathJoinSubstitution([
      FindPackageShare('arm_perception'),
      'launch',
      'perception.launch.py',
    ])
  ]),
)
```

## Running vision in a separate container from the arm

1. Build one time

```bash
cd ~/6DoF-Arm/docker
docker compose build
```

2. Launch vision/perception

```bash
cd ~/6DoF-Arm/docker
docker compose up
```

This will launch `perception.launch.py`, which starts RViz and all nodes except `webcam_demo_node` (`realsense_node` is still commented out for now).


3. Optional: using a realsense playback
After launching, open the container's bash and source ros and workspace setup:

```bash
docker exec -it <container_name> bash
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
```

Play the realsense bag file:

```bash
ros2 launch realsense2_camera rs_launch.py   align_depth.enable:=true   pointcloud.enable:=true   rosbag_filename:=/ros2_ws/bag_files/<bag_file_name>.bag
```


## Webcam demo
No docker needed. Refer to `WEBCAM_DEMO.md`


# Important Notes

- Camera calibration and alignment are essential for accurate 3D results.
- Depth-based 3D conversion depends on valid camera intrinsics and proper frame definitions and TF tree published by `robot_state_publisher`
- Object detection results should be checked against the robot’s coordinate conventions and task requirements.
- The exact topic names and launch configuration may vary depending on the rest of the workspace.