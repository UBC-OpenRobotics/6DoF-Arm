# arm_perception

This package provides the perception/vision pipeline for a 6-DoF robotic arm. It includes camera input (upstream realsense driver), RGB image preprocessing, YOLO-based object detection, 3D point extraction, localization, and PC generation nodes.

Currently only runs on Linux.

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
- publish detection results to downstream nodes


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
- If TF lookup fails, the detection is skipped.
- The depth sampling is intentionally local and robust, but it is still a simplified estimate of object center depth.


## `get_3d_point_node.py`

This node acts as a small vision request/response bridge for the vision pipeline. It listens to the latest 3D detections produced by the localization stage and exposes the 3D point of a requested object on a topic. 

### Inputs
The node subscribes to:

- `localization_3d_topic`
  - Default: `/perception/detections_3d`
  - Type: `DetectedObjectArray`

- `object_request_topic`
  - Default: `/vision/find_request`
  - Type: `std_msgs/String`

### Outputs
The node publishes:

- `object_point_topic`
  - Default: `/vision/object_point`
  - Type: `geometry_msgs/PointStamped`

### Important note
- The original implementation uses an action server. For simplicity and testing, it was changed to a topic-based request/response service.
- Currently, it only stores the last seen position of a cup. Once the OD is trained for other objects, the node can be scaled to store the last seen positions of other class names.


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