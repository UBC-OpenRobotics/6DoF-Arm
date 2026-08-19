"""Mapping Node (Action Server)

Subscribe to raw depth streams, transforms incoming cloud into base_link using TF2, clean statistical
and radius outliers, voxel downsample, crop to workspace bounds,
and merge clouds into a single obstacle map for motion.

trigger the map building through CLI:
ros2 service call /mapping_node/build_map std_srvs/srv/Trigger
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Header, Empty

#
# Stand-alone helper functions.
#

def pointcloud2_to_numpy(cloud_msg: PointCloud2) -> np.ndarray:
    """Convert a ``PointCloud2`` message into an ``(N, 3)`` NumPy array.

    Args:
        cloud_msg: The incoming point cloud message.

    Returns:
        An ``(N, 3)`` float64 array of XYZ points, with N being number of points. Points that contain NaN
        or infinite values in any coordinate are dropped.
    """
    # `read_points_numpy` returns a structured/flat array of the requested
    # fields; skip_nans=True drops NaNs.
    points_struct = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True
    )

    if points_struct.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    #np.stack is used to combine the structured array into a regular 2D array of shape (N, 3), 
    # axis -1 means that the new axis is added at the end, so the resulting array has shape (N, 3) where N is the number of points. 
    #The astype(np.float64) ensures that the data type of the resulting array is float64.
    points = np.stack(
        [points_struct["x"], points_struct["y"], points_struct["z"]], axis=-1
    ).astype(np.float64)

    # guard against any remaining NaN or infinite values
    finite_mask = np.isfinite(points).all(axis=1)

    return points[finite_mask]


def numpy_to_pointcloud2(
    points: np.ndarray, frame_id: str, stamp
) -> PointCloud2:
    """Convert an ``(N, 3)`` NumPy array into a ``PointCloud2`` message.

    Args:
        points: An ``(N, 3)`` array of XYZ points.
        frame_id: The frame the output cloud should be stamped with.
        stamp: A ``builtin_interfaces.msg.Time`` stamp for the header.

    Returns:
        A ``PointCloud2`` message containing the given points.
    """

    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    points_list = [tuple(point) for point in points.astype(np.float32)]
    return point_cloud2.create_cloud_xyz32(header, points_list)


def transform_cloud(
    points: np.ndarray, transform
) -> np.ndarray:
    """Apply a TF2 ``TransformStamped`` to an ``(N, 3)`` point array.

    Args:
        points: An ``(N, 3)`` array of XYZ points expressed in the source
            frame of ``transform``.
        transform: A ``geometry_msgs/msg/TransformStamped`` obtained from a
            TF2 buffer lookup.

    Returns:
        An ``(N, 3)`` array of points expressed in the target frame of
        ``transform``.
    """

    # if the input array is empty, return it immediately
    if points.shape[0] == 0:
        return points

    translation = transform.transform.translation
    rotation = transform.transform.rotation

    # Build a rotation matrix from the quaternion without hand-rolling the
    # trigonometry -- this relies on Open3D's quaternion utility
    # (w, x, y, z ordering) rather than writing our own transform math.
    quat_wxyz = np.array([rotation.w, rotation.x, rotation.y, rotation.z])
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(quat_wxyz)
    translation_vec = np.array([translation.x, translation.y, translation.z])

    # return the transformed points by applying the rotation and translation to the input points.\
    # the @ operator performs matrix multiplication, and the .T transposes the rotation matrix to align with the point array.
    return points @ rotation_matrix.T + translation_vec


def merge_clouds(clouds: List[np.ndarray]) -> np.ndarray:
    """Concatenate a list of point arrays into a single array.

    Args:
        clouds: A list of ``(N_i, 3)`` point arrays, all in the same frame.

    Returns:
        A single ``(sum(N_i), 3)`` array. Returns an empty ``(0, 3)`` array
        if ``clouds`` is empty.
    """
    valid_clouds = [cloud for cloud in clouds if cloud.shape[0] > 0]
    if not valid_clouds:
        return np.empty((0, 3), dtype=np.float64)
    return np.concatenate(valid_clouds, axis=0)


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel-downsample a point array using Open3D.

    Args:
        points: An ``(N, 3)`` array of XYZ points.
        voxel_size: The edge length (in meters) of each voxel.

    Returns:
        The downsampled ``(M, 3)`` point array, ``M <= N``.
    """
    if points.shape[0] == 0:
        return points

    #pcd stands for Point Cloud Data.
    pcd = o3d.geometry.PointCloud()
    
    # convert the numpy array of points into an Open3D point cloud object.
    pcd.points = o3d.utility.Vector3dVector(points)

    # apply voxel downsampling to the point cloud using the specified voxel size.
    downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

    return np.asarray(downsampled.points)



def remove_statistical_outliers(
    points: np.ndarray, nb_neighbors: int, std_ratio: float
) -> np.ndarray:
    """Remove statistical outliers using Open3D's statistical filter.

    Args:
        points: An ``(N, 3)`` array of XYZ points.
        nb_neighbors: Number of neighbors used to compute the mean distance
            for each point.
        std_ratio: Standard deviation multiplier threshold; points whose
            mean distance is further than ``std_ratio`` standard deviations
            from the mean are removed.

    Returns:
        The filtered ``(M, 3)`` point array, ``M <= N``.
    """
    if points.shape[0] == 0:
        return points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    filtered_pcd, _inlier_indices = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )
    return np.asarray(filtered_pcd.points)


def remove_radius_outliers(
    points: np.ndarray, radius: float, min_neighbors: int
) -> np.ndarray:
    """Remove radius outliers using Open3D's radius filter.

    Args:
        points: An ``(N, 3)`` array of XYZ points.
        radius: Search radius (in meters) around each point.
        min_neighbors: Minimum number of neighbors required within
            ``radius`` for a point to be kept.

    Returns:
        The filtered ``(M, 3)`` point array, ``M <= N``.
    """
    if points.shape[0] == 0:
        return points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    filtered_pcd, _inlier_indices = pcd.remove_radius_outlier(
        nb_points=min_neighbors, radius=radius
    )
    return np.asarray(filtered_pcd.points)


# def crop_workspace(
#     points: np.ndarray,
#     x_min: float,
#     x_max: float,
#     y_min: float,
#     y_max: float,
#     z_min: float,
#     z_max: float,
# ) -> np.ndarray:
#     """Discard points outside of the configured workspace bounding box/enclosure.

#     Args:
#         points: An ``(N, 3)`` array of XYZ points.
#         x_min: Minimum x bound (meters), inclusive.
#         x_max: Maximum x bound (meters), inclusive.
#         y_min: Minimum y bound (meters), inclusive.
#         y_max: Maximum y bound (meters), inclusive.
#         z_min: Minimum z bound (meters), inclusive.
#         z_max: Maximum z bound (meters), inclusive.

#     Returns:
#         The cropped ``(M, 3)`` point array, ``M <= N``.
#     """
#     if points.shape[0] == 0:
#         return points

#     in_bounds = (
#         (points[:, 0] >= x_min)
#         & (points[:, 0] <= x_max)
#         & (points[:, 1] >= y_min)
#         & (points[:, 1] <= y_max)
#         & (points[:, 2] >= z_min)
#         & (points[:, 2] <= z_max)
#     )
#     return points[in_bounds]


# --------------------------------------------------------------------------- #
# Node implementation.
# --------------------------------------------------------------------------- #


class MappingNode(Node):
    """Builds a static, filtered obstacle map from RealSense point clouds.

    Subscribes to the raw RealSense point cloud stream during the robot's
    startup scan sweep, transforms and accumulates each cloud into
    ``base_link``, and (once the sweep is finished) merges, filters, and
    publishes a single clean obstacle map on ``/planning/point_cloud`` for
    the Motion team's planning pipeline.
    """

    INPUT_TOPIC = "/camera/camera/depth/color/points"
    OUTPUT_TOPIC = "/planning/point_cloud"
    START_TOPIC = "/sweep/start" #TODO: make a wait function to start
    STOP_TOPIC = "/sweep/stop"

    def __init__(self) -> None:
        super().__init__("mapping_node")

        self._declare_parameters()

        # TF2 buffer/listener used for all frame transforms.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Accumulated, already-transformed clouds (each an (N, 3) array in
        # base_link) collected over the course of the scan sweep.
        self._collected_clouds: List[np.ndarray] = []
        self._map_built = False

        self._start_sub = self.create_subscription(
            Empty,
            self.START_TOPIC,
            self._start_callback,
            qos_profile_sensor_data,
        )

        self._stop_sub = self.create_subscription(
            Empty,
            self.STOP_TOPIC,
            self._stop_callback,
            qos_profile_sensor_data,
        )

        self._cloud_sub = self.create_subscription(
            PointCloud2,
            self.INPUT_TOPIC,
            self._cloud_callback,
            qos_profile_sensor_data,
        )

        #qos_profile_sensor_data is for high-frequency sensor data like point clouds, images, and IMU readings. It prioritizes low latency and best-effort delivery

        self._map_pub = self.create_publisher(PointCloud2, self.OUTPUT_TOPIC, 10)

        # # A service lets Planning explicitly
        # # signal "the scan sweep is finished, build the map now" once all
        # # predefined scan poses have been visited.
        
        # self._build_map_srv = self.create_service(
        #     Trigger, "~/build_map", self._build_map_service_callback
        # ) 

        self.get_logger().info(
            f"MappingNode initialized. Subscribed to '{self.INPUT_TOPIC}', "
            f"publishing merged map on '{self.OUTPUT_TOPIC}'. "
        )

    # ------------------------------------------------------------------ #
    # Parameter handling.
    # ------------------------------------------------------------------ #

    def _declare_parameters(self) -> None:
        """Declare all configurable ROS2 parameters with their defaults."""
        self.declare_parameter("target_frame", "rx150/base_link")
        self.declare_parameter("voxel_size", 0.01) 
        self.declare_parameter("statistical_nb_neighbors", 20)
        self.declare_parameter("statistical_std_ratio", 2.0)
        self.declare_parameter("radius_outlier_radius", 0.02)
        self.declare_parameter("radius_outlier_min_neighbors", 10)
        self.declare_parameter("enable_radius_outlier_removal", True)
        

        # Workspace crop bounds (meters, expressed in target_frame).
        # TODO: remove these, might not be necessary if we have walls
        self.declare_parameter("workspace_x_min", -1.0)
        self.declare_parameter("workspace_x_max", 1.0)
        self.declare_parameter("workspace_y_min", -1.0)
        self.declare_parameter("workspace_y_max", 1.0)
        self.declare_parameter("workspace_z_min", -0.1)
        self.declare_parameter("workspace_z_max", 1.5)

    @property
    def target_frame(self) -> str:
        return self.get_parameter("target_frame").get_parameter_value().string_value

    @property
    def voxel_size(self) -> float:
        return self.get_parameter("voxel_size").get_parameter_value().double_value

    @property
    def statistical_nb_neighbors(self) -> int:
        return (
            self.get_parameter("statistical_nb_neighbors")
            .get_parameter_value()
            .integer_value
        )

    @property
    def statistical_std_ratio(self) -> float:
        return (
            self.get_parameter("statistical_std_ratio")
            .get_parameter_value()
            .double_value
        )

    @property
    def radius_outlier_radius(self) -> float:
        return (
            self.get_parameter("radius_outlier_radius")
            .get_parameter_value()
            .double_value
        )

    @property
    def radius_outlier_min_neighbors(self) -> int:
        return (
            self.get_parameter("radius_outlier_min_neighbors")
            .get_parameter_value()
            .integer_value
        )

    @property
    def enable_radius_outlier_removal(self) -> bool:
        return (
            self.get_parameter("enable_radius_outlier_removal")
            .get_parameter_value()
            .bool_value
        )

    @property
    def workspace_bounds(self) -> tuple:
        """Return ``(x_min, x_max, y_min, y_max, z_min, z_max)``."""
        get = lambda name: self.get_parameter(name).get_parameter_value().double_value
        return (
            get("workspace_x_min"),
            get("workspace_x_max"),
            get("workspace_y_min"),
            get("workspace_y_max"),
            get("workspace_z_min"),
            get("workspace_z_max"),
        )

    # ------------------------------------------------------------------ #
    # Subscription callback
    # ------------------------------------------------------------------ #

    def _cloud_callback(self, cloud_msg: PointCloud2) -> None:
        """Receive, transform, and accumulate a single incoming point cloud.

        this callback only converts to NumPy, transform via TF2, and store. All filtering happens
        later in build_map() once the initial sweep is completed.
        """
        points = pointcloud2_to_numpy(cloud_msg)
        self.get_logger().info(
            f"Received cloud on '{self.INPUT_TOPIC}' with {points.shape[0]} "
            f"valid points (frame='{cloud_msg.header.frame_id}')."
        )

        if points.shape[0] == 0:
            self.get_logger().warning("Received empty/invalid cloud, skipping.")
            return

        transform = self._lookup_transform(
            cloud_msg.header.frame_id, cloud_msg.header.stamp
        )
        if transform is None:
            # Warning already logged inside _lookup_transform.
            return

        transformed_points = transform_cloud(points, transform)
        self._collected_clouds.append(transformed_points)

        self.get_logger().info(
            f"Transformed cloud into '{self.target_frame}' and stored "
            f"({transformed_points.shape[0]} points). "
            f"Total stored clouds: {len(self._collected_clouds)}."
        )

    def _lookup_transform(self, source_frame: str, stamp) -> Optional[object]:
        """Look up the TF2 transform from ``source_frame`` to ``target_frame``.

        Args:
            source_frame: The frame the incoming cloud is expressed in. -> camera_optical_frame
            stamp: The timestamp of the incoming cloud message. 

        Returns:
            The ``TransformStamped`` on success, or ``None`` if the
            transform could not be resolved (a warning is logged in that
            case).
        """
        try:
            return self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(stamp),
            )
        except (LookupException, ConnectivityException, ExtrapolationException,
                TransformException) as error:
            self.get_logger().warning(
                f"TF lookup failed for '{source_frame}' -> "
                f"'{self.target_frame}': {error}. Skipping this cloud."
            )
            return None

    # ------------------------------------------------------------------ #
    # Map-building trigger.
    # ------------------------------------------------------------------ #

    def _start_callback(self, msg: Empty) -> None:
        """Callback to start the mapping process."""
        del msg  # Empty message carries no data.
        self.get_logger().info("Mapping sweep started. Accumulating clouds...")

    def _stop_callback(self, msg: Empty) -> None:
        """Callback to stop the mapping process and build the map."""
        del msg  # Empty message carries no data.
        self.get_logger().info("Mapping sweep stopped. Building map...")

        if not self._collected_clouds:
            self.get_logger().warning("No point clouds have been collected yet. Cannot build map.")

        else:
            self.build_map()
            self.get_logger().info(
                f"Obstacle map built and published on '{self.OUTPUT_TOPIC}'."
            )

    # def _build_map_service_callback(
    #     self, request: Trigger.Request, response: Trigger.Response
    # ) -> Trigger.Response:
    #     """Service handler that triggers map building on demand.

    #     Intended to be called once Motion has finished sweeping through all
    #     predefined scan poses.
    #     """
    #     del request  # Trigger.Request carries no fields.

    #     if not self._collected_clouds:
    #         response.success = False
    #         response.message = "No point clouds have been collected yet."
    #         self.get_logger().warning(response.message)
    #         return response

    #     self.build_map()
    #     response.success = True
    #     response.message = (
    #         f"Obstacle map built and published on '{self.OUTPUT_TOPIC}'."
    #     )
    #     return response

    # ------------------------------------------------------------------ #
    # Map building pipeline -- runs once the scan sweep is complete.
    # ------------------------------------------------------------------ #

    def build_map(self) -> None:
        """Merge, filter, and publish the final obstacle map.

        Runs the full post-processing pipeline over all clouds collected
        during the scan sweep:
            merge -> voxel downsample -> statistical outlier removal ->
            (optional) radius outlier removal -> workspace crop -> publish.
        """

        # merge list of points into one single array of points
        merged_points = merge_clouds(self._collected_clouds)
        self.get_logger().info(
            f"Merged {len(self._collected_clouds)} clouds into "
            f"{merged_points.shape[0]} points."
        )

        if merged_points.shape[0] == 0:
            self.get_logger().warning("Merged cloud is empty, nothing to publish.")
            return

        downsampled_points = voxel_downsample(merged_points, self.voxel_size)
        self.get_logger().info(
            f"Voxel downsampling (voxel_size={self.voxel_size}): "
            f"{merged_points.shape[0]} -> {downsampled_points.shape[0]} points."
        )

        statistically_filtered_points = remove_statistical_outliers(
            downsampled_points,
            self.statistical_nb_neighbors,
            self.statistical_std_ratio,
        )
        self.get_logger().info(
            "Statistical outlier removal "
            f"(nb_neighbors={self.statistical_nb_neighbors}, "
            f"std_ratio={self.statistical_std_ratio}): "
            f"{downsampled_points.shape[0]} -> "
            f"{statistically_filtered_points.shape[0]} points "
            f"({downsampled_points.shape[0] - statistically_filtered_points.shape[0]} "
            "removed)."
        )

        if self.enable_radius_outlier_removal:
            radius_filtered_points = remove_radius_outliers(
                statistically_filtered_points,
                self.radius_outlier_radius,
                self.radius_outlier_min_neighbors,
            )
            self.get_logger().info(
                "Radius outlier removal "
                f"(radius={self.radius_outlier_radius}, "
                f"min_neighbors={self.radius_outlier_min_neighbors}): "
                f"{statistically_filtered_points.shape[0]} -> "
                f"{radius_filtered_points.shape[0]} points "
                f"({statistically_filtered_points.shape[0] - radius_filtered_points.shape[0]} "
                "removed)."
            )
        else:
            radius_filtered_points = statistically_filtered_points
            self.get_logger().info("Radius outlier removal disabled, skipping.")

        x_min, x_max, y_min, y_max, z_min, z_max = self.workspace_bounds
        
        
        # cropped_points = crop_workspace(
        #     radius_filtered_points, x_min, x_max, y_min, y_max, z_min, z_max
        # )

        # self.get_logger().info(
        #     f"Workspace crop (x:[{x_min},{x_max}] y:[{y_min},{y_max}] "
        #     f"z:[{z_min},{z_max}]): {radius_filtered_points.shape[0]} -> "
        #     f"{cropped_points.shape[0]} points."
        # )

        self.get_logger().info(
            f"Final obstacle map contains {radius_filtered_points.shape[0]} points."
        )

        self.publish_map(radius_filtered_points)
        self._map_built = True

    def publish_map(self, points: np.ndarray) -> None:
        """Publish the final filtered point cloud on ``/planning/point_cloud``.

        Args:
            points: The final ``(N, 3)`` filtered point array, expressed in ``target_frame`` (base_link).
        """

        # convert the cleaned numpy array of points into a PointCloud2 message
        cloud_msg = numpy_to_pointcloud2(
            points, self.target_frame, self.get_clock().now().to_msg()
        )

        self._map_pub.publish(cloud_msg)
        self.get_logger().info(
            f"Published obstacle map ({points.shape[0]} points) on "
            f"'{self.OUTPUT_TOPIC}' in frame '{self.target_frame}'. "
            "Map publication complete."
        )

        #TODO: shutdown node after publishing the map? or keep it alive for future updates?


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()