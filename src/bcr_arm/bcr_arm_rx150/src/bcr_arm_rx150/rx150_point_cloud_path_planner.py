#!/usr/bin/env python3

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2


GridIndex = Tuple[int, int]


class Rx150PointCloudPathPlanner(Node):
    """Plan a simple collision-aware Cartesian path from a point cloud."""

    def __init__(self) -> None:
        super().__init__('rx150_point_cloud_path_planner')

        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('point_cloud_topic', '/planning/point_cloud')
        self.declare_parameter('joint_state_topic', '/rx150/joint_states')
        self.declare_parameter('target_topic', '/cartesian_target')
        self.declare_parameter('path_topic', '/planned_cartesian_path')
        self.declare_parameter('grid_resolution', 0.02)
        self.declare_parameter('grid_x_min', 0.05)
        self.declare_parameter('grid_x_max', 0.38)
        self.declare_parameter('grid_y_min', -0.25)
        self.declare_parameter('grid_y_max', 0.25)
        self.declare_parameter('obstacle_height_threshold', 0.01)
        self.declare_parameter('obstacle_inflation_cells', 1)
        self.declare_parameter('path_height_margin', 0.05)
        self.declare_parameter('simplify_collinear_path', True)
        self.declare_parameter('waypoint_spacing', 0.04)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._point_cloud_topic = str(self.get_parameter('point_cloud_topic').value)
        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._target_topic = str(self.get_parameter('target_topic').value)
        self._path_topic = str(self.get_parameter('path_topic').value)
        self._grid_resolution = float(self.get_parameter('grid_resolution').value)
        self._grid_x_min = float(self.get_parameter('grid_x_min').value)
        self._grid_x_max = float(self.get_parameter('grid_x_max').value)
        self._grid_y_min = float(self.get_parameter('grid_y_min').value)
        self._grid_y_max = float(self.get_parameter('grid_y_max').value)
        self._obstacle_height_threshold = float(
            self.get_parameter('obstacle_height_threshold').value
        )
        self._obstacle_inflation_cells = max(
            0, int(self.get_parameter('obstacle_inflation_cells').value)
        )
        self._path_height_margin = float(self.get_parameter('path_height_margin').value)
        self._simplify_collinear_path = bool(
            self.get_parameter('simplify_collinear_path').value
        )
        self._waypoint_spacing = max(
            self._grid_resolution,
            float(self.get_parameter('waypoint_spacing').value),
        )

        self._joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        self._origins = [
            np.array([0.0, 0.0, 0.06566], dtype=float),
            np.array([0.0, 0.0, 0.03891], dtype=float),
            np.array([0.05, 0.0, 0.15], dtype=float),
            np.array([0.15, 0.0, 0.0], dtype=float),
            np.array([0.065, 0.0, 0.0], dtype=float),
        ]
        self._axes = [
            np.array([0.0, 0.0, 1.0], dtype=float),
            np.array([0.0, 1.0, 0.0], dtype=float),
            np.array([0.0, 1.0, 0.0], dtype=float),
            np.array([0.0, 1.0, 0.0], dtype=float),
            np.array([1.0, 0.0, 0.0], dtype=float),
        ]
        self._tool_offset = np.array([0.108, 0.0, 0.0], dtype=float)

        self._current_q: Optional[np.ndarray] = None
        self._latest_cloud: Optional[np.ndarray] = None

        self.create_subscription(JointState, self._joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(
            PointCloud2,
            self._point_cloud_topic,
            self._point_cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(PointStamped, self._target_topic, self._target_cb, 10)
        self._path_pub = self.create_publisher(Path, self._path_topic, 10)

        self.get_logger().info(
            'RX-150 point-cloud path planner listening on %s and %s; publishing %s'
            % (self._point_cloud_topic, self._target_topic, self._path_topic)
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self._joint_names):
            self._current_q = np.array([positions[name] for name in self._joint_names], dtype=float)

    def _point_cloud_cb(self, msg: PointCloud2) -> None:
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
        )
        if points.size == 0:
            self._latest_cloud = None
            return
        self._latest_cloud = np.asarray(points, dtype=np.float32)

    def _target_cb(self, msg: PointStamped) -> None:
        frame = msg.header.frame_id or self._world_frame
        if frame != self._world_frame:
            self.get_logger().warning(
                "Ignoring target in frame '%s'; expected '%s'." % (frame, self._world_frame)
            )
            return

        if self._current_q is None:
            self.get_logger().warning('No current joint state yet; cannot plan path.')
            return
        if self._latest_cloud is None:
            self.get_logger().warning('No planning point cloud yet; cannot plan path.')
            return

        start_xyz = self._forward_kinematics(self._current_q)
        goal_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        occupancy = self._build_occupancy(self._latest_cloud)

        start_idx = self._world_to_grid(start_xyz[:2])
        goal_idx = self._world_to_grid(goal_xyz[:2])
        if start_idx is None or goal_idx is None:
            self.get_logger().warning('Start or goal lies outside planner grid bounds.')
            return

        occupancy.discard(start_idx)
        occupancy.discard(goal_idx)

        cell_path = self._astar(start_idx, goal_idx, occupancy)
        if cell_path is None:
            self.get_logger().warning('Planner could not find a path to the target.')
            return

        raw_waypoint_count = len(cell_path)
        if self._simplify_collinear_path:
            cell_path = self._simplify_cell_path(cell_path)

        planned_points = self._cell_path_to_xyz(cell_path, start_xyz, goal_xyz)
        self._publish_path(planned_points)
        self.get_logger().info(
            'Published planned Cartesian path with %d waypoint(s) from %d raw grid cell(s).'
            % (len(planned_points), raw_waypoint_count)
        )

    def _build_occupancy(self, points: np.ndarray) -> set[GridIndex]:
        obstacle_points = points[points[:, 2] > self._obstacle_height_threshold]
        occupied: set[GridIndex] = set()
        for point in obstacle_points:
            idx = self._world_to_grid(point[:2])
            if idx is None:
                continue
            occupied.add(idx)

        if self._obstacle_inflation_cells <= 0:
            return occupied

        inflated: set[GridIndex] = set()
        for gx, gy in occupied:
            for dx in range(-self._obstacle_inflation_cells, self._obstacle_inflation_cells + 1):
                for dy in range(-self._obstacle_inflation_cells, self._obstacle_inflation_cells + 1):
                    inflated.add((gx + dx, gy + dy))
        return {
            idx for idx in inflated
            if 0 <= idx[0] < self._grid_width() and 0 <= idx[1] < self._grid_height()
        }

    def _astar(
        self,
        start: GridIndex,
        goal: GridIndex,
        occupied: set[GridIndex],
    ) -> Optional[List[GridIndex]]:
        frontier: List[Tuple[float, GridIndex]] = []
        heapq.heappush(frontier, (0.0, start))
        came_from: Dict[GridIndex, Optional[GridIndex]] = {start: None}
        cost_so_far: Dict[GridIndex, float] = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                return self._reconstruct_path(came_from, goal)

            for neighbor, move_cost in self._neighbors(current):
                if neighbor in occupied:
                    continue
                new_cost = cost_so_far[current] + move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        return None

    def _neighbors(self, cell: GridIndex) -> List[Tuple[GridIndex, float]]:
        gx, gy = cell
        candidates = []
        for dx, dy in [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]:
            nx = gx + dx
            ny = gy + dy
            if 0 <= nx < self._grid_width() and 0 <= ny < self._grid_height():
                candidates.append(((nx, ny), float(np.hypot(dx, dy))))
        return candidates

    def _reconstruct_path(
        self,
        came_from: Dict[GridIndex, Optional[GridIndex]],
        goal: GridIndex,
    ) -> List[GridIndex]:
        path = [goal]
        current = goal
        while came_from[current] is not None:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _heuristic(a: GridIndex, b: GridIndex) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _cell_path_to_xyz(
        self,
        cell_path: List[GridIndex],
        start_xyz: np.ndarray,
        goal_xyz: np.ndarray,
    ) -> List[np.ndarray]:
        if len(cell_path) == 1:
            return [start_xyz.copy(), goal_xyz.copy()]

        travel_z = max(
            start_xyz[2],
            goal_xyz[2],
            self._obstacle_height_threshold + self._path_height_margin,
        )
        anchor_points: List[np.ndarray] = [start_xyz.copy()]
        for cell in cell_path[1:-1]:
            x_coord, y_coord = self._grid_to_world(cell)
            anchor_points.append(np.array([x_coord, y_coord, travel_z], dtype=float))
        anchor_points.append(goal_xyz.copy())
        return self._resample_xyz_path(anchor_points)

    def _simplify_cell_path(self, cell_path: List[GridIndex]) -> List[GridIndex]:
        if len(cell_path) <= 2:
            return cell_path

        simplified = [cell_path[0]]
        prev_dx = cell_path[1][0] - cell_path[0][0]
        prev_dy = cell_path[1][1] - cell_path[0][1]

        for idx in range(1, len(cell_path) - 1):
            curr = cell_path[idx]
            nxt = cell_path[idx + 1]
            dx = nxt[0] - curr[0]
            dy = nxt[1] - curr[1]
            if (dx, dy) != (prev_dx, prev_dy):
                simplified.append(curr)
            prev_dx, prev_dy = dx, dy

        simplified.append(cell_path[-1])
        return simplified

    def _resample_xyz_path(self, anchor_points: List[np.ndarray]) -> List[np.ndarray]:
        if len(anchor_points) <= 1:
            return anchor_points

        resampled = [anchor_points[0].copy()]
        for start_point, end_point in zip(anchor_points[:-1], anchor_points[1:]):
            segment = end_point - start_point
            distance = float(np.linalg.norm(segment))
            if distance <= 1e-9:
                continue

            step_count = max(1, int(np.ceil(distance / self._waypoint_spacing)))
            for step_idx in range(1, step_count + 1):
                fraction = step_idx / step_count
                resampled.append(start_point + fraction * segment)

        return resampled

    def _publish_path(self, xyz_points: List[np.ndarray]) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._world_frame

        for xyz in xyz_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self._path_pub.publish(path)

    def _world_to_grid(self, xy: np.ndarray) -> Optional[GridIndex]:
        x_coord, y_coord = float(xy[0]), float(xy[1])
        if not (self._grid_x_min <= x_coord <= self._grid_x_max):
            return None
        if not (self._grid_y_min <= y_coord <= self._grid_y_max):
            return None
        gx = int((x_coord - self._grid_x_min) / self._grid_resolution)
        gy = int((y_coord - self._grid_y_min) / self._grid_resolution)
        gx = min(gx, self._grid_width() - 1)
        gy = min(gy, self._grid_height() - 1)
        return gx, gy

    def _grid_to_world(self, cell: GridIndex) -> Tuple[float, float]:
        gx, gy = cell
        x_coord = self._grid_x_min + (gx + 0.5) * self._grid_resolution
        y_coord = self._grid_y_min + (gy + 0.5) * self._grid_resolution
        return x_coord, y_coord

    def _grid_width(self) -> int:
        return int(np.ceil((self._grid_x_max - self._grid_x_min) / self._grid_resolution))

    def _grid_height(self) -> int:
        return int(np.ceil((self._grid_y_max - self._grid_y_min) / self._grid_resolution))

    def _forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        for origin, axis, joint_angle in zip(self._origins, self._axes, q):
            transform = transform @ self._translation(origin)
            transform = transform @ self._rotation(axis, joint_angle)
        transform = transform @ self._translation(self._tool_offset)
        return transform[:3, 3].copy()

    @staticmethod
    def _translation(offset: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, 3] = offset
        return transform

    @staticmethod
    def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
        axis = axis / np.linalg.norm(axis)
        x_axis, y_axis, z_axis = axis
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        one_minus_cos = 1.0 - cos_theta
        return np.array(
            [
                [
                    cos_theta + x_axis * x_axis * one_minus_cos,
                    x_axis * y_axis * one_minus_cos - z_axis * sin_theta,
                    x_axis * z_axis * one_minus_cos + y_axis * sin_theta,
                    0.0,
                ],
                [
                    y_axis * x_axis * one_minus_cos + z_axis * sin_theta,
                    cos_theta + y_axis * y_axis * one_minus_cos,
                    y_axis * z_axis * one_minus_cos - x_axis * sin_theta,
                    0.0,
                ],
                [
                    z_axis * x_axis * one_minus_cos - y_axis * sin_theta,
                    z_axis * y_axis * one_minus_cos + x_axis * sin_theta,
                    cos_theta + z_axis * z_axis * one_minus_cos,
                    0.0,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rx150PointCloudPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
