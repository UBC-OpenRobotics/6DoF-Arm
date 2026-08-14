#!/usr/bin/env python3

from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from bcr_arm_common import rx150_kinematics
from bcr_arm_rx150 import rx150_rrt_fallback
from bcr_arm_rx150.rx150_dls_solver import DlsSolver, DlsSolverConfig
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter('marker_topic', '/planning/visualization')
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
        self.declare_parameter('body_collision_check', True)
        self.declare_parameter('max_replan_attempts', 6)
        self.declare_parameter('collision_extra_margin', 0.0)
        self.declare_parameter('reachable_position_tolerance', 0.015)
        self.declare_parameter('grasp_clearance_radius', 0.08)
        self.declare_parameter('use_rrt_fallback', True)
        self.declare_parameter('joint_path_topic', '/planned_joint_path')
        self.declare_parameter('rrt_step', 0.10)
        self.declare_parameter('rrt_check_step', 0.05)
        self.declare_parameter('rrt_max_iters', 3000)
        self.declare_parameter('rrt_time_budget_sec', 1.5)
        self.declare_parameter('rrt_smoothing_iters', 100)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._point_cloud_topic = str(self.get_parameter('point_cloud_topic').value)
        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self._target_topic = str(self.get_parameter('target_topic').value)
        self._path_topic = str(self.get_parameter('path_topic').value)
        self._marker_topic = str(self.get_parameter('marker_topic').value)
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
        self._body_collision_check = bool(
            self.get_parameter('body_collision_check').value
        )
        self._max_replan_attempts = max(
            1, int(self.get_parameter('max_replan_attempts').value)
        )
        self._collision_extra_margin = float(
            self.get_parameter('collision_extra_margin').value
        )
        self._reachable_position_tolerance = max(
            0.0, float(self.get_parameter('reachable_position_tolerance').value)
        )
        self._grasp_clearance_radius = max(
            0.0, float(self.get_parameter('grasp_clearance_radius').value)
        )
        self._use_rrt_fallback = bool(self.get_parameter('use_rrt_fallback').value)
        self._joint_path_topic = str(self.get_parameter('joint_path_topic').value)
        self._rrt_step = float(self.get_parameter('rrt_step').value)
        self._rrt_check_step = float(self.get_parameter('rrt_check_step').value)
        self._rrt_max_iters = int(self.get_parameter('rrt_max_iters').value)
        self._rrt_time_budget_sec = float(self.get_parameter('rrt_time_budget_sec').value)
        self._rrt_smoothing_iters = int(self.get_parameter('rrt_smoothing_iters').value)

        self._joint_names = list(rx150_kinematics.JOINT_NAMES)
        self._solver = DlsSolver(DlsSolverConfig())

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
        self._joint_path_pub = self.create_publisher(
            JointTrajectory, self._joint_path_topic, 10
        )

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(
            MarkerArray, self._marker_topic, marker_qos
        )
        self._goal_marker: Optional[Marker] = None
        self._waypoint_marker: Optional[Marker] = None
        self._rrt_marker: Optional[Marker] = None

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

        self._publish_goal_marker((msg.point.x, msg.point.y, msg.point.z))

        if self._current_q is None:
            self.get_logger().warning('No current joint state yet; cannot plan path.')
            return
        if self._latest_cloud is None:
            self.get_logger().warning('No planning point cloud yet; cannot plan path.')
            return

        start_xyz = rx150_kinematics.forward_kinematics(self._current_q)[0]
        goal_xyz = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

        planning_cloud = self._filter_cloud_near_goal(self._latest_cloud, goal_xyz)
        base_occupancy = self._build_occupancy(planning_cloud)
        obstacle_points = self._obstacle_points_3d(planning_cloud)

        start_idx = self._world_to_grid(start_xyz[:2])
        goal_idx = self._world_to_grid(goal_xyz[:2])
        if start_idx is None or goal_idx is None:
            self.get_logger().warning('Start or goal lies outside planner grid bounds.')
            return


        body_blocked: set[GridIndex] = set()

        for attempt in range(self._max_replan_attempts):
            occupancy = set(base_occupancy) | body_blocked
            occupancy.discard(start_idx)
            occupancy.discard(goal_idx)

            cell_path = self._astar(start_idx, goal_idx, occupancy)
            if cell_path is None:
                self.get_logger().info(
                    'A* found no 2D route to the target after %d replan '
                    'attempt(s); handing off to the whole-body fallback.' % attempt
                )
                break

            raw_waypoint_count = len(cell_path)
            if self._simplify_collinear_path:
                cell_path = self._simplify_cell_path(cell_path)

            planned_points = self._cell_path_to_xyz(cell_path, start_xyz, goal_xyz)

            blocking_cell = self._first_body_collision_cell(
                planned_points, obstacle_points
            )
            if blocking_cell is None:
                self._publish_path(planned_points)
                self.get_logger().info(
                    'Published body-checked Cartesian path with %d waypoint(s) '
                    'from %d raw grid cell(s) after %d replan attempt(s).'
                    % (len(planned_points), raw_waypoint_count, attempt)
                )
                return


            self._add_blocked_cell(body_blocked, blocking_cell)


        if self._use_rrt_fallback and self._try_rrt_fallback(
            start_xyz, goal_xyz, obstacle_points
        ):
            return

        self.get_logger().warning(
            'Could not find a whole-body collision-free path within %d attempt(s); '
            'publishing nothing.' % self._max_replan_attempts
        )

    def _try_rrt_fallback(
        self,
        start_xyz: np.ndarray,
        goal_xyz: np.ndarray,
        obstacle_points: np.ndarray,
    ) -> bool:
        """Joint-space whole-body fallback. Returns True if a path was published."""
        self.get_logger().info(
            'A* exhausted; invoking joint-space RRT-Connect whole-body fallback.'
        )
        result = rx150_rrt_fallback.plan_joint_path(
            self._current_q,
            goal_xyz,
            obstacle_points,
            self._solver,
            target_rotation=None,
            extra_margin=self._collision_extra_margin,
            step=self._rrt_step,
            check_step=self._rrt_check_step,
            max_iters=self._rrt_max_iters,
            time_budget_sec=self._rrt_time_budget_sec,
            smoothing_iters=self._rrt_smoothing_iters,
        )
        stats = result.stats
        if not result.succeeded:
            self.get_logger().warning(
                'RRT-Connect fallback found no path (goals=%d, result=%s, '
                'iterations=%s, nodes=%s, elapsed=%.3fs). Target is likely '
                'unreachable for the whole arm.'
                % (
                    result.goal_count,
                    stats.get('result'),
                    stats.get('iterations'),
                    stats.get('nodes'),
                    float(stats.get('elapsed_sec', 0.0)),
                )
            )
            return False

        self._publish_joint_path(result.path)
        self._publish_rrt_path_marker(result.path)
        self.get_logger().info(
            'RRT-Connect fallback published a whole-body joint path with %d '
            'waypoint(s) (goals=%d, iterations=%s, nodes=%s, elapsed=%.3fs).'
            % (
                len(result.path),
                result.goal_count,
                stats.get('iterations'),
                stats.get('nodes'),
                float(stats.get('elapsed_sec', 0.0)),
            )
        )
        return True

    def _publish_joint_path(self, configs: List[np.ndarray]) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._world_frame
        msg.joint_names = list(self._joint_names)
        for config in configs:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in config]
            msg.points.append(point)
        self._joint_path_pub.publish(msg)

    def _filter_cloud_near_goal(self, cloud: np.ndarray, goal_xyz: np.ndarray) -> np.ndarray:
        """Drop cloud points within grasp_clearance_radius of the goal.

        Those points are the object being reached for (the cup), not an obstacle;
        keeping them would make the gripper collide with its own target. Points
        outside the sphere are untouched and remain obstacles.
        """
        if self._grasp_clearance_radius <= 0.0 or cloud is None or cloud.shape[0] == 0:
            return cloud
        deltas = cloud[:, :3].astype(float) - goal_xyz
        distances_sq = np.einsum('ij,ij->i', deltas, deltas)
        keep = distances_sq > (self._grasp_clearance_radius ** 2)
        removed = int(cloud.shape[0] - int(np.count_nonzero(keep)))
        if removed > 0:
            self.get_logger().info(
                'Grasp clearance: treating %d cloud point(s) within %.3f m of the '
                'goal as the target object (excluded from collision checks).'
                % (removed, self._grasp_clearance_radius)
            )
        return cloud[keep]

    def _obstacle_points_3d(self, cloud: np.ndarray) -> np.ndarray:
        """Above-threshold points, voxel-downsampled, for whole-arm collision."""
        obstacle_points = cloud[cloud[:, 2] > self._obstacle_height_threshold]
        if obstacle_points.shape[0] == 0:
            return np.empty((0, 3), dtype=float)
        # Downsample to one representative point per voxel to bound cost on dense
        # clouds while preserving obstacle geometry in all three axes.
        voxel = max(self._grid_resolution, 1e-6)
        keys = np.floor(obstacle_points[:, :3] / voxel).astype(np.int64)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        return obstacle_points[unique_idx, :3].astype(float)

    def _first_body_collision_cell(
        self, planned_points: List[np.ndarray], obstacle_points: np.ndarray
    ) -> Optional[GridIndex]:
        """Return the grid cell of the first waypoint that is unreachable or whose
        solved arm configuration collides with an obstacle, or None if the whole
        path is reachable and body-clear. Chains the IK seed along the path like
        the executor does, so A* re-routes around bad cells."""
        if not self._body_collision_check:
            return None

        has_obstacles = obstacle_points is not None and obstacle_points.shape[0] > 0
        seed = self._current_q.copy()
        for waypoint in planned_points:
            solution = self._solver.solve(seed, waypoint, target_rotation=None)
            if solution is None:
                # No IK solution at all: treat as blocked so A* routes around it.
                return self._world_to_grid(waypoint[:2])
            q_solution, position_error, _, _ = solution

            if position_error > self._reachable_position_tolerance:
                self.get_logger().info(
                    'Waypoint [%.3f, %.3f, %.3f] is beyond reach (best IK error '
                    '%.3f m > %.3f m); replanning around it.'
                    % (
                        waypoint[0], waypoint[1], waypoint[2],
                        position_error, self._reachable_position_tolerance,
                    )
                )
                return self._world_to_grid(waypoint[:2])
            seed = q_solution
            if has_obstacles:
                joints = rx150_kinematics.link_positions(q_solution)
                collided, segment = rx150_kinematics.check_arm_collision(
                    joints, obstacle_points, extra_margin=self._collision_extra_margin
                )
                if collided:
                    self.get_logger().info(
                        'Body collision on link segment %d at waypoint '
                        '[%.3f, %.3f, %.3f]; replanning around it.'
                        % (segment, waypoint[0], waypoint[1], waypoint[2])
                    )
                    return self._world_to_grid(waypoint[:2])
        return None

    def _add_blocked_cell(self, blocked: set[GridIndex], cell: Optional[GridIndex]) -> None:
        if cell is None:
            return
        radius = max(1, self._obstacle_inflation_cells)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = cell[0] + dx, cell[1] + dy
                if 0 <= nx < self._grid_width() and 0 <= ny < self._grid_height():
                    blocked.add((nx, ny))

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
        self._publish_waypoint_markers(xyz_points)

    # ------------------------------------------------------------------
    # Visualization markers (goal + waypoints) on self._marker_topic.
    # ------------------------------------------------------------------
    def _new_marker(self, namespace: str, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _refresh_markers(self) -> None:
        marker_array = MarkerArray()
        if self._goal_marker is not None:
            marker_array.markers.append(self._goal_marker)
        if self._waypoint_marker is not None:
            marker_array.markers.append(self._waypoint_marker)
        if self._rrt_marker is not None:
            marker_array.markers.append(self._rrt_marker)
        if marker_array.markers:
            self._marker_pub.publish(marker_array)

    def _publish_goal_marker(self, goal_xyz) -> None:
        marker = self._new_marker('goal', Marker.SPHERE)
        marker.pose.position.x = float(goal_xyz[0])
        marker.pose.position.y = float(goal_xyz[1])
        marker.pose.position.z = float(goal_xyz[2])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.03
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.1, 0.1, 1.0
        self._goal_marker = marker
        # A fresh target invalidates the previous plan's waypoints and any RRT
        # fallback path; tell RViz to drop them until a new path is published.
        clear = self._new_marker('waypoints', Marker.SPHERE_LIST)
        clear.action = Marker.DELETE
        self._waypoint_marker = clear
        clear_rrt = self._new_marker('rrt_path', Marker.LINE_STRIP)
        clear_rrt.action = Marker.DELETE
        self._rrt_marker = clear_rrt
        self._refresh_markers()

    def _publish_waypoint_markers(self, xyz_points: List[np.ndarray]) -> None:
        marker = self._new_marker('waypoints', Marker.SPHERE_LIST)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.015
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.9, 0.1, 1.0
        for xyz in xyz_points:
            marker.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
        self._waypoint_marker = marker
        self._refresh_markers()

    def _publish_rrt_path_marker(self, configs: List[np.ndarray]) -> None:
        """Draw the RRT fallback solution as the tool-tip trace (distinct color).

        The RRT path is joint-space; showing its forward-kinematics tool tip as a
        green LINE_STRIP makes a successful whole-body fallback visible and
        distinguishable from the cyan A* Cartesian path.
        """
        marker = self._new_marker('rrt_path', Marker.LINE_STRIP)
        marker.scale.x = 0.006  # line width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 1.0, 0.2, 1.0
        for config in configs:
            tip = rx150_kinematics.forward_kinematics(np.asarray(config, dtype=float))[0]
            marker.points.append(Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])))
        self._rrt_marker = marker
        self._refresh_markers()

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
