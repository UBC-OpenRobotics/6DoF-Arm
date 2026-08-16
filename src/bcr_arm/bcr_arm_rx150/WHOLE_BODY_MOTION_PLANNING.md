# RX-150 Whole-Body Motion Planning — Implementation Overview

How the RX-150 planning stack turns a Cartesian target into a collision-aware, whole-body-safe
motion. This is the system overview — both planners, the shared collision model, and
execution. For a deep dive on the RRT-Connect algorithm specifically, see
[RRT_CONNECT_README.md](RRT_CONNECT_README.md). For running it, see
[SIM_COMMANDS.md](SIM_COMMANDS.md) / [HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md).

## What it does

Given a Cartesian point on `/cartesian_target` (frame `rx150/base_link`) and a point-cloud
obstacle map on `/planning/point_cloud`, the planner produces a path that keeps the arm's
**whole body** — not just the end-effector — clear of obstacles, and drives the arm along it.
If no safe motion exists, it publishes nothing (fail-safe).

## Pipeline

Two planners in series: a fast 2D grid A\* for the common case, and a joint-space RRT-Connect
fallback for motions that need the whole body to route around an obstacle.

```
target arrives (_target_cb)
   │
   ▼
2D grid A* + whole-body capsule check
   │
   ├─ path found ─► nav_msgs/Path ─► Cartesian waypoint executor ─► IK executor
   │
   └─ no whole-body-clear path
          │
          ▼
     joint-space RRT-Connect
          │
          ├─ path found ─► joint waypoints ─► IK executor's direct joint channel
          │
          └─ no path ─► publish nothing (fail-safe); goal marker still shows
```

The two output channels are mutually exclusive per target, so there is no command contention.

## The collision model (shared by both planners)

Both planners reason about obstacles the same way, so they agree on what "collision" means:

- **Obstacle points** — `/planning/point_cloud`, filtered above a height threshold,
  voxel-downsampled, in `rx150/base_link` (`_obstacle_points_3d`).
- **Whole-arm capsules** — the arm is modeled as 5 capsules along the link chain
  (`rx150_kinematics.link_positions` + `check_arm_collision`). A configuration collides if any
  obstacle point falls within a capsule's radius plus a safety margin.
- **Grasp clearance** — cloud points within `grasp_clearance_radius` (default 0.08 m) of the
  goal are treated as the object being reached for, and excluded from collision checks for that
  target, so the gripper is allowed to approach and reach it. Points elsewhere stay obstacles.
- **Reachability** — a waypoint whose best DLS-IK solution lands farther than
  `reachable_position_tolerance` (default 0.015 m) from the requested point is treated as
  unreachable and routed around, so the planner only produces motions the arm can actually
  follow.

## Primary planner — 2D grid A\* with whole-body check

For the common case, the end-effector travels across a 2D grid at a transit height:

1. Build a 2D occupancy grid from the obstacle points (inflated by a configurable margin).
2. Run A\* from the start cell to the goal cell.
3. For each waypoint on the resulting path, solve DLS IK to get the arm configuration, then run
   the whole-arm capsule + reachability check.
4. If a waypoint's configuration collides or is unreachable, block that grid cell and re-search
   (up to `max_replan_attempts`), so the arm routes its whole body around the obstacle.
5. On a whole-body-clear path, publish a `nav_msgs/Path` on `/planned_cartesian_path`.

This is fast and deterministic whenever the tool tip can reach the goal traveling at a fixed
height.

## Fallback planner — joint-space RRT-Connect

When A\* can't find a whole-body-clear path, the planner searches over the **5 joint angles**.
A joint configuration fully determines where every link is, so the collision check is exact for
the whole arm, and the search can find motions like "retract, rotate the waist to a clear
sector, re-approach."

- **Goal configurations** come from the shared DLS solver, multi-seeded (current, neutral,
  waist-/elbow-flipped, azimuth-aligned) to cover several IK branches; colliding goal configs
  are dropped.
- **Two trees** (rooted at the start config and the goal configs) grow toward random samples
  and connect when they meet.
- **Every edge** is validated at `rrt_check_step` resolution against the shared collision model
  (so a step can't sweep the arm through an obstacle), and the found path is shortcut-smoothed.
- On success, the joint path is published on `/planned_joint_path`.

The full algorithm walkthrough is in [RRT_CONNECT_README.md](RRT_CONNECT_README.md).

## Execution

The path is executed as the kind of path it is — never re-solved into a different posture:

- **Cartesian path (A\*)** → `rx150_path_waypoint_executor` sends each waypoint to the DLS IK
  executor, advancing on an end-effector distance tolerance.
- **Joint path (RRT)** → `rx150_joint_waypoint_executor` steps each configuration to the IK
  executor's **direct joint-command input** (`/rx150/joint_command`), which commands it without
  re-running IK, advancing on a joint-space tolerance from `/rx150/joint_states`.

Either way the arm command goes out through the IK executor's `command_mode` abstraction —
`group` / `JointGroupCommand` on the physical arm, `trajectory` / `JointTrajectory` in sim — so
the same planner output drives both platforms.

## Fail-safe

If neither planner finds a whole-body-clear, reachable path, the planner publishes nothing
rather than a colliding or unreachable path. The goal marker still appears in RViz, so a goal
with no path/trace means "target received, no valid whole-body motion." A successful RRT
fallback additionally draws a green tool-tip trace on `/planning/visualization`.

## Parameters (all tunable)

| Param | Default | Meaning |
|-------|---------|---------|
| `body_collision_check` | true | run the whole-arm capsule check |
| `max_replan_attempts` | 6 | A\* re-search attempts around blocked cells |
| `collision_extra_margin` | 0.0 m | extra clearance on every capsule |
| `grasp_clearance_radius` | 0.08 m | near-goal points treated as the target object |
| `reachable_position_tolerance` | 0.015 m | reject waypoints IK can't reach |
| `use_rrt_fallback` | true | enable the joint-space fallback |
| `rrt_step` | 0.10 rad | RRT tree growth increment |
| `rrt_check_step` | 0.05 rad | RRT edge collision-check resolution |
| `rrt_max_iters` | 3000 | RRT iteration cap |
| `rrt_time_budget_sec` | 1.5 | RRT wall-clock cap |
| `rrt_smoothing_iters` | 100 | RRT shortcut attempts |

## Portability (sim ↔ physical)

The planning side is platform-independent: FK, the capsule collision model, DLS IK, and joint
limits are pure math; joint state (`/rx150/joint_states`) and the obstacle cloud
(`/planning/point_cloud`) arrive on the same topics in sim and on hardware. The only sim/real
difference is the arm command channel, which the IK executor abstracts — so the planner, the
RRT fallback, and the executors run on the physical RX-150 unchanged. The one hardware
requirement is a real depth camera feeding the cloud.
