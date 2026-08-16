# RX-150 Sim Stack — Docker Command Reference

Command reference for building, launching, and driving the RX-150 Gazebo sim stack
(`rx150_dls_sim_stack.launch.py`) through the workspace's Docker Compose setup.

## 1. Build & Setup

One-time, or after changing the Dockerfile or package dependencies.

```bash
cd ~/openRobotics/src/bcr_arm
xhost +local:docker                       # let containers open your X display
docker compose --profile sim build        # or: docker compose build rx150-sim
```

## 2. Launch the Sim Stack (Terminal 1 — keep running)

```bash
cd ~/openRobotics/src/bcr_arm
docker compose run --rm --service-ports rx150-sim
```

This runs `rosdep install` + `colcon build --packages-up-to bcr_arm_rx150`, then launches
`rx150_dls_sim_stack.launch.py`:

- Gazebo Classic with the `rx150_obstacles.world` and the depth-camera-equipped arm
- RViz, pre-configured with the `/planning/point_cloud` display, plus path-planning
  visualization: the planned path (`/planned_cartesian_path`, cyan line) and planning
  markers (`/planning/visualization`) — a red sphere at the goal you send and yellow
  spheres at each waypoint. The goal sphere appears even when planning fails (fail-safe),
  so a red sphere with no line/waypoints means "target received, no valid path"
- `rx150_dls_ik_executor` — the custom DLS IK solver (also exposes a direct
  joint-command channel on `/rx150/joint_command` used by the RRT fallback)
- `scene_point_cloud` — bridges the gripper camera cloud into `/planning/live_point_cloud`
- `rx150_point_cloud_path_planner` — obstacle-avoiding grid planner, with a
  joint-space **RRT-Connect whole-body fallback** (publishes a joint path on
  `/planned_joint_path` when the 2D grid search can't route the whole arm around
  an obstacle; see §4)
- `rx150_path_waypoint_executor` — drives a planned Cartesian path through the IK executor
- `rx150_joint_waypoint_executor` — steps a planned **joint** path (from the RRT
  fallback) through the IK executor's direct joint-command channel

All commands below run in a **new terminal**, exec'd into that same running container:

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && <command>"
```

The `<command>` shown in each section below goes in place of `<command>` in that wrapper. See #3 and #4 as examples.

## 3. Scene Sweep (builds the obstacle map)

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && 
ros2 run data_collector scene_sweep_mapper"
```

Tucks the arm into a fixed scan posture, sweeps 8 waist angles for full 360° coverage, and
publishes the merged map to `/planning/point_cloud` (latched, republished every 2s so late
subscribers still get it).

## 4. Send a Cartesian Target (through the planner)

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && 
ros2 topic pub --once /cartesian_target geometry_msgs/msg/PointStamped \
'{header: {frame_id: rx150/base_link}, point: {x: 0.22, y: 0.00, z: 0.16}}'"
```



`frame_id` must be `rx150/base_link` — `rx150_point_cloud_path_planner` drops targets in any
other frame. This routes: planner plans around `/planning/point_cloud` →
`/planned_cartesian_path` → waypoint executor drives it through the IK executor.

**Whole-body checking + RRT-Connect fallback:** the planner validates the *whole arm* — not
just the end-effector — against the obstacle map, solving IK at each waypoint and checking
every link with a capsule collision model. If the 2D grid search cannot find a path that
keeps the entire arm clear (after its replan attempts), it hands off to a **joint-space
RRT-Connect whole-body fallback** (`use_rrt_fallback`, default on). RRT-Connect plans in the
5 joint angles — so it can discover "retract, rotate the waist, re-approach from a clear
side" motions the tool-tip grid search can't represent — validating every configuration
against the *same* point cloud + capsule model. On success it publishes a joint path on
`/planned_joint_path`, which `rx150_joint_waypoint_executor` steps to the arm via the IK
executor's direct `/rx150/joint_command` channel (no Cartesian round-trip, so the checked
posture is what actually executes). In RViz a successful fallback shows as a **green tool-tip
trace** (distinct from the cyan A* path).

**Fail-safe behavior (expected, not a bug):** if *both* the grid search and the RRT fallback
fail, the planner deliberately publishes **nothing** rather than a colliding path — so "I
sent a target and the arm didn't move" against a cluttered map is the safety check working,
and it now means the target is genuinely unreachable for the whole arm (a real search proved
it, not an early give-up). Look for logs like `RRT-Connect fallback found no path (... result=
max_iters ...)` or, when the fallback is disabled, `Could not find a whole-body collision-
free path within N attempt(s); publishing nothing`. Tunable via the `body_collision_check`,
`max_replan_attempts`, `collision_extra_margin`, `use_rrt_fallback`, `rrt_time_budget_sec`,
`rrt_max_iters`, `rrt_step`, and `rrt_check_step` planner params.

**Grasp clearance (reaching *to* an object):** the point cloud doesn't distinguish an
obstacle from the object you're grasping — the cup sits in the cloud too, so without help the
gripper would "collide" with its own target and every grasp point would be reported
unreachable. The planner therefore treats cloud points within `grasp_clearance_radius` (m) of
the goal as the target object and excludes them from *all* collision checks for that target;
points elsewhere stay hard obstacles. Default `0.08`; look for a log like `Grasp clearance:
treating K cloud point(s) within 0.080 m of the goal as the target object`. Set it to `0.0`
to make the whole cloud an obstacle (strict collision, no reaching into anything).

## 5. Other Nodes You Can Run Manually

### Reset / pose the arm

```bash
ros2 run bcr_arm_rx150 rx150_named_pose --pose neutral_carry
# other poses: neutral_carry_yaw_left, neutral_carry_yaw_right
```

### Smoke test (small automated lift-and-return)

```bash
ros2 run bcr_arm_rx150 rx150_smoke_test
```

### Staged Cartesian target suites

These default to `world_frame: base_link`, matching the standalone hardware/DLS stack, not
this sim stack's planner (`rx150/base_link`) — override if pointing at the sim stack.

```bash
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite smoke
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite lateral
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite pose
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite full --auto --pause-sec 5.0 \
  --ros-args -p world_frame:=rx150/base_link
```

The `collision` suite is the whole-body regression for the original bug (the arm's elbow
hitting `obstacle_box_front_left` while the end-effector path looked clear). Run it against
the sim stack's planner — the box must be in `/planning/point_cloud` first (run the sweep),
and the frame must be overridden to `rx150/base_link`:

```bash
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite collision \
  --ros-args -p world_frame:=rx150/base_link
```

C1 is the exact reported-bug target (the arm must reroute or refuse, not collide), C2 aims
straight into the box (expect reroute or the fail-safe "publishing nothing"), and C3 is an
unobstructed control that must still plan and execute.

### One-off target straight through the IK solver (bypasses the planner)

```bash
ros2 run bcr_arm_rx150 rx150_dls_ik_executor --x 0.20 --y 0.00 --z 0.16 --frame rx150/base_link
# optional orientation: --qx --qy --qz --qw
```

### Point-cloud utilities (`data_collector`, not wired into this launch by default)

```bash
ros2 run data_collector point_cloud    # standalone filtered-cloud publisher + TF broadcaster
ros2 run data_collector collector      # logs /joint_states to a pickle file for offline analysis
```

## 6. Inspect Live State

```bash
ros2 topic echo /rx150/joint_states
ros2 topic echo /planned_cartesian_path
ros2 topic info /planning/point_cloud
```
