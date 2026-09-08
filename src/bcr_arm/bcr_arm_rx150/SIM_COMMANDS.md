# RX-150 Sim Stack — Docker Command Reference

Command reference for building, launching, and driving the RX-150 Gazebo sim stack
(`rx150_dls_sim_stack.launch.py`) through the workspace's Docker Compose setup.

## 1. Build & Setup

One-time, or after changing the Dockerfile or package dependencies.

```bash
# run from src/bcr_arm/ in your checkout
xhost +local:docker                       # let containers open your X display
docker compose --profile sim build        # or: docker compose build rx150-sim
```

## 2. Launch the Sim Stack (Terminal 1 — keep running)

```bash
# run from src/bcr_arm/ in your checkout
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
  (the plain sim stack has no object detection; that comes with the pick-and-place
  mission launch, §7)
- `rx150_point_cloud_path_planner` — obstacle-avoiding grid planner, with a
  joint-space **RRT-Connect whole-body fallback** (publishes a joint path on
  `/planned_joint_path` when the 2D grid search can't route the whole arm around
  an obstacle; see §4)
- `rx150_path_waypoint_executor` — drives a planned Cartesian path through the IK executor
- `rx150_joint_waypoint_executor` — steps a planned **joint** path (from the RRT
  fallback) through the IK executor's direct joint-command channel

### Sim time

Every launch here defaults to `use_sim_time:=true`. **Nothing to set.**

It matters because Gazebo stamps camera images and TF with its own clock. A node left on
the wall clock cannot match those stamps to anything, so its data is dropped — silently,
with the node and camera both looking healthy.

The one way to get it wrong is launching `perception.launch.py` **by hand** against a
running sim: its default is `false`, because its default target is hardware. Pass it
explicitly:

```bash
ros2 launch arm_perception perception.launch.py use_sim_time:=true enable_camera:=false \
  color_topic:=/gripper_camera/image_raw \
  depth_topic:=/gripper_camera/depth/image_raw \
  camera_info_topic:=/gripper_camera/camera_info
```

`localization_3d_node` checks this a few seconds after startup:

```
Clock check OK: use_sim_time=True, /clock present.        <- healthy
[ERROR] A simulator is publishing /clock but this node is on the WALL clock.
        ... EVERY DETECTION WILL BE DROPPED -- silently.  <- relaunch with use_sim_time:=true
```

On hardware it is `false` everywhere and the check stays quiet — there is no `/clock`.

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

Tucks the arm into a fixed scan posture, sweeps 8 waist angles taking 2 looks at each
(the second with the wrist tilted 0.15 rad further down, which is what puts objects
lying on the floor inside 0.335 m into frame), and
publishes the merged map to `/planning/point_cloud` (latched, republished every 2s so late
subscribers still get it).

This standalone form is for the plain sim stack (§2). By default
(`wait_for_trigger:=false`) it sweeps once on startup, as above; with
`wait_for_trigger:=true` it idles and sweeps on each `/sweep/start` instead.

> **Do not run this while the pick-and-place mission is up.** That launch already runs
> this node in triggered mode. A second copy would mean two publishers on
> `/planning/point_cloud`, both republishing every 2 s and overwriting each other —
> RViz flickers between the two maps, and the planner's obstacle map alternates with
> them, making whether a path is found depend on which cloud arrived last. The mission
> sweeps for real on its own; see [PICK_PLACE_MISSION.md](PICK_PLACE_MISSION.md) §6.

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

### Open / close the gripper

The sim stack launches `rx150_gripper_controller` automatically. In sim the gripper is the
ros2_control `gripper_controller` driving the two prismatic finger joints (`left_finger` =
`+pos`, `right_finger` = `-pos`). Command it three ways on `/rx150/gripper_command` — a named
state or a value. The stack runs in **normalized** units, so the value is `0.0` (closed) ..
`1.0` (open) and **the exact same command works on hardware**:

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: open}'"

docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && 
ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: close}'"

docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && 
ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: "0.7"}'   # 70% open"
```

Also available: a one-shot CLI and a `Float64` channel.

```bash
ros2 run bcr_arm_rx150 rx150_gripper_controller --state open      # or: --state close
ros2 run bcr_arm_rx150 rx150_gripper_controller --position 0.7 --ros-args -p command_units:=normalized
ros2 topic pub --once /rx150/gripper_position std_msgs/msg/Float64 '{data: 0.7}'
```

> The launch ships `command_units:=normalized` (0–1) so sim and hardware take the **same**
> numbers. The node's own default is `native` (raw finger metres `0.015`–`0.037` in sim); the
> standalone CLI above sets `normalized` explicitly to match the running stack. Named states
> (`open`/`close`) always match regardless of units.

The same node runs on hardware in `single` mode (one `gripper` servo) — see
[HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md) §10.

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

---

## 7. Pick-and-Place Mission

Full reference: **[PICK_PLACE_MISSION.md](PICK_PLACE_MISSION.md)**. Short version:

```bash
# Terminal 1 -- the whole mission stack
# run from src/bcr_arm/ in your checkout
xhost +local:docker

docker compose run --rm --name rx150 --service-ports rx150-sim bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place_sim.launch.py carry_level:=true"

# Terminal 2 -- keyboard (s = start, x = stop, r = restart, q = quit)
docker exec -it rx150 bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

Press `s` once and wait — phase 1 is a ~2 minute sweep. The most useful launch flags are
`grasp_value`, `grasp_z_offset`, `autostart`, `use_vision_stub` and `phase_delay_sec`;
see PICK_PLACE_MISSION.md §1 for the table.

**The sweep is what finds the cup**, not a hardcoded vector and not a fixed observation
pose. The detector runs throughout the sweep, every sighting is kept, and the sweep turns
the camera across a 90° front sector (±45°, 3 stations) by default — so the cup can be
moved anywhere in front of the arm between runs and still be found. Widen to the full
circle with `scan_waist_angles:='[-3.10,-2.356,-1.571,-0.785,0.0,0.785,1.571,2.356]'`
(8 stations); anything outside the swept sector is neither searched for the cup nor
mapped as an obstacle. Measured: cup at `(0.300, 0.000, 0.090)` reported as
`(0.304, 0.001, 0.090)`, answered in ~1 ms from cache.

The scene has a small cup on a book-stack riser at `[0.30, 0.0]`, top at z = 0.060. The
riser matters: a low object on the floor is much harder for the scan to see, and the
elevation is part of why sweep-based detection works.

> **Two sim-only accommodations.** `yolo_confidence` is `0.15` and `cup_classes` is
> `[cup, frisbee, bowl, toilet]`.
>
> COCO-trained YOLOv8n *localises* the sim cup perfectly but *labels* a flat-shaded
> Gazebo primitive by silhouette, so it often scores `frisbee` rather than `cup`. Wrong
> label, right pixels — the 3D point is still correct.
>
> **`cup_classes` is a priority order, not a set.** First class with a fresh detection
> wins; confidence only breaks ties *within* a class. Ranking by confidence across the
> whole list is dangerous here — the blue obstacle cylinder scores `vase` 0.49 and would
> win. **Only add classes the cup itself produces**: `umbrella` (0.43) belongs to the
> obstacle boxes, `vase` to the cylinder.
>
> Narrow back to `[cup]` / `0.5` once a task-trained model replaces `yolov8n.pt`, and on
> hardware, where a real cup photographs like a cup.

**The goal is still a placeholder.** Nothing in the world is a detectable drop-off
target, so a `"goal"` request always answers `goal_fallback_xyz` and logs a warning.

Phase 1 is a real sweep — 8 waist angles, 2 looks each, ~123 s — and the mission blocks
on `sweep:complete` before planning. Do not also run the standalone sweep from §3.

If detection misbehaves and you just want the mission to run, use `use_vision_stub:=true`.
