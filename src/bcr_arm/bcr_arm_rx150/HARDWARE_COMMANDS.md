# RX-150 Hardware Stack — Docker Command Reference

Command reference for building, launching, and driving the **physical** RX-150 DLS stack
(`rx150_dls_stack.launch.py`) through the workspace's Docker Compose setup. This is the
hardware counterpart to [SIM_COMMANDS.md](SIM_COMMANDS.md).

> **Two modes.** The physical launch has a `minimal` flag:
> - **`minimal:=false` (default)** — the **full** stack: real arm driver (`xs_sdk`) + DLS IK
>   executor + the obstacle **planner**, the **RRT-Connect** fallback, both waypoint
>   executors, and the cloud **relay** — the same planning stack as sim, on the real arm.
> - **`minimal:=true`** — **just** the arm driver + DLS IK executor. Drive it directly with
>   Cartesian targets, no camera/planner. Use this for a bare bring-up.
>
> **The camera is now wired — in the mission launch.** `rx150_pick_place.launch.py` (§11)
> starts the RealSense driver, the sweep, and YOLO detection, so the full mission runs on
> real data. What is *not* wired is `rx150_dls_stack.launch.py` on its own: launched
> directly it still has no camera driver, so its relay has no input and the planner
> **idles with no cloud**. To drive the bare stack today, use `minimal:=true` and the IK
> executor directly (§5), or feed a canned cloud (§9). Extrinsic calibration remains open —
> see [TASK_camera_extrinsics_urdf.md](TASK_camera_extrinsics_urdf.md); the map is only as
> good as the camera→`base_link` numbers in the URDF. Everything uses the sim frame —
> **`rx150/base_link`** — which is the real TF base frame the Interbotix description
> publishes, so there's no sim/real divergence.

## 0. Before you power on (safety)

- Clear the arm's workspace — it **will** move when you launch and send targets.
- Keep the power switch / e-stop within reach. Kill power if anything looks wrong.
- Make sure the U2D2 serial adapter is plugged in and the arm is powered before launching.
  The `rx150-hardware` container is `privileged` and mounts `/dev` + `/run/udev`, so it can
  reach the U2D2 without extra host config.

## 1. Build & Setup

One-time, or after changing the Dockerfile or package dependencies.

```bash
# run from src/bcr_arm/ in your checkout
xhost +local:docker                            # let containers open your X display (RViz)
docker compose --profile hardware build        # or: docker compose build rx150-hardware
```

## 2. Launch the Hardware Stack (Terminal 1 — keep running)

**Full stack (default):**
```bash
# run from src/bcr_arm/ in your checkout
docker compose --profile hardware run --rm --service-ports rx150-hardware
```

**Minimal — arm + IK only (recommended until the RealSense is wired):**
```bash
# run from src/bcr_arm/ in your checkout
docker compose run --rm --service-ports rx150-hardware bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py minimal:=true"
```

> **Why the `bash -lc "source ... && ..."` wrapper?** Passing a command to
> `docker compose run` *replaces* the service's `command:`, and that `command:` is
> what sourced the workspace. The image entrypoint only sources
> `/opt/ros/humble`, so a bare `... rx150-hardware ros2 launch ...` fails with
> `package 'bcr_arm_rx150' not found`. Source it yourself whenever you override
> the command. The no-argument form above needs no wrapper — it keeps the
> service's own `command:`, which additionally runs `setup_workspace.sh`
> (rosdep + `colcon build`). The overrides skip that, so build first if sources
> changed.

Either way you get `rx150_dls_stack.launch.py`. Always present:

- `rx150_control.launch.py` — the Interbotix RX-150 control launch (the **real** low-level
  driver, `xs_sdk`), which talks to the motors over the U2D2 and publishes `/rx150/joint_states`
- RViz (`use_rviz:=true` by default) on `rviz/rx150_dls_stack.rviz` — robot model,
  TF, the swept obstacle map (`/planning/point_cloud`), the planned path and the
  planning markers, plus a `LiveCameraCloud` display for the raw RealSense cloud
  that starts unticked. Override with `rvizconfig:=/path/to.rviz`.
- `rx150_dls_ik_executor` — the custom DLS IK solver in **`group`** command mode
  (`interbotix_xs_msgs/JointGroupCommand` on `/rx150/commands/joint_group`), with
  `world_frame: rx150/base_link` and **conservative motion params** (slow velocity, small
  steps, heavier damping, 4 s goal time) so the real arm moves gently. Raise
  `max_joint_velocity` / `max_joint_step` in
  [rx150_dls_stack.launch.py](launch/rx150_dls_stack.launch.py) once you trust the setup.

**Full mode also adds** (all in `rx150/base_link`, mirroring sim):

- `rx150_point_cloud_path_planner` — A* + RRT-Connect whole-body fallback (listens on
  `/cartesian_target`)
- `rx150_path_waypoint_executor` + `rx150_joint_waypoint_executor` — drive the planned
  Cartesian / joint paths through the IK executor
- `scene_point_cloud` — the cloud relay (RealSense frame → `base_link`). **Needs the RealSense
  driver** publishing `camera_points_topic` (default `/camera/depth/color/points`); until that
  driver + a sweep exist, the planner idles with no cloud.

In full mode the IK executor listens on `/ik_waypoint_target` (driven by the executors), and
your Cartesian targets go to the **planner** on `/cartesian_target`. In minimal mode the IK
executor listens on `/cartesian_target` directly.

Wait until the arm has torqued on and RViz shows the current pose before sending anything.

All commands below run in a **new terminal**, exec'd into that same running container:

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && <command>"
```

The `<command>` shown in each section below goes in place of `<command>` in that wrapper.

### Sim time (nothing to do here)

`use_sim_time` is `false` everywhere on hardware and there is no flag to set — there is
no `/clock`, every node already shares the system clock. `localization_3d_node` logs
`Clock check OK: use_sim_time=False, /clock absent.` at startup; that is the healthy line.

`allow_latest_tf` is `false` in both sim and hardware, and should stay that way. Here the
risk is camera driver latency rather than a clock mismatch, but the consequence is the
same — a stale arm pose places the object somewhere it is not. Dropping a frame is the
safe failure.

## 3. First move — reset / pose the arm (verify comms)

The safest first command: send the arm to a known named pose. If this works, the driver and IK
executor are talking to the motors.

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_named_pose --pose neutral_carry"
# other poses: neutral_carry_yaw_left, neutral_carry_yaw_right
```

## 4. Smoke test (small automated lift-and-return)

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_smoke_test"
```

## 5. Send a Cartesian target (minimal mode — direct through the IK solver)

**In `minimal:=true` mode**, `/cartesian_target` goes **straight** to the DLS IK executor
(no planner, no obstacle checking). Send this target **first** — it was the cleanest
successful solve in the hardware logs, so it's the best probe:

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /cartesian_target geometry_msgs/msg/PointStamped \
'{header: {frame_id: rx150/base_link}, point: {x: 0.22, y: 0.00, z: 0.16}}'"
```

`frame_id` must be `rx150/base_link`. There is **no obstacle checking** in minimal mode — the
arm goes straight for the point, so keep the workspace clear and start with small, known-good
targets. Motion is intentionally slow (conservative params in the launch).

> In **full** mode this same `/cartesian_target` goes to the **planner** instead, which needs a
> point cloud — see §9 for driving the full stack.

### One-off target via the CLI (spins up its own solver, bypasses the running one)

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_dls_ik_executor --x 0.22 --y 0.00 --z 0.16 --frame rx150/base_link"
# optional orientation: --qx --qy --qz --qw
```

## 6. Staged Cartesian target suites

These publish to `/cartesian_target` like the sim suites; override the frame to
`rx150/base_link`. Run against **minimal** mode (they drive the IK executor directly):

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite smoke \
  --ros-args -p world_frame:=rx150/base_link"
# suites: smoke, lateral, pose, full
```

## 7. Inspect live state

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic echo /rx150/joint_states"

# the commands the IK executor is sending to the motors (hardware = group mode):
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic echo /rx150/commands/joint_group"
```

## 8. Shut down

- `Ctrl-C` in Terminal 1 stops the stack and detaches the driver.
- Power off the arm afterward. Do not leave it torqued on unattended.

## 9. Driving the FULL stack (planner + RRT)

In full mode (default), a target on `/cartesian_target` goes to the **planner**, which needs a
point cloud on `/planning/point_cloud`. Two ways to get one:

**A) With the RealSense (once the port is done).** Start the RealSense driver publishing to
`camera_points_topic` (default `/camera/depth/color/points`), then run the sweep to build the
map — same command as sim:
```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run data_collector scene_sweep_mapper"
```
⚠️ The sweep physically moves the real arm through the scan poses — clear the workspace.

**B) Without the camera — feed a canned cloud (to test planning/RRT on hardware now).**
Publish a recorded or hand-authored `sensor_msgs/PointCloud2` (frame `rx150/base_link`) on
`/planning/point_cloud`, then send a target on `/cartesian_target`. The planner, RRT fallback,
and executors will plan and drive the **real arm** against it — proving the whole motion
pipeline before the RealSense is ready.

Either way, once a cloud exists, send targets exactly like sim
([SIM_COMMANDS.md](SIM_COMMANDS.md) §4): a `PointStamped` on `/cartesian_target` in
`rx150/base_link`. The planner routes it, and on success the arm executes the Cartesian path,
or a **green** RRT joint path if the whole-body fallback fires.

## 10. Open / close the gripper

The stack launches `rx150_gripper_controller` automatically (both minimal and full modes). On
hardware the gripper is a **single servo** named `gripper` (motor ID 6), driven with a
`JointSingleCommand` on `/rx150/commands/joint_single`. The node gives you three ways to command
it — a named state or a value — on `/rx150/gripper_command`. The stack runs in **normalized**
units, so the value is `0.0` (closed) .. `1.0` (open) and **the exact same command works in sim**:

```bash
# Open / close (named states -> the open_position / closed_position params)
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: open}'"

docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: close}'"

# A specific openness: 0.0 = closed, 1.0 = open (here 70% open)
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /rx150/gripper_command std_msgs/msg/String '{data: \"0.7\"}'"
```

There is also a one-shot CLI (spins up its own node) and a `Float64` channel:

```bash
ros2 run bcr_arm_rx150 rx150_gripper_controller --state open      # or: --state close
ros2 run bcr_arm_rx150 rx150_gripper_controller --position 0.7 --ros-args -p command_units:=normalized
1ros2 topic pub --once /rx150/gripper_position std_msgs/msg/Float64 '{data: 0.7}'
```

> **Units + safety (read before first grip).** The launch ships `command_units:=normalized`, so
> `open`/`close`/`0..1` behave the same as sim. Under the hood the node maps `0..1` onto
> `closed_position`..`open_position`, whose values are in the gripper motor's **operating mode**
> units: `position` mode -> servo **radians**, `pwm` mode -> raw PWM effort. The endpoint defaults
> (`open=1.5`, `close=0.6`) are **position-mode placeholders — verify on the real arm** before
> trusting them, since a wrong closed value can stall the servo against the fingers. Override per
> launch, e.g. `-p open_position:=… -p closed_position:=…`. (To command raw servo units directly
> instead of `0..1`, set `-p command_units:=native`.)

## 11. Run the full pick-and-place mission (orchestrator)

Instead of driving the steps by hand across several terminals, one launch brings up
the whole mission — the full stack **plus** the conductor
(`rx150_pick_place_orchestrator`) that sequences it: sweep → find cup → move → grasp
→ lift → find goal → move → release → lift clear → home. It issues the *same*
`/cartesian_target` and `/rx150/gripper_command` messages you send manually, and
waits for each move to actually finish before the next — the planner and executors
now emit lifecycle **events** on `/motion/status` (`path:complete`/`joint:complete`
on success, `path:aborted`/`joint:aborted`/`planner:no_path` on failure), with a
`move_timeout_sec` backstop so a lost event can't hang the mission.

Both stubs now default **off**, and both real sources are wired, so this launch runs
the real pipeline end to end:

```
RealSense → scene_point_cloud (relay) → scene_sweep_mapper → /planning/point_cloud
RealSense → YOLO → localization_3d_node → vision_bridge → /vision/object_point
```

⚠️ **The sweep physically moves the arm** through three scan poses across the 90°
front sector — more if you widen `scan_waist_angles`. Clear the workspace before
starting.

```bash
# Terminal 1 — bring up the mission (nothing moves yet; waits for a start trigger)
# run from src/bcr_arm/ in your checkout
docker compose run --rm --service-ports rx150-hardware bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true"

# Terminal 2 — start the mission
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /mission/start std_msgs/msg/Empty '{}'"
```

The stubs remain as deliberate fallbacks — `use_vision_stub:=true` (canned cup/goal
points) and `use_sweep_stub:=true` (canned obstacle box, no arm motion) — for
exercising mission logic without a camera or a physical scan.

**Camera-specific args worth setting on the first run:**

| arg | default | set it when |
| --- | --- | --- |
| `goal_fallback_xyz` | `[0.20, 0.22, 0.06]` | **always** — the drop-off point is a fixed guess, not something the camera found |
| `min_depth_m` | `0.2` | already raised for a D435 (cannot focus closer). Lower it for a D405. |
| `cup_classes` | `[cup]` | the detector calls your cup something else (check `ros2 topic echo /perception/detections`) |
| `grasp_value` | `0.3` | **the main tuning dial** — how far to close on the cup |
| `grasp_z_offset` | `0.018` | move the grab point up or down the cup's body |
| `scan_waist_angles` | `[]` (90° front sector, 3 stations) | the scene sits outside ±45° — `'[-3.10,-2.356,-1.571,-0.785,0.0,0.785,1.571,2.356]'` sweeps the full circle in 8 stations |

Or start immediately on launch with `autostart:=true` (skips the trigger — only once
you trust the scene).

**Vision contract.** The orchestrator asks on `/vision/find_request`
(`std_msgs/String`, `"cup"` or `"goal"`) and expects a `geometry_msgs/PointStamped`
back on `/vision/object_point` in `rx150/base_link` — the exact vector you'd type into
`/cartesian_target`. `arm_perception`'s `vision_bridge` (`get_3d_point_node`) answers
it. Two properties of that node matter operationally:

- **Silence means "not found."** If nothing matching is detected it publishes
  *nothing*, the orchestrator times out, and the mission aborts. It never answers a
  cup request with a guess — an unseen cup must not send the arm somewhere.
- **Detections expire** after `detection_ttl_sec` (**300 s**). The cup is seen *during
  the sweep* and answered from cache a phase later, so this **must exceed one full
  sweep** — otherwise early sightings expire before the mission asks, and it reports
  "vision returned no cup".

See the VISION CONTRACT block in `rx150_pick_place_orchestrator.py`,
`arm_perception/VISION_README.md`, and
[PICK_PLACE_ORCHESTRATOR_PLAN.md](PICK_PLACE_ORCHESTRATOR_PLAN.md).

**Keep the cup level (`carry_level`).** By default the arm tracks target *positions*
only — the wrist is free to rotate as it moves, which can tip a full cup. Launch
with `carry_level:=true` to force a **level** gripper orientation while executing a
planned path. The IK solver uses a soft constraint: it keeps the wrist **mostly flat**
(pitch within ~±5° of horizontal), allowing the 5-DOF arm to still reach otherwise
unreachable positions by tilting slightly if necessary. The approach axis is flattened
to horizontal, gripper-up aligns to world-up (preserving facing), and it tracks that
orientation preferentially across the motion. A grasped cup stays upright in most poses;
tight reachability corners may incur small tilts. (Note: this *levels* the pose, it does
not merely hold the incoming one — a wrist that solved to e.g. 30° up is flattened to 0°.)

```bash
docker compose run --rm --service-ports rx150-hardware bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true \
     use_sweep_stub:=true use_vision_stub:=true"
```

> **Two caveats:**
> 1. This soft-constraint applies only on the **Cartesian** path (the normal case). If the
>    planner falls back to the **RRT** whole-body detour (cluttered scene), that path is
>    executed as raw joint configs and does **not** hold orientation — so in tight scenes a
>    level guarantee isn't absolute.
> 2. The pitch tolerance (default 5°) is tunable per-launch as `carry_level_pitch_tolerance_deg`.
>    Lower tolerances force stricter level-keeping but may make some positions unreachable;
>    higher tolerances allow more freedom but accept more tilt. Same flag/tolerance exist on
>    `rx150_pick_place_sim.launch.py`.

> ### The camera is now in the robot description
>
> `rx150_dls_stack.launch.py` passes `external_urdf_loc` =
> `urdf/rx150_realsense_camera.urdf.xacro`, which adds `rx150/camera_mount_link`
> on the gripper, and `perception.launch.py` publishes an identity static
> transform from it to the driver's `camera_link`. That is what gives the
> RealSense's frames a path to `rx150/base_link`.
>
> Previously there was no camera link at all on hardware, so the driver's TF tree
> floated free of the arm and **every** camera→base lookup failed — silently
> taking out both the sweep relay (no obstacle map) and 3D localization (no
> detections). Verify with:
>
> ```bash
> python3 bcr_arm_rx150/test/test_camera_tf_chain.py
> ```
>
> ⚠️ **The mounting numbers in that xacro are placeholders.** They prove
> connectivity, not correctness — the cloud will land confidently in the wrong
> place until they are measured. See
> [TASK_camera_extrinsics_urdf.md](TASK_camera_extrinsics_urdf.md).
>
> ### Sweep filtering is on for hardware
>
> The mission launch runs `scene_sweep_mapper` with statistical + radius outlier
> removal enabled (ported from `arm_perception/mapping_node.py`). Real depth
> carries flying pixels at depth discontinuities that a voxel grid preserves as
> solid-looking obstacles, and a few in the workspace make the planner refuse a
> clear path. Sim leaves them off — Gazebo's cloud is synthetic and clean.
> Tunables: `statistical_nb_neighbors`, `statistical_std_ratio`,
> `radius_outlier_radius`, `radius_outlier_min_neighbors`.
>
> ### Two pose parameters that matter on hardware
>
> **`home_joints` is now `[0, -0.65, -0.20, -1.00, 0]`, not the arm's zero.** Zero lays
> a link horizontally at **z = 0.255 across x = 0.265–0.373** — straight over the front
> workspace. Anything standing under that bar is in collision with the arm *while
> parked*. `home_on_abort` drives home through the **joint** executor, which does no
> collision checking, so with the old home an abort would drive the real arm into
> whatever is sitting in front of it. The retracted pose keeps every link within
> 0.051 m of the base axis. (`neutral_carry` and `lift_ready` are worse, not better —
> they reach down to z = 0.204 and 0.188.)
>
> **The sweep is what finds the cup.** Its sightings are kept and it turns the camera
> through a full circle, so the cup can be placed anywhere in the workspace — there is
> no fixed pose to re-tune, and no wedge it has to sit in. What to verify instead is
> that the sweep can *see* your object: it needs to stand on a riser, and
> `detection_ttl_sec` (300 s) must exceed one full sweep. See REMAINING_WORK.md §2.1.
>
> `observe_joints` still exists as an optional extra look, off by default (`[]`).
> Pass a pose to enable it: `observe_joints:='[0.0,-1.65,1.07,1.25,0.0]'`.

**Safety.** If the planner can't find a whole-body path it emits `planner:no_path`,
so the mission **aborts immediately and returns home without closing the gripper** —
it will not grasp empty air. Key params (override with `-p name:=value` on the
orchestrator, or edit the launch): `grasp_value` (placeholder — **set a real value
for the cup**, see §10 units note), `lift_dz`, `clearance_dz`, `place_height`,
`move_timeout_sec` (event backstop).

## Notes / differences from sim (quick reference)

| | Sim ([SIM_COMMANDS.md](SIM_COMMANDS.md)) | Hardware (this doc) |
|---|---|---|
| Compose service / profile | `rx150-sim` / `sim` | `rx150-hardware` / `hardware` |
| Launch file | `rx150_dls_sim_stack.launch.py` | `rx150_dls_stack.launch.py` |
| Arm interface | Gazebo ros2_control | Interbotix `xs_sdk` (real driver) |
| IK executor command mode | `trajectory` → `/rx150/arm_controller/joint_trajectory` | `group` → `/rx150/commands/joint_group` |
| Target `frame_id` | `rx150/base_link` | `rx150/base_link` |
| Motion speed | default | conservative (slow) for safe bring-up |
| Planner / RRT / executors | yes | **yes** (full mode, default) |
| Camera / sweep | yes (Gazebo) | yes (RealSense, in the mission launch) |
| Object detection | YOLO on the Gazebo camera | YOLO on the RealSense |

The IK solver, DLS math, joint limits, planner, RRT fallback, **and the whole perception
pipeline** are identical on both — only the arm command channel (`group` vs `trajectory`)
and the camera **source** differ. The perception launch takes the three camera topics as
arguments precisely so the same nodes serve Gazebo and the RealSense.

---

## Pick-and-Place Mission

Full reference: **[PICK_PLACE_MISSION.md](PICK_PLACE_MISSION.md)**.
Outstanding work before a physical run: **[REMAINING_WORK.md](REMAINING_WORK.md)**.

```bash
# Terminal 1 -- the whole mission stack
# run from src/bcr_arm/ in your checkout

docker compose run --rm --name rx150 --service-ports rx150-hardware bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true"

# Terminal 2 -- keyboard control (s = start, x = stop, r = restart, q = quit)
docker exec -it rx150 bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

⚠️ **The sweep physically moves the arm.** Clear the workspace before pressing `s`.

Flags: see PICK_PLACE_MISSION.md §1 and §2. Set `goal_fallback_xyz` every run.

> **Both gaps are now closed.** With the stubs off (the default),
> `scene_sweep_mapper` subscribes to `/sweep/start` and emits `sweep:complete`, and
> `vision_bridge` answers `/vision/find_request` on `/vision/object_point`. The arm
> reaches for **whatever the camera sees** — except the *goal*, which is still the
> fixed `goal_fallback_xyz` point, because the detector has no drop-off class.
>
> To exercise the mission without a camera or a physical scan:
>
> ```bash
> ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py \
>   carry_level:=true use_sweep_stub:=true use_vision_stub:=true
> ```

**Do not trust the first physical grasp.** The gripper open/closed endpoints
(`_HW_SERVO_CLOSED` 0.6 / `_HW_SERVO_OPEN` 1.5 in `rx150_gripper_controller.py`) are
unverified placeholders in servo radians, and `grasp_value` interpolates between them.
Verify those on the arm first.
