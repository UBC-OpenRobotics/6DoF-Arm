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
> **The one piece still pending: the camera.** The planner/RRT/executors are fully portable
> and run on hardware now, but they need a point cloud on `/planning/point_cloud`, which comes
> from the **RealSense driver + sweep** — the in-progress port tracked in
> [TASK_camera_extrinsics_urdf.md](TASK_camera_extrinsics_urdf.md) and
> [TASK_realsense_sweep_pipeline.md](TASK_realsense_sweep_pipeline.md). Until the RealSense is
> wired, the full stack comes up but the planner **idles with no cloud**. So to actually move
> the arm today: use `minimal:=true` and drive the IK executor directly (§5), or feed a canned
> cloud into the full stack (§9). Everything uses the sim frame — **`rx150/base_link`** — which
> is the real TF base frame the Interbotix description publishes, so there's no sim/real
> divergence.

## 0. Before you power on (safety)

- Clear the arm's workspace — it **will** move when you launch and send targets.
- Keep the power switch / e-stop within reach. Kill power if anything looks wrong.
- Make sure the U2D2 serial adapter is plugged in and the arm is powered before launching.
  The `rx150-hardware` container is `privileged` and mounts `/dev` + `/run/udev`, so it can
  reach the U2D2 without extra host config.

## 1. Build & Setup

One-time, or after changing the Dockerfile or package dependencies.

```bash
cd ~/openRobotics/src/bcr_arm
xhost +local:docker                            # let containers open your X display (RViz)
docker compose --profile hardware build        # or: docker compose build rx150-hardware
```

## 2. Launch the Hardware Stack (Terminal 1 — keep running)

**Full stack (default):**
```bash
cd ~/openRobotics/src/bcr_arm
docker compose --profile hardware run --rm --service-ports rx150-hardware
```

**Minimal — arm + IK only (recommended until the RealSense is wired):**
```bash
cd ~/openRobotics/src/bcr_arm
docker compose --profile hardware run --rm --service-ports rx150-hardware \
  ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py minimal:=true
```

The launch runs the in-container workspace setup + build, then `rx150_dls_stack.launch.py`.
Always present:

- `rx150_control.launch.py` — the Interbotix RX-150 control launch (the **real** low-level
  driver, `xs_sdk`), which talks to the motors over the U2D2 and publishes `/rx150/joint_states`
- RViz (`use_rviz:=true` by default) showing the live robot model
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
| Camera / sweep | yes (Gazebo) | **pending** RealSense port |

The IK solver, DLS math, joint limits, planner, and RRT fallback are identical on both — only
the arm command channel (`group` vs `trajectory`) and the cloud **source** (RealSense vs
Gazebo) differ. That's by design: the planner and RRT fallback already run on this hardware
launch unchanged; the only missing piece is the RealSense feeding the cloud.
