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
ros2 topic pub --once /rx150/gripper_position std_msgs/msg/Float64 '{data: 0.7}'
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

Vision is external (a teammate owns it). Until it's connected, two stubs let the
whole mission run: `vision_placeholder` (answers the vision request with a canned
point) and `sweep_placeholder` (latches a canned obstacle cloud). Swap each out with
`use_vision_stub:=false` / `use_sweep_stub:=false` once the real nodes publish the
same topics.

```bash
# Terminal 1 — bring up the mission (nothing moves yet; waits for a start trigger)
cd ~/openRobotics/src/bcr_arm
docker compose run --rm --service-ports rx150-hardware \
  ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py

# Terminal 2 — start the mission
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /mission/start std_msgs/msg/Empty '{}'"
```

Or start immediately on launch with `autostart:=true` (skips the trigger — only once
you trust the scene).

**Vision contract (for the teammate).** The orchestrator asks on
`/vision/find_request` (`std_msgs/String`, `"cup"` or `"goal"`) and expects a
`geometry_msgs/PointStamped` back on `/vision/object_point` in `rx150/base_link` — the
exact vector you'd type into `/cartesian_target`. That's the whole interface; see the
VISION CONTRACT block in `rx150_pick_place_orchestrator.py` and
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
docker compose run --rm --service-ports rx150-hardware \
  ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true
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
| Camera / sweep | yes (Gazebo) | **pending** RealSense port |

The IK solver, DLS math, joint limits, planner, and RRT fallback are identical on both — only
the arm command channel (`group` vs `trajectory`) and the cloud **source** (RealSense vs
Gazebo) differ. That's by design: the planner and RRT fallback already run on this hardware
launch unchanged; the only missing piece is the RealSense feeding the cloud.

---

## Pick-and-Place Mission

Full reference: **[PICK_PLACE_MISSION.md](PICK_PLACE_MISSION.md)**.

```bash
# Terminal 1 -- the whole mission stack
docker compose run --rm --service-ports rx150-hardware bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true"

# Terminal 2 -- keyboard control (s = start, x = stop, r = restart, q = quit)
docker compose exec -it rx150-hardware bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

**Do not trust the first physical grasp.** The gripper open/closed endpoints
(`_HW_SERVO_CLOSED` 0.6 / `_HW_SERVO_OPEN` 1.5 in `rx150_gripper_controller.py`) are
unverified placeholders in servo radians, and `grasp_value` interpolates between them.
Verify those on the arm first. Vision and the sweep are also still stubs -- see
PICK_PLACE_MISSION.md §6.
