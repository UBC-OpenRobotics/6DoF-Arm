# RX-150 Hardware Stack — Docker Command Reference

Command reference for building, launching, and driving the **physical** RX-150 DLS stack
(`rx150_dls_stack.launch.py`) through the workspace's Docker Compose setup. This is the
hardware counterpart to [SIM_COMMANDS.md](SIM_COMMANDS.md).

There is no autonomous planning or vision in this stack: the arm driver, the DLS IK executor,
and the gripper controller are the whole stack. `/cartesian_target` goes straight to the IK
executor — no obstacle checking. A human (teleop/haptics) is the collision-avoidance, so clear
the workspace before moving.

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

```bash
# run from src/bcr_arm/ in your checkout
docker compose --profile hardware run --rm --service-ports rx150-hardware
```

> **Why the `bash -lc "source ... && ..."` wrapper when overriding the command?** Passing a
> command to `docker compose run` *replaces* the service's `command:`, and that `command:` is
> what sourced the workspace. The image entrypoint only sources `/opt/ros/humble`, so a bare
> `... rx150-hardware ros2 launch ...` fails with `package 'bcr_arm_rx150' not found`. Source
> it yourself whenever you override the command:
> ```bash
> docker compose run --rm --service-ports rx150-hardware bash -lc \
>   "source /workspaces/bcr_arm/install/setup.bash && ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py"
> ```
> The no-argument form above needs no wrapper — it keeps the service's own `command:`, which
> additionally runs `setup_workspace.sh` (rosdep + `colcon build`).

This runs `rx150_dls_stack.launch.py`:

- `rx150_control.launch.py` — the Interbotix RX-150 control launch (the **real** low-level
  driver, `xs_sdk`), which talks to the motors over the U2D2 and publishes `/rx150/joint_states`
- RViz (`use_rviz:=true` by default) on `rviz/rx150_dls_stack.rviz` — robot model + TF.
  Override with `rvizconfig:=/path/to.rviz`.
- `rx150_dls_ik_executor` — the custom DLS IK solver in **`group`** command mode
  (`interbotix_xs_msgs/JointGroupCommand` on `/rx150/commands/joint_group`), with
  `world_frame: rx150/base_link` and **conservative motion params** (slow velocity, small
  steps, heavier damping, 4 s goal time) so the real arm moves gently. Raise
  `max_joint_velocity` / `max_joint_step` in
  [rx150_dls_stack.launch.py](launch/rx150_dls_stack.launch.py) once you trust the setup.
  It listens directly on `/cartesian_target` (`geometry_msgs/PointStamped`) and
  `/cartesian_target_pose` (`geometry_msgs/PoseStamped`) — this is the seam a teleop/haptic
  input source publishes onto.
- `rx150_gripper_controller`

Wait until the arm has torqued on and RViz shows the current pose before sending anything.

All commands below run in a **new terminal**, exec'd into that same running container:

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && <command>"
```

The `<command>` shown in each section below goes in place of `<command>` in that wrapper.

### Sim time (nothing to do here)

`use_sim_time` is `false` everywhere on hardware and there is no flag to set — there is
no `/clock`, every node already shares the system clock.

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

## 5. Send a Cartesian target (direct through the IK solver)

`/cartesian_target` goes **straight** to the DLS IK executor — no planner, no obstacle
checking. Send this target **first** — it was the cleanest successful solve in the hardware
logs, so it's the best probe:

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic pub --once /cartesian_target geometry_msgs/msg/PointStamped \
'{header: {frame_id: rx150/base_link}, point: {x: 0.22, y: 0.00, z: 0.16}}'"
```

`frame_id` must be `rx150/base_link`. There is **no obstacle checking** — the arm goes
straight for the point, so keep the workspace clear and start with small, known-good
targets. Motion is intentionally slow (conservative params in the launch).

### One-off target via the CLI (spins up its own solver, bypasses the running one)

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_dls_ik_executor --x 0.22 --y 0.00 --z 0.16 --frame rx150/base_link"
# optional orientation: --qx --qy --qz --qw
```

## 6. Staged Cartesian target suites

These publish to `/cartesian_target` like the sim suites; override the frame to
`rx150/base_link`:

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite smoke \
  --ros-args -p world_frame:=rx150/base_link"
# suites: smoke, lateral, pose, full
```

## 7. Recorded joint-pose replay (no IK, no vision)

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 launch bcr_arm_rx150 rx150_joint_sequence.launch.py"
```

Replays a fixed list of hand-recorded joint poses + gripper open/close — proves the arm and
gripper work before anything computes a target. The arm moves on launch; clear the workspace.

## 8. Inspect live state

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic echo /rx150/joint_states"

# the commands the IK executor is sending to the motors (hardware = group mode):
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 topic echo /rx150/commands/joint_group"
```

## 9. Shut down

- `Ctrl-C` in Terminal 1 stops the stack and detaches the driver.
- Power off the arm afterward. Do not leave it torqued on unattended.

## 10. Open / close the gripper

The stack launches `rx150_gripper_controller` automatically. On hardware the gripper is a
**single servo** named `gripper` (motor ID 6), driven with a `JointSingleCommand` on
`/rx150/commands/joint_single`. The node gives you three ways to command it — a named state or
a value — on `/rx150/gripper_command`. The stack runs in **normalized** units, so the value is
`0.0` (closed) .. `1.0` (open) and **the exact same command works in sim**:

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

## 11. Joint-state logging

```bash
docker compose exec rx150-hardware bash -lc "source /workspaces/bcr_arm/install/setup.bash && \
ros2 run data_collector collector"
```

Logs `/rx150/joint_states` to a timestamped pickle + csv under
`~/openRobotics/bcr_arm/data/positions` — useful for recording a teleop session for offline
analysis.

## Notes / differences from sim (quick reference)

| | Sim ([SIM_COMMANDS.md](SIM_COMMANDS.md)) | Hardware (this doc) |
|---|---|---|
| Compose service / profile | `rx150-sim` / `sim` | `rx150-hardware` / `hardware` |
| Launch file | `rx150_dls_sim_stack.launch.py` | `rx150_dls_stack.launch.py` |
| Arm interface | Gazebo ros2_control | Interbotix `xs_sdk` (real driver) |
| IK executor command mode | `trajectory` → `/rx150/arm_controller/joint_trajectory` | `group` → `/rx150/commands/joint_group` |
| Target `frame_id` | `rx150/base_link` | `rx150/base_link` |
| Motion speed | default | conservative (slow) for safe bring-up |

The IK solver, DLS math, and joint limits are identical on both — only the arm command
channel (`group` vs `trajectory`) differs.
