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

- Gazebo Classic with the bare RX-150 arm
- `rx150_dls_ik_executor` — the custom DLS IK solver, listening directly on
  `/cartesian_target` (`geometry_msgs/PointStamped`) and `/cartesian_target_pose`
  (`geometry_msgs/PoseStamped`) — no planner, no camera. This is the seam a teleop/haptic
  input source publishes onto.
- `rx150_gripper_controller`

### Sim time

Every launch here defaults to `use_sim_time:=true`. **Nothing to set.**

It matters because Gazebo stamps TF with its own clock. A node left on the wall clock
cannot match those stamps to anything.

All commands below run in a **new terminal**, exec'd into that same running container:

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && <command>"
```

## 3. Send a Cartesian Target (directly to the IK executor)

```bash
docker compose exec rx150-sim bash -lc "source /workspaces/bcr_arm/install/setup.bash && 
ros2 topic pub --once /cartesian_target geometry_msgs/msg/PointStamped \
'{header: {frame_id: rx150/base_link}, point: {x: 0.22, y: 0.00, z: 0.16}}'"
```

`frame_id` must be `rx150/base_link`. There is no obstacle checking on this path — the arm
moves straight to the target. This is intentional: a human (via teleop/haptics) is the
collision-avoidance.

## 4. Other Nodes You Can Run Manually

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
[HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md) §4.

### Smoke test (small automated lift-and-return)

```bash
ros2 run bcr_arm_rx150 rx150_smoke_test
```

### Recorded joint-pose replay (no IK, no vision)

```bash
ros2 launch bcr_arm_rx150 rx150_joint_sequence.launch.py
```

### Staged Cartesian target suites

```bash
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite smoke
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite lateral
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite pose
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite full --auto --pause-sec 5.0 \
  --ros-args -p world_frame:=rx150/base_link
```

### One-off target straight through the IK solver

```bash
ros2 run bcr_arm_rx150 rx150_dls_ik_executor --x 0.20 --y 0.00 --z 0.16 --frame rx150/base_link
# optional orientation: --qx --qy --qz --qw
```

### Joint-state logging (`data_collector`, not wired into this launch by default)

```bash
ros2 run data_collector collector   # logs /joint_states to a pickle + csv for offline analysis
```

## 5. Inspect Live State

```bash
ros2 topic echo /rx150/joint_states
ros2 topic echo /cartesian_target
```
