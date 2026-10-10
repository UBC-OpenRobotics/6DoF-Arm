# BCR Arm

ROS 2 workspace for the RX-150 teleoperation track: a 5-DOF Interbotix RX-150 driven directly
by a human operator (haptic input), with a custom damped-least-squares (DLS) Cartesian IK
solver translating Cartesian targets into joint commands in real time. Runs in both simulation
and on the physical arm.

- **Simulation:** [bcr_arm_rx150/SIM_COMMANDS.md](bcr_arm_rx150/SIM_COMMANDS.md)
- **Hardware:** [bcr_arm_rx150/HARDWARE_COMMANDS.md](bcr_arm_rx150/HARDWARE_COMMANDS.md)

There is no autonomous planning or vision in this stack by design: a human is the
collision-avoidance, driving the arm by publishing Cartesian targets (position and/or
orientation) that the IK executor tracks continuously. The teleop/haptic input device
integration itself (joystick, haptic controller, force feedback) is separate, upcoming work —
this workspace currently provides the arm-side half: bring-up, the IK solver/executor, and the
gripper.

## Relevant Packages

- `bcr_arm_rx150`: RX-150 launch files, the custom DLS IK solver/executor, gripper control, and safe hardware test utilities
- `bcr_arm_common`: shared RX-150 forward kinematics used by `bcr_arm_rx150`
- `data_collector`: joint-state logging utility, useful for recording teleop sessions
- `interbotix`: vendored subset of the Interbotix RX-150 support stack used by the RX-150 sim, control, and MoveIt launches

## Available Setups

### RX-150 Gazebo

The RX-150 simulation workflow is exposed through `bcr_arm_rx150`, a thin wrapper around
the upstream Interbotix RX-150 Gazebo Classic stack.

Primary launch (bare Gazebo bringup):

```bash
ros2 launch bcr_arm_rx150 rx150_gz_classic.launch.py
```

**IK executor + gripper on top of Gazebo, and all setup/run commands:** see
**[bcr_arm_rx150/SIM_COMMANDS.md](bcr_arm_rx150/SIM_COMMANDS.md)**.

### RX-150 Hardware

This is the physical RX-150 workflow. It uses this repo for both the project-level scripts
and the vendored RX-150 support stack, including the low-level driver, descriptions, and
launch infrastructure.

Use this when the goal is to:

- control the real RX-150 from `bcr_arm`
- run safe named poses and smoke tests
- drive the custom DLS IK executor directly (e.g. from a teleop/haptic input source)

Primary launch (driver + IK executor + gripper):

```bash
ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py
```

`rx150_control.launch.py` brings up just the low-level driver if you need it standalone.

**Full setup/run commands (Docker launch, safe bring-up, driving the stack):** see
**[bcr_arm_rx150/HARDWARE_COMMANDS.md](bcr_arm_rx150/HARDWARE_COMMANDS.md)**.

## Prerequisites

Recommended environment:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11 for the RX-150 Gazebo simulation

Install the base tools:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  gazebo \
  python3-colcon-common-extensions \
  python3-rosdep
```

If `rosdep` has not been initialized yet:

```bash
sudo rosdep init
rosdep update
```

## Build

These commands assume this repository itself is the colcon workspace root.

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The vendored `interbotix` subtree is included so the RX-150 workflows can build from a fresh clone without requiring a separate `~/interbotix_ws`.

## Docker Support

This repo includes a workspace-level Docker setup for the RX-150 simulation and hardware workflows:

- `workspace`: interactive development shell
- `rx150-sim`: RX-150 Gazebo sim plus the custom DLS solver
- `rx150-hardware`: physical RX-150 plus the custom DLS solver

Support expectations:

- Ubuntu/Linux host: supported for development, RX-150 sim, and RX-150 hardware
- macOS/Windows host: useful for development and limited containerized workflows, but not recommended as the primary path for physical robot access

Important note:

- The Docker setup makes the repo environment reproducible.
- Physical RX-150 use still depends on the host exposing the USB/serial devices correctly.
- The `rx150-hardware` Docker service is intended for Linux hosts.

Build the image:

```bash
# run from src/bcr_arm/ in your checkout
docker compose build
```

Open an interactive development shell:

```bash
docker compose run --rm workspace
```

Launch the RX-150 sim stack in Docker:

```bash
xhost +local:docker
docker compose run --rm --service-ports rx150-sim
```

Launch the RX-150 hardware stack in Docker on Linux:

```bash
xhost +local:docker
docker compose run --rm --service-ports rx150-hardware
```

The helper script `bcr-setup-workspace` inside the container runs:

- `rosdep install --from-paths . --ignore-src -r -y`
- `colcon build --packages-up-to bcr_arm_rx150 --symlink-install`

This means the Docker launch services build the RX-150 path before starting the requested stack.

## Optional External Underlay

If an existing Interbotix workspace is already available, it can still be sourced as an underlay. This is optional for the RX-150 path in this repo, not required.

Typical setup:

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
# run from src/bcr_arm/ in your checkout
colcon build --symlink-install
source install/setup.bash
```

When both are present, the local vendored packages in this repo are intended to be the project-facing source of truth.

For RX-150 Gazebo Classic simulation, it is also helpful to source Gazebo Classic's setup file:

```bash
source /usr/share/gazebo-11/setup.sh
```

## RX-150 Hardware Workflow

Launch the physical RX-150 driver from this repo:

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bcr_arm_rx150 rx150_control.launch.py
```

Send a named pose in another terminal:

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run bcr_arm_rx150 rx150_named_pose --pose neutral_carry
```

Run the small lift-and-return smoke test:

```bash
ros2 run bcr_arm_rx150 rx150_smoke_test
```

Launch the MoveIt interface for the real arm:

```bash
ros2 launch bcr_arm_rx150 rx150_moveit_interface.launch.py
```

Run the custom DLS stack for the physical RX-150:

```bash
ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py
```

This brings up the arm driver, the DLS IK executor (conservative motion, frame
`rx150/base_link`), and the gripper controller. The IK executor listens directly on
`/cartesian_target` (`geometry_msgs/PointStamped`) and `/cartesian_target_pose`
(`geometry_msgs/PoseStamped`) — this is the seam a teleop/haptic input source publishes onto.
Targets use frame `rx150/base_link`.

**See [bcr_arm_rx150/HARDWARE_COMMANDS.md](bcr_arm_rx150/HARDWARE_COMMANDS.md)** for the full,
authoritative hardware command reference (Docker launch, named poses, smoke test, targets, and
suites). Quick examples:

```bash
ros2 run bcr_arm_rx150 rx150_named_pose --pose neutral_carry
ros2 run bcr_arm_rx150 rx150_smoke_test
ros2 run bcr_arm_rx150 rx150_target_test_suite --suite smoke --ros-args -p world_frame:=rx150/base_link
# suites: smoke (safest), lateral (center/left/right), pose (exact poses), full (all)
ros2 run bcr_arm_rx150 rx150_dls_ik_executor --x 0.22 --y 0.00 --z 0.16 --frame rx150/base_link
```

## RX-150 Gazebo Sim Workflow

Launch the Interbotix RX-150 Gazebo Classic sim from this repo:

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bcr_arm_rx150 rx150_gz_classic.launch.py
```

Useful variants:

```bash
ros2 launch bcr_arm_rx150 rx150_gz_classic.launch.py use_rviz:=false
ros2 launch bcr_arm_rx150 rx150_gz_classic.launch.py paused:=true
```

If you want MoveIt on top of RX-150 Gazebo sim, launch the existing wrapper with
`hardware_type:=gz_classic`:

```bash
ros2 launch bcr_arm_rx150 rx150_moveit_interface.launch.py hardware_type:=gz_classic
```

This sim entry point is a thin wrapper around the vendored Interbotix RX-150 sim stack in `interbotix`, so the RX-150 workflow remains locally launchable from `bcr_arm`.

For the IK executor + gripper on top of Gazebo (the sim equivalent of the hardware stack
above), launch `rx150_dls_sim_stack.launch.py` — see
**[bcr_arm_rx150/SIM_COMMANDS.md](bcr_arm_rx150/SIM_COMMANDS.md)** for the full Docker command
reference (build, launch, sending targets). Same IK executor node that runs on the physical
arm (see the hardware workflow above).
