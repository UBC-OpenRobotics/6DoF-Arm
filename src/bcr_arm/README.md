# BCR Arm

ROS 2 workspace for the RX-150 teleoperation track: a 5-DOF Interbotix RX-150 driven directly
by a human operator (haptic input), with a custom damped-least-squares (DLS) Cartesian IK
solver translating Cartesian targets into joint commands in real time. Runs in both simulation
and on the physical arm.

There is no autonomous planning or vision in this stack by design: a human is the
collision-avoidance, driving the arm by publishing Cartesian targets (position and/or
orientation) that the IK executor tracks continuously. The teleop/haptic input device
integration itself (joystick, haptic controller, force feedback) is separate, upcoming work —
this workspace currently provides the arm-side half: bring-up, the IK solver/executor, and the
gripper.

## Quick Start

**Sim**, driver + IK executor + gripper on top of Gazebo:

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch bcr_arm_rx150 rx150_dls_sim_stack.launch.py
```

**Hardware**, same stack on the real arm (conservative motion, frame `rx150/base_link`):

```bash
# run from src/bcr_arm/ in your checkout
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch bcr_arm_rx150 rx150_dls_stack.launch.py
```

Either way, the IK executor listens directly on `/cartesian_target`
(`geometry_msgs/PointStamped`) and `/cartesian_target_pose` (`geometry_msgs/PoseStamped`) — the
seam a teleop/haptic input source publishes onto:

```bash
ros2 topic pub --once /cartesian_target geometry_msgs/msg/PointStamped \
  '{header: {frame_id: rx150/base_link}, point: {x: 0.22, y: 0.00, z: 0.16}}'
```

For everything else (Docker launch, named poses, smoke test, target suites, gripper commands,
MoveIt) see the full command references:
**[bcr_arm_rx150/SIM_COMMANDS.md](bcr_arm_rx150/SIM_COMMANDS.md)** and
**[bcr_arm_rx150/HARDWARE_COMMANDS.md](bcr_arm_rx150/HARDWARE_COMMANDS.md)**.

## Relevant Packages

- `bcr_arm_rx150`: RX-150 launch files, the custom DLS IK solver/executor, gripper control, and safe hardware test utilities
- `bcr_arm_common`: shared RX-150 forward kinematics used by `bcr_arm_rx150`
- `data_collector`: joint-state logging utility, useful for recording teleop sessions
- `interbotix`: vendored subset of the Interbotix RX-150 support stack used by the RX-150 sim, control, and MoveIt launches

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
