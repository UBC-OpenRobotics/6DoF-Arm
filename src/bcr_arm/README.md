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

## Fresh Machine Setup

Two paths: **native Linux** or **Docker**. Both get you to the same `colcon build`.

> **Use native Linux for end-to-end testing.** Teleop + haptics is a closed control loop —
> operator input to arm motion, and back through force feedback — where timing jitter is felt
> directly as lag or instability, not just a logged number. Docker here runs with
> `network_mode: host` and `privileged: true` specifically to stay close to native performance,
> and is fine for development and the sim workflow. But it's still an extra runtime layer
> between the control loop and the hardware (container scheduling, cgroups, device
> passthrough), and on anything other than a Linux host (Docker Desktop on macOS/Windows) that
> gap gets much bigger. For latency-sensitive end-to-end testing on the physical arm, run
> native on Linux.

### Native Linux

Recommended environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

```bash
# 1. ROS 2 + Gazebo Classic + build tools
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  gazebo \
  python3-colcon-common-extensions \
  python3-rosdep

# 2. rosdep, if not already initialized
sudo rosdep init
rosdep update

# 3. Clone and build (src/bcr_arm is the colcon workspace root)
git clone https://github.com/UBC-OpenRobotics/6DoF-Arm.git
cd 6DoF-Arm/src/bcr_arm
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The vendored `interbotix` subtree is included so the RX-150 workflows build from a fresh clone
without requiring a separate `~/interbotix_ws`.

**Physical arm only — U2D2/Dynamixel udev rule (one-time, on the host):**

```bash
sudo cp interbotix/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"   # log out/in (or reboot) for the group change to apply
```

This rule also sets the U2D2's USB latency timer to 1 ms (vs. the Linux default 16 ms) — it's
part of why servo communication is fast and consistent on native Linux. Without it the
Dynamixel bus still works, just slower and jerkier.

Then run the [Quick Start](#quick-start) commands above.

### Docker

Install Docker Engine + the Compose plugin on an Ubuntu/Linux host (this is the supported
path — macOS/Windows via Docker Desktop works for development but is not recommended for
physical-robot use).

```bash
# 1. let containers open your X display (for RViz/Gazebo GUIs)
xhost +local:docker

# 2. build the image
# run from src/bcr_arm/ in your checkout
docker compose build
```

**Physical arm only — the U2D2/Dynamixel udev rule still goes on the *host*, not in the
container.** `rx150-hardware` mounts `/run/udev` read-only and `/dev` directly — it reads
whatever the host kernel already set up, so do the same one-time step as the native guide
above, on the host, before launching the hardware service.

```bash
# Development shell
docker compose run --rm workspace

# Sim stack
xhost +local:docker
docker compose run --rm --service-ports rx150-sim

# Hardware stack (Linux host only)
xhost +local:docker
docker compose run --rm --service-ports rx150-hardware
```

The helper script `bcr-setup-workspace` inside the container runs `rosdep install` and
`colcon build --packages-up-to bcr_arm_rx150` before each launch, so the services build the
RX-150 path automatically — no separate build step needed inside the container.

### Optional External Underlay

If an existing Interbotix workspace is already available, it can be sourced as an underlay
instead of (or alongside) the vendored `interbotix` subtree — optional, not required:

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
# run from src/bcr_arm/ in your checkout
colcon build --symlink-install
source install/setup.bash
```

When both are present, the local vendored packages in this repo are the project-facing source
of truth. For RX-150 Gazebo Classic simulation, it also helps to source Gazebo Classic's setup
file:

```bash
source /usr/share/gazebo-11/setup.sh
```
