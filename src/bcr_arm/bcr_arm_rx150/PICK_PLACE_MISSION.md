# RX-150 Pick-and-Place Mission

End-to-end cup pick-and-place, in one launch, in **sim or on the physical arm**. The
mission is repeatable and keyboard-driven: it idles, runs a full cycle on request,
then returns to idle ready to go again — no relaunching.

This document is the single reference for the mission. For the lower-level manual
workflows (send one Cartesian target, run the sweep by hand, pose the arm) see
[SIM_COMMANDS.md](SIM_COMMANDS.md) and [HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md).

---

## 1. Run it in simulation

Two terminals, both from the repo's `src/bcr_arm` directory.

**Terminal 1 — the mission stack** (Gazebo + planner + executors + gripper + vision
and sweep stubs + orchestrator):

```bash
cd ~/openRobotics/src/bcr_arm

docker compose run --rm --service-ports rx150-sim bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place_sim.launch.py carry_level:=true"
```

Wait for the build to finish and Gazebo to come up. Nothing moves; the orchestrator
logs:

```
Idle. Waiting for mission start on /mission/start (or /mission/restart to run again, /mission/stop to stop).
```

**Terminal 2 — keyboard control:**

```bash
cd ~/openRobotics/src/bcr_arm

docker compose exec -it rx150-sim bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

> **Terminal 2 must wait for Terminal 1's build.** `mission_keyboard` does not exist
> until `setup_workspace.sh` finishes; starting it early gives "executable not found".

> **If `compose exec` cannot find the service:** `docker compose run` names its
> container `bcr_arm-rx150-sim-run-<hash>`. Add `--name rx150` to the Terminal 1
> command and use `docker exec -it rx150 bash -lc "..."` in Terminal 2.

---

## 2. Run it on the physical arm

Identical, against the hardware profile. The arm must be powered and connected before
Terminal 1.

```bash
# Terminal 1
cd ~/openRobotics/src/bcr_arm

docker compose run --rm --service-ports rx150-hardware bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true"

# Terminal 2
docker compose exec -it rx150-hardware bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

**Before the first real grasp**, read §6 — the gripper open/closed endpoints are
unverified placeholders, and the mission uses the canned vision/sweep stubs unless you
turn them off.

---

## 3. Keyboard controls

| key | action |
| --- | --- |
| `s` | **start** a mission |
| `x` | **stop** — cancels motion immediately, parks the arm home, returns to idle |
| `r` | **restart** — stop, then begin a fresh mission straight away (works from idle too) |
| `q` | quit the keyboard node only; the mission stack keeps running |

The `-it` is required. Without a TTY the node cannot read key presses; it says so and
exits rather than silently ignoring you. It must run in its **own terminal** — a node
started by `ros2 launch` shares one multiplexed stdin with every other node in that
launch and cannot reliably read keys.

Equivalent topics, if you would rather script it:

```bash
ros2 topic pub --once /mission/start   std_msgs/msg/Empty '{}'
ros2 topic pub --once /mission/stop    std_msgs/msg/Empty '{}'
ros2 topic pub --once /mission/restart std_msgs/msg/Empty '{}'
```

---

## 4. What a cycle does

| phase | action |
| --- | --- |
| 1 | **Sweep** — request an obstacle map on `/planning/point_cloud` |
| 2 | **Vision** — locate the cup |
| 3 | **Move to cup** — travel to a hover point above it, open the gripper, descend |
| 4 | **Grasp** — close to `grasp_value` |
| 5 | **Lift** — raise `lift_dz` straight up |
| 6 | **Vision** — locate the goal |
| 7 | **Move to goal** — hover above it, descend |
| 8 | **Release** — open |
| 9 | **Lift clear** — raise `clearance_dz` off the placed cup |
| 10 | **Home** |

Three design points that are easy to undo by accident:

**The approach is hover-then-descend, not straight in.** 

**Level-carry is on for phases 3–7 only.**

**The gripper only actuates when stationary.** `path:complete` 

### Event contract

Motion nodes emit lifecycle events on `/motion/status` (`std_msgs/String`); the
orchestrator waits for a terminal one before advancing.

- success: `path:complete`, `joint:complete`
- failure: `path:aborted`, `joint:aborted`, `planner:no_path`, `ik:tilt_exceeded`

`move_timeout_sec` (60 s) is a backstop for a lost event, not the primary signal.

### How stop actually stops

`/mission/stop` unwinds the orchestrator's phase sequence *and* broadcasts
`std_msgs/Empty` on **`/motion/cancel`**, which the Cartesian executor, the joint
executor and the DLS IK executor all subscribe to. The executors drop their remaining
waypoints; the IK node forgets its servo target (without that it would keep driving to
the last waypoint). The arm halts where it is, then the orchestrator parks it home.

Every cycle re-runs the sweep: the orchestrator clears its cached cloud count at the
start of each mission, so phase 1 waits for a cloud published *after* that cycle's
`/sweep/start` rather than being satisfied instantly by the previous one.

---

## 5. Tuning

All are launch args on both `rx150_pick_place_sim.launch.py` and
`rx150_pick_place.launch.py`:

```bash
ros2 launch bcr_arm_rx150 rx150_pick_place_sim.launch.py \
  carry_level:=true grasp_value:=0.25
```

| arg | default | meaning |
| --- | --- | --- |
| `carry_level` | `false` | hold the gripper level while the cup is held |
| `grasp_value` | `0.3` | how far to close on the cup — **placeholder, see §6** |
| `pregrasp_value` | `open` | opening set at the hover point before descending |
| `autostart` | `false` | run the first mission without waiting for a trigger |
| `use_vision_stub` | `true` | run `vision_placeholder` |
| `use_sweep_stub` | `true` | run `sweep_placeholder` |

`grasp_value` and `pregrasp_value` are `0.0` (fully closed) .. `1.0` (fully open), or
the words `open`/`close`. The gripper controller runs in `normalized` units so the same
number means the same openness in sim and on hardware.

Node parameters worth knowing (set in the orchestrator, not exposed as launch args):
`approach_height` (0.10 m hover), `lift_dz` (0.06), `clearance_dz` (0.03),
`phase_delay_sec` (**2.0 — a debugging aid; set to 0.0 for normal running**).

---

## 6. What is still to implement

### Blocking a real physical run

- **Gripper endpoints are unverified.** `_HW_SERVO_CLOSED` (0.6) and `_HW_SERVO_OPEN`
  (1.5) in `rx150_gripper_controller.py` are placeholders in servo radians, flagged as
  "verify/tune on the real arm" in that file's own docstring. `grasp_value`
  interpolates between them, so the grasp is only as trustworthy as those two numbers.
- **`grasp_value` is a guess by construction.** There is no cup in
  `worlds/rx150_obstacles.world` to calibrate against. Tune against the real cup.
- **Vision is a stub.** `vision_placeholder` answers with canned points
  (`cup_xyz`, `goal_xyz`). Real vision is teammate-owned; the contract is one request
  on `/vision/find_request` → one `PointStamped` on `/vision/object_point` in
  `rx150/base_link`. Swap the node, keep the topics — the orchestrator does not care
  who produces the point.
- **The sweep is a stub.** `sweep_placeholder` latches one canned box at
  `[0.34, 0.14, 0.12]`. The cup itself is *not* in the obstacle cloud, so the planner
  cannot currently route around the object it is reaching for.

### Camera / RealSense integration (deferred)

The sim has a gripper-mounted depth camera; hardware does not. Four separate gaps:

1. **No driver in the image** — `ros-humble-realsense2-camera` is not installed by
   `docker/Dockerfile`.
2. **No camera TF on hardware** — the sim gets its camera frame because
   `rx150_gz_classic.launch.py` forwards `external_urdf_loc`. The Interbotix
   `rx150_control.launch.py` used by the hardware path does **not** forward it, so
   `rx150_gripper_depth_camera.urdf.xacro` never reaches the hardware robot
   description and there is no `camera_link` to transform the cloud from.
3. **Extrinsics are unmeasured** — the `xyz="0 0 0.065"` in the sim xacro is a
   placeholder, not a measured mount offset.
4. **Frame-name join** — the RealSense driver publishes its own TF tree; those names
   have to be reconciled with the URDF's.

See [TASK_realsense_sweep_pipeline.md](TASK_realsense_sweep_pipeline.md) and
[TASK_camera_extrinsics_urdf.md](TASK_camera_extrinsics_urdf.md).

Also note a sim-to-real gap: the sim camera's near clip is 0.02 m, but a RealSense
D435's minimum range is ~0.3 m. Close-range geometry that the sim sees will simply be
missing on hardware.

### Smaller known gaps

- **`path:complete` means "arrived", not "stopped"** everywhere except the gripper
  actions, which are explicitly gated on a settle check. Other consumers of that event
  see the arm still decelerating.
- **`carry_waypoint_reached_tolerance` is 0.022 m**, loosened back when level-carry
  also locked yaw. That over-constraint is gone, so it can probably return to 0.015 m.
- **A restart during the post-stop home move abandons that recovery** and plans the new
  mission from wherever the arm is. Correct reading of "restart now", but the arm does
  not pass through home between those two cycles.
- **`phase_delay_sec` defaults to 2.0 s**, a debugging aid that makes every cycle ~20 s
  longer than it needs to be.

### Already done (do not re-plan these)

The whole-body collision work from `WHOLE_BODY_MOTION_PLANNING.md` is implemented:
a shared FK/collision module (`bcr_arm_common/rx150_kinematics.py`, replacing three
drifting copies), per-link capsule checking against the 3D obstacle points, replanning
around body-colliding waypoints, a joint-space RRT-Connect whole-body fallback, and a
`collision` suite in `rx150_target_test_suite.py`.

Note the A* grid itself is still 2D (XY, height-thresholded). That is deliberate, not
an oversight: the *search* is 2D and cheap, and every candidate waypoint is then
validated in 3D with the real solved arm posture via `_first_body_collision_cell`.

---

## 7. Files

| file | role |
| --- | --- |
| `launch/rx150_pick_place_sim.launch.py` | sim mission bring-up |
| `launch/rx150_pick_place.launch.py` | hardware mission bring-up |
| `src/bcr_arm_rx150/rx150_pick_place_orchestrator.py` | the conductor / state machine |
| `src/bcr_arm_rx150/mission_keyboard.py` | keyboard control (own terminal) |
| `src/bcr_arm_rx150/rx150_gripper_controller.py` | gripper, sim + hardware |
| `src/bcr_arm_rx150/vision_placeholder.py` | vision stub — replace |
| `src/bcr_arm_rx150/sweep_placeholder.py` | sweep stub — replace |
