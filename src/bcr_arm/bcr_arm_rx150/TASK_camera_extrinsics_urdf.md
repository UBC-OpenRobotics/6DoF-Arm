# Task: Camera Extrinsics + URDF (Physical RX-150 + RealSense)

**Owner:** _(assignee)_
**From:** Mudasser (team lead)
**Pairs with:** [TASK_realsense_sweep_pipeline.md](TASK_realsense_sweep_pipeline.md) — worth agreeing on the camera **frame name** early (see "Handoff").

## Context — what this task is, in plain terms

The arm has a depth camera mounted near the gripper. That camera produces a **point cloud** —
a set of 3D points describing whatever is in front of it (a box, a cup, the table). For the
robot to avoid those obstacles or reach for that cup, it has to know **where each of those
points is relative to its own base**, not just relative to the camera.

Here's the catch: the camera reports points in **its own coordinate frame**. To make them
useful, the software converts them into the arm's base frame. That conversion depends entirely
on knowing **exactly where the camera is bolted onto the arm** — its position and its tilt.
Capturing that mounting pose accurately is this whole task. It's a small change (a few
numbers), but it's the foundation everything else sits on.

## A little background on the moving parts

If you're new to ROS/TF, this is the mental model you need:

- **Frame & transform.** Every rigid part of the robot (the base, each arm link, the camera)
  has its own coordinate frame — an origin and an XYZ orientation. A **transform** is just the
  position + rotation that relates one frame to another ("the camera sits 6 cm above the
  gripper, rotated like *this*").
- **TF.** ROS runs a live system called **TF** that keeps a tree of all these frames and the
  transforms between them, and publishes it continuously. Because of TF, any point measured in
  one frame (the camera's) can be re-expressed in another (the base). The arm's root frame is
  **`base_link`**.
- **URDF / xacro.** The robot's physical description lives in a **URDF** file (ours is a
  `.xacro`, a templated URDF). It defines **links** (rigid parts) and **joints** (how they
  connect), *including where the camera is mounted*. A node called `robot_state_publisher` reads
  this file and publishes the TF tree from it. The **same file is used in sim and on the real
  arm**.
- **Extrinsics vs. intrinsics.** The camera's pose relative to what it's bolted to is called
  its **extrinsics** (the mounting transform) — that's what you're setting. Not to be confused
  with **intrinsics** (the lens/focal calibration), which the RealSense handles internally and
  is not part of this task.

Put together, the relevant transform chain is:

```
base_link → ... → gripper_link → camera_link → camera_optical_frame
                              └── you set this piece ──┘   └── the RealSense driver
                                  (the mount pose)             publishes this piece
```

Your job is the `gripper_link → camera_link` piece — the mount pose — in the URDF. TF stitches
the whole chain together automatically once that's right.

## Why it's the highest-leverage number in the stack

Everything the robot "sees" gets transformed into `base_link` using that chain. If the URDF
says the camera is 2 cm over and 5° rotated from where it physically is, **the entire obstacle
map comes out shifted and rotated** — and the planner, the RRT fallback, and the grasp logic
all inherit that error, with no way to recover downstream. So this is small in code but large
in impact, which is why it's worth taking the time to measure carefully.

## The one place the numbers go

Good news: the camera is already in the URDF — no new file to create. It's the same file sim
uses. Open:

- **[urdf/rx150_gripper_depth_camera.urdf.xacro](urdf/rx150_gripper_depth_camera.urdf.xacro)**

The mount pose is the `origin` of the `gripper_camera_mount` joint (around **line 17**):

```xml
<joint name="..._gripper_camera_mount" type="fixed">
  <parent link=".../gripper_link"/>
  <child  link=".../gripper_camera_link"/>
  <origin xyz="0.0 0.0 0.065" rpy="0 0 0"/>   <!-- ← the mount pose -->
</joint>
```

- `xyz` = position (meters) of the camera relative to `gripper_link`.
- `rpy` = orientation (radians: roll, pitch, yaw).

The current `0.0 0.0 0.065` is an idealized value used for sim. The task is to replace it with
the camera's real measured pose on the physical arm.

## Suggested path

1. **Confirm where it bolts on.** The camera is mounted on `gripper_link` — worth confirming
   that's still where it physically attaches on the real arm; if it moved, update the
   `parent link`.
2. **Find the camera's reference point.** The RealSense datasheet shows where its `camera_link`
   origin sits on the physical device (relative to the mounting screws / left imager). You're
   measuring *to that point*, so it helps to know where it is.
3. **Measure `xyz` + `rpy`** from `gripper_link` to the camera's `camera_link` — calipers/ruler
   for position, and work out the orientation from how it's seated on the mount. Hand
   measurement is a perfectly good starting point.
4. **Enter the numbers on the mount joint.** That's the core deliverable.
5. **Connect our camera link to the RealSense driver's frames.** The `realsense2_camera` driver
   publishes its own little frame tree rooted at `camera_link`. Our URDF link needs to line up
   with that — either name our link `camera_link`, or add a fixed joint from
   `gripper_camera_link` → the driver's `camera_link`. This is the one spot to sync frame names
   with the sweep-pipeline owner (Handoff below).
6. **(If needed) refine with hand-eye calibration.** If the map looks tilted/offset after hand
   measuring, a standard hand-eye calibration routine solves the extrinsics precisely. Only
   reach for this if hand measurement isn't accurate enough.

## Leave these alone

- **The optical joint** (around **line 49**, `rpy="${-pi/2} 0 ${-pi/2}"`) is the *standard ROS
  optical-frame convention* — a fixed convention, not a measurement. No need to touch it.
- **The `<gazebo>` block** (~lines 54–80) is the **sim-only** fake depth sensor; the real robot
  ignores it and the RealSense driver replaces it. Safe to leave as-is (or strip it in a
  physical-only variant if you prefer it clean).

## Files worth reading

- The xacro above — it's short, worth reading top to bottom.
- The base Interbotix arm URDF (`interbotix_xsarm_descriptions/urdf/rx150.urdf.xacro`) — to see
  the link chain `gripper_link` hangs off of.
- The RealSense model's datasheet — for where `camera_link` sits on the device.
- Interbotix's `interbotix_xsarm_perception` — a good reference for mounting a RealSense on
  these arms (naming + calibration patterns worth cribbing).

## Definition of done

- `ros2 run tf2_ros tf2_echo rx150/base_link camera_depth_optical_frame` prints a transform
  that matches a tape-measure sanity check.
- In RViz, put a known object in front of the camera and confirm the cloud lands where the
  object physically is (not shifted or rotated).
- **Heads-up:** this same file drives sim, so after you set the real value, re-run the sim
  sweep once and confirm coverage still looks good — the old value was tuned for the sim
  camera's field of view.

## Handoff / let's sync

- The **camera frame name** is the one interface between you and the sweep-pipeline owner:
  whatever you name the camera link has to match what they point the relay at and what the
  driver publishes. Worth agreeing on before either of you goes deep.
- Ping me once you have measured numbers in — happy to eyeball the extrinsic together before we
  trust the map.
