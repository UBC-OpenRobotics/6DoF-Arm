# Real Hardware TF Frame Check — Testing Checklist

**Goal:** confirm the real frame names being published on real hardware
(robot + RealSense), so we can:
- set the correct `target_frame` parameter on the vision nodes
  (`mapping_node`, `localization_3d_node`), and
- confirm whether the `camera_base_link` ↔ camera-driver static transform
  publisher is still needed, and with which frame names.

Run this **once with both the robot description and the RealSense driver
up together** — checking them separately won't answer the question, since
the robot's URDF-driven frames and the RealSense driver's own frames are
two independent TF sources that only connect if something bridges them.

---

## 1. Launch everything together

```bash
# Terminal 1: Motion/Robot description (adjust to however the real robot is launched)

# Terminal 2: Vision (realsense driver included)
ros2 launch arm_perception perception.launch.py
```

- [ ] Both processes are up with no errors in either terminal.

---

## 2. Confirm the point cloud's actual frame_id

```bash
ros2 topic echo camera/camera/depth/color/points --once --field header.frame_id
```
(swap in the real topic name from step 2 if different)

- [ ] Record the frame_id here: `____________________________`

---

## 3. Confirm the robot's camera-mount frame name

Check whatever the robot's URDF actually declares for the camera mount —
this may be `gripper_camera_link` or `gripper_camera_optical_link`
(per the `gripper_camera` xacro) — **confirm which is real**:

```bash
ros2 param get /robot_state_publisher robot_description | grep -i camera
```

- [ ] Record the camera-mount link name(s) found here:
      `____________________________`
- [ ] Record whether it's namespaced (e.g. `rx150/gripper_camera_link`)
      or not: `____________________________`

---

## 5. Check whether the two trees are connected

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF (`frames_<timestamp>.pdf` in the current directory)
showing the full TF tree.

- [ ] Open the PDF. Is there **one single connected tree** from the
      robot's base frame all the way to the camera's optical frame?
- [ ] Or are there **two disconnected trees/islands** — one ending at the
      robot's camera-mount frame (from step 4), and a separate one rooted
      at the driver's own frame (from step 3)?

**If two islands:** a static transform publisher bridging the two frame
names is still required — record the exact frame names from steps 3 and 4
above; those are the two arguments it needs.

**If one connected tree:** no bridge needed — something already connects
them (worth noting how, for future reference).

---

## 6. Sanity-check with a live echo

```bash
ros2 run tf2_ros tf2_echo <robot_base_frame> <camera_optical_frame_from_step_3>
```

- [ ] Does this resolve successfully (prints a translation/rotation), or
      does it time out / error?
- [ ] If it resolves: does the translation look physically reasonable
      (roughly matches where the camera is actually mounted), or does it
      look like a placeholder (e.g. exactly `0 0 0`)?

---

## 7. Report back

Please share:
1. The completed blanks above (topic name, frame_id, camera-mount link
   name, namespace).
2. The `view_frames` PDF.
3. Whether step 6 resolved successfully or not.

This is everything needed to set the right `target_frame` param on the
vision nodes and confirm the static transform publisher's final
arguments.
