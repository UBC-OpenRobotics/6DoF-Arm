# Task: RealSense Sweep Pipeline (Physical RX-150)

## Context — building on what you already have

You already have a good chunk of this working: a camera pipeline that reads a sweep and
processes a point cloud. The two gaps are that it ran from a **pre-recorded** capture rather
than live, and the cloud stayed in **`camera_link`** — never transformed into `base_link`.
That `camera_link → base_link` transform is exactly the piece you asked me about.

Since then I've built out the whole downstream: the transform, the sweep/accumulate step, and
the obstacle planner + RRT-Connect whole-body fallback that consumes the map. So rather than
bolt the transform onto your pipeline, I think the cleaner path is the reverse — **use this
baseline as the backbone, and bring your camera setup/reading logic into it, switched from
pre-recorded playback to live.** Your work slots into the front of the pipeline; everything
after it (the transform you needed, plus planning) is already done.

That's a suggestion, not a mandate — if you see a reason your structure fits better, let's talk through it.
## The goal

Get **live RealSense data** flowing through this pipeline so a sweep produces a usable obstacle
map in `base_link`. Done when `/planning/point_cloud` has a good map and the existing planner
can plan against it.

## How your existing work maps in

| What you have | Where it goes here |
|---|---|
| Camera setup + reading a point cloud | The **live camera source** at the front (replaces the sim's Gazebo camera **and** your pre-recorded playback) |
| Cloud stuck in `camera_link` | Solved — the **relay** transforms `camera_link → base_link` for you (the piece you were missing) |
| — | Everything downstream (sweep, planner, RRT) is reused as-is |

## Pipeline: sim today → physical target

The structure is identical; only the **front** changes. `═══` marks a component that carries
over; `REPLACE` marks the one that swaps.

```
   SIM TODAY                                     PHYSICAL (target)
   ───────────────────────────────────          ───────────────────────────────────────────
   Gazebo depth-camera plugin                    RealSense — LIVE                    REPLACE
   /gripper_camera/points (camera frame)         /camera/.../points (camera frame)
   (in sim, a fake sensor)                       (your camera-read logic, made live —
        │                                         no more pre-recorded playback)
        │                                              │
        ▼                                              ▼
   scene_point_cloud (RELAY)          ═══════════  scene_point_cloud (RELAY)          REUSE
   transforms camera→base_link (TF)                same node, pointed at the real
   /planning/live_point_cloud (base_link)          camera frame (config only)
        │                                              │   ← this is the camera_link→base_link
        ▼                                              ▼      transform you were missing
   scene_sweep_mapper                 ═══════════  scene_sweep_mapper                 REUSE
   accumulates → /planning/point_cloud             (param tuning only)
        │                                              │
        ▼                                              ▼
   planner → RRT → executors → arm    ═══════════  planner → RRT → executors → arm    REUSE
```

So the only genuinely new piece is the **live camera source** — which is where your existing
code lives. The relay (with the transform), sweeper, and planner all carry over.

## Suggested path

1. **Get a live cloud publishing.** Either stand up the stock `realsense2_camera` driver
   (pointcloud enabled), or adapt your existing camera-read code to publish **live** instead of
   from the recording. Either way the interface is the same: a live `PointCloud2` in the
   camera's optical frame on a known topic. Note the topic and the optical-frame name.
2. **Point the relay at it.** `scene_point_cloud.py` already does the transform — it just needs
   configuring: `input_topic` → your live cloud, `target_frame=rx150/base_link`, TF mode (leave
   `source_frame` empty so it reads the frame from the cloud header; keep `broadcast_static_tf=false`
   since the URDF provides the TF). Tune `max_range` for the sensor.
3. **Add the camera + relay to the physical launch.** The physical stack
   ([launch/rx150_dls_stack.launch.py](launch/rx150_dls_stack.launch.py)) has the relay slot
   wired but no driver yet — the sim launch's camera section is a good template.
4. **Filter/downsample the real cloud.** Real depth is noisier and denser than sim (flying
   pixels, edge noise, dropouts; hundreds of k points at 30 Hz). Some outlier removal +
   downsampling — in the relay or a small filter node — will likely matter for map quality.
   (If your existing processing already does some of this, that's a natural thing to bring in.)
5. **Tune the sweeper params** (no logic changes): bounds `x/y/z_min/max`, `voxel_size`,
   `max_input_range`, `settle_sec`/`sample_sec`. Keep `world_frame`/`frame_id` = `rx150/base_link`.

## Things worth watching

- **Min depth vs. scan standoff.** The scan posture keeps the camera close (~0.09 m). Some
  RealSense models (e.g. D435) can't focus closer than ~0.2–0.3 m; a D405 is fine up close. If
  the model can't see close in, we may need a different standoff / scan posture
  (`SCAN_TUCKED_JOINTS` / `SCAN_WAIST_ANGLES`) — flag it early and we'll sort it out.
- **Sampling while stationary.** The relay uses "latest" TF, which stays accurate because the
  sweep settles and samples while the arm is still. Good to preserve that when going live.

## Files to touch / read

| File | Note |
|------|------|
| [launch/rx150_dls_stack.launch.py](launch/rx150_dls_stack.launch.py) | Add the live camera source + relay (mirror the sim launch's camera section). |
| `data_collector/.../scene_point_cloud.py` | The **relay** — where the transform lives (`_lookup_transform` / `_point_cloud_cb`). Config for TF mode + real topic; good place for filtering. |
| `data_collector/.../scene_sweep_mapper.py` | The **sweep** (tuck → sweep → accumulate → crop → voxel → publish). Params only. |
| [launch/rx150_dls_sim_stack.launch.py](launch/rx150_dls_sim_stack.launch.py) | Reference wiring for relay + sweep. |
| [SIM_COMMANDS.md](SIM_COMMANDS.md) | How the sweep runs today. |
| `realsense2_camera` docs, Interbotix `interbotix_xsarm_perception` | Driver + RealSense-on-arm patterns. |

## Definition of done

- `/planning/live_point_cloud` shows the live RealSense cloud correctly placed in `base_link`
  in RViz (put a known object in front; it should land where it physically is).
- A sweep completes and populates `/planning/point_cloud` with a clean merged map.
- The existing planner plans against that real map end to end.

## Handoff / let's sync

- The **camera frame name** is the one interface between you and the extrinsics/URDF work — the
  relay's `target_frame` and the frame your driver publishes need to line up with the camera
  link in the URDF. Worth agreeing on before either of you goes deep.
- The map is only as good as the extrinsic calibration — if the cloud looks placed wrong, it
  might be the URDF numbers rather than your pipeline, we can debug together. I will likely assign this calibration to someone next meeting who is available in person.
- Ping me once `/planning/live_point_cloud` looks right in RViz — that's the natural checkpoint
  before a full sweep, and a good moment to compare notes on bringing your logic in.
