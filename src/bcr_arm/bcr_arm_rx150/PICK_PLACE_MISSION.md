# RX-150 Pick-and-Place Mission

End-to-end cup pick-and-place, in one launch, in **sim or on the physical arm**. The
mission is repeatable and keyboard-driven: it idles, runs a full cycle on request,
then returns to idle ready to go again — no relaunching.

This document is the single reference for the mission. For the lower-level manual
workflows (send one Cartesian target, run the sweep by hand, pose the arm) see
[SIM_COMMANDS.md](SIM_COMMANDS.md) and [HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md).

---

## 1. Run it in simulation

Two terminals.

**Terminal 1 — the mission stack:**

```bash
# run from src/bcr_arm/ in your checkout
xhost +local:docker

docker compose run --rm --name rx150 --service-ports rx150-sim bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place_sim.launch.py carry_level:=true"
```

Wait for the build and Gazebo. Nothing moves until:

```
Idle. Waiting for mission start on /mission/start ...
```

### Flags worth knowing

Append these to the `ros2 launch` line. `--show-args` lists them all; `use_sim_time` is
handled for you.

| flag | default | use it to |
| --- | --- | --- |
| `carry_level:=true` | `false` | keep the gripper level so a held cup stays upright |
| `grasp_value:=0.55` | `0.60` | **the main tuning dial** — how far to close on the cup |
| `grasp_z_offset:=0.025` | `0.018` | move the grab point up or down the cup's body |
| `surface_to_centre_m:=0.021` | `0.021` | half the object's depth — pushes the detected point from its near face to its axis |
| `autostart:=true` | `false` | run immediately, no keyboard |
| `use_vision_stub:=true` | `false` | canned cup/goal points — exercise the mission without detection |
| `phase_delay_sec:=0.0` | `2.0` | drop the debugging pause between phases (~20 s/cycle) |
| `cup_classes:='[cup]'` | `[cup, frisbee, bowl, toilet]` | narrow the accepted labels — **priority order, see §7** |
| `observe_joints:='[0.0,-1.65,1.07,1.25,0.0]'` | off (`[]`) | add a fixed look-down pose; rarely needed — the sweep finds the cup |

**Terminal 2 — keyboard control:**

```bash
docker exec -it rx150 bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

Press **`s` once**, then leave it. Phase 1 is a **~2 minute sweep**, so a long silence is
normal — `r` means *stop and start over*, and will cancel the run you just started.

### If something goes wrong

| symptom | cause |
| --- | --- |
| `MISSION ABORTED: sweep did not complete`, no cloud | `xhost +local:docker` not run — the depth camera renders through X and silently publishes nothing |
| `START -> ... NOTHING IS SUBSCRIBED` | Terminal 1 is not up yet, or Terminal 2 is in a different container |
| `mission_keyboard: executable not found` | Terminal 1's build has not finished |
| `compose exec` cannot find the service | use `docker exec -it rx150` — `compose run` names its container `bcr_arm-rx150-sim-run-<hash>` unless you pass `--name` |
| detector finds nothing, node looks healthy | clock mismatch — check for `Clock check OK: use_sim_time=True, /clock present.` |

Healthy camera rates are ~5 Hz on `/gripper_camera/points` and ~4 Hz on
`/planning/live_point_cloud` (`ros2 topic hz`).

---

## 2. Run it on the physical arm

Same shape as sim. The arm must be powered and connected first.

⚠️ **The sweep physically moves the arm** through eight scan poses. Clear the workspace.

```bash
# Terminal 1
# run from src/bcr_arm/ in your checkout

docker compose run --rm --name rx150 --service-ports rx150-hardware bash -lc \
  "bash /workspaces/bcr_arm/docker/setup_workspace.sh && set +u && \
   source /workspaces/bcr_arm/install/setup.bash && \
   ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py carry_level:=true"

# Terminal 2
docker exec -it rx150 bash -lc \
  "source /workspaces/bcr_arm/install/setup.bash && \
   ros2 run bcr_arm_rx150 mission_keyboard"
```

Every flag from §1 applies. Three more matter on the first hardware run:

| flag | default | use it to |
| --- | --- | --- |
| `goal_fallback_xyz:='[x, y, z]'` | `[0.20, 0.22, 0.06]` | **set this every time** — the drop-off point is a fixed guess, not something the camera found |
| `min_depth_m:=0.2` | `0.2` | already raised for a D435 (cannot focus closer). Lower it for a D405 |
| `cup_classes:='[cup]'` | `[cup]` | change if the detector labels your cup something else (`ros2 topic echo /perception/detections`) |

Both stubs default off, so this runs the real pipeline. To exercise mission logic with
no camera and no physical scan:

```bash
ros2 launch bcr_arm_rx150 rx150_pick_place.launch.py \
  carry_level:=true use_sweep_stub:=true use_vision_stub:=true
```

**Before the first real grasp**, read §6 — the gripper endpoints are unverified
placeholders and the goal is a fixed point.

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
| 1 | **Sweep** — physically scan the scene, and wait for it to finish |
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

- **The approach is hover-then-descend, not straight in** — and the hover sits *behind*
  the object as well as above it, so the gripper's rear bracket does not strike the cup
  on the way down (`approach_back_off`).
- **Level-carry goes on at the hover, not before the approach.** The gripper is empty
  until the grasp, so constraining the transit only costs reach. See §7.
- **The gripper only actuates when stationary.** `path:complete` means "arrived", not
  "stopped", so the grasp waits on a settle check rather than on that event alone.


### Event contract

Motion nodes emit lifecycle events on `/motion/status` (`std_msgs/String`); the
orchestrator waits for a terminal one before advancing.

- motion success: `path:complete`, `joint:complete`
- motion failure: `path:aborted`, `joint:aborted`, `planner:no_path`, `ik:tilt_exceeded`
- sweep success: `sweep:complete`
- sweep failure: `sweep:failed` (scan could not run), `sweep:aborted` (operator stop)

Sweep tokens are a separate set from motion tokens, so neither can satisfy the
other's wait.

`move_timeout_sec` (60 s) is a backstop for a lost event, not the primary signal.

### How stop actually stops

`/mission/stop` unwinds the orchestrator's phase sequence *and* broadcasts
`std_msgs/Empty` on **`/motion/cancel`**, which the Cartesian executor, the joint
executor and the DLS IK executor all subscribe to. The executors drop their remaining
waypoints; the IK node forgets its servo target (without that it would keep driving to
the last waypoint). The arm halts where it is, then the orchestrator parks it home.

Every cycle re-runs the sweep, and phase 1 blocks until it is genuinely finished —
it waits for **`sweep:complete`**, which the sweep node emits only after that cycle's
merged map is on the wire.

Waiting on cloud *arrival* instead would be wrong, and subtly so: a sweep node
republishes its previous map every 2 s, so from the second cycle onward that test
would pass ~2 s after the request — while the arm is still physically scanning — and
the mission would plan against the **previous** cycle's map with two things
commanding the arm at once. Measured: cycle 2's phase 1 takes ~42 s (a real scan),
not ~2 s.

---

## 5. Tuning

All are launch args on both `rx150_pick_place_sim.launch.py` and
`rx150_pick_place.launch.py`:

```bash
ros2 launch bcr_arm_rx150 rx150_pick_place_sim.launch.py \
  carry_level:=true grasp_value:=0.25
```

| arg | sim default | hw default | meaning |
| --- | --- | --- | --- |
| `carry_level` | `false` | `false` | hold the gripper level while the cup is held |
| `grasp_value` | `0.60` | `0.3` | how far to close on the cup — **the main tuning dial, see §6** |
| `grasp_z_offset` | `0.018` | `0.018` | added to the detected z; moves the grab point up the cup's body |
| `surface_to_centre_m` | `0.021` | `0.021` | near face → object axis, applied horizontally |
| `bbox_anchor` | `bottom` | `bottom` | which edge of the detection box to sample depth at |
| `pregrasp_value` | `open` | `open` | opening set at the hover before descending |
| `autostart` | `false` | `false` | run the first mission without waiting for a trigger |
| `use_vision_stub` | `false` | `false` | canned points instead of real detection |
| `use_sweep_stub` | — | `false` | canned cloud instead of a physical sweep; not an arg in sim |
| `cup_classes` | `[cup, frisbee, bowl, toilet]` | `[cup]` | detector labels accepted as "the cup" — **priority order** |
| `yolo_confidence` | `0.15` | `0.5` | low in sim on purpose; see §7 |
| `goal_fallback_xyz` | `[0.20, 0.18, 0.16]` | `[0.20, 0.22, 0.06]` | **placeholder** drop-off — the detector has no goal class |
| `min_depth_m` | `0.2` | `0.2` | depths below this are invalid; a D435 cannot focus closer |
| `detection_ttl_sec` | `300.0` | `300.0` | a sighting older than this is not an answer — **must exceed one sweep** |
| `approach_height` | `0.15` | `0.15` | hover this far above a grasp point — **hard geometric floor, see below** |
| `approach_back_off` | `0.045` | `0.045` | how far *behind* the object the hover sits, so the gripper descends diagonally |
| `scan_posture` | `tilted` | `tilted` | tucked posture the sweep scans from: `tilted` or `level` |
| `sweep_timeout_sec` | `240.0` | `240.0` | deadline for a hung sweep; a two-look sim scan measures ~123 s |

**Not** a launch arg: `SCAN_WRIST_OFFSETS` (`[0.0, 0.15]`) is a module constant in
`scene_sweep_mapper.py` — the looks taken at each waist station. Edit it there;
`[0.0]` restores the old single-look sweep.

| `observe_joints` | `[]` (off) | `[]` (off) | optional fixed look-down pose; the sweep finds the cup without it |
| `phase_delay_sec` | `2.0` | `2.0` | pause between phases; a debugging aid — set `0.0` for normal running |
| `check_grasp` | `true` | `true` | abort if the fingers close on empty air |

> Both stubs now default to `false` on both launches, and both real sources are
> wired, so the default command runs the real pipeline end to end. The stubs remain
> as deliberate fallbacks, not as the happy path.

`grasp_value` and `pregrasp_value` are `0.0` (fully closed) .. `1.0` (fully open), or
the words `open`/`close`. The gripper controller runs in `normalized` units so the same
number means the same openness in sim and on hardware.

### `approach_height` has a hard geometric floor

The planner checks the whole arm with a capsule model, and the gripper link's capsule
is `LINK_CAPSULE_RADII[-1]` (0.07) + `CAPSULE_SAFETY_MARGIN` (0.015) = **0.085 m**.
The object being grasped is excluded from collision checks only within
`grasp_clearance_radius` of *the goal* — and the hover point is a different, higher
goal, so while flying to the hover the object is a **hard obstacle**. For an object of
height `h` grasped at its centre:

```
approach_height  >  0.085 + h/2
```

The old default of `0.10` could not clear anything taller than ~30 mm, and failed on a
100 mm cup with `Body collision on link segment 4 at waypoint [...]` followed by RRT
`start_in_collision`. That pair of messages reads like "the target is unreachable" but
actually means **the hover point is inside the object**. Default is now `0.15`.

Other node parameters worth knowing (set in the orchestrator): `lift_dz` (0.06),
`clearance_dz` (0.03), `phase_delay_sec` (**2.0 — a debugging aid; set to 0.0 for
normal running**), `grasp_clearance_radius` on the planner (0.08).

---

## 6. What is still to implement

> **The full backlog lives in [REMAINING_WORK.md](REMAINING_WORK.md)** — every
> outstanding task, why it matters, and which file it lives in. This section keeps
> only the notes that need the surrounding mission context.

### Blocking a real physical run

- **Gripper endpoints are unverified.** `_HW_SERVO_CLOSED` (0.6) and `_HW_SERVO_OPEN`
  (1.5) in `rx150_gripper_controller.py` are placeholders in servo radians, flagged as
  "verify/tune on the real arm" in that file's own docstring. `grasp_value`
  interpolates between them, so the grasp is only as trustworthy as those two numbers.
- **`grasp_value` is still a guess for the *real* cup.** `worlds/rx150_obstacles.world`
  now contains a small Solo cup standing on the floor at `[0.30, 0.0]` — 0.056 m at
  the rim tapering to 0.040 m, 0.075 m tall, sized to the gripper's 0.030–0.074 m
  finger gap. So sim can be calibrated against something, but the sim gripper and a
  real one do not share a closing curve. Tune against the physical cup.
- **The cup is now detected, not canned.** This is wired and is the default on both
  launches. See §7 for the pipeline and its remaining rough edges.
- **Sim sweeps for real; hardware does not.** In sim there is no canned-cloud
  option any more: `rx150_pick_place_sim.launch.py` always runs
  `data_collector scene_sweep_mapper` in triggered mode, phase 1 fires
  `/sweep/start`, the arm tucks and scans 8 waist angles, 2 looks each (~123 s), and the mission
  waits for `sweep:complete` before planning anything.

  **Hardware now sweeps for real too.** `rx150_pick_place.launch.py` runs the same
  `scene_sweep_mapper` in triggered mode whenever `use_sweep_stub:=false` (the
  default). Its input is `/planning/live_point_cloud`, produced by the
  `scene_point_cloud` relay already in `rx150_dls_stack.launch.py` — that relay is
  what applies the camera → `base_link` transform. So the chain is:

  ```
  RealSense → scene_point_cloud (relay) → scene_sweep_mapper → /planning/point_cloud
  ```

  `sweep_placeholder` is still there behind `use_sweep_stub:=true`: it answers
  instantly with a canned box at `[0.34, 0.14, 0.12]` and **no arm motion**, which is
  the right choice when you want to exercise mission logic without driving the arm
  through eight scan poses. It emits the same `sweep:complete`, so phase 1 is
  identical either way — the map is just fake.

  > **Never run a second sweep node by hand.** Two nodes publishing
  > `/planning/point_cloud` both republish every 2 s and overwrite each other:
  > RViz flickers between the maps, and the planner's obstacle map alternates,
  > making whether a path is found depend on which cloud landed last.

- **The cup is not in the obstacle cloud** under either sweep, so the planner cannot
  route around the object it is reaching for.

### Camera / RealSense integration (deferred)

The sim has a gripper-mounted depth camera; hardware does not. The driver is now
**installed** — `docker/Dockerfile` pulls `ros-humble-realsense2-camera`,
`ros-humble-realsense2-description` and `ultralytics` — so the remaining gaps are
about frames, not packages:

1. **No sweep node on the hardware launch** — `scene_sweep_mapper` is only wired into
   the sim launch. Hardware needs it added (and a camera feeding
   `/planning/live_point_cloud`) before `use_sweep_stub:=false` does anything useful.
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

## 7. The vision pipeline

The mission does not hardcode where the cup is. It asks, and `arm_perception`
answers from the camera.

```
camera ── color ──> color_preprocessing ──> yolo_detector ──> /perception/detections
   │                                                                    │
   └── depth + camera_info ─────────────────> localization_3d_node <─────┘
                                                      │  (deproject + TF to base_link)
                                                      ▼
                                          /perception/detections_3d
                                                      │
   orchestrator ──/vision/find_request──>       vision_bridge
   ("cup" | "goal")                                   │
   orchestrator <──/vision/object_point──────────────-┘
                   (PointStamped, rx150/base_link)
```

The same nodes serve sim and hardware; only the three camera topics differ, which is
why `perception.launch.py` takes them as arguments:

| | sim | hardware |
| --- | --- | --- |
| color | `/gripper_camera/image_raw` | `/camera/camera/color/image_raw` |
| depth | `/gripper_camera/depth/image_raw` (32FC1, metres) | `/camera/camera/aligned_depth_to_color/image_raw` (16UC1, mm) |
| info | `/gripper_camera/camera_info` | `/camera/camera/color/camera_info` |

### Two properties that keep this safe on a real arm

**Silence means "not found".** If nothing matching is detected, `vision_bridge`
publishes *nothing*; the orchestrator times out and aborts. It never answers a cup
request with a guess. (The earlier version published a default-constructed
`PointStamped` on failure — `(0,0,0)`, which the orchestrator accepts as a valid target
and drives the arm into **its own base**.) There is deliberately no `cup_fallback_xyz`.

**Detections expire** after `detection_ttl_sec` (**300 s**). The camera is on the
gripper, so the cup is seen *during the sweep* and answered from cache a phase later.
The TTL stops that cache answering from a cup carried off minutes ago — but it **must
exceed one full sweep** (~123 s in sim, slower on real servos). At the old 45 s every
sighting from the first two thirds of the sweep expired before the mission asked, which
surfaces as "vision returned no cup" and looks nothing like a timing problem.

### `cup_classes` is a priority order, not a set

The list is tried in order; the first class with a fresh detection wins, and confidence
only breaks ties *within* a class. Ranking by confidence across the whole list is
actively dangerous: in sim the blue obstacle cylinder scores `vase` **0.49** while the
real cup scores `cup` **0.06**, so a max-confidence rule answered a cup request with
the *obstacle's* position and sent the arm to grasp it. Only add aliases the target
object itself produces — never ones other scene objects produce.

### The obstacle map is latched state, not a sensor stream

Every publisher on `/planning/point_cloud` (`scene_sweep_mapper`,
`sweep_placeholder`, `arm_perception`'s `mapping_node`) offers **RELIABLE +
TRANSIENT_LOCAL**, and the planner now *requests* the same. Keep it that way when
adding a publisher — a `VOLATILE` one will not connect to the planner at all, silently.

This was a live bug. The planner subscribed with `qos_profile_sensor_data`
(`BEST_EFFORT` + `VOLATILE`), which meant a ~700 kB merged cloud could simply be
dropped: the sweep logged `Published map with 57905 points`, the orchestrator logged
`Obstacle map present (57905 points)`, and four seconds later the planner said
**`No planning point cloud yet; cannot plan path`** and the mission aborted. It was
intermittent until running YOLO on the CPU alongside Gazebo made the drops frequent.
A dropped camera frame is fine; a dropped map is not.

### The scan: 8 waist stations, 2 looks each

The sweep tucks the arm and rotates the waist through 8 angles, taking **two looks at
each** — one at the posture's own wrist angle, one with the wrist nudged 0.15 rad
(8.6°) further down — then straightens the wrist before slewing to the next station.

**Why two looks.** One posture has one fixed 42.3° vertical band, and `tilted`'s band
reaches the floor no closer than 0.335 m. Everything lying on the ground inside that
radius was invisible to the scan — which is most of the workspace, since the cup lives
at 0.20–0.45 m. The sweep builds the **obstacle map**, so a low object near the base
was simply absent from it and the planner would route straight through it. Measured
azimuthal coverage of a ring of points:

| | cup top R=0.30 | cup base R=0.30 | cup top R=0.22 | floor R=0.45 |
| --- | --- | --- | --- | --- |
| one look | 100% | **0%** | **0%** | 100% |
| two looks | 100% | **100%** | **100%** | 100% |

This coverage now serves **both** jobs. It was built for the map, but the mission also
**locates the cup from the sweep**: the detector runs throughout, every sighting is
kept, and phase 2 no longer clears them. So this table is also the answer to "where can
the cup be put and still be found" — a full circle of waist stations, rather than the
single ~55° wedge a fixed observation pose can see.

The offset is small because there is little room: `wrist_angle` stops at 2.147 rad and
`tilted` already sits at 1.95. It is enough — it pulls the near floor edge from 0.335 m
in to 0.241 m, and the two bands overlap by 31°, so nothing falls between them. Offsets
are clamped to the servo limit rather than commanded past it, because a joint that
never reports arrival is a sweep that hangs on settle.

**Measured, sim, same scene back to back:**

| | map points | unexplained points | outcome |
| --- | --- | --- | --- |
| one look | 899 | — | reached grasp |
| two looks, old sampling | 1505 | — | reached grasp |
| two looks, reliable sampling | 5762 | ~4100 (smear) | **planner refused** |
| two looks + drain (current) | **1656** | **0** | reached grasp |

The final map accounts for every obstacle in the world (357–428 points each, where the
rear stations previously contributed nothing) and now contains the cup itself (72
points) — a 0.075 m object at R=0.30 that one look could not see at all. Hover
clearance to the nearest obstacle point is 0.110 m against a 0.070 m gripper capsule.

The middle two rows are the interesting ones and are described under *stale frames*
below: capturing frames reliably made a **pre-existing** transform bug visible, and it
had to be fixed before the extra looks were usable.

The wrist straightens between stations (`scan_return_to_neutral`) so every station is
entered from the same configuration. That keeps the DLS solver on one solution branch
for the whole scan — the same reason the initial fold is its own move — and avoids
slewing the waist with the wrist parked near its stop.

**The waist angles were left alone.** Against the corrected D435i horizontal FOV of
54.5° (not the 62° the code used to assume) eight stations already give 100% azimuthal
coverage with margin: a cup at R=0.30 stays inside the central **70%** of the frame at
some station, everywhere on the circle. A ninth station only starts to matter if you
demand the central 50%, and respacing the eight evenly is strictly *worse* — 50.8°
steps leave a 3.2° hole. Coverage was never the horizontal problem.

### Stale frames: why more looks briefly made things worse — now fixed

`scene_point_cloud` used to transform each cloud with the **latest** TF rather than the
one matching when the cloud was captured. It had to: nothing in the stack set
`use_sim_time`, so Gazebo stamps were sim time while TF ran on wall time, and a stamped
lookup failed outright.

The cost was that a cloud captured while the waist was still turning, but delivered
after it stopped, merged at the **wrong bearing**. Measured: an obstacle at azimuth +22°
landed in the map at −3°, a 26° smear. That phantom geometry left the hover point
0.048 m from a "surface" — inside the 0.070 m gripper capsule — so the planner refused
every approach (`RRT-Connect ... goals=0, result=no_goal_config`).

The bug predated the extra looks. It was invisible because the old sampling window
caught 0–1 frames per station by luck; capturing frames reliably also captured the bad
ones.

**All three fixes are in:**

* `use_sim_time` is set consistently across every node, so **stamped lookups now work** —
  this is the real fix, and it retired the latest-TF fallback. `allow_latest_tf` is
  `false` in both sim and hardware launches; using a stale arm pose is now a silent
  mis-placement rather than a workaround.
* `scene_point_cloud` subscribes at **depth 1** instead of the depth-5
  `qos_profile_sensor_data`, so at most one stale frame can queue.
* each look **drains** briefly after the move before keeping anything.

Together these took the map from 5762 points with ~4100 phantoms to ~1600 with none.
`localization_3d_node` checks the clocks agree a few seconds after startup and says so
if they do not — see SIM_COMMANDS.md.

### Two scan postures, switchable with `scan_posture`

| | `tilted` (default) | `level` |
| --- | --- | --- |
| camera | (0.011, 0.400), 29.8° down | (0.090, 0.208), parallel to ground |
| 0.24 m obstacle @ R=0.30 | **100%** | 53% |
| 0.075 m floor object @ R=0.30 | **100%** | **0%** |
| max link radius over the sweep | **0.099 m** | 0.198 m |

A **level** camera's visible band is symmetric about its own height, so it can never see
the floor within the arm's reach — the floor first enters frame ~0.55 m out, past the
0.45 m limit. That is why `level` maps none of anything lying on the ground, and why
re-orienting a level camera cannot fix it. `tilted` sits higher *and* aims down, which
is what lets it gain floor coverage without losing the obstacle tops.

The halved link radius is the sleeper difference: at 0.198 m the scanning arm strikes
anything closer than ~0.28 m. At 0.099 m that constraint largely lifts.

`level` only ever looked adequate while the sim camera was a generic invention that
could resolve to 0.02 m. Modelled honestly on the D435i's ~0.2 m minimum depth, the
level scan sits *inside* its own blind spot and collects a small fraction of the points
`tilted` does. `tilted`'s own near blind spot is now covered by the second look at each
station rather than by the posture.

Note the sweep **is** where the cup is detected. That was not always true — detection
used to happen from a fixed `observe_joints` pose against a cache cleared just
beforehand — so older measurements claiming the scan posture leaves the vision result
untouched no longer apply. The scan posture now affects what vision can find.

### One sweep node, with the vision side's filtering ported in

There were two sweep implementations publishing to `/planning/point_cloud`:

| | `data_collector/scene_sweep_mapper.py` | `arm_perception/mapping_node.py` |
| --- | --- | --- |
| Moves the arm | yes, 8 waist angles x 2 looks | **no** — passive accumulator |
| Signals the end | `sweep:complete` on `/motion/status` | none; waits for `/sweep/stop` |
| Filtering | crop + voxel | voxel + statistical + radius (Open3D) |

Only the first can drive the mission: phase 1 blocks on `sweep:complete`, which
`mapping_node` never emits, so the mission would wait out `sweep_timeout_sec` and
abort. And two publishers on one topic make the planner's obstacle map alternate
between them.

So `scene_sweep_mapper` is the sweep, `mapping_node` is disabled in the mission
launches (`enable_mapping:=false`), and **its filtering has been ported across** —
statistical + radius outlier removal, same parameter names. `mapping_node` is
deliberately left in the tree; this is a consolidation, not a deletion.

The filters default **off** (sim is unchanged — Gazebo's cloud is synthetic and
clean) and are switched **on** in the hardware mission launch. Crop runs *before*
them, unlike in the original: both do nearest-neighbour searches, so discarding
the floor and everything outside the workspace first is what keeps them cheap.
Open3D is soft-imported, so a workspace without it still sweeps.

Covered by `data_collector/test/test_sweep_filtering.py`.

### Crop the obstacle map, or the RRT fallback is dead weight

Both mission launches now pass the sweep `x/y` bounds of ±0.60 and **`z_min: 0.03`**,
overriding the node defaults (±1.60, `z_min: -0.05`).

`z_min` is the important one. With the default the whole **ground plane** is kept as an
obstacle — and the base link's capsule is 0.08 + 0.015 margin = **0.095 m**, so at *any*
posture it contains ground points at z = 0. Every RRT-Connect start check therefore
failed with `start_in_collision` (`iterations=0, nodes=0`), meaning the whole-body
fallback never actually ran and only A* was ever doing work. Cropping above the floor
is what makes the fallback usable.

The `x/y` crop is secondary but real: the sweep was keeping tens of thousands of points
out to 1.5 m — far beyond the 0.45 m reach — inflating a cloud already large enough to
be dropped in transit (see the QoS note above).

### Park retracted: the arm's zero pose lies across the workspace

`home_joints` is **`[0, -0.65, -0.20, -1.00, 0]`**, not the arm's `[0,0,0,0,0]` zero.

Zero lays a link horizontally at **z = 0.255 spanning x = 0.265 to 0.373** — straight
across the front workspace. Anything standing under that bar is in collision with the
arm *while it is parked*. That produced two symptoms that both look like planner bugs
and are not:

- `Body collision on link segment 4 at waypoint [...]`, replanned six times with an
  identical waypoint each time (the blocked cell is the start cell, which the next
  iteration's `occupancy.discard(start_idx)` immediately removes);
- `RRT-Connect fallback found no path (result=start_in_collision, iterations=0)` —
  the checker correctly reporting that the arm was already inside the object.

The named poses are no better: `neutral_carry` reaches down to z = 0.204 and
`lift_ready` to z = 0.188, both *lower* than zero. The retracted pose keeps every link
within 0.051 m of the base axis, clearing an object anywhere on the front half-ring,
and sits well inside the joint limits.

This is a safety fix, not just a planning one: `home_on_abort` drives home through the
**joint** executor, which does no collision checking. Parking somewhere that overlaps
the workspace means an abort drives the real arm into whatever is standing there.

> This never showed up before because with both stubs on, the cup was a *phantom* —
> no object in the world, no cloud points, nothing to collide with. It appears the
> moment something real is in the map.

### Level-carry starts at the hover, not at the start of the approach

The constraint exists to keep a **held** cup upright, and the gripper is empty until
the grasp — so applying it to the long transit buys nothing and costs reach on a 5-DOF
arm. `_descend_onto(..., level_on_descent=True)` turns it on once the arm is at the
hover point; since hover and target differ only in z, the descent is still level and
the gripper closes in its final orientation with nothing to rotate afterwards.

It used to be enabled at the top of phase 3, which contradicted the code's own note
that "the empty-gripper approach and retreat stay unconstrained so they keep full
reach". That mattered once `observe_joints` became a deeper tuck: the first hover
waypoint could not satisfy the 10° tilt ceiling and the move died with
`ik:tilt_exceeded` before the arm had moved at all.

### The sweep locates the cup; the look-down pose is optional

`observe_joints` is **empty by default** and phase 2 normally moves nothing. The sweep
turns the camera through a full circle and every sighting it makes is kept, so the cup
can be placed anywhere in the workspace and some station will have looked at it.

This used to be a mandatory fixed pose, for a reason that no longer holds. The old
single-look sweep held the camera at **z = 0.208 aimed horizontally**, so a 0.075 m
object on the floor did not enter frame until a **0.38 m** radius — past the useful
workspace, making the sweep useless for low objects. Two changes fixed that:
`SCAN_WRIST_OFFSETS` gives every station a second, wrist-dipped look that pulls the
near floor edge in to 0.241 m, and the cup now stands on a riser (top at z = 0.060)
rather than on the floor. Measured sweep coverage at the cup's radius is 100%.

Depending on a fixed pose also capped the findable region at the ~55° the camera sees
from one heading. Move the cup outside that wedge and the mission aborted with "vision
returned no cup" — even though the sweep had stared straight at it.

To re-enable the move, pass a pose: `observe_joints:=[0.0,-1.65,1.07,1.25,0.0]` (camera
at 0.300 m standoff, outside a D435's minimum focus, which Gazebo does not model).
`observe_joints:=[]` is the "off" value and is the default. Anything that is not
exactly 5 joints is treated as off, so a truncated list can never reach the arm through
the joint executor (which does no collision checking).

It is deliberately *not* used for the phase-6 goal request: that is a large joint move
that would swing a held cup around, and the goal is a fixed fallback point anyway.

### Grasp clearance follows the object, not the current goal

The planner's `grasp_clearance_radius` sphere is centred on `/planning/grasp_anchor`
when the orchestrator has published one, and on the goal otherwise.

An approach is not one move — the orchestrator hovers `approach_height` above the
object, then descends onto it. Anchored on the goal, the sphere jumps with each
intermediate goal, so during the **hover** move (whose goal is 0.15 m up in clear air)
the object falls *outside* the sphere and becomes a hard obstacle: the planner refuses
to fly over the very thing it was sent to pick up. Anchoring on the object keeps the
exclusion attached to it for the whole approach.

The orchestrator sets the anchor before the hover, clears it once the cup is in the
gripper (it no longer sits at a fixed place in the map), re-anchors on the place point,
and clears it again after release and at the start of every cycle — a stale anchor
would keep a hole punched in the obstacle map.

### The mission now verifies the grasp

A position command is not a grasp. The fingers **reach** the commanded position when
they close on empty air, and **stall short of it** when an object is between them. So
phase 4 compares the two and aborts if the gripper closed on nothing.

`rx150_gripper_controller` republishes actual openness on `/rx150/gripper_state`,
normalized 0 (closed) .. 1 (open) — it owns the endpoints and units, so the
orchestrator's check needs no calibration of its own and works identically in sim
(finger metres) and on hardware (servo radians).

This only works when `grasp_value` is commanded **tighter** than the object, which is
the correct tuning anyway. Tunable via `grasp_detect_margin` (0.05); set
`check_grasp:=false` to disable. It deliberately does **not** abort when it cannot
tell — missing feedback, or a named token like `close` with no numeric target — since
an unverifiable grasp should not fail an otherwise fine mission.

It earned its place on the first run: `GRASP FAILED: commanded 0.40 and the fingers
reached 0.40 ... closed on empty air`, on a run that would previously have reported
`MISSION COMPLETE` while the cup sat where it started, nudged 25 mm and rotated.

Still worth confirming the physical outcome independently in sim:

```bash
gz model -m cup -p        # did it reach the goal, or just get nudged?
```

Covered by `bcr_arm_rx150/test/test_grasp_detection.py`.

### Known limitations

- **The goal is a placeholder.** `goal_classes` is empty, so a `"goal"` request always
  answers `goal_fallback_xyz` and logs a warning. Read that warning as "the arm is
  about to place the cup at a coordinate nobody looked at."
- **The 3D point is a surface sample, not the object's centroid.**
  `localization_3d_node` reads the median depth of a small patch inside the detection
  box, so the raw point sits on the face nearest the camera. Two corrections turn it
  into a grasp point, and with `bbox_anchor: bottom` both are properties of the
  **object** rather than of the observation pose:
  `surface_to_centre_m` (near face → axis, applied **horizontally** in the base frame)
  and `grasp_z_offset` (base → mid-body).

  Anchoring on the box *centre* instead makes both corrections view-dependent, because
  the centre wanders with viewing angle — high on the object seen from above, on the
  near face seen from the side. Measured in sim, from the look-down pose, against a
  cup whose true centre is `(0.300, 0.000, 0.038)`:

  | configuration | reported | error |
  | --- | --- | --- |
  | centre anchor, ray correction | `(0.331, 0.001, 0.053)` | 34 mm |
  | centre anchor, retuned offsets | `(0.291, 0.001, 0.075)` | 38 mm |
  | **bottom anchor, horizontal correction** | **`(0.302, 0.002, 0.046)`** | **9 mm** |

  Correcting along the full 3D view ray is only equivalent when the camera looks
  horizontally; from a look-down pose it drags the point downward as well as outward,
  and for a point anchored at the object's base that pushes it *through the floor* (a
  cup whose base is at z=0 came out at z=−0.030). The correction is horizontal because
  the axis of an upright object sits directly behind its near face at the same height.

- **COCO YOLOv8n is a stand-in.** `yolov8n.pt` was never trained on the task objects.
  It labels an untextured Gazebo primitive poorly (§1) and has no drop-off class at
  all. The sim accommodations (`yolo_confidence:=0.05`, widened `cup_classes`) exist
  only because of this, and should be reverted when a task-trained model lands.

---

## 8. Files

| file | role |
| --- | --- |
| `launch/rx150_pick_place_sim.launch.py` | sim mission bring-up |
| `launch/rx150_pick_place.launch.py` | hardware mission bring-up |
| `src/bcr_arm_rx150/rx150_pick_place_orchestrator.py` | the conductor / state machine |
| `src/bcr_arm_rx150/mission_keyboard.py` | keyboard control (own terminal) |
| `src/bcr_arm_rx150/rx150_gripper_controller.py` | gripper, sim + hardware |
| `src/bcr_arm_rx150/vision_placeholder.py` | vision stub — fallback only (`use_vision_stub:=true`) |
| `src/bcr_arm_rx150/sweep_placeholder.py` | sweep stub — fallback only (`use_sweep_stub:=true`) |
| `arm_perception/launch/perception.launch.py` | the perception pipeline, sim + hardware |
| `arm_perception/arm_perception/get_3d_point_node.py` | **`vision_bridge`** — the seam with the motion stack |
| `arm_perception/arm_perception/localization_3d_node.py` | 2D detection + depth → 3D point in `base_link` |
| `data_collector/.../scene_sweep_mapper.py` | the arm-driven scan sweep (sim + hardware) |
| `worlds/rx150_obstacles.world` | sim scene, including the cup + table to detect |
