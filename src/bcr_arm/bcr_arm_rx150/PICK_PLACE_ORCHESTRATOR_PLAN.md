# Plan: Pick-and-Place Orchestrator (Physical RX-150)

A single node that runs the whole cup pick-and-place **mission** end to end, by
issuing the *same* commands you send by hand today (a `PointStamped` on
`/cartesian_target`, a `String` on `/rx150/gripper_command`, a home pose) — just
sequenced automatically, with each step waiting for the previous one to actually
finish before the next begins.

It is a **conductor**, not a new planner: it reuses the existing planner, RRT
fallback, IK executor, waypoint executors, and gripper node unchanged. It only
decides *what* to send and *when*.

---

## 1. The core problem: how does a step know the previous one finished?

**We added completion events.** The motion nodes previously had no
machine-readable "done" signal — the executors just cleared their waypoint list
(or logged a message) and the sweep node sequenced by a fixed `settle_sec` sleep.
The mission has variable-length motions (the cup can be anywhere), so a fixed
sleep can't tell a finished reach from "the planner refused and the arm never
moved." So each motion node now **emits lifecycle events** on `/motion/status`
(`std_msgs/String`), and the orchestrator waits for a terminal one before
advancing.

### The event contract (`/motion/status`, `std_msgs/String`)

| Event | Emitter | Meaning | Orchestrator |
|-------|---------|---------|--------------|
| `path:complete` | `rx150_path_waypoint_executor` | Cartesian path finished | success → advance |
| `path:aborted` | `rx150_path_waypoint_executor` | gave up (unreachable waypoint) | failure → abort |
| `joint:complete` | `rx150_joint_waypoint_executor` | joint/RRT path finished (also home) | success → advance |
| `joint:aborted` | `rx150_joint_waypoint_executor` | gave up (stuck waypoint) | failure → abort |
| `planner:no_path` | `rx150_point_cloud_path_planner` | fail-safe: no whole-body path found | failure → abort |

The planner does **not** emit a success event — whichever executor drives the
published path reports completion. `planner:no_path` fires the moment planning
fails, so the orchestrator **fails fast** rather than waiting on a timeout.

The orchestrator's `_wait_for_motion` clears `_last_status`, the command is sent,
and it waits for the next terminal event: a success token returns True, a failure
token returns False. `move_timeout_sec` (≈ 60 s) remains **only as a backstop** so
a lost event can never hang the mission — it is not the primary signal.

Why events over watching `/rx150/joint_states` for arrival (the alternative first
considered): events fire **exactly** when a move ends, **disambiguate success from
failure** explicitly (a geometric arrival check can't tell "planner refused" from
"still moving" — only a timeout can), and don't depend on tuning position/velocity
tolerances. The cost was small and contained: a few lines in each of the three
motion nodes (a `status_topic` param, a publisher, an `_emit` call at each
terminal branch), plus a stuck-abort added to the joint executor (which previously
never gave up) so it, too, always reaches a terminal event.

**Home** is routed through the joint executor as a one-waypoint joint path on
`/planned_joint_path`, so it reports `joint:complete` on the same channel — no
separate arrival check. `/rx150/joint_states` is still consumed, but only to
compute the relative "lift" targets (current end-effector + `dz`).

---

## 2. Architecture

```
                    rx150_pick_place_orchestrator  (NEW — the conductor)
                                 │
   issues the SAME topics a human uses:                 waits on events:
     /cartesian_target      (PointStamped)   ┐          /motion/status (String)
     /rx150/gripper_command (String)         ├── to ──►   path:complete / path:aborted
     /planned_joint_path    (home)           ┘            joint:complete / joint:aborted
                                 │                        planner:no_path
                                 ├─ asks VISION:  /vision/find_request (String "cup"|"goal")
                                 └─ gets back:    /vision/object_point (PointStamped, base frame)

   The existing stack now EMITS /motion/status events (small additions):
     /cartesian_target → planner (+RRT) → /planned_(cartesian|joint)_path
                       → waypoint executors → IK executor → arm
                       → executors emit path:*/joint:* ; planner emits planner:no_path
     /rx150/gripper_command → rx150_gripper_controller → gripper
   (/rx150/joint_states is still read, only to compute relative "lift" targets.)
```

The orchestrator is a **procedural node** in the style of `scene_sweep_mapper`: a
blocking `run()` that steps through the mission phases, `spin`-ning between them
and waiting on the arrival predicate. Easy to read top-to-bottom, easy to abort.

---

## 3. The mission state machine

Each phase: **what it sends** → **how it waits** → **failure handling**. `RETURN`
in your description = "wait until this move is done before continuing" = wait for a
terminal `/motion/status` event.

| # | Phase | Sends | Waits for | On failure |
|---|-------|-------|-----------|------------|
| 0 | **Env up** | (launch brings up the physical stack + gripper + placeholders) | start trigger (`/mission/start`, unless `autostart`) | — |
| 1 | **Sweep** | trigger the sweep (placeholder today) → produces `/planning/point_cloud` | map present + non-empty on `/planning/point_cloud` | abort — no obstacle map, unsafe to plan |
| 2 | **Find cup** | `/vision/find_request` = `"cup"` | a fresh `/vision/object_point` within `vision_timeout_sec` | abort — nothing to grasp |
| 3 | **Move to cup** | that point → `/cartesian_target` | `/motion/status` = `path:complete`/`joint:complete` | `path:aborted`/`joint:aborted`/`planner:no_path` → abort, do **not** grasp |
| 4 | **Grasp** | `/rx150/gripper_command` = `"close"` (or `grasp_value`) | fixed `grasp_settle_sec` (servo is fast; slip makes exact position unreliable) | continue but log |
| 5 | **Lift cup** | current EE with `+lift_dz` → `/cartesian_target` | terminal motion event | abort — gripper stays closed (holding cup) |
| 6 | **Find goal** | `/vision/find_request` = `"goal"` | fresh `/vision/object_point` | abort |
| 7 | **Move to goal** | goal point (+ optional place height) → `/cartesian_target` | terminal motion event | abort |
| 8 | **Release** | `/rx150/gripper_command` = `"open"` | `grasp_settle_sec` | continue |
| 9 | **Lift clear** | current EE with `+clearance_dz` → `/cartesian_target` | terminal motion event | continue (best-effort) |
| 10 | **Home** | `[0,0,0,0,0]` as a one-waypoint joint path → `/planned_joint_path` | `/motion/status` = `joint:complete` | log if not reached |

Phases 5 and 9 (“lift”, “lift a little”) are **relative** moves: read the current
EE from `joint_states`+FK, add a `dz`, publish that as a new `/cartesian_target`.
That keeps them independent of exactly where the cup/goal ended up.

Any phase's failure calls `_abort(reason)`: log loudly, stop sending targets, and
(configurably) send the arm home so it doesn't sit in a cluttered pose.

---

## 4. Vision interface — the contract for your teammate

Designed so vision returns **exactly what you send by hand**: a
`geometry_msgs/PointStamped` in `rx150/base_link`. No custom messages, trivial to
stub in any language. This is the whole handoff:

```python
# ── VISION CONTRACT (orchestrator ⇄ vision node) ─────────────────────────────
#
# REQUEST  (orchestrator → vision)
#   topic : /vision/find_request        std_msgs/String
#   data  : "cup"   -> locate the cup to pick
#           "goal"  -> locate the drop-off / goal location
#
# RESPONSE (vision → orchestrator)
#   topic : /vision/object_point        geometry_msgs/PointStamped
#   header.frame_id : "rx150/base_link"   ← MUST be the arm base frame
#                     (if vision can only produce camera-frame points, publish
#                      in the camera frame + we TF-transform — but base_link is
#                      preferred and matches the manual /cartesian_target format)
#   header.stamp    : capture time (used to reject stale/previous detections)
#   point.{x,y,z}   : object position in metres, base frame — the SAME vector you
#                     would type into `ros2 topic pub /cartesian_target ...`
#
# CONTRACT NOTES
#   • One request → one fresh response. Orchestrator ignores any response whose
#     stamp is older than the request it just sent (avoids acting on a stale fix).
#   • Vision owns *how* it finds things (detection, depth, filtering). We only
#     depend on the point it returns being reachable-ish and in metres/base_link.
#   • Until vision is ready, `vision_placeholder` (below) answers with a canned
#     point so the full mission runs end to end.
# ─────────────────────────────────────────────────────────────────────────────
```

**Why request/response topics instead of a service:** a `std_srvs`-style service
would be marginally cleaner, but a topic pair needs zero custom `.srv` build and
your teammate can stub it in five lines. If we later want strict req/resp
semantics, promoting `/vision/find_request` + `/vision/object_point` to a single
service `/vision/locate(string) -> PointStamped` is a drop-in change — noted in
the orchestrator as a TODO so the swap is localized.

---

## 5. Placeholder nodes (so the mission runs before teammates connect)

Both are throwaway stubs with the real interface, clearly named so they're easy to
delete/replace:

- **`vision_placeholder`** — subscribes `/vision/find_request`; on `"cup"` /
  `"goal"` publishes a hardcoded `PointStamped` (params `cup_xyz`, `goal_xyz`,
  frame `rx150/base_link`). Lets us validate the *whole* sequence + arrival logic
  with the real motion stack today.
- **`sweep_placeholder`** — the real sweep needs the RealSense (not done). Options,
  cheapest first:
  1. reuse the **sim** `scene_sweep_mapper` when a cloud source exists; or
  2. a stub that latches a **canned `/planning/point_cloud`** (e.g. one box) so
     the planner has a map to check against.
  The orchestrator's phase-1 only requires "a non-empty map exists on
  `/planning/point_cloud`," so either satisfies it. Real sweep drops in later with
  no orchestrator change (this is exactly the boundary in
  [TASK_realsense_sweep_pipeline.md](TASK_realsense_sweep_pipeline.md)).

---

## 6. "One command to run it" — the entrypoint

You pictured it opening the terminals and typing the commands for you. A ROS node
shouldn't spawn terminals; the clean equivalent is **one launch that brings up the
whole mission**, with the orchestrator as just another node in it:

- **`launch/rx150_pick_place.launch.py`** — includes
  [rx150_dls_stack.launch.py](src/bcr_arm/bcr_arm_rx150/launch/rx150_dls_stack.launch.py)
  (driver + gripper + planner + RRT + executors + relay), plus `vision_placeholder`,
  plus (optionally) `sweep_placeholder`, plus `rx150_pick_place_orchestrator`.
  Arg `autostart` (default `false`) so the arm doesn't move the instant you launch
  — you launch, confirm everything's green, then flip a `/mission/start` trigger.

So the manual "4 terminals" workflow (stack / sweep / target / gripper) collapses
into: `docker compose run --rm --service-ports rx150-hardware ros2 launch
bcr_arm_rx150 rx150_pick_place.launch.py`, then start the mission. Documented in
[HARDWARE_COMMANDS.md](HARDWARE_COMMANDS.md). (If you truly want separate log
windows, a thin `run_pick_place.sh` that opens them is a nice-to-have, but the
single launch is the primary path.)

---

## 7. Files

| File | New/edit | Purpose |
|------|----------|---------|
| `src/bcr_arm_rx150/rx150_pick_place_orchestrator.py` | **new** | the conductor / state machine |
| `src/bcr_arm_rx150/vision_placeholder.py` | **new** | stub answering the vision contract |
| `src/bcr_arm_rx150/sweep_placeholder.py` | **new** | canned `/planning/point_cloud` |
| `launch/rx150_pick_place.launch.py` | **new** | one-command mission bring-up |
| `rx150_path_waypoint_executor.py` | edit | emit `path:complete`/`path:aborted` on `/motion/status` |
| `rx150_joint_waypoint_executor.py` | edit | emit `joint:complete`/`joint:aborted` + add a stuck-abort |
| `rx150_point_cloud_path_planner.py` | edit | emit `planner:no_path` on the fail-safe |
| `setup.py` | edit | register the three console scripts |
| `HARDWARE_COMMANDS.md` | edit | "run the full mission" section |
| `bcr_arm_common.rx150_kinematics` | reuse | FK for relative-lift targets (no change) |

No new package dependencies — it's `rclpy` + existing messages
(`geometry_msgs`, `std_msgs`, `sensor_msgs`, `trajectory_msgs`,
`interbotix_xs_msgs` already in use). The three motion-node edits each add only a
`status_topic` param, a `String` publisher, and an `_emit` call — the planners/
executors otherwise behave exactly as before.

---

## 8. Parameters (all tunable, sane defaults)

- Motion events: `move_timeout_sec` 60 (backstop only — events are the primary
  signal), `status_topic` `/motion/status`. The joint executor's
  `stuck_waypoint_abort_sec` (12 s) bounds how long before it emits `joint:aborted`.
- Gripper: `grasp_value` (placeholder — **verify on hardware**, see the gripper
  node's units note), `release_value` `open`, `grasp_settle_sec` 1.5.
- Geometry: `lift_dz` 0.06 m, `clearance_dz` 0.03 m, optional `place_height`.
- Frames: `planning_frame` `rx150/base_link` (matches the whole stack).
- Vision: `vision_timeout_sec` 10, request/response topic names.
- Safety: `home_on_abort` true, `autostart` false.

---

## 9. Open questions (remaining before real hardware)

1. **Grasp value** — need a real closed value/force for the cup on hardware (ties
   into the gripper operating-mode/units question already flagged).
2. **Place height at the goal** — does vision's "goal" point already include the
   right z to release at, or should we hover a fixed height above it?
3. **Gripper feedback** — grasp/release currently use a fixed `grasp_settle_sec`
   (a servo has no clean "closed" event; force/slip makes exact position
   unreliable). Fine for now; could add a joint_states check later if needed.
4. **Sweep placeholder flavor** — canned cloud (built) vs. reuse the sim sweep —
   swap in the real RealSense sweep on `/planning/point_cloud` when ready.
```
