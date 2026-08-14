# RX-150 RRT-Connect — How the Whole-Body Fallback Planner Works

Explainer for the joint-space RRT-Connect fallback. This is the *logic* reference — what the
algorithm does, why it's in joint space, how it validates against the point cloud, what it
reuses, and how it executes on both sim and the physical arm. For the design *rationale* —
why these choices were made over the alternatives — see
[WHOLE_BODY_MOTION_PLANNING.md](WHOLE_BODY_MOTION_PLANNING.md).

**Status: implemented.** Where the code
lives:

| Concern | File |
|---------|------|
| RRT-Connect algorithm (ROS-free) | `src/bcr_arm_rx150/rx150_rrt_connect.py` |
| Goal configs + `collision_fn` + `plan_joint_path` glue | `src/bcr_arm_rx150/rx150_rrt_fallback.py` |
| Fallback hook + `/planned_joint_path` + green RRT marker | `rx150_point_cloud_path_planner.py` |
| Direct joint-command channel (no IK) | `rx150_dls_ik_executor.py` (`/rx150/joint_command`) |
| Joint-space waypoint stepper | `src/bcr_arm_rx150/rx150_joint_waypoint_executor.py` |
| Unit tests | `test/test_rx150_rrt_connect.py`, `test/test_rx150_rrt_fallback.py` |

## Why it exists

The primary planner (`rx150_point_cloud_path_planner.py`) searches a **2D grid for the tool
tip at one transit height**, then checks the whole arm and blocks offending cells. That
solves the easy majority of targets in milliseconds, but it structurally cannot solve the
case where **start and goal are on opposite sides of a tall obstacle**: the tool-tip path has
no "up and over" or "retract, swing the waist around, re-approach" representation, and
lifting the tip doesn't lift the base-anchored elbow/gripper over the obstacle (confirmed
live — `segment 4` still collided at a 0.31 m transit height).

The binding constraint isn't the end-effector path — it's the **arm's whole configuration**.
So the fallback plans in the space where that constraint is exact: the joint angles.

## The core idea: plan in joint space

RRT-Connect searches over the **5 joint angles** (waist, shoulder, elbow, wrist_angle,
wrist_rotate), not over Cartesian tool-tip positions.

Why that matters: a joint configuration `q` fully determines *where every link is*. Given
`q`, forward kinematics returns every joint/link position, so the collision check is exact
for the **entire arm body**, not just one tracked point. And the search can discover motions
that simply don't exist as a tool-tip path — e.g. retract the arm, rotate the waist to a
clear sector, then re-approach the goal from the open side.

A "path" here is therefore a **list of joint configurations** from start to goal, not a list
of XYZ waypoints.

## How RRT-Connect works

RRT-Connect grows **two trees** of collision-free configurations — one rooted at the start
config, one at the goal config — and tries to join them.

```
q_start ●───●───●        (start tree grows toward random samples)
             \
              ●·····?·····●
                          /
        ●───●───●───● q_goal   (goal tree grows to meet it)
```

Each iteration:

1. **Sample** a random joint configuration within the joint limits.
2. **Extend** the current tree: find its nearest node (Euclidean distance in joint space) and
   step `step` rad from that node toward the sample, checking the edge for collisions along
   the way. Add the new node if the step is collision-free.
3. **Connect** the *other* tree toward that new node: keep stepping (`step` rad at a time)
   until it either reaches the node — **the trees are joined, a path exists** — or hits a
   collision and stops.
4. **Swap** the two trees and repeat.

When the trees connect, walk start-tree node → root and goal-tree node → root, stitch them,
and you have a start-to-goal joint path. Then **smooth** it: repeatedly pick two non-adjacent
configs on the path and try to replace the span between them with a straight
configuration-space segment; keep the shortcut if it's collision-free. This removes the
random jitter RRT produces and yields a cleaner motion.

It stops and returns the path on connection, or returns **None** when it exhausts its
iteration cap (`max_iters`) or wall-clock budget (`time_budget_sec`) — a budget exhaustion is
a strong "genuinely unreachable within budget" signal, not a silent give-up.

**Probabilistic completeness:** if a valid motion exists, RRT-Connect finds one given enough
samples; it does not guarantee the *shortest* path (smoothing approximates that).

## Edge checking (the important detail)

A single joint step can move the arm far enough to sweep *through* an obstacle even when both
endpoints are clear. So every edge between two configs is validated by **interpolating at a
finer `check_step` resolution** and collision-checking each intermediate config, endpoints
included. Node validity alone is not enough — the motion between nodes is what's checked.

## Collision checking = validating against the point cloud

RRT-Connect has exactly one source of truth for "is this posture in collision": the
**point cloud**. Every validity test routes through one function:

```python
def collision_fn(q):
    if out_of_joint_limits(q):
        return True                              # outside the reachable range → invalid
    joints = rx150_kinematics.link_positions(q)  # FK: where every link is for this config
    collided, _ = rx150_kinematics.check_arm_collision(
        joints, obstacle_points_3d, extra_margin=collision_extra_margin)
    return collided
```

- `obstacle_points_3d` is the planner's existing `_obstacle_points_3d(cloud)` — the **same
  `/planning/point_cloud`** the A\* body check uses (swept cloud, above the height threshold
  so the floor is filtered, voxel-downsampled, in `rx150/base_link`).
- `check_arm_collision` is the **same 5-capsule whole-arm model** the primary planner already
  calls, with the same safety margin. Each arm segment is a capsule; if any obstacle point
  falls within a capsule's radius, that's a collision.

So the two planners agree on what "collision" means, and **the point cloud is the only
obstacle information RRT has** — validated at three granularities: every sampled config,
every interpolated edge step, and every candidate goal config.

Two honest caveats, both shared with A\* and not new:

- **It's a snapshot.** The cloud is captured by the sweep *before* planning; RRT plans
  against that static map and does not re-sense during motion. Anything that moves into the
  scene after the sweep is invisible until you re-sweep.
- **It's point-based, so it's only as good as the cloud's coverage.** The capsule-vs-points
  test relies on obstacle surfaces being sampled densely enough that no thin segment threads
  between sparse points — which is exactly why camera placement / sweep coverage matter. A
  sparse or wrong cloud makes RRT confidently plan into things it can't see, same as A\*.

## What it reuses vs. what's new — and the IK relationship

RRT-Connect is *not* an IK solver and does not replace one. The relationship:

| Piece | Who does it | Notes |
|-------|-------------|-------|
| **Start config** | read from `/rx150/joint_states` | identical on sim and physical |
| **Goal config(s)** | the existing **DLS IK solver**, run **once** | multi-seeded (current, neutral, waist-/elbow-flipped) to get several IK branches; the far-side goal is often reachable on only one. Drop any goal config that itself collides. |
| **Search** | RRT-Connect over joint angles | uses **FK**, not IK, to evaluate every sampled config |
| **Collision** | `rx150_kinematics` FK + capsule check | same model as A\* |
| **Execution** | the existing `rx150_dls_ik_executor` as a mode-aware arm gateway | the joint path is commanded directly, **bypassing** the IK solve |

So DLS IK is used **only** to turn the Cartesian target into goal configs; the search itself
is pure FK + collision. **No MoveIt** anywhere.

## How it fits as a fallback (no regression)

RRT-Connect is layered *on top of* the current A\*, invoked only when the fast planner gives
up. A\* stays the deterministic millisecond path for the easy ~90%; RRT (randomized, up to
~1–2 s) fires only at the exact branch where the primary planner already logs
`Could not find a whole-body collision-free path … publishing nothing`.

```
target arrives (_target_cb)
   │
   ▼
2D A* + whole-body capsule check         ── existing, unchanged
   │
   ├─ body-clear path ─► nav_msgs/Path ─► Cartesian waypoint executor ─► IK executor
   │
   └─ no body-clear path
          │
          ▼
     Joint-space RRT-Connect              ── the fallback
          │
          ├─ path found ─► joint waypoints ─► IK executor's mode-aware arm channel
          │
          └─ budget exhausted ─► publish nothing (fail-safe); goal marker still shows
```

The two output channels are mutually exclusive per target, so there's no command contention,
and easy targets are provably unaffected.

## Execution: joint waypoints, portable to the physical arm

RRT's guarantee is in joint space, so the path **must be executed as joint angles** — never
re-solved back through Cartesian IK, which could land in a different, unchecked posture. It
also must not hardcode the sim's trajectory topic.

`rx150_dls_ik_executor` already abstracts the command channel with a `command_mode` param:

- **`group`** (default, physical) → `interbotix_xs_msgs/JointGroupCommand` on
  `/rx150/commands/joint_group`
- **`trajectory`** (sim override) → `trajectory_msgs/JointTrajectory` on
  `/rx150/arm_controller/joint_trajectory`

The fallback reuses that abstraction: feed each RRT config to the executor's direct
joint-command path (which calls its existing publish routine **without** the IK solve), so it
emits the right message per platform automatically. The dense, smoothed RRT path is played
**waypoint by waypoint** — command a config, wait until `/rx150/joint_states` is within a
joint-space tolerance, advance, paced by a per-segment joint-speed limit. This step-wise
approach works identically in sim and on hardware.

**Physical portability is real, not assumed:** joint state is the same `/rx150/joint_states`
on both; FK, capsules, DLS IK and joint limits are pure math; the only sim/real difference is
the command channel, which is already abstracted. The one pre-existing dependency shared with
A\* is that on hardware the point cloud needs the real depth camera mounted and calibrated.

## Honest scope

Some far-side targets are **genuinely infeasible** for a base-anchored 5-DOF arm. The win is
that RRT-Connect *finds* a whole-body-clear motion when one exists and *proves* none exists
(within budget) when it doesn't — replacing today's ambiguous early give-up with a real
search. That's valuable even when the answer is "can't be done."

## Key parameters (all tunable)

| Param | Default | Meaning |
|-------|---------|---------|
| `step` | 0.10 rad | tree growth increment per extend |
| `check_step` | 0.05 rad | edge collision-check resolution |
| `max_iters` | 3000 | hard iteration cap |
| `time_budget_sec` | 1.5 | wall-clock cap (whichever hits first) |
| `smoothing_iters` | 100 | shortcut attempts after connect |
| goal IK seeds | ~4–5 | IK-branch diversity for the goal config |
| `use_rrt_fallback` | true | master toggle for the fallback |
