# Test Protocol: Mapping Area Must Be Near the Drone GPS Origin

## Direct answer

**Yes.** There is a hard, real requirement — enforced nowhere in code, no validation, no
GCS warning — that a drawn/custom mapping area (boundary) must be geographically close to
the drones' true GPS origin: **28.682412°N, 77.499734°E**
(`project_gazebo/config/world_swarm.yaml`'s `origin:` block). If the boundary is placed far
from that coordinate, the mission still "runs" (arms, takes off, reports `complete`) but the
computed flight target is nowhere near where the drones physically are, which is consistent
with what you saw: takeoff, no visible attempt to fly toward the drawn area, then landing.

## Why this is a real constraint, not a guess

Traced end-to-end through the actual code (not assumed):

1. **`origin:` in `world_swarm.yaml` is the Gazebo world's `<spherical_coordinates>`
   reference**, injected via Jinja templating at world-generation time
   (`as2_gazebo_assets/src/as2_gazebo_assets/world.py` → `grass.sdf.jinja`'s
   `<spherical_coordinates><latitude_deg>{{ origin.latitude }}</latitude_deg>...`).
   This is what Gazebo's GPS sensor uses to convert each drone's *local* simulated
   position into the lat/lon it reports.
2. **All 4 drones spawn within ~2m of Gazebo-local (0,0,0)**
   (`world_swarm.yaml`'s per-drone `xyz:` values: `[-2,0,0.2]`, `[2,0,0.2]`, `[0,0,0.2]`,
   `[0,2,0.2]`), which spherical_coordinates maps to ~28.682412°N, 77.499734°E ± a couple
   meters. **This is where the drones actually, physically are** — nothing in the system
   ever relocates it.
3. **`nidar_mission_manager`'s `zone_splitter`/`path_planner`/`geo_utils` have zero
   proximity validation.** They do real geodetic ENU math (`pymap3d`) and will happily
   produce a mathematically valid zone/lawnmower path for a boundary anywhere on Earth —
   including a `mission.get('home')` custom launch site from "Set/Randomize Launch Site",
   which we confirmed does **not** actually matter here: the ENU round-trip
   (`latlon_to_enu` → geometry → `enu_to_latlon`) is exact regardless of which origin is
   used as the intermediate reference, so the *output* waypoints always land at the real
   lat/lon of wherever the boundary was actually drawn. The "home" field is a red herring —
   it does not decouple the mission from the true origin, drawing the boundary far away does.
4. **AS2's own flight commands (`go_to_gps_point`, `follow_path`) independently
   re-derive the local target** using `self.__drone.gps.origin`
   (`as2_python_api/behavior_actions/go_to_behavior.py:113`,
   `followpath_behavior.py:114`) — **not** anything our script passes. This per-drone
   origin is set by the `ground_truth` state estimator plugin (this project's configured
   plugin — `project_gazebo/config/config.yaml`: `plugin_name: "ground_truth"`,
   `use_gps: true`, `set_origin_on_start: true`), which — since no explicit
   `set_origin.lat/lon/alt` params are configured — **auto-sets each drone's origin to its
   own first GPS fix at boot** (`ground_truth.hpp`'s `gps_callback`). That's the same
   ~28.682412°N/77.499734°E ± spawn offset, independently confirmed via a second mechanism.

So there are two *independent* systems (our Python math, and AS2's own C++ state
estimator) that both anchor "reality" to the true spawn location — neither one cares what
the GCS map's boundary coordinates are, and neither one will ever move the drones' actual
launch point to match a drawn area far away.

## What was confirmed live this session

- A boundary-coverage mission with a small (~30m) boundary drawn **at** the true origin
  produced genuine, continuous flight: drone0's GPS latitude was sampled 1734 times over
  ~60s and moved steadily and monotonically toward its planned waypoint — this is real
  motion, not a stub or an instant no-op.
- A direct comparison test (same mission, boundary ~500m+ from the origin) was set up but
  the simulation hung (Gazebo's own `/clock` and `/world/grass/stats` both went silent,
  outside of anything this refactor/investigation touched) before it could complete — **this
  was not resolved and is reported honestly rather than guessed at.** Phase 2 below is
  written so you (or a follow-up session) can complete exactly this comparison cleanly.

## Test protocol

Run each phase on a **fresh** `./scripts/run_simulation.sh` (a clean sim avoids one test's
armed/flying state contaminating the next one's readings — this bit me during live testing
above). Use `ros2 topic pub <topic> ... --rate 2` for one second instead of `pub -1` when
testing manually — `-1` fires a single message immediately and can lose the race against
DDS discovery matching (confirmed live this session); the real GCS doesn't have this problem
since it keeps a persistent rosbridge connection, but a CLI test can silently swallow a
message and look like a hang that isn't one.

### Phase 1 — Baseline (control)

1. Fresh sim up. Confirm `ros2 node list` shows all 4 `/droneN/platform` nodes.
2. Load and start a boundary-coverage mission with a small boundary (~30–50m) centered
   **exactly on** `28.682412, 77.499734`.
3. Watch one drone's `sensor_measurements/gps` topic for the mission's duration
   (`ros2 topic echo /drone0/sensor_measurements/gps --field latitude`, piped to a file,
   not `tail -1` directly — signals can kill the pipe before `tail` flushes).
4. **Expected / already confirmed:** steady, monotonic GPS movement; `mission_executor.log`
   progresses `starting → taking_off → running → landing → complete`.

### Phase 2 — Distance sweep (the actual diagnostic)

Repeat the same boundary shape/size, only re-centered at increasing distances from the true
origin, one fresh sim per distance so a stuck/slow far test can't block the next one. Use a
non-trivial `speed_mps` (e.g. `5.0`–`8.0`) so each attempt finishes in a reasonable window —
`_run_boundary_coverage_mission` defaults to `0.5 m/s`, which makes anything beyond a few
tens of meters impractical to sit and watch.

| Step | Offset from true origin | Purpose |
|---|---|---|
| 2a | ~100m | Still "close" — sanity check the mechanism holds a bit past Phase 1's very-tight boundary |
| 2b | ~500m | Where the earlier live test hung — retry in isolation, watch `/clock` and `/world/grass/stats` (`gz topic -e -t /world/grass/stats`) alongside GPS to tell a real hang from a very slow flight |
| 2c | ~2km | Representative of "user drew the area somewhere unrelated on the map" |

At each step, record: does `mission_executor.log` reach `running`? Does GPS visibly move
toward the target, stall partway, or never move? Does the mission reach `complete` (implying
`follow_path` returned success) despite no real alignment, or does it report
`error: follow_path failed for: [...]` (in which case, per `mission_executor_node.py`, land
is **never** called — the drones would stay hovering, not land, which doesn't match what you
saw — so if 2b/2c instead shows a clean `complete`, that's the concrete confirmation of the
"silently succeeds without flying" failure mode)?

### Phase 3 — Isolate boundary-coverage vs. manual GPS commands

Run a plain JSON-waypoint mission (`mission.drones`, bypassing `zone_splitter`/
`path_planner` entirely) with one waypoint ~500m from origin, to check whether
`go_to_gps_point` (used by the waypoint path) behaves the same as `follow_path` (used by
the boundary path) at distance, or differently — they are different AS2 behaviors
(`GoToBehavior` vs `FollowPathBehavior`) and were **not** confirmed to behave identically
this session.

### Phase 4 — Confirm the fix

Once Phase 2 pins down the exact threshold/failure mode, the fix is almost certainly one or
both of:
- **Validate at mission-start time**: before dispatching a boundary-coverage mission,
  compute the boundary's distance from `self.origin` (reuse `geo_utils.latlon_to_enu`) and
  reject with a clear GCS-visible error (`_publish_status('error', 'boundary is Xm from
  the drone launch site — redraw closer to the marked launch point')`) if it exceeds a
  sane threshold (a few hundred meters, tunable).
- **GCS-side guardrail**: warn in `MapView.tsx`/`MappingAreaToolbar.tsx` while drawing if
  the boundary strays far from `DEFAULT_CENTER` (which already correctly equals the true
  origin, `28.682412, 77.499734` — confirmed in `MapView.tsx`), so the operator catches it
  before ever clicking Start, not after watching a drone silently do nothing.

Re-run Phase 2's failing distance after the fix and confirm it now either flies correctly
(if within threshold) or is rejected with a clear error at mission-start (if not) — not a
silent takeoff-and-land.
