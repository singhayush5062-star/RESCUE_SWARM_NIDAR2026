# Phase 1 — Aerostack2 in Simulation + GCS Skeleton: Test Report

**Machine tested:** `ayush-LOQ-15APH8` (dev machine), Ubuntu 22.04.5 LTS
**Date:** 2026-08-23
**Tested by:** Ayush (singhayush5062@gmail.com)

---

## 1. Blocker fix: TakeoffBehavior never climbed (carried over from Phase 0)

Phase 0 testing found that `TakeoffBehavior` reported `RUNNING` forever without the
drone ever leaving the ground, in the stock `project_gazebo` example. That report
suspected a reference-frame mismatch. It wasn't.

**Root cause:** `project_gazebo/config/pid_speed_controller.yaml` wraps every PID
gain under an extra `pid_speed_controller:` key that shouldn't be there:

```yaml
/**:
  ros__parameters:
    pid_speed_controller:        # <- this level doesn't exist in the plugin's schema
      position_control:
        kp: {x: 1.0, y: 1.0, z: 1.0}
```

The plugin's actual default schema
(`as2_motion_controller/plugins/pid_speed_controller/config/controller_default.yaml`)
is flat — `position_control.kp.x` directly under `ros__parameters`, defaulted to
`0.0`. Because of the extra nesting, ROS's parameter merge treats the override file's
values as inert siblings instead of overriding the real (zero) defaults. Mode
negotiation, TF, and the reference topic all look completely healthy — the PID
literally runs with `Kp = Ki = Kd = 0`, so `computeOutput()` always returns zero
velocity. This is confirmed to be a bug in the upstream `aerostack2/project_gazebo`
example itself (checked byte-for-byte against the GitHub source), not a local
misconfiguration.

**Fix:** de-indented `project_gazebo/config/pid_speed_controller.yaml` to match the
flat schema. Verified: `ros2 param get /drone0/controller_manager position_control.kp.x`
now returns `1.0`.

**Result after the fix**, all re-tested from a clean sim relaunch:
- Single-drone mission (`mission.py`): arm → offboard → takeoff → 8 waypoints
  (keep-yaw + path-facing) → land → manual — all steps report success.
- 3-drone swarm choreography (`mission_swarm.py`): takeoff → triangle/line formation
  dance → land, for `drone0`/`drone1`/`drone2` concurrently — completes cleanly.
- Re-running a mission a second time in the same live session also works (ruled out
  as a contributing factor during a later investigation, see §3).

---

## 2. Second bug found: `DroneInterfaceGPS` couldn't even be constructed

While building the mission-file loader (§4), GPS-referenced waypoints were needed.
`as2_python_api`'s `DroneInterfaceGPS` — used by the stock `mission_gps.py` example —
crashed immediately, even unmodified:

```
TypeError: Can't instantiate abstract class GpsModule with abstract method __call__
```

**Root cause:** `as2_python_api/modules/module_base.py`'s `ModuleBase` is an ABC
requiring every subclass to implement `__call__`. Every other module
(`motion_reference_handler_module.py`, `rtl_module.py`, `dummy_module.py`) implements
it — `gps_module.py` was simply missing it.

**Fix:** added the same no-op pattern used elsewhere in the codebase
(`motion_reference_handler_module.py:70`) to `as2_python_api/modules/gps_module.py`:

```python
def __call__(self):
    pass  # implementation is required from the abstract class ModuleBase
```

The file is symlink-installed (`colcon build --symlink-install`), so no rebuild was
needed — verified immediately by re-running `mission_gps.py`, which then flew a real
GPS-referenced waypoint (climbed to 5m, moved ~7m from home) successfully.

---

## 3. Environment fix: baked into `scripts/setup_nidar_ros.sh`

`conda deactivate` (used to work around the Phase 0 conda-vs-system-python3 issue)
requires `conda init` to have run for the shell, which isn't guaranteed — it failed
silently in a non-interactive shell during this session and let conda's Python 3.14
back onto `$PATH`, which then broke `ros2 launch` itself (not just directly-run
scripts) for both `rosbridge_server` and `rosapi_node`.

**Fix:** the script now strips conda's directories from `$PATH` directly instead of
calling `conda deactivate`, and also exports `GZ_IP`/`GZ_PARTITION` automatically
(from the Phase 0 multicast fix). One `source scripts/setup_nidar_ros.sh` now
reliably sets up a working shell for AS2 + Gazebo + rosbridge work.

**Investigation note:** a "TakeoffBehavior stuck again" scare during this session
(after the fixes above) turned out to be self-inflicted — an earlier crashed test of
`mission_file_executor.py` (§4) left a half-constructed `DroneInterfaceGPS` node
running in the background, which then contended with a fresh flight attempt on the
same drone. It was not a new AS2 bug; isolated re-tests from clean sim restarts
confirmed both the PID fix and the GPS fix hold up correctly. The executor's error
handling was hardened as a result (see §4).

---

## 4. Multi-drone bring-up

Brought up 2 drones (`drone0`, `drone1`) via `./launch_as2.bash -m -n drone0,drone1`
using `config/world_swarm.yaml`. Both come up with a fully symmetric node/topic set
and independent GPS/battery/state-estimator pipelines. Also verified 3 concurrent
drones via the stock `mission_swarm.py` (§1).

## 5. Behavior-tree mission reproduced end-to-end

Ran the stock `trees/square.xml` via `as2_behavior_tree` (already wired into
`tmuxinator/aerostack2.yaml`'s `mission_execution` window). It waits on a
`/drone0/start` (`std_msgs/String`) event, then runs
`Arm → Offboard → TakeOff → GoTo×4 → Land` as a `Sequence`. Triggered it manually
and confirmed every node reports `SUCCESS`, matching Phase 1's "reproduce one
behavior-tree mission end-to-end in sim" requirement.

---

## 6. GCS frontend

**Stack:** React + Vite + TypeScript, Leaflet/`react-leaflet` for the map,
`roslib` over `rosbridge_suite`'s WebSocket bridge (`ws://localhost:9090`) — chosen
over a native PyQt/rclpy GCS for faster UI iteration on the map/dashboard the
mission brief requires. Scaffolded at `/home/ayush/NIDAR/gcs`.

- `src/ros/RosConnection.ts` — single shared rosbridge connection (the mission
  brief requires one operator interface for all drones, not one per drone).
- `src/ros/useDroneTelemetry.ts` — subscribes to
  `/<namespace>/sensor_measurements/{gps,battery}` per drone; marks a drone
  disconnected if telemetry goes stale for 5s (radio loss / crash), not just on
  rosbridge disconnect.
- `src/components/MapView.tsx` — live drone markers, connected/disconnected
  styling, boundary polygon + planned-waypoint overlay from a loaded mission file.
- `src/components/DroneStatusPanel.tsx` — per-drone GPS/altitude/battery cards +
  rosbridge connection state.

**Bugs found and fixed while building it** (both confirmed via a headless-Chromium
screenshot pass, not just `tsc`):
- Passing `icon={undefined}` explicitly to `react-leaflet@5`'s `Marker` overrides
  Leaflet's own default-icon assignment instead of falling back to it — crashes
  with `Cannot read properties of undefined (reading 'createIcon')`. Only surfaced
  once live GPS data actually populated a marker; fixed by always passing an
  explicit icon (`connectedIcon` / `disconnectedIcon`).
- `ROSLIB.Message` doesn't exist in the installed `roslib` runtime (a
  `@types/roslib` vs. actual-package mismatch) — `Topic.publish()` just takes a
  plain object. Fixed in `useMissionControl.ts`.
- The initial Vite scaffold's `index.css` constrained `#root` to a fixed
  `1126px` centered column — replaced with a plain full-viewport reset so the
  GCS layout (sidebar + full-bleed map) actually fills the screen.

## 7. Mission-file loader

The organisers don't release their boundary/mission file format until competition
setup (Mission Brief §3), so a placeholder JSON schema was defined instead —
documented in `DOCUMENTS/mission_file_schema.md`, with the parsing layer
(`gcs/src/mission/parseMissionFile.ts`) isolated so only that file needs to change
once the real format is known.

**Flow implemented**, reusing the same `WaitForEvent`-on-a-topic convention the
stock behavior tree already uses:
1. Operator clicks **Load Mission File** in the GCS → parsed client-side → published
   to `/gcs/mission_load` (`std_msgs/String`, JSON payload).
2. `project_gazebo/mission_file_executor.py` (a new ROS 2 node, generic across any
   drone count/waypoint list — unlike the hardcoded `mission.py`/`mission_swarm.py`
   examples) receives and stores it.
3. Operator clicks **Start Mission** → published to `/gcs/mission_start`. The mission
   brief explicitly permits this as the one non-manual trigger ("mission start" is
   not manual intervention).
4. The executor arms, takes off, and flies each drone's file-defined GPS waypoint
   list (via `DroneInterfaceGPS.go_to.go_to_gps_point`, fixed in §2) fully
   autonomously, publishing progress to `/gcs/mission_status` for the GCS to display.

**Verified end-to-end**, driven entirely through the actual browser UI (headless
Chromium via Playwright, not just curl/ros2 CLI): loaded a 2-drone test mission file
through the file picker, clicked Start, and watched — both in the GCS UI and in the
ROS logs — `loaded → starting → taking_off → running (waypoint 1/3, 2/3, 3/3) →
landing → complete`, with both drones' map markers moving to match. Zero manual
waypoint entry after the file load, matching Phase 1's exit criterion.

**Known gap, scoped out of Phase 1 deliberately:** the mission-status stream isn't
latched, so a GCS page reload mid-mission loses the boundary/waypoint overlay (ROS-side
status messages still arrive and update the live status text, just not the loaded
mission's map overlay). Not fixed now because it's a UX polish item, not a
functional blocker — flagging it so it doesn't get rediscovered as a surprise later.

---

## 8. Phase 1 status — checklist

| Phase 1 item | Status |
|---|---|
| Bring up 2 simulated drones with stock behaviors (takeoff, follow-reference, land) | ✅ Done — tested with 2 and 3 drones |
| Reproduce one behavior-tree mission end-to-end in sim | ✅ Done (`trees/square.xml`) |
| GCS frontend: connect, subscribe to gps/battery for both drones, render live positions | ✅ Done, verified visually via headless-browser screenshots |
| Mission-file loader: parse a boundary/mission file into an executable script | ✅ Done against a placeholder schema (real organiser format not yet published) |

**Exit criteria met:** the GCS shows two simulated drones moving on a map, driven
entirely by a loaded mission file, with no manual waypoint entry.
