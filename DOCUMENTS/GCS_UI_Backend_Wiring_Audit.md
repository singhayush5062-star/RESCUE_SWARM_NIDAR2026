# GCS UI → backend wiring audit

Every interactive control in the GCS, checked against the backend it claims to
drive, on a running 4-drone simulation. The method was not code reading: a
harness drives each control through rosbridge with exactly the payload the
component's own hook publishes, then waits for the backend evidence that
control is supposed to produce. PASS means the round trip happened.

Harness: `ui_wiring_test.py` (scratch, not committed — it drives the same
topics any operator action would).

Final result, on a clean 4-drone simulation: **16 PASS, 0 FAIL, 6 DEAD**.

```
[PASS] Load mission (MissionLoader)       | /gcs/mission_status -> loaded
[PASS] Altitude/speed inputs              | re-load accepted by backend
[PASS] Add random survivors               | survivor_runtime_0 spawned, success=true
[PASS] Survivor markers on map            | /gcs/survivors/list carries the placement
[PASS] ARM (DroneControlPanel)            | success=True
[PASS] DISARM (DroneControlPanel)         | success=True
[PASS] Place / randomize drone spots      | success=True
[PASS] Live GPS telemetry                 | 4/4 drones publishing
[PASS] Live battery telemetry             | 4/4 drones publishing
[PASS] LAUNCH SWARM (HeaderNavbar)        | mission_status -> starting
[PASS] Zone overlay on map                | 4 msgs on /gcs/mission/zone_allocation
[PASS] Planned lawnmower paths            | 1 msg on /gcs/mission/planned_paths
[PASS] Mission timer                      | 18 msgs on /gcs/mission/progress
[PASS] Detection counters                 | DetectionResult, conf 0.97
[PASS] Video feeds (VideoPanel)           | 4/4 annotated feeds live
[PASS] Console log stream                 | 67 lines from 13 sources
[DEAD] x6 -- see "Buttons that looked live" below; all now disabled in the UI
```

## What was broken

### 1. The map and telemetry panel showed synthetic drones, not the real ones

`App.tsx`'s `effectiveDrones` preferred real telemetry only when
`executionMode === 'HARDWARE'`. The default mode is `SIMULATION`, so on every
normal run the map, the drone markers and the whole SWARM TELEMETRY panel were
fed by a `setInterval` that flew four fake drones in circles — while Gazebo was
publishing genuine GPS the whole time (measured: 4/4 drones on
`/droneN/sensor_measurements/gps`).

The mode switch is about which *vehicles* fly, simulated or real hardware. It
is not a reason to discard telemetry; the simulator IS the ROS backend. Fixed
to prefer real telemetry in both modes, with the synthetic drones kept only as
an offline layout demo — and now reporting `connected: false`, so nothing
renders them as healthy aircraft.

Two related fakes went with it:

* `DroneStatusPanel` computed `isConnected = executionMode === 'SIMULATION' || drone.connected`,
  so in the default mode all four status dots were green and every badge read
  ONLINE with the entire backend down. It is `drone.connected` now.
* The same panel substituted `14 SATS`, `-62 dBm` and `EKF OK` for fields
  nothing publishes. An invented value formatted like a reading is
  indistinguishable from a measurement, so those render as `—`.

### 2. The console showed only its own echo

`LogsConsolePanel` was fed a local `useState` array seeded with two hard-coded
lines describing a startup nobody had verified, and appended to only by the
GCS's own click handlers. No backend log ever reached it.

Now wired end to end:

* **`nidar_gcs_bridge`** relays `/rosout` → `/gcs/log`, filtered to
  operator-relevant nodes (NIDAR's own, plus the AS2 flight behaviors),
  floored at INFO, and rate-capped at 20/s with an explicit
  "suppressed N messages" line so throttling is never silent. `/rosout` carries
  every node in the graph — 120 publishers in a 4-drone run — and the bulk is
  transform-listener noise, which is why the filter is an allow-list.
* **`useMissionLog.ts`** subscribes `/gcs/log`, buffers into a ref and flushes
  to React every 250 ms rather than setting state per line, and keeps the last
  500 entries.
* `App.tsx` merges backend lines with this browser's own actions, ordered by a
  numeric `sortKey`. The displayed timestamp comes from `toLocaleTimeString()`
  and cannot be sorted — on a 12-hour locale `"9:59:00 PM"` sorts before
  `"10:00:00 AM"`.
* The SOURCE dropdown offered `GCS`, `SwarmManager` and bare `drone0`, matched
  by string equality. Real sources are ROS node names — `mission_executor`,
  `detection_drone0`, `drone2.FollowPathBehavior` — so every option matched
  nothing. It is built from the sources actually present, and drone filtering
  is substring-based so picking `drone2` catches every node speaking for it.

Measured on a live mission: `/rosout` 3.5 msg/s, 0.8 KB/s; the relay forwarded
8 of 9 messages in a 20 s window, dropping the one outside the allow-list.

### 3. Buttons that looked live and commanded nothing

| control | what it did |
|---|---|
| RECALL (header) | wrote "RTL signal broadcast to all drones" to the log, published nothing |
| ABORT (header) | wrote "CRITICAL ABORT: Emergency landing commanded", published nothing |
| D-pad N/S/E/W/HOLD | fell through `handleNudge` to a `console.log` |
| EMERGENCY CUT MOTORS | no `onClick` at all |
| MAX GROUND SPEED slider | local state, never published |

`nidar_mission_executor` accepts `arm`, `disarm`, `takeoff` and
`set_launch_position` on `/nidar/drone_command`, and nothing else. Its
return-to-launch runs only as the last phase of a mission; there is no way to
trigger it, or to interrupt a running mission, from outside.

These are now **disabled with a tooltip naming what is missing**, and the two
log lines are gone. A log entry asserting a command that was never sent is
worse than silence, and a live-looking ABORT is worse than no ABORT — the
operator stops looking for another way to stop the aircraft. Restoring them is
a one-line change at each call site once the backend commands exist.

### 4. Numbers that were decoration

* Header badge read "N DETECTED" from the count of survivors the operator had
  *placed* in the simulator. It now shows detections (distinct ByteTrack ids)
  and, separately, a PLACED badge for ground truth.
* `MissionSummaryModal` took its duration from a browser `setInterval` that
  never reset, `survivorsCount={... || 2}` and `areaM2={... || 4500}`, and
  hard-coded `status: 'SUCCESSFUL_RETURN'` even when opened by ABORT. It now
  uses the backend's own monotonic mission clock, the real detection count, no
  fallbacks, and a real outcome.
* The modal only ever opened from the ABORT button, so a mission that ran to
  completion — the normal case — produced no report. It now opens on
  `complete` and on `error`.
* `VideoPanel`'s HUD printed `ALT: 25.4m`, `FPS: 15`, `LAT: 28.5412° N` over
  every feed regardless of the drone. It shows that drone's real altitude and
  latitude, and LIVE/NO SIGNAL.
* The THERMAL / EO_RGB toggle restyles the *synthetic placeholder* canvas
  only; there is one RGB camera per drone and no thermal sensor anywhere in
  the sim or on the airframe. It is hidden once a real feed is arriving, which
  is the only state where offering it implies a sensor switch that never
  happens.

## "The launch station does not work" was the same display bug

Reported symptom: create a launch station, the drones do not move into it;
place a drone manually, it does not move either.

The backend does all of it correctly. Driving the exact sequence
`handleSetLaunchSite` fires — `/gcs/mission_load`, then `/gcs/mission_load`
again with `drone_launch_positions`, then four `set_launch_position` commands,
all in one tick — against a launch site 55 m from the world origin:

```
=== BEFORE ===                        (distance from the new site)
  drone0: 28.6823975, 77.4997176        58.8 m
  drone1: 28.6823975, 77.4997503        56.6 m
  drone2: 28.6824264, 77.4997176        56.6 m
  drone3: 28.6824264, 77.4997503        54.3 m

=== STATUS REPLIES ===   all four: success=True  ok

=== AFTER ===
  drone0:  0.0 m from commanded spot -> MOVED
  drone1:  0.0 m from commanded spot -> MOVED
  drone2:  0.0 m from commanded spot -> MOVED
  drone3:  0.0 m from commanded spot -> MOVED
```

The drones moved 55 m in Gazebo, every time. What did not move was the **map**,
because it was rendering the synthetic ticker rather than real telemetry — the
`effectiveDrones` bug above. The operator sets a launch site, the drones
teleport, and the four markers stay exactly where they were, because those
markers were never connected to the drones in the first place.

One refinement went in on top of the original fix: `effectiveDrones` now keys
on `real.gps` rather than `real.connected`. A drone whose telemetry goes stale
is still a real drone at a real last-known position, and that position is the
most useful thing to show an operator who has just lost it. Keying on
`connected` sent it back to the mock's fixed start coordinate, so a drone that
had flown 55 m away would appear to teleport home the instant its feed
hiccuped. `MapView` already draws `connected: false` with a distinct marker.

## "drone1 is offline" is DDS half-discovery, and the launcher could not see it

Every recurrence traced to the same thing: FastDDS leaves a
`/dev/shm/sem.fastrtps_port*` lock per participant, `kill -9`ed processes never
release them, and past roughly 110 participants new ones start failing with
`Failed init_port ... open_and_lock_file failed`. The affected drone's process
is alive and its services are advertised, but its participant never completes,
so no telemetry reaches any subscriber. Which drone loses is not stable across
runs — drone2 one session, drone0 the next, drone1 for the operator.

A clean `./scripts/run_simulation.sh stop` clears the locks (measured: 388
entries in `/dev/shm` before, 0 after) and all four come back.

The launcher's startup gate could not detect this, because it checks that each
drone's OS *processes* exist — the right primary signal, instant and immune to
discovery latency, but blind to a process that is running and unreachable. It
now runs a short, **non-fatal** telemetry check before printing the ready
banner: subscribe to every drone's GPS for 30 s and name the ones that never
publish, with the remedy. Deliberately a warning rather than a teardown —
discovery on a loaded machine can legitimately take tens of seconds, and a
false failure here costs a full relaunch.

## Latent backend bug the test exposed

The first full run aborted 10 s after Start with `drone0 failed to enter
offboard`, with the drone sitting healthy on the pad.

Cause: the harness had exercised the manual ARM and DISARM buttons first —
exactly what the drone control panel is for. AS2's platform returns
`success=False` when asked to enter a state it is **already in**, and
`_call_bounded` retries, so the second attempt hits the same guard. The
mission could never start after a manual arm.

`_arm_and_offboard` now skips a transition the drone has already made, and on
a failed call believes the platform's own `PlatformInfo` over the call's
return value — the same principle `_call_physical_action` already applied to
takeoff. Note this was only visible *because* the arm and offboard failures
had just been split into separate messages; the old combined
"failed to arm/offboard" would have sent the search in the wrong direction.

## The coverage pattern was flown at the world origin, not the launch site

Found by watching the drones' actual positions during a mission launched from
a station 25 m north-east of the world origin. `mission_executor` reported:

```
[coverage] drone0: zone 8.0x8.0m | 2 lines, 4 waypoints | path 22m
[drone0] path compensated for gps origin (28.6823976,77.4997176);
         first waypoint earth (0.00,-1.21) |1.2m from plan origin
```

`first waypoint earth (0.00,-1.21)` is 1.2 m from the **world origin** — but
the arena was 25 m away. The swarm flew its lawnmower pattern around the wrong
point, and drone0 was measured at ENU (-7.95, -4.13) when its zone lay in
(17..33, 17..33).

`_compensate_gps_path` exists to invert AS2's `geodetic2enu(waypoint,
drone.gps.origin)` so a path survives each drone's own pinned spawn origin. It
computed the waypoint's ENU relative to `plan_origin` (the mission `home`) and
handed that to `enu_to_latlon` around the drone's origin. But the vector AS2
publishes goes into the shared `earth` frame, which is anchored at the **world
origin**. Measuring from `home` silently subtracts the launch site's own offset
from the world origin — so every waypoint came out exactly 25 m short.

Invisible until now because every prior test mission used the world origin as
its `home`; with `home == self.origin` the two readings are identical. It
appears the moment an operator puts a launch station anywhere else, which is
the normal case, and it is very likely the "drones launch and RTL spots are
different" report from an earlier session.

Fixed by anchoring the ENU at `self.origin` and deleting the `plan_origin`
parameter, which was actively harmful. `_return_to_launch` is the cross-check:
it feeds `enu_to_latlon` a position taken straight from
`self_localization/pose`, already earth-frame, with no home indirection — same
frame, same transform, and that path was measured landing within 0.2 m.

## Launcher could not see a drone that was running but unreachable

Added a short, **non-fatal** telemetry check before the ready banner: subscribe
to every drone's GPS for up to 30 s and name any that never publish, with the
remedy. Measured on a healthy sim: `All drones reporting telemetry` in 11 s.

Two things worth recording about building it:

* The first version subscribed with rclpy's default RELIABLE QoS. These are
  BEST_EFFORT sensor topics, so the subscription was **incompatible** and
  received nothing — the check would have declared all four drones dead on a
  perfectly healthy sim. rclpy says so out loud ("offering incompatible QoS.
  No messages will be received"), but the check's own output would just have
  looked like a failure. It uses `qos_profile_sensor_data` now, the same
  reasoning already documented in `detection_node.py`.
* Editing `run_simulation.sh` while an instance of it was still running
  corrupted that run — bash reads a script incrementally, so it executed the
  new bytes from the old offset and died with a syntax error on a file that
  `bash -n` passes cleanly. Stop the sim before editing its launcher.

## The mission hung on its first waypoint: an acceptance radius the
## controller cannot satisfy

Reproduced and root-caused by measurement rather than inference.

`FollowPathBehavior` marks a waypoint reached when the drone is within
`follow_path_threshold`, whose AS2 default is **0.2 m**. This airframe's
`pid_speed_controller` does not hold that. Measured, repeatedly: the drone
settles **0.235 - 0.24 m** from the waypoint and stays there, with the
controller still commanding ~0.24 m/s toward it and the position not
changing. Directly observed with the first waypoint at earth `x=25.02` and
drone0 parked at `x=24.78` for minutes, `FollowPathBehavior` reporting
`RUNNING` throughout.

Because 0.24 > 0.20, the waypoint is never accepted and the mission hangs on
its **first** one. Raising it at runtime does not help — AS2 behaviors read
the parameter at goal acceptance, so an already-running goal keeps the old
value.

Set `follow_path_threshold: 0.5` in `project_gazebo/config/config.yaml`, which
is the project's own AS2 config rather than the upstream default, so it
survives an AS2 update. It costs nothing in coverage: the camera footprint at
scan altitude is about 9 m across, so half a metre of waypoint slop is a
rounding error against the 20% line overlap the path planner already applies.

## A diagnostic that broke the thing it diagnosed — removed

Worth recording as a caution rather than as a feature. A startup telemetry
check was added to catch the "drone is running but unreachable" case the
process-based gate cannot see. It worked once (`All drones reporting
telemetry` in 11 s) and then hung the launcher indefinitely on
`Checking drone telemetry...` with a perfectly healthy sim behind it.

Two attempted fixes did not hold: a `timeout -s KILL` around the Python (the
process died on schedule, the shell still blocked) and writing to a file
instead of capturing with `$( )`. The remaining suspect is a descriptor
inherited by a background ROS daemon, but rather than keep guessing at a
convenience feature, it was **removed**. The information it provided is
already visible in the GCS, and a startup diagnostic that can prevent startup
is strictly worse than none.

Two general lessons from that attempt, both cheap to avoid:

* Do not edit `run_simulation.sh` while an instance is running. Bash reads a
  script incrementally, so it executes new bytes from the old offset and dies
  with a syntax error on a file that `bash -n` passes cleanly.
* `run_simulation.sh stop` kills the simulation processes but not the
  launcher shells, which sit in `while true; do sleep 3600; done`. Eight had
  accumulated across this session. They are harmless individually but make
  process inspection confusing; kill them by PID.

## End-to-end result: one small mission, every UI path

Launch station 25 m from the world origin, 16 x 16 m arena, 4 zones, 8 m
altitude, 3 survivors placed at runtime. Driven entirely through the GCS's own
rosbridge topics, in the GCS's own order.

```
[PASS] Load mission                     state -> loaded
[PASS] Launch station: drones relocate  4/4 within 1 m of commanded spot
[PASS] Place survivors                  3 survivors on /gcs/survivors/list
[PASS] Start mission                    state -> starting

  13:31:29  starting        13:33:45  returning
  13:31:37  taking_off      13:34:05  landing
  13:32:35  running         13:36:35  complete

[PASS] Zones published                  4 zone msgs
[PASS] Planned paths published          1 path msg
[PASS] Mission timer ticking            335 ticks, elapsed 305.6 s
[PASS] Video feeds                      4/4 annotated feeds
[PASS] Detections                       214 results, 20 unique track ids
[PASS] Console log stream               537 lines from 28 sources
[PASS] Coverage flown at launch site    all 4 drones within 2 m of home
[PASS] Mission reached completion       starting -> taking_off -> running
                                        -> returning -> landing -> complete
[PASS] RTL: back at launch station      drone0 0.12 m, drone1 0.03 m,
                                        drone2 0.02 m, drone3 0.02 m
===== 13/13 PASSED =====
```

The full phase sequence completes for the first time, the swarm flies the
arena at the operator's launch station rather than at the world origin, and
all four drones land within 12 cm of where they took off -- comfortably inside
the competition's 12 ft box.

## Known gaps, not fixed

* **RECALL / ABORT need backend commands.** The competition brief requires
  RTH and failsafe behaviour, so this is real work, not polish. It means
  adding `rtl` and `abort` actions to `DroneCommand` and — the hard part —
  a way to interrupt `_run_boundary_coverage_mission`, which currently blocks
  inside AS2 `follow_path` joins for minutes at a time.
* **Manual nudge / hold / motor cut** need a velocity-command path that does
  not exist. Worth deciding whether the competition needs manual flight at all
  before building it.
* **Drone cards in SWARM TELEMETRY are styled selectable** but `App` passes no
  `onSelectDrone`. The component self-gates correctly (cursor stays default,
  no handler fires), so nothing is broken — but wiring it to set the manual
  flight target would connect two panels that currently ignore each other.
