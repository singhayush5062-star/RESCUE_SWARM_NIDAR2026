# NIDAR RescueSwarm — Phase-wise Implementation Plan

**Starting point:** Working 3-drone Gazebo SITL with `mission_file_executor.py` + React/Leaflet GCS (sequential GoTo waypoints, KML/JSON loading, basic telemetry display)

**Target:** Full autonomous multi-drone SAR system scoring 600/600 with detection, geotagging, delivery, zone-based coordination, and competition-ready GCS

**Timeline:** 14 weeks (late August → end November 2026)

---

## Gap Analysis: Current State vs Target

| Capability | Current State | Target State |
|---|---|---|
| Simulation | 3-drone Gazebo, `as2_platform_gazebo` | 4-drone Gazebo, then Pixhawk hardware |
| Mission orchestration | `mission_file_executor.py` — sequential GoTo per drone, no coordination | `nidar_mission_manager` — zone splitting, coverage paths, delivery allocation, failsafes |
| Path execution | Sequential `GoTo` waypoint calls | `FollowPath` with trajectory generation for scan, `GoTo` for delivery hops |
| Waypoint generation | Manual JSON or KML-boundary-only (no auto waypoints) | Auto lawnmower/boustrophedon from boundary polygon |
| Custom messages | None — all communication via `std_msgs/String` JSON | `nidar_msgs` package with typed DetectionResult, SurvivorTag, PayloadStatus, MissionStatus, etc. |
| Detection | Not implemented | Onboard YOLO node on Jetson, TensorRT, annotated image stream |
| Geotagging | Not implemented | Pinhole projection from pixel bbox + RTK GPS + pose → lat/lon |
| Survivor tracking | Not implemented | Aggregation, deduplication (cross-drone), delivery assignment |
| Payload release | Not implemented | Action server controlling servo, kit tracking |
| GCS frontend | Map + telemetry + mission load/start/status | + video feeds, zone overlays, survivor markers, delivery tracking, health dashboard |
| Failsafes | FC-level RTL only (no software handling) | Abort/recall buttons, battery/link-loss reassignment, geofence monitoring |
| Camera | No camera in sim or GCS | Sim camera plugin → detection pipeline; real A12 on hardware |
| Hardware | Gazebo only | Pixhawk FC, Jetson companion, RTK GPS, telemetry radios, payload mechanism |
| Drone count | 3 (drone0, drone1, drone2) | 4 (drone_1 through drone_4) |

---

## Phase 0 — Foundation & Messages (Weeks 1–2) — ✅ COMPLETE (2026-08-24)

**Goal:** Establish the `nidar_msgs` package, add the 4th drone to simulation, switch from `/drone0`-style naming to `/drone_1`-style, and integrate a simulated camera into the Gazebo world.

**Naming decision:** kept `drone0`/`drone1`/`drone2`/`drone3` rather than migrating to `drone_1..4`. The rename is purely cosmetic (the competition doesn't see our internal namespace) and would have touched every file built so far — GCS, `mission_file_executor.py`, configs, and three prior docs — for zero functional benefit. Revisit only if a real reason to rename shows up later.

### 0.1 — Create `nidar_msgs` package — ✅ Done

This unblocks every subsequent phase. All custom nodes depend on these types.

**Tasks:**

- Create a new `ament_cmake` package `nidar_msgs` in `src/nidar_msgs/`.
- Define all `.msg` files exactly as specified in the ROS2 Architecture doc: `DetectionResult.msg`, `SurvivorTag.msg`, `PayloadStatus.msg`, `MissionBoundary.msg`, `ZoneAllocation.msg`, `MissionStatus.msg`, `DeliveryStatus.msg`.
- Define `PayloadRelease.action` (Goal: target lat/lon + survivor_id; Result: success/fail + drop_distance_m; Feedback: approach progress percentage).
- Add `geographic_msgs` as a build dependency (for `GeoPoint` in `MissionBoundary` and `ZoneAllocation`).
- Write `CMakeLists.txt` and `package.xml` with `rosidl_generate_interfaces()`.
- Build with `colcon build --packages-select nidar_msgs` and verify with `ros2 interface show nidar_msgs/msg/SurvivorTag`.

**Deliverable:** `nidar_msgs` compiles and all message types are introspectable via `ros2 interface show`. ✅ Verified live — `colcon build --packages-select nidar_msgs` succeeds, `ros2 interface show nidar_msgs/msg/SurvivorTag` and `.../action/PayloadRelease` both print full field lists, `ros2 interface package nidar_msgs` lists all 7 messages + the action.

### 0.2 — 4th drone + namespace alignment — ✅ Done (naming kept as-is, see decision above)

- ~~Update `world_swarm.yaml` to include `drone_3`~~ → added `drone3` (kept `drone0..2` naming, see decision note above) to `project_gazebo/config/world_swarm.yaml`.
- `run_simulation.sh` needed no change — it already auto-detects every drone in `world_swarm.yaml` via `utils/get_drones.py` rather than hardcoding a count, so it picked up drone3 automatically.
- GCS: `App.tsx`'s `DRONE_NAMESPACES` extended to include `drone3` (4 entries). **Not** made fully config-driven as the plan's literal wording asked — it's still a hardcoded array, just a longer one. Revisit if the drone count needs to change at runtime (e.g. from a loaded mission file) rather than at build time.
- Verified live: all 4 drones (`drone0`-`drone3`) show up in `ros2 node list` with a full, identical AS2 node set each.

**Deliverable:** 4-drone SITL running end-to-end with existing mission flow. ✅ Verified live — `project_gazebo/missions/phase0_4drone_verify.json` (2 waypoints × 4 drones) run through `mission_file_executor.py` exactly as the GCS would trigger it: `loaded → starting (arming 4 drone(s)) → taking_off → waypoint 1/2 → waypoint 2/2 → landing → complete`, no errors.

### 0.3 — Simulated camera in Gazebo — ✅ Done, with one path difference from the plan

- Added the camera via AS2's stock `gimbal_speed` + `hd_camera` payload pair (the same pattern already used in the single-drone `world.yaml` example) to all 4 drones in `world_swarm.yaml`, rather than authoring a new raw camera SDF sensor by hand — reuses a payload type AS2 already ships and bridges correctly, no new SDF/bridge code needed.
- **Path difference:** the resulting topic is `/<drone_id>/sensor_measurements/gimbal/camera/image_raw` (+ `camera_info`), not the flat `/<drone_id>/sensor_measurements/camera` the plan and the ROS2 Architecture doc assume. Whatever consumes this in Phase 2 (`nidar_detection`) needs to subscribe to the gimbal path, or a topic remap can be added in `tmuxinator/aerostack2.yaml` later if the flat path is worth preserving for that doc's accuracy.
- Verified live: `ros2 topic list -t` shows `image_raw` (`sensor_msgs/msg/Image`) and `camera_info` (`sensor_msgs/msg/CameraInfo`) for all 4 drones simultaneously.

**Deliverable:** ✅ confirmed via live topic list (not yet checked in `rqt_image_view` specifically, since headless — topic existence + correct type is the meaningful signal at this stage; visual inspection matters once Phase 2's detection node needs real image content).

### 0.4 — Simulated survivors in Gazebo world — ✅ Done

- Added 8 static "survivor" placeholders (`project_gazebo/models/survivor/survivor.sdf` — a simple orange torso box + head sphere, static, no physics) as `objects:` entries in `world_swarm.yaml` (the `World` schema supports `objects` alongside `drones`, spawned via the same launch path).
- Custom model lives outside the AS2 install tree (`project_gazebo/models/`, found via `GZ_SIM_RESOURCE_PATH`, now exported by `scripts/setup_nidar_ros.sh`) so it survives a colcon rebuild.
- **All 4 drone spawn points and all 8 survivors fall within a 30m × 30m area** centered on the world origin (every x/y coordinate in `[-15, 15]`) — the ground-truth reference for later detection/geotagging accuracy validation (Phase 3.4) is the `world_swarm.yaml` file itself (each survivor's `xyz` is its ground-truth local-ENU position; converts to lat/lon via the same origin the state estimator uses).
- Verified live: `gz model --list` shows `survivor_0` through `survivor_7` alongside all 4 drones; spot-checked `gz model -m survivor_0 -p` returns the exact configured pose `[8.0, 10.0, 0.0]`.

**Deliverable:** ✅ confirmed live — 8 visible survivor models at known, recorded positions, all within the requested 30×30m arena.

---

## Phase 1 — Coverage Path Planning & Zone Splitting (Weeks 2–3)

**Goal:** Replace "operator provides per-drone waypoints in JSON" with "operator provides boundary polygon, system auto-generates coverage waypoints per drone."

This is where the system starts earning the **Multi-Drone Collaboration** (50 pts) and **Fast Completion** (50 pts) scoring criteria.

### 1.1 — Zone splitter module

- Create `nidar_mission_manager/zone_splitter.py`.
- Input: list of `GeoPoint` boundary vertices (the polygon from the organiser's KML/boundary file).
- Output: N sub-polygons (one per drone), each represented as a list of `GeoPoint` vertices.
- Algorithm: simple axis-aligned slicing. Compute the bounding box of the polygon, determine the longer axis (lat or lon span), divide into N equal strips along that axis, clip each strip against the original polygon. This handles convex and mildly concave boundaries. For complex concave shapes, a Voronoi partition or longitudinal-strip approach works.
- Unit tests: provide a known rectangular boundary, verify 4 equal-area zones with no overlap and full coverage.

### 1.2 — Lawnmower path planner

- Create `nidar_mission_manager/path_planner.py`.
- Input: a sub-polygon (zone) + scan altitude + camera FOV + desired overlap percentage.
- Output: ordered list of GPS waypoints forming a boustrophedon (lawnmower) pattern that covers the zone.
- Computation: from the zone polygon, compute the swath width (ground footprint width at scan altitude given camera FOV, minus overlap), generate parallel flight lines spaced by swath width, clip lines to zone boundary, connect with alternating direction (boustrophedon).
- Parameters: `scan_altitude_m` (default 25), `camera_hfov_deg` (default 60), `overlap_pct` (default 20), `flight_line_heading` (default: along shorter zone axis for fewer turns).
- Unit test: verify waypoint count, coverage area, and that no waypoint falls outside the zone polygon.

### 1.3 — Integrate zone splitter + path planner into mission_file_executor

- When a KML boundary-only file is loaded (no per-drone waypoints), the executor now:
  1. Calls `zone_splitter.split(boundary, num_drones)` → per-drone zones.
  2. Calls `path_planner.generate(zone, altitude, fov, overlap)` → per-drone waypoints.
  3. Publishes zone allocations as `/gcs/mission/zone_allocation` (using `ZoneAllocation.msg` from `nidar_msgs`).
  4. Proceeds with the existing takeoff → fly → land flow, but now using `FollowPath` instead of sequential `GoTo` for the scan phase.
- The mission_file_executor starts evolving toward the `nidar_mission_manager` role.

### 1.4 — GCS zone overlay

- Update `MapView.tsx` to subscribe to `/gcs/mission/zone_allocation` and render each drone's zone as a color-coded polygon overlay on the Leaflet map.
- Show planned lawnmower paths as dashed polylines within each zone (already partially implemented for JSON waypoints — extend to auto-generated ones).

**Deliverable:** Load a KML boundary → 4 zones auto-generated → lawnmower paths visible on map → drones fly coverage pattern in sim → land.

---

## Phase 2 — Detection Pipeline (Weeks 3–5) — ⚙️ PIPELINE COMPLETE (2026-08-27)

**Status:** the full camera → inference → `DetectionResult` → GCS path is built,
wired, and verified live (102 detections on `/gcs/detections`, boxes centred on a
survivor and scaling correctly with altitude). Running on stock YOLO26 weights as a
placeholder; retrained overhead-person weights drop in by copying one file
(`project_gazebo/models/detection/nidar_person.pt`). §2.1 model training is being
handled separately. Full write-up, measurements, and deviations:
**`DOCUMENTS/Phase2_Detection_Notes.md`**.

Two blockers had to be fixed first, both recorded in that document: the camera was
mounted looking at the horizon rather than nadir, and stock COCO weights score a
top-down person at 0.03–0.12 confidence even when the figure fills the frame (a
viewpoint gap, not a resolution one — which is what makes §2.1 a hard prerequisite
for accuracy rather than later polish).

**Goal:** Build the `nidar_detection` node that runs YOLO inference on the camera stream and publishes `DetectionResult` messages. This is the highest-scoring capability (250 pts for detection + geotagging).

### 2.1 — Model training / fine-tuning

- Collect and prepare training data from VisDrone, HERIDAL, and SARD datasets (aerial human detection, nadir/oblique views, 20–50m altitude).
- Fine-tune YOLOv8n or YOLOv9t on this combined dataset (person class only — single-class detector simplifies everything).
- Export to ONNX format.
- Target metrics: >0.7 mAP@0.5 on a held-out test set of aerial person images.
- Export to TensorRT `.engine` format for Jetson deployment (FP16).

**Note:** Model training can run in parallel on a GPU workstation while other phases proceed. Start this in Week 1 and iterate.

### 2.2 — Detection node (sim version)

- Create `nidar_detection/detection_node.py` as a standard `rclpy` node (NOT an AS2 behavior — it runs continuously).
- Subscribe to `/<drone_id>/sensor_measurements/camera` (`sensor_msgs/Image`).
- Run YOLO inference on each frame (in sim: use ONNX runtime on CPU/GPU; on hardware: TensorRT on Jetson).
- For each detection with confidence above threshold:
  - Assign a `detection_id` (monotonic counter per drone).
  - Publish `DetectionResult.msg` to `/<drone_id>/detection/results`.
- Draw bounding boxes on the image and publish annotated frame to `/<drone_id>/detection/image_annotated`.
- Parameters: `model_path`, `confidence_threshold` (default 0.5), `nms_threshold` (default 0.45), `input_size` (default 640), `device` (cpu/cuda).
- Throttle inference rate to match available compute (e.g., 2–5 FPS in sim, 5–10 FPS on Jetson).

### 2.3 — Detection node integration test in sim

- Fly a single drone over the simulated survivors from Phase 0.4.
- Verify `DetectionResult` messages appear on the topic with correct bounding boxes.
- Tune confidence threshold against sim camera images (sim textures differ from real aerial images — threshold may need separate sim vs real values).
- Verify annotated image stream displays bounding boxes in `rqt_image_view`.

### 2.4 — GCS camera feed panel

- Add a `VideoPanel` component to the GCS React app.
- Subscribe to `/<drone_id>/detection/image_annotated` (or `sensor_measurements/camera` if detection isn't running) via rosbridge.
- Display 4-up grid of live video feeds (one per drone).
- Use `roslib`'s `compressed_image_transport` or convert Image messages to base64 JPEG on a bridge node for bandwidth efficiency.
- Handle feed disconnection gracefully (show "No Signal" placeholder).

**Deliverable:** Drones fly lawnmower pattern → YOLO detects simulated survivors → bounding boxes visible in GCS video panel → `DetectionResult` messages on ROS2 topics.

---

## Phase 3 — Geotagging Pipeline (Weeks 5–6)

**Goal:** Build the `nidar_geotag` node that converts pixel-space detections into world-frame GPS coordinates, and build the survivor aggregation system on the GCS side.

### 3.1 — Geotag node

- Create `nidar_geotag/geotag_node.py` as a standard `rclpy` node.
- Subscribe to:
  - `/<drone_id>/detection/results` (DetectionResult)
  - `/<drone_id>/sensor_measurements/gps` (NavSatFix)
  - `/<drone_id>/self_localization/pose` (PoseStamped)
- On each `DetectionResult`:
  1. Get the latest GPS position and drone pose (cache via subscribers with `best_effort` QoS).
  2. Compute the 3D ray from the camera through the bbox center pixel using camera intrinsics (pinhole model: `ray_cam = K_inv @ [u, v, 1]`).
  3. Transform the ray from camera frame to world frame using the drone's pose (rotation from quaternion) and known camera mount transform (pitch angle from nadir).
  4. Intersect the ray with the ground plane (z = 0 in local frame, or the known terrain altitude).
  5. Convert the intersection point from local ENU to GPS coordinates using the origin from `set_origin`/`get_origin`.
  6. Assign a `survivor_id` (or reuse if within dedup radius of a previous detection from this drone).
  7. Publish `SurvivorTag.msg` to `/<drone_id>/geotag/survivors`.
- Parameters: `camera_intrinsics` (fx, fy, cx, cy), `camera_mount_pitch_deg`, `min_confidence_threshold`, `local_dedup_radius_m` (default 3.0).

### 3.2 — Survivor aggregator (GCS-side)

- Create `nidar_mission_manager/survivor_aggregator.py`.
- Subscribe to `/<drone_id>/geotag/survivors` from all drones.
- Maintain a global survivor list with deduplication: if a new SurvivorTag's lat/lon is within `global_dedup_radius_m` (default 5.0) of an existing entry, merge (update confidence, keep the tag with higher confidence, note multiple detecting drones).
- Publish the deduplicated list on `/gcs/survivors/aggregated` as a `SurvivorList` (array of `SurvivorTag`).
- This module is integrated into the evolving `mission_manager_node.py`.

### 3.3 — GCS survivor markers

- Update `MapView.tsx` to subscribe to `/gcs/survivors/aggregated`.
- Render each survivor as a pin/icon on the Leaflet map with:
  - Survivor ID label.
  - Color coding: red = detected/undelivered, yellow = delivery en route, green = delivered.
  - Click popup showing: lat/lon, confidence, detecting drone, delivery status.

### 3.4 — Geotagging accuracy validation

- Fly drones over known-position simulated survivors.
- Compare geotagged lat/lon against ground truth from Phase 0.4.
- Target: <3m positional error (well within Zone A scoring at competition).
- If error is too high, debug: is it the detection bbox jitter, the pose delay, or the projection math?
- Log results for the test report.

**Deliverable:** Detection → geotag → survivor markers on GCS map with <3m accuracy against ground truth in sim.

---

## Phase 4 — Mission Manager Refactor (Weeks 6–8)

**Goal:** Evolve `mission_file_executor.py` into the full `nidar_mission_manager` with scan-then-deliver logic, delivery task allocation, and proper mission status reporting. This phase targets the **Delivery Accuracy** (200 pts) scoring.

### 4.1 — Refactor into `nidar_mission_manager` package

- Create the `nidar_mission_manager` package with proper ROS2 Python package structure.
- Move `zone_splitter.py` and `path_planner.py` (from Phase 1) into this package.
- Move `survivor_aggregator.py` (from Phase 3) into this package.
- Create `task_allocator.py` (new).
- Create `mission_manager_node.py` as the main orchestration node, replacing `mission_file_executor.py`.

### 4.2 — Mission state machine

Implement the full mission lifecycle as an explicit state machine in `mission_manager_node.py`:

```
IDLE → SETUP → ARMED → SCANNING → DELIVERING → RTL → COMPLETE
                                               ↗
                    ABORT ──────────────────────→ LANDING
```

- **IDLE:** Waiting for mission file load.
- **SETUP:** Boundary parsed, zones split, paths generated, zone allocations published. Waiting for operator start.
- **ARMED:** All drones armed and in offboard mode. Transition to SCANNING.
- **SCANNING:** All drones executing FollowPath through their lawnmower waypoints. Detection and geotagging active. Each drone transitions individually to DELIVERING when its scan path completes.
- **DELIVERING:** Drone assigned to deliver kits to nearest undelivered survivors. Uses GoTo → descend → PayloadRelease action → ascend → next delivery or RTL.
- **RTL:** Drone returning to launch point. GoTo(home) → Land.
- **COMPLETE:** All drones landed. Final report published.
- **ABORT/LANDING:** Emergency path — cancel all behaviors, command immediate land.

Publish state transitions to `/gcs/mission_status` using `MissionStatus.msg` (no longer JSON strings).

### 4.3 — Delivery task allocator

- Create `nidar_mission_manager/task_allocator.py`.
- Input: list of undelivered survivors (from aggregator), list of available drones (with current position, kits remaining, current state).
- Algorithm: nearest-available assignment.
  - For each undelivered survivor, find the drone that is: (a) in DELIVERING or finished SCANNING state, (b) has kits remaining, (c) is nearest by geodesic distance.
  - Assign that drone → that survivor. Mark survivor as `delivery_assigned`.
  - If a drone becomes unavailable (RTL due to battery, crash, link loss), unmark its pending assignments and re-run allocation.
- Output: list of (drone_id, survivor_id, target_lat, target_lon) delivery tasks.

### 4.4 — Delivery execution sequence

For each delivery task assigned to a drone:

1. `GoTo` the survivor's GPS position at scan altitude.
2. Descend to drop altitude (e.g., 5–8m) via a second `GoTo` at lower altitude.
3. Call `/<drone_id>/payload/release` action (Phase 5) with target lat/lon and survivor_id.
4. Wait for action result (success/fail + measured drop distance).
5. Ascend back to transit altitude.
6. Update `/gcs/delivery/status` with `DeliveryStatus.msg` (status: DROPPED, drop_distance_m).
7. Check: more undelivered survivors assigned to this drone? → repeat. Else → RTL.

In simulation (before Phase 5 builds the real payload node), stub the payload action server: immediately return success with 0m drop distance.

### 4.5 — GCS delivery tracking

- Add delivery status display to the GCS:
  - Subscribe to `/gcs/delivery/status`.
  - Show a delivery log panel: table of (survivor_id, assigned_drone, status, drop_distance).
  - Update survivor markers on map to reflect delivery status (color change).
- Update `MissionLoader.tsx` to show the new typed `MissionStatus` (phase, elapsed time, survivors detected, deliveries complete, active drones).

### 4.6 — FollowPath integration

- Replace sequential `GoTo` calls during the scan phase with a single `FollowPath` action call per drone.
- Pass all lawnmower waypoints as a path to the `FollowPathBehavior` action server.
- This enables the trajectory generator to produce smooth flight through waypoints instead of stop-and-go at each point — faster scan, better for the 50-pt fast completion bonus.

**Deliverable:** Full scan-then-deliver mission cycle in sim: auto zone split → lawnmower scan → detections geotagged → delivery tasks assigned → drones fly to survivors → (stub) payload drop → RTL → mission complete on GCS.

---

## Phase 5 — Payload System (Weeks 7–9)

**Goal:** Build the `nidar_payload` node (action server for payload release) and integrate the physical release mechanism. This runs in parallel with Phase 4.

### 5.1 — Payload action server

- Create `nidar_payload/payload_node.py` as an `rclpy` action server.
- Action: `/<drone_id>/payload/release` using `PayloadRelease.action`.
- Maintains internal state: `kits_remaining`, `kits_total` (from config).
- Publishes `/<drone_id>/payload/status` (`PayloadStatus.msg`) on every state change.
- On goal received:
  1. Validate: kits_remaining > 0? If not, abort with failure.
  2. Publish feedback: "approaching" (the motion to the drop point is handled by the mission manager's GoTo call, not by this node — this node just manages the release).
  3. Actuate the release mechanism: send a servo command (GPIO PWM via Jetson GPIO library, or MAVLink `DO_SET_SERVO` via the FC).
  4. Decrement `kits_remaining`.
  5. Compute drop distance (compare drone GPS at release moment vs target lat/lon).
  6. Return result: success, drop_distance_m.

### 5.2 — Simulated payload in Gazebo

- For sim testing, the payload node doesn't need real GPIO — instead, on release command:
  - Log the release event.
  - Optionally spawn a small box model in Gazebo at the drone's current position (visual confirmation of drop).
  - Return success with computed drop distance.

### 5.3 — Physical release mechanism design

- Servo-actuated release mechanism: a hinged tray or gripper holding the payload box (20×10×5cm, 200g).
- Each drone carries 2–3 kits in stacked or side-by-side mounts, each with an independent servo release.
- PWM control via Jetson GPIO (preferred for simplicity) or FC auxiliary servo output channel.
- Bench-test the mechanism: servo travel, release reliability, vibration resistance.

### 5.4 — Integration test

- Run the full mission cycle in sim with the payload node active (sim stub mode).
- Verify: kit count decrements, PayloadStatus updates, delivery status flows to GCS, drop distance computed correctly.

**Deliverable:** Payload action server integrated into mission cycle; kits tracked on GCS; physical mechanism bench-tested.

---

## Phase 6 — Failsafes & Safety (Weeks 8–9)

**Goal:** Implement the software-level failsafe handling required by Rule 8.19: RTH, comm-loss recovery, low-battery, geofence, mission abort. Penalties for crashes (-50), geofence breach (-20), and landing outside zone (-10) make this phase critical for protecting scored points.

### 6.1 — Abort and recall buttons

- Wire the GCS "Abort" and "Recall" buttons to publish `/gcs/emergency/abort` and `/gcs/emergency/recall`.
- In the mission manager:
  - **Abort:** Cancel all active behavior action goals (via `cancel_goal_async()`). Command all drones to `Land` immediately at current position.
  - **Recall:** Cancel all active behavior action goals. Command all drones to `GoTo(home_position)` then `Land`.
- Test: mid-mission abort → all drones land within 10 seconds. Recall → all drones fly home and land.

### 6.2 — Battery monitoring

- Subscribe to `/<drone_id>/sensor_measurements/battery` in the mission manager.
- Define thresholds: `battery_warning_pct` (30%), `battery_critical_pct` (20%).
- On warning: mark drone for early RTL after current task completes. Reassign its pending deliveries.
- On critical: immediately cancel current behavior, command RTL + Land. Reassign deliveries.
- Display battery warnings in GCS status panel (color-coded: green > 50%, yellow 30–50%, red < 30%).

### 6.3 — Communication loss detection

- In the mission manager, track the timestamp of the last received message from each drone (any topic — GPS, battery, platform/info).
- If no message received for `comm_loss_timeout_sec` (default 10s), mark drone as "COMM_LOST".
- On COMM_LOST: the FC's own failsafe should trigger RTL (configured in PX4/ArduPilot params). The mission manager reassigns the drone's pending deliveries to remaining drones.
- Display COMM_LOST status prominently in GCS.

### 6.4 — Geofence enforcement

- In the mission manager, before sending any GoTo or FollowPath command, validate that all waypoints fall within the mission boundary polygon.
- During flight, monitor each drone's GPS position against the boundary.
- If a drone approaches within `geofence_margin_m` (default 10m) of the boundary, log a warning.
- If a drone exits the boundary (by GPS), trigger RTL for that drone immediately.
- FC-level geofence is a second line of defense (configure in PX4/ArduPilot params).

### 6.5 — Drone loss reassignment

- When any drone is lost (COMM_LOST, battery-critical RTL, crash detected via no GPS updates + altitude drop):
  - Remove it from the active drone list.
  - Unmark all its pending delivery assignments.
  - Re-run the task allocator with the remaining drones.
  - Update mission status: `active_drones` decremented.

### 6.6 — Landing zone validation

- Before commanding land, validate that the drone's current position is within the 12×12 ft (3.66×3.66m) launch zone.
- If not, GoTo(home) first, then land.
- Stagger landings if multiple drones arrive at the landing zone simultaneously (avoid collision risk).

**Deliverable:** All failsafes tested in sim: abort, recall, battery-critical RTL, comm-loss handling, geofence enforcement. No penalty conditions triggered during clean runs.

---

## Phase 7 — GCS Polish & Rule 8.14 Compliance (Weeks 9–11)

**Goal:** Ensure the GCS displays every item mandated by Rule 8.14, with a polished operator experience. This phase targets the **Single GCS Interface** (50 pts) scoring — it's binary (all-or-nothing).

### 7.1 — Rule 8.14 compliance checklist

| Required Display | Implementation | Status |
|---|---|---|
| Mission status | `MissionStatus.msg` → status bar showing phase, elapsed time, progress | Phase 4 |
| Live camera feed from each drone | 4-up video grid from `detection/image_annotated` | Phase 2 |
| Position of each drone | Drone markers on Leaflet map from GPS | Current (done) |
| Assigned search area per drone | Zone polygon overlays on map | Phase 1 |
| Detected and geotagged survivor locations | Survivor markers on map from aggregated list | Phase 3 |
| Survival-kit delivery status | Delivery log table + marker color coding | Phase 4 |
| Communication and system health | Per-drone connection status, battery level, COMM_LOST alerts | Phase 6 |
| Consolidated mission progress | Summary panel: X/10 detected, Y/10 delivered, Z drones active, elapsed time | Phase 4 |

### 7.2 — GCS layout redesign

Design a competition-ready layout with all panels visible simultaneously on a single screen:

```
┌──────────────────────────────────────────────────────────────────┐
│  NIDAR RescueSwarm GCS          [Mission Status Bar]    [Clock] │
├────────────────────────┬─────────────────────────────────────────┤
│                        │   Camera Feeds (2×2 grid)              │
│   Map View             │   ┌──────────┬──────────┐              │
│   (Leaflet)            │   │ Drone 1  │ Drone 2  │              │
│   - Drone positions    │   ├──────────┼──────────┤              │
│   - Zone overlays      │   │ Drone 3  │ Drone 4  │              │
│   - Survivor markers   │   └──────────┴──────────┘              │
│   - Flight paths       ├─────────────────────────────────────────┤
│                        │   Status Panel                         │
│                        │   Per-drone: battery, state, kits left  │
├────────────────────────┤   Delivery log table                   │
│  Controls              │   Health: conn status, alerts           │
│  [START] [ABORT] [RTL] │   Progress: 7/10 detected, 5/10 deliv  │
│  Mission file loader   │                                        │
└────────────────────────┴─────────────────────────────────────────┘
```

### 7.3 — Video feed optimization

- Camera images over rosbridge are bandwidth-heavy. Options:
  - **Option A:** Use `image_transport`'s `compressed` plugin to publish JPEG-compressed images; subscribe in GCS via rosbridge to the `compressed` topic.
  - **Option B:** Run a lightweight MJPEG HTTP server node on the GCS laptop that subscribes to the ROS image topics and serves frames over HTTP; GCS frontend displays via `<img>` tag.
  - Option B is recommended for lower latency and less rosbridge congestion.
- Target: 2–5 FPS per feed at 640×480 — sufficient for operator awareness.

### 7.4 — Mission report generation

- When mission completes, the mission manager publishes a final `MissionStatus` with phase=COMPLETE.
- GCS displays a mission summary dialog:
  - Total survivors detected (X/10).
  - Total deliveries completed (Y/10).
  - Per-delivery drop distances and zone classification (A/B/C).
  - Mission elapsed time.
  - Any penalties incurred (geofence breaches, out-of-zone landings).
- This summary is what judges evaluate.

### 7.5 — `ros2 bag` recording

- Add `ros2 bag record` to the launch scripts, capturing all relevant topics for post-mission analysis.
- Topics to record: all `/drone_{n}/sensor_measurements/gps`, `/gcs/mission_status`, `/gcs/survivors/aggregated`, `/gcs/delivery/status`, all detection results, all geotag survivors.

**Deliverable:** GCS passes the Rule 8.14 checklist completely. Clean, readable layout. Video feeds working. Mission report generated on completion.

---

## Phase 8 — Hardware Integration (Weeks 10–13)

**Goal:** Transition from Gazebo simulation to real Pixhawk hardware. This phase runs in parallel with GCS polish.

### 8.1 — Single-drone Pixhawk bringup

- Swap `as2_platform_gazebo` for `as2_platform_pixhawk` (PX4) or `as2_platform_mavlink` (ArduPilot) in the launch config.
- Connect Pixhawk to companion computer (Jetson) via serial (TELEM2 port) or USB.
- Configure PX4/ArduPilot parameters:
  - COM_OBF_LOSS_T (offboard loss timeout).
  - NAV_RCL_ACT (RC loss action → RTL).
  - GF_ACTION (geofence action → RTL).
  - Battery failsafe thresholds.
- Switch state estimator from `ground_truth` plugin to `raw_odometry` plugin (uses FC's EKF).
- Basic flight test: arm → takeoff → hover → land via `as2_python_api` CLI.

### 8.2 — RTK GPS setup

- Configure RTK base station on the GCS laptop (u-blox base or similar).
- Configure rover modules on each drone's Pixhawk (GPS2 port).
- RTCM corrections via MAVLink injection or serial passthrough.
- Verify RTK fix in `sensor_measurements/gps` (status field = `STATUS_GBAS_FIX`).
- Test geotagging accuracy: fly over a known point, check geotagged position error.

### 8.3 — Camera driver

- Install and configure the A12 camera driver on Jetson.
- Publish to `/<drone_id>/sensor_measurements/camera` via a `camera_driver_node` (V4L2, GStreamer, or manufacturer SDK).
- Verify detection node processes real camera frames and produces detections.

### 8.4 — Telemetry radio link

- Configure telemetry radios (SiK radios, RFD900x, or equivalent) between each drone and the GCS laptop.
- Each drone needs its own radio link (or a mesh radio system).
- Verify ROS2 topic flow over the radio: GPS, battery, detection results, geotag survivors, payload status.
- Bandwidth test: can 4 simultaneous links carry telemetry + compressed video + detection data?
- If bandwidth is tight, prioritize: telemetry (small, critical) > detection results (small) > compressed video (large, nice-to-have). Video may need to be local-only (display on Jetson HDMI for testing, don't transmit).

### 8.5 — Payload mechanism integration

- Mount the servo release mechanism on the drone frame.
- Wire servo to Jetson GPIO or FC aux channel.
- Update `nidar_payload` node to use real GPIO/MAVLink commands instead of sim stubs.
- Test: command release → servo actuates → payload drops.

### 8.6 — Multi-drone hardware test

- Start with 2 drones, then 3, then 4.
- Run the full mission cycle on a small test area (e.g., 1 hectare with 2–3 "survivors").
- Verify: zone splitting, coverage flight, detection, geotagging, delivery, RTL, landing.
- Record `ros2 bag` for every test flight.

### 8.7 — Launch configuration for hardware

- Create `drone_onboard.launch.py` (replaces `sim_full_system.launch.py` for real hardware):
  - Launches per-drone: AS2 platform (pixhawk) + controller + state estimator + motion behaviors + detection + geotag + payload.
  - Parameterized by `drone_id`, serial port, radio channel.
- Create `gcs.launch.py`:
  - Launches: mission manager + GCS frontend + rosbridge.
- Create per-drone config files: `drone_1.yaml` through `drone_4.yaml`.

**Deliverable:** Full mission running on real hardware with 4 drones, real detection, real payload drops, real GPS coordinates.

---

## Phase 9 — Integration Testing & Trial Buffer (Weeks 13–14)

**Goal:** Full system integration tests, bug fixes, performance tuning. NO new features — this is fix-and-polish time.

### 9.1 — Full dress rehearsal

- Set up a 10-hectare equivalent test area (or the largest available space).
- Place 10 "survivors" (mannequins, team members lying down, or poster cutouts).
- Run the complete mission end-to-end with all 4 drones.
- Time the mission. Target: under 15 minutes for the 50-pt fast completion bonus.
- Score the run against the scoring rubric.

### 9.2 — Scoring breakdown analysis

After each rehearsal run, compute:

| Criteria | Max | Achieved | Notes |
|---|---|---|---|
| Detection + geotagging | 250 | ? | How many of 10 survivors detected and geotagged on GCS? |
| Delivery accuracy | 200 | ? | Per-drop: Zone A (<1m)=20, Zone B (<2m)=14, Zone C (<3m)=8 |
| Multi-drone collaboration | 50 | ? | All drones coordinated, single mission logic? |
| Single GCS interface | 50 | ? | All Rule 8.14 displays present? |
| Fast completion | 50 | ? | Under 15 minutes? |

Identify the weakest scoring area and prioritize fixes.

### 9.3 — Penalty mitigation

Review each penalty condition and verify countermeasures:

| Penalty | Mitigation |
|---|---|
| Landing outside zone (-10/drone) | Landing zone validation in mission manager (Phase 6.6) |
| Geofence breach (-20/instance) | Software + FC geofence (Phase 6.4) |
| Repeated geofence breach (-20/drone) | Same drone won't breach twice if RTL triggers on first |
| Manual intervention (-50/instance) | Only 3 buttons exposed (start, abort, recall). No other inputs possible. |
| Crash (-50/crash) | Conservative altitude, speed limits, staggered landing |

### 9.4 — Performance tuning

- **Detection speed:** If detection is too slow (drones fly past survivors before detection fires), reduce flight speed or increase inference rate.
- **Geotagging accuracy:** If drop distances are consistently >3m, investigate: bbox jitter (apply temporal smoothing / multi-frame confirmation), GPS delay (use the GPS timestamp, not current time), projection errors.
- **Mission time:** If over 15 minutes, optimize: increase flight speed, reduce overlap, skip already-scanned areas, start deliveries before all scanning is complete (interleave scan and deliver).
- **Delivery accuracy:** If drop distance is consistently >1m, lower the drop altitude or add a position-hold loop before release.

### 9.5 — 5-minute setup drill

Practice the competition setup sequence to fit within the 5-minute window:

1. **T+0:00** — Two team members begin: unpack and position drones on 12×12 ft pad (4 drones, pre-assembled, batteries pre-installed).
2. **T+1:00** — Power on all drones. FC boot, GPS acquisition, AS2 stack auto-launches on companion computer boot.
3. **T+2:00** — GCS laptop running. Operator loads the organiser's mission boundary file.
4. **T+2:30** — Mission manager auto-generates zones and paths. GCS displays zone overlays and planned paths.
5. **T+3:00** — Verify all 4 drones show "connected" on GCS. RTK fix acquired on all.
6. **T+4:00** — Payloads loaded (if not pre-loaded). Final check.
7. **T+5:00** — Ready. Operator presses "Start Mission."

Automate as much as possible: drone boots → AS2 launches → radios connect → GCS detects automatically. The only manual step after physical setup should be loading the boundary file and pressing start.

---

## Parallel Work Streams Summary

Not all phases are strictly sequential. Here's what can run in parallel:

| Weeks | Stream A (Mission Logic) | Stream B (Perception) | Stream C (Hardware) | Stream D (GCS) |
|---|---|---|---|---|
| 1–2 | Phase 0: msgs, 4th drone, sim camera | Model training starts | Component sourcing & ordering | — |
| 2–3 | Phase 1: zone split, path planner | Model training continues | Parts arriving | Phase 1: zone overlay on map |
| 3–5 | Phase 4 starts: mission manager refactor | Phase 2: detection node | Frame assembly begins | Phase 2: video panel |
| 5–6 | Phase 4 continues: state machine, delivery flow | Phase 3: geotag node | Single-drone bench test | Phase 3: survivor markers |
| 6–8 | Phase 4 completes: task allocator, FollowPath | Integration testing (det+geo) | Phase 8.1: Pixhawk bringup | Phase 4: delivery tracking |
| 7–9 | Phase 5: payload node | — | Phase 8.2–8.3: RTK, camera | — |
| 8–9 | Phase 6: failsafes | — | Phase 8.4–8.5: radios, payload mech | Phase 6: failsafe UI |
| 9–11 | — | — | Phase 8.6–8.7: multi-drone HW test | Phase 7: full GCS polish |
| 13–14 | Phase 9: integration & rehearsals | Phase 9: tuning | Phase 9: full hardware rehearsals | Phase 9: final fixes |

---

## Risk Register

| Risk | Impact | Likelihood | Mitigation |
|---|---|---|---|
| Hardware procurement delays | Pushes Phase 8 back, compresses testing | High | Order components in Week 1. Have backup suppliers identified. |
| YOLO detection fails on real aerial images | Detection score = 0 (250 pts lost) | Medium | Start model training early. Use VisDrone/HERIDAL/SARD. Test on real drone footage ASAP. |
| RF link can't carry 4 simultaneous drone feeds | No video on GCS, or telemetry drops | Medium | Prioritize telemetry over video. Use compressed topics. Test bandwidth early. |
| Geotagging accuracy >3m | Delivery scoring drops (Zone C only, or miss) | Medium | Multi-frame confirmation. RTK GPS. Temporal smoothing on bbox. |
| 5-minute setup not achievable | Penalty risk if not ready | Low-Medium | Practice weekly from Week 10. Automate everything possible. |
| Payload mechanism unreliable | Delivery score drops, crash risk (-50) | Medium | Bench-test extensively. Simple servo design. Redundant release trigger. |
| AS2 bugs on real hardware | Mission failure | Medium | Test single-drone hardware early (Week 10). Log everything. |

---

## Weekly Milestones

| Week | Milestone | Validation |
|---|---|---|
| 1 | `nidar_msgs` compiles, 4-drone sim, sim camera working | `ros2 interface show`, 4 drones fly square pattern |
| 2 | Zone splitter + path planner working | KML load → 4 zones + lawnmower paths visible on map |
| 3 | Detection node running in sim | DetectionResult messages on topics, annotated images visible |
| 4 | Detection node tuned, GCS video panel working | Bounding boxes on sim survivors, 4-up video in GCS |
| 5 | Geotag node working, survivor markers on GCS map | Geotagged positions within 3m of ground truth |
| 6 | Mission manager state machine complete | Full scan-then-deliver cycle in sim (stub payload) |
| 7 | Delivery task allocation working, FollowPath integrated | 10 survivors detected and "delivered" in sim |
| 8 | Payload node done, failsafes implemented | Abort/recall tested, battery failsafe tested in sim |
| 9 | GCS passes Rule 8.14 checklist | All required displays present and functional |
| 10 | Single-drone Pixhawk flight working | Takeoff → hover → land on real hardware |
| 11 | 2-drone hardware mission working | Detection + delivery on real hardware, 2 drones |
| 12 | 4-drone hardware mission working | Full mission, 4 drones, real area |
| 13 | Dress rehearsal #1 | Scored run against rubric |
| 14 | Dress rehearsal #2 + final fixes | Target: >500/600 score |
