# NIDAR RescueSwarm — Complete ROS2 Package Architecture

**Backend:** Aerostack2 (AS2) on ROS 2 Humble
**Frontend:** Custom GCS (your build)
**Fleet:** 4 drones, Pixhawk FC, Jetson companion computers

---

## PART 1 — Corrections to the proposed drone split

Your proposed configuration: 2 delivery+mapping drones (A12 camera) and 2 scout-only drones (stereo cameras). After cross-referencing with the scoring rubric and the AS2 capabilities, there are three issues with this plan.

### Problem 1: Pure-scout drones are a scoring liability

The scoring breakdown is:

| Criteria | Max points | What scores |
|---|---|---|
| Detection + geotagging | 250 | 25 per survivor correctly detected and geotagged on GCS map |
| Delivery accuracy | 200 | 8-20 per drop depending on zone accuracy |
| Multi-drone collaboration | 50 | Binary: all drones coordinated, or zero |
| Single GCS interface | 50 | Binary: unified display, or zero |
| Fast completion (under 15 min) | 50 | Binary: complete within half of 30 min, or zero |

Detection is worth 250 and delivery is worth 200. A scout-only drone contributes to detection but cannot contribute to delivery at all. With 10 survivors and only 2 delivery drones, each delivery drone must carry 5 kits (5 × 200g = 1 kg payload each) and fly 5 separate delivery sorties. That makes the delivery drones the bottleneck for the entire mission clock, threatening the 50-point fast-completion bonus.

**Correction:** Make at least 3 of 4 drones delivery-capable. Every drone detects; most drones also deliver. A drone that finishes its scan zone and has kits onboard can immediately deliver to the nearest tagged survivor without waiting for the 2 overloaded delivery drones to get around to it.

### Problem 2: Stereo cameras are wasted weight and compute for this mission

Human detection from 20–50 m altitude is a 2D object detection problem, not a 3D depth-estimation problem. A monocular camera running a YOLO-class detector is the standard approach for aerial person detection (VisDrone, HERIDAL, SARD datasets are all monocular). Stereo gives you dense depth maps, which are useful for obstacle avoidance or 3D scene reconstruction — neither of which is scored, and neither of which matters in an open-area flood-rescue scenario.

What stereo cameras cost you for zero scoring return:
- Double the sensor weight and power draw versus a single camera.
- Double the image data throughput on the companion computer and the RF link.
- Significant additional compute load for stereo matching, reducing what's available for detection inference.
- Additional calibration complexity that can fail in the field.

**Correction:** Equip all drones with the same monocular camera (your A12 or equivalent). RTK GPS (which you're already planning on all 4) gives you the geotagging accuracy you need — the camera's job is detection, not depth.

### Problem 3: Heterogeneous drone types add complexity for no scoring benefit

Having two different drone configurations (scout vs delivery) means two different AS2 launch configs, two different behavior trees, two different payload configs, two different weight profiles, and twice the integration testing. The scoring rubric gives zero points for drone specialization — it only cares about detection count, delivery accuracy, coordination, and speed.

**Correction:** Build one drone design, replicate it 4 times. Every drone gets: Pixhawk FC, Jetson companion, RTK GPS, monocular camera (A12), payload release mechanism with 2-3 kits. This halves your integration testing, halves your spare-parts inventory, and means any drone can cover any task.

### Recommended configuration

| Drone | Camera | RTK GPS | Payload capacity | Role |
|---|---|---|---|---|
| drone_1 | A12 (mono) | Yes | 2-3 kits | Scan zone A → detect → deliver → RTL |
| drone_2 | A12 (mono) | Yes | 2-3 kits | Scan zone B → detect → deliver → RTL |
| drone_3 | A12 (mono) | Yes | 2-3 kits | Scan zone C → detect → deliver → RTL |
| drone_4 | A12 (mono) | Yes | 2-3 kits | Scan zone D → detect → deliver → RTL |

10 survivors across 4 drones = 2-3 deliveries each. With 3 kits per drone (4 × 3 = 12), you have 2 spare kits for edge cases. Combined payload weight: 12 × 200g = 2.4 kg, leaving ~22.6 kg for the 4 airframes+batteries+electronics within the 25 kg collective limit.

---

## PART 2 — ROS2 namespace structure

Every drone runs its own full Aerostack2 stack, namespaced by drone ID. The GCS subscribes to all 4 namespaces from one process.

```
/drone_1/
    platform/info                              # PlatformInfo (connection, arming, offboard, fly status)
    sensor_measurements/camera                 # sensor_msgs/Image (A12 feed)
    sensor_measurements/gps                    # sensor_msgs/NavSatFix (RTK position)
    sensor_measurements/imu                    # sensor_msgs/Imu
    sensor_measurements/battery                # sensor_msgs/BatteryState
    self_localization/odom                      # nav_msgs/Odometry (state estimator output)
    self_localization/pose                      # geometry_msgs/PoseStamped
    self_localization/twist                     # geometry_msgs/TwistStamped
    actuator_command/pose                       # geometry_msgs/PoseStamped (to FC)
    actuator_command/twist                      # geometry_msgs/TwistStamped (to FC)
    actuator_command/thrust                     # as2_msgs/Thrust (to FC)
    motion_reference/pose                       # geometry_msgs/PoseStamped (controller input)
    motion_reference/twist                      # geometry_msgs/TwistStamped (controller input)
    platform/set_arming_state                   # service: std_srvs/SetBool
    platform/set_offboard_mode                  # service: std_srvs/SetBool
    platform/set_control_mode                   # service: as2_srvs/SetControlMode
    platform/takeoff                            # service: std_srvs/SetBool
    platform/land                               # service: std_srvs/SetBool
    TakeOffBehavior/_action/                    # action: as2_msgs/TakeOff
    GoToWaypointBehavior/_action/               # action: as2_msgs/GoToWaypoint
    FollowPathBehavior/_action/                 # action: as2_msgs/FollowPath
    LandBehavior/_action/                       # action: as2_msgs/Land

    # --- YOUR CUSTOM TOPICS (new) ---
    detection/results                           # custom: DetectionResult.msg
    detection/image_annotated                   # sensor_msgs/Image (bounding-box overlay for GCS feed)
    geotag/survivors                            # custom: SurvivorTag.msg
    payload/status                              # custom: PayloadStatus.msg
    payload/release                             # custom: PayloadRelease action

/drone_2/  ...  (identical structure)
/drone_3/  ...
/drone_4/  ...

# --- GCS-SIDE TOPICS (new, no drone namespace) ---
/gcs/
    mission/boundary                            # custom: MissionBoundary.msg (parsed organiser file)
    mission/status                              # custom: MissionStatus.msg (overall progress)
    mission/zone_allocation                     # custom: ZoneAllocation.msg (which drone gets which zone)
    survivors/aggregated                        # custom: SurvivorList.msg (deduped across all drones)
    delivery/status                             # custom: DeliveryStatusList.msg (per-survivor delivery tracking)
    emergency/abort                             # std_msgs/Bool (operator abort trigger)
    emergency/recall                            # std_msgs/Bool (operator recall-all trigger)
```

---

## PART 3 — ROS2 packages (what you build vs what AS2 provides)

### Packages provided by Aerostack2 (use as-is)

| AS2 Package | What it does | Your config |
|---|---|---|
| `as2_platform_pixhawk` or `as2_platform_mavlink` | Bridges FC ↔ ROS2 topics. Publishes all `sensor_measurements/*`, `platform/info`, subscribes to `actuator_command/*`. | Choose one based on your FC firmware (PX4 → pixhawk; ArduPilot → mavlink). Launch config only. |
| `as2_motion_controller` + PID Speed Controller plugin | Converts position/speed references into actuator commands the FC accepts. | Plugin selection via launch parameter. |
| `as2_state_estimator` + Raw Odometry plugin | Publishes `self_localization/odom`, `/pose`, `/twist` from FC sensor fusion. RTK GPS feeds in through the platform. | Plugin selection via launch parameter. |
| `as2_behaviors_motion` | TakeOff, Land, GoTo, FollowPath action servers. These are the behavior servers your mission logic calls. | Plugin selection (position vs trajectory) per behavior via launch. |
| `as2_behavior_tree` | XML-defined mission trees that sequence behaviors. | You write the tree XML; the executor is stock AS2. |
| `as2_python_api` | Python API wrapping all behavior action clients + telemetry subscriptions. Your GCS backend uses this. | Import and call from your GCS Python code. |

### Packages you build (custom)

#### 1. `nidar_msgs` — Custom message definitions

```
msg/
    DetectionResult.msg
        std_msgs/Header header
        uint32 detection_id
        float32 confidence
        float32 bbox_x          # pixel coordinates in camera frame
        float32 bbox_y
        float32 bbox_w
        float32 bbox_h
        string drone_id

    SurvivorTag.msg
        std_msgs/Header header
        uint32 survivor_id
        float64 latitude        # from geotag fusion
        float64 longitude
        float64 altitude
        float32 confidence
        string detecting_drone_id
        bool delivery_assigned
        bool delivery_complete

    PayloadStatus.msg
        std_msgs/Header header
        string drone_id
        uint8 kits_remaining
        uint8 kits_total

    MissionBoundary.msg
        std_msgs/Header header
        geographic_msgs/GeoPoint[] boundary_vertices

    ZoneAllocation.msg
        std_msgs/Header header
        string drone_id
        geographic_msgs/GeoPoint[] zone_vertices

    MissionStatus.msg
        std_msgs/Header header
        uint8 total_survivors_detected
        uint8 total_deliveries_complete
        float32 elapsed_time_sec
        uint8 active_drones
        uint8 phase                # SETUP / SCANNING / DELIVERING / RTL / COMPLETE

    DeliveryStatus.msg
        std_msgs/Header header
        uint32 survivor_id
        string assigned_drone_id
        uint8 status               # PENDING / EN_ROUTE / DROPPED / CONFIRMED
        float32 drop_distance_m    # distance from survivor for zone scoring
```

#### 2. `nidar_detection` — Onboard human detection node

Runs on each drone's companion computer. Subscribes to the camera topic, runs YOLOv8/v9 inference, publishes detections.

```
Subscribes:
    /<drone_id>/sensor_measurements/camera       (sensor_msgs/Image)

Publishes:
    /<drone_id>/detection/results                (DetectionResult.msg)
    /<drone_id>/detection/image_annotated         (sensor_msgs/Image)

Runs on:    Companion computer (Jetson)
Compute:    GPU inference via TensorRT
```

This is NOT an AS2 behavior — it's a continuously-running perception node. It does not control the drone's motion; it only observes and publishes. Making it a behavior would mean starting/stopping it via action calls, which adds latency and complexity for no benefit — you want detection running the entire time the drone is in the air.

#### 3. `nidar_geotag` — Onboard geotag fusion node

Runs on each drone's companion computer. Takes a detection (pixel bbox center) + drone RTK GPS + drone attitude + camera intrinsics, and projects the detection into a world-frame lat/lon.

```
Subscribes:
    /<drone_id>/detection/results                (DetectionResult.msg)
    /<drone_id>/sensor_measurements/gps          (sensor_msgs/NavSatFix)
    /<drone_id>/self_localization/pose            (geometry_msgs/PoseStamped)

Publishes:
    /<drone_id>/geotag/survivors                 (SurvivorTag.msg)

Parameters:
    camera_intrinsics (fx, fy, cx, cy)
    camera_mount_pitch (angle from nadir)
    min_confidence_threshold (filter weak detections)
```

Simple pinhole projection is sufficient here: the terrain in a flood-zone settlement is near-flat, and RTK GPS gives you centimeter-level drone position. The main error source is detection bbox jitter, not GPS.

#### 4. `nidar_payload` — Onboard payload release node

Runs on each drone's companion computer. Manages the release mechanism (servo via companion GPIO or FC aux channel).

```
Subscribes:
    /<drone_id>/self_localization/pose            (geometry_msgs/PoseStamped)

Publishes:
    /<drone_id>/payload/status                   (PayloadStatus.msg)

Action Server:
    /<drone_id>/payload/release                  (custom action)
        Goal: target lat/lon, survivor_id
        Result: success/fail, measured drop distance
        Feedback: approach progress

Runs on:    Companion computer (Jetson)
Hardware:   Servo/actuator via GPIO or MAVLink DO_SET_SERVO
```

This IS a behavior-pattern node (action server with goal/result/feedback) so it integrates cleanly into AS2 behavior trees alongside stock TakeOff/GoTo/Land.

#### 5. `nidar_mission_manager` — GCS-side mission orchestration

Runs on the GCS laptop. This is the brain of the operation. It uses `as2_python_api` to command all 4 drones and coordinates the full mission sequence.

```
Subscribes:
    /drone_{1..4}/platform/info                  (platform status)
    /drone_{1..4}/sensor_measurements/gps        (position)
    /drone_{1..4}/sensor_measurements/battery    (battery)
    /drone_{1..4}/geotag/survivors               (new detections)
    /drone_{1..4}/payload/status                 (kits remaining)

Publishes:
    /gcs/mission/status                          (MissionStatus.msg)
    /gcs/mission/zone_allocation                 (ZoneAllocation.msg)
    /gcs/survivors/aggregated                    (SurvivorList.msg)
    /gcs/delivery/status                         (DeliveryStatusList.msg)

Subscribes (operator input):
    /gcs/emergency/abort                         (std_msgs/Bool)
    /gcs/emergency/recall                        (std_msgs/Bool)

Uses:
    as2_python_api DroneInterface for each drone
```

Core responsibilities:
- Parse boundary file → split into 4 scan zones → publish zone allocations.
- Generate boustrophedon/lawnmower waypoint paths per zone.
- Call AS2 behaviors per drone: TakeOff → FollowPath (scan) → GoTo (delivery) → Land.
- Aggregate survivor tags across all drones, deduplicate (two drones near a zone boundary might both detect the same person).
- Assign delivery tasks: nearest available drone with kits remaining.
- Handle failsafes: if a drone RTLs due to battery/link-loss, reassign its pending deliveries.
- Handle abort/recall: cancel all behavior actions, command all drones to land.

#### 6. `nidar_gcs_frontend` — Custom operator interface

The actual visual application the operator sees. Runs on the GCS laptop.

```
Subscribes (for display):
    /drone_{1..4}/sensor_measurements/camera     (live video feeds)
    /drone_{1..4}/sensor_measurements/gps        (map positions)
    /drone_{1..4}/sensor_measurements/battery    (health)
    /drone_{1..4}/platform/info                  (connection/arming state)
    /drone_{1..4}/detection/image_annotated      (annotated camera feeds)
    /gcs/mission/status                          (overall progress)
    /gcs/mission/zone_allocation                 (zone overlay on map)
    /gcs/survivors/aggregated                    (survivor markers on map)
    /gcs/delivery/status                         (delivery tracking)

Publishes (operator controls — ONLY these three):
    /gcs/mission/start                           (std_msgs/Bool)
    /gcs/emergency/abort                         (std_msgs/Bool)
    /gcs/emergency/recall                        (std_msgs/Bool)

Technology:
    Option A: PyQt5/6 + rclpy (native desktop app)
    Option B: Web app via rosbridge_suite (Flask/React + websocket)
```

GCS display requirements from the rules (Rule 8.14), mapped to topics:

| Required display | Source topic |
|---|---|
| Mission status | `/gcs/mission/status` |
| Live camera feed from each drone | `/drone_{n}/sensor_measurements/camera` or `/detection/image_annotated` |
| Position of each drone | `/drone_{n}/sensor_measurements/gps` |
| Assigned search area per drone | `/gcs/mission/zone_allocation` |
| Detected and geotagged survivor locations | `/gcs/survivors/aggregated` |
| Survival-kit delivery status | `/gcs/delivery/status` |
| Communication and system health | `/drone_{n}/platform/info` + `/drone_{n}/sensor_measurements/battery` |
| Consolidated mission progress | `/gcs/mission/status` |

#### 7. `nidar_bringup` — Launch files

One package holding all launch files and config YAML for the full system.

```
launch/
    drone_onboard.launch.py        # Launches per-drone: AS2 platform + controller +
                                   # state estimator + motion behaviors + detection +
                                   # geotag + payload nodes. Parameterized by drone_id.

    gcs.launch.py                  # Launches GCS-side: mission_manager + gcs_frontend.

    sim_full_system.launch.py      # Launches 4 simulated drones in Gazebo + GCS.
                                   # Used for development and integration testing.

config/
    drone_common.yaml              # Shared params: camera intrinsics, detection thresholds,
                                   # control mode, behavior plugins.
    drone_1.yaml ... drone_4.yaml  # Per-drone overrides: namespace, FC serial port,
                                   # radio channel.
    mission_manager.yaml           # Zone-split algorithm params, dedup radius,
                                   # delivery assignment strategy.
```

---

## PART 4 — Per-drone node graph (what runs on each companion computer)

Each companion computer runs these nodes (all namespaced under `/drone_N/`):

```
┌─────────────────────────────────────────────────────────┐
│  Companion Computer (Jetson) — drone_1                  │
│                                                         │
│  [AS2 — stock, unmodified]                              │
│   ├── as2_platform_pixhawk_node      (FC bridge)        │
│   ├── as2_state_estimator_node       (odom/pose/twist)  │
│   ├── as2_motion_controller_node     (PID controller)   │
│   ├── as2_behaviors_takeoff_node     (TakeOff action)   │
│   ├── as2_behaviors_land_node        (Land action)      │
│   ├── as2_behaviors_goto_node        (GoTo action)      │
│   └── as2_behaviors_followpath_node  (FollowPath action)│
│                                                         │
│  [NIDAR — your custom nodes]                            │
│   ├── nidar_detection_node           (YOLO inference)   │
│   ├── nidar_geotag_node             (pixel → lat/lon)   │
│   └── nidar_payload_node            (release mechanism) │
│                                                         │
│  [Driver]                                               │
│   └── camera_driver_node            (A12 → Image topic) │
└─────────────────┬───────────────────────────────────────┘
                  │ Local RF link (telemetry radio)
                  │ No GSM / LTE / Wi-Fi internet / cloud
                  │
┌─────────────────┴───────────────────────────────────────┐
│  GCS Laptop                                             │
│                                                         │
│   ├── nidar_mission_manager_node    (orchestration)     │
│   └── nidar_gcs_frontend_node       (operator UI)       │
└─────────────────────────────────────────────────────────┘
```

Total node count per drone: 10 (7 AS2 stock + 3 custom).
Total node count GCS-side: 2.
Entire system: 42 nodes (10 × 4 drones + 2 GCS).

---

## PART 5 — AS2 plugin selections

Based on the documented plugin tables and your mission profile:

| Component | Plugin choice | Rationale |
|---|---|---|
| Motion Controller | **PID Speed Controller** | Supports Position + Speed + Trajectory input modes with Yaw Angle. Simpler than differential-flatness; sufficient for waypoint missions at moderate speeds. |
| State Estimator | **Raw Odometry** | Uses the FC's own EKF fusion (which already incorporates RTK GPS + IMU). No need for external mocap or ground truth in a field deployment. |
| TakeOff Behavior | **Position plugin** | Sends a position reference to target altitude. More predictable than speed-based for competition reliability. |
| Land Behavior | **Speed plugin** | Controlled descent rate. Platform-land delegates to FC autoland which may be less predictable on uneven flood terrain. |
| GoTo Behavior | **Position plugin** | Simple position reference tracking. Trajectory plugin adds smoothness but GoTo is used for short delivery hops, not long paths. |
| FollowPath Behavior | **Trajectory plugin** (`dynamic_mav_trajectory_generator`) | Smooth trajectory through scan waypoints. Supports online replanning via `on_modify` if delivery tasks interrupt a scan path. |

---

## PART 6 — Mission execution flow

```
SETUP (5 min max)
│
├── Operator loads boundary file into GCS
├── mission_manager parses boundary → splits into 4 zones
├── mission_manager generates lawnmower waypoints per zone
├── Drones powered on, FC connected, AS2 stack launched
├── GCS confirms all 4 drones show "connected" on platform/info
│
MISSION START (operator presses single button)
│
├── mission_manager calls TakeOff behavior on all 4 drones (parallel)
├── Each drone ascends to scan altitude (e.g. 25-30m)
│
├── mission_manager calls FollowPath on each drone (its assigned zone waypoints)
│   ├── detection_node runs continuously, publishing DetectionResult
│   ├── geotag_node converts each detection to SurvivorTag with lat/lon
│   ├── SurvivorTags flow to GCS via RF link
│   └── mission_manager aggregates, deduplicates (configurable radius, e.g. 5m)
│
├── When a drone finishes its scan zone:
│   ├── mission_manager checks: are there undelivered survivors?
│   │   ├── YES + drone has kits → assign nearest survivor
│   │   │   ├── GoTo survivor position
│   │   │   ├── Descend to drop altitude
│   │   │   ├── Call payload/release action
│   │   │   ├── Confirm drop, update delivery/status
│   │   │   └── Loop: next undelivered survivor or RTL
│   │   └── NO or no kits → RTL
│   └── mission_manager calls Land behavior
│
├── [FAILSAFE — any drone]
│   ├── Low battery → FC triggers RTL autonomously
│   ├── Link loss → FC triggers RTL autonomously (AS2 watchdog)
│   ├── Geofence breach → FC triggers RTL autonomously
│   └── mission_manager detects drone loss → reassigns pending deliveries
│
├── [ABORT — operator presses button]
│   ├── mission_manager cancels all active behavior actions
│   └── Commands all drones to Land immediately
│
MISSION COMPLETE
│
├── All drones landed in 12×12 ft area
├── GCS shows final mission report: survivors detected, deliveries completed
└── ros2 bag recording stopped
```

---

## PART 7 — Weight budget check (25 kg collective limit)

| Component | Per drone | × 4 |
|---|---|---|
| Frame + motors + ESCs + props | ~2.5 kg | 10.0 kg |
| Battery (6S 3000-4000 mAh) | ~0.6 kg | 2.4 kg |
| Pixhawk + GPS + RTK module | ~0.2 kg | 0.8 kg |
| Jetson Nano/Orin Nano | ~0.15 kg | 0.6 kg |
| Camera (A12) | ~0.1 kg | 0.4 kg |
| Telemetry radio module | ~0.05 kg | 0.2 kg |
| Payload mechanism (servo + mount) | ~0.1 kg | 0.4 kg |
| Payload (3 kits × 200g) | 0.6 kg | 2.4 kg |
| Wiring, mounting, misc | ~0.2 kg | 0.8 kg |
| **Per-drone total** | **~4.5 kg** | **~18.0 kg** |

Margin: ~7 kg under the 25 kg limit. This is comfortable and allows for heavier batteries or larger frames if needed. Adjust based on your actual component weights.

---

## PART 8 — Workspace directory structure

```
nidar_rescueswarm_ws/
├── src/
│   ├── aerostack2/                        # Git submodule — the full AS2 repo
│   │   ├── as2_core/
│   │   ├── as2_platform_pixhawk/          # (or as2_platform_mavlink)
│   │   ├── as2_motion_controller/
│   │   ├── as2_state_estimator/
│   │   ├── as2_behaviors_motion/
│   │   ├── as2_behavior_tree/
│   │   ├── as2_python_api/
│   │   ├── as2_msgs/
│   │   └── ...
│   │
│   ├── nidar_msgs/                        # Your custom message definitions
│   │   ├── msg/
│   │   │   ├── DetectionResult.msg
│   │   │   ├── SurvivorTag.msg
│   │   │   ├── PayloadStatus.msg
│   │   │   ├── MissionBoundary.msg
│   │   │   ├── ZoneAllocation.msg
│   │   │   ├── MissionStatus.msg
│   │   │   └── DeliveryStatus.msg
│   │   ├── action/
│   │   │   └── PayloadRelease.action
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── nidar_detection/                   # Onboard YOLO detection node
│   │   ├── nidar_detection/
│   │   │   ├── detection_node.py
│   │   │   └── model/                     # Trained weights (onnx/engine)
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── nidar_geotag/                      # Onboard geotag fusion node
│   │   ├── nidar_geotag/
│   │   │   └── geotag_node.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── nidar_payload/                     # Onboard payload release node
│   │   ├── nidar_payload/
│   │   │   └── payload_node.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── nidar_mission_manager/             # GCS-side orchestration
│   │   ├── nidar_mission_manager/
│   │   │   ├── mission_manager_node.py
│   │   │   ├── zone_splitter.py           # Boundary → per-drone zones
│   │   │   ├── path_planner.py            # Zone → lawnmower waypoints
│   │   │   ├── survivor_aggregator.py     # Dedup + tracking
│   │   │   └── task_allocator.py          # Delivery assignment
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── nidar_gcs_frontend/               # Custom operator UI
│   │   ├── nidar_gcs_frontend/
│   │   │   ├── gcs_app.py                 # Main window (PyQt6)
│   │   │   ├── map_widget.py              # Map with drone positions + zones + survivor markers
│   │   │   ├── video_panel.py             # 4-up camera feed display
│   │   │   ├── status_panel.py            # Battery, connection, mission progress
│   │   │   └── control_panel.py           # Start / Abort / Recall buttons
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── nidar_bringup/                     # Launch files + configs
│       ├── launch/
│       │   ├── drone_onboard.launch.py
│       │   ├── gcs.launch.py
│       │   └── sim_full_system.launch.py
│       ├── config/
│       │   ├── drone_common.yaml
│       │   ├── drone_1.yaml
│       │   ├── drone_2.yaml
│       │   ├── drone_3.yaml
│       │   ├── drone_4.yaml
│       │   └── mission_manager.yaml
│       ├── behavior_trees/
│       │   └── rescue_mission.xml          # AS2 behavior tree definition
│       └── package.xml
│
├── build/
├── install/
├── log/
└── colcon.meta
```

---

## Summary — what you code vs what you configure

| Category | Effort |
|---|---|
| Aerostack2 core | **Configure only** — launch params, plugin selection, namespace |
| `nidar_msgs` | **Define** — message/action files, ~1 day |
| `nidar_detection` | **Build** — YOLO integration, TensorRT optimization, ~2-3 weeks |
| `nidar_geotag` | **Build** — projection math + dedup, ~1 week |
| `nidar_payload` | **Build** — action server + hardware driver, ~1-2 weeks |
| `nidar_mission_manager` | **Build** — zone split, path planning, task allocation, ~3-4 weeks |
| `nidar_gcs_frontend` | **Build** — full operator GUI, ~3-4 weeks |
| `nidar_bringup` | **Write** — launch files + config YAML, ongoing |
