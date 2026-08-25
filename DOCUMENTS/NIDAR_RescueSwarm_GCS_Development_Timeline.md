# NIDAR RescueSwarm — Custom GCS Development Timeline

**Architecture:** Aerostack2 (ROS 2 backend) + custom QGC/Mission-Planner-style frontend
**Kickoff:** Monday, August 24, 2026
**Target completion:** Monday, November 30, 2026 (14 weeks)

## Assumptions

- ROS 2 Humble on Ubuntu 22.04, Aerostack2 installed from source (BSD-3-Clause, so it can be forked/extended directly).
- Flight controller is Pixhawk-class on ArduPilot or PX4, bridged through `as2_platform_mavlink` or `as2_platform_pixhawk`.
- One Jetson-class companion computer per drone, at least 2 drones total.
- Hardware (FC, companion computers, telemetry radios, cameras, release mechanism) is ordered no later than Week 1 — this timeline assumes hardware arrives by Week 3-4. If procurement slips, Phases 2-3 slip with it.
- At least 3-4 people available so hardware, perception, and GCS tracks can run in parallel (the 2-person setup limit in the rules applies only to competition-day setup, not to your dev team).

## Timeline at a glance

| Phase | Weeks | Dates | Focus |
|---|---|---|---|
| 0 | 1 | Aug 24 – Aug 30 | Environment, repo, architecture freeze |
| 1 | 2-3 | Aug 31 – Sep 13 | Aerostack2 in simulation, GCS skeleton |
| 2 | 4-5 | Sep 14 – Sep 27 | FC hardware bring-up, comms, payload bench test |
| 3 | 6 | Sep 28 – Oct 4 | Single real drone, full autonomy + failsafes |
| 4 | 4-8 (parallel) | Sep 14 – Oct 18 | Detection model: train → bench → flight |
| 5 | 9-10 | Oct 19 – Nov 1 | Geotagging + payload behaviors, single-drone validation |
| 6 | 11-12 | Nov 2 – Nov 15 | Multi-drone coordination + full GCS build-out |
| 7 | 13-14 | Nov 16 – Nov 29 | Full-system field trials, hardening |
| 8 | — | Nov 30 | Freeze, final rehearsal |

---

## Phase 0 — Environment & architecture freeze
**Weeks 1 · Aug 24 – Aug 30**

- [x] Install ROS 2 Humble + Aerostack2 on every dev machine and on each companion computer; confirm `colcon build` succeeds and the Gazebo/Multirotor sim examples run. **Tested and passing on dev machine `ayush-LOQ-15APH8` (2026-08-23)** — full writeup, required environment fixes (`GZ_IP`/`GZ_PARTITION` for Gazebo multicast, conda-vs-system `python3`), and a flight-control issue found ahead of schedule (TakeoffBehavior doesn't climb — likely a reference-frame mismatch) in [`Phase0_Environment_Test_Report.md`](./Phase0_Environment_Test_Report.md). Still needs the same install+test pass repeated on every other dev machine and each companion computer before this item is fully closed team-wide; the takeoff issue should be resolved before Phase 1 starts.
- Freeze the platform choice (mavlink-via-mavros vs. native PX4 DDS) and companion computer model.
- Freeze custom interface names now, before any team starts coding against them: `DetectionResult`, `SurvivorTag`, `DeliveryStatus`, plus the drone namespace convention (`/drone_1/...`, `/drone_2/...`).
- Set up the git repo: one workspace, one package per subteam, branch-per-feature, a shared `docs/interfaces.md`.
- Confirm hardware orders are placed (FC, companion computers, telemetry radios, camera, release mechanism).

**Exit criteria:** everyone can build the empty workspace; interface contract is written down and agreed.

---

## Phase 1 — Aerostack2 in simulation + GCS skeleton
**Weeks 2-3 · Aug 31 – Sep 13**

- [x] Bring up 2 simulated drones in Gazebo using stock Aerostack2 behaviors (takeoff, follow-reference, land) driven by `as2_python_api` scripts.
- [x] Study and reproduce one behavior-tree mission end-to-end in sim.
- [x] Start the custom GCS frontend against the **simulated** namespace only: connect, subscribe to `sensor_measurements/gps` and `/battery` for both drones, render live positions on a map.
- [x] Stand up the mission-file loader: parse the organiser's boundary/mission file format into an `as2_python_api` mission script or behavior-tree XML.

**Tested and passing on dev machine `ayush-LOQ-15APH8` (2026-08-23)** — full writeup, including two more upstream AS2 bugs found and fixed (a takeoff-blocking PID config bug carried over from Phase 0, and a broken `DroneInterfaceGPS`), the GCS build (React + Vite + Leaflet + roslib/rosbridge), and an end-to-end verified mission-file-driven flight in [`Phase1_Test_Report.md`](./Phase1_Test_Report.md). The mission-file schema is a placeholder (see [`mission_file_schema.md`](./mission_file_schema.md)) since the organisers don't publish their real format until competition setup — swapping it in later only touches one parsing function.

**Exit criteria:** GCS shows two simulated drones moving on a map, driven entirely by a loaded mission file, no manual waypoint entry. **Met** — verified via headless-browser test of the actual GCS UI.

---

## Phase 2 — FC hardware bring-up, comms, payload bench test
**Weeks 4-5 · Sep 14 – Sep 27**

Runs in parallel across three independent tracks:

- **FC/autopilot track:** bench-connect the real FC to the companion computer via `as2_platform_mavlink`/`as2_platform_pixhawk`; confirm state/heartbeat topics, arm with props off, read/write a parameter.
- **Comms track:** bench and then field-test the telemetry radio link (range, RSSI, packet loss) with zero GSM/LTE/Wi-Fi-internet dependency, per the no-network rule.
- **Payload track:** bench-test the release mechanism (servo/actuator) on a ground rig through repeated drop cycles, independent of any flying hardware.

**Exit criteria:** FC reports `connected: true` to Aerostack2 on the bench; radio link validated to the field's working range; release mechanism reliable on the bench rig.

---

## Phase 3 — Single real drone, full autonomy + failsafes
**Week 6 · Sep 28 – Oct 4**

- Run the full auto sequence on one real drone using stock Aerostack2 behaviors: arm → takeoff → waypoint mission → land, triggered only by the GCS "mission start" (no manual waypoint edits).
- Trigger every failsafe deliberately — low battery, C2 link loss, geofence breach, abort — first in SITL, then on the bench/field, and confirm the GCS reflects each state change live.
- GCS: single-drone mission load → fly → status display working end-to-end on real hardware.

**Exit criteria:** one real drone completes a scripted mission with zero manual intervention, and all failsafes visibly trigger RTL/abort in the GCS.

---

## Phase 4 — Detection model (runs parallel to Phases 2-3)
**Weeks 4-8 · Sep 14 – Oct 18**

- Curate/fine-tune on aerial-person data (VisDrone, HERIDAL, SARD) — not ground-level datasets.
- Bench-test on the companion computer with a tripod camera; iterate cheaply before touching a drone.
- Pole/gimbal walk-test to simulate operational altitude and angle.
- Once Phase 3's drone is flying, run real low-altitude detection passes over dummies; tune thresholds against measured false positive/negative rates.
- Wrap the finished model as `detection_behavior`, following Aerostack2's documented "Writing a New Behavior" pattern so it plugs into the same mission layer as stock behaviors.

**Exit criteria:** `detection_behavior` runs on the companion computer in flight and publishes detections at an acceptable false-positive rate.

---

## Phase 5 — Geotagging + payload behaviors, single-drone validation
**Weeks 9-10 · Oct 19 – Nov 1**

- Build `geotag_behavior`: fuse detection pixel coordinates with drone GPS/attitude/camera intrinsics into a lat/lon estimate. Validate in isolation first (known pose, known target) before trusting it in flight.
- Build `payload_release_behavior` (pattern-matched against Aerostack2's existing Gripper Behavior) and integrate the bench-tested mechanism from Phase 2.
- Single-drone flight validation: detect → geotag → fly to drop point → release, fully autonomous.

**Exit criteria:** one drone autonomously detects a dummy, geotags it within acceptable error, and drops a payload near it — no manual triggering.

---

## Phase 6 — Multi-drone coordination + full GCS build-out
**Weeks 11-12 · Nov 2 – Nov 15**

- Build the coverage/area-division planner: splits the boundary polygon into per-drone lawnmower patterns sized to battery/time budget.
- Build the task allocator + survivor aggregator: dedupes tags across drones, assigns the nearest available drone to each delivery.
- Complete the GCS: mission-file load confirmation, live multi-drone map, survivor markers, delivery status per survivor, mission progress, live video panel, and the only permitted manual controls (abort/recall).
- Validate the whole pipeline in Gazebo SITL with 2+ simulated drones before it ever touches real hardware.

**Exit criteria:** in SITL, 2+ drones split the area, both report detections to one GCS, tasks are allocated without collision, and the GCS shows complete mission status for all drones.

---

## Phase 7 — Full-system field trials
**Weeks 13-14 · Nov 16 – Nov 29**

- Run the complete pipeline on 2+ real drones against the actual competition budget: 5-minute setup, 30-minute flight.
- Inject failures mid-mission (force a battery failsafe on one drone) and confirm the system reallocates and the others continue.
- Confirm the whole mission runs on the local radio link with zero GSM/LTE/Wi-Fi-internet dependency.
- Record every run with `ros2 bag` for debugging and scoring rehearsal.
- Reserve the back half of this window purely as integration-bug buffer — this is the phase most likely to slip.

**Exit criteria:** at least one clean end-to-end run, within budget, with all required GCS displays working and one injected failure survived.

---

## Phase 8 — Freeze & final rehearsal
**Nov 30**

- Lock the software version and back up all configs (mission files, radio settings, model weights).
- Run one final rehearsal that exactly mimics competition-day conditions: 5-minute setup timer, cold-start hardware, real boundary file.
- Prepare spares (batteries, props, a backup companion computer image).

---

## Parallel team split (reference)

| Track | Owns | Can start | Depends on |
|---|---|---|---|
| FC/autopilot | `as2_platform_*` config, failsafes | Week 1 (sim), Week 4 (real) | Hardware arrival for real tests |
| Comms | Radio link, range/reliability | Week 4 | Hardware arrival |
| Perception | `detection_behavior`, model training | Week 4 | Nothing — works offline on recorded video |
| Geotag/fusion | `geotag_behavior` | Week 9 | Perception output format frozen (Week 0) |
| Payload | Release mechanism + `payload_release_behavior` | Week 4 (bench) | Nothing for bench; drone for flight validation |
| Coordination | Coverage planner, task allocator | Week 2 (sim) | Interface freeze (Week 0) |
| GCS/frontend | Mission loader, map, dashboard, video | Week 2 (sim) | Interface freeze (Week 0) |

## Key risks to watch

- **Hardware lead time** is the single biggest threat to this schedule — if FC/companion computers/radios aren't in hand by Week 3, Phases 2-3 (and everything after) slip 1:1.
- **Detection model accuracy** often takes longer than planned — Phase 4's 5-week window includes buffer for a second training pass; don't compress it to hit other deadlines.
- **Multi-drone SITL testing (Phase 6) is not optional** — skipping straight to real hardware for coordination logic is the most common cause of late-stage schedule blowouts in these competitions.
- Weeks 13-14 are deliberately kept as a trial-and-fix buffer, not new-feature time. If Phases 0-6 slip, protect this buffer first.
