# Phase 2 — Detection Pipeline: implementation notes

Covers `nidar_detection` and the mission clock (`nidar_mission_clock`) added
alongside it. Records what was built, what was measured, and the decisions
that differ from `NIDAR_Implementation_Plan.md`'s original wording.

---

## What runs

```
/droneN/sensor_measurements/gimbal/camera/image_raw   (30 Hz, 1280x960)
              │
              ▼   nidar_detection/detection_node   (one per drone, GPU, 2 Hz)
              │
              ├── /droneN/detection/results          DetectionResult (+ ByteTrack track_id)
              ├── /droneN/detection/image_annotated               Image        (only if subscribed)
              └── /droneN/detection/image_annotated/compressed    CompressedImage (only if subscribed)
                            │
                            ▼   nidar_gcs_bridge  (fans per-drone -> one topic)
                            │
                     /gcs/detections  ──►  GCS DetectionPanel
```

It runs by default. `NIDAR_DETECTION=false ./scripts/run_simulation.sh`
skips it when the GPU is needed for something else.

| env var | default | meaning |
|---|---|---|
| `NIDAR_DETECTION` | `true` | start detection nodes at all |
| `NIDAR_DETECTION_MODEL` | `project_gazebo/models/detection/nidar_person.pt` | weights |
| `NIDAR_DETECTION_HZ` | `2.0` | inference rate per drone |
| `NIDAR_DETECTION_CONF` | `0.5` | confidence threshold |

---

## Swapping in retrained weights

Copy over one file. No code, config, or launch change:

```bash
cp /path/to/retrained.pt project_gazebo/models/detection/nidar_person.pt
```

`detector.py` wraps ultralytics' `YOLO`, which dispatches on the file
extension — `.pt` (PyTorch, sim), `.onnx` (ONNX Runtime), `.engine`
(TensorRT, the Phase 8 Jetson target) all load from this same path.

---

## Measured results

All figures from live runs against the simulator, not estimates.

### The pipeline works end to end

Flying drone0 over `survivor_0` with `conf=0.10`:

| altitude | best confidence | bbox (x, y, w×h) |
|---|---|---|
| 6 m | 0.492 | (574, 430) 91×248 |
| 4 m | 0.613 | (544, 371) 144×388 |

102 `DetectionResult` messages arrived on `/gcs/detections`. Boxes sit near
the image centre (640, 480) — where a survivor directly beneath the drone
belongs — and grow as altitude drops, which is the correct perspective
relationship.

### Throttling behaves as designed

`188 frames seen, 19 inferred` over 10 s: the camera publishes at 30 Hz and
the detector runs at 2 Hz, so ~93 % of frames are dropped before inference.
A drone at 2 m/s covers 1 m between inferences, well inside the ~9 m camera
footprint at scan altitude, so nothing is flown past unseen.

### The GCS feed is 620× smaller than raw

The annotated preview is JPEG-encoded at 640×480: **5 947 bytes** versus
**3 686 400** for a raw 1280×960 BGR frame. Both annotated topics are also
published *only while something is subscribed* (`get_subscription_count`),
so an unwatched feed costs nothing. This matters — unthrottled image traffic
over rosbridge is what drove this GCS out of memory earlier in the project.

---

## Two things that had to change before any of this could work

### 1. The camera was pointing at the horizon, not the ground

The `hd_camera` payload was mounted at its default orientation, looking
**forward**. Every frame was sky and horizon. All of this project's coverage
maths already assumes nadir — `path_planner.ground_footprint_m` derives the
lawnmower swath from a footprint directly beneath the drone — so the camera
was contradicting the flight plan.

Fixed by pitching the mount 90° down in `world_swarm.yaml`. The non-obvious
part: the `rpy` must go on the **gimbal** entry, not on the `hd_camera`
payload inside it. AS2's gimbal templates apply the *gimbal's* pose to the
gimbaled sensor and ignore the sensor's own — their own `FIXME`, at
`speed_gimbal.sdf.jinja:88`. An `rpy` on the camera entry is silently
discarded; confirmed live, the rendered frame did not change at all.

### 2. Stock COCO weights cannot see a person from directly above

Measured against a real nadir render of `survivor_actor`:

| model | 640 px | 1280 px | 1920 px |
|---|---|---|---|
| yolov8n | 0 | 0 | 0 |
| yolov8s | 0 | 0.030 | 0 |
| yolov8m | 0 | 0 | 0.124 |

Cropping and upscaling the survivor until it filled the frame — equivalent
to ~1.3 m altitude — still gave only **0.038**. So this is a **viewpoint**
gap, not a resolution one: COCO contains essentially no overhead people, and
flying lower does not fix it.

This is why `NIDAR_Implementation_Plan.md` §2.1 (fine-tune on aerial human
imagery) is a hard prerequisite for real detection accuracy, not later
polish. YOLO26 does noticeably better than YOLOv8 at the low altitudes
tested above, but the retrained weights are what this pipeline is built to
receive.

---

## Three bugs found by a failed live run (2026-08-27)

A user run produced no camera feed and no mapping. All three causes were in
the logs; none of them reported an error.

### 1. No feed: detection was opt-in, so it never started

`/tmp/nidar_sim_logs/` had no `detection.log` at all — the nodes were simply
never launched, because `NIDAR_DETECTION` defaulted to `false`. The GCS panel
correctly showed "No signal" for a topic nobody was publishing, and nothing
anywhere said why. **Detection is now ON by default**; `NIDAR_DETECTION=false`
opts out.

### 2. No mapping: coverage paths had the RTL frame bug

The mission reached `[running] flying coverage pattern` and sat there for
7+ minutes on a 39 m path, with zero errors. All four drones were `FLYING`,
armed and offboard, hovering at exactly their scan altitude, and
`FollowPathBehavior` kept logging `RUNNING` — goal accepted, executing,
going nowhere.

Cause: AS2's `FollowPathGpsModule` converts every waypoint with
`geodetic2enu(waypoint, drone.gps.origin)` and publishes the result as an
**absolute point in the shared `earth` frame**
(`as2_python_api/behavior_actions/followpath_behavior.py`, the
`isinstance(path, GeoPath)` branch). Since `set_origin_on_start` pins each
drone's `gps.origin` to its own spawn fix, a path planned around the mission
origin comes out shifted by each drone's own offset from it.

This is the *same* bug already fixed for RTL in `_return_to_launch` — the fix
had only ever been applied there. `_compensate_gps_path` now inverts AS2's
transform for coverage paths too, per drone:

```
[drone0] path compensated for gps origin (28.6823976,77.4997176); first waypoint earth (-2.01,-0.22)
[drone1] path compensated for gps origin (28.6823976,77.4997504); first waypoint earth ( 2.21,-0.22)
[drone2] path compensated for gps origin (28.6824264,77.4997176); first waypoint earth (-2.01,-0.22)
[drone3] path compensated for gps origin (28.6824264,77.4997504); first waypoint earth ( 2.21,-0.22)
```

Verified live: the mission that previously hung now runs
`running -> returning -> landing -> complete` in 5m 38s.

### 3. ByteTrack silently failed on every frame: missing `lap`

`inference failed: No module named 'lap'` on every frame of all four nodes.
ByteTrack needs `lap` for detection-to-track assignment; ultralytics tries to
auto-install it at the first `track()` call, into a root-owned
`dist-packages`, and fails with `Permission denied`. Installed explicitly
(`pip install --user lap`) and documented in `nidar_detection/package.xml`.

---

## ByteTrack: counting people, not observations

`detection_id` counts *observations* and increments every frame a person is
visible — counting it reports one survivor many times over. `track_id` is a
ByteTrack identity that stays with one physical person across frames, so the
answer to "how many survivors" is the number of **distinct track_ids**.

Wired through `model.track(persist=True, tracker='bytetrack.yaml')`.
`persist=True` is what carries tracker state between calls; without it every
frame starts a fresh tracker and everything gets id 1 — worse than no ids,
because it looks like it works.

Measured on the verification flight:

```
[detect] frames seen 2174, inferred 611, detections 34, unique people (track ids) 2
[detect] 1 person(s), best conf 0.67 | tracks in frame [12] | unique tracks so far 2
[detect] 1 person(s), best conf 0.56 | tracks in frame [12] | unique tracks so far 2
[detect] 1 person(s), best conf 0.77 | tracks in frame [12] | unique tracks so far 2
```

**34 detections collapsed to 2 people**, with track `#12` held stable across
consecutive frames. The GCS panel now reads "N people" from distinct track
ids and shows the raw detection count only as a liveness signal.

Scope: track ids are unique **within one drone**. Two drones seeing the same
survivor produce two unrelated ids; reconciling those needs geographic
positions, which is Phase 3's survivor aggregator. A frame whose tracks are
not yet confirmed reports `track_id = -1` (`UNTRACKED`) — those detections are
real and are still published, just without an identity yet.

---

## GCS video panel

Camera feeds have their own **400 px column to the right of the map**
(`VideoPanel`), matching the plan document's own section 7.2 layout sketch.
Previously they were per-drone opt-in toggles inside the 280 px sidebar,
which meant a normal mission showed no video at all unless the operator knew
to go and switch each one on.

- **All four feeds are always subscribed.** ~7 KB a frame at ~1.5 Hz each,
  so ~30 KB/s for the whole swarm. The memory incident that originally made
  this cautious involved *raw* 3.6 MB frames — three orders of magnitude
  different.
- **Click any feed to expand it full-screen**, which is the only practical
  way to read a bounding box and its track id on a 1280x960 frame scaled
  into a column.
- **A headline "Detected persons" count** sits above the feeds: distinct
  ByteTrack ids across the swarm, with the raw detection count underneath as
  a liveness signal only.
- `DetectionPanel` in the sidebar is now numbers only (per-drone tracked
  count, detections, best confidence).

Verified mid-mission, with all four drones flying a coverage pattern:

```
drone0:  24 frames/20s, 6266 B avg -> LIVE
drone1:  28 frames/20s, 6266 B avg -> LIVE
drone2:  24 frames/20s, 6259 B avg -> LIVE
drone3:  22 frames/20s, 6261 B avg -> LIVE
```

---

## Stale FastDDS shared memory is what makes feeds and drones drop out

This is the single most disruptive environmental problem in this project and
it has now been traced properly.

A 4-drone AS2 stack plus detection is ~100 DDS participants. FastDDS gives
each one a shared-memory port lock file in `/dev/shm`, and a `kill -9`ed
process never releases its own. They accumulate across restarts until new
participants cannot acquire a port:

```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7661:
    open_and_lock_file failed -> Function open_port_internal
```

The failure is **silent and partial** — nodes still start and log "ready",
they are just not fully connected. Measured symptoms, all from this:

| symptom | measurement |
|---|---|
| drones "not active" | drone1 + drone3 published **zero** GPS while drone0/drone2 worked |
| video feed missing | a subscriber matched **1 of 4** camera feeds |
| mission never started | `gcs_bridge` hit it at startup; a published mission never reached the executor |

It is contention, not capacity: 371 shm files with **7 GB of /dev/shm still
free**. After a clean stop that clears them, the same setup gives **4/4 feeds
live** and all four drones streaming.

`run_simulation.sh`'s stop path now removes `sem.fastrtps_*` as well as
`fastrtps_*` — the `sem.` port-lock files are the ones that actually exhaust
the pool, and the old cleanup left every one of them behind.

**If drones or feeds go missing, stop and relaunch rather than launching over
a running sim.** Launching on top leaves both instances half-connected.

### CycloneDDS was tried and rejected

CycloneDDS has no per-participant SHM port locks and does fix discovery —
all four drones' GPS streamed where FastDDS had left two silent. But it drops
the raw 1280x960 camera images (3.6 MB a frame) entirely without buffer
tuning; every detection node reported "no camera frames received yet". That
trades an intermittent problem for a total one, so FastDDS stays the default.
`NIDAR_RMW=rmw_cyclonedds_cpp` switches, for debugging discovery problems
where the camera is not needed.

---

## Raw camera bandwidth was starving three of the four drones

Symptom: "video frames are not working". Measured: Gazebo was rendering all
four cameras correctly (`gz topic -e` returned frames on all four sensors,
real-time factor 1.002x), the `parameter_bridge` for every drone was running
with identical, correct remaps -- and yet only **drone0's** ROS image topic
delivered. drone1/drone3 delivered nothing at all and drone2 was stuck at one
frame.

The cause is volume, not configuration:

```
1280 x 960 x 3 bytes  = 3.69 MB per frame
       x 20 Hz        =  74 MB/s per drone
       x 4 drones     = 295 MB/s through DDS shared memory
```

The transport cannot sustain that on this machine, and it degrades by
starving whole topics rather than dropping frames evenly — which is why it
looked like a per-drone fault.

**Fix: `hd_camera` update_rate 20 Hz -> 4 Hz** (59 MB/s, a 5x cut).
Resolution is deliberately left at 1280x960, so detection accuracy and the
geotag intrinsics (fx = fy = 1108.5) are unaffected, and 4 Hz is still double
`nidar_detection`'s 2 Hz inference rate.

Verified after the change — all four pipelines alive:

```
drone         raw camera       annotated     size
drone0      18 @ 0.90Hz       8 @ 0.40Hz    7031B
drone1      22 @ 1.10Hz      13 @ 0.65Hz    7442B
drone2      25 @ 1.25Hz      17 @ 0.85Hz    7139B
drone3      16 @ 0.80Hz      12 @ 0.60Hz    6938B
raw: 4/4   annotated: 4/4
```

Delivered rate sits below 4 Hz because the machine is still loaded; the point
is that all four now flow instead of one.

## Launching over a running sim

The other half of "the simulation is not starting": a launch begun while a
previous sim was still shutting down left **both** running. Confirmed live —
a fresh sim came up beside a stale one and their detection nodes fought for
the same camera topics, two reporting 16721 frames seen while the two new
ones reported 1.

`stop_all` now polls until every simulation process is actually gone (and
re-kills plus re-clears `/dev/shm` if any survive) instead of assuming a
fixed `sleep 2` was enough.

**Always `./scripts/run_simulation.sh stop` before relaunching.**

---

## Deviations from the plan document

- **§2.2 says `.../sensor_measurements/camera`.** The real topic is
  `/<drone_id>/sensor_measurements/gimbal/camera/image_raw` — already noted
  as a difference in the plan's own Phase 0.3 entry. `detection_node` uses
  the real one. Getting this wrong subscribes successfully to a topic nobody
  publishes and silently detects nothing forever, which is why the node logs
  a warning every 10 s when no frames have arrived.
- **§2.2 says ONNX Runtime.** Ultralytics is used instead, which loads
  `.pt`/`.onnx`/`.engine` through one API, so the Jetson path stays a
  parameter change. ONNX Runtime is not installed.
- **§2.4 says a permanent 4-up video grid.** Feeds are opt-in per drone
  instead. The backend only encodes for subscribed feeds, so off costs
  nothing, and four simultaneous feeds is precisely the traffic pattern that
  has crashed this GCS before. The grid can come back in Phase 7.3 with the
  MJPEG-server approach that section already prefers.

---

## Mission clock (`nidar_mission_clock`)

Separate package, one node, one task: measure how long a mission run takes.

- Subscribes `/nidar/mission_status`, publishes `nidar_msgs/MissionStatus`
  on `/nidar/mission_progress` at 1 Hz, relayed to `/gcs/mission/progress`.
- Populates `elapsed_time_sec`, which `MissionStatus.msg` has declared since
  Phase 0 and which Phase 7.1's Rule 8.14 checklist requires.
- Added `ABORTED=5` to the phase enum, so a failed run is never displayed as
  a successful one.

**Why not a timestamp inside `nidar_mission_executor`:** that node spends
most of a mission blocked inside AS2 calls (`follow_path` joins every
drone's thread — minutes with no return to its own event loop). It therefore
*cannot* publish a ticking clock; its status messages only appear at phase
transitions, which can be many minutes apart. A node with its own timer can.

Verified live by driving a full phase sequence:

```
before any mission:        elapsed=   0.0s  running=False phase=SETUP
after starting (+3s):      elapsed=   2.7s  running=True  phase=SETUP
mid-scan (+6s):            elapsed=  12.7s  running=True  phase=SCANNING
during RTL (+3s):          elapsed=  15.7s  running=True  phase=RTL
at complete:               elapsed=  18.3s  running=False phase=COMPLETE
5s after complete:         elapsed=  18.3s  running=False phase=COMPLETE   <- frozen
after loading next:        elapsed=  18.3s  running=False phase=SETUP      <- still readable
```

It also logs where the time went, which is the counterpart to the executor's
`[coverage]` plan log — that one says what the mission intended to do, this
says how long each part took:

```
[clock] mission completed in 18s (18.3s)
[clock]   starting             3s  (16.4%)
[clock]   taking_off           4s  (22.4%)
[clock]   running              6s  (33.3%)
[clock]   returning            3s  (16.4%)
[clock]   landing              2s  (11.5%)
```

---

## Simulator behaviours worth knowing

Each of these cost real debugging time and is not documented upstream.

- **Gazebo `<actor>` entities ignore `SetEntityPose`.** The service returns
  `success: true` and the actor does not move — its trajectory script
  re-asserts its pose every tick. Actors can only be positioned at spawn
  time. (`survivor_actor` is an `<actor>`; `nidar_survivor_manager`'s
  runtime spawn path is unaffected, but any future "move a survivor" feature
  will not work this way.)
- **`survivors.yaml` is the single source of truth.**
  `utils/sync_survivors.py` regenerates `world_swarm.yaml`'s `objects:`
  block before every launch, so hand-edits to that block are silently
  overwritten at the next start.
- **Stale FastDDS shared memory silently kills a fresh sim.**
  `run_simulation.sh`'s cleanup removed `/dev/shm/fastrtps_*` but *not*
  `/dev/shm/sem.fastrtps_*` — and the `sem.` port-lock files are the ones
  that exhaust the pool. At 93 stale segments a freshly launched sim
  published zero camera frames while every new node logged `Failed init_port
  fastrtps_port####: open_and_lock_file failed`. Now cleaned up properly.
- **`gz model --list` does not show actors.** Already noted in
  `survivors.yaml`'s own header — verify actor placement visually or through
  the camera, not with that tool.

---

## Test coverage

`framework_ws/src/nidar_detection/tests/` — 45 tests, all passing:

- `test_detector.py` (19) — which model outputs become detections, bbox
  geometry, reversed corners, device resolution, annotation, downscaling.
- `test_projection.py` (14) — world→pixel geometry against the sim's real
  intrinsics (fx = fy = 1108.5), including that the 10 m footprint comes out
  at the documented 11.55 m.
- `test_silhouette.py` (12) — label extraction, including that shadows are
  not mistaken for survivors.

`framework_ws/src/nidar_mission_clock/tests/` — 20 tests, all passing:
clock start/freeze/reset rules, phase accounting, duration formatting.

---

## The trained model (2026-08-29): PERSON_DETECTION_MODEL_V3, on ncnn

The placeholder stock weights are gone. `nidar_person.pt` is now YOLO26n
fine-tuned on MANNEQUIN_PERSON_V3, single class `person`, trained at 640x640
(its own validation: P 0.961, R 0.950, mAP50 0.974, mAP50-95 0.917).

The difference against this simulator's nadir camera is not marginal:

| weights | confidence on a sim survivor, top-down |
|---|---|
| stock COCO YOLO26n | 0.03 - 0.12 (even filling the frame) |
| PERSON_DETECTION_MODEL_V3 | **0.76 - 0.97** |

That closes the viewpoint gap this document previously recorded as the
blocking issue for real detection.

### ncnn is the CPU backend, and it is chosen automatically

The model shipped as three exports of the same weights: `.pt`, `.onnx`, and
an ncnn directory. They are not three models and nothing downstream should
have to pick between them, so `detector.resolve_model_path()` does it by
device. Measured on this box, live 640x480 sim frame, same weights:

```
.pt  on CUDA, imgsz 640 ....  13.2 ms   (76.0 FPS)
.pt  on CPU,  imgsz 640 ...  379.4 ms   ( 2.6 FPS)
.pt  on CPU,  imgsz 416 ...  213.2 ms   ( 4.7 FPS)
ncnn on CPU,  imgsz 640 ...  122.2 ms   ( 8.2 FPS)
```

So: CUDA available -> load the `.pt` (9x faster than ncnn). CPU only -> load
the ncnn export (3.1x faster than the `.pt`, at the SAME input size). The
second row of that table is why the node previously cut CPU inference to
imgsz 416; ncnn at full 640 beats that anyway, so `CPU_FALLBACK_INPUT_SIZE`
is now applied only to the PyTorch backend and ncnn keeps the resolution the
model was trained at.

`model_path` remains the single swap point. `resolve_model_path()` looks for
`<stem>_ncnn_model/` beside it, which is exactly what
`yolo export format=ncnn` writes, so re-exporting retrained weights is enough
to keep the fast path. A `.pt` with no ncnn export still works; it just runs
slower and the node says so. `prefer_ncnn_on_cpu:=false` forces the literal
path.

Loading an ncnn export needs `pip install ncnn`. It is declared in
`package.xml` but is not rosdep-resolvable.

To exercise the ncnn path deliberately on a machine that HAS a GPU -- worth
doing, since it is the backend the competition companion computer runs:

```bash
NIDAR_DETECTION_DEVICE=cpu ./scripts/run_simulation.sh
```

## The detection nodes had never once started from the launcher

Every previous "detection verified" result in this document came from running
the nodes by hand with an explicit `PYTHONPATH`. Launched the normal way,
they failed:

```
Package 'nidar_detection' not found: "package 'nidar_detection' not found,
searching: [... every other nidar package, but not this one ...]"
```

The package built cleanly, installed completely, appeared in `ros2 pkg list`,
and had a correct ament index marker. It was simply absent from
`AMENT_PREFIX_PATH`.

Cause: `package.xml` contained a **doubled ASCII hyphen inside an XML
comment** (in a comment explaining the `lap` dependency). That is illegal
XML. Nothing anywhere reported it. colcon could not parse the manifest, so it
identified the package as build type `python` instead of `ros.ament_python`
-- visible only via `colcon list`:

```
nidar_detection      src/nidar_detection      (python)            <- wrong
nidar_mission_clock  src/nidar_mission_clock  (ros.ament_python)  <- right
```

The `ament_python` build task is the only thing that adds the
`ament_prefix_path` environment hook. No hook -> not on `AMENT_PREFIX_PATH`
-> `ros2 launch` cannot find it. Comparing `install/*/share/*/package.dsv`
between the two packages shows it directly: the working one sources three
`ament_prefix_path.*` lines, the broken one sources none, despite the hook
files existing on disk.

Fixed by rewording the comment. `tests/test_package_manifest.py` now asserts
every `nidar_*/package.xml` parses, declares a build type, and matches its
directory name -- because the failure mode is silent everywhere else.

Two secondary fixes fell out of this:

* `run_simulation.sh` tested the model path with `-f`, which is false for an
  ncnn export (a directory), and logged "model not found" while skipping
  detection entirely. Now `-e`.
* `NIDAR_DETECTION_DEVICE` is passed through to the launch file, so the ncnn
  path can be selected without editing anything.

## "drone2 failed to arm/offboard" was never an arming problem

A mission aborted 30s after start with `[error] drone2 failed to arm/offboard`.
Inspecting the still-running graph:

* `/drone2/platform` process alive,
* `/drone2/set_arming_state` and `/drone2/set_offboard_mode` both advertised,
* but `ros2 topic echo /drone2/platform/info` received **nothing**, while
  drone0/1/3 answered immediately, and
* `ros2 topic info -v /drone2/platform/info` reported its publisher as
  `_NODE_NAME_UNKNOWN_` / `_NODE_NAMESPACE_UNKNOWN_`.

That is FastDDS half-discovery: the endpoint was announced but the
participant never completed, so every service call into it hung until its own
timeout. `_call_bounded`'s 3 attempts x 8s bound is exactly the 27s the
mission spent before giving up.

Nothing about `arm()` could have fixed that, so arming was the wrong thing to
try. Two changes:

1. **`_wait_platform_ready()` runs before arming.** `connected` in
   as2_python_api's `PlatformInfoData` defaults to `False` and is only ever
   set from a received `PlatformInfo` message, so it is a direct test of "has
   any platform status reached our own subscription". A drone that fails it
   now reports *"platform node is not reachable, no /droneN/platform/info
   received in 10s"* immediately, instead of a generic arming failure 27s
   later that sends the operator looking for a flight-control bug.
2. **`_call_bounded` logs each attempt** with its elapsed time and which
   shape of failure it was: a call that *returns* False fast is the service
   answering no; a call still running at the timeout is the service never
   having been reached. Those need completely different fixes and were
   previously indistinguishable.

The arm/offboard sequence was also duplicated verbatim in both mission entry
points; it is now one `_arm_and_offboard()` so the two cannot drift.

Worth knowing: the missing drone is **not stable across runs**. On the next
clean launch it was drone0 that failed the first probe and answered normally
seconds later -- which is why the gate polls for 10s rather than sampling
once.

## Not done here

- **Geotagging** (pixel → lat/lon). Phase 3. `projection.py` implements and
  tests the *forward* direction (world → pixel), so Phase 3's inverse has a
  verified reference to agree with.
- **Cross-drone de-duplication.** Needs geographic positions, which only
  exist after geotagging; Phase 3's survivor aggregator owns it.
- **`total_survivors_detected` / `total_deliveries_complete`** in
  `MissionStatus`. Left at 0 rather than guessed — those belong to the
  detection and delivery pipelines, not to a clock. `mission_clock_node`
  already subscribes `/nidar/survivor_count` for when the aggregator exists.
- **`dataset_capture_node`.** Built (captures labelled frames by projecting
  known actor positions and matching them to pixel silhouettes) but **not
  validated end to end** — its pure components are tested, the full capture
  loop is not. Retraining is being handled separately.
