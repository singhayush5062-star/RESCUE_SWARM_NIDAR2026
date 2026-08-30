# Plan: live mission log in the GCS

Answering "how do I watch, in real time, what the drones are doing and what
commands they were given?"

## Status today: not in the UI

There is **no log view in the GCS**. `gcs/src/components/` has
`DroneStatusPanel`, `DetectionPanel`, `VideoPanel`, `MissionTimer`,
`MappingAreaToolbar`, `MapView`, `MissionLoader`, `DroneControlPanel` — none
of them show log lines. The operator currently sees *state* (position,
battery, mission phase, detection counts) but never *events*.

### What you can use in the meantime

| where | what it gives you | command |
|---|---|---|
| per-process log files | one file per component, full history | `tail -f /tmp/nidar_sim_logs/mission_executor.log` |
| the whole ROS graph | every node's every log line, live | `ros2 topic echo /rosout` |
| one drone's behaviors | just that drone's takeoff/followpath/land | `ros2 topic echo /rosout \| grep drone2` |
| tmux | the raw AS2 panes | `tmux attach -t aerostack2` |

`mission_executor.log` is the one that answers "what command did it get and
what did it do" — it already logs every phase transition, every zone
allocation, every takeoff state change, and (new) every arm/offboard attempt
with its elapsed time and failure shape.

## The data source already exists and is cheap

`/rosout` (`rcl_interfaces/msg/Log`) carries every `get_logger()` call from
every node — 120 publishers in a running 4-drone sim. The concern with this
project has always been DDS volume, so it was measured during an **active
4-drone coverage mission with detection running**:

```
/rosout: 71 msgs in 20s = 3.5 Hz, 0.8 KB/s
by level: {'INFO': 71}
top talkers:
     13  drone2.FollowPathBehavior     <- drone movement
     13  drone3.FollowPathBehavior
     12  drone0.FollowPathBehavior
      9  detection_drone1
      6  detection_drone2
      3  drone3.TakeoffBehavior
      2  mission_executor              <- commands
```

0.8 KB/s is three orders of magnitude below the camera feed that caused this
project's memory incident. Streaming it to the browser costs nothing, and the
top talkers are exactly the two things asked for: **movement**
(FollowPath/Takeoff/Land behaviors) and **commands** (mission_executor).

Note `/rosout` is quiet at idle — it publishes on events, not on a timer, so
`ros2 topic hz /rosout` prints nothing between missions. That is correct
behaviour, not a broken topic.

## Plan

### 1. Backend: relay in `nidar_gcs_bridge` (no new package)

`gcs_bridge_node` already exists for exactly this job — it relays
`/nidar/mission_status`, `/nidar/zone_allocation`, `/nidar/mission_progress`
and per-drone `DetectionResult` into the `/gcs/*` namespace. A log relay is
the same task, so it belongs there rather than in a new package. (Contrast
`nidar_mission_clock`, which *did* need its own package because the mission
executor blocks inside AS2 calls for minutes and cannot tick a clock.)

Add: subscribe `/rosout` → publish `/gcs/log` (`rcl_interfaces/msg/Log`).

The relay is not a pass-through; it does three things the browser must not be
asked to do:

1. **Filter by publisher.** Keep `mission_executor`, `detection_*`,
   `survivor_manager`, `mission_clock`, and the per-drone AS2 behaviors
   (`*.TakeoffBehavior`, `*.FollowPathBehavior`, `*.LandBehavior`,
   `*.platform`). Drop `transform_listener_impl_*` and the ros_gz bridges —
   they are noise for an operator and are the bulk of any spike.
   Parameterise the allow-list (`log_sources`) rather than hard-coding it.
2. **Floor the level.** Parameter `min_log_level`, default INFO. An operator
   watching a competition run wants WARN and above by default; INFO is for
   debugging. Expose both.
3. **Rate-limit.** A node stuck in an error loop can emit hundreds of
   identical lines a second. Cap at N/sec (parameter, default 20) and emit a
   single "suppressed X messages" line when the cap bites, so the panel
   degrades into a summary instead of freezing the browser.

Subscribe with a depth-100 queue: log lines are bursty, and dropping the
burst is exactly the wrong thing for the messages that explain a failure.

### 2. Frontend: `useMissionLog.ts` + `MissionLogPanel.tsx`

`gcs/src/ros/useMissionLog.ts`, same shape as `useDetections.ts`:

- subscribe `/gcs/log`, `throttle_rate: 0` (the backend already rate-limits;
  rosbridge throttling here would only add latency, the mistake already
  corrected once in `useDetections`'s `FEED_THROTTLE_MS` comment),
  `queue_length: 0` so nothing is dropped client-side.
- keep a **bounded ring buffer** of the last 500 entries in a `useRef`, and
  publish to React state on a ~200 ms timer rather than per message. Setting
  state per message re-renders the whole panel at message rate; this is the
  single thing most likely to make the UI feel laggy, and this project has
  already paid for that lesson once with the video feed.
- derive `{entries, counts: {warn, error}, sources}`.

`gcs/src/components/MissionLogPanel.tsx`:

- virtualised or capped list, newest last, colour by level
  (INFO neutral / WARN amber / ERROR red).
- each row: `HH:MM:SS · source · message`.
- **filter chips** by drone namespace and by level — during a 4-drone run the
  single most useful action is "show me only drone2", which is precisely the
  question that arose when drone2 failed to arm.
- **auto-scroll with a pause-on-scroll-up latch.** An operator scrolling back
  to read an error must not be yanked to the bottom by the next INFO line.
- a persistent ERROR/WARN count badge in the header so a failure that scrolled
  past is still visible.
- **Copy / download** the buffer as text — the competition needs a post-run
  record, and today that means SSHing to `/tmp/nidar_sim_logs/`.

Placement: the right column already holds `VideoPanel` (400px). The log wants
width, not height. Put it as a **collapsible bottom drawer** spanning the map
+ right column, default collapsed, auto-expanding on the first ERROR. That
keeps the map usable, which is the panel operators actually fly from.

### 3. Verification

1. `ros2 topic hz /gcs/log` while a mission runs — confirm it is non-zero and
   materially below `/rosout` (the filter is doing something).
2. Force a real failure (kill one drone's platform node mid-mission) and
   confirm the ERROR reaches the panel, the badge increments, and the drawer
   auto-expands.
3. Flood test: a node logging in a tight loop must produce a "suppressed"
   line, not a frozen tab.
4. Confirm auto-scroll releases when the operator scrolls up and resumes when
   they scroll back to the bottom.

## Scope note

This is a **log view**, not a flight recorder. Replaying a mission
(timestamped position + command history, scrubable on the map) is a
genuinely different feature built on `rosbag2`, not on `/rosout`, and is not
part of this plan.
