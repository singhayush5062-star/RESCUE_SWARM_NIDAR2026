# Phase 0 — Environment & Architecture Freeze: Test Report

**Item under test:** "Install ROS 2 Humble + Aerostack2 on every dev machine and on each companion computer; confirm `colcon build` succeeds and the Gazebo/Multirotor sim examples run."
**Machine tested:** `ayush-LOQ-15APH8` (dev machine), Ubuntu 22.04.5 LTS
**Date:** 2026-08-23
**Tested by:** Ayush (singhayush5062@gmail.com)

---

## 1. Result summary

| Check | Result |
|---|---|
| ROS 2 Humble installed | ✅ Pass |
| Aerostack2 source checked out (`framework_ws/src/aerostack2`) | ✅ Pass |
| `colcon build` (28 packages) | ✅ Pass — clean build, 0 errors, 5.95s |
| Gazebo (`gz sim` 8.14.0) launches | ✅ Pass (after one environment fix — see §3) |
| Single-drone sim spawns, GPS/battery/IMU topics live | ✅ Pass |
| Full scripted mission (arm → takeoff → waypoints → land) via `as2_python_api` | ✅ Pass — root-caused and fixed during Phase 1 kickoff, see [`Phase1_Test_Report.md`](./Phase1_Test_Report.md) §1 (was ❌ failing as of this report's original writing, §4 below kept for the diagnostic trail) |

**Verdict:** The literal Phase 0 exit bar — *"everyone can build the empty workspace"* — is met on this machine. The install is sound and the simulator comes up correctly. One flight-control issue was found while stress-testing further into Phase 1 territory (an actual mission run); it does not block Phase 0 but **will block Phase 1's first milestone** and should be fixed before that work starts.

---

## 2. What was verified

```
$ colcon build --symlink-install
Starting >>> as2_msgs, as2_cli, as2_core, ...
Summary: 28 packages finished [5.95s]
```

Brought up the stock `project_gazebo` example (`./launch_as2.bash`) and confirmed, via `ros2 node list` / `ros2 topic echo`:

- Gazebo Sim spawns `drone0` (quadrotor + GPS + gimbal + HD camera payload) in the `empty` world.
- All expected AS2 nodes come up: `platform`, `controller_manager`, `state_estimator`, `TakeoffBehavior`, `GoToBehavior`, `FollowPathBehavior`, `LandBehavior`, `TrajectoryGeneratorBehavior`, `bt_manager`, plus the `ros_gz_bridge` topics.
- Live sensor data flows correctly:
  - `/drone0/sensor_measurements/battery` → 12.69V, 100%
  - `/drone0/sensor_measurements/gps` → lat 40.4405287, lon -3.6898277, alt ~100m (matches `config/world.yaml` origin)
  - `/clock` ticking under `use_sim_time`
- TF tree fully connected: `earth → drone0/map → drone0/odom → drone0 → drone0/base_link` (and all sensor/gimbal frames).

This confirms the ROS 2 + Aerostack2 + Gazebo toolchain is correctly installed and wired together on this machine.

---

## 3. Environment fixes required (apply on every machine, including companion computers)

Two machine-level issues had to be fixed before the sim would run. Both are environment quirks of this machine, not Aerostack2 bugs — but they'll likely recur on every dev machine and companion computer, so they belong in the setup checklist.

### 3.1 Gazebo Transport multicast discovery fails
**Symptom:** `gz sim` floods `Exception sending a multicast message: Network is unreachable` and the simulation entity never spawns cleanly.
**Cause:** Gazebo Transport's discovery layer picks an interface for multicast that isn't reliably routable on this machine (multiple interfaces present: `wlo1` up, `enp2s0`/`docker0` down/no-carrier, `lo` without multicast flag).
**Fix:** Pin the interface explicitly before launching:
```bash
export GZ_IP=<your-machine's-active-IP>   # e.g. the wlan0/wlo1 address
export GZ_PARTITION=<any-unique-name>      # avoids collisions if multiple sims run on the same LAN
```
**Action item:** Add these two exports to the team's shared shell profile / launch wrapper script referenced in Phase 0's git-repo setup, so no one has to rediscover this.

### 3.2 `python3` resolves to Conda's Python 3.14, not the system Python 3.10 ROS Humble needs
**Symptom:** Any `as2_python_api` mission script run with a bare `python3` fails immediately:
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```
**Cause:** Miniconda's `python3` (3.14) shadows `/usr/bin/python3` (3.10) on `$PATH`. ROS 2 Humble's compiled `rclpy` extension only exists for 3.10.
**Fix:** Run AS2 mission scripts with `/usr/bin/python3` explicitly, or deactivate conda's base env (`conda deactivate`) before sourcing ROS, or scope a dedicated conda env that excludes `PATH` shadowing.
**Action item:** Document this in the repo's setup instructions — anyone with Conda/Anaconda installed will hit this identically.

---

## 4. Issue found beyond Phase 0 scope: TakeoffBehavior never climbs

While stress-testing past the Phase 0 bar, `python3 mission.py` (stock `project_gazebo` single-drone mission) was run end-to-end:

- `arm()` → succeeds, platform transitions `LANDED → TAKING_OFF`.
- `offboard()` → succeeds.
- `takeoff(height=1.0)` → **behavior reports `RUNNING` indefinitely; altitude never leaves ~0.04m** (confirmed both via ROS `self_localization/pose` and the raw Gazebo `/model/drone0/pose` topic, ruling out a state-estimator artifact).

Diagnosis so far:
- `TakeoffBehavior` **is** publishing a correct target reference at 10Hz (`/drone0/motion_reference/pose`, target z ≈ 1.06m).
- `controller_manager` negotiates `POSITION YAW_ANGLE LOCAL_ENU_FRAME → SPEED YAW_SPEED BODY_FLU_FRAME` correctly and logs the expected input/output frames.
- PID gains in `config/pid_speed_controller.yaml` are non-degenerate for `position_control` (kp=1.0 on x/y/z).
- TF tree is fully connected (`earth → … → drone0/base_link`), so it isn't a broken transform.
- **But** the controller's final output, `/drone0/actuator_command/twist`, stays exactly zero on every axis despite a ~1m position error, and `/model/drone0/cmd_vel` in Gazebo is correspondingly zero — so the drone never receives a climb command.

The one anomaly noted: `motion_reference/pose` is published with `header.frame_id: earth`, while `controller_manager` logs its expected `input_pose_frame_id` as `drone0/odom`. This mismatch is the leading suspect, though it wasn't fully root-caused (would need to step into `pid_speed_controller`'s source to confirm whether it does a strict frame-id string check instead of a `tf2` lookup).

**This did not block Phase 0** (the exit criterion is build + sim bring-up, not a working flight) but would have blocked Phase 1. **Update:** root-caused and fixed at the start of Phase 1 — it was not a frame mismatch (the frame anomaly above was a red herring; TF lookups work fine) but a config-nesting bug that silently zeroed every PID gain. Full root cause and fix in [`Phase1_Test_Report.md`](./Phase1_Test_Report.md) §1.

---

## 5. Phase 0 status — full checklist

| Phase 0 item | Status |
|---|---|
| Install ROS 2 Humble + Aerostack2, confirm build + sim run | ✅ **Done** (this report) |
| Freeze platform choice (mavlink-via-mavros vs. native PX4 DDS) + companion computer model | ⬜ Not yet — needs a team decision, not a test |
| Freeze custom interface names (`DetectionResult`, `SurvivorTag`, `DeliveryStatus`, namespace convention) | ⬜ Not yet — no `docs/interfaces.md` or custom `.msg` files exist in the repo yet |
| Set up git repo (workspace, per-subteam packages, branch-per-feature) | ⬜ Not yet — `/home/ayush/NIDAR` is not currently a git repository |
| Confirm hardware orders placed | ⬜ Not yet — outside the scope of this test, needs procurement confirmation |

Only the install/build/sim item was in scope for this test pass. The remaining four items are decisions/process work for the team, not something to verify by running code — flagging them here so Phase 0 isn't marked fully closed until they're addressed too.
