#!/bin/bash
# NIDAR RescueSwarm — single-command full simulation launcher.
#
# Brings up: Gazebo + AS2 for every drone in project_gazebo/config/world_swarm.yaml,
# rosbridge_server, the nidar_gcs_bridge/nidar_mission_executor/nidar_survivor_manager
# nodes, and the GCS dev server. Ctrl+C tears everything back down. Also syncs survivor
# placements from
# project_gazebo/config/survivors.yaml before every launch — edit that file to
# add/remove/move survivors, not world_swarm.yaml's auto-generated objects: block.
#
# Usage:
#   ./scripts/run_simulation.sh                       # all drones in world_swarm.yaml
#   ./scripts/run_simulation.sh drone0,drone1         # a specific subset
#   GCS_PORT=5180 ./scripts/run_simulation.sh         # non-default GCS port
#
# Every fix below exists because we hit the failure it prevents earlier in
# development — see DOCUMENTS/Phase0_Environment_Test_Report.md and
# DOCUMENTS/Phase1_Test_Report.md for the incidents:
#   - GZ_IP/GZ_PARTITION (Phase 0): Gazebo Transport's multicast discovery
#     fails with "Network is unreachable" on machines with multiple/inactive
#     network interfaces unless pinned to the active one.
#   - PATH stripped of conda (Phase 0/1): conda's python3 shadowing the
#     system one breaks rclpy for every ROS 2 CLI tool, not just directly-run
#     scripts.
#   - Full process-tree kill before AND after (this session, repeatedly):
#     `tmux kill-server` does not reliably kill the `ros2 launch` process
#     trees underneath it, and a stale mission_executor_node left running
#     from a previous session will race a fresh one for control of the same
#     drones and silently corrupt results.
#   - Default to every drone in world_swarm.yaml, not a hardcoded subset:
#     the exact bug where Gazebo spawns 3 drones but only 2 get real AS2
#     nodes because the launch command only named 2.
# Not `set -u`: ROS 2's own /opt/ros/humble/setup.bash references unset
# variables internally (AMENT_TRACE_SETUP_FILES) and fails immediately under
# nounset — confirmed the hard way on the first test run of this script.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NIDAR_ROOT="$(dirname "$SCRIPT_DIR")"
PROJECT_GAZEBO="$NIDAR_ROOT/project_gazebo"
GCS_DIR="$NIDAR_ROOT/gcs"
LOG_DIR="/tmp/nidar_sim_logs"
GCS_PORT="${GCS_PORT:-5173}"
DRONES="${1:-}"
# Gazebo's own GUI (rendering, not the physics/sensor server) is a heavy,
# separate process per launch and buys nothing for the actual workflow --
# operators interact through the GCS web map, not the Gazebo 3D viewport.
# Defaults to headless (server only) since a 4-drone full stack has
# repeatedly pushed this machine into severe resource contention (load
# average 40-60+, confirmed live) that degrades AS2's own timing-sensitive
# behaviors (arm/offboard state propagation, takeoff acceptance) badly
# enough to cause real mission failures independent of any code bug.
# Set NIDAR_GUI=true to opt back into the GUI for visual debugging.
NIDAR_GUI="${NIDAR_GUI:-false}"
# DDS middleware. Defaults to CycloneDDS, not the ROS 2 default FastDDS.
#
# A 4-drone AS2 stack plus detection is ~100 DDS participants on one machine.
# FastDDS gives every participant its own shared-memory port lock file in
# /dev/shm and, at that scale, new participants start failing to acquire one:
#   [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7661:
#   open_and_lock_file failed
# Nodes still start and log "ready", but come up only partially connected --
# confirmed live, with 371 shm files (and 7 GB of /dev/shm still free, so it
# is contention, not capacity): gcs_bridge hit it at startup, and a
# subscriber could match only 1 of 4 camera feeds. Silent half-connection is
# the worst possible failure mode here, and it is what "the video feed does
# not show up" and "the mission never started" both looked like.
#
# CycloneDDS was tried as a fix (it has no per-participant SHM port locks)
# and does solve discovery -- all four drones' GPS streamed, where FastDDS had
# left drone1 and drone3 silent. But it drops the raw 1280x960 camera images
# (3.6 MB a frame) entirely without buffer tuning: every detection node
# reported "no camera frames received yet". That trades an intermittent
# problem for a total one, so FastDDS stays the default.
#
# NIDAR_RMW=rmw_cyclonedds_cpp switches, for debugging discovery problems
# where the camera is not needed. Making it work properly for images needs
# CYCLONEDDS_URI with a raised MaxMessageSize and socket buffers.
export RMW_IMPLEMENTATION="${NIDAR_RMW:-rmw_fastrtps_cpp}"
LAUNCH_AS2_ARGS=(-m -n "$DRONES")
if [[ "$NIDAR_GUI" != "true" ]]; then
  LAUNCH_AS2_ARGS=(-e "${LAUNCH_AS2_ARGS[@]}")
fi

# tmuxinator (which launch_as2.bash uses to bring up the per-drone AS2 stack)
# shells out to tmux, and tmux refuses to run under TERM=dumb with
# "open terminal failed: not a terminal". A non-interactive caller -- nohup,
# a CI runner, an agent shell -- often inherits exactly that, and the failure
# is easy to misread: the script still prints "simulation is up" because
# rosbridge and the nidar nodes start fine, while Gazebo and every drone
# never launched at all. Confirmed live: sim.log held only that one line.
if [[ -z "${TERM:-}" || "$TERM" == "dumb" || "$TERM" == "unknown" ]]; then
  export TERM=xterm-256color
fi

mkdir -p "$LOG_DIR"

log() { echo "[$(date +%T)] $*"; }

# ---------------------------------------------------------------------------
# 1. Full teardown of anything left over from a previous run. Pattern list
#    intentionally broad — a partial match list here is exactly what let a
#    stale mission_file_executor.py silently corrupt a test earlier.
# ---------------------------------------------------------------------------
stop_all() {
  log "Stopping any previous simulation processes..."
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "as2_platform_gazebo_node" 2>/dev/null
  pkill -9 -f "as2_" 2>/dev/null
  pkill -9 -f "rosbridge_websocket" 2>/dev/null
  pkill -9 -f "rosapi_node" 2>/dev/null
  # mission_file_executor.py was split into 3 single-responsibility ROS 2
  # packages (see DOCUMENTS/standard_implementation_plan_ros2_framework.md)
  # each launched via `ros2 run` as its own binary -- same orphaning risk
  # as parameter_bridge/static_transform_publisher below, so each needs its
  # own explicit pattern rather than relying on the "ros2 run" pkill above.
  pkill -9 -f "gcs_bridge_node" 2>/dev/null
  pkill -9 -f "mission_executor_node" 2>/dev/null
  pkill -9 -f "survivor_manager_node" 2>/dev/null
  pkill -9 -f "mission_clock_node" 2>/dev/null
  pkill -9 -f "detection_node" 2>/dev/null
  # nidar_detection also ships a dataset-capture tool that arms and flies a
  # drone. It is run by hand rather than by this script, but it must still be
  # killed here -- confirmed live, three orphaned copies survived a stop and
  # sat holding DDS participants plus a live drone interface.
  pkill -9 -f "dataset_capture_node" 2>/dev/null
  pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f "ros2 run" 2>/dev/null
  # `ros_gz_bridge`'s parameter_bridge is spawned by `ros2 launch` as a
  # *child* process (one per drone, bridging IMU/pose/battery/cmd_vel/
  # camera topics, plus one for /clock) with its own executable path, not a
  # `ros2 launch`/`ros2 run` command line — so it never matched any pattern
  # above. kill -9 on the `ros2 launch` parent skips its normal child
  # shutdown handling, so these were orphaned (reparented to `systemd
  # --user`) instead of dying with their parent, on every single
  # stop/relaunch cycle. Confirmed: 47 of them found still running from
  # launches as much as an hour+ earlier in one debugging session, all
  # fighting fresh bridges for the exact same per-drone topic names. This —
  # not launch ordering — is what was actually behind drone2/drone3 never
  # getting a working gimbal camera.
  pkill -9 -f "parameter_bridge" 2>/dev/null
  # Same orphaning mechanism, different binary: tf2_ros's
  # static_transform_publisher (one per drone, for the gimbal camera optical
  # frame) is a stock ROS 2 tool, not an AS2 one, so it doesn't match the
  # "as2_" pattern above either — and like parameter_bridge, its own command
  # line never contains "ros2 launch"/"ros2 run". Confirmed: 94 of these
  # found still running, accumulated one stop/relaunch cycle at a time
  # across a single long debugging session, degrading DDS discovery badly
  # enough to make basic pub/sub matching unreliable session-wide (not just
  # for the drones directly using them). Any other stock ROS 2 tool spawned
  # the same way by launch_as2.bash would leak identically — if a new one
  # shows up, add it here the same way.
  pkill -9 -f "static_transform_publisher" 2>/dev/null
  # Note: this also kills a session the launcher itself might be running in,
  # so run_simulation.sh cannot be started with `tmux new-session -d`. Killing
  # only the drone sessions instead was tried and is NOT a safe swap: leaving
  # the tmux server alive carries stale state into the next launch and the
  # per-drone AS2 stacks then failed to come up at all (the startup health
  # check timed out with platform/gimbal_bridge processes missing). Launch
  # detached with `setsid nohup` rather than from inside tmux.
  tmux kill-server 2>/dev/null
  lsof -ti:"$GCS_PORT" -sTCP:LISTEN 2>/dev/null | xargs -r kill -9
  # Same kill -9-skips-cleanup problem applies to FastDDS's own
  # /dev/shm/fastrtps_* shared-memory segments: they never get released,
  # and left unchecked (520+ found accumulated in one session) a new DDS
  # participant created later — e.g. a drone's platform/gimbal_bridge node
  # on a busy multi-drone launch — can hang acquiring its own SHM port and
  # never join the ROS graph at all (process alive, invisible to `ros2 node
  # list`, forever — not a slow start). Safe to clear here: every process
  # that could hold one open has just been killed above. Hard-kill the
  # daemon too, not `ros2 daemon stop` — that's itself a DDS call and can
  # hang under the exact contention this cleanup exists to fix (confirmed
  # hanging indefinitely mid-testing).
  pkill -9 -f "ros2-daemon" 2>/dev/null

  # sem.fastrtps_* too, not just fastrtps_*: the port *lock* files are the
  # ones that actually exhaust the pool. Confirmed live -- 93 stale
  # segments survived this cleanup because it only removed the segments and
  # left every sem.fastrtps_port#### behind, and a freshly-launched sim
  # then published zero camera frames while every new node logged
  # "Failed init_port fastrtps_port####: open_and_lock_file failed".
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

  # Verify the teardown actually finished instead of assuming a fixed sleep
  # was long enough. A launch that begins while the previous sim is still
  # dying ends up with BOTH running: confirmed live, a fresh sim came up
  # beside a stale one and their detection nodes fought for the same camera
  # topics -- two nodes reported 16721 frames seen while the two new ones
  # reported 1. From the outside that looks exactly like "the simulation is
  # not starting" and "the video frames are not working".
  #
  # The shm cleanup above also only helps if it runs when everything is
  # really dead; re-running it after the wait catches anything that released
  # its segments on the way out.
  local pattern='gz sim|as2_|parameter_bridge|static_transform_publisher|detection_node|dataset_capture_node|lib/nidar_|rosbridge_websocket'
  local remaining=0
  for _ in $(seq 1 20); do
    remaining=$(pgrep -f "$pattern" 2>/dev/null | grep -vc "^$$\$" || true)
    [[ "${remaining:-0}" -eq 0 ]] && break
    sleep 1
  done
  if [[ "${remaining:-0}" -ne 0 ]]; then
    log "WARNING: $remaining simulation process(es) survived teardown; killing again"
    pkill -9 -f "$pattern" 2>/dev/null
    sleep 2
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  return 0
}

if [[ "${1:-}" == "stop" ]]; then
  stop_all
  log "Stopped."
  exit 0
fi

stop_all

# ---------------------------------------------------------------------------
# 2. Environment (ROS 2 + Gazebo + PATH fix + GZ_IP/GZ_PARTITION)
# ---------------------------------------------------------------------------
log "Loading ROS/Gazebo environment..."
source "$SCRIPT_DIR/setup_nidar_ros.sh"

# ---------------------------------------------------------------------------
# 3. Drone list: default to every drone in world_swarm.yaml (not a hardcoded
#    subset) so Gazebo's spawned entities and AS2's running nodes always match.
# ---------------------------------------------------------------------------
cd "$PROJECT_GAZEBO"

log "Syncing survivor placements from config/survivors.yaml..."
python3 utils/sync_survivors.py || exit 1

if [[ -z "$DRONES" ]]; then
  DRONES=$(python3 utils/get_drones.py -p config/world_swarm.yaml --sep ',')
fi
log "Drones: $DRONES"

# ---------------------------------------------------------------------------
# 4. Gazebo + AS2 stack for every drone
# ---------------------------------------------------------------------------
log "Launching Gazebo + AS2 (this takes ~15-20s)..."
nohup ./launch_as2.bash "${LAUNCH_AS2_ARGS[@]}" > "$LOG_DIR/sim.log" 2>&1 &

IFS=',' read -r -a DRONE_ARR <<< "$DRONES"

# Check every drone's platform + gimbal_bridge processes actually exist, not
# just a GPS topic on drone0 — a GPS-only check can't catch a drone whose
# platform/gimbal never came up. This used to happen for real: gimbal_bridge
# had a startup race in its vendored source (subscribed to a gz-transport
# topic before creating the ROS publishers its own callback uses, so an
# early message could segfault it — fixed in
# framework_ws/src/aerostack2/.../gimbal_bridge.cpp).
#
# Checking via `ros2 node list` (DDS graph discovery) rather than the OS
# process directly was tried and abandoned: under the load of 4 full drone
# stacks + gz sim's GUI on one machine, discovery latency alone regularly
# exceeded 150s even with zero crashes — confirmed via gdb and direct
# process inspection that the nodes were alive and healthy the whole time,
# `ros2 node list` was just slow to learn about them. Checking `pgrep` for
# the actual OS process instead sidesteps DDS discovery entirely: it's
# instant, and a segfaulted process shows 0 matches immediately rather than
# eventually — a strictly better signal for exactly the failure this checks
# for.
# Startup budget. 60s was tight enough to fail on a loaded machine even when
# the sim was coming up fine -- 4 drones' AS2 stacks, Gazebo rendering 4
# cameras, and 4 YOLO models loading onto one GPU all compete during exactly
# this window, and a false ERROR here tears down a healthy sim. Override with
# NIDAR_STARTUP_TIMEOUT if a slower machine needs longer still.
SIM_STARTUP_TIMEOUT="${NIDAR_STARTUP_TIMEOUT:-150}"
sim_ready=false
for _ in $(seq 1 "$SIM_STARTUP_TIMEOUT"); do
  all_up=true
  for d in "${DRONE_ARR[@]}"; do
    if ! pgrep -f "as2_platform_gazebo_node.*__ns:=/${d} " >/dev/null 2>&1 \
      || ! pgrep -f "gimbal_bridge --ros-args -r __ns:=/${d} " >/dev/null 2>&1; then
      all_up=false
      break
    fi
  done
  if [[ "$all_up" == "true" ]]; then
    sim_ready=true
    break
  fi
  sleep 1
done
# Trust the loop's own result rather than re-running the same check
# independently right after — a transient hiccup on a second call (e.g.
# under the CPU load of a heavier scene just spawning) was reporting ERROR
# and exiting even when the sim had genuinely come up fine, and — worse —
# skipped stop_all() on the way out, leaving Gazebo/tmux orphaned. Confirmed
# the hard way testing the survivor_actor model swap.
if [[ "$sim_ready" != "true" ]]; then
  log "ERROR: sim did not come up within ${SIM_STARTUP_TIMEOUT}s — one or more drones are missing platform/gimbal_bridge processes, check $LOG_DIR/sim.log"
  stop_all
  exit 1
fi
log "Sim is up."

# ---------------------------------------------------------------------------
# 5. rosbridge
# ---------------------------------------------------------------------------
log "Starting rosbridge_server..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG_DIR/rosbridge.log" 2>&1 &

for _ in $(seq 1 20); do
  grep -q "Rosbridge WebSocket server started" "$LOG_DIR/rosbridge.log" 2>/dev/null && break
  sleep 1
done
log "rosbridge is up on ws://localhost:9090."

# ---------------------------------------------------------------------------
# 6. NIDAR nodes — gcs_bridge (GCS <-> typed /nidar/* topics), mission_executor
#    (flight orchestration + manual drone control), survivor_manager (runtime
#    Gazebo survivor spawn/remove), mission_clock (mission elapsed time +
#    per-phase breakdown). Exactly one instance of each, or the GCS's
#    mission_load/mission_start topics race between old and new instances
#    (see header). Split out of the old single mission_file_executor.py --
#    see DOCUMENTS/standard_implementation_plan_ros2_framework.md.
# ---------------------------------------------------------------------------
log "Starting NIDAR nodes (gcs_bridge, mission_executor, survivor_manager, mission_clock)..."
nohup ros2 run nidar_mission_executor mission_executor_node > "$LOG_DIR/mission_executor.log" 2>&1 &
nohup ros2 run nidar_survivor_manager survivor_manager_node > "$LOG_DIR/survivor_manager.log" 2>&1 &
nohup ros2 run nidar_gcs_bridge gcs_bridge_node --ros-args \
  -p "drone_ids:=[$(echo "$DRONES" | sed 's/[^,]*/"&"/g')]" > "$LOG_DIR/gcs_bridge.log" 2>&1 &
nohup ros2 run nidar_mission_clock mission_clock_node > "$LOG_DIR/mission_clock.log" 2>&1 &
sleep 2
log "NIDAR nodes are up."

# ---------------------------------------------------------------------------
# 6b. Detection (Phase 2) — one node per drone, each on its own camera.
#     ON by default. It was opt-in at first (a YOLO model per drone on a
#     shared GPU is not free), but that meant a normal launch produced no
#     detection topics at all and the GCS camera panel sat on "No signal"
#     with nothing in any log to explain why -- confirmed live, a whole run
#     with no detection.log because the nodes were simply never started.
#     Detection is the point of Phase 2; make it the default and let it be
#     turned OFF explicitly instead. NIDAR_DETECTION=false skips it.
# ---------------------------------------------------------------------------
NIDAR_DETECTION="${NIDAR_DETECTION:-true}"
if [[ "$NIDAR_DETECTION" == "true" ]]; then
  DETECTION_MODEL="${NIDAR_DETECTION_MODEL:-$PROJECT_GAZEBO/models/detection/nidar_person.pt}"
  # -e, not -f: an ncnn export is a DIRECTORY (nidar_person_ncnn_model/), and
  # -f silently rejected it as "model not found" while detection quietly did
  # not start at all.
  if [[ ! -e "$DETECTION_MODEL" ]]; then
    log "WARNING: detection model not found at $DETECTION_MODEL — skipping detection nodes"
  else
    # 'auto' picks CUDA when torch can genuinely reach it and CPU otherwise;
    # on CPU the node then loads the ncnn export beside the weights, which is
    # 3x faster than PyTorch-on-CPU at the same input size. Set
    # NIDAR_DETECTION_DEVICE=cpu to exercise that ncnn path deliberately --
    # it is the backend the competition companion computer will actually run,
    # so being able to test it on a box that HAS a GPU matters.
    log "Starting detection nodes (model: $(basename "$DETECTION_MODEL"), device: ${NIDAR_DETECTION_DEVICE:-auto})..."
    nohup ros2 launch nidar_detection detection.launch.py \
      "drone_ids:=$DRONES" "model_path:=$DETECTION_MODEL" \
      "inference_rate_hz:=${NIDAR_DETECTION_HZ:-2.0}" \
      "confidence_threshold:=${NIDAR_DETECTION_CONF:-0.5}" \
      "device:=${NIDAR_DETECTION_DEVICE:-auto}" \
      > "$LOG_DIR/detection.log" 2>&1 &
    sleep 3
    log "Detection nodes are up."
  fi
fi

# ---------------------------------------------------------------------------
# 7. GCS dev server
# ---------------------------------------------------------------------------
if [[ ! -d "$GCS_DIR/node_modules" ]]; then
  log "GCS dependencies not installed yet, running npm install..."
  (cd "$GCS_DIR" && npm install > "$LOG_DIR/npm_install.log" 2>&1)
fi
log "Starting GCS dev server on port $GCS_PORT..."
(cd "$GCS_DIR" && nohup npm run dev -- --port "$GCS_PORT" > "$LOG_DIR/gcs.log" 2>&1 &)

for _ in $(seq 1 20); do
  curl -sf "http://localhost:$GCS_PORT" >/dev/null 2>&1 && break
  sleep 1
done

echo ""
echo "================================================================"
echo " NIDAR RescueSwarm simulation is up"
echo "================================================================"
echo " GCS:         http://localhost:$GCS_PORT"
echo " rosbridge:   ws://localhost:9090"
echo " Drones:      $DRONES"
echo " Logs:        $LOG_DIR/{sim,rosbridge,executor,gcs}.log"
echo ""
echo " Press Ctrl+C to stop everything, or run:"
echo "   ./scripts/run_simulation.sh stop"
echo "================================================================"

cleanup() {
  echo ""
  stop_all
  log "Stopped."
  exit 0
}
trap cleanup SIGINT SIGTERM

# Keep the script alive so Ctrl+C has something to catch; the launched
# processes are already detached (nohup + background), not children we wait() on.
while true; do sleep 3600; done
