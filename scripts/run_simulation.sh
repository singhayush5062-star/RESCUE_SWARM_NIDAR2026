#!/bin/bash
# NIDAR RescueSwarm — single-command full simulation launcher.
#
# Brings up: Gazebo + AS2 for every drone in project_gazebo/config/world_swarm.yaml,
# rosbridge_server, mission_file_executor.py, and the GCS dev server. Ctrl+C tears
# everything back down. Also syncs survivor placements from
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
#     trees underneath it, and a stale mission_file_executor.py left running
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
  pkill -9 -f "mission_file_executor.py" 2>/dev/null
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
  rm -f /dev/shm/fastrtps_* 2>/dev/null
  sleep 2
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
nohup ./launch_as2.bash -m -n "$DRONES" > "$LOG_DIR/sim.log" 2>&1 &

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
sim_ready=false
for _ in $(seq 1 60); do
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
  log "ERROR: sim did not come up within 60s — one or more drones are missing platform/gimbal_bridge processes, check $LOG_DIR/sim.log"
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
# 6. Mission executor — exactly one instance, or the GCS's mission_load/
#    mission_start topics race between old and new instances (see header).
# ---------------------------------------------------------------------------
log "Starting mission_file_executor.py..."
nohup python3 -u mission_file_executor.py > "$LOG_DIR/executor.log" 2>&1 &
sleep 2
log "Mission executor is up."

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
