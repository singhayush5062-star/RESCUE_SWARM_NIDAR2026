#!/bin/bash
# `conda deactivate` requires `conda init` to have run for this shell, which
# isn't guaranteed (e.g. non-interactive/CI shells). Strip conda's dirs from
# PATH directly instead, so ROS's python3 (3.10) always wins over conda's
# (which is often a newer, ABI-incompatible version rclpy can't load).
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd: -)

source /opt/ros/humble/setup.bash
source ~/NIDAR/framework_ws/install/setup.bash

export GZ_VERSION=harmonic

# Gazebo Transport's multicast discovery fails ("Network is unreachable") on
# machines with multiple/inactive network interfaces (docker0, unplugged
# ethernet, etc). Pinning GZ_IP to the machine's active interface fixes it.
# See DOCUMENTS/Phase0_Environment_Test_Report.md section 3.1.
# Resolve the IP fresh every launch, and make sure the interface carrying it
# can actually do multicast.
#
# Two ways this bites, both seen live:
#   1. The address goes stale. A sim launched on one network keeps its old
#      GZ_IP; move to another (or just reconnect wifi) and every bridge logs
#      "Exception sending a multicast message: Network is unreachable" while
#      Gazebo itself looks perfectly healthy. Symptom: not one drone appears
#      in the GCS, because no telemetry ever reaches ROS. Confirmed with a
#      sim still pinned to 10.10.149.36 on a machine that had become
#      192.168.1.7.
#   2. No multicast route exists at all, so discovery cannot leave the host
#      even with the right IP.
#
# `hostname -I` also lists docker/virtual addresses (172.17.0.1 and friends);
# picking blindly with `awk '{print $1}'` can hand Gazebo a bridge interface
# with no route to anything. Prefer the address of the interface that owns
# the default route, and fall back to the old behaviour.
GZ_IFACE=$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')
if [[ -n "$GZ_IFACE" ]]; then
  GZ_IP_RESOLVED=$(ip -4 -brief addr show "$GZ_IFACE" 2>/dev/null | awk '{print $3}' | cut -d/ -f1)
fi
if [[ -z "${GZ_IP_RESOLVED:-}" ]]; then
  GZ_IP_RESOLVED=$(hostname -I | awk '{print $1}')
  GZ_IFACE=""
fi
export GZ_IP="$GZ_IP_RESOLVED"

# Gazebo Transport discovery is multicast; without a route for 224.0.0.0/4 it
# cannot send at all. Adding it needs root, so only attempt it when we can do
# so without prompting, and say so plainly otherwise rather than failing later
# in a way that looks like a simulator bug.
if ! ip route show | grep -qE '^224\.0\.0\.0/4'; then
  if [[ -n "$GZ_IFACE" ]] && sudo -n true 2>/dev/null; then
    sudo -n ip route add 224.0.0.0/4 dev "$GZ_IFACE" 2>/dev/null \
      && echo "Added multicast route on $GZ_IFACE"
  else
    echo "WARNING: no 224.0.0.0/4 multicast route. If drones never appear in" \
         "the GCS, run: sudo ip route add 224.0.0.0/4 dev ${GZ_IFACE:-<iface>}"
  fi
fi
# FastDDS tuning is available but NOT enabled -- see config/fastdds_nidar.xml
# for the analysis and the exact reason it is off. Short version: the default
# 512 KB shared-memory segment is genuinely too small for a camera frame
# (640x480x3 = 921 KB), but the profile that fixes it also has to avoid
# touching the builtin discovery transport, and the version tested here
# regressed the graph from 3/4 drones to 1/4. Enable deliberately, and
# re-measure, with:
#   export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/NIDAR/config/fastdds_nidar.xml

# DDS also needs kernel socket buffers far larger than Ubuntu's default for
# its UDP path. This cannot be set from here (needs root); warn once so it is
# not mistaken for a simulator fault later.
if [[ "$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)" -lt 8388608 ]]; then
  echo "NOTE: net.core.rmem_max is $(cat /proc/sys/net/core/rmem_max) (small for DDS)." \
       "For best throughput run once:" \
       "sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216"
fi

export GZ_PARTITION=nidar_$(whoami)

# Custom world objects (e.g. project_gazebo/models/survivor for Phase 0.4's
# simulated survivor placeholders) live outside the AS2 install tree so they
# survive a colcon rebuild. as2_gazebo_assets' Object model resolution checks
# GZ_SIM_RESOURCE_PATH in addition to its own bundled models/ directory.
export GZ_SIM_RESOURCE_PATH="$HOME/NIDAR/project_gazebo/models:$GZ_SIM_RESOURCE_PATH"

echo "NIDAR ROS environment loaded"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "GZ_VERSION=$GZ_VERSION"
echo "GZ_IP=$GZ_IP"
echo "GZ_PARTITION=$GZ_PARTITION"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
