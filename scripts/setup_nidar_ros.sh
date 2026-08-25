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
export GZ_IP=$(hostname -I | awk '{print $1}')
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
