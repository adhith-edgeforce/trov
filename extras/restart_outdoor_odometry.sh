#!/bin/bash
# ============================================================
# restart_odometry.sh
# Kills ICP odometry and relaunches it.
#
# Usage:
#   ./restart_odometry.sh
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

ICP_LAUNCH_FILE="icp_odometry_outdoor.launch.py"

echo "INFO: Restarting ICP odometry..."

# Kill existing odometry processes
pkill -9 -f "icp_odometry" 2>/dev/null || true
pkill -9 -f "rtabmap_odom" 2>/dev/null || true
sleep 2

# Relaunch in background
ros2 launch trov "$ICP_LAUNCH_FILE" &

echo "OK: ICP Odometry launched (PID $!)"
exit 0
