#!/bin/bash
# ============================================================
# restart_navigation.sh
# Kills navigation stack and relaunches it.
#
# Usage:
#   ./restart_navigation.sh
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

NAVIGATION_LAUNCH_FILE="navigation_indoor.launch.py"

echo "INFO: Restarting navigation stack..."

# Kill existing navigation processes
pkill -9 -f "controller_server"            2>/dev/null || true
pkill -9 -f "planner_server"               2>/dev/null || true
pkill -9 -f "behavior_server"              2>/dev/null || true
pkill -9 -f "bt_navigator"                 2>/dev/null || true
pkill -9 -f "smoother_server"              2>/dev/null || true
pkill -9 -f "velocity_smoother"            2>/dev/null || true
pkill -9 -f "collision_monitor"            2>/dev/null || true
pkill -9 -f "waypoint_follower"            2>/dev/null || true
pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true
sleep 2

# Relaunch in background
ros2 launch trov "$NAVIGATION_LAUNCH_FILE" &

echo "OK: Navigation launched (PID $!)"
exit 0