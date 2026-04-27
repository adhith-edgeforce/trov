#!/bin/bash
# ============================================================
# start_mapping.sh
# Stops localization + navigation, then launches GMapping SLAM.
#
# Usage:
#   ./start_mapping.sh
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

MAPPING_LAUNCH_FILE="slam.launch.py"   # ← update to your actual launch file name

echo "INFO: Killing any existing mapping processes..."
pkill -9 -f "async_slam_toolbox_node"  2>/dev/null || true
pkill -9 -f "slam_toolbox"             2>/dev/null || true
pkill -9 -f "pointcloud_to_laserscan"  2>/dev/null || true
sleep 2

echo "INFO: Stopping localization..."
pkill -9 -f "map_server"                    2>/dev/null || true
pkill -9 -f "amcl"                          2>/dev/null || true
pkill -9 -f "pointcloud_to_laserscan"       2>/dev/null || true
pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true

echo "INFO: Stopping navigation..."
pkill -9 -f "controller_server"             2>/dev/null || true
pkill -9 -f "planner_server"               2>/dev/null || true
pkill -9 -f "behavior_server"              2>/dev/null || true
pkill -9 -f "bt_navigator"                 2>/dev/null || true
pkill -9 -f "smoother_server"              2>/dev/null || true
pkill -9 -f "velocity_smoother"            2>/dev/null || true
pkill -9 -f "collision_monitor"            2>/dev/null || true
pkill -9 -f "waypoint_follower"            2>/dev/null || true
pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true

sleep 2

echo "INFO: Starting GMapping SLAM..."
ros2 launch trov "$MAPPING_LAUNCH_FILE" &

echo "OK: Mapping started (PID $!)"
exit 0