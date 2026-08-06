#!/bin/bash
# ============================================================
# start_mapping_outdoor.sh
# OUTDOOR counterpart of start_mapping.sh.
# Stops outdoor localization + navigation, then launches lidarslam
# (scanmatcher_node + graph_based_slam_node) to build a PCD map.
#
# Usage:
#   ./start_mapping_outdoor.sh
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

# lidarslam writes map.pcd relative to CWD when /map_save is called.
cd /data/trov_ws

echo "INFO: Killing any existing mapping processes..."
pkill -9 -f "scanmatcher_node"      2>/dev/null || true
pkill -9 -f "graph_based_slam_node" 2>/dev/null || true
pkill -9 -f "lidarslam_outdoor"     2>/dev/null || true
sleep 2

echo "INFO: Stopping outdoor localization..."
pkill -9 -f "outdoor_lidar_localization" 2>/dev/null || true
pkill -9 -f "nav2_lidar_localization"    2>/dev/null || true
pkill -9 -f "lidar_localization_node"    2>/dev/null || true

echo "INFO: Stopping navigation..."
pkill -9 -f "controller_server"            2>/dev/null || true
pkill -9 -f "planner_server"               2>/dev/null || true
pkill -9 -f "behavior_server"              2>/dev/null || true
pkill -9 -f "bt_navigator"                 2>/dev/null || true
pkill -9 -f "smoother_server"              2>/dev/null || true
pkill -9 -f "velocity_smoother"            2>/dev/null || true
pkill -9 -f "collision_monitor"            2>/dev/null || true
pkill -9 -f "waypoint_follower"            2>/dev/null || true
pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true

# NOTE: odometry left running (mirrors indoor). scanmatcher needs its point
# cloud (/points_raw remapped to /input_cloud) + /imu/data — keep the lidar up.

sleep 2

echo "INFO: Starting lidarslam (PCD mapping)..."
ros2 launch lidarslam lidarslam_outdoor.launch.py &

echo "OK: Outdoor mapping started (PID $!)"
exit 0
