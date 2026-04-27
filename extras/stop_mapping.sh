#!/bin/bash
# ============================================================
# stop_mapping.sh
# Stops GMapping and saves the current map to a temp location.
# The map stays at /tmp/trov_map_temp until save_map.sh is called.
#
# Usage:
#   ./stop_mapping.sh
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

TEMP_MAP_PATH="/tmp/trov_map_temp"

echo "INFO: Saving map to temp location: $TEMP_MAP_PATH"

# Save the map before killing gmapping
ros2 run nav2_map_server map_saver_cli \
    -f "$TEMP_MAP_PATH" \
    --ros-args -p save_map_timeout:=5.0

if [ $? -ne 0 ]; then
    echo "ERROR: map_saver_cli failed — map may not have been saved"
    exit 1
fi

echo "INFO: Map saved. Stopping GMapping..."
pkill -9 -f "async_slam_toolbox_node"  2>/dev/null || true
pkill -9 -f "slam_toolbox"             2>/dev/null || true
pkill -9 -f "pointcloud_to_laserscan"  2>/dev/null || true

sleep 1

echo "OK: Mapping stopped. Temp map at: ${TEMP_MAP_PATH}.yaml + ${TEMP_MAP_PATH}.pgm"
exit 0