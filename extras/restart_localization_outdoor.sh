#!/bin/bash
# ============================================================
# restart_localization_outdoor.sh
# OUTDOOR counterpart of restart_localization.sh.
# Switches the PCD map by rewriting map_path in outdoor_localization.yaml,
# then relaunches the pinned localization + outdoor waypoint/delivery nodes.
#
# Usage:
#   ./restart_localization_outdoor.sh <map_name>
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

# ── CONFIG — KEEP IN SYNC WITH the API outdoor block + save_map_outdoor.sh ──
MAPS_DIR="/data/trov_ws/pcd_maps"
MAP_EXT=".pcd"
MAP_NAME="${1:-map_outdoor}"
MAP_PATH="$MAPS_DIR/$MAP_NAME$MAP_EXT"

# The yaml the pinned launch actually reads (src path, per the launch file):
LOCALIZATION_YAML="/data/trov_ws/src/trov/config/outdoor/outdoor_localization.yaml"
MAP_KEY="map_path"   # confirmed from outdoor_localization.yaml

# Validate map exists
if [ ! -f "$MAP_PATH" ]; then
    echo "ERROR: Map '$MAP_NAME' not found at $MAP_PATH"
    echo "Available maps:"
    ls "$MAPS_DIR"/*"$MAP_EXT" 2>/dev/null | xargs -I{} basename {} "$MAP_EXT" | sed 's/^/  /'
    exit 1
fi

echo "INFO: Restarting OUTDOOR localization with map: $MAP_NAME"

# Kill existing outdoor localization (pinned wrapper + nested launch + node)
pkill -9 -f "outdoor_lidar_localization" 2>/dev/null || true
pkill -9 -f "nav2_lidar_localization"    2>/dev/null || true
pkill -9 -f "lidar_localization_node"    2>/dev/null || true

# Kill waypoint + delivery nodes so they restart fresh with the new map
pkill -9 -f "outdoor_waypoints_follower" 2>/dev/null || true
pkill -9 -f "outdoor_waypoints_recorder" 2>/dev/null || true
pkill -9 -f "delivery_runner_outdoor"    2>/dev/null || true

sleep 2

# ── Point localization at the selected PCD by rewriting map_path in the yaml ──
# Matches ONLY the active (uncommented) map_path line; leaves #map_path lines alone.
if [ ! -f "$LOCALIZATION_YAML" ]; then
    echo "ERROR: Localization yaml not found: $LOCALIZATION_YAML"
    exit 1
fi
sed -i "s|^\([[:space:]]*\)${MAP_KEY}:.*|\1${MAP_KEY}: \"${MAP_PATH}\"|" "$LOCALIZATION_YAML"
echo "INFO: Set ${MAP_KEY} -> ${MAP_PATH} in $(basename "$LOCALIZATION_YAML")"

# Relaunch the pinned localization (NO extra args — it declares none)
ros2 launch trov outdoor_lidar_localization_pinned.launch.py &
LOC_PID=$!
echo "INFO: Outdoor localization launched (PID $LOC_PID)"

echo "INFO: Waiting 10s for localization to initialize..."
sleep 10

# Persist selected outdoor map (separate file from indoor's .trov_last_map)
echo "$MAP_NAME" > "$HOME/.trov_last_pcd_map"

# Relaunch waypoint + delivery nodes
ros2 run trov outdoor_waypoints_follower.py &
echo "INFO: outdoor_waypoints_follower started (PID $!)"

ros2 run trov outdoor_waypoints_recorder.py &
echo "INFO: outdoor_waypoints_recorder started (PID $!)"

ros2 run trov delivery_runner_outdoor.py &
echo "INFO: delivery_runner_outdoor started (PID $!)"

echo "OK: Outdoor localization + waypoint + delivery nodes launched (map: $MAP_NAME)"
exit 0
