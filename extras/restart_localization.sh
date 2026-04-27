# #!/bin/bash
# # ============================================================
# # restart_localization.sh
# # Kills localization + waypoint nodes and relaunches with given map name.
# #
# # Usage:
# #   ./restart_localization.sh <map_name>
# #   ./restart_localization.sh warehouse2
# # ============================================================

# source /opt/ros/humble/setup.bash
# source /data/trov_ws/install/setup.bash

# MAPS_DIR="/data/trov_ws/install/trov/share/trov/maps"
# MAP_NAME="${1:-indoor3}"
# MAP_PATH="$MAPS_DIR/$MAP_NAME.yaml"

# # Validate map exists
# if [ ! -f "$MAP_PATH" ]; then
#     echo "ERROR: Map '$MAP_NAME' not found at $MAP_PATH"
#     echo "Available maps:"
#     ls "$MAPS_DIR"/*.yaml | xargs -I{} basename {} .yaml | sed 's/^/  /'
#     exit 1
# fi

# echo "INFO: Restarting localization with map: $MAP_NAME"

# # Kill existing localization processes
# pkill -9 -f "map_server"                     2>/dev/null || true
# pkill -9 -f "amcl"                           2>/dev/null || true
# pkill -9 -f "pointcloud_to_laserscan"        2>/dev/null || true
# pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true

# # Kill waypoint nodes so they restart fresh with the new map
# pkill -9 -f "waypoint_follower_poses"        2>/dev/null || true
# pkill -9 -f "waypoint_recorder"              2>/dev/null || true

# sleep 2

# # Relaunch localization
# ros2 launch trov localization.launch.py map_name:="$MAP_NAME" &
# LOC_PID=$!
# echo "INFO: Localization launched with map: $MAP_NAME (PID $LOC_PID)"

# # Wait for map_server to be up before starting waypoint nodes
# # They need map_server's yaml_filename parameter to be populated at init
# echo "INFO: Waiting 10s for map_server to initialize..."
# sleep 10

# echo "$new_map_name" > "$HOME/.trov_last_map"

# # Relaunch waypoint nodes
# ros2 run trov waypoint_follower_poses.py &
# echo "INFO: waypoint_follower_poses started (PID $!)"

# ros2 run trov waypoint_recorder.py &
# echo "INFO: waypoint_recorder started (PID $!)"

# echo "OK: Localization + waypoint nodes launched with map: $MAP_NAME"
# exit 0


#!/bin/bash
# ============================================================
# restart_localization.sh
# Kills localization + waypoint nodes and relaunches with given map name.
#
# Usage:
#   ./restart_localization.sh <map_name>
#   ./restart_localization.sh warehouse2
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

MAPS_DIR="/data/trov_ws/install/trov/share/trov/maps"
MAP_NAME="${1:-indoor3}"
MAP_PATH="$MAPS_DIR/$MAP_NAME.yaml"

# Validate map exists
if [ ! -f "$MAP_PATH" ]; then
    echo "ERROR: Map '$MAP_NAME' not found at $MAP_PATH"
    echo "Available maps:"
    ls "$MAPS_DIR"/*.yaml | xargs -I{} basename {} .yaml | sed 's/^/  /'
    exit 1
fi

echo "INFO: Restarting localization with map: $MAP_NAME"

# Kill existing localization processes
pkill -9 -f "map_server"                     2>/dev/null || true
pkill -9 -f "amcl"                           2>/dev/null || true
pkill -9 -f "pointcloud_to_laserscan"        2>/dev/null || true
pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true

# Kill waypoint nodes so they restart fresh with the new map
pkill -9 -f "waypoint_follower_poses"        2>/dev/null || true
pkill -9 -f "waypoint_recorder"              2>/dev/null || true

# Kill delivery_runner so it re-detects the new map on restart
pkill -9 -f "delivery_runner"               2>/dev/null || true

sleep 2

# Relaunch localization
ros2 launch trov localization.launch.py map_name:="$MAP_NAME" &
LOC_PID=$!
echo "INFO: Localization launched with map: $MAP_NAME (PID $LOC_PID)"

# Wait for map_server to be up before starting waypoint nodes
# They need map_server's yaml_filename parameter to be populated at init
echo "INFO: Waiting 10s for map_server to initialize..."
sleep 10

echo "$MAP_NAME" > "$HOME/.trov_last_map"

# Relaunch waypoint nodes
ros2 run trov waypoint_follower_poses.py &
echo "INFO: waypoint_follower_poses started (PID $!)"

ros2 run trov waypoint_recorder.py &
echo "INFO: waypoint_recorder started (PID $!)"

# Relaunch delivery_runner after waypoint nodes are up
# It contacts map_server on init — map_server has been up for 10s+ so it resolves immediately
ros2 run trov delivery_runner.py &
echo "INFO: delivery_runner started (PID $!)"

echo "OK: Localization + waypoint + delivery nodes launched with map: $MAP_NAME"
exit 0