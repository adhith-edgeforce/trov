#!/bin/bash
# ============================================================
# save_map_outdoor.sh
# 1) Move the temp PCD (from stop_mapping_outdoor.sh) -> pcd_maps/<name>.pcd
# 2) Convert it to a 2D occupancy grid (grid_maps/<name>.pgm + .yaml) via the
#    running pcd_to_grid node, so the map appears in the UI maps list + display.
#
# Usage:
#   ./save_map_outdoor.sh <map_name>
# ============================================================

source /opt/ros/humble/setup.bash
source /data/trov_ws/install/setup.bash

# --- Paths (keep in sync with the API + restart_localization_outdoor.sh) ---
MAPS_DIR="/data/trov_ws/pcd_maps"
GRID_DIR="/data/trov_ws/pcd_maps/grid_maps"
TEMP_MAP_PATH="/data/pcd_map_temp"
NODE="/pcd_to_grid"                 # node name from pcd_to_grid.launch.py (name='pcd_to_grid')
MAP_NAME="${1:-}"

# --- Validate name ---
if [ -z "$MAP_NAME" ]; then
    echo "ERROR: No map name provided"; echo "Usage: ./save_map_outdoor.sh <map_name>"; exit 1
fi
if [[ ! "$MAP_NAME" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo "ERROR: Invalid map name '$MAP_NAME'. Use letters, numbers, underscores, hyphens."; exit 1
fi

# --- Temp PCD present? ---
if [ ! -f "${TEMP_MAP_PATH}.pcd" ]; then
    echo "ERROR: Temp PCD not found at ${TEMP_MAP_PATH}.pcd (run stop_mapping_outdoor.sh first)"; exit 1
fi

mkdir -p "$MAPS_DIR" "$GRID_DIR"

# --- Collision check (pcd + grid) ---
DEST_PCD="$MAPS_DIR/$MAP_NAME.pcd"
if [ -f "$DEST_PCD" ] || [ -f "$GRID_DIR/$MAP_NAME.yaml" ]; then
    echo "ERROR: Map '$MAP_NAME' already exists. Choose a different name."; exit 1
fi

# --- 1) Save the PCD ---
cp "${TEMP_MAP_PATH}.pcd" "$DEST_PCD" || { echo "ERROR: Failed to copy PCD"; exit 1; }
rm -f "${TEMP_MAP_PATH}.pcd"
echo "OK: PCD saved -> $DEST_PCD"

# --- 2) Convert PCD -> occupancy grid via the running pcd_to_grid node ---
# CONFIRMED from pcd_to_grid.cpp: /convert_map -> convert() -> refreshParams()
# re-reads pcd_path/output_dir/map_name each call, so setting them here then
# calling /convert_map converts the NEW map into grid_maps/<name>.{pgm,yaml}.
ros2 param set "$NODE" pcd_path   "$DEST_PCD"   || echo "WARN: could not set pcd_path on $NODE"
ros2 param set "$NODE" output_dir "$GRID_DIR/"  || true
ros2 param set "$NODE" map_name   "$MAP_NAME"   || true
ros2 service call /convert_map std_srvs/srv/Empty

# --- FALLBACK: if your node only reads pcd_path from the yaml at startup ---
# (delete the block above and uncomment this; set the correct installed yaml path)
# PCD_TO_GRID_YAML="/data/trov_ws/install/pcd_to_grid/share/pcd_to_grid/config/pcd_to_grid.yaml"
# sed -i "s|^\(\s*pcd_path:\).*|\1 \"$DEST_PCD\"|" "$PCD_TO_GRID_YAML"
# sed -i "s|^\(\s*map_name:\).*|\1 \"$MAP_NAME\"|" "$PCD_TO_GRID_YAML"
# pkill -9 -f pcd_to_grid_node; sleep 1
# ros2 launch pcd_to_grid pcd_to_grid.launch.py &

# --- Verify the grid landed ---
sleep 2
if [ -f "$GRID_DIR/$MAP_NAME.yaml" ] && [ -f "$GRID_DIR/$MAP_NAME.pgm" ]; then
    echo "OK: Grid saved -> $GRID_DIR/$MAP_NAME.{pgm,yaml}"
else
    echo "WARN: Grid not found yet at $GRID_DIR/$MAP_NAME.* — check how pcd_to_grid"
    echo "      picks up the new pcd_path on /convert_map (param vs yaml-at-startup)."
fi
exit 0
