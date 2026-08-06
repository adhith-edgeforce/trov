#!/bin/bash
# ============================================================
# stop_mapping_outdoor.sh  (hardened error reporting)
# Saves the lidarslam PCD (/map_save), moves it to the temp path, stops lidarslam.
# ============================================================

# Source ROS quietly — trov_ws chains a stale autoware underlay that prints a
# harmless "not found" on stderr; silence it so REAL errors surface in the UI.
source /opt/ros/humble/setup.bash       2>/dev/null
source /data/trov_ws/install/setup.bash 2>/dev/null

# >>> CONFIRM where /map_save writes map.pcd (run it manually + `find`) <<<
PCD_OUTPUT="/data/trov_ws/map.pcd"
TEMP_MAP_PATH="/data/pcd_map_temp"

rm -f "$PCD_OUTPUT"

echo "INFO: Requesting lidarslam to save the map..."
ros2 service call /map_save std_srvs/srv/Empty || echo "WARN: /map_save returned non-zero" >&2

echo "INFO: Waiting for $PCD_OUTPUT ..."
waited=0
while [ $waited -lt 60 ]; do          # 60s — large outdoor clouds take a while
    [ -f "$PCD_OUTPUT" ] && break
    sleep 1
    waited=$((waited + 1))
done

if [ ! -f "$PCD_OUTPUT" ]; then
    {
      echo "ERROR: Expected PCD not found at $PCD_OUTPUT after 60s."
      echo "Recently-written .pcd files (where /map_save actually wrote it):"
      find /data /home/nvidia -maxdepth 3 -name '*.pcd' -newermt '-3 min' 2>/dev/null | sed 's/^/  /'
      echo "-> Set PCD_OUTPUT in stop_mapping_outdoor.sh to the correct path."
    } >&2                              # to STDERR so the UI shows the REAL cause
    exit 1
fi

mv "$PCD_OUTPUT" "${TEMP_MAP_PATH}.pcd"

echo "INFO: Stopping lidarslam..."
pkill -9 -f "scanmatcher_node"      2>/dev/null || true
pkill -9 -f "graph_based_slam_node" 2>/dev/null || true
pkill -9 -f "lidarslam_outdoor"     2>/dev/null || true
pkill -9 -f "rviz2"                 2>/dev/null || true
sleep 1

echo "OK: Mapping stopped. Temp PCD at: ${TEMP_MAP_PATH}.pcd"
exit 0
