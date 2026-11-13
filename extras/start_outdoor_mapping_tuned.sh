#!/bin/bash
# ============================================================
# start_outdoor_mapping_tuned.sh   (added 2026-11-11)
#
# One command to build a new OUTDOOR PCD map with the lidarslam settings tuned
# on 2026-11-10 (the ones that produced map 506_ver1):
#   config : /data/trov_ws/src/trov/config/outdoor/lidarslam_outdoor_tuned.yaml
#   launch : /data/trov_ws/src/trov/launch/outdoor/lidarslam_mapping_outdoor.launch.py
#
# What it does:
#   1. If trov_outdoor.service is NOT running, starts it and waits until the
#      stack is up (lidar driver, IMU, MAVROS + drive bridge, FAST-LIO,
#      rosbridge/UI). That is everything you need to drive and map.
#   2. Stops outdoor localization + Nav2. They are not needed while mapping,
#      and localization alone uses ~130% CPU. FAST-LIO keeps running.
#   3. Starts the tuned lidarslam mapping (scanmatcher + graph_based_slam).
#
# Usage:
#   ~/start_outdoor_mapping_tuned.sh            # with RViz
#   ~/start_outdoor_mapping_tuned.sh --no-rviz  # headless / over SSH
#
# Then drive the route. To finish (the same scripts the UI uses):
#   /data/trov_ws/stop_mapping_outdoor.sh           # saves /data/pcd_map_temp.pcd and stops mapping
#   /data/trov_ws/save_map_outdoor.sh <map_name>    # -> /data/trov_ws/pcd_maps/<map_name>.pcd (+ 2D grid)
#
# Tips: the map origin is wherever the robot is when mapping starts. Finish
# the drive facing the same way you started.
#
# Why this file lives in /home/nvidia and not /data/trov_ws: when the service
# starts, it kills every process whose command line contains "/data/trov_ws".
# That would include this script.
#
# Unchanged: /data/trov_ws/start_mapping_outdoor.sh (stock lidarslam settings,
# used by the UI "start mapping" button).
# ============================================================

RVIZ=true
[ "$1" = "--no-rviz" ] && RVIZ=false

SERVICE=trov_outdoor
LOG=/home/nvidia/.ros/outdoor_mapping_tuned.log

source /opt/ros/humble/setup.bash               2>/dev/null
source /data/trov_ws/install/setup.bash         2>/dev/null

# ---- 1. Make sure the outdoor stack is running ----------------------------
if systemctl is-active --quiet "$SERVICE"; then
    echo "INFO: $SERVICE is already running."
else
    if systemctl is-active --quiet trov; then
        echo "ERROR: the INDOOR service (trov) is running. Stop it first:  sudo systemctl stop trov"
        exit 1
    fi
    echo "INFO: $SERVICE is not running — starting it..."
    start_ts=$(date '+%Y-%m-%d %H:%M:%S')
    sudo systemctl start "$SERVICE" || { echo "ERROR: could not start $SERVICE"; exit 1; }

    echo "INFO: Waiting for the stack to come up (up to 180 s)..."
    deadline=$((SECONDS + 180))
    until journalctl -u "$SERVICE" --since "$start_ts" --no-pager -o cat 2>/dev/null \
            | grep -q 'Stack is running'; do
        if [ $SECONDS -ge $deadline ]; then
            echo "ERROR: stack not up after 180 s. Check: journalctl -u $SERVICE -e"
            exit 1
        fi
        if ! systemctl is-active --quiet "$SERVICE"; then
            echo "ERROR: $SERVICE stopped during startup. Check: journalctl -u $SERVICE -e"
            exit 1
        fi
        sleep 3
    done
    echo "OK: $SERVICE is up."
fi

# ---- 2. Sanity check: lidar data present ----------------------------------
if ! timeout 10 ros2 topic echo /points --once --field header.frame_id >/dev/null 2>&1; then
    echo "ERROR: no data on /points (lidar covered or not connected?). Not starting mapping."
    exit 1
fi
echo "OK: /points is publishing."

# ---- 3. Stop anything already mapping, then localization + navigation -----
echo "INFO: Stopping any previous mapping run, localization and navigation..."
for p in scanmatcher_node graph_based_slam_node lidarslam_outdoor lidarslam_mapping_outdoor \
         outdoor_lidar_localization nav2_lidar_localization lidar_localization_node \
         controller_server planner_server behavior_server bt_navigator smoother_server \
         velocity_smoother collision_monitor waypoint_follower lifecycle_manager_navigation; do
    pkill -9 -f "$p" 2>/dev/null || true
done
sleep 2

# ---- 4. Start the tuned mapping -------------------------------------------
# lidarslam writes map.pcd to its working directory when /map_save is called;
# stop_mapping_outdoor.sh expects /data/trov_ws/map.pcd.
cd /data/trov_ws || exit 1
nohup ros2 launch trov lidarslam_mapping_outdoor.launch.py rviz:=$RVIZ > "$LOG" 2>&1 &
pid=$!
sleep 5
if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: mapping launch exited. See $LOG"
    tail -20 "$LOG"
    exit 1
fi

echo ""
echo "OK: Tuned outdoor mapping is running (PID $pid, log: $LOG)."
echo "    Drive the route now. When finished:"
echo "      /data/trov_ws/stop_mapping_outdoor.sh"
echo "      /data/trov_ws/save_map_outdoor.sh <map_name>"
exit 0
