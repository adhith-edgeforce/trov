#!/bin/bash
# =====================================================================
# trov_bringup.sh — launches the TROV stack in order, 3 s apart.
#
#   - Ctrl+C (or kill of this script) terminates every launched node.
#   - If ANY single node dies/crashes, the ENTIRE stack is torn down.
#   - Every node's stdout/stderr is streamed live to this terminal
#     (prefixed with "[name]") AND written to its own log file.
#
# Usage:
#   chmod +x trov_bringup.sh
#   ./trov_bringup.sh
#
# Logs: each node writes to /data/trov_ws/logs/<name>.log
# =====================================================================

WS=/data/trov_ws
LOG_DIR="$WS/logs"
mkdir -p "$LOG_DIR"

# ---------------- Source ROS + workspace ----------------
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

set -u

# ---------------- State ----------------
PIDS=()
NAMES=()
TAIL_PIDS=()
SHUTTING_DOWN=0

# ---------------- Cleanup ----------------
cleanup() {
    # Guard against re-entry (signal during cleanup)
    if [ "$SHUTTING_DOWN" -eq 1 ]; then return; fi
    SHUTTING_DOWN=1

    echo ""
    echo "[trov_bringup] Shutting down all nodes..."
    # SIGINT first so ros2 launch/run shut their children down cleanly
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    sleep 3
    # Force-kill anything still alive
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[trov_bringup] PID $pid still alive, sending SIGKILL"
            kill -KILL "$pid" 2>/dev/null
        fi
    done

    # Stop the background log-streamers too
    for tpid in "${TAIL_PIDS[@]}"; do
        kill -KILL "$tpid" 2>/dev/null
    done

    wait 2>/dev/null
    echo "[trov_bringup] All nodes stopped."
    exit "${1:-0}"
}
trap 'cleanup 0' SIGINT SIGTERM

start() {
    local name="$1"; shift
    echo "[trov_bringup] Starting: $name"

    local logfile="$LOG_DIR/$name.log"
    : > "$logfile"   # truncate/create fresh log for this run

    # Run the node, writing its combined stdout/stderr straight to its log file.
    "$@" >> "$logfile" 2>&1 &
    local pid=$!
    PIDS+=("$pid")
    NAMES+=("$name")

    # Stream that log file to the terminal live, prefixed with the node name,
    # so you can see everything without opening each log file separately.
    tail -n 0 -F "$logfile" 2>/dev/null | sed -u "s/^/[$name] /" &
    TAIL_PIDS+=("$!")

    sleep 3

    # Fail fast: if the node already died during its 3 s settle time
    # (bad package name, serial port busy, etc.), abort the whole bringup.
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[trov_bringup] ERROR: '$name' died immediately. Check $logfile"
        cleanup 1
    fi
}

# ---------------- Launch sequence (3 s apart) ----------------

start mavros \
    ros2 run mavros mavros_node --ros-args -p fcu_url:=serial://ttyACM0:57600

start realsense \
    ros2 launch realsense2_camera rs_launch.py

start line_follower \
    ros2 run line_follower line_follower_node

start aruco_station_detector \
    ros2 run line_follower aruco_station_detector.py

start line_follower_bridge \
    ros2 run cpp_pubsub line_follower_bridge

start rosbridge \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

echo "[trov_bringup] All nodes launched. Logs in $LOG_DIR/"
echo "[trov_bringup] Streaming all node output below (prefixed by node name)."
echo "[trov_bringup] Press Ctrl+C to stop everything."
echo "[trov_bringup] Monitoring: if any node dies, the whole stack stops."

# ---------------- Watchdog: any node dies -> stop everything ----------------
# 'wait -n' returns as soon as ANY background job exits (including tail streamers,
# so we specifically re-check the tracked node PIDs before deciding to tear down).
while true; do
    wait -n 2>/dev/null
    # If we're already shutting down via Ctrl+C, the trap handles it.
    if [ "$SHUTTING_DOWN" -eq 1 ]; then break; fi

    # Check whether any *node* (not a tail streamer) actually died.
    died=0
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
            echo "[trov_bringup] Node '${NAMES[$i]}' has died. Check $LOG_DIR/${NAMES[$i]}.log"
            died=1
        fi
    done

    if [ "$died" -eq 1 ]; then
        cleanup 1
    fi
    # otherwise it was just a tail streamer or other bg job exiting; keep waiting
done
