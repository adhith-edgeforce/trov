#!/bin/bash
# ============================================================
# TROV Workspace Launch Script
# Launches (in order):
#   1. RSLidar SDK
#   2. WIT IMU
#   3. rslidar_to_lio
#   4. hrtk_odom
#   5. MAVROS
#   6. Drive bridge
#   7. gps_frame_changer
# ============================================================

# NOTE: 'set -e' removed intentionally.
# Previously, if drive_bridge exited with a non-zero code,
# 'set -e' would abort the entire script before hrtk_odom
# ever got launched. Individual node failures are now handled
# explicitly inside wait_for_node().

# ---------- Colors ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()  { echo -e "${CYAN}[INFO]  $(date '+%H:%M:%S') | $1${NC}"; }
log_ok()    { echo -e "${GREEN}[OK]    $(date '+%H:%M:%S') | $1${NC}"; }
log_warn()  { echo -e "${YELLOW}[WARN]  $(date '+%H:%M:%S') | $1${NC}"; }
log_error() { echo -e "${RED}[ERROR] $(date '+%H:%M:%S') | $1${NC}"; }

# ---------- Config ----------
#WORKSPACE_DIR="$HOME/trov_ws"
WORKSPACE_DIR="/data/trov_ws/"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE_DIR/install/setup.bash"

LIDAR_IP="192.168.2.202"
LIDAR_PING_TIMEOUT=5
LIDAR_PING_COUNT=3

IMU_PORT="/dev/ttyUSB0"
FCU_PORT="/dev/ttyACM0"
FCU_BAUD="57600"

# PIDs for cleanup
declare -A NODE_PIDS   # associative: name → pid

# ---------- Cleanup ----------
cleanup() {
    echo ""
    log_warn "Caught exit signal — shutting down all launched processes..."
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            log_info "Killing $name (PID $pid)"
            kill "$pid" 2>/dev/null || true
        fi
    done
    # Give nodes 2s to die, then force kill
    sleep 2
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Force killing $name (PID $pid)"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    log_info "Cleanup done. Goodbye."
}
trap cleanup EXIT INT TERM

# ---------- Helper: launch and register ----------
launch_node() {
    local name="$1"
    shift
    log_info "Launching: $name"
    "$@" &
    local pid=$!
    NODE_PIDS["$name"]=$pid
    log_ok "$name started (PID $pid)"
}

# ---------- Helper: wait and check a node is still alive ----------
# Second argument: wait_secs
# Third argument (optional): "critical" (default) or "warn"
#   - critical: abort entire script if node dies
#   - warn:     log a warning but continue launching remaining nodes
wait_for_node() {
    local name="$1"
    local wait_secs="$2"
    local mode="${3:-critical}"   # default = critical
    local pid="${NODE_PIDS[$name]}"

    log_info "Waiting ${wait_secs}s for $name to initialize..."
    sleep "$wait_secs"

    if ! kill -0 "$pid" 2>/dev/null; then
        if [ "$mode" = "warn" ]; then
            log_warn "$name (PID $pid) died during startup — continuing anyway (non-critical node)."
        else
            log_error "$name (PID $pid) died during startup — aborting."
            exit 1
        fi
    else
        log_ok "$name is alive."
    fi
}

# ============================================================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}   TROV Full Stack Launch Script           ${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

# ---------- 1. Source ROS2 ----------
log_info "Sourcing ROS2 Humble: $ROS2_SETUP"
if [ ! -f "$ROS2_SETUP" ]; then
    log_error "ROS2 setup not found: $ROS2_SETUP"
    exit 1
fi
source "$ROS2_SETUP"
log_ok "ROS2 Humble sourced."

# ---------- 2. Source workspace ----------
log_info "Sourcing workspace: $WS_SETUP"
if [ ! -f "$WS_SETUP" ]; then
    log_error "Workspace not found. Run colcon build first."
    exit 1
fi
source "$WS_SETUP"
log_ok "Workspace sourced."

log_info "ROS_DISTRO     = $ROS_DISTRO"
log_info "ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID:-0 (default)}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
 
log_info "ROS_DISTRO         = $ROS_DISTRO"
log_info "ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID:-0 (default)}"
log_info "RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
log_info "CYCLONEDDS_URI     = $CYCLONEDDS_URI"
 

# ---------- 3. Check LiDAR ----------
echo ""
log_info "Pinging LiDAR at $LIDAR_IP ..."
if ping -c "$LIDAR_PING_COUNT" -W "$LIDAR_PING_TIMEOUT" "$LIDAR_IP" > /dev/null 2>&1; then
    log_ok "LiDAR reachable."
else
    log_error "Cannot reach LiDAR at $LIDAR_IP — check cable / IP / power."
    exit 1
fi

# ---------- 4. Check IMU ----------
echo ""
log_info "Checking IMU port: $IMU_PORT"
if [ -e "$IMU_PORT" ]; then
    log_ok "IMU port found: $IMU_PORT"
else
    log_error "IMU port $IMU_PORT not found. Available ports:"
    ls /dev/ttyUSB* 2>/dev/null || echo "  (none)"
    exit 1
fi

# ---------- 5. Check FCU (Pixhawk) ----------
echo ""
log_info "Checking FCU port: $FCU_PORT"
if [ -e "$FCU_PORT" ]; then
    log_ok "FCU port found: $FCU_PORT"
else
    log_warn "FCU port $FCU_PORT not found — MAVROS may fail."
    log_warn "Available ACM ports:"
    ls /dev/ttyACM* 2>/dev/null || echo "  (none)"
    # Not a hard exit — operator may want to continue without drive
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting hardware drivers ---${NC}"
echo ""
cd "$WORKSPACE_DIR"

# ---------- 6. RSLidar SDK ----------
launch_node "rslidar_sdk" \
    ros2 launch rslidar_sdk humble_start.py
wait_for_node "rslidar_sdk" 5 "critical"

# ---------- 7. rslidar_to_lio ----------
#launch_node "rslidar_to_lio for autoware" \
#    ros2 run trov rslidar_to_lio #--ros-args -r points_raw:=/sensing/lidar/top/pointcloud_raw
#wait_for_node "rslidar_to_lio for autoware" 5 "critical"

# ---------- 8. WIT IMU ----------
launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py
wait_for_node "wit_imu" 3 "critical"

# ---------- 9. MAVROS (from home dir — no workspace dep) ----------
echo ""
echo -e "${CYAN}--- Starting MAVROS ---${NC}"
echo ""
cd "$HOME"
launch_node "mavros" \
    ros2 run mavros mavros_node \
        --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"
wait_for_node "mavros" 15 "critical"
cd "$WORKSPACE_DIR"

echo ""
echo -e "${CYAN}--- Starting drive bridge ---${NC}"
echo ""

# ---------- 10. Drive Bridge ----------
# Marked as "warn" so that if it fails, the script continues
# and hrtk_odom still gets launched.
# Depends on: MAVROS (/mavros/manual_control/send), Nav2 (/cmd_vel)
launch_node "drive_bridge" \
    ros2 run cpp_pubsub drive
wait_for_node "drive_bridge" 3 "warn"   # <-- FIX: was "critical" (implicit), now non-fatal

# ---------- 11. hrtk_odom ----------
echo ""
echo -e "${CYAN}--- Starting hrtk_odom ---${NC}"
echo ""
# FIX: hrtk_odom is now explicitly reached regardless of drive_bridge status.
# Previously, if drive_bridge failed and set -e was active, the script
# would abort here. Now it always launches.
log_info "About to launch hrtk_odom..."
launch_node "hrtk_odom" \
    ros2 run trov hrtk_odom
wait_for_node "hrtk_odom" 5 "critical"

# ---------- 12. gps_frame_changer ----------
#echo ""
#echo -e "${CYAN}--- Starting gps_frame_changer ---${NC}"
#echo ""
#log_info "About to launch gps_frame_changer..."
#launch_node "gps_frame_changer" \
#    ros2 run trov gps_frame_changer.py
#wait_for_node "gps_frame_changer" 5 "critical"

# ============================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   All nodes launched successfully!        ${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
log_info "Active nodes:"
for name in "${!NODE_PIDS[@]}"; do
    pid="${NODE_PIDS[$name]}"
    if kill -0 "$pid" 2>/dev/null; then
        log_ok "  $name → PID $pid"
    else
        log_error "  $name → PID $pid (DEAD)"
    fi
done
echo ""
log_info "Press Ctrl+C to stop all nodes."
echo ""

# ---------- Monitor: restart dead nodes (optional watchdog) ----------
# Uncomment below to auto-restart critical nodes if they die
# while true; do
#     sleep 10
#     for name in "${!NODE_PIDS[@]}"; do
#         pid="${NODE_PIDS[$name]}"
#         if ! kill -0 "$pid" 2>/dev/null; then
#             log_warn "Node '$name' (PID $pid) died — manual restart needed"
#         fi
#     done
# done

# ---------- Wait ----------
wait
