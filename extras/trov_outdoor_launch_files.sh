#!/bin/bash
# ============================================================
# TROV Workspace Launch Script
# Launches (in order):
#   1. RSLidar SDK
#   2. rslidar_to_lio
#   3. WIT IMU
#   4. MAVROS
#   5. Drive bridge
#   6. RealSense Camera
#   7. Segformer Node
#   8. ICP Odometry Outdoor
#   9. Lidar Localization (Pinned)
#  10. Outdoor Waypoint Recorder
#  11. Outdoor Waypoint Follower
#  12. Delivery Runner Outdoor
#  13. Navigation Outdoor
#  14. MediaMTX
# ============================================================

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
WORKSPACE_DIR="/data/trov_ws/"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE_DIR/install/setup.bash"

LIDAR_IP="192.168.2.202"
LIDAR_PING_TIMEOUT=5
LIDAR_PING_COUNT=3

IMU_PORT="/dev/ttyUSB0"
FCU_PORT="/dev/ttyACM0"
FCU_BAUD="57600"

MEDIAMTX_BIN="$HOME/mediamtx"

# PIDs for cleanup
declare -A NODE_PIDS

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

# ---------- Helper: wait and check ----------
wait_for_node() {
    local name="$1"
    local wait_secs="$2"
    local mode="${3:-critical}"
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
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting hardware drivers ---${NC}"
echo ""
cd "$WORKSPACE_DIR"

# ---------- 6. RSLidar SDK ----------
launch_node "rslidar_sdk" \
    env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>' \
    ros2 launch rslidar_sdk humble_start.py
wait_for_node "rslidar_sdk" 5 "critical"

# ---------- 7. rslidar_to_lio ----------
launch_node "rslidar_to_lio" \
    ros2 run trov rslidar_to_lio
wait_for_node "rslidar_to_lio" 5 "critical"

# ---------- 8. WIT IMU ----------
launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py
wait_for_node "wit_imu" 3 "critical"

# ---------- 9. MAVROS ----------
echo ""
echo -e "${CYAN}--- Starting MAVROS ---${NC}"
echo ""
cd "$HOME"
launch_node "mavros" \
    ros2 run mavros mavros_node \
        --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"
wait_for_node "mavros" 15 "critical"
cd "$WORKSPACE_DIR"

# ---------- 10. Drive Bridge ----------
echo ""
echo -e "${CYAN}--- Starting drive bridge ---${NC}"
echo ""
launch_node "drive_bridge" \
    ros2 run cpp_pubsub drive_outdoor
wait_for_node "drive_bridge" 3 "warn"

# ---------- 11. RealSense Camera ----------
# Publishes:
#   /camera/camera/color/image_raw       → segformer RGB input
#   /camera/camera/depth/image_rect_raw  → segformer depth input
#   /camera/camera/color/camera_info     → segformer intrinsics
#   camera_link TF                       → Nav2 STVL layer
echo ""
echo -e "${CYAN}--- Starting RealSense Camera ---${NC}"
echo ""
launch_node "realsense_camera" \
    ros2 launch realsense2_camera rs_launch.py
wait_for_node "realsense_camera" 10 "warn"

# ---------- 12. Segformer Node ----------
# Runs in its own virtual environment — cannot share the main ROS2 env
# Publishes:
#   /fusion_segmentation/traversability  → visualization
#   /fusion_segmentation/semantic        → visualization
#   /semantic_obstacle_points            → Nav2 STVL costmap input
echo ""
echo -e "${CYAN}--- Starting Segformer Node ---${NC}"
echo ""
launch_node "segformer" bash -c "
    export LD_PRELOAD='/usr/local/lib/python3.10/dist-packages/torch/lib/libc10.so /usr/local/lib/python3.10/dist-packages/torch/lib/libtorch_cpu.so /usr/local/lib/python3.10/dist-packages/torch/lib/libtorch_cuda.so'
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eno1</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
    cd /data/trov_ws
    ros2 run segformer_cpp segformer_node
"
wait_for_node "segformer" 15 "warn"

# ---------- 13. ICP Odometry Outdoor ----------
echo ""
echo -e "${CYAN}--- Starting ICP Odometry Outdoor ---${NC}"
echo ""
launch_node "icp_odometry" \
    ros2 launch trov icp_odometry_outdoor.launch.py
wait_for_node "icp_odometry" 10 "critical"

# ---------- 14. Lidar Localization (Pinned) ----------
echo ""
echo -e "${CYAN}--- Starting Lidar Localization ---${NC}"
echo ""
launch_node "lidar_localization" \
    ros2 launch trov outdoor_lidar_localization_pinned.launch.py
wait_for_node "lidar_localization" 15 "critical"

# ---------- 15. Outdoor Waypoint Recorder ----------
# Non-critical: starts immediately but waits for lidar_localization to come up.
# active_map stays None until localization is running; recorder retries every
# map_retry_interval seconds automatically — it will NOT crash or exit.
echo ""
echo -e "${CYAN}--- Starting Outdoor Waypoint Recorder ---${NC}"
echo ""
launch_node "waypoint_recorder" \
    ros2 run trov outdoor_waypoints_recorder.py
wait_for_node "waypoint_recorder" 3 "warn"

# ---------- 16. Outdoor Waypoint Follower ----------
# Non-critical: starts immediately but blocks route execution until a map is
# detected from lidar_localization. Retries every routes_poll_interval seconds
# automatically — it will NOT crash or exit while map is unavailable.
echo ""
echo -e "${CYAN}--- Starting Outdoor Waypoint Follower ---${NC}"
echo ""
launch_node "waypoint_follower" \
    ros2 run trov outdoor_waypoints_follower.py
wait_for_node "waypoint_follower" 3 "warn"

# ---------- 17. Delivery Runner Outdoor ----------
echo ""
echo -e "${CYAN}--- Starting Delivery Runner Outdoor ---${NC}"
echo ""
launch_node "delivery_runner" \
    ros2 run trov delivery_runner_outdoor.py
wait_for_node "delivery_runner" 5 "warn"

# ---------- 18. Navigation Outdoor ----------
echo ""
echo -e "${CYAN}--- Starting Navigation Outdoor ---${NC}"
echo ""
launch_node "navigation_outdoor" \
    ros2 launch trov navigation_outdoor.launch.py
wait_for_node "navigation_outdoor" 15 "critical"

# ---------- 19. MediaMTX ----------
echo ""
echo -e "${CYAN}--- Starting MediaMTX ---${NC}"
echo ""
if [ ! -f "$MEDIAMTX_BIN" ]; then
    log_warn "mediamtx binary not found at $MEDIAMTX_BIN — skipping."
else
    cd "$HOME"
    launch_node "mediamtx" \
        "$MEDIAMTX_BIN"
    wait_for_node "mediamtx" 3 "warn"
    cd "$WORKSPACE_DIR"
fi

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
        log_warn "  $name → PID $pid (DEAD)"
    fi
done
echo ""
log_info "Press Ctrl+C to stop all nodes."
echo ""

# ---------- Wait ----------
wait