#!/bin/bash
# ============================================================
# TROV Outdoor Workspace Launch Script
# Launches (in order):
#   1. RSLidar SDK
#   2. WIT IMU
#   3. RSLidar to LIO
#   4. MAVROS
#   5. Drive bridge
#   6. Sensor Health Status  ─┐
#   7. Beeper                  │  launched in parallel,
#   8. Battery Monitor         │  poll-based readiness check
#   9. Floodlight              │  exits as soon as all 4 are alive
#  10. Headlight Controller   ─┘
#  11. RealSense Camera       
#  12. Segformer Node          [DISABLED — see note below]
#  13. ICP Odometry Outdoor
#  14. Lidar Localization (Pinned)
#  15. Outdoor Waypoint Recorder
#  16. Outdoor Waypoint Follower
#  17. Delivery Runner Outdoor
#  18. Navigation Outdoor
#  19. MediaMTX
#  20. Rosbridge_server
#
# NOTE: Segformer node are commented out
# below to free up CPU on the Tegra board. Segformer normally
# feeds /semantic_obstacle_points into the Nav2 STVL costmap
# layer — with it disabled, that obstacle source will not be
# populated. Re-enable both blocks (search "DISABLED") once
# CPU headroom allows, or if semantic obstacle avoidance is
# needed.
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
WORKSPACE_DIR="/data/trov_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE_DIR/install/setup.bash"
 
LIDAR_IP="192.168.2.202"
LIDAR_PING_TIMEOUT=5
LIDAR_PING_COUNT=3

IMU_PORT="/dev/ttyUSB0"
FCU_PORT="/dev/ttyACM0"
FCU_BAUD="57600"

MEDIAMTX_BIN="$HOME/mediamtx"

# Launch files
ICP_LAUNCH_FILE="icp_odometry_outdoor.launch.py"

# PIDs for cleanup
declare -A NODE_PIDS   # associative: name → pid
declare -A NODE_PGIDS  # associative: name → process group id

# ---------- Helper: recursively collect all descendant PIDs of a process ----------
# IMPORTANT: `ros2 launch` spawns each node it manages in its OWN session
# (it calls the equivalent of setsid() internally so it can drive its own
# SIGINT -> SIGTERM -> SIGKILL escalation). That means those node processes
# are NOT members of the process group captured for the "ros2 launch"
# wrapper process — signaling -PGID only reaches the wrapper itself.
# setsid does NOT change the kernel's parent/child (PPID) relationship
# though, so walking the PID tree with pgrep -P reliably finds every
# descendant regardless of which session/process-group it ended up in.
get_descendants() {
    local parent="$1"
    local children
    children=$(pgrep -P "$parent" 2>/dev/null)
    local c
    for c in $children; do
        echo "$c"
        get_descendants "$c"
    done
}

# ---------- Cleanup ----------
cleanup() {
    echo ""
    log_warn "Caught exit signal — shutting down all launched processes..."

    # ── Unset trap first to prevent re-entrant calls ──
    trap - EXIT INT TERM HUP

    # ── Step 1: Snapshot full process trees BEFORE signaling anything.
    #    Must happen first — once a parent dies its children get reparented
    #    to init and pgrep -P parent_pid can no longer find them. ──
    local all_pids=""
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        all_pids="$all_pids $pid $(get_descendants "$pid")"
    done
    all_pids=$(echo "$all_pids" | tr ' ' '\n' | sort -un | grep -v '^$')

    if [ -n "$all_pids" ]; then
        log_info "Captured $(echo "$all_pids" | wc -l) process(es) across all launched trees."
    fi

    # ── Step 2: SIGINT every PID in every tree directly. Don't rely on
    #    PGID alone — ros2 launch isolates its children into their own
    #    sessions, so this is the part that actually reaches them. ──
    for pid in $all_pids; do
        kill -0 "$pid" 2>/dev/null && kill -INT "$pid" 2>/dev/null || true
    done
    # Belt-and-suspenders: also try the original PGID route.
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        kill -INT -"$pgid" 2>/dev/null || true
    done

    # ── Step 3: Wait for graceful shutdown, polling instead of a blind
    #    sleep so we don't wait longer than needed but still give Nav2 /
    #    lifecycle nodes (which can take longer than 5s) room to exit. ──
    log_info "Waiting up to 8s for graceful shutdown..."
    local waited=0
    while [ $waited -lt 8 ]; do
        local any_alive=false
        for pid in $all_pids; do
            kill -0 "$pid" 2>/dev/null && any_alive=true
        done
        [ "$any_alive" = false ] && break
        sleep 1
        waited=$((waited + 1))
    done

    # ── Step 4: Re-scan descendants (catches anything spawned during the
    #    grace period) and SIGKILL whatever is still alive. ──
    local final_pids=""
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        final_pids="$final_pids $pid $(get_descendants "$pid")"
    done
    final_pids=$(echo "$final_pids $all_pids" | tr ' ' '\n' | sort -un | grep -v '^$')

    for pid in $final_pids; do
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Force killing leftover PID $pid ($(ps -o comm= -p "$pid" 2>/dev/null))"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    # ── Step 5: PGID kill as before, for anything still alive ──
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        if kill -0 -"$pgid" 2>/dev/null; then
            log_warn "Force killing $name (PGID -$pgid)"
            kill -9 -"$pgid" 2>/dev/null || true
        fi
    done

    # ── Step 6: Also kill by registered PID as fallback ──
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Force killing $name by PID ($pid)"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    # ── Step 7: Named fallback — widened to include processes that
    #    ros2 launch commonly orphans (rviz2, nav2 lifecycle/component
    #    processes, TF/state publishers spawned inside launch files). ──
    pkill -9 -f "mavros_node" 2>/dev/null || true
    pkill -9 -f "rslidar_sdk" 2>/dev/null || true
    pkill -9 -f "rslidar_to_lio" 2>/dev/null || true
    pkill -9 -f "wit_ros2_imu" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "realsense2_camera" 2>/dev/null || true
    pkill -9 -f "segformer_node" 2>/dev/null || true
    pkill -9 -f "icp_odometry" 2>/dev/null || true
    pkill -9 -f "drive_outdoor" 2>/dev/null || true
    pkill -9 -f "outdoor_lidar_localization" 2>/dev/null || true
    pkill -9 -f "outdoor_waypoints_recorder" 2>/dev/null || true
    pkill -9 -f "outdoor_waypoints_follower" 2>/dev/null || true
    pkill -9 -f "delivery_runner_outdoor" 2>/dev/null || true
    pkill -9 -f "pcd_to_grid_node" 2>/dev/null || true
    pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "smoother_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "velocity_smoother" 2>/dev/null || true
    pkill -9 -f "collision_monitor" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "mediamtx" 2>/dev/null || true

    log_ok "Cleanup done. Goodbye."
    exit 0
}

# ── Trap all exit signals ──
trap cleanup EXIT INT TERM HUP

# ---------- Pre-flight: kill leftovers from any previous run ----------
# This is a different safety net than cleanup() above. cleanup() handles a
# *graceful* shutdown of *this* run's processes via trap (SIGINT/TERM/HUP).
# It CANNOT help if the previous run was ended with SIGKILL (e.g. a terminal
# emulator force-closing its window) or if the offending process was never
# a child of this script to begin with (e.g. a separate service) — SIGKILL
# is uncatchable by design, and a process-tree walk only finds descendants
# of what THIS script launched. So instead of trying to trace ancestry, we
# just hunt by name/identifier and by the actual hardware resource (serial
# ports), system-wide, before starting anything new.
preflight_cleanup() {
    echo ""
    log_info "Pre-flight: checking for leftover processes from a previous run..."

    local found=false

    # Free the serial ports directly. Two processes fighting over the same
    # /dev/ttyUSB0 or /dev/ttyACM0 is exactly what produces IMU checksum
    # errors / MAVROS connection flakiness on a fresh launch.
    if command -v fuser >/dev/null 2>&1; then
        local port
        for port in "$IMU_PORT" "$FCU_PORT"; do
            if [ -e "$port" ]; then
                local holders
                holders=$(fuser "$port" 2>/dev/null)
                if [ -n "$holders" ]; then
                    log_warn "Port $port is held by PID(s):$holders — killing."
                    fuser -k "$port" 2>/dev/null || true
                    found=true
                fi
            fi
        done
    else
        log_warn "fuser not found (sudo apt install psmisc) — cannot directly check/free serial ports."
    fi

    # Kill by known node/process names — same widened list used in cleanup().
    # Also includes identifiers pulled straight from observed error logs
    # (camera IP / RTSP path) so leftovers get caught even if you don't know
    # which script/node they belong to.
    local patterns=(
        mavros_node rslidar_sdk rslidar_to_lio wit_ros2_imu rviz2
        realsense2_camera segformer_node icp_odometry drive_outdoor
        outdoor_lidar_localization outdoor_waypoints_recorder
        outdoor_waypoints_follower delivery_runner_outdoor pcd_to_grid_node rosbridge_websocket
        controller_server planner_server behavior_server smoother_server
        bt_navigator velocity_smoother collision_monitor lifecycle_manager
        component_container robot_state_publisher static_transform_publisher
        mediamtx
        "192.168.1.161" "realmonitor"
    )
    local p
    for p in "${patterns[@]}"; do
        if pkill -9 -f "$p" 2>/dev/null; then
            log_warn "Killed leftover process(es) matching: $p"
            found=true
        fi
    done

    # Broad net: anything still running out of this workspace at all,
    # in case it's a node we don't have a specific pattern for.
    if pkill -9 -f "$WORKSPACE_DIR" 2>/dev/null; then
        log_warn "Killed leftover process(es) running from $WORKSPACE_DIR"
        found=true
    fi

    if [ "$found" = true ]; then
        log_info "Waiting 2s for ports/sockets to be released..."
        sleep 2
    else
        log_ok "No leftover processes found."
    fi
}

preflight_cleanup

# ---------- Helper: launch in its own process group and register ----------
launch_node() {
    local name="$1"
    shift
    log_info "Launching: $name"

    setsid "$@" &
    local pid=$!
    local pgid
    for i in $(seq 1 10); do
        sleep 0.2
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -n "$pgid" ] && [ "$pgid" != "0" ] && break
    done
    pgid="${pgid:-$pid}"

    NODE_PIDS["$name"]=$pid
    NODE_PGIDS["$name"]=$pgid
    log_ok "$name started (PID $pid | PGID $pgid)"
}

# ---------- Helper: poll until a node process is alive ----------
wait_for_node() {
    local name="$1"
    local timeout="${2:-30}"
    local mode="${3:-critical}"
    local pid="${NODE_PIDS[$name]}"
    local elapsed=0
    local alive_count=0

    log_info "Polling $name process (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            if [ "$mode" = "warn" ]; then
                log_warn "$name (PID $pid) died during startup — continuing anyway (non-critical)."
                return 1
            else
                log_error "$name (PID $pid) died during startup — aborting."
                exit 1
            fi
        fi
        alive_count=$((alive_count + 1))
        if [ $alive_count -ge 2 ]; then
            log_ok "$name is alive (confirmed after ${elapsed}s)."
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_ok "$name is alive."
}

# ---------- Helper: wait for a topic to have publishers ----------
wait_for_topic() {
    local topic="$1"
    local timeout="$2"
    local elapsed=0

    log_info "Waiting for topic $topic (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
            log_ok "Topic $topic has publishers (after ${elapsed}s)"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_warn "Topic $topic has no publishers after ${timeout}s"
    return 1
}

# ---------- Helper: wait for a TF to be available ----------
wait_for_tf() {
    local parent="$1"
    local child="$2"
    local timeout="$3"
    local elapsed=0

    log_info "Waiting for TF: $parent → $child (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -q "Translation"; then
            log_ok "TF $parent → $child is available (after ${elapsed}s)"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_warn "TF $parent → $child not available after ${timeout}s"
    return 1
}

# ---------- Helper: wait for IMU data ----------
wait_for_imu() {
    local topic="$1"
    local timeout="$2"
    local elapsed=0

    log_info "Waiting for IMU data on $topic (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if timeout 2 ros2 topic echo "$topic" --once 2>/dev/null | grep -q "orientation"; then
            log_ok "IMU data available on $topic (after ${elapsed}s)"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_warn "IMU not publishing on $topic after ${timeout}s"
    return 1
}

# ---------- Helper: wait for a ROS2 node to appear in node list ----------
wait_for_ros_node() {
    local node_name="$1"
    local timeout="$2"
    local elapsed=0

    log_info "Waiting for ROS2 node $node_name to register (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            log_ok "ROS2 node $node_name is registered (after ${elapsed}s)"
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_warn "ROS2 node $node_name did not appear after ${timeout}s"
    return 1
}

# ============================================================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}   TROV Outdoor Stack Launch Script        ${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

# ---------- 1. Source ROS2 ----------
log_info "Sourcing ROS2 Humble: $ROS2_SETUP"
[ -f "$ROS2_SETUP" ] || { log_error "ROS2 setup not found: $ROS2_SETUP"; exit 1; }
source "$ROS2_SETUP"
log_ok "ROS2 Humble sourced."

# ---------- 2. Source workspace ----------
log_info "Sourcing workspace: $WS_SETUP"
[ -f "$WS_SETUP" ] || { log_error "Workspace not found. Run colcon build first."; exit 1; }
source "$WS_SETUP"
log_ok "Workspace sourced."

log_info "ROS_DISTRO    = $ROS_DISTRO"
log_info "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-0 (default)}"

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
    ros2 launch rslidar_sdk humble_start.py

# ---------- 7. WIT IMU ----------
launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# ── Poll both drivers alive before continuing ──────────────────────────────────
wait_for_node "rslidar_sdk" 15 "critical"
wait_for_node "wit_imu"     15 "critical"

# ---------- 8. RSLidar to LIO ----------
# Converts raw rslidar point cloud to LIO-compatible format.
# Depends on rslidar_sdk /points being available first.
echo ""
echo -e "${CYAN}--- Starting RSLidar to LIO ---${NC}"
echo ""

wait_for_topic "/points" 30 || log_warn "/points not ready — rslidar_to_lio may fail to convert"

launch_node "rslidar_to_lio" \
    ros2 run trov rslidar_to_lio

wait_for_node "rslidar_to_lio" 10 "critical"

# ---------- 9. MAVROS ----------
echo ""
echo -e "${CYAN}--- Starting MAVROS ---${NC}"
echo ""
cd "$HOME"
launch_node "mavros" \
    ros2 run mavros mavros_node \
        --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"

wait_for_topic "/mavros/state" 30 || log_warn "MAVROS /mavros/state not seen — FCU connection may have issues"
cd "$WORKSPACE_DIR"

# ---------- 10. Drive Bridge ----------
echo ""
echo -e "${CYAN}--- Starting drive bridge ---${NC}"
echo ""
launch_node "drive_bridge" \
    ros2 run cpp_pubsub drive_outdoor
wait_for_node "drive_bridge" 10 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting peripheral nodes (parallel) ---${NC}"
echo ""

# ---------- 13. Sensor Health Status ----------
launch_node "sensors_health_status" \
    ros2 run trov sensors_health_status

# ---------- 14. Battery Monitor ----------
launch_node "battery_monitor" \
    ros2 run trov battery_monitor

# ---------- 15. Floodlight ----------
launch_node "floodlight" \
    ros2 run trov floodlight.py

# ---------- 16. Headlight Controller ----------
launch_node "headlight_controller" \
    ros2 run trov headlight_controller

# ── Poll all 4 peripheral nodes — exits as soon as all are alive ──────────────
# No fixed sleep. Each is checked every 1s up to 15s total.
log_info "Polling peripheral nodes until all are alive (timeout: 15s)..."
peripheral_timeout=15
peripheral_elapsed=0
while [ $peripheral_elapsed -lt $peripheral_timeout ]; do
    all_alive=true
    for pname in sensors_health_status battery_monitor floodlight headlight_controller; do
        pid="${NODE_PIDS[$pname]}"
        if ! kill -0 "$pid" 2>/dev/null; then
            all_alive=false
            break
        fi
    done
    if [ "$all_alive" = true ]; then
        log_ok "All peripheral nodes are alive (after ${peripheral_elapsed}s)"
        break
    fi
    sleep 1
    peripheral_elapsed=$((peripheral_elapsed + 1))
done

# Final status report — continue even if some died
for pname in sensors_health_status battery_monitor floodlight headlight_controller; do
    pid="${NODE_PIDS[$pname]}"
    if kill -0 "$pid" 2>/dev/null; then
        log_ok "  $pname → alive (PID $pid)"
    else
        log_error "  $pname → DEAD (PID $pid) — continuing anyway"
    fi
done
# ─────────────────────────────────────────────────────────────────────────────


# ---------- 11. RealSense Camera — DISABLED to free CPU ----------
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

# wait_for_topic "/camera/camera/color/image_raw" 30 || log_warn "RealSense color stream not ready — segformer may not receive RGB input"

# ---------- 12. Segformer Node — DISABLED to free CPU ----------
# Runs in its own virtual environment — cannot share the main ROS2 env.
# Publishes:
#   /fusion_segmentation/traversability  → visualization
#   /fusion_segmentation/semantic        → visualization
#   /semantic_obstacle_points            → Nav2 STVL costmap input
# echo ""
# echo -e "${CYAN}--- Starting Segformer Node ---${NC}"
# echo ""
#
# launch_node "segformer" bash -c "
#     export LD_PRELOAD='/usr/local/lib/python3.10/dist-packages/torch/lib/libc10.so /usr/local/lib/python3.10/dist-packages/torch/lib/libtorch_cpu.so /usr/local/lib/python3.10/dist-packages/torch/lib/libtorch_cuda.so'
#     cd /data/trov_ws
#     ros2 run segformer_cpp segformer_node
# "
#
# wait_for_node "segformer" 30 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting ICP Odometry Outdoor ---${NC}"
echo ""

# ── Gate: all 3 sensor prerequisites must be confirmed before ICP launches ────
log_info "Confirming sensor prerequisites for ICP..."

lidar_ok=true
tf_ok=true
imu_ok=true

wait_for_topic "/points"                45 || { log_warn "LiDAR /points not ready";           lidar_ok=false; }
wait_for_tf    "base_link" "lidar_link" 30 || { log_warn "TF base_link→lidar_link not ready"; tf_ok=false;    }
wait_for_imu   "/imu/data"             30 || { log_warn "IMU /imu/data not ready";            imu_ok=false;   }

if [ "$lidar_ok" = false ] || [ "$tf_ok" = false ] || [ "$imu_ok" = false ]; then
    log_warn "One or more ICP prerequisites are not fully ready — ICP may be unreliable."
    log_warn "Proceeding anyway. Check sensor connections if odometry fails."
fi

# ── ICP launch with retry — polls /odom to confirm it is actually working ─────
max_attempts=3
icp_success=false

for attempt in $(seq 1 $max_attempts); do
    log_info "ICP Odometry launch attempt $attempt/$max_attempts"

    launch_node "icp_odometry" \
        ros2 launch trov "$ICP_LAUNCH_FILE"

    log_info "Polling for /odom from ICP (timeout: 60s)..."
    icp_elapsed=0
    icp_ready=false
    while [ $icp_elapsed -lt 60 ]; do
        if ! kill -0 "${NODE_PIDS[icp_odometry]}" 2>/dev/null; then
            log_warn "ICP process died on attempt $attempt"
            break
        fi
        if timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -q "pose"; then
            log_ok "ICP Odometry publishing /odom (attempt $attempt, after ${icp_elapsed}s)"
            icp_ready=true
            break
        fi
        sleep 1
        icp_elapsed=$((icp_elapsed + 1))
    done

    if [ "$icp_ready" = true ]; then
        icp_success=true
        break
    fi

    log_warn "ICP Odometry failed on attempt $attempt"
    if [ $attempt -lt $max_attempts ]; then
        pkill -9 -f "icp_odometry" 2>/dev/null || true
        sleep 2
    fi
done

if [ "$icp_success" = false ]; then
    log_error "ICP Odometry failed after $max_attempts attempts"
    log_error "Continuing anyway — odometry will not work. Localization may drift."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Lidar Localization ---${NC}"
echo ""

wait_for_topic "/odom" 30 || log_warn "/odom not available — localization may have poor initial pose"

launch_node "lidar_localization" \
    ros2 launch trov outdoor_lidar_localization_pinned.launch.py

if ! wait_for_ros_node "lidar_localization" 60; then
    log_warn "lidar_localization node never appeared — localization may be unstable"
    log_warn "Continuing anyway."
else
    log_ok "Lidar localization is registered and running."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting PCD -> Occupancy Grid (/outdoor_map) ---${NC}"
echo ""

# Publishes the active PCD map as a 2D occupancy grid on /outdoor_map so the UI
# can render it. Localization still uses the raw .pcd; this is display-only.
launch_node "pcd_to_grid" \
    ros2 launch pcd_to_grid pcd_to_grid.launch.py
wait_for_node "pcd_to_grid" 10 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Outdoor Waypoint Recorder ---${NC}"
echo ""

launch_node "waypoint_recorder" \
    ros2 run trov outdoor_waypoints_recorder.py
wait_for_node "waypoint_recorder" 10 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Outdoor Waypoint Follower ---${NC}"
echo ""

launch_node "waypoint_follower" \
    ros2 run trov outdoor_waypoints_follower.py
wait_for_node "waypoint_follower" 10 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Delivery Runner Outdoor ---${NC}"
echo ""

launch_node "delivery_runner" \
    ros2 run trov delivery_runner_outdoor.py
wait_for_node "delivery_runner" 10 "warn"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Navigation Outdoor ---${NC}"
echo ""

launch_node "navigation_outdoor" \
    ros2 launch trov navigation_outdoor.launch.py

if ! wait_for_ros_node "lifecycle_manager_navigation" 60; then
    log_error "lifecycle_manager_navigation never appeared — navigation launch likely failed"
else
    log_ok "Navigation stack is registered and running."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting MediaMTX ---${NC}"
echo ""

if [ ! -f "$MEDIAMTX_BIN" ]; then
    log_warn "mediamtx binary not found at $MEDIAMTX_BIN — skipping."
else
    cd "$HOME"
    launch_node "mediamtx" \
        "$MEDIAMTX_BIN"
    wait_for_node "mediamtx" 10 "warn"
    cd "$WORKSPACE_DIR"
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting ROSBridge Server ---${NC}"
echo ""

# ---------- 17. ROSBridge WebSocket Server ----------
launch_node "rosbridge_server" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    call_services_in_new_thread:=true \
    send_action_goals_in_new_thread:=true \
    default_call_service_timeout:=5.0
wait_for_node "rosbridge_server" 10


# ============================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   All nodes launched!                     ${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
log_info "Active nodes:"
for name in "${!NODE_PIDS[@]}"; do
    pid="${NODE_PIDS[$name]}"
    if kill -0 "$pid" 2>/dev/null; then
        log_ok "  $name → PID $pid | PGID ${NODE_PGIDS[$name]}"
    else
        log_error "  $name → PID $pid (DEAD)"
    fi
done
echo ""
log_info "MediaMTX RTSP   available at rtsp://<robot_ip>:8554"
log_info "MediaMTX WebRTC available at http://<robot_ip>:8889"
log_info "Press Ctrl+C to stop all nodes."
echo ""

# ---------- Keep-alive loop ----------
log_info "Stack is running. Monitoring nodes every 30s..."
while true; do
    sleep 30
    dead_nodes=""
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if ! kill -0 "$pid" 2>/dev/null; then
            dead_nodes="$dead_nodes $name"
        fi
    done
    if [ -n "$dead_nodes" ]; then
        log_error "Dead nodes detected:$dead_nodes"
        log_error "Use: sudo systemctl restart trov_outdoor"
    fi
done
