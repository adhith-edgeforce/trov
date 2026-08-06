#!/bin/bash
# ============================================================
# TROV Workspace Launch Script for Indoor
# Launches (in order):
#   1. RSLidar SDK
#   2. WIT IMU
#   3. RealSense Camera
#   4. MAVROS
#   5. Drive bridge
#   6. AMR Failsafe              ← stops robot + publishes error code on subsystem failure
#   7. Sensor Health Status  ─┐
#   8. Beeper                  │  launched in parallel,
#   9. Battery Monitor         │  poll-based readiness check
#  10. Floodlight              │  exits as soon as all 4 are alive
#  11. Headlight Controller   ─┘
#  12. ROSBridge Server
#  13. MediaMTX (RTSP/WebRTC Server)
#  14. ICP Odometry
#  15. Localization (map_server + AMCL)  ← lifecycle-polled, exits early when ready
#  16. Delivery Runner
#  17. Navigation (Nav2 stack)
#  18. Waypoint Follower Poses
#  19. Waypoint Recorder
#
# USAGE:
#   ./launch_trov.sh              → uses last saved map, or failure2 if none saved
#   ./launch_trov.sh warehouse2   → uses warehouse2.yaml and saves it
#
# MAP PERSISTENCE:
#   Selected map is saved to ~/.trov_last_map on every localization launch.
#   On next launch (e.g. via systemd service), that map is automatically restored.
#   File is created automatically — no manual setup required.
#
# INTERACTIVE CONTROLS:
#   Press 'r' to restart localization (prompts for new map name)
#   Press 'n' to restart navigation
#   Press 'q' to quit everything
# ============================================================

# ---------- Colors ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

log_info()        { echo -e "${CYAN}[INFO]  $(date '+%H:%M:%S') | $1${NC}"; }
log_ok()          { echo -e "${GREEN}[OK]    $(date '+%H:%M:%S') | $1${NC}"; }
log_warn()        { echo -e "${YELLOW}[WARN]  $(date '+%H:%M:%S') | $1${NC}"; }
log_error()       { echo -e "${RED}[ERROR] $(date '+%H:%M:%S') | $1${NC}"; }
log_interactive() { echo -e "${MAGENTA}[>>>]   $(date '+%H:%M:%S') | $1${NC}"; }

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

MEDIAMTX_DIR="/home/nvidia"

# ── Map state persistence ──────────────────────────────────────────────────────
STATE_FILE="$HOME/.trov_last_map"
DEFAULT_MAP="failure2"

# Priority: CLI argument → saved state file → hardcoded default
if [ -n "${1:-}" ]; then
    SELECTED_MAP="$1"
    log_info "Map from CLI argument: $SELECTED_MAP"
elif [ -f "$STATE_FILE" ]; then
    SELECTED_MAP="$(cat "$STATE_FILE" | tr -d '[:space:]')"
    log_info "Map restored from state file: $SELECTED_MAP"
else
    SELECTED_MAP="$DEFAULT_MAP"
    log_info "No state file found — using default map: $SELECTED_MAP"
fi
# ──────────────────────────────────────────────────────────────────────────────

# Launch files
ICP_LAUNCH_FILE="icp_odometry_indoor.launch.py"
LOCALIZATION_LAUNCH_FILE="localization.launch.py"
NAVIGATION_LAUNCH_FILE="navigation_indoor.launch.py"

# PIDs for cleanup
declare -A NODE_PIDS   # associative: name → pid
declare -A NODE_PGIDS  # associative: name → process group id

# Navigation state flag — used by the background monitor
NAV_FAILED=false

# ---------- Cleanup ----------
cleanup() {
    echo ""
    log_warn "Caught exit signal — shutting down all launched processes..."

    # ── Unset trap first to prevent re-entrant calls ──
    trap - EXIT INT TERM HUP

    # ── Step 1: Send SIGINT to all process groups (graceful ROS2 shutdown) ──
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        if kill -0 -"$pgid" 2>/dev/null; then
            log_info "Sending SIGINT to $name (PGID -$pgid)"
            kill -INT -"$pgid" 2>/dev/null || true
        fi
    done

    # ── Step 2: Wait up to 5s for graceful shutdown ──
    log_info "Waiting 5s for graceful shutdown..."
    sleep 5

    # ── Step 3: Force kill any survivors with SIGKILL ──
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        if kill -0 -"$pgid" 2>/dev/null; then
            log_warn "Force killing $name (PGID -$pgid)"
            kill -9 -"$pgid" 2>/dev/null || true
        fi
    done

    # ── Step 4: Also kill by PID as fallback ──
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Force killing $name by PID ($pid)"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    # ── Step 5: Kill any lingering processes ──
    pkill -9 -f "mavros_node" 2>/dev/null || true
    pkill -9 -f "rslidar_sdk" 2>/dev/null || true
    pkill -9 -f "wit_ros2_imu" 2>/dev/null || true
    pkill -9 -f "icp_odometry" 2>/dev/null || true
    pkill -9 -f "drive_bridge" 2>/dev/null || true
    pkill -9 -f "drive$" 2>/dev/null || true
    # pkill -9 -f "amr_failsafe_node" 2>/dev/null || true
    pkill -9 -f "sensors_health_status" 2>/dev/null || true
    # pkill -9 -f "beeper" 2>/dev/null || true
    pkill -9 -f "battery_monitor" 2>/dev/null || true
    pkill -9 -f "floodlight" 2>/dev/null || true
    pkill -9 -f "headlight_controller" 2>/dev/null || true
    pkill -9 -f "waypoint_follower_poses" 2>/dev/null || true
    pkill -9 -f "waypoint_recorder" 2>/dev/null || true
    pkill -9 -f "delivery_runner" 2>/dev/null || true
    pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
    pkill -9 -f "mediamtx" 2>/dev/null || true
    pkill -9 -f "map_server" 2>/dev/null || true
    pkill -9 -f "amcl" 2>/dev/null || true
    pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "velocity_smoother" 2>/dev/null || true
    pkill -9 -f "collision_monitor" 2>/dev/null || true

    log_ok "Cleanup done. Goodbye."
    exit 0
}

# ── Trap all exit signals ──
trap cleanup EXIT INT TERM HUP

# ---------- Helper: launch in its own process group and register ----------
launch_node() {
    local name="$1"
    shift
    log_info "Launching: $name"

    setsid "$@" &
    local pid=$!
    local pgid
    # Retry PGID lookup up to 10 times (handles slow process startup)
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

# ---------- Helper: kill a specific node ----------
kill_node() {
    local name="$1"
    local pgid="${NODE_PGIDS[$name]}"
    local pid="${NODE_PIDS[$name]}"

    if [ -n "$pgid" ] && kill -0 -"$pgid" 2>/dev/null; then
        log_info "Stopping $name (PGID -$pgid)..."
        kill -INT -"$pgid" 2>/dev/null || true
        sleep 2
        kill -9 -"$pgid" 2>/dev/null || true
    fi

    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
    fi

    unset NODE_PIDS["$name"]
    unset NODE_PGIDS["$name"]
    log_ok "$name stopped."
}

# ---------- Helper: poll until a node process is alive (replaces fixed sleep) ----------
# Exits as soon as the process is confirmed alive for 2 consecutive checks.
# Aborts if the process dies.
wait_for_node() {
    local name="$1"
    local timeout="${2:-30}"
    local pid="${NODE_PIDS[$name]}"
    local elapsed=0
    local alive_count=0

    log_info "Polling $name process (timeout: ${timeout}s)..."
    while [ $elapsed -lt $timeout ]; do
        if ! kill -0 "$pid" 2>/dev/null; then
            log_error "$name (PID $pid) died during startup — aborting."
            exit 1
        fi
        alive_count=$((alive_count + 1))
        # Two consecutive alive checks = good enough
        if [ $alive_count -ge 2 ]; then
            log_ok "$name is alive (confirmed after ${elapsed}s)."
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    log_ok "$name is alive."
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

# ---------- Helper: check if map topic is publishing ----------
check_map_publishing() {
    if timeout 3 ros2 topic echo /map --once 2>/dev/null | grep -q "data"; then
        return 0
    fi
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
# Lifecycle state polling helper
# ============================================================
wait_for_lifecycle() {
    local node="$1"
    local timeout="$2"
    local elapsed=0

    log_info "Waiting for lifecycle node $node to become ACTIVE (timeout: ${timeout}s)..."

    while [ $elapsed -lt $timeout ]; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            local state
            state=$(ros2 lifecycle get "$node" 2>/dev/null | grep -o 'active')
            if [ "$state" = "active" ]; then
                log_ok "$node is ACTIVE (after ${elapsed}s)"
                return 0
            fi
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    log_error "$node did NOT become ACTIVE within ${timeout}s"
    return 1
}

# ============================================================
# Background navigation health monitor
# ============================================================
nav_health_monitor() {
    local check_interval=15
    # Only poll true lifecycle nodes — NOT lifecycle_manager (it's a regular node)
    local nav2_nodes=(
        "/planner_server"
        "/controller_server"
        "/bt_navigator"
        "/behavior_server"
    )

    log_info "[NAV_MONITOR] Navigation health monitor started (checking every ${check_interval}s)"

    while true; do
        sleep "$check_interval"

        if [ -n "${NODE_PIDS[navigation]}" ]; then
            local nav_pid="${NODE_PIDS[navigation]}"
            if ! kill -0 "$nav_pid" 2>/dev/null; then
                log_error "████████████████████████████████████████████████████"
                log_error "  NAV FAILURE: navigation launch process is DEAD"
                log_error "  PID $nav_pid no longer exists"
                log_error "  ACTION REQUIRED: Press 'n' to restart navigation"
                log_error "████████████████████████████████████████████████████"
                NAV_FAILED=true
                continue
            fi
        fi

        local any_failed=false
        for node in "${nav2_nodes[@]}"; do
            local state
            state=$(ros2 lifecycle get "$node" 2>/dev/null | grep -o 'active')
            if [ "$state" != "active" ]; then
                log_error "████████████████████████████████████████████████████"
                log_error "  NAV FAILURE: $node is NOT active"
                log_error "  Current state: $(ros2 lifecycle get "$node" 2>/dev/null || echo 'node not found')"
                log_error "  ACTION REQUIRED: Press 'n' in console to restart navigation"
                log_error "████████████████████████████████████████████████████"
                any_failed=true
            fi
        done

        if [ "$any_failed" = true ]; then
            NAV_FAILED=true
        else
            if [ "$NAV_FAILED" = true ]; then
                log_ok "[NAV_MONITOR] Navigation has recovered — all nodes are ACTIVE"
                NAV_FAILED=false
            fi
        fi
    done
}

# ---------- Function: Launch Localization ----------
launch_localization() {
    local map_name="${1:-$SELECTED_MAP}"

    log_info "Launching localization stack with map: $map_name"

    if [ -n "${NODE_PIDS[localization]:-}" ]; then
        kill_node "localization"
        pkill -9 -f "map_server" 2>/dev/null || true
        pkill -9 -f "amcl" 2>/dev/null || true
        pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
        pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true
        sleep 2
    fi

    launch_node "localization" \
        ros2 launch trov "$LOCALIZATION_LAUNCH_FILE" map_name:="$map_name"

    SELECTED_MAP="$map_name"
    echo "$map_name" > "$STATE_FILE"
    log_ok "Map '$map_name' saved to $STATE_FILE — will be used on next launch."

    # ── Poll for lifecycle_manager_localization to appear ──────────────────
    if ! wait_for_ros_node "lifecycle_manager_localization" 60; then
        log_warn "lifecycle_manager_localization never appeared — localization may be unstable"
        log_warn "Continuing anyway. Press 'r' to restart localization if needed."
        return 1
    fi

    # ── Poll lifecycle states — exits as soon as each is active ───────────
    # NOTE: lifecycle_manager itself is NOT a lifecycle node — do NOT call
    # ros2 lifecycle get on it. Only poll the nodes it manages (amcl, map_server).
    local loc_ok=true
    wait_for_lifecycle "/amcl"       90 || { log_warn "AMCL did not become active"; loc_ok=false; }
    wait_for_lifecycle "/map_server" 30 || { log_warn "map_server did not become active"; loc_ok=false; }

    # Confirm the manager process is still alive (not a lifecycle check — just PID)
    if kill -0 "${NODE_PIDS[localization]}" 2>/dev/null; then
        log_ok "lifecycle_manager_localization process is alive"
    else
        log_warn "localization process died unexpectedly"
        loc_ok=false
    fi

    # ── Poll /map topic — exits as soon as it's publishing ────────────────
    local map_ok=false
    log_info "Polling /map topic (timeout: 30s)..."
    local map_elapsed=0
    while [ $map_elapsed -lt 30 ]; do
        if check_map_publishing; then
            log_ok "Map is publishing! (after ${map_elapsed}s)"
            map_ok=true
            break
        fi
        sleep 1
        map_elapsed=$((map_elapsed + 1))
    done

    if [ "$map_ok" = false ]; then
        log_warn "Map not publishing after 30s. Press 'r' to restart localization if needed."
    fi

    if [ "$loc_ok" = true ] && [ "$map_ok" = true ]; then
        log_ok "════════════════════════════════════════════"
        log_ok "  Localization stack is FULLY ACTIVE"
        log_ok "  amcl                          → active"
        log_ok "  lifecycle_manager_localization → active"
        log_ok "  /map                          → publishing"
        log_ok "════════════════════════════════════════════"
    fi

    return 0
}

# ---------- Function: Launch Navigation ----------
launch_navigation() {
    log_info "Launching navigation stack..."

    if [ -n "${NODE_PIDS[navigation]:-}" ]; then
        kill_node "navigation"
        pkill -9 -f "controller_server" 2>/dev/null || true
        pkill -9 -f "planner_server" 2>/dev/null || true
        pkill -9 -f "behavior_server" 2>/dev/null || true
        pkill -9 -f "bt_navigator" 2>/dev/null || true
        pkill -9 -f "smoother_server" 2>/dev/null || true
        pkill -9 -f "velocity_smoother" 2>/dev/null || true
        pkill -9 -f "collision_monitor" 2>/dev/null || true
        pkill -9 -f "waypoint_follower" 2>/dev/null || true
        pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true
        sleep 2
    fi

    launch_node "navigation" \
        ros2 launch trov "$NAVIGATION_LAUNCH_FILE"

    # ── Poll for lifecycle_manager_navigation to appear ────────────────────
    if ! wait_for_ros_node "lifecycle_manager_navigation" 60; then
        log_error "lifecycle_manager_navigation never appeared — navigation launch likely failed"
        log_error "Nav2 will NOT be active. Press 'n' to retry."
        NAV_FAILED=true
        return 1
    fi

    # ── Poll each lifecycle node — exits as soon as each is active ─────────
    # NOTE: lifecycle_manager_navigation is NOT a lifecycle node itself.
    # Only poll the nodes it manages. Manager health = its managed nodes' health.
    local nav_ok=true

    wait_for_lifecycle "/planner_server"    120 || { log_error "planner_server never became active";    nav_ok=false; }
    wait_for_lifecycle "/controller_server" 120 || { log_error "controller_server never became active"; nav_ok=false; }
    wait_for_lifecycle "/bt_navigator"      120 || { log_error "bt_navigator never became active";      nav_ok=false; }
    wait_for_lifecycle "/behavior_server"   120 || { log_error "behavior_server never became active";   nav_ok=false; }

    # Confirm manager process is alive (PID check only — not a lifecycle query)
    if ! kill -0 "${NODE_PIDS[navigation]}" 2>/dev/null; then
        log_error "navigation launch process died during activation"
        nav_ok=false
    fi

    if [ "$nav_ok" = true ]; then
        log_ok "════════════════════════════════════════════"
        log_ok "  Nav2 stack is FULLY ACTIVE"
        log_ok "  planner_server    → active"
        log_ok "  controller_server → active"
        log_ok "  bt_navigator      → active"
        log_ok "  behavior_server   → active"
        log_ok "════════════════════════════════════════════"
        NAV_FAILED=false

        nav_health_monitor &
        log_info "Navigation health monitor started in background (PID $!)"
    else
        log_error "════════════════════════════════════════════"
        log_error "  NAV FAILURE: Nav2 stack did NOT fully activate"
        log_error "  One or more lifecycle nodes failed to reach ACTIVE state"
        log_error "  Robot CANNOT navigate until this is resolved"
        log_error "  → Press 'n' in the interactive console to retry"
        log_error "  → Or run: journalctl -u trov -f   to watch logs"
        log_error "════════════════════════════════════════════"
        NAV_FAILED=true

        nav_health_monitor &
        log_info "Navigation health monitor started in background (PID $!)"

        return 1
    fi

    return 0
}

# ---------- Function: Interactive control loop ----------
interactive_control() {
    log_interactive "=========================================="
    log_interactive "  INTERACTIVE CONTROLS ACTIVE"
    log_interactive "=========================================="
    log_interactive "  Press 'r' + Enter → Restart Localization"
    log_interactive "  Press 'n' + Enter → Restart Navigation"
    log_interactive "  Press 's' + Enter → Show node status"
    log_interactive "  Press 'm' + Enter → Check map topic"
    log_interactive "  Press 'f' + Enter → Show failsafe error code"
    log_interactive "  Press 'q' + Enter → Quit everything"
    log_interactive "=========================================="
    echo ""

    while true; do
        if read -t 5 -r key 2>/dev/null; then
            key="$(echo "$key" | tr -d '[:space:]')"
            case "$key" in
                r|R)
                    echo ""
                    log_interactive "Enter map name to load (blank = keep '$SELECTED_MAP'):"
                    read -r new_map
                    new_map="${new_map:-$SELECTED_MAP}"
                    log_interactive "Restarting Localization with map: $new_map"
                    launch_localization "$new_map"
                    echo ""
                    log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, f=failsafe, q=quit"
                    ;;
                n|N)
                    echo ""
                    log_interactive "Restarting Navigation..."
                    launch_navigation
                    echo ""
                    log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, f=failsafe, q=quit"
                    ;;
                s|S)
                    echo ""
                    log_interactive "Node Status:"
                    for name in "${!NODE_PIDS[@]}"; do
                        pid="${NODE_PIDS[$name]}"
                        if kill -0 "$pid" 2>/dev/null; then
                            log_ok "  $name → PID $pid (ALIVE)"
                        else
                            log_error "  $name → PID $pid (DEAD)"
                        fi
                    done
                    echo ""
                    log_interactive "Navigation failed flag: $NAV_FAILED"
                    echo ""
                    ;;
                m|M)
                    echo ""
                    log_interactive "Checking /map topic..."
                    if check_map_publishing; then
                        log_ok "Map is publishing!"
                    else
                        log_error "Map is NOT publishing. Press 'r' to restart localization."
                    fi
                    echo ""
                    ;;
                f|F)
                    # ── Show current failsafe error code ──────────────────────
                    echo ""
                    log_interactive "Checking AMR Failsafe error code..."
                    local err_code
                    err_code=$(timeout 3 ros2 topic echo /amr/failsafe/error_code \
                                   --once 2>/dev/null | grep "data:" | awk '{print $2}')
                    if [ -n "$err_code" ]; then
                        case "$err_code" in
                            "0x00") log_ok    "  Failsafe: $err_code — All systems nominal" ;;
                            "0x0R") log_error "  Failsafe: $err_code — ICP Odometry failure" ;;
                            "0x0L") log_error "  Failsafe: $err_code — AMCL Localization failure" ;;
                            "0x0N") log_error "  Failsafe: $err_code — Navigation failure" ;;
                            *)      log_warn  "  Failsafe: $err_code — Unknown code" ;;
                        esac
                    else
                        log_warn "  Could not read /amr/failsafe/error_code (node may be starting)"
                    fi

                    # Show whether failsafe node process is alive
                    if [ -n "${NODE_PIDS[amr_failsafe]:-}" ]; then
                        local fs_pid="${NODE_PIDS[amr_failsafe]}"
                        if kill -0 "$fs_pid" 2>/dev/null; then
                            log_ok "  amr_failsafe process → PID $fs_pid (ALIVE)"
                        else
                            log_error "  amr_failsafe process → PID $fs_pid (DEAD)"
                        fi
                    fi
                    echo ""
                    ;;
                q|Q)
                    echo ""
                    log_interactive "Quitting..."
                    cleanup
                    ;;
            esac
        fi
    done
}

# ============================================================
echo ""
echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}   TROV Full Stack Launch Script           ${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""

log_info "Selected map: $SELECTED_MAP"

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

log_info "ROS_DISTRO     = $ROS_DISTRO"
log_info "ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID:-0 (default)}"

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

# ---------- 6. Check MediaMTX ----------
echo ""
log_info "Checking MediaMTX binary: $MEDIAMTX_DIR/mediamtx"
if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
    log_ok "MediaMTX binary found and executable."
else
    log_warn "MediaMTX not found or not executable at $MEDIAMTX_DIR/mediamtx"
    log_warn "Media streaming will not be available."
fi

# ---------- 7. Check launch files exist ----------
echo ""
log_info "Checking launch files..."

ICP_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$ICP_LAUNCH_FILE"
LOC_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$LOCALIZATION_LAUNCH_FILE"
NAV_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$NAVIGATION_LAUNCH_FILE"

[ -f "$ICP_LAUNCH_PATH" ] && log_ok "ICP: $ICP_LAUNCH_FILE" || { log_error "ICP launch not found: $ICP_LAUNCH_PATH"; exit 1; }
[ -f "$LOC_LAUNCH_PATH" ] && log_ok "Localization: $LOCALIZATION_LAUNCH_FILE" || { log_error "Localization launch not found: $LOC_LAUNCH_PATH"; exit 1; }
[ -f "$NAV_LAUNCH_PATH" ] && log_ok "Navigation: $NAVIGATION_LAUNCH_FILE" || { log_error "Navigation launch not found: $NAV_LAUNCH_PATH"; exit 1; }

# ── Validate selected map exists ──────────────────────────────────────────────
MAPS_DIR="$WORKSPACE_DIR/install/trov/share/trov/maps"
MAP_PATH="$MAPS_DIR/$SELECTED_MAP.yaml"
if [ ! -f "$MAP_PATH" ]; then
    log_error "Map '$SELECTED_MAP' not found at $MAP_PATH"
    log_error "Available maps:"
    ls "$MAPS_DIR"/*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  /'
    if [ -f "$STATE_FILE" ] && [ -z "${1:-}" ]; then
        log_warn "State file contained an invalid map name — falling back to default: $DEFAULT_MAP"
        SELECTED_MAP="$DEFAULT_MAP"
        MAP_PATH="$MAPS_DIR/$SELECTED_MAP.yaml"
        if [ ! -f "$MAP_PATH" ]; then
            log_error "Default map '$DEFAULT_MAP' also not found. Aborting."
            exit 1
        fi
        log_ok "Fell back to default map: $SELECTED_MAP"
    else
        exit 1
    fi
fi
log_ok "Map '$SELECTED_MAP' found."

# ============================================================
echo ""
echo -e "${CYAN}--- Starting hardware drivers ---${NC}"
echo ""
cd "$WORKSPACE_DIR"

# ---------- 8. RSLidar SDK ----------
# No dependency — launch immediately and poll for /points topic
launch_node "rslidar_sdk" \
    ros2 launch rslidar_sdk humble_start.py

# ---------- 9. WIT IMU ----------
# No dependency — launch immediately and poll for /imu/data topic
launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# ── Poll both drivers alive (2 consecutive checks, ~2s each max) ──────────────
# We don't block on topics here — the topic polls happen later right before
# whoever needs them (ICP). This lets LiDAR + IMU warm up in parallel with
# MAVROS + drive_bridge below.
wait_for_node "rslidar_sdk" 15
wait_for_node "wit_imu"     15

# ---------- 10. RealSense Camera (disabled) ----------
#launch_node "realsense_camera" \
#    ros2 launch realsense2_camera rs_launch.py
#wait_for_node "realsense_camera" 10

# ---------- 11. MAVROS ----------
# Depends on: FCU port (already checked above)
echo ""
echo -e "${CYAN}--- Starting MAVROS ---${NC}"
echo ""
cd "$HOME"
launch_node "mavros" \
   ros2 run mavros mavros_node \
       --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"

# Poll for /mavros/state topic — proves MAVROS is actually talking to the FCU.
# This replaces the old blind 15s sleep.
wait_for_topic "/mavros/state" 30 || log_warn "MAVROS /mavros/state not seen — FCU connection may have #issues"
cd "$WORKSPACE_DIR"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting drive bridge ---${NC}"
echo ""

# ---------- 12. Drive Bridge ----------
# Depends on: MAVROS (needs FCU comms path set up)
launch_node "drive_bridge" \
   ros2 run cpp_pubsub drive
wait_for_node "drive_bridge" 10

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
echo -e "${CYAN}--- Starting MediaMTX (RTSP/WebRTC Server) ---${NC}"
echo ""

# ---------- 18. MediaMTX ----------
if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
    cd "$MEDIAMTX_DIR"
    launch_node "mediamtx" \
        ./mediamtx
    wait_for_node "mediamtx" 10
    cd "$WORKSPACE_DIR"
else
    log_warn "Skipping MediaMTX — binary not found."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting ICP Odometry ---${NC}"
echo ""

# ── Gate: all 3 sensor prerequisites must be confirmed before ICP launches ────
# These polls exit as soon as the condition is met — no fixed sleep.
# ICP MUST NOT start blind; if sensors aren't ready it will crash or drift.

log_info "Confirming sensor prerequisites for ICP..."

lidar_ok=true
tf_ok=true
imu_ok=true

wait_for_topic "/points"              45 || { log_warn "LiDAR /points not ready";          lidar_ok=false; }
wait_for_tf    "base_link" "lidar_link" 30 || { log_warn "TF base_link→lidar_link not ready"; tf_ok=false;    }
wait_for_imu   "/imu/data"            30 || { log_warn "IMU /imu/data not ready";           imu_ok=false;   }

if [ "$lidar_ok" = false ] || [ "$tf_ok" = false ] || [ "$imu_ok" = false ]; then
    log_warn "One or more ICP prerequisites are not fully ready — ICP may be unreliable."
    log_warn "Proceeding anyway. Check sensor connections if odometry fails."
fi

# ── ICP launch with retry — polls /odom instead of sleeping ───────────────────
max_attempts=3
icp_success=false

for attempt in $(seq 1 $max_attempts); do
    log_info "ICP Odometry launch attempt $attempt/$max_attempts"

    launch_node "icp_odometry" \
        ros2 launch trov "$ICP_LAUNCH_FILE"

    # Poll for /odom — exits as soon as ICP is publishing.
    # Max 60s per attempt. No blind sleep.
    log_info "Polling for /odom from ICP (timeout: 60s)..."
    icp_elapsed=0
    icp_ready=false
    while [ $icp_elapsed -lt 60 ]; do
        # First confirm the process hasn't crashed
        if ! kill -0 "${NODE_PIDS[icp_odometry]}" 2>/dev/null; then
            log_warn "ICP process died on attempt $attempt"
            break
        fi
        # Then check if /odom is publishing
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
echo -e "${CYAN}--- Starting Localization (map_server + AMCL) ---${NC}"
echo ""

# ── Gate: /odom must be available before localization starts ──────────────────
# AMCL fuses odometry with laser — launching without /odom = bad initial pose.
wait_for_topic "/odom" 30 || log_warn "/odom not available — AMCL may have poor initial pose"

launch_localization

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Delivery Runner ---${NC}"
echo ""

# ---------- 19. Delivery Runner ----------
# No hard dependency on nav — can start as soon as localization is up
launch_node "delivery_runner" \
    ros2 run trov delivery_runner.py
wait_for_node "delivery_runner" 10

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Navigation (Nav2 stack) ---${NC}"
echo ""

# ── Gate: /map must be publishing before Nav2 launches ────────────────────────
# Nav2 costmaps depend on the map — launching without it causes immediate failure.
wait_for_topic "/map" 30 || log_warn "/map not available — Nav2 costmaps may fail to initialize"

launch_navigation

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Waypoint nodes ---${NC}"
echo ""

# ---------- 20. Waypoint Follower Poses ----------
launch_node "waypoint_follower_poses" \
    ros2 run trov waypoint_follower_poses.py
wait_for_node "waypoint_follower_poses" 10

# ---------- 21. Waypoint Recorder ----------
launch_node "waypoint_recorder" \
    ros2 run trov waypoint_recorder.py
wait_for_node "waypoint_recorder" 10

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
log_info "Selected map: $SELECTED_MAP"
log_info "Map state file: $STATE_FILE"
log_info "Navigation status: $([ "$NAV_FAILED" = true ] && echo 'FAILED — restart service to retry' || echo 'OK')"
log_info "Failsafe error code topic: /amr/failsafe/error_code  "
log_info "ROSBridge WebSocket available at ws://<robot_ip>:9090"
log_info "MediaMTX RTSP available at rtsp://<robot_ip>:8554"
log_info "MediaMTX WebRTC available at http://<robot_ip>:8889"
log_info "Headlight topic: /trov/headlight (publish true/false)"
echo ""

# ---------- Keep-alive loop (no TTY needed under systemd) ----------
# Monitors all launched nodes every 30s and logs any that have died.
# Script stays alive so systemd manages the process lifetime.
# To stop:    sudo systemctl stop trov
# To restart: sudo systemctl restart trov
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
        log_error "Use: sudo systemctl restart trov"
    fi
done
