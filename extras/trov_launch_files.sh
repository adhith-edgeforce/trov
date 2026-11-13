#!/bin/bash
# ============================================================
# TROV Workspace Launch Script for Indoor  (fast-start version)
#
# Startup order:
#   1. RSLidar SDK + WIT IMU                 (sensors, launched first)
#   2. MAVROS, Drive bridge, Sensor Health,  (independent of odometry, launched
#      Battery, Floodlight, Headlight,        immediately WITHOUT blocking, so they
#      ROSBridge, MediaMTX                    warm up while sensors are being gated)
#   3. Sensor gate: /points, TF, IMU         (checked IN PARALLEL)
#   4. Odometry (FAST-LIO2 default, ICP selectable — see ODOM_BACKEND)
#   5. Localization (map_server + AMCL)      (lifecycle-polled, exits early)
#   6. Delivery Runner
#   7. Navigation (Nav2)                     (4 lifecycle nodes polled in parallel)
#   8. Waypoint Follower Poses + Waypoint Recorder
#
# AMR Failsafe is currently NOT launched by this script.
#
# USAGE:
#   ./launch_trov.sh              → last saved map, or failure2 if none saved
#   ./launch_trov.sh warehouse2   → uses warehouse2.yaml and saves it
#
# MAP PERSISTENCE:
#   Selected map is saved to ~/.trov_last_map on every localization launch.
#
# ODOMETRY BACKEND:
#   ODOM_BACKEND=icp ./launch_trov.sh
#
# TIMING:
#   Every stage prints how long it took, and a summary table is shown at the end.
#
# INTERACTIVE CONTROLS (only when run from a terminal; under systemd it just
# monitors nodes every 30s):
#   r = restart localization   n = restart navigation   s = node status
#   m = check /map             f = failsafe code        q = quit
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

# ---------- Stage timing ----------
SCRIPT_T0=$SECONDS
STAGE_T0=$SECONDS
STAGE_LOG=()
stage_done() {
    local now=$SECONDS
    local d=$((now - STAGE_T0))
    STAGE_LOG+=("$(printf '%-42s %4ss' "$1" "$d")")
    log_info "⏱  $1 took ${d}s (total so far: $((now - SCRIPT_T0))s)"
    STAGE_T0=$now
}

# ---------- Config ----------
WORKSPACE_DIR="/data/trov_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WORKSPACE_DIR/install/setup.bash"

LIDAR_IP="192.168.2.202"
LIDAR_PING_TIMEOUT=3
LIDAR_PING_COUNT=1
LIDAR_PING_TRIES=3

IMU_PORT="/dev/ttyUSB0"
FCU_PORT="/dev/ttyACM0"
FCU_BAUD="57600"

MEDIAMTX_DIR="/home/nvidia"

# Seconds to wait for the first /odom message on each odometry attempt.
ODOM_TIMEOUT="${ODOM_TIMEOUT:-60}"
ODOM_MAX_ATTEMPTS="${ODOM_MAX_ATTEMPTS:-3}"

# ── Odometry backend ──────────────────────────────────────────────────────────
#   fastlio : FAST-LIO2 + RS-Airy built-in IMU + lio_odom_bridge   (default)
#   icp     : rtabmap icp_odometry + WIT IMU                       (legacy)
#
# Measured return-to-home error, 40 m indoor drive, same route:
#   icp     : 8.40 m position, 54.8 deg heading   (21.0% of distance)
#   fastlio : 0.12 m position,  0.20 deg heading  ( 0.30% of distance)
#
# Both publish /odom and the odom -> base_link TF, so everything downstream is
# identical either way.
ODOM_BACKEND="${ODOM_BACKEND:-fastlio}"

# ── Map state persistence ──────────────────────────────────────────────────────
STATE_FILE="$HOME/.trov_last_map"
DEFAULT_MAP="failure2"

if [ -n "${1:-}" ]; then
    SELECTED_MAP="$1"
    log_info "Map from CLI argument: $SELECTED_MAP"
elif [ -f "$STATE_FILE" ]; then
    SELECTED_MAP="$(tr -d '[:space:]' < "$STATE_FILE")"
    log_info "Map restored from state file: $SELECTED_MAP"
else
    SELECTED_MAP="$DEFAULT_MAP"
    log_info "No state file found — using default map: $SELECTED_MAP"
fi

# Launch files
ICP_LAUNCH_FILE="icp_odometry_indoor.launch.py"
FASTLIO_LAUNCH_FILE="fast_lio_odometry_indoor.launch.py"
LOCALIZATION_LAUNCH_FILE="localization.launch.py"
NAVIGATION_LAUNCH_FILE="navigation_indoor.launch.py"

# ── Resolve the odometry backend into concrete settings ──────────────────────
case "$ODOM_BACKEND" in
    fastlio)
        ODOM_LAUNCH_FILE="$FASTLIO_LAUNCH_FILE"
        ODOM_LABEL="FAST-LIO2 Odometry"
        ODOM_IMU_TOPIC="/rslidar_imu_data"
        ODOM_PKILL_PATS=("fastlio_mapping" "lio_odom_bridge" "lidar_to_imu_static_tf")
        WIT_IMU_CRITICAL="no"
        ;;
    icp)
        ODOM_LAUNCH_FILE="$ICP_LAUNCH_FILE"
        ODOM_LABEL="ICP Odometry"
        ODOM_IMU_TOPIC="/imu/data"
        ODOM_PKILL_PATS=("icp_odometry" "rtabmap_odom")
        WIT_IMU_CRITICAL="yes"
        ;;
    *)
        echo "[ERROR] Unknown ODOM_BACKEND '$ODOM_BACKEND' — use 'fastlio' or 'icp'." >&2
        exit 1
        ;;
esac

# PIDs for cleanup
declare -A NODE_PIDS    # name → pid
declare -A NODE_PGIDS   # name → process group id

NAV_FAILED=false
NAV_MON_PID=""

# ---------- Cleanup ----------
cleanup() {
    echo ""
    log_warn "Caught exit signal — shutting down all launched processes..."
    trap - EXIT INT TERM HUP

    [ -n "$NAV_MON_PID" ] && kill "$NAV_MON_PID" 2>/dev/null

    # Step 1: SIGINT to every process group (graceful ROS2 shutdown)
    local name pgid pid i any
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        if kill -0 -- "-$pgid" 2>/dev/null; then
            log_info "Sending SIGINT to $name (PGID -$pgid)"
            kill -INT -- "-$pgid" 2>/dev/null || true
        fi
    done

    # Step 2: wait up to 5s, but stop as soon as everything is gone
    for i in $(seq 1 10); do
        any=false
        for name in "${!NODE_PGIDS[@]}"; do
            if kill -0 -- "-${NODE_PGIDS[$name]}" 2>/dev/null; then any=true; break; fi
        done
        [ "$any" = false ] && break
        sleep 0.5
    done

    # Step 3: SIGKILL survivors
    for name in "${!NODE_PGIDS[@]}"; do
        pgid="${NODE_PGIDS[$name]}"
        if kill -0 -- "-$pgid" 2>/dev/null; then
            log_warn "Force killing $name (PGID -$pgid)"
            kill -9 -- "-$pgid" 2>/dev/null || true
        fi
    done

    # Step 4: PID fallback
    for name in "${!NODE_PIDS[@]}"; do
        pid="${NODE_PIDS[$name]}"
        if kill -0 "$pid" 2>/dev/null; then
            log_warn "Force killing $name by PID ($pid)"
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    # Step 5: lingering processes
    pkill -9 -f "mavros_node" 2>/dev/null || true
    pkill -9 -f "rslidar_sdk" 2>/dev/null || true
    pkill -9 -f "wit_ros2_imu" 2>/dev/null || true
    # Both odometry backends are killed regardless of ODOM_BACKEND so a leftover
    # from a previous run can never fight the current one over /odom and TF.
    pkill -9 -f "icp_odometry" 2>/dev/null || true
    pkill -9 -f "rtabmap_odom" 2>/dev/null || true
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "lio_odom_bridge" 2>/dev/null || true
    # Name-specific on purpose: a bare "static_transform_publisher" would also
    # kill base_link->lidar_link and every other static TF.
    pkill -9 -f "lidar_to_imu_static_tf" 2>/dev/null || true
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

trap cleanup EXIT INT TERM HUP

# ---------- Helper: launch in its own process group and register ----------
launch_node() {
    local name="$1"
    shift
    log_info "Launching: $name"

    setsid "$@" &
    local pid=$!
    local pgid i
    # setsid makes the child its own group leader (PGID == PID). Right after the
    # fork `ps` can still show OUR group, so wait only until it matches the PID
    # (normally 1 quick iteration) and otherwise fall back to the PID. This must
    # never record our own PGID, or a later `kill -- -PGID` would kill this script.
    for i in $(seq 1 10); do
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ "$pgid" = "$pid" ] && break
        sleep 0.05
    done
    pgid="$pid"

    NODE_PIDS["$name"]=$pid
    NODE_PGIDS["$name"]=$pgid
    log_ok "$name started (PID $pid | PGID $pgid)"
}

# ---------- Helper: kill a specific node ----------
kill_node() {
    local name="$1"
    local pgid="${NODE_PGIDS[$name]:-}"
    local pid="${NODE_PIDS[$name]:-}"
    local i

    if [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null; then
        log_info "Stopping $name (PGID -$pgid)..."
        kill -INT -- "-$pgid" 2>/dev/null || true
        for i in $(seq 1 6); do
            kill -0 -- "-$pgid" 2>/dev/null || break
            sleep 0.5
        done
        kill -9 -- "-$pgid" 2>/dev/null || true
    fi

    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
    fi

    unset "NODE_PIDS[$name]"
    unset "NODE_PGIDS[$name]"
    log_ok "$name stopped."
}

# ---------- Helper: confirm a launched process is still alive ----------
# critical=yes → abort the whole script if it died.
check_alive() {
    local name="$1"
    local critical="${2:-yes}"
    local pid="${NODE_PIDS[$name]:-}"

    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        log_ok "  $name → alive (PID $pid)"
        return 0
    fi
    if [ "$critical" = "yes" ]; then
        log_error "  $name (PID ${pid:-?}) died during startup — aborting."
        exit 1
    fi
    log_error "  $name → DEAD (PID ${pid:-?}) — continuing anyway"
    return 1
}

# ============================================================
# Polling helpers.
# All deadlines use wall-clock ($SECONDS), NOT a loop counter, because every
# `ros2` CLI call itself takes 1-3s on a Jetson. Every CLI call is wrapped in
# `timeout` so a single hung call can never stall a stage.
# ============================================================

# ---------- Wait for a TF to be available ----------
wait_for_tf() {
    local parent="$1" child="$2" timeout_s="$3"
    local t0=$SECONDS
    local deadline=$((SECONDS + timeout_s))

    log_info "Waiting for TF: $parent → $child (timeout: ${timeout_s}s)..."
    while [ $SECONDS -lt $deadline ]; do
        # tf2_echo never exits by itself — bound it, or this check can hang.
        if timeout -k 1 4 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -q "Translation"; then
            log_ok "TF $parent → $child is available (after $((SECONDS - t0))s)"
            return 0
        fi
        sleep 0.5
    done
    log_warn "TF $parent → $child not available after ${timeout_s}s"
    return 1
}

# ---------- Wait for IMU data ----------
wait_for_imu() {
    local topic="$1" timeout_s="$2"
    local t0=$SECONDS
    local deadline=$((SECONDS + timeout_s))

    log_info "Waiting for IMU data on $topic (timeout: ${timeout_s}s)..."
    while [ $SECONDS -lt $deadline ]; do
        if timeout 6 ros2 topic echo "$topic" --once 2>/dev/null | grep -q "orientation"; then
            log_ok "IMU data available on $topic (after $((SECONDS - t0))s)"
            return 0
        fi
        sleep 0.5
    done
    log_warn "IMU not publishing on $topic after ${timeout_s}s"
    return 1
}

# ---------- Wait for a topic to have publishers ----------
wait_for_topic() {
    local topic="$1" timeout_s="$2"
    local t0=$SECONDS
    local deadline=$((SECONDS + timeout_s))

    log_info "Waiting for topic $topic (timeout: ${timeout_s}s)..."
    while [ $SECONDS -lt $deadline ]; do
        if timeout 5 ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
            log_ok "Topic $topic has publishers (after $((SECONDS - t0))s)"
            return 0
        fi
        sleep 0.5
    done
    log_warn "Topic $topic has no publishers after ${timeout_s}s"
    return 1
}

# ---------- Is /map actually delivering a message? ----------
check_map_publishing() {
    timeout 5 ros2 topic echo /map --once 2>/dev/null | grep -q "data"
}

# ---------- Is /odom actually delivering a message? ----------
# Needs headroom: CLI startup + DDS discovery alone can eat 2s+.
odom_publishing() {
    timeout 6 ros2 topic echo /odom --once 2>/dev/null | grep -q "pose"
}

# ---------- Wait for a ROS2 node to appear in node list ----------
wait_for_ros_node() {
    local node_name="$1" timeout_s="$2"
    local t0=$SECONDS
    local deadline=$((SECONDS + timeout_s))

    log_info "Waiting for ROS2 node $node_name to register (timeout: ${timeout_s}s)..."
    while [ $SECONDS -lt $deadline ]; do
        if timeout 5 ros2 node list 2>/dev/null | grep -q "$node_name"; then
            log_ok "ROS2 node $node_name is registered (after $((SECONDS - t0))s)"
            return 0
        fi
        sleep 0.5
    done
    log_warn "ROS2 node $node_name did not appear after ${timeout_s}s"
    return 1
}

# ---------- Lifecycle: is the node in the ACTIVE state? ----------
# `ros2 lifecycle get` prints e.g. "active [3]" or "inactive [2]".
# Anchor on the start of the line — a plain grep for 'active' also matches
# "inactive" and would report the node as ready too early.
lifecycle_is_active() {
    timeout 5 ros2 lifecycle get "$1" 2>/dev/null | grep -qE '^active'
}

wait_for_lifecycle() {
    local node="$1" timeout_s="$2"
    local t0=$SECONDS
    local deadline=$((SECONDS + timeout_s))

    log_info "Waiting for lifecycle node $node to become ACTIVE (timeout: ${timeout_s}s)..."
    while [ $SECONDS -lt $deadline ]; do
        if lifecycle_is_active "$node"; then
            log_ok "$node is ACTIVE (after $((SECONDS - t0))s)"
            return 0
        fi
        sleep 0.5
    done
    log_error "$node did NOT become ACTIVE within ${timeout_s}s"
    return 1
}

# ============================================================
# Background navigation health monitor
# ============================================================
nav_health_monitor() {
    local check_interval=15
    # Only true lifecycle nodes — NOT lifecycle_manager (it's a regular node)
    local nav2_nodes=("/planner_server" "/controller_server" "/bt_navigator" "/behavior_server")

    log_info "[NAV_MONITOR] Navigation health monitor started (checking every ${check_interval}s)"

    while true; do
        sleep "$check_interval"

        if [ -n "${NODE_PIDS[navigation]:-}" ]; then
            local nav_pid="${NODE_PIDS[navigation]}"
            if ! kill -0 "$nav_pid" 2>/dev/null; then
                log_error "████████████████████████████████████████████████████"
                log_error "  NAV FAILURE: navigation launch process is DEAD"
                log_error "  PID $nav_pid no longer exists"
                log_error "  ACTION REQUIRED: Press 'n' to restart navigation"
                log_error "████████████████████████████████████████████████████"
                continue
            fi
        fi

        local node
        for node in "${nav2_nodes[@]}"; do
            if ! lifecycle_is_active "$node"; then
                log_error "████████████████████████████████████████████████████"
                log_error "  NAV FAILURE: $node is NOT active"
                log_error "  Current state: $(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || echo 'node not found')"
                log_error "  ACTION REQUIRED: Press 'n' in console to restart navigation"
                log_error "████████████████████████████████████████████████████"
            fi
        done
    done
}

# Start exactly one monitor (restarts used to leak a new one every time).
# NOTE: this runs in a background subshell, so it can only LOG failures — it
# cannot update NAV_FAILED in the main shell.
start_nav_monitor() {
    [ -n "$NAV_MON_PID" ] && kill "$NAV_MON_PID" 2>/dev/null
    nav_health_monitor &
    NAV_MON_PID=$!
    log_info "Navigation health monitor running in background (PID $NAV_MON_PID)"
}

# ---------- Function: Launch Localization ----------
launch_localization() {
    local map_name="${1:-$SELECTED_MAP}"
    local t0=$SECONDS

    log_info "Launching localization stack with map: $map_name"

    if [ -n "${NODE_PIDS[localization]:-}" ]; then
        kill_node "localization"
        pkill -9 -f "map_server" 2>/dev/null || true
        pkill -9 -f "amcl" 2>/dev/null || true
        pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
        pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true
        sleep 1
    fi

    launch_node "localization" \
        ros2 launch trov "$LOCALIZATION_LAUNCH_FILE" map_name:="$map_name"

    SELECTED_MAP="$map_name"
    echo "$map_name" > "$STATE_FILE"
    log_ok "Map '$map_name' saved to $STATE_FILE — will be used on next launch."

    if ! wait_for_ros_node "lifecycle_manager_localization" 60; then
        log_warn "lifecycle_manager_localization never appeared — localization may be unstable"
        log_warn "Continuing anyway. Press 'r' to restart localization if needed."
        return 1
    fi

    # lifecycle_manager itself is NOT a lifecycle node — only poll what it manages.
    # amcl and map_server are polled in parallel so one slow node can't hold up the other.
    local loc_ok=true
    wait_for_lifecycle "/amcl" 90 & local p_amcl=$!
    wait_for_lifecycle "/map_server" 30 & local p_map=$!
    wait "$p_amcl" || { log_warn "AMCL did not become active"; loc_ok=false; }
    wait "$p_map"  || { log_warn "map_server did not become active"; loc_ok=false; }

    if kill -0 "${NODE_PIDS[localization]}" 2>/dev/null; then
        log_ok "localization launch process is alive"
    else
        log_warn "localization process died unexpectedly"
        loc_ok=false
    fi

    local map_ok=false
    local map_deadline=$((SECONDS + 15))
    log_info "Polling /map topic (timeout: 15s)..."
    while [ $SECONDS -lt $map_deadline ]; do
        if check_map_publishing; then
            log_ok "Map is publishing!"
            map_ok=true
            break
        fi
        sleep 0.5
    done
    [ "$map_ok" = false ] && log_warn "Map not publishing after 15s. Press 'r' to restart localization if needed."

    if [ "$loc_ok" = true ] && [ "$map_ok" = true ]; then
        log_ok "════════════════════════════════════════════"
        log_ok "  Localization stack is FULLY ACTIVE ($((SECONDS - t0))s)"
        log_ok "  amcl        → active"
        log_ok "  map_server  → active"
        log_ok "  /map        → publishing"
        log_ok "════════════════════════════════════════════"
    fi
    return 0
}

# ---------- Function: Launch Navigation ----------
launch_navigation() {
    local t0=$SECONDS
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
        sleep 1
    fi

    launch_node "navigation" \
        ros2 launch trov "$NAVIGATION_LAUNCH_FILE"

    if ! wait_for_ros_node "lifecycle_manager_navigation" 60; then
        log_error "lifecycle_manager_navigation never appeared — navigation launch likely failed"
        log_error "Nav2 will NOT be active. Press 'n' to retry."
        NAV_FAILED=true
        return 1
    fi

    # The four Nav2 nodes are polled IN PARALLEL (they used to be checked one
    # after another, each with its own 120s budget).
    local nav_ok=true
    wait_for_lifecycle "/planner_server"    120 & local p1=$!
    wait_for_lifecycle "/controller_server" 120 & local p2=$!
    wait_for_lifecycle "/bt_navigator"      120 & local p3=$!
    wait_for_lifecycle "/behavior_server"   120 & local p4=$!
    wait "$p1" || { log_error "planner_server never became active";    nav_ok=false; }
    wait "$p2" || { log_error "controller_server never became active"; nav_ok=false; }
    wait "$p3" || { log_error "bt_navigator never became active";      nav_ok=false; }
    wait "$p4" || { log_error "behavior_server never became active";   nav_ok=false; }

    if ! kill -0 "${NODE_PIDS[navigation]}" 2>/dev/null; then
        log_error "navigation launch process died during activation"
        nav_ok=false
    fi

    if [ "$nav_ok" = true ]; then
        log_ok "════════════════════════════════════════════"
        log_ok "  Nav2 stack is FULLY ACTIVE ($((SECONDS - t0))s)"
        log_ok "  planner_server / controller_server /"
        log_ok "  bt_navigator / behavior_server → active"
        log_ok "════════════════════════════════════════════"
        NAV_FAILED=false
        start_nav_monitor
    else
        log_error "════════════════════════════════════════════"
        log_error "  NAV FAILURE: Nav2 stack did NOT fully activate"
        log_error "  One or more lifecycle nodes failed to reach ACTIVE state"
        log_error "  Robot CANNOT navigate until this is resolved"
        log_error "  → Press 'n' in the interactive console to retry"
        log_error "  → Or run: journalctl -u trov -f   to watch logs"
        log_error "════════════════════════════════════════════"
        NAV_FAILED=true
        start_nav_monitor
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

    local key new_map name pid err_code fs_pid
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
                    echo ""
                    log_interactive "Checking AMR Failsafe error code..."
                    err_code=$(timeout 3 ros2 topic echo /amr/failsafe/error_code \
                                   --once 2>/dev/null | grep "data:" | awk '{print $2}')
                    if [ -n "$err_code" ]; then
                        case "$err_code" in
                            "0x00") log_ok    "  Failsafe: $err_code — All systems nominal" ;;
                            "0x0R") log_error "  Failsafe: $err_code — Odometry failure [$ODOM_BACKEND]" ;;
                            "0x0L") log_error "  Failsafe: $err_code — AMCL Localization failure" ;;
                            "0x0N") log_error "  Failsafe: $err_code — Navigation failure" ;;
                            *)      log_warn  "  Failsafe: $err_code — Unknown code" ;;
                        esac
                    else
                        log_warn "  Could not read /amr/failsafe/error_code (node not running?)"
                    fi

                    if [ -n "${NODE_PIDS[amr_failsafe]:-}" ]; then
                        fs_pid="${NODE_PIDS[amr_failsafe]}"
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

# ---------- 1. Source ROS2 + workspace ----------
log_info "Sourcing ROS2 Humble: $ROS2_SETUP"
[ -f "$ROS2_SETUP" ] || { log_error "ROS2 setup not found: $ROS2_SETUP"; exit 1; }
source "$ROS2_SETUP"
log_ok "ROS2 Humble sourced."

log_info "Sourcing workspace: $WS_SETUP"
[ -f "$WS_SETUP" ] || { log_error "Workspace not found. Run colcon build first."; exit 1; }
source "$WS_SETUP"
log_ok "Workspace sourced."

log_info "ROS_DISTRO     = $ROS_DISTRO"
log_info "ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID:-0 (default)}"

# Start the ROS 2 daemon once so every later `ros2 topic/node/lifecycle` call
# reuses cached discovery instead of re-discovering the graph from scratch.
log_info "Starting ros2 daemon (caches discovery for all later CLI calls)..."
ros2 daemon start >/dev/null 2>&1 || log_warn "ros2 daemon failed to start — CLI calls will be slower"
stage_done "Source ROS + start daemon"

# ---------- 2. Hardware pre-checks ----------
echo ""
log_info "Pinging LiDAR at $LIDAR_IP ..."
lidar_reachable=false
for _try in $(seq 1 "$LIDAR_PING_TRIES"); do
    if ping -c "$LIDAR_PING_COUNT" -W "$LIDAR_PING_TIMEOUT" "$LIDAR_IP" > /dev/null 2>&1; then
        lidar_reachable=true
        break
    fi
done
if [ "$lidar_reachable" = true ]; then
    log_ok "LiDAR reachable."
else
    log_error "Cannot reach LiDAR at $LIDAR_IP — check cable / IP / power."
    exit 1
fi

echo ""
log_info "Checking IMU port: $IMU_PORT"
if [ -e "$IMU_PORT" ]; then
    log_ok "IMU port found: $IMU_PORT"
else
    ls /dev/ttyUSB* 2>/dev/null || echo "  (none)"
    if [ "$ODOM_BACKEND" = "icp" ]; then
        log_error "IMU port $IMU_PORT not found — ICP odometry requires the WIT IMU."
        exit 1
    else
        log_warn "IMU port $IMU_PORT not found."
        log_warn "ODOM_BACKEND=$ODOM_BACKEND uses the RS-Airy built-in IMU, so odometry"
        log_warn "is unaffected. Continuing — but anything else on /imu/data will be down."
    fi
fi

echo ""
log_info "Checking FCU port: $FCU_PORT"
if [ -e "$FCU_PORT" ]; then
    log_ok "FCU port found: $FCU_PORT"
else
    log_warn "FCU port $FCU_PORT not found — MAVROS may fail."
    log_warn "Available ACM ports:"
    ls /dev/ttyACM* 2>/dev/null || echo "  (none)"
fi

echo ""
log_info "Checking MediaMTX binary: $MEDIAMTX_DIR/mediamtx"
MEDIAMTX_OK=false
if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
    log_ok "MediaMTX binary found and executable."
    MEDIAMTX_OK=true
else
    log_warn "MediaMTX not found or not executable at $MEDIAMTX_DIR/mediamtx"
    log_warn "Media streaming will not be available."
fi

# ---------- 3. Launch files + map exist ----------
echo ""
log_info "Checking launch files..."

LAUNCH_DIR="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor"
ODOM_LAUNCH_PATH="$LAUNCH_DIR/$ODOM_LAUNCH_FILE"
LOC_LAUNCH_PATH="$LAUNCH_DIR/$LOCALIZATION_LAUNCH_FILE"
NAV_LAUNCH_PATH="$LAUNCH_DIR/$NAVIGATION_LAUNCH_FILE"

[ -f "$ODOM_LAUNCH_PATH" ] && log_ok "Odometry [$ODOM_BACKEND]: $ODOM_LAUNCH_FILE" || { log_error "Odometry launch not found: $ODOM_LAUNCH_PATH"; exit 1; }
[ -f "$LOC_LAUNCH_PATH" ]  && log_ok "Localization: $LOCALIZATION_LAUNCH_FILE"      || { log_error "Localization launch not found: $LOC_LAUNCH_PATH"; exit 1; }
[ -f "$NAV_LAUNCH_PATH" ]  && log_ok "Navigation: $NAVIGATION_LAUNCH_FILE"          || { log_error "Navigation launch not found: $NAV_LAUNCH_PATH"; exit 1; }

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
stage_done "Pre-flight checks"

# ============================================================
echo ""
echo -e "${CYAN}--- Launching drivers and peripherals (non-blocking) ---${NC}"
echo ""
cd "$WORKSPACE_DIR"

# Everything below is independent of odometry, so it is launched back-to-back
# with NO waiting. It warms up while the sensor gate (next section) runs, and a
# single liveness check at the end replaces ~12 separate blocking waits.

# ---------- Sensors ----------
launch_node "rslidar_sdk" \
    ros2 launch rslidar_sdk humble_start.py

launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py

# ---------- RealSense Camera (disabled) ----------
#launch_node "realsense_camera" \
#    ros2 launch realsense2_camera rs_launch.py

# ---------- MAVROS ----------
cd "$HOME"
launch_node "mavros" \
    ros2 run mavros mavros_node \
        --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"
cd "$WORKSPACE_DIR"

# Verify MAVROS in the background; it is only collected at the very end so it
# can never delay odometry.
( wait_for_topic "/mavros/state" 30 >/dev/null 2>&1 ) &
MAV_CHECK_PID=$!

# ---------- Drive bridge ----------
launch_node "drive_bridge" \
    ros2 run cpp_pubsub drive

# ---------- Peripherals ----------
launch_node "sensors_health_status" ros2 run trov sensors_health_status
launch_node "battery_monitor"       ros2 run trov battery_monitor
launch_node "floodlight"            ros2 run trov floodlight.py
launch_node "headlight_controller"  ros2 run trov headlight_controller

# ---------- ROSBridge ----------
launch_node "rosbridge_server" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    call_services_in_new_thread:=true \
    send_action_goals_in_new_thread:=true \
    default_call_service_timeout:=5.0

# ---------- MediaMTX ----------
if [ "$MEDIAMTX_OK" = true ]; then
    cd "$MEDIAMTX_DIR"
    launch_node "mediamtx" ./mediamtx
    cd "$WORKSPACE_DIR"
else
    log_warn "Skipping MediaMTX — binary not found."
fi
stage_done "Launch drivers + peripherals"

# ============================================================
echo ""
echo -e "${CYAN}--- Sensor gate for $ODOM_LABEL [backend: $ODOM_BACKEND] ---${NC}"
echo ""

# The three prerequisites are independent, so they are checked IN PARALLEL
# (worst case = slowest single check, not the sum of all three).
#   fastlio -> /rslidar_imu_data (RS-Airy built-in)    icp -> /imu/data (WIT IMU)
log_info "Confirming sensor prerequisites for $ODOM_LABEL (parallel)..."

wait_for_topic "/points"                  45 & g1=$!
wait_for_tf    "base_link" "lidar_link"   30 & g2=$!
wait_for_imu   "$ODOM_IMU_TOPIC"          30 & g3=$!

lidar_ok=true; tf_ok=true; imu_ok=true
wait "$g1" || { log_warn "LiDAR /points not ready";             lidar_ok=false; }
wait "$g2" || { log_warn "TF base_link→lidar_link not ready";   tf_ok=false;    }
wait "$g3" || { log_warn "IMU $ODOM_IMU_TOPIC not ready";       imu_ok=false;   }

if [ "$lidar_ok" = false ] || [ "$tf_ok" = false ] || [ "$imu_ok" = false ]; then
    log_warn "One or more prerequisites are not fully ready — $ODOM_LABEL may be unreliable."
    log_warn "Proceeding anyway. Check sensor connections if odometry fails."
fi

# Single liveness check for everything launched above (they've had the whole
# gate duration to start, so no per-node waiting is needed).
log_info "Verifying launched nodes are alive..."
check_alive "rslidar_sdk"            yes
check_alive "wit_imu"                "$WIT_IMU_CRITICAL"
check_alive "mavros"                 no
check_alive "drive_bridge"           yes
check_alive "sensors_health_status"  no
check_alive "battery_monitor"        no
check_alive "floodlight"             no
check_alive "headlight_controller"   no
check_alive "rosbridge_server"       yes
[ "$MEDIAMTX_OK" = true ] && check_alive "mediamtx" yes
stage_done "Sensor gate (parallel) + liveness check"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting $ODOM_LABEL  [backend: $ODOM_BACKEND] ---${NC}"
echo ""

odom_success=false
for attempt in $(seq 1 "$ODOM_MAX_ATTEMPTS"); do
    log_info "$ODOM_LABEL launch attempt $attempt/$ODOM_MAX_ATTEMPTS"

    launch_node "odometry" \
        ros2 launch trov "$ODOM_LAUNCH_FILE"

    log_info "Polling for /odom from $ODOM_LABEL (timeout: ${ODOM_TIMEOUT}s)..."
    odom_t0=$SECONDS
    odom_deadline=$((SECONDS + ODOM_TIMEOUT))
    odom_ready=false
    while [ $SECONDS -lt $odom_deadline ]; do
        if ! kill -0 "${NODE_PIDS[odometry]}" 2>/dev/null; then
            log_warn "$ODOM_LABEL process died on attempt $attempt"
            break
        fi
        if odom_publishing; then
            log_ok "$ODOM_LABEL publishing /odom (attempt $attempt, after $((SECONDS - odom_t0))s)"
            odom_ready=true
            break
        fi
        sleep 0.5
    done

    if [ "$odom_ready" = true ]; then
        odom_success=true
        break
    fi

    log_warn "$ODOM_LABEL failed on attempt $attempt"
    if [ "$attempt" -lt "$ODOM_MAX_ATTEMPTS" ]; then
        kill_node "odometry"
        for pat in "${ODOM_PKILL_PATS[@]}"; do
            pkill -9 -f "$pat" 2>/dev/null || true
        done
        sleep 1
    fi
done

if [ "$odom_success" = false ]; then
    log_error "$ODOM_LABEL failed after $ODOM_MAX_ATTEMPTS attempts"
    log_error "Continuing anyway — odometry will not work. Localization may drift."
fi
stage_done "Odometry ($ODOM_BACKEND, attempts used: ${attempt:-1})"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Localization (map_server + AMCL) ---${NC}"
echo ""

# /odom was just confirmed by the odometry stage above — no second wait here.
launch_localization
stage_done "Localization (map_server + AMCL)"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Delivery Runner ---${NC}"
echo ""

launch_node "delivery_runner" \
    ros2 run trov delivery_runner.py

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Navigation (Nav2 stack) ---${NC}"
echo ""

# launch_localization already confirmed /map is delivering data; this is only a
# quick publisher sanity check before Nav2's costmaps need it.
wait_for_topic "/map" 10 || log_warn "/map not available — Nav2 costmaps may fail to initialize"

launch_navigation
stage_done "Navigation (Nav2)"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Waypoint nodes ---${NC}"
echo ""

launch_node "waypoint_follower_poses" \
    ros2 run trov waypoint_follower_poses.py

launch_node "waypoint_recorder" \
    ros2 run trov waypoint_recorder.py

sleep 1
check_alive "delivery_runner"          yes
check_alive "waypoint_follower_poses"  yes
check_alive "waypoint_recorder"        yes

# Collect the background MAVROS check that has been running since launch.
if wait "$MAV_CHECK_PID"; then
    log_ok "MAVROS /mavros/state has publishers"
else
    log_warn "MAVROS /mavros/state not seen — FCU connection may have issues"
fi
stage_done "Delivery runner + waypoint nodes"

# ============================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   All nodes launched!  ($((SECONDS - SCRIPT_T0))s total)${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
log_info "Startup time per stage:"
for line in "${STAGE_LOG[@]}"; do
    log_info "  $line"
done
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
log_info "Odometry backend: $ODOM_BACKEND ($ODOM_LABEL) — IMU: $ODOM_IMU_TOPIC"
log_info "Map state file: $STATE_FILE"
log_info "Navigation status: $([ "$NAV_FAILED" = true ] && echo 'FAILED — restart service to retry' || echo 'OK')"
log_info "Failsafe error code topic: /amr/failsafe/error_code"
log_info "ROSBridge WebSocket available at ws://<robot_ip>:9090"
log_info "MediaMTX RTSP available at rtsp://<robot_ip>:8554"
log_info "MediaMTX WebRTC available at http://<robot_ip>:8889"
log_info "Headlight topic: /trov/headlight (publish true/false)"
echo ""

# ---------- Keep-alive ----------
# Terminal → interactive controls. systemd (no TTY) → monitor nodes every 30s.
# To stop:    sudo systemctl stop trov
# To restart: sudo systemctl restart trov
if [ -t 0 ]; then
    interactive_control
else
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
fi
