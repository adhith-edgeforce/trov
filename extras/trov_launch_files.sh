# #!/bin/bash
# # ============================================================
# # TROV Workspace Launch Script
# # Launches (in order):
# #   1. RSLidar SDK
# #   2. WIT IMU
# #   3. RealSense Camera
# #   4. MAVROS
# #   5. Drive bridge
# #   6. Sensor Health Status
# #   7. Collision Beacon
# #   8. Battery Monitor
# #   9. Floodlight
# #  10. Headlight Controller
# #  11. ROSBridge Server
# #  12. MediaMTX (RTSP/WebRTC Server)
# #  13. ICP Odometry
# #  14. Localization (map_server + AMCL)
# #  15. Navigation (Nav2 stack)
# #  16. Waypoint Follower Poses
# #  17. Waypoint Recorder
# #
# # USAGE:
# #   ./launch_trov.sh              → uses default map (indoor3)
# #   ./launch_trov.sh warehouse2   → uses warehouse2.yaml
# #
# # INTERACTIVE CONTROLS:
# #   Press 'r' to restart localization if map doesn't appear
# #   Press 'n' to restart navigation
# #   Press 'q' to quit everything
# # ============================================================

# # ---------- Colors ----------
# RED='\033[0;31m'
# GREEN='\033[0;32m'
# YELLOW='\033[1;33m'
# CYAN='\033[0;36m'
# MAGENTA='\033[0;35m'
# NC='\033[0m'

# log_info()  { echo -e "${CYAN}[INFO]  $(date '+%H:%M:%S') | $1${NC}"; }
# log_ok()    { echo -e "${GREEN}[OK]    $(date '+%H:%M:%S') | $1${NC}"; }
# log_warn()  { echo -e "${YELLOW}[WARN]  $(date '+%H:%M:%S') | $1${NC}"; }
# log_error() { echo -e "${RED}[ERROR] $(date '+%H:%M:%S') | $1${NC}"; }
# log_interactive() { echo -e "${MAGENTA}[>>>]   $(date '+%H:%M:%S') | $1${NC}"; }

# # ---------- Config ----------
# WORKSPACE_DIR="/data/trov_ws"
# ROS2_SETUP="/opt/ros/humble/setup.bash"
# WS_SETUP="$WORKSPACE_DIR/install/setup.bash"

# LIDAR_IP="192.168.2.202"
# LIDAR_PING_TIMEOUT=5
# LIDAR_PING_COUNT=3

# IMU_PORT="/dev/ttyUSB0"
# FCU_PORT="/dev/ttyACM0"
# FCU_BAUD="57600"

# MEDIAMTX_DIR="/home/nvidia"

# # ── Map selection: use first argument, default to indoor3 ──
# SELECTED_MAP="${1:-indoor3}"

# # Launch files
# ICP_LAUNCH_FILE="icp_odometry_indoor.launch.py"
# LOCALIZATION_LAUNCH_FILE="localization.launch.py"
# NAVIGATION_LAUNCH_FILE="navigation_indoor.launch.py"

# # PIDs for cleanup
# declare -A NODE_PIDS   # associative: name → pid
# declare -A NODE_PGIDS  # associative: name → process group id

# # ---------- Cleanup ----------
# cleanup() {
#     echo ""
#     log_warn "Caught exit signal — shutting down all launched processes..."

#     # ── Step 1: Send SIGINT to all process groups (graceful ROS2 shutdown) ──
#     for name in "${!NODE_PGIDS[@]}"; do
#         pgid="${NODE_PGIDS[$name]}"
#         if kill -0 -"$pgid" 2>/dev/null; then
#             log_info "Sending SIGINT to $name (PGID -$pgid)"
#             kill -INT -"$pgid" 2>/dev/null || true
#         fi
#     done

#     # ── Step 2: Wait up to 5s for graceful shutdown ──
#     log_info "Waiting 5s for graceful shutdown..."
#     sleep 5

#     # ── Step 3: Force kill any survivors with SIGKILL ──
#     for name in "${!NODE_PGIDS[@]}"; do
#         pgid="${NODE_PGIDS[$name]}"
#         if kill -0 -"$pgid" 2>/dev/null; then
#             log_warn "Force killing $name (PGID -$pgid)"
#             kill -9 -"$pgid" 2>/dev/null || true
#         fi
#     done

#     # ── Step 4: Also kill by PID as fallback ──
#     for name in "${!NODE_PIDS[@]}"; do
#         pid="${NODE_PIDS[$name]}"
#         if kill -0 "$pid" 2>/dev/null; then
#             log_warn "Force killing $name by PID ($pid)"
#             kill -9 "$pid" 2>/dev/null || true
#         fi
#     done

#     # ── Step 5: Kill any lingering processes ──
#     pkill -9 -f "mavros_node" 2>/dev/null || true
#     pkill -9 -f "rslidar_sdk" 2>/dev/null || true
#     pkill -9 -f "wit_ros2_imu" 2>/dev/null || true
#     pkill -9 -f "icp_odometry" 2>/dev/null || true
#     pkill -9 -f "drive_bridge\|drive$" 2>/dev/null || true
#     pkill -9 -f "sensors_health_status" 2>/dev/null || true
#     pkill -9 -f "collision_beacon" 2>/dev/null || true
#     pkill -9 -f "battery_monitor" 2>/dev/null || true
#     pkill -9 -f "floodlight" 2>/dev/null || true
#     pkill -9 -f "headlight_controller" 2>/dev/null || true
#     pkill -9 -f "waypoint_follower_poses" 2>/dev/null || true
#     pkill -9 -f "waypoint_recorder" 2>/dev/null || true
#     pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
#     pkill -9 -f "mediamtx" 2>/dev/null || true
#     pkill -9 -f "map_server" 2>/dev/null || true
#     pkill -9 -f "amcl" 2>/dev/null || true
#     pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
#     pkill -9 -f "lifecycle_manager" 2>/dev/null || true
#     pkill -9 -f "controller_server" 2>/dev/null || true
#     pkill -9 -f "planner_server" 2>/dev/null || true
#     pkill -9 -f "behavior_server" 2>/dev/null || true
#     pkill -9 -f "bt_navigator" 2>/dev/null || true
#     pkill -9 -f "velocity_smoother" 2>/dev/null || true
#     pkill -9 -f "collision_monitor" 2>/dev/null || true

#     log_ok "Cleanup done. Goodbye."
#     exit 0
# }

# # ── Trap all exit signals ──
# trap cleanup EXIT INT TERM HUP

# # ---------- Helper: launch in its own process group and register ----------
# launch_node() {
#     local name="$1"
#     shift
#     log_info "Launching: $name"

#     setsid "$@" &
#     local pid=$!
#     local pgid
#     sleep 0.3
#     pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ') || pgid=$pid

#     NODE_PIDS["$name"]=$pid
#     NODE_PGIDS["$name"]=$pgid
#     log_ok "$name started (PID $pid | PGID $pgid)"
# }

# # ---------- Helper: kill a specific node ----------
# kill_node() {
#     local name="$1"
#     local pgid="${NODE_PGIDS[$name]}"
#     local pid="${NODE_PIDS[$name]}"
    
#     if [ -n "$pgid" ] && kill -0 -"$pgid" 2>/dev/null; then
#         log_info "Stopping $name (PGID -$pgid)..."
#         kill -INT -"$pgid" 2>/dev/null || true
#         sleep 2
#         kill -9 -"$pgid" 2>/dev/null || true
#     fi
    
#     if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
#         kill -9 "$pid" 2>/dev/null || true
#     fi
    
#     unset NODE_PIDS["$name"]
#     unset NODE_PGIDS["$name"]
#     log_ok "$name stopped."
# }

# # ---------- Helper: wait and check a node is still alive ----------
# wait_for_node() {
#     local name="$1"
#     local wait_secs="$2"
#     local pid="${NODE_PIDS[$name]}"
#     log_info "Waiting ${wait_secs}s for $name to initialize..."
#     sleep "$wait_secs"
#     if ! kill -0 "$pid" 2>/dev/null; then
#         log_error "$name (PID $pid) died during startup — aborting."
#         exit 1
#     fi
#     log_ok "$name is alive."
# }

# # ---------- Helper: wait for a TF to be available ----------
# wait_for_tf() {
#     local parent="$1"
#     local child="$2"
#     local timeout="$3"
#     local elapsed=0
    
#     log_info "Waiting for TF: $parent → $child (timeout: ${timeout}s)..."
#     while [ $elapsed -lt $timeout ]; do
#         if ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -q "Translation"; then
#             log_ok "TF $parent → $child is available"
#             return 0
#         fi
#         sleep 1
#         elapsed=$((elapsed + 1))
#     done
#     log_warn "TF $parent → $child not available after ${timeout}s"
#     return 1
# }

# # ---------- Helper: wait for IMU data ----------
# wait_for_imu() {
#     local topic="$1"
#     local timeout="$2"
#     local elapsed=0
    
#     log_info "Waiting for IMU data on $topic (timeout: ${timeout}s)..."
#     while [ $elapsed -lt $timeout ]; do
#         if timeout 2 ros2 topic echo "$topic" --once 2>/dev/null | grep -q "orientation"; then
#             log_ok "IMU data available on $topic"
#             return 0
#         fi
#         sleep 1
#         elapsed=$((elapsed + 1))
#     done
#     log_warn "IMU not publishing on $topic after ${timeout}s"
#     return 1
# }

# # ---------- Helper: wait for a topic to have publishers ----------
# wait_for_topic() {
#     local topic="$1"
#     local timeout="$2"
#     local elapsed=0
    
#     log_info "Waiting for topic $topic (timeout: ${timeout}s)..."
#     while [ $elapsed -lt $timeout ]; do
#         if ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
#             log_ok "Topic $topic has publishers"
#             return 0
#         fi
#         sleep 1
#         elapsed=$((elapsed + 1))
#     done
#     log_warn "Topic $topic has no publishers after ${timeout}s"
#     return 1
# }

# # ---------- Helper: check if map topic is publishing ----------
# check_map_publishing() {
#     if timeout 3 ros2 topic echo /map --once 2>/dev/null | grep -q "data"; then
#         return 0
#     fi
#     return 1
# }

# # ---------- Function: Launch Localization ----------
# launch_localization() {
#     # Accept optional map name argument, fall back to SELECTED_MAP
#     local map_name="${1:-$SELECTED_MAP}"

#     log_info "Launching localization stack with map: $map_name"
    
#     # Kill existing localization if running
#     if [ -n "${NODE_PIDS[localization]}" ]; then
#         kill_node "localization"
#         # Also kill related processes
#         pkill -9 -f "map_server" 2>/dev/null || true
#         pkill -9 -f "amcl" 2>/dev/null || true
#         pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
#         pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true
#         sleep 2
#     fi
    
#     launch_node "localization" \
#         ros2 launch trov "$LOCALIZATION_LAUNCH_FILE" map_name:="$map_name"
    
#     # Update SELECTED_MAP so 'r' in interactive mode reuses the last chosen map
#     SELECTED_MAP="$map_name"

#     # Wait for localization to initialize
#     log_info "Waiting 10s for localization to initialize..."
#     sleep 10
    
#     # Check if map is publishing
#     local map_ok=false
#     for i in 1 2 3; do
#         log_info "Checking for /map topic (attempt $i/3)..."
#         if check_map_publishing; then
#             log_ok "Map is publishing!"
#             map_ok=true
#             break
#         fi
#         sleep 3
#     done
    
#     if [ "$map_ok" = false ]; then
#         log_warn "Map not publishing yet. You may need to restart localization."
#         log_warn "Press 'r' to restart localization once the system is running."
#     fi
    
#     return 0
# }

# # ---------- Function: Launch Navigation ----------
# launch_navigation() {
#     log_info "Launching navigation stack..."
    
#     # Kill existing navigation if running
#     if [ -n "${NODE_PIDS[navigation]}" ]; then
#         kill_node "navigation"
#         # Also kill related processes
#         pkill -9 -f "controller_server" 2>/dev/null || true
#         pkill -9 -f "planner_server" 2>/dev/null || true
#         pkill -9 -f "behavior_server" 2>/dev/null || true
#         pkill -9 -f "bt_navigator" 2>/dev/null || true
#         pkill -9 -f "smoother_server" 2>/dev/null || true
#         pkill -9 -f "velocity_smoother" 2>/dev/null || true
#         pkill -9 -f "collision_monitor" 2>/dev/null || true
#         pkill -9 -f "waypoint_follower" 2>/dev/null || true
#         pkill -9 -f "lifecycle_manager_navigation" 2>/dev/null || true
#         sleep 2
#     fi
    
#     launch_node "navigation" \
#         ros2 launch trov "$NAVIGATION_LAUNCH_FILE"
    
#     # Wait for navigation to initialize
#     log_info "Waiting 15s for navigation stack to initialize..."
#     sleep 15
    
#     if kill -0 "${NODE_PIDS[navigation]}" 2>/dev/null; then
#         log_ok "Navigation stack is running"
#     else
#         log_error "Navigation stack failed to start"
#         return 1
#     fi
    
#     return 0
# }

# # ---------- Function: Interactive control loop ----------
# interactive_control() {
#     log_interactive "=========================================="
#     log_interactive "  INTERACTIVE CONTROLS ACTIVE"
#     log_interactive "=========================================="
#     log_interactive "  Press 'r' + Enter → Restart Localization"
#     log_interactive "  Press 'n' + Enter → Restart Navigation"
#     log_interactive "  Press 's' + Enter → Show node status"
#     log_interactive "  Press 'm' + Enter → Check map topic"
#     log_interactive "  Press 'q' + Enter → Quit everything"
#     log_interactive "=========================================="
#     echo ""
    
#     while true; do
#         # Read with timeout so we can check if children are still alive
#         if read -t 5 -n 1 key 2>/dev/null; then
#             case "$key" in
#                 r|R)
#                     echo ""
#                     log_interactive "Restarting Localization (map: $SELECTED_MAP)..."
#                     launch_localization
#                     echo ""
#                     log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, q=quit"
#                     ;;
#                 n|N)
#                     echo ""
#                     log_interactive "Restarting Navigation..."
#                     launch_navigation
#                     echo ""
#                     log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, q=quit"
#                     ;;
#                 s|S)
#                     echo ""
#                     log_interactive "Node Status:"
#                     for name in "${!NODE_PIDS[@]}"; do
#                         pid="${NODE_PIDS[$name]}"
#                         if kill -0 "$pid" 2>/dev/null; then
#                             log_ok "  $name → PID $pid (ALIVE)"
#                         else
#                             log_error "  $name → PID $pid (DEAD)"
#                         fi
#                     done
#                     echo ""
#                     ;;
#                 m|M)
#                     echo ""
#                     log_interactive "Checking /map topic..."
#                     if check_map_publishing; then
#                         log_ok "Map is publishing!"
#                     else
#                         log_error "Map is NOT publishing. Press 'r' to restart localization."
#                     fi
#                     echo ""
#                     ;;
#                 q|Q)
#                     echo ""
#                     log_interactive "Quitting..."
#                     cleanup
#                     ;;
#             esac
#         fi
#     done
# }

# # ============================================================
# echo ""
# echo -e "${CYAN}============================================${NC}"
# echo -e "${CYAN}   TROV Full Stack Launch Script           ${NC}"
# echo -e "${CYAN}============================================${NC}"
# echo ""

# log_info "Selected map: $SELECTED_MAP"

# # ---------- 1. Source ROS2 ----------
# log_info "Sourcing ROS2 Humble: $ROS2_SETUP"
# [ -f "$ROS2_SETUP" ] || { log_error "ROS2 setup not found: $ROS2_SETUP"; exit 1; }
# source "$ROS2_SETUP"
# log_ok "ROS2 Humble sourced."

# # ---------- 2. Source workspace ----------
# log_info "Sourcing workspace: $WS_SETUP"
# [ -f "$WS_SETUP" ] || { log_error "Workspace not found. Run colcon build first."; exit 1; }
# source "$WS_SETUP"
# log_ok "Workspace sourced."

# log_info "ROS_DISTRO     = $ROS_DISTRO"
# log_info "ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID:-0 (default)}"

# # ---------- 3. Check LiDAR ----------
# echo ""
# log_info "Pinging LiDAR at $LIDAR_IP ..."
# if ping -c "$LIDAR_PING_COUNT" -W "$LIDAR_PING_TIMEOUT" "$LIDAR_IP" > /dev/null 2>&1; then
#     log_ok "LiDAR reachable."
# else
#     log_error "Cannot reach LiDAR at $LIDAR_IP — check cable / IP / power."
#     exit 1
# fi

# # ---------- 4. Check IMU ----------
# echo ""
# log_info "Checking IMU port: $IMU_PORT"
# if [ -e "$IMU_PORT" ]; then
#     log_ok "IMU port found: $IMU_PORT"
# else
#     log_error "IMU port $IMU_PORT not found. Available ports:"
#     ls /dev/ttyUSB* 2>/dev/null || echo "  (none)"
#     exit 1
# fi

# # ---------- 5. Check FCU (Pixhawk) ----------
# echo ""
# log_info "Checking FCU port: $FCU_PORT"
# if [ -e "$FCU_PORT" ]; then
#     log_ok "FCU port found: $FCU_PORT"
# else
#     log_warn "FCU port $FCU_PORT not found — MAVROS may fail."
#     log_warn "Available ACM ports:"
#     ls /dev/ttyACM* 2>/dev/null || echo "  (none)"
# fi

# # ---------- 6. Check MediaMTX ----------
# echo ""
# log_info "Checking MediaMTX binary: $MEDIAMTX_DIR/mediamtx"
# if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
#     log_ok "MediaMTX binary found and executable."
# else
#     log_warn "MediaMTX not found or not executable at $MEDIAMTX_DIR/mediamtx"
#     log_warn "Media streaming will not be available."
# fi

# # ---------- 7. Check launch files exist ----------
# echo ""
# log_info "Checking launch files..."

# ICP_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$ICP_LAUNCH_FILE"
# LOC_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$LOCALIZATION_LAUNCH_FILE"
# NAV_LAUNCH_PATH="$WORKSPACE_DIR/install/trov/share/trov/launch/indoor/$NAVIGATION_LAUNCH_FILE"

# [ -f "$ICP_LAUNCH_PATH" ] && log_ok "ICP: $ICP_LAUNCH_FILE" || { log_error "ICP launch not found: $ICP_LAUNCH_PATH"; exit 1; }
# [ -f "$LOC_LAUNCH_PATH" ] && log_ok "Localization: $LOCALIZATION_LAUNCH_FILE" || { log_error "Localization launch not found: $LOC_LAUNCH_PATH"; exit 1; }
# [ -f "$NAV_LAUNCH_PATH" ] && log_ok "Navigation: $NAVIGATION_LAUNCH_FILE" || { log_error "Navigation launch not found: $NAV_LAUNCH_PATH"; exit 1; }

# # ── Validate selected map exists ──────────────────────────────────────────────
# MAPS_DIR="$WORKSPACE_DIR/install/trov/share/trov/maps"
# MAP_PATH="$MAPS_DIR/$SELECTED_MAP.yaml"
# if [ ! -f "$MAP_PATH" ]; then
#     log_error "Map '$SELECTED_MAP' not found at $MAP_PATH"
#     log_error "Available maps:"
#     ls "$MAPS_DIR"/*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  /'
#     exit 1
# fi
# log_ok "Map '$SELECTED_MAP' found."

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting hardware drivers ---${NC}"
# echo ""
# cd "$WORKSPACE_DIR"

# # ---------- 8. RSLidar SDK ----------
# launch_node "rslidar_sdk" \
#     ros2 launch rslidar_sdk humble_start.py
# wait_for_node "rslidar_sdk" 5

# # ---------- 9. WIT IMU ----------
# launch_node "wit_imu" \
#     ros2 launch wit_ros2_imu rviz_and_imu.launch.py
# wait_for_node "wit_imu" 5

# # ---------- 10. RealSense Camera ----------
# launch_node "realsense_camera" \
#     ros2 launch realsense2_camera rs_launch.py
# wait_for_node "realsense_camera" 5

# # ---------- 11. MAVROS ----------
# echo ""
# echo -e "${CYAN}--- Starting MAVROS ---${NC}"
# echo ""
# cd "$HOME"
# launch_node "mavros" \
#     ros2 run mavros mavros_node \
#         --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"
# wait_for_node "mavros" 15
# cd "$WORKSPACE_DIR"

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting drive bridge ---${NC}"
# echo ""

# # ---------- 12. Drive Bridge ----------
# launch_node "drive_bridge" \
#     ros2 run cpp_pubsub drive
# wait_for_node "drive_bridge" 3

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting sensor health & peripheral nodes ---${NC}"
# echo ""

# # ---------- 13. Sensor Health Status ----------
# launch_node "sensors_health_status" \
#     ros2 run trov sensors_health_status
# wait_for_node "sensors_health_status" 3

# # ---------- 14. Collision Beacon ----------
# launch_node "collision_beacon" \
#     ros2 run trov collision_beacon
# wait_for_node "collision_beacon" 3

# # ---------- 15. Battery Monitor ----------
# launch_node "battery_monitor" \
#     ros2 run trov battery_monitor
# wait_for_node "battery_monitor" 3

# # ---------- 16. Floodlight ----------
# launch_node "floodlight" \
#     ros2 run trov floodlight.py
# wait_for_node "floodlight" 3

# # ---------- 17. Headlight Controller ----------
# launch_node "headlight_controller" \
#     ros2 run trov headlight_controller
# wait_for_node "headlight_controller" 3

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting ROSBridge Server ---${NC}"
# echo ""

# # ---------- 18. ROSBridge WebSocket Server ----------
# # fix for Humble: service calls and action goals must run in new threads
# # otherwise the Tornado main loop blocks and the websocket freezes on connect
# launch_node "rosbridge_server" \
#     ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
#     call_services_in_new_thread:=true \
#     send_action_goals_in_new_thread:=true \
#     default_call_service_timeout:=5.0
# wait_for_node "rosbridge_server" 3

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting MediaMTX (RTSP/WebRTC Server) ---${NC}"
# echo ""

# # ---------- 19. MediaMTX ----------
# if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
#     cd "$MEDIAMTX_DIR"
#     launch_node "mediamtx" \
#         ./mediamtx
#     wait_for_node "mediamtx" 3
#     cd "$WORKSPACE_DIR"
# else
#     log_warn "Skipping MediaMTX — binary not found."
# fi

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting ICP Odometry ---${NC}"
# echo ""

# # ---------- 20. ICP Odometry ----------
# wait_for_topic "/points" 30 || log_warn "LiDAR topic /points not ready"
# wait_for_tf "base_link" "lidar_link" 30 || log_warn "TF base_link→lidar_link not ready"
# wait_for_imu "/imu/data" 30 || log_warn "IMU topic /imu/data not ready"

# log_info "Waiting 5s for sensor data to stabilize..."
# sleep 5

# max_attempts=3
# icp_success=false

# for attempt in $(seq 1 $max_attempts); do
#     log_info "ICP Odometry launch attempt $attempt/$max_attempts"
    
#     launch_node "icp_odometry" \
#         ros2 launch trov "$ICP_LAUNCH_FILE"
    
#     log_info "Waiting 15s for ICP to initialize..."
#     sleep 15
    
#     if kill -0 "${NODE_PIDS[icp_odometry]}" 2>/dev/null; then
#         log_info "Process alive, checking for /odom topic..."
#         if timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -q "pose"; then
#             log_ok "ICP Odometry started successfully and publishing /odom"
#             icp_success=true
#             break
#         fi
#     fi
    
#     log_warn "ICP Odometry failed on attempt $attempt"
    
#     if [ $attempt -lt $max_attempts ]; then
#         pkill -9 -f "icp_odometry" 2>/dev/null || true
#         sleep 3
#     fi
# done

# if [ "$icp_success" = false ]; then
#     log_error "ICP Odometry failed after $max_attempts attempts"
#     log_error "Continuing anyway — odometry may not work."
# fi

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting Localization (map_server + AMCL) ---${NC}"
# echo ""

# # ---------- 21. Localization ----------
# # Wait for /odom before starting localization
# wait_for_topic "/odom" 30 || log_warn "/odom not available — localization may have issues"

# launch_localization

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting Navigation (Nav2 stack) ---${NC}"
# echo ""

# # ---------- 22. Navigation ----------
# # Wait for /map before starting navigation
# wait_for_topic "/map" 30 || log_warn "/map not available — you may need to restart localization (press 'r')"

# launch_navigation

# # ============================================================
# echo ""
# echo -e "${CYAN}--- Starting Waypoint nodes ---${NC}"
# echo ""

# # ---------- 23. Waypoint Follower Poses ----------
# launch_node "waypoint_follower_poses" \
#     ros2 run trov waypoint_follower_poses.py
# wait_for_node "waypoint_follower_poses" 3

# # ---------- 24. Waypoint Recorder ----------
# launch_node "waypoint_recorder" \
#     ros2 run trov waypoint_recorder.py
# wait_for_node "waypoint_recorder" 3

# # ============================================================
# echo ""
# echo -e "${GREEN}============================================${NC}"
# echo -e "${GREEN}   All nodes launched!                     ${NC}"
# echo -e "${GREEN}============================================${NC}"
# echo ""
# log_info "Active nodes:"
# for name in "${!NODE_PIDS[@]}"; do
#     pid="${NODE_PIDS[$name]}"
#     if kill -0 "$pid" 2>/dev/null; then
#         log_ok "  $name → PID $pid | PGID ${NODE_PGIDS[$name]}"
#     else
#         log_error "  $name → PID $pid (DEAD)"
#     fi
# done
# echo ""
# log_info "Selected map: $SELECTED_MAP"
# log_info "ROSBridge WebSocket available at ws://<robot_ip>:9090"
# log_info "MediaMTX RTSP available at rtsp://<robot_ip>:8554"
# log_info "MediaMTX WebRTC available at http://<robot_ip>:8889"
# log_info "Headlight topic: /trov/headlight (publish true/false)"
# echo ""

# # ---------- Enter interactive control mode ----------
# interactive_control








#!/bin/bash
# ============================================================
# TROV Workspace Launch Script
# Launches (in order):
#   1. RSLidar SDK
#   2. WIT IMU
#   3. RealSense Camera
#   4. MAVROS
#   5. Drive bridge
#   6. Sensor Health Status
#   7. Collision Beacon
#   8. Battery Monitor
#   9. Floodlight
#  10. Headlight Controller
#  11. ROSBridge Server
#  12. MediaMTX (RTSP/WebRTC Server)
#  13. ICP Odometry
#  14. Localization (map_server + AMCL)
#  15. Navigation (Nav2 stack)
#  16. Waypoint Follower Poses
#  17. Waypoint Recorder
#
# USAGE:
#   ./launch_trov.sh              → uses last saved map, or indoor3 if none saved
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

log_info()  { echo -e "${CYAN}[INFO]  $(date '+%H:%M:%S') | $1${NC}"; }
log_ok()    { echo -e "${GREEN}[OK]    $(date '+%H:%M:%S') | $1${NC}"; }
log_warn()  { echo -e "${YELLOW}[WARN]  $(date '+%H:%M:%S') | $1${NC}"; }
log_error() { echo -e "${RED}[ERROR] $(date '+%H:%M:%S') | $1${NC}"; }
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
DEFAULT_MAP="indoor3"

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

# ---------- Cleanup ----------
cleanup() {
    echo ""
    log_warn "Caught exit signal — shutting down all launched processes..."

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
    pkill -9 -f "drive_bridge\|drive$" 2>/dev/null || true
    pkill -9 -f "sensors_health_status" 2>/dev/null || true
    pkill -9 -f "collision_beacon" 2>/dev/null || true
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
    sleep 0.3
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ') || pgid=$pid

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

# ---------- Helper: wait and check a node is still alive ----------
wait_for_node() {
    local name="$1"
    local wait_secs="$2"
    local pid="${NODE_PIDS[$name]}"
    log_info "Waiting ${wait_secs}s for $name to initialize..."
    sleep "$wait_secs"
    if ! kill -0 "$pid" 2>/dev/null; then
        log_error "$name (PID $pid) died during startup — aborting."
        exit 1
    fi
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
            log_ok "TF $parent → $child is available"
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
            log_ok "IMU data available on $topic"
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
            log_ok "Topic $topic has publishers"
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

# ---------- Function: Launch Localization ----------
launch_localization() {
    # Accept optional map name argument, fall back to SELECTED_MAP
    local map_name="${1:-$SELECTED_MAP}"

    log_info "Launching localization stack with map: $map_name"
    
    # Kill existing localization if running
    if [ -n "${NODE_PIDS[localization]}" ]; then
        kill_node "localization"
        # Also kill related processes
        pkill -9 -f "map_server" 2>/dev/null || true
        pkill -9 -f "amcl" 2>/dev/null || true
        pkill -9 -f "pointcloud_to_laserscan" 2>/dev/null || true
        pkill -9 -f "lifecycle_manager_localization" 2>/dev/null || true
        sleep 2
    fi
    
    launch_node "localization" \
        ros2 launch trov "$LOCALIZATION_LAUNCH_FILE" map_name:="$map_name"
    
    # ── Update and persist the selected map ───────────────────────────────────
    SELECTED_MAP="$map_name"
    echo "$map_name" > "$STATE_FILE"
    log_ok "Map '$map_name' saved to $STATE_FILE — will be used on next launch."
    # ──────────────────────────────────────────────────────────────────────────

    # Wait for localization to initialize
    log_info "Waiting 10s for localization to initialize..."
    sleep 10
    
    # Check if map is publishing
    local map_ok=false
    for i in 1 2 3; do
        log_info "Checking for /map topic (attempt $i/3)..."
        if check_map_publishing; then
            log_ok "Map is publishing!"
            map_ok=true
            break
        fi
        sleep 3
    done
    
    if [ "$map_ok" = false ]; then
        log_warn "Map not publishing yet. You may need to restart localization."
        log_warn "Press 'r' to restart localization once the system is running."
    fi
    
    return 0
}

# ---------- Function: Launch Navigation ----------
launch_navigation() {
    log_info "Launching navigation stack..."
    
    if [ -n "${NODE_PIDS[navigation]}" ]; then
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
    
    # Wait for 30s timer + activation time
    log_info "Waiting 45s for navigation stack to fully activate..."
    sleep 45

    # Poll until planner is active (up to 50s extra)
    local planner_state
    for attempt in 1 2 3 4 5; do
        planner_state=$(ros2 lifecycle get /planner_server 2>/dev/null | grep -o 'active')
        if [ "$planner_state" = "active" ]; then
            log_ok "Nav2 ACTIVE — planner confirmed ready"
            return 0
        fi
        log_warn "Planner not active yet (attempt $attempt/5) — waiting 10s..."
        sleep 10
    done

    log_warn "Nav2 may not be fully active — check with: ros2 lifecycle get /planner_server"
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
    log_interactive "  Press 'q' + Enter → Quit everything"
    log_interactive "=========================================="
    echo ""
    
    while true; do
        # Read with timeout so we can check if children are still alive
        if read -t 5 -n 1 key 2>/dev/null; then
            case "$key" in
                r|R)
                    echo ""
                    log_interactive "Enter map name to load (blank = keep '$SELECTED_MAP'):"
                    read -r new_map
                    new_map="${new_map:-$SELECTED_MAP}"
                    log_interactive "Restarting Localization with map: $new_map"
                    launch_localization "$new_map"
                    echo ""
                    log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, q=quit"
                    ;;
                n|N)
                    echo ""
                    log_interactive "Restarting Navigation..."
                    launch_navigation
                    echo ""
                    log_interactive "Controls: r=restart loc, n=restart nav, s=status, m=check map, q=quit"
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
    # ── If the bad map came from the state file, warn and fall back ──
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
launch_node "rslidar_sdk" \
    ros2 launch rslidar_sdk humble_start.py
wait_for_node "rslidar_sdk" 5

# ---------- 9. WIT IMU ----------
launch_node "wit_imu" \
    ros2 launch wit_ros2_imu rviz_and_imu.launch.py
wait_for_node "wit_imu" 5

# ---------- 10. RealSense Camera ----------
launch_node "realsense_camera" \
    ros2 launch realsense2_camera rs_launch.py
wait_for_node "realsense_camera" 5

# ---------- 11. MAVROS ----------
echo ""
echo -e "${CYAN}--- Starting MAVROS ---${NC}"
echo ""
cd "$HOME"
launch_node "mavros" \
    ros2 run mavros mavros_node \
        --ros-args -p fcu_url:=serial://"$FCU_PORT":"$FCU_BAUD"
wait_for_node "mavros" 15
cd "$WORKSPACE_DIR"

# ============================================================
echo ""
echo -e "${CYAN}--- Starting drive bridge ---${NC}"
echo ""

# ---------- 12. Drive Bridge ----------
launch_node "drive_bridge" \
    ros2 run cpp_pubsub drive
wait_for_node "drive_bridge" 3

# ============================================================
echo ""
echo -e "${CYAN}--- Starting sensor health & peripheral nodes ---${NC}"
echo ""

# ---------- 13. Sensor Health Status ----------
launch_node "sensors_health_status" \
    ros2 run trov sensors_health_status
wait_for_node "sensors_health_status" 3

# ---------- 14. Collision Beacon ----------
launch_node "collision_beacon" \
    ros2 run trov collision_beacon
wait_for_node "collision_beacon" 3

# ---------- 15. Battery Monitor ----------
launch_node "battery_monitor" \
    ros2 run trov battery_monitor
wait_for_node "battery_monitor" 3

# ---------- 16. Floodlight ----------
launch_node "floodlight" \
    ros2 run trov floodlight.py
wait_for_node "floodlight" 3

# ---------- 17. Headlight Controller ----------
launch_node "headlight_controller" \
    ros2 run trov headlight_controller
wait_for_node "headlight_controller" 3

# ============================================================
echo ""
echo -e "${CYAN}--- Starting ROSBridge Server ---${NC}"
echo ""

# ---------- 18. ROSBridge WebSocket Server ----------
launch_node "rosbridge_server" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    call_services_in_new_thread:=true \
    send_action_goals_in_new_thread:=true \
    default_call_service_timeout:=5.0
wait_for_node "rosbridge_server" 3

# ============================================================
echo ""
echo -e "${CYAN}--- Starting MediaMTX (RTSP/WebRTC Server) ---${NC}"
echo ""

# ---------- 19. MediaMTX ----------
if [ -x "$MEDIAMTX_DIR/mediamtx" ]; then
    cd "$MEDIAMTX_DIR"
    launch_node "mediamtx" \
        ./mediamtx
    wait_for_node "mediamtx" 3
    cd "$WORKSPACE_DIR"
else
    log_warn "Skipping MediaMTX — binary not found."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting ICP Odometry ---${NC}"
echo ""

# ---------- 20. ICP Odometry ----------
wait_for_topic "/points" 30 || log_warn "LiDAR topic /points not ready"
wait_for_tf "base_link" "lidar_link" 30 || log_warn "TF base_link→lidar_link not ready"
wait_for_imu "/imu/data" 30 || log_warn "IMU topic /imu/data not ready"

log_info "Waiting 5s for sensor data to stabilize..."
sleep 5

max_attempts=3
icp_success=false

for attempt in $(seq 1 $max_attempts); do
    log_info "ICP Odometry launch attempt $attempt/$max_attempts"
    
    launch_node "icp_odometry" \
        ros2 launch trov "$ICP_LAUNCH_FILE"
    
    log_info "Waiting 15s for ICP to initialize..."
    sleep 15
    
    if kill -0 "${NODE_PIDS[icp_odometry]}" 2>/dev/null; then
        log_info "Process alive, checking for /odom topic..."
        if timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -q "pose"; then
            log_ok "ICP Odometry started successfully and publishing /odom"
            icp_success=true
            break
        fi
    fi
    
    log_warn "ICP Odometry failed on attempt $attempt"
    
    if [ $attempt -lt $max_attempts ]; then
        pkill -9 -f "icp_odometry" 2>/dev/null || true
        sleep 3
    fi
done

if [ "$icp_success" = false ]; then
    log_error "ICP Odometry failed after $max_attempts attempts"
    log_error "Continuing anyway — odometry may not work."
fi

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Localization (map_server + AMCL) ---${NC}"
echo ""

# ---------- 21. Localization ----------
# Wait for /odom before starting localization
wait_for_topic "/odom" 30 || log_warn "/odom not available — localization may have issues"

launch_localization

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Navigation (Nav2 stack) ---${NC}"
echo ""

# ---------- 22. Navigation ----------
# Wait for /map before starting navigation
wait_for_topic "/map" 30 || log_warn "/map not available — you may need to restart localization (press 'r')"

launch_navigation

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Waypoint nodes ---${NC}"
echo ""

# ---------- 23. Waypoint Follower Poses ----------
launch_node "waypoint_follower_poses" \
    ros2 run trov waypoint_follower_poses.py
wait_for_node "waypoint_follower_poses" 3

# ---------- 24. Waypoint Recorder ----------
launch_node "waypoint_recorder" \
    ros2 run trov waypoint_recorder.py
wait_for_node "waypoint_recorder" 3

# ============================================================
echo ""
echo -e "${CYAN}--- Starting Delivery Runner ---${NC}"
echo ""

# ---------- 25. Delivery Runner ----------
log_info "Verifying nav2 is active before starting delivery runner..."
nav2_ready=false
for attempt in 1 2 3 4 5; do
    state=$(ros2 lifecycle get /planner_server 2>/dev/null | grep -o 'active')
    if [ "$state" = "active" ]; then
        log_ok "Nav2 confirmed active — launching delivery runner"
        nav2_ready=true
        break
    fi
    log_warn "Nav2 not active yet (attempt $attempt/5) — waiting 10s..."
    sleep 10
done

if [ "$nav2_ready" = false ]; then
    log_error "Nav2 never became active — delivery runner will start but missions will fail until nav2 is ready"
fi

launch_node "delivery_runner" \
    ros2 run trov delivery_runner.py
wait_for_node "delivery_runner" 5

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
log_info "ROSBridge WebSocket available at ws://<robot_ip>:9090"
log_info "MediaMTX RTSP available at rtsp://<robot_ip>:8554"
log_info "MediaMTX WebRTC available at http://<robot_ip>:8889"
log_info "Headlight topic: /trov/headlight (publish true/false)"
echo ""

# ---------- Enter interactive control mode ----------
interactive_control