#!/usr/bin/env python3
"""
Indoor Waypoint Follower — Map-Scoped Edition
Package : trov
Place at : trov_ws/src/trov/scripts/indoor_waypoint_follower.py

Follows routes that were recorded by indoor_waypoint_recorder.py.
Routes are scoped to the currently loaded map: the active map name is
auto-detected from the map_server node's yaml_filename parameter at
startup. Attempting to follow a route that was recorded under a
different map (and NOT the active map) is refused loudly.

YAML structure read from indoor_waypoints.yaml:
  maps:
    adibatla_indoor_box:
      routes:
        patrol_zone_a:
          metadata: { total: 12, distance_threshold_m: 1.5, frame: map }
          waypoints:
            - { index: 0, position: {...}, orientation: {...}, yaw_deg: 45.0 }
            - ...

═══════════════════════════════════════════════════════════
  SUBSCRIBED TOPICS  (UI → node)
═══════════════════════════════════════════════════════════
  route_follower/set_route      std_msgs/String
      Set the active route name. Accepted only when IDLE or STOPPED.
      Ignored with a warning while RUNNING.

  route_follower/start          std_msgs/Empty
      Begin route from nearest waypoint.

  route_follower/stop           std_msgs/Empty
      Halt immediately and save resume position.

  route_follower/resume         std_msgs/Empty
      Continue from the saved position.

═══════════════════════════════════════════════════════════
  PUBLISHED TOPICS  (node → UI)
═══════════════════════════════════════════════════════════
  route_follower/status         std_msgs/String   LATCHED
      {
        "state":        "IDLE" | "RUNNING" | "STOPPED",
        "map":          "adibatla_indoor_box",
        "route":        "patrol_zone_a",
        "segment":      4,
        "total":        12,
        "dist_to_goal": 2.31
      }

  available_routes_follower     std_msgs/String   LATCHED
      JSON object scoped to the current map only:
      {
        "map":    "adibatla_indoor_box",
        "routes": ["patrol_zone_a", "delivery_loop"]
      }

  route_follower/full_path      nav_msgs/Path     LATCHED
  route_follower/remaining_path nav_msgs/Path     LATCHED

═══════════════════════════════════════════════════════════
  PARAMETERS
═══════════════════════════════════════════════════════════
  waypoints_file      Path to indoor_waypoints.yaml
  route_name          Initial active route (default: 'default')
  frame_id            TF map frame (default: map)
  loop                Ping-pong loop (default: false)
  map_server_node     Name of the map_server node (default: 'map_server')
  map_detect_timeout  Seconds to wait for map_server (default: 10.0)
  routes_poll_interval  Seconds between route list refresh (default: 5.0)
  start_topic         default: route_follower/start
  stop_topic          default: route_follower/stop
  resume_topic        default: route_follower/resume
  set_route_topic     default: route_follower/set_route
  status_topic        default: route_follower/status
  routes_topic        default: available_routes_follower

═══════════════════════════════════════════════════════════
  STATE MACHINE
═══════════════════════════════════════════════════════════
  IDLE    → set_route → IDLE    (route name updated)
  IDLE    → START     → RUNNING
  RUNNING → set_route → (ignored, warning logged)
  RUNNING → STOP      → STOPPED (saves remaining waypoints + target)
  STOPPED → set_route → STOPPED (route name updated for next fresh start)
  STOPPED → RESUME    → RUNNING (replans to saved target, continues)
  STOPPED → START     → RUNNING (fresh run, clears saved state)
  RUNNING → (route complete / loop=false) → IDLE
"""

import copy
import json
import math
import os
import time
import threading

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathThroughPoses, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty, String
import tf2_ros

from rcl_interfaces.srv import GetParameters
import yaml


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _map_stem_from_yaml_path(yaml_path: str) -> str:
    """
    '/full/path/to/adibatla_indoor_box.yaml'  →  'adibatla_indoor_box'
    """
    return os.path.splitext(os.path.basename(yaml_path))[0]


def quat_to_yaw_deg(oz: float, ow: float) -> float:
    return math.degrees(2.0 * math.atan2(oz, ow))


def dist2d(ax: float, ay: float, bx: float, by: float) -> float:
    return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)


def _wait_for_future(future, timeout_sec: float = 30.0) -> bool:
    """Spin-wait on a future. Returns True if it completed, False if timed out."""
    deadline = time.time() + timeout_sec
    while not future.done():
        if time.time() > deadline:
            return False
        time.sleep(0.02)
    return True


# ─────────────────────────────────────────────────────────────────────────────
# States
# ─────────────────────────────────────────────────────────────────────────────

class State:
    IDLE    = 'IDLE'
    RUNNING = 'RUNNING'
    STOPPED = 'STOPPED'


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class IndoorRouteFollower(Node):

    def __init__(self):
        super().__init__('indoor_route_follower')

        self._sub_group    = ReentrantCallbackGroup()
        self._action_group = MutuallyExclusiveCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            'waypoints_file',
            os.path.expanduser('/data/trov_ws/src/trov/routes/indoor_waypoints.yaml')
        )
        self.declare_parameter('route_name',           'default')
        self.declare_parameter('frame_id',             'map')
        self.declare_parameter('loop',                 False)
        self.declare_parameter('map_server_node',      'map_server')
        self.declare_parameter('map_detect_timeout',   10.0)
        self.declare_parameter('routes_poll_interval', 5.0)
        self.declare_parameter('start_topic',          'route_follower/start')
        self.declare_parameter('stop_topic',           'route_follower/stop')
        self.declare_parameter('resume_topic',         'route_follower/resume')
        self.declare_parameter('set_route_topic',      'route_follower/set_route')
        self.declare_parameter('status_topic',         'route_follower/status')
        self.declare_parameter('routes_topic',         'available_routes_follower')

        self.waypoints_file        = self.get_parameter('waypoints_file').value
        self.route_name            = self.get_parameter('route_name').value
        self.frame_id              = self.get_parameter('frame_id').value
        self.loop                  = self.get_parameter('loop').value
        self._map_server_node      = self.get_parameter('map_server_node').value
        self._map_detect_timeout   = self.get_parameter('map_detect_timeout').value
        self._routes_poll_interval = self.get_parameter('routes_poll_interval').value
        start_topic                = self.get_parameter('start_topic').value
        stop_topic                 = self.get_parameter('stop_topic').value
        resume_topic               = self.get_parameter('resume_topic').value
        set_route_topic            = self.get_parameter('set_route_topic').value
        status_topic               = self.get_parameter('status_topic').value
        routes_topic               = self.get_parameter('routes_topic').value

        # ── Active map — populated by _detect_active_map() ───────────────────
        self.active_map: str | None = None

        # ── Cached route list — used to detect changes and avoid noisy publishes
        self._last_published_routes: list = []

        # ── Action clients ────────────────────────────────────────────────────
        self._planner = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses',
            callback_group=self._action_group
        )
        self._controller = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=self._action_group
        )
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self._action_group
        )

        # ── TF ────────────────────────────────────────────────────────────────
        self._current_x   = 0.0
        self._current_y   = 0.0
        self._map_frame   = self.frame_id
        self._robot_frame = 'base_link'
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Sensors ───────────────────────────────────────────────────────────
        self._odom_received    = False
        self._cmd_vel_blocked  = False
        self._cmd_vel_received = False
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
            callback_group=self._sub_group
        )
        self.create_subscription(
            Twist, '/cmd_vel_safe', self._cmd_vel_cb, 10,
            callback_group=self._sub_group
        )

        # ── Publishers ────────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_full_path      = self.create_publisher(
            Path,   'route_follower/full_path',      latched)
        self._pub_remaining_path = self.create_publisher(
            Path,   'route_follower/remaining_path', latched)
        self._pub_status         = self.create_publisher(
            String, status_topic,                    latched)
        self._pub_routes         = self.create_publisher(
            String, routes_topic,                    latched)

        # ── Internal state ────────────────────────────────────────────────────
        self._state      = State.IDLE
        self._state_lock = threading.Lock()

        self._stop_requested     = False
        self._active_goal_handle = None
        self._run_thread         = None

        self._current_segment = 0
        self._total_segments  = 0
        self._dist_to_goal    = 0.0

        # Saved on STOP, consumed on RESUME
        self._resume_target     = None
        self._resume_remaining  = None
        self._resume_direction  = None
        self._resume_go_forward = True

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, set_route_topic,  self._set_route_cb, 10,
            callback_group=self._sub_group)
        self.create_subscription(
            Empty,  start_topic,      self._start_cb,     10,
            callback_group=self._sub_group)
        self.create_subscription(
            Empty,  stop_topic,       self._stop_cb,      10,
            callback_group=self._sub_group)
        self.create_subscription(
            Empty,  resume_topic,     self._resume_cb,    10,
            callback_group=self._sub_group)

        # ── Detect active map (blocking, with timeout) ────────────────────────
        self._detect_active_map()

        # ── Publish initial state to UI ───────────────────────────────────────
        self._publish_available_routes()
        self._publish_status()

        # ── Poll YAML for new routes every N seconds ──────────────────────────
        # Also retries map detection if it failed at startup (active_map is None)
        self.create_timer(
            self._routes_poll_interval,
            self._routes_poll_cb,
            callback_group=self._sub_group
        )

        self.get_logger().info(
            f'\n[IndoorRouteFollower] Ready\n'
            f'  Active map      : {self.active_map}\n'
            f'  Active route    : {self.route_name}\n'
            f'  Waypoints       : {self.waypoints_file}\n'
            f'  Loop            : {self.loop}\n'
            f'  Routes poll     : every {self._routes_poll_interval}s\n'
            f'  set_route ←     : {set_route_topic}\n'
            f'  start ←         : {start_topic}\n'
            f'  stop ←          : {stop_topic}\n'
            f'  resume ←        : {resume_topic}\n'
            f'  status →        : {status_topic}\n'
            f'  routes →        : {routes_topic}'
        )

    # =========================================================================
    # Map detection
    # =========================================================================

    def _detect_active_map(self):
        """
        Query map_server for its yaml_filename parameter.
        Retries until map_detect_timeout seconds have elapsed.
        On success: sets self.active_map.
        On failure: leaves self.active_map as None and logs a loud error.
        Route following is disabled while active_map is None.
        The poll timer (_routes_poll_cb) will keep retrying this automatically.
        """
        srv_name = f'/{self._map_server_node}/get_parameters'
        client   = self.create_client(GetParameters, srv_name)

        self.get_logger().info(
            f'[MapDetect] Contacting {srv_name} '
            f'(timeout: {self._map_detect_timeout}s)...'
        )

        deadline = time.time() + self._map_detect_timeout

        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                self.get_logger().error(
                    f'[MapDetect] ✗ map_server not available after '
                    f'{self._map_detect_timeout}s. '
                    f'Will retry every {self._routes_poll_interval}s automatically.'
                )
                return
            self.get_logger().warn(
                '[MapDetect] map_server not ready — retrying...',
                throttle_duration_sec=2.0
            )

        request       = GetParameters.Request()
        request.names = ['yaml_filename']
        future        = client.call_async(request)

        remaining = max(deadline - time.time(), 2.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)

        if not future.done():
            self.get_logger().error(
                '[MapDetect] ✗ Parameter request timed out. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'[MapDetect] ✗ Service call raised exception: {e}. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        if not response.values:
            self.get_logger().error(
                '[MapDetect] ✗ map_server returned no value for yaml_filename. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        yaml_path = response.values[0].string_value
        if not yaml_path:
            self.get_logger().error(
                '[MapDetect] ✗ yaml_filename parameter is empty on map_server. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        self.active_map = _map_stem_from_yaml_path(yaml_path)
        self.get_logger().info(
            f'[MapDetect] ✓ Active map: "{self.active_map}" '
            f'(yaml_filename: {yaml_path})'
        )

    # =========================================================================
    # Routes poll timer callback
    # =========================================================================

    def _routes_poll_cb(self):
        """
        Called every routes_poll_interval seconds.

        FIX: If active_map is still None (map detection failed at startup),
        retry _detect_active_map() here instead of silently returning.
        This means the node self-heals — no restart needed if map_server
        wasn't ready when the node launched.

        If active_map is set, re-read the YAML and republish
        available_routes_follower only if the route list has changed.
        Silent when nothing changed — no log spam.
        """
        # ── FIX: retry map detection if it failed at startup ──────────────────
        if self.active_map is None:
            self.get_logger().info(
                '[RoutesPoll] active_map not set — retrying map detection...'
            )
            self._detect_active_map()
            if self.active_map is not None:
                # Detection succeeded — publish now that we have a map
                self.get_logger().info(
                    f'[RoutesPoll] Map detection recovered: "{self.active_map}"'
                )
                self._publish_available_routes()
                self._publish_status()
            return
        # ─────────────────────────────────────────────────────────────────────

        current_routes = self._read_route_names_for_map(self.active_map)

        if current_routes != self._last_published_routes:
            self.get_logger().info(
                f'[RoutesPoll] Route list changed for map "{self.active_map}": '
                f'{self._last_published_routes} → {current_routes}. Republishing.'
            )
            self._publish_available_routes()

    # =========================================================================
    # set_route callback
    # =========================================================================

    def _set_route_cb(self, msg: String):
        new_name = msg.data.strip()
        if not new_name:
            self.get_logger().warn('set_route: received empty string — ignoring')
            return

        with self._state_lock:
            if self._state == State.RUNNING:
                self.get_logger().warn(
                    f'set_route: ignored — robot is RUNNING. '
                    f'Stop first, then set route to "{new_name}".'
                )
                return
            self.route_name = new_name

        self.get_logger().info(f'Active route set to "{self.route_name}"')
        self._publish_available_routes()
        self._publish_status()

    # =========================================================================
    # Start / Stop / Resume
    # =========================================================================

    def _start_cb(self, _msg: Empty):
        # Hard guard: no map detected
        if self.active_map is None:
            self.get_logger().error(
                '[RouteFollower] ✗ CANNOT START — active map not detected. '
                'Is map_server running? Will retry map detection automatically.'
            )
            return

        with self._state_lock:
            if self._state == State.RUNNING:
                self.get_logger().warn('Already running — ignoring start')
                return
            self._resume_target     = None
            self._resume_remaining  = None
            self._resume_direction  = None
            self._resume_go_forward = True
            self._stop_requested    = False
            self._state             = State.RUNNING

        self.get_logger().info(
            f'Route following STARTED '
            f'(map: "{self.active_map}"  route: "{self.route_name}")'
        )
        self._publish_status()
        self._run_thread = threading.Thread(
            target=self._run, args=(False,), daemon=True
        )
        self._run_thread.start()

    def _stop_cb(self, _msg: Empty):
        with self._state_lock:
            if self._state != State.RUNNING:
                self.get_logger().warn('Not running — ignoring stop')
                return
            self._stop_requested = True

        self.get_logger().info('Stop requested — robot will halt and save position')
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()

    def _resume_cb(self, _msg: Empty):
        # Hard guard: no map detected
        if self.active_map is None:
            self.get_logger().error(
                '[RouteFollower] ✗ CANNOT RESUME — active map not detected.'
            )
            return

        with self._state_lock:
            if self._state != State.STOPPED:
                self.get_logger().warn(
                    f'Cannot resume — current state is {self._state} (must be STOPPED)'
                )
                return
            if self._resume_target is None:
                self.get_logger().warn('No saved position to resume from')
                return
            self._stop_requested = False
            self._state          = State.RUNNING

        self.get_logger().info('Route following RESUMED')
        self._publish_status()
        self._run_thread = threading.Thread(
            target=self._run, args=(True,), daemon=True
        )
        self._run_thread.start()

    # =========================================================================
    # Status + routes publishers
    # =========================================================================

    def _publish_status(self, segment: int = None, total: int = None,
                        dist: float = None):
        with self._state_lock:
            state_str = self._state

        payload = {
            'state':        state_str,
            'map':          self.active_map if self.active_map else 'unknown',
            'route':        self.route_name,
            'segment':      segment if segment is not None else self._current_segment,
            'total':        total   if total   is not None else self._total_segments,
            'dist_to_goal': round(
                dist if dist is not None else self._dist_to_goal, 2
            ),
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._pub_status.publish(msg)

    def _publish_available_routes(self):
        """
        Publish ONLY the routes for the currently active map.
        The payload includes the map name so the UI can confirm which
        map the route list belongs to and discard stale messages.
        Also updates _last_published_routes so the poll timer can
        detect changes without re-publishing unnecessarily.

        Format:
          { "map": "adibatla_indoor_box", "routes": ["patrol_zone_a", ...] }

        Routes from other maps stored in the YAML are NEVER included here.
        """
        if self.active_map is None:
            payload = {'map': 'unknown', 'routes': []}
            self._last_published_routes = []
            self.get_logger().warn(
                'available_routes: publishing empty list — map not detected yet.'
            )
        else:
            routes  = self._read_route_names_for_map(self.active_map)
            payload = {'map': self.active_map, 'routes': routes}
            self._last_published_routes = routes
            self.get_logger().info(
                f'Available routes for map "{self.active_map}": {routes}'
            )

        msg      = String()
        msg.data = json.dumps(payload)
        self._pub_routes.publish(msg)

    # =========================================================================
    # Sensor callbacks
    # =========================================================================

    def _odom_cb(self, msg: Odometry):
        self._odom_received = True

    def _cmd_vel_cb(self, msg: Twist):
        was_blocked           = self._cmd_vel_blocked
        self._cmd_vel_blocked = (
            abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001
        )
        self._cmd_vel_received = True

        if not was_blocked and self._cmd_vel_blocked:
            self.get_logger().warn(
                'Robot STOPPED by collision monitor — obstacle in StopZone'
            )
        elif was_blocked and not self._cmd_vel_blocked:
            self.get_logger().info('Obstacle cleared — resuming')

    # =========================================================================
    # TF pose
    # =========================================================================

    def _update_robot_pose(self) -> bool:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            self._current_x = t.transform.translation.x
            self._current_y = t.transform.translation.y
            return True
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=2.0
            )
            return False

    # =========================================================================
    # Costmap
    # =========================================================================

    def _clear_costmaps(self):
        if self._clear_global.wait_for_service(timeout_sec=2.0):
            future = self._clear_global.call_async(ClearEntireCostmap.Request())
            _wait_for_future(future, timeout_sec=5.0)

    # =========================================================================
    # YAML helpers
    # =========================================================================

    def _load_yaml(self) -> dict:
        """Load indoor_waypoints.yaml. Returns {} if missing or unparseable."""
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(
                f'Waypoints file not found: {self.waypoints_file}'
            )
            return {}
        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                return data if isinstance(data, dict) else {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML parse error: {e}')
            return {}

    def _read_route_names_for_map(self, map_name: str) -> list:
        """
        Return ONLY the route names stored under map_name.
        Routes from other maps are NEVER included.
        Returns [] if the map has no routes or doesn't exist in the file.
        """
        data      = self._load_yaml()
        maps_data = data.get('maps', {})
        if not isinstance(maps_data, dict):
            return []
        map_entry = maps_data.get(map_name, {})
        if not isinstance(map_entry, dict):
            return []
        routes = map_entry.get('routes', {})
        if not isinstance(routes, dict):
            return []
        return list(routes.keys())

    # =========================================================================
    # Load poses — strictly map-scoped
    # =========================================================================

    def _load_poses(self) -> list:
        """
        Load waypoints for self.route_name under self.active_map ONLY.

        The route must exist under self.active_map in the YAML.
        If the route is found ONLY under a different map (and not under
        the active map), that is treated as an error and [] is returned.
        If the same route name happens to exist under multiple maps,
        only the active map's copy is used — this is correct behaviour
        for a multi-map system where different maps can share route names.
        """
        if self.active_map is None:
            self.get_logger().error(
                'Cannot load poses — active map not detected.'
            )
            return []

        data = self._load_yaml()

        if 'maps' not in data or not isinstance(data.get('maps'), dict):
            self.get_logger().error(
                f'indoor_waypoints.yaml has no "maps" key. '
                f'Record some routes first with indoor_waypoint_recorder.'
            )
            return []

        maps_data = data['maps']

        # ── Check active map exists in file ───────────────────────────────────
        if self.active_map not in maps_data:
            available_maps = list(maps_data.keys())
            self.get_logger().error(
                f'No routes recorded for active map "{self.active_map}". '
                f'Maps with routes: {available_maps}. '
                f'Record routes for this map first.'
            )
            return []

        map_entry = maps_data[self.active_map]
        if not isinstance(map_entry, dict):
            self.get_logger().error(
                f'Malformed entry for map "{self.active_map}" in YAML.'
            )
            return []

        routes = map_entry.get('routes', {})
        if not isinstance(routes, dict):
            self.get_logger().error(
                f'No routes dict found under map "{self.active_map}".'
            )
            return []

        # ── Check requested route exists under active map ─────────────────────
        if self.route_name not in routes:
            available_for_this_map = list(routes.keys())

            # Inform the user if the route exists under OTHER maps, so they
            # know it's a map-mismatch issue rather than a typo.
            other_maps_with_route = [
                m for m, entry in maps_data.items()
                if m != self.active_map
                and isinstance(entry, dict)
                and isinstance(entry.get('routes'), dict)
                and self.route_name in entry['routes']
            ]
            if other_maps_with_route:
                self.get_logger().error(
                    f'Route "{self.route_name}" is NOT recorded for the active '
                    f'map "{self.active_map}". '
                    f'It exists under map(s): {other_maps_with_route}. '
                    f'Switch to one of those maps, or record this route again '
                    f'under "{self.active_map}". '
                    f'Routes available for "{self.active_map}": '
                    f'{available_for_this_map}'
                )
            else:
                self.get_logger().error(
                    f'Route "{self.route_name}" not found under map '
                    f'"{self.active_map}". '
                    f'Available routes for this map: {available_for_this_map}'
                )
            return []

        route_entry   = routes[self.route_name]
        waypoint_list = route_entry.get('waypoints', [])

        if not waypoint_list:
            self.get_logger().error(
                f'Route "{self.route_name}" under map "{self.active_map}" '
                f'has no waypoints.'
            )
            return []

        self.get_logger().info(
            f'Loading route "{self.route_name}" under map "{self.active_map}" '
            f'({len(waypoint_list)} waypoints)'
        )

        poses = []
        for wp in waypoint_list:
            # Skip waypoints at the exact origin (likely invalid)
            if wp['position']['x'] == 0.0 and wp['position']['y'] == 0.0:
                self.get_logger().warn(
                    f'Skipping waypoint at origin (index {wp.get("index", "?")})'
                )
                continue

            ps = PoseStamped()
            ps.header.frame_id    = self.frame_id
            ps.header.stamp       = self.get_clock().now().to_msg()
            ps.pose.position.x    = float(wp['position']['x'])
            ps.pose.position.y    = float(wp['position']['y'])
            ps.pose.position.z    = float(wp['position']['z'])
            ps.pose.orientation.x = float(wp['orientation']['x'])
            ps.pose.orientation.y = float(wp['orientation']['y'])
            ps.pose.orientation.z = float(wp['orientation']['z'])
            ps.pose.orientation.w = float(wp['orientation']['w'])
            poses.append(ps)

        self.get_logger().info(
            f'Loaded {len(poses)} valid poses from route "{self.route_name}" '
            f'(map: "{self.active_map}")'
        )
        return poses

    # =========================================================================
    # Build reverse leg
    # =========================================================================

    def _build_reverse_leg(self, forward_poses: list) -> list:
        reverse_poses = []
        for pose in reversed(forward_poses[:-1]):
            rp                   = copy.deepcopy(pose)
            oz, ow               = rp.pose.orientation.z, rp.pose.orientation.w
            rp.pose.orientation.x =  0.0
            rp.pose.orientation.y =  0.0
            rp.pose.orientation.z =  ow
            rp.pose.orientation.w = -oz
            reverse_poses.append(rp)
        return reverse_poses

    # =========================================================================
    # Nearest waypoint
    # =========================================================================

    def _find_nearest_waypoint(self, poses: list) -> int:
        self._update_robot_pose()
        min_dist, nearest_idx = float('inf'), 0
        for i, pose in enumerate(poses):
            d = dist2d(
                self._current_x, self._current_y,
                pose.pose.position.x, pose.pose.position.y
            )
            if d < min_dist:
                min_dist, nearest_idx = d, i
        self.get_logger().info(
            f'Starting from nearest waypoint {nearest_idx + 1} '
            f'({min_dist:.2f} m away)'
        )
        return nearest_idx

    def _find_nearest_remaining_waypoint(self, poses: list) -> int:
        min_dist, nearest_idx = float('inf'), 0
        for i, pose in enumerate(poses):
            d = dist2d(
                self._current_x, self._current_y,
                pose.pose.position.x, pose.pose.position.y
            )
            if d < min_dist:
                min_dist, nearest_idx = d, i
        self.get_logger().info(
            f'Nearest remaining waypoint for resume: index {nearest_idx} '
            f'({min_dist:.2f} m away)'
        )
        return nearest_idx

    # =========================================================================
    # Visualisation
    # =========================================================================

    def _publish_full_path(self, poses: list):
        msg             = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.poses           = list(poses)
        self._pub_full_path.publish(msg)
        self._pub_remaining_path.publish(msg)

    def _publish_remaining_path(self, remaining_poses: list):
        msg             = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.poses           = list(remaining_poses)
        self._pub_remaining_path.publish(msg)

    # =========================================================================
    # Obstacle wait
    # =========================================================================

    def _wait_for_obstacle_clear(self):
        self.get_logger().warn(
            'Holding position — waiting for obstacle to clear...'
        )
        while self._cmd_vel_blocked and not self._stop_requested and rclpy.ok():
            time.sleep(0.5)

    # =========================================================================
    # Plan entire leg — single batched ComputePathThroughPoses call
    # =========================================================================

    def _plan_leg(self, goal_poses: list, leg_label: str):
        """
        Send all waypoints in a single ComputePathThroughPoses request.
        Returns the planned nav_msgs/Path or None on failure.
        """
        stamped_goals = []
        now           = self.get_clock().now().to_msg()
        for ps in goal_poses:
            sp              = copy.deepcopy(ps)
            sp.header.stamp = now
            stamped_goals.append(sp)

        self._planner.wait_for_server()

        goal       = ComputePathThroughPoses.Goal()
        goal.goals = stamped_goals

        future = self._planner.send_goal_async(goal)
        if not _wait_for_future(future, timeout_sec=30.0):
            self.get_logger().error(f'[{leg_label}] Planner timed out on send')
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{leg_label}] Planner rejected goal')
            return None

        result_future = goal_handle.get_result_async()
        if not _wait_for_future(result_future, timeout_sec=120.0):
            self.get_logger().error(f'[{leg_label}] Plan result timed out')
            return None

        result = result_future.result().result
        if not result.path.poses:
            self.get_logger().error(f'[{leg_label}] Planner returned empty path')
            return None

        self.get_logger().info(
            f'[{leg_label}] Planned path with {len(result.path.poses)} poses '
            f'through {len(stamped_goals)} waypoints'
        )
        return result.path

    # =========================================================================
    # Follow one planned path
    # =========================================================================

    def _follow_path(self, path: Path, leg_label: str) -> bool:
        self._controller.wait_for_server()

        goal                 = FollowPath.Goal()
        goal.path            = path
        goal.controller_id   = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'

        future = self._controller.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        if not _wait_for_future(future, timeout_sec=30.0):
            self.get_logger().error(f'[{leg_label}] FollowPath timed out on send')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{leg_label}] FollowPath rejected')
            return False

        self._active_goal_handle = goal_handle
        result_future            = goal_handle.get_result_async()

        if not _wait_for_future(result_future, timeout_sec=600.0):
            self.get_logger().error(f'[{leg_label}] FollowPath timed out')
            self._active_goal_handle = None
            return False

        self._active_goal_handle = None

        if self._stop_requested:
            return False

        status = result_future.result().status
        self._update_robot_pose()

        if status == GoalStatus.STATUS_SUCCEEDED:
            return True

        if self._cmd_vel_blocked:
            self._wait_for_obstacle_clear()
            return False

        self.get_logger().warn(
            f'[{leg_label}] Navigation failed (status {status})'
        )
        return False

    # =========================================================================
    # Feedback
    # =========================================================================

    def _feedback_cb(self, feedback_msg):
        dist               = feedback_msg.feedback.distance_to_goal
        self._dist_to_goal = dist
        self.get_logger().info(
            f'Distance to goal: {dist:.2f} m', throttle_duration_sec=5.0
        )
        self._publish_status()

    # =========================================================================
    # Execute one leg
    # =========================================================================

    def _execute_leg(self, ordered_poses: list, direction: str,
                     go_forward: bool) -> bool:
        """
        Plan the entire ordered_poses list in a single ComputePathThroughPoses
        call, then execute with FollowPath.

        On STOP:  saves resume state at nearest remaining waypoint.
        On failure: replans once from current robot pose, retries follow.
        """
        n = len(ordered_poses)
        self._total_segments  = n
        self._current_segment = 0

        self.get_logger().info(
            f'{direction}: planning batched path through {n} waypoints '
            f'(map: "{self.active_map}"  route: "{self.route_name}")'
        )
        self._publish_status(segment=0, total=n)
        self._publish_full_path(ordered_poses)

        # STOP check before planning
        if self._stop_requested:
            self._save_resume_state(
                ordered_poses[0], ordered_poses[1:], direction, go_forward, 0, n
            )
            return False

        self._update_robot_pose()
        self._clear_costmaps()

        leg_label = f'{direction} (batched {n} wp)'

        # ── Plan ──────────────────────────────────────────────────────────────
        path = self._plan_leg(ordered_poses, leg_label)
        if path is None:
            self.get_logger().warn(
                f'[{leg_label}] Plan failed — retrying in 2 s...'
            )
            time.sleep(2.0)
            self._update_robot_pose()
            self._clear_costmaps()
            path = self._plan_leg(ordered_poses, leg_label)
            if path is None:
                self.get_logger().error(
                    f'[{leg_label}] Retry plan failed — aborting leg'
                )
                return False

        self._current_segment = 1
        self._publish_status(segment=1, total=n)

        # ── Follow ────────────────────────────────────────────────────────────
        success = self._follow_path(path, leg_label)

        # STOP triggered during follow
        if self._stop_requested:
            self._update_robot_pose()
            nearest        = self._find_nearest_remaining_waypoint(ordered_poses)
            remaining_after = ordered_poses[nearest + 1:]
            self._save_resume_state(
                ordered_poses[nearest], remaining_after,
                direction, go_forward,
                nearest + 1, n
            )
            return False

        if not success:
            self.get_logger().info(
                f'[{leg_label}] Follow failed — replanning from current position...'
            )
            time.sleep(1.0)
            self._update_robot_pose()
            self._clear_costmaps()
            retry_path = self._plan_leg(ordered_poses, leg_label + ' retry')
            if retry_path is None:
                self.get_logger().error(
                    f'[{leg_label}] Retry plan failed — aborting'
                )
                return False
            success = self._follow_path(retry_path, leg_label + ' retry')
            if not success:
                self.get_logger().error(
                    f'[{leg_label}] Retry follow failed — aborting'
                )
                return False

        self._publish_remaining_path([])
        self._current_segment = n
        self._publish_status(segment=n, total=n, dist=0.0)
        self.get_logger().info(
            f'{direction} leg complete — all {n} waypoints reached'
        )
        return True

    # =========================================================================
    # Resume state save / restore
    # =========================================================================

    def _save_resume_state(self, target: PoseStamped, remaining: list,
                           direction: str, go_forward: bool,
                           seg_num: int, total: int):
        self._resume_target     = target
        self._resume_remaining  = remaining
        self._resume_direction  = direction
        self._resume_go_forward = go_forward
        self.get_logger().info(
            f'Saved resume state at waypoint {seg_num}/{total} — '
            f'target: ({target.pose.position.x:.2f}, {target.pose.position.y:.2f})  '
            f'remaining after target: {len(remaining)} waypoints'
        )
        with self._state_lock:
            self._state = State.STOPPED
        self._publish_status(segment=seg_num, total=total)

    # =========================================================================
    # Main run entry point
    # =========================================================================

    def _run(self, is_resume: bool):
        if is_resume:
            self._run_resume()
        else:
            self._run_fresh()

        with self._state_lock:
            if self._state == State.RUNNING:
                self._state = State.IDLE
        self._publish_status(segment=0, total=0, dist=0.0)
        self.get_logger().info('IndoorRouteFollower stopped')

    # ── Fresh start ───────────────────────────────────────────────────────────

    def _run_fresh(self):
        base_poses = self._load_poses()
        if len(base_poses) < 2:
            self.get_logger().error(
                'Need at least 2 valid waypoints — aborting. '
                'Check map name, route name, and that indoor_waypoints.yaml exists.'
            )
            with self._state_lock:
                self._state = State.IDLE
            return

        time.sleep(2.0)

        if not self._cmd_vel_received:
            self.get_logger().warn(
                'No cmd_vel_safe received — is collision monitor running?'
            )

        self._update_robot_pose()

        go_forward = True
        start_idx  = self._find_nearest_waypoint(base_poses)
        ordered    = base_poses[start_idx:] + base_poses[:start_idx]

        success = self._execute_leg(ordered, 'FORWARD', go_forward)

        if self._stop_requested:
            self.get_logger().info('Route STOPPED — publish resume to continue')
            return

        if not self.loop:
            self.get_logger().info('Route complete')
            return

        if success:
            self._loop(base_poses, not go_forward)
        else:
            self.get_logger().warn('Leg failed — stopping')

    # ── Resume ────────────────────────────────────────────────────────────────

    def _run_resume(self):
        target     = self._resume_target
        remaining  = list(self._resume_remaining)
        direction  = self._resume_direction
        go_forward = self._resume_go_forward

        self._resume_target    = None
        self._resume_remaining = None
        self._resume_direction = None

        self.get_logger().info(
            f'Resuming toward '
            f'({target.pose.position.x:.2f}, {target.pose.position.y:.2f})  '
            f'then {len(remaining)} more waypoints'
        )

        resumed_poses = [target] + remaining
        success       = self._execute_leg(resumed_poses, direction, go_forward)

        if self._stop_requested:
            self.get_logger().info('Route STOPPED — publish resume to continue')
            return

        if not self.loop:
            self.get_logger().info('Route complete')
            return

        if success:
            base_poses = self._load_poses()
            if len(base_poses) >= 2:
                self._loop(base_poses, not go_forward)
        else:
            self.get_logger().warn('Resumed leg failed — stopping')

    # ── Loop ──────────────────────────────────────────────────────────────────

    def _loop(self, base_poses: list, go_forward: bool):
        while rclpy.ok() and not self._stop_requested:
            direction = 'FORWARD' if go_forward else 'REVERSE'
            ordered   = (
                list(base_poses) if go_forward
                else self._build_reverse_leg(base_poses)
            )

            success = self._execute_leg(ordered, direction, go_forward)

            if self._stop_requested:
                self.get_logger().info('Route STOPPED — publish resume to continue')
                return

            if success:
                go_forward = not go_forward
            else:
                self.get_logger().warn(f'Leg incomplete — retrying {direction}')


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node     = IndoorRouteFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()