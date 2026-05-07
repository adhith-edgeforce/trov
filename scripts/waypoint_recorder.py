#!/usr/bin/env python3
"""
Indoor Waypoint Recorder — Map-Scoped Edition
Package : trov
Place at : trov_ws/src/trov/scripts/indoor_waypoint_recorder.py

Records waypoints in the MAP frame via TF (drive mode), or generates
interpolated waypoints from map-click coordinates (map_select mode).
Routes are stored under the currently active map name, auto-detected
from the map_server node's yaml_filename parameter.

YAML structure written to indoor_waypoints.yaml:
  maps:
    adibatla_indoor_box:
      routes:
        patrol_zone_a:
          metadata: { total: 12, distance_threshold_m: 1.5, frame: map }
          waypoints:
            - { index: 0, position: {...}, orientation: {...}, yaw_deg: 45.0 }
            - ...
    jetson_trov:
      routes:
        ...

═══════════════════════════════════════════════════════════
  MODES
═══════════════════════════════════════════════════════════
  IDLE          → Robot not recording, not collecting points
  DRIVE         → Recording waypoints as robot physically moves
  MAP_SELECT    → Collecting map-click points for interpolation
  State machine:
    IDLE  → start              → DRIVE
    DRIVE → stop               → IDLE
    IDLE  → start_map_select   → MAP_SELECT
    MAP_SELECT → add_point     → MAP_SELECT  (accumulate clicks)
    MAP_SELECT → generate_interpolated → IDLE  (save route)
    MAP_SELECT → cancel_map_select     → IDLE  (discard)

═══════════════════════════════════════════════════════════
  SUBSCRIBED TOPICS  (UI → node)
═══════════════════════════════════════════════════════════
  waypoint_recorder/set_route             std_msgs/String
      Set the route name. Ignored while DRIVE or MAP_SELECT is active.
  waypoint_recorder/start                 std_msgs/Empty
      Enter DRIVE mode. Begin recording waypoints as robot moves.
  waypoint_recorder/stop                  std_msgs/Empty
      Stop DRIVE recording and save to YAML.
  waypoint_recorder/clear_routes          std_msgs/Empty
      Delete ALL routes for the CURRENT MAP ONLY. Ignored unless IDLE.
  waypoint_recorder/start_map_select      std_msgs/Empty
      Enter MAP_SELECT mode. UI should now show point picker.
  waypoint_recorder/add_point             geometry_msgs/Point
      Add a map-frame x,y coordinate to the pending point list.
      Only accepted in MAP_SELECT mode.
      z is ignored (set to 0.0 internally).
  waypoint_recorder/generate_interpolated std_msgs/Empty
      Interpolate waypoints between pending points and save the route.
      Only accepted in MAP_SELECT mode with >= 1 pending point.
      If only 1 point, robot's current TF position is used as the start.
  waypoint_recorder/cancel_map_select     std_msgs/Empty
      Discard all pending points and return to IDLE.

═══════════════════════════════════════════════════════════
  PUBLISHED TOPICS  (node → UI)
═══════════════════════════════════════════════════════════
  waypoint_recorder/status          std_msgs/String   LATCHED
      {
        "mode":           "idle" | "drive" | "map_select",
        "recording":      true | false,
        "map":            "adibatla_indoor_box",
        "route":          "patrol_zone_a",
        "count":          7,
        "pending_points": 2
      }
  available_routes                  std_msgs/String   LATCHED
      JSON object scoped to the current map:
      {
        "map":    "adibatla_indoor_box",
        "routes": ["patrol_zone_a", "delivery_loop"]
      }

═══════════════════════════════════════════════════════════
  PARAMETERS
═══════════════════════════════════════════════════════════
  route_name              Initial route name (default: 'default')
  distance_threshold      Min metres between waypoints (default: 1.5)
  min_waypoints           Min waypoints required to save in drive mode (default: 2)
  map_frame               TF map frame (default: 'map')
  robot_frame             TF robot frame (default: 'base_link')
  map_server_node         Name of the map_server node (default: 'map_server')
  map_detect_timeout      Seconds to wait for map_server (default: 10.0)
  map_retry_interval      Seconds between map detection retries if startup
                          failed (default: 5.0)
  output_file             Path to indoor_waypoints.yaml
"""

import json
import math
import os
import time

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
import yaml

from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Point


# ─────────────────────────────────────────────────────────────────────────────
# Mode constants
# ─────────────────────────────────────────────────────────────────────────────

class Mode:
    IDLE       = 'idle'
    DRIVE      = 'drive'
    MAP_SELECT = 'map_select'


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _map_stem_from_yaml_path(yaml_path: str) -> str:
    """
    '/full/path/to/adibatla_indoor_box.yaml'  →  'adibatla_indoor_box'
    """
    return os.path.splitext(os.path.basename(yaml_path))[0]


def _yaw_between(x0: float, y0: float, x1: float, y1: float) -> float:
    """Return yaw angle (radians) pointing from (x0,y0) toward (x1,y1)."""
    return math.atan2(y1 - y0, x1 - x0)


def _yaw_to_quat(yaw: float) -> dict:
    """Convert a yaw angle (radians) to a quaternion dict (x,y,z,w)."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': round(math.sin(yaw / 2.0), 6),
        'w': round(math.cos(yaw / 2.0), 6),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class IndoorWaypointRecorder(Node):

    def __init__(self):
        super().__init__('indoor_waypoint_recorder')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('route_name',          'default')
        self.declare_parameter('distance_threshold',  1.5)
        self.declare_parameter('min_waypoints',       2)
        self.declare_parameter('map_frame',           'map')
        self.declare_parameter('robot_frame',         'base_link')
        self.declare_parameter('map_server_node',     'map_server')
        self.declare_parameter('map_detect_timeout',  10.0)
        self.declare_parameter('map_retry_interval',  5.0)   # FIX: retry period
        self.declare_parameter('start_topic',         'waypoint_recorder/start')
        self.declare_parameter('stop_topic',          'waypoint_recorder/stop')
        self.declare_parameter('set_route_topic',     'waypoint_recorder/set_route')
        self.declare_parameter('clear_routes_topic',  'waypoint_recorder/clear_routes')
        self.declare_parameter('status_topic',        'waypoint_recorder/status')
        self.declare_parameter('routes_topic',        'available_routes_recorder')
        self.declare_parameter('start_map_select_topic',      'waypoint_recorder/start_map_select')
        self.declare_parameter('add_point_topic',             'waypoint_recorder/add_point')
        self.declare_parameter('generate_interpolated_topic', 'waypoint_recorder/generate_interpolated')
        self.declare_parameter('cancel_map_select_topic',     'waypoint_recorder/cancel_map_select')
        self.declare_parameter(
            'output_file',
            os.path.expanduser('/data/trov_ws/src/trov/routes/indoor_waypoints.yaml')
        )

        self.route_name           = self.get_parameter('route_name').value
        self.dist_threshold       = self.get_parameter('distance_threshold').value
        self.min_waypoints        = self.get_parameter('min_waypoints').value
        self.map_frame            = self.get_parameter('map_frame').value
        self.robot_frame          = self.get_parameter('robot_frame').value
        self._map_server_node     = self.get_parameter('map_server_node').value
        self._map_detect_timeout  = self.get_parameter('map_detect_timeout').value
        self._map_retry_interval  = self.get_parameter('map_retry_interval').value
        self.output_file          = self.get_parameter('output_file').value

        start_topic             = self.get_parameter('start_topic').value
        stop_topic              = self.get_parameter('stop_topic').value
        set_route_topic         = self.get_parameter('set_route_topic').value
        clear_routes_topic      = self.get_parameter('clear_routes_topic').value
        status_topic            = self.get_parameter('status_topic').value
        routes_topic            = self.get_parameter('routes_topic').value
        start_map_select_topic  = self.get_parameter('start_map_select_topic').value
        add_point_topic         = self.get_parameter('add_point_topic').value
        gen_interp_topic        = self.get_parameter('generate_interpolated_topic').value
        cancel_map_sel_topic    = self.get_parameter('cancel_map_select_topic').value

        # ── Active map ────────────────────────────────────────────────────────
        self.active_map: str | None = None

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Mode + drive state ────────────────────────────────────────────────
        self.mode: str            = Mode.IDLE
        self.last_x: float | None = None
        self.last_y: float | None = None
        self.waypoints: list      = []

        # ── Map-select state ──────────────────────────────────────────────────
        self.pending_points: list[tuple[float, float]] = []

        # ── Publishers ────────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status = self.create_publisher(String, status_topic, latched)
        self._pub_routes = self.create_publisher(String, routes_topic, latched)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String, set_route_topic,       self._set_route_cb,              10)
        self.create_subscription(Empty,  start_topic,           self._start_cb,                  10)
        self.create_subscription(Empty,  stop_topic,            self._stop_cb,                   10)
        self.create_subscription(Empty,  clear_routes_topic,    self._clear_routes_cb,           10)
        self.create_subscription(Empty,  start_map_select_topic,self._start_map_select_cb,       10)
        self.create_subscription(Point,  add_point_topic,       self._add_point_cb,              10)
        self.create_subscription(Empty,  gen_interp_topic,      self._generate_interpolated_cb,  10)
        self.create_subscription(Empty,  cancel_map_sel_topic,  self._cancel_map_select_cb,      10)

        # ── TF poll timer at 10 Hz (drive mode recording) ─────────────────────
        self.timer = self.create_timer(0.1, self._timer_callback)

        # ── FIX: Map detection retry timer ────────────────────────────────────
        # The original node had no mechanism to recover if map_server wasn't
        # ready when this node started. This timer retries _detect_active_map()
        # every map_retry_interval seconds until it succeeds, then cancels itself.
        self._map_retry_timer = self.create_timer(
            self._map_retry_interval, self._map_retry_cb
        )
        # ─────────────────────────────────────────────────────────────────────

        # ── Detect active map at startup ──────────────────────────────────────
        self._detect_active_map()

        # ── Publish initial state ─────────────────────────────────────────────
        self._publish_available_routes()
        self._publish_status()

        self.get_logger().info(
            f'\n[IndoorWaypointRecorder] Ready\n'
            f'  Active map   : {self.active_map}\n'
            f'  Active route : {self.route_name}\n'
            f'  Frames       : {self.robot_frame} → {self.map_frame}\n'
            f'  Gap          : {self.dist_threshold} m\n'
            f'  Output       : {self.output_file}\n'
            f'  Map retry    : every {self._map_retry_interval}s if not detected'
        )

    # =========================================================================
    # Map detection
    # =========================================================================

    def _detect_active_map(self):
        """
        Query map_server for its yaml_filename parameter.
        Retries until map_detect_timeout seconds have elapsed.
        On success: sets self.active_map and cancels the retry timer.
        On failure: leaves self.active_map as None — retry timer will keep trying.
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
                    f'Will retry every {self._map_retry_interval}s automatically.'
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
                f'Will retry every {self._map_retry_interval}s automatically.'
            )
            return

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f'[MapDetect] ✗ Service call exception: {e}. '
                f'Will retry every {self._map_retry_interval}s automatically.'
            )
            return

        if not response.values:
            self.get_logger().error(
                '[MapDetect] ✗ map_server returned no yaml_filename. '
                f'Will retry every {self._map_retry_interval}s automatically.'
            )
            return

        yaml_path = response.values[0].string_value
        if not yaml_path:
            self.get_logger().error(
                '[MapDetect] ✗ yaml_filename is empty. '
                f'Will retry every {self._map_retry_interval}s automatically.'
            )
            return

        self.active_map = _map_stem_from_yaml_path(yaml_path)
        self.get_logger().info(
            f'[MapDetect] ✓ Active map: "{self.active_map}"'
        )

    # =========================================================================
    # FIX: Map detection retry timer callback
    # =========================================================================

    def _map_retry_cb(self):
        """
        Fires every map_retry_interval seconds.
        If active_map is already set, cancels itself — job done.
        If active_map is still None, retries _detect_active_map().
        On success, publishes routes and status, then cancels itself.
        """
        if self.active_map is not None:
            # Already have the map — cancel this timer, it's no longer needed
            self._map_retry_timer.cancel()
            return

        self.get_logger().info(
            '[MapRetry] active_map not set — retrying map detection...'
        )
        self._detect_active_map()

        if self.active_map is not None:
            self.get_logger().info(
                f'[MapRetry] Map detection recovered: "{self.active_map}". '
                f'Stopping retry timer.'
            )
            self._publish_available_routes()
            self._publish_status()
            self._map_retry_timer.cancel()

    # =========================================================================
    # Subscriber callbacks — drive mode
    # =========================================================================

    def _set_route_cb(self, msg: String):
        new_name = msg.data.strip()
        if not new_name:
            self.get_logger().warn('set_route: empty string — ignoring')
            return
        if self.mode != Mode.IDLE:
            self.get_logger().warn(
                f'set_route: ignored — currently in {self.mode} mode. '
                f'Return to IDLE first.'
            )
            return
        self.route_name = new_name
        self.get_logger().info(f'Active route set to "{self.route_name}"')
        self._publish_available_routes()
        self._publish_status()

    def _start_cb(self, _msg: Empty):
        if self.active_map is None:
            self.get_logger().error(
                '[DRIVE] ✗ Cannot start — active map not detected yet. '
                'Retry is running automatically in the background.'
            )
            return
        if self.mode != Mode.IDLE:
            self.get_logger().warn(
                f'[DRIVE] Already in {self.mode} mode — ignoring start.'
            )
            return
        if self.route_name == 'default':
            self.get_logger().warn(
                '[DRIVE] Route name is still "default". '
                'Consider setting a descriptive name first.'
            )
        existing_routes = self._read_route_names_for_map(self.active_map)
        if self.route_name in existing_routes:
            self.get_logger().warn(
                f'[DRIVE] Route "{self.route_name}" already exists — '
                f'recording will OVERWRITE it.'
            )
        self.waypoints = []
        self.last_x    = None
        self.last_y    = None
        self.mode      = Mode.DRIVE
        self.get_logger().info(
            f'[DRIVE] ▶ Recording STARTED  '
            f'(map: "{self.active_map}"  route: "{self.route_name}")'
        )
        self._publish_status()

    def _stop_cb(self, _msg: Empty):
        if self.mode != Mode.DRIVE:
            self.get_logger().warn(
                f'[DRIVE] stop ignored — not in DRIVE mode (currently {self.mode}).'
            )
            return
        count = len(self.waypoints)
        if count < self.min_waypoints:
            self.get_logger().warn(
                f'[DRIVE] Only {count} waypoint(s) recorded — '
                f'minimum is {self.min_waypoints}. Nothing saved.'
            )
            self.mode      = Mode.IDLE
            self.waypoints = []
            self._publish_status()
            return
        self.mode = Mode.IDLE
        self.get_logger().info(
            f'[DRIVE] ■ Recording STOPPED. '
            f'{count} waypoints saved to '
            f'map: "{self.active_map}"  route: "{self.route_name}"'
        )
        self._publish_status()
        self._publish_available_routes()

    def _clear_routes_cb(self, _msg: Empty):
        if self.mode != Mode.IDLE:
            self.get_logger().warn(
                f'clear_routes: ignored — currently in {self.mode} mode.'
            )
            return
        if self.active_map is None:
            self.get_logger().error('clear_routes: ✗ active map not detected.')
            return
        existing  = self._load_yaml()
        maps_data = existing.get('maps', {})
        if not isinstance(maps_data, dict) or self.active_map not in maps_data:
            self.get_logger().info(
                f'clear_routes: no routes found for map "{self.active_map}".'
            )
            return
        existing['maps'][self.active_map] = {'routes': {}}
        self._write_yaml(existing)
        self.get_logger().warn(
            f'✗ All routes cleared for map "{self.active_map}".'
        )
        self._publish_available_routes()
        self._publish_status()

    # =========================================================================
    # Subscriber callbacks — map_select mode
    # =========================================================================

    def _start_map_select_cb(self, _msg: Empty):
        if self.active_map is None:
            self.get_logger().error(
                '[MAP_SELECT] ✗ Cannot start — active map not detected yet. '
                'Retry is running automatically in the background.'
            )
            return
        if self.mode != Mode.IDLE:
            self.get_logger().warn(
                f'[MAP_SELECT] Cannot enter MAP_SELECT — '
                f'currently in {self.mode} mode. Return to IDLE first.'
            )
            return
        self.pending_points = []
        self.mode           = Mode.MAP_SELECT
        self.get_logger().info(
            f'[MAP_SELECT] ▶ Entered MAP_SELECT mode  '
            f'(map: "{self.active_map}"  route: "{self.route_name}")\n'
            f'  Publish x,y points to waypoint_recorder/add_point\n'
            f'  Then publish to waypoint_recorder/generate_interpolated'
        )
        self._publish_status()

    def _add_point_cb(self, msg: Point):
        if self.mode != Mode.MAP_SELECT:
            self.get_logger().warn(
                f'[MAP_SELECT] add_point ignored — not in MAP_SELECT mode '
                f'(currently {self.mode}).'
            )
            return
        x, y = round(msg.x, 4), round(msg.y, 4)
        self.pending_points.append((x, y))
        self.get_logger().info(
            f'[MAP_SELECT] Point {len(self.pending_points)} added: '
            f'({x}, {y})  —  total pending: {len(self.pending_points)}'
        )
        self._publish_status()

    def _generate_interpolated_cb(self, _msg: Empty):
        if self.mode != Mode.MAP_SELECT:
            self.get_logger().warn(
                '[MAP_SELECT] generate_interpolated ignored — not in MAP_SELECT mode.'
            )
            return
        n_pts = len(self.pending_points)
        if n_pts == 0:
            self.get_logger().warn(
                '[MAP_SELECT] No points collected — nothing to generate. '
                'Add at least 1 point first.'
            )
            return
        anchors = self._resolve_anchors()
        if anchors is None:
            return
        waypoints = self._interpolate_anchors(anchors)
        if len(waypoints) < 2:
            self.get_logger().warn(
                f'[MAP_SELECT] Interpolation produced only {len(waypoints)} waypoint(s). '
                f'Points may be too close together (threshold: {self.dist_threshold} m). '
                f'Nothing saved.'
            )
            self.pending_points = []
            self.mode           = Mode.IDLE
            self._publish_status()
            return
        self.waypoints = waypoints
        self._save()
        self.get_logger().info(
            f'[MAP_SELECT] ✓ Generated {len(waypoints)} waypoints  '
            f'(map: "{self.active_map}"  route: "{self.route_name}")'
        )
        self.pending_points = []
        self.waypoints      = []
        self.mode           = Mode.IDLE
        self._publish_status()
        self._publish_available_routes()

    def _cancel_map_select_cb(self, _msg: Empty):
        if self.mode != Mode.MAP_SELECT:
            self.get_logger().warn(
                '[MAP_SELECT] cancel ignored — not in MAP_SELECT mode.'
            )
            return
        n = len(self.pending_points)
        self.pending_points = []
        self.mode           = Mode.IDLE
        self.get_logger().info(
            f'[MAP_SELECT] ✗ Cancelled. {n} pending point(s) discarded.'
        )
        self._publish_status()

    # =========================================================================
    # Interpolation helpers
    # =========================================================================

    def _resolve_anchors(self) -> list[tuple[float, float]] | None:
        """
        Return the ordered list of anchor (x, y) points to interpolate through.
        1 pending point  → [robot_current_pos, pending_point]
        2+ pending points → pending_points as-is
        """
        if len(self.pending_points) == 1:
            robot_pos = self._get_robot_xy()
            if robot_pos is None:
                self.get_logger().error(
                    '[MAP_SELECT] ✗ Could not look up robot position via TF. '
                    'Is localization running? Try adding a second point manually.'
                )
                return None
            rx, ry = robot_pos
            self.get_logger().info(
                f'[MAP_SELECT] 1-point mode — using robot position '
                f'({rx:.3f}, {ry:.3f}) as start'
            )
            return [(rx, ry)] + list(self.pending_points)
        else:
            return list(self.pending_points)

    def _interpolate_anchors(
        self, anchors: list[tuple[float, float]]
    ) -> list[dict]:
        """
        Walk through each consecutive pair of anchor points and interpolate
        evenly-spaced waypoints at dist_threshold spacing.
        Yaw at each waypoint faces toward the next point in the segment.
        The final waypoint inherits the yaw of the last segment.
        Returns a flat list of waypoint dicts in the same format as drive mode.
        """
        step   = self.dist_threshold
        result = []
        for seg_idx in range(len(anchors) - 1):
            x0, y0  = anchors[seg_idx]
            x1, y1  = anchors[seg_idx + 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            yaw     = _yaw_between(x0, y0, x1, y1)
            yaw_deg = round(math.degrees(yaw), 2)
            quat    = _yaw_to_quat(yaw)

            if seg_len < step:
                if seg_idx == 0:
                    result.append(self._make_wp(len(result), x0, y0, yaw_deg, quat))
                continue

            n_steps = int(seg_len / step)
            dx      = (x1 - x0) / seg_len * step
            dy      = (y1 - y0) / seg_len * step

            for i in range(n_steps):
                px = round(x0 + i * dx, 4)
                py = round(y0 + i * dy, 4)
                result.append(self._make_wp(len(result), px, py, yaw_deg, quat))

        # Always add the final anchor as the last waypoint
        if len(anchors) >= 2:
            x_last, y_last = anchors[-1]
            x_prev, y_prev = anchors[-2]
            final_yaw      = _yaw_between(x_prev, y_prev, x_last, y_last)
            final_yaw_deg  = round(math.degrees(final_yaw), 2)
            final_quat     = _yaw_to_quat(final_yaw)
            result.append(
                self._make_wp(len(result), x_last, y_last, final_yaw_deg, final_quat)
            )

        return result

    @staticmethod
    def _make_wp(index: int, x: float, y: float,
                 yaw_deg: float, quat: dict) -> dict:
        return {
            'index': index,
            'position': {
                'x': round(x, 4),
                'y': round(y, 4),
                'z': 0.0,
            },
            'orientation': {
                'x': round(quat['x'], 6),
                'y': round(quat['y'], 6),
                'z': round(quat['z'], 6),
                'w': round(quat['w'], 6),
            },
            'yaw_deg': yaw_deg,
        }

    def _get_robot_xy(self) -> tuple[float, float] | None:
        """Look up current robot position in map frame. Returns (x, y) or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[TF] lookup failed: {e}')
            return None

    # =========================================================================
    # Publishers
    # =========================================================================

    def _publish_status(self):
        payload = {
            'mode':           self.mode,
            'recording':      self.mode == Mode.DRIVE,
            'map':            self.active_map if self.active_map else 'unknown',
            'route':          self.route_name,
            'count':          len(self.waypoints),
            'pending_points': len(self.pending_points),
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._pub_status.publish(msg)

    def _publish_available_routes(self):
        if self.active_map is None:
            payload = {'map': 'unknown', 'routes': []}
            self.get_logger().warn(
                'available_routes: publishing empty list — map not detected yet.'
            )
        else:
            routes  = self._read_route_names_for_map(self.active_map)
            payload = {'map': self.active_map, 'routes': routes}
            self.get_logger().info(
                f'Available routes for map "{self.active_map}": {routes}'
            )
        msg      = String()
        msg.data = json.dumps(payload)
        self._pub_routes.publish(msg)

    # =========================================================================
    # TF poll timer — drive mode recording
    # =========================================================================

    def _timer_callback(self):
        if self.mode != Mode.DRIVE:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        q = t.transform.rotation

        if self._dist(x, y) < self.dist_threshold:
            return

        yaw = self._yaw_from_tf(t)
        wp = {
            'index': len(self.waypoints),
            'position': {
                'x': round(x, 4),
                'y': round(y, 4),
                'z': round(z, 4),
            },
            'orientation': {
                'x': round(q.x, 6),
                'y': round(q.y, 6),
                'z': round(q.z, 6),
                'w': round(q.w, 6),
            },
            'yaw_deg': round(math.degrees(yaw), 2),
        }
        self.waypoints.append(wp)
        self.last_x, self.last_y = x, y
        self.get_logger().info(
            f'[WP {wp["index"]:03d}]  '
            f'x={x:7.3f}  y={y:7.3f}  yaw={wp["yaw_deg"]:6.1f} deg  '
            f'(total: {len(self.waypoints)})  '
            f'map: "{self.active_map}"  route: "{self.route_name}"'
        )
        self._save()
        self._publish_status()

    # =========================================================================
    # YAML helpers
    # =========================================================================

    def _load_yaml(self) -> dict:
        if not os.path.exists(self.output_file):
            return {}
        try:
            with open(self.output_file, 'r') as f:
                data = yaml.safe_load(f)
                return data if isinstance(data, dict) else {}
        except yaml.YAMLError as e:
            self.get_logger().warn(
                f'Could not parse YAML ({e}) — treating as empty.'
            )
            return {}

    def _write_yaml(self, data: dict):
        os.makedirs(
            os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True
        )
        tmp = self.output_file + '.tmp'
        with open(tmp, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        os.replace(tmp, self.output_file)

    def _read_route_names_for_map(self, map_name: str) -> list:
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

    def _save(self):
        """
        Upsert self.waypoints into the YAML under active_map / route_name.
        All other maps and routes are left untouched.
        """
        existing = self._load_yaml()
        if 'maps' not in existing or not isinstance(existing['maps'], dict):
            existing['maps'] = {}
        if self.active_map not in existing['maps'] or \
                not isinstance(existing['maps'][self.active_map], dict):
            existing['maps'][self.active_map] = {'routes': {}}
        map_entry = existing['maps'][self.active_map]
        if 'routes' not in map_entry or not isinstance(map_entry['routes'], dict):
            map_entry['routes'] = {}
        map_entry['routes'][self.route_name] = {
            'metadata': {
                'total':                len(self.waypoints),
                'distance_threshold_m': self.dist_threshold,
                'frame':                self.map_frame,
            },
            'waypoints': self.waypoints,
        }
        self._write_yaml(existing)

    # =========================================================================
    # Maths helpers
    # =========================================================================

    @staticmethod
    def _yaw_from_tf(t) -> float:
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _dist(self, x: float, y: float) -> float:
        if self.last_x is None:
            return float('inf')
        return math.hypot(x - self.last_x, y - self.last_y)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = IndoorWaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'[IndoorWaypointRecorder] Stopped. '
            f'map: "{node.active_map}"  route: "{node.route_name}"  '
            f'→ {node.output_file}'
        )
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()