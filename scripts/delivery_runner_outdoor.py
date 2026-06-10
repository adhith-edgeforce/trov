#!/usr/bin/env python3
"""
delivery_runner.py
Package : trov
Place at : trov_ws/src/trov/scripts/delivery_runner.py

Standalone delivery mission executor for the T-ROV UGV.

Changes from indoor/2D version:
  - Map detection queries /lidar_localization (map_path parameter)
    instead of /map_server (yaml_filename parameter).
    Map name is derived from the PCD filename stem.
  - _routes_poll_cb now retries _detect_active_map() when active_map
    is None, giving the node the same self-healing behaviour as
    outdoor_waypoint_recorder and outdoor_waypoint_follower.
  - Default waypoints_file updated to outdoor_waypoints.yaml.
  - map_param_name parameter added so the queried parameter name
    can be overridden from the launch file without touching this file.
"""

import copy
import json
import math
import os
import subprocess
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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String
import tf2_ros

from rcl_interfaces.srv import GetParameters
import yaml

try:
    import gpiod
    from gpiod.line import Direction, Value
    GPIOD_AVAILABLE = True
except ImportError:
    GPIOD_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _map_stem_from_path(map_path: str) -> str:
    """
    '/full/path/to/adibatla_outdoor.pcd' → 'adibatla_outdoor'
    Works for any extension (.pcd, .yaml, etc.)
    """
    return os.path.splitext(os.path.basename(map_path))[0]


def dist2d(ax: float, ay: float, bx: float, by: float) -> float:
    return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)


def _wait_for_future(future, timeout_sec: float = 30.0) -> bool:
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
    IDLE      = 'IDLE'
    OUTBOUND  = 'OUTBOUND'
    ARRIVED   = 'ARRIVED'
    DEPARTING = 'DEPARTING'
    RETURNING = 'RETURNING'


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class DeliveryRunner(Node):

    def __init__(self):
        super().__init__('delivery_runner')

        self._sub_group    = ReentrantCallbackGroup()
        self._action_group = MutuallyExclusiveCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            'waypoints_file',
            os.path.expanduser('/data/trov_ws/src/trov/routes/outdoor_waypoints.yaml')
        )
        self.declare_parameter('frame_id',                  'map')
        self.declare_parameter('map_server_node',           'lidar_localization')
        self.declare_parameter('map_param_name',            'map_path')
        self.declare_parameter('map_detect_timeout',        10.0)
        self.declare_parameter('arrival_pulses',            3)
        self.declare_parameter('arrival_pulse_on_sec',      3.0)
        self.declare_parameter('arrival_pulse_off_sec',     3.0)
        self.declare_parameter('departure_beacon_sec',      5.0)
        self.declare_parameter('gpio_chip_path',            '/dev/gpiochip1')
        self.declare_parameter('gpio_line',                 9)
        self.declare_parameter('return_stabilize_sec',      8.0)
        self.declare_parameter('predeparture_delay_sec',    5.0)
        self.declare_parameter('beep_sound_path',           '/data/trov_ws/beep_cut.mp3')
        self.declare_parameter('beep_volume',               50)
        self.declare_parameter('routes_poll_interval',      5.0)

        self.waypoints_file        = self.get_parameter('waypoints_file').value
        self.frame_id              = self.get_parameter('frame_id').value
        self._map_server_node      = self.get_parameter('map_server_node').value
        self._map_param_name       = self.get_parameter('map_param_name').value
        self._map_detect_timeout   = self.get_parameter('map_detect_timeout').value
        self._arrival_pulses       = self.get_parameter('arrival_pulses').value
        self._pulse_on             = self.get_parameter('arrival_pulse_on_sec').value
        self._pulse_off            = self.get_parameter('arrival_pulse_off_sec').value
        self._departure_secs       = self.get_parameter('departure_beacon_sec').value
        self._gpio_chip_path       = self.get_parameter('gpio_chip_path').value
        self._gpio_line_num        = self.get_parameter('gpio_line').value
        self._return_stabilize     = self.get_parameter('return_stabilize_sec').value
        self._predeparture_delay   = self.get_parameter('predeparture_delay_sec').value
        self._beep_sound_path      = self.get_parameter('beep_sound_path').value
        self._beep_volume          = self.get_parameter('beep_volume').value
        self._routes_poll_interval = self.get_parameter('routes_poll_interval').value

        # ── Active map ────────────────────────────────────────────────────────
        self.active_map: str | None = None

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
        self._clear_local = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=self._action_group
        )

        # ── TF ────────────────────────────────────────────────────────────────
        self._current_x   = 0.0
        self._current_y   = 0.0
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Internal state ────────────────────────────────────────────────────
        self._state          = State.IDLE
        self._state_lock     = threading.Lock()
        self._route_name     = ''
        self._unloaded_event = threading.Event()

        self._active_goal_handle = None
        self._mission_thread     = None
        self._outbound_poses: list = []

        # ── Cached route list ─────────────────────────────────────────────────
        self._last_published_routes: list = []

        # ── GPIO16 persistent line-holder ─────────────────────────────────────
        self._gpio16_ok         = self._check_gpio16()
        self._gpio16_high_event = threading.Event()
        self._gpio16_stop_event = threading.Event()
        self._gpio16_thread     = threading.Thread(
            target=self._gpio16_holder_thread, daemon=True
        )
        self._gpio16_thread.start()

        # ── Beep loop thread ──────────────────────────────────────────────────
        self._beep_active_event = threading.Event()
        self._beep_stop_event   = threading.Event()
        self._beep_thread       = threading.Thread(
            target=self._beep_loop_thread, daemon=True
        )
        self._beep_thread.start()

        # ── Publishers ────────────────────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status    = self.create_publisher(String, '/delivery/status',           latched)
        self._pub_routes    = self.create_publisher(String, '/delivery/available_routes', latched)
        self._pub_full_path = self.create_publisher(Path,   '/delivery/full_path',        latched)
        self._pub_rem_path  = self.create_publisher(Path,   '/delivery/remaining_path',   latched)
        self._pub_pin32     = self.create_publisher(Bool,   'gpio_pin32_control',         10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, '/delivery/go',
            self._go_cb, 10,
            callback_group=self._sub_group
        )
        self.create_subscription(
            Empty, '/delivery/unloaded',
            self._unloaded_cb, 10,
            callback_group=self._sub_group
        )

        # ── Detect active map ─────────────────────────────────────────────────
        self._detect_active_map()

        # ── Publish initial state ─────────────────────────────────────────────
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
            f'\n[DeliveryRunner] Ready\n'
            f'  Active map          : {self.active_map}\n'
            f'  Waypoints file      : {self.waypoints_file}\n'
            f'  Loc node            : /{self._map_server_node} '
            f'(param: {self._map_param_name})\n'
            f'  Routes poll         : every {self._routes_poll_interval}s\n'
            f'  Pre-departure delay : {self._predeparture_delay}s\n'
            f'  Arrival pulses      : {self._arrival_pulses} × '
            f'({self._pulse_on}s ON / {self._pulse_off}s OFF)\n'
            f'  Departure beacon    : {self._departure_secs}s\n'
            f'  Return stabilize    : {self._return_stabilize}s\n'
            f'  GPIO chip/line      : {self._gpio_chip_path} / {self._gpio_line_num} '
            f'({"OK" if self._gpio16_ok else "DISABLED"})\n'
            f'  Beep sound          : {self._beep_sound_path}  vol {self._beep_volume}%\n'
        )

    # =========================================================================
    # GPIO16 — hardware check
    # =========================================================================

    def _check_gpio16(self) -> bool:
        if not GPIOD_AVAILABLE:
            self.get_logger().warn('[GPIO] gpiod not importable — pin 16 DISABLED.')
            return False
        if not os.path.exists(self._gpio_chip_path):
            self.get_logger().warn(
                f'[GPIO] {self._gpio_chip_path} not found — pin 16 DISABLED.'
            )
            return False
        try:
            with gpiod.request_lines(
                self._gpio_chip_path,
                consumer='delivery_runner_check',
                config={
                    self._gpio_line_num: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE
                    )
                }
            ):
                pass
            self.get_logger().info(
                f'[GPIO] pin16 check OK — '
                f'{self._gpio_chip_path} line {self._gpio_line_num}'
            )
            return True
        except Exception as e:
            self.get_logger().error(
                f'[GPIO] pin16 check failed: {e} — pin 16 DISABLED.'
            )
            return False

    # =========================================================================
    # GPIO16 — persistent line-holder thread
    # =========================================================================

    def _gpio16_holder_thread(self):
        if not self._gpio16_ok:
            self._gpio16_stop_event.wait()
            return

        try:
            with gpiod.request_lines(
                self._gpio_chip_path,
                consumer='delivery_runner_gpio16',
                config={
                    self._gpio_line_num: gpiod.LineSettings(
                        direction=Direction.OUTPUT,
                        output_value=Value.INACTIVE
                    )
                }
            ) as req:
                self.get_logger().info('[GPIO16-thread] Line acquired — holder running')
                current_high = False

                while not self._gpio16_stop_event.is_set():
                    want_high = self._gpio16_high_event.is_set()

                    if want_high != current_high:
                        req.set_value(
                            self._gpio_line_num,
                            Value.ACTIVE if want_high else Value.INACTIVE
                        )
                        current_high = want_high
                        self.get_logger().info(
                            f'[GPIO16-thread] pin16 → '
                            f'{"HIGH ▲" if want_high else "LOW  ▼"}'
                        )

                    self._gpio16_high_event.wait(timeout=0.05)

                req.set_value(self._gpio_line_num, Value.INACTIVE)
                self.get_logger().info('[GPIO16-thread] Exiting — pin16 → LOW')

        except Exception as e:
            self.get_logger().error(f'[GPIO16-thread] Fatal: {e}')

    def _gpio16_set(self, high: bool):
        if high:
            self._gpio16_high_event.set()
        else:
            self._gpio16_high_event.clear()

    # =========================================================================
    # GPIO16 — departure beacon
    # =========================================================================

    def _departure_beacon(self):
        self.get_logger().info(
            f'[GPIO] pin16 → HIGH ▲  (holding for {self._departure_secs}s)'
        )
        self._gpio16_set(True)
        time.sleep(self._departure_secs)
        self._gpio16_set(False)
        self.get_logger().info('[GPIO] pin16 → LOW  ▼  (departure beacon done)')

    # =========================================================================
    # GPIO32
    # =========================================================================

    def _gpio32_set(self, value: bool):
        try:
            msg      = Bool()
            msg.data = value
            self._pub_pin32.publish(msg)
            self.get_logger().info(
                f'[GPIO] pin32 → {"HIGH ▲" if value else "LOW  ▼"}'
            )
        except Exception:
            pass

    # =========================================================================
    # Beep loop thread
    # =========================================================================

    def _beep_loop_thread(self):
        vol_fraction = max(0.0, min(1.0, self._beep_volume / 100.0))
        self.get_logger().info('[Beep-thread] Ready')

        while not self._beep_stop_event.is_set():
            if not self._beep_active_event.wait(timeout=0.1):
                continue

            if self._beep_stop_event.is_set():
                break

            if not os.path.exists(self._beep_sound_path):
                self.get_logger().warn(
                    f'[Beep-thread] Sound file not found: {self._beep_sound_path} — '
                    f'waiting for active event to clear'
                )
                self._beep_active_event.wait(timeout=1.0)
                continue

            self.get_logger().info('[Beep-thread] Starting beep loop')

            while self._beep_active_event.is_set() and not self._beep_stop_event.is_set():
                try:
                    cmd  = ['play', '-q', self._beep_sound_path, 'vol', str(vol_fraction)]
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    while proc.poll() is None:
                        if not self._beep_active_event.is_set() or \
                                self._beep_stop_event.is_set():
                            proc.terminate()
                            try:
                                proc.wait(timeout=1.0)
                            except subprocess.TimeoutExpired:
                                proc.kill()
                            break
                        time.sleep(0.05)
                except Exception as e:
                    self.get_logger().error(f'[Beep-thread] play error: {e}')
                    time.sleep(0.5)

            self.get_logger().info('[Beep-thread] Beep loop stopped')

        self.get_logger().info('[Beep-thread] Exiting')

    def _beep_set(self, active: bool):
        if active:
            self._beep_active_event.set()
        else:
            self._beep_active_event.clear()

    # =========================================================================
    # Convenience: set beacon light + beep together
    # =========================================================================

    def _mission_signal(self, active: bool):
        self._gpio16_set(active)
        self._beep_set(active)

    # =========================================================================
    # Map detection
    # =========================================================================

    def _detect_active_map(self):
        """
        Query lidar_localization for its map_path parameter.
        Retries until map_detect_timeout seconds have elapsed.
        On success: sets self.active_map.
        On failure: leaves self.active_map as None — _routes_poll_cb will
        keep retrying automatically so no manual restart is needed.

        Map name is derived from the PCD filename stem:
          '/path/to/adibatla_outdoor.pcd' → 'adibatla_outdoor'
        """
        srv_name = f'/{self._map_server_node}/get_parameters'
        client   = self.create_client(GetParameters, srv_name)
        self.get_logger().info(
            f'[MapDetect] Contacting {srv_name} '
            f'(param: "{self._map_param_name}", '
            f'timeout: {self._map_detect_timeout}s)...'
        )
        deadline = time.time() + self._map_detect_timeout

        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                self.get_logger().error(
                    f'[MapDetect] ✗ /{self._map_server_node} not available after '
                    f'{self._map_detect_timeout}s. '
                    f'Will retry every {self._routes_poll_interval}s automatically.'
                )
                return
            self.get_logger().warn(
                f'[MapDetect] /{self._map_server_node} not ready — retrying...',
                throttle_duration_sec=2.0
            )

        request       = GetParameters.Request()
        request.names = [self._map_param_name]
        future        = client.call_async(request)
        remaining     = max(deadline - time.time(), 2.0)
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
                f'[MapDetect] ✗ Exception: {e}. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        if not response.values or not response.values[0].string_value:
            self.get_logger().error(
                f'[MapDetect] ✗ "{self._map_param_name}" empty or missing on '
                f'/{self._map_server_node}. '
                f'Will retry every {self._routes_poll_interval}s automatically.'
            )
            return

        map_path        = response.values[0].string_value
        self.active_map = _map_stem_from_path(map_path)
        self.get_logger().info(
            f'[MapDetect] ✓ Active map: "{self.active_map}" '
            f'(from {self._map_param_name}: {map_path})'
        )

    # =========================================================================
    # Routes poll timer callback
    # =========================================================================

    def _routes_poll_cb(self):
        """
        Called every routes_poll_interval seconds.

        If active_map is None (map detection failed at startup), retries
        _detect_active_map() so the node self-heals without a restart.

        If active_map is set, re-reads the YAML and republishes
        /delivery/available_routes only if the route list has changed.
        Silent when nothing changed — no log spam.
        """
        if self.active_map is None:
            self.get_logger().info(
                '[RoutesPoll] active_map not set — retrying map detection...'
            )
            self._detect_active_map()
            if self.active_map is not None:
                self.get_logger().info(
                    f'[RoutesPoll] Map detection recovered: "{self.active_map}"'
                )
                self._publish_available_routes()
                self._publish_status()
            return

        current_routes = self._read_route_names_for_map(self.active_map)

        if current_routes != self._last_published_routes:
            self.get_logger().info(
                f'[RoutesPoll] Route list changed for map "{self.active_map}": '
                f'{self._last_published_routes} → {current_routes}. Republishing.'
            )
            self._publish_available_routes()

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _go_cb(self, msg: String):
        route = msg.data.strip()
        if not route:
            self.get_logger().warn('[/delivery/go] Empty route name — ignoring')
            return
        with self._state_lock:
            if self._state != State.IDLE:
                self.get_logger().warn(
                    f'[/delivery/go] Mission already running '
                    f'(state: {self._state}) — ignoring "{route}"'
                )
                return
            if self.active_map is None:
                self.get_logger().error(
                    '[/delivery/go] Active map not detected — cannot start mission'
                )
                return
            self._route_name = route
            self._state      = State.OUTBOUND

        self.get_logger().info(
            f'[DeliveryRunner] Mission START → '
            f'route: "{route}"  map: "{self.active_map}"'
        )

        self._mission_signal(True)
        self.get_logger().info('[Mission] Beacon ON + Beep START (outbound leg)')

        self._publish_status()

        self._mission_thread = threading.Thread(
            target=self._run_mission, daemon=True
        )
        self._mission_thread.start()

    def _unloaded_cb(self, _msg: Empty):
        with self._state_lock:
            if self._state != State.ARRIVED:
                self.get_logger().warn(
                    f'[/delivery/unloaded] Ignored — '
                    f'state is {self._state} (must be ARRIVED)'
                )
                return
        self.get_logger().info('[/delivery/unloaded] ✓ Unloaded signal received')

        self._mission_signal(True)
        self.get_logger().info('[Mission] Beacon ON + Beep START (return leg)')

        self._unloaded_event.set()

    # =========================================================================
    # Mission thread
    # =========================================================================

    def _run_mission(self):
        try:
            self._mission_body()
        except Exception as e:
            self.get_logger().error(f'[Mission] Unhandled exception: {e}')
        finally:
            self._mission_signal(False)
            self._gpio32_set(False)
            with self._state_lock:
                self._state = State.IDLE
            self._publish_status()
            self.get_logger().info(
                '[DeliveryRunner] Mission ended — IDLE, Beacon OFF, Beep STOP'
            )

    def _mission_body(self):

        # ── 0. Pre-departure delay ────────────────────────────────────────────
        self.get_logger().info(
            f'[Mission] Pre-departure delay — waiting {self._predeparture_delay}s '
            f'before outbound navigation (beacon + beep already active)'
        )
        time.sleep(self._predeparture_delay)

        # ── 1. Load outbound waypoints ────────────────────────────────────────
        poses = self._load_poses(self._route_name)
        if len(poses) < 2:
            self.get_logger().error(
                f'[Mission] Route "{self._route_name}" has fewer than 2 valid waypoints'
            )
            return

        self._outbound_poses = poses

        # ── 2. Always start from WP0 ──────────────────────────────────────────
        self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] Robot pose before outbound: '
            f'x={self._current_x:.3f}  y={self._current_y:.3f}'
        )

        ordered = poses  # always WP0 first

        self.get_logger().info('[DEBUG] Outbound waypoint order (first→last):')
        for i, p in enumerate(ordered):
            self.get_logger().info(
                f'  WP{i}: x={p.pose.position.x:.3f}  y={p.pose.position.y:.3f}'
            )

        self._publish_full_path(ordered)

        # ── 3. OUTBOUND navigation ────────────────────────────────────────────
        self.get_logger().info(f'[Mission] OUTBOUND — {len(ordered)} waypoints')
        success = self._execute_leg(ordered, 'OUTBOUND')
        if not success:
            self.get_logger().error('[Mission] OUTBOUND failed — aborting')
            return

        # ── 4. ARRIVED ────────────────────────────────────────────────────────
        with self._state_lock:
            self._state = State.ARRIVED
        self._publish_status()

        self._mission_signal(False)
        self.get_logger().info('[Mission] ARRIVED — Beacon OFF, Beep STOP')

        self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] Robot pose at ARRIVED: '
            f'x={self._current_x:.3f}  y={self._current_y:.3f}'
        )
        self.get_logger().info('[Mission] ARRIVED — pulsing arrival beacon (pin32)')
        self._pulse_arrival_beacon()

        # ── 5. Wait for /delivery/unloaded ───────────────────────────────────
        self._unloaded_event.clear()
        self.get_logger().info('[Mission] Waiting for /delivery/unloaded from operator...')
        self._unloaded_event.wait()

        # ── 6. DEPARTING ──────────────────────────────────────────────────────
        with self._state_lock:
            self._state = State.DEPARTING
        self._publish_status()
        self.get_logger().info(
            f'[Mission] DEPARTING — beacon already HIGH, '
            f'holding {self._departure_secs}s before return navigation'
        )
        time.sleep(self._departure_secs)

        # ── 7. Stabilization wait ─────────────────────────────────────────────
        self.get_logger().info(
            f'[DEBUG] Stabilization wait START — waiting {self._return_stabilize}s '
            f'for person to clear and ICP to re-lock'
        )
        time.sleep(self._return_stabilize)

        odom_ok = self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] Stabilization wait END — '
            f'odom_ok={odom_ok}  '
            f'robot pose: x={self._current_x:.3f}  y={self._current_y:.3f}'
        )

        self.get_logger().info('[DEBUG] Clearing costmaps before return trip')
        self._clear_costmaps()
        self._clear_local_costmap()
        self.get_logger().info('[DEBUG] Costmaps cleared')

        # ── 8. RETURNING ──────────────────────────────────────────────────────
        with self._state_lock:
            self._state = State.RETURNING
        self._publish_status()

        return_poses = self._build_reverse_leg(ordered)

        self.get_logger().info('[DEBUG] Return waypoint order (first→last):')
        for i, p in enumerate(return_poses):
            self.get_logger().info(
                f'  WP{i}: x={p.pose.position.x:.3f}  y={p.pose.position.y:.3f}  '
                f'oz={p.pose.orientation.z:.3f}  ow={p.pose.orientation.w:.3f}'
            )

        self._publish_full_path(return_poses)

        self.get_logger().info(
            f'[Mission] RETURNING — {len(return_poses)} waypoints (reversed)'
        )
        success = self._execute_leg(return_poses, 'RETURNING')
        if not success:
            self.get_logger().error(
                '[Mission] RETURNING failed — robot may need manual recovery'
            )
            return

        self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] Robot pose at home: '
            f'x={self._current_x:.3f}  y={self._current_y:.3f}'
        )
        self.get_logger().info('[Mission] ✓ Return complete — back home')

    # =========================================================================
    # Arrival beacon — pin32 pulse sequence
    # =========================================================================

    def _pulse_arrival_beacon(self):
        for i in range(self._arrival_pulses):
            self.get_logger().info(
                f'[Beacon] Pulse {i + 1}/{self._arrival_pulses} — HIGH'
            )
            self._gpio32_set(True)
            time.sleep(self._pulse_on)
            self._gpio32_set(False)
            if i < self._arrival_pulses - 1:
                time.sleep(self._pulse_off)
        self.get_logger().info('[Beacon] Arrival pulse sequence complete')

    # =========================================================================
    # YAML helpers
    # =========================================================================

    def _load_yaml(self) -> dict:
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

    def _load_poses(self, route_name: str) -> list:
        if self.active_map is None:
            self.get_logger().error('Cannot load poses — active map not detected')
            return []

        data      = self._load_yaml()
        maps_data = data.get('maps', {})

        if not isinstance(maps_data, dict) or self.active_map not in maps_data:
            available = list(maps_data.keys()) if isinstance(maps_data, dict) else []
            self.get_logger().error(
                f'No routes for active map "{self.active_map}". '
                f'Maps in file: {available}'
            )
            return []

        routes = maps_data[self.active_map].get('routes', {})
        if not isinstance(routes, dict) or route_name not in routes:
            available_routes = list(routes.keys()) if isinstance(routes, dict) else []
            self.get_logger().error(
                f'Route "{route_name}" not found under map '
                f'"{self.active_map}". Available: {available_routes}'
            )
            return []

        waypoint_list = routes[route_name].get('waypoints', [])
        if not waypoint_list:
            self.get_logger().error(f'Route "{route_name}" has no waypoints')
            return []

        poses = []
        for wp in waypoint_list:
            if wp['position']['x'] == 0.0 and wp['position']['y'] == 0.0:
                self.get_logger().warn(
                    f'Skipping origin waypoint (index {wp.get("index", "?")})'
                )
                continue

            raw_z = float(wp['orientation']['z'])
            raw_w = float(wp['orientation']['w'])
            mag   = math.sqrt(raw_z**2 + raw_w**2)
            if mag < 1e-6:
                mag = 1.0

            ps                    = PoseStamped()
            ps.header.frame_id    = self.frame_id
            ps.header.stamp       = self.get_clock().now().to_msg()
            ps.pose.position.x    = float(wp['position']['x'])
            ps.pose.position.y    = float(wp['position']['y'])
            ps.pose.position.z    = float(wp['position']['z'])
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = raw_z / mag
            ps.pose.orientation.w = raw_w / mag
            poses.append(ps)

        self.get_logger().info(
            f'Loaded {len(poses)} waypoints — '
            f'route: "{route_name}"  map: "{self.active_map}"'
        )
        return poses

    # =========================================================================
    # Reverse leg
    # =========================================================================

    def _build_reverse_leg(self, forward_poses: list) -> list:
        reverse_poses = []
        for pose in reversed(forward_poses[:-1]):
            rp  = copy.deepcopy(pose)
            oz  = rp.pose.orientation.z
            ow  = rp.pose.orientation.w
            new_oz = ow
            new_ow = -oz
            mag    = math.sqrt(new_oz**2 + new_ow**2)
            if mag < 1e-6:
                mag = 1.0
            rp.pose.orientation.x = 0.0
            rp.pose.orientation.y = 0.0
            rp.pose.orientation.z = new_oz / mag
            rp.pose.orientation.w = new_ow / mag
            reverse_poses.append(rp)
        return reverse_poses

    # =========================================================================
    # TF pose
    # =========================================================================

    def _update_robot_pose(self) -> bool:
        try:
            t = self._tf_buffer.lookup_transform(
                self.frame_id, 'base_link',
                rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            self._current_x = t.transform.translation.x
            self._current_y = t.transform.translation.y
            return True
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'[DEBUG] TF lookup failed: {e}', throttle_duration_sec=2.0
            )
            return False

    # =========================================================================
    # Costmap clears
    # =========================================================================

    def _clear_costmaps(self):
        if self._clear_global.wait_for_service(timeout_sec=2.0):
            future = self._clear_global.call_async(ClearEntireCostmap.Request())
            ok     = _wait_for_future(future, timeout_sec=5.0)
            self.get_logger().info(
                f'[DEBUG] Global costmap clear: {"OK" if ok else "TIMEOUT"}'
            )
        else:
            self.get_logger().warn('[DEBUG] Global costmap clear service not available')

    def _clear_local_costmap(self):
        if self._clear_local.wait_for_service(timeout_sec=2.0):
            future = self._clear_local.call_async(ClearEntireCostmap.Request())
            ok     = _wait_for_future(future, timeout_sec=5.0)
            self.get_logger().info(
                f'[DEBUG] Local costmap clear: {"OK" if ok else "TIMEOUT"}'
            )
        else:
            self.get_logger().warn('[DEBUG] Local costmap clear service not available')

    # =========================================================================
    # Plan leg
    # =========================================================================

    def _plan_leg(self, goal_poses: list, label: str):
        stamped = []
        now     = self.get_clock().now().to_msg()
        for ps in goal_poses:
            sp              = copy.deepcopy(ps)
            sp.header.stamp = now
            stamped.append(sp)

        self._planner.wait_for_server()

        goal       = ComputePathThroughPoses.Goal()
        goal.goals = stamped

        self.get_logger().info(
            f'[DEBUG] [{label}] Sending plan request — '
            f'{len(stamped)} goals, '
            f'first=({stamped[0].pose.position.x:.2f},'
            f'{stamped[0].pose.position.y:.2f}) '
            f'last=({stamped[-1].pose.position.x:.2f},'
            f'{stamped[-1].pose.position.y:.2f})'
        )

        future = self._planner.send_goal_async(goal)
        if not _wait_for_future(future, timeout_sec=30.0):
            self.get_logger().error(f'[{label}] Planner send timed out')
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'[{label}] Planner rejected goal — '
                f'check waypoints are within map bounds and in free space. '
                f'First: ({stamped[0].pose.position.x:.2f},'
                f'{stamped[0].pose.position.y:.2f}) '
                f'Last: ({stamped[-1].pose.position.x:.2f},'
                f'{stamped[-1].pose.position.y:.2f})'
            )
            return None

        result_future = goal_handle.get_result_async()
        if not _wait_for_future(result_future, timeout_sec=120.0):
            self.get_logger().error(f'[{label}] Plan result timed out')
            return None

        result = result_future.result().result
        if not result.path.poses:
            self.get_logger().error(f'[{label}] Planner returned empty path')
            return None

        self.get_logger().info(
            f'[{label}] Planned {len(result.path.poses)} poses '
            f'through {len(stamped)} waypoints'
        )
        return result.path

    # =========================================================================
    # Follow path
    # =========================================================================

    def _follow_path(self, path: Path, label: str) -> bool:
        self._controller.wait_for_server()

        goal                 = FollowPath.Goal()
        goal.path            = path
        goal.controller_id   = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info(
            f'[DEBUG] [{label}] Sending follow_path — {len(path.poses)} poses'
        )

        future = self._controller.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        if not _wait_for_future(future, timeout_sec=30.0):
            self.get_logger().error(f'[{label}] FollowPath send timed out')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'[{label}] FollowPath rejected')
            return False

        self._active_goal_handle = goal_handle
        result_future            = goal_handle.get_result_async()

        if not _wait_for_future(result_future, timeout_sec=600.0):
            self.get_logger().error(f'[{label}] FollowPath timed out')
            self._active_goal_handle = None
            return False

        self._active_goal_handle = None
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{label}] ✓ Navigation succeeded')
            return True

        self.get_logger().warn(f'[{label}] Navigation ended with status {status}')
        return False

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(
            f'Distance to goal: {dist:.2f} m', throttle_duration_sec=5.0
        )

    # =========================================================================
    # Execute one leg
    # =========================================================================

    def _execute_leg(self, ordered_poses: list, label: str) -> bool:
        self.get_logger().info(
            f'[{label}] Planning batched path through {len(ordered_poses)} waypoints'
        )

        self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] [{label}] Robot pose before planning: '
            f'x={self._current_x:.3f}  y={self._current_y:.3f}'
        )
        self._clear_costmaps()

        path = self._plan_leg(ordered_poses, label)
        if path is None:
            self.get_logger().warn(f'[{label}] Plan failed — retrying in 2s...')
            time.sleep(2.0)
            self._update_robot_pose()
            self.get_logger().info(
                f'[DEBUG] [{label}] Robot pose before retry: '
                f'x={self._current_x:.3f}  y={self._current_y:.3f}'
            )
            self._clear_costmaps()
            path = self._plan_leg(ordered_poses, label)
            if path is None:
                self.get_logger().error(f'[{label}] Retry plan failed — aborting')
                return False

        success = self._follow_path(path, label)

        if not success:
            self.get_logger().info(f'[{label}] Follow failed — replanning once...')
            time.sleep(1.0)
            self._update_robot_pose()
            self.get_logger().info(
                f'[DEBUG] [{label}] Robot pose before replan: '
                f'x={self._current_x:.3f}  y={self._current_y:.3f}'
            )
            self._clear_costmaps()
            retry_path = self._plan_leg(ordered_poses, f'{label} retry')
            if retry_path is None:
                self.get_logger().error(f'[{label}] Retry plan failed')
                return False
            success = self._follow_path(retry_path, f'{label} retry')

        return success

    # =========================================================================
    # Publishers
    # =========================================================================

    def _publish_status(self):
        with self._state_lock:
            state = self._state
        payload  = {
            'state': state,
            'route': self._route_name,
            'map':   self.active_map if self.active_map else 'unknown',
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._pub_status.publish(msg)

    def _publish_available_routes(self):
        if self.active_map is None:
            payload = {'map': 'unknown', 'routes': []}
            self._last_published_routes = []
            self.get_logger().warn('available_routes: empty — map not detected yet')
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

    def _publish_full_path(self, poses: list):
        msg                 = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.poses           = list(poses)
        self._pub_full_path.publish(msg)
        self._pub_rem_path.publish(msg)

    # =========================================================================
    # Shutdown
    # =========================================================================

    def destroy_node(self):
        self.get_logger().info('[DeliveryRunner] Shutting down — GPIOs LOW, beep STOP')
        self._mission_signal(False)
        self._gpio32_set(False)
        self._gpio16_stop_event.set()
        self._beep_stop_event.set()
        self._beep_active_event.set()
        self._gpio16_thread.join(timeout=2.0)
        self._beep_thread.join(timeout=3.0)
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node     = DeliveryRunner()
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