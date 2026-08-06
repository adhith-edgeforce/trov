#!/usr/bin/env python3
"""
delivery_runner.py
Package : trov
Place at : trov_ws/src/trov/scripts/delivery_runner.py

Standalone delivery mission executor for the T-ROV UGV.

Control topics:
    /delivery/start (std_msgs/String) — begin a mission for the named route.
    /delivery/stop  (std_msgs/Empty)  — cancel navigation immediately and stop.

auto_return parameter:
    False (default) : single out-and-back — WP0 -> WPN, wait, WPN -> WP0, IDLE.
    True            : forever loop — WP0 -> WPN -> WP0 -> WPN -> ... until
                      /delivery/stop is published. Beacon + beep stay ON the
                      whole time; a wait of return_stabilize_sec (default 8 s)
                      is taken at each endpoint (WPN and WP0) before the next leg.

resume_from_nearest parameter (default True):
    On /delivery/start the first leg begins from the waypoint nearest the
    robot, in the direction it was travelling when last stopped (OUTBOUND →
    forward to WPN, RETURNING → back to WP0), then falls into the normal
    full WP0↔WPN loop. When the robot is already home this is just WP0.
    Set False to always start from WP0.

NOTE: the /delivery/unloaded handshake and the pin32 arrival "flashlight" have
      been retired. The unloaded subscriber/callback are left commented out
      below for reference only.
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
from std_msgs.msg import Empty, String
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

def _map_stem_from_yaml_path(yaml_path: str) -> str:
    return os.path.splitext(os.path.basename(yaml_path))[0]


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
            os.path.expanduser('/data/trov_ws/src/trov/routes/indoor_waypoints.yaml')
        )
        self.declare_parameter('frame_id',                  'map')
        self.declare_parameter('map_server_node',           'map_server')
        self.declare_parameter('map_detect_timeout',        10.0)
        self.declare_parameter('gpio_chip_path',            '/dev/gpiochip1')
        self.declare_parameter('gpio_line',                 9)
        self.declare_parameter('return_stabilize_sec',      8.0)
        self.declare_parameter('predeparture_delay_sec',    5.0)
        self.declare_parameter('beep_sound_path',           '/data/trov_ws/beep_cut.mp3')
        self.declare_parameter('beep_volume',               2.5)
        self.declare_parameter('routes_poll_interval',      5.0)
        # ── Continuous-loop mode ──────────────────────────────────────────────
        self.declare_parameter('auto_return',               False)
        # ── Resume from nearest waypoint (direction-aware) ────────────────────
        self.declare_parameter('resume_from_nearest',       True)

        self.waypoints_file        = self.get_parameter('waypoints_file').value
        self.frame_id              = self.get_parameter('frame_id').value
        self._map_server_node      = self.get_parameter('map_server_node').value
        self._map_detect_timeout   = self.get_parameter('map_detect_timeout').value
        self._gpio_chip_path       = self.get_parameter('gpio_chip_path').value
        self._gpio_line_num        = self.get_parameter('gpio_line').value
        self._return_stabilize     = self.get_parameter('return_stabilize_sec').value
        self._predeparture_delay   = self.get_parameter('predeparture_delay_sec').value
        self._beep_sound_path      = self.get_parameter('beep_sound_path').value
        self._beep_volume          = self.get_parameter('beep_volume').value
        self._routes_poll_interval = self.get_parameter('routes_poll_interval').value
        self._auto_return          = self.get_parameter('auto_return').value
        self._resume_from_nearest  = self.get_parameter('resume_from_nearest').value

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
        # Retained only for the retired /delivery/unloaded handshake (disabled).
        self._unloaded_event = threading.Event()
        # Stop signal for the current mission / auto-return loop.
        self._stop_event     = threading.Event()
        # Which leg (OUTBOUND/RETURNING) was active when the last /delivery/stop
        # fired — used for direction-aware resume. None = no memory.
        self._interrupted_leg = None

        self._active_goal_handle = None
        self._mission_thread     = None
        self._outbound_poses: list = []

        # ── Cached route list — used to detect changes and avoid noisy publishes
        self._last_published_routes: list = []

        # ── GPIO16 persistent line-holder (the steady beacon light) ───────────
        #
        # One background thread owns the gpiod line for the node's full
        # lifetime.  _gpio16_high_event set → HIGH, cleared → LOW.
        # All callers use _gpio16_set(True/False) only.
        #
        self._gpio16_ok         = self._check_gpio16()
        self._gpio16_high_event = threading.Event()
        self._gpio16_stop_event = threading.Event()
        self._gpio16_thread     = threading.Thread(
            target=self._gpio16_holder_thread, daemon=True
        )
        self._gpio16_thread.start()

        # ── Beep loop thread ─────────────────────────────────────────────────
        #
        # Mirrors the beacon: set _beep_active_event → continuous looping
        # play until the event is cleared.  Uses the sox `play` command.
        #
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

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, '/delivery/start',
            self._start_cb, 10,
            callback_group=self._sub_group
        )
        # ── /delivery/unloaded — RETIRED (commented out, kept for reference) ──
        # The unload handshake has been removed from the mission flow. Uncomment
        # this subscriber (and the _unloaded_cb method further below) to restore.
        # self.create_subscription(
        #     Empty, '/delivery/unloaded',
        #     self._unloaded_cb, 10,
        #     callback_group=self._sub_group
        # )
        # ── stop the current mission / auto-return loop ───────────────────────
        self.create_subscription(
            Empty, '/delivery/stop',
            self._stop_cb, 10,
            callback_group=self._sub_group
        )

        # ── Detect active map ─────────────────────────────────────────────────
        self._detect_active_map()

        # ── Publish initial state ─────────────────────────────────────────────
        self._publish_available_routes()
        self._publish_status()

        # ── Poll YAML for new routes every N seconds ──────────────────────────
        self.create_timer(
            self._routes_poll_interval,
            self._routes_poll_cb,
            callback_group=self._sub_group
        )

        self.get_logger().info(
            f'\n[DeliveryRunner] Ready\n'
            f'  Active map          : {self.active_map}\n'
            f'  Waypoints file      : {self.waypoints_file}\n'
            f'  Routes poll         : every {self._routes_poll_interval}s\n'
            f'  auto_return (LOOP)  : {self._auto_return}\n'
            f'  resume_from_nearest : {self._resume_from_nearest}\n'
            f'  Pre-departure delay : {self._predeparture_delay}s\n'
            f'  Endpoint wait       : {self._return_stabilize}s (at WPN and WP0)\n'
            f'  GPIO chip/line      : {self._gpio_chip_path} / {self._gpio_line_num} '
            f'({"OK" if self._gpio16_ok else "DISABLED"})\n'
            f'  Beep sound          : {self._beep_sound_path}  vol {self._beep_volume}x\n'
            f'  Control topics      : start=/delivery/start  stop=/delivery/stop\n'
        )

    # =========================================================================
    # GPIO16 — hardware check (once, before holder thread starts)
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
    # GPIO16 — persistent line-holder thread (steady beacon)
    # =========================================================================

    def _gpio16_holder_thread(self):
        """
        Owns the gpiod line for the node's entire lifetime.
        Drives HIGH while _gpio16_high_event is set, LOW otherwise.
        Exits when _gpio16_stop_event is set.
        """
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
        """Drive GPIO16 (beacon) HIGH or LOW via the persistent holder thread."""
        if high:
            self._gpio16_high_event.set()
        else:
            self._gpio16_high_event.clear()

    # =========================================================================
    # Beep loop thread
    # =========================================================================

    def _beep_loop_thread(self):
        """
        Continuously plays the beep sound while _beep_active_event is set.
        Each play() call blocks until the file finishes, then loops immediately.
        Stops after the current play finishes once the event is cleared.
        Exits when _beep_stop_event is set.
        """
        vol_fraction = self._beep_volume
        self.get_logger().info('[Beep-thread] Ready')

        while not self._beep_stop_event.is_set():
            # Wait until beep is requested
            if not self._beep_active_event.wait(timeout=0.1):
                continue

            if self._beep_stop_event.is_set():
                break

            # Check sound file exists
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
                    cmd = [
                        'play', '-q',
                        self._beep_sound_path,
                        'vol', str(vol_fraction)
                    ]
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    # Poll so we can react quickly if the event is cleared
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
        """Start (active=True) or stop (active=False) the continuous beep loop."""
        if active:
            self._beep_active_event.set()
        else:
            self._beep_active_event.clear()

    # =========================================================================
    # Convenience: set beacon light + beep together
    # =========================================================================

    def _mission_signal(self, active: bool):
        """Turn GPIO16 beacon and beep ON or OFF together."""
        self._gpio16_set(active)
        self._beep_set(active)

    # =========================================================================
    # Map detection
    # =========================================================================

    def _detect_active_map(self):
        srv_name = f'/{self._map_server_node}/get_parameters'
        client   = self.create_client(GetParameters, srv_name)
        self.get_logger().info(
            f'[MapDetect] Contacting {srv_name} '
            f'(timeout: {self._map_detect_timeout}s)...'
        )
        deadline = time.time() + self._map_detect_timeout
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                self.get_logger().error('[MapDetect] ✗ map_server not available.')
                return
            self.get_logger().warn(
                '[MapDetect] map_server not ready — retrying...',
                throttle_duration_sec=2.0
            )
        request       = GetParameters.Request()
        request.names = ['yaml_filename']
        future        = client.call_async(request)
        remaining     = max(deadline - time.time(), 2.0)
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done():
            self.get_logger().error('[MapDetect] ✗ Parameter request timed out.')
            return
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'[MapDetect] ✗ Exception: {e}')
            return
        if not response.values or not response.values[0].string_value:
            self.get_logger().error('[MapDetect] ✗ yaml_filename empty or missing.')
            return
        self.active_map = _map_stem_from_yaml_path(response.values[0].string_value)
        self.get_logger().info(f'[MapDetect] ✓ Active map: "{self.active_map}"')

    # =========================================================================
    # Routes poll timer callback
    # =========================================================================

    def _routes_poll_cb(self):
        """
        Called every routes_poll_interval seconds.
        Re-reads the YAML and republishes /delivery/available_routes only
        if the route list has changed since the last publish.
        Silent when nothing changed — no log spam.
        """
        if self.active_map is None:
            return

        current_routes = self._read_route_names_for_map(self.active_map)

        if current_routes != self._last_published_routes:
            self.get_logger().info(
                f'[RoutesPoll] Route list changed for map "{self.active_map}": '
                f'{self._last_published_routes} → {current_routes}. Republishing.'
            )
            self._publish_available_routes()

    # =========================================================================
    # State helper
    # =========================================================================

    def _set_state(self, state: str):
        with self._state_lock:
            self._state = state
        self._publish_status()

    # =========================================================================
    # Subscriber callbacks
    # =========================================================================

    def _start_cb(self, msg: String):
        route = msg.data.strip()
        if not route:
            self.get_logger().warn('[/delivery/start] Empty route name — ignoring')
            return
        with self._state_lock:
            if self._state != State.IDLE:
                self.get_logger().warn(
                    f'[/delivery/start] Mission already running '
                    f'(state: {self._state}) — ignoring "{route}"'
                )
                return
            if self.active_map is None:
                self.get_logger().error(
                    '[/delivery/start] Active map not detected — cannot start mission'
                )
                return
            self._route_name = route
            self._state      = State.OUTBOUND

        # Fresh mission — clear any stale stop request
        self._stop_event.clear()

        mode = 'LOOP (forever)' if self._auto_return else 'single trip'
        self.get_logger().info(
            f'[DeliveryRunner] Mission START → '
            f'route: "{route}"  map: "{self.active_map}"  mode: {mode}'
        )

        # ── Beacon + beep ON immediately when /delivery/start is received ─────
        self._mission_signal(True)
        self.get_logger().info('[Mission] Beacon ON + Beep START')

        self._publish_status()

        self._mission_thread = threading.Thread(
            target=self._run_mission, daemon=True
        )
        self._mission_thread.start()

    # ── /delivery/unloaded callback — RETIRED (commented out, kept for ref) ──
    # def _unloaded_cb(self, _msg: Empty):
    #     with self._state_lock:
    #         if self._state != State.ARRIVED:
    #             self.get_logger().warn(
    #                 f'[/delivery/unloaded] Ignored — '
    #                 f'state is {self._state} (must be ARRIVED)'
    #             )
    #             return
    #     self.get_logger().info('[/delivery/unloaded] ✓ Unloaded signal received')
    #     self._mission_signal(True)
    #     self._unloaded_event.set()

    def _stop_cb(self, _msg: Empty):
        """
        Stop the current mission / auto-return loop immediately.
        Cancels the in-flight FollowPath goal so the robot halts now, and sets
        _stop_event so the mission thread exits cleanly at the next checkpoint.
        The _run_mission finally block turns beacon + beep off and returns to IDLE.
        """
        with self._state_lock:
            running = self._state != State.IDLE
            current = self._state
        if not running:
            self.get_logger().warn('[/delivery/stop] No mission running — ignoring')
            return

        # Remember which leg was interrupted, for direction-aware resume.
        if current in (State.OUTBOUND, State.RETURNING):
            self._interrupted_leg = current
            self.get_logger().info(
                f'[/delivery/stop] Interrupted during {current} — '
                f'remembered for next resume'
            )

        self.get_logger().info(
            '[/delivery/stop] ✓ Stop requested — cancelling navigation immediately'
        )
        self._stop_event.set()
        self._cancel_active_goal()

    def _cancel_active_goal(self):
        """Cancel the in-flight FollowPath goal, if any."""
        gh = self._active_goal_handle
        if gh is None:
            self.get_logger().info('[Stop] No active navigation goal to cancel')
            return
        try:
            self.get_logger().info('[Stop] Cancelling active FollowPath goal')
            gh.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f'[Stop] Cancel request failed: {e}')

    # =========================================================================
    # Mission thread
    # =========================================================================

    def _run_mission(self):
        try:
            self._mission_body()
        except Exception as e:
            self.get_logger().error(f'[Mission] Unhandled exception: {e}')
        finally:
            # Always ensure beacon and beep are off when the mission ends
            self._mission_signal(False)
            # If the mission ended on its own (not via /delivery/stop), there is
            # no interrupted leg to resume from next time — clear the memory.
            if not self._stop_event.is_set():
                self._interrupted_leg = None
            with self._state_lock:
                self._state = State.IDLE
            self._publish_status()
            self.get_logger().info(
                '[DeliveryRunner] Mission ended — IDLE, Beacon OFF, Beep STOP'
            )

    def _mission_body(self):

        # ── 0. Pre-departure delay (interruptible) ───────────────────────────
        # Beacon + beep are already ON (started in _start_cb).
        self.get_logger().info(
            f'[Mission] Pre-departure delay — waiting {self._predeparture_delay}s '
            f'before first outbound leg (beacon + beep already active)'
        )
        if self._stop_event.wait(timeout=self._predeparture_delay):
            self.get_logger().info('[Mission] Stop requested during pre-departure delay')
            return

        # ── 1. Load waypoints ────────────────────────────────────────────────
        poses = self._load_poses(self._route_name)
        if len(poses) < 2:
            self.get_logger().error(
                f'[Mission] Route "{self._route_name}" has fewer than 2 valid waypoints'
            )
            return

        self._outbound_poses = poses

        # ── 2. Canonical outbound order is always WP0 → WPN ─────────────────
        # `ordered` is never cyclically reordered. The first leg MAY start from
        # a mid-route waypoint (see _plan_first_leg / resume_from_nearest), but
        # only ever as a forward or backward *slice* of this list — never a
        # wrap-around — so the route can't fold back on itself.
        self._update_robot_pose()
        self.get_logger().info(
            f'[DEBUG] Robot pose before first leg: '
            f'x={self._current_x:.3f}  y={self._current_y:.3f}'
        )

        ordered = poses  # canonical WP0 → WPN

        self.get_logger().info('[DEBUG] Outbound waypoint order (WP0 → WPN):')
        for i, p in enumerate(ordered):
            self.get_logger().info(
                f'  WP{i}: x={p.pose.position.x:.3f}  y={p.pose.position.y:.3f}'
            )

        # Build the reverse leg once (WPN → WP0); reused every lap in loop mode.
        return_poses = self._build_reverse_leg(ordered)

        # ── 3. Run laps (one, or forever if auto_return) ─────────────────────
        self._run_laps(ordered, return_poses)

    # =========================================================================
    # Lap runner — shared by single-trip and forever-loop modes
    # =========================================================================

    def _run_laps(self, ordered: list, return_poses: list):
        """
        Leg-based runner shared by single-trip and forever-loop modes.

        A "leg" is one traversal that ends at an endpoint (WPN or WP0):
            OUTBOUND  → WP0 → WPN
            RETURNING → WPN → WP0
        Steady state alternates OUTBOUND / RETURNING with an endpoint wait
        (return_stabilize_sec) at every endpoint.

        The FIRST leg is chosen by _plan_first_leg(): with resume_from_nearest
        it may be a *partial* leg starting from the waypoint nearest the robot,
        in the direction it was travelling when last stopped. Every leg after
        the first is a full WP0↔WPN traversal.

        auto_return == False : finish after the first RETURNING reaches WP0.
        auto_return == True  : alternate forever until /delivery/stop.

        Beacon + beep stay ON the whole time (lean loop). A stop cancels the
        active navigation immediately and breaks out.
        """
        mode = 'LOOP forever (until /delivery/stop)' if self._auto_return \
            else 'single out-and-back'
        self.get_logger().info(f'[Mission] Running — {mode}')

        # Decide the first leg (resume-from-nearest, direction-aware).
        leg_type, leg_poses, reason = self._plan_first_leg(ordered, return_poses)
        self.get_logger().info(f'[Mission] First leg → {leg_type}: {reason}')

        leg_num = 0
        while not self._stop_event.is_set():
            leg_num += 1
            endpoint = 'WPN' if leg_type == State.OUTBOUND else 'WP0'

            # ── Execute the current leg ──────────────────────────────────────
            self._set_state(leg_type)
            self._publish_full_path(leg_poses)
            self.get_logger().info(
                f'[Leg {leg_num}] {leg_type} → {endpoint} '
                f'({len(leg_poses)} waypoints)'
            )
            if not self._execute_leg(leg_poses, f'{leg_type} (leg {leg_num})'):
                self._report_leg_break(leg_type, leg_num)
                break
            if self._stop_event.is_set():
                break

            # ── Single trip ends once a RETURNING leg reaches WP0 ────────────
            if not self._auto_return and leg_type == State.RETURNING:
                self.get_logger().info('[Mission] ✓ Single out-and-back complete — at WP0')
                break

            # ── Endpoint wait before the next leg ────────────────────────────
            if self._endpoint_wait(leg_num, endpoint):
                break

            # ── Alternate to the next FULL leg ───────────────────────────────
            if leg_type == State.OUTBOUND:
                leg_type, leg_poses = State.RETURNING, return_poses
            else:
                leg_type, leg_poses = State.OUTBOUND, ordered

        if self._stop_event.is_set():
            self.get_logger().info(
                f'[Mission] ✓ Stopped by /delivery/stop after {leg_num} leg(s)'
            )
        else:
            self.get_logger().info(f'[Mission] Finished after {leg_num} leg(s)')

    def _plan_first_leg(self, ordered: list, return_poses: list):
        """
        Choose the first leg based on resume_from_nearest and the robot pose.

        Returns (leg_type, leg_poses, reason_str) where leg_type is
        State.OUTBOUND or State.RETURNING.

        Default (resume disabled, TF unavailable, or robot already at an
        endpoint): a full leg from WP0 / WPN. Otherwise a partial leg starting
        from the nearest waypoint, in the direction the robot was last going:
          - interrupted while OUTBOUND (or no memory) → forward, nearest → WPN
          - interrupted while RETURNING               → backward, nearest → WP0
        """
        if not self._resume_from_nearest:
            return State.OUTBOUND, ordered, 'full outbound from WP0 (resume disabled)'

        if not self._update_robot_pose():
            return State.OUTBOUND, ordered, 'full outbound from WP0 (TF pose unavailable)'

        nearest_idx = self._nearest_waypoint_index(ordered)
        last_idx    = len(ordered) - 1
        resume_returning = (self._interrupted_leg == State.RETURNING)

        if resume_returning:
            # Head home from the nearest waypoint: partial RETURNING → WP0.
            backward = self._build_reverse_leg(ordered[:nearest_idx + 1])
            if len(backward) >= 1:
                return (State.RETURNING, backward,
                        f'partial RETURNING from WP{nearest_idx} → WP0 '
                        f'(resuming return direction)')
            # nearest is WP0 already → nothing to return; do a full outbound.
            return State.OUTBOUND, ordered, 'already at WP0 → full outbound'

        # Head out from the nearest waypoint: partial OUTBOUND → WPN.
        forward = ordered[nearest_idx:]
        if len(forward) >= 2:
            return (State.OUTBOUND, forward,
                    f'partial OUTBOUND from WP{nearest_idx} → WPN '
                    f'(resuming outbound direction)')
        # nearest is WPN already → nothing to go out to; do a full return.
        return State.RETURNING, return_poses, f'already at WP{last_idx} (WPN) → full return'

    def _nearest_waypoint_index(self, poses: list) -> int:
        """Index of the waypoint geometrically closest to the current robot pose."""
        best_i, best_d = 0, float('inf')
        for i, p in enumerate(poses):
            d = dist2d(self._current_x, self._current_y,
                       p.pose.position.x, p.pose.position.y)
            if d < best_d:
                best_d, best_i = d, i
        self.get_logger().info(
            f'[Resume] Nearest waypoint: WP{best_i} at {best_d:.2f} m '
            f'(robot x={self._current_x:.2f}, y={self._current_y:.2f})'
        )
        return best_i

    def _endpoint_wait(self, leg_num: int, where: str) -> bool:
        """
        Interruptible stabilization wait at an endpoint (WPN or WP0), followed
        by a costmap clear so the next leg plans cleanly. Beacon + beep stay ON.
        Returns True if a stop was requested (caller should break the loop).
        """
        self._update_robot_pose()
        self.get_logger().info(
            f'[Leg {leg_num}] Reached {where} '
            f'(x={self._current_x:.3f}, y={self._current_y:.3f}) — '
            f'waiting {self._return_stabilize}s for ICP re-lock'
        )
        if self._stop_event.wait(timeout=self._return_stabilize):
            return True
        self.get_logger().info(f'[Leg {leg_num}] Clearing costmaps at {where}')
        self._clear_costmaps()
        self._clear_local_costmap()
        return self._stop_event.is_set()

    def _report_leg_break(self, leg: str, leg_num: int):
        """Log why a leg ended (stop vs failure)."""
        if self._stop_event.is_set():
            self.get_logger().info(f'[Leg {leg_num}] Stopped during {leg}')
        else:
            self.get_logger().error(f'[Leg {leg_num}] {leg} failed — ending mission')

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

            # Force Z=0 and pure-yaw quaternion for 2D NavFn planner
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
            ps.pose.position.z    = 0.0
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
        """
        Reverse the waypoint list and flip each pose yaw by 180° using
        correct quaternion multiplication for 2D rotation.
        """
        reverse_poses = []
        for pose in reversed(forward_poses[:-1]):
            rp = copy.deepcopy(pose)

            oz = rp.pose.orientation.z
            ow = rp.pose.orientation.w

            # Rotate 180° around Z: multiply quaternion by (0, 0, sin90, cos90) = (0,0,1,0)
            # Result: new_oz = ow, new_ow = -oz
            new_oz = ow
            new_ow = -oz

            mag = math.sqrt(new_oz**2 + new_ow**2)
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
            ok = _wait_for_future(future, timeout_sec=5.0)
            self.get_logger().info(
                f'[DEBUG] Global costmap clear: {"OK" if ok else "TIMEOUT"}'
            )
        else:
            self.get_logger().warn(
                '[DEBUG] Global costmap clear service not available'
            )

    def _clear_local_costmap(self):
        if self._clear_local.wait_for_service(timeout_sec=2.0):
            future = self._clear_local.call_async(ClearEntireCostmap.Request())
            ok = _wait_for_future(future, timeout_sec=5.0)
            self.get_logger().info(
                f'[DEBUG] Local costmap clear: {"OK" if ok else "TIMEOUT"}'
            )
        else:
            self.get_logger().warn(
                '[DEBUG] Local costmap clear service not available'
            )

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

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'[{label}] Navigation cancelled (stop requested)')
            return False

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
        # Bail out early if a stop was requested before we even start.
        if self._stop_event.is_set():
            return False

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
            if self._stop_event.is_set():
                return False
            self.get_logger().warn(f'[{label}] Plan failed — retrying in 2s...')
            if self._stop_event.wait(timeout=2.0):
                return False
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

        # Don't start following if a stop landed during planning.
        if self._stop_event.is_set():
            return False

        success = self._follow_path(path, label)

        if not success:
            # A stop cancels the goal → do NOT retry, just exit.
            if self._stop_event.is_set():
                self.get_logger().info(f'[{label}] Navigation cancelled by stop')
                return False
            self.get_logger().info(f'[{label}] Follow failed — replanning once...')
            if self._stop_event.wait(timeout=1.0):
                return False
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
            if self._stop_event.is_set():
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
            'state':       state,
            'route':       self._route_name,
            'map':         self.active_map if self.active_map else 'unknown',
            'auto_return': self._auto_return,
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
        self.get_logger().info('[DeliveryRunner] Shutting down — beacon LOW, beep STOP')
        # Stop any running mission / loop and cancel in-flight navigation.
        self._stop_event.set()
        self._cancel_active_goal()
        self._mission_signal(False)
        self._gpio16_stop_event.set()
        self._beep_stop_event.set()
        self._beep_active_event.set()   # unblock the beep thread if it's waiting
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