#!/usr/bin/env python3
"""
rc_delivery_trigger.py
Package : trov
Place at : trov_ws/src/trov/scripts/rc_delivery_trigger.py

Starts / continues a delivery mission from a 3-position switch on the RC
transmitter, by reading one channel of /mavros/rc/in.

    switch HIGH  (pwm > high_pwm, default 1900)   -> /delivery/go        (String: route_name)
    switch LOW   (pwm < low_pwm,  default 1100)   -> /delivery/unloaded  (Empty)
    switch MID   (neutral_min..neutral_max)       -> neutral, nothing is sent

Rules (all of them exist to stop a mission starting by accident):

  * One message per flick. Holding the switch HIGH or LOW sends nothing more.
  * The switch must REST IN NEUTRAL first. A command is only sent when the
    switch goes neutral -> HIGH or neutral -> LOW. So:
      - the position the switch is in when this node starts is never a command
        (the stack can boot with the switch left HIGH and nothing happens);
      - a jump straight from HIGH to LOW (or LOW to HIGH) is ignored.
    Set require_neutral:=false to allow HIGH <-> LOW directly (startup position
    is still ignored).
  * Debounce: a position counts only after it has been seen continuously for
    hold_s seconds AND min_samples messages in a row.
  * Values in the gaps (low_pwm..neutral_min and neutral_max..high_pwm) are
    "in between": they change nothing and restart the debounce.
  * Bad values (0, outside valid_min..valid_max, channel missing) or no
    /mavros/rc/in for stale_s seconds: the node forgets the switch position and
    needs to see neutral again before it will send anything.

This node does not look at the delivery state. delivery_runner already ignores
/delivery/go unless IDLE and /delivery/unloaded unless ARRIVED.

Debug: /rc_delivery_trigger/status (JSON String, latched, published on change).
"""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

from mavros_msgs.msg import RCIn
from std_msgs.msg import Empty, String


NEUTRAL = 'NEUTRAL'
HIGH    = 'HIGH'
LOW     = 'LOW'
BETWEEN = 'BETWEEN'
INVALID = 'INVALID'
UNKNOWN = 'UNKNOWN'


class RcDeliveryTrigger(Node):

    def __init__(self):
        super().__init__('rc_delivery_trigger')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rc_in_topic',     '/mavros/rc/in')
        self.declare_parameter('channel',         5)          # 1-based
        self.declare_parameter('high_pwm',        1900)       # HIGH when pwm >  this
        self.declare_parameter('low_pwm',         1100)       # LOW  when pwm <  this
        self.declare_parameter('neutral_min',     1400)       # NEUTRAL when min <= pwm <= max
        self.declare_parameter('neutral_max',     1600)
        self.declare_parameter('valid_min',       800)        # outside valid_min..valid_max = bad value
        self.declare_parameter('valid_max',       2200)
        self.declare_parameter('hold_s',          0.5)
        self.declare_parameter('min_samples',     2)
        self.declare_parameter('stale_s',         1.5)
        self.declare_parameter('require_neutral', True)
        self.declare_parameter('route_name',      'FGG_6')
        self.declare_parameter('go_topic',        '/delivery/go')
        self.declare_parameter('unloaded_topic',  '/delivery/unloaded')

        gp = lambda n: self.get_parameter(n).value
        self._rc_in_topic     = gp('rc_in_topic')
        self._channel         = int(gp('channel'))
        self._high_pwm        = int(gp('high_pwm'))
        self._low_pwm         = int(gp('low_pwm'))
        self._neutral_min     = int(gp('neutral_min'))
        self._neutral_max     = int(gp('neutral_max'))
        self._valid_min       = int(gp('valid_min'))
        self._valid_max       = int(gp('valid_max'))
        self._hold_s          = float(gp('hold_s'))
        self._min_samples     = int(gp('min_samples'))
        self._stale_s         = float(gp('stale_s'))
        self._require_neutral = bool(gp('require_neutral'))
        self._route_name      = str(gp('route_name')).strip()
        self._go_topic        = gp('go_topic')
        self._unloaded_topic  = gp('unloaded_topic')

        if not (self._valid_min < self._low_pwm < self._neutral_min
                <= self._neutral_max < self._high_pwm < self._valid_max):
            raise ValueError(
                'rc_delivery_trigger: thresholds must satisfy valid_min < low_pwm < '
                'neutral_min <= neutral_max < high_pwm < valid_max'
            )
        if self._channel < 1:
            raise ValueError('rc_delivery_trigger: channel is 1-based and must be >= 1')
        if not self._route_name:
            raise ValueError('rc_delivery_trigger: route_name must not be empty')

        # ── State ─────────────────────────────────────────────────────────────
        self._stable     = UNKNOWN   # last debounced switch position
        self._cand       = None      # position being debounced
        self._cand_t0    = 0.0
        self._cand_n     = 0
        self._last_rx    = None      # monotonic time of the last rc/in message
        self._last_pwm   = 0
        self._last_event = 'none'
        self._bad_logged = False

        # ── Publishers / subscriber ───────────────────────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_go       = self.create_publisher(String, self._go_topic, 10)
        self._pub_unloaded = self.create_publisher(Empty, self._unloaded_topic, 10)
        self._pub_status   = self.create_publisher(String, '/rc_delivery_trigger/status', latched)

        self.create_subscription(RCIn, self._rc_in_topic, self._rc_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._stale_check)

        self.get_logger().info(
            f'RC delivery trigger: {self._rc_in_topic} ch{self._channel} | '
            f'>{self._high_pwm} -> {self._go_topic} "{self._route_name}" | '
            f'<{self._low_pwm} -> {self._unloaded_topic} | '
            f'neutral {self._neutral_min}-{self._neutral_max} | '
            f'hold {self._hold_s}s | require_neutral={self._require_neutral}'
        )
        self._publish_status()

    # =========================================================================

    def _classify(self, pwm: int) -> str:
        if pwm < self._valid_min or pwm > self._valid_max:
            return INVALID
        if pwm > self._high_pwm:
            return HIGH
        if pwm < self._low_pwm:
            return LOW
        if self._neutral_min <= pwm <= self._neutral_max:
            return NEUTRAL
        return BETWEEN

    def _forget(self, why: str):
        """Drop the known switch position; neutral must be seen again."""
        if self._stable != UNKNOWN or self._cand is not None:
            self.get_logger().warn(
                f'{why} — switch position forgotten, move it to neutral before the next command'
            )
        self._stable = UNKNOWN
        self._cand   = None
        self._publish_status()

    def _rc_cb(self, msg: RCIn):
        now = time.monotonic()
        self._last_rx = now

        idx = self._channel - 1
        if idx >= len(msg.channels):
            self._last_pwm = 0
            if not self._bad_logged:
                self.get_logger().error(
                    f'{self._rc_in_topic} has {len(msg.channels)} channels, '
                    f'need channel {self._channel}'
                )
                self._bad_logged = True
            self._forget('RC channel missing')
            return

        pwm = int(msg.channels[idx])
        self._last_pwm = pwm
        zone = self._classify(pwm)

        if zone == INVALID:
            if not self._bad_logged:
                self.get_logger().warn(f'ch{self._channel} pwm {pwm} is not a valid RC value')
                self._bad_logged = True
            self._forget(f'Bad RC value ({pwm})')
            return
        self._bad_logged = False

        if zone == BETWEEN or zone == self._stable:
            self._cand = None
            return

        if zone != self._cand:
            self._cand, self._cand_t0, self._cand_n = zone, now, 1
            return

        self._cand_n += 1
        if (now - self._cand_t0) < self._hold_s or self._cand_n < self._min_samples:
            return

        # ── Debounced change of switch position ───────────────────────────────
        prev         = self._stable
        self._stable = zone
        self._cand   = None

        if zone == NEUTRAL:
            self.get_logger().info(f'Switch NEUTRAL (pwm {pwm}) — ready for a command')
        elif prev == NEUTRAL or (not self._require_neutral and prev != UNKNOWN):
            self._fire(zone, pwm)
        elif prev == UNKNOWN:
            self.get_logger().warn(
                f'Switch is {zone} (pwm {pwm}) at startup / after RC data came back — '
                f'NOT sending. Move it to neutral first, then to {zone}.'
            )
            self._last_event = f'ignored_{zone}_no_neutral'
        else:
            self.get_logger().warn(
                f'Switch went {prev} -> {zone} (pwm {pwm}) without resting in neutral — '
                f'NOT sending. Hold neutral for {self._hold_s}s, then move to {zone}.'
            )
            self._last_event = f'ignored_{zone}_no_neutral'
        self._publish_status()

    def _fire(self, zone: str, pwm: int):
        if zone == HIGH:
            pub, topic = self._pub_go, self._go_topic
            msg = String()
            msg.data = self._route_name
            what = f'{topic} "{self._route_name}"'
            self._last_event = 'go'
        else:
            pub, topic = self._pub_unloaded, self._unloaded_topic
            msg = Empty()
            what = topic
            self._last_event = 'unloaded'

        if pub.get_subscription_count() == 0:
            self.get_logger().error(
                f'Switch {zone} (pwm {pwm}) — sending {what}, but nothing is subscribed '
                f'(is delivery_runner running?)'
            )
        else:
            self.get_logger().info(f'Switch {zone} (pwm {pwm}) — sending {what}')
        pub.publish(msg)

    def _stale_check(self):
        if self._last_rx is None:
            return
        if (time.monotonic() - self._last_rx) > self._stale_s:
            self._last_rx = None
            self._forget(f'No {self._rc_in_topic} for {self._stale_s}s')

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'channel':    self._channel,
            'pwm':        self._last_pwm,
            'position':   self._stable,
            'ready':      self._stable == NEUTRAL
                          or (not self._require_neutral and self._stable != UNKNOWN),
            'last_event': self._last_event,
            'route':      self._route_name,
        })
        self._pub_status.publish(msg)


def main():
    rclpy.init()
    node = RcDeliveryTrigger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise          # a real error, not the Ctrl+C race inside rclpy
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
