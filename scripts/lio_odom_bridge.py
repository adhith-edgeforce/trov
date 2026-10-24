#!/usr/bin/env python3
"""
lio_odom_bridge.py

Bridges FAST-LIO's /Odometry (camera_init -> body, where `body` is the IMU
frame) into a nav2-compatible odom -> base_link odometry source.

Why this exists
---------------
FAST-LIO hardcodes its frames to camera_init -> body and `body` IS THE IMU
FRAME, not base_link. If your IMU (built-in to the lidar, or separate) sits at
a significant offset/rotation from base_link, you CANNOT simply rename the
frames -- you must compose the extrinsic. This node does that.

It also:
  * Re-origins so that odom starts at identity at the robot's initial base_link
    pose. This is the ROS convention for an odom frame, and it conveniently
    cancels out FAST-LIO's arbitrary initial orientation (camera_init is the
    initial IMU frame, which on a tilted mount is NOT level).
  * Optionally flattens to 2D (z = 0, roll = pitch = 0), which is what a
    gmapping-map + AMCL + nav2 2D stack wants.

Usage
-----
  ros2 run <your_pkg> lio_odom_bridge.py --ros-args \
      -p lio_body_frame:=lidar_link \
      -p two_d_mode:=true

Set publish_tf:=false if something else owns odom -> base_link.

IMPORTANT: run FAST-LIO with its own TF broadcast DISABLED, otherwise two
nodes fight over the tree. Comment out the `tf_br->sendTransform(trans);`
line in publish_odometry() in src/laserMapping.cpp and rebuild.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import tf2_ros


# ---------------------------------------------------------------------------
# Small quaternion / homogeneous-transform helpers (avoids a hard dependency
# on transforms3d / tf_transformations, which are not always installed).
# Quaternions are (x, y, z, w) to match ROS message ordering.
# ---------------------------------------------------------------------------

def quat_to_mat(x, y, z, w):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m):
    """3x3 rotation matrix -> quaternion (x,y,z,w). Shepperd's method."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def make_tf(trans, quat):
    """(3,) translation + (x,y,z,w) quaternion -> 4x4 homogeneous transform."""
    t = np.eye(4)
    t[:3, :3] = quat_to_mat(*quat)
    t[:3, 3] = trans
    return t


def split_tf(t):
    """4x4 -> (translation (3,), quaternion (x,y,z,w))."""
    return t[:3, 3].copy(), mat_to_quat(t[:3, :3])


def invert_tf(t):
    """Fast inverse of a rigid 4x4 transform."""
    out = np.eye(4)
    r_t = t[:3, :3].T
    out[:3, :3] = r_t
    out[:3, 3] = -r_t @ t[:3, 3]
    return out


def yaw_of(m):
    """Extract yaw (rad) from a 3x3 rotation matrix."""
    return math.atan2(m[1, 0], m[0, 0])


def yaw_to_quat(yaw):
    """Yaw (rad) -> quaternion (x,y,z,w) about +Z."""
    return np.array([0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)])


def flatten_2d(t):
    """Zero out z, roll and pitch, keeping x, y and yaw."""
    out = np.eye(4)
    out[:3, :3] = quat_to_mat(*yaw_to_quat(yaw_of(t[:3, :3])))
    out[0, 3] = t[0, 3]
    out[1, 3] = t[1, 3]
    out[2, 3] = 0.0
    return out


class LioOdomBridge(Node):

    def __init__(self):
        super().__init__('lio_odom_bridge')

        # --- parameters ----------------------------------------------------
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/odom')
        # The frame FAST-LIO's `body` child frame physically corresponds to.
        # Built-in lidar IMU  -> 'lidar_link'
        # Separate wit IMU    -> 'imu_link'
        self.declare_parameter('lio_body_frame', 'lidar_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        # Flatten to x/y/yaw. Recommended for a 2D gmapping + AMCL + nav2 stack.
        self.declare_parameter('two_d_mode', True)
        # Re-origin so odom starts at the robot's initial pose.
        self.declare_parameter('zero_origin', True)

        gp = self.get_parameter
        self.input_topic = gp('input_topic').value
        self.output_topic = gp('output_topic').value
        self.lio_body_frame = gp('lio_body_frame').value
        self.base_frame = gp('base_frame').value
        self.odom_frame = gp('odom_frame').value
        self.publish_tf = gp('publish_tf').value
        self.two_d_mode = gp('two_d_mode').value
        self.zero_origin = gp('zero_origin').value

        # --- state ---------------------------------------------------------
        # T_body_base: pose of base_link expressed in the LIO body frame.
        self.t_body_base = None
        # T_correct: maps FAST-LIO's camera_init into our odom frame.
        self.t_correct = None

        # --- tf --------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, qos)
        self.sub = self.create_subscription(
            Odometry, self.input_topic, self.on_odom, qos)

        self.get_logger().info(
            f'lio_odom_bridge: {self.input_topic} -> {self.output_topic} '
            f'[{self.odom_frame} -> {self.base_frame}], '
            f'lio body frame = {self.lio_body_frame}, '
            f'2d={self.two_d_mode}, publish_tf={self.publish_tf}')

    # -----------------------------------------------------------------------

    def lookup_extrinsic(self):
        """Resolve and cache T_body_base from the static TF tree."""
        if self.t_body_base is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.lio_body_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'waiting for {self.lio_body_frame} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return False

        tr = tf.transform.translation
        rot = tf.transform.rotation
        self.t_body_base = make_tf(
            np.array([tr.x, tr.y, tr.z]),
            np.array([rot.x, rot.y, rot.z, rot.w]))
        self.get_logger().info(
            f'resolved extrinsic {self.lio_body_frame} -> {self.base_frame}: '
            f't=[{tr.x:.3f} {tr.y:.3f} {tr.z:.3f}]')
        return True

    def on_odom(self, msg: Odometry):
        if not self.lookup_extrinsic():
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Pose of the LIO body frame in FAST-LIO's camera_init frame.
        t_init_body = make_tf(
            np.array([p.x, p.y, p.z]),
            np.array([q.x, q.y, q.z, q.w]))

        # Compose in the extrinsic to get base_link's pose.
        t_init_base = t_init_body @ self.t_body_base

        # Re-origin: define odom so the first sample lands at identity.
        if self.zero_origin and self.t_correct is None:
            self.t_correct = invert_tf(t_init_base)
            self.get_logger().info('origin captured; odom frame initialised')
        if self.t_correct is not None:
            t_odom_base = self.t_correct @ t_init_base
        else:
            t_odom_base = t_init_base

        if self.two_d_mode:
            t_odom_base = flatten_2d(t_odom_base)

        trans, quat = split_tf(t_odom_base)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = float(trans[0])
        out.pose.pose.position.y = float(trans[1])
        out.pose.pose.position.z = float(trans[2])
        out.pose.pose.orientation.x = float(quat[0])
        out.pose.pose.orientation.y = float(quat[1])
        out.pose.pose.orientation.z = float(quat[2])
        out.pose.pose.orientation.w = float(quat[3])
        # Twist from FAST-LIO is in the body frame and is not rotated here;
        # nav2 does not consume it, but do not trust it downstream as-is.
        out.pose.covariance = msg.pose.covariance
        self.pub.publish(out)

        if self.publish_tf:
            tfs = TransformStamped()
            tfs.header.stamp = msg.header.stamp
            tfs.header.frame_id = self.odom_frame
            tfs.child_frame_id = self.base_frame
            tfs.transform.translation.x = float(trans[0])
            tfs.transform.translation.y = float(trans[1])
            tfs.transform.translation.z = float(trans[2])
            tfs.transform.rotation.x = float(quat[0])
            tfs.transform.rotation.y = float(quat[1])
            tfs.transform.rotation.z = float(quat[2])
            tfs.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
