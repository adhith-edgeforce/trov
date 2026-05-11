from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
  package_name = 'trov'
  pkg_share = get_package_share_directory(package_name)

  robot_localization_params_file = os.path.join(
    pkg_share, "config", "outdoor", "dual_ekf_navsat_params_hybrid.yaml")

  # ── EKF1: odom→base_link ──────────────────────────────────────────────────
  # Fuses:
  #   twist0: /mavros/local_position/velocity_body  (body-frame vx,vy)
  #   imu0:   /imu/data                             (absolute yaw + yaw rate)
  # Publishes: odometry/filtered/local  +  TF odom→base_link
  #
  # velocity_body is in base_link frame so vx=forward, vy=lateral.
  # No coordinate frame mismatch → robot moves in correct direction.
  ekf_filter_node_odom = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_odom",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': False}],
    remappings=[
      ("odometry/filtered", "odometry/filtered/local"),
    ],
  )

  # ── EKF2: map→odom ────────────────────────────────────────────────────────
  # Fuses:
  #   odom0: /odometry/gps  (GPS position in map frame from navsat_transform)
  #   imu0:  /imu/data      (absolute yaw + yaw rate)
  # Publishes: odometry/filtered/global  +  TF map→odom
  ekf_filter_node_map = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_map",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': False}],
    remappings=[
      ("odometry/filtered", "odometry/filtered/global"),
    ],
  )

  # ── navsat_transform ───────────────────────────────────────────────────────
  # Converts GPS fix → cartesian odometry using hardcoded datum.
  #
  # KEY: subscribes to odometry/filtered/global (EKF2, world_frame=map).
  # This makes navsat publish /odometry/gps with frame_id=map.
  # EKF2 then receives GPS already in map frame → no TF lookup → no loop.
  #
  # If navsat subscribed to local EKF (world_frame=odom), /odometry/gps
  # would be in odom frame, EKF2 would look up odom→map TF (which it
  # publishes itself) → circular feedback → position explosion.
  navsat_transform_node = Node(
    package="robot_localization",
    executable="navsat_transform_node",
    name="navsat_transform",
    output="screen",
    parameters=[robot_localization_params_file, {'use_sim_time': False}],
    remappings=[
      ("imu",               "/imu/data"),
      ("gps/fix",           "/mavros/global_position/global"),
      ("odometry/gps",      "odometry/gps"),
      ("odometry/filtered", "odometry/filtered/global"),  # ← EKF2 output (map frame)
    ],
  )

  return LaunchDescription([
    ekf_filter_node_odom,
    ekf_filter_node_map,
    navsat_transform_node,
  ])