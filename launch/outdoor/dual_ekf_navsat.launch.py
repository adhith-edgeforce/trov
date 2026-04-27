# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription
# from launch.substitutions import Command,  LaunchConfiguration
# from launch_ros.actions import Node
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# import os

# def generate_launch_description():
#   package_name = 'trov'
#   pkg_share = get_package_share_directory(package_name)
  
#   robot_localization_params_file = os.path.join(pkg_share, "config", "outdoor", "dual_ekf_navsat_params.yaml")
  
#   rviz_file = os.path.join(pkg_share, 'rviz', 'dual_ekf_localization.rviz')

#   ekf_filter_node_odom = Node(
#     package="robot_localization",
#     executable="ekf_node",
#     name="ekf_filter_node_odom",
#     output="screen",
#     parameters=[robot_localization_params_file, {'use_sim_time': False}],
#     remappings=[("odometry/filtered", "odometry/filtered/local")],
#   ) 

#   ekf_filter_node_map = Node(
#     package="robot_localization",
#     executable="ekf_node",
#     name="ekf_filter_node_map",
#     output="screen",
#     parameters=[robot_localization_params_file, {'use_sim_time': False}],
#     remappings=[("odometry/filtered", "odometry/filtered/global")],
#   ) 

#   navsat_transform_node = Node(
#     package="robot_localization",
#     executable="navsat_transform_node",
#     name="navsat_transform",
#     output="screen",
#     parameters=[robot_localization_params_file, {'use_sim_time': False}],
#     remappings=[
#       ("imu", "/imu/data"),
#       ("gps/fix", "/mavros/global_position/global"),
#       ("odometry/gps", "odometry/gps"),
#       ("odometry/filtered", "odometry/filtered/global"),
#     ],
#   )

#   rviz_node = Node(
#     package='rviz2',
#     executable='rviz2',
#     name='rviz2',
#     arguments=['-d', rviz_file],
#     output='screen',
#     parameters=[{'use_sim_time': False}],
#   )

#   return LaunchDescription([
#     ekf_filter_node_odom,
#     ekf_filter_node_map,
#     navsat_transform_node,
#     # rviz_node
#   ])

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
  package_name = 'trov'
  pkg_share = get_package_share_directory(package_name)

  robot_localization_params_file = os.path.join(
    pkg_share, "config", "outdoor", "dual_ekf_navsat_params_claude.yaml")

  rviz_file = os.path.join(pkg_share, 'rviz', 'dual_ekf_localization.rviz')

  # ── EKF 1: odom → base_link (local, drift-prone) ──────────────────────────
  # Fuses: /odom (GPS-derived local pose) + /imu/data
  # Publishes: odometry/filtered/local  +  TF odom → base_link
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

  # ── EKF 2: map → odom (global, GPS-corrected) ─────────────────────────────
  # Fuses: /odometry/gps (from navsat_transform) + /imu/data
  # Publishes: odometry/filtered/global  +  TF map → odom
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
  # Converts GPS fix → cartesian odometry in the map frame
  # MUST subscribe to the LOCAL odom EKF output to get robot yaw
  # Publishes: /odometry/gps  (consumed by ekf_filter_node_map)
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
      # ↓ KEY FIX: navsat needs the LOCAL ekf output for yaw, NOT global
      ("odometry/filtered", "odometry/filtered/local"),
    ],
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file],
    output='screen',
    parameters=[{'use_sim_time': False}],
  )

  return LaunchDescription([
    ekf_filter_node_odom,    # odom → base_link TF
    ekf_filter_node_map,     # map → odom TF
    navsat_transform_node,   # GPS fix → /odometry/gps
    #rviz_node
  ])