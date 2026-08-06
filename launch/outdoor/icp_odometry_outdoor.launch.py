# launch the wit_ros2_imu and rslidar_sdk before this node launches, so that the necessary topics are available for subscription.
# this launch file is only can generate odom->base_link TF.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'trov'
  pkg_share = get_package_share_directory(package_name)

  icp_params      = os.path.join(pkg_share, 'config', 'outdoor', 'icp_odometry_outdoor.yaml')

  # ── icp_odometry ──────────────────────────────────────────────────────
    # Input:  /scan_cloud  (PointCloud2) ← remapped from /points
    #         /imu/data    (Imu)         ← used as motion guess between scans
    # Output: /odom        (Odometry)    + odom→base_link TF
    #
    # Key remappings:
    #   /points      → /scan_cloud   (icp_odometry's expected input topic)
    #   /imu/data    → /imu/data     (no change needed, shown for clarity)
  icp_odometry = Node(
    package='rtabmap_odom',
    executable='icp_odometry',
    name='icp_odometry',
    #prefix='taskset -c 6',  # <- pin to core 6
    output='screen',
    parameters=[
        icp_params,
        {
            'use_sim_time': False,
            'subscribe_scan': False,       # disable LaserScan subscription
            'subscribe_scan_cloud': True,  # enable PointCloud2 subscription
        }
    ],
    remappings=[
        ('scan_cloud', '/points'),
        ('imu', '/imu/data'),
        ('odom', '/odom'),
    ],
    arguments=['--ros-args', '--log-level', 'icp_odometry:=WARN']
  )

  pointcloud_to_laserscan = Node(
      package='pointcloud_to_laserscan',
      executable='pointcloud_to_laserscan_node',
      name='pointcloud_to_laserscan_node',
      output='screen',
      parameters=[{
          'use_sim_time':        False,
          'target_frame':        'base_link',
          'transform_tolerance': 1.0,

          # ── Height filter ─────────────────────────────────────────────
          # Scan slice is taken between min_height and max_height in
          # base_link frame (i.e. relative to the robot body).
          #
          # KEY CHANGE: max_height lowered from 1.5 → 0.8
          # This cuts the scan at shin/knee height, which means:
          #   - Walls still appear (they extend floor to ceiling)
          #   - People's torsos and upper bodies are excluded
          #   - People's feet/lower legs may still appear briefly
          #     but beam_skip in AMCL handles the residual noise
          #
          # If people's legs still cause AMCL jumps, lower to 0.5.
          # If walls start disappearing, raise back toward 1.0.
          'min_height': 0.3,     # was 0.1 — raise above floor reflections
          'max_height': 0.8,     # was 1.5 — lower to exclude torsos

          'angle_min':       -1.57,
          'angle_max':        1.57,
          'angle_increment':  0.00873,   # ~1 degree resolution
          'scan_time':        0.1,
          'range_min':        0.5,
          'range_max':        25.0,
          'use_inf':          True,
          'concurrency_level': 1,
      }],
      remappings=[
          ('cloud_in', '/points'),
          ('scan',     '/scan'),
      ],
  )

  return LaunchDescription([
    icp_odometry,
    pointcloud_to_laserscan
  ])
