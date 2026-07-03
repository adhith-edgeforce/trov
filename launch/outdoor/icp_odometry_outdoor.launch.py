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

  return LaunchDescription([
    icp_odometry
  ])
