# launch rslidar_sdk BEFORE this node launches, so that /points and
# /rslidar_imu_data are available for subscription.
# this launch file only generates the odom->base_link TF.
#
# OUTDOOR port of launch/indoor/fast_lio_odometry_indoor.launch.py, and a
# DROP-IN REPLACEMENT for icp_odometry_outdoor.launch.py
#   same output topic  : /odom   (nav_msgs/Odometry)
#   same frames        : odom -> base_link
#   same TF edge       : odom -> base_link
#   same rate          : ~10 Hz (lidar frame rate)
#   same extra node    : pointcloud_to_laserscan (/points -> /scan)
# so navigation_outdoor.launch.py, outdoor_lidar_localization_pinned.launch.py
# and drive_outdoor need no changes at all.
#
# NOTE: the built-in RS-Airy IMU is used, NOT wit_ros2_imu.
#
# Differences from icp_odometry_outdoor that are worth knowing:
#   * /odom is flattened to x / y / yaw (two_d_mode). icp_odometry published
#     full 6-DoF. NDT localization owns the 3D map -> base_link pose and
#     publishes map -> odom as the difference, so z / roll / pitch simply
#     move into map -> odom. Set two_d_mode to False below for 6-DoF odom.
#   * /odom carries pose only (twist is zero). icp_odometry filled twist.
#     Nothing in the outdoor stack reads it: RPP has velocity-scaled
#     lookahead off, velocity_smoother is OPEN_LOOP, and drive_outdoor
#     differentiates the pose itself.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'trov'
    pkg_share = get_package_share_directory(package_name)

    fastlio_params = os.path.join(
        pkg_share, 'config', 'outdoor', 'fast_lio_outdoor.yaml')

    # ── lidar_link -> imu_link static TF ─────────────────────────────────
    # Identical to indoor: FAST-LIO's `body` frame is the RS-Airy IMU frame,
    # rotated 180 deg from the lidar optical frame. See the indoor launch
    # file for the derivation and the gravity-vector verification.
    lidar_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_imu_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.7071068', '--qy', '-0.7071068', '--qz', '0', '--qw', '0',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    # ── FAST-LIO2 ────────────────────────────────────────────────────────
    # Input:  /points            (PointCloud2, XYZIRT) <- rslidar_sdk
    #         /rslidar_imu_data  (Imu, 200 Hz)         <- RS-Airy built-in IMU
    # Output: /Odometry          (camera_init -> body)
    #
    # Its internal TF broadcast is DISABLED in the fork's laserMapping.cpp so
    # that lio_odom_bridge owns odom->base_link.
    fast_lio = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        name='laser_mapping',
        output='screen',
        parameters=[
            fastlio_params,
            {'use_sim_time': False},
        ],
    )

    # ── odometry bridge ──────────────────────────────────────────────────
    # Converts FAST-LIO's IMU pose in its own world frame into base_link's
    # pose in an odom frame that starts at the robot's initial pose.
    lio_bridge = Node(
        package='trov',
        executable='lio_odom_bridge.py',
        name='lio_odom_bridge',
        output='screen',
        parameters=[{
            'input_topic': '/Odometry',
            'output_topic': '/odom',
            'lio_body_frame': 'imu_link',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'publish_tf': True,
            'two_d_mode': True,
            'zero_origin': True,
        }],
    )

    # ── pointcloud_to_laserscan ──────────────────────────────────────────
    # Copied unchanged from icp_odometry_outdoor.launch.py so /scan keeps
    # being published exactly as before.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[{
            'use_sim_time':        False,
            'target_frame':        'base_link',
            'transform_tolerance': 1.0,
            'min_height': 0.3,
            'max_height': 0.8,
            'angle_min':       -1.57,
            'angle_max':        1.57,
            'angle_increment':  0.00873,
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
        lidar_to_imu_tf,
        fast_lio,
        lio_bridge,
        pointcloud_to_laserscan,
    ])
