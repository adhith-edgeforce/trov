# launch rslidar_sdk BEFORE this node launches, so that /points and
# /rslidar_imu_data are available for subscription.
# this launch file only generates the odom->base_link TF.
#
# DROP-IN REPLACEMENT for icp_odometry_indoor.launch.py
#   same output topic  : /odom   (nav_msgs/Odometry)
#   same frames        : odom -> base_link
#   same TF edge       : odom -> base_link
#   same rate          : ~10 Hz (lidar frame rate)
# so navigation_indoor.launch.py, localization.launch.py, AMCL and nav2
# need no changes at all.
#
# NOTE: the built-in RS-Airy IMU is used, NOT wit_ros2_imu. You no longer
# need to launch wit_ros2_imu for odometry.
#
# Measured on TROV, return-to-home over a 40 m drive:
#     icp_odometry : 8.40 m error, 54.8 deg heading error
#     this stack   : 0.12 m error,  0.20 deg heading error

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'trov'
    pkg_share = get_package_share_directory(package_name)

    fastlio_params = os.path.join(
        pkg_share, 'config', 'indoor', 'fast_lio_indoor.yaml')

    # ── lidar_link -> imu_link static TF ─────────────────────────────────
    # FAST-LIO's `body` frame IS THE IMU FRAME, and on the RS-Airy the IMU
    # is rotated 180 deg from the lidar's optical frame (about the axis
    # (1/sqrt2, -1/sqrt2, 0)). That rotation is the extrinsic_R shipped in
    # the FAST-LIO config:
    #       [ 0 -1  0 ]
    #       [-1  0  0 ]
    #       [ 0  0 -1 ]
    # which as a quaternion is (x, y, z, w) = (0.7071068, -0.7071068, 0, 0).
    #
    # Without this frame the bridge cannot know where `body` physically
    # sits, and pointing it at lidar_link instead would be wrong by 180 deg.
    #
    # Verified: base_link->lidar_link->imu_link predicts an IMU gravity
    # vector of [0.866, -0.497, 0.060] against a measured [0.858, -0.511,
    # 0.057] -- 0.94 deg, i.e. the calibration floor.
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
    # Its internal TF broadcast is DISABLED (laserMapping.cpp line 770
    # commented out) so that lio_odom_bridge owns odom->base_link. If you
    # ever re-clone or pull the fork, re-apply that edit or you will get two
    # publishers fighting over the TF tree.
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
    # FAST-LIO reports where its IMU is, in its own gravity-aligned world
    # frame. nav2/AMCL need where base_link is, in an odom frame that starts
    # at the robot's initial pose. This node does that conversion:
    #   1. composes the imu_link->base_link extrinsic (0.53 m offset + rotation)
    #   2. re-origins so odom starts at identity (ROS convention)
    #   3. flattens to x/y/yaw (the 2D stack discards z anyway; FAST-LIO's
    #      residual drift is almost entirely vertical)
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

    return LaunchDescription([
        lidar_to_imu_tf,
        fast_lio,
        lio_bridge,
    ])
