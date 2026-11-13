# Outdoor PCD mapping with lidarslam, tuned for the forward-tilted RS-Airy.
#
# New-file alternative to `ros2 launch lidarslam lidarslam_outdoor.launch.py`
# (which start_mapping_outdoor.sh uses). Differences:
#   * reads /points straight from rslidar_sdk instead of /points_raw, so the
#     rslidar_to_lio converter node is not needed for mapping
#   * loads config/outdoor/lidarslam_outdoor_tuned.yaml (see that file for
#     what changed and why)
#   * RViz is optional:  rviz:=false
#
# Save the map exactly as before:
#   ros2 service call /map_save std_srvs/srv/Empty
# map.pcd is written to the directory this launch was started from.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('trov'),
        'config', 'outdoor', 'lidarslam_outdoor_tuned.yaml')

    rviz_config = os.path.join(
        get_package_share_directory('lidarslam'), 'rviz', 'mapping.rviz')

    mapping = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[params],
        remappings=[
            ('/input_cloud', '/points'),
            # only read when use_imu is true (it is false in the tuned yaml)
            ('/imu', '/imu/data'),
        ],
        output='screen',
    )

    graphbasedslam = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[params],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the lidarslam mapping view'),
        mapping,
        graphbasedslam,
        rviz,
    ])
