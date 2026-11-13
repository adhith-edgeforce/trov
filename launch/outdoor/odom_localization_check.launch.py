# Standalone check of OUTDOOR odometry + localization, with RViz.
#
# Starts only what odometry and localization need:
#   1. rslidar_sdk          (lidar driver + base_link -> lidar_link static TF)
#   2. FAST-LIO2 odometry   (fast_lio_odometry_outdoor.launch.py: /odom, odom -> base_link)
#   3. NDT localization     (outdoor_lidar_localization_pinned.launch.py:
#                            map from outdoor_localization.yaml map_path, map -> odom)
#   4. RViz                 (rviz/odom_localization_check.rviz)
# No Nav2, MAVROS, drive bridge, cameras or UI nodes.
#
# STOP THE SERVICES FIRST — this starts its own lidar driver and odometry, and
# two copies fight over the lidar ports and the TF tree:
#     sudo systemctl stop trov_outdoor.service trov.service
#     ros2 launch trov odom_localization_check.launch.py
#
# Then in RViz: "2D Pose Estimate" -> click where the robot is on the map and
# drag in the direction it faces. Localization does not start until this is done
# (set_initial_pose: false). The map origin is where the mapping drive started.
#
# Arguments:  rviz:=false   (e.g. over SSH without a display)
#
# The lidar driver is started as its own `ros2 launch` process on purpose:
# humble_start.py sets RMW_IMPLEMENTATION=rmw_cyclonedds_cpp inside the launch
# process, so including it here would switch every node in this file to
# Cyclone DDS too. Running it separately matches trov_outdoor_launch_files.sh.

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _warn_if_services_running(context, *args, **kwargs):
    actions = []
    for svc in ('trov_outdoor.service', 'trov.service'):
        try:
            state = subprocess.run(['systemctl', 'is-active', svc],
                                   capture_output=True, text=True).stdout.strip()
        except Exception:
            state = 'unknown'
        if state == 'active':
            actions.append(LogInfo(msg=(
                f'WARNING: {svc} is running. It already starts a lidar driver and '
                f'odometry; stop it first: sudo systemctl stop {svc}')))
    return actions


def generate_launch_description():
    trov_share = get_package_share_directory('trov')
    rviz_config = os.path.join(trov_share, 'rviz', 'odom_localization_check.rviz')

    # 1. lidar driver (separate process, see header)
    lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'rslidar_sdk', 'humble_start.py'],
        output='screen',
    )

    # 2. FAST-LIO2 odometry — started after the driver has had time to come up
    odometry = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            trov_share, 'launch', 'outdoor', 'fast_lio_odometry_outdoor.launch.py'))),
    ])

    # 3. NDT localization on the map configured in outdoor_localization.yaml
    localization = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            trov_share, 'launch', 'outdoor', 'outdoor_lidar_localization_pinned.launch.py'))),
    ])

    # 4. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_odom_localization_check',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        OpaqueFunction(function=_warn_if_services_running),
        lidar,
        odometry,
        localization,
        rviz,
    ])
