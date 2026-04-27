from launch import LaunchDescription
import launch.actions
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'trov'
    pkg_share = get_package_share_directory(package_name)

    mapviz_config_file = os.path.join(pkg_share, "config", "outdoor", "gps_wpf_demo.mvc")

    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        output='screen',
        parameters=[{"config": mapviz_config_file}, {"use_sim_time": False}]
    )

    initialize_origin = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ],
        remappings=[
            ("fix", "/mavros/global_position/global"),
        ]
    )

    # Fixed: Using manual origin with your GPS coordinates
    initialize_origin_manual = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'local_xy_frame': 'map'},
            {'local_xy_origin': 'manual'},
            {'local_xy_latitude': 17.249742},
            {'local_xy_longitude': 78.5568129},
            {'local_xy_altitude': 0.0},
        ],
    )

    # Single map to origin transform (removed duplicate swri_transform)
    map_to_origin = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_origin",
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    base_link_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_map",
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
    )

    return LaunchDescription([
        initialize_origin_manual,      # Start origin first
        map_to_origin,
        base_link_to_map,
        mapviz_node,            # Mapviz last so transforms are ready
    ])