import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'trov'
    pkg_share = get_package_share_directory(package_name)

    # ── Launch Arguments ──────────────────────────────────────────────────
    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        #default_value='adibatla_indoor_box',
        default_value='pre_meerut',
        description='Map file name without .yaml extension (e.g. indoor3, testin1)'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto start nav2 lifecycle stack'
    )

    # ── Substitutions ─────────────────────────────────────────────────────
    map_name  = LaunchConfiguration('map_name')
    autostart = LaunchConfiguration('autostart')

    # Dynamically build: <pkg_share>/maps/<map_name>.yaml
    map_yaml_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'maps',
        [map_name, '.yaml']
    ])

    amcl_file = os.path.join(pkg_share, 'config', 'indoor', 'amcl_params.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes_localization = ['map_server', 'amcl']

    # ── Nodes ─────────────────────────────────────────────────────────────

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
            'range_max':        15.0,
            'use_inf':          True,
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/points'),
            ('scan',     '/scan'),
        ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time':  False},
            {'yaml_filename': map_yaml_file},
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_file, {'use_sim_time': False}],
        remappings=remappings,
    )

    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart':    autostart},
            {'node_names':   lifecycle_nodes_localization},
            {'use_sim_time': False},
        ],
    )

    # ── Launch Description ────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(localization_lifecycle_manager)

    return ld