import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'trov'
    pkg_share = get_package_share_directory(package_name)

    nav2_params_file = os.path.join(pkg_share, 'config', 'indoor', 'nav2_params_indoor.yaml')

    lifecycle_nodes_nav2 = [
      'controller_server', 'planner_server', 'behavior_server',
      'velocity_smoother', 'bt_navigator', 'waypoint_follower',
      'collision_monitor',# 'smoother_server',   # smoother last
    ]

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto start nav2 stack')

    # ── Nav2 Nodes ──────────────────────────────────────────────────────────

    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    nav2_collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': False}],
    )

    # ── Lifecycle Manager ────────────────────────────────────────────────────
    # Delayed by 5s to give all nodes time to register their lifecycle
    # services before the manager starts the configure sequence.
    # service_availability_timeout gives extra headroom per-node.
    # bond_timeout prevents premature failure on slow Jetson startup.

    # Change the lifecycle manager Node parameters:
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            nav2_params_file,
            {'autostart': True},
            {'node_names': lifecycle_nodes_nav2},
            {'use_sim_time': False},
            {'bond_timeout': 0.0},                   # ← was 4.0
            {'service_availability_timeout': 60.0},  # ← was 10.0
            {'attempt_respawn_reconnection': True},
        ],
    )

    # Change the timer delay:
    nav2_lifecycle_manager_delayed = TimerAction(
        period=10.0,   # ← was 5.0
        actions=[nav2_lifecycle_manager]
    )

    # ── LaunchDescription ────────────────────────────────────────────────────

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(nav2_controller)
    #ld.add_action(nav2_smoother)
    ld.add_action(nav2_planner)
    ld.add_action(nav2_behaviors)
    ld.add_action(nav2_bt_navigator)
    ld.add_action(nav2_waypoint_follower)
    ld.add_action(nav2_velocity_smoother)
    ld.add_action(nav2_collision_monitor)
    ld.add_action(nav2_lifecycle_manager_delayed)  # ← delayed, not immediate

    return ld