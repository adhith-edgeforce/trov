import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'trov'
  pkg_share = get_package_share_directory(package_name)
  
  nav2_params_file = os.path.join(pkg_share, 'config', 'outdoor', 'nav2_params_outdoor.yaml')
  
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
  
  lifecycle_nodes_nav2 = [
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'velocity_smoother', 'bt_navigator', 
    'waypoint_follower'#, 'collision_monitor'
  ]
  
  # Declare launch arguments
  declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time')
  declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Auto start nav2 stack')

  # Start Nav2 Nodes
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
    # remappings=remappings,
  )
  
  nav2_planner = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    # remappings=remappings,
  )
  
  nav2_behaviors = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    # remappings=remappings,
  )
  
  nav2_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    #remappings=remappings,
  )
  
  nav2_waypoint_follower = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    # remappings=remappings,
  )
  
  nav2_velocity_smoother = Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='velocity_smoother',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    # remappings=remappings,
  )
  
  nav2_collision_monitor = Node(
    package='nav2_collision_monitor',
    executable='collision_monitor',
    name='collision_monitor',
    output='screen',
    parameters=[nav2_params_file, {'use_sim_time': False}],
    # remappings=remappings,
  )

  nav2_lifecycle_nodes_manager = Node(  
    # Your existing nav2_lifecycle_nodes_manager
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{'autostart': True}, 
                {'node_names': lifecycle_nodes_nav2}, 
                {'use_sim_time': False}],
  )
  
  ld = LaunchDescription()

  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_autostart_cmd)

  #ld.add_action(localization)
  #ld.add_action(dual_ekf)
  ld.add_action(nav2_controller)
  ld.add_action(nav2_smoother)
  ld.add_action(nav2_planner)
  ld.add_action(nav2_behaviors)
  ld.add_action(nav2_bt_navigator)
  ld.add_action(nav2_waypoint_follower)
  ld.add_action(nav2_velocity_smoother)
  #ld.add_action(nav2_collision_monitor)
  ld.add_action(nav2_lifecycle_nodes_manager)

  return ld

