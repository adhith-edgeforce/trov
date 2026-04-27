import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'trov'
  pkg_share = get_package_share_directory(package_name)

  slam_params_file= os.path.join(pkg_share, 'config', 'indoor', 'mapper_params_online_async.yaml')
  
  rviz_launch = LaunchConfiguration('rviz_launch', default='true') 
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
    output='screen',
    parameters=[{'use_sim_time': False}],
    condition=IfCondition(rviz_launch),
  )
  
  pointcloud_to_laserscan = Node(
      package='pointcloud_to_laserscan',
      executable='pointcloud_to_laserscan_node',
      name='pointcloud_to_laserscan',
      parameters=[{
        'use_sim_time': False,
        'target_frame': 'base_link',
        'transform_tolerance': 0.1,
        'min_height': 0.25,
        'max_height': 2.0,
        'angle_min': -3.14159,
        'angle_max':  3.14159,
        'angle_increment': 0.00436,
        'scan_time': 0.1,
        'range_min': 0.5,
        'range_max': 15.0,
        'use_inf': False,
      }],
      remappings=[('cloud_in', '/points'), ('scan', '/scan')],
      output='screen'
  )


  start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
  )


  return LaunchDescription([
    #rviz_node,
    start_async_slam_toolbox_node,
    pointcloud_to_laserscan,
  ])
