from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'taskset', '-c', '4,5,6,7',        # ← PIN TO CORE 7
                'ros2', 'launch',
                'lidar_localization_ros2',
                'nav2_lidar_localization.launch.py',
                'cloud_topic:=/points',
                'imu_topic:=/imu/data',
                #'odom_topic:=/odom',
                'lidar_frame_id:=lidar_link',
                'publish_lidar_tf:=false',
                'publish_imu_tf:=false',
                'use_sim_time:=false',
                'localization_param_dir:=/data/trov_ws/src/trov/config/indoor/indoor_localization.yaml',
            ],
            output='screen',
        )
    ])