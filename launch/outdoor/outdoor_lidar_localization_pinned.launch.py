from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'taskset', '-c', '7',        # ← PIN TO CORE 7
                'ros2', 'launch',
                'lidar_localization_ros2',
                'nav2_lidar_localization.launch.py',
                'cloud_topic:=/points_raw',
                'imu_topic:=/imu/data',
                #'odom_topic:=/odom',
                'lidar_frame_id:=lidar_link',
                'publish_lidar_tf:=false',
                'publish_imu_tf:=false',
                'localization_param_dir:=/data/trov_ws/src/trov/config/outdoor/outdoor_localization.yaml',
            ],
            output='screen',
        )
    ])