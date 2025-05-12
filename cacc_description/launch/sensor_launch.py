from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='imu_mpu6050',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='caccbot_gps',
            executable='cacc_gps',
            name='cacc_gps',
            output='screen'
        ),
        
    ])