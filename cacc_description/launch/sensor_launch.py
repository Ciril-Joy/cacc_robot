from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navsat_transform_params = os.path.join(
        get_package_share_directory('cacc_description'), 'config', 'navsat_transform.yaml'
    )

    ekf_params = os.path.join(
        get_package_share_directory('cacc_description'), 'config', 'ekf.yaml'
    )

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
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_transform_params]
        ),
        Node(
            package='robot_localization',
            executable='ekf_filter_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]
        )
    ])
