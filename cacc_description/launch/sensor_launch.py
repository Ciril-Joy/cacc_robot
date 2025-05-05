from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU driver node (MPU6050)
        Node(
            package='imu_mpu6050',  # Package that contains the MPU6050 driver
            executable='imu_node',  # Executable node that runs the MPU6050 driver
            name='imu_node',
            output='screen',
            remappings=[
                ('/imu/data', '/imu')  # Ensures consistency for robot_localization
            ]
        ),

        # GPS driver node (example, replace with actual package)
        Node(
            package='caccbot_gps',  # Replace with actual package name
            executable='cacc_gps',   # Replace with actual node executable
            name='gps_node',
            output='screen',
            remappings=[
                ('/gps/fix', '/fix')  # Standard ROS GPS topic
            ]
        )
    ])
