import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'cacc_description' # Replace with your package name
    pkg_dir = get_package_share_directory(pkg_name)

    # For robot_state_publisher and URDF
    description_launch_file = os.path.join(pkg_dir, 
                                           'launch', 'tf_pub.launch.py') # Your TF publisher launch

    localization_launch_file = os.path.join(pkg_dir, 'launch', 'localization.launch.py')
    slam_launch_file = os.path.join(pkg_dir, 'launch', 'slam.launch.py')
    navigation_launch_file = os.path.join(pkg_dir, 'launch', 'navigation.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_file),
            # Optionally pass use_sim_time if needed by robot_state_publisher
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_file)
        ),
        ## Add your robot's gps
        #Node(
        #    package='caccbot_gps',
        #    executable='cacc_gps', # Translates /cmd_vel to motor commands
        #    name='cacc_gps',
        #    output='screen'
        #),
#
        ## Add your robot's IMU
        #Node(
        #    package='imu_mpu6050',
        #    executable='imu_node', # Translates /cmd_vel to motor commands
        #    name='ackermann_controller',
        #    output='screen'
        #),
#
        # Add your robot's low-level controller node here
        # Node(
        #     package='your_robot_driver_pkg',
        #     executable='ackermann_controller_node', # Translates /cmd_vel to motor commands
        #     name='ackermann_controller',
        #     output='screen'
        # ),

        # Add your robot's low-level controller node here
        # Node(
        #     package='your_robot_driver_pkg',
        #     executable='ackermann_controller_node', # Translates /cmd_vel to motor commands
        #     name='ackermann_controller',
        #     output='screen'
        # ),
    ])