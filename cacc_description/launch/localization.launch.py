import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess # Not used

def generate_launch_description():
    pkg_name = 'cacc_description'
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            parameters=[os.path.join(config_dir, 'ekf_local.yaml')],
            remappings=[('odometry/filtered', 'odometry/local_ekf')]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(config_dir, 'navsat_transform.yaml')],
            ros_arguments=['--log-level', 'debug'], # Good for debugging
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/fix'),
                ('odometry/gps', '/odometry/gps'),       # Output of navsat_transform_node
                # This INPUT 'odometry/filtered' for navsat_transform_node needs to be
                # the OUTPUT of ekf_filter_node_global.
                # ekf_filter_node_global's output is remapped to '/odometry/global_ekf'.
                ('odometry/filtered', '/odometry/global_ekf') # CORRECTED: This is for use_odometry_yaw=true
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            output='screen',
            parameters=[os.path.join(config_dir, 'ekf_global.yaml')],
            remappings=[('odometry/filtered', 'odometry/global_ekf')] # This node outputs on /odometry/global_ekf
        ),
    ])
