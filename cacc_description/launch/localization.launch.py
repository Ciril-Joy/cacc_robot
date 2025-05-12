import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess # For running external commands if needed

def generate_launch_description():
    pkg_name = 'cacc_description' # Replace with your package name
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')

    return LaunchDescription([
        # Node( # For publishing static TFs if not done elsewhere
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      name='static_tf_pub_gps',
        #      arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'gps_link'], # XYZ RPY Parent Child
        #  ),
        #  Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      name='static_tf_pub_imu',
        #      arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        #  ),
        # Node( # If you have robot_state_publisher, this is better
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    output='screen',
        #    parameters=[{'robot_description': open(os.path.join(get_package_share_directory('cacc_description'),'urdf','robot.urdf.xacro')).read(),
        #                 'use_sim_time': False}] # Set use_sim_time accordingly
        # ),

        # EKF for odom frame (fusing IMU and wheel odom if available)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local', # Must match the node name in the yaml
            output='screen',
            parameters=[os.path.join(config_dir, 'ekf_local.yaml')],
            remappings=[('odometry/filtered', 'odometry/local_ekf')] # Remap output
        ),

        # Navsat Transform Node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(config_dir, 'navsat_transform.yaml')],
            ros_arguments=['--log-level', 'debug'], # ADD THIS            
            remappings=[('imu', '/imu/data'), # Default is /imu, remap if yours is /imu/data
                        ('gps/fix', '/fix'),
                        ('gps/filtered', '/gps/filtered'), # Output for visualization
                        ('odometry/gps', '/odometry/gps'), # Output for global EKF
                        ('odometry/filtered', 'odometry/global_ekf_navsat_input')] # Input from global EKF
                                                                                   # This creates a loop for datum correction
        ),

        # EKF for map frame (fusing local EKF output and GPS-derived odometry)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global', # Must match the node name in the yaml
            output='screen',
            parameters=[os.path.join(config_dir, 'ekf_global.yaml')],
            remappings=[('odometry/filtered', 'odometry/global_ekf'), # Final fused output
                        # Ensure odom0 in ekf_global.yaml points to 'odometry/local_ekf'
                        # Ensure odom1 in ekf_global.yaml points to '/odometry/gps'
                        ]
        ),
    ])