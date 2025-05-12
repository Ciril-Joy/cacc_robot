import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'cacc_description' # Replace with your package name
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    slam_params_file = os.path.join(config_dir, 'slam_online_async.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[ # Ensure scan topic is correct
            ('/scan', '/scan') # Replace /scan with your actual lidar scan topic if different
        ])

    return LaunchDescription([
        declare_use_sim_time_argument,
        start_async_slam_toolbox_node
    ])