import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'cacc_description' # Replace with your package name
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local', default='true') # For SLAM
    params_file = LaunchConfiguration('params_file', 
                                      default=os.path.join(get_package_share_directory(pkg_name),'config','nav2_params.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local',
            default_value='true',
            description='Whether to subscribe to the map topic for SLAM'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory(pkg_name), 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'false', # Autostart Nav2 lifecycle nodes
                'map_subscribe_transient_local': map_subscribe_transient_local,
                'launch_robot_state_publisher': 'false',
                # 'robot_description': '' # TRY ADDING THIS AS A TEST - it might break things but tells us if it's being read
            }.items(),
        )
    ])