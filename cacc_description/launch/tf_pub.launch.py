import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'cacc_description' # Replace with your package name
    urdf_file_name = 'robot.urdf.xacro' # Replace with your URDF file

    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file_name)
    robot_desc = xacro.process_file(urdf_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
    ])