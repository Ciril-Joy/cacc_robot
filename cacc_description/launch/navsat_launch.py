from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navsat_transform_params = os.path.join(
        get_package_share_directory('cacc_description'), 'config', 'navsat_transform.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_transform_params]
        )
    ])
