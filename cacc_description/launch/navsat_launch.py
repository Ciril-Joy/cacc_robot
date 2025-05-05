from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False,
                'use_manual_datum': False,
                'broadcast_utm_transform': True
            }],
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/fix'),
                ('/odometry/gps', '/odometry/gps')
            ]
        )
    ])
