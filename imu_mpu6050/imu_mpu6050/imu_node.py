#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math
import tf_transformations
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
from rclpy.time import Time as ROS2Time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.sensor = mpu6050(0x68)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        imu_msg = Imu()

        # Timestamp
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
        imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
        imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']

        # Orientation unknown (set to 0 and covariance high)
        imu_msg.orientation = Quaternion()
        imu_msg.orientation_covariance[0] = -1.0  # Indicate unknown orientation

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

