#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import ClockType # Import ClockType
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math
# from geometry_msgs.msg import Quaternion # Not strictly needed

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.sensor_ok = False # Initialize to False

        try:
            self.sensor = mpu6050(0x68)
            self.get_logger().info("MPU6050 initialized successfully.")
            self.sensor_ok = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
            self.get_logger().error("MPU6050 node will not publish data.")
            # Optional: make the node exit if sensor init fails
            # rclpy.shutdown() # This can be abrupt if called from __init__
            # return

        if self.sensor_ok:
            timer_period = 0.05  # seconds (for 20 Hz)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f"Publishing IMU data at ~{1.0/timer_period:.1f} Hz.")
        else:
            self.timer = None

    def timer_callback(self):
        if not self.sensor_ok:
            return

        try:
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()
        except Exception as e:
            self.get_logger().warn(f"Failed to read MPU6050 data during callback: {e}")
            return

        imu_msg = Imu()
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"

        imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
        imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
        imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']

        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance[0] = -1.0

        accel_cov_diagonal = 0.01
        gyro_cov_diagonal = 0.0001

        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = accel_cov_diagonal
        imu_msg.linear_acceleration_covariance[4] = accel_cov_diagonal
        imu_msg.linear_acceleration_covariance[8] = accel_cov_diagonal

        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = gyro_cov_diagonal
        imu_msg.angular_velocity_covariance[4] = gyro_cov_diagonal
        imu_msg.angular_velocity_covariance[8] = gyro_cov_diagonal

        self.publisher_.publish(imu_msg)

        self.get_logger().info(
            'IMU data published.')

def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        node = MPU6050Node()
        if not node.sensor_ok:
            node.get_logger().error("MPU6050 sensor failed to initialize. Node will exit.")
            # Node is created, but sensor_ok is false. We'll let it get to destroy_node.
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        if node: # Check if node object was successfully created
            node.get_logger().info('MPU6050 node interrupted by Ctrl+C.')
    except Exception as e:
        if node: # Check if node object was successfully created
            node.get_logger().error(f"Unhandled exception in MPU6050 node: {e}")
        else: # Exception might have occurred during __init__ before node was fully assigned
            print(f"Unhandled exception during MPU6050Node instantiation: {e}")
    finally:
        if node is not None and rclpy.ok(): # Ensure node exists and rclpy hasn't been shutdown by another thread
            node.destroy_node()
        if rclpy.ok(): # Check if rclpy context is still valid
            rclpy.shutdown()

if __name__ == '__main__':
    main()
