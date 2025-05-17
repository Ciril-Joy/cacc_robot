#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050 # Assuming this is the adafruit-circuitpython-mpu6050 or similar
import math
from geometry_msgs.msg import Quaternion # Not strictly needed if you set x,y,z,w directly
# from builtin_interfaces.msg import Time # Not used
# from rclpy.time import Time as ROS2Time # Not used

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        try:
            self.sensor = mpu6050(0x68) # Make sure I2C address is correct
            self.get_logger().info("MPU6050 initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
            # Consider raising an exception or exiting if sensor init fails
            rclpy.shutdown() # Or handle more gracefully
            return

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz (1.0 / 0.05 = 20)

    def timer_callback(self):
        try:
            accel_data = self.sensor.get_accel_data() # Returns m/s^2
            gyro_data = self.sensor.get_gyro_data()   # Returns deg/s
        except Exception as e:
            self.get_logger().warn(f"Failed to read MPU6050 data: {e}")
            return

        imu_msg = Imu()

        # Timestamp
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link" # CORRECT

        # Angular velocity (rad/s) - MPU6050 library usually returns deg/s
        imu_msg.angular_velocity.x = math.radians(gyro_data['x']) # CORRECT CONVERSION
        imu_msg.angular_velocity.y = math.radians(gyro_data['y']) # CORRECT CONVERSION
        imu_msg.angular_velocity.z = math.radians(gyro_data['z']) # CORRECT CONVERSION

        # Linear acceleration (m/s^2) - MPU6050 library usually returns m/s^2
        imu_msg.linear_acceleration.x = accel_data['x'] # CORRECT
        imu_msg.linear_acceleration.y = accel_data['y'] # CORRECT
        imu_msg.linear_acceleration.z = accel_data['z'] # CORRECT

        # Orientation unknown (set to 0 and covariance high)
        # This is the part we need to address for navsat_transform_node
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0 # Identity quaternion

        # Covariances:
        # For unknown orientation, setting the first diagonal element to -1 is standard.
        # All other elements for orientation should be 0.
        imu_msg.orientation_covariance = [0.0] * 9 # Initialize with zeros
        imu_msg.orientation_covariance[0] = -1.0  # Indicate unknown/unreliable orientation

        # For angular velocity and linear acceleration, if you don't have specific
        # noise characteristics, provide small positive diagonal values.
        # Avoid 0.0 if the EKF is configured to use these, as 0 covariance means perfect.
        # If you *don't* want the EKF to use a particular axis, set its covariance very high.
        # For now, let's assume some small noise.
        default_lin_accel_cov = 0.01 # Example: (0.1 m/s^2)^2
        default_ang_vel_cov = 0.001 # Example: (approx 1.8 deg/s)^2

        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = default_lin_accel_cov # X
        imu_msg.linear_acceleration_covariance[4] = default_lin_accel_cov # Y
        imu_msg.linear_acceleration_covariance[8] = default_lin_accel_cov # Z

        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = default_ang_vel_cov # X
        imu_msg.angular_velocity_covariance[4] = default_ang_vel_cov # Y
        imu_msg.angular_velocity_covariance[8] = default_ang_vel_cov # Z

        self.publisher_.publish(imu_msg)
        # self.get_logger().info('Publishing IMU data', throttle_duration_sec=1.0) # Optional: for debugging

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    # Check if node initialization failed (e.g. sensor not found)
    if not rclpy.ok(): # A bit of a hack, better to check a flag set in __init__
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MPU6050 node interrupted.')
    finally:
        if rclpy.ok() and node: # Check if node still exists and rclpy is ok
             node.destroy_node()
        if rclpy.ok(): # Check if rclpy is still ok before shutting down
             rclpy.shutdown()

if __name__ == '__main__':
    main()
