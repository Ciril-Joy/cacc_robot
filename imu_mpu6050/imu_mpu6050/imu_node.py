#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050 # Assuming this is the adafruit-circuitpython-mpu6050 or similar
import math
from geometry_msgs.msg import Quaternion # Not strictly needed if you set x,y,z,w directly

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # Try to initialize the sensor
        try:
            self.sensor = mpu6050(0x68) # Make sure I2C address is correct (0x68 or 0x69)
            self.get_logger().info("MPU6050 initialized successfully.")
            self.sensor_ok = True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {e}")
            self.get_logger().error("MPU6050 node will not publish data.")
            self.sensor_ok = False
            # Optional: make the node exit if sensor init fails
            # rclpy.shutdown()
            # return # if you don't shutdown, the node will exist but do nothing

        # Only create timer if sensor initialized correctly
        if self.sensor_ok:
            timer_period = 0.05  # seconds (for 20 Hz)
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f"Publishing IMU data at ~{1.0/timer_period:.1f} Hz.")
        else:
            self.timer = None # No timer if sensor failed

    def timer_callback(self):
        if not self.sensor_ok: # Double check, though timer shouldn't run if not ok
            return

        try:
            accel_data = self.sensor.get_accel_data() # Returns m/s^2 by default for some libraries
            gyro_data = self.sensor.get_gyro_data()   # Returns deg/s by default for some libraries
        except Exception as e:
            self.get_logger().warn(f"Failed to read MPU6050 data during callback: {e}")
            return # Skip publishing this cycle if read fails

        imu_msg = Imu()

        # Timestamp
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = "imu_link"

        # Angular velocity (rad/s) - MPU6050 library usually returns deg/s
        imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
        imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
        imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

        # Linear acceleration (m/s^2) - MPU6050 library usually returns m/s^2
        # Ensure your library's output matches these units, or convert.
        imu_msg.linear_acceleration.x = accel_data['x']
        imu_msg.linear_acceleration.y = accel_data['y']
        imu_msg.linear_acceleration.z = accel_data['z']

        # Orientation unknown (set to identity and covariance -1)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0 # Identity quaternion

        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.orientation_covariance[0] = -1.0  # Indicate unknown/unreliable orientation

        # --- Covariances for accel/gyro ---
        # If you have calibrated values or datasheet noise specs, use them.
        # Otherwise, use small, positive, reasonable defaults.
        # Example: (0.1 m/s^2)^2 for accel, (0.01 rad/s)^2 for gyro
        # (0.01 rad/s is approx 0.57 deg/s)
        accel_cov_diagonal = 0.01 # (m/s^2)^2
        gyro_cov_diagonal = 0.0001 # (rad/s)^2

        imu_msg.linear_acceleration_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance[0] = accel_cov_diagonal
        imu_msg.linear_acceleration_covariance[4] = accel_cov_diagonal
        imu_msg.linear_acceleration_covariance[8] = accel_cov_diagonal

        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance[0] = gyro_cov_diagonal
        imu_msg.angular_velocity_covariance[4] = gyro_cov_diagonal
        imu_msg.angular_velocity_covariance[8] = gyro_cov_diagonal

        self.publisher_.publish(imu_msg)

        # ADDED LOGGER MESSAGE FOR EACH PUBLISH
        # self.get_logger().info('IMU data published.') # This will be very verbose
        self.get_logger().info('IMU data published.', throttle_duration_sec=1.0, throttle_time_source_type='RCUTILS_STEADY_TIME')
        # The throttle_duration_sec makes it log at most once per second.
        # throttle_time_source_type can be RCUTILS_ROS_TIME (uses ROS time, good if sim time is used)
        # or RCUTILS_STEADY_TIME (uses wall clock, good for real hardware).

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()

    # Check if node initialization (including sensor) was successful
    if not node.sensor_ok and rclpy.ok():
        # If sensor init failed but node object was created, destroy it before shutdown
        node.get_logger().error("MPU6050 node failed to initialize sensor. Shutting down.")
        node.destroy_node()
        rclpy.shutdown()
        return # Exit script

    if not rclpy.ok(): # If rclpy itself had an issue during init
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MPU6050 node interrupted by Ctrl+C.')
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in MPU6050 node: {e}")
    finally:
        # Ensure node is destroyed and rclpy is shut down cleanly
        if hasattr(node, 'is_spin_once_active') and node.is_spin_once_active(): # Check if still spinning
             pass # spin will exit on its own

        if rclpy.ok() and node is not None and node.executor is not None : #More robust check
             if node.executor.is_spinning():
                 node.get_logger().info("Node was spinning, attempting to destroy.")

        if rclpy.ok() and node is not None :
             # node.get_logger().info("Destroying MPU6050 node.") # Can be noisy if already destroyed
             node.destroy_node()

        if rclpy.ok():
             # node.get_logger().info("Shutting down RCLPY.") # Can be noisy
             rclpy.shutdown()
        # node.get_logger().info("MPU6050 node finished.")


if __name__ == '__main__':
    main()
