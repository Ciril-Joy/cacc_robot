#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus

class SKG13BLGPSNode(Node):
    def __init__(self):
        super().__init__('skg13bl_gps_node')

        self.declare_parameter('port', '/dev/serial1')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_port = serial.Serial(port, baud, timeout=1.0)
        self.publisher_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.timer = self.create_timer(0.5, self.read_gps_data)

        self.get_logger().info(f"Started SKG13BL GPS Node on {port} at {baud} baud.")

    def read_gps_data(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)
                gps_msg = NavSatFix()

                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps_link'

                gps_msg.status.status = NavSatStatus.STATUS_FIX
                gps_msg.status.service = NavSatStatus.SERVICE_GPS

                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = float(msg.altitude)

                # Optionally set covariance or leave it unknown
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(gps_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to read GPS: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SKG13BLGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
