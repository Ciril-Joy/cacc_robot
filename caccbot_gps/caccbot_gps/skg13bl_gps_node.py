#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
import serial

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        self.serial_port = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        self.timer = self.create_timer(0.5, self.read_gps)

    def read_gps(self):
        line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('$GPGGA'):
            parts = line.split(',')
            if len(parts) < 15 or parts[6] == '0':
                return  # no fix
            try:
                msg = NavSatFix()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'gps_link'

                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                msg.latitude = self.parse_lat(parts[2], parts[3])
                msg.longitude = self.parse_lon(parts[4], parts[5])
                msg.altitude = float(parts[9])

                msg.position_covariance = [0.0] * 9
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}')
            except Exception as e:
                self.get_logger().error(f'Failed to parse GPGGA: {e}')

    def parse_lat(self, raw_val, hemi):
        if not raw_val:
            return 0.0
        degrees = float(raw_val[:2])
        minutes = float(raw_val[2:])
        value = degrees + minutes / 60.0
        return -value if hemi == 'S' else value

    def parse_lon(self, raw_val, hemi):
        if not raw_val:
            return 0.0
        degrees = float(raw_val[:3])
        minutes = float(raw_val[3:])
        value = degrees + minutes / 60.0
        return -value if hemi == 'W' else value

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
