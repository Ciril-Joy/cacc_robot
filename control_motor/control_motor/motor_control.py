import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            exit(1)

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        cmd = None

        if linear > 0:
            cmd = b'o'  # Forward
            self.get_logger().info('Forward (i)')
        elif linear < 0:
            cmd = b'>'  # Backward
            self.get_logger().info('Backward (<)')
        elif angular > 0:
            cmd = b'k'  # Turn Left
            self.get_logger().info('Turn Left (j)')
        elif angular < 0:
            cmd = b';'  # Turn Right
            self.get_logger().info('Turn Right (l)')
        elif linear == 0 and angular == 0:
            cmd = b'x'  # Stop
            self.get_logger().info('Stop (x)')

        if cmd:
            try:
                self.serial_port.write(cmd)
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send serial command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
