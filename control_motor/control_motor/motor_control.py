import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if angular > 0:
            self.serial_port.write(b'a')  # Turn Left
            print('a')
        elif angular < 0:
            self.serial_port.write(b'd')  # Turn Right
            print('d')

        if linear > 0:
            self.serial_port.write(b'w')  # Forward
            print('w')
        elif linear < 0:
            print('s')
            self.serial_port.write(b's')  # Backward
        elif linear == 0 and angular == 0:
            print('x')
            self.serial_port.write(b'x')  # Stop

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

