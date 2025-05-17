# fake_odometry_publisher.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Header

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odometry/global_ekf_navsat_input', 10) # Publish directly to the topic navsat listens to
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish at 10 Hz
        self.get_logger().info("Fake Odometry Publisher started, publishing to /odometry/global_ekf_navsat_input")

    def timer_callback(self):
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # Or "odom" if that's what navsat expects its input odom to be in
                                    # For use_odometry_yaw, the frame_id of this odom msg is important for TF.
                                    # If global_ekf output is map->base_link, then header.frame_id="map"
        msg.child_frame_id = "base_link"

        # Dummy pose - ensure orientation is not identity
        msg.pose.pose = Pose(
            position=Point(x=0.1, y=0.1, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.1, w=0.99) # Slight yaw
        )
        # Dummy non-zero covariances for pose (diagonal)
        msg.pose.covariance[0] = 0.1  # x
        msg.pose.covariance[7] = 0.1  # y
        msg.pose.covariance[14] = 0.1 # z
        msg.pose.covariance[21] = 0.05 # roll
        msg.pose.covariance[28] = 0.05 # pitch
        msg.pose.covariance[35] = 0.05 # yaw

        # Dummy twist (can be zero)
        msg.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        # Dummy non-zero covariances for twist (diagonal)
        msg.twist.covariance[0] = 0.1 # vx
        msg.twist.covariance[7] = 0.1 # vy
        msg.twist.covariance[35] = 0.05 # wz

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
