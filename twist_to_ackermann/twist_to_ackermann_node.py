import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math


class TwistToAckermann(Node):

    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.wheelbase = 0.55 # distance between front and rear axle of hunter_se urdf
        self.subscriber_ = self.create_subscription(Twist, '/cmd_vel_differential_drive', self.differential_drive_vel_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def differential_drive_vel_callback(self, msg_diff):

        msg = Twist()
        msg.linear.x = msg_diff.linear.x  # Linear velocity (rear axle) in m/s
        # if abs(msg_diff.angular.z) > 0.001: # TODO: edge case pure rotation
        msg.angular.z = math.atan(self.wheelbase * msg_diff.angular.z / msg_diff.linear.x)  # Steering angle (front axle) in rad
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Ackermann steering command: linear.x=%f, angular.z=%f' % (msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)

    twist_to_ackermann = TwistToAckermann()
    rclpy.spin(twist_to_ackermann)
    
    twist_to_ackermann.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
