""" ROS2 node for converting differential drive velocity commands to Ackermann steering commands. """

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math


class TwistToAckermann(Node):

    def __init__(self):
        """ Sets up subscribers and publishers for velocity command conversion with TwistMessages.
        Configures the wheelbase and maximum steering angle parameters for the Hunter SE vehicle, taken from the urdf.
        """
        super().__init__('twist_to_ackermann')
        self.subscriber_ = self.create_subscription(Twist, '/cmd_vel_differential_drive', self.differential_drive_vel_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wheelbase = 0.55               # [m]
        self.steering_angle_limit = 0.69    # [rad]
        self.min_vel = 0.001                # [m/s]
        self.get_logger().info('Initialized twist_to_ackermann node, waiting to recieve differential drive velocity commands...')

    def differential_drive_vel_callback(self, msg_differential_drive):
        """Convert differential drive velocity to Ackermann steering command."""

        lin_vel = msg_differential_drive.linear.x
        ang_vel = msg_differential_drive.angular.z

        # Handle pure rotation edge case
        if abs(lin_vel) < self.min_vel:
            lin_vel = 0.0
            sign = (ang_vel > 0) - (ang_vel < 0)
            steering_angle = sign * self.steering_angle_limit
        else:
            # Calculate ackermann steering angle
            steering_angle = math.atan(self.wheelbase * ang_vel / lin_vel)
            steering_angle = max(-self.steering_angle_limit, min(steering_angle, self.steering_angle_limit)) # Inline clamp
        
        # Publish ackermann steering command
        msg_ackermann = Twist()
        msg_ackermann.linear.x = lin_vel
        msg_ackermann.angular.z = steering_angle
        self.publisher_.publish(msg_ackermann)


def main(args=None):
    rclpy.init(args=args)

    twist_to_ackermann = TwistToAckermann()
    rclpy.spin(twist_to_ackermann)
    
    twist_to_ackermann.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
