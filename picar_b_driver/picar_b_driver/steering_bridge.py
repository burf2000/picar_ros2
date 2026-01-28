#!/usr/bin/env python3
"""Bridge node that extracts angular.z from cmd_vel and publishes to /steering.

This is needed because teleop_twist_joy publishes Twist messages designed for
differential drive, but PiCar-B uses Ackermann steering with a separate /steering topic.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SteeringBridge(Node):
    def __init__(self):
        super().__init__('steering_bridge')

        # Parameter for scaling angular.z to steering range (-1.0 to 1.0)
        self.declare_parameter('steering_scale', 1.0)
        self.steering_scale = self.get_parameter('steering_scale').value

        # Subscribe to cmd_vel (from teleop_twist_joy)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publish to steering topic
        self.steering_pub = self.create_publisher(Float64, 'steering', 10)

        self.get_logger().info('Steering bridge started')

    def cmd_vel_callback(self, msg: Twist):
        # Extract angular.z and publish to /steering
        steering_msg = Float64()
        steering_msg.data = msg.angular.z * self.steering_scale
        self.steering_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
