#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class VelocityConverter(Node):
    def __init__(self):
        super().__init__('velocity_converter')

        # Robot parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_base = 0.332    # meters (distance between wheels)

        # Initial velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Subscriber to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for wheel velocities
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)

        # Timer for publishing at fixed rate (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_wheel_velocities)

    def cmd_vel_callback(self, msg):
        # Store latest velocities
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_wheel_velocities(self):
        # Use stored values to compute wheel velocities
        v = self.linear_velocity
        omega = self.angular_velocity

        v_l = (v - omega * self.wheel_base / 2) / self.wheel_radius
        v_r = (v + omega * self.wheel_base / 2) / self.wheel_radius

        # Publish
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [v_l, v_r]
        self.wheel_pub.publish(wheel_msg)

        self.get_logger().info(f'Timer published wheel velocities -> Left: {v_l:.2f} rad/s, Right: {v_r:.2f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
