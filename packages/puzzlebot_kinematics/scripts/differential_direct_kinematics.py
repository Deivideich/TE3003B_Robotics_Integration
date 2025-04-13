#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import math
import time

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.19    # meters

        # Robot state (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time tracking
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                         self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        # Subscription to wheel angular velocities [left, right]
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/wheel_velocities',  # Format: [omega_left, omega_right]
            self.wheel_callback,
            10
        )

        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(PoseStamped, '/odom', 10)

    def wheel_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        dt = now - self.last_time
        self.last_time = now

        if len(msg.data) < 2:
            self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        omega_l = msg.data[0]  # rad/s
        omega_r = msg.data[1]  # rad/s

        # Direct kinematics
        v = self.wheel_radius * (omega_r + omega_l) / 2
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_base

        # Update pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Publish pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
