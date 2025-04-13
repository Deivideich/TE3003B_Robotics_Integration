#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math as m

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')

        # Goal pose
        self.goal = None
        self.reached_goal = False

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Tolerances
        self.goal_tolerance = 0.1  # meters

        # Control gains
        self.k_lin = 1.0
        self.k_ang = 1.0

        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.reached_goal = False
        self.get_logger().info(f"New goal received: x={self.goal.position.x:.3f}, y={self.goal.position.y:.3f}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = m.atan2(siny_cosp, cosy_cosp)
        print(f"Current position: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")
        self.get_logger().info(f"Current position: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")

    def control_loop(self):
        if not self.goal or self.reached_goal:
            return

        # Compute position error
        dx = self.goal.position.x - self.x
        dy = self.goal.position.y - self.y
        print(f"dx: {dx:.3f}, dy: {dy:.3f}")
        # Compute distance to goal
        distance = m.sqrt(dx*dx + dy*dy)
        print(f"Distance to goal: {distance:.3f}")
        
        # Compute desired heading
        target_theta = m.atan2(dy, dx)
        angle_error = target_theta - self.theta
        
        # Normalize angle error to [-pi, pi]
        angle_error = m.atan2(m.sin(angle_error), m.cos(angle_error))
        print(f"Angle error: {angle_error:.3f}")

        cmd = Twist()

        if abs(angle_error) > 0.05:
            # Rotate to face goal
            cmd.angular.z = self.k_ang * angle_error
            print(f"Rotating: {cmd.angular.z:.3f}")
            cmd.linear.x = 0.0
        elif distance > self.goal_tolerance:
            # Move straight to goal
            cmd.linear.x = self.k_lin * distance
            print(f"Moving: {cmd.linear.x:.3f}")
            cmd.angular.z = 0.0
        else:
            # Goal reached
            self.reached_goal = True
            self.get_logger().info("Goal reached!")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
