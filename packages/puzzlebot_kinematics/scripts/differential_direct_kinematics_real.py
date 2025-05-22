#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
from sensor_msgs.msg import JointState
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
                         
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )


        # Publisher for estimated pose
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Extract wheel velocities from JointState message
        if len(msg.velocity) < 2:
            self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        omega_l = msg.velocity[0]
        omega_r = msg.velocity[1]

        # Direct kinematics
        v = self.wheel_radius * (omega_r + omega_l) / 2
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_base

        # Update pose
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        dt = now - self.last_time
        self.last_time = now

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        # Convert yaw to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Zero covariance
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
