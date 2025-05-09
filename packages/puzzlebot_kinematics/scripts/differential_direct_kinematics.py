#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import math
import time
import numpy as np

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

        # Covariance parameters
        self.kr = 0.5  # covariance for right wheel
        self.kl = 0.5  # covariance for left wheel

        # Covariance matrix
        self.cov = np.zeros((3, 3))

        #Last theta
        self.last_theta = 0.0

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
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def model_jacobian(self, v, dt, theta):
        H_k = np.array([[1, 0, -dt * v * np.sin(theta)],
                         [0, 1, dt * v * np.cos(theta)],
                         [0, 0, 1]])
        
        return H_k
    
    def error_matrix(self, r, dt, theta, l, omega_l, omega_r):
        delta_wk = 0.5 * r * dt * np.array([[np.cos(theta), np.cos(theta)],
                                             [np.sin(theta), np.sin(theta)],
                                             [2 / l, -2 / l]])
        sigma_delta_k = np.array([[self.kr * omega_r, 0],
                                   [0, self.kl * omega_l]])
        
        Q_k = delta_wk @ sigma_delta_k @ delta_wk.T
        return Q_k
    
    def model_covariance(self, v, dt, theta, last_cov, omega_l, omega_r):
        # Jacobian of the motion model
        H_k = self.model_jacobian(v, dt, theta)

        # Process noise covariance
        Q = self.error_matrix(self.wheel_radius, dt, theta, self.wheel_base, omega_l, omega_r)
        # Q = np.array([[0.5, 0.01, 0.01],
        #               [0.01, 0.5, 0.01],
        #               [0.01, 0.01, 0.2]])

        # Compute the covariance of the state
        cov = H_k @ last_cov @ H_k.T + Q
        
        self.get_logger().info(f'Covariance: {cov}')

        return cov
    
    def convert_covariance_puzzlebot_to_ros(self, cov):
        # Convert covariance matrix to ROS format
        ros_cov = np.zeros(36)
        ros_cov[0] = cov[0, 0]
        ros_cov[1] = cov[0, 1]
        ros_cov[5] = cov[0, 2]
        ros_cov[6] = cov[1, 0]
        ros_cov[7] = cov[1, 1]
        ros_cov[11] = cov[1, 2]
        ros_cov[30] = cov[2, 0]
        ros_cov[31] = cov[2, 1]
        ros_cov[35] = cov[2, 2]

        return ros_cov

    def wheel_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        dt = now - self.last_time
        self.last_time = now

        if len(msg.data) < 2:
            # self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        omega_l = msg.data[0]
        omega_r = msg.data[1]

        # Direct kinematics
        v = self.wheel_radius * (omega_r + omega_l) / 2
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_base



        # Update pose
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
        self.cov = self.model_covariance(v, dt, self.last_theta, self.cov, omega_l, omega_r)
        ros_cov = self.convert_covariance_puzzlebot_to_ros(self.cov)
        odom.pose.covariance = ros_cov
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
        self.last_theta = self.theta
        # self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
