#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped
import math


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        # Robot parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.19    # meters

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous joint positions
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.last_time = None

        # Create subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_callback(self, msg: JointState):
        # Get current time
        now = self.get_clock().now()
        now_sec = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

        # Extract joint positions
        try:
            left_idx = msg.name.index('wheel_l_joint')
            right_idx = msg.name.index('wheel_r_joint')
        except ValueError:
            self.get_logger().warn('Expected joint names not found')
            return

        pos_left = msg.position[left_idx]
        pos_right = msg.position[right_idx]
        
        # Initialize time and previous positions
        if self.prev_left_pos is None or self.prev_right_pos is None:
            self.prev_left_pos = pos_left
            self.prev_right_pos = pos_right
            self.last_time = now_sec
            return

        dt = now_sec - self.last_time
        if dt <= 0.0:
            return  # Skip if time didn't advance

        # Compute angular velocities
        omega_l = (pos_left - self.prev_left_pos) / dt
        omega_r = (pos_right - self.prev_right_pos) / dt

        # Save for next iteration
        self.prev_left_pos = pos_left
        self.prev_right_pos = pos_right
        self.last_time = now_sec

        # Direct kinematics
        v = self.wheel_radius * (omega_r + omega_l) / 2
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_base

        # Integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Quaternion from theta
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [0.0] * 36

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom)

        # TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
