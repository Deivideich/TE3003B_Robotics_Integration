#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros


class InitialPoseHandler(Node):
    def __init__(self):
        super().__init__('initial_pose_handler')
        self.frame_id = self.declare_parameter('frame_id', 'map').get_parameter_value().string_value
        self.child_frame_id = self.declare_parameter('child_frame_id', 'odom').get_parameter_value().string_value
        self.timer_period = self.declare_parameter('timer_period', 0.1).get_parameter_value().double_value  # seconds

        #### TF HANDLERS ####
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.latest_transform = None

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.timer = self.create_timer(self.get_parameter('timer_period').get_parameter_value().double_value, self.publish_transform)

    def initial_pose_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.latest_transform = t
        self.get_logger().info("Initial pose received and transform saved.")

    def publish_transform(self):
        if self.latest_transform:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.latest_transform.header.frame_id
            t.child_frame_id = self.latest_transform.child_frame_id
            t.transform = self.latest_transform.transform
            self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rclpy.init()
    node = InitialPoseHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()