#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from nav_msgs.msg import Odometry

class DeadReckonSubscriber(Node):
    def __init__(self):
        super().__init__('dead_reckon_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def listener_callback(self, msg):
        # Log received position for debugging
        # self.get_logger().info(f"Received Pose: {msg.pose}")
        
        # Extract position and orientation from the PoseStamped message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Prepare the transformation message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # The parent frame is 'odom'
        transform.child_frame_id = 'base_footprint'  # The child frame is 'base_footprint'
        
        # Set the translation (position)
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        
        # Set the rotation (orientation)
        transform.transform.rotation = orientation
        
        # Publish the transform to the tf tree
        self.tf_broadcaster.sendTransform(transform)
        # self.get_logger().info(f"Published transform to /tf: {transform}")

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckonSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
