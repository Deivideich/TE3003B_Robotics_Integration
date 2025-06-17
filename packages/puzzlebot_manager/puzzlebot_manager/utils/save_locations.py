#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import signal
import sys


class SaveLocationsNode(Node):
    def __init__(self):
        super().__init__('save_locations_node')
        
        # List to store poses
        self.poses = []
        
        # Subscriber to /goal_pose topic
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Save locations node started. Listening to /goal_pose...')
        self.get_logger().info('Press Ctrl+C to save and exit.')
        
        # Register signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def pose_callback(self, msg):
        """Callback function to handle received poses"""
        pose_data = {
            'position': [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ],
            'orientation': [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
        }
        
        self.poses.append(pose_data)
        self.get_logger().info(f'Saved pose #{len(self.poses)}: position=[{pose_data["position"][0]:.2f}, {pose_data["position"][1]:.2f}, {pose_data["position"][2]:.2f}]')
    
    def save_to_yaml(self):
        """Save all collected poses to a YAML file"""
        if not self.poses:
            self.get_logger().warn('No poses to save!')
            return
        
        filename = 'exploration_goals.yaml'
        
        try:
            with open(filename, 'w') as file:
                yaml.dump(self.poses, file, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f'Successfully saved {len(self.poses)} poses to {filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save poses: {str(e)}')
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C signal"""
        self.get_logger().info('\nReceived Ctrl+C, saving poses and exiting...')
        self.save_to_yaml()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SaveLocationsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()