#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigation.navigation_manager import NavigationManager

from enum import Enum

mock_modules = [
    'navigation',
]

# states for state machine
class PuzzlebotState(Enum):
    INITIALIZING = 0
    IDLE = 1
    NAVIGATING = 2
    ERROR = 3

class PuzzlebotManager(Node):
    def __init__(self):
        super().__init__('puzzlebot_manager')
        self.mock_data = False
        
        self.get_logger().info("Initializing PuzzlebotManager...")
        self.get_logger().info(f"Mock modules: {', '.join(mock_modules)}")
        
        self.initialize_modules()
        
        self.get_logger().info("Module initialization complete.")
        self.get_logger().info("PuzzlebotManager initialized successfully.")
        
        self.run()
        
    def initialize_modules(self):
        self.navigation_manager = NavigationManager(self, mock=('navigation' in mock_modules))
        
        
    def run(self):
        """
        Main loop for the PuzzlebotManager.
        """
        self.navigation_manager.publish_goal_pose(PoseStamped())
        
        self.get_logger().info("PuzzlebotManager run")
        

def main(args=None):
    rclpy.init(args=args)
    puzzlebot_manager = PuzzlebotManager()
    
    puzzlebot_manager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()